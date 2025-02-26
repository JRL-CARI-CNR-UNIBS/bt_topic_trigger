#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/plugins.hpp>

#include <cnr_param/cnr_param.h>

template<typename T>
inline bool get_param(rclcpp::Node *node, std::string ns, std::string param_name, T& param, std::string what)
{
  if(cnr::param::has(ns + param_name, what))
    {
      if(!cnr::param::get(ns + param_name, param, what))
        {
          RCLCPP_ERROR_STREAM(node->get_logger(), "Cannot load " << ns + param_name + " parameter.");
          RCLCPP_ERROR_STREAM(node->get_logger(), "what:" << what);
          return false;
        }
    }
  else
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), ns + param_name + " parameter not available.");
      RCLCPP_ERROR_STREAM(node->get_logger(), "what: " << what);
      return false;
    }
  return true;
}

void searchForPlugins(std::vector<std::string>& plugin_paths)
{
  std::string ament_prefix_path = std::getenv("AMENT_PREFIX_PATH");
  std::istringstream iss(ament_prefix_path);

  std::string path;
  while(std::getline(iss, path, ':'))
    {
      std::string fs_path = std::filesystem::path(path);
      if(!std::filesystem::exists(fs_path) || !std::filesystem::is_directory(fs_path))
        continue;

      for (const auto& entry : std::filesystem::recursive_directory_iterator(fs_path))
        {
          if (entry.is_regular_file())
            {
              std::string filename = entry.path().filename().string();
              if (filename.find("plugin.so") != std::string::npos)
                plugin_paths.push_back(entry.path().string());
            }
        }
    }
}

class BTTopicTriggerNode
{    
private: 
  std::mutex mtx_;
  bool new_trigger_;
  std::string trigger_;
  BT::BehaviorTreeFactory& factory_;
  std::string bt_topic_="/bt_trigger";
  const rclcpp::Node::SharedPtr& node_;
  std::vector<std::string> trees_paths_;
  std::vector<std::string> priority_trees_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr bt_subscriber_;

  void setTrigger(const std::string& trigger)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    new_trigger_ = true;
    trigger_ = trigger;
  }

  std::string getTrigger()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    new_trigger_ = false;
    return trigger_;
  }

  void btCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    setTrigger(msg->data);
  }

  int isPriorityTrigger(std::string trigger)
  {
    std::transform(trigger.begin(), trigger.end(), trigger.begin(),
                   ::tolower);

    int priority_level = std::numeric_limits<int>::max(); // the lower, the higher priority
    for(unsigned int i=0; i<priority_trees_.size();i++)
      {
        std::string priority_tree = priority_trees_[i];
        std::transform(priority_tree.begin(), priority_tree.end(), priority_tree.begin(),
                       ::tolower);
        if(trigger.compare(priority_tree) == 0)
          {
            priority_level = i;
            RCLCPP_INFO(node_->get_logger(), "Triggering the priority tree %s with priority %d",
                        trigger.c_str(), priority_level);
            return priority_level;
          }

      }
    return priority_level;
  }

  bool elaborateTrigger(std::filesystem::path& path_to_bt, int& priority_level, bool& kill_execution)
  {
    if(!new_trigger_)
      return false;

    std::string trigger = getTrigger();
    std::transform(trigger.begin(), trigger.end(), trigger.begin(),
                   ::tolower);

    if(trigger == "kill")
      {
        kill_execution = true;
        return true;
      }
    else
      {
        kill_execution = false;
        priority_level = isPriorityTrigger(trigger);

        std::string bt_name = trigger + "_tree.xml";

        for (const auto& dir : trees_paths_)
          {
            std::filesystem::path bt_file_path = std::filesystem::path(dir) / bt_name;
            if (std::filesystem::exists(bt_file_path))
              {
                path_to_bt = bt_file_path;
                RCLCPP_INFO(node_->get_logger(), "Behavior Tree file %s found!", bt_name.c_str());

                return true;
              }
          }

        RCLCPP_ERROR(node_->get_logger(), "Behavior Tree file %s not found!", bt_name.c_str());
        return false;
      }
  }

  bool betterPriority(const int& trigger_priority, const int& priority)
  {
    return (trigger_priority<priority);
  }

public:
  BTTopicTriggerNode(const rclcpp::Node::SharedPtr& node,
                     BT::BehaviorTreeFactory& factory,
                     const std::vector<std::string>& trees_paths,
                     const std::vector<std::string>& priority_trees,
                     const std::string &bt_topic = "/bt_trigger")
    : node_(node), factory_(factory), trees_paths_(trees_paths),
      priority_trees_(priority_trees), bt_topic_(bt_topic)
  {
    new_trigger_ = false;
    bt_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
          bt_topic_, 1, std::bind(&BTTopicTriggerNode::btCallback, this, std::placeholders::_1));
  }

  bool runBTs()
  {
    BT::Tree tree;
    bool kill = false;
    bool tree_loaded = false;
    bool tree_running = false;
    int priority, trigger_priority;
    std::filesystem::path path_to_bt;

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_);

    while (!kill && rclcpp::ok())
      {
        if(elaborateTrigger(path_to_bt,trigger_priority,kill))
          {
            if(kill)
              continue;

            if(!tree_loaded || (tree_running && betterPriority(trigger_priority,priority)))
              {
                if(tree_running)
                  tree.haltTree();

                factory_.clearRegisteredBehaviorTrees();
                factory_.registerBehaviorTreeFromFile(path_to_bt);
                tree = factory_.createTree("MainTree");

                tree_loaded = true;
                priority = trigger_priority;

                RCLCPP_INFO(node_->get_logger(), "Behavior Tree loaded!");
              }
          }

        if(tree_loaded)
          tree_running = tree.rootNode()->executeTick() == BT::NodeStatus::RUNNING;

        executor.spin_some();
      }

    RCLCPP_INFO(node_->get_logger(), "Behavior Tree execution finished.");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bt_topic_trigger_node");
  auto logger = node->get_logger();

  std::vector<std::string> available_plugins;
  searchForPlugins(available_plugins);

  // Display found plugin paths
  std::string plugins_found = "\nPlugins found:";
  for (const auto& plugin_path : available_plugins)
    plugins_found = plugins_found+"\n -"+plugin_path;
  RCLCPP_INFO_STREAM(logger,plugins_found);

  std::string ns= "/bt_topic_trigger";

  std::vector<std::string> plugins_to_load;
  if(cnr::param::has(ns+"/plugins",w))
    {
      if(!cnr::param::get(ns+"/plugins",plugins_to_load,w))
        {
          RCLCPP_ERROR_STREAM(logger,"cannot load "<<ns+"/plugins");
          RCLCPP_ERROR_STREAM(logger,"what:\n"<<w);

          return 1;
        }
    }
  else
    {
      RCLCPP_ERROR_STREAM(logger,ns+"/plugins is not an available parameter");
      RCLCPP_ERROR_STREAM(logger,"what:\n"<<w);

      return 1;
    }

  std::vector<std::string> trees_paths;
  if(cnr::param::has(ns+"/trees_paths",w))
    {
      if(!cnr::param::get(ns+"/trees_paths",trees_paths,w))
        {
          RCLCPP_ERROR_STREAM(logger,"cannot load "<<ns+"/trees_paths");
          RCLCPP_ERROR_STREAM(logger,"what:\n"<<w);

          return 1;
        }
    }
  else
    {
      RCLCPP_ERROR_STREAM(logger,ns+"/trees_paths is not an available parameter");
      RCLCPP_ERROR_STREAM(logger,"what:\n"<<w);

      return 1;
    }

  std::vector<std::string> priority_trees;
  if(cnr::param::has(ns+"/priority_trees",w))
    {
      if(!cnr::param::get(ns+"/priority_trees",priority_trees,w))
        {
          RCLCPP_ERROR_STREAM(logger,"cannot load "<<ns+"/priority_trees");
          RCLCPP_ERROR_STREAM(logger,"what:\n"<<w);

          return 1;
        }
    }
  else
    {
      RCLCPP_ERROR_STREAM(logger,ns+"/priority_trees is not an available parameter");
      RCLCPP_ERROR_STREAM(logger,"what:\n"<<w);

      return 1;
    }

  std::string bt_topic;
  if(cnr::param::has(ns+"/bt_topic",w))
    {
      if(!cnr::param::get(ns+"/bt_topic",priority_trees,w))
        {
          RCLCPP_ERROR_STREAM(logger,"cannot load "<<ns+"/bt_topic");
          RCLCPP_ERROR_STREAM(logger,"what:\n"<<w);

          return 1;
        }
    }
  else
    {
      RCLCPP_ERROR_STREAM(logger,ns+"/bt_topic is not an available parameter");
      RCLCPP_ERROR_STREAM(logger,"what:\n"<<w);

      return 1;
    }

  BT::BehaviorTreeFactory factory;

  for(const std::string& plugin_name:plugins_to_load)
    {
      std::string path_to_plugin;
      for(const std::string& this_plugin:available_plugins)
        {
          if(this_plugin.find(plugin_name) != std::string::npos)
            {
              path_to_plugin = this_plugin;
              break;
            }
        }

      BT::RosNodeParams params;
      params.nh = node;
      params.server_timeout = std::chrono::milliseconds(10000);
      params.wait_for_server_timeout = std::chrono::milliseconds(10000);

      RCLCPP_INFO_STREAM(logger,"Path to plugin loaded "<<path_to_plugin);
      RegisterRosNode(factory,path_to_plugin,params);
    }

  BTTopicTriggerNode bt(node,factory,trees_paths,priority_trees,bt_topic);
  bt.runBTs();

  rclcpp::shutdown();
  return 0;
}
