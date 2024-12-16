/*
 * @Author: chasey melancholycy@gmail.com
 * @Date: 2024-12-09 15:02:28
 * @FilePath: /mesh_planner/src/mesh_planner/utils/src/utils_node.cpp
 * @Description: 
 * 
 * Copyright (c) 2024 by chasey (melancholycy@gmail.com), All Rights Reserved. 
 */

#include "utils/common.hpp"  

namespace utils{

  class utilsNode: public rclcpp::Node{
    public:
    utilsNode(const rclcpp::NodeOptions& options): Node("utils_node", options){
      RCLCPP_INFO(this->get_logger(),"utils_node starting!!");
      numPub_ = this->create_publisher<utils::Float32MsgType>("/num",10);
      auto num_cb = bindCallbackWrapper(&utilsNode::printNumCallback,this);
      numSub_ = this->create_subscription<utils::Float32MsgType>("/num",10,num_cb); 
      auto timer_cb = bindCallbackWrapper(&utilsNode::timerCallback,this);
      timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), timer_cb);
    }

    private://membershup function
      void printNumCallback(utils::Float32MsgType::ConstSharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "num:%.2f", msg->data);
      }
      void timerCallback(){
        utils::Float32MsgType num;
        num.data = 3.1415926;
        numPub_->publish(num);
      }

    private://membership variable
      utils::PubSPtrType<utils::Float32MsgType> numPub_;
      utils::SubSPtrType<utils::Float32MsgType> numSub_;
      utils::TimerSPtrType timer_;

  } ;
}   
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(utils::utilsNode)   