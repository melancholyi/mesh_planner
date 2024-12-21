/*
 * @Author: chasey melancholycy@gmail.com
 * @Date: 2024-12-09 15:02:28
 * @FilePath: /mesh_planner/src/mesh_planner/utils/src/utils_node.cpp
 * @Description: 
 * 
 * Copyright (c) 2024 by chasey (melancholycy@gmail.com), All Rights Reserved. 
 */

#include "utils/common.hpp"  
#include "utils/voxel.hpp"
#include "utils/voxel_map.hpp"  


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

      ellipsoidPub_ = this->create_publisher<utils::MarkerMsgType>("ellipsoid", 10);
      ellipsoidArrayPub_ = this->create_publisher<utils::MarkerArrayMsgType>("ellipsoid_array", 10);
    
      singleVoxel_ = std::make_unique<utils::Voxel>(Eigen::Vector3i(0,0,0), Eigen::Vector3d(1,1,1), 1);
    }

    private://membershup function
      void printNumCallback(utils::Float32MsgType::ConstSharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "num:%.2f", msg->data);
      }
      float count = 0;
      int id = 0;
      // void timerCallback(){
      //   utils::Float32MsgType num;
      //   num.data = 3.1415926;
      //   numPub_->publish(num);
        
      //   utils::MarkerMsgType ellipsoid_msg;
      //   // header
      //   ellipsoid_msg.header.frame_id = "world";  
      //   ellipsoid_msg.header.stamp = this->now();

      //   ellipsoid_msg.ns = "multi_spheres";  
      //   ellipsoid_msg.id = id++;  
      //   ellipsoid_msg.type = visualization_msgs::msg::Marker::SPHERE_LIST;  
      //   ellipsoid_msg.action = visualization_msgs::msg::Marker::ADD;  

      //   // Set the pose - for a list of spheres, you can set the poses here
      //   ellipsoid_msg.pose.orientation.w = 1.0;

      //   // Set the scale - this will be the same for all spheres
      //   ellipsoid_msg.scale.x = 0.5;
      //   ellipsoid_msg.scale.y = 0.2;
      //   ellipsoid_msg.scale.z = 0.2;

      //   // Set the color - this will be the same for all spheres
      //   ellipsoid_msg.color.r = 1.0f;
      //   ellipsoid_msg.color.g = 0.0f;
      //   ellipsoid_msg.color.b = 0.0f;
      //   ellipsoid_msg.color.a = 1.0;

      //   // Create a list of points for the spheres
      //   std::vector<geometry_msgs::msg::Point> points;
      //   for (int i = 0; i < 5; ++i) {
      //       geometry_msgs::msg::Point point;
      //       point.x = 1.0 * i;
      //       point.y = count;
      //       point.z = 0.0;
      //       points.push_back(point);
      //   }
      //   count += 0.1;    
      //   ellipsoid_msg.points = points;

      //   ellipsoidPub_->publish(ellipsoid_msg);
      // }
      int flag = 1;
      void timerCallback(){
        if(flag == 1){
          flag = 0;
          const double x = 0.5;  
          const double y = 0.5;  
          const double z = 0.5;  
          for(double i =0.0; i <=1.0;i += 0.1){  
            singleVoxel_->insertPoint(Eigen::Vector3d(i, y, z));
            singleVoxel_->insertPoint(Eigen::Vector3d(x, i, z));
            // printf("%.2f",i);
          }
          for(double i =0.25; i <=0.75;i += 0.1){  
            singleVoxel_->insertPoint(Eigen::Vector3d(x, y, i));
          }
          singleVoxel_->updateState(1);  
          std_msgs::msg::Header header;
          header.frame_id = "world";
          header.stamp = this->now();
          auto marker = singleVoxel_->getGaussianEllipsoid(header, 10);
          ellipsoidPub_->publish(marker);
        }
        


      }
    private://membership variable
      utils::PubSPtrType<utils::Float32MsgType> numPub_;
      utils::SubSPtrType<utils::Float32MsgType> numSub_;
      utils::PubSPtrType<utils::MarkerMsgType> ellipsoidPub_;
      utils::PubSPtrType<utils::MarkerArrayMsgType> ellipsoidArrayPub_;
      utils::Voxel::UPtr singleVoxel_;  
    
      utils::TimerSPtrType timer_;
  } ;
  using utilsNodeSPtr = utilsNode::SharedPtr;
}   
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(utils::utilsNode)   