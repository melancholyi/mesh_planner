/*
 * @Author: chasey melancholycy@gmail.com
 * @Date: 2024-12-09 15:02:28
 * @FilePath: /mesh_planner/src/mesh_planner/mesh_bgk/src/mesh_bgk_node.cpp
 * @Description: 
 * 
 * Copyright (c) 2024 by chasey (melancholycy@gmail.com), All Rights Reserved. 
 */

#include "utils/common.hpp"   
namespace mesh_bgk{
  class MeshBGK: public rclcpp::Node{
    public:
    MeshBGK(const rclcpp::NodeOptions& options): Node("mesh_bgk_node", options){
      RCLCPP_INFO(this->get_logger(),"mesh_bgk_node starting!!");
    }

    private:

  } ;
}   
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(mesh_bgk::MeshBGK)   