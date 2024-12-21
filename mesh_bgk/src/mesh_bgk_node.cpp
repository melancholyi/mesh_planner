/*
 * @Author: chasey melancholycy@gmail.com
 * @Date: 2024-12-09 15:02:28
 * @FilePath: /mesh_planner/src/mesh_planner/mesh_bgk/src/mesh_bgk_node.cpp
 * @Description: 
 * 
 * Copyright (c) 2024 by chasey (melancholycy@gmail.com), All Rights Reserved. 
 */

#include "utils/common.hpp"  
#include "pcl/filters/crop_box.h"  
#include "mesh_bgk/bgk_interface.hpp"  

namespace mesh_bgk{
  struct Param
  {
    //map
    double mapRes;   // map resolution   
    double rangeMax; // lidar range max
    
    // BGK
    double kernelLen;
    double kernelSaclar;
  };
  

  class MeshBGK: public rclcpp::Node{
    public://membership function
      MeshBGK(const rclcpp::NodeOptions& options): Node("mesh_bgk_node", options){
        RCLCPP_INFO(this->get_logger(),"mesh_bgk_node starting!!");

        //! declare parameter  
        param_.rangeMax = this->declare_parameter("range_max", 10.0);
        param_.mapRes = this->declare_parameter("map_res",0.1);  
        param_.kernelSaclar = this->declare_parameter("bgk/kernel_scalar", 1.0);    
        param_.kernelLen = this->declare_parameter("bgk/kernel_len", 1.2); 
          

        //! bgk
        bgkInterface_ = std::make_shared<bgk_interface::BGK21dType>(param_.kernelLen, param_.kernelSaclar);



        //! ros 
        auto pc2_cb = utils::bindCallbackWrapper(&MeshBGK::rcvPC2Callback,this);
        pc2Sub_ = this->create_subscription<utils::PC2MsgType>("/velodyne/points", 10, pc2_cb);

      }


    private://membership function
      //==========Helper  
      void downsampleFilter();
      void overhangFilter();    


      //==========Callback 
      void rcvPC2Callback(utils::PC2MsgType::ConstSharedPtr pc2Msg){
        // convert pc2 to pcl::PointCloud  
        utils::PclCloudXYZType cloud;
        pcl::fromROSMsg(*pc2Msg, cloud);
        
        //=====Transform to World

        //=====Filter  

        //=====BGK terrain model  
        //BKG train dataset  
        std::vector<double> X,Y;  
        for(auto pt: cloud.points){
          std::cout << pt.z;
          
          
          
        }


        //=====Vis 

        

      }


    private://membership variable
      //! config  
      Param param_;

      //! variables  
      bgk_interface::BGK21dSPtr bgkInterface_;  
      


      //! ros
      utils::SubSPtrType<utils::PC2MsgType> pc2Sub_;
      utils::PubSPtrType<utils::PC2MsgType> predPC2Pub_;

      //! pcl
      utils::PclCloudXYZIType::Ptr pclPCOrigin_;   
      
      

  } ;
}   
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(mesh_bgk::MeshBGK)   