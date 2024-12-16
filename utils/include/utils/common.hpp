/*
 * @Author: chasey melancholycy@gmail.com
 * @Date: 2024-12-16 13:07:24
 * @FilePath: /mesh_planner/src/mesh_planner/utils/include/utils/common.hpp
 * @Description: 
 * 
 * Copyright (c) 2024 by chasey (melancholycy@gmail.com), All Rights Reserved. 
 */
#ifndef UTILS_COMMON_HPP
#define UTILS_COMMON_HPP

// rclcpp
#include "rclcpp/rclcpp.hpp"

// std_msgs
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"

// nav_mags
#include "nav_msgs/msg/path.hpp"  
#include "nav_msgs/msg/odometry.hpp"

// sensor_msgs
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

// geometry_msgs
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

// visualization_msgs
#include "visualization_msgs/msg/image_marker.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace utils{
    //========== using custom type
    //! std_msgs 
    using BoolMsgType = std_msgs::msg::Bool;
    using Int32MsgType = std_msgs::msg::Int32;
    using Float32MsgType = std_msgs::msg::Float32;

    //! nav_msgs
    using PathMsgType = nav_msgs::msg::Path;
    using OdomMsgType = nav_msgs::msg::Odometry;

    //! sensor_msgs
    using ImuMsgType = sensor_msgs::msg::Imu;
    using PC2MsgType = sensor_msgs::msg::PointCloud2;

    //! geometry_msgs
    using PointMsgType = geometry_msgs::msg::Point;
    using PointStampedMsgType = geometry_msgs::msg::PointStamped;
    
    using QuatMsgType = geometry_msgs::msg::Quaternion;
    using QuatStampedMsgType = geometry_msgs::msg::QuaternionStamped;

    using PoseMsgType = geometry_msgs::msg::Pose;
    using PoseStampedMsgType = geometry_msgs::msg::PoseStamped;

    using TFMsgType = geometry_msgs::msg::Transform;
    using TFStampedMsgType = geometry_msgs::msg::TransformStamped;

    using TwistMsgType = geometry_msgs::msg::Twist;
    using TwistStampedMsgType = geometry_msgs::msg::TwistStamped;

    //! visualization_msgs  
    using MarkerMsgType = visualization_msgs::msg::Marker;

    //========== communication 
    //! pub sub type
    template<typename MsgT> // publisher
    using PubSPtrType = typename rclcpp::Publisher<MsgT>::SharedPtr;

    template<typename MsgT> // subscription
    using SubSPtrType = typename rclcpp::Subscription<MsgT>::SharedPtr;
    
    template<typename MsgT> // service
    using SrvSPtrType = typename rclcpp::Service<MsgT>::SharedPtr;
    
    template<typename MsgT> // client
    using ClientSPtrType = typename rclcpp::Client<MsgT>::SharedPtr;
    
    using TimerSPtrType = rclcpp::TimerBase::SharedPtr; // timer
    


    /**
     * @description: Helper function. The wrapper of callback binding.  
     * @attendion: register callback. 
     * @return {*} std::function  
     */    
    template<typename T, typename Ret, typename... Args>
    std::function<Ret(Args...)> bindCallbackWrapper(Ret(T::*func)(Args...), T* obj) {
        return [obj, func](Args... args) -> Ret {
            return (obj->*func)(args...);
        };
    }


}














#endif