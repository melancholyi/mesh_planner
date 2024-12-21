/*
 * @Author: chasey melancholycy@gmail.com
 * @Date: 2024-12-18 12:28:26
 * @FilePath: /mesh_planner/src/mesh_planner/utils/include/utils/voxel_map.hpp
 * @Description: 
 * 
 * Copyright (c) 2024 by chasey (melancholycy@gmail.com), All Rights Reserved. 
 */
#ifndef VOXEL_MAP_HPP
#define VOXEL_MAP_HPP

#include "utils/voxel.hpp"  
#include "utils/common.hpp"  

//========== cpp stl
#include <unordered_map>
#include <map>
#include <set>
#include <list>

//========== 
  
namespace utils{  

  
  
  class VoxelMap{
    private://helper function
    //! SpaceHashMapType 
      struct SpaceHashFunctor {
        inline size_t operator()(const Eigen::Array3i& key) const {
          return size_t( ((key[0]) * long(73856093)) 
                        ^((key[1]) * long(471943)) 
                        ^((key[2]) * long(83492791))) 
                        % size_t(1000000000);
          }
      };
      struct XYHashFunctor {
        inline size_t operator()(const std::pair<int,int>& xy) const {
          return size_t( ((xy.first) * long(73856093)) 
                        ^((xy.second) * long(83492791))) 
                        % size_t(1000000000);
          }
      };
      struct XYEqualFunctor {
        bool operator()(const std::pair<int, int> &lhs, const std::pair<int, int> &rhs) const {
            return lhs.first == rhs.first && lhs.second == rhs.second;
        }
      };
      struct EqualWithFunctor {
        inline bool operator()(const Eigen::Array3i& a, const Eigen::Array3i& b) const {
            return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
        }
      };
    public:
      using SpaceHashMapType = std::unordered_map<
            Eigen::Array3i, 
            typename std::list<std::pair<Eigen::Array3i, std::unique_ptr<Voxel>>>::iterator, 
            SpaceHashFunctor,
            EqualWithFunctor>;
      using XYHashZSetType = std::unordered_map<std::pair<int,int>, std::set<int>, XYHashFunctor, XYEqualFunctor>;
      using BoundaryPairType = std::pair<Eigen::Array3i, Eigen::Array3i>;

    public://membership function
    
      VoxelMap(const Eigen::Array3d &res, const int &minPtsPerVoxel = 3):  
        res_(res), invRes_(Eigen::Array3d::Ones()/res_), minPtsPerVoxel_(minPtsPerVoxel){  
          boundary_ = std::make_pair(Eigen::Array3i::Zero(), Eigen::Array3i::Zero()); 
          voxelNum_.fill(0);  
          ijkMulti_.fill(0);
      }

      void setInputPclCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
          
        SpaceHashMapType update_list;    
        
        //! insert to voxel
        for (auto pt : cloud->points){
          auto idx3d = pos2idx(pt);
          auto iter = voxelsMap_.find(idx3d);
          //!!  
          if(iter != voxelsMap_.end()){// find it, should handle exception  

          } else { // insert new voxel  
            voxelsList_.push_front(std::make_pair(idx3d, std::make_unique<Voxel>(idx3d, res_)));
            voxelsMap_.insert(std::make_pair(idx3d, voxelsList_.begin()));
            iter = voxelsMap_.find(idx3d);
          }
          update_list.insert(*iter);
        }

        //! update voxels and boundary  
        for (auto &it : update_list){
          auto voxel = *(it.second->second);  
          bool flag = voxel.updateState(minPtsPerVoxel_);
          if(flag){ //update boundary  
            Eigen::Array3i min_b = boundary_.first, max_b = boundary_.second;  
            auto key3d = it.second->first;  
            min_b[0] = key3d[0] < min_b[0] ? key3d[0] : min_b[0];
            min_b[1] = key3d[1] < min_b[1] ? key3d[1] : min_b[1];
            min_b[2] = key3d[2] < min_b[2] ? key3d[2] : min_b[2];
            max_b[0] = key3d[0] > max_b[0] ? key3d[0] : max_b[0];
            max_b[1] = key3d[1] > max_b[1] ? key3d[1] : max_b[1];
            max_b[2] = key3d[2] > max_b[2] ? key3d[2] : max_b[2];
            boundary_.first = min_b; boundary_.second = max_b;
            
            voxelNum_ = boundary_.second - boundary_.first + Eigen::Array3i::Ones();
            ijkMulti_(0) = 1;
            ijkMulti_(1) = voxelNum_(0);
            ijkMulti_(2) = voxelNum_(0) * voxelNum_(1);  

            //!! update gridZset  
            auto grid = voxel.getIndex3d();
            auto mean = voxel.getMeanAndCov().first;
            gridZSet_[std::pair<int,int>{grid(0), grid(1)}].insert(mean(2));    
          }
        }
      }  
      
      VoxelConstRawPtr getVoxel(const Eigen::Array3d& pos){
        auto it = voxelsMap_.find(pos2idx(pos));
        return it->second->second.get();
      }
      std::set<int> getZSet(const int &x, const int &y){
        auto it = gridZSet_.find(std::pair<int,int>{x, y});
        if (it != gridZSet_.end()) {
            return it->second;
        } else {
            // Return an empty set if the key is not found
            return std::set<int>();  
        }
      }

      MarkerArrayMsgType getVisMarkerArray(){
        utils::MarkerArrayMsgType marker_array;
        for(auto &iter : voxelsList_){  
          auto voxel = *(iter.second);      
          

        }
        return marker_array;

      }
        
    
    
    private://membership function
      inline Eigen::Array3i pos2idx(const pcl::PointXYZ &posPcl){  
        Eigen::Array3d pos_eigen(posPcl.x, posPcl.y, posPcl.z);
        return (pos_eigen * invRes_).cast<int>();
      }
      inline Eigen::Array3i pos2idx(const Eigen::Array3d &posEigen){
        return (posEigen * invRes_).cast<int>();
      }
      // inline size_t idx2key(const Eigen::Array3i &idx){
      //   return (idx * ijkMulti_).sum();
      // }
      // inline size_t pos2key(const Eigen::Array3d &pos){
      //   return idx2key(pos2idx(pos));
      // }

    
    public://membership variable  
    
    
    
    private://membership variable
      Eigen::Array3d res_;    //map resolution, maybe extend to un-uniform   
      Eigen::Array3d invRes_; // inverse map resulution  
      int minPtsPerVoxel_;
      
      BoundaryPairType boundary_;
      Eigen::Array3i voxelNum_;
      Eigen::Array3i ijkMulti_;  //index multiplication
      
      SpaceHashMapType voxelsMap_;
      std::list<std::pair<Eigen::Array3i, std::unique_ptr<Voxel>>> voxelsList_;

      XYHashZSetType gridZSet_;

     






  };





}


#endif //VOXEL_MAP_HPP