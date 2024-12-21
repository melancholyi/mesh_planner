/*
 * @Author: chasey melancholycy@gmail.com
 * @Date: 2024-12-17 06:09:52
 * @FilePath: /mesh_planner/src/mesh_planner/utils/include/utils/voxel.hpp
 * @Description: 
 * 
 * Copyright (c) 2024 by chasey (melancholycy@gmail.com), All Rights Reserved. 
 */

#ifndef VOXEL_HPP
#define VOXEL_HPP

#include <vector>  
//! eigen
#include <Eigen/Core>
#include <Eigen/Dense>
//! pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

//! utils/common.h  
#include "utils/common.hpp"
  
namespace utils{
  const double LARGE_VAR = 100;//variance=100 means verry large
  struct PointsWithVar{
      Eigen::Matrix3Xd points;
      Eigen::Matrix<double, 1, Eigen::Dynamic> variance;
      int count;  
      const int resizeSize = 64;  

      PointsWithVar(int size){
        points.resize(3, size); points.fill(0);
        variance.resize(1, size); variance.fill(LARGE_VAR); 
        count = 0;
      }
      void addPoints(const Eigen::Matrix3Xd &pts){
        //! insert points to Voxel  
        int cnt_col = pts.cols();  
        if(count + cnt_col > (int)points.cols()){ 
          auto step = resizeSize * ((cnt_col + resizeSize)/resizeSize);   
          points.conservativeResize(Eigen::NoChange_t(3), points.cols() + step);
          variance.conservativeResize(Eigen::NoChange_t(1), points.cols() + step);
          points.rightCols(step).fill(0);
          variance.rightCols(step).fill(LARGE_VAR);
        }  
        for(int i = 0 ; i < cnt_col; i++){  
          points.col(count + i) = pts.col(i);    
        }
      }
      void clear(){
        points.fill(0); 
        variance.fill(0);
        count = 0; 
      }
  };
  
  class Voxel{
    
    public://membership function  
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /**
       * @description: Voxel class's construction  
       */      
      Voxel(const Eigen::Array3i &idx3d, const Eigen::Array3d &voxelLWH, int initSize = 64, bool isPtsCov = false): 
        points_(initSize), pointsNew_(0), isPtsWithCov_(isPtsCov), index3d_(idx3d), location_(index3d_.cast<double>() * voxelLWH), voxelLWH_(voxelLWH), centriod_(location_ + voxelLWH_/2){       
        // index_.fill(0);
        // keyIdx_ = 0;
        mean_.fill(0);
        cov_.fill(0);
        invCov_.fill(0);
        eigenValue_.fill(0); 
        eigenVecs_.fill(0);  
        minVecVal_.fill(0);
        std::cout << centriod_ << std::endl;
      }
      inline void insertPoint(const Eigen::Vector3d &pt){
        insertPoints(pt);
      }
      inline void insertPoint(const pcl::PointXYZ &pt){
        insertPoint(Eigen::Vector3d(pt.x, pt.y, pt.z));
      }
      inline void insertPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pclPts){  
        Eigen::Matrix3Xd pts = Eigen::Matrix3Xd::Zero(3, pclPts->points.size());
        for (size_t i = 0; i < pclPts->points.size(); ++i) {
          pts(0, i) = pclPts->points[i].x; // x-coordinate
          pts(1, i) = pclPts->points[i].y; // y-coordinate
          pts(2, i) = pclPts->points[i].z; // z-coordinate
        }
        insertPoints(pts);
      }
      void insertPoints(const Eigen::Matrix3Xd &pts){  
        // TODO: invalid points check, such insert an point which shouldn't belong to this voxel  
        
        pointsNew_.addPoints(pts);  //  
        pointsNew_.count += pts.cols();//update count 
      }
      // void insertPointsWithVar(const std::vector<Eigen::Vector4d> &ptsVar);
      // void insertPointsWithVar(const Eigen::Matrix4Xd &ptsVar);

      bool updateState(const int &updateThres = 3){    
        auto pts = pointsNew_.points.block<3,-1>(0, 0, 3, pointsNew_.count);  
        if(pts.cols() < updateThres) {
          pointsNew_.clear();  
          return false;
        }
        
        //! pointsNew -> points
        std::cout << "\npoints new:\n" << pointsNew_.points << std::endl;
        std::cout << "\npoints new count: " << pointsNew_.count << std::endl;
        points_.addPoints(pts);   
        std::cout << "\npoints :\n" << points_.points << std::endl;
        std::cout << "\npoints  count: " << points_.count << std::endl;

        //! compute mean and cov of new points data   
        Eigen::Vector3d mean_inc;  
        Eigen::Matrix3d cov_inc;  
        int cnt_col = pts.cols();
        computeMeanAndCov(pts, cnt_col, mean_inc, cov_inc);
        std::cout <<  "\ncnt_col :\n" << cnt_col << std::endl;
        std::cout <<  "\nmean_new :\n" << mean_inc << std::endl;
        std::cout <<  "\ncov_inc :\n" << cov_inc << std::endl;

        //! incremental update mean and cov       
        updateMeanAndCov(pts.cols(), mean_inc, cov_inc);
        std::cout <<  "\nfinally cnt_col :\n" << points_.count << std::endl;
        std::cout <<  "\nfinally mean :\n" << mean_ << std::endl;
        std::cout <<  "\nfinally cov :\n" << cov_ << std::endl;
        std::cout <<  "\nfinally eigen values :\n" << eigenValue_ << std::endl;
        std::cout <<  "\nfinally eigen vectors :\n" << eigenVecs_ << std::endl;
        
        //! clear point new  
        pointsNew_.clear();
        return true;
      }

      Eigen::Array3i getIndex3d() const{
        return index3d_;
      }

      std::pair<Eigen::Vector3d, Eigen::Matrix3d> getMeanAndCov() const{
        return std::make_pair(mean_, cov_);
      }
      std::pair<Eigen::Vector3d, Eigen::Matrix3d> getEigenValueAndVecs() const{
        return std::make_pair(eigenValue_, eigenVecs_);  
      }
      Eigen::Vector4d& getMinValAndVec(){
        return minVecVal_;
      }
    
      void clear(){

      }

      inline utils::MarkerMsgType getGaussianEllipsoid(const utils::HeaderMsgType &header, const float &scale){
        static int32_t id = 0;
        utils::MarkerMsgType marker;
        marker.header = header;
        marker.ns = "voxel_gaussian";  
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        //! pose
        marker.pose.position.x = mean_(0);
        marker.pose.position.y = mean_(1);
        marker.pose.position.z = mean_(2);

        Eigen::Quaterniond q;
        if (eigenVecs_.determinant() < 0) {
          Eigen::Matrix3d flipZ180 = Eigen::Matrix3d::Identity();
          flipZ180(2,2) = -1;
          q = eigenVecs_ * flipZ180;
        } else {
          q = eigenVecs_;
        }
        q.normalize();
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        // marker.scale.x = sqrt(eigenValue_(0)) * scale; // 2 sigma ~95,4%
        // marker.scale.y = sqrt(eigenValue_(1)) * scale; // 2 sigma ~95,4%
        // marker.scale.z = sqrt(eigenValue_(2)) * scale; // 2 sigma ~95,4%
        marker.scale.x = scale * eigenValue_(0) >= 0.01 ? sqrt(eigenValue_(0)) : 0.01; 
        marker.scale.y = scale * eigenValue_(1) >= 0.01 ? sqrt(eigenValue_(1)) : 0.01; 
        marker.scale.z = scale * eigenValue_(2) >= 0.01 ? sqrt(eigenValue_(2)) : 0.01; 

        // cast normal direction to rgb value.    
        float value = std::fabs(minVecVal_(3));      
        auto factor = value > 1.f ? 1.f : value;
        factor = value < 0.f ? 0.f : value;
        auto rgb = Eigen::Vector3f(factor, 1 - factor, 0);
        marker.color.a = 0.5;  
        marker.color.r = rgb(0);
        marker.color.g = rgb(1);
        marker.color.b = rgb(2);

        return marker;

      }
      inline utils::MarkerMsgType getVoxelLine(){
        utils::MarkerMsgType marker;
        return marker;
      }

      


    private://membership function    
      inline void computeMeanAndCov(const Eigen::Matrix3Xd &points, const int ptcnt, Eigen::Vector3d &mean, Eigen::Matrix3d &cov){  
        mean = points.leftCols(ptcnt).rowwise().sum() / ptcnt;
        auto diff = points.leftCols(ptcnt).colwise() - mean;
        cov = diff * diff.transpose() / (ptcnt - 1);
      }

      inline void updateMeanAndCov(const int &cntInc, const Eigen::Vector3d &meanInc, const Eigen::Matrix3d &covInc){
        double cnt_old = points_.count, cnt_inc = cntInc, cnt_new = 0;
        Eigen::Vector3d mean_old = mean_, mean_new;    
        Eigen::Matrix3d cov_old = cov_, cov_new;  

        //! compute updated cnt mean and cov  
        cnt_new  = cnt_old + cnt_inc;
        mean_new = (cnt_inc * meanInc + cnt_old * mean_old) / cnt_new;
        cov_new  = (cnt_inc *  covInc + cnt_old *  cov_old + cnt_inc * cnt_old / cnt_new * meanInc * meanInc.transpose()) / cnt_new;
        
        //! update
        points_.count = cnt_new;
        mean_ = mean_new;
        cov_ = cov_new;
        invCov_ = cov_.inverse();

        //! update eigenValue, eigenVector, minVecVal_  
        eigenSlover_.compute(cov_);
        eigenValue_ = eigenSlover_.eigenvalues();
        eigenVecs_ = eigenSlover_.eigenvectors();  
        minVecVal_ << eigenValue_(0), eigenVecs_(0,0), eigenVecs_(1,0), eigenVecs_(2,0);
      } 
       


    public://membership variable

    private://membership variable
      PointsWithVar points_, pointsNew_;  

      bool isPtsWithCov_;
      Eigen::Array3i index3d_;
      Eigen::Array3d location_;  // voxel corner(Fort-Left-Down) position
      Eigen::Array3d voxelLWH_;  // voxel size, LWH(XYZ)
      Eigen::Array3d centriod_;   // voxel center position
      // Eigen::Array3i index_;     // index in map
      // size_t keyIdx_;          // hash key
      
      Eigen::Vector3d mean_;
      Eigen::Matrix3d cov_;
      Eigen::Matrix3d invCov_;

      //
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSlover_;
      Eigen::Vector3d eigenValue_;
      Eigen::Matrix3d eigenVecs_;
      Eigen::Vector4d minVecVal_; // min eigenVal[3] && corresponding Vector[0-2]  
      // Eigen::Array3i index_;
    public:
      using UPtr = std::unique_ptr<Voxel>;
  };
  using VoxelConstSPtr = std::shared_ptr<Voxel const>;
  using VoxelSPtr = std::shared_ptr<Voxel>;
  
  using VoxelRawPtr = Voxel*;  
  using VoxelConstRawPtr = const Voxel*;

  using VoxelUPtr = std::unique_ptr<Voxel>;
}





#endif //VOXEL_HPP
