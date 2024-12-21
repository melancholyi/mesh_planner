/*
 * @Author: chasey melancholycy@gmail.com
 * @Date: 2024-12-09 15:02:28
 * @FilePath: /mesh_planner/src/mesh_planner/mesh_bgk/include/mesh_bgk/bgk_interface.hpp
 * @Description: 
 * 
 * Copyright (c) 2024 by chasey (melancholycy@gmail.com), All Rights Reserved. 
 */
#ifndef BGK_INTERFACE_H
#define BGK_INTERFACE_H

// Eigen  
#include <Eigen/Core>
#include <memory>

namespace bgk_interface{
    
  template<int INPUT_DIM, int OUTPUT_DIM, typename T>
  class BGKInterface{
  public://membership function
    using InputType  = Eigen::Matrix<T, -1, INPUT_DIM>;  // n *  input_dim
    using OutputType = Eigen::Matrix<T, -1, OUTPUT_DIM>; // n * output_dim
    using KernelType = Eigen::Matrix<T, -1, -1>; // 
    BGKInterface(double kLen, double kScalar): kernelLen_(kLen), kernelScaler_(kScalar), isRcvTrainData_(false){}


    void train(const std::vector<T> &trainX, const std::vector<T> &trainY) {
      assert(trainX.size() % INPUT_DIM == 0 && trainY.size() % OUTPUT_DIM == 0);  
      InputType  temp_x = Eigen::Map<const InputType> (trainX.data(), trainX.size()/INPUT_DIM , INPUT_DIM);
      OutputType temp_y = Eigen::Map<const OutputType>(trainY.data(), trainY.size()/OUTPUT_DIM, OUTPUT_DIM);
      this->trainX_ = InputType(temp_x);
      this->trainY_ = OutputType(temp_y);
      isRcvTrainData_ = true;
    }

    void predict(const std::vector<T> &predX, std::vector<T> &ybar, std::vector<T> &kbar){
      assert(predX.size() % INPUT_DIM == 0 && isRcvTrainData_ == true);
      if(!this->isRcvTrainData_) return;
      InputType temp_predX = Eigen::Map<const InputType>(predX.data(), predX.size()/INPUT_DIM , INPUT_DIM);
      InputType temp_ybar, temp_kbar;

      KernelType kernel;  
      covSparse(predX, this->trainX_, kernel);  
      ybar = (kernel * this->trainY_).array();
      kbar = kernel.rowwise().sum().array();
    }


  private://membership function  
  
    void computeDistMat(const InputType &predX, const InputType &trainX, KernelType &dist){
      dist = KernelType::Zero(predX.rows(), trainX.rows());
      for (int i = 0; i < predX.rows(); i++) {
          dist.row(i) = (trainX.rowwise() - predX.row(i)).rowwise().norm();
      }
    }

    void covSparse(const InputType &predX, const InputType &trainX, KernelType &kernel) const {
      dist(predX / this->kernelLen_, trainX / this->kernelLen_, kernel);
      kernel = (((2.0f + (kernel * 2.0f * 3.1415926f).array().cos()) * (1.0f - kernel.array()) / 3.0f) +
            (kernel * 2.0f * 3.1415926f).array().sin() / (2.0f * 3.1415926f)).matrix() * this->kernelScaler_;

      // Clean up for values with distance outside length scale
      // Possible because Kxz <= 0 when dist >= this->kernelLen_
      for (int i = 0; i < kernel.rows(); ++i)
      {
          for (int j = 0; j < kernel.cols(); ++j)
              if (kernel(i,j) < 0.0)
                  kernel(i,j) = 0.0f;
      }
    }


  private://membership variables  
    InputType  trainX_;
    OutputType trainY_;
    T kernelLen_;
    T kernelScaler_; 
    bool isRcvTrainData_;  
  };
  typedef BGKInterface<2, 1, float>  BGK21fType;
  typedef BGKInterface<2, 1, double> BGK21dType;
  typedef std::shared_ptr<BGKInterface<2, 1, float>> BGK21fSPtr;
  typedef std::shared_ptr<BGKInterface<2, 1, double>> BGK21dSPtr;
}
#endif //GK_INTERFACE_H
