
#ifndef CalibrationData_H_
#define CalibrationData_H_

#include <nodes/NodeDataModel>
#include <opencv2/opencv.hpp>

#include <rgbd_tools/map3d/DataFrame.h>

using QtNodes::NodeDataType;
using QtNodes::NodeData;

/// The class can potentially incapsulate any user data which
/// need to be transferred within the Node Editor graph
class CalibrationData : public NodeData {
  public:
    CalibrationData(){};
    
    CalibrationData(const cv::Mat &_intrinsics, const cv::Mat &_distCoeff){
      mIntrinsics  = _intrinsics.clone();
      mDistCoeff = _distCoeff.clone();
    }

    NodeDataType type() const override {
      return NodeDataType {"calibration", "Calibration"};
    }

    cv::Mat intrinsics() const { 
      return mIntrinsics; 
    }

    cv::Mat distCoefficients() const { 
      return mDistCoeff; 
    }

  private:

  cv::Mat mIntrinsics, mDistCoeff;
};

#endif