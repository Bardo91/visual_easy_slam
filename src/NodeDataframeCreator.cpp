#include "NodeDataframeCreator.hpp"

#include <rgbd_tools/cjson/json.h>

NodeDataframeCreator::NodeDataframeCreator(){
    mFeatureDetector = cv::ORB::create(1000);//(2000, 4,31,0,3, cv::ORB::HARRIS_SCORE,31,30);
}


unsigned int NodeDataframeCreator::nPorts(PortType portType) const {
  unsigned int result;

  if (portType == PortType::In)
    result = 3;
  else
    result = 2;

  return result;
}


NodeDataType NodeDataframeCreator::dataType(PortType _type, PortIndex _index) const {
  if (_type == PortType::In){
    if(_index == 0){
      return ImageData().type();
    }else if (_index == 1) {
      return ImageData().type();
    }else if (_index == 2) {
      return CalibrationData().type();
    }
  }
  else{
    if(_index == 0){
      return ImageData().type();
    }else if (_index == 1) {
      return DataframeData().type();    
    }
  }
}


std::shared_ptr<NodeData> NodeDataframeCreator::outData(PortIndex portIndex) {
  if(portIndex == 0)
    return std::static_pointer_cast<NodeData>(mDebugImage);
  if(portIndex == 1)
    return std::static_pointer_cast<NodeData>(mDataframe);
}


void NodeDataframeCreator::setInData(std::shared_ptr<NodeData> data, PortIndex portIndex) {
  if(portIndex == 0){
    auto imageData = std::dynamic_pointer_cast<ImageData>(data);
    mInputImageRgb = imageData;
  }else if(portIndex == 1){
    auto imageData = std::dynamic_pointer_cast<ImageData>(data);
    mInputImageDepth = imageData;
  }else if(portIndex == 2){
    auto calibrationData = std::dynamic_pointer_cast<CalibrationData>(data);
    mCalibration = calibrationData;
    
    if(mInputImageDepth.lock() != nullptr && mInputImageRgb.lock() != nullptr&& mCalibration.lock() != nullptr){
      buildDataframe();
    }
  }
}


NodeValidationState NodeDataframeCreator::validationState() const {
  return modelValidationState;
}


QString NodeDataframeCreator::validationMessage() const {
  return modelValidationError;
}

void NodeDataframeCreator::buildDataframe() {
  mDataframe = std::shared_ptr<DataframeData>(new DataframeData);
  undistort(mInputImageRgb.lock()->image(), mDataframe->mDataframe->left, mCalibration.lock()->intrinsics(), mCalibration.lock()->distCoefficients());


  // Detect and compute descriptors
  cv::Mat descriptors;
  std::vector<cv::KeyPoint> kpts;
  
  cv::Mat leftGrayUndistort;
  cv::cvtColor(mDataframe->mDataframe->left, leftGrayUndistort, CV_BGR2GRAY);
  mFeatureDetector->detectAndCompute(leftGrayUndistort, cv::Mat(), kpts, descriptors);
/*
  pcl::PointCloud<PointType_> cloud;
  _df->cloud = cloud.makeShared();
*/

/*
  // Create feature cloud.
  _df->featureCloud = pcl::PointCloud<PointType_>::Ptr(new pcl::PointCloud<PointType_>());
  for (unsigned k = 0; k < kpts.size(); k++) {
      cv::Point3f point;
      if (((rgbd::StereoCameraVirtual *)mCamera)->colorPixelToPoint(kpts[k].pt, point)) { // Using coordinates of distorted points to match depth 
          float dist = sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
          if (!std::isnan(point.x) && dist > 0.25 && dist < 6.0) { // 666 min and max dist? 
              PointType_ pointpcl;
              pointpcl.x = point.x;
              pointpcl.y = point.y;
              pointpcl.z = point.z;
              pointpcl.r = 255;
              pointpcl.g = 0;
              pointpcl.b = 0;
              _df->featureCloud->push_back(pointpcl);
              _df->featureDescriptors.push_back(descriptors.row(k)); // 666 TODO: filter a bit?
              _df->featureProjections.push_back(distortedPoints[k]);    //  Store undistorted points
          }
      }
  }
*/
  // Filling new dataframe
  mDataframe->mDataframe->orientation = Eigen::Matrix3f::Identity();
  mDataframe->mDataframe->position = Eigen::Vector3f::Zero();
  mDataframe->mDataframe->id = mDfCounter;
  mDfCounter++;   // TODO: Database info?



  //////////////
  cv::Mat debugImage;
  cv::drawKeypoints(mDataframe->mDataframe->left, kpts, debugImage);
  mDebugImage = std::shared_ptr<ImageData>(new ImageData(debugImage, ImageData::eImageType::RGB));



  emit dataUpdated(0);
  emit dataUpdated(1);
}