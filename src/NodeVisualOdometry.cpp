#include "NodeVisualOdometry.hpp"


unsigned int NodeVisualOdometry::nPorts(PortType portType) const {
  unsigned int result;

  if (portType == PortType::In)
    result = 1;
  else
    result = 2;

  return result;
}


NodeDataType NodeVisualOdometry::dataType(PortType _type, PortIndex _index) const {
  if (_type == PortType::In){
    return ImageData().type();
  }
  else{
    if(_index == 0){
      return ImageData().type();
    }else if (_index == 1) {
      return PoseData().type();    
    }
  }
}


std::shared_ptr<NodeData> NodeVisualOdometry::outData(PortIndex portIndex) {
  if(portIndex == 0)
    return std::static_pointer_cast<NodeData>(mDebugImage);
  if(portIndex == 1)
    return std::static_pointer_cast<NodeData>(mResultPose);
}


void NodeVisualOdometry::setInData(std::shared_ptr<NodeData> data, PortIndex portIndex) {
  auto imageData = std::dynamic_pointer_cast<ImageData>(data);

  mInputImage = imageData;

  compute();
}


NodeValidationState NodeVisualOdometry::validationState() const {
  return modelValidationState;
}


QString NodeVisualOdometry::validationMessage() const {
  return modelValidationError;
}

void NodeVisualOdometry::compute() {
  cv::Mat src = mInputImage.lock()->image().clone();

  Eigen::Matrix4f randomPose = Eigen::Matrix4f::Random();
  mResultPose = std::shared_ptr<PoseData>(new PoseData(randomPose));

  std::vector<cv::KeyPoint> keypointsD;
  
  cv::Ptr<cv::FastFeatureDetector> detector= cv::FastFeatureDetector::create();
  detector->detect(src, keypointsD, cv::Mat());
  
  cv::drawKeypoints(src, keypointsD, src);

  mDebugImage = std::shared_ptr<ImageData>(new ImageData(src));

  emit dataUpdated(1);
  emit dataUpdated(0);
}