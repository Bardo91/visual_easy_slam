//
//
//
//
// 

#include <nodes/operations/NodeDataframeCreator.hpp>

#include <rgbd_tools/cjson/json.h>
#include <chrono>

NodeDataframeCreator::NodeDataframeCreator(){
    mFeatureDetector = cv::ORB::create(1000);//(2000, 4,31,0,3, cv::ORB::HARRIS_SCORE,31,30);
}


QString NodeDataframeCreator::caption() const { 
  return QStringLiteral("Dataframe Creator"); 
}

bool NodeDataframeCreator::portCaptionVisible(PortType portType, PortIndex portIndex) const {
  Q_UNUSED(portType); Q_UNUSED(portIndex);
  return true;
}

QString  NodeDataframeCreator::portCaption(PortType portType, PortIndex portIndex) const  {
  switch (portType) {
    case PortType::In:
      if (portIndex == 0)
        return QStringLiteral("rgb_image");
      if (portIndex == 1)
        return QStringLiteral("depth_image");
      if (portIndex == 2)
        return QStringLiteral("calibration");      
      break;
    case PortType::Out:
      if (portIndex == 0)
        return QStringLiteral("debug_image");
      else if (portIndex == 1)
        return QStringLiteral("dataframe");

    default:
      break;
  }
  return QString();
}

QString  NodeDataframeCreator::name() const  { 
  return QStringLiteral("Dataframe Creator"); 
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
  if (portIndex == 0) {
    auto imageData = std::dynamic_pointer_cast<ImageData>(data);
    if (imageData)
      mInputImageRgb = imageData;
  }
  else if (portIndex == 1) {
    auto imageData = std::dynamic_pointer_cast<ImageData>(data);
    if (imageData)
      mInputImageDepth = imageData;
  }
  else if (portIndex == 2) {
    auto calibrationData = std::dynamic_pointer_cast<CalibrationData>(data);
    if (calibrationData){
      mCalibration = calibrationData;

      if (mInputImageDepth.lock() != nullptr && mInputImageRgb.lock() != nullptr && mCalibration.lock() != nullptr) {
        buildDataframe();
      }
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
  // Create dataframe
  mDataframe = std::shared_ptr<DataframeData>(new DataframeData);
  
  // Set basic raw data
  cv::Mat intrinsics = mCalibration.lock()->intrinsics();
  cv::Mat coeffs = mCalibration.lock()->distCoefficients();
  undistort(mInputImageRgb.lock()->image(), mDataframe->mDataframe->left, intrinsics, coeffs);
  mDataframe->mDataframe->intrinsic  = intrinsics;
  mDataframe->mDataframe->coefficients  = coeffs;
  mDataframe->mDataframe->depth = mInputImageDepth.lock()->image();
  mDataframe->mDataframe->id = mDfCounter;
  mDfCounter++;
  
  // Detect and compute descriptors
  cv::Mat descriptors;
  std::vector<cv::KeyPoint> kpts;
  
  cv::Mat leftGrayUndistort;
  cv::cvtColor(mDataframe->mDataframe->left, leftGrayUndistort, CV_BGR2GRAY);
  mFeatureDetector->detectAndCompute(leftGrayUndistort, cv::Mat(), kpts, descriptors);


  // Create feature cloud.
  mDataframe->mDataframe->featureCloud = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
  int pointCounter = 0;
  for (unsigned k = 0; k < kpts.size(); k++) {
      cv::Point3f point;
      cv::Point2f p2d = kpts[k].pt;
      
      // Retrieve the 16-bit depth value and map it into a depth in meters
      uint16_t depth_value = mDataframe->mDataframe->depth.at<uint16_t>(p2d.y, p2d.x);
      float depth_in_meters = depth_value * 0.001;  // 666 depthscale plz
      
      // Set invalid pixels with a depth value of zero, which is used to indicate no data
      if (depth_value == 0)
        continue;
      else {
          float x = (p2d.x - intrinsics.at<float>(0,2)) / intrinsics.at<float>(0,0);
          float y = (p2d.y - intrinsics.at<float>(1,2)) / intrinsics.at<float>(1,1);

          point.x = x*depth_in_meters;
          point.y = y*depth_in_meters;
          point.z = depth_in_meters;
      }

      if (!std::isnan(point.x)) { // 666 min and max dist? 
          pcl::PointXYZRGBNormal pointpcl;
          pointpcl.x = point.x;
          pointpcl.y = point.y;
          pointpcl.z = point.z;
          pointpcl.r = 255;
          pointpcl.g = 0;
          pointpcl.b = 0;
          
          mDataframe->mDataframe->featureCloud->push_back(pointpcl);
          mDataframe->mDataframe->featureDescriptors.push_back(descriptors.row(k));
          mDataframe->mDataframe->featureProjections.push_back(kpts[k].pt);
      }
  }

  cv::Mat debugImage;
  cv::drawKeypoints(mDataframe->mDataframe->left, kpts, debugImage);
  mDebugImage = std::shared_ptr<ImageData>(new ImageData(debugImage, ImageData::eImageType::RGB));

  emit dataUpdated(0);
  emit dataUpdated(1);
}