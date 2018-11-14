//
//
//
//
//

#include <nodes/operations/NodeVisualOdometry.hpp>

#include <rgbd_tools/cjson/json.h>

NodeVisualOdometry::NodeVisualOdometry(){
  mOdometry = new rgbd::OdometryRgbd<pcl::PointXYZRGBNormal, rgbd::DebugLevels::Debug>();
  
  cjson::Json mConfigFile;
  
  mConfigFile["descriptorDistanceFactor"] = (float) 35.0;
  mConfigFile["ransacIterations"]  = (int) 1000;
  mConfigFile["ransacMaxDistance"]  = (float) 0.03;
  mConfigFile["ransacMinInliers"]  = (int) 20;
  mConfigFile["ransacRefineIterations"]  = (int) 5;
  mConfigFile["icpEnabled"]  = (int) 0;
  mConfigFile["icpMaxCorrespondenceDistance"]  = (float) 0.3;
  mConfigFile["icpMaxFitnessScore"]  = (float) 5.0;
  mConfigFile["icpMaxIterations"]  = (int) 3;
  mConfigFile["icpMaxTransformationEpsilon"]  = (float) 0.00001;
  mConfigFile["icpVoxelDistance"]  = (float) 0.1;
  mConfigFile["knearestneighbors"]  = (int) 1;

  if (!mOdometry->init(mConfigFile)) {
      std::cout << "Error initializing odometry parameters" << std::endl;
  }
}


unsigned int NodeVisualOdometry::nPorts(PortType portType) const {
  unsigned int result;

  if (portType == PortType::In)
    result = 1;
  else
    result = 1;

  return result;
}


NodeDataType NodeVisualOdometry::dataType(PortType _type, PortIndex _index) const {
  if (_type == PortType::In){
    return DataframeData().type();
  }
  else{
    return PoseData().type();    
    
  }
}


std::shared_ptr<NodeData> NodeVisualOdometry::outData(PortIndex portIndex) {
  return std::static_pointer_cast<NodeData>(mResultPose);
}


void NodeVisualOdometry::setInData(std::shared_ptr<NodeData> data, PortIndex portIndex) {
  auto dataframe = std::dynamic_pointer_cast<DataframeData>(data);

  if(dataframe){
    mDataframe = dataframe;
    compute();
  }
}


NodeValidationState NodeVisualOdometry::validationState() const {
  return modelValidationState;
}


QString NodeVisualOdometry::validationMessage() const {
  return modelValidationError;
}

void NodeVisualOdometry::compute() {
  auto df = mDataframe.lock()->mDataframe;
  if(mLastCluster == nullptr){
    mLastCluster = std::shared_ptr<rgbd::ClusterFrames<pcl::PointXYZRGBNormal>>(new rgbd::ClusterFrames<pcl::PointXYZRGBNormal>(df, df->id));
  }else{
    if (mOdometry->computeOdometry(mLastCluster, df)) {
      mResultPose = std::shared_ptr<PoseData>(new PoseData(df->pose));
      emit dataUpdated(0);
    }
  }
}