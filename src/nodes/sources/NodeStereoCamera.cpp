//
//
//
//
//

#include <nodes/sources/NodeStereoCamera.hpp>
#include <data_types/StringData.hpp>

#include <QtCore/QJsonValue>
#include <QtGui/QDoubleValidator>
#undef foreach

#include <rgbd_tools/StereoCamera.h>
#include <chrono>

NodeStereoCamera::NodeStereoCamera() : mPlayButton(new QPushButton("Paused")) {
  mLeftImageData = std::shared_ptr<ImageData>(new ImageData(ImageData::eImageType::RGB));
  mRightImageData = std::shared_ptr<ImageData>(new ImageData(ImageData::eImageType::RGB));
  mDepthImageData = std::shared_ptr<ImageData>(new ImageData(ImageData::eImageType::DEPTH16));

  mPlayButton->setMaximumSize(mPlayButton->sizeHint());

  connect(this, &NodeStereoCamera::updatedImage, this, &NodeStereoCamera::onDataUpdated);
  connect(mPlayButton, &QPushButton::clicked, this, &NodeStereoCamera::togglePause);
}

void NodeStereoCamera::imageAcquisitionThread(){
  rgbd::StereoCamera *camera = rgbd::StereoCamera::create(mConfigFile["type"]);

  if(camera && camera->init(mConfigFile["config"])){
    mRunning = true;
    while(mRunning){
      if(mPaused){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }else{
        cv::Mat left, right, depth;
        camera->grab();
        camera->rgb(left, right);
        camera->depth(depth);

        mImageMutex.lock();
        mLeftImageData = std::shared_ptr<ImageData>(new ImageData(left, ImageData::eImageType::RGB));
        mRightImageData = std::shared_ptr<ImageData>(new ImageData(right, ImageData::eImageType::RGB));
        mDepthImageData = std::shared_ptr<ImageData>(new ImageData(depth, ImageData::eImageType::DEPTH16));
        cv::Mat intrinsics, coeffs;
        camera->leftCalibration(intrinsics, coeffs);
        mCalibration = std::shared_ptr<CalibrationData>(new CalibrationData(intrinsics, coeffs));
        mImageMutex.unlock();

        emit updatedImage(); 
        mPaused = true;
      }
    }
  }else{
    modelValidationState = NodeValidationState::Warning;
    modelValidationError = QStringLiteral("Failed camera initialization");
  }
}

QJsonObject NodeStereoCamera::save() const {
  QJsonObject modelJson = NodeDataModel::save();
  return modelJson;
}


void NodeStereoCamera::restore(QJsonObject const &p) {
  
}

void NodeStereoCamera::setInData(std::shared_ptr<NodeData> _data, int _portIndex) { 
  if (_portIndex == 0) { // Calibration filepath
    auto stringData = std::dynamic_pointer_cast<StringData>(_data);

    if (stringData){
      std::string configFilePath = stringData->string().toStdString();
      std::ifstream file(configFilePath);
      if (!file.is_open()) {
          modelValidationState = NodeValidationState::Warning;
          modelValidationError = QStringLiteral("Config file not found");
          return;
      }
      if (!mConfigFile.parse(file)) {    
          modelValidationState = NodeValidationState::Warning;
          modelValidationError = QStringLiteral("Failed parse of config file");
          return;
      }
      mRunning = false;
      if(mImageAcquisitionThread.joinable())
        mImageAcquisitionThread.join();

      modelValidationState = NodeValidationState::Valid;
      modelValidationError = QString();
      mImageAcquisitionThread = std::thread(&NodeStereoCamera::imageAcquisitionThread, this);
    }else{

    }
  }
}


unsigned int NodeStereoCamera::nPorts(PortType portType) const {
  unsigned int result = 1;

  switch (portType) {
    case PortType::In:
      result = 1;
      break;

    case PortType::Out:
      result = 4;

    default:
      break;
  }

  return result;
}



NodeDataType NodeStereoCamera::dataType(PortType _portType, PortIndex _index) const {
  switch (_portType) {
    case PortType::In:
      return StringData().type();
      break;
    case PortType::Out:
      if(_index == 0){
        return ImageData().type();
      }else if (_index == 1) {
        return ImageData().type();    
      }if(_index == 2){
        return ImageData().type();
      }else if (_index == 3) {
        return CalibrationData().type();    
      }
      break;
    default:
      break;
  }

    
}


std::shared_ptr<NodeData> NodeStereoCamera::outData(PortIndex _index) {
  std::lock_guard<std::mutex> locker(mImageMutex);

  if(_index == 0){
    return mLeftImageData;    
  }else  if(_index == 1){
    return mRightImageData;
  }else if(_index == 2){
    return mDepthImageData;
  }else if(_index == 3){
    return mCalibration;
  }
}
