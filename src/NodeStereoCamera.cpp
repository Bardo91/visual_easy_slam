#include "NodeStereoCamera.hpp"

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

  connect(this, &NodeStereoCamera::updatedImage, this, &NodeStereoCamera::onTextEdited);
  connect(mPlayButton, &QPushButton::clicked, this, &NodeStereoCamera::togglePause);

  mImageAcquisitionThread = std::thread([&](){
    rgbd::StereoCamera *camera = rgbd::StereoCamera::create(rgbd::StereoCamera::eModel::Virtual);

    cjson::Json configFile;

    configFile["input"]["left"] = "/home/bardo91/programming/A-DATASETS/rgbd_dataset_freiburg1_room/rgb/left_%d.png";
    configFile["input"]["right"] = "";
    configFile["input"]["depth"] = "/home/bardo91/programming/A-DATASETS/rgbd_dataset_freiburg1_room/depth/depth_%d.png";
    configFile["input"]["pointCloud"] = "";
    configFile["calibFile"] = "/home/bardo91/programming/A-DATASETS/rgbd_dataset_freiburg1_room/CalibrationFile.xml";
    configFile["stepIdx"]  = (int) 1;

    if(camera->init(configFile)){
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
          mImageMutex.unlock();

          emit updatedImage();
        }
      }
    }
  });
}


QJsonObject NodeStereoCamera::save() const {
  QJsonObject modelJson = NodeDataModel::save();
  return modelJson;
}


void NodeStereoCamera::restore(QJsonObject const &p) {
  
}


unsigned int NodeStereoCamera::nPorts(PortType portType) const {
  unsigned int result = 1;

  switch (portType) {
    case PortType::In:
      result = 0;
      break;

    case PortType::Out:
      result = 3;

    default:
      break;
  }

  return result;
}


void NodeStereoCamera::onTextEdited() {
    emit dataUpdated(0);
    emit dataUpdated(1);
    emit dataUpdated(2);
}


NodeDataType NodeStereoCamera::dataType(PortType, PortIndex) const {
  return ImageData().type();
}


std::shared_ptr<NodeData> NodeStereoCamera::outData(PortIndex _index) {
  std::lock_guard<std::mutex> locker(mImageMutex);
  if(_index == 0){
    return mLeftImageData;    
  }else  if(_index == 1){
    return mRightImageData;
  }else{
    return mDepthImageData;
  }
}