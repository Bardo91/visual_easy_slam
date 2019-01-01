
#ifndef NODESTEREOCAMERA_H_
#define NODESTEREOCAMERA_H_

#include <QtCore/QObject>
#include <QtWidgets/QPushButton>

#include <nodes/NodeDataModel>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <mutex>
#include <thread>

#include <data_types/ImageData.hpp>
#include <data_types/CalibrationData.hpp>

#include <rgbd_tools/cjson/json.h>


class DecimalData;

using QtNodes::PortType;
using QtNodes::PortIndex;
using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDataModel;
using QtNodes::NodeValidationState;

/// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class NodeStereoCamera : public NodeDataModel
{
  Q_OBJECT

public:
  NodeStereoCamera();

  virtual
  ~NodeStereoCamera() { 
    mRunning = false;
    if(mImageAcquisitionThread.joinable())
      mImageAcquisitionThread.join();
    }

public:

  QString  caption() const override { return QStringLiteral("Stereo Camera"); }

  bool captionVisible() const override { return false; }

  QString name() const override { return QStringLiteral("Stereo Camera"); }

  QString   portCaption(PortType portType, PortIndex portIndex) const override {
    switch (portType) {
      case PortType::In:
        if (portIndex == 0)
            return QStringLiteral("calibration_file");
        else
          break;
      case PortType::Out:
        if (portIndex == 0)
          return QStringLiteral("left_image");
        else if (portIndex == 1)
          return QStringLiteral("right_image");
        else if (portIndex == 2)
          return QStringLiteral("depth_image");
        else if (portIndex == 3)
          return QStringLiteral("calibration");

      default:
        break;
    }
    return QString();
  }

public:

  QJsonObject save() const override;

  void restore(QJsonObject const &p) override;

public:

  unsigned int nPorts(PortType portType) const override;

  NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

  std::shared_ptr<NodeData>  outData(PortIndex port) override;

  void setInData(std::shared_ptr<NodeData>, int) override;

  QWidget * embeddedWidget() override { return mPlayButton; }

private slots:
  void onDataUpdated() { 
    emit dataUpdated(0); 
    emit dataUpdated(1); 
    emit dataUpdated(2); 
    emit dataUpdated(3); 
  }
  void togglePause(){
    mPaused = !mPaused;
    if(mPaused){
      mPlayButton->setText("Paused");
    }else{
      mPlayButton->setText("Playing");
    }
  }

signals:
  void updatedImage();

private:
  void imageAcquisitionThread();

private:
  std::shared_ptr<ImageData> mLeftImageData;
  std::shared_ptr<ImageData> mRightImageData;
  std::shared_ptr<ImageData> mDepthImageData;
  std::shared_ptr<CalibrationData> mCalibration;
  

  std::mutex mImageMutex;

  std::thread mImageAcquisitionThread;
  bool mRunning = true;
  bool mPaused = true;

  QPushButton *mPlayButton;
  cjson::Json mConfigFile;

};

#endif