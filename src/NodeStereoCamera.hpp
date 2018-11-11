
#ifndef NODESTEREOCAMERA_H_
#define NODESTEREOCAMERA_H_

#include <QtCore/QObject>
#include <QtWidgets/QPushButton>

#include <nodes/NodeDataModel>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <mutex>
#include <thread>

#include "ImageData.hpp"

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

  QString  caption() const override { return QStringLiteral("StereoCamera"); }

  bool captionVisible() const override { return false; }

  QString name() const override { return QStringLiteral("StereoCamera"); }

public:

  QJsonObject save() const override;

  void restore(QJsonObject const &p) override;

public:

  unsigned int nPorts(PortType portType) const override;

  NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

  std::shared_ptr<NodeData>  outData(PortIndex port) override;

  void setInData(std::shared_ptr<NodeData>, int) override { }

  QWidget * embeddedWidget() override { return mPlayButton; }

private slots:
  void onTextEdited();
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
  std::shared_ptr<ImageData> mLeftImageData;
  std::shared_ptr<ImageData> mRightImageData;
  std::shared_ptr<ImageData> mDepthImageData;

  std::mutex mImageMutex;

  std::thread mImageAcquisitionThread;
  bool mRunning = true;
  bool mPaused = true;

  QPushButton *mPlayButton;
};

#endif