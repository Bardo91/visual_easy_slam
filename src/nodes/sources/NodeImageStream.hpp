//
//
//
//
//

#ifndef NODEIMAGESTREAM_H_
#define NODEIMAGESTREAM_H_

#include <QtCore/QObject>
#include <QtWidgets/QLineEdit>

#include <nodes/NodeDataModel>

#include <data_types/ImageData.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <mutex>
#include <thread>

class DecimalData;

using QtNodes::PortType;
using QtNodes::PortIndex;
using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDataModel;
using QtNodes::NodeValidationState;

/// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class NodeImageStream : public NodeDataModel
{
  Q_OBJECT

public:
  NodeImageStream();

  virtual
  ~NodeImageStream() { 
    mRunning = false;
    if(mImageAcquisitionThread.joinable())
      mImageAcquisitionThread.join();
    }

public:

  QString  caption() const override { return QStringLiteral("Image Stream"); }

  bool captionVisible() const override { return false; }

  QString name() const override { return QStringLiteral("Image Stream"); }

public:

  QJsonObject save() const override;

  void restore(QJsonObject const &p) override;

public:

  unsigned int nPorts(PortType portType) const override;

  NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

  std::shared_ptr<NodeData>  outData(PortIndex port) override;

  void setInData(std::shared_ptr<NodeData>, int) override { }

  QWidget * embeddedWidget() override { return _lineEdit; }

private slots:
  void onTextEdited();

signals:
  void updatedImage();

private:
  std::shared_ptr<ImageData> mImageData;
  std::mutex mImageMutex;

  std::thread mImageAcquisitionThread;
  bool mRunning = true;

  QLineEdit * _lineEdit;
};

#endif