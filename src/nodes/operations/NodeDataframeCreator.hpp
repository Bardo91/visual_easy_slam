//
//
//
//
// 

#ifndef NODEDATAFRAMECREATOR_H_
#define NODEDATAFRAMECREATOR_H_

#include <QtCore/QObject>
#include <QtCore/QJsonObject>
#include <QtWidgets/QLabel>

#include <nodes/NodeDataModel>

#include <rgbd_tools/map3d/Odometry.h>
#include <rgbd_tools/map3d/OdometryRgbd.h>

#include <data_types/ImageData.hpp>
#include <data_types/DataframeData.hpp>
#include <data_types/CalibrationData.hpp>
#include <mutex>
#include <iostream>

class DecimalData;

using QtNodes::PortType;
using QtNodes::PortIndex;
using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDataModel;
using QtNodes::NodeValidationState;

/// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class NodeDataframeCreator : public NodeDataModel {
  Q_OBJECT

public:
  //-------------- CONSTRUCTORS --------------
  NodeDataframeCreator();
  ~NodeDataframeCreator() {}

  //-------------- DISPLAY INFO --------------
  QString caption() const override;
  bool portCaptionVisible(PortType portType, PortIndex portIndex) const override;
  QString  portCaption(PortType portType, PortIndex portIndex) const override;
  QString  name() const override;

  //-------------- DATA FLOW --------------
  unsigned int nPorts(PortType portType) const override;
  NodeDataType dataType(PortType portType, PortIndex portIndex) const override;
  std::shared_ptr<NodeData> outData(PortIndex port) override;
  void setInData(std::shared_ptr<NodeData> data, PortIndex portIndex) override;
  QWidget * embeddedWidget() override { return nullptr; }
  NodeValidationState validationState() const override;
  QString validationMessage() const override;

protected:
  void buildDataframe();

protected:
  std::weak_ptr<ImageData> mInputImageRgb;
  std::weak_ptr<ImageData> mInputImageDepth;
  std::weak_ptr<CalibrationData> mCalibration; 
  cv::Ptr<cv::ORB> mFeatureDetector ;

  std::shared_ptr<ImageData> mDebugImage;
  std::shared_ptr<DataframeData> mDataframe; 

  NodeValidationState modelValidationState = NodeValidationState::Warning;
  QString modelValidationError = QString("Missing or incorrect inputs");

  std::mutex mCreationLocker;
  bool mWorking = false;

  int mDfCounter = 0;
};

#endif