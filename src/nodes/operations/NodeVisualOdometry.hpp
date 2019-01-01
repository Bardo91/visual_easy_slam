//
//
//
//
//

#ifndef NODEVISUALODOMETRY_H_
#define NODEVISUALODOMETRY_H_

#include <rgbd_tools/map3d/Odometry.h>
#include <rgbd_tools/map3d/OdometryRgbd.h>

#include <data_types/DataframeData.hpp>
#include <data_types/PoseData.hpp>

#include <QtCore/QObject>
#include <QtCore/QJsonObject>
#include <QtWidgets/QLabel>

#include <nodes/NodeDataModel>

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
class NodeVisualOdometry : public NodeDataModel {
  Q_OBJECT

public:
  NodeVisualOdometry();
  ~NodeVisualOdometry() {}

  QString caption() const override { 
    return QStringLiteral("RGBDOdometry"); 
  }

  bool portCaptionVisible(PortType portType, PortIndex portIndex) const override {
    Q_UNUSED(portType); Q_UNUSED(portIndex);
    return true;
  }

  QString   portCaption(PortType portType, PortIndex portIndex) const override {
    switch (portType) {
      case PortType::In:
        return QStringLiteral("dataframe");
        break;

      case PortType::Out:
        return QStringLiteral("result_pose");
        break;
      default:
        break;
    }
    return QString();
  }

  QString  name() const override { 
    return QStringLiteral("RGBD Odometry"); 
  }

public:

  unsigned int nPorts(PortType portType) const override;

  NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

  std::shared_ptr<NodeData> outData(PortIndex port) override;

  void setInData(std::shared_ptr<NodeData> data, PortIndex portIndex) override;

  QWidget * embeddedWidget() override { return nullptr; }

  NodeValidationState validationState() const override;

  QString validationMessage() const override;

protected:
  void compute();

protected:
  std::weak_ptr<DataframeData> mDataframe;

  std::shared_ptr<PoseData> mResultPose; 

  std::shared_ptr<rgbd::ClusterFrames<pcl::PointXYZRGBNormal>> mLastCluster = nullptr;

  NodeValidationState modelValidationState = NodeValidationState::Warning;
  QString modelValidationError = QString("Missing or incorrect inputs");

  rgbd::Odometry<pcl::PointXYZRGBNormal, rgbd::DebugLevels::Debug> *mOdometry;
};

#endif