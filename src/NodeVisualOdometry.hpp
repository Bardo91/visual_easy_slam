#pragma once

#include <QtCore/QObject>
#include <QtCore/QJsonObject>
#include <QtWidgets/QLabel>

#include <nodes/NodeDataModel>

#include "ImageData.hpp"
#include "PoseData.hpp"

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
  ~NodeVisualOdometry() {}

  QString
  caption() const override
  { return QStringLiteral("VisualOdometry"); }

  bool
  portCaptionVisible(PortType portType, PortIndex portIndex) const override
  {
    Q_UNUSED(portType); Q_UNUSED(portIndex);
    return true;
  }

  QString
  portCaption(PortType portType, PortIndex portIndex) const override
  {
    switch (portType)
    {
      case PortType::In:
        return QStringLiteral("Image");

        break;

      case PortType::Out:
        if (portIndex == 0)
          return QStringLiteral("debug_image");
        else if (portIndex == 1)
          return QStringLiteral("result_pose");

      default:
        break;
    }
    return QString();
  }

  QString
  name() const override
  { return QStringLiteral("Division"); }

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
  std::weak_ptr<ImageData> mInputImage;

  std::shared_ptr<ImageData> mDebugImage;
  std::shared_ptr<PoseData> mResultPose; 

  NodeValidationState modelValidationState = NodeValidationState::Warning;
  QString modelValidationError = QString("Missing or incorrect inputs");
};
