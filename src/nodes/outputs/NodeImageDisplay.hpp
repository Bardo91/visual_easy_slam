//
//
//
//
//

#ifndef NODEIMAGEDISPLAY_H_
#define NODEIMAGEDISPLAY_H_

#include <QtCore/QObject>
#include <QtWidgets/QLabel>

#include <nodes/NodeDataModel>

#include <iostream>

using QtNodes::PortType;
using QtNodes::PortIndex;
using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDataModel;
using QtNodes::NodeValidationState;

/// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class NodeImageDisplay : public NodeDataModel
{
  Q_OBJECT

public:
  NodeImageDisplay();

  virtual
  ~NodeImageDisplay() {}

public:
  QString caption() const override {
    return QStringLiteral("Image Display"); 
  }

  bool captionVisible() const override { 
    return false; 
  }

  QString name() const override  { 
    return QStringLiteral("Image Display"); 
  }

public:
  unsigned int nPorts(PortType portType) const override;

  NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

  std::shared_ptr<NodeData> outData(PortIndex port) override;

  void setInData(std::shared_ptr<NodeData> data, int) override;

  QWidget *embeddedWidget() override { return _label; }

  NodeValidationState validationState() const override;

  QString validationMessage() const override;

private:

  NodeValidationState modelValidationState = NodeValidationState::Warning;
  QString modelValidationError = QStringLiteral("Missing or incorrect inputs");

  QLabel * _label;
};


#endif