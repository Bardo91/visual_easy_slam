//
//
//
//
//

#ifndef NodeDisplay3D_H_
#define NodeDisplay3D_H_

#include <QtCore/QObject>
#include <QtWidgets/QLabel>

#include <nodes/NodeDataModel>
#include <iostream>

#include <pcl/visualization/pcl_visualizer.h>
#include <QVTKWidget.h>
#include <vtkRenderWindow.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using QtNodes::PortType;
using QtNodes::PortIndex;
using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDataModel;
using QtNodes::NodeValidationState;

/// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class NodeDisplay3D : public NodeDataModel {
  Q_OBJECT

public:
  //-------------- CONSTRUCTORS --------------
  NodeDisplay3D();
  virtual ~NodeDisplay3D() {}

  //-------------- DISPLAY INFO --------------
  QString caption() const override;
  bool captionVisible() const override;
  QString name() const override;

  //-------------- DATA FLOW --------------
  unsigned int nPorts(PortType portType) const override;
  NodeDataType dataType(PortType portType, PortIndex portIndex) const override;
  std::shared_ptr<NodeData> outData(PortIndex port) override;
  void setInData(std::shared_ptr<NodeData> data, int) override;
  QWidget *embeddedWidget() override { return mViewerHolder; }
  NodeValidationState validationState() const override;
  QString validationMessage() const override;

private:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> mViewer3d;

  NodeValidationState modelValidationState = NodeValidationState::Warning;
  QString modelValidationError = QStringLiteral("Missing or incorrect inputs");

  QVTKWidget *mViewerHolder;
};


#endif