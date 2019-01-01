//
//
//
//
//

#include <nodes/outputs/NodeDisplay3D.hpp>

#include <data_types/PointCloudData.hpp>

NodeDisplay3D::NodeDisplay3D(): mViewerHolder(new QVTKWidget()) {
  // Set up the QVTK window
  mViewer3d.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  mViewerHolder->SetRenderWindow (mViewer3d->getRenderWindow ());
  mViewer3d->setupInteractor (mViewerHolder->GetInteractor (), mViewerHolder->GetRenderWindow ());
  mViewerHolder->update ();
}

QString NodeDisplay3D::caption() const {
  return QStringLiteral("Display 3D"); 
}

bool NodeDisplay3D::captionVisible() const { 
  return false; 
}

QString NodeDisplay3D::name() const  { 
  return QStringLiteral("Display 3D"); 
}

unsigned int NodeDisplay3D::nPorts(PortType portType) const {
  unsigned int result = 1;

  switch (portType) {
    case PortType::In:
      result = 1;
      break;
    case PortType::Out:
      result = 0;
      break;
    default:
      break;
  }
  return result;
}


NodeDataType NodeDisplay3D::dataType(PortType, PortIndex) const {
  return PointCloudData().type();
}


std::shared_ptr<NodeData> NodeDisplay3D::outData(PortIndex) {
  std::shared_ptr<NodeData> ptr;
  return ptr;
}


void NodeDisplay3D::setInData(std::shared_ptr<NodeData> data, int) {
  auto pcData = std::dynamic_pointer_cast<PointCloudData>(data);

  if (pcData) {
    modelValidationState = NodeValidationState::Valid;
    modelValidationError = QString();
    
  }
  else {
    modelValidationState = NodeValidationState::Warning;
    modelValidationError = QStringLiteral("Missing or incorrect inputs");
  }

  mViewerHolder->adjustSize();
}


NodeValidationState NodeDisplay3D::validationState() const {
  return modelValidationState;
}


QString NodeDisplay3D::validationMessage() const {
  return modelValidationError;
}
