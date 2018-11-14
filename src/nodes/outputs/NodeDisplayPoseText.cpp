//
//
//
//
//

#include <nodes/outputs/NodeDisplayPoseText.hpp>

#include <data_types/DecimalData.hpp>

NodeDisplayPoseText::NodeDisplayPoseText() : _label(new QLabel()) {
  _label->setMargin(3);
}


unsigned int NodeDisplayPoseText::nPorts(PortType portType) const {
  unsigned int result;
  switch (portType) {
    case PortType::In:
      result = 1;
      break;

    case PortType::Out:
      result = 0;

    default:
      break;
  }

  return result;
}


NodeDataType NodeDisplayPoseText::dataType(PortType, PortIndex) const {
  return PoseData().type();
}


std::shared_ptr<NodeData> NodeDisplayPoseText:: outData(PortIndex) {
  std::shared_ptr<NodeData> ptr;
  return ptr;
}


void NodeDisplayPoseText::setInData(std::shared_ptr<NodeData> data, int) {
  auto poseData = std::dynamic_pointer_cast<PoseData>(data);
  if (poseData) {
    modelValidationState = NodeValidationState::Valid;
    modelValidationError = QString();
    std::stringstream ss;
    ss << poseData->pose();
    _label->setText(ss.str().c_str());
  }
  else {
    modelValidationState = NodeValidationState::Warning;
    modelValidationError = QStringLiteral("Missing or incorrect inputs");
    _label->clear();
  }

  _label->adjustSize();
}


NodeValidationState NodeDisplayPoseText::validationState() const {
  return modelValidationState;
}


QString NodeDisplayPoseText::validationMessage() const {
  return modelValidationError;
}
