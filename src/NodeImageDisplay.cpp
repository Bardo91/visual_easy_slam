#include "NodeImageDisplay.hpp"

#include "ImageData.hpp"

NodeImageDisplay::NodeImageDisplay(): _label(new QLabel()) {
  _label->setMargin(3);
}


unsigned int NodeImageDisplay::nPorts(PortType portType) const {
  unsigned int result = 1;

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


NodeDataType NodeImageDisplay::dataType(PortType, PortIndex) const {
  return ImageData().type();
}


std::shared_ptr<NodeData> NodeImageDisplay::outData(PortIndex) {
  std::shared_ptr<NodeData> ptr;
  return ptr;
}


void NodeImageDisplay::setInData(std::shared_ptr<NodeData> data, int) {
  auto imageData = std::dynamic_pointer_cast<ImageData>(data);

  if (imageData) {
    modelValidationState = NodeValidationState::Valid;
    modelValidationError = QString();
    cv::Mat cvImage = imageData->image();
    QPixmap qtImage = QPixmap::fromImage(QImage((unsigned char*) cvImage.data, cvImage.cols, cvImage.rows, QImage::Format_RGB888));
    _label->setPixmap(qtImage);
  }
  else {
    modelValidationState = NodeValidationState::Warning;
    modelValidationError = QStringLiteral("Missing or incorrect inputs");
    _label->clear();
  }

  _label->adjustSize();
}


NodeValidationState NodeImageDisplay::validationState() const {
  return modelValidationState;
}


QString NodeImageDisplay::validationMessage() const {
  return modelValidationError;
}
