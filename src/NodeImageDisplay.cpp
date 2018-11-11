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
    if(imageData->image().rows != 0){
      if(imageData->imageType() == ImageData::eImageType::RGB){
        cv::Mat cvImage;
        cv::cvtColor(imageData->image(), cvImage, CV_RGB2BGR);
        QPixmap qtImage = QPixmap::fromImage(QImage((unsigned char*) cvImage.data, cvImage.cols, cvImage.rows, QImage::Format_RGB888));
        _label->setPixmap(qtImage);  
      }else if(imageData->imageType() == ImageData::eImageType::DEPTH16 || imageData->imageType() == ImageData::eImageType::DEPTH32){ 
        cv::Mat cvImage;
        cv::normalize(imageData->image(), cvImage, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        QPixmap qtImage = QPixmap::fromImage(QImage((unsigned char*) cvImage.data, cvImage.cols, cvImage.rows, QImage::Format_Grayscale8));
        _label->setPixmap(qtImage);
      }
    }
    
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
