#include "NodeImageStream.hpp"

#include <QtCore/QJsonValue>
#include <QtGui/QDoubleValidator>


NodeImageStream::NodeImageStream() : _lineEdit(new QLineEdit()) {
  mImageData = std::shared_ptr<ImageData>(new ImageData);

  _lineEdit->setValidator(new QDoubleValidator());

  _lineEdit->setMaximumSize(_lineEdit->sizeHint());

  //connect(_lineEdit, &QLineEdit::textChanged, this, &NodeImageStream::onTextEdited);

  _lineEdit->setText("0.0");
  // }
  // else {
  //   emit dataInvalidated(0);
  // }
  connect(this, &NodeImageStream::updatedImage, this, &NodeImageStream::onTextEdited);

  mImageAcquisitionThread = std::thread([&](){
    cv::VideoCapture camera(0);
    while(mRunning){
      cv::Mat image;
      camera >> image;

      mImageMutex.lock();
      mImageData = std::shared_ptr<ImageData>(new ImageData(image, ImageData::eImageType::RGB));
      mImageMutex.unlock();

      emit updatedImage();
    }
  });
}


QJsonObject NodeImageStream::save() const {
  QJsonObject modelJson = NodeDataModel::save();

  return modelJson;
}


void NodeImageStream::restore(QJsonObject const &p) {
  
}


unsigned int NodeImageStream::nPorts(PortType portType) const {
  unsigned int result = 1;

  switch (portType) {
    case PortType::In:
      result = 0;
      break;

    case PortType::Out:
      result = 1;

    default:
      break;
  }

  return result;
}


void NodeImageStream::onTextEdited() {
  // Q_UNUSED(string);

    emit dataUpdated(0);
  // }
  // else {
  //   emit dataInvalidated(0);
  // }
}


NodeDataType NodeImageStream::dataType(PortType, PortIndex) const {
  return ImageData().type();
}


std::shared_ptr<NodeData> NodeImageStream::outData(PortIndex) {
  std::lock_guard<std::mutex> locker(mImageMutex);
  return mImageData;
}
