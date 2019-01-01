//
//
//
//
//

#include <nodes/sources/StringSourceData.hpp>

#include <QtCore/QJsonValue>
#include <QtGui/QDoubleValidator>

#include <data_types/StringData.hpp>

StringSourceData::StringSourceData(): mTextHolder(new QLineEdit()) {
  mTextHolder->setMaximumSize(mTextHolder->sizeHint());
  connect(mTextHolder, &QLineEdit::textChanged, this, &StringSourceData::onTextEdited);
  mTextHolder->setText("string");
}


QJsonObject StringSourceData::save() const {
  QJsonObject modelJson = NodeDataModel::save();

  if (mStringData)
    modelJson["string"] = mStringData->string();

  return modelJson;
}


void StringSourceData::restore(QJsonObject const &p) {
  QJsonValue v = p["string"];

  if (!v.isUndefined()) {
    QString str = v.toString();
    mTextHolder->setText(str);
  }
}


unsigned int StringSourceData::nPorts(PortType portType) const {
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


void StringSourceData::onTextEdited(QString const &_string){
  mStringData = std::make_shared<StringData>(_string);
  emit dataUpdated(0);
}


NodeDataType StringSourceData::dataType(PortType, PortIndex) const {
  return StringData().type();
}


std::shared_ptr<NodeData> StringSourceData::outData(PortIndex) {
  return mStringData;
}
