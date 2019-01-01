//
//
//
//
//

#include <nodes/sources/StringSourceData.hpp>

#include <QtCore/QJsonValue>
#include <QtGui/QDoubleValidator>

#include <data_types/StringData.hpp>

StringSourceData::StringSourceData(): _lineEdit(new QLineEdit()) {
  _lineEdit->setMaximumSize(_lineEdit->sizeHint());
  connect(_lineEdit, &QLineEdit::textChanged, this, &StringSourceData::onTextEdited);
  _lineEdit->setText("");
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
    _lineEdit->setText(str);
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


void StringSourceData::onTextEdited(QString const &string){
  Q_UNUSED(string);

  mStringData = std::make_shared<StringData>(string.toStdString());
  emit dataUpdated(0);
}


NodeDataType StringSourceData::dataType(PortType, PortIndex) const {
  return StringData().type();
}


std::shared_ptr<NodeData> StringSourceData::outData(PortIndex) {
  return mStringData;
}
