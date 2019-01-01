//
//
//
//
//
//

#ifndef VISUALSLAMEASY_StringSourceData_H_
#define VISUALSLAMEASY_StringSourceData_H_

#include <QtCore/QObject>
#include <QtWidgets/QLineEdit>

#include <nodes/NodeDataModel>

#include <iostream>

class StringData;

using QtNodes::PortType;
using QtNodes::PortIndex;
using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDataModel;
using QtNodes::NodeValidationState;

/// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class StringSourceData : public NodeDataModel {
  Q_OBJECT

public:
  //-------------- CONSTRUCTORS --------------
  StringSourceData();
  virtual ~StringSourceData() { }


  //-------------- DISPLAY INFO --------------
  QString caption() const override{ 
    return QStringLiteral("String Source"); 
  }

  bool captionVisible() const override { 
    return false; 
  }

  QString name() const override { 
    return QStringLiteral("String Source"); 
  }

  //-------------- PERSISTENCE --------------
    QJsonObject save() const override;
  void restore(QJsonObject const &p) override;

  //-------------- DATAFLOW --------------
  unsigned int nPorts(PortType portType) const override;

  NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

  std::shared_ptr<NodeData> outData(PortIndex port) override;

  void setInData(std::shared_ptr<NodeData>, int) override { }

  QWidget *embeddedWidget() override { 
    return mTextHolder; 
  }

private slots:
  void onTextEdited(QString const &string);

private:
  std::shared_ptr<StringData> mStringData;

  QLineEdit * mTextHolder;
};

#endif