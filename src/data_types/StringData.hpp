#ifndef VISUALEASYSLAM_DATATYPES_STRINGDATA_H_
#define VISUALEASYSLAM_DATATYPES_STRINGDATA_H_

#include <nodes/NodeDataModel>

using QtNodes::NodeDataType;
using QtNodes::NodeData;

/// The class can potentially incapsulate any user data which
/// need to be transferred within the Node Editor graph
class StringData : public NodeData {
public:
  StringData(){

  }

  StringData(const std::string _string): mString(_string){

  }

  StringData(const QString _string): mString(_string.toStdString()){

  }

  NodeDataType type() const override {
    return NodeDataType {"string",
                         "string"};
  }

  QString string() const  { 
    return QString(mString.c_str()); 
  }

private:
  std::string mString = "";
};

#endif