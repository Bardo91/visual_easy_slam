#pragma once

#include <nodes/NodeDataModel>

using QtNodes::NodeDataType;
using QtNodes::NodeData;

/// The class can potentially incapsulate any user data which
/// need to be transferred within the Node Editor graph
class StringData : public NodeData
{
public:

  StringData(): mString(""){

  }

  StringData(std::string const _string): mString(mString){

  }

  NodeDataType type() const override {
    return NodeDataType {"string",
                         "string"};
  }

  QString string() const  { 
    return QString(mString.c_str()); 
  }

private:

  std::string mString;
};
