#pragma once

#include <data_types/DecimalData.hpp>
#include <data_types/IntegerData.hpp>

using QtNodes::PortType;
using QtNodes::PortIndex;
using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDataModel;

class DecimalData;
class IntegerData;


class DecimalToIntegerConverter
{

public:

  std::shared_ptr<NodeData>
  operator()(std::shared_ptr<NodeData> data);

private:

  std::shared_ptr<NodeData> _integer;
};


class IntegerToDecimalConverter
{

public:

  std::shared_ptr<NodeData>
  operator()(std::shared_ptr<NodeData> data);

private:

  std::shared_ptr<NodeData> _decimal;
};
