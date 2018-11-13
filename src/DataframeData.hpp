
#ifndef DataframeData_H_
#define DataframeData_H_

#include <nodes/NodeDataModel>
#include <opencv2/opencv.hpp>

#include <rgbd_tools/map3d/DataFrame.h>

using QtNodes::NodeDataType;
using QtNodes::NodeData;

/// The class can potentially incapsulate any user data which
/// need to be transferred within the Node Editor graph
class DataframeData : public NodeData {
  public:
  typedef std::shared_ptr<rgbd::DataFrame<pcl::PointXYZRGBNormal>> DataframePtr;
    DataframeData(){
      mDataframe = DataframePtr(new rgbd::DataFrame<pcl::PointXYZRGBNormal>);
    }

    NodeDataType type() const override {
      return NodeDataType {"dataframe", "Dataframe"};
    }

    DataframePtr image() const { 
      return mDataframe; 
    }

  public:

    DataframePtr mDataframe;
};

#endif