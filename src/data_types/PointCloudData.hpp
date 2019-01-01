
#ifndef DataframeData_H_
#define DataframeData_H_

#include <nodes/NodeDataModel>
#include <opencv2/opencv.hpp>

#include <rgbd_tools/map3d/DataFrame.h>

using QtNodes::NodeDataType;
using QtNodes::NodeData;

/// The class can potentially incapsulate any user data which
/// need to be transferred within the Node Editor graph
class PointCloudData : public NodeData {
  public:
  typedef std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBNormal>> PointcloudPtr;
    PointCloudData(){
      mPointCloud = PointcloudPtr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    }

    NodeDataType type() const override {
      return NodeDataType {"pointcloud", "pointcloud"};
    }

    PointcloudPtr image() const { 
      return mPointCloud; 
    }

    PointcloudPtr mPointCloud;
};

#endif