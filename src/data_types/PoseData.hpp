
#ifndef POSEDATADATA_H_
#define POSEDATADATA_H_

#include <nodes/NodeDataModel>
#include <Eigen/Eigen>

using QtNodes::NodeDataType;
using QtNodes::NodeData;

/// The class can potentially incapsulate any user data which
/// need to be transferred within the Node Editor graph
class PoseData : public NodeData {
  public:
    PoseData() {
      mPose = Eigen::Matrix4f::Identity();
    }

    PoseData(const Eigen::Matrix4f &_pose) {
      mPose = _pose;
    }

    NodeDataType type() const override {
      return NodeDataType {"pose", "Pose"};
    }

    Eigen::Matrix4f pose() const { 
      return mPose; 
    }

  private:
    Eigen::Matrix4f mPose;

};

#endif