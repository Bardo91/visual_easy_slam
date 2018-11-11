
#ifndef IMAGEDATA_H_
#define IMAGEDATA_H_

#include <nodes/NodeDataModel>
#include <opencv2/opencv.hpp>

using QtNodes::NodeDataType;
using QtNodes::NodeData;

/// The class can potentially incapsulate any user data which
/// need to be transferred within the Node Editor graph
class ImageData : public NodeData {
  public:
    ImageData() {
      mImage = cv::Mat(480,640, CV_8UC3, cv::Scalar(0,0,0));
    }

    ImageData(const std::string &_path) {
      mImage = cv::imread(_path);
    }

    ImageData(const cv::Mat &_image) {
      mImage = _image;
    }

    NodeDataType type() const override {
      return NodeDataType {"image", "Image"};
    }

    cv::Mat image() const { 
      return mImage; 
    }

  private:
    cv::Mat mImage;

};

#endif