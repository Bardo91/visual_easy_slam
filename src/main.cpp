#include <nodes/NodeData>
#include <nodes/FlowScene>
#include <nodes/FlowView>
#include <nodes/ConnectionStyle>
#include <nodes/TypeConverter>

#include <QtWidgets/QApplication>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QMenuBar>

#include <nodes/DataModelRegistry>

#include "Converters.hpp"

#include <nodes/sources/NodeImageStream.hpp>
#include <nodes/sources/NodeStereoCamera.hpp>
#include <nodes/sources/StringSourceData.hpp>

#include <nodes/operations/NodeVisualOdometry.hpp>
#include <nodes/operations/NodeDataframeCreator.hpp>

#include <nodes/outputs/NodeImageDisplay.hpp>
#include <nodes/outputs/NodeDisplayPoseText.hpp>
#include <nodes/outputs/NodeDisplay3D.hpp>


using QtNodes::DataModelRegistry;
using QtNodes::FlowScene;
using QtNodes::FlowView;
using QtNodes::ConnectionStyle;
using QtNodes::TypeConverter;
using QtNodes::TypeConverterId;

static std::shared_ptr<DataModelRegistry>
registerDataModels()
{
  auto ret = std::make_shared<DataModelRegistry>();

  ret->registerModel<NodeImageStream>("Sources");
  ret->registerModel<NodeStereoCamera>("Sources");
  ret->registerModel<StringSourceData>("Sources");
  ret->registerModel<NodeDataframeCreator>("Sources");

  ret->registerModel<NodeDisplayPoseText>("Displays");
  ret->registerModel<NodeImageDisplay>("Displays");
  ret->registerModel<NodeDisplay3D>("Displays");

  ret->registerModel<NodeVisualOdometry>("Odometry");

  ret->registerTypeConverter(std::make_pair(DecimalData().type(),
                                            IntegerData().type()),
                             TypeConverter{DecimalToIntegerConverter()});



  ret->registerTypeConverter(std::make_pair(IntegerData().type(),
                                            DecimalData().type()),
                             TypeConverter{IntegerToDecimalConverter()});

  return ret;
}


static
void
setStyle()
{
  ConnectionStyle::setConnectionStyle(
  R"(
  {
    "ConnectionStyle": {
      "ConstructionColor": "gray",
      "NormalColor": "black",
      "SelectedColor": "gray",
      "SelectedHaloColor": "deepskyblue",
      "HoveredColor": "deepskyblue",

      "LineWidth": 3.0,
      "ConstructionLineWidth": 2.0,
      "PointDiameter": 10.0,

      "UseDataDefinedColors": true
    }
  }
  )");
}


int
main(int argc, char *argv[])
{
  QApplication app(argc, argv);

  setStyle();

  QWidget mainWidget;

  auto menuBar    = new QMenuBar();
  auto saveAction = menuBar->addAction("Save..");
  auto loadAction = menuBar->addAction("Load..");

  QVBoxLayout *l = new QVBoxLayout(&mainWidget);

  l->addWidget(menuBar);
  auto scene = new FlowScene(registerDataModels(), &mainWidget);
  l->addWidget(new FlowView(scene));
  l->setContentsMargins(0, 0, 0, 0);
  l->setSpacing(0);

  QObject::connect(saveAction, &QAction::triggered,
                   scene, &FlowScene::save);

  QObject::connect(loadAction, &QAction::triggered,
                   scene, &FlowScene::load);

  mainWidget.setWindowTitle("Dataflow tools: simplest calculator");
  mainWidget.resize(800, 600);
  mainWidget.showNormal();

  return app.exec();
}
