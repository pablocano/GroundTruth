

#pragma once
#include "Representations/Blobs.h"
#include "Representations/CameraInfo.h"
#include "Representations/RobotIdentifier.h"
#include "Representations/RobotPercept.h"
#include "Representations/RobotPose.h"
#include "Tools/ModuleManager/Module.h"
#include "Representations/ColorModel/ColorModel.h"
#include "Representations/Image.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <iostream>
#define PI 3.14159265


MODULE(RobotPoseProvider,
{,
  REQUIRES(CameraInfo),
  REQUIRES(Blobs),
  REQUIRES(Image),
  REQUIRES(RobotsIdentifiers),
  REQUIRES(ColorModel),
  REQUIRES(RobotPercept),
  PROVIDES(RobotsPoses),
});

class RobotPoseProvider : public RobotPoseProviderBase {
  
public:
  
  RobotPoseProvider() : minDistance(10) {}
  
  void update(RobotsPoses& robotPose);
  
  struct PosibleRobot{
    std::vector<Blobs::Blob> shoulders;
    Vector2<int> leftShoulder;
    Vector2<int> rightShoulder;
    bool valid;
    int team;
    int number;
    void clear()
    {
        shoulders.clear();
        valid = false;
    }
  };
  
private:
  
  void analizeBlob(const RobotPercept::Robot& robot);

  void getPicDir(cv::Mat &image, int &centerX, int &centerY, float &anguloOrient);

  void analizePosibleRobot();
  
  void calculatePose(RobotsPoses& robotPose);
  
  int minDistance;
  
  PosibleRobot posibleRobot;
};
