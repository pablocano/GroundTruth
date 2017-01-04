#pragma once
#include "Tools/Math/Pose2D.h"
#include "Tools/Streamable.h"
#include <opencv2/core/core.hpp>

class RobotsPoses : public Streamable {
public:
  struct RobotPose : public Pose2D
  {
    RobotPose(const Vector2<int>& pointDir, const Vector2<int>& positionInImage): centerInImage(positionInImage), point(pointDir) {}

    Vector2<int> centerInImage;
    Vector2<int> point;
    int team;
    int number;

  };
  
  void draw() const;
  
  std::vector<RobotPose> robotPoses;
};
