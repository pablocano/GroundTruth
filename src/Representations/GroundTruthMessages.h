#pragma once
#include "Tools/Math/Pose2D.h"

struct GroundTruthRobot
{
  int teamNumber;
  int robotNumber;
  Pose2D robotPose;
};

struct GroundTruthBall {
  Vector2<> ballPosition;
};

class GroundTruthMessageOutput {
};
