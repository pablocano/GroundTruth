#pragma once
#include "Tools/Line.h"

class RobotPose
{
public:
    void draw(cv::Mat& image) const;

    std::vector<Line> ls;
};