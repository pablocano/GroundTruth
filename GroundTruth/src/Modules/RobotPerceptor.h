
#pragma once
#include "Representations/Regions.h"
#include "Representations/ColorModel/ColorModel.h"
#include "Representations/Image.h"
#include "Representations/CameraInfo.h"
#include "Representations/RobotPercept.h"
#include "Tools/ModuleManager/Module.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <iostream>

MODULE(RobotPerceptor,
{,
  REQUIRES(CameraInfo),
  REQUIRES(Image),
  REQUIRES(ColorModel),
  REQUIRES(Regions),
  PROVIDES(RobotPercept),
});


class RobotPerceptor : public RobotPerceptorBase
{
public:
    void update(RobotPercept& robotPercept);
};
