
#pragma once

#include "Representations/Image.h"
#include "Representations/ColorModel/ColorModel.h"
#include "Representations/Regions.h"
#include "Tools/ModuleManager/Module.h"
#include <sstream>
#include <iostream>

MODULE(Regionizer,
{,
  REQUIRES(ColorModel),
  REQUIRES(Image),
  REQUIRES(MovementImage),
  PROVIDES(Regions),
});


class Regionizer : public RegionizerBase
{
public:
  
  Regionizer() :
      step(2),
      mask((cv::Mat_<float>(3,3) << 1/9.0, 1/9.0, 1/9.0, 1/9.0, 1/9.0, 1/9.0, 1/9.0, 1/9.0, 1/9.0))
  {}
  
  void update(Regions& regions);
  
  void dirProvider(int &iLast, int &jLast, int &i, int &j, cv::Mat theMovementImage, bool &breakLoop, int &iMin, int &jMin, int &iMax, int &jMax);
  
  void regionSearcher(cv::Mat theMovementImage, cv::Mat theImage, Regions &regions);

  void convolucionBinaria(cv::Mat imageGrey, cv::Mat mask, cv::Mat &outputImage, float threshold);

  int step;

private:

  cv::Mat mask;

  
};
