#pragma once
#include "Camera.h"
#include "Tools/ModuleManager/Module.h"
#include "Tools/Debugging/Debugging.h"
#include "Representations/CameraInfo.h"
#include <sstream>
#include <iostream>

MODULE(Descriptor,
{,
  REQUIRES(CameraInfo),
  /*PROVIDES(CameraInfo),
  PROVIDES(ImageBGR),
  REQUIRES(ImageBGR),
  PROVIDES(Image),*/
});

class Descriptor : public DescriptorBase
{
public:
  Descriptor();
  //void update(CameraInfo& cameraInfo);
  void static update();

private:

};
