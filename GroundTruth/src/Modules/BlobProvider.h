#pragma once

#include "Representations/CameraInfo.h"
#include "Representations/Regions.h"
#include "Representations/Blobs.h"
#include "Tools/ModuleManager/Module.h"

MODULE(BlobProvider,
{,
  REQUIRES(CameraInfo),
  //REQUIRES(Regions),
  //PROVIDES(Blobs),
});

class BlobProvider : public BlobProviderBase {
  
public:
  void update(Blobs &blobs);

  
private:
  struct Segment : public Regions::Rect
  {
    Segment(const Regions::Rect& rect) : Regions::Rect(rect.center,rect.leftUpper,rect.rightBottom,rect.color), label(0) {}
    int label;
  };
  
  struct Group
  {
    Group() = default;
    std::vector<Segment> segments;
    bool itBelongs(const Segment& line, int segment);
    Vector2<int> getCenter();
    Vector2<int> getLeftUpper();
    Vector2<int> getRightBottom();
    ColorModel::Colors color;
  };
  
  void createBlobs();
  
  std::vector<Segment> segments;
  std::vector<Group> groups;
};
