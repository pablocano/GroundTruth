#include "Regions.h"
#include "Tools/Debugging/DebugDrawings.h"

void Regions::draw(/*cv::Mat &image*/) const
{
  DECLARE_DEBUG_DRAWING("representation:Region", "drawingOnImage");
  for(auto& region : regions)
  {
    cv::Scalar color = DrawingColors::getDrawingColor(region.color.colors);
    RECTANGLE("representation:Region", region.leftUpper.x, region.leftUpper.y, region.rightBottom.x, region.rightBottom.y, 5, Drawings::ps_solid, ColorRGBA(color[0],color[3],color[0]));
  }
}
