#include "RobotPerceptor.h"
#include "Tools/Math/Transformation.h"

MAKE_MODULE(RobotPerceptor, GroundTruth)

void RobotPerceptor::update(RobotPercept& robotPercept)
{
    robotPercept.robots.clear();
  
    for(auto& blob : theBlobs.blobs)
    {
        if(blob.color.is(white))
        {
            Vector2<int> pos = Transformation::imageToImageCorrected(blob.center,theCameraInfo);
            robotPercept.robots.push_back(RobotPercept::Robot(blob.center,blob.leftUpper,blob.rightBottom,pos));
        }
    }

}
