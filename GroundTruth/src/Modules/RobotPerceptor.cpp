#include "RobotPerceptor.h"
#include "Tools/Math/Transformation.h"

MAKE_MODULE(RobotPerceptor, GroundTruth)

void RobotPerceptor::update(RobotPercept& robotPercept)
{
    robotPercept.robots.clear();
  

    for(auto& region : theRegions.regions)
    {
        int blueCount = 0;
        int orangeCount = 0;
        for(int i = region.leftUpper.y; i < region.rightBottom.y; i++)
        {
            for (int j = region.leftUpper.x; j < region.rightBottom.x; j++)
            {
                if (theColorModel.getColor(theImage.at<cv::Vec3b>(i,j)).is(blue))
                {
                    //std::cout << blueCount << std::endl;
                    blueCount++;
                }
                if (theColorModel.getColor(theImage.at<cv::Vec3b>(i,j)).is(orange))
                {
                    //std::cout << blueC << std::endl;
                    orangeCount++;
                }
            }
        }
        if (orangeCount > 30 && blueCount > 30)
        {
            Vector2<int> pos = Transformation::imageToImageCorrected(region.center,theCameraInfo);
            robotPercept.robots.push_back(RobotPercept::Robot(region.center,region.leftUpper,region.rightBottom,pos));

        }
        //std::cout << region.leftUpper.x << std::endl;

        //bool hola = region.color.is(orange);
        //std::cout << theSegmentedImage << std::endl;
        //cv::imshow("Imagen", theSegmentedImage);
        /*if(blob.color.is(white))
        {
            Vector2<int> pos = Transformation::imageToImageCorrected(blob.center,theCameraInfo);
            robotPercept.robots.push_back(RobotPercept::Robot(blob.center,blob.leftUpper,blob.rightBottom,pos));
        }*/
    }

}
