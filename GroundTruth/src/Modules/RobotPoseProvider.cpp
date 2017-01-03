//
//  RobotPoseProvider.cpp
//  GroundTruth
//
//  Created by Pablo Cano Montecinos on 02-08-16.
//
//

#include "RobotPoseProvider.h"
#include "Tools/Math/Transformation.h"

MAKE_MODULE(RobotPoseProvider, GroundTruth)

void RobotPoseProvider::update(RobotsPoses &robotsPoses)
{
  robotsPoses.robotPoses.clear();
  // Generar imágenes binarias de cada color
  for(auto& robot : theRobotPercept.robots)
  {
      //std::cout << "width: " << robot.rightBottom.x-robot.leftUpper.x << std::endl;
      //std::cout << "height: " << (robot.leftUpper.y-robot.rightBottom.y) << std::endl;
      //std::cout << "1" << std::endl;
      int width =  std::abs(robot.rightBottom.x-robot.leftUpper.x);
      int height = std::abs(robot.leftUpper.y-robot.rightBottom.y);
      //cv::Rect rect(robot.leftUpper.x, robot.leftUpper.y, std::abs(robot.rightBottom.x-robot.leftUpper.x), std::abs(robot.leftUpper.y-robot.rightBottom.y));
      //cv::Mat croppedImage = theImage(rect);
      //cv::imshow("Cropped", croppedImage);
      //std::cout << "2" << std::endl;
      cv::Mat blueSegmented = cv::Mat::zeros(theImage.rows, theImage.cols, CV_32FC1);
      cv::Mat orangeSegmented = cv::Mat::zeros(theImage.rows, theImage.cols, CV_32FC1);
      //std::cout << "3" << std::endl;
      for(int i = robot.leftUpper.y; i < robot.rightBottom.y; i++)
      {
          for (int j = robot.leftUpper.x; j < robot.rightBottom.x; j++)
          {
              if (theColorModel.getColor(theImage.at<cv::Vec3b>(i,j)).is(blue))
              {
                  //std::cout << "blue" << std::endl;
                  blueSegmented.at<float>(i,j) = 255;
                  //cv::imshow("blueSegmented", blueSegmented);
                  //std::cout << blueCount << std::endl;
                  //blueCount++;
              }
              //std::cout << blueSegmented.at<float>(i,j) << std::endl;
              if (theColorModel.getColor(theImage.at<cv::Vec3b>(i,j)).is(orange))
              {
                  orangeSegmented.at<float>(i,j) = 255;
                  //std::cout << blueC << std::endl;
                  //orangeCount++;
              }
          }
      }
      //std::cout << blueSegmented << std::endl;
      int centerBlueX, centerBlueY;
      int centerOrangeX, centerOrangeY;
      float anguloOrientBlue, anguloOrientOrange;
      getPicDir(blueSegmented, centerBlueX, centerBlueY, anguloOrientBlue);
      getPicDir(orangeSegmented, centerOrangeX, centerOrangeY, anguloOrientOrange);
      int naoCenterX = (centerBlueX + centerOrangeX)/2;
      int naoCenterY = (centerBlueY + centerOrangeY)/2;
      // Rotación en 90 grados
      //int naoDirX = -(centerBlueY-naoCenterY);
      //int naoDirY = centerBlueX-naoCenterX;
      //int naoDirY = 20*(centerBlueX-naoCenterX);
      //int naoDirX = 20*(centerBlueY-naoCenterY);
      //float anguloOrient = (std::abs(anguloOrientBlue) + std::abs(anguloOrientOrange))/2;
      float anguloOrient = std::abs(anguloOrientOrange);
      cv::Point pt1;
      cv::Point pt2;
      pt1.x = naoCenterX;
      pt1.y = naoCenterY;
      if (std::abs(centerBlueX - centerOrangeX) > std::abs(centerBlueY - centerOrangeY))
      {
          std::cout << "ganó x" << std::endl;
          if(centerBlueX < naoCenterX)
          {
              pt2.x = naoCenterX+50*cos(anguloOrient);
              pt2.y = naoCenterY+50*sin(anguloOrient);
          }
          else
          {
              pt2.x = naoCenterX-50*cos(anguloOrient);
              pt2.y = naoCenterY-50*sin(anguloOrient);
          }
      }
      else
      {
          std::cout << "ganó y" << std::endl;
          if(centerBlueY > naoCenterY)
          {
              pt2.x = naoCenterX+50*cos(anguloOrient);
              pt2.y = naoCenterY+50*sin(anguloOrient);
          }
          else
          {
              pt2.x = naoCenterX-50*cos(anguloOrient);
              pt2.y = naoCenterY-50*sin(anguloOrient);
          }
      }
      cv::Mat dirNao = theImage.clone();
      cv::line(dirNao, pt1, pt2, cv::Scalar( 110, 220, 0 ));
      cv::imshow("DirNao", dirNao);

      //cv::imshow("blueSegmented", blueSegmented);
      //cv::imshow("orangeSegmented", orangeSegmented);

      //std::cout << "5" << std::endl;
  }

  /*for (const RobotPercept::Robot& robot : theRobotPercept.robots) {
    analizeBlob(robot);
    analizePosibleRobot();
    if (posibleRobot.valid) {
      calculatePose(robotsPoses);
    }
  }*/

}

void RobotPoseProvider::analizeBlob(const RobotPercept::Robot &robot)
{
  posibleRobot.clear();
  for (const Blobs::Blob &blob : theBlobs.blobs) {
    if(blob.center.x < robot.leftUpper.x - minDistance || blob.center.x > robot.rightBottom.x + minDistance || blob.center.y < robot.leftUpper.y - minDistance || blob.center.y > robot.rightBottom.y + minDistance)
      continue;
    if (!blob.color.is(none) && !blob.color.is(green) && !blob.color.is(white))
    {
      posibleRobot.shoulders.push_back(blob);
    }
  }
}

void RobotPoseProvider::getPicDir(cv::Mat &image, int &centerX, int &centerY, float &anguloOrient)
{
    cv::Moments Momentos = cv::moments(image, 1);
    //std::cout << Momentos.mu20 << std::endl;

    // Calcular orientación
    float mu20 = Momentos.mu20;
    float mu00 = Momentos.m00;
    float mu02 = Momentos.mu02;
    float mu11 = Momentos.mu11;
    float muP20 = mu20/mu00;
    float muP02 = mu02/mu00;
    float muP11 = mu11/mu00;

    //std::cout << muP20 << std::endl;

    anguloOrient = 0.5*atan2((2*muP11),(muP20-muP02));
    //float anguloOrient = 0.5*atan2((muP20-muP02),(2*muP11));
    //float anguloOrient = 0.5*atan((2*muP11)/(muP20-muP02));
    //std::cout << "Orientación Imagen: " << anguloOrient*180/PI << std::endl;

    // Calcular centros de masa

    centerX = Momentos.m10/Momentos.m00;
    centerY = Momentos.m01/Momentos.m00;
    //float dirX = centroX+50*cos(anguloOrient);


    //cv::Point pt1;
    //cv::Point pt2;
    //pt1.x = centroX;
    //pt1.y = centroY;
    //pt2.x = centroX+50*cos(anguloOrient);
    //pt2.y = centroY+50*sin(anguloOrient);

   // Dibujar orientación

    //cv::line(image, pt1, pt2, cv::Scalar( 110, 220, 0 ));

    //cv::imshow("Imagen Prueba Binaria", image);
}

void RobotPoseProvider::analizePosibleRobot()
{
  /*if(posibleRobot.shoulders.size() != 2)
  {
    posibleRobot.valid = false;
    return;
  }
  for(const RobotsIdentifiers::Identifier& identifier : theRobotsIdentifiers.identifiers)
  {
    if(posibleRobot.shoulders[0].color.is(identifier.leftShoulder) && posibleRobot.shoulders[1].color.is(identifier.rightShoulder))
    {
      posibleRobot.valid = true;
      posibleRobot.team = identifier.team;
      posibleRobot.number = identifier.number;
      posibleRobot.leftShoulder = Transformation::imageToImageCorrected(posibleRobot.shoulders[0].center,theCameraInfo);
      posibleRobot.rightShoulder = Transformation::imageToImageCorrected(posibleRobot.shoulders[1].center,theCameraInfo);
      return;
    }
    if(posibleRobot.shoulders[0].color.is(identifier.rightShoulder) && posibleRobot.shoulders[1].color.is(identifier.leftShoulder))
    {
      posibleRobot.valid = true;
      posibleRobot.team = identifier.team;
      posibleRobot.number = identifier.number;
      posibleRobot.leftShoulder = Transformation::imageToImageCorrected(posibleRobot.shoulders[1].center,theCameraInfo);
      posibleRobot.rightShoulder = Transformation::imageToImageCorrected(posibleRobot.shoulders[0].center,theCameraInfo);
      return;
    }
  }*/


}


void RobotPoseProvider::calculatePose(RobotsPoses &robotsPoses)
{
  Vector2<> center = Vector2<>((posibleRobot.leftShoulder.x + posibleRobot.rightShoulder.x)/2.f,(posibleRobot.leftShoulder.y + posibleRobot.rightShoulder.y)/2.f);
  
  Vector2<> direction = Vector2<>(posibleRobot.rightShoulder.x - posibleRobot.leftShoulder.x, posibleRobot.rightShoulder.y - posibleRobot.leftShoulder.y).rotateLeft();
  
  Vector2<int> centerInImage = Transformation::imageCorrectedToImage(Vector2<int>(center.x, center.y),theCameraInfo);
  
  robotsPoses.robotPoses.push_back(RobotsPoses::RobotPose(direction.angle(),center,centerInImage,posibleRobot.team,posibleRobot.number));
  
}
