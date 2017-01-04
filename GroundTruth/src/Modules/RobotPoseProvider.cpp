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



      cv::Mat puntosIm = theImage.clone();
      // Calcurar sentido robot
      int distBlue = 100000;
      int distOrange = 100000;
      int lastBlueX = 0;
      int lastBlueY = 0;
      int lastOrangeX = 0;
      int lastOrangeY = 0;
      std::vector<cv::Point> pointsRegresionOrange;
      std::vector<cv::Point> pointsRegresionBlue;
      std::cout << "1" << std::endl;
      for(int i = robot.leftUpper.y; i < robot.rightBottom.y; i++)
      {
          for (int j = robot.leftUpper.x; j < robot.rightBottom.x; j++)
          {
              if (theColorModel.getColor(theImage.at<cv::Vec3b>(i,j)).is(blue))
              {
                  distBlue = 0;
                  lastBlueX = j;
                  lastBlueY = i;
                  std::cout << "distOrange: " << distOrange << std::endl;
                  if (distOrange < 10)
                  {
                      pointsRegresionBlue.push_back(cv::Point((lastOrangeX + j)/2, (lastOrangeY + i)/2));
                      std::cout << (lastOrangeX + j)/2 << std::endl;
                      std::cout << (lastOrangeY + i)/2 << std::endl;
                      //circle(puntosIm, cv::Point((lastOrangeX + j)/2, (lastOrangeY + i)/2), 2, cv::Scalar( 110, 220, 0 ));
                      //imshow("plx", puntosIm);
                      distOrange = 10000;
                  }
                  //std::cout << "2" << std::endl;

              }

              if (theColorModel.getColor(theImage.at<cv::Vec3b>(i,j)).is(orange))
              {
                  distOrange = 0;
                  lastOrangeX = j;
                  lastOrangeY = i;
                  std::cout << "distBlue: " << distBlue << std::endl;
                  if (distBlue < 10)
                  {
                      pointsRegresionOrange.push_back(cv::Point((lastBlueX + j)/2, (lastBlueY + i)/2));
                      std::cout << (lastBlueX + j)/2 << std::endl;
                      std::cout << (lastBlueY + i)/2 << std::endl;
                      distBlue = 10000;
                  }
                  //std::cout << "3" << std::endl;
              }
              distBlue++;
              distOrange++;
          }
      }
      // Ver cuál vector gana
      cv::Vec4f puntosLinea;
      cv::Point pt1;
      cv::Point pt2;
      if(pointsRegresionBlue.size() > 3 || pointsRegresionOrange.size() > 3)
      {
          if (pointsRegresionBlue.size() > pointsRegresionOrange.size())
          {
              std::cout << "4" << std::endl;
              // Blue a la derecha
              cv::fitLine(pointsRegresionBlue, puntosLinea, CV_DIST_L2, 0, 0.01, 0.01);
          }
          else
          {
              std::cout << "5" << std::endl;
              // Orange a la derecha
              cv::fitLine(pointsRegresionOrange, puntosLinea, CV_DIST_L2, 0, 0.01, 0.01);
          }
          // Revisar Linea

         /* int centerBlueX, centerBlueY;
          int centerOrangeX, centerOrangeY;
          float anguloOrientBlue, anguloOrientOrange;
          getPicDir(blueSegmented, centerBlueX, centerBlueY, anguloOrientBlue);
          getPicDir(orangeSegmented, centerOrangeX, centerOrangeY, anguloOrientOrange);
          float anguloOrient = std::abs(anguloOrientOrange);
          int naoCenterX = (centerBlueX + centerOrangeX)/2;
          int naoCenterY = (centerBlueY + centerOrangeY)/2;
          pt1.x = naoCenterX;
          pt1.y = naoCenterY;*/
          pt1.x = puntosLinea[2];
          pt1.y = puntosLinea[3];

          int xTest = pt1.y+6*puntosLinea[0];
          int yTest = pt1.x-6*puntosLinea[1];

          if(theColorModel.getColor(theImage.at<cv::Vec3b>(xTest,yTest)).is(orange) || theColorModel.getColor(theImage.at<cv::Vec3b>(xTest+1,yTest)).is(orange) || theColorModel.getColor(theImage.at<cv::Vec3b>(xTest-1,yTest)).is(orange) || theColorModel.getColor(theImage.at<cv::Vec3b>(xTest,yTest+1)).is(orange) || theColorModel.getColor(theImage.at<cv::Vec3b>(xTest,yTest-1)).is(orange) || theColorModel.getColor(theImage.at<cv::Vec3b>(xTest+1,yTest+1)).is(orange) || theColorModel.getColor(theImage.at<cv::Vec3b>(xTest-1,yTest-1)).is(orange) || theColorModel.getColor(theImage.at<cv::Vec3b>(xTest-1,yTest+1)).is(orange) || theColorModel.getColor(theImage.at<cv::Vec3b>(xTest-1,yTest-1)).is(orange))
          {
              pt2.x = pt1.x-30*puntosLinea[0];
              pt2.y = pt1.y-30*puntosLinea[1];
          }
          else
          {
              pt2.x = pt1.x+30*puntosLinea[0];
              pt2.y = pt1.y+30*puntosLinea[1];
          }
          std::cout << "6" << std::endl;
          std::cout << "prueba punto: " << puntosLinea[0] << std::endl;
      }
      else
      {
          return;
          std::cout << "No hay suficientes puntos" << std::endl;
      }








      //std::cout << blueSegmented << std::endl;







      //pt1.x = naoCenterX;
      //pt1.y = naoCenterY;
      /*if (std::abs(centerBlueX - centerOrangeX) > std::abs(centerBlueY - centerOrangeY))
      {
          std::cout << "ganó x" << std::endl;
          if(centerBlueX < naoCenterX)
          {
              std::cout << "1" << std::endl;
              pt2.x = naoCenterX+50*cos(anguloOrient);
              pt2.y = naoCenterY+50*sin(anguloOrient);
          }
          else
          {
              std::cout << "2" << std::endl;
              pt2.x = naoCenterX-50*cos(anguloOrient);
              pt2.y = naoCenterY-50*sin(anguloOrient);
          }
      }
      else
      {
          std::cout << "ganó y" << std::endl;
          if(centerBlueY > naoCenterY)
          {
              std::cout << "3" << std::endl;
              pt2.x = naoCenterX+50*cos(anguloOrient);
              pt2.y = naoCenterY+50*sin(anguloOrient);
          }
          else
          {
              std::cout << "4" << std::endl;
              pt2.x = naoCenterX-50*cos(anguloOrient);
              pt2.y = naoCenterY-50*sin(anguloOrient);
          }
      }*/
      //cv::Mat dirNao = theImage.clone();
      //cv::line(dirNao, pt1, pt2, cv::Scalar( 110, 220, 0 ));
      //cv::imshow("DirNao", dirNao);



      //cameraSwitch = cameraSwitch*(-1);
      if (cameraSwitch == 1)
      {
          std::cout << "1" << std::endl;
          if (updatePose1 == (pond-1))
          {
              robotsPoses.robotPoses.clear();
              pt1X1 = pt1X1/pond;
              pt1Y1 = pt1Y1/pond;
              pt2X1 = pt2X1/pond;
              pt2Y1 = pt2Y1/pond;
              Vector2<int> centerInImage = Vector2<int>(pt1X1, pt1Y1);
              Vector2<int> pointDir = Vector2<int>(pt2X1, pt2Y1);
              robotsPoses.robotPoses.push_back(RobotsPoses::RobotPose(centerInImage,pointDir));
              updatePose1 = 0;
          }
          else
          {
            pt1X1 += pt1.x;
            pt1Y1 += pt1.y;
            pt2X1 += pt2.x;
            pt2Y1 += pt2.y;
            updatePose1++;
          }
      }
      else
      {
          std::cout << "2" << std::endl;
          if (updatePose2 == (pond-1))
          {
              robotsPoses.robotPoses.clear();
              pt1X2 = pt1X2/pond;
              pt1Y2 = pt1Y2/pond;
              pt2X2 = pt2X2/pond;
              pt2Y2 = pt2Y2/pond;
              Vector2<int> centerInImage = Vector2<int>(pt1X2, pt1Y2);
              Vector2<int> pointDir = Vector2<int>(pt2X2, pt2Y2);
              robotsPoses.robotPoses.push_back(RobotsPoses::RobotPose(centerInImage,pointDir));
              updatePose2 = 0;
          }
          else
          {
            pt1X2 += pt1.x;
            pt1Y2 += pt1.y;
            pt2X2 += pt2.x;
            pt2Y2 += pt2.y;
            updatePose2++;
          }
      }

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
  
  //robotsPoses.robotPoses.push_back(RobotsPoses::RobotPose(direction.angle(),center,centerInImage,posibleRobot.team,posibleRobot.number));
  
}
