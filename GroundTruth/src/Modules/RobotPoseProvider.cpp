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



      //cv::Mat puntosIm = theImage.clone();

      // Calcurar sentido robot recorriendo en x
      int distBlueHor = 100000;
      int distOrangeHor = 100000;
      int lastBlueXHor = 0;
      int lastBlueYHor = 0;
      int lastOrangeXHor = 0;
      int lastOrangeYHor = 0;
      std::vector<cv::Point> pointsRegresionOrangeHor;
      std::vector<cv::Point> pointsRegresionBlueHor;
      std::cout << "1" << std::endl;
      for(int i = robot.leftUpper.y; i < robot.rightBottom.y; i++)
      {
          for (int j = robot.leftUpper.x; j < robot.rightBottom.x; j++)
          {
              if (theColorModel.getColor(theImage.at<cv::Vec3b>(i,j)).is(blue))
              {
                  distBlueHor = 0;
                  lastBlueXHor = j;
                  lastBlueYHor = i;
                  std::cout << "distOrangeHor: " << distOrangeHor << std::endl;
                  if (distOrangeHor < 10)
                  {
                      pointsRegresionBlueHor.push_back(cv::Point((lastOrangeXHor + j)/2, (lastOrangeYHor + i)/2));
                      std::cout << (lastOrangeXHor + j)/2 << std::endl;
                      std::cout << (lastOrangeYHor + i)/2 << std::endl;
                      //circle(puntosIm, cv::Point((lastOrangeX + j)/2, (lastOrangeY + i)/2), 2, cv::Scalar( 110, 220, 0 ));
                      //imshow("plx", puntosIm);
                      distOrangeHor = 10000;
                  }
                  //std::cout << "2" << std::endl;

              }

              if (theColorModel.getColor(theImage.at<cv::Vec3b>(i,j)).is(orange))
              {
                  distOrangeHor = 0;
                  lastOrangeXHor = j;
                  lastOrangeYHor = i;
                  std::cout << "distBlueHor: " << distBlueHor << std::endl;
                  if (distBlueHor < 10)
                  {
                      pointsRegresionOrangeHor.push_back(cv::Point((lastBlueXHor + j)/2, (lastBlueYHor + i)/2));
                      std::cout << (lastBlueXHor + j)/2 << std::endl;
                      std::cout << (lastBlueYHor + i)/2 << std::endl;
                      distBlueHor = 10000;
                  }
                  //std::cout << "3" << std::endl;
              }
              distBlueHor++;
              distOrangeHor++;
          }
      }

      // Calcular sentido del robot recorriendo en y

      int distBlueVer = 100000;
      int distOrangeVer = 100000;
      int lastBlueXVer = 0;
      int lastBlueYVer = 0;
      int lastOrangeXVer = 0;
      int lastOrangeYVer = 0;
      std::vector<cv::Point> pointsRegresionOrangeVer;
      std::vector<cv::Point> pointsRegresionBlueVer;
      std::cout << "1" << std::endl;
      for (int j = robot.leftUpper.x; j < robot.rightBottom.x; j++)
      {
          for(int i = robot.leftUpper.y; i < robot.rightBottom.y; i++)
          {
              if (theColorModel.getColor(theImage.at<cv::Vec3b>(i,j)).is(blue))
              {
                  distBlueVer = 0;
                  lastBlueXVer = j;
                  lastBlueYVer = i;
                  std::cout << "distOrangeVer: " << distOrangeVer << std::endl;
                  if (distOrangeVer < 10)
                  {
                      pointsRegresionBlueVer.push_back(cv::Point((lastOrangeXVer + j)/2, (lastOrangeYVer + i)/2));
                      std::cout << (lastOrangeXVer + j)/2 << std::endl;
                      std::cout << (lastOrangeYVer + i)/2 << std::endl;
                      //circle(puntosIm, cv::Point((lastOrangeX + j)/2, (lastOrangeY + i)/2), 2, cv::Scalar( 110, 220, 0 ));
                      //imshow("plx", puntosIm);
                      distOrangeVer = 10000;
                  }
                  //std::cout << "2" << std::endl;

              }

              if (theColorModel.getColor(theImage.at<cv::Vec3b>(i,j)).is(orange))
              {
                  distOrangeVer = 0;
                  lastOrangeXVer = j;
                  lastOrangeYVer = i;
                  std::cout << "distBlueVer: " << distBlueVer << std::endl;
                  if (distBlueVer < 10)
                  {
                      pointsRegresionOrangeVer.push_back(cv::Point((lastBlueXVer + j)/2, (lastBlueYVer + i)/2));
                      std::cout << (lastBlueXVer + j)/2 << std::endl;
                      std::cout << (lastBlueYVer + i)/2 << std::endl;
                      distBlueVer = 10000;
                  }
                  //std::cout << "3" << std::endl;
              }
              distBlueVer++;
              distOrangeVer++;
          }
      }


      // Ver cuál vector gana
      cv::Vec4f puntosLinea;
      cv::Point pt1;
      cv::Point pt2;
      if(pointsRegresionBlueHor.size() > 3 || pointsRegresionOrangeHor.size() > 3 || pointsRegresionBlueVer.size() > 3 || pointsRegresionOrangeVer.size() > 3)
      {
          if (pointsRegresionBlueHor.size() > pointsRegresionOrangeHor.size())
          {
              if (pointsRegresionBlueHor.size() > pointsRegresionBlueVer.size())
              {
                 if (pointsRegresionBlueHor.size() > pointsRegresionOrangeVer.size())
                 {
                     std::cout << "8" << std::endl;
                     // Blue a la derecha
                     cv::fitLine(pointsRegresionBlueHor, puntosLinea, CV_DIST_L2, 0, 0.01, 0.01);
                 }
              }
          }

          if (pointsRegresionOrangeHor.size() > pointsRegresionBlueHor.size())
          {
              if (pointsRegresionOrangeHor.size() > pointsRegresionBlueVer.size())
              {
                 if (pointsRegresionOrangeHor.size() > pointsRegresionOrangeVer.size())
                 {
                     std::cout << "9" << std::endl;
                     // Blue a la derecha
                     cv::fitLine(pointsRegresionOrangeHor, puntosLinea, CV_DIST_L2, 0, 0.01, 0.01);
                 }
              }
          }

          if (pointsRegresionBlueVer.size() > pointsRegresionOrangeHor.size())
          {
              if (pointsRegresionBlueVer.size() > pointsRegresionBlueHor.size())
              {
                 if (pointsRegresionBlueVer.size() > pointsRegresionOrangeVer.size())
                 {
                     std::cout << "10" << std::endl;
                     // Blue a la derecha
                     cv::fitLine(pointsRegresionBlueVer, puntosLinea, CV_DIST_L2, 0, 0.01, 0.01);
                 }
              }
          }

          if (pointsRegresionOrangeVer.size() > pointsRegresionBlueHor.size())
          {
              if (pointsRegresionOrangeVer.size() > pointsRegresionBlueVer.size())
              {
                 if (pointsRegresionOrangeVer.size() > pointsRegresionOrangeHor.size())
                 {
                     std::cout << "11" << std::endl;
                     // Blue a la derecha
                     cv::fitLine(pointsRegresionOrangeVer, puntosLinea, CV_DIST_L2, 0, 0.01, 0.01);
                 }
              }
          }
          /*
          else
          {
              std::cout << "5" << std::endl;
              // Orange a la derecha
              cv::fitLine(pointsRegresionOrangeHor, puntosLinea, CV_DIST_L2, 0, 0.01, 0.01);
          }*/
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
