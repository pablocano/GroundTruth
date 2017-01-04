/**
 * @file FieldCalibrator.cpp
 * FieldCalibrator based on perspective projections
 * @author Matías Mattamala
 */

#include <iostream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Representations/FieldDimensions.h"

#define CAM1 1
#define CAM2 2

using namespace cv;
using namespace std;

// program state enum
enum
{
    IDLE = 0,
    CALIBRATING = 1,
    DONE = 3,
    TEST = 4
};

// rotation enum
enum
{
    CLOCKWISE = 90,
    COUNTERCLOCKWISE = -90
};

// Field Dimensions
FieldDimensions fieldDimensions;

static int width = 640;
static int height = 480;

static int xMouse1;
static int yMouse1;
static int xMouse2;
static int yMouse2;

// vector of captured points
int cam1Counter = 0;
int cam2Counter = 0;
std::vector<cv::Point2d> cam1Points;
std::vector<cv::Point2d> cam2Points;

bool isCam1Ready = false;
bool isCam2Ready = false;

// vectors of real 3D points
std::vector<cv::Point3d> field1Points;
std::vector<cv::Point3d> field2Points;

// vector of points for calibration
//std::vector<cv::Point> calibrationPoints1;
//std::vector<cv::Point> calibrationPoints2;
cv::Point2f calibrationPoints1[4];
cv::Point2f calibrationPoints2[4];

cv::Mat transform1;
cv::Mat transform2;
// calibrating info
// camera 1
cv::Mat image1, image2;
cv::Mat K1, d1;
// camera 2
cv::Mat K2, d2;

int count1 = 0;
int count2 = 0;

int just1 = 0;
int just2 = 0;

void readCameraCalibration(int cam)
{
    if(cam == 1)
    {
        cv::FileStorage file("../../Config/cameraCalibrationTmp1.yml", cv::FileStorage::READ);
        if(!file.isOpened())
        {
          std::cout << "Could not open the camera calibration file"<< std::endl;
        }
        file["Camera_Matrix" ] >> K1;
        file["Distortion_Coefficients"] >> d1;
        file.release();

        cout << K1 << endl;
        cout << d1 << endl;
    }
    else
    {
        cv::FileStorage file("../../Config/cameraCalibrationTmp2.yml", cv::FileStorage::READ);
        if(!file.isOpened())
        {
          std::cout << "Could not open the camera calibration file"<< std::endl;
        }
        file["Camera_Matrix" ] >> K2;
        file["Distortion_Coefficients"] >> d2;
        file.release();

        cout << K2 << endl;
        cout << d2 << endl;
    }
}

void rotateImage90(cv::Mat &src, cv::Mat &dst, int angle)
{
    dst.create(src.size(), src.type());
    if(angle == 270 || angle == -90){
        // Rotate clockwise 270 degrees
        cv::transpose(src, dst);
        cv::flip(dst, dst, 0);
    }else if(angle == 180 || angle == -180){
        // Rotate clockwise 180 degrees
        cv::flip(src, dst, -1);
    }else if(angle == 90 || angle == -270){
        // Rotate clockwise 90 degrees
        cv::transpose(src, dst);
        cv::flip(dst, dst, 1);
    }else if(angle == 360 || angle == 0){
        if(src.data != dst.data){
            src.copyTo(dst);
        }
    }
}

void combineImages(cv::Mat& img1, cv::Mat& img2, cv::Mat& combined)
{
    combined = cv::Mat(img1.rows, 2*img1.cols, img1.type());

    combined.adjustROI(0, 0, 0, -img1.cols);
    img1.copyTo(combined);
    combined.adjustROI(0,0,0,height);
    combined.adjustROI(0, 0, -img1.cols, img1.cols);
    img2.copyTo(combined);
    combined.adjustROI(0,0, img1.cols, 0);
}


cv::Point img1_P1(0,0);
cv::Point img1_P2(0,0);
cv::Point img2_P1(0,0);
cv::Point img2_P2(0,0);
cv::Point center1(0,0);
cv::Point center2(0,0);
bool clicked1 = false;
bool clicked2 = false;

void showImage(char cam)
{
    if(cam == 1)
    {
        cv::Mat img = image1.clone();
        line(img, img1_P1, img1_P2, Scalar(0, 0, 255), 3);
        imshow("cam1", img);
    }
    else if (cam == 2)
    {
        cv::Mat img = image2.clone();
        line(img, img2_P1, img2_P2, cv::Scalar(0, 0, 255), 3);
        imshow("cam2", img);
    }
    else
    {
        cout << "Error displaying the image" << endl;
    }
}

void applyTransform(cv::Point &point, const cv::Mat &transform)
{
    int x = point.x;
    int y = point.y;
    /*
    point.x = (transform.at<float>(0,0)*x + transform.at<float>(0,1)*y + transform.at<float>(0,2))/(transform.at<float>(2,0)*x + transform.at<float>(2,1)*y + transform.at<float>(2,2));
    point.y = (transform.at<float>(1,0)*x + transform.at<float>(1,1)*y + transform.at<float>(1,2))/(transform.at<float>(2,0)*x + transform.at<float>(2,1)*y + transform.at<float>(2,2));
    */
    //cv::Point2f origPoints[1];
    //cv::Point2f transformedPoints[1];
    vector<Point2f> origPoints;
    vector<Point2f> transformedPoints;
    //cout << "a ver x: " << point.x << std::endl;
    //cout << "a ver y: " << point.y << std::endl;
    origPoints.push_back(Point(x,y));
    //origPoints[0] = Point2f(point.x,point.y);
    cv::perspectiveTransform(origPoints, transformedPoints, transform);
    point.x = transformedPoints[0].x;
    point.y = transformedPoints[0].y;
    std::cout << "Transformed points: " << transformedPoints << std::endl;

}

void test1(int event, int x, int y, int flags, void* userdata)
{
    switch(event){

        case  CV_EVENT_LBUTTONDOWN:
            clicked1 = true;
            img1_P1.x = x;
            img1_P1.y = y;
            applyTransform(img1_P1,transform1);
            cout << "Camera 1 - Mouse position (" << x << ", " << y << ")" << endl;
            cout << "Coordenada: " << img1_P1.x << ", " << img1_P1.y << endl;
            break;

        default: break;
    }
    if(clicked1)
        showImage(1);
}

void test2(int event, int x, int y, int flags, void* userdata)
{
    switch(event){

        case  CV_EVENT_LBUTTONDOWN:
            clicked1 = true;
            img2_P1.x = x;
            img2_P1.y = y;
            applyTransform(img2_P1,transform2);
            cout << "Camera 2 - Mouse position (" << x << ", " << y << ")" << endl;
            cout << "Coordenada: " << img2_P1.x << ", " << img2_P1.y << endl;
            break;

        default: break;
    }
    if(clicked1)
        showImage(1);
}

void onMouseRuler1( int event, int x, int y, int flags, void* userdata)
{
    switch(event){

        case  CV_EVENT_LBUTTONDOWN:
            clicked1 = true;
            img1_P1.x = x;
            img1_P1.y = y;
            img1_P2.x = x;
            img1_P2.y = y;
            cout << "Camera 1 - Mouse position (" << x << ", " << y << ")" << endl;
            break;

        case  CV_EVENT_RBUTTONDOWN:
            clicked1 = true;
            center1.x = x;
            center1.y = y;
            cout << "Field center 1 obtained." << std::endl;
            break;

        case  CV_EVENT_LBUTTONUP:
            calibrationPoints1[count1] = Point2f(img1_P1.x, img1_P1.y);
            count1++;
            img1_P2.x = x;
            img1_P2.y = y;
            clicked1 = false;
            break;

        case  CV_EVENT_MOUSEMOVE:
            if(clicked1){
                img1_P2.x = x;
                img1_P2.y = y;
            }
            break;

        default: break;
    }
    if(clicked1)
        showImage(1);
}

void onMouseRuler2( int event, int x, int y, int flags, void* userdata)
{
    switch(event){

        case  CV_EVENT_LBUTTONDOWN:
            clicked2 = true;
            img2_P1.x = x;
            img2_P1.y = y;
            img2_P2.x = x;
            img2_P2.y = y;
            cout << "Camera 2 - Mouse position (" << x << ", " << y << ")" << endl;
            break;

        case  CV_EVENT_RBUTTONDOWN:
            clicked1 = true;
            center2.x = x;
            center2.y = y;
            cout << "Field center 2 obtained." << std::endl;
            break;

        case  CV_EVENT_LBUTTONUP:
            img2_P2.x = x;
            img2_P2.y = y;
            calibrationPoints2[count2] = Point2f(img2_P1.x, img2_P1.y);
            count2++;
            clicked2 = false;
            break;

        case  CV_EVENT_MOUSEMOVE:
            if(clicked2){
                img2_P2.x = x;
                img2_P2.y = y;
            }
            break;

        default: break;
    }
    if(clicked2)
        showImage(2);
}

void computeImageFieldRatiosAndSave()
{
    Point dist1 = img1_P1-img1_P2;
    Point dist2 = img2_P1-img2_P2;

    Point fieldCenter1 = (img1_P1+img1_P2) * 0.5f;
    Point fieldCenter2 = (img2_P1+img2_P2) * 0.5f;

    float distImg1 = sqrt(dist1.dot(dist1));
    float distImg2 = sqrt(dist2.dot(dist2));

    float fieldDist = 2*fieldDimensions.yLimit;

    float pix2World1 = fieldDist/distImg1;
    float pix2World2 = fieldDist/distImg2;

    cout << "Pixel-to-World, Cam1: " << pix2World1 << endl;
    cout << "Pixel-to-World, Cam2: " << pix2World2 << endl;


    // save configuration
    cv::FileStorage fs1( "../../Config/cameraCalibration1.yml", FileStorage::WRITE );
    cv::FileStorage fs2( "../../Config/cameraCalibration2.yml", FileStorage::WRITE );

    fs1 << "Camera_Matrix" << K1;
    fs1 << "Distortion_Coefficients" << d1;
    fs1 << "Pixel_to_World" << pix2World1;
    fs1 << "Field_Center" << fieldCenter1;

    fs2 << "Camera_Matrix" << K2;
    fs2 << "Distortion_Coefficients" << d2;
    fs2 << "Pixel_to_World" << pix2World2;
    fs2 << "Field_Center" << fieldCenter2;
}

void computeCalibrationAndSave()
{
    //Point dist1 = img1_P1-img1_P2;
    //Point dist2 = img2_P1-img2_P2;

    Point fieldCenter1 = center1;
    Point fieldCenter2 = center2;

    //float distImg1 = sqrt(dist1.dot(dist1));
    //float distImg2 = sqrt(dist2.dot(dist2));

    //float fieldDist = 2*fieldDimensions.yLimit;

    //float pix2World1 = fieldDist/distImg1;
    //float pix2World2 = fieldDist/distImg2;

    //cout << "Pixel-to-World, Cam1: " << pix2World1 << endl;
    //cout << "Pixel-to-World, Cam2: " << pix2World2 << endl;

    /*cv::Point real1_P1(3000,2000);
    cv::Point real1_P2(0,-2000);
    cv::Point real1_P3(0,2000);
    cv::Point real1_P4(3000,2000);*/

    //std::vector<cv::Point> real1;
    cv::Point2f real1[4];

    real1[0] = Point2f(3000,-2000);
    real1[1] = Point2f(0,-2000);
    real1[2] = Point2f(0,2000);
    real1[3] = Point2f(3000,2000);
    /*real1.push_back(real1_P1);
    real1.push_back(real1_P2);
    real1.push_back(real1_P2);
    real1.push_back(real1_P2);*/

    cv::Point2f real2[4];

    real2[0] = Point2f(0,-2000);
    real2[1] = Point2f(-3000,-2000);
    real2[2] = Point2f(-3000,2000);
    real2[3] = Point2f(0,2000);

    /*cv::Point real2_P1(0,-2000);
    cv::Point real2_P2(3000,-2000);
    cv::Point real2_P3(-3000,2000);
    cv::Point real2_P4(0,2000);

    std::vector<cv::Point> real2;

    real2.push_back(real2_P1);
    real2.push_back(real2_P2);
    real2.push_back(real2_P2);
    real2.push_back(real2_P2);*/

    transform1 = cv::getPerspectiveTransform(calibrationPoints1, real1);
    transform2 = cv::getPerspectiveTransform(calibrationPoints2, real2);

    std::cout << "Transformation 1: " << transform1 << std::endl;
    std::cout << "Transformation 2: " << transform2 << std::endl;

   /* int x = 309;
    int y = 317;

    cv::Point pTest(x,y);
    applyTransform(pTest, transform1);*/



    // save configuration
    cv::FileStorage fs1( "../../Config/cameraCalibrationTransform1.yml", FileStorage::WRITE );
    cv::FileStorage fs2( "../../Config/cameraCalibrationTransform2.yml", FileStorage::WRITE );

    fs1 << "Camera_Matrix" << K1;
    fs1 << "Distortion_Coefficients" << d1;
    fs1 << "Pixel_to_World" << transform1;
    fs1 << "Field_Center" << fieldCenter1;

    fs2 << "Camera_Matrix" << K2;
    fs2 << "Distortion_Coefficients" << d2;
    fs2 << "Pixel_to_World" << transform2;
    fs2 << "Field_Center" << fieldCenter2;
}

// main program
int main(int argc, char* argv[])
{
    // some state variables
    int state = IDLE;

    // read camera calibration files
    readCameraCalibration(1);
    readCameraCalibration(2);

    // prepare cameras
    // open first camera
    cv::VideoCapture cam1(CAM1);
    cam1.set(CV_CAP_PROP_FRAME_HEIGHT, height);
    cam1.set(CV_CAP_PROP_FRAME_WIDTH, width);

    // open second camera
    cv::VideoCapture cam2(CAM2);
    cam2.set(CV_CAP_PROP_FRAME_HEIGHT, height);
    cam2.set(CV_CAP_PROP_FRAME_WIDTH, width);

    // create windows and set properties
    //Create a window
    namedWindow("cam1", WINDOW_NORMAL | CV_GUI_EXPANDED);
    namedWindow("cam2", WINDOW_NORMAL | CV_GUI_EXPANDED);
    namedWindow("combined", WINDOW_NORMAL);

    char key;
    cv::Mat img1, img2, undistorted1, undistorted2, rotated1, rotated2, combined;


    /** MAIN LOOP **/
    while (true){

        key = cv::waitKey(1);
        if(key == 27)
            break;



        /*if(calibrationPoints1.size() == 4 && just1 == 0)
        {
            std::cout << "Se tienen 4 puntos para calibar en 1." << std::endl;
            just1 = 1;
        }

        if(calibrationPoints2.size() == 4 && just2 == 2)
        {
            std::cout << "Se tienen 4 puntos para calibar en 2." << std::endl;
            just2 = 1;
        }*/

        if(state == IDLE)
        {
            // get images (we assume this run right everytime)
            cam1 >> img1;
            cam2 >> img2;

            undistorted1 = img1.clone();
            undistorted2 = img2.clone();

            cv::undistort(img1, undistorted1, K1, d1);
            cv::undistort(img2, undistorted2, K2, d2);

            rotateImage90(undistorted1, rotated1, CLOCKWISE);
            rotateImage90(undistorted2, rotated2, COUNTERCLOCKWISE);
            combineImages(rotated1, rotated2, combined);

            // show images
            imshow("cam1", rotated1);
            imshow("cam2", rotated2);
            imshow("combined", combined);

            if(key == 'c')
            {
                image1 = rotated1;
                image2 = rotated2;
                state = CALIBRATING;
            }
            if(key == 't')
            {
                image1 = rotated1;
                image2 = rotated2;
                state = TEST;
            }


        }

        if(state == CALIBRATING)
        {
            setMouseCallback( "cam1", onMouseRuler1, NULL);
            setMouseCallback( "cam2", onMouseRuler2, NULL);

            // cancel calibrationg of field depth
            if(key == 'x')
            {
                //computeImageFieldRatiosAndSave();
                computeCalibrationAndSave();
                state = IDLE;
            }


        }

        if(state == TEST)
        {
            setMouseCallback( "cam1", test1, NULL);
            setMouseCallback( "cam2", test2, NULL);
        }

    }

    return 0;
}
