#include "descriptortest.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
using namespace cv;
using namespace std;

void genTransform(cv::DMatch match, std::vector<cv::KeyPoint> &keypoints1, std::vector<cv::KeyPoint> &keypoints2, double &e, double &theta, double &tx, double &ty);

int computeConsensus(vector<DMatch> &matches, vector<KeyPoint> &keypoints1, vector<KeyPoint> &keypoints2, vector<int> &selected, double e, double theta, double tx, double ty, double umbralpos);

bool ransac(vector<DMatch> &matches, vector<KeyPoint> &keypoints1, vector<KeyPoint> &keypoints2, vector<DMatch> &accepted, double &e, double &theta, double &tx, double &ty);

void Descriptor::update()
{
    //std::cout << "Prueba descriptor" << std::endl;
    cv::Mat image = cv::imread("/home/rodrigo/Videos/NAOs/captura10.jpg", CV_LOAD_IMAGE_GRAYSCALE);   // Read the file
    cv::Mat imageComp = cv::imread("/home/rodrigo/Videos/NAOs/captura54.jpg", CV_LOAD_IMAGE_GRAYSCALE);   // Read the file


    /**
     * Prepare Camera Info
     * */
    cv::Mat K, d;
    cv::Point fieldCenter;
    float pix2World;
    // Load Camera 1 config
    cv::FileStorage file1( std::string(File::getGTDir()) + "/Config/cameraCalibration1.yml", cv::FileStorage::READ);
    if(!file1.isOpened())
    {
      std::cout << "Could not open the camera 1 calibration file"<< std::endl;
    }
    file1["Camera_Matrix" ] >> K;
    file1["Distortion_Coefficients"] >> d;
    file1["Pixel_to_World"] >> pix2World;
    file1["Field_Center"] >> fieldCenter;
    file1.release();
    //CameraInfo(CameraInfo::eastCam, "Camera 1", K, d, fieldCenter, pix2World);

    cv::Mat undistorted, rotated;

    // correct and rotate images
    cv::undistort(image, undistorted, K, d);
    Camera::rotateImage90(undistorted, rotated, 90);
    //cv::namedWindow( "Display window", cv::WINDOW_NORMAL);// Create a window for display.
    //cv::imshow( "Display window", image);                   // Show our image inside it.
    //cv::waitKey(0);

    //Extract Features
    std::vector<cv::KeyPoint> keypoints1;
    std::vector<cv::KeyPoint> keypoints2;
    cv::Mat descriptors1, descriptors2, output, outputComp;

    //SIFT extractor;
    //SURF extractor;
    cv::ORB extractor;
    extractor(image, cv::Mat(), keypoints1, descriptors1);
    extractor(imageComp, cv::Mat(), keypoints2, descriptors2);

    //Matching
    cv::BFMatcher matcher(cv::NORM_HAMMING); // Para ORB y otros binarios
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors2, descriptors1, matches);

    //Filtering matches using RANSAC
    double e=1, theta=0, tx=0, ty=0;
    DMatch match = matches[0];
    vector<DMatch> acceptedRansac;
    ransac(matches, keypoints1, keypoints2, acceptedRansac, e, theta, tx, ty);

    // Draw Images
    /*cv::drawKeypoints(image, keypoints1, output);
    cv::imshow("keypoints1", output);
    cv::drawKeypoints(imageComp, keypoints2, outputComp);
    cv::namedWindow( "keypoints2", cv::WINDOW_NORMAL);// Create a window for display.
    cv::imshow("keypoints2", outputComp);*/

    /*cv::namedWindow("matches", cv::WINDOW_NORMAL);
    cv::Mat img_matches;
    cv::drawMatches(imageComp, keypoints2, image, keypoints1, matches, img_matches);
    cv::imshow("matches", img_matches);*/

    cv::namedWindow("acceptedRansac", cv::WINDOW_NORMAL);
    Mat img_matches;
    drawMatches(imageComp, keypoints2, image, keypoints1, acceptedRansac, img_matches);
    imshow("acceptedRansac", img_matches);
    imwrite("acceptedRansac.jpg", img_matches);

    cv::waitKey(0);
}

// Codigo necesario para RANSAC
void genTransform(DMatch match, vector<KeyPoint> &keypoints1, vector<KeyPoint> &keypoints2, double &e, double &theta, double &tx, double &ty)
{
    // Calcular transformacion de semejanza (e,theta,tx,ty) a partir de un calce "match".
    // Se debe notar que los keypoints de OpenCV tienen orientacion en grados.

    // Obtener valores de los descriptores y puntos de interés
    int tIdx = match.trainIdx;
    int qIdx = match.queryIdx;
    int queryX = keypoints2[qIdx].pt.x;
    int queryY = keypoints2[qIdx].pt.y;
    int trainX = keypoints1[tIdx].pt.x;
    int trainY = keypoints1[tIdx].pt.y;
    double queryAngle = keypoints2[qIdx].angle;
    double trainAngle = keypoints1[tIdx].angle;
    int queryScale = keypoints2[qIdx].size;
    int trainScale = keypoints1[tIdx].size;
    // Obtener parámetros transformación
    e = double(trainScale+1)/(queryScale+1);
    theta = (trainAngle - queryAngle)*M_PI/180;
    tx = trainX-e*(queryX*cos(theta)-queryY*sin(theta));
    ty = trainY-e*(queryX*sin(theta)+queryY*cos(theta));

}

// Codigo necesario para RANSAC
int computeConsensus(vector<DMatch> &matches, vector<KeyPoint> &keypoints1, vector<KeyPoint> &keypoints2, vector<int> &selected, double e, double theta, double tx, double ty, double umbralpos)
{
    int cons = 0;
    selected.clear();
    int tIdx, qIdx, queryX, queryY, trainX, trainY;
    double error, errorX, errorY;

    // Recorrer calces y computar error
    for (int i=0;i<matches.size();i++)
    {
        tIdx = matches[i].trainIdx;
        qIdx = matches[i].queryIdx;
        queryX = keypoints2[qIdx].pt.x;
        queryY = keypoints2[qIdx].pt.y;
        trainX = keypoints1[tIdx].pt.x;
        trainY = keypoints1[tIdx].pt.y;
        //Calcular error
        errorX = e*(cos(theta)*trainX-sin(theta)*trainY) + tx - queryX;
        errorY = e*(sin(theta)*trainX+cos(theta)*trainY) + ty - queryY;
        error = sqrt(errorX*errorX+errorY*errorY);
        //Si el error es menor que un umbral, entonces se cuenta
        if (error < umbralpos)
        {
            cons++;
            selected.push_back(i);
        }
    }
    return cons;
}

// Codigo necesario para RANSAC
bool ransac(vector<DMatch> &matches, vector<KeyPoint> &keypoints1, vector<KeyPoint> &keypoints2, vector<DMatch> &accepted, double &e, double &theta, double &tx, double &ty)
{
    double pi = 3.14159265359;

    bool success = false;
    vector<int> selected;
    vector<int> bestSelected;
    // Parametros de Ransac a probar
    double umbralpos = 200;
    int umbralcons = 60;
    int ntrials = 1000;
    int nMatches = matches.size();
    int matchSelected;
    int cons;
    int bestCons = 0;
    double eBest, thetaBest, tyBest, txBest;
    // Iterar ntrials veces
    for (int t=0;t<ntrials; t++)
    {
        // Elegir un calce al azar
        matchSelected = rand() % nMatches;
        // Encontrar los parámetros de la transformación de semejanza
        genTransform(matches[matchSelected], keypoints1, keypoints2, e, theta, tx, ty);
        // Encontrar el consenso
        cons = computeConsensus(matches, keypoints1, keypoints2, selected, e, theta, tx, ty, umbralpos);
        // Ver si el consenso obtenido es superior al mejor encontrado
        if (cons > bestCons)
        {
            bestSelected = selected; // Pasa a ser el mejor consenso
            bestCons = cons; // Se guarda el mejor consenso
            eBest = e;
            thetaBest = theta;
            tyBest = ty;
            txBest = tx;
            //cout << "e: " << e << ", theta: " << theta*180/pi << ", tx: " << tx << ", ty: " << ty << endl;
        }
    }
    cout << "e: " << eBest << ", theta: " << thetaBest*180/pi << ", tx: " << txBest << ", ty: " << tyBest << endl;
    cout << "Best cons: " << bestCons << endl;
    // Ver si el mejor consenso supera el umbral
    if (bestCons > umbralcons)
    {
        for (int i = 0; i < bestSelected.size(); i++)
        {
            accepted.push_back(matches[bestSelected[i]]); // Si es así, entonces se guarda
        }
    }
    return false;
}

