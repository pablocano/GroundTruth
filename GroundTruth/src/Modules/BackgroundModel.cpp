/**
 * @file BackgroundModel.cpp
 * Implementacion de la clase de modelo de fondo.
 * @author Pablo Cano Montecinos.
 */

#include "BackgroundModel.h"
#include <opencv2/imgproc/imgproc.hpp>

MAKE_MODULE(BackgroundModel, GroundTruth)

void BackgroundModel::update(MovementImage& movementImage)
{
    if (count < goal)
    {
        count++;
        alpha = 1/count;
        std::cout<< "cuenta: " << count << std::endl;
    }
    else
    {
        alpha = 0.0005f;
    }
    if(theCameraInfo.type == CameraInfo::eastCam)
        currentModel = &model1;
    else
        currentModel = &model2;
    cv::cvtColor(theImageBGR,frame,CV_BGR2GRAY);
    frame.convertTo(frame, CV_32F);
    frame = frame/255.f;
    if(currentModel->empty())
    {
        frame.copyTo(*currentModel);
        cv::Mat empty = cv::Mat::zeros(currentModel->size(),CV_32F);
        movementImage = empty;
        return;
    }
    for (unsigned i = 0; i < currentModel->rows; i++) {
        for (unsigned j = 0; j < currentModel->cols; j++) {
            if (!movementImage.at<u_int8_t>(i,j)) {
                updatePixel(i, j,alpha);
            }
            else
            {
                updatePixel(i, j,beta + limit);
            }
        }
    }
    beta = beta - beta*gamma;

    cv::Mat empty = abs(frame - *currentModel) > thrld;
    movementImage = empty;
}

void BackgroundModel::updatePixel(int i, int j,float alpha)
{
    currentModel->at<float>(i, j) = (1.f - alpha)*currentModel->at<float>(i, j) + alpha*frame.at<float>(i,j);
}
