#include "Transformation.h"
#include "Representations/CameraInfo.h"

Vector2<> Transformation::imageToField(const Vector2<> &positionInImage, const CameraInfo& cameraInfo)
{
    // rotate the camera info parameters
    Vector2<> opticalCenter(cameraInfo.opticalCenter.y, cameraInfo.opticalCenter.x);
    float invFx = cameraInfo.invFy;
    float invFy = cameraInfo.invFx;

    // First compute the difference between the point in image and the field center in image
    Vector2<> res = (positionInImage - cameraInfo.fieldCenterInImage);
    // Then we multiply by the inverse of the Camera Calibration Matrix (K) to project the points in world coordinates up to a scale factor
    //res.x = invFx * res.x;
    //res.y = invFy * res.y;
    // Finally we scale the unscaled point with the previously computed piwel to millimiter factor
    res *= cameraInfo.pix2World;
    return res;
}