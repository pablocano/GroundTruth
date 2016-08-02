#include "Regions.h"
#include <opencv2/imgproc/imgproc.hpp>

void Regions::draw(cv::Mat &image) const
{
    for (auto& region : regions) {
        cv::Point pt1 = cv::Point(region.right.y, region.right.x);
        cv::Point pt2 = cv::Point(region.left.y, region.left.x);
        cv::Scalar paint(128,128,128);
        if (region.color.is(white)) {
            paint = cv::Scalar(255,255,255);
        }
        else if (region.color.is(green)) {
            paint = cv::Scalar(0,255,0);
        }
        else if (region.color.is(blue)) {
            paint = cv::Scalar(255,0,0);
        }
        else if (region.color.is(red)) {
            paint = cv::Scalar(0,0,255);
        }
        else if (region.color.is(orange)) {
            paint = cv::Scalar(0,128,255);
        }
        else if (region.color.is(yellow)) {
            paint = cv::Scalar(0,255,255);
        }
        else if (region.color.is(black)) {
            paint = cv::Scalar(0,0,0);
        }
        cv::line(image, pt1, pt2, paint);
    }
}
