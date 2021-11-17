#include <murAPI.hpp>

int main() {
    cv::imshow("Bin", mur.getCameraTwoFrame());
    /*
    int hMin = 0;
    int hMax = 255;

    int sMin = 0;
    int sMax = 255;

    int vMin = 0;
    int vMax = 255;
    */
    int hMin = 0;
    int hMax = 255;

    int sMin = 0;
    int sMax = 255;

    int vMin = 0;
    int vMax = 255;
    
    int chMax = 255;
    
    
    char hMinName[50];
    char hMaxName[50];
    
    char sMinName[50];
    char sMaxName[50];
    
    char vMinName[50];
    char vMaxName[50];
    
    std::sprintf(hMinName, "H min", hMin);
    std::sprintf(hMaxName, "H max", hMax);
    
    std::sprintf(sMinName, "S min", sMin);
    std::sprintf(sMaxName, "S max", sMax);
    
    std::sprintf(vMinName, "V min", vMin);
    std::sprintf(vMaxName, "V max", vMax);

    cv::createTrackbar(hMinName, "Bin", &hMin, chMax);
    cv::createTrackbar(hMaxName, "Bin", &hMax, chMax);

    cv::createTrackbar(sMinName, "Bin", &sMin, chMax);
    cv::createTrackbar(sMaxName, "Bin", &sMax, chMax);

    cv::createTrackbar(vMinName, "Bin", &vMin, chMax);
    cv::createTrackbar(vMaxName, "Bin", &vMax, chMax);

    while (true) {
        cv::Mat image = mur.getCameraTwoFrame();
        cv::imshow("Image", image);
        cv::cvtColor(image, image, CV_BGR2HSV);
        cv::Scalar lower(hMin, sMin, vMin);
        cv::Scalar upper(hMax, sMax, vMax);
        cv::inRange(image, lower, upper, image);
        cv::imshow("Bin", image);
        cv::waitKey(1);
    }
}