#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
using std::string;
using namespace cv;
using namespace std;

Mat rotate_img(Mat src, double angle){
    Mat dst;
    Point2f pt(src.cols/2., src.rows/2.);
    Mat r = getRotationMatrix2D(pt, angle,1.0);
    warpAffine(src,dst,r,Size(src.cols,src.rows));
    return dst;
}
int main(){
    Mat src = imread("full.png");
    Mat dst,dst_gray,dst_threshold;
    dst = rotate_img(src,-11);
    cvtColor( dst, dst_gray, CV_BGR2GRAY );

    threshold(dst_gray,dst_threshold,210,255,0);

    imshow("src", src);
    imshow("dst", dst_threshold);
    waitKey(0);
    return 0;
}