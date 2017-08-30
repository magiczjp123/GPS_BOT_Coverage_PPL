#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;

Mat src;
Mat src_gray;
int Thresh = 80;
int Thresh_max = 255;
RNG rng(12345);
Mat canny_mat;
Mat drawing;
vector<vector<Point>> contours;
vector<Vec4i> hierarchy;
/* void on_ThreshChange(int, void*);
void on_contoursChange(int c_thresh, void*); */


int main(int argc, char** argv){
    cout << "argc = " << argc << endl;
    if (argc != 1){
        src = imread(argv[1], IMREAD_GRAYSCALE);    
    }else if (argc == 1){
        cout << "Image read default" << endl;
        src = imread("map.png", IMREAD_GRAYSCALE);
    }
    //Mat src = imread("fb_map.jpg", IMREAD_GRAYSCALE);
    if (!src.data){
        return -1;
        cout << "image read failed" << endl;
    }


    //cout << "src_gray size is " << src.data << endl;
    src_gray = src;
    
    // Reduce noise
    blur(src_gray, src_gray, Size(3,3));
    namedWindow("Source Image", WINDOW_AUTOSIZE);
    imshow("Source Image", src);
    /* // Track bar
    createTrackbar("Canny level", "Source Image", &Thresh, Thresh_max, on_ThreshChange );
    on_ThreshChange(0,0);   */
    
    Canny(src_gray, canny_mat,Thresh, Thresh*2,3);
    findContours(canny_mat, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0,0));
    drawing.create(src_gray.rows,src_gray.cols, CV_8UC1);
    drawing = Scalar(255);
    cout << "the size of contour is " << contours.size() << endl;
    //cout << "Scalar(0) is " << Scalar(0) << endl;
    cout << "the size of hierarchy is " << hierarchy.size() << endl;
    drawContours(drawing, contours, -1, Scalar(0), 2, 8, hierarchy, 1, Point());
    int size_min = (*contours.begin()).size();
    auto contours_p = contours.begin();
    for(auto it = contours.begin(); it != contours.end()-1; ++it){
        //cout << " type is " << it.type() << endl;
        if(it->size() < (it+1)->size()){
            size_min = (it+1)->size();
            contours_p = it+1;
        }
    }
    cout << "size_min is = " << size_min << " it is = " << *contours_p << endl;
    namedWindow("Contours", WINDOW_AUTOSIZE);
    imshow("Contours", drawing);
    imwrite("map_contours.jpg", drawing);
    //waitKey(0);
    
    Mat ocuppied_map(src.rows, src.cols,CV_8UC1);
    ocuppied_map = (uchar)255;


    
    /* for(int i = 0; i < contours.size(); i++){
        for(int j = 0; j < contours[i].size(); j++){
            ocuppied_map(contours[i][j].x,contours[i][j].y) = 0;
        }
    } */
    for(auto it = contours.begin(); it != contours.end(); ++it){
        for(auto it2 = it->begin(); it2 != it->end(); ++it2){
            ocuppied_map.at<uchar>(it2->y,it2->x) = 0;
        }
    }

    
    cout << "the 200,300 is " << (int)ocuppied_map.at<uchar>(218,260) << endl;
    namedWindow("contours_plain", WINDOW_AUTOSIZE);
    imshow("contours_plain", Mat(ocuppied_map));
    //imwrite("map_contours.jpg", ocuppied_map);
    waitKey(0);
    return 0;
}
/*void on_ThreshChange(int, void*){
    Canny(src_gray, canny_mat, Thresh, Thresh*2, 3);
    findContours(canny_mat, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0,0));
    drawing = Mat::zeros(canny_mat.size(), CV_8UC3);
    cout << "the size of contours is =" << contours.size() << endl;
    for( int i = 0; i < contours.size();i++){
        Scalar color = Scalar( rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
        drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
    }
    namedWindow("Contours", WINDOW_AUTOSIZE);
    imshow("Contours", drawing);
    int c_thresh = contours.size();
    int c_thresh_max = contours.size();
    
    createTrackbar("Contours Index", "Contours", &c_thresh, c_thresh_max, on_contoursChange );
    //on_contoursChange(c_thresh,0);
}
void on_contoursChange(int c_thresh, void*){
    drawing = Mat::zeros(canny_mat.size(), CV_8UC3);
    for( int i = 0; i < c_thresh; i++){
        Scalar color = Scalar( rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
        drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
    } 
    Scalar color = Scalar( rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
    drawContours(drawing, contours, c_thresh-1, color, 2, 8, hierarchy, 0, Point());
    imshow("Contours", drawing);
}*/