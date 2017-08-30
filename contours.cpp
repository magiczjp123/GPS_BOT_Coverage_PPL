// to compile g++ -std=c++11 contours.cpp `pkg-config --libs --cflags opencv` -o contours

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
using namespace cv;
using namespace std;
Mat src; Mat src_gray;
int thresh = 80;
int max_thresh = 255;
RNG rng(12345);
void thresh_callback(int, void* );
//void image_resize(int, void*);
void cell_decompose(Mat map, vector<vector<Point>> contours);

int main( int, char** argv )
{
  src = imread("map.png", IMREAD_COLOR);
  // src = imread("fb_map.jpg", IMREAD_COLOR);
  if (src.empty())
  {
    cerr << "No image supplied ..." << endl;
    return -1;
  }
  cvtColor( src, src_gray, COLOR_BGR2GRAY );
  blur( src_gray, src_gray, Size(3,3) );
  const char* source_window = "Source";
  namedWindow( source_window, WINDOW_AUTOSIZE );
  imshow( source_window, src );
  createTrackbar( " Canny thresh:", "Source", &thresh, max_thresh, thresh_callback );
  thresh_callback( 0, 0 );
/*   createTrackbar( " Image resize", "Source", &thresh, max_thresh, image_resize );
  image_resize(0,0); */
  waitKey(0);
  return(0);
}
void thresh_callback(int, void* )
{
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  Canny( src_gray, canny_output, thresh, thresh*2, 3 );
  findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  for( size_t i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, (int)i, color, 0.5, 8, hierarchy, 0, Point() );
     }
  
  vector<Point> img_contours;

  /* for(int i = 0; i < canny_img.rows; ++i){
      for(int j = 0; j < canny_img.cols; ++j){
          Point pt(j,i);
          if((int)canny_img.at<uchar>(pt.x,pt.y) == 255){
              //cout << "point at location is " << (int)canny_img.at<uchar>(pt.x,pt.y) << endl;
              img_contours.push_back(pt);
          }
      }
  } */
  for(auto it = contours.begin(); it != contours.end(); ++it){
    for(auto it2 = it->begin(); it2 != it->end(); ++it2){
        // ocuppied_map.at<uchar>(it2->y,it2->x) = 0;
      img_contours.push_back(*it2);
    }
  }
  RotatedRect minRect;
  Point2f rect_points[4];
  minRect = minAreaRect(Mat(img_contours));
  minRect.points(rect_points);
  for(int j = 0; j < 4; j++){
    line( drawing, rect_points[j], rect_points[(j+1)%4], Scalar(0,0,255),1,8);
  }
  cout << "the 4 vertices are = " << rect_points[0] 
            << rect_points[1]
            << rect_points[2]
            << rect_points[3]
            << endl;
    
  namedWindow( "Contours", WINDOW_AUTOSIZE );
  imshow( "Contours", drawing );
  cout << "the size of contour is = " << contours.size() << endl;
//   cout << "the [20] of contours is = " << contours[20] << endl;
//   cout << "the size of hierarchy is = " << hierarchy.size() << endl;
  Mat independent_array(src_gray.rows,src_gray.cols, CV_8UC1, 255);
  //independent_array.create(src_gray.size(), src_gray.type());
  int j = 0;
  vector<Point> pt_array;
  /* for(auto it = contours.begin(); it != contours.end(); ++it ){
      cout << "the "<< j << " th element of contous is = "<< *it << endl;
      pt_array = *it;
      //cout << "the 3 th element of it is = " << src.at<Vec3b>(pt_array[2]) << endl;
      //cout << "Independenct Contours at it is = " << independent_array.at<Vec3b>((*it)[2]) << endl;
      ++j;
  } */
  pt_array = *contours.begin();
  pt_array = contours[0];
  Point2i pt;
  pt = *pt_array.begin();
  for(auto it = pt_array.begin(); it != pt_array.end(); ++it){
    independent_array.at<uchar>((*it).y,(*it).x) = 255;
  }
  cout << "contours is = " << contours[0] << endl;
  cout << "the size of src_gray is = " << src_gray.size() << endl;
  cout << "the src_gray image at point is = " << int(src_gray.at<uchar>(pt.y,pt.x)) << endl;
  //src.at<Vec3b>(pt.y,pt.x) = (255,255,255);
  cout << "the src image at point is = " << src.at<Vec3b>(pt.y,pt.x) << endl;
  cout << "the channels of src and src_gray image is = " << src.channels() << " and " << src_gray.channels() << endl;
  drawContours(independent_array, contours, -1, Scalar(120), 2, 8, hierarchy, 1, Point() );
  //drawContours( drawing, contours, (int)i, color, 2, 8, hierarchy, 0, Point() );
  /* for(auto it = independent_array.begin(); it != independent_array.end(); ++it){
    independent_array.at<Vec3b>((*it)) = (255,255,255);

  } */
  /* int size_min = (*contours.begin()).size();
  auto contours_p = contours.begin();
  for(auto it = contours.begin(); it != contours.end()-1; ++it){
      //cout << " type is " << it.type() << endl;
      if(it->size() < (it+1)->size()){
          size_min = (it+1)->size();
          contours_p = it+1;
      }
  }
  cout << "size_max is = " << size_min << " it is = " << *contours_p << endl; */

  vector<Point2f> corners_v;
  goodFeaturesToTrack(independent_array, corners_v, 500, 0.1, 4);
  for(auto it = corners_v.begin(); it != corners_v.end(); ++it){
    circle(independent_array, *it, 5, Scalar(0));
  }
  cout << " the corners are = " << Mat(corners_v) << endl;
  /* for(auto it = pt_array.begin(); it != pt_array.end(); ++it){
    independent_array.at<Vec3b>((*it)) = (0,0,0);

  } */
  
  namedWindow( "Independent Contours show", WINDOW_AUTOSIZE);
  imshow( "Independent Contours show", independent_array);
  imwrite("independent_img_gray.jpg", independent_array);
}

void cell_decompose(Mat map, vector<vector<Point>> contours){
  // init cell edge
  


}