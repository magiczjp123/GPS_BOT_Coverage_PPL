#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#define Pi 3.1415926
using namespace cv;
using namespace std;


int main(){
    /* cells c1;  
    vector<cells> cells_v;
    cells_v.push_back(c1);
    cells_v.push_back(c1);
    cells_v.push_back(c1);
    cells_v.push_back(c1);
    cells_v.push_back(c1);
    RNG rng(12345);
    cells_v[2].color = Scalar(rng.uniform(0,220), rng.uniform(0, 220), rng.uniform(0, 220));

    cout << "color is " << cells_v[2].color << endl;
    Point pt(2,2);
    Rect rt(pt.x,pt.y,5,5); */

    /* vector<int> myvector = {0,1,2,3,4,5,6,7,8,9};
    myvector.erase(myvector.begin()+4);
    std::cout << "myvector contains:";
    for (unsigned i=0; i<myvector.size(); ++i)
        std::cout << ' ' << myvector[i];
    std::cout << '\n';
    return 0; */
    int max = 10;
    // int num = 1;
    int i = 0;
    int j = 0;
    int f = 0;
    vector<int> v_i;
    /* while(i < 10){

        ++i;
        cout << " i is = " << i << endl;
        if(i == 8){
            break;
        }
    } */
    
    // cout << " pt is " << 8-0.5 << endl;
    
    Mat logo = imread("company_logo.jpg", IMREAD_COLOR);
    // namedWindow("logo");
    // imshow("logo", logo);
    cout << " size of logo is " << logo.size() << endl;
    
    Rect r(logo.cols/2. - 236, logo.rows/2. - 204, 236*2,208*2);
    Mat roi(logo,r);

    
    Mat roi_resize;
    resize(roi,roi_resize,Size(9,9));
    
    cout << "logo_roi_resize size is " << roi_resize.size() << endl;
    // imshow("logo resize_ed", roi_resize);
    
    // imwrite("company_logo_small.png", roi_resize);
    // roi_resize = imread("company_logo_small.png");
    imwrite("company_logo_small.png", roi);
    roi = imread("company_logo_small.png");
    Mat logo_trans(roi_resize.rows,roi_resize.cols,CV_8UC4);
    
    Vec3b intensity;
    Vec4b intensity1;
    for(i = 0; i < roi_resize.cols; ++i){
        for(j = 0; j < roi_resize.rows; ++j){
            intensity = roi_resize.at<Vec3b>(i,j);
            /* if(roi_resize.at<Vec4b>(i,j) == Scalar(255,255,255,0)){
                cout << " the pixel is " << i << " " << j << endl;
            } */
            logo_trans.at<Vec4b>(i,j) = Scalar((int)intensity.val[0],(int)intensity.val[1],(int)intensity.val[2],0);
            intensity1 = logo_trans.at<Vec4b>(i,j);
            cout << " the 4 channels are " << (int)intensity1.val[0] << " " << (int)intensity1.val[1] << " " << (int)intensity1.val[2] << " " << intensity1.val[3] << "+++++++" << endl;
            
        }
    }
    logo_trans.copyTo(roi.rowRange(192,192+roi_resize.cols).colRange(54,54+roi_resize.rows));
    
    imshow("company_logo_small", roi_resize);
    
    imshow("logo roi", roi);
    waitKey(0);
    



    return 0;
}






































/* void my_mouse_callback(int event, int x, int y, int flags, void* param);

int main(){
    vector<Point> contour;
    contour.push_back(Point2f(0, 0));
    contour.push_back(Point2f(10, 0));
    contour.push_back(Point2f(10, 10));
    contour.push_back(Point2f(5, 4));
    double area0 = contourArea(contour);
    vector<Point> approx;
    approxPolyDP(contour, approx, 5, true);
    double area1 = contourArea(approx);
    cout << "area0 =" << area0 << endl <<
            "area1 =" << area1 << endl <<
            "approx =" << approx << endl <<
            "approx poly vertices" << approx.size() << endl;


    Mat img = imread("map.png", IMREAD_GRAYSCALE);

    namedWindow("map", WINDOW_AUTOSIZE);
    imshow("map", img);

    // Mouse call back function
    setMouseCallback("map", my_mouse_callback, (void*)&img);

    waitKey(0);
    //cout << getBuildInformation() << endl;
    return 0;
}
// Mouse call back function
void my_mouse_callback(int event, int x, int y, int flags, void* param){
    Mat& image = *(Mat*) param;
    Mat temp = image.clone();
    switch(event){
        case EVENT_MOUSEMOVE: {
            cout << "(" << x << "," << y <<") = " << (unsigned int)image.at<uchar>(y,x) << endl;
            putText(temp, format("(%d,%d)=%d", x, y, (unsigned int)image.at<uchar>(y,x)), Point(x,y), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,255), 2, 8, false);
        }
        imshow("map", temp);
    }
}
 */

/* class cells
{
public:
    int number;
    vector<Point> vertices;
    vector<Point> edge_up;
    vector<Point> edge_lo;
    vector<Point> edge_lt;
    vector<Point> edge_rt;
    vector<int> connected;
    vector<int> branch;


    
};

int main(){
    cells c1;
    c1.number = 1;
    cout << " c1 number is " << c1.number << endl;
    c1.vertices.push_back(Point(2,2));
    cout << " c1 vertices is " << c1.vertices << endl;

    vector<Point> line;
    line.push_back(Point(3,3));
    line.push_back(Point(4,4));
    line.push_back(Point(5,5));

    Mat src;
    src = imread("map.png", IMREAD_GRAYSCALE);

    cout << "the size of line is = " << line.size() << endl;
    cout << "the cols and rows of src " << src.rows << endl;
    circle()
    return 0;
} */

/* #include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

/// Global variables
Mat src, src_gray;
int thresh = 200;
int max_thresh = 255;

char* source_window = "Source image";
char* corners_window = "Corners detected";

/// Function header
void cornerHarris_demo( int, void* );

//  @function main 
int main( int argc, char** argv )
{
  /// Load source image and convert it to gray
  src = imread( argv[1], 1 );
  cvtColor( src, src_gray, CV_BGR2GRAY );

  /// Create a window and a trackbar
  namedWindow( source_window, CV_WINDOW_AUTOSIZE );
  createTrackbar( "Threshold: ", source_window, &thresh, max_thresh, cornerHarris_demo );
  imshow( source_window, src );

  cornerHarris_demo( 0, 0 );

  waitKey(0);
  return(0);
}

//  @function cornerHarris_demo 
void cornerHarris_demo( int, void* )
{

  Mat dst, dst_norm, dst_norm_scaled;
  dst = Mat::zeros( src.size(), CV_32FC1 );

  /// Detector parameters
  int blockSize = 2;
  int apertureSize = 3;
  double k = 0.04;

  /// Detecting corners
  cornerHarris( src_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT );

  /// Normalizing
  normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
  convertScaleAbs( dst_norm, dst_norm_scaled );

  /// Drawing a circle around corners
  for( int j = 0; j < dst_norm.rows ; j++ )
     { for( int i = 0; i < dst_norm.cols; i++ )
          {
            if( (int) dst_norm.at<float>(j,i) > thresh )
              {
               circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
              }
          }
     }
  /// Showing the result
  namedWindow( corners_window, CV_WINDOW_AUTOSIZE );
  imshow( corners_window, dst_norm_scaled );
} */
    // Example program

    /* vector<int> v_i;
    int size_i = 10;
    int i;
    for(i = 0; i < size_i; ++i){
    //cout << " i is " << i << endl;      
        v_i.push_back(i);
    }
    cout << "size v_i is " << v_i[0] << endl;
  //cout << " i final is " << i << endl;



    return 0; */




    /* Mat gx, gy;
    Sobel(contours_img,gx, CV_32F,1,0,1);
    Sobel(contours_img,gy, CV_32F,0,1,1);
    Mat mag, angle;
    cartToPolar(gx,gy,mag,angle,1);

    // for each interest point find the mag and angle
    cout << "size of mag is = " << mag.size() << "size of angle is = " << angle.size() << endl;
    cout << "mag at (498,328) is =" << mag.at<float>(328,498) << " angle at (498,328) is =" << angle.at<float>(328,498) << endl;
    cout << "value at (497,329) is = " << (int)contours_img.at<uchar>(329,497) << endl;
     */
//====================================== ppl previous method ================
/* vector<Point2f> corners_v;
    vector<Point2f> vertices_v;
    goodFeaturesToTrack(masked_img, corners_v, 500, 0.1, 5);
    for(auto it = corners_v.begin(); it != corners_v.end(); ++it){
        // for each interest point print out the mag and angle of gradient
        // cout << "the corner is = " << *it << " mag is " << mag.at<float>(it->y,it->x) << " angle is " << angle.at<float>(it->y,it->x) << endl;
        if( mag.at<float>(it->y,it->x) == 0 && angle.at<float>(it->y,it->x) == 0){
            // circle(masked_img, *it, 5, Scalar(0));
            vertices_v.push_back(*it);
        }
        // circle(contours_img, *it, 5, Scalar(0));
    }
    // cout << "vertices are " << Mat(vertices_v) << endl; */


/*int i;
    for(i = 0; i < masked_img.cols; i++){
        yslice = yslice_gen(i,map_size_y);
        intersect = find_intersection_line(masked_img, yslice);
        if(intersect.size()>1){
            cout << "intersect is " << intersect << endl;
            break;
        }
    }
    if(intersect.size() == 2){
        cells c1;
        c1.number = 0;
        c1.status = "open";
        c1.edge_lo = intersect;
        c1.edge_up = c1.edge_lo;

        cells_v.push_back(c1);
        cout << " cells_v[1] = " << cells_v[0].edge_lo << endl;
    }
    for(i; i < masked_img.cols; i++){
        yslice = yslice_gen(i,map_size_y);
        intersect = find_intersection_line(masked_img, yslice);
        vector<vector<Point> > edge;

        if(intersect.size() % 2 == 0 && intersect.size()>0){
            // form edge vector based on intersection information
            for(int j = 0; j < intersect.size(); j++){
                if(j % 2 == 0){
                    vector<Point> edge_current;
                    edge_current.push_back(intersect[j]);
                    edge_current.push_back(intersect[j+1]);
                    edge.push_back(edge_current);
                }
            }
            //================= printout edge ========
            cout << "intersect is " << intersect << endl;
            for(int j = 0; j < edge.size(); ++j){
                cout << "edge [" << j << "] is " << edge[j] << endl;
            }
            
            
            
            // for(int j = 0; j < vertices_v.size(); ++j){
                // if(ismember(vertices_v[j], intersect)){
                    // cout << "the vertices is " << vertices_v[j] << endl;
                // }
            // }
            
            if(intersect.size()>2 && connectivity_change == 0){
                //connectivity change
                cells_v[0].status = "close";
                // form two new cells;
                cells c1;
                cells c2;
                c1.status = "open";
                c1.connected.push_back(0);
                c1.number = cells_v.size();
                c1.edge_lo = edge[0];
                c1.edge_up = c1.edge_lo;
                c1.edge_lt.push_back(c1.edge_lo[0]);
                c1.edge_rt.push_back(c1.edge_lo[1]);
                //=============================
                c2.status = "open";
                c2.connected.push_back(0);
                c2.number = cells_v.size()+1;
                c2.edge_lo = edge[1];
                c2.edge_up = c2.edge_lo;
                c2.edge_lt.push_back(c2.edge_lo[0]);
                c2.edge_rt.push_back(c2.edge_lo[1]);
                //==============================
                cells_v.push_back(c1);
                cells_v.push_back(c2);
                
                connectivity_change = 1;
            }
            if(intersect.size() == 2 && connectivity_change == 1){
                for(int j = 0; j < cells_v.size(); ++j){
                    if(cells_v[j].status == "open"){
                        cells_v[j].status = "close";
                    }
                }
                cells c1;
                c1.status = "open";
                // c1.connected = 
                c1.number = cells_v.size();
                c1.edge_lo = edge[0];
                c1.edge_up = c1.edge_lo;
                c1.edge_lt.push_back(c1.edge_lo[0]);
                c1.edge_rt.push_back(c1.edge_lo[1]);
                cells_v.push_back(c1);
                connectivity_change = 0;
            }
            // reorder open cells for edge adding
            vector<int> open_cell_num;
            for(int j = 0; j < cells_v.size(); ++j){
                if(cells_v[j].status == "open"){
                    open_cell_num.push_back(j);
                }
            }
            cout << "size open_cell_num is " << open_cell_num.size() << endl;
            if(open_cell_num.size()>1){
                for(int j = 0; j < open_cell_num.size() - 1; ++j){
                    
                    if(cells_v[open_cell_num[j]].edge_up[0].y > cells_v[open_cell_num[j+1]].edge_up[0].y){
                        int temp;
                        temp = open_cell_num[j];
                        open_cell_num[j] = open_cell_num[j+1];
                        open_cell_num[j+1] = temp;
                    }
                }
                for(int j = 0; j < open_cell_num.size(); ++j){
                    int index = open_cell_num[j];
                    cells_v[index].edge_up = edge[j];
                    cells_v[index].edge_lt.push_back(edge[j][0]);
                    cells_v[index].edge_rt.push_back(edge[j][1]);
                    
                }
            }else if(open_cell_num.size() == 1){
                cells_v[open_cell_num[0]].edge_up = edge[0];
                cells_v[open_cell_num[0]].edge_lt.push_back(edge[0][0]);
                cells_v[open_cell_num[0]].edge_rt.push_back(edge[0][1]);
                
            }





        }else{
            cout << " Intersection calculation error " << endl;
        }
    }
    cout << "size of cells_v is " << cells_v.size() << endl;
    RNG rng(12345);
    for(int j = 0; j < cells_v.size(); ++j){
        Scalar color = Scalar(rng.uniform(0,255));
        for(int k = 0; k < cells_v[j].edge_lo.size(); ++k){
            circle(src,cells_v[j].edge_lo[k],2,color,-1,8,0);
        }
        for(int k = 0; k < cells_v[j].edge_up.size(); ++k){
            circle(src,cells_v[j].edge_up[k],2,color,-1,8,0);
        }
        for(int k = 0; k < cells_v[j].edge_lt.size(); ++k){
            circle(src,cells_v[j].edge_lt[k],2,color,-1,8,0);
        }
        for(int k = 0; k < cells_v[j].edge_rt.size(); ++k){
            circle(src,cells_v[j].edge_rt[k],2,color,-1,8,0);
        }
        cout << "the " << j << "th cell" << endl;
        cout << "the edge_lo is " << cells_v[j].edge_lo << endl;
        cout << "the edge_up is " << cells_v[j].edge_up << endl;
        cout << "the edge_lt is " << cells_v[j].edge_lt << endl;
        cout << "the edge_rt is " << cells_v[j].edge_rt << endl;
        

    }
    //cout << "is a member " << ismember(intersect[0],intersect);
    for(auto it = corners_v.begin(); it != corners_v.end(); ++it){
        // for each interest point print out the mag and angle of gradient
        // cout << "the corner is = " << *it << " mag is " << mag.at<float>(it->y,it->x) << " angle is " << angle.at<float>(it->y,it->x) << endl;
        if( mag.at<float>(it->y,it->x) == 0 && angle.at<float>(it->y,it->x) == 0){
            circle(masked_img, *it, 5, Scalar(0));
            vertices_v.push_back(*it);
        }
        // circle(contours_img, *it, 5, Scalar(0));
    }*/
    /* int count = 0;
    for (int i = 0; i < masked_img.cols; ++i){
        yslice = yslice_gen(i,map_size_y);
        intersect = find_intersection_line(masked_img,yslice);
        if(intersect.size()%2 != 0){
            ++count;
        }
    } */
    /* yslice = yslice_gen(488,map_size_y);
    intersect = find_intersection_line(masked_img,yslice);
    
    
    cout << " the intersection is " << intersect << endl;
    cout << " the size of intersection is " << intersect.size() << endl; */
    // cout << " masked_img(54,32) is " << (int)masked_img(241,104) << endl;


    //=========================================
    /* int ismember(Point2f pt, vector<Point> yslice){
        for(int i = 0; i < yslice.size(); ++i){
            if((int)pt.x == yslice[i].x && (int)pt.y == yslice[i].y){
                return 1;
            }else if((int)pt.x-1 == yslice[i].x && (int)pt.y == yslice[i].y){
                return 1;
            }else if((int)pt.x+1 == yslice[i].x && (int)pt.y == yslice[i].y){
                return 1;
            }else if((int)pt.x == yslice[i].x && (int)pt.y-1 == yslice[i].y){
                return 1;
            }else if((int)pt.x+1 == yslice[i].x && (int)pt.y+1 == yslice[i].y){
                return 1;
            }
        }
        return 0;
    } */
//============================== New Method ==================================
/* vector<Point> yslice;
    vector<Point> intersect;   
    vector<cells> cells_v;
    // push first cell
    cout << "*************" << endl;
    cout << " Point_check(134,260) and (135,269) is " << point_check(Point(134,160),Point(135,166)) << endl;
    cout << " ++++++++++++" << endl;
    int i;
    // ===================== check for first pairs intersect to show ====
    for(i = 0; i < masked_img.cols; ++i){
        yslice = yslice_gen(i,map_size_y);
        intersect = find_intersection_line(masked_img, yslice);
        if(intersect.size()>1){
            cout << "intersect is " << intersect << endl;
            cout << " i is " << i << endl;
            break;
        }
    }
    // ===================== produce cell from first pairs intersect =====
    if(intersect.size() == 2){
        cells c1;
        c1.number = 0;
        c1.status = "open";
        c1.edge_lo = intersect;
        c1.edge_up = c1.edge_lo;
        c1.edge_lt.push_back(c1.edge_up[0]);
        c1.edge_rt.push_back(c1.edge_up[1]);
    }else if(intersect.size() > 2){
        cells c1;
        vector<Point> edge_2p;
        for(int j = 0; j < intersect.size(); ++j){
            if(j%2 == 0){
                c1.number = cells_v.size();
                c1.status = "open";
                edge_2p.push_back(intersect[j]);
                edge_2p.push_back(intersect[j+1]);
                c1.edge_lo = edge_2p;
                c1.edge_up = c1.edge_lo;
                c1.edge_lt.push_back(c1.edge_up[0]);
                c1.edge_rt.push_back(c1.edge_up[1]);
                cells_v.push_back(c1);     
            }
        }
    }
    for(i; i < masked_img.cols; ++i){
        yslice = yslice_gen(i,map_size_y);
        intersect = find_intersection_line(masked_img, yslice);
        for(int j = 0; j < cells_v.size(); ++j){
            if(cells_v[j].status == "open"){
                int index_i = match_intersect(cells_v[j],intersect);
                if(index_i != -1){
                    vector<Point> edge_2p;
                    edge_2p.push_back(intersect[index_i]);
                    edge_2p.push_back(intersect[index_i+1]);
                    
                    cells_v[j].edge_up = edge_2p;
                    cells_v[j].edge_lt.push_back(cells_v[j].edge_up[0]);
                    cells_v[j].edge_rt.push_back(cells_v[j].edge_up[1]);
                    intersect.erase(intersect.begin()+index_i,intersect.begin()+index_i+2);
                }else if(index_i == -1){
                    cells_v[j].status = "close";
                }
            }
        }
        if(intersect.size()>1){
            for(int j = 0; j < intersect.size(); j = j + 2){
                cells c1;
                vector<Point> edge_2p;
                c1.number = cells_v.size();
                c1.status = "open";
                edge_2p.push_back(intersect[j]);
                edge_2p.push_back(intersect[j+1]);
                c1.edge_lo = edge_2p;
                c1.edge_up = c1.edge_lo;
                c1.edge_lt.push_back(c1.edge_up[0]);
                c1.edge_rt.push_back(c1.edge_up[1]);
                cells_v.push_back(c1);
            }
        }
        
    } */



    //********************** Old ppl algorithm ******************************************
            /* int finish = 0;
            int count = 20;
            
            // generate path for downward movement
            while(finish == 0){
            // while(count > 0){
                //============================ move down ===================
                // while(robot_move_down() != 0){};
                int move_down_R = 1;
                // int move_down_L = 1;
                
                int try_count = 4;
                while(move_down_R == 1){
                    if(robot_move_down() == 0){
                        for(i = 0; i < 10; ++i){
                            robot.path.push_back(Point(robot.path.back().x+i,robot.path.back().y));
                            if(robot_move_down() == 0){
                                robot.path.pop_back();
                            }else if(robot_move_down() != 0){
                                // robot.path.erase(robot.path.end()-2);
                                Point tmp(robot.path.back().x,robot.path.back().y);
                                robot.path.pop_back();
                                robot.path.pop_back();
                                robot.path.push_back(tmp);
                                try_count = try_count - i;
                                break;
                            }
                        }
                        if(try_count < 0 || i == 10){
                            move_down_R = 0;
                        }
                        
                    }
                    for(i = 0; i < it->edge_rt.size(); ++i){
                        if(it->edge_rt[i].x == robot.path.back().x){
                            break;
                        }
                    }
                    if(robot.path.back().y >= it->edge_rt[i].y - robot_size){
                        Point tmp(robot.path.back().x, it->edge_rt[i].y - robot_size);
                        robot.path.pop_back();
                        robot.path.push_back(tmp);
                        break;
                    }
                }
                --count;
                //============================ finish count ===================
                if(robot.path.back().x >= it->edge_lt.back().x-robot_size || robot.path.back().x < it->edge_lt.front().x){
                    finish = 1;
                    cout << " finish is " << finish << endl;
                    break;
                }
                //============================ move right ===================
                int move_right_U =1;
                int move_right_D =1;
                int move_count = 9;
                int robot_move_right_result;
                // while(move_right != 10 || robot.path.back().x < it->edge_lt.x-10){
                while((move_right_U ==1 || move_right_D == 1) ){        
                    
                    
                    robot_move_right_result = robot_move_right();
                    move_count = move_count - robot_move_right_result;
                    if(move_count <= 0){
                        Point tmp(robot.path.back().x+move_count,robot.path.back().y);
                        robot.path.pop_back();
                        robot.path.push_back(tmp);
                        break;
                    }
                    if(move_count > 0){
                        for(i = 0; i < 10; ++i){
                            robot.path.push_back(Point(robot.path.back().x,robot.path.back().y+i));
                            robot_move_right_result = robot_move_right();
                            move_count = move_count - robot_move_right_result;
                            if(robot_move_right_result == 0){
                                robot.path.pop_back();
                            }else if(robot_move_right_result != 0){
                                if(move_count <= 0){
                                    Point tmp(robot.path.back().x+move_count,robot.path.back().y);
                                    robot.path.pop_back();
                                    robot.path.pop_back();
                                    robot.path.push_back(tmp);
                                    break;
                                }
                            }
                            
                        }
                        if(i == 10){
                            move_right_D = 0;
                        }
                        
                        for(i = 0; i < 10; ++i){
                            robot.path.push_back(Point(robot.path.back().x,robot.path.back().y-i));
                            robot_move_right_result = robot_move_right();
                            move_count = move_count - robot_move_right_result;
                            if(robot_move_right_result == 0){
                                robot.path.pop_back();
                            }else if(robot_move_right_result != 0){
                                if(move_count <= 0){
                                    Point tmp(robot.path.back().x+move_count,robot.path.back().y);
                                    robot.path.pop_back();
                                    robot.path.pop_back();
                                    robot.path.push_back(tmp);
                                    break;
                                }
                            }
                            
                        }
                        if(i == 10){
                            move_right_U = 0;
                        }
                    }
                }
                if(move_count == 9){
                    finish = 1;
                    cout << " finish is " << finish << endl;
                    break;
                }
                
                --count;
                
                //============================ move up ===================
                int move_up_R = 1;
                int move_up_L = 1;
                try_count = 4;
                
                while(move_up_R == 1){
                    if(robot_move_up() == 0){
                        for(i = 0; i < 10; ++i){
                            robot.path.push_back(Point(robot.path.back().x+i,robot.path.back().y));
                            if(robot_move_up() == 0){
                                robot.path.pop_back();
                            }else if(robot_move_up() != 0){
                                // robot.path.erase(robot.path.end()-2);
                                Point tmp(robot.path.back().x,robot.path.back().y);
                                robot.path.pop_back();
                                robot.path.pop_back();
                                robot.path.push_back(tmp);
                                try_count = try_count - i;
                                break;
                            }
                        }
                        if(try_count < 0 || i == 10){
                            move_up_R = 0;
                        }
                        
                    }
                    for(i = 0; i < it->edge_lt.size(); ++i){
                        if(it->edge_lt[i].x == robot.path.back().x){
                            break;
                        }
                    }
                    if(robot.path.back().y <= it->edge_lt[i].y){
                        Point tmp(robot.path.back().x, it->edge_lt[i].y);
                        robot.path.pop_back();
                        robot.path.push_back(tmp);
                        break;
                    }
                }

                //============================ finish count ===================
                if(robot.path.back().x >= it->edge_lt.back().x-robot_size*2 || robot.path.back().x < it->edge_lt.front().x){
                    finish = 1;
                    break;
                }
                --count;
                //============================ move right ===================
                move_right_U =1;
                move_right_D =1;
                move_count = 9;
                // while(move_right != 10 || robot.path.back().x < it->edge_lt.x-10){
                while((move_right_U ==1 || move_right_D == 1) ){        
                    
                    
                    robot_move_right_result = robot_move_right();
                    move_count = move_count - robot_move_right_result;
                    if(move_count <= 0){
                        Point tmp(robot.path.back().x+move_count,robot.path.back().y);
                        robot.path.pop_back();
                        robot.path.push_back(tmp);
                        break;
                    }
                    if(move_count > 0){
                        for(i = 0; i < 10; ++i){
                            robot.path.push_back(Point(robot.path.back().x,robot.path.back().y-i));
                            robot_move_right_result = robot_move_right();
                            move_count = move_count - robot_move_right_result;
                            if(robot_move_right_result == 0){
                                robot.path.pop_back();
                            }else if(robot_move_right_result != 0){
                                if(move_count <= 0){
                                    Point tmp(robot.path.back().x+move_count,robot.path.back().y);
                                    robot.path.pop_back();
                                    robot.path.pop_back();
                                    robot.path.push_back(tmp);
                                    break;
                                }
                            }
                        }
                        if(i == 10){
                            move_right_U = 0;
                        }
                        for(i = 0; i < 10; ++i){
                            robot.path.push_back(Point(robot.path.back().x,robot.path.back().y+i));
                            robot_move_right_result = robot_move_right();
                            move_count = move_count - robot_move_right_result;
                            if(robot_move_right_result == 0){
                                robot.path.pop_back();
                            }else if(robot_move_right_result != 0){
                                if(move_count <= 0){
                                    Point tmp(robot.path.back().x+move_count,robot.path.back().y);
                                    robot.path.pop_back();
                                    robot.path.pop_back();
                                    robot.path.push_back(tmp);
                                    break;
                                }
                            }
                        }
                        if(i == 10){
                            move_right_D = 0;
                        }
                    }
                }
                if(move_count == 9){
                    finish = 1;
                    cout << " finish is " << finish << endl;
                    break;
                }
                
                --count; */
         
                         
              
/* int robot_move_up(){
    int flag = 1;
    int i;
    int j;
    for(i = 0; i < robot_size; ++i){
        for(j = 0; j < robot_size; ++j){
            if((int)masked_img.at<uchar>(robot.path.back().y-robot_size+i, robot.path.back().x+j) != 255){
                flag = 0;
            }
        }
    }
    if(flag == 1){
        robot.path.push_back(Point(robot.path.back().x ,robot.path.back().y-robot_size));
        robot.update_pose(robot.path.back());
        return robot_size;
    }
    if(flag == 0){
        flag = 1;
        int count = robot_size;
        while(count>0){
            for(i = 0; i < robot_size; ++i){
                for(j = 0; j < robot_size; ++j){
                    if((int)masked_img.at<uchar>(robot.path.back().y-count+i, robot.path.back().x+j) != 255){
                        flag = 0;
                    }
                }
            }
            if(flag == 1){
                robot.path.push_back(Point(robot.path.back().x ,robot.path.back().y-count));
                robot.update_pose(robot.path.back());
                return count;
            }
            flag = 1;
            --count;
        }
    }

    return 0;
}
int robot_move_right(){
    int flag = 1;
    int i;
    int j;
    for(i = 0; i < robot_size; ++i){
        for(j = 0; j < robot_size; ++j){
            if((int)masked_img.at<uchar>(robot.path.back().y+i, robot.path.back().x+robot_size+j) != 255){
                flag = 0;
            }
        }
    }
    if(flag == 1){
        robot.path.push_back(Point(robot.path.back().x+robot_size ,robot.path.back().y));
        robot.update_pose(robot.path.back());
        return robot_size;
    }
    if(flag == 0){
        flag = 1;
        int count = robot_size;
        while(count>0){
            for(i = 0; i < robot_size; ++i){
                for(j = 0; j < robot_size; ++j){
                    if((int)masked_img.at<uchar>(robot.path.back().y+i, robot.path.back().x+count+j) != 255){
                        flag = 0;
                    }
                }
            }
            if(flag == 1){
                robot.path.push_back(Point(robot.path.back().x+count ,robot.path.back().y));
                robot.update_pose(robot.path.back());
                return count;
            }
            flag = 1;
            --count;
        }
    }

    return 0;
}
int robot_move_down(){
    int flag = 1;
    int i;
    int j;
    for(i = 0; i < robot_size; ++i){
        for(j = 0; j < robot_size; ++j){
            if((int)masked_img.at<uchar>(robot.path.back().y+robot_size+i, robot.path.back().x+j) != 255){
                flag = 0;
            }
        }
    }
    if(flag == 1){
        robot.path.push_back(Point(robot.path.back().x ,robot.path.back().y+robot_size));
        robot.update_pose(robot.path.back());
        return robot_size;
    }
    if(flag == 0){
        flag = 1;
        int count = robot_size;
        while(count>0){
            for(i = 0; i < robot_size; ++i){
                for(j = 0; j < robot_size; ++j){
                    if((int)masked_img.at<uchar>(robot.path.back().y+count+i, robot.path.back().x+j) != 255){
                        flag = 0;
                    }
                }
            }
            if(flag == 1){
                robot.path.push_back(Point(robot.path.back().x ,robot.path.back().y+count));
                robot.update_pose(robot.path.back());
                return count;
            }
            flag = 1;
            --count;
        }
    }

    return 0;
} */