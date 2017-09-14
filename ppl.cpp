// to complie g++ -std=c++11 ppl.cpp `pkg-config --libs --cflags opencv` -o ppl
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#define Pi 3.1415926
using std::string;
using namespace cv;
using namespace std;

//============================== Class define ====================
class cells
{
public:
    int number;
    // vector<Point> vertices;
    vector<Point> edge_up;
    vector<Point> edge_lo;
    vector<Point> edge_lt;
    vector<Point> edge_rt;
    vector<Point> path;
    vector<vector<int> > pixel;
    vector<int> connected;
    vector<int> branch;
    string status;
    int area;
    int cleaned = 0;
    Scalar color;
};
class robot
{
private:
    int size = 9;
    int min_entry_width = 8;

public:
    Rect pose;
    vector<Point> path;
    void update_pose(Point pt){
        pose = Rect(pt.x,pt.y,size,size);
    }
    void robot_pose_init(Point pt){
        update_pose(pt);
        path.push_back(pt);
    }
};
//============================= Function define ==================
vector<Point> find_intersection_line(Mat map, vector<Point> line);
vector<Point> yslice_gen(int x_current, int map_size_y);
vector<cells> cell_decompose();
Mat rotate_img(Mat src, double asngle);
int point_check(Point pt1, Point pt2);
int match_intersect(const cells &c1, vector<Point> intersect);
Mat print_cells();
int get_cell_area(const cells &c1);
void erase_small_cells(int size_s);
void cell_pixel_init();
void erase_tiny_cells(int min_width);
void cell_img_mouse_callback(int event, int x, int y, int flags, void* param);
void cell_to_clean_img_mouse_callback(int event, int x, int y, int flags, void* param);
void coverage_path_planning();
// void reorder_cell_cleaning();
void cells_D_generation();
int cell_edge_check(cells &c1, cells &c2);
int check_map_rect(Point pt);
Point2f rotatePoint(Point p1, float angle);
void erase_unreachable_cells();
vector<int> ACO();

//============================= Global var =======================
Mat src;
Mat masked_img;
Mat cell_decompose_result;
vector<cells> cells_v;
vector<cells> cleaned_cells;
vector<int> cells_order;
int map_size_x;
int map_size_y;
int robot_size = 9;        // size of robot set to 10 x 10;
int min_entry_width = 8;
robot robot;
vector<int> begin_of_cells;
Point2f img_center;
float angle;
vector<vector<int> > D_matrix;
//================================================================
int main(int argc, char ** argv){
    if (argc != 1){
        src = imread(argv[1], IMREAD_GRAYSCALE);    
    }else if (argc == 1){
        cout << "Image read default" << endl;
        src = imread("full.png", IMREAD_GRAYSCALE);
    }
    if (!src.data){
        return -1;
        cout << "image read failed" << endl;
    }
    //=================== Read file complete ==================//

    // map image rotation and edge sharpening
    Mat src_rotate,src_threshold;
    angle = -10.5;
    src_rotate = rotate_img(src,angle);

    // map image thresholding
    threshold(src_rotate,src_threshold,210,255,0);
    cout << "src size is " << src.size() << "src (20,20) color is =" << (int)src.at<uchar>(20,20)<< endl;
    /* namedWindow("Source image", WINDOW_AUTOSIZE);
    imshow("Source image", src); */

    map_size_x = src.cols;
    map_size_y = src.rows;
    img_center.x = map_size_x/2+0.5;
    img_center.y = map_size_y/2+0.5;
    // Blur image
    Mat blurred_img(src.size(), CV_8UC1);
    blur( src_threshold, blurred_img, Size(3,3));
    // namedWindow("Blurred image", WINDOW_AUTOSIZE);
    // imshow("Blurred image", blurred_img);

    // Apply canny edge algorithm to the map image
    Mat canny_img(src.size(), CV_8UC1);
    Canny(blurred_img, canny_img, 80, 100*2, 3);
    // namedWindow("Canny src image", WINDOW_AUTOSIZE);
    // imshow("Canny src image", canny_img);
    
    // Find contours of map canny image
    Mat contours_img(src.rows,src.cols,CV_8UC1,Scalar(255));
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(canny_img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0,0));
    drawContours(contours_img, contours, -1, Scalar(0), 2, 8, hierarchy, 1, Point());
    
    cout << " the size of contours = " << contours.size() << endl;
    // Generate cells
    // build base yslice line

    // cout << " map (182,245) is " << (int)src.at<uchar>(182,245) << endl;
    // find the gradient at interested point
    

    // floodfill target map
    // Mat1b mask;
    Rect roi;
    uchar seedColor = 200;
    Mat mask_img = contours_img.clone();
    // copyMakeBorder(contours_img, mask, 1, 1, 1, 1, BORDER_CONSTANT, Scalar(0));
    // floodFill(contours_img, mask, Point(11,11), Scalar(0), &roi, Scalar(5), Scalar(5), 4 | (int(seedColor) << 8) | FLOODFILL_MASK_ONLY);
    floodFill(mask_img,Point(150,150),Scalar(200),&roi,Scalar(5),Scalar(5));
    masked_img = (mask_img == seedColor);
    cout << " size of result " << masked_img.size() << endl;


    cells_v = cell_decompose();  
    cout << " size of cells_v is " << cells_v.size() << endl;
    
    // cell_decompose_result = print_cells(cells_v);
    // namedWindow("image contours before");
    // imshow("image contours before", masked_img);
    erase_small_cells(10);
    cells_v = cell_decompose();  
    cout << " size of cells_v is " << cells_v.size() << endl;
    // Mat cell_decompose_result;
    // cell_decompose_result = print_cells();
    for(int i = 0; i < cells_v.size(); ++i){
        cells_v[i].area = get_cell_area(cells_v[i]);
    }
    // Mat map_before_remove = print_cells();
    erase_tiny_cells(10);
    cell_pixel_init();
    
    cell_decompose_result = print_cells();
    
    cout << " size of cells_v is " << cells_v.size() << endl;
    // cout << " the value of cell 2 . pixel " << cells_v[2].pixel[0][0] << endl;
   // cout << " count for small cells is " << count << endl;
    namedWindow("image contours after");
    imshow("image contours after", masked_img);
    // imwrite("map_masked.png", masked_img);
    namedWindow("Cells");
    setMouseCallback("Cells", cell_img_mouse_callback, (void*)&cell_decompose_result);
    // imshow("Cells before remove", map_before_remove);
    imshow("Cells",cell_decompose_result);

    coverage_path_planning();
    Mat draw_path = cell_decompose_result.clone();
    namedWindow("Path");
    int index_cell = 1;
    // Mat dst;
    Mat src_color;
    namedWindow("Source image", WINDOW_AUTOSIZE);
    cvtColor(src,src_color,CV_GRAY2BGR);
    imshow("Source image", src_color);
    int empty_cell_num = 0;
    for(int i = 0; i < cells_v.size(); ++i){
        cout << " the " << i << " th cell" << endl;
        for(int j = 0; j < cells_v[i].path.size(); ++j){
            cout << " the " << i << " th cell path point is " << cells_v[i].path[j] << endl;
        }
        if(cells_v[i].path.size() == 0){
            ++empty_cell_num;
        }else if(cells_v[i].path.size() > 1){
            cleaned_cells.push_back(cells_v[i]);
        }
    }
    cout << "一共 " << cells_v.size() << " 个分区，已清扫 " << cells_v.size() - empty_cell_num << " 个分区" << endl;
    cout << " the size of cleaned_cells is " << cleaned_cells.size() << endl;
    
    // ========================== Shrink company logo to put in graph =======================
    Mat logo = imread("company_logo.jpg", IMREAD_COLOR);
    Rect r(logo.cols/2. - 236, logo.rows/2. - 204, 236*2,208*2);
    Mat roi_logo(logo,r);
    Mat roi_resize;
    resize(roi_logo,roi_resize,Size(9,9));

    // ========================== Erase unreachable cells ===================================
    erase_unreachable_cells();

    namedWindow("Cells to clean");
    setMouseCallback("Cells to clean", cell_to_clean_img_mouse_callback, (void*)&cell_decompose_result);
    // imshow("Cells before remove", map_before_remove);
    imshow("Cells to clean",cell_decompose_result);
    // ========================== Generate D_matrix for ACO algorithm =======================
    cells_D_generation();

    // find center of cells and write to file;
    vector<int> cells_center_x;
    vector<int> cells_center_y;
    for(auto it = cleaned_cells.begin(); it != cleaned_cells.end(); ++it){
        if(it->path.size()>0){
            cells_center_x.push_back((it->edge_lt[0].x+it->edge_lt.back().x+it->edge_rt[0].x+it->edge_rt.back().x)/4);
            cells_center_y.push_back((it->edge_lt[0].y+it->edge_lt.back().y+it->edge_rt[0].y+it->edge_rt.back().y)/4);
        }
    }
    int i = 0;
    ofstream ofile;
    ofile.open("cells_center_xy.txt");
    for(i = 0; i < cells_center_x.size(); ++i){
        ofile<<cells_center_x[i]<< endl;
    }
    for(i = 0; i < cells_center_y.size(); ++i){
        ofile<<cells_center_y[i]<< endl;
    }
    ofile.close();

    // int i;
    // ofstream ofile;
    ofile.open("D_matrix.txt");
    
    for(int i = 0; i < D_matrix.size(); ++i){
        for(int j = 0; j < D_matrix[i].size(); ++j){
            ofile<<D_matrix[i][j]<<endl;
        }
    }
    ofile.close();

    
    vector<int> aco_path;
    // aco_path = ACO();


    fstream myfile("aco_path.txt", ios_base::in);
    int int_line;
    if(myfile.is_open()){
        cout << " txt opened" << endl;
        while( myfile >> int_line){
            aco_path.push_back(int_line);
            cells_order.push_back(int_line);
        }
    }
    myfile.close();

    int full_length = 0;
    cout << " the shortest length is " << endl;
    for (i = 0; i < aco_path.size() - 1; ++i){
        cout << " From node " << aco_path[i] << " to " << aco_path[i+1] << " is " << D_matrix[ aco_path[i] ][ aco_path[i+1] ] << endl;
        full_length += D_matrix[ aco_path[i] ][ aco_path[i+1] ];
    }
    cout << " the full length is " << full_length << endl;
    

    cout << " the size of aco_path is " << aco_path.size() << endl;
    
    // Iterate through robot.path to print out robot path, skip begin of cells path
    /* for(auto it = cells_v.begin(); it != cells_v.end(); ++it, ++i){
        if(it->path.size() > 1){
            for(auto it2 = it->path.begin(); it2 != it->path.end()-1; ++it2){
                LineIterator it_line(draw_path, Point(it2->x+4,it2->y+4), Point((it2+1)->x+4,(it2+1)->y+4), 8);
                LineIterator it_line2 = it_line;
                ++it_line2;
                LineIterator it_line3(draw_path, rotatePoint(Point(it2->x+4,it2->y+4),-angle), rotatePoint(Point((it2+1)->x+4,(it2+1)->y+4),-angle), 8);
                LineIterator it_line4 = it_line3;
                ++it_line4;
                LineIterator it_line5(draw_path, rotatePoint(*it2,-angle), rotatePoint(*(it2+1),-angle), 8);
                for(int j = 0; j < it_line.count; j++, ++it_line, ++it_line2, ++it_line3, ++it_line5){
                    
                    // circle(draw_path,it_line.pos(),4.5,Scalar(255,255,255),-1);
                    circle(src_color,it_line3.pos(),4,Scalar(215,142,34),-1);
                    line(draw_path, it_line.pos(), it_line2.pos(), Scalar(255,255,255));
                    Mat draw_path_with_shape = draw_path.clone();
                    circle(draw_path_with_shape,it_line.pos(),4,Scalar(0,0,0),-1);
                    Mat src_color_with_logo = src_color.clone();
                    roi_resize.copyTo(src_color_with_logo.rowRange(it_line5.pos().y,it_line5.pos().y + 9).colRange(it_line5.pos().x,it_line5.pos().x+9));
                    // cout << " it_line5.pos() is " << it_line5.pos() << endl;
                    imshow("Path", draw_path_with_shape);
                    imshow("Source image",src_color_with_logo);
                    if(waitKey(1) >=0) return 0;
                }
            }
        }
        circle(draw_path, Point(cells_center_x[i],cells_center_y[i]), 5, Scalar(0,0,0), -1);
    } */
    /* for( i = 0; i < cleaned_cells.size(); ++i){
        if(cleaned_cells[i].path.size() > 1){
            for( auto it2 = cleaned_cells[i].path.begin(); it2 != cleaned_cells[i].path.end(); ++it2){
                LineIterator it_line(draw_path, Point(it2->x+4,it2->y+4), Point((it2+1)->x+4,(it2+1)->y+4), 8);
                LineIterator it_line2 = it_line;
                ++it_line2;
                LineIterator it_line3(draw_path, rotatePoint(Point(it2->x+4,it2->y+4),-angle), rotatePoint(Point((it2+1)->x+4,(it2+1)->y+4),-angle), 8);
                LineIterator it_line4 = it_line3;
                ++it_line4;
                LineIterator it_line5(draw_path, rotatePoint(*it2,-angle), rotatePoint(*(it2+1),-angle), 8);
                for(int j = 0; j < it_line.count; j++, ++it_line, ++it_line2, ++it_line3, ++it_line5){
                    
                    // circle(draw_path,it_line.pos(),4.5,Scalar(255,255,255),-1);
                    circle(src_color,it_line3.pos(),4,Scalar(215,142,34),-1);
                    line(draw_path, it_line.pos(), it_line2.pos(), Scalar(255,255,255));
                    Mat draw_path_with_shape = draw_path.clone();
                    circle(draw_path_with_shape,it_line.pos(),4,Scalar(0,0,0),-1);
                    Mat src_color_with_logo = src_color.clone();
                    roi_resize.copyTo(src_color_with_logo.rowRange(it_line5.pos().y,it_line5.pos().y + 9).colRange(it_line5.pos().x,it_line5.pos().x+9));
                    // cout << " it_line5.pos() is " << it_line5.pos() << endl;
                    imshow("Path", draw_path_with_shape);
                    imshow("Source image",src_color_with_logo);
                    if(waitKey(1) >=0) return 0;
                }
            }
        }
        circle(draw_path, Point(cells_center_x[aco_path[i]],cells_center_y[aco_path[i]]), 5, Scalar(0,0,0), -1);
    } */
    

    cells_order = aco_path;

    for(int i = 0; i < cells_order.size(); ++i){
        if(cleaned_cells[cells_order[i]].path.size() > 1){
            for(auto it2 = cleaned_cells[cells_order[i]].path.begin(); it2 != cleaned_cells[cells_order[i]].path.end()-1; ++it2){
                LineIterator it_line(draw_path, Point(it2->x+4,it2->y+4), Point((it2+1)->x+4,(it2+1)->y+4), 8);
                LineIterator it_line2 = it_line;
                ++it_line2;
                LineIterator it_line3(draw_path, rotatePoint(Point(it2->x+4,it2->y+4),-angle), rotatePoint(Point((it2+1)->x+4,(it2+1)->y+4),-angle), 8);
                LineIterator it_line4 = it_line3;
                ++it_line4;
                LineIterator it_line5(draw_path, rotatePoint(*it2,-angle), rotatePoint(*(it2+1),-angle), 8);
                for(int j = 0; j < it_line.count; j++, ++it_line, ++it_line2, ++it_line3, ++it_line5){
                    
                    // circle(draw_path,it_line.pos(),4.5,Scalar(255,255,255),-1);
                    circle(src_color,it_line3.pos(),4,Scalar(215,142,34),-1);
                    line(draw_path, it_line.pos(), it_line2.pos(), Scalar(255,255,255));
                    Mat draw_path_with_shape = draw_path.clone();
                    circle(draw_path_with_shape,it_line.pos(),4,Scalar(0,0,0),-1);
                    Mat src_color_with_logo = src_color.clone();
                    roi_resize.copyTo(src_color_with_logo.rowRange(it_line5.pos().y,it_line5.pos().y + 9).colRange(it_line5.pos().x,it_line5.pos().x+9));
                    // cout << " it_line5.pos() is " << it_line5.pos() << endl;
                    imshow("Path", draw_path_with_shape);
                    imshow("Source image",src_color_with_logo);
                    if(waitKey(1) >=0) return 0;
                }
            }
        }
    }

    

    
    
    
    // imshow("Source image", src);
    waitKey(0);
    return 0;
}
//============================== Function define ================
vector<cells> cell_decompose(){
    vector<Point> yslice;
    vector<Point> intersect;   
    vector<cells> cells_v;
    // push first cell
    /* cout << "*************" << endl;
    cout << " Point_check(134,260) and (135,269) is " << point_check(Point(134,160),Point(135,166)) << endl;
    cout << " ++++++++++++" << endl; */
    int i;
    // ===================== check for first pairs intersect to show ====
    for(i = 0; i < masked_img.cols; ++i){
        yslice = yslice_gen(i,map_size_y);
        intersect = find_intersection_line(masked_img, yslice);
        if(intersect.size()>1){
            /* cout << "intersect is " << intersect << endl;
            cout << " i is " << i << endl; */
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
        
    }

    return cells_v;
}
vector<Point> find_intersection_line(Mat map, vector<Point> line){
    // Point is of (y,x) == (rows, cols)
    vector<Point> intersection;    
    int i;
    for(i = 0; i < line.size(); ++i){
        if((int)map.at<uchar>(line[i].y,line[i].x) == 0){
            // cout << "the intersection are = " << line[i] << endl;
            if(line[i].y > 1 && (int)map.at<uchar>(line[i].y-1,line[i].x) == 255){
                intersection.push_back(line[i]);
            }
            if(line[i].y < map.rows-1 && (int)map.at<uchar>(line[i].y+1,line[i].x) == 255){
                intersection.push_back(line[i]);
            }            
        }
    }
    //cout << " current intersection is = " << intersection << endl;
    return intersection;
}
vector<Point> yslice_gen(int x_current, int map_size_y){
    vector<Point> yslice;
    for(int i = 0; i < map_size_y; ++i){
        yslice.push_back(Point(x_current,i));
    }
    
    return yslice;
}
Mat rotate_img(Mat src, double angle){
    Mat dst;
    Point2f pt(src.cols/2., src.rows/2.);
    Mat r = getRotationMatrix2D(pt, angle,1.0);
    warpAffine(src,dst,r,Size(src.cols,src.rows));
    return dst;
}
int point_check(Point pt1, Point pt2){
    int returnVal = 0;
    if(pt1.y == pt2.y){
        if((int)masked_img.at<uchar>(pt1.y,pt1.x) == 0 && (int)masked_img.at<uchar>(pt2.y,pt2.x) == 0){
            return 1;
        }
    }else if(pt1.y < pt2.y){
        int y_err = pt2.y - pt1.y;
        if((int)masked_img.at<uchar>(pt2.y,pt2.x-1) == 0){
            for(int i = 1; i <= y_err; ++i){
                if((int)masked_img.at<uchar>(pt2.y-i,pt2.x-1) != 0){
                    return 0;
                }
            }
            return y_err + 1;
        }else if((int)masked_img.at<uchar>(pt1.y,pt1.x+1) == 0){
            for(int i = 1; i <= y_err; ++i){
                if((int)masked_img.at<uchar>(pt1.y+i,pt1.x+1) != 0){
                    return 0;
                }
            }
            return y_err + 1;
        }else{
            return 0;
        }
    }else if(pt1.y > pt2.y){
        int y_err = pt1.y - pt2.y;
        /* if((int)masked_img.at<uchar>(pt1.y,pt1.x+1) == 0){
            for(int i = 1; i <= y_err; ++i){
                if((int)masked_img.at<uchar>(pt1.y-i,pt1.x+1) != 0){
                    return 0;
                }
            }
            return y_err + 1; */
        if((int)masked_img.at<uchar>(pt2.y,pt2.x-1) == 0){
            for(int i = 1; i <= y_err; ++i){
                if((int)masked_img.at<uchar>(pt2.y+i,pt2.x-1) != 0){
                    return 0;
                }
            }
            return y_err + 1;
        }else if((int)masked_img.at<uchar>(pt1.y,pt1.x+1) == 0){
            for(int i = 1; i <= y_err; ++i){
                if((int)masked_img.at<uchar>(pt1.y-i,pt1.x+1) != 0){
                    return 0;
                }
            }
            return y_err + 1;
        }else{
            return 0;
        }
    }
    return returnVal;
}
int match_intersect(const cells &c1, vector<Point> intersect){
    for(int i = 0; i < intersect.size(); i = i + 2){
        if(point_check(c1.edge_up[0],intersect[i]) != 0 && point_check(c1.edge_up[1],intersect[i+1])){
            return i;
        }
    }

    return -1;
}
Mat print_cells(){
    Mat result(masked_img.rows,masked_img.cols,CV_8UC3,Scalar(255,255,255));
    RNG rng(12345);
    for(int i = 0; i < cells_v.size(); ++i){
        cells_v[i].color = Scalar(rng.uniform(0,220), rng.uniform(0, 220), rng.uniform(0, 220));
        for(int j = 0; j < cells_v[i].edge_lt.size(); ++j){
            Rect region(cells_v[i].edge_lt[j].x,cells_v[i].edge_lt[j].y+1,1,cells_v[i].edge_rt[j].y-cells_v[i].edge_lt[j].y-1);
            result(region) = cells_v[i].color;
        }
        /* cout << " the " << i << " th cell" << endl;
        cout << " the size of cell" << i << "is " << cells_v[i].edge_lt.size() << endl;
        if(cells_v[i].edge_lt.size() != cells_v[i].edge_rt.size()) ++count; */

    }
    return result;
}
int get_cell_area(const cells &c1){
    int area = 0;;
    for(int i = 0; i < c1.edge_lt.size(); ++i){
        area = area + c1.edge_rt[i].y - c1.edge_lt[i].y - 1;
    }
    return area;
}
void erase_small_cells(int size_s){
    for(int i = 0; i < cells_v.size(); ++i){
        cells_v[i].area = get_cell_area(cells_v[i]);
        // cout << " the area of " << i << " th cell is " << cells_v[i].area << endl;
        if(cells_v[i].area <= size_s){
            for(int j = 0; j < cells_v[i].edge_lt.size(); ++j){
                Rect region(cells_v[i].edge_lt[j].x,cells_v[i].edge_lt[j].y+1,1,cells_v[i].edge_rt[j].y-cells_v[i].edge_lt[j].y-1);
                masked_img(region) = 0;
            }

        }
    }
}
void cell_pixel_init(){
    int i = 0;
    int j = 0;
    for(auto it = cells_v.begin(); it != cells_v.end(); ++it){
        for(i = 0; i < it->edge_lt.size(); ++i){
            it->pixel.push_back(vector<int>(it->edge_rt[i].y-it->edge_lt[i].y-1,0));
            // cout << "currenct cell is "<< it-cells_v.begin() << " line space is " << it->pixel[i].size() << endl;
        }

    }
}
void cell_img_mouse_callback(int event, int x, int y, int flags, void* param){
    Mat& img = *(Mat*) param; 
    if(event == EVENT_MOUSEMOVE){
        string msg = "Current Cell: ";
        string msg1;
        int i;
        int j;
        for(i = 0; i < cells_v.size(); ++i){
            for(j = 0; j < cells_v[i].edge_lt.size(); ++j){
                if(x == cells_v[i].edge_lt[j].x && y < cells_v[i].edge_rt[j].y && y > cells_v[i].edge_lt[j].y){
                    msg1 = to_string(i) + "   " + "Area is " + to_string(cells_v[i].area);
                }
            }
        }
        displayOverlay("Cells", msg+msg1);
    }
}
void cell_to_clean_img_mouse_callback(int event, int x, int y, int flags, void* param){
    Mat& img = *(Mat*) param; 
    if(event == EVENT_MOUSEMOVE){
        string msg = "Current Cell: ";
        string msg1;
        int i;
        int j;
        for(i = 0; i < cleaned_cells.size(); ++i){
            for(j = 0; j < cleaned_cells[i].edge_lt.size(); ++j){
                if(x == cleaned_cells[i].edge_lt[j].x && y < cleaned_cells[i].edge_rt[j].y && y > cleaned_cells[i].edge_lt[j].y){
                    msg1 = to_string(i) + "   " + "Area is " + to_string(cleaned_cells[i].area);
                }
            }
        }
        displayOverlay("Cells to clean", msg+msg1);
    }
}
void coverage_path_planning(){
    Point path_pt;
    int i;
    int j;
    for(auto it = cells_v.begin(); it != cells_v.end(); ++it){
        int start_f = 0;
        // find the start position of robot path in current cell through check map
        path_pt = it->edge_lt[0];
        if(it->edge_rt[0].y - it->edge_lt[0].y < 4){
            path_pt = it->edge_lt[1];
        }
        ++path_pt.y;
        while(path_pt.y < it->edge_rt[0].y){
            if(check_map_rect(path_pt)){
                it->path.push_back(path_pt);
                robot.robot_pose_init(path_pt);
                start_f = 1;
                break;
            }
            ++path_pt.y;
        }
        // if cell width is < robot_size * 0.4 then it will not be cleaned
        if(start_f == 1 && it->edge_lt.size() < robot_size*0.4){
            robot.path.pop_back();
            it->path.pop_back();
            start_f = 0;
        }
        cout <<" the current cell is " << it-cells_v.begin() << " Area is " << it->area << endl;
        cout << " start_f is " << start_f << endl;
        if(start_f == 1){
            begin_of_cells.push_back(robot.path.end() - robot.path.begin());
            // cout << " the begin of t current cell is " << begin_of_cells.back() << endl;
            float vertical_traj_num = (float)it->edge_lt.size()/robot_size;
            
            //======================= find below position ===========
            // look for left most down most corner of cell map
            Point pt_left_most = it->edge_rt[0];

            /* if(begin_of_cells.back() == 251){
                cout << " (1) pt_left_most is " << pt_left_most << endl;
            } */
            for(i = 0; i < robot_size/2; ++i){
                if(it->edge_rt[i].y > pt_left_most.y){
                    pt_left_most = it->edge_rt[i];
                }
            }
            /* if(begin_of_cells.back() == 251){
                cout << " (2) pt_left_most is " << pt_left_most << endl;
            } */
            // offset corner to robot size and check to fit robot size on map
            pt_left_most.y = pt_left_most.y - robot_size;
            while(pt_left_most.y > it->edge_lt[pt_left_most.x - path_pt.x].y){
                if(check_map_rect(pt_left_most)){
                    break;
                }
                --pt_left_most.y;
            }
            /* if(begin_of_cells.back() == 251){
                cout << " (3) pt_left_most is " << pt_left_most << endl;
            }
            cout << " the current cell vertical_traj_num " << vertical_traj_num << endl; */
            
            if(vertical_traj_num < 1+0.3){
                robot.path.push_back(pt_left_most);
                it->path.push_back(pt_left_most);
                /* if(begin_of_cells.back() == 251){
                    cout << " (4) pt_left_most is " << robot.path.back() << endl;
                } */
                
            }if(vertical_traj_num > 1+0.3){
                robot.path.push_back(pt_left_most);
                it->path.push_back(pt_left_most);
                /* if(begin_of_cells.back() == 251){
                    cout << " (5) pt_left_most is " << robot.path.back() << endl;
                } */
                Point pt_upper, pt_lower;
                int settle_f = 0;
                for(i = 0; i < floor(vertical_traj_num); ++i){
                    // upper point for strip take point at start_pt's x value
                    // take border - 9 instead if over come
                    pt_upper.x = path_pt.x+robot_size*(i+1);
                    if(pt_upper.x > it->edge_lt.back().x - robot_size+1){
                        pt_upper.x = it->edge_lt.back().x - robot_size+1;
                    }
                    // upper point y value takes edge_lt at upper point's x
                    pt_upper.y = it->edge_lt[pt_upper.x - path_pt.x].y+1;
                    if(check_map_rect(pt_upper) == 0){
                        settle_f =0;
                        j = 0;
                        while(j < 9){
                            pt_upper.x = pt_upper.x - j;
                            pt_upper.y = it->edge_lt[pt_upper.x - path_pt.x].y+1;
                            while(pt_upper.y < it->edge_rt[pt_upper.x - path_pt.x].y){
                                if(check_map_rect(pt_upper)){
                                    settle_f = 1;
                                    break;
                                }
                                ++pt_upper.y;
                            }
                            if(settle_f){
                                break;
                            }
                            ++j;
                        }
                    }

                    pt_lower.x = path_pt.x+robot_size*(i+1);
                    if(pt_lower.x > it->edge_rt.back().x - robot_size+1){
                        pt_lower.x = it->edge_rt.back().x - robot_size+1;
                    }
                    pt_lower.y = it->edge_rt[pt_lower.x - path_pt.x].y-robot_size;
                    if(check_map_rect(pt_lower) == 0){
                        settle_f = 0;
                        j = 0;
                        while(j < 9){
                            pt_lower.x = pt_lower.x - j;
                            pt_lower.y = it->edge_rt[pt_lower.x - path_pt.x].y-robot_size;
                            while(pt_lower.y > it->edge_lt[pt_lower.x - path_pt.x].y){
                                if(check_map_rect(pt_lower)){
                                    settle_f = 1;
                                    break;
                                }
                                --pt_lower.y;
                            }
                            if(settle_f){
                                break;
                            }
                            ++j;
                        }
                    }
                    // process the finish edge ofrobot path
                    

                    if(i%2 == 0){
                        robot.path.push_back(pt_lower);
                        robot.path.push_back(pt_upper);
                        it->path.push_back(pt_lower);
                        it->path.push_back(pt_upper);     
                    }
                    if(i%2 == 1){
                        robot.path.push_back(pt_upper);
                        robot.path.push_back(pt_lower);
                        it->path.push_back(pt_upper);
                        it->path.push_back(pt_lower);
                    }
                }       
            }
        }
    
    }   
}
// the function to generate order for cells to be cleaned
/* void reorder_cell_cleaning(){
    // vector<int> v_i1;
    int i =0;
    int j = 0;
    int current_cell = 0;
    int tmp;
    for(auto it = cleaned_cells.begin(); it != cleaned_cells.end(); ++it, ++i){
        it->number = i;
        // cout << " the " << it->number << " cells edge_lo is  " << it->edge_lo  << endl;
    }
    cleaned_cells[0].branch = {0};
    current_cell = 0;
    cells_order.push_back(current_cell);
    for(i = 0; i < cleaned_cells.size(); ++i){
        if(i != current_cell && i != cleaned_cells[current_cell].branch[0]){
            cell_edge_check(cleaned_cells[current_cell], cleaned_cells[i]);
        }
    }
    if(cleaned_cells[current_cell].connected.size() > 0){
        
        cleaned_cells[current_cell].cleaned = 1;
        tmp = current_cell;
        
        
        current_cell = cleaned_cells[current_cell].connected[0];
        cleaned_cells[tmp].connected.erase(cleaned_cells[tmp].connected.begin());
    }else if(cleaned_cells[current_cell].connected.size() == 0){
        current_cell = cleaned_cells[current_cell].branch[0];
    }
    i = 0;
    while(current_cell != 0){
        if(cleaned_cells[current_cell].cleaned == 0){
            for(i = 0; i < cleaned_cells.size(); ++i){
                if(i != current_cell && i != cleaned_cells[current_cell].branch[0]){
                    cell_edge_check(cleaned_cells[current_cell], cleaned_cells[i]);
                }
            }
            cells_order.push_back(current_cell);
        }
        if(cleaned_cells[current_cell].connected.size() > 0){
            
            cleaned_cells[current_cell].cleaned = 1;
            tmp = current_cell;
            
            current_cell = cleaned_cells[current_cell].connected[0];
            cleaned_cells[tmp].connected.erase(cleaned_cells[tmp].connected.begin());
            
            
        }else if(cleaned_cells[current_cell].connected.size() == 0){
            current_cell = cleaned_cells[current_cell].branch[0];
        }
        cout << "+++++++++++++++++++++++" << endl;
        cout << " current cell is " << current_cell  << " Area is " << cleaned_cells[current_cell].area << endl;
        cout << " current cell.connected size is " << cleaned_cells[current_cell].connected.size() << endl;
        ++i;
    }
    cout << " reorder calculation time is " << i << endl;
    

} */
// check if cells c2 is a connect cell of cells c1

void erase_tiny_cells(int min_width){
    /* for(int i = 0; i < cells_v.size(); ++i){
        if(cells_v[i].edge_rt[0].y - cells_v[i].edge_lt[0].y -1 <= min_width || cells_v[i].edge_rt.back().y - cells_v[i].edge_lt.back().y -1 <= min_width){
            // ++count;
            cells_v.erase(cells_v.begin()+i);
        }
    } */
    int count;
    for(auto it = cells_v.begin(); it != cells_v.end(); ++it){
        /* if(it->edge_rt[0].y - it->edge_lt[0].y -1 <= min_width || it->edge_rt.back().y - it->edge_lt.back().y -1 <= min_width){
            // ++count;
            cells_v.erase(it);
        } */
        count = 0;
        for(int i = 0; i < it->edge_lt.size(); ++i){
            if(it->edge_rt[i].y - it->edge_lt[i].y - 1 <= min_width){
                ++count;
            }
        }
        // cout <<" the current cell is " << it-cells_v.begin() << " the count is " << count << endl;
        // cout << " the size of it->edge_lt.size() is " << it->edge_lt.size() << endl;
        // cout << " size to remove cell is " << (float)count/it->edge_lt.size() << endl;
        if(((float)(count/it->edge_lt.size()) > (float)0.2) && (count > 0)){
            
            // cout << " cell removed " << endl;
            cells_v.erase(it);
        }
    }
    // cout << " the number of tiny cells is " << count << endl;
}
int check_map_rect(Point pt){
    int i;
    int j;
    for(i = 0; i < robot_size; ++i){
        for(j = 0; j < robot_size; ++j){
            if((int)masked_img.at<uchar>(pt.y+i, pt.x+j) != 255){
                return 0;
            }
        }
    }


    return 1;
}
Point2f rotatePoint(Point p1, float angle){
    Point2f result;

    /* result.x = ((p1.x - img_center.x) * cos(angle*(Pi/180))) - ((p1.y - img_center.y) * sin(angle*(Pi/180))) + img_center.x;
    result.y = ((p1.x - img_center.x) * sin(angle*(Pi/180))) - ((p1.y - img_center.y) * cos(angle*(Pi/180))) + img_center.y;
    return result; */
    
    Point2f pt(src.cols/2., src.rows/2.);
    Mat r = getRotationMatrix2D(pt, angle,1.0);
    // warpAffine(src,dst,r,Size(src.cols,src.rows));
    result.x = r.at<double>(0,0)*p1.x + r.at<double>(0,1)*p1.y + r.at<double>(0,2);
    result.y = r.at<double>(1,0)*p1.x + r.at<double>(1,1)*p1.y + r.at<double>(1,2);
    
    return result;
}
//================================= ACO algorithm for tsp ====================
vector<int> ACO(){
    
    /* vector<int> cells_center_x;
    vector<int> cells_center_y; */
    int NC_max = 50;
    int m = D_matrix.size();      // number of ants
    int n = m;      // number of cities
    double alpha = 1.5;
    double beta = 2;
    double rho = 0.1;
    double Q = pow(10,6);
    vector<double> zeroVec(n,0.0);
    vector<double> onesVec(n,1.0);
    vector<int> zeroVec_i(n,1);
    int i,j,k;
    
    vector<vector<double> > D;             // distance for cities
    // for(i = 0; i < n; ++i) D.push_back(zeroVec);
    vector<vector<double> > eta;
    // for(i = 0; i < n; ++i) eta.push_back(zeroVec);
    //====================== initialize distance matrix and its inverse
    /* for(i =0; i < n; ++i){
        for(j = 0; j < n; ++j){
            if(i!=j){
                D[i][j] = pow(pow(cells_center_x[i] - cells_center_x[j],2) + pow(cells_center_y[i] - cells_center_y[j],2) ,0.5);
                eta[i][j] = 1./D[i][j];
            }
        }
    } */
    for(i = 0; i < D_matrix.size(); ++i){
        vector<double> tmp_D;
        vector<double> tmp_eta;
        for(j = 0; j < D_matrix[i].size(); ++j){
            tmp_D.push_back((double) D_matrix[i][j]);
            tmp_eta.push_back((double) 1./D_matrix[i][j]);
        }
        D.push_back(tmp_D);
        eta.push_back(tmp_eta);
    }
    cout << " the D matrix is " << endl;
    for(auto it = D.begin(); it != D.end(); ++it){
        cout << endl << " the " << it-D.begin() << " th row is " << endl;
        for(auto it2 = (*it).begin(); it2 != (*it).end(); ++it2){
            cout << *it2 << setw(4);
        }
        
    }
    //===================== init Pheromone Matrix ===============
    vector<vector<double> > Tau;        // Pheromone
    for(i=0;i<n;++i) Tau.push_back(onesVec);
    
    // for(i=0;i<m;++i) Tabu.push_back(zeroVec);
    int NC = 1;
    vector<vector<int> > R_best;
    // for(i=0;i<NC_max;++i) R_best.push_back(zeroVec_i);
    vector<double> L_best;          // Shortest ant mileage for each iteration
    vector<double> L_ave;           // Average of ant mileage for each iteration

    while( NC < NC_max ){
        vector<vector<int> > Tabu;          // Ant path
        // Place ant randomly on cities
        vector<int> rand_pos;

        for(i=0;i<m;++i) rand_pos.push_back(i);
        random_shuffle(rand_pos.begin(),rand_pos.end());
        Tabu.push_back(rand_pos);
        /* //=================== Display 2D vector ========================
        for(i = 0; i < n; ++i){
            cout << " the " <<i<<" row is ";
            for(int j = 0; j < n; ++j){
                cout << Tabu[i][j]<<setw(2) << " ";
            }
            cout << endl;
        }
        //============================================================== */
        // each ant move towards next city based on its probability
        // cout << " the size of Tabu is " << Tabu.size() << endl;
        // int pt_c = 0;
        // if( NC == 1){
        for(i = 0; i < n-1; ++i){
        // for(i = 0; i < 1; ++i){
            vector<int> current_city;
            //=================== Display 2D vector ========================
            /* cout << " i = " << i << " Tabu[] = " << endl;
            for(int I = 0; I < Tabu.size(); ++I){
                cout << " the " <<I<<" row is ";
                for(int J = 0; J < n; ++J){
                    cout << Tabu[I][J]<<setw(2) << " ";
                }
                cout << endl;
            } */
            //==============================================================
            for(j = 0; j < m; ++j){
                vector<int> to_visit;
                for(k = 0; k < n; ++k){
                    int found = 0;
                    for(auto it = Tabu.begin(); it != Tabu.end(); ++it){
                        if(k == (*it)[j]){
                            found = 1;
                        }
                    }
                    if(found == 0){
                        to_visit.push_back(k);
                    }
                }
                // cout << " n number " << i << endl;
                /* cout << " the " << j << " th ant's to_visit ";
                for(k = 0; k < to_visit.size(); ++k){
                    cout << to_visit[k] << " ";
                }
                cout << endl; */
                vector<double> P(to_visit.size(), 0.0);
                // Reverse to visit city 
                // reverse(to_visit.begin(), to_visit.end());
                // for each city to be visited, calculate the associated probability from fomular P(Cij)
                double P_sum = 0.0;
                for(k = 0; k < to_visit.size(); ++k){
                    P[k] = pow(Tau[Tabu.back()[j]][to_visit[k]],alpha) * pow(eta[Tabu.back()[j]][to_visit[k]],beta);
                    P_sum += P[k];
                }
                for(k = 0; k < P.size(); ++k){
                    P[k] = P[k]/P_sum;
                }
                /* cout << " the " << j << " normalised P ";
                for(k = 0; k < P.size(); ++k){
                    cout << P[k] << " ";
                }
                cout << endl; */
                // reorder P[k] ascending order
                /* vector<v_i> v_i1;
                v_i v_i_temp;
                for(auto it = P.begin(); it != P.end(); ++it){
                    v_i_temp.num = *it;
                    v_i_temp.idx = it - P.begin();
                    v_i1.push_back(v_i_temp);
                } */
                /* auto idx = sort_indexes(P);
                for(k = 0; k < idx.size(); ++k){
                    to_visit[k] = idx[k];
                    
                }
                
                sort(P.begin(), P.end());
                cout << " size of to_visit and P is " << to_visit.size() << " " << P.size() << endl;
                // Cumulate probability */
                vector<double> P_cumsum(P.size(),0.0);
                partial_sum(P.begin(),P.end(),P_cumsum.begin());
                /* cout << " the " << j << " P_cumsum ";
                for(k = 0; k < P_cumsum.size(); ++k){
                    cout << P_cumsum[k] << " ";
                }
                cout << endl; */

                for(k = 0; k < P_cumsum.size(); ++k){
                    if(P_cumsum[k] >= ((double)rand() / (RAND_MAX))){
                        // cout << " the k has P > rand " << k << endl;
                        current_city.push_back(to_visit[k]);
                        // cout << " the to visit[k] is " << to_visit[k] << endl;
                        break;
                    }
                }  
            }
            Tabu.push_back(current_city);
        }
        //=================== Display 2D vector ========================
        /* for(i = 0; i < n; ++i){
            cout << " the " <<i<<" row is ";
            for(int j = 0; j < n; ++j){
                cout << Tabu[i][j]<<setw(2) << " ";
            }
            cout << endl;
        } */
        //==============================================================
        // keep record of best route for current iterate
        vector<double> ant_mileage;
        for(i = 0; i < m; ++i){
            double current_ant = 0.0;
            for(j = 0; j < n-1; ++j){
                current_ant += D[ Tabu[j][i] ][ Tabu[j+1][i] ];
            }
            ant_mileage.push_back(current_ant);
        }
        cout << " the ant_mileage is ";
        for(i = 0; i < ant_mileage.size(); ++i){
            cout << ant_mileage[i] << " ";
        }
        cout << endl;

        // Store shoreted path to L_best
        L_best.push_back(*min_element(ant_mileage.begin(), ant_mileage.end()));
        int best_index = min_element(ant_mileage.begin(), ant_mileage.end()) - ant_mileage.begin();
        vector<int> current_best_r;
        for(i = 0; i < n; ++i){
            current_best_r.push_back(Tabu[i][best_index]);
        }
        R_best.push_back(current_best_r);
        /* cout << " the L_best is " << L_best.back() << endl;
        cout << " the current_best_r is ";
        for(i = 0; i < current_best_r.size(); ++i){
            cout << current_best_r[i] << " ";
        }
        cout << endl; */
        L_ave.push_back(accumulate(ant_mileage.begin(), ant_mileage.end(), 0.0) / ant_mileage.size());

        //============================ Update Pheromone ======================================
        vector<vector<double> > Delta_Tau;
        for(i=0;i<n;++i) Delta_Tau.push_back(zeroVec);
        
        /* for(i = 0; i < n; ++i){
            cout << " the " <<i<<" row is ";
            for(int j = 0; j < n; ++j){
                cout << Delta_Tau[i][j]<<setw(2) << " ";
            }
            cout << endl;
        } */
        for(i = 0; i < m; ++i){
            for(j = 0; j < n-1 ; ++j){
                Delta_Tau[ Tabu[j][i] ][ Tabu[j+1][i] ] += Q/ant_mileage[i];
            }
            Delta_Tau[ Tabu[n-1][i] ][ Tabu[0][i] ] += Q/ant_mileage[i];
        }
        /* for(i = 0; i < n; ++i){
            cout << " the " <<i<<" row is ";
            for(int j = 0; j < n; ++j){
                cout << Delta_Tau[i][j]<<setw(2) << " ";
            }
            cout << endl;
        } */

        for(i = 0; i < n; ++i){
            for(j = 0; j < n; ++j){
                Tau[i][j] *= (1.0-rho);
                Tau[i][j] += Delta_Tau[i][j];
            }
        }
        ++NC;

    }

    /* cout << " R_best" << endl;
    for(i = 0; i < R_best.size(); ++i){
        cout << " the " <<i<<" row is ";
        for(int j = 0; j < n; ++j){
            cout << R_best[i][j]<<setw(2) << " ";
        }
        cout << endl;
    }
    cout << "***********************************"<< endl; */

    cout << "L_best" << endl;
    for(i = 0; i < L_best.size(); ++i){
        cout << " the " << i << " row is " << L_best[i] << endl;

    }
    int best_i = min_element(L_best.begin(), L_best.end()) - L_best.begin();

    cout << " the shortest travel distance is " << L_best[best_i] << endl;
    cout << " the shortest path is " << endl;
    for(i = 0; i < R_best[best_i].size(); ++i){
        cout << " cell NO. " << R_best[best_i][i] << "  " << endl;
        // cout << " ( " << cleaned_cells[R_best[best_i][i]].number <<  " ) " << endl;
    }
    /* Mat path_plot(120, 120, CV_8UC3, Scalar(255,255,255));
    
    for(i = 0; i < R_best[best_i].size()-1; ++i){
        line(path_plot, Point(cells_center_x[R_best[best_i][i]], cells_center_y[R_best[best_i][i]]), Point(cells_center_x[R_best[best_i][i+1]], cells_center_y[R_best[best_i][i+1]]), Scalar(0,0,0));
        circle(path_plot, Point(cells_center_x[R_best[best_i][i]], cells_center_y[R_best[best_i][i]]),4,Scalar(0,0,0),-1);
    }
    line(path_plot, Point(cells_center_x[R_best[best_i][i]], cells_center_y[R_best[best_i][i]]), Point(cells_center_x[R_best[best_i][0]], cells_center_y[R_best[best_i][0]]), Scalar(0,0,0));
    circle(path_plot, Point(cells_center_x[R_best[best_i][i]], cells_center_y[R_best[best_i][i]]),4,Scalar(0,0,0),-1);


    namedWindow("Plot", WINDOW_FREERATIO);
    imshow("Plot", path_plot);
    

    waitKey(0); */

    return R_best[best_i];
}

int cell_edge_check(cells &c1, cells &c2){
    // connected on the left edge (edge_lo) of c1
    /* if(c1.edge_lo[0].x - (c2.edge_up[0].x+1) < robot_size && c1.edge_lo[0].x - (c2.edge_up[0].x+1) >= 0){
        if((c2.edge_up[0].y > c1.edge_lo[0].y && c2.edge_up[0].y < c1.edge_lo[1].y) || (c2.edge_up[1].y > c1.edge_lo[0].y && c2.edge_up[0].y < c1.edge_lo[1].y) || (c2.edge_up[1].y > c1.edge_lo[1].y && c2.edge_up[0].y < c1.edge_lo[0].y)){
            c1.connected.push_back(c2.number);
            c2.branch.push_back(c1.number);
            return 1;
        }
    }else if(c2.edge_lo[0].x - (c1.edge_up[0].x+1) < robot_size && c2.edge_lo[0].x - (c1.edge_up[0].x+1) >= 0){
        if((c1.edge_up[0].y > c2.edge_lo[0].y && c1.edge_up[0].y < c2.edge_lo[1].y) || (c1.edge_up[1].y > c2.edge_lo[0].y && c1.edge_up[0].y < c2.edge_lo[1].y) || (c1.edge_up[1].y > c2.edge_lo[1].y && c1.edge_up[0].y < c2.edge_lo[0].y)){
            c1.connected.push_back(c2.number);
            c2.branch.push_back(c1.number);
            return 1;
        }
    } */

    // version with no branch information 
    if(c1.edge_lo[0].x - (c2.edge_up[0].x+1) < robot_size && c1.edge_lo[0].x - (c2.edge_up[0].x+1) >= 0){
        if((c2.edge_up[0].y > c1.edge_lo[0].y && c2.edge_up[0].y < c1.edge_lo[1].y) || (c2.edge_up[1].y > c1.edge_lo[0].y && c2.edge_up[0].y < c1.edge_lo[1].y) || (c2.edge_up[1].y > c1.edge_lo[1].y && c2.edge_up[0].y < c1.edge_lo[0].y)){
            c1.connected.push_back(c2.number);
            c2.connected.push_back(c1.number);
            return 1;
        }
    }else if(c2.edge_lo[0].x - (c1.edge_up[0].x+1) < robot_size && c2.edge_lo[0].x - (c1.edge_up[0].x+1) >= 0){
        if((c1.edge_up[0].y > c2.edge_lo[0].y && c1.edge_up[0].y < c2.edge_lo[1].y) || (c1.edge_up[1].y > c2.edge_lo[0].y && c1.edge_up[0].y < c2.edge_lo[1].y) || (c1.edge_up[1].y > c2.edge_lo[1].y && c1.edge_up[0].y < c2.edge_lo[0].y)){
            c1.connected.push_back(c2.number);
            c2.connected.push_back(c1.number);
            return 1;
        }
    }



    return 0;
}

// ======================================== generate cells distance =================
void cells_D_generation(){
    // vector<int> v_i1;
    int i = 0;
    int j = 0;
    int k = 0;
    int current_cell = 0;
    int tmp;
    /* for(auto it = cleaned_cells.begin(); it != cleaned_cells.end(); ++it, ++i){
        it->number = i;
        // cout << " the " << it->number << " cells edge_lo is  " << it->edge_lo  << endl;
        
    } */
    /* for(auto it = cleaned_cells.begin(); it != cleaned_cells.end(); ++it){
        // cout << " the " << it->number << " cells edge_lo is  " << it->edge_lo  << endl;
        for(auto it2 = it; it2 != cleaned_cells.end(); ++it2){
            if(it2 != it){
                cell_edge_check(*it, *it2);
            }
        }
    }
    // loop through every posible path,
    // untill all path 's last connected is null
    for(auto it = cleaned_cells.begin(); it != cleaned_cells.end(); ++it){
        if(it->connected.size() == 0){
            cleaned_cells.erase(it);
        }
    }
    cout << "****** the size of cleaned_cells is " << cleaned_cells.size() << endl;
    i = 0;
    for(auto it = cleaned_cells.begin(); it != cleaned_cells.end(); ++it, ++i){
        it->connected.clear();
        it->number = it - cleaned_cells.begin();
        // cout << " the " << it->number << " cells edge_lo is  " << it->edge_lo  << endl;
        
    } */
    for(auto it = cleaned_cells.begin(); it != cleaned_cells.end(); ++it){
        // cout << " the " << it->number << " cells edge_lo is  " << it->edge_lo  << endl;

        for(auto it2 = it; it2 != cleaned_cells.end(); ++it2){
            if(it2 != it){
                cell_edge_check(*it, *it2);
            }
        }
    }
    // ======================= Display all connected information of cells =========================
    for(auto it = cleaned_cells.begin(); it != cleaned_cells.end(); ++it){
        cout << " the curent cell " << it - cleaned_cells.begin() << " connected is:  "<< endl;
        if(it->connected.size()>0){
            for(i = 0; i < it->connected.size(); ++i){                
                cout << it->connected[i] << setw(4);                
            }
        }
        cout << endl;
    }
    cout << " the number of connected cells is " << cleaned_cells.size() << endl;
    for(auto it = cleaned_cells.begin(); it != cleaned_cells.end(); ++it){
        vector<int> D_row(cleaned_cells.size(),0);
        vector<int> openL;
        vector<int> closeL;
        int level = 1;
        openL = it->connected;
        // while(closeL.size() <= cleaned_cells.size()){
        while(openL.size() != 0){
            vector<int> tmp;
            for(i = 0; i < openL.size(); ++i){
                D_row[openL[i]] = level;
                closeL.push_back(openL[i]);
                
            }
            // cout << " the size of closeL is " << closeL.size() << endl;
            for(i = 0; i < openL.size(); ++i){
                for(j = 0; j < cleaned_cells[openL[i]].connected.size(); ++j){
                    
                    if(find(closeL.begin(), closeL.end(), cleaned_cells[openL[i]].connected[j]) == closeL.end()){
                        tmp.push_back(cleaned_cells[openL[i]].connected[j]);
                    }
                }
            }
            openL = tmp;
              
            ++level;
        }
        cout << " the current D_cell is " << it-cleaned_cells.begin() << endl;
        D_row[it-cleaned_cells.begin()] = INT_MAX;
        D_matrix.push_back(D_row);
        // cout << " the size of D_row is " << D_row.size() << endl;
    }
    // ================================== Print out the D_matrix ====================================
    cout << " the D_maxtrix is " << endl;
    for(i = 0; i < D_matrix.size(); ++i){
        cout << " the " << i << " th row is " << endl;
        for(j = 0; j < D_matrix[i].size(); ++j){
            cout << D_matrix[i][j] << setw(4);
        }
        cout << endl;
    }
}
void erase_unreachable_cells(){
    for(auto it = cleaned_cells.begin(); it != cleaned_cells.end(); ++it){
        // cout << " the " << it->number << " cells edge_lo is  " << it->edge_lo  << endl;
        for(auto it2 = it; it2 != cleaned_cells.end(); ++it2){
            if(it2 != it){
                cell_edge_check(*it, *it2);
            }
        }
    }
    // loop through every posible path,
    // untill all path 's last connected is null
    for(auto it = cleaned_cells.begin(); it != cleaned_cells.end(); ++it){
        if(it->connected.size() == 0){
            cleaned_cells.erase(it);
        }
    }
    cout << "****** the size of cleaned_cells is " << cleaned_cells.size() << endl;
    
    for(auto it = cleaned_cells.begin(); it != cleaned_cells.end(); ++it){
        it->connected.clear();
        it->number = it - cleaned_cells.begin();
        // cout << " the " << it->number << " cells edge_lo is  " << it->edge_lo  << endl;
        
    }
}