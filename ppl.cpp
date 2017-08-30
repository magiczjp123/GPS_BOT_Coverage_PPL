#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
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
Mat rotate_img(Mat src, double angle);
int point_check(Point pt1, Point pt2);
int match_intersect(const cells &c1, vector<Point> intersect);
Mat print_cells();
int get_cell_area(const cells &c1);
void erase_small_cells(int size_s);
void cell_pixel_init();
void erase_tiny_cells(int min_width);
void cell_img_mouse_callback(int event, int x, int y, int flags, void* param);
void coverage_path_planning();
int robot_move_up();
int robot_move_right();
int robot_move_down();
int check_map_rect(Point pt);
//============================= Global var =======================
Mat src;
Mat masked_img;
Mat cell_decompose_result;
vector<cells> cells_v;
int map_size_x;
int map_size_y;
int robot_size = 9;        // size of robot set to 10 x 10;
int min_entry_width = 8;
robot robot;
vector<int> begin_of_cells;
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
    src_rotate = rotate_img(src,-10.5);

    // map image thresholding
    threshold(src_rotate,src_threshold,210,255,0);
    cout << "src size is " << src.size() << "src (20,20) color is =" << (int)src.at<uchar>(20,20)<< endl;
    namedWindow("Source image", WINDOW_AUTOSIZE);
    imshow("Source image", src);

    map_size_x = src.cols;
    map_size_y = src.rows;

    Mat blurred_img(src.size(), CV_8UC1);
    blur( src_threshold, blurred_img, Size(3,3));
    namedWindow("Blurred image", WINDOW_AUTOSIZE);
    imshow("Blurred image", blurred_img);

    Mat canny_img(src.size(), CV_8UC1);
    Canny(blurred_img, canny_img, 80, 100*2, 3);
    namedWindow("Canny src image", WINDOW_AUTOSIZE);
    imshow("Canny src image", canny_img);
    
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
    namedWindow("image contours before");
    imshow("image contours before", masked_img);
    erase_small_cells(10);
    cells_v = cell_decompose();  
    cout << " size of cells_v is " << cells_v.size() << endl;
    // Mat cell_decompose_result;
    // cell_decompose_result = print_cells();
    for(int i = 0; i < cells_v.size(); ++i){
        cells_v[i].area = get_cell_area(cells_v[i]);
    }
    Mat map_before_remove = print_cells();
    erase_tiny_cells(10);
    cell_pixel_init();
    
    cell_decompose_result = print_cells();
    
    cout << " size of cells_v is " << cells_v.size() << endl;
    cout << " the value of cell 2 . pixel " << cells_v[2].pixel[0][0] << endl;
   // cout << " count for small cells is " << count << endl;
    namedWindow("image contours after");
    imshow("image contours after", masked_img);
    // imwrite("map_masked.png", masked_img);
    namedWindow("Cells");
    setMouseCallback("Cells", cell_img_mouse_callback, (void*)&cell_decompose_result);
    imshow("Cells before remove", map_before_remove);
    imshow("Cells",cell_decompose_result);

    coverage_path_planning();
    cout << " the size of robot path is " << robot.path.size() << endl;
    cout << " cell[0].edge_lt.back() is " << cells_v[0].edge_lt.back() << endl;
    Mat draw_path = cell_decompose_result.clone();
    namedWindow("Path");
    
    
    int index_cell = 1;
    for(int i = 0; i < robot.path.size() -1 ; ++i){
        // circle(draw_path,Point(robot.path[i].x+4,robot.path[i].y+4),4.5,Scalar(255,255,255),-1);
        if(i+1 != begin_of_cells[index_cell] - 1){
            line(draw_path, Point(robot.path[i].x+4,robot.path[i].y+4), Point(robot.path[i+1].x+4,robot.path[i+1].y+4),Scalar(255,255,255));
            imshow("Path", draw_path);
        }else if(i+1 == begin_of_cells[index_cell] - 1){
            ++index_cell;
        }
        waitKey(100);
    }
    
    // imwrite("map_cells.png", cell_decompose_result);
    
    /* robot.robot_pose_init(Point(105,197));
    cout << " the size of robot path is " << robot.path.size() << endl;
    int go_f = 1;
    while(go_f == 1){
        if(robot_move_down() == 0){
            go_f = 0;
        }
    }
    // while(robot_move_down() != 0){ cout << "+++++++++++++" << endl;};
    // cout << " robor move up result " << robot_move_down() << endl;
    cout << " the size of robot path is " << robot.path.size() << endl;
    cout << " the current robot pose is " << robot.pose << endl; */
    
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
void coverage_path_planning(){
    Point path_pt;
    int i;
    int j;
    for(auto it = cells_v.begin(); it != cells_v.end(); ++it){
        int start_f = 0;
        path_pt = it->edge_lt[0];
        ++path_pt.y;
        while(path_pt.y < it->edge_rt[0].y){
            if(check_map_rect(path_pt)){
                robot.robot_pose_init(path_pt);
                start_f = 1;
                break;
            }
            ++path_pt.y;
        }
        cout <<" the current cell is " << it-cells_v.begin() << " Area is " << it->area << endl;
        cout << " start_f is " << start_f << endl;
        if(start_f == 1){
            begin_of_cells.push_back(robot.path.end() - robot.path.begin());
            //*************************** New version algorithm **************
            
            
            
            
            
            
            
            //****************************************************************
            int finish = 0;
            int count = 20;
            
            // generate path for downward movement
            while(finish == 0){
            // while(count > 0){
                //============================ move down ===================
                // while(robot_move_down() != 0){};
                int move_down_R = 1;
                // int move_down_L = 1;
                /* while(robot_move_down() == 0 && move_down == 1){
                    for(i = 0; i < 10; ++i){
                        robot.path.push_back(Point(robot.path.back().x+i,robot.path.back().y));
                        if(robot_move_down() == 0){
                            robot.path.pop_back();
                        }else if(robot_move_down() != 0){
                            break;
                        }
                    }
                    if(i == 10){
                        move_down = 0;
                    }
                } */
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
                        /* for(i = 0; i < 10; ++i){
                            robot.path.push_back(Point(robot.path.back().x-i,robot.path.back().y));
                            if(robot_move_down() == 0){
                                robot.path.pop_back();
                            }else if(robot_move_down() != 0){
                                // robot.path.erase(robot.path.end()-2);
                                Point tmp(robot.path.back().x,robot.path.back().y);
                                robot.path.pop_back();
                                robot.path.pop_back();
                                robot.path.push_back(tmp);
                                break;
                            }
                        }
                        if(i == 10){
                            move_down_L = 0;
                        } */
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
                    /* for(i = 0; i < 10; ++i){
                        robot.path.push_back(Point(robot.path.back().x,robot.path.back().y-i));
                        if(robot_move_right()> 5 || robot.path.back().x == it->edge_lt.back().x-8){
                            move_right = 0;
                            cout << "+++++++++++++" << endl;
                            break;
                        }else{
                            robot.path.pop_back();
                        }
                    } */
                    
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
                /* while(robot_move_up() !=0){};
                move_up = 1;
                while(robot_move_up() == 0 && move_up == 1){
                    for(i = 0; i < 10; ++i){
                        robot.path.push_back(Point(robot.path.back().x+i,robot.path.back().y));
                        if(robot_move_up() == 0){
                            robot.path.pop_back();
                        }else if(robot_move_up() != 0){
                            break;
                        }
                    }
                    if(i == 10){
                        move_up = 0;
                    }
                }*/
                //============================ move up ===================
                int move_up_R = 1;
                int move_up_L = 1;
                try_count = 4;
                /* while(robot_move_down() == 0 && move_down == 1){
                    for(i = 0; i < 10; ++i){
                        robot.path.push_back(Point(robot.path.back().x+i,robot.path.back().y));
                        if(robot_move_down() == 0){
                            robot.path.pop_back();
                        }else if(robot_move_down() != 0){
                            break;
                        }
                    }
                    if(i == 10){
                        move_down = 0;
                    }
                } */
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
                        /* for(i = 0; i < 10; ++i){
                            robot.path.push_back(Point(robot.path.back().x-i,robot.path.back().y));
                            if(robot_move_up() == 0){
                                robot.path.pop_back();
                            }else if(robot_move_up() != 0){
                                robot.path.erase(robot.path.end()-2);
                                break;
                            }
                        }
                        if(i == 10){
                            move_up_L = 0;
                        } */
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
                    /* for(i = 0; i < 10; ++i){
                        robot.path.push_back(Point(robot.path.back().x,robot.path.back().y-i));
                        if(robot_move_right()> 5 || robot.path.back().x == it->edge_lt.back().x-8){
                            move_right = 0;
                            cout << "+++++++++++++" << endl;
                            break;
                        }else{
                            robot.path.pop_back();
                        }
                    } */
                    
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
                /* move_right =1;
                // while(move_right != 10 || robot.path.back().x < it->edge_lt.x-10){
                while(move_right ==1){        
                    for(i = 0; i < 10; ++i){
                        robot.path.push_back(Point(robot.path.back().x,robot.path.back().y+i));
                        if(robot_move_right() > 5 || robot.path.back().x == it->edge_lt.back().x-8){
                            move_right = 0;
                            break;
                        }else{
                            robot.path.pop_back();
                        }
                    }
                } */
                --count;
         
                         
                
            }
        }
    
    }

    
}
int robot_move_up(){
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
}
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
        cout <<" the current cell is " << it-cells_v.begin() << " the count is " << count << endl;
        cout << " the size of it->edge_lt.size() is " << it->edge_lt.size() << endl;
        cout << " size to remove cell is " << (float)count/it->edge_lt.size() << endl;
        if(((float)(count/it->edge_lt.size()) > (float)0.2) && (count > 0)){
            
            cout << " cell removed " << endl;
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