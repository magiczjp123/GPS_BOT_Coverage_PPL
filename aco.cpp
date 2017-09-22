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
class v_i{
public:
    double num;
    int idx;
};
bool comparator( const v_i& l, const v_i& r) { return l.num < r.num;}

// ========================== out put index of sort =====================



int main(){

    vector<int> cells_order;
    // vector<int> cells_center_x = {41,37,54,25,7,2,68,71,54,83,64,18,22,83,91,25,24,58,71,74,87,18,13,82,62,58,45,41,44,4};
    // vector<int> cells_center_y = {94,84,67,62,64,99,58,44,62,69,60,54,60,46,38,38,42,69,71,78,76,40,40,7,32,35,21,26,35,50};
    vector<int> cells_center_x;
    vector<int> cells_center_y;
    int n = 118;
    vector<vector<int> > D_matrix;
    fstream myfile("D_matrix.txt", ios_base::in);
    int int_line;
    if(myfile.is_open()){
        cout << " D_matrix txt opened" << endl;
        /* for(int i = 0; i < 120; ++i){
            // getline(myfile, line);
            myfile >> int_line;
            cells_center_x.push_back(int_line);
        }
        for(int i = 0; i < 120; ++i){
            // getline(myfile, line);
            myfile >> int_line;
            cells_center_y.push_back(int_line);
        } */
        vector<int> tmp;
        for(int i = 0; i < n; ++i){
            vector<int> tmp;
            for(int j = 0; j < n; ++j){
                myfile >> int_line;
                tmp.push_back(int_line);
            }
            D_matrix.push_back(tmp);
        }
    }
    myfile.close();
    cout << " the size of D is " << D_matrix.size() << endl;

    /* cout << " the D matrix is " << endl;
    for(auto it = D_matrix.begin(); it != D_matrix.end(); ++it){
        cout << endl << " the " << it-D_matrix.begin() << " th row is " << endl;
        for(auto it2 = (*it).begin(); it2 != (*it).end(); ++it2){
            cout << *it2 << setw(4);
        }
        
    } */
    fstream myfile1("cells_center_xy.txt", ios_base::in);
    
    if(myfile1.is_open()){
        cout << " cells_center txt opened" << endl;
        for(int i = 0; i < n; ++i){
            // getline(myfile1, line);
            myfile1 >> int_line;
            // cout << " int_line is " << int_line << endl;
            cells_center_x.push_back(int_line);
        }
        for(int i = 0; i < n; ++i){
            // getline(myfile1, line);
            myfile1 >> int_line;
            cells_center_y.push_back(int_line);
        }
        
    }
    myfile1.close();
    
    /* cout << " cells_center_x is " << endl;
    for(int i = 0; i < cells_center_x.size(); ++i){
        cout << cells_center_x[i] << setw(6);
    }
    cout << endl;
    cout << " cells_center_y is " << endl;
    for(int i = 0; i < cells_center_y.size(); ++i){
        cout << cells_center_y[i] << setw(6);
    }
    cout << endl; */
    int NC_max = 50;
    int m = D_matrix.size();      // number of ants
    // int n = m;      // number of cities
    double alpha = 1.5;
    double beta = 2;
    double rho = 0.1;
    double Q = pow(10,6);
    double E = 1.5;
    vector<double> zeroVec(n,0.0);
    vector<double> onesVec(n,1.0);
    vector<int> zeroVec_i(n,1);
    int i,j,k;
    char bar[102];
    bar[0] = 0;
    
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
            tmp_eta.push_back((double) 1./(D_matrix[i][j]));
        }
        D.push_back(tmp_D);
        eta.push_back(tmp_eta);
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
        /* cout << " the ant_mileage is ";
        for(i = 0; i < ant_mileage.size(); ++i){
            cout << ant_mileage[i] << " ";
        }
        cout << endl; */

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
        //=============================== Ranked Ant system ==================
        vector<int> ranks(ant_mileage.size(),0);        // Ranks array to store ranks of ant mileage
        vector<double> ant_mileage_sort = ant_mileage;
        vector<int> ranks_value;

        sort(ant_mileage_sort.begin(),ant_mileage_sort.end());
        int count = 1;
        ranks_value.push_back(count);
        // produce sorted rank values according to the sorted ant mileage
        for(i = 1; i < ant_mileage_sort.size(); ++i){
            if(ant_mileage_sort[i] > ant_mileage_sort[i-1]){
                ++count;
                ranks_value.push_back(count);
            }else{
                ranks_value.push_back(count);
            }
        }
        ++count;        // increment count to substract
        for(i = 0; i < ranks.size(); ++i){
            ranks[i] = ranks_value[ find(ant_mileage_sort.begin(), ant_mileage_sort.end(), ant_mileage[i]) - ant_mileage_sort.begin()];
            ranks[i] = count - ranks[i];
        }
        /* cout << " the ranks vector is " << endl;
        for(auto it = ranks.begin(); it!= ranks.end(); ++it){
            cout << " " << *it << " ";
        }
        cout << endl; */
        //===================================================================
        
        for(i = 0; i < m; ++i){
            for(j = 0; j < n-1 ; ++j){
                Delta_Tau[ Tabu[j][i] ][ Tabu[j+1][i] ] += (Q/ant_mileage[i]) * ranks[i];
            }
            Delta_Tau[ Tabu[n-1][i] ][ Tabu[0][i] ] += (Q/ant_mileage[i]) * ranks[i];
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
        
        
        // ============================= progress bar========================
        // cout << " the current iteration is " << NC << endl;
        double progress = (double)NC/NC_max;
        int barWidth = 70;
        cout << "[";
        int pos = barWidth * progress;
        for (int i = 0; i < barWidth; ++i) {
            if (i < pos) std::cout << "=";
            else if (i == pos) std::cout << ">";
            else std::cout << " ";
        }
        std::cout << "] " << int(progress * 100.0) << " %\r";
        std::cout.flush();
    }
    cout << endl;

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

    

    ofstream ofile;
    ofile.open("aco_path.txt");
    for(i = 0; i < R_best[best_i].size(); ++i){
        ofile<< R_best[best_i][i] << endl;
    }
    ofile.close();

    cout << " the shortest travel distance is " << L_best[best_i] << endl;
    cout << " the shortest path is " << endl;
    for(i = 0; i < R_best[best_i].size(); ++i){
        cout << " ( " << R_best[best_i][i] << " ) " << endl;
        // cout << " ( " << cells_center_x[R_best[best_i][i]] << " , " << cells_center_y[R_best[best_i][i]] << " ) " << endl;
    }

    cout << " cells_center_x is " << endl;
    for(i = 0; i << cells_center_x.size(); ++i){
        cout << cells_center_x[i] << setw(6);
    }
    cout << endl;
    cout << " cells_center_y is " << endl;
    for(i = 0; i << cells_center_y.size(); ++i){
        cout << cells_center_y[i] << setw(6);
    }
    cout << endl;
    // Mat path_plot(480, 736, CV_8UC3, Scalar(255,255,255));
    Mat path_plot = imread("map_cells.png");
    RNG rng(12345);
    for(i = 0; i < R_best[best_i].size()-1; ++i){
        Scalar color = Scalar(rng.uniform(0,220), rng.uniform(0, 220), rng.uniform(0, 220));
        line(path_plot, Point(cells_center_x[R_best[best_i][i]], cells_center_y[R_best[best_i][i]]), Point(cells_center_x[R_best[best_i][i+1]], cells_center_y[R_best[best_i][i+1]]), color);
        circle(path_plot, Point(cells_center_x[R_best[best_i][i+1]], cells_center_y[R_best[best_i][i+1]]),4,color,-1);
    }
    Scalar color = Scalar(rng.uniform(0,220), rng.uniform(0, 220), rng.uniform(0, 220));
    line(path_plot, Point(cells_center_x[R_best[best_i][i]], cells_center_y[R_best[best_i][i]]), Point(cells_center_x[R_best[best_i][0]], cells_center_y[R_best[best_i][0]]), color);
    circle(path_plot, Point(cells_center_x[R_best[best_i][0]], cells_center_y[R_best[best_i][0]]),4,color,-1);


    namedWindow("Plot", WINDOW_FREERATIO);
    imshow("Plot", path_plot);
    

    waitKey(0);
    return 0;
}