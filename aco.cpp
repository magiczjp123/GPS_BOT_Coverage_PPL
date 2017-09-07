#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <cmath>
#define Pi 3.1415926
using std::string;
using namespace cv;
using namespace std;




int main(){

    vector<int> cells_order;
    vector<int> cells_center_x = {41,37,54,25,7,2,68,71,54,83};
    vector<int> cells_center_y = {94,84,67,62,64,99,58,44,62,69};
    int NC_max = 50;
    int m = cells_center_x.size();      // number of ants
    int n = cells_center_x.size();      // number of cities
    double alpha = 1.5;
    double beta = 2;
    double rho = 0.1;
    double Q = pow(10,6);
    vector<double> zeroVec(n,0.0);
    vector<double> onesVec(n,1.0);
    vector<int> zeroVec_i(n,1);
    int i,j,k;
    
    vector<vector<double> > D;             // distance for cities
    for(i = 0; i < n; ++i) D.push_back(zeroVec);
    vector<vector<double> > eta;
    for(i = 0; i < n; ++i) eta.push_back(zeroVec);
    //====================== initialize distance matrix and its inverse
    for(i =0; i < n; ++i){
        for(j = 0; j < n; ++j){
            if(i!=j){
                D[i][j] = pow(pow(cells_center_x[i] - cells_center_x[j],2) + pow(cells_center_y[i] - cells_center_y[j],2) ,0.5);
                eta[i][j] = 1./D[i][j];
            }
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
                // Cumulate probability
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
        for(i = 0; i < n; ++i){
            cout << " the " <<i<<" row is ";
            for(int j = 0; j < n; ++j){
                cout << Delta_Tau[i][j]<<setw(2) << " ";
            }
            cout << endl;
        }

        for(i = 0; i < n; ++i){
            for(j = 0; j < n; ++j){
                Tau[i][j] *= (1.0-rho);
                Tau[i][j] += Delta_Tau[i][j];
            }
        }
        ++NC;

    }

    cout << " R_best" << endl;
    for(i = 0; i < R_best.size(); ++i){
        cout << " the " <<i<<" row is ";
        for(int j = 0; j < n; ++j){
            cout << R_best[i][j]<<setw(2) << " ";
        }
        cout << endl;
    }
    cout << "***********************************"<< endl;

    cout << "L_best" << endl;
    for(i = 0; i < L_best.size(); ++i){
        cout << " the " << i << " row is " << L_best[i] << endl;

    }

    Mat path_plot(120, 120, CV_8UC3);
    for(i = 0; i < R_best[46].size()-1; ++i){
        line(path_plot, Point(cells_center_x[R_best.back()[i]], cells_center_y[R_best.back()[i]]), Point(cells_center_x[R_best.back()[i+1]], cells_center_y[R_best.back()[i+1]]), Scalar(255,255,255));

    }
    imshow("Plot", path_plot);
    

    waitKey(0);
    return 0;
}