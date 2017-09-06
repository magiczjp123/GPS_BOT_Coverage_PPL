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
    int i,j;
    
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
    vector<vector<double> > Tabu;          // Ant path
    // for(i=0;i<m;++i) Tabu.push_back(zeroVec);
    int NC = 1;
    vector<vector<double> > R_best;
    for(i=0;i<NC_max;++i) R_best.push_back(zeroVec);

    while( NC < NC_max ){
        // Place ant randomly on cities
        vector<vector<double> > rand_pos;

        for(i=0;i<m;++i) rand_pos.push_back(i);
        random_shuffle(rand_pos.begin(),rand_pos.end());
        Tabu.push_back(rand_pos);
        // each ant move towards next city based on its probability
        for()



    }



    //=================== Display 2D vector ========================
    for(i = 0; i < n; ++i){
        cout << " the " <<i<<" row is ";
        for(int j = 0; j < n; ++j){
            cout << eta[i][j]<<setw(2) << " ";
        }
        cout << endl;
    }
    //==============================================================


    return 0;
}