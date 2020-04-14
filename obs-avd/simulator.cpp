// #pragma once
//Append "-std=c++11 -I/usr/include/python2.7 -lpython2.7" to compile when using matplotlibcpp
#include "GaussianPF_PathPlanning.cpp"
#include "ODG-PF.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

// Drawing Boxes for all obstacles in the env

// Showing Start point and end point through scatter plot

// Showing optimal header angles at all points on the path through a quiver plot

const int START_X = 1;
const int START_Y = 1;
const int GOAL_X = 4;
const int GOAL_Y = 3; 

const int eps = 0.2; //Epsilon permitted error in reaching the goal


// TODO //
vector<float> get_scan_dat(float x, float y); // Getting the reading at point (x ,y)

int main(){
    float curx, cury;
    vector<float> x, y, u, v; // (u[i], v[i]) is the header dir vector at point (x[i],y[i])

    curx = START_X;
    cury = START_Y;
    while((abs(curx-GOAL_X)>eps) and (abs(cury-GOAL_Y)>eps)){
        x.push_back(curx);
        y.push_back(cury);
        vector<float> data = get_scan_dat(curx,cury);
        float goal_angle = get_goal_angle(curx, cury, GOAL_X, GOAL_Y);
        auto head_theta = get_header_rad(data, goal_angle);
        u.push_back(cos(head_theta));
        v.push_back(sin(head_theta));
        curx += cos(head_theta);
        cury += sin(head_theta);
    }

    plt::quiver(x, y, u, v);


    return 0;
}