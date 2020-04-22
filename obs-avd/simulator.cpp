//Append "-std=c++11 -I/usr/include/python2.7 -lpython2.7" to compile when using matplotlibcpp

#pragma once
#include "GaussianPF_PathPlanning.cpp"
#include <stdlib.h>
#include "ODG-PF.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

vector<float> get_scan_data(vector<rect> env, float x, float y){
    vector<float> res;
    for(int i=0;i<360;i++){
        float theta = index_to_angle(i);
        float max = MAX_DIST;
        for(rect ob:env){
            float d = ob.get_min_dist(x, y, theta);
            if(d < max)  max = d;
        }
        res.push_back(max);
    }
    return res;
}  

const float START_X = 0;
const float START_Y = 0;
const float GOAL_X = 5;
const float GOAL_Y = 5; 

const int eps = 0.6; //Epsilon permitted error in reaching the goal

float find_path(float gam, bool showPlot = true){

    Gamma = gam;
    vector<rect> env;

    // env.push_back(rect(1, -1, 3, 1));
    // env.push_back(rect(1, -4, 3, -2));


    env.push_back(rect(1,1, 2, 2));
    env.push_back(rect(1, 5, 2, 4));
    env.push_back(rect(3, 3,4, 4));
    env.push_back(rect(3, 0, 4, 1));

    float curx, cury;
    vector<float> x(1), y(1), u(1), v(1); // (u[i], v[i]) is the header dir vector at point (x[i],y[i])

    curx = START_X;
    cury = START_Y;
    int iter = 0;
    float reward = 0;
    vector<float> min_dist;
    while( iter++<100 and sqrt((pow(curx-GOAL_X, 2)+ pow(cury-GOAL_Y, 2)) >= 0.4)){
        x.push_back(curx);
        y.push_back(cury);
        vector<float> data = get_scan_data(env, curx,cury);
        float goal_angle = get_goal_angle(curx, cury, GOAL_X, GOAL_Y);
        vector<obstacle> obs = get_obstacles(data);
        for(auto r:env){
            if(r.on_line(curx, cury))
                reward -= 100;
        }
        if(showPlot)
            cout << "Goal angle : " << goal_angle << endl;
        auto head_theta = get_header_rad(data, goal_angle, showPlot);
        if(showPlot)
            cout << "Header angle : "<< head_theta << endl;
        u.push_back(cos(head_theta));
        v.push_back(sin(head_theta));
        curx += cos(head_theta)*0.2;
        cury += sin(head_theta)*0.2;
        min_dist.push_back(pow(pow(curx-GOAL_X, 2)+ pow(cury-GOAL_Y, 2), 0.5));

        // Plotting the steps
        if(showPlot){
            plt::clf(); // Clear previous plot
            for(auto r:env)
                r.draw();
            plt::quiver(x, y, u, v);
            plt::xlim(START_X-1, GOAL_X+1);
            plt::ylim(START_Y-1, GOAL_Y+1);
            plt::pause(0.01);
        }
        
    }       
    float m = min_dist[std::distance(min_dist.begin(), std::min_element(min_dist.begin(), min_dist.end()))];
    if (m <= 0.40)
        reward += 1;
    if(showPlot){
        plt::show();
        cout << "Min = " << m << endl;
    }
    reward += (101-iter)/5;
    reward -= min_dist[min_dist.size()-1];
    return reward;
}


float find_path_mobile(float gam, bool showPlot = true){

    Gamma = gam;
    vector<rect> env;

    float yy = 2 ;
    env.push_back(rect(2, yy, 3,yy+1));
    env.push_back(rect(4, yy, 5,yy+1));

    float curx, cury;
    vector<float> x(1), y(1), u(1), v(1); // (u[i], v[i]) is the header dir vector at point (x[i],y[i])

    curx = START_X;
    cury = START_Y;
    int iter = 0;
    float reward = 0;
    vector<float> min_dist;
    while( iter++<100 and sqrt((pow(curx-GOAL_X, 2)+ pow(cury-GOAL_Y, 2)) >= 0.4)){
        x.push_back(curx);
        y.push_back(cury);
        vector<float> data = get_scan_data(env, curx,cury);
        float goal_angle = get_goal_angle(curx, cury, GOAL_X, GOAL_Y);
        vector<obstacle> obs = get_obstacles(data);
        for(auto r:env){
            if(r.on_line(curx, cury))
                reward -= 100;
        }
        if(showPlot)
            cout << "Goal angle : " << goal_angle << endl;
        auto head_theta = get_header_rad(data, goal_angle, showPlot);
        if(showPlot)
            cout << "Header angle : "<< head_theta << endl;
        u.push_back(cos(head_theta));
        v.push_back(sin(head_theta));
        curx += cos(head_theta)*0.2;
        cury += sin(head_theta)*0.2;
        min_dist.push_back(pow(pow(curx-GOAL_X, 2)+ pow(cury-GOAL_Y, 2), 0.5));

        // Plotting the steps
        if(showPlot){
            plt::clf(); // Clear previous plot
            for(auto r:env)
                r.draw();
            plt::quiver((vector<float>){x[x.size()-1]}, (vector<float>){y[y.size()-1]}, (vector<float>){u[u.size()-1]}, (vector<float>){v[v.size()-1]});
            plt::xlim(START_X-1, GOAL_X+1);
            plt::ylim(START_Y-1, GOAL_Y+1);
            plt::pause(0.01);
        }
        env[0] = rect(2, yy + 2.5*sin(0.2*iter), 3, yy + 1 +2.5*sin(0.2*iter));
        env[1] = rect(4, yy + 2.5*sin(0.2*iter), 5, yy + 1+ 2.5*sin(0.2*iter));
    }       
    
    float m = min_dist[std::distance(min_dist.begin(), std::min_element(min_dist.begin(), min_dist.end()))];
    if (m <= 0.40)
        reward += 1;
    if(showPlot){
        plt::show();
        cout << "Min = " << m << endl;
    }
    reward += (101-iter)/5;
    reward -= min_dist[min_dist.size()-1];
    return reward;
}