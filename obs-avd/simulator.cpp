// #pragma once
//Append "-std=c++11 -I/usr/include/python2.7 -lpython2.7" to compile when using matplotlibcpp
#include "GaussianPF_PathPlanning.cpp"
#include <stdlib.h>
#include "ODG-PF.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

class rect{
    float x1, x2, x3, x4;
    float y1, y2, y3, y4;
 public:
    rect(float x1_, float y1_, float x3_, float y3_){   
        x1 = x1_;
        x2 = x3_;
        x3 = x3_;
        x4 = x1_;
        y1 = y1_;
        y2 = y1_;
        y3 = y3_;
        y4 = y3_;
    }
    bool on_line(float x, float y);
    float get_min_dist(float x, float y, float theta);
    void draw();

};

bool rect::on_line(float x, float y){
    float eps = 0.2;
    float cx = (x1+x2)/2;
    float cy = (y1+y4)/2;
    float rad = abs(x1-x2)/2 + eps;
    if((x-cx)*(x-cx)+ (y-cy)*(y-cy) <= rad*rad)
        return true;
    
    return false;
}

float rect::get_min_dist(float x, float y, float theta){
    float d = 0;
    if(this->on_line(x, y)){
        float cx = (x1+x2)/2;
        float cy = (y1+y4)/2;
        float atoc = get_goal_angle(x, y,cx, cy);
        return abs(theta-atoc)/PI;
    }
    for(d = 0; d < MAX_DIST; d+=0.001){
        float xi, yi;
        xi = x + d*cos(theta); 
        yi = y + d*sin(theta);

        if(this->on_line(xi, yi))
            break;

    }
    return d;
}

void rect::draw(){
    draw_circle((x1+x2)/2, (y1+y3)/2, abs(x2-x1)/2);
}

vector<float> res(360, 2);

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

const float START_X = 5;
const float START_Y = 5;
const float GOAL_X = 0;
const float GOAL_Y = 0; 

const int eps = 0.6; //Epsilon permitted error in reaching the goal

int main(int argc, char** argv){


    if(argc>1){
        Gamma = atof(argv[1]);
    }
    vector<rect> env;

    env.push_back(rect(1,1, 2, 2));
    env.push_back(rect(0, 5, 1, 4));
    env.push_back(rect(3, 3,4, 4));
    env.push_back(rect(4, 0, 5, 1));

    
    float curx, cury;
    vector<float> x, y, u, v; // (u[i], v[i]) is the header dir vector at point (x[i],y[i])

    curx = START_X;
    cury = START_Y;
    int iter = 0;
    vector<float> min_dist;
    while( iter++<100 and sqrt((pow(curx-GOAL_X, 2)+ pow(cury-GOAL_Y, 2)) >= 0.09)){
        x.push_back(curx);
        y.push_back(cury);
        vector<float> data = get_scan_data(env, curx,cury);
        float goal_angle = get_goal_angle(curx, cury, GOAL_X, GOAL_Y);
        cout << "Goal angle : " << goal_angle << endl;
        auto head_theta = get_header_rad(data, goal_angle);
        cout << "Header angle : "<< head_theta << endl;
        u.push_back(cos(head_theta));
        v.push_back(sin(head_theta));
        curx += cos(head_theta)*0.1;
        cury += sin(head_theta)*0.1;
        min_dist.push_back(pow(pow(curx-GOAL_X, 2)+ pow(cury-GOAL_Y, 2), 0.5));
    }
    plt::quiver(x, y, u, v);
    for(auto r:env){
        r.draw();
    }
    plt::show();           
    cout << "Minimum distance from the goal = " << min_dist[std::distance(min_dist.begin(), std::min_element(min_dist.begin(), min_dist.end()))] << endl;
    cout << "Steps = "<<iter << endl;
    return 0;
}
