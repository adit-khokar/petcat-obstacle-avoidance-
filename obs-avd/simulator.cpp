// #pragma once
//Append "-std=c++11 -I/usr/include/python2.7 -lpython2.7" to compile when using matplotlibcpp
#include "GaussianPF_PathPlanning.cpp"
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
    float eps = 0.8;
    
    if( abs((x2-x1)*(y+y2) - (y2-y1)*(x+x2)) <= eps )
        return true;
    if( abs((x3-x2)*(y+y3) - (y3-y2)*(x+x3)) <= eps )
        return true;
    if( abs((x4-x3)*(y+y4) - (y4-y3)*(x+x4)) <= eps )
        return true;
    if( abs((x1-x4)*(y+y1) - (y1-y4)*(x+x1)) <= eps )
        return true;
    return false;
}

float rect::get_min_dist(float x, float y, float theta){
    float d = 0;
    for(d = 0;d<MAX_DIST;d+=0.1){
        float xi, yi;
        xi = x + d*cos(theta); 
        yi = y + d*sin(theta);

        if(this->on_line(xi, yi))
            break;

    }
    return d;
}

void rect::draw(){
    vector<float> X = {x1, x2, x3, x4, x1};
    vector<float> Y = {y1, y2, y3, y4, y1};
    plt::plot(X, Y);
}

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

const int START_X = 1;
const int START_Y = 1;
const int GOAL_X = 4;
const int GOAL_Y = 3; 

const int eps = 0.2; //Epsilon permitted error in reaching the goal

int main(){

    vector<rect> env;
    env.push_back(rect(2, 2, 3, 2.5));
    for(auto r:env){
        r.draw();
    }
    float curx, cury;
    vector<float> x, y, u, v; // (u[i], v[i]) is the header dir vector at point (x[i],y[i])

    curx = START_X;
    cury = START_Y;
    
    vector<float> first_scan = get_scan_data(env, 1, 1);
    for(auto x:first_scan){
        cout  << x <<" ";
    }

    while((curx<GOAL_X)and (cury<GOAL_Y)){
        x.push_back(curx);
        y.push_back(cury);
        vector<float> data = get_scan_data(env, curx,cury);
        float goal_angle = get_goal_angle(curx, cury, GOAL_X, GOAL_Y);
        auto head_theta = get_header_rad(data, goal_angle);
        assert(head_theta>0);
        u.push_back(cos(head_theta));
        v.push_back(sin(head_theta));
        curx += cos(head_theta)*0.1;
        cury += sin(head_theta)*0.1;
    }
    plt::quiver(x, y, u, v);
    plt::show();

    return 0;
}