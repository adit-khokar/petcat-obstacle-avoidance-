// int argMax = std::distance(x.begin(), std::max_element(x.begin(), x.end()));
# pragma once

#include <math.h>
#include <vector>
#include <iostream>
using namespace std;
#define PI 3.14159265

using namespace std;

// constants
const float Gamma = 5;    //for attractive field to g (Had to change from gamma as that's a predefined var in mathcalls.h)
const float LC = 1; // Least count of angle for the sensor
const float MAX_DIST = 2; // Threshold Distance for the algo
const float WIDTH = 0.3;//Width of the robot

//sample input array contaioning distanse values at 
vector<float> input[360];


//sample output array for storing potential field values
vector<float> field[360];

//will need more work
float index_to_angle(int i){
    return (PI*i)/180;
}

float angle_to_index(float a){
    return (a*180)/PI;
}


class obstacle
{
private:
    float d;    //average distance to each obstacle 
    float phi;  //the angle occupied by it
    // float sigma;    //half of the angle occupied by the obstacle considering for size of bot
    float theta;    //center angle for obstacle
    float A ;

public:
    obstacle(float d_in, float phi_in,float theta_in);  //constructor
    void increase_width(float w); // 
    void compute_field(vector<float>& field);   //compute and add field to the total field
    float get_theta(){ return theta;}
};

obstacle::obstacle(float d_in, float phi_in,float theta_in)
{
    d = d_in;
    phi = phi_in;
    theta = theta_in;
    A = (MAX_DIST-d)*exp(0.5);
    // sigma = atan2((d*(tan(phi/2))+width/2),d);
}

void obstacle::increase_width(float w){
    phi = index_to_angle(phi);
    phi = 2*atan(2 * (d * (tan(phi/2)+ (w/2) ) ));
    phi = angle_to_index(phi);
}

void obstacle::compute_field(vector<float>& field)
{
    float theta = index_to_angle(theta);
    float phi = index_to_angle(phi); 
    for(int i=0;i < field.size();i++){
        float angle = index_to_angle(i);
        field[i] += A*exp(-((theta-angle)*(theta-angle))/(2*phi*phi));
    }
}


float get_goal_angle(float x, float y, float goal_x, float goal_y){
    return atan((goal_y-y)/(goal_x-x));
}

void goal_field(vector<float> &field, float goal_angle){
    for (int i = 0; i < field.size(); i++)
    {
        float angle = index_to_angle(i);
        auto temp = Gamma*abs(angle - goal_angle);
        field[i] += temp;
    }
    
}