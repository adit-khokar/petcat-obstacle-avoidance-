#include <math.h>
#include <vector>

#define PI 3.14159265

using namespace std;

// constants
const float Gamma;    //for attractive field to g (Had to change from gamma as that's a predefined var in mathcalls.h)
const float LC = 1; // Least count of angle for the sensor
const float MAX_DIST = 2; // Threshold Distance for the 
const float WIDTH = 0.3;//Width of the robot



//sample input array contaioning distanse values at 
vector<float> input[360];


//sample output array for storing potential field values
vector<float> field[360];


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
    phi = 2*atan(2 * (d * (tan(phi/2)+ (w/2) ) ));
}

void obstacle::compute_field(vector<float>& field)
{
    float theta = index_to_angle(theta);
    float phi = index_to_angle(phi);
    for(int i=0;i < field.size();i++){
        float angle = index_to_angle(i);
        field[i] = A*exp(-((theta-angle)*(theta-angle))/(2*phi*phi));
    }
}


//will need more work
float index_to_angle(int i){
    return (PI*i)/180;
}


void goal_field(vector<float>& field, float goal_angle){
    for (int i = 0; i < field.size(); i++)
    {
        float angle = index_to_angle(i);
        field[i] = Gamma*abs(angle - goal_angle);
    }
    
}