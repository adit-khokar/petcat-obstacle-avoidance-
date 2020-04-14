#include <math.h>
#include <vector>

#define PI 3.14159265

using namespace std;

// constants
float threshold;    //for ditection of object
float Gamma;    //for attractive field to g (Had to change from gamma as that's a predefined var in mathcalls.h)
float D_max = threshold;    // the maximum detection rangeof the range sensor(I think they are refering to the threshold value)
float width;


//sample input array contaioning distanse values at 
vector<float> input[360];


//sample output array for storing potential field values
vector<float> field[360];


class obstacle
{
private:
    float d;    //average distance to each obstacle 
    float phi;  //the angle occupied by it
    float sigma;    //half of the angle occupied by the obstacle considering for size of bot
    float theta;    //center angle for obstacle

public:
    obstacle(float d_in, float phi_in,float theta_in);  //constructor
    void increase_width(float w); // 
    void compute_field(vector<float> field);   //compute and add field to the total field
};

obstacle::obstacle(float d_in, float phi_in,float theta_in)
{
    d = d_in;
    phi = phi_in;
    theta = theta_in;
    // sigma = atan2((d*(tan(phi/2))+width/2),d);
}

void obstacle::increase_width(float w){
    phi = 2*atan(2 * (d * (tan(phi/2)+ (w/2) ) ));
}

void obstacle::compute_field(vector<float> field)
{
}
