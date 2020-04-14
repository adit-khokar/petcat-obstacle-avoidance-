
#include "ODG-PF.h"
#include <iostream>
#include <numeric>
#include <cassert>
using namespace std;

// const float LC = 1; // Least count of angle for the sensor
// const float MAX_DIST = 2; // Threshold Distance for the 
// const float WIDTH = 0.3;//Width of the robot
//moved to ODG-PF.h


vector<obstacle> get_obstacles(vector<float> polar_dat){
    vector<obstacle> obs;
    int flag = 0; // represents whether an obs is being read
    float theta1;
    for(int i=0;i<polar_dat.size();i++){
        auto dist = polar_dat[i];
        switch(flag - (dist<MAX_DIST)){
            case -1 : // Start a new  obstacle
                flag = 1;
                theta1 = i;
                break;
            case 1: // End of current obstacle
              {  flag = 0;
                float theta2 = i-1;
                float anglular_width = theta2-theta1;
                float mean_angle = (theta2+theta1)/2;
                float dist = accumulate(&polar_dat[theta1], &polar_dat[theta2], 0.0f) / (theta2-theta1);
                assert(anglular_width >= 0);
                // if(temp.anglular_width > 0) // Ignoring noise?
                    obs.push_back(obstacle(dist, anglular_width, mean_angle));
                break;}
            default: // Continue as is for 0, 2
                break;
        }
    }
    return obs;
}

vector<obstacle> process_obs(vector<obstacle> obs){
    for (obstacle o : obs){
        o.increase_width(WIDTH);
    }
    return obs;
}

vector<float> polar_data(360); // vector of size 360 : length of free path along each dir
vector<float> potential(360); // Final potential at each angle

int main(int argc, char** argv){
    vector<obstacle> Obs = get_obstacles(polar_data);
    Obs = process_obs(Obs);
    
    return 0;
}