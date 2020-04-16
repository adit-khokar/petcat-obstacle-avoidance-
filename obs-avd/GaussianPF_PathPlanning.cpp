# pragma once

#include "ODG-PF.h"
#include <iostream>
#include <bits/stl_algo.h>
#include <bits/stdc++.h>
#include <numeric>
#include <cassert>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
using namespace std;


vector<obstacle> get_obstacles(vector<float> polar_dat){
    vector<obstacle> obs;
    int flag = 0; // represents whether an obs is being read
    float theta1;
    for(int i=0;i<polar_dat.size();i++){
        auto dist = polar_dat[i];
        switch(flag - (int)(dist<MAX_DIST)){
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

int get_best_header(vector<float> potential){
    int header = std::distance(potential.begin()+1, std::min_element(potential.begin(), potential.end()));
    return header;
}

vector<float> polar_data(360); // vector of size 360 : length of free path along each dir
vector<float> potential(360); // Final potential at each angle

float get_header_rad(vector<float> polardat, float goal_angle){
    float header;
    
    auto obstacles = get_obstacles(polardat);
    obstacles = process_obs(obstacles);
    // For debugging
    for(auto a:obstacles){
        cout << "Obstacle is " << a.get_dist()<< "m away at angle "<<a.get_theta()<<" with width "<<a.get_phi()<<" \n";
    }
    cout << "Num obstacles = " << obstacles.size() << endl;
    vector<float> potfield(360, 0);
    for(auto ob: obstacles){
        ob.compute_field(potfield);
    }
    goal_field(potfield, goal_angle);
    header = get_best_header(potfield);

    return index_to_angle(header);
}