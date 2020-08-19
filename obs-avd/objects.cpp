#pragma once
#include "pvector.h"

//Class Vehicle
class vehicle{
protected:
    PVector velocity, acceleration, position;
    BasisVector orientation;
    float r, speed, maxspeed, maxforce;
public:
    vehicle(){}
    vehicle(float x, float y);
    void update(float time);
    PVector getvelocity(){
        return velocity;
    }
    PVector getpose(){
        return position;
    }
    PVector getorientation(){
        return orientation;
    }
    void setVelocity(PVector v){
        velocity = v;
    }
    void applyForce(PVector force);
    PVector getLocation(){
        return position;
    }
    void seek(PVector target);
    void arrive(PVector target, float rad);
    std::pair<float, float> display();
    void display(int a);
};

vehicle::vehicle(float x, float y){
    position  = PVector(x, y);
    acceleration  = PVector(0,0);
    velocity = PVector(0,0);
    orientation = BasisVector(PVector(1, 0, 0), PVector(0, 0, 1));
    r=6;
    speed = maxspeed = 1;
    maxforce = 0.5;
}

void vehicle::update(float time = 1.0f){
    if ((velocity.getx() == velocity.gety()) and (velocity.getx() == 0))
        cout << "Null V" << endl;
    velocity.add(acceleration.into(time));
    // velocity.setMag(speed);
    orientation.update(velocity);
    // velocity.limit(maxspeed);
    position.add(velocity.into(time));
    acceleration.mult(0);
}

void vehicle::applyForce(PVector force){
    acceleration.add(force);
}

std::pair<float, float> vehicle::display(){
    return make_pair(position.getx(), position.gety());    
}

//Class Autonomous Vehicle

class autonomous_vehicle: public vehicle{
protected:
    float maxspeed;
public:
    autonomous_vehicle(float x, float y){
        vehicle(x, y);
        acceleration =PVector(0, 0);
        velocity = PVector(0, 0);
        position = PVector(x, y, 0);
        vehicle::maxspeed = 1;
        maxspeed = vehicle::maxspeed/1.5;
        speed = maxspeed;
        r=10;
        maxforce = 0.5;
    }
    // PVector followPath(path p);
    void seek(PVector target);
    void arrive(PVector target);
    
    bool inProximity(PVector target){
        return (this->position.dist(target) < r);
    }

};


void autonomous_vehicle::seek(PVector target){
    PVector vdesired = target.sub(target, position);
    PVector steering = target.sub(vdesired, velocity);
    steering.setMag(maxforce);
    applyForce(steering);
    // if(this->inProximity(target))
        mapx(speed, maxspeed, 0, r, 0.01, this->position.dist(target));
}


void autonomous_vehicle::arrive(PVector target){
    mapx(speed, maxspeed, 0, 5, 0, position.dist(target));   
}