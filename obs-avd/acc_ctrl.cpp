// #include "pvector.h"
#include "objects.cpp"
#include "GaussianPF_PathPlanning.cpp"
#include "matplotlibcpp.h"
#include <chrono>
#include "simulator.cpp"
using namespace std;
namespace plt = matplotlibcpp;

float T = 1.0f;

// Test interface
int seek_test(){
    autonomous_vehicle a(0, 0);
    PVector target(5, 5);
    a.setVelocity(PVector(5, 0));
    vector<float> x, y, u, v;
    while (a.getpose().dist(target)>=0.1){
        plt::clf();
        a.seek(target);
        a.update();
        x = {a.display().first};
        y = {a.display().second};
        u = {a.getorientation().getx()*10};
        v = {a.getorientation().gety()*10};
        // x.push_back(a.display().first);
        // y.push_back(a.display().second);
        cout << x[x.size()-1] << " " << y[y.size()-1] << endl;
        plt::quiver(x, y, u, v);
        // plt::plot(x, y);
        // plt::show();
        plt::pause(0.01);
    }
    vector<float> c{1, 2, 3};
    vector<float> b{5, 4, 6};
    // plt::plot(c, b);
    plt::show();

    return 0;
}

float find_path_acc(float gam, bool showPlot = true){

    // GOAL_X  = 5;
    // GOAL_Y = 5;
    Gamma = gam;
    vector<rect> env;

    // env.push_back(rect(1, -1, 3, 1));
    // env.push_back(rect(1, -4, 3, -2));
    autonomous_vehicle bot(0, 0);


    env.push_back(rect(1,1, 2, 2));
    env.push_back(rect(1, 5, 2, 4));
    env.push_back(rect(3, 3,4, 4));
    // env.push_back(rect(6, 7, 7, 8));
    env.push_back(rect(3, 0, 4, 1));

    float curx, cury;
    vector<float> x(1), y(1), u(1), v(1); // (u[i], v[i]) is the header dir vector at point (x[i],y[i])

    curx = 0;
    cury = 0;
    int iter = 0;
    float reward = 0;
    vector<float> min_dist;

    // An iteration for time:
    {
        auto start = std::chrono::steady_clock::now();
        
        vector<float> data = get_scan_data(env, curx,cury);
        float goal_angle = get_goal_angle(curx, cury, GOAL_X, GOAL_Y);
        vector<obstacle> obs = get_obstacles(data);
        for(auto r:env){
            if(r.on_line(curx, cury))
                reward -= 100;
        }
        auto cmd = get_header_rad(data, goal_angle, showPlot);
        auto head_theta = cmd.first;
        auto vel = cmd.second;
        // u.push_back(cos(head_theta));
        // v.push_back(sin(head_theta));
        // curx += cos(head_theta)*0.2;
        // cury += sin(head_theta)*0.2;
        PVector vel_req = PVector().rotate(PVector(vel, 0), head_theta);
        // vel_req.setMag(0.1);
        PVector acc = vel_req.minus(bot.getvelocity());
        // acc.setMag(0.2);
        bot.applyForce(acc);
        bot.update(T);
        cout << "*\tSpeed = " << bot.getvelocity().modulus() << endl;
        min_dist.push_back(pow(pow(curx-GOAL_X, 2)+ pow(cury-GOAL_Y, 2), 0.5));

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<float> iteration_time = end-start;
        std::cout << "elapsed time: " << iteration_time.count() << "s\n";
        T = iteration_time.count();
        // int z;
        // cin >> z;
    }


    while( iter++<1000 and sqrt((pow(curx-GOAL_X, 2)+ pow(cury-GOAL_Y, 2)) >= 0.2)){

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
        
        auto cmd = get_header_rad(data, goal_angle, showPlot);
        auto head_theta = cmd.first;
        auto vel = cmd.second;

        if(showPlot)
            cout << "Header angle : "<< head_theta << endl;
        PVector vel_req = PVector().rotate(PVector(vel, 0), head_theta);
        // vel_req.print();
        PVector acc = vel_req.minus(bot.getvelocity());
        acc.limit(5);
        // assert ((acc.modulus() <= 5.0f) or !(printf("Jingalala %f",acc.modulus())));
        bot.applyForce(acc);
        bot.update(0.1);
        cout << "*\tSpeed = " << bot.getvelocity().modulus() << endl;
        
        PVector ux, uy;
        curx = bot.getpose().getx();
        cury = bot.getpose().gety(); 
        cout << "At " << curx << ", " << cury << endl;
        u.push_back(bot.getvelocity().normalize(ux).getx());
        v.push_back(bot.getvelocity().normalize(uy).gety());
        min_dist.push_back(pow(pow(curx-GOAL_X, 2)+ pow(cury-GOAL_Y, 2), 0.5));

        // Plotting the steps
        if(showPlot){
            plt::clf(); // Clear previous plot
            for(auto r:env)
                r.draw();
            
            plt::quiver((vector<float>){curx}, (vector<float>){cury}, (vector<float>){u[u.size()-1]}, (vector<float>){v[v.size()-1]});
            // plt::xlim(0, 10);
            // plt::ylim(0, 10);
            plt::pause(0.01);
        }
        
    }   
    cout <<"Condition "<< ( iter<1000 and sqrt((pow(curx-GOAL_X, 2)+ pow(cury-GOAL_Y, 2)) >= 0.1)) << endl;   
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

int main(){
    cout << "Reward = " << find_path_acc(0.39) << endl;    
    return 0;
}