//Append "-std=c++11 -I/usr/include/python2.7 -lpython2.7" to compile when using matplotlibcpp

# include "ODG-PF.h"
# include "GaussianPF_PathPlanning.cpp"
# include "simulator.cpp"
using namespace std;
namespace plt = matplotlibcpp;

int main(){
    vector<float> gammas;
    vector<float> rewards;

    for(float i=0;i<=1.5;i+=0.01){
        cout << "Gamma = " << i ;
        gammas.push_back(i);
        float reward = find_path(i, false);
        cout << ", Reward = " << reward << endl;
        rewards.push_back(reward);

    }
    plt::plot(gammas, rewards);
    plt::xlabel("Gamma");
    plt::ylabel("Rewards");
    plt::title("Rewards vs Gamma");
    int i_max = std::distance(rewards.end(), std::max_element(rewards.begin(), rewards.end()));
    i_max = rewards.size() + i_max;
    vector<float> g_max = {gammas[i_max]};
    vector<float> r_max = {rewards[i_max]};
    plt::scatter(g_max, r_max);
    plt::show();

    // Showing the path with max reward
    // cout << "Reward = " << find_path(g_max[0], true) << endl;
    cout << "Reward = " << find_path_mobile(g_max[0], true) << endl;    
    cout << "Best gamma = " << g_max[0]<< endl;

    return 0;
}
