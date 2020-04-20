//Append "  " to compile when using matplotlibcpp

# include "ODG-PF.h"
# include "GaussianPF_PathPlanning.cpp"
# include "simulator.cpp"
using namespace std;
namespace plt = matplotlibcpp;

int main(){
    vector<float> gammas;
    vector<float> rewards;

    for(float i=0;i<=2;i+=0.01){
        cout << "Gamma = " << i << endl;
        gammas.push_back(i);
        rewards.push_back(find_path(i, false));
    }
    plt::plot(gammas, rewards);
    plt::show();

    // Showing the path with maximum reward
    auto g_max = gammas[std::distance(rewards.begin(), std::max_element(rewards.begin(), rewards.end()))];
    find_path(g_max, true);

    return 0;
}