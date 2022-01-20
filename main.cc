#include <iostream>
#include "micro_rrt.h"
#include "matplotlibcpp.h"

using namespace std;
namespace plt = matplotlibcpp;

int main(int argc, char ** argv){
    auto x_init = Eigen::Vector2d();
    x_init << 0., 0.;
    
    auto x_goal = Eigen::Vector2d();
    x_goal << 0.75, -0.5;
    
    auto x_min = Eigen::Vector2d();
    x_min << -1., -1.;
    auto x_max = Eigen::Vector2d();
    x_max << 1., 1;
    auto results = DoRRT(x_init, x_goal, x_min, x_max);
    
    cout << "RRT generated " << results.tree.num_nodes() << " nodes and " << results.tree.num_edges() << " edges." << endl;
    cout << "Found this path: " << endl;
    for (const auto index : results.path){
        cout << "\t" << index << ": " << results.tree.get_node(index).transpose() << endl;
    }
    
    // Plot tree.
    //plt::figure_size(1200, 780);
    for (int i=0; i<results.tree.num_edges(); i++){
        const auto edge = results.tree.get_edge(i);
        const auto n1 = results.tree.get_node(edge.first);
        const auto n2 = results.tree.get_node(edge.second);
        std::vector<double> x = {n1[0], n2[0]};
        std::vector<double> y = {n1[1], n2[1]};
        plt::plot(x, y, {{"c", "k"}});
    }
    // Plot path.
    std::vector<double> x;
    std::vector<double> y;
    for (const auto index : results.path){
        const auto n1 = results.tree.get_node(index);
        x.push_back(n1[0]);
        y.push_back(n1[1]);
    }
    plt::plot(x, y, {{"c", "blue"}});
    plt::save("rrt_result.png");
    return 0;
}