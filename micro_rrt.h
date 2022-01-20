#pragma once
#include <vector>
#include <random>
#include <Eigen/Core>


template <int N>
class Tree {
  public:
    // TODO: Eigen::Matrix<double, N, 1> should be a class
    // typename. Throwing errors when I ttry it now; not doing something
    // right...
    typedef Eigen::Matrix<double, N, 1> Point;

    unsigned int insert(Point x) {
        _nodes.push_back(x);
        return _nodes.size() - 1;
    }
    void add_edge(unsigned int i, unsigned int j) {
        _edges.push_back({i, j});
    }
    unsigned int num_nodes(){
        return _nodes.size();
    }
    unsigned int num_edges(){
        return _edges.size();
    }
    // TODO: Bounds check?
    const Point& get_node(unsigned int i){
        return _nodes[i];
    }
    const std::pair<unsigned int, unsigned int>& get_edge(unsigned int i){
        return _edges[i];
    }

   private:
    std::vector<Point> _nodes;
    std::vector<std::pair<unsigned int, unsigned int>> _edges;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

template <int N>
struct RRTResult {
    // TODO: Initializing requires a copy. Smart pointer instead?
    Tree<N> tree;
    std::vector<unsigned int> path;
};
/*
Compute an RRT from x_init to x_goal, staying within the specified bounds.
Returns the generated tree, and the path as indices into the nodes
of that tree.
*/
template <int N>
// TODO: Need the right invocation to grab arbitrary correctly-shaped Eigen matrix
// expressions... yuck.
RRTResult<N> DoRRT(Eigen::Matrix<double, N, 1> x_init, Eigen::Matrix<double, N, 1> x_goal,
                   Eigen::Matrix<double, N, 1> x_min, Eigen::Matrix<double, N, 1> x_max,
                   double dx_max=0.05){
    // Setup for random sampling of points in the volume.
    // If this has low setup cost, this could go in a function,
    // but I'm keeping it here so I don't reinitialize these a bunch.
    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<std::uniform_real_distribution<>> distributions;
    for (int i = 0; i < N; i++){
        distributions.push_back(std::uniform_real_distribution<>(x_min[i], x_max[i]));
    }
    
    Tree<N> tree;
    // Add initial point.
    unsigned int start_i = tree.insert(x_init);
    
    // This index will be set to >= 0 when
    // the goal is connected in the tree.
    unsigned int goal_i = -1;
    while (goal_i == -1){
        // Choose a random point in the bounds. 25%
        // of the time, try to connect to the goal
        // instead.
        bool connecting_to_goal = (rand() % 2000) < 1;
        Eigen::Matrix<double, N, 1> x_new;
        if (connecting_to_goal){
            x_new = x_goal;
        } else {
            for (int i = 0; i < N; i++){
                x_new[i] = distributions[i](gen);
            }
        }
        
        // Find the closest point in the tree.
        // (This could be accelerated with a KD-tree for
        //  fast nearest-neighbor lookup.)
        int nearest_i = -1;
        // All distances will be lower than diagonal distance across
        // the domain.
        double nearest_squared_dist = (x_max - x_min).squaredNorm();
        for (int i = 0; i < tree.num_nodes(); i++){
            // TODO: Check if it's legal to connect with a straight line
            // between them, and skip this node if it isn't.
            double squared_dist = (tree.get_node(i) - x_new).squaredNorm();
            if (squared_dist < nearest_squared_dist){
                nearest_i = i;
                nearest_squared_dist = squared_dist;
            }
        }
        // If the node is too far away, try to extend in its direction
        // instead.
        if (nearest_squared_dist >= dx_max){
            const auto x_nearest = tree.get_node(nearest_i);
            Eigen::Matrix<double, N, 1> delta = (x_new - x_nearest);
            delta = delta / delta.norm();
            x_new = x_nearest + dx_max * delta;
            connecting_to_goal = false;
        }

        // Connect to the node.
        if (nearest_i >= 0){
            int new_ind = tree.insert(x_new);
            tree.add_edge(nearest_i, new_ind);
            if (connecting_to_goal){
                goal_i = new_ind;
            }
        }
    }
    
    // Recover the path backwards from goal to start by
    // traversing edges backwards. We constructed
    // a tree, so this search will always terminate at
    // the root, which was the start node. We build this
    // list backwards (goal to start) since I believe std::vector
    // prefers this ordering.
    std::vector<unsigned int> path;
    path.push_back(goal_i);
    unsigned int index = goal_i;
    while (index != start_i){
        // Search edges for the outgoing edge to the current node.
        for (int i = 0; i < tree.num_edges(); i++){
            const auto edge = tree.get_edge(i);
            if (edge.second == index){
                index = edge.first;
                break;
            }
        }
        path.push_back(index);
    };
    // Flip path order so it's start->goal.
    std::reverse(std::begin(path), std::end(path));
    return RRTResult<N>{tree, path};
}