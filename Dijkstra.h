#ifndef DIJKSTRA
#define DIJKSTRA

#include<iostream>
#include<cmath>
#include<limits>
#include<queue>
#include<vector>

using namespace std;

class Node 
{
    public:
        int x;
        int y;
        float cost;
        Node* parent_node;
    public:
        Node(int x_, int y_, float cost_, Node* parent_node_ = NULL) {
            x           = x_;
            y           = y_;
            cost        = cost_;
            parent_node = parent_node_;
        };
};

class Dijkstra
{
private:
    float sx;
    float sy;
    float gx;
    float gy;
    float resolution;
    float robot_radius;
public:
    // obstacle vector
    vector<float> ox;
    vector<float> oy;
private:
    vector<Node> get_motion_model();
    vector<vector<float>> calc_final_path(Node * goal_node, float resolution);
    vector<vector<int>> calc_obstacle_map(vector<int> ox, vector<int> oy, const int min_ox, const int max_ox, const int min_oy, const int max_oy, float resolution, float robot_radius);
    bool verify_node(Node* node, vector<vector<int>> obmap, int min_ox, int max_ox, int min_oy, int max_oy);
public:
    Dijkstra();
    ~Dijkstra();
    void dijkstra_method(vector<float> ox_, vector<float> oy_);
};

#endif // DIJKSTRA
