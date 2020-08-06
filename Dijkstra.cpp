#include<iostream>
#include<cmath>
#include<limits>
#include<queue>
#include<vector>

#include"Dijkstra.h"

using namespace std;

Dijkstra::Dijkstra()
{
    sx = 5.0;
    sy = 5.0;
    gx = 50.0;
    gy = 50.0;
    resolution   = 1.0;
    robot_radius = 1.0;
}

Dijkstra::~Dijkstra()
{
    cout << "GOAL!!" << endl;
}

vector<Node> Dijkstra::get_motion_model()
{
    return {Node(1, 0, 1),
            Node(0, 1, 1),
            Node(-1, 0, 1),
            Node(0, -1, 1),
            Node(-1, -1, sqrt(2)),
            Node(-1, 1, sqrt(2)),
            Node(1, -1, sqrt(2)),
            Node(1, 1, sqrt(2))};
}

vector<vector<float>> Dijkstra::calc_final_path(Node * goal_node, float resolution)
{
    vector<float> rx;
    vector<float> ry;
    Node* node = goal_node;

    FILE *fp;
    if ((fp = fopen("path.txt", "w")) == NULL) {
        printf("Error\n");
        exit(1);
    }

    while (node -> parent_node != NULL){
        node = node -> parent_node;
        rx.push_back(node -> x * resolution);
        ry.push_back(node -> y * resolution);
        fprintf(fp, "%lf\t%lf\n", node ->x * resolution, node -> y * resolution);
    }

    fclose(fp);
    return {rx, ry};
}

vector<vector<int>> Dijkstra::calc_obstacle_map(vector<int> ox, vector<int> oy, const int min_ox, const int max_ox, const int min_oy, const int max_oy, float resolution, float robot_radius)
{
    int xwidth = max_ox - min_ox;
    int ywidth = max_oy - min_oy;

    vector<vector<int>> obmap(ywidth, vector<int>(xwidth, 0));

    for (int ix = 0; ix < xwidth; ix++) {
        int x = ix + min_ox;
        for (int iy = 0; iy < ywidth; iy++) {
            int y = iy + min_oy;
            for (int k = 0; k < ox.size(); k++) {
                float d = sqrt(pow((ox[k] - x), 2) + pow((oy[k] - y), 2));
                if (d <= robot_radius / resolution) {
                    obmap[ix][iy] = 1;
                    break;
                }
            }
        }
    }

    return obmap;
}

bool Dijkstra::verify_node(Node* node, vector<vector<int>> obmap, int min_ox, int max_ox, int min_oy, int max_oy)
{
    if (node -> x < min_ox || node -> y < min_oy || node -> x >= max_ox || node -> y >= max_oy) {
        return false;
    }

    if (obmap[node->x-min_ox][node->y-min_oy]) return false;

    return true;
}

void Dijkstra::dijkstra_method(vector<float> ox_, vector<float> oy_)
{
    Node* start_node = new Node((int)round(sx / resolution), (int)round(sy / resolution), 0.0);
    Node *goal_node  = new Node((int)round(gx / resolution), (int)round(gy / resolution), 0.0);

    vector<int> ox;
    vector<int> oy;

    int min_ox = numeric_limits<int>::max();
    int max_ox = numeric_limits<int>::min();
    int min_oy = numeric_limits<int>::max();
    int max_oy = numeric_limits<int>::min();

    for (float iox:ox_) {
        int map_x = (int)round(iox * 1.0 / resolution);
        ox.push_back(map_x);
        min_ox = min(map_x, min_ox);
        max_ox = max(map_x, max_ox);
    }
    for (float ioy:oy_) {
        int map_y = (int)round(ioy * 1.0 / resolution);
        oy.push_back(map_y);
        min_oy = min(map_y, min_oy);
        max_oy = max(map_y, max_oy);
    }

    int xwidth = max_ox - min_ox;
    int ywidth = max_oy - min_oy;

    vector<vector<int> > visit_map(xwidth, vector<int>(ywidth, 0));

    vector<vector<int> > obmap = calc_obstacle_map(ox, oy, min_ox, max_ox, min_oy, max_oy, resolution, robot_radius);

    auto cmp = [](const Node* left, const Node* right){return left->cost > right->cost;};
    priority_queue<Node*, vector<Node*>, decltype(cmp)> open_set(cmp);

    open_set.push(start_node);
    vector<Node> motion = get_motion_model();

    while (true) {
        Node * node = open_set.top();

        if (visit_map[node -> x - min_ox][node -> y - min_oy] == 1) {
            open_set.pop();
            delete node;
            continue;
        } else {
            open_set.pop();
            visit_map[node -> x - min_ox][node -> y - min_oy] = 1;
        }

        if (node -> x == goal_node -> x && node -> y == goal_node -> y) {
            goal_node ->cost = node -> cost;
            goal_node -> parent_node = node;
            break;
        }

        for (int i = 0; i < motion.size(); i++) {
            Node * new_node = new Node(node -> x + motion[i].x, node -> y + motion[i].y, node -> cost + motion[i].cost, node);

            if (!verify_node(new_node, obmap, min_ox, max_ox, min_oy, max_oy)) {
                delete new_node;
                continue;
            }

            if (visit_map[new_node -> x - min_ox][new_node -> y - min_oy]){
                delete new_node;
                continue;
            }

            open_set.push(new_node);
        }
    }

    calc_final_path(goal_node, resolution);
    delete goal_node;
    delete start_node;
}
