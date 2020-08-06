#include<queue>
#include<vector>

#include"Dijkstra.h"

using namespace std;

int main()
{
    Dijkstra DK;

    // obstacle range
    for(int i = 0; i < 60; i++){
        DK.ox.push_back(i);
        DK.oy.push_back(60.0);
    }
    for(int i = 0; i < 60; i++){
        DK.ox.push_back(60.0);
        DK.oy.push_back(i);
    }
    for(int i = 0; i < 61; i++){
        DK.ox.push_back(i);
        DK.oy.push_back(60.0);
    }
    for(int i = 0; i < 61; i++){
        DK.ox.push_back(0.0);
        DK.oy.push_back(i);
    }
    for(int i = 0; i < 40; i++){
        DK.ox.push_back(20.0);
        DK.oy.push_back(i);
    }
    for(int i = 0; i < 40; i++){
        DK.ox.push_back(40.0);
        DK.oy.push_back(60.0 - i);
    }

    DK.dijkstra_method(DK.ox, DK.oy);

    return 0;
}