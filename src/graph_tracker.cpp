#include <iostream>
#include "DirectedGraph.h"

#include "ros/ros.h"

using namespace std;
using namespace graph;

int main(int argc, char **argv){
    ros::init(argc,argv,"graphTracker");
    ros::NodeHandle nh;

    cout << "HELLO WORLD!" << endl;

    return 0;
}