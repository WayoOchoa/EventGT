#include <iostream>
#include <gflags/gflags.h>
#include <thread>
#include "MultiGraph.h"

#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace std;
using namespace graph;

//DEFINE_int32(pixels_threshold,5,"Window threshold in pixels (x,y) to make associations with old corners");
//DEFINE_double(time_threshold,0.1,"Temporal threshold for feature associations");

class GraphTracker{
    public:
        mgraph::MultiGraph graph_of_tracks_; // Contains all the graphs for corresponding tracks

        // Contructor
        GraphTracker(){
            graph_of_tracks_ = mgraph::MultiGraph();
        }
        GraphTracker(viewer::Viewer* viewer){
            graph_of_tracks_ = mgraph::MultiGraph(viewer);
        }
        ~GraphTracker(){}
        
        // methods
        void cornerCallbackTest(const dvs_msgs::EventArray::ConstPtr& msg){
            //cout << endl;
            for(int i=0; i < msg->events.size(); i++){
                //cout << "Number of graphs before: " << graph_of_tracks_.size() << endl;
                mgraph::EventCorner detected_corner(static_cast<int>(msg->events[i].x), static_cast<int>(msg->events[i].y), msg->events[i].ts.toSec());
                graph_of_tracks_.ProcessCorner(detected_corner);
                //cout << "Number of graphs after: " << graph_of_tracks_.size() << endl;
            //cout << "\n\n";
            }
        }

};

int main(int argc, char **argv){
    google::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc,argv,"graphTracker");
    ros::NodeHandle nh;

    // Graph object
    viewer::Viewer* viewer = new viewer::Viewer();
    std::thread* mptViewer = new thread(&viewer::Viewer::displayTracks,viewer);
    //mptViewer->detach();
    GraphTracker tracks(viewer);

    //ros::Subscriber corner_sub = nh.subscribe("/dvs/corners",3,cornercallback);
    rosbag::Bag bag;
    bag.open("/home/eduardo/Dataset/EBC/shapes_6dof_5sec_corners.bag",rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("/dvs/image_raw"));
    topics.push_back(std::string("/dvs/corners"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view){
        dvs_msgs::EventArray::ConstPtr corner = m.instantiate<dvs_msgs::EventArray>();
        sensor_msgs::Image::ConstPtr image = m.instantiate<sensor_msgs::Image>();

        if (corner != NULL){
            if(corner->events.size() == 0) continue; // skip if the message has no corner data
            tracks.cornerCallbackTest(corner);
        }
    }

    bag.close();
    //ros::spin();

    return 0;
}