#include <iostream>
#include "MultiGraph.h"

using namespace std;

namespace mgraph{
    // EventCorner class members definition 
    EventCorner::EventCorner(): xy_coord(0,0), timestamp(0){ 
    }

    EventCorner::EventCorner(int x_coord,int y_coord, double time){
        xy_coord = cv::Point2f(x_coord,y_coord);
        timestamp = time;
    }

    // MultiGraph member definition
    MultiGraph::MultiGraph(){
        number_of_tracks_ = 0;
    }

    int MultiGraph::TrackCorner(EventCorner& corner){
        // Check if there are no grpahs to track
        if(tracked_corners_.size()==0){
            FirstInitialization(corner);
        }else{

        }
        return 0;
    }

    void MultiGraph::FirstInitialization(EventCorner& corner){
        graph::Graph new_track;

        // Initializing the first graph
        tracked_corners_.push_back(new_track);

        // Add vertex data
        int id = tracked_corners_[0].getNumberVertices();
        graph::Vertex vertex(corner.xy_coord,corner.timestamp,id);
        tracked_corners_[0].AddVertex(vertex);

        cout << "INDEX:"<<tracked_corners_[0].getVertexIndex( &tracked_corners_[0].vertices[0])<<endl;
        int x;
        cin >> x;

        // Add vertex to active vertices list

    }

    int MultiGraph::size(){
        return tracked_corners_.size();
    }
}