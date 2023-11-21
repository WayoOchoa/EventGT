#include <iostream>
#include "MultiGraph.h"

using namespace std;

#define pixels_threshold 5
#define time_threshold 0.1
//DECLARE_int32(pixels_threshold);
//DECLARE_double(time_threshold);

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

    int MultiGraph::ProcessCorner(EventCorner& corner){
        // Check if there are no grpahs to track
        if(tracked_corners_.size()==0){
            FirstInitialization(corner);
        }else{
            TrackCorner(corner);
        }
        return 0;
    }

    void MultiGraph::FirstInitialization(EventCorner& corner){
        shared_ptr<graph::Graph> new_track(new graph::Graph);
        // Initializing the first graph
        tracked_corners_.push_back(new_track);

        // Add vertex data
        int id = tracked_corners_[0]->getNumberVertices(); // NOTE: CAREFUL THIS DOESNT GIVE THE NUMBER STARTING FROM 0
        shared_ptr<graph::Vertex> vertex(new graph::Vertex(corner.xy_coord,corner.timestamp,id,new_track));
        (tracked_corners_.back())->AddVertex(vertex);
        
        // Add vertex to active vertices list
        AddToActiveVertices(vertex);
    }

    void MultiGraph::TrackCorner(EventCorner& corner){
        vector<shared_ptr<graph::Vertex>> neighbor_vertices;
        for(const auto& active_v: active_vertices_){
            //Evaluate that the corner is within the pixel and time limits
            /*cout << "graph v_x:"<< active_v->event_corner_xy_.x << " --- corner x:"<<corner.xy_coord.x<<endl;
            cout << "Condition 1:" << active_v->event_corner_xy_.x << ">=" << corner.xy_coord.x - pixels_threshold << ":" << (active_v->event_corner_xy_.x >= (corner.xy_coord.x - pixels_threshold)) << endl;
            cout << "Condition 2:" << active_v->event_corner_xy_.x << "<=" << corner.xy_coord.x + pixels_threshold << ":" << (active_v->event_corner_xy_.x <= (corner.xy_coord.x + pixels_threshold)) << endl;
            cout << "graph v_y:"<< active_v->event_corner_xy_.y << " --- corner y:"<<corner.xy_coord.y <<endl;
            cout << "Condition 3:" << active_v->event_corner_xy_.y << ">=" << corner.xy_coord.y - pixels_threshold << ":" << (active_v->event_corner_xy_.y >= (corner.xy_coord.y - pixels_threshold)) << endl;
            cout << "Condition 4:" << active_v->event_corner_xy_.y << "<=" << corner.xy_coord.y + pixels_threshold << ":" << (active_v->event_corner_xy_.y >= (corner.xy_coord.y + pixels_threshold)) << endl;
            cout << "graph time:"<< active_v->timestamp_ << " --- corner y:"<<corner.timestamp<<endl;
            cout << "Condition 5:" << (active_v->timestamp_ > (corner.timestamp - time_threshold))<<endl;*/
            if(active_v->event_corner_xy_.x >= (corner.xy_coord.x - pixels_threshold) && active_v->event_corner_xy_.x <= (corner.xy_coord.x + pixels_threshold)
            && active_v->event_corner_xy_.y >= (corner.xy_coord.y - pixels_threshold) && active_v->event_corner_xy_.y <= (corner.xy_coord.y - pixels_threshold)
            && active_v->timestamp_ > (corner.timestamp - time_threshold)){ // NOTE: CHECK FOR AN OPTIMIZATION FOR THIS COMPARISON
                // TODO: COMPARE DISTANCE OR CREATE A SET OF NEIGHBOR NODES
                neighbor_vertices.push_back(active_v);
            }
        }
        // TODO: Check if neighbors are zero
        if(neighbor_vertices.size() == 0){
            CreateNewTrack(corner);
        }else{

        }

        //cout << "A\n";
        int x;
        cin >> x;
    }

    void MultiGraph::CreateNewTrack(EventCorner& corner){
        // Initializing a new graph of tracks with the non-associated corner as root
        shared_ptr<graph::Graph> new_track(new graph::Graph);
        tracked_corners_.push_back(new_track);
        shared_ptr<graph::Vertex> v_new(new graph::Vertex(corner.xy_coord,corner.timestamp,0,new_track));
        (tracked_corners_.back())->AddVertex(v_new);
        // Add the new node to the list of active vertices
        AddToActiveVertices(v_new);
    }

    void MultiGraph::AddToActiveVertices(shared_ptr<graph::Vertex>& v_new){
        active_vertices_.push_back(v_new);
    }

    int MultiGraph::size(){
        return tracked_corners_.size();
    }
}