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
            /*cout << "graph: " << active_v->event_corner_xy_ << " corner " << corner.xy_coord << endl;
            cout << "graph v_x:"<< active_v->event_corner_xy_.x << " --- corner x:"<<corner.xy_coord.x<<endl;
            cout << "Condition 1:" << active_v->event_corner_xy_.x << ">=" << corner.xy_coord.x - pixels_threshold << ":" << (active_v->event_corner_xy_.x >= (corner.xy_coord.x - pixels_threshold)) << endl;
            cout << "Condition 2:" << active_v->event_corner_xy_.x << "<=" << corner.xy_coord.x + pixels_threshold << ":" << (active_v->event_corner_xy_.x <= (corner.xy_coord.x + pixels_threshold)) << endl;
            cout << "graph v_y:"<< active_v->event_corner_xy_.y << " --- corner y:"<<corner.xy_coord.y <<endl;
            cout << "Condition 3:" << active_v->event_corner_xy_.y << ">=" << corner.xy_coord.y - pixels_threshold << ":" << (active_v->event_corner_xy_.y >= (corner.xy_coord.y - pixels_threshold)) << endl;
            cout << "Condition 4:" << active_v->event_corner_xy_.y << "<=" << corner.xy_coord.y + pixels_threshold << ":" << (active_v->event_corner_xy_.y <= (corner.xy_coord.y + pixels_threshold)) << endl;
            cout << "graph time:"<< active_v->timestamp_ << " --- corner y:"<<corner.timestamp<<endl;
            cout << "Condition 5:" << (active_v->timestamp_ > (corner.timestamp - time_threshold))<<endl << endl;*/
            if(active_v->event_corner_xy_.x >= (corner.xy_coord.x - pixels_threshold) && active_v->event_corner_xy_.x <= (corner.xy_coord.x + pixels_threshold)
            && active_v->event_corner_xy_.y >= (corner.xy_coord.y - pixels_threshold) && active_v->event_corner_xy_.y <= (corner.xy_coord.y + pixels_threshold)
            && active_v->timestamp_ > (corner.timestamp - time_threshold)){ // NOTE: CHECK FOR AN OPTIMIZATION FOR THIS COMPARISON
                // TODO: COMPARE DISTANCE OR CREATE A SET OF NEIGHBOR NODES
                neighbor_vertices.push_back(active_v);
            }
        }
        // TODO: Check if neighbors are zero
        if(neighbor_vertices.size() == 0){
            cout << "A\n";
            CreateNewTrack(corner);
        }else{
            vector<int> leaf_nodes_idx = ObtainLeafNodes(neighbor_vertices); // The data association is primarily done with the leaf nodes of all graphs
            vector<int> non_leaf_nodes_idx = ObtainNonLeafNodes(neighbor_vertices);
            shared_ptr<graph::Vertex> associated_vertex = nullptr;
            if(leaf_nodes_idx.size() == 0){ // If there are no leaf nodes then try to associate the new corner with all the neighbor nodes
                associated_vertex = CornerAssociation(corner,neighbor_vertices);
            }else{
                associated_vertex = CornerAssociation(corner,neighbor_vertices,false,leaf_nodes_idx);
                if(associated_vertex == nullptr){
                    cout << "B\n";
                    associated_vertex = CornerAssociation(corner,neighbor_vertices,false,non_leaf_nodes_idx);
                }
            }
            cout << "as node: " << associated_vertex->event_corner_xy_ << " corner: " << corner.xy_coord << endl;
            AddCornerToTrack(associated_vertex,corner);
        }

        cout << "SIZE ACTIVE:" << active_vertices_.size() << endl;
        
        int i = 0;
        for(const auto& t: tracked_corners_){
            cout << "i:"<<i<<" &: "<< t << endl;
            for(const auto& v: t->vertices){
                cout << "v_id:"<<v->getStateID()<<" (x,y)="<<v->event_corner_xy_<<endl;
            }
            i++;
        }

        int x;
        cin >> x;

        // TODO: REMOVE NODES THAT ARE NOT CONSIDERED ACTIVE ANYMORE (BASED ON GRAPH DEPTH)
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

    vector<int> MultiGraph::ObtainLeafNodes(vector<shared_ptr<graph::Vertex>>& v_neighbors){
        vector<int> leaf_nodes_idx;
        int i = 0;
        for(const auto& v: v_neighbors){
            if(v->b_leaf_==true){
                leaf_nodes_idx.push_back(i);
            }
            i++;
        }
        return leaf_nodes_idx;
    }

    vector<int> MultiGraph::ObtainNonLeafNodes(vector<shared_ptr<graph::Vertex>>& v_neighbors){
        vector<int> leaf_nodes_idx;
        int i = 0;
        for(const auto& v: v_neighbors){
            if(v->b_leaf_==false){
                leaf_nodes_idx.push_back(i);
            }
            i++;
        }
        return leaf_nodes_idx;
    }

    /**
     * Performs the Corner Data association.
     * 
     * @param corner The new detected corner
     * @param v_neighbors A vector of pointers to the active nodes inside the neighborhood of the new detected corner
     * @param check_all Flag that checks if all the active nodes should be checked (Default=true)
     * @param indexes Indexes to the neighbor nodes that should be checke (Default=empty vector)
     * @return parent_node returns a pointer to the associated vertex
     */
    shared_ptr<graph::Vertex> MultiGraph::CornerAssociation(EventCorner& corner, vector<shared_ptr<graph::Vertex>>& v_neighbors,bool check_all, vector<int> indexes){
    
        double min_distance = 10000; // Initialize the min distance
        double prev_time = 0; // Variable to check in the case that two nodes have the same distance. Then, we take the newest one
        shared_ptr<graph::Vertex> parent_node;
        if(!check_all){ // Only check for nodes inside the indexes vector
            for(const auto& it: indexes){
                double dist = cv::norm(v_neighbors[it]->event_corner_xy_-corner.xy_coord);
                if(dist < min_distance){
                    min_distance = dist;
                    prev_time = v_neighbors[it]->timestamp_;
                    parent_node = v_neighbors[it];
                }else if(dist==min_distance && v_neighbors[it]->timestamp_ > prev_time){
                    min_distance = dist;
                    prev_time = v_neighbors[it]->timestamp_;
                    parent_node = v_neighbors[it];
                }
            }
        }else{ // Check all the neighbor vertices
            for(const auto& it: v_neighbors){
                double dist = cv::norm(it->event_corner_xy_-corner.xy_coord);
                if(dist < min_distance){
                    min_distance = dist;
                    prev_time = it->timestamp_;
                    parent_node = it;
                }else if(dist==min_distance && it->timestamp_ > prev_time){
                    min_distance = dist;
                    prev_time = it->timestamp_;
                    parent_node = it;
                }
            }
        }
        return parent_node;
    }

    void MultiGraph::AddCornerToTrack(shared_ptr<graph::Vertex>& parent_node, EventCorner& corner){
        // Add the new corner to its parent node's graph
        shared_ptr<graph::Graph> parent_graph = parent_node->getParentGraph();
        int node_id = parent_graph->getNumberVertices();
        shared_ptr<graph::Vertex> v_new(new graph::Vertex(corner.xy_coord,corner.timestamp,node_id,parent_graph));
        parent_graph->AddVertex(v_new);

        // Assign the parent node to the new added vertex
        v_new->assignParentVertex(parent_node);
        AddToActiveVertices(v_new);
        cout << "graph_parent: "<< parent_node->getParentGraph() << " corner: " << v_new->getParentGraph()<<endl;

        // Assign a new edge connecting parent_node ---> v_new
        parent_node->b_leaf_ = false; // set to false as now it has a child node associated
        //TODO: AddEdge()

    }

    void MultiGraph::AddToActiveVertices(shared_ptr<graph::Vertex>& v_new){
        active_vertices_.push_back(v_new);
    }

    int MultiGraph::size(){
        return tracked_corners_.size();
    }
}