#pragma once

#include <iostream>
#include <bits/stdc++.h>
#include <cmath>
#include <mutex>

#include <opencv2/opencv.hpp>

#include <gflags/gflags.h>

using namespace std;

namespace viewer{
    class Viewer;
}

namespace graph{
    class Edge;
    class Vertex;
    class Graph;
}

namespace mgraph{
    class EventCorner{
        //private:
        public:
            // data members
            cv::Point2f xy_coord;
            double timestamp;

            // Contructor
            EventCorner();
            EventCorner(int x_coord,int y_coord,double time);
            ~EventCorner(){};
    };

    class MultiGraph{
        private:
            vector<shared_ptr<graph::Graph>> tracked_corners_;
            vector<shared_ptr<graph::Vertex>> active_vertices_;

            // Other threads
            viewer::Viewer* viewer_ptr_ = NULL;
        public:
            // Data members

            // Constructor
            MultiGraph();
            MultiGraph(viewer::Viewer* viewer);
            ~MultiGraph(){}

            // Methods
            int size();
            int ProcessCorner(EventCorner& corner);
            void FirstInitialization(EventCorner& corner);
            void TrackCorner(EventCorner& corner);
            void AddToActiveVertices(shared_ptr<graph::Vertex>& v_new);
            void CreateNewTrack(EventCorner& corner);
            vector<int> ObtainLeafNodes(vector<shared_ptr<graph::Vertex>>& v_neighbors);
            vector<int> ObtainNonLeafNodes(vector<shared_ptr<graph::Vertex>>& v_neighbors);
            shared_ptr<graph::Vertex> CornerAssociation(EventCorner& corner, vector<shared_ptr<graph::Vertex>>& v_neighbors,bool check_all=true, vector<int> indexes = vector<int>());
            void AddCornerToTrack(shared_ptr<graph::Vertex>& parent_node, EventCorner& corner);
            void UpdateActiveVertices();
            /**
             * @brief Checks all graphs to localize those without active nodes
            */
           void CheckTracks();
           bool FindActiveNodeInGraph(shared_ptr<graph::Graph> &g);
    };
}