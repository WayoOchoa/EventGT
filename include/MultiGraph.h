#include <iostream>
#include <bits/stdc++.h>
#include <gflags/gflags.h>
#include "DirectedGraph.h"

using namespace std;

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
            int number_of_tracks_;
            vector<graph::Vertex*> active_vertices_;
        public:
            // Data members

            // Constructor
            MultiGraph();
            ~MultiGraph(){}

            // Methods
            int size();
            int ProcessCorner(EventCorner& corner);
            void FirstInitialization(EventCorner& corner);
            void TrackCorner(EventCorner& corner);
            void AddToActiveVertices(graph::Vertex* v_new);
            void CreateNewTrack(EventCorner& corner);
    };
}