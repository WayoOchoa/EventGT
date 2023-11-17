#include <iostream>
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
            vector<graph::Graph> tracked_corners_;
            int number_of_tracks_;
        public:
            MultiGraph();
            ~MultiGraph(){}

            // Data members

            // Methods
            int size();
            int TrackCorner(EventCorner& corner);
            void FirstInitialization(EventCorner& corner);
    };
}