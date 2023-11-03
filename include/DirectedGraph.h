#include <iostream>
#include <list>
#include <vector>
#include <iterator>

#include <opencv2/opencv.hpp>
#include "fa_harris_detector.h"

using namespace std;

namespace graph{
    class Edge{
        private:
            int DestinationVertexID;//TODO: Has to be a pointer
            int weight_;
        public:
            Edge();
            ~Edge();
    };

    class Vertex
    {
    private:
        int state_id;
        list<Vertex *> edgeList; //Adjacent list of current vertex Before: list<Edge>
        int relative_depth_; //Depth from the root node to the current vertex 
        Vertex *parent_vertex_;
    public:
        Vertex(cv::Point2f xy, float time, bool leaf_flag=true, bool active_flag=true);
        ~Vertex();

        //data members
        cv::Point2f event_corner_xy_;
        float timestamp_;
        bool b_leaf_;
        bool b_active_;

        //methods
        int getStateID();
        void AddEdge(Vertex destinationVertex);
        Vertex* getParentVertex();
        void assignParentVertex(Vertex* parent);
        void deleteEdge(int destinationVertexId);
    };
    

    class Graph{
        private:
            vector<Vertex> vertices;
            int graph_id_;
            int max_depth_; //maximum depth of the graph
        public:
            Graph();
            ~Graph();
            
            //methods
            void AddVertex(Vertex newVertex); //Note for main: Before, we need to create a new Vertex object and get the event corner info
            void DeleteVertex(int vertexId); //Before, you need to obtain the vertex id of the node you want to erase
    };

}