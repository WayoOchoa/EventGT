#pragma once

#include <iostream>
#include <list>
#include <vector>
#include <iterator>
#include <mutex>

#include <opencv2/opencv.hpp>
#include "fa_harris_detector.h"

using namespace std;

namespace graph{
    class Graph; // Forward declaration of graph class

    class Edge{ // Class not used at the moment
        private:
            int DestinationVertexID;
            int weight_;
        public:
            Edge();
            ~Edge(){}
    };

    class Vertex
    {
    private:
        int vertex_idx_;
        list<shared_ptr<Vertex>> edgeList; //Adjacent list of current vertex Before: list<Edge>
        int relative_depth_; //Depth from the root node to the current vertex 
        shared_ptr<Vertex> parent_vertex_;
        shared_ptr<graph::Graph> parent_graph_;
    public:
        //data members
        cv::Point2f event_corner_xy_;
        double timestamp_;
        bool b_leaf_;
        bool b_active_;

        // Constructor
        Vertex(cv::Point2f xy, double time, int idx, shared_ptr<graph::Graph> parent_graph, bool leaf_flag=true, bool active_flag=true);
        ~Vertex(){}

        //methods
        int getStateID();
        int getVertexDepth();
        int getRelDepth();
        void assignVertexDepth(int depth=0);
        void AddEdge(shared_ptr<Vertex>& destinationVertex);
        shared_ptr<Vertex> getParentVertex();
        shared_ptr<Graph> getParentGraph();
        void assignParentGraph(shared_ptr<Graph>& g_parent);
        void assignParentVertex(shared_ptr<Vertex>& parent);
        void deleteEdge(int destinationVertexId);
        /**
         * @brief Gives a vector list of the previous N vertices of the current vertex
        */
        vector<cv::Point> TrackNVertices(int n);

        // Printing functionalities
        friend ostream& operator<<(ostream& os, const Vertex& v);
    };
    

    class Graph{
        private:
            int graph_id_;
            int max_depth_; //maximum depth of the graph
            shared_ptr<graph::Vertex> vertex_with_max_depth_;
            double last_added_v_timestamp_;
        public:
            vector<shared_ptr<Vertex>> vertices; // TODO: Put it back to private when you add a function to access it
            int number_of_vertices_;

            // Constructor
            Graph();
            ~Graph(){}
            
            //methods
            void AddVertex(shared_ptr<Vertex>& newVertex);
            void DeleteVertex(int vertexId); //Before, you need to obtain the vertex id of the node you want to erase
            int getNumberVertices();
            int getVertexIndexv1(Vertex* v); // V1
            int getVertexIndex(Vertex* v);
            int getMaxDepth();
            double getLastTimestamp();
            void UpdateMaxDepth(int new_depth);
            void UpdateVertexWithMaxDepth(shared_ptr<graph::Vertex> & v);
            void UpdateLastTimestamp(double &t);
            shared_ptr<graph::Vertex>& GetMaxVertexDepth();
    };
}