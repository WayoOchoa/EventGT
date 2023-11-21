#include <DirectedGraph.h>

using namespace std;

namespace graph{
    // Edge methods

    // Vertex methods
    Vertex::Vertex(cv::Point2f xy, double time, int idx, shared_ptr<graph::Graph> parent_graph, bool leaf_flag, bool active_flag)
    : event_corner_xy_(xy), timestamp_(time), vertex_idx_(idx), b_leaf_(leaf_flag), b_active_(active_flag)
    {
        parent_graph_ = parent_graph;
        parent_vertex_=nullptr;
    }

    int Vertex::getStateID(){
        return vertex_idx_;
    }

    void Vertex::AddEdge(Vertex destinationVertex){
        edgeList.push_back(&destinationVertex);
        destinationVertex.assignParentVertex(this);
    }

    void Vertex::deleteEdge(int destinationVertexId){
        for(auto it = edgeList.begin(); it != edgeList.end(); it++){
            if((*it)->getStateID() == destinationVertexId){ // CHECK FOR POSSIBLE REFERENCING ERRORS!!!!
                edgeList.erase(it);
            }
        }
    }

    void Vertex::assignParentVertex(Vertex* parent){
        /**
         * Assigns a pointer to the parent
         * of the current vertex
        */
        this->parent_vertex_ = parent;
    }


    Vertex* Vertex::getParentVertex(){ 
        /**
         * Returns a Vertex pointer corresponding
         * to the parent of the current vertex
        */
        return parent_vertex_;
    }

    ostream& operator<<(ostream& os, const Vertex& v){
        os << "Vertex: " << v.vertex_idx_ << "\nLocation:" << v.event_corner_xy_
        << endl;

        return os;
    }

    // Graph functions
    Graph::Graph(): graph_id_(0), max_depth_(0), number_of_vertices_(0){
    }

    void Graph::AddVertex(shared_ptr<Vertex>& newVertex){
        vertices.push_back(newVertex);
        number_of_vertices_++;
    }

    void Graph::DeleteVertex(int vertexId){
        int vIndex=0;
        for(int i=0; i < vertices.size(); i++){
            // get the index of the current vertex
            if(vertices.at(i)->getStateID() == vertexId){
                vIndex = i; //saves the index of the vertex that needs to be erased
            }
        }

        // Delete the edges pointing to a certain vertex
        Vertex* parent_v = vertices.at(vIndex)->getParentVertex();
        parent_v->deleteEdge(vertices.at(vIndex)->getStateID());

        // Delete vertex
        vertices.erase(vertices.begin()+vIndex);
    }

    int Graph::getNumberVertices(){
        return number_of_vertices_;
    }

    int Graph::getVertexIndexv1(Vertex* v){
        /*
        for(auto it = vertices.begin(); it < vertices.end(); it++){
            cout << "A: " <<  &(*it) << " B: "<< v <<endl;
            if(&(*it)==v){
                auto idx = it - vertices.begin();
                return idx;
            }
        }*/
        
        return -1;
    }

    int Graph::getVertexIndex(Vertex* v){
        int idx = v->getStateID();
        return idx;
    }

}