#include <DirectedGraph.h>

namespace graph{
    // Edge methods

    // Vertex methods
    Vertex::Vertex(cv::Point2f xy, float time, bool leaf_flag=true, bool active_flag=true)
    : event_corner_xy_(xy), timestamp_(time), b_leaf_(leaf_flag), b_active_(active_flag)
    {
        parent_vertex_=nullptr;
    }

    int Vertex::getStateID(){
        return state_id;
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

    // Graph functions
    Graph::Graph(){}
    Graph::~Graph(){}

    void Graph::AddVertex(Vertex newVertex){
        vertices.push_back(newVertex);
    }

    void Graph::DeleteVertex(int vertexId){
        int vIndex=0;
        for(int i=0; i < vertices.size(); i++){
            // get the index of the current vertex
            if(vertices.at(i).getStateID() == vertexId){
                vIndex = i; //saves the index of the vertex that needs to be erased
            }
        }

        // Delete the edges pointing to a certain vertex
        Vertex* parent_v = vertices.at(vIndex).getParentVertex();
        parent_v->deleteEdge(vertices.at(vIndex).getStateID());

        // Delete vertex
        vertices.erase(vertices.begin()+vIndex);
    }

}