#include <DirectedGraph.h>

using namespace std;

namespace graph{
    // Edge methods

    // Vertex methods
    Vertex::Vertex(cv::Point2f xy, double time, int idx, shared_ptr<graph::Graph> parent_graph, bool leaf_flag, bool active_flag)
    : event_corner_xy_(xy), timestamp_(time), vertex_idx_(idx), b_leaf_(leaf_flag), b_active_(active_flag), relative_depth_(0)
    {
        parent_graph_ = parent_graph;
        parent_vertex_= nullptr;
    }

    int Vertex::getStateID(){
        return vertex_idx_;
    }

    int Vertex::getVertexDepth(){
        return relative_depth_;
    }

    int Vertex::getRelDepth(){
        return relative_depth_;
    }

    void Vertex::assignVertexDepth(int depth){
        relative_depth_ = depth;
    }

    void Vertex::AddEdge(shared_ptr<Vertex>& destinationVertex){
        this->edgeList.push_back(destinationVertex);
    }

    void Vertex::deleteEdge(int destinationVertexId){
        for(auto it = edgeList.begin(); it != edgeList.end(); it++){
            if((*it)->getStateID() == destinationVertexId){
                edgeList.erase(it);
            }
        }
    }

    /** 
     * Assigns a pointer to the parent
     *  of the current vertex
     */
    void Vertex::assignParentVertex(shared_ptr<Vertex>& parent){
        this->parent_vertex_ = parent;
    }


    shared_ptr<Vertex> Vertex::getParentVertex(){ 
        /**
         * Returns a Vertex pointer corresponding
         * to the parent of the current vertex
        */
        return parent_vertex_;
    }

    shared_ptr<Graph> Vertex::getParentGraph(){
        return parent_graph_;
    }

    void Vertex::assignParentGraph(shared_ptr<Graph>& g_parent){
        this->parent_graph_ = g_parent;
    }

    ostream& operator<<(ostream& os, const Vertex& v){
        os << "Vertex: " << v.vertex_idx_ << "\nLocation:" << v.event_corner_xy_
        << endl;

        return os;
    }

    vector<cv::Point> Vertex::TrackNVertices(int n){
        vector<cv::Point> track_path;
        shared_ptr<Vertex> parent_v = this->getParentVertex();
        if(parent_v != nullptr && n > 1){
            track_path = parent_vertex_->TrackNVertices(n-1);
            cv::Point vertex_xy = this->event_corner_xy_;
            track_path.push_back(vertex_xy);
            return track_path;
        } else{
            cv::Point vertex_xy = this->event_corner_xy_;
            track_path.push_back(vertex_xy);
            return track_path;
        }
    }

    // Graph functions
    Graph::Graph(): graph_id_(0), max_depth_(0), number_of_vertices_(0){
        vertex_with_max_depth_ = nullptr;
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
        shared_ptr<Vertex> parent_v = vertices.at(vIndex)->getParentVertex();
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

    int Graph::getMaxDepth(){
        return max_depth_;
    }

    double Graph::getLastTimestamp(){
        return last_added_v_timestamp_;
    }

    void Graph::UpdateMaxDepth(int new_depth){
        max_depth_ = new_depth;
    }

    void Graph::UpdateVertexWithMaxDepth(shared_ptr<graph::Vertex> &v){
        vertex_with_max_depth_ = v;
    }

    void Graph::UpdateLastTimestamp(double &t){
        last_added_v_timestamp_ = t;
    }

    shared_ptr<graph::Vertex>& Graph::GetMaxVertexDepth(){
        return vertex_with_max_depth_;
    }

}