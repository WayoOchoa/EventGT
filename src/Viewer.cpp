#include <iostream>
#include <mutex>

#include "Viewer.h"

using namespace std;
using namespace chrono;

/* Performance testing code
auto start = high_resolution_clock::now();
auto stop = high_resolution_clock::now();
auto duration = duration_cast<seconds>(stop-start);
cout << "DISPLAY TIME: " << duration.count() << endl;*/

namespace viewer{
    // Constructor
    Viewer::Viewer(bool draw_path_flag): bFinishRequested(false), draw_path(draw_path_flag)
    {
    }

    /**
     * Main thread responsible of displaying data when received
    */
    void Viewer::displayTracks(){
        // Setting the viewer parameters
        cv::namedWindow("Event Graph Tracks", cv::WINDOW_NORMAL);
        cv::resizeWindow("Event Graph Tracks", 600, 400);
    
        while(true){
            if(img_data_ != NULL){
                //cout <<"A\n";
                drawOnImage();
            }

            if(CheckifStop()){
                cv::waitKey(100);
                break;
            }
            
        }
    }

    /**
     * Copies the data from the tracker
     * @param tracked_corners Contains all the graphs corresponding to features tracked
    */
    void Viewer::setViewData(vector<shared_ptr<graph::Graph>>& tracked_corners){
        std::lock_guard<std::mutex> lock(mData);
        tracked_corners_ = tracked_corners;
    }

    void Viewer::drawOnImage(){
        
        std::unique_lock<std::mutex> lock(mData);
        vector<shared_ptr<graph::Graph>> corners = tracked_corners_;
        lock.unlock();

        vector<shared_ptr<graph::Vertex>> corner_locations;
        if(corners.size() != 0){
            //cout << "BBBB\n";
            for(const auto& it: corners){
                shared_ptr<graph::Vertex> corner = it->GetMaxVertexDepth();

                // Save corner for display
                corner_locations.push_back(corner);
            }

            publishImage(corner_locations);
        }
    }

    void Viewer::publishImage(vector<shared_ptr<graph::Vertex>>& corner_locations){

        // Display data
        cv::Mat new_img;
        std::lock_guard<std::mutex> lock(mImgData);
        img_data_->image.copyTo(new_img);
        for(const auto& c: corner_locations){
            cv::circle(new_img,c->event_corner_xy_,2,cv::Scalar(0,0,255),cv::FILLED,cv::LINE_4);

            if(draw_path){
                vector<cv::Point> path = c->TrackNVertices(5);
                if(path.size() != 0){
                    cv::polylines(new_img,path,false,cv::Scalar(0,255,0),1,4);
                }
            }
        }

        //cout << "IIIIII\n";
        cv::imshow("Event Graph Tracks",new_img);
        cv::waitKey(1);
    }

    void Viewer::UpdateImgData(cv_bridge::CvImagePtr new_img){
        std::lock_guard<std::mutex> lock(mImgData);
        img_data_=new_img;   
    }

    void Viewer::StopRequest(){
        unique_lock<mutex> lock(mMutexStop);
        bFinishRequested = true;
    }

    bool Viewer::CheckifStop(){
        unique_lock<mutex> lock(mMutexStop);
        return bFinishRequested;
    }
}