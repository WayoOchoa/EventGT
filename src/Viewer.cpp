#include <iostream>
#include <mutex>

#include "Viewer.h"

using namespace std;

namespace viewer{
    // Constructor
    Viewer::Viewer(): bFinishRequested(false)
    {
    }

    /**
     * Main thread responsible of displaying data when received
    */
    void Viewer::displayTracks(){
    // Setting the viewer parameters

        while(true){
            if(img_data_ != NULL){
                drawOnImage();
            }

            if(CheckifStop()){
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
        
        std::lock_guard<std::mutex> lock(mData);

        vector<shared_ptr<graph::Vertex>> corner_locations;
        if(tracked_corners_.size() != 0){
            for(const auto& it: tracked_corners_){
                shared_ptr<graph::Vertex> corner = it->GetMaxVertexDepth();

                // Save corner for display
                corner_locations.push_back(corner);

                //TODO: tracked N vertices through its parents
            }
        }

        publishImage(corner_locations);
    }

    void Viewer::publishImage(vector<shared_ptr<graph::Vertex>>& corner_locations){
        cv::namedWindow("Event Graph Tracks", cv::WINDOW_NORMAL);
        cv::resizeWindow("Event Graph Tracks", 600, 400);

        // Display data
        cv::Mat new_img;
        std::lock_guard<std::mutex> lock(mImgData);
        img_data_->image.copyTo(new_img);
        for(const auto& c: corner_locations){
            cv::circle(new_img,c->event_corner_xy_,2,cv::Scalar(0,0,255),cv::FILLED,cv::LINE_4);
        }

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