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
        cv::namedWindow("Event Graph Tracks", cv::WINDOW_NORMAL);
        cv::resizeWindow("Event Graph Tracks", 600, 400);
        
        std::lock_guard<std::mutex> lock(mData);

        vector<shared_ptr<graph::Vertex>> corner_locations;
        for(const auto& it: tracked_corners_){
            //TODO: Find vertices with bigger relative depth
            for(const auto &it: tracked_corners_){
                int max_rel_depth = -1; // No corner tracked
                shared_ptr<graph::Vertex> corner;
                for(const auto & c: it->vertices){
                    if(c->getRelDepth() > max_rel_depth){
                        corner = c;
                    }
                }
                // Save corner for display
                corner_locations.push_back(corner);
            }

            //TODO: tracked N vertices through its parents
        }

        // Display data
        cv::Mat new_img;
        img_data_->image.copyTo(new_img);
        for(const auto& c: corner_locations){
            cv::circle(new_img,c->event_corner_xy_,2,cv::Scalar(0,0,255),cv::FILLED,cv::LINE_8);
        }

        /*
        cv::Mat new_img;
        img_data_->image.copyTo(new_img); 
        cv::circle(new_img,corner_.xy_coord,2,cv::Scalar(0,0,255),cv::FILLED,cv::LINE_8);*/

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