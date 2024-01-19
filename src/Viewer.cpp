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
                std::lock_guard<std::mutex> lock(mData);
                drawOnImage();
                //cv::imshow("Event Graph Tracks",img_data_->image);
                //cv::waitKey(1);
            }

            // TODO: PART TO CORRECTLY TERMINATE THE THREAD AS IT RUNS INDEFINETILY AND CAUSE BREAKS
            if(CheckifStop()){
                break;
            }
        }
    }

    void Viewer::setViewData(mgraph::EventCorner& c){
        std::lock_guard<std::mutex> lock(mData);
        corner_ = c;
    }

    void Viewer::drawOnImage(){
        cv::namedWindow("Event Graph Tracks", cv::WINDOW_NORMAL);
        cv::resizeWindow("Event Graph Tracks", 600, 400);

        cv::Mat new_img;
        img_data_->image.copyTo(new_img); 
        cv::circle(new_img,corner_.xy_coord,2,cv::Scalar(0,0,255),cv::FILLED,cv::LINE_8);

        cv::imshow("Event Graph Tracks",new_img);
        cv::waitKey(1);

    }

    void Viewer::UpdateImgData(cv_bridge::CvImagePtr new_img){
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