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
        cv::namedWindow("Event Graph Tracks", cv::WINDOW_NORMAL);
        cv::resizeWindow("Event Graph Tracks", 600, 400);

        while(true){
            if(img_data_ != NULL){
                cv::imshow("Event Graph Tracks",img_data_->image);
                cv::waitKey(1);
            }

            // TODO: PART TO CORRECTLY TERMINATE THE THREAD AS IT RUNS INDEFINETILY AND CAUSE BREAKS
            if(CheckifStop()){
                break;
            }
        }
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