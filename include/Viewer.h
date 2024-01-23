#pragma once

#include <iostream>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <mutex>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/video/tracking.hpp>

#include "MultiGraph.h"
#include "DirectedGraph.h"

namespace viewer{
    /**
     * @brief object to visualize the tracks of event corners
    */
   class Viewer{
    private:
        // Handler to publish data on ROS
        ros::NodeHandle nh_;
        
        // Publishers and Subscribers
        //image_transport::Publisher tracks_pub_;
        //image_transport::ImageTransport it_;

        // Methods
        bool CheckifStop();

        // Data members
        cv_bridge::CvImagePtr img_data_;
        vector<shared_ptr<graph::Graph>> tracked_corners_;

        bool bFinishRequested;
    public:
        Viewer();
        ~Viewer(){};

        /**
         * @brief main running thread
        */
       void displayTracks();
       void UpdateImgData(cv_bridge::CvImagePtr new_img); //TODO: Not as reference yet as Im going to test if the copying is well done
       void StopRequest();
       void setViewData(vector<shared_ptr<graph::Graph>> &tracked_corners);
       void drawOnImage();
       void publishImage(vector<shared_ptr<graph::Vertex>>& corner_locations);

       // Data
       std::mutex mMutexStop;
       std::mutex mData;
       std::mutex mImgData;
   };
}