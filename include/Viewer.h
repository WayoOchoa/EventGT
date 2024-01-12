#include <iostream>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>

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
    public:
        Viewer();
        ~Viewer(){};

        /**
         * @brief main running thread
        */
       void displayTracks();
   };
}