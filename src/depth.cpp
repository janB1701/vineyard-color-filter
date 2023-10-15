//get robot distance from vine
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Float32.h"

using namespace std;

static const uint32_t QUEUE_SIZE = 1000;
std_msgs::Float32 minDistance;

void imgcb(const sensor_msgs::Image::ConstPtr& msg)
{
    try {



        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(msg);

        //dimenzije slike
        static bool runOnce = true;
        if (runOnce){
            cout << "Image dimension (Row,Col): " << cv_ptr->image.rows << " x " << cv_ptr->image.cols << endl;
            runOnce = false;
        }

        //get global max depth value
        double max = 0.0;
        //trebalo bi implementirati da nam ocitava udaljenost od sredista konture!!!! ovo dolje
        //std::cout << cv_ptr->image[100][200];
        cv::minMaxLoc(cv_ptr->image, 0, &max, 0, 0);
        std::cout << "Max value: " << max << endl;

        //get global min depth value
        double min = 0.0;
        cv::minMaxLoc(cv_ptr->image, &min, &max, 0, 0);
        std::cout << "Min value: " << min << endl;
        std::cout << "--------------------------------------" << endl;
        minDistance.data = min;
        

        //get depth value at a point
        //float distanceVal = cv_ptr->image.at<float>(100, 100);
        //std::cout << "Distance value: " << distanceVal << "m" << endl;

    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void publish (double min_distance) {
    
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "show_depth");

    std::cout << "Getting Image depth value!" << std::endl;

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("camera/depth/image_raw", QUEUE_SIZE, imgcb);
    ros::Publisher chatter_pub = nh.advertise<std_msgs::Float32>("chatter", 1000);

    ros::Rate loop_rate(10); 
    while (ros::ok()) {
        chatter_pub.publish(minDistance);
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::cout << "Done" << std::endl;

    return 0;
}
