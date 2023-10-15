#ifndef VINEYARD_COLOR_FILTER_H_
#define VINEYARD_COLOR_FILTER_H_

#include "ros/ros.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/CompressedImage.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"


#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>


// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <compressed_image_transport/compressed_publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <dynamic_reconfigure/server.h>
#include <vineyard_color_filter/ColorFilterParametersConfig.h>

#include <vector>
#include <CentroidTracker.h>

class VineyardColorFilter
{
	public:

		int nContours = 0;

		VineyardColorFilter ( ros::NodeHandle& nh);

		void filter(int argc, char** argv);

		void cameraCallback(const sensor_msgs::CompressedImage::ConstPtr& ros_msg);
		void applyColorFilter(cv_bridge::CvImagePtr input_img_ptr, cv::Mat &filtered_img,
							cv::Mat &mask);
		void findContourCandidates(	std::vector<std::vector<cv::Point> > contours, int threshold,
									cv::Mat &output_img,
									std::vector<std::vector<cv::Point> > &brick_pile_candidates);
		void chooseOneContour( std::vector<std::vector<cv::Point> > brick_pile_candidates,
								std::vector<cv::Point> &maxAreaContour , cv::Mat &output_img);
		void calculateAndPublishOffsets(ros::Publisher& off1, ros::Publisher& off2, std::vector<cv::Point> contour);
		void cfParamsCb(
			vineyard_color_filter::ColorFilterParametersConfig& configMsg,
			uint32_t level);

	private:
		ros::NodeHandle nh_;
		ros::Subscriber sub_camera_;

		image_transport::Publisher pub_img_;
        image_transport::Publisher pub_img_clean_;
		image_transport::Publisher pub_mask_;
		ros::Publisher pub_y_err_;
		ros::Publisher pub_x_err_;
		ros::Publisher pub_nContours_;
		ros::Publisher pub_nPatches_;

		/** Define Dynamic Reconfigure parameters **/
		boost::recursive_mutex _colorFilterMutex;
		dynamic_reconfigure::
			Server<vineyard_color_filter::ColorFilterParametersConfig>
			_cfConfigServer {_colorFilterMutex, ros::NodeHandle("vineyard_color_filter_config")};
		dynamic_reconfigure::
			Server<vineyard_color_filter::ColorFilterParametersConfig>::CallbackType
			_cfParamCallback;

		bool new_frame_ = false;
		cv_bridge::CvImagePtr img_ptr_;
		cv::Mat filtered_img;
		cv::Mat color_mask;

		double _minBrickSize, _minPatchSize, _erosion_factor;
		cent_ns::CentroidTracker m_tracker;
};

#endif /* VINEYARD_COLOR_FILTER_H_ */
