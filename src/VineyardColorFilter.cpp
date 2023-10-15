#include "VineyardColorFilter.h"

VineyardColorFilter::VineyardColorFilter( ros::NodeHandle& nh )
    : m_tracker(50)
{
    nh_ = nh;

    sub_camera_ = nh_.subscribe("/camera/color/image_raw/compressed", 1, &VineyardColorFilter::cameraCallback, this);

 
    image_transport::ImageTransport it_(nh_);
    pub_img_ = it_.advertise("filtered_img", 1);
    pub_img_clean_ = it_.advertise("filtered_img_clean", 1);
    pub_mask_ = it_.advertise("mask", 1);

    pub_y_err_ = nh_.advertise<std_msgs::Float32>("y_err", 1);
    pub_x_err_ = nh_.advertise<std_msgs::Float32>("x_err", 1);

    pub_nContours_ = nh_.advertise<std_msgs::Int32>("nContours", 1);
    pub_nPatches_ = nh_.advertise<std_msgs::Int32>("nPatches", 1);

    // Setup dynamic reconfigure server
    // TODO Leave as is for now
    vineyard_color_filter::ColorFilterParametersConfig cfg;
    nh_.getParam("h_lower", cfg.h_lower);
    nh_.getParam("h_upper", cfg.h_upper);
    nh_.getParam("s_lower", cfg.s_lower);
    nh_.getParam("s_upper", cfg.s_upper);
    nh_.getParam("v_lower", cfg.v_lower);
    nh_.getParam("v_upper", cfg.v_upper);
    nh_.getParam("min_brick_size", cfg.min_brick_size);
    nh_.getParam("min_patch_size", cfg.min_patch_size);
    nh_.getParam("erosion_factor", cfg.erosion_factor);
    _minBrickSize = cfg.min_brick_size;
    _minPatchSize = cfg.min_patch_size;
    _cfConfigServer.updateConfig(cfg);
    _cfParamCallback = boost::bind(
        &VineyardColorFilter::cfParamsCb, this, _1, _2);
    _cfConfigServer.setCallback(_cfParamCallback);
}


void VineyardColorFilter::filter(int argc, char** argv)
{
    ros::Rate loop_rate(30);
    while(nh_.ok())
    {
        //TODO move everything to loop function

        ros::spinOnce();
        loop_rate.sleep();

        // If no new frame just continue;
        if (!new_frame_)	continue;

        applyColorFilter(img_ptr_, filtered_img, color_mask);

        cv::Mat filtered_img_clean = filtered_img.clone();

        // Find contours
        cv::Mat contourOutput = color_mask.clone();
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(contourOutput, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        std::vector<std::vector<cv::Point> >hull( contours.size() );

        // Find best contour candidates
        std::vector<std::vector<cv::Point> > brick_pile_candidates;
        findContourCandidates(contours, _minBrickSize, filtered_img, brick_pile_candidates);

        cv::Mat brick_mask(filtered_img.size().height, filtered_img.size().width, CV_8UC1, cv::Scalar(0));
        if (brick_pile_candidates.size() == 0)
        {
            // There are no brick pile candidates
            std_msgs::Float32 float_msg;
            float_msg.data = 0.0;
            pub_y_err_.publish(float_msg);
            pub_x_err_.publish(float_msg);
        } 
        else 
        {
            // Make a brick_mask with all the convex hull candidates
            for (int i = 0; i < brick_pile_candidates.size(); i++)
            {
                cv::drawContours(filtered_img, brick_pile_candidates, i, cv::Scalar(0, 255, 0), 3);
                cv::drawContours(brick_mask, brick_pile_candidates, i, cv::Scalar(255, 255, 255), -1);
            }

            // Calculate error of the largest convex hull
            std::vector<cv::Point> maxAreaContour;
            chooseOneContour(brick_pile_candidates, maxAreaContour, filtered_img);
            calculateAndPublishOffsets(pub_y_err_, pub_x_err_, maxAreaContour);
        }

        // Always publish number of contours
        std_msgs::Int32 int_msg;
        int_msg.data = brick_pile_candidates.size();
        pub_nContours_.publish(int_msg);

        //------------publishanje-------------
        std_msgs::Header header;
        header.frame_id = "";
        header.stamp = ros::Time::now();
        header.seq = img_ptr_->header.seq;

        cv_bridge::CvImage filtered_cv_img(header, "bgr8", filtered_img);
        pub_img_.publish(filtered_cv_img.toImageMsg());

        cv_bridge::CvImage filtered_cv_img_clean(header, "bgr8", filtered_img_clean);
        pub_img_clean_.publish(filtered_cv_img_clean.toImageMsg());

        cv_bridge::CvImage mask_img(header, "mono8", color_mask);
        pub_mask_.publish(mask_img.toImageMsg());

        cv::Mat eroded_mask(brick_mask.size().height, brick_mask.size().width, CV_8UC1, cv::Scalar(0));

        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                    cv::Size( 2*_erosion_factor + 1, 2*_erosion_factor+1 ),
                                                    cv::Point( _erosion_factor, _erosion_factor ) );

        cv::erode(brick_mask, eroded_mask, element);
        cv_bridge::CvImage brick_mask_img(header, "mono8", eroded_mask);
        pub_mask_.publish(brick_mask_img.toImageMsg());

        new_frame_ = false;
    }
}

void VineyardColorFilter::cfParamsCb(vineyard_color_filter::ColorFilterParametersConfig& configMsg, uint32_t level)
{
    nh_.setParam("h_lower", configMsg.h_lower);
    nh_.setParam("h_upper", configMsg.h_upper);
    nh_.setParam("s_lower", configMsg.s_lower);
    nh_.setParam("s_upper", configMsg.s_upper);
    nh_.setParam("v_lower", configMsg.v_lower);
    nh_.setParam("v_upper", configMsg.v_upper);
    _minBrickSize = configMsg.min_brick_size;
    _minPatchSize = configMsg.min_patch_size;
    _erosion_factor = configMsg.erosion_factor;
}

void VineyardColorFilter::cameraCallback(const sensor_msgs::CompressedImage::ConstPtr& ros_msg)
{
    new_frame_ = true;
    img_ptr_ = cv_bridge::toCvCopy(ros_msg, sensor_msgs::image_encodings::BGR8);
}

void VineyardColorFilter::applyColorFilter(cv_bridge::CvImagePtr input_img_ptr, cv::Mat &filtered_img, cv::Mat &mask)
{
    //img_ptr_->image
    cv::Mat blurred_img;
    cv::GaussianBlur(input_img_ptr->image, blurred_img, cv::Size(9, 9), 0);

    cv::Mat hsv_img;
    cv::cvtColor(blurred_img, hsv_img, cv::ColorConversionCodes::COLOR_BGR2HSV);

    int H_low_1, H_high_1;
    int S_low_1, S_high_1;
    int V_low_1, V_high_1;

    std::string node_name = nh_.getNamespace();
    nh_.getParam("h_lower", H_low_1);
    nh_.getParam("h_upper", H_high_1);
    nh_.getParam("s_lower", S_low_1);
    nh_.getParam("s_upper", S_high_1);
    nh_.getParam("v_lower", V_low_1);
    nh_.getParam("v_upper", V_high_1);

    // Check whether the Hue range goes across the 0/360 spot.
    if (H_low_1 > H_high_1) {
        cv::Mat mask1, mask2;
        int H_low_2 = 0;
        int H_high_2 = H_high_1;
        H_high_1 = 179;
        cv::inRange(hsv_img, cv::Scalar(H_low_1, S_low_1, V_low_1), cv::Scalar(H_high_1, S_high_1, V_high_1), mask1);
        cv::inRange(hsv_img, cv::Scalar(H_low_2, S_low_1, V_low_1), cv::Scalar(H_high_2, S_high_1, V_high_1), mask2);
        cv::bitwise_xor(mask1, mask2, mask);
    }
    else {
        cv::inRange(hsv_img, cv::Scalar(H_low_1, S_low_1, V_low_1), cv::Scalar(H_high_1, S_high_1, V_high_1), mask);
    }

    cv::Mat temp_filtered_img;
    cv::bitwise_and(input_img_ptr->image, input_img_ptr->image, temp_filtered_img, mask);
    filtered_img = temp_filtered_img;
}

void VineyardColorFilter::findContourCandidates(std::vector<std::vector<cv::Point> > contours, int threshold,
                                                cv::Mat &output_img,
                                                std::vector<std::vector<cv::Point> > &brick_pile_candidates)
{
    for (int i = 0; i < contours.size(); i++) 
    {
        cv::Moments M = cv::moments(contours[i]);
        if (M.m00 == 0) continue;
        
        int cX = int(M.m10 / M.m00);
        int cY = int(M.m01 / M.m00);

        if (cv::contourArea(contours[i]) > threshold)
        {
            cv::circle(output_img, cv::Point(cX, cY), 3, cv::Scalar(150, 150, 150), -1);
            brick_pile_candidates.push_back(contours[i]);
        }
    }
}

void VineyardColorFilter::chooseOneContour( std::vector<std::vector<cv::Point> > brick_pile_candidates,
                                            std::vector<cv::Point> &maxAreaContour , cv::Mat& output_img)
{
    double maxArea = cv::contourArea(brick_pile_candidates[0]);
    std::vector<cv::Point> maxAreaCnt = brick_pile_candidates[0];

    for (int i = 1; i < brick_pile_candidates.size(); i++) {

        double contourArea = cv::contourArea(brick_pile_candidates[i]);
        if (contourArea > maxArea) {
            maxArea = contourArea;
            maxAreaCnt = brick_pile_candidates[i];
        }
    }

    cv::Moments M = cv::moments(maxAreaCnt);
    int cX = int(M.m10 / M.m00);
    int cY = int(M.m01 / M.m00);
    cv::circle(output_img, cv::Point(cX, cY), 10, cv::Scalar(0, 255, 255), 2);

    maxAreaContour = maxAreaCnt;
}


void VineyardColorFilter::calculateAndPublishOffsets(ros::Publisher& off1, ros::Publisher& off2, std::vector<cv::Point> contour)
{
    cv::Moments M = cv::moments(contour);
    int cX = int(M.m10 / M.m00);
    int cY = int(M.m01 / M.m00);
    std::cout << "cX, cY = " << cX << ", " << cY << "\n";
    float yaw_err = float(cX - filtered_img.size().width / 2) / float(filtered_img.size().width / 2);
    float pitch_err = float(cY - filtered_img.size().height / 2) / float(filtered_img.size().height / 2);

    std_msgs::Float32 float_msg;
    float_msg.data = yaw_err;
    off1.publish(float_msg);

    float_msg.data = pitch_err;
    off2.publish(float_msg);
}
