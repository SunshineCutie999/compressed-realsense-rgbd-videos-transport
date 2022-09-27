#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <librealsense2/rs.hpp> // Include Intel RealSense Cross Platform API

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pub_cam_node");
    ros::NodeHandle nh;

    std::unique_ptr<rs2::pipeline> pipe;

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    pipe.reset(new rs2::pipeline());

    // // Start streaming with default recommended configuration
    // rs2::pipeline_profile profile = pipe.start();

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    // Instruct pipeline to start streaming with the requested configuration
    rs2::pipeline_profile profile = pipe->start(cfg);

    //image_transport will publish the video that can be compressed
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_color = it.advertise("D435/color", 1);
    image_transport::Publisher pub_depth = it.advertise("D435/depth", 1);

    cv::Mat color, depth;

    while (ros::ok()) {
        rs2::frameset frames = pipe->wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame color_frame = frames.get_color_frame();
        rs2::frame depth_frame = frames.get_depth_frame();
        color = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        depth = cv::Mat(cv::Size(640, 480), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
        pub_color.publish(cv_bridge::CvImage(std_msgs::Header(), "rgb8", color).toImageMsg());
        pub_depth.publish(cv_bridge::CvImage(std_msgs::Header(), "mono16", depth).toImageMsg());
    }
    ros::spin();
}
