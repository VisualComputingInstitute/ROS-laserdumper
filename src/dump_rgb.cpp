#include <iostream>
#include <fstream>
#include <iomanip>

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// For filename and stats.
size_t g_counter = 0;

void cb(const sensor_msgs::LaserScanConstPtr& s, const sensor_msgs::ImageConstPtr& rgb)
{
    // Dump the image.
    cv_bridge::CvImageConstPtr cv_rgb;
    try {
        cv_rgb = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8);
    } catch(const cv_bridge::Exception& e) {
        ROS_ERROR("Couldn't convert image: %s", e.what());
        return;
    }

    std::ostringstream imgname;
    imgname << s->header.seq << ".jpg";
    if(!cv::imwrite(imgname.str(), cv_rgb->image)) {
        ROS_ERROR("Error writing image %s", imgname.str().c_str());
        return;
    }

    g_counter++;
    std::cout << "\rScans: " << g_counter << std::flush;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "dump_laser");
    ros::NodeHandle nh;

    ros::NodeHandle nh_("~");

    std::string topic_laser, topic_rgb;
    nh_.param("scan", topic_laser, std::string("/scan"));
    nh_.param("rgb", topic_rgb, std::string("/head_xtion/rgb/image_rect_color"));

    message_filters::Subscriber<sensor_msgs::LaserScan> sub_laser(nh_, topic_laser.c_str(), 1);

    image_transport::ImageTransport it(nh_);
    image_transport::SubscriberFilter sub_rgb(it, topic_rgb.c_str(), 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::Image> SyncType;
    const SyncType sync_policy(20);

    message_filters::Synchronizer<SyncType> sync(sync_policy, sub_laser, sub_rgb);
    sync.registerCallback(boost::bind(&cb, _1, _2));

    ros::spin();

    // TODO: ROS_INFO isn't being output after the spin is done?
    //ROS_INFO("Dumped a total of %d track frames.", g_counter);
    std::cout << "Dumped a total of " << g_counter << " laser scans." << std::endl;
    return 0;
}
