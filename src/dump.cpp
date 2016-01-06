#include <iostream>
#include <fstream>
#include <iomanip>

#include <wordexp.h>

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

// These are set by params to the node.
std::ofstream g_of;

// For filename and stats.
size_t g_counter = 0;

void cb(const sensor_msgs::LaserScanConstPtr& s)
{
    g_of << s->header.seq << ",";
    for(size_t i = 0 ; i < s->ranges.size() ; ++i) {
        g_of << s->ranges[i];
        if(i < s->ranges.size())
            g_of << ",";
    }
    g_of << std::endl;

    g_counter++;
    std::cout << "\rScans: " << g_counter << std::flush;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "dump_laser");
    ros::NodeHandle nh;

    ros::NodeHandle nh_("~");
    std::string fname;
    nh_.param("file", fname, std::string("scan.csv"));

    // equivalend to Python's expanduser(dir)
    wordexp_t exp_result;
    if(0 !=wordexp(fname.c_str(), &exp_result, WRDE_NOCMD | WRDE_SHOWERR))
        return 1;
    fname = exp_result.we_wordv[0];
    wordfree(&exp_result);

    g_of.open(fname.c_str(), std::ofstream::app);
    if(!g_of) {
        ROS_ERROR("Error opening/creating output file %s", fname.c_str());
        return 2;
    }
    //g_of << std::fixed; // << std::setprecision(5);

    ROS_INFO("APPENDING laser-scan to %s", fname.c_str());

    std::string topic;
    nh_.param("scan", topic, std::string("/scan"));
    ros::Subscriber sub = nh_.subscribe<sensor_msgs::LaserScan>(topic.c_str(), 1, cb);

    ros::spin();

    // TODO: ROS_INFO isn't being output after the spin is done?
    //ROS_INFO("Dumped a total of %d track frames.", g_counter);
    std::cout << "Dumped a total of " << g_counter << " laser scans." << std::endl;
    return 0;
}
