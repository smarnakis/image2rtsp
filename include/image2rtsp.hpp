#ifndef IMAGE2RTSP_IMAGE2RTSP_HPP
#define IMAGE2RTSP_IMAGE2RTSP_HPP

#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;

class Image2rtsp : public rclcpp::Node{
public:
    Image2rtsp();
    GstRTSPServer *rtsp_server;
    // GstRTSPServer *rtsp_server_comp;

private:
    string source;
    string topic;
    string topic_compressed;
    string mountpoint;
    string bitrate;
    string framerate;
    string caps_1;
    string caps_2;
    string caps_3;
    string port;
    string port_compressed;
    string pipeline;
    string pipeline_head;
    string pipeline_tail;
    bool local_only;
    bool camera;
    bool raw;
    GstAppSrc *appsrc;
    GstAppSrc *appsrcComp;

    void video_mainloop_start();
    void rtsp_server_add_url(const char *url, const char *sPipeline, GstElement **appsrc);
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void topic_compressed_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    GstRTSPServer *rtsp_server_create(const string &port, const bool local_only);
    GstCaps *gst_caps_new_from_image(const sensor_msgs::msg::Image::SharedPtr &msg);
    GstCaps *gst_caps_new_from_compressed_image(const sensor_msgs::msg::CompressedImage::SharedPtr &msg);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscriptionCompressed_;
};

static void media_configure(GstRTSPMediaFactory *factory, GstRTSPMedia *media, GstElement **appsrc);
static void *mainloop(void *arg);
static gboolean session_cleanup(Image2rtsp *node, rclcpp::Logger logger, gboolean ignored);

#endif // IMAGE2RTSP_IMAGE2RTSP_HPP
