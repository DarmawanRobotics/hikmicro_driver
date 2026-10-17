// hikmicro_camera_node: one RTSP stream of the camera as sensor_msgs/Image (and compressed via
// image_transport). Run one per stream: visible (channel 1) and thermal (channel 2).
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include "hikmicro_ptz_driver/http_client.hpp"
#include "hikmicro_ptz_driver/rtsp_capture.hpp"

using namespace hikmicro_ptz_driver;

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "hikmicro_camera");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  CaptureConfig c;
  std::string host, user, password;
  int rtsp_port = 554, channel = 1, stream = 1;
  pnh.param<std::string>("url", c.url, "");
  pnh.param<std::string>("host", host, "192.168.1.64");
  pnh.param("rtsp_port", rtsp_port, 554);
  pnh.param<std::string>("user", user, "admin");
  pnh.param<std::string>("password", password, "");
  if (password.empty()) {   // roslaunch prints params: prefer the environment variable
    std::string env_name;
    pnh.param<std::string>("password_env", env_name, "HIKMICRO_PASSWORD");
    if (const char * env = std::getenv(env_name.c_str())) {
      password = env;
    }
  }
  pnh.param("channel", channel, 1);       // 1 visible, 2 thermal
  pnh.param("stream", stream, 1);         // 1 main, 2 sub
  pnh.param<std::string>("codec", c.codec, "auto");
  pnh.param<std::string>("decoder", c.decoder, "auto");
  pnh.param("latency_ms", c.latency_ms, 0);
  pnh.param<std::string>("protocols", c.protocols, "tcp");
  pnh.param("reconnect_sec", c.reconnect_sec, 1.0);
  pnh.param("timeout_ms", c.timeout_ms, 5000);
  std::string frame_id, encoding;
  double max_fps = 0.0;
  int width = 0, height = 0;
  pnh.param<std::string>("frame_id", frame_id, "hikmicro_visible_optical_frame");
  pnh.param<std::string>("encoding", encoding, "bgr8");
  pnh.param("max_fps", max_fps, 0.0);
  pnh.param("width", width, 0);
  pnh.param("height", height, 0);

  if (encoding != "bgr8" && encoding != "mono8") {
    ROS_FATAL("~encoding must be bgr8 or mono8, not '%s'", encoding.c_str());
    return 1;
  }
  std::string shown;
  if (c.url.empty()) {
    const std::string path = ":" + std::to_string(rtsp_port) + "/Streaming/Channels/" +
      std::to_string(channel * 100 + stream);
    std::string cred;
    if (!user.empty()) {
      cred = urlEncode(user) + (password.empty() ? "" : ":" + urlEncode(password)) + "@";
    }
    c.url = "rtsp://" + cred + host + path;
    shown = "rtsp://" + (user.empty() ? "" : user + "@") + host + path;  // no password in logs
  } else {
    shown = c.url;
  }

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("image_raw", 1);
  std::chrono::steady_clock::time_point last_pub{};

  std::unique_ptr<RtspCapture> capture;
  try {
    capture = std::make_unique<RtspCapture>(
      c,
      [&](const cv::Mat & frame) {
        const ros::Time stamp = ros::Time::now();
        const auto t = std::chrono::steady_clock::now();
        if (max_fps > 0.0 && t - last_pub < std::chrono::duration<double>(1.0 / max_fps)) {
          return;
        }
        if (pub.getNumSubscribers() == 0) {
          return;
        }
        last_pub = t;
        cv::Mat out = frame;
        if (width > 0 && height > 0 && (frame.cols != width || frame.rows != height)) {
          cv::resize(frame, out, cv::Size(width, height), 0, 0, cv::INTER_AREA);
        }
        if (encoding == "mono8") {
          cv::Mat gray;
          cv::cvtColor(out, gray, cv::COLOR_BGR2GRAY);
          out = gray;
        }
        std_msgs::Header header;
        header.stamp = stamp;
        header.frame_id = frame_id;
        pub.publish(cv_bridge::CvImage(header, encoding, out).toImageMsg());
      },
      [&](const std::string & m) {
        // keep the password out of the log
        std::string msg = m;
        const size_t at = msg.find(c.url);
        if (at != std::string::npos) {
          msg.replace(at, c.url.size(), shown);
        }
        ROS_INFO("%s", msg.c_str());
      });
  } catch (const std::exception & e) {
    ROS_FATAL("%s", e.what());
    return 1;
  }

  ROS_INFO("streaming %s to %s", shown.c_str(), pub.getTopic().c_str());
  capture->start();
  ros::spin();
  capture->stop();
  return 0;
}
