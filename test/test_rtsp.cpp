// RTSP pipelines and decoder order; the capture loop against a local video file (FFmpeg).
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "hikmicro_ptz_driver/rtsp_capture.hpp"
#include "hikmicro_ptz_driver/rtsp_pipeline.hpp"

using namespace hikmicro_ptz_driver;
using namespace std::chrono_literals;

TEST(RtspPipeline, SoftwareAutoUsesDecodebin)
{
  const auto p = gstreamerPipeline("rtsp://192.168.1.64:554/Streaming/Channels/201", "auto", "software");
  EXPECT_EQ(
    p,
    "rtspsrc location=rtsp://192.168.1.64:554/Streaming/Channels/201 latency=0 protocols=tcp "
    "tcp-timeout=5000000 ! decodebin ! videoconvert ! video/x-raw,format=BGR ! "
    "appsink drop=true max-buffers=1 sync=false");
}

TEST(RtspPipeline, CodecSpecificDecoders)
{
  const auto sw = gstreamerPipeline("rtsp://x/Streaming/Channels/101", "H265", "software", 50, "udp", 2000);
  EXPECT_NE(sw.find("latency=50 protocols=udp tcp-timeout=2000000"), std::string::npos);
  EXPECT_NE(sw.find("rtph265depay ! h265parse ! avdec_h265"), std::string::npos);
  const auto hw = gstreamerPipeline("rtsp://x/Streaming/Channels/201", "h264", "jetson");
  EXPECT_NE(hw.find("rtph264depay ! h264parse ! nvv4l2decoder ! nvvidconv"), std::string::npos);
  EXPECT_NE(hw.find("appsink drop=true max-buffers=1"), std::string::npos);
}

TEST(RtspPipeline, RejectsBadCombinations)
{
  EXPECT_THROW(gstreamerPipeline("u", "mjpeg", "software"), std::invalid_argument);
  EXPECT_THROW(gstreamerPipeline("u", "auto", "jetson"), std::invalid_argument);
  EXPECT_THROW(gstreamerPipeline("u", "h264", "ffmpeg"), std::invalid_argument);
  EXPECT_THROW(decoderCandidates("cuda", "h264"), std::invalid_argument);
}

TEST(RtspPipeline, DecoderOrder)
{
  EXPECT_EQ(decoderCandidates("auto", "h265"),
    (std::vector<std::string>{"jetson", "software", "ffmpeg"}));
  EXPECT_EQ(decoderCandidates("AUTO", "auto"), (std::vector<std::string>{"software", "ffmpeg"}));
  EXPECT_EQ(decoderCandidates("ffmpeg", "auto"), (std::vector<std::string>{"ffmpeg"}));
}

TEST(RtspPipeline, DetectsGstreamerInBuildInfo)
{
  EXPECT_TRUE(opencvHasGstreamer("  Video I/O:\n    GStreamer:                   YES (1.20.3)\n"));
  EXPECT_FALSE(opencvHasGstreamer("  Video I/O:\n    GStreamer:                   NO\n"));
  EXPECT_FALSE(opencvHasGstreamer("  Video I/O:\n    FFMPEG: YES\n"));
}

TEST(RtspCapture, ConstructorValidatesParameters)
{
  CaptureConfig c;
  c.decoder = "cuda";
  EXPECT_THROW(RtspCapture(c, [](const cv::Mat &) {}), std::invalid_argument);
  c.decoder = "jetson";
  c.codec = "auto";
  EXPECT_THROW(RtspCapture(c, [](const cv::Mat &) {}), std::invalid_argument);
  c.decoder = "auto";
  c.codec = "vp9";
  EXPECT_THROW(RtspCapture(c, [](const cv::Mat &) {}), std::invalid_argument);
}

// The FFmpeg path end to end, with a file instead of the pod: frames arrive, and the loop
// reopens the source when it ends (like a dropped stream).
TEST(RtspCapture, FfmpegFramesAndReconnect)
{
  const std::string path = ::testing::TempDir() + "hikmicro_capture_test.avi";
  {
    cv::VideoWriter w(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10.0, cv::Size(64, 48));
    if (!w.isOpened()) {
      GTEST_SKIP() << "OpenCV cannot write test video (no FFmpeg backend)";
    }
    for (int i = 0; i < 5; ++i) {
      w.write(cv::Mat(48, 64, CV_8UC3, cv::Scalar(i * 40, 0, 0)));
    }
  }

  CaptureConfig c;
  c.url = path;
  c.decoder = "ffmpeg";
  c.reconnect_sec = 0.1;
  std::atomic<int> frames{0};
  std::atomic<int> width{0};
  std::atomic<int> opens{0};
  RtspCapture cap(
    c, [&](const cv::Mat & m) {width = m.cols; ++frames;},
    [&](const std::string & msg) {
      if (msg.rfind("streaming", 0) == 0) {
        ++opens;
      }
    });
  cap.start();
  const auto end = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < end && opens < 2) {
    std::this_thread::sleep_for(10ms);
  }
  cap.stop();
  std::remove(path.c_str());
  EXPECT_GE(opens.load(), 2);   // played through, reopened
  EXPECT_GE(frames.load(), 5);
  EXPECT_EQ(width.load(), 64);
  EXPECT_EQ(cap.activeDecoder(), "ffmpeg");
}
