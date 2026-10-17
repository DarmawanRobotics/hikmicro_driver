// HikCamera end to end against test/fake_camera.py: real HTTP, digest authentication, PTZ
// moving at finite speed, presets, auxiliaries, day/night, snapshots, radiometric frames.
#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <string>
#include <thread>

#include "hikmicro_ptz_driver/camera.hpp"
#include "hikmicro_ptz_driver/motion.hpp"

#ifndef FAKE_CAMERA_PY
#define FAKE_CAMERA_PY "test/fake_camera.py"
#endif

using namespace hikmicro_ptz_driver;
using namespace std::chrono_literals;

namespace
{

int freePort()
{
  int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  ::bind(fd, reinterpret_cast<sockaddr *>(&a), sizeof(a));
  socklen_t len = sizeof(a);
  ::getsockname(fd, reinterpret_cast<sockaddr *>(&a), &len);
  ::close(fd);
  return ntohs(a.sin_port);
}

bool reachable(int port)
{
  int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons(static_cast<uint16_t>(port));
  a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  const bool ok = ::connect(fd, reinterpret_cast<sockaddr *>(&a), sizeof(a)) == 0;
  ::close(fd);
  return ok;
}

class FakeCamera
{
public:
  explicit FakeCamera(bool no_absolute_ex = false)
  {
    port_ = freePort();
    pid_ = ::fork();
    if (pid_ == 0) {
      const std::string port = std::to_string(port_);
      if (no_absolute_ex) {
        ::execlp("python3", "python3", FAKE_CAMERA_PY, "--port", port.c_str(),
          "--no-absolute-ex", static_cast<char *>(nullptr));
      } else {
        ::execlp("python3", "python3", FAKE_CAMERA_PY, "--port", port.c_str(),
          static_cast<char *>(nullptr));
      }
      ::_exit(127);
    }
    for (int i = 0; i < 100 && !reachable(port_); ++i) {
      std::this_thread::sleep_for(50ms);
    }
  }
  ~FakeCamera()
  {
    ::kill(pid_, SIGTERM);
    int st = 0;
    ::waitpid(pid_, &st, 0);
  }
  int port() const {return port_;}
  bool up() const {return reachable(port_);}

private:
  int port_{0};
  pid_t pid_{-1};
};

CameraConfig config(int port, const std::string & password = "test12345")
{
  CameraConfig c;
  c.host = "127.0.0.1";
  c.port = port;
  c.user = "admin";
  c.password = password;
  c.timeout = 2.0;
  return c;
}

// Polls the camera like the ROS action does until the detector decides.
ArrivalDetector::State waitArrival(HikCamera & cam, const PtzPose & target, bool check_zoom,
  ArrivalDetector * out = nullptr)
{
  ArrivalConfig cfg;
  cfg.check_zoom = check_zoom;
  cfg.timeout_sec = 10.0;
  cfg.stall_sec = 1.0;
  const auto t0 = std::chrono::steady_clock::now();
  ArrivalDetector det(target, cfg, 0.0);
  ArrivalDetector::State s = ArrivalDetector::MOVING;
  while (s == ArrivalDetector::MOVING) {
    std::this_thread::sleep_for(50ms);
    const auto p = cam.position();
    if (!p) {
      continue;
    }
    const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    s = det.update(t, *p);
  }
  if (out) {
    *out = det;
  }
  return s;
}

}  // namespace

TEST(CameraFake, DigestAuthAndDeviceInfo)
{
  FakeCamera fake;
  ASSERT_TRUE(fake.up());
  HikCamera cam(config(fake.port()));
  std::string err;
  const auto info = cam.deviceInfo(&err);
  ASSERT_TRUE(info) << err;
  EXPECT_EQ(info->model, "HM-TD5567T-15/W");

  HikCamera wrong(config(fake.port(), "nope"));
  EXPECT_FALSE(wrong.deviceInfo(&err));
  EXPECT_NE(err.find("HTTP 401"), std::string::npos) << err;
}

TEST(CameraFake, HighPrecisionMoveArrives)
{
  FakeCamera fake;
  HikCamera cam(config(fake.port()));
  ASSERT_TRUE(cam.probeHighPrecision());

  PtzPose target;
  target.pan = 45.125;
  target.tilt = -20.5;
  target.zoom = 4.0;
  std::string err;
  ASSERT_TRUE(cam.moveAbsolute(target, 0.0, &err)) << err;
  ArrivalDetector det(target, ArrivalConfig(), 0.0);
  EXPECT_EQ(waitArrival(cam, target, true, &det), ArrivalDetector::REACHED);
  const auto p = cam.position();
  ASSERT_TRUE(p);
  EXPECT_NEAR(p->pan, 45.125, 0.002);   // 0.001 deg resolution
  EXPECT_NEAR(p->tilt, -20.5, 0.002);
  EXPECT_NEAR(p->zoom, 4.0, 0.01);
}

TEST(CameraFake, BasicPtzFallbackAndStall)
{
  FakeCamera fake(true);   // camera without absoluteEx
  HikCamera cam(config(fake.port()));
  EXPECT_FALSE(cam.probeHighPrecision());

  PtzPose target;
  target.pan = 300.0;
  target.tilt = 10.0;
  std::string err;
  ASSERT_TRUE(cam.moveAbsolute(target, 0.0, &err)) << err;   // zoom kept
  EXPECT_EQ(waitArrival(cam, target, false), ArrivalDetector::REACHED);

  target.tilt = 120.0;   // beyond the end stop of the fake (90)
  ASSERT_TRUE(cam.moveAbsolute(target, 0.0, &err)) << err;
  ArrivalDetector det(target, ArrivalConfig(), 0.0);
  EXPECT_EQ(waitArrival(cam, target, false, &det), ArrivalDetector::STALLED);
  EXPECT_NEAR(det.last().tilt, 90.0, 0.11);
}

TEST(CameraFake, ContinuousAndStop)
{
  FakeCamera fake;
  HikCamera cam(config(fake.port()));
  cam.probeHighPrecision();
  const auto p0 = cam.position();
  ASSERT_TRUE(p0);
  ASSERT_TRUE(cam.continuous(50, 0, 0));
  std::this_thread::sleep_for(300ms);
  ASSERT_TRUE(cam.stop());
  const auto p1 = cam.position();
  std::this_thread::sleep_for(200ms);
  const auto p2 = cam.position();
  ASSERT_TRUE(p1 && p2);
  EXPECT_GT(angleDiff(p1->pan, p0->pan), 5.0);       // it turned
  EXPECT_NEAR(angleDiff(p2->pan, p1->pan), 0.0, 1e-6);  // and stopped
}

TEST(CameraFake, Presets)
{
  FakeCamera fake;
  HikCamera cam(config(fake.port()));
  cam.probeHighPrecision();
  std::string err;
  PtzPose a;
  a.pan = 120.0;
  a.tilt = -15.0;
  a.zoom = 2.0;
  ASSERT_TRUE(cam.moveAbsolute(a));
  ASSERT_EQ(waitArrival(cam, a, true), ArrivalDetector::REACHED);
  ASSERT_TRUE(cam.setPreset(3, "trafo <1>", &err)) << err;

  const auto list = cam.presets(&err);
  ASSERT_TRUE(list) << err;
  ASSERT_EQ(list->size(), 1u);
  EXPECT_EQ((*list)[0].id, 3);
  EXPECT_EQ((*list)[0].name, "trafo <1>");
  EXPECT_TRUE((*list)[0].has_position);
  EXPECT_NEAR((*list)[0].pose.pan, 120.0, 0.11);

  PtzPose b;
  b.pan = 10.0;
  ASSERT_TRUE(cam.moveAbsolute(b));
  ASSERT_EQ(waitArrival(cam, b, false), ArrivalDetector::REACHED);
  ASSERT_TRUE(cam.gotoPreset(3, &err)) << err;
  EXPECT_EQ(waitArrival(cam, (*list)[0].pose, true), ArrivalDetector::REACHED);

  EXPECT_FALSE(cam.gotoPreset(9, &err));
  EXPECT_NE(err.find("presetNotExist"), std::string::npos) << err;
  ASSERT_TRUE(cam.removePreset(3, &err)) << err;
  EXPECT_TRUE(cam.presets()->empty());
}

TEST(CameraFake, AuxDayNightFocus)
{
  FakeCamera fake;
  HikCamera cam(config(fake.port()));
  std::string err;
  ASSERT_TRUE(cam.setAux("light", true, &err)) << err;
  ASSERT_TRUE(cam.setAux("WIPER", true, &err)) << err;
  auto aux = cam.auxDevices();
  ASSERT_TRUE(aux);
  ASSERT_EQ(aux->size(), 2u);
  EXPECT_TRUE((*aux)[0].on);
  EXPECT_TRUE((*aux)[1].on);
  ASSERT_TRUE(cam.setAux("LIGHT", false));
  EXPECT_FALSE(cam.auxDevices()->at(0).on);
  EXPECT_FALSE(cam.setAux("HEATER", true, &err));
  EXPECT_NE(err.find("no HEATER"), std::string::npos) << err;

  EXPECT_EQ(cam.dayNight().value_or(""), "auto");
  ASSERT_TRUE(cam.setDayNight("night", &err)) << err;
  EXPECT_EQ(cam.dayNight().value_or(""), "night");
  EXPECT_FALSE(cam.setDayNight("disco", &err));

  EXPECT_TRUE(cam.onePushFocus(&err)) << err;
}

TEST(CameraFake, SnapshotAndRadiometric)
{
  FakeCamera fake;
  HikCamera cam(config(fake.port()));
  std::string err;
  const auto jpg = cam.snapshot(1, &err);
  ASSERT_TRUE(jpg) << err;
  EXPECT_EQ(static_cast<unsigned char>((*jpg)[0]), 0xFF);
  EXPECT_TRUE(cam.snapshot(2, &err));

  const auto f = cam.radiometric(&err);
  ASSERT_TRUE(f) << err;
  EXPECT_EQ(f->width, 160);
  EXPECT_EQ(f->height, 120);
  EXPECT_FALSE(f->thermal_jpeg.empty());
  EXPECT_FALSE(f->visible_jpeg.empty());
  const auto s = regionStats(*f, 0, 0, f->width - 1, f->height - 1);
  ASSERT_TRUE(s);
  EXPECT_NEAR(s->max, 85.5 + 0.02 * 100, 0.01);   // hot spot of the fake at (100, 40)
  EXPECT_EQ(s->max_x, 100);
  EXPECT_EQ(s->max_y, 40);
  EXPECT_EQ(s->min_x, 10);
  EXPECT_EQ(s->min_y, 110);
  EXPECT_LT(s->min, 13.0);
}

TEST(CameraFake, RawAndErrors)
{
  FakeCamera fake;
  HikCamera cam(config(fake.port()));
  const auto r = cam.raw("GET", "/ISAPI/System/deviceInfo");
  EXPECT_EQ(r.status, 200);
  EXPECT_NE(r.body.find("<model>"), std::string::npos);
  const auto bad = cam.raw("GET", "/ISAPI/Nope");
  EXPECT_EQ(bad.status, 403);

  HikCamera nobody(config(freePort()));   // nothing listens there
  std::string err;
  EXPECT_FALSE(nobody.position(&err));
  EXPECT_FALSE(err.empty());

  EXPECT_EQ(cam.rtspUrl(2), "rtsp://admin:test12345@127.0.0.1:554/Streaming/Channels/201");
  CameraConfig c = config(1, "p@ss:w/rd");
  EXPECT_EQ(HikCamera(c).rtspUrl(1, 2, 8554),
    "rtsp://admin:p%40ss%3Aw%2Frd@127.0.0.1:8554/Streaming/Channels/102");
}
