// Multipart parsing and jpegPicWithAppendData decoding (float32 and 16-bit Kelvin matrices).
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "hikmicro_ptz_driver/radiometric.hpp"

using namespace hikmicro_ptz_driver;

namespace
{

std::string part(const std::string & ctype, const std::string & data, bool with_length = true)
{
  std::string s = "--boundary\r\nContent-Type: " + ctype + "\r\n";
  if (with_length) {
    s += "Content-Length: " + std::to_string(data.size()) + "\r\n";
  }
  return s + "\r\n" + data + "\r\n";
}

std::string floats(int w, int h, float base, int hx, int hy, float hot)
{
  std::string out;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      float v = (x == hx && y == hy) ? hot : base + 0.01f * x;
      uint32_t bits;
      std::memcpy(&bits, &v, 4);
      for (int i = 0; i < 4; ++i) {
        out.push_back(static_cast<char>((bits >> (8 * i)) & 0xFF));
      }
    }
  }
  return out;
}

const std::string kJpeg = std::string("\xff\xd8\xff\xe0", 4) + "fake jpeg" +
  std::string("\xff\xd9", 2);

}  // namespace

TEST(Multipart, Boundary)
{
  EXPECT_EQ(multipartBoundary("multipart/form-data; boundary=boundary").value_or(""), "boundary");
  EXPECT_EQ(multipartBoundary("multipart/mixed; Boundary=\"a b\"; charset=x").value_or(""), "a b");
  EXPECT_FALSE(multipartBoundary("application/xml"));
}

TEST(Multipart, BinarySafeWithAndWithoutLength)
{
  // binary data containing CRLF and something that looks like a delimiter
  const std::string tricky = std::string("ab\r\n--boundar\0y", 15);
  const std::string body = part("application/octet-stream", tricky) +
    part("text/plain", "no length", false) + "--boundary--\r\n";
  const auto parts = parseMultipart(body, "boundary");
  ASSERT_EQ(parts.size(), 2u);
  EXPECT_EQ(parts[0].body, tricky);
  EXPECT_EQ(parts[0].contentType(), "application/octet-stream");
  EXPECT_EQ(parts[1].body, "no length");
}

TEST(Multipart, LfOnly)
{
  const std::string body =
    "--b\nContent-Type: text/plain\n\nhello\n--b\nContent-Type: text/plain\n\nworld\n--b--\n";
  const auto parts = parseMultipart(body, "b");
  ASSERT_EQ(parts.size(), 2u);
  EXPECT_EQ(parts[0].body, "hello");
  EXPECT_EQ(parts[1].body, "world");
}

TEST(Radiometric, Float32Frame)
{
  const int w = 16, h = 12;
  const std::string temps = floats(w, h, 20.0f, 5, 7, 91.25f);
  const std::string meta = R"({"JpegPictureWithAppendData":{"channel":2,"jpegPicLen":15,)"
    R"("jpegPicWidth":16,"jpegPicHeight":12,"p2pDataLen":768,"isFreezedata":false,)"
    R"("temperatureDataLength":4,"visiblePicLen":15}})";
  const std::string body = part("application/json", meta) + part("image/pjpeg", kJpeg) +
    part("application/octet-stream", temps) + part("image/pjpeg", kJpeg + "v") +
    "--boundary--\r\n";

  std::string err;
  const auto f = decodeRadiometric("multipart/form-data; boundary=boundary", body, &err);
  ASSERT_TRUE(f) << err;
  EXPECT_EQ(f->width, 16);
  EXPECT_EQ(f->height, 12);
  EXPECT_EQ(f->data_length, 4);
  EXPECT_FLOAT_EQ(f->at(5, 7), 91.25f);
  EXPECT_FLOAT_EQ(f->at(3, 0), 20.03f);
  EXPECT_EQ(f->thermal_jpeg, kJpeg);
  EXPECT_EQ(f->visible_jpeg, kJpeg + "v");

  auto all = regionStats(*f, 0, 0, w - 1, h - 1);
  ASSERT_TRUE(all);
  EXPECT_FLOAT_EQ(all->max, 91.25f);
  EXPECT_EQ(all->max_x, 5);
  EXPECT_EQ(all->max_y, 7);
  EXPECT_FLOAT_EQ(all->min, 20.0f);
  EXPECT_EQ(all->min_x, 0);

  auto pt = regionStats(*f, 5, 7, 5, 7);
  ASSERT_TRUE(pt);
  EXPECT_FLOAT_EQ(pt->max, 91.25f);
  EXPECT_FLOAT_EQ(pt->mean, 91.25f);

  // corners swapped and out of the frame: clamped
  auto box = regionStats(*f, 100, 100, 10, 10);
  ASSERT_TRUE(box);
  EXPECT_EQ(box->x1, 10);
  EXPECT_EQ(box->y1, 10);
  EXPECT_EQ(box->x2, w - 1);
  EXPECT_EQ(box->y2, h - 1);
}

TEST(Radiometric, Kelvin16Frame)
{
  // 2 x 1 pixels, raw/scale + offset Kelvin
  std::string temps;
  for (uint16_t raw : {uint16_t(2981), uint16_t(3731)}) {   // /10 -> 298.1 K, 373.1 K
    temps.push_back(static_cast<char>(raw & 0xFF));
    temps.push_back(static_cast<char>(raw >> 8));
  }
  const std::string meta = R"({"JpegPictureWithAppendData":{"jpegPicWidth":2,"jpegPicHeight":1,)"
    R"("temperatureDataLength":2,"scale":10,"offset":0,"p2pDataLen":4}})";
  const std::string body = part("application/json", meta) + part("image/jpeg", kJpeg) +
    part("application/octet-stream", temps) + "--boundary--\r\n";
  const auto f = decodeRadiometric("multipart/form-data; boundary=boundary", body);
  ASSERT_TRUE(f);
  EXPECT_NEAR(f->at(0, 0), 24.95f, 1e-3);
  EXPECT_NEAR(f->at(1, 0), 99.95f, 1e-3);
  EXPECT_TRUE(f->visible_jpeg.empty());
}

TEST(Radiometric, Errors)
{
  std::string err;
  EXPECT_FALSE(decodeRadiometric("application/xml", "<ResponseStatus/>", &err));
  EXPECT_NE(err.find("not a multipart"), std::string::npos);

  const std::string meta = R"({"JpegPictureWithAppendData":{"jpegPicWidth":4,"jpegPicHeight":4,)"
    R"("temperatureDataLength":4}})";
  const std::string body = part("application/json", meta) + part("image/jpeg", kJpeg) +
    part("application/octet-stream", "short") + "--boundary--\r\n";
  EXPECT_FALSE(decodeRadiometric("multipart/form-data; boundary=boundary", body, &err));
  EXPECT_NE(err.find("too short"), std::string::npos);

  RadiometricFrame empty;
  EXPECT_FALSE(regionStats(empty, 0, 0, 1, 1));
}
