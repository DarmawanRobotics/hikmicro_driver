// Pixel-to-pixel thermometry: GET /ISAPI/Thermal/channels/<ID>/thermometry/jpegPicWithAppendData
// ?format=json answers with a multipart body: JSON metadata (JpegPictureWithAppendData), the
// thermal JPEG, the temperature matrix and, on bi-spectrum cameras, the visible JPEG.
// The matrix is float32 degrees Celsius (temperatureDataLength 4) or 16-bit raw Kelvin with
// scale/offset (temperatureDataLength 2). No ROS here.
#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace hikmicro_ptz_driver
{

struct MultipartPart
{
  std::map<std::string, std::string> headers;  // lower-case names
  std::string body;

  std::string header(const std::string & name) const;
  std::string contentType() const {return header("content-type");}
};

// boundary=... from a Content-Type header (quotes removed); nullopt if missing.
std::optional<std::string> multipartBoundary(const std::string & content_type);

// Splits a multipart body. Honours per-part Content-Length (binary safe) and falls back to the
// next boundary; accepts CRLF and LF line endings.
std::vector<MultipartPart> parseMultipart(const std::string & body, const std::string & boundary);

struct RadiometricFrame
{
  int width{0};
  int height{0};
  std::vector<float> celsius;      // row-major, width * height
  std::string thermal_jpeg;
  std::string visible_jpeg;
  int data_length{0};              // bytes per pixel the camera sent

  float at(int x, int y) const {return celsius[static_cast<size_t>(y) * width + x];}
  bool valid() const
  {
    return width > 0 && height > 0 && celsius.size() == static_cast<size_t>(width) * height;
  }
};

std::optional<RadiometricFrame> decodeRadiometric(
  const std::string & content_type, const std::string & body, std::string * error = nullptr);

struct TemperatureStats
{
  float max{0.f};
  float min{0.f};
  float mean{0.f};
  int max_x{0}, max_y{0};
  int min_x{0}, min_y{0};
  int x1{0}, y1{0}, x2{0}, y2{0};  // pixel box actually used (inclusive, clamped)
};

// Statistics over the inclusive box (x1, y1)-(x2, y2), corners in any order, clamped to the
// frame. A point is a box with x1 == x2 and y1 == y2. nullopt for an invalid frame.
std::optional<TemperatureStats> regionStats(
  const RadiometricFrame & frame, int x1, int y1, int x2, int y2);

}  // namespace hikmicro_ptz_driver
