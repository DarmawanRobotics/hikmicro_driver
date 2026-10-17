#include "hikmicro_ptz_driver/radiometric.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>

namespace hikmicro_ptz_driver
{

namespace
{

std::string lower(std::string s)
{
  for (auto & c : s) {c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));}
  return s;
}

std::string trim(const std::string & s)
{
  size_t b = 0, e = s.size();
  while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) {++b;}
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) {--e;}
  return s.substr(b, e - b);
}

void fail(std::string * error, const std::string & msg)
{
  if (error) {
    *error = msg;
  }
}

// End of the header block starting at `pos`: returns the index of the body start.
size_t headersEnd(const std::string & s, size_t pos, size_t * header_len)
{
  const size_t crlf = s.find("\r\n\r\n", pos);
  const size_t lf = s.find("\n\n", pos);
  if (crlf != std::string::npos && (lf == std::string::npos || crlf <= lf)) {
    *header_len = crlf - pos;
    return crlf + 4;
  }
  if (lf != std::string::npos) {
    *header_len = lf - pos;
    return lf + 2;
  }
  return std::string::npos;
}

}  // namespace

std::string MultipartPart::header(const std::string & name) const
{
  auto it = headers.find(lower(name));
  return it == headers.end() ? std::string() : it->second;
}

std::optional<std::string> multipartBoundary(const std::string & content_type)
{
  const std::string l = lower(content_type);
  const size_t p = l.find("boundary=");
  if (p == std::string::npos) {
    return std::nullopt;
  }
  std::string b = content_type.substr(p + 9);
  const size_t semi = b.find(';');
  if (semi != std::string::npos) {
    b = b.substr(0, semi);
  }
  b = trim(b);
  if (b.size() >= 2 && b.front() == '"' && b.back() == '"') {
    b = b.substr(1, b.size() - 2);
  }
  if (b.empty()) {
    return std::nullopt;
  }
  return b;
}

std::vector<MultipartPart> parseMultipart(const std::string & body, const std::string & boundary)
{
  std::vector<MultipartPart> parts;
  const std::string delim = "--" + boundary;
  size_t pos = body.find(delim);
  while (pos != std::string::npos) {
    pos += delim.size();
    if (body.compare(pos, 2, "--") == 0) {
      break;  // closing delimiter
    }
    // rest of the delimiter line
    const size_t eol = body.find('\n', pos);
    if (eol == std::string::npos) {
      break;
    }
    const size_t hstart = eol + 1;
    size_t hlen = 0;
    const size_t bstart = headersEnd(body, hstart, &hlen);
    if (bstart == std::string::npos) {
      break;
    }

    MultipartPart part;
    size_t line_start = hstart;
    const size_t hend = hstart + hlen;
    while (line_start < hend) {
      size_t line_end = body.find('\n', line_start);
      if (line_end == std::string::npos || line_end > hend) {
        line_end = hend;
      }
      std::string line = body.substr(line_start, line_end - line_start);
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      const size_t colon = line.find(':');
      if (colon != std::string::npos) {
        part.headers[lower(trim(line.substr(0, colon)))] = trim(line.substr(colon + 1));
      }
      line_start = line_end + 1;
    }

    size_t next = std::string::npos;
    const std::string len = part.header("content-length");
    if (!len.empty()) {
      const long n = std::strtol(len.c_str(), nullptr, 10);
      if (n >= 0 && bstart + static_cast<size_t>(n) <= body.size()) {
        part.body = body.substr(bstart, static_cast<size_t>(n));
        next = body.find(delim, bstart + static_cast<size_t>(n));
      }
    }
    if (next == std::string::npos && part.body.empty()) {
      next = body.find(delim, bstart);
      size_t bend = next == std::string::npos ? body.size() : next;
      // the CRLF before the delimiter belongs to the delimiter
      if (bend >= bstart + 2 && body.compare(bend - 2, 2, "\r\n") == 0) {
        bend -= 2;
      } else if (bend >= bstart + 1 && body[bend - 1] == '\n') {
        bend -= 1;
      }
      part.body = body.substr(bstart, bend - bstart);
    }
    parts.push_back(std::move(part));
    pos = next;
  }
  return parts;
}

std::optional<RadiometricFrame> decodeRadiometric(
  const std::string & content_type, const std::string & body, std::string * error)
{
  const auto boundary = multipartBoundary(content_type);
  if (!boundary) {
    // Errors come back as plain XML/JSON.
    fail(error, "not a multipart answer (" + content_type + ")");
    return std::nullopt;
  }
  const auto parts = parseMultipart(body, *boundary);
  if (parts.empty()) {
    fail(error, "empty multipart answer");
    return std::nullopt;
  }

  // Identify parts by type, with the documented order as fallback:
  // JSON, thermal JPEG, temperature data, visible JPEG.
  const MultipartPart * meta = nullptr;
  const MultipartPart * data = nullptr;
  std::vector<const MultipartPart *> jpegs;
  for (const auto & p : parts) {
    const std::string ct = lower(p.contentType());
    const std::string head = trim(p.body.substr(0, 64));
    if (!meta && (ct.find("json") != std::string::npos ||
      (ct.empty() && !head.empty() && head[0] == '{')))
    {
      meta = &p;
    } else if (ct.find("jpeg") != std::string::npos || ct.find("jpg") != std::string::npos) {
      jpegs.push_back(&p);
    } else if (!data && (ct.find("octet") != std::string::npos || ct.empty())) {
      data = &p;
    }
  }
  if (!meta && !parts.empty()) {
    meta = &parts[0];
  }
  if (!data && parts.size() >= 3) {
    data = &parts[2];
  }

  const auto j = nlohmann::json::parse(meta->body, nullptr, false);
  if (j.is_discarded() || !j.contains("JpegPictureWithAppendData")) {
    fail(error, "missing JpegPictureWithAppendData metadata");
    return std::nullopt;
  }
  const auto & m = j["JpegPictureWithAppendData"];
  RadiometricFrame f;
  f.width = m.value("jpegPicWidth", 0);
  f.height = m.value("jpegPicHeight", 0);
  f.data_length = m.value("temperatureDataLength", 4);
  const size_t pixels = static_cast<size_t>(std::max(f.width, 0)) * std::max(f.height, 0);
  if (pixels == 0 || !data) {
    fail(error, "no temperature matrix in the answer");
    return std::nullopt;
  }
  if (f.data_length != 4 && f.data_length != 2) {
    fail(error, "unsupported temperatureDataLength " + std::to_string(f.data_length));
    return std::nullopt;
  }
  const size_t need = pixels * static_cast<size_t>(f.data_length);
  if (data->body.size() < need) {
    fail(error, "temperature matrix too short: " + std::to_string(data->body.size()) + " of " +
      std::to_string(need) + " bytes");
    return std::nullopt;
  }

  f.celsius.resize(pixels);
  const auto * raw = reinterpret_cast<const unsigned char *>(data->body.data());
  if (f.data_length == 4) {
    // float32, little endian (the cameras are little-endian ARM)
    for (size_t i = 0; i < pixels; ++i) {
      uint32_t bits = static_cast<uint32_t>(raw[4 * i]) |
        (static_cast<uint32_t>(raw[4 * i + 1]) << 8) |
        (static_cast<uint32_t>(raw[4 * i + 2]) << 16) |
        (static_cast<uint32_t>(raw[4 * i + 3]) << 24);
      float v;
      std::memcpy(&v, &bits, sizeof(v));
      f.celsius[i] = v;
    }
  } else {
    // 16 bit raw: kelvin = raw / scale + offset
    const double scale = m.value("scale", 1.0) > 0.0 ? m.value("scale", 1.0) : 1.0;
    const double offset = m.value("offset", 0.0);
    for (size_t i = 0; i < pixels; ++i) {
      const uint16_t r = static_cast<uint16_t>(raw[2 * i] | (raw[2 * i + 1] << 8));
      f.celsius[i] = static_cast<float>(r / scale + offset - 273.15);
    }
  }
  if (!jpegs.empty()) {
    f.thermal_jpeg = jpegs[0]->body;
  }
  if (jpegs.size() >= 2) {
    f.visible_jpeg = jpegs[1]->body;
  }
  return f;
}

std::optional<TemperatureStats> regionStats(
  const RadiometricFrame & frame, int x1, int y1, int x2, int y2)
{
  if (!frame.valid()) {
    return std::nullopt;
  }
  const auto clampX = [&](int v) {return std::min(std::max(v, 0), frame.width - 1);};
  const auto clampY = [&](int v) {return std::min(std::max(v, 0), frame.height - 1);};
  TemperatureStats s;
  s.x1 = clampX(std::min(x1, x2));
  s.x2 = clampX(std::max(x1, x2));
  s.y1 = clampY(std::min(y1, y2));
  s.y2 = clampY(std::max(y1, y2));
  s.max = -std::numeric_limits<float>::infinity();
  s.min = std::numeric_limits<float>::infinity();
  double sum = 0.0;
  size_t n = 0;
  for (int y = s.y1; y <= s.y2; ++y) {
    for (int x = s.x1; x <= s.x2; ++x) {
      const float v = frame.at(x, y);
      sum += v;
      ++n;
      if (v > s.max) {s.max = v; s.max_x = x; s.max_y = y;}
      if (v < s.min) {s.min = v; s.min_x = x; s.min_y = y;}
    }
  }
  s.mean = static_cast<float>(sum / static_cast<double>(n));
  return s;
}

}  // namespace hikmicro_ptz_driver
