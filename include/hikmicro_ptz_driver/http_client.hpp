// Small libcurl wrapper for ISAPI: HTTP(S) with digest authentication (RFC 7616, what the
// camera requires), a pool of reused handles so the digest nonce is not renegotiated on every
// poll, and per-request timeouts. Thread-safe.
#pragma once

#include <mutex>
#include <string>
#include <vector>

typedef void CURL;

namespace hikmicro_ptz_driver
{

struct HttpResponse
{
  long status{0};              // HTTP status, 0 when the request never got an answer
  std::string content_type;
  std::string body;
  std::string error;           // transport error (timeout, refused, ...)

  bool ok() const {return error.empty() && status >= 200 && status < 300;}
};

class HttpClient
{
public:
  struct Options
  {
    std::string base_url;        // e.g. http://192.168.1.64:80
    std::string user;
    std::string password;
    double connect_timeout{2.0}; // s
    double timeout{3.0};         // s, whole request
    bool verify_tls{false};      // cameras ship self-signed certificates
    size_t pool_size{4};
  };

  explicit HttpClient(Options options);
  ~HttpClient();

  HttpClient(const HttpClient &) = delete;
  HttpClient & operator=(const HttpClient &) = delete;

  // `path` starts with '/'. `timeout` <= 0 uses Options::timeout.
  HttpResponse request(
    const std::string & method, const std::string & path, const std::string & body = "",
    const std::string & content_type = "application/xml; charset=\"UTF-8\"",
    double timeout = 0.0);

  HttpResponse get(const std::string & path, double timeout = 0.0)
  {
    return request("GET", path, "", "", timeout);
  }
  HttpResponse put(const std::string & path, const std::string & body = "")
  {
    return request("PUT", path, body);
  }

  const Options & options() const {return options_;}

private:
  CURL * acquire();
  void release(CURL * handle);

  Options options_;
  std::mutex pool_mtx_;
  std::vector<CURL *> pool_;
};

// Percent-encoding for credentials in RTSP URLs (RFC 3986 unreserved characters kept).
std::string urlEncode(const std::string & s);

}  // namespace hikmicro_ptz_driver
