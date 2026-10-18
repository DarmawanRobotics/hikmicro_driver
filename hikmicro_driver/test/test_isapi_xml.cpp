// ISAPI builders and parsers, partly against the example messages of the ISAPI document
// "Thermography thermal cameras PT series" (which carry <!-- --> comments inside the values).
#include <gtest/gtest.h>

#include <string>

#include "hikmicro_driver/isapi_xml.hpp"

using namespace hikmicro_driver;

TEST(IsapiXml, AbsoluteExBuildAndParse)
{
  PtzPose t;
  t.pan = 123.4567;
  t.tilt = -12.3456;
  t.zoom = 7.5;
  const std::string x = isapi::absoluteExXml(t);
  EXPECT_NE(x.find("<PTZAbsoluteEx xmlns=\"http://www.isapi.org/ver20/XMLSchema\""),
    std::string::npos);
  EXPECT_NE(x.find("<elevation>-12.346</elevation>"), std::string::npos);
  EXPECT_NE(x.find("<azimuth>123.457</azimuth>"), std::string::npos);
  EXPECT_NE(x.find("<absoluteZoom>7.50</absoluteZoom>"), std::string::npos);
  EXPECT_EQ(x.find("Speed"), std::string::npos);

  const auto p = isapi::parseAbsoluteEx(x);
  ASSERT_TRUE(p);
  EXPECT_NEAR(p->pan, 123.457, 1e-9);
  EXPECT_NEAR(p->tilt, -12.346, 1e-9);
  EXPECT_NEAR(p->zoom, 7.5, 1e-9);
}

TEST(IsapiXml, AbsoluteExKeepsZoomAndAddsSpeed)
{
  PtzPose t;
  t.pan = 10.0;
  t.zoom = 0.0;
  const std::string x = isapi::absoluteExXml(t, 25.0);
  EXPECT_EQ(x.find("absoluteZoom"), std::string::npos);
  EXPECT_NE(x.find("<horizontalSpeed>25.00</horizontalSpeed>"), std::string::npos);
  EXPECT_NE(x.find("<verticalSpeed>25.00</verticalSpeed>"), std::string::npos);
}

TEST(IsapiXml, AbsoluteExDocumentExample)
{
  // GET .../absoluteEx response from the document, comments included
  const std::string doc = R"(<?xml version="1.0" encoding="UTF-8"?>
  <PTZAbsoluteEx xmlns="http://www.isapi.org/ver20/XMLSchema" version="2.0">
    <!--ro, req, object, PTZ absolute parameters, attr:version{req, string}-->
    <elevation>
      <!--ro, opt, float, range of the tilt, range:[-90.000,270.000], step:0.001-->-45.125
    </elevation>
    <azimuth>
      <!--ro, opt, float, range of the pan, range:[0,360.000], step:0.001-->359.999
    </azimuth>
    <absoluteZoom>
      <!--ro, opt, float, range of the zoom, range:[0,10000]-->12.5
    </absoluteZoom>
    <zoomCoordinate><!--ro, opt, int-->0</zoomCoordinate>
  </PTZAbsoluteEx>)";
  const auto p = isapi::parseAbsoluteEx(doc);
  ASSERT_TRUE(p);
  EXPECT_DOUBLE_EQ(p->tilt, -45.125);
  EXPECT_DOUBLE_EQ(p->pan, 359.999);
  EXPECT_DOUBLE_EQ(p->zoom, 12.5);
}

TEST(IsapiXml, AbsoluteHighTenths)
{
  PtzPose t;
  t.pan = 359.96;   // rounds to 3600
  t.tilt = -45.04;
  t.zoom = 3.25;    // tenths: 33 (lround(32.5))
  const std::string x = isapi::absoluteHighXml(t);
  EXPECT_NE(x.find("<PTZData xmlns="), std::string::npos);
  EXPECT_NE(x.find("<AbsoluteHigh><elevation>-450</elevation><azimuth>3600</azimuth>"
    "<absoluteZoom>33</absoluteZoom></AbsoluteHigh>"), std::string::npos);
  t.zoom = 0.0;
  EXPECT_EQ(isapi::absoluteHighXml(t).find("absoluteZoom"), std::string::npos);
}

TEST(IsapiXml, PtzStatusDocumentExample)
{
  const std::string doc = R"(<?xml version="1.0" encoding="UTF-8"?>
  <PTZStatus xmlns="http://www.isapi.org/ver20/XMLSchema" version="2.0">
    <!--ro, opt, object, attr:version{req, string, protocolVersion}-->
    <AbsoluteHigh>
      <!--ro, opt, object, absolute height-->
      <elevation>
        <!--ro, opt, int, altitude, range:[-900,2700], desc:altitude-->-125
      </elevation>
      <azimuth>
        <!--ro, opt, int, panning angle, range:[0,3600], desc:panning angle-->1805
      </azimuth>
      <absoluteZoom>
        <!--ro, opt, int, zooming parameter, range:[0,1000], desc:zooming parameter-->40
      </absoluteZoom>
    </AbsoluteHigh>
  </PTZStatus>)";
  const auto p = isapi::parsePtzStatus(doc);
  ASSERT_TRUE(p);
  EXPECT_DOUBLE_EQ(p->tilt, -12.5);
  EXPECT_DOUBLE_EQ(p->pan, 180.5);
  EXPECT_DOUBLE_EQ(p->zoom, 4.0);
  EXPECT_FALSE(isapi::parsePtzStatus("<PTZStatus/>"));
  EXPECT_FALSE(isapi::parsePtzStatus("not xml"));
}

TEST(IsapiXml, Continuous)
{
  EXPECT_NE(isapi::continuousXml(60, -150, 0).find("<pan>60</pan><tilt>-100</tilt><zoom>0</zoom>"),
    std::string::npos);
}

TEST(IsapiXml, PresetListDocumentExample)
{
  const std::string doc = R"(<?xml version="1.0" encoding="UTF-8"?>
  <PTZPresetList xmlns="http://www.isapi.org/ver20/XMLSchema" version="2.0">
    <PTZPreset>
      <enabled><!--ro, req, bool-->true</enabled>
      <id><!--ro, req, string-->3</id>
      <presetName><!--ro, req, string-->trafo 1 bushing</presetName>
      <AbsoluteHigh>
        <elevation><!--ro, opt, int, range:[-900,2700]-->-300</elevation>
        <azimuth><!--ro, opt, int, range:[0,3600]-->900</azimuth>
        <absoluteZoom><!--ro, opt, int, range:[0,1000]-->100</absoluteZoom>
        <focus><!--ro, opt, int, range:[1,100000]-->1</focus>
      </AbsoluteHigh>
    </PTZPreset>
    <PTZPreset><enabled>false</enabled><id>test</id><presetName>x</presetName></PTZPreset>
    <PTZPreset><enabled>true</enabled><id>7</id><presetName>no pose</presetName></PTZPreset>
  </PTZPresetList>)";
  const auto list = isapi::parsePresetList(doc);
  ASSERT_EQ(list.size(), 2u);  // "test" is not a numeric id
  EXPECT_EQ(list[0].id, 3);
  EXPECT_EQ(list[0].name, "trafo 1 bushing");
  EXPECT_TRUE(list[0].has_position);
  EXPECT_DOUBLE_EQ(list[0].pose.pan, 90.0);
  EXPECT_DOUBLE_EQ(list[0].pose.tilt, -30.0);
  EXPECT_DOUBLE_EQ(list[0].pose.zoom, 10.0);
  EXPECT_EQ(list[1].id, 7);
  EXPECT_FALSE(list[1].has_position);
}

TEST(IsapiXml, PresetNameIsEscaped)
{
  const std::string x = isapi::presetXml(5, "CT <A> & B");
  EXPECT_NE(x.find("<id>5</id>"), std::string::npos);
  EXPECT_NE(x.find("<presetName>CT &lt;A&gt; &amp; B</presetName>"), std::string::npos);
}

TEST(IsapiXml, AuxListAndCommand)
{
  const std::string doc = R"(<PTZAuxList xmlns="http://www.isapi.org/ver20/XMLSchema" version="2.0">
    <PTZAux><id><!--ro, req, string, ID-->1</id><type><!--ro-->LIGHT</type><status>on</status></PTZAux>
    <PTZAux><id>2</id><type>wiper</type><status>off</status></PTZAux>
  </PTZAuxList>)";
  const auto list = isapi::parseAuxList(doc);
  ASSERT_EQ(list.size(), 2u);
  EXPECT_EQ(list[0].type, "LIGHT");
  EXPECT_TRUE(list[0].on);
  EXPECT_EQ(list[1].type, "WIPER");  // upper-cased
  EXPECT_FALSE(list[1].on);
  EXPECT_NE(isapi::auxXml(2, "WIPER", true).find("<id>2</id><type>WIPER</type><status>on</status>"),
    std::string::npos);
}

TEST(IsapiXml, Ircut)
{
  EXPECT_NE(isapi::ircutXml("night").find("<IrcutFilterType>night</IrcutFilterType>"),
    std::string::npos);
  const std::string doc = R"(<IrcutFilter xmlns="http://www.isapi.org/ver20/XMLSchema" version="2.0">
    <IrcutFilterType><!--ro, opt, enum-->auto</IrcutFilterType>
    <dayToNightFilterLevel>4</dayToNightFilterLevel><dayNightStatus>day</dayNightStatus>
  </IrcutFilter>)";
  EXPECT_EQ(isapi::parseIrcutMode(doc).value_or(""), "auto");
  const auto changed = isapi::setElementText(doc, "IrcutFilterType", "night");
  ASSERT_TRUE(changed);
  EXPECT_EQ(isapi::parseIrcutMode(*changed).value_or(""), "night");
  EXPECT_NE(changed->find("<dayToNightFilterLevel>4</dayToNightFilterLevel>"), std::string::npos);
  EXPECT_FALSE(isapi::setElementText(doc, "missing", "x"));
}

TEST(IsapiXml, DeviceInfo)
{
  const std::string doc = R"(<?xml version="1.0" encoding="UTF-8"?>
<DeviceInfo version="2.0" xmlns="http://www.isapi.org/ver20/XMLSchema">
<deviceName>Thermal PT</deviceName><deviceID>48443</deviceID><model>HM-TD5567T-15/W</model>
<serialNumber>HM-TD5567T-15/W20240101AAWRF12345678</serialNumber>
<macAddress>bc:5e:33:00:11:22</macAddress><firmwareVersion>V5.5.61</firmwareVersion>
<firmwareReleasedDate>build 240315</firmwareReleasedDate><deviceType>IPCamera</deviceType>
</DeviceInfo>)";
  const auto d = isapi::parseDeviceInfo(doc);
  ASSERT_TRUE(d);
  EXPECT_EQ(d->model, "HM-TD5567T-15/W");
  EXPECT_EQ(d->firmware, "V5.5.61");
  EXPECT_EQ(d->firmware_date, "build 240315");
  EXPECT_EQ(d->mac, "bc:5e:33:00:11:22");
}

TEST(IsapiXml, StatusXmlAndJson)
{
  const std::string ok = R"(<ResponseStatus xmlns="http://www.isapi.org/ver20/XMLSchema" version="2.0">
    <requestURL>/ISAPI/PTZCtrl/channels/1/absoluteEx</requestURL>
    <statusCode>1</statusCode><statusString>OK</statusString><subStatusCode>ok</subStatusCode>
  </ResponseStatus>)";
  auto s = isapi::parseStatus(ok);
  EXPECT_TRUE(s.parsed);
  EXPECT_TRUE(s.ok());

  // the error example of the document (section 4.2.6)
  const std::string err = R"({
    "requestURL": "/ISAPI/Event/triggers/notifications/channels/whiteLightAlarm",
    "statusCode": 4,
    "statusString": "Invalid Operation",
    "subStatusCode": "notSupport",
    "errorCode": 1073741825,
    "errorMsg": "notSupport"
  })";
  s = isapi::parseStatus(err);
  EXPECT_TRUE(s.parsed);
  EXPECT_FALSE(s.ok());
  EXPECT_EQ(s.sub_status_code, "notSupport");
  EXPECT_EQ(isapi::describeError(403, err), "HTTP 403: Invalid Operation (notSupport)");
  EXPECT_EQ(isapi::describeError(401, ""), "HTTP 401: wrong user or password");
  EXPECT_FALSE(isapi::parseStatus("\xff\xd8\xff\xe0 jpeg bytes").parsed);
}
