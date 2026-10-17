#!/usr/bin/env python3
"""A fake HIKMICRO PT series camera for tests: ISAPI over HTTP with digest authentication.

Simulates a PTZ that moves at finite speed (so "has it arrived" can be tested), presets,
auxiliaries (LIGHT, WIPER), day/night, one-push focus, JPEG snapshots and pixel-to-pixel
thermometry (jpegPicWithAppendData, float32 matrix with a hot spot). Every request is logged
as "METHOD path body" for the smoke test. Python 3.8, standard library only.
"""

import argparse
import base64
import hashlib
import json
import math
import re
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

NS = 'http://www.isapi.org/ver20/XMLSchema'
REALM = 'IP Camera(TEST)'
JPEG = base64.b64decode(
    '/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDABQODxIPDRQSEBIXFRQYHjIhHhwcHj0sLiQySUBMS0dARkVQWnNiUFVtVkVGZIhl'
    'bXd7gYKBTmCNl4x9lnN+gXz/2wBDARUXFx4aHjshITt8U0ZTfHx8fHx8fHx8fHx8fHx8fHx8fHx8fHx8fHx8fHx8fHx8fHx8'
    'fHx8fHx8fHx8fHx8fHz/wAARCAAYACADASIAAhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAA'
    'AgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3ODk6'
    'Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXG'
    'x8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/8QAHwEAAwEBAQEBAQEBAQAAAAAAAAECAwQFBgcICQoL/8QAtREA'
    'AgECBAQDBAcFBAQAAQJ3AAECAxEEBSExBhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYkNOEl8RcYGRomJygpKjU2Nzg5'
    'OkNERUZHSElKU1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6goOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPE'
    'xcbHyMnK0tPU1dbX2Nna4uPk5ebn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwDFFv7U8W/tWiLf2p4t/asKNY6nXM4W/tTx'
    'b+1aIt/ani39q9ijWM3XLYt/ani39qKK+VoyZ5jmx4t/ani39qKK9ijJmbmz/9k=')
THERMAL_W, THERMAL_H = 160, 120
HOT_X, HOT_Y, HOT_C = 100, 40, 85.5      # hot spot (a loose connector)
COLD_X, COLD_Y, COLD_C = 10, 110, 12.25


def md5(s):
    return hashlib.md5(s.encode()).hexdigest()


def xml(root, inner):
    return ('<?xml version="1.0" encoding="UTF-8"?>\n<%s xmlns="%s" version="2.0">%s</%s>'
            % (root, NS, inner, root))


def status_xml(code=1, string='OK', sub='ok'):
    return xml('ResponseStatus', '<requestURL></requestURL><statusCode>%d</statusCode>'
               '<statusString>%s</statusString><subStatusCode>%s</subStatusCode>'
               % (code, string, sub))


def tag(body, name):
    m = re.search(r'<%s>\s*([^<]*?)\s*</%s>' % (name, name), body)
    return m.group(1) if m else None


def temperature(x, y):
    """Ambient 25 C gradient, a hot spot and a cold spot."""
    t = 25.0 + 0.02 * x
    t += (HOT_C - 25.0) * math.exp(-((x - HOT_X) ** 2 + (y - HOT_Y) ** 2) / 8.0)
    t -= (25.0 - COLD_C) * math.exp(-((x - COLD_X) ** 2 + (y - COLD_Y) ** 2) / 8.0)
    return t


class Ptz:
    PAN_SPEED, TILT_SPEED, ZOOM_SPEED = 90.0, 45.0, 8.0   # deg/s, deg/s, x/s
    TILT_MIN, TILT_MAX, ZOOM_MIN, ZOOM_MAX = -90.0, 90.0, 1.0, 32.0

    def __init__(self):
        self.lock = threading.Lock()
        self.pan, self.tilt, self.zoom = 0.0, 0.0, 1.0
        self.target = None
        self.vel = (0.0, 0.0, 0.0)
        self.t = time.monotonic()

    def _step(self):
        now = time.monotonic()
        dt, self.t = now - self.t, now
        if self.target:
            tp, tt, tz = self.target
            dp = (tp - self.pan + 540.0) % 360.0 - 180.0
            self.pan = (self.pan + max(-self.PAN_SPEED * dt, min(self.PAN_SPEED * dt, dp))) % 360
            self.tilt += max(-self.TILT_SPEED * dt, min(self.TILT_SPEED * dt, tt - self.tilt))
            self.zoom += max(-self.ZOOM_SPEED * dt, min(self.ZOOM_SPEED * dt, tz - self.zoom))
        else:
            vp, vt, vz = self.vel
            self.pan = (self.pan + vp * dt) % 360.0
            self.tilt += vt * dt
            self.zoom += vz * dt
        self.tilt = max(self.TILT_MIN, min(self.TILT_MAX, self.tilt))
        self.zoom = max(self.ZOOM_MIN, min(self.ZOOM_MAX, self.zoom))

    def get(self):
        with self.lock:
            self._step()
            return self.pan, self.tilt, self.zoom

    def move(self, pan, tilt, zoom):
        with self.lock:
            self._step()
            self.target = (pan % 360.0, tilt, zoom if zoom and zoom > 0 else self.zoom)
            self.vel = (0.0, 0.0, 0.0)

    def continuous(self, p, t, z):
        with self.lock:
            self._step()
            self.target = None
            self.vel = (p / 100.0 * self.PAN_SPEED, t / 100.0 * self.TILT_SPEED,
                        z / 100.0 * self.ZOOM_SPEED)


class Camera:
    def __init__(self, args):
        self.args = args
        self.ptz = Ptz()
        self.presets = {}
        self.aux = {1: ['LIGHT', 'off'], 2: ['WIPER', 'off']}
        self.ircut = 'auto'
        self.nonce = md5(str(time.time()))
        self.log = open(args.log, 'a', buffering=1) if args.log else None
        self.log_lock = threading.Lock()

    def record(self, line):
        if self.log:
            with self.log_lock:
                self.log.write(line + '\n')


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    cam = None

    def log_message(self, fmt, *a):
        if self.cam.args.verbose:
            sys.stderr.write(fmt % a + '\n')

    # --- digest authentication (RFC 2617 / 7616, MD5, qop=auth) ---
    def authorized(self, method):
        h = self.headers.get('Authorization', '')
        if not h.startswith('Digest '):
            return False
        f = dict(re.findall(r'(\w+)="?([^",]*)"?', h[7:]))
        a = self.cam.args
        ha1 = md5('%s:%s:%s' % (a.user, REALM, a.password))
        ha2 = md5('%s:%s' % (method, f.get('uri', '')))
        if f.get('qop'):
            want = md5('%s:%s:%s:%s:%s:%s' % (ha1, f.get('nonce'), f.get('nc'),
                                             f.get('cnonce'), f.get('qop'), ha2))
        else:
            want = md5('%s:%s:%s' % (ha1, f.get('nonce'), ha2))
        return (f.get('username') == a.user and f.get('nonce') == self.cam.nonce and
                f.get('response') == want)

    def send(self, code, body, ctype='application/xml; charset="UTF-8"', extra=None):
        if isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def handle_any(self, method):
        n = int(self.headers.get('Content-Length', 0) or 0)
        body = self.rfile.read(n).decode('utf-8', 'replace') if n else ''
        if not self.authorized(method):
            self.send(401, status_xml(4, 'Invalid Operation', 'unAuthorized'), extra={
                'WWW-Authenticate': 'Digest realm="%s", nonce="%s", qop="auth", algorithm="MD5"'
                % (REALM, self.cam.nonce)})
            return
        self.cam.record('%s %s %s' % (method, self.path, ' '.join(body.split())))
        if self.cam.args.latency:
            time.sleep(self.cam.args.latency)
        try:
            self.route(method, self.path, body)
        except Exception as e:  # noqa: B902
            self.send(500, status_xml(3, 'Device Error', str(e)))

    def do_GET(self):
        self.handle_any('GET')

    def do_PUT(self):
        self.handle_any('PUT')

    def do_POST(self):
        self.handle_any('POST')

    def do_DELETE(self):
        self.handle_any('DELETE')

    def not_supported(self):
        self.send(403, json.dumps({'statusCode': 4, 'statusString': 'Invalid Operation',
                                   'subStatusCode': 'notSupport', 'errorCode': 1073741825,
                                   'errorMsg': 'notSupport'}), 'application/json')

    def route(self, method, path, body):
        cam, ptz = self.cam, self.cam.ptz
        p = path.split('?')[0]
        m = re.match(r'^/ISAPI/PTZCtrl/channels/1/(.*)$', p)
        if p == '/ISAPI/System/deviceInfo':
            return self.send(200, xml('DeviceInfo',
                '<deviceName>fake</deviceName><deviceID>1</deviceID><model>HM-TD5567T-15/W</model>'
                '<serialNumber>FAKE0001</serialNumber><macAddress>00:11:22:33:44:55</macAddress>'
                '<firmwareVersion>V5.5.0</firmwareVersion><firmwareReleasedDate>build 240101'
                '</firmwareReleasedDate><deviceType>IPCamera</deviceType>'))
        if m:
            leaf = m.group(1)
            if leaf == 'absoluteEx':
                if cam.args.no_absolute_ex:
                    return self.not_supported()
                if method == 'GET':
                    pan, tilt, zoom = ptz.get()
                    return self.send(200, xml('PTZAbsoluteEx',
                        '<elevation>%.3f</elevation><azimuth>%.3f</azimuth>'
                        '<absoluteZoom>%.2f</absoluteZoom>' % (tilt, pan, zoom)))
                ptz.move(float(tag(body, 'azimuth')), float(tag(body, 'elevation')),
                         float(tag(body, 'absoluteZoom') or 0))
                return self.send(200, status_xml())
            if leaf == 'absolute' and method == 'PUT':
                z = tag(body, 'absoluteZoom')
                ptz.move(int(tag(body, 'azimuth')) / 10.0, int(tag(body, 'elevation')) / 10.0,
                         int(z) / 10.0 if z else 0)
                return self.send(200, status_xml())
            if leaf == 'status':
                pan, tilt, zoom = ptz.get()
                return self.send(200, xml('PTZStatus',
                    '<AbsoluteHigh><elevation>%d</elevation><azimuth>%d</azimuth>'
                    '<absoluteZoom>%d</absoluteZoom></AbsoluteHigh>'
                    % (round(tilt * 10), round(pan * 10) % 3600, round(zoom * 10))))
            if leaf == 'continuous':
                ptz.continuous(int(tag(body, 'pan') or 0), int(tag(body, 'tilt') or 0),
                               int(tag(body, 'zoom') or 0))
                return self.send(200, status_xml())
            if leaf == 'presets' and method == 'GET':
                items = ''.join(
                    '<PTZPreset><enabled>true</enabled><id>%d</id><presetName>%s</presetName>'
                    '<AbsoluteHigh><elevation>%d</elevation><azimuth>%d</azimuth>'
                    '<absoluteZoom>%d</absoluteZoom></AbsoluteHigh></PTZPreset>'
                    % (i, nm, round(t * 10), round(pa * 10), round(z * 10))
                    for i, (nm, pa, t, z) in sorted(cam.presets.items()))
                return self.send(200, xml('PTZPresetList', items))
            pm = re.match(r'^presets/(\d+)(/goto)?$', leaf)
            if pm:
                pid = int(pm.group(1))
                if pm.group(2):
                    if pid not in cam.presets:
                        return self.send(400, status_xml(4, 'Invalid Operation', 'presetNotExist'))
                    _, pa, t, z = cam.presets[pid]
                    ptz.move(pa, t, z)
                    return self.send(200, status_xml())
                if method == 'DELETE':
                    cam.presets.pop(pid, None)
                    return self.send(200, status_xml())
                pan, tilt, zoom = ptz.get()
                cam.presets[pid] = (tag(body, 'presetName') or str(pid), pan, tilt, zoom)
                return self.send(200, status_xml())
            if leaf == 'auxcontrols' and method == 'GET':
                items = ''.join('<PTZAux><id>%d</id><type>%s</type><status>%s</status></PTZAux>'
                                % (i, t, s) for i, (t, s) in sorted(cam.aux.items()))
                return self.send(200, xml('PTZAuxList', items))
            am = re.match(r'^auxcontrols/(\d+)$', leaf)
            if am and method == 'PUT':
                aid = int(am.group(1))
                if aid not in cam.aux:
                    return self.send(400, status_xml(6, 'Invalid XML Content', 'badParameters'))
                cam.aux[aid][1] = tag(body, 'status') or 'off'
                return self.send(200, status_xml())
            if leaf == 'onepushfoucs/start':
                return self.send(200, status_xml())
            return self.not_supported()
        if p == '/ISAPI/Image/channels/1/ircutFilter':
            if method == 'GET':
                return self.send(200, xml('IrcutFilter', '<IrcutFilterType>%s</IrcutFilterType>'
                                                         '<nightToDayFilterLevel>4'
                                                         '</nightToDayFilterLevel>' % cam.ircut))
            mode = tag(body, 'IrcutFilterType')
            if mode not in ('auto', 'day', 'night', 'schedule'):
                return self.send(400, status_xml(6, 'Invalid XML Content', 'badParameters'))
            cam.ircut = mode
            return self.send(200, status_xml())
        if re.match(r'^/ISAPI/Streaming/channels/(101|201)/picture$', p):
            return self.send(200, JPEG, 'image/jpeg')
        if p == '/ISAPI/Thermal/channels/2/thermometry/jpegPicWithAppendData':
            return self.radiometric()
        return self.not_supported()

    def radiometric(self):
        temps = b''.join(struct.pack('<f', temperature(x, y))
                         for y in range(THERMAL_H) for x in range(THERMAL_W))
        meta = json.dumps({'JpegPictureWithAppendData': {
            'channel': 2, 'jpegPicLen': len(JPEG), 'jpegPicWidth': THERMAL_W,
            'jpegPicHeight': THERMAL_H, 'p2pDataLen': len(temps), 'isFreezedata': False,
            'temperatureDataLength': 4, 'visiblePicLen': len(JPEG)}}).encode()
        b = b'boundary'
        out = b''
        for ctype, data in (('application/json', meta), ('image/pjpeg', JPEG),
                            ('application/octet-stream', temps), ('image/pjpeg', JPEG)):
            out += (b'--' + b + b'\r\nContent-Type: ' + ctype.encode() +
                    b'\r\nContent-Length: ' + str(len(data)).encode() + b'\r\n\r\n' + data +
                    b'\r\n')
        out += b'--' + b + b'--\r\n'
        self.send(200, out, 'multipart/form-data; boundary=boundary')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--port', type=int, default=8080)
    ap.add_argument('--user', default='admin')
    ap.add_argument('--password', default='test12345')
    ap.add_argument('--no-absolute-ex', action='store_true',
                    help='answer notSupport to absoluteEx (basic PTZ only)')
    ap.add_argument('--latency', type=float, default=0.0, help='s added to every answer')
    ap.add_argument('--log', default='', help='append "METHOD path body" lines here')
    ap.add_argument('--verbose', action='store_true')
    args = ap.parse_args()
    Handler.cam = Camera(args)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    server.daemon_threads = True
    print('fake camera on http://127.0.0.1:%d' % args.port, flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
