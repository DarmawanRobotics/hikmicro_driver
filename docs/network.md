# Network setup

The camera ships with `192.168.1.64/24`. The computer running the driver needs an address in the
same subnet, or the camera gets a new one (SADP tool or the camera's web page).

## An address in the camera's subnet

Quick test, lost on reboot:

```bash
sudo ip addr add 192.168.1.10/24 dev eth0      # your interface: ip -br link
ping -c 3 192.168.1.64
```

Permanent, with netplan (`/etc/netplan/60-hikmicro.yaml`):

```yaml
network:
  version: 2
  ethernets:
    eth0:
      addresses:
        - 192.168.1.10/24
```

```bash
sudo netplan apply
```

## Ports

| Port | Use |
|---|---|
| 80 | ISAPI (HTTP, digest authentication), `port:=` |
| 554 | RTSP, `/Streaming/Channels/101` visible, `/201` thermal |

## Check before launching

```bash
curl --digest -u admin:<password> http://192.168.1.64/ISAPI/System/deviceInfo
ffprobe -v error -show_streams "rtsp://admin:<password>@192.168.1.64:554/Streaming/Channels/101" | grep -E 'codec_name|width'
```

A 401 means a wrong password; no answer means the address or the subnet is wrong. Several
failed logins lock the account for a while (camera setting).
