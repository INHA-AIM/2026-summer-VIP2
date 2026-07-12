# Docker (ROS Noetic) — Windows MORAI + Docker Desktop

레포 루트를 `/root/vip2`에 마운트한다.  
MORAI는 Bridge **`127.0.0.1:9090`** (rosbridge WebSocket).

> `network_mode: host` 는 Docker Desktop에서 Windows localhost로 9090이 안 열리는 경우가 많아 **쓰지 않는다**.  
> compose는 `9090:9090` 등 포트를 publish 한다.

```bash
# 레포 루트에서
export VIP2="$(pwd)"
cd docker
docker compose -f docker-compose.pc.yaml up -d --build
docker ps --filter "name=vip2-noetic-pc"
# PORTS 에 0.0.0.0:9090->9090/tcp 가 보여야 함
```

Windows에서:

```powershell
netstat -an | findstr 9090
```

컨테이너:

```bash
docker exec -it vip2-noetic-pc bash
cd /root/vip2
./init_setting.sh   # 최초 또는 recreate 후
./build.sh

# 터미널1 (하나만)
./scripts/bridge.sh 9090

# 다른 터미널들
./scripts/camera.sh
./scripts/lidar.sh
source set_mode.sh va
./scripts/lattice_planner.sh
```

MORAI: Bridge Connect → Publisher 토픽 `/gps` `/imu` `/Ego_topic` `/lidar3D` `/image_jpeg/compressed` → 제어 `/ctrl_cmd_0`.

```bash
rostopic echo /connected_clients
rostopic hz /lidar3D /costmap /gps
```

이미지 재빌드(의존성 변경 시):

```bash
docker compose -f docker-compose.pc.yaml build --no-cache
docker compose -f docker-compose.pc.yaml up -d --force-recreate
```

팀 절차 본문: [docs/setup.md](../docs/setup.md) · [docs/troubleshooting.md](../docs/troubleshooting.md)
