# Docker (ROS Noetic)

레포 루트를 `/root/vip2`에 마운트한다.

```bash
# 1. X11 (호스트)
xhost +local:docker

# 2. 빌드 / 실행 (레포 루트에서)
export VIP2="$(pwd)"   # 또는 절대 경로
cd docker
docker compose -f docker-compose.pc.yaml build
docker compose -f docker-compose.pc.yaml up -d
docker ps -a --filter "name=vip2-noetic-pc"

# 3. 접속
docker exec -it vip2-noetic-pc bash
# 컨테이너 안:
source /opt/ros/noetic/setup.bash
cd /root/vip2
./init_setting.sh   # 최초 1회
./build.sh
./scripts/start_system.sh
# 다른 터미널:
source set_mode.sh va
./scripts/lattice_planner.sh

# 4. 종료
docker compose -f docker-compose.pc.yaml down
xhost -local:docker
```
