# 더 안정적인 베이스: ros:noetic (Ubuntu 20.04)
FROM ros:noetic

ENV DEBIAN_FRONTEND=noninteractive

# 타임존/로케일
RUN apt-get update && apt-get install -y --no-install-recommends \
    tzdata locales && \
    ln -fs /usr/share/zoneinfo/Asia/Seoul /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata && \
    locale-gen en_US.UTF-8 ko_KR.UTF-8 && \
    update-locale LANG=en_US.UTF-8

# 데스크탑 풀 + MORAI bridge/빌드 의존성
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    ros-noetic-rosbridge-server \
    python3-bson-ext \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-pcl-ros \
    git curl wget nano vim tmux sudo bash-completion \
    build-essential cmake \
    python3-pip python3-dev \
    iputils-ping net-tools mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# YOLO 인지 스택 (Python 3.8 기준). 2026-08 vip2-noetic-pc 컨테이너에서
# 실제로 쓰던 버전을 그대로 고정했다 — torch 2.4.1 이 py3.8 마지막 지원 버전.
# torch 는 기본 PyPI 인덱스라 nvidia-*-cu12 (CUDA 12.1) 런타임을 함께 끌어온다(~5GB).
RUN pip3 install --no-cache-dir --upgrade "pip<25" && \
    pip3 install --no-cache-dir \
      "torch==2.4.1" \
      "torchvision==0.19.1" \
      "ultralytics==8.4.92" \
      "opencv-python==5.0.0.93" \
      "numpy==1.24.4" \
      "pandas==2.0.3" \
      "polars==1.8.2" \
      "matplotlib==3.7.5"

# 편의 bash 설정
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo '[ -f /root/vip2/devel/setup.bash ] && source /root/vip2/devel/setup.bash' >> /root/.bashrc && \
    echo "cd /root/vip2 2>/dev/null || true" >> /root/.bashrc && \
    echo "export LANG=en_US.UTF-8" >> /root/.bashrc && \
    echo "export LC_ALL=en_US.UTF-8" >> /root/.bashrc && \
    echo 'export DISPLAY="${DISPLAY:-:0}"' >> /root/.bashrc && \
    echo "export QT_X11_NO_MITSHM=1" >> /root/.bashrc && \
    echo "unset LIBGL_ALWAYS_INDIRECT" >> /root/.bashrc

# 워크스페이스(호스트 레포 루트를 /root/vip2 에 마운트)
WORKDIR /root/vip2

# X11 공유메모리 이슈 회피 + WSLg 기본 디스플레이
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:0

CMD ["bash"]
