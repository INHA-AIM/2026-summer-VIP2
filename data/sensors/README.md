# MORAI 센서 프리셋

| 파일 | 용도 |
|------|------|
| `24.R2.H2/VIP2.json` | 하계 VIP2 팀 센서 세팅 (MORAI **24.R2.H2**) |

## 적용 (Windows MORAI)

1. 파일을 아래 경로에 복사 (또는 이미 있으면 Load):

   `MoraiLauncher_Win_Data/SaveFile/Sensor/24.R2.H2/VIP2.json`

2. MORAI → Sensor Setting → 해당 프리셋 로드  
3. Network Publisher 토픽·ON 상태는 [docs/setup.md](../../docs/setup.md) 표와 일치하는지 확인

## 권장 주기 (이 프리셋)

| 센서 | `sensorPeriod` | 실효 Hz | Topic |
|------|----------------|---------|--------|
| Camera | 0.1 | **10** | `/image_jpeg/compressed` |
| LiDAR 3D | 0.1 | **10** | `/lidar3D` |
| IMU | 0.1 | **10** | `/imu` |
| GPS | 0.2 | **5** | `/gps` |

**50 Hz는 비권장.** rosbridge WebSocket + LiDAR/Camera 부하로 지연·TF 중복·경로 이탈이 난다. 상세: [docs/troubleshooting.md](../../docs/troubleshooting.md)
