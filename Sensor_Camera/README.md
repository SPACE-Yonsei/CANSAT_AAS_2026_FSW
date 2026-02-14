
## 하드웨어 설명 ##

1) 컨테이너 내부를 촬영하는 카메라
   라즈베리파이 카메라 모듈3 wide 모듈
   CSI 연결
   https://www.devicemart.co.kr/goods/view?no=14933041&srsltid=AfmBOorDyELdARatq3CNLnmsH2FBQo-R_73Pz7g0wLyOtexw_ksDPR6A

## 작동 ##

rpicam-hello --list-cameras

sudo nano /boot/firmware/config.txt
[all]
camera_auto_detect=0
dtoverlay=ov5647



점유하는 카메라 프로세스 확인
sudo fuser -v /dev/video0 /dev/media0 /dev/media1

ps -fp <PID>

sudo kill <PID>




## Windows에서 영상 재생이 안 될 때 ##

- **원인**: 예전에는 raw H.264를 .mp4로 저장해서 Windows 기본 플레이어가 인식하지 못함.
- **코드 수정**: `picam.py`에서 `FfmpegOutput` 사용 시 **진짜 MP4 컨테이너**로 저장되어 Windows에서 재생 가능.
- **라즈베리 파이에 ffmpeg 설치** (MP4 저장을 위해 필요):
  ```bash
  sudo apt update && sudo apt install -y ffmpeg
  ```
- **이미 받은 영상이 재생 안 되면** (Windows에서 변환):
  ```bash
  ffmpeg -i P_0214_123456.mp4 -c copy P_0214_123456_fixed.mp4
  ```
  또는 **VLC**로 재생 시도 (raw H.264도 재생 가능한 경우 있음).

h264->mp4 변환
ffmpeg -framerate 30 -i test.h264 -c copy test.mp4

scp pi@192.168.1.103:/home/pi/CANSAT_AAS_2026_FSW/test.mp4 "$env:USERPROFILE\Downloads\"





FFmpeg 설치
https://www.gyan.dev/ffmpeg/builds/
 여기에서 ffmpeg-releasae-essentials.zip
 다운로드 후 압축 해제
 ffmpeg-7.1.1-essentials_build\bin 을 시스템 환경변수 > PATH에 추가
 scp로 파일 다운로드
 scp pi@raspberrypi:~/(영상경로)/recorded_video.mjpeg
 영상 변환
ffmpeg -i recorded_video.mjpeg recorded_video.mp4




