
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




