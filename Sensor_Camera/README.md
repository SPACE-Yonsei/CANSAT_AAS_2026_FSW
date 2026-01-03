

## 공지사항 ##
https://github.com/jmpark3972/Camera_Control 의 파일들을
https://github.com/SPACE-Yonsei/CANSAT_AAS_2025_CAMERA로 그대로 fork했습니다.
수정 시에는 https://github.com/jmpark3972/Camera_Control 수정 후 https://github.com/SPACE-Yonsei/CANSAT_AAS_2025_CAMERA  에서 update branch 부탁드립니다.

## 하드웨어 설명 ##

1) 컨테이너 내부를 촬영하는 카메라
   라즈베리파이 카메라 모듈3 wide 모듈
   CSI 연결
   https://www.devicemart.co.kr/goods/view?no=14933041&srsltid=AfmBOorDyELdARatq3CNLnmsH2FBQo-R_73Pz7g0wLyOtexw_ksDPR6A

## 작동 ##
USB_csi_camera 폴더의 start.py를 실행하면 됩니다.
start_desc.py는 같은 내용이지만 주석으로 설명이 추가되었습니다.

csi_camera_0321.py 돌리면 csi만 잘 됨

csi_usb_mfeg.py는 usb까지.

FFmpeg 설치
https://www.gyan.dev/ffmpeg/builds/
 여기에서 ffmpeg-releasae-essentials.zip
 다운로드 후 압축 해제
 ffmpeg-7.1.1-essentials_build\bin 을 시스템 환경변수 > PATH에 추가
 scp로 파일 다운로드
 scp pi@raspberrypi:~/(영상경로)/recorded_video.mjpeg
 영상 변환
ffmpeg -i recorded_video.mjpeg recorded_video.mp4




