from datetime import datetime
import os, time, pathlib

# ──────────────────────────────────────────────
PICAM_VIDEO_DIR = pathlib.Path("PICAM_Video")
PICAM_VIDEO_NAME_HEADER = "P"
PICAM_RECORDING = True
RECORD_SEC = 5
FPS = 30
FRAME_US = int(1_000_000 / FPS)
WIDTH, HEIGHT = 640, 480
# ──────────────────────────────────────────────

def init_cam():
    from picamera2 import Picamera2
    from picamera2.encoders import H264Encoder

    # Check if any cameras are available before initializing
    try:
        cam = Picamera2()
        
        # Verify camera is actually available
        if not cam.camera_properties:
            print("No camera detected (camera_properties empty)")
            cam.close()
            return None, None
            
    except IndexError:
        # "list index out of range" means no cameras found
        print("No camera detected (IndexError)")
        return None, None
    except Exception as e:
        print(f"Camera initialization failed: {e}")
        return None, None

    try:
        cfg = cam.create_video_configuration(
            main={"size": (WIDTH, HEIGHT), "format": "RGB888"},
            controls={"FrameDurationLimits": (FRAME_US, FRAME_US)}
        )
        cam.configure(cfg)
        
        if not cam.camera_config:
            print("Camera configuration failed")
            cam.close()
            return None, None
        
        cam.start()

        enc = H264Encoder()

        return cam, enc
        
    except Exception as e:
        print(f"Camera setup failed: {e}")
        try:
            cam.close()
        except:
            pass
        return None, None

def record(cam, enc, sec: int):
    from picamera2.outputs import FileOutput
    global PICAM_RECORDING
    
    PICAM_VIDEO_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%m%d_%H%M%S")
    fname = PICAM_VIDEO_DIR / f"{PICAM_VIDEO_NAME_HEADER}_{stamp}.mp4"

    #cam.start()
    counter = 0
    cam.start_recording(enc, FileOutput(str(fname)))
    while counter < sec:
        if PICAM_RECORDING == False:
            break
        time.sleep(1)
        counter += 1
    cam.stop_recording()
    #cam.close()

def stop_record(cam):
    cam.stop_recording()
    return

def terminate(cam):
    if cam is None:
        return
    cam.close()


if __name__ == "__main__":
    cam, enc = init_cam()
    record(cam, enc, RECORD_SEC)
    terminate(cam)
