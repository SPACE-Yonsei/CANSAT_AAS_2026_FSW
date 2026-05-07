"""Camera device abstraction with graceful degrade."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import time


@dataclass
class CameraHandle:
    cam: object
    encoder: object
    available: bool
    output_dir: Path


def _default_output_dir() -> Path:
    out = Path("PICAM_Video")
    out.mkdir(parents=True, exist_ok=True)
    return out


def init_cam() -> tuple[CameraHandle, object]:
    """Initialize camera backend.

    Returns a handle even when hardware backend is unavailable.
    """
    output_dir = _default_output_dir()
    try:
        from picamera2 import Picamera2  # type: ignore
        from picamera2.encoders import H264Encoder  # type: ignore

        cam = Picamera2()
        conf = cam.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
        cam.configure(conf)
        cam.start()
        enc = H264Encoder()
        handle = CameraHandle(cam=cam, encoder=enc, available=True, output_dir=output_dir)
        return handle, enc
    except Exception:
        handle = CameraHandle(cam=None, encoder=None, available=False, output_dir=output_dir)
        return handle, None


def _timestamped_name(ext: str) -> str:
    return f"P_{datetime.now().strftime('%m%d_%H%M%S_%f')}.{ext}"


def record(cam_handle: CameraHandle, enc, sec: float) -> Path | None:
    """Record one segment.

    - Real camera backend: attempts mp4/h264 output.
    - Fallback backend: creates placeholder file for pipeline continuity.
    """
    if cam_handle is None:
        return None

    if cam_handle.available:
        try:
            from picamera2.outputs import FfmpegOutput, FileOutput  # type: ignore

            mp4_path = cam_handle.output_dir / _timestamped_name("mp4")
            out_path: Path = mp4_path
            try:
                output = FfmpegOutput(str(mp4_path))
            except Exception:
                h264_path = cam_handle.output_dir / _timestamped_name("h264")
                out_path = h264_path
                output = FileOutput(str(h264_path))
            cam_handle.cam.start_recording(enc, output)
            time.sleep(max(0.0, float(sec)))
            cam_handle.cam.stop_recording()
            return out_path
        except Exception:
            # fall through to placeholder when backend errors at runtime
            pass

    placeholder = cam_handle.output_dir / _timestamped_name("txt")
    placeholder.write_text("camera unavailable; placeholder segment\n", encoding="utf-8")
    time.sleep(max(0.0, float(sec)))
    return placeholder


def terminate(cam_handle: CameraHandle | None):
    if cam_handle is None:
        return
    if cam_handle.available and cam_handle.cam is not None:
        try:
            cam_handle.cam.close()
        except Exception:
            pass
