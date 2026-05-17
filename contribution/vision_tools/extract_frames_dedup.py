# save as: /home/student24/robotproject/tools/extract_frames_dedup.py
import cv2, os, math, glob
from pathlib import Path

def average_hash(img, hash_size=8):
    resized = cv2.resize(img, (hash_size, hash_size), interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    mean = gray.mean()
    return (gray > mean).astype('uint8').flatten()

def hamming(a, b):
    return int((a != b).sum())

def extract(video_path, out_dir, fps_target=2, dedup=True, ham_thresh=5):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f'Open failed: {video_path}')
        return
    native_fps = cap.get(cv2.CAP_PROP_FPS) or 30
    step = max(1, int(round(native_fps / fps_target)))
    os.makedirs(out_dir, exist_ok=True)
    i = 0
    last_hash = None
    saved = 0
    while True:
        ret, frame = cap.read()
        if not ret: break
        if i % step == 0:
            if dedup:
                ah = average_hash(frame)
                if last_hash is None or hamming(ah, last_hash) > ham_thresh:
                    cv2.imwrite(os.path.join(out_dir, f'frame_{i:06d}.jpg'), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                    last_hash = ah
                    saved += 1
            else:
                cv2.imwrite(os.path.join(out_dir, f'frame_{i:06d}.jpg'), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                saved += 1
        i += 1
    cap.release()
    print(f'{video_path} -> {saved} frames')

if __name__ == "__main__":
    src_dir = "/home/student24/robotproject/raw_videos"   # 放各类视频的目录（按类分子文件夹更好）
    dst_dir = "/home/student24/robotproject/datasets/shapes/raw_frames"
    os.makedirs(dst_dir, exist_ok=True)
    for vp in glob.glob(os.path.join(src_dir, "**/*.mp4"), recursive=True):
        name = Path(vp).stem
        extract(vp, os.path.join(dst_dir, name), fps_target=2, dedup=True, ham_thresh=5)