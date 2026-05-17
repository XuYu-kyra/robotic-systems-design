import os
import shutil
import random
from pathlib import Path
import argparse

# 使用方法：
# python3 tools/split_yolo_dataset.py \
#   --images /home/student24/robotproject/datasets/shapes/images_all \
#   --labels /home/student24/robotproject/datasets/shapes/labels_all \
#   --out /home/student24/robotproject/datasets/shapes \
#   --train 0.8 --val 0.1 --test 0.1


def pair_exists(img_path: Path, labels_dir: Path) -> bool:
    stem = img_path.stem
    label = labels_dir / f"{stem}.txt"
    return label.exists()


def main(images_dir: str, labels_dir: str, out_root: str, ratios):
    images_dir = Path(images_dir)
    labels_dir = Path(labels_dir)
    out_root = Path(out_root)

    assert images_dir.exists(), f"images_dir 不存在: {images_dir}"
    assert labels_dir.exists(), f"labels_dir 不存在: {labels_dir}"

    img_paths = []
    for ext in ("*.jpg", "*.jpeg", "*.png", "*.bmp"):
        img_paths.extend(images_dir.glob(ext))
    img_paths = [p for p in img_paths if pair_exists(p, labels_dir)]

    random.shuffle(img_paths)
    n = len(img_paths)
    n_train = int(n * ratios[0])
    n_val = int(n * ratios[1])
    n_test = n - n_train - n_val

    splits = {
        'train': img_paths[:n_train],
        'val': img_paths[n_train:n_train + n_val],
        'test': img_paths[n_train + n_val:]
    }

    for split in ['train', 'val', 'test']:
        (out_root / f"images/{split}").mkdir(parents=True, exist_ok=True)
        (out_root / f"labels/{split}").mkdir(parents=True, exist_ok=True)

    for split, paths in splits.items():
        for img_p in paths:
            stem = img_p.stem
            lbl_p = labels_dir / f"{stem}.txt"
            dst_img = out_root / f"images/{split}/{img_p.name}"
            dst_lbl = out_root / f"labels/{split}/{stem}.txt"
            shutil.copy2(img_p, dst_img)
            shutil.copy2(lbl_p, dst_lbl)

    print(f"总样本: {n} -> train {len(splits['train'])}, val {len(splits['val'])}, test {len(splits['test'])}")


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--images', required=True, help='包含全部图片的目录（未划分）')
    ap.add_argument('--labels', required=True, help='对应 YOLO 标签目录（同名 .txt）')
    ap.add_argument('--out', required=True, help='输出根目录，里面将创建 images/ 和 labels/ 的子目录')
    ap.add_argument('--train', type=float, default=0.8)
    ap.add_argument('--val', type=float, default=0.1)
    ap.add_argument('--test', type=float, default=0.1)
    args = ap.parse_args()

    s = args.train + args.val + args.test
    assert abs(s - 1.0) < 1e-6, 'train/val/test 比例之和必须为1'

    main(args.images, args.labels, args.out, (args.train, args.val, args.test))
