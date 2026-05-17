#!/usr/bin/env python3
import argparse
from pathlib import Path
from collections import defaultdict

def analyze_yolo_dataset(data_yaml_path: str):
    import yaml
    with open(data_yaml_path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    
    data_root = Path(cfg['path'])
    names = cfg['names']
    id_to_name = {k: v for k, v in names.items()}
    
    print(f"数据集路径: {data_root}")
    print(f"类别: {list(names.values())}\n")
    
    total_stats = defaultdict(lambda: {'images': set(), 'instances': 0})
    
    for split_name in ['train', 'val', 'test']:
        labels_dir = data_root / 'labels' / split_name
        if not labels_dir.exists():
            print(f"警告: {split_name} 标签目录不存在: {labels_dir}")
            continue
        
        stats = defaultdict(lambda: {'images': set(), 'instances': 0})
        label_files = list(labels_dir.glob('*.txt'))
        
        for lbl_file in label_files:
            with open(lbl_file, 'r') as f:
                classes_in_file = set()
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    parts = line.split()
                    if len(parts) >= 5:
                        cls_id = int(parts[0])
                        cls_name = id_to_name.get(cls_id, f'class_{cls_id}')
                        stats[cls_name]['instances'] += 1
                        classes_in_file.add(cls_name)
            
            for cls_name in classes_in_file:
                stats[cls_name]['images'].add(lbl_file.stem)
                total_stats[cls_name]['images'].add((split_name, lbl_file.stem))
        
        print(f"\n{split_name.upper()}:")
        print(f"  总标签文件数: {len(label_files)}")
        for cls_name in sorted(id_to_name.values()):
            img_count = len(stats[cls_name]['images'])
            inst_count = stats[cls_name]['instances']
            print(f"  {cls_name}: {img_count} 张图片, {inst_count} 个实例")
            total_stats[cls_name]['instances'] += inst_count
    
    print(f"\n总计:")
    for cls_name in sorted(id_to_name.values()):
        total_imgs = len(total_stats[cls_name]['images'])
        total_inst = total_stats[cls_name]['instances']
        print(f"  {cls_name}: {total_imgs} 张图片, {total_inst} 个实例")

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', default='/home/student24/robotproject/datasets/shapes/images/data.yaml',
                    help='data.yaml 路径')
    args = ap.parse_args()
    analyze_yolo_dataset(args.data)

