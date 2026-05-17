import cv2
import yaml
import numpy as np
from typing import Dict, List, Tuple

HSVRange = Tuple[int, int, int]

class HSVColorEstimator:
    def __init__(self, yaml_path: str):
        with open(yaml_path, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        self.color_to_ranges: Dict[str, List[HSVRange]] = {}

        for k, v in cfg.items():
            if k == 'color_ratio_threshold':
                continue
            if isinstance(v, list) and all(isinstance(x, list) and len(x) == 6 for x in v):
                self.color_to_ranges[k] = [tuple(map(int, x)) for x in v]

        th_cfg = cfg.get('color_ratio_threshold', 0.18)

        if isinstance(th_cfg, dict):
            self.default_thresh = float(th_cfg.get('default', 0.18))
            self.color_thresh = {k: float(v) for k, v in th_cfg.items() if k != 'default'}
        else:
            self.default_thresh = float(th_cfg)
            self.color_thresh = {}

        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

    def _mask_for_color(self, hsv_img: np.ndarray, ranges: List[HSVRange]) -> np.ndarray:
        masks = []
        for (h1, s1, v1, h2, s2, v2) in ranges:
            lower = np.array([s1, v1], dtype=np.uint8)
            upper = np.array([s2, v2], dtype=np.uint8)

            if h1 <= h2:
                mask_h = cv2.inRange(hsv_img[:, :, 0], h1, h2)
            else:
                mask_h1 = cv2.inRange(hsv_img[:, :, 0], h1, 179)
                mask_h2 = cv2.inRange(hsv_img[:, :, 0], 0, h2)
                mask_h = cv2.bitwise_or(mask_h1, mask_h2)

            mask_sv = cv2.inRange(hsv_img[:, :, 1:], lower, upper)
            masks.append(cv2.bitwise_and(mask_h, mask_sv))

        mask_total = np.bitwise_or.reduce(masks)
        mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_OPEN, self.kernel)
        mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_CLOSE, self.kernel)
        return mask_total


    def _prepare_hsv(self, bgr_img: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
        hsv = cv2.GaussianBlur(hsv, (3, 3), 0)
        return hsv

    def estimate_from_roi(self, bgr_roi: np.ndarray):
        if bgr_roi.size == 0:
            return 'unknown', 0.0, {}
        hsv = self._prepare_hsv(bgr_roi)

        scores = {}
        total_pixels = hsv.shape[0] * hsv.shape[1]

        for name, ranges in self.color_to_ranges.items():
            mask_total = self._mask_for_color(hsv, ranges)
            ratio = float(mask_total.sum()) / (255.0 * total_pixels)
            scores[name] = ratio

        if not scores:
            return 'unknown', 0.0, {}

        best = max(scores, key=scores.get)
        ratio = scores[best]

        th = self.color_thresh.get(best, self.default_thresh)
        if ratio < th:
            return 'unknown', ratio, scores
        return best, ratio, scores

    def estimate_from_mask(self, bgr_img: np.ndarray, mask: np.ndarray):
        if bgr_img.size == 0 or mask.size == 0:
            return 'unknown', 0.0, {}

        hsv = self._prepare_hsv(bgr_img)
        valid = (mask > 0).astype(np.uint8)
        valid_count = int(valid.sum())

        if valid_count == 0:
            return 'unknown', 0.0, {}

        scores = {}
        for name, ranges in self.color_to_ranges.items():
            mask_total = self._mask_for_color(hsv, ranges)
            masked = cv2.bitwise_and(mask_total, mask_total, mask=valid)
            ratio = float(masked.sum()) / (255.0 * valid_count)
            scores[name] = ratio

        best = max(scores, key=scores.get)
        ratio = scores[best]

        if ratio < self.ratio_thresh:
            return 'unknown', ratio, scores
        return best, ratio, scores
