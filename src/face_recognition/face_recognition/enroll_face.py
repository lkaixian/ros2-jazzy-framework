"""Enroll an identity: ros2 run face_recognition enroll_face NAME IMAGE [--model ..]"""
import argparse

import cv2

from face_recognition.gallery import Gallery


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('name')
    ap.add_argument('image')
    ap.add_argument('--model', default='yolov11n-face.pt')
    ap.add_argument('--embed-model', default='yolo11n-cls.pt')
    ap.add_argument('--gallery', default='faces')
    a = ap.parse_args()
    from ultralytics import YOLO
    img = cv2.imread(a.image)
    if img is None:
        raise SystemExit(f'cannot read {a.image}')
    boxes = YOLO(a.model).predict(img, verbose=False)[0].boxes
    if len(boxes) != 1:
        raise SystemExit(f'expected exactly 1 face, found {len(boxes)}')
    x1, y1, x2, y2 = (int(v) for v in boxes[0].xyxy[0])
    emb = YOLO(a.embed_model).embed(img[y1:y2, x1:x2], verbose=False)[0].cpu().numpy()
    try:
        Gallery(a.gallery).add(a.name, emb)
    except ValueError as e:
        raise SystemExit(str(e))
    print(f'enrolled {a.name} -> {a.gallery}/{a.name}.npy')


if __name__ == '__main__':
    main()
