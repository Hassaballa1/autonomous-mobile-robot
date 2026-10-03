#!/usr/bin/env python3
"""Fine-tune YOLOv8n on the stairs / drop-off dataset and export it for the Raspberry Pi.

  pip install ultralytics
  python3 train_hazards.py --data hazards.yaml --epochs 100

The NCNN export runs much faster than PyTorch on the Pi's ARM CPU. Point the detector at it:
  ros2 run amr_perception yolo_detector --ros-args -p model:=/path/to/best_ncnn_model
"""
import argparse

from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', default='hazards.yaml')
    parser.add_argument('--epochs', type=int, default=100)
    parser.add_argument('--imgsz', type=int, default=320)
    parser.add_argument('--device', default=None, help='e.g. 0 for the first GPU, cpu, mps')
    args = parser.parse_args()

    model = YOLO('yolov8n.pt')  # start from COCO weights
    model.train(data=args.data, epochs=args.epochs, imgsz=args.imgsz, device=args.device,
                project='runs', name='hazards')
    metrics = model.val()
    print(f'mAP50: {metrics.box.map50:.3f}  mAP50-95: {metrics.box.map:.3f}')

    path = model.export(format='ncnn', imgsz=args.imgsz)
    print(f'Exported NCNN model for the Pi: {path}')


if __name__ == '__main__':
    main()
