# Perception: your own detector

## Your own detector (bounding boxes only)
Plain YOLO11 boxes, no segmentation (a 4 GB GPU cannot afford SAM-class models). Verified on a GTX 1650: the nano detector runs ~10 ms/frame (~95 FPS) using ~100 MiB of VRAM, and a smoke fine-tune at batch 8 / 640 px peaked at **1.6 GB**.
```
ros2 run object_detection train_detector --data my_items.yaml --base yolo11n.pt --epochs 100   # defaults sized for 4 GB
# then in param/perception.yaml:  object_detection_node.model: <path to best.pt>
ros2 run bt_slm make_bt_dataset --labels @my_labels.txt --out-dir bt_dataset                   # SLM data with YOUR labels
```
The detector publishes its class names (`object_detection/classes`, latched). The validator then **rejects any `DetectObject label` the model cannot produce**, and the LLM prompt lists those labels plus every usable BT node with its description (`bt_validator/catalog`). Without a running detector set `bt_validator_node.labels` in `param/pipeline.yaml` or use `validate_bt --labels a,b,c`. Boxes: `ObjectArray` (`x_min,y_min,x_max,y_max` in pixels), annotated frames on `detections/image`.
Training needs matplotlib importable; if pip's NumPy 2 and apt's matplotlib clash, `train_detector` says so and `./setup.sh --only python` installs a compatible one.
