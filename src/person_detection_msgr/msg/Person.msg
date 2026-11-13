int32 track_id            # -1 when tracking is disabled
float32 confidence
float32 x_min
float32 y_min
float32 x_max
float32 y_max
# COCO-17 keypoints (yolo11*-pose models); empty when pose is unavailable
float32[] keypoints_x
float32[] keypoints_y
float32[] keypoints_conf
