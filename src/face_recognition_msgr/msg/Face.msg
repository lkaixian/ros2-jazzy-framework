float32 confidence        # detector confidence
float32 x_min
float32 y_min
float32 x_max
float32 y_max
string identity           # gallery name, or "unknown"
float32 similarity        # cosine similarity to best gallery match, [-1, 1]
float32[] embedding       # YOLO11 embedding of the face crop (may be empty)
