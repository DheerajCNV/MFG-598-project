import os
import math
import numpy as np
from PIL import Image


pcd_path = "~/catkin_ws/src/drone_controller/bagfiles/bagfiles"
pcd_name = "102.964000000.pcd"
# pcd_mat = ".txt"
pcd_number = 1000


with open("~/catkin_ws/src/drone_controller/bagfiles/bagfiles/102.964000000.pcd") as pcd_file:
    lines = [line.strip().split(" ") for line in pcd_file.readlines()]

img_height = 480
img_width = 640
is_data = False
min_d = 0
max_d = 0
img_depth = np.zeros((img_height, img_width), dtype='f8')
for line in lines:
    if line[0] == 'DATA':  # skip the header
        is_data = True
        continue
    if is_data:
        d = max(0., float(line[2]))
        i = int(line[4])
        col = i % img_width
        row = math.floor(i / img_width)
        img_depth[row, col] = d
        min_d = min(d, min_d)
        max_d = max(d, max_d)

max_min_diff = max_d - min_d


def normalize(x):
    return 255 * (x - min_d) / max_min_diff
normalize = np.vectorize(normalize, otypes=[np.float])
img_depth = normalize(img_depth)
img_depth_file = Image.fromarray(img_depth)
img_depth_file.convert('RGB').save(os.path.join("~/catkin_ws/src/drone_controller/bagfiles",str(102.964000000)+'_depth_image.png'))