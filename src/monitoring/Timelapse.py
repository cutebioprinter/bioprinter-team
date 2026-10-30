#!/usr/bin/env python3

import os
import imageio
from glob import glob

PHOTO_FOLDER = '/home/pi/timelapse_frames'
OUTPUT_GIF = '/home/pi/Timelapses/GIF2.gif'
fps = 4

images = sorted(glob(os.path.join(PHOTO_FOLDER, '*.jpg')))

if not images:
    print("No images found")
    exit()
frames = []
for img_path in images:
    image = imageio.imread(img_path)
    frames.append(image)
frames = frames[11:20]
imageio.mimsave(OUTPUT_GIF, frames, duration=1/fps)
