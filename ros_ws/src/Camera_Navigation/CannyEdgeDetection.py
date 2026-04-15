## CANNY EDGE DETECTION IMPLEMENTATI

import numpy as np

W, H, FPS = 640, 480, 30

pipe = rs.pipeline()
cfg = rs.config()

cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)

pipe.start(cfg)

try:
	while True:
		frames = pipe.wait_for_frames(5000) 
		color = frames.get_color_frame()
		if not color:
			continue
		img = np.asanyarray(color.get_data())
		
		# Convert to grayscale
		gray_img = (img[...,0] * .1140 + # blue
		img[..., 1] * .5870 + # green
		img[..., 2] * .2989 # red

		# Gaussian Blur
		# Kernel
		size = 5
		sigma = 1
	
		# Get coordinates
		coords = np.linspace(-(size // 2), size // 2, size)
		x, y = np.meshgrid(coords,coords)

		# Function
		gaussian = (1/(2*np.pi*(sigma**2)) * np.exp(-(x**2 + y**2) / (2 * sigma **2))

		# Norm
		norm = gaussian/np.sum(gaussian)

		# Blur
		blur = convolve2d(gray_img, norm, mode='same', boundary='symm')try:                                                                                                   while True:                                                                                            frames = pipe.wait_for_frames(5000)                                                            color = frames.get_color_frame()                                                               if not color:                                                                                          continue                                                                               img = np.asanyarray(color.get_data())                                                                                                                                                         # Convert to grayscale                                                                         gray_img = (img[...,0] * .1140 + # blue                                                        img[..., 1] * .5870 + # green                                                                  img[..., 2] # red                                                                                                                                                                             # Gaussian Blur                                                                          
