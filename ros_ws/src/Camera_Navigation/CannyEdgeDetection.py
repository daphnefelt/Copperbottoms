## CANNY EDGE DETECTION IMPLEMENTATI
import realsense2 as rs
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
		img[..., 2]) # red

		# Gaussian Blur
		# Kernel
		size = 5
		sigma = 1
	
		# Get coordinates
		coords = np.linspace(-(size // 2), size // 2, size)
		x, y = np.meshgrid(coords,coords)

		# Function
		gaussian = (1/(2*np.pi*(sigma**2))) * np.exp(-(x**2 + y**2) / (2 * sigma **2))

		# Norm
		norm = gaussian/np.sum(gaussian)

		# Convolve
		blurred_img = np.zeros_like(gray_img)
		for i in range(gray_img.shape[0]):
			for j in range(gray_img.shape[1]):
				blurred_img[i, j] = np.sum(gray_img[i:i+size, j:j+size] * norm)
		# Sobel Filters
		sobel_x = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]])
		sobel_y = np.array([[1, 2, 1], [0, 0, 0], [-1, -2, -1]])
		grad_x = np.zeros_like(blurred_img)
		grad_y = np.zeros_like(blurred_img)
		for i in range(blurred_img.shape[0]):
			for j in range(blurred_img.shape[1]):
				grad_x[i, j] = np.sum(blurred_img[i:i+3, j:j+3] * sobel_x)
				grad_y[i, j] = np.sum(blurred_img[i:i+3, j:j+3] * sobel_y)
		# Gradient Magnitude and Direction
		grad_magnitude = np.sqrt(grad_x**2 + grad_y**2)
		grad_direction = np.arctan2(grad_y, grad_x) * (180. / np.pi)
		# Non-Maximum Suppression
		nms_img = np.zeros_like(grad_magnitude)
		for i in range(1, grad_magnitude.shape[0]-1):
			for j in range(1, grad_magnitude.shape[1]-1):
				angle = grad_direction[i, j]
				if (0 <= angle < 22.5) or (157.5 <= angle <= 180):
					if (grad_magnitude[i, j] >= grad_magnitude[i, j+1]) and (grad_magnitude[i, j] >= grad_magnitude[i, j-1]):
						nms_img[i, j] = grad_magnitude[i, j]
				elif (22.5 <= angle < 67.5):
					if (grad_magnitude[i, j] >= grad_magnitude[i+1, j-1]) and (grad_magnitude[i, j] >= grad_magnitude[i-1, j+1]):
						nms_img[i, j] = grad_magnitude[i, j]
				elif (67.5 <= angle < 112.5):
					if (grad_magnitude[i, j] >= grad_magnitude[i+1, j]) and (grad_magnitude[i, j] >= grad_magnitude[i-1, j]):
						nms_img[i, j] = grad_magnitude[i, j]
				elif (112.5 <= angle < 157.5):
					if (grad_magnitude[i, j] >= grad_magnitude[i-1, j-1]) and (grad_magnitude[i, j] >= grad_magnitude[i+1, j+1]):
						nms_img[i, j] = grad_magnitude[i, j]
		# Double Thresholding
		high_threshold = 100
		low_threshold = 50
		edges = np.zeros_like(nms_img)
		for i in range(nms_img.shape[0]):
			for j in range(nms_img.shape[1]):
				if nms_img[i, j] >= high_threshold:
					edges[i, j] = 255
				elif nms_img[i, j] >= low_threshold:
					edges[i, j] = 128
		# Edge Tracking by Hysteresis
		for i in range(1, edges.shape[0]-1):
			for j in range(1, edges.shape[1]-1):
				if edges[i, j] == 128:
					if ((edges[i+1, j-1] == 255) or (edges[i+1, j] == 255) or (edges[i+1, j+1] == 255) or
						(edges[i, j-1] == 255) or (edges[i, j+1] == 255) or
						(edges[i-1, j-1] == 255) or (edges[i-1, j] == 255) or (edges[i-1, j+1] == 255)):
						edges[i, j] = 255
					else:
						edges[i, j] = 0

finally:
	pipe.stop()

