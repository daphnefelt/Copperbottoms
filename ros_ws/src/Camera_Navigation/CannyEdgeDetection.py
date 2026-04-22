## CANNY EDGE DETECTION IMPLEMENTATION
import numpy as np

# Gray scale function
def grayscale(image):
	return (image[...,0] * .1140 + # blue
			image[..., 1] * .5870 + # green
			image[..., 2]) * .2989 # red

def gaussian_blur(image, kernel_size=5, sigma=1):
	# Get coordinates
	coords = np.linspace(-(kernel_size // 2), kernel_size // 2, kernel_size)
	x, y = np.meshgrid(coords, coords)

	# Function
	gaussian = (1/(2*np.pi*(sigma**2))) * np.exp(-(x**2 + y**2) / (2 * sigma **2))

	# Norm
	norm = gaussian/np.sum(gaussian)

	# Convolve
	blurred_img = np.zeros_like(image)
	for i in range(image.shape[0]):
		for j in range(image.shape[1]):
			blurred_img[i, j] = np.sum(image[i:i+kernel_size, j:j+kernel_size] * norm)

	return blurred_img

def sobel_filters(image):
	Kx = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]])
	Ky = np.array([[1, 2, 1], [0, 0, 0], [-1, -2, -1]])

	x = np.zeros_like(image)
	y = np.zeros_like(image)
	for i in range(image.shape[0]):
		for j in range(image.shape[1]):
			x[i, j] = np.sum(image[i:i+3, j:j+3] * Kx)
			y[i, j] = np.sum(image[i:i+3, j:j+3] * Ky)
	
	return x, y

def gradient_calculations(x, y):
	grad_magnitude = np.sqrt(x**2 + y**2)
	grad_direction = np.arctan2(y, x) * (180. / np.pi)
	grad_direction[grad_direction < 0] += 180
	return grad_magnitude, grad_direction

def non_maximum_suppression(grad_magnitude, grad_direction):
	m, n = grad_magnitude.shape
	nms_img = np.zeros((m, n), dtype=np.float32)
	angle = grad_direction * (180 / np.pi)
	angle[angle < 0] += 180

	for i in range(1, m-1):
		for j in range(1, n-1):

			if (0 <= angle[i, j] < 22.5) or (157.5 <= angle[i, j] <= 180):
				if (grad_magnitude[i, j] >= grad_magnitude[i, j+1]) and (grad_magnitude[i, j] >= grad_magnitude[i, j-1]):
					nms_img[i, j] = grad_magnitude[i, j]
			elif (22.5 <= angle[i, j] < 67.5):
				if (grad_magnitude[i, j] >= grad_magnitude[i+1, j-1]) and (grad_magnitude[i, j] >= grad_magnitude[i-1, j+1]):
					nms_img[i, j] = grad_magnitude[i, j]
			elif (67.5 <= angle[i, j] < 112.5):
				if (grad_magnitude[i, j] >= grad_magnitude[i+1, j]) and (grad_magnitude[i, j] >= grad_magnitude[i-1, j]):
					nms_img[i, j] = grad_magnitude[i, j]
			elif (112.5 <= angle[i, j] < 157.5):
				if (grad_magnitude[i, j] >= grad_magnitude[i-1, j-1]) and (grad_magnitude[i, j] >= grad_magnitude[i+1, j+1]):
					nms_img[i, j] = grad_magnitude[i, j]
	return nms_img

def double_threshold(nms_img, low_threshold=50, high_threshold=100):
	edges = np.zeros_like(nms_img, dtype=np.uint8)
	for i in range(nms_img.shape[0]):
		for j in range(nms_img.shape[1]):
			if nms_img[i, j] >= high_threshold:
				edges[i, j] = 255
			elif nms_img[i, j] >= low_threshold:
				edges[i, j] = 128
	return edges

def hysteresis(threshold_img):
	for i in range(1, threshold_img.shape[0]-1):
		for j in range(1, threshold_img.shape[1]-1):
			if threshold_img[i, j] == 128:
				if ((threshold_img[i+1, j-1] == 255) or (threshold_img[i+1, j] == 255) or (threshold_img[i+1, j+1] == 255) or
					(threshold_img[i, j-1] == 255) or (threshold_img[i, j+1] == 255) or
					(threshold_img[i-1, j-1] == 255) or (threshold_img[i-1, j] == 255) or (threshold_img[i-1, j+1] == 255)):
					threshold_img[i, j] = 255
				else:
					threshold_img[i, j] = 0
	return threshold_img

class CannyEdgeDetection():

	def __init__(self, image):
		self.image = image

	def detect_edges(self):	
		gray_img = grayscale(self.image)
		gaussian_img = gaussian_blur(gray_img, kernel_size=5, sigma=1)
		sobel_x, sobel_y = sobel_filters(gaussian_img)
		self.angle_x, self.angle_y = sobel_x, sobel_y
		gradient = gradient_calculations(sobel_x, sobel_y)
		grad_magnitude, grad_direction = gradient
		supressed_img = non_maximum_suppression(grad_magnitude, grad_direction)
		threshold_img = double_threshold(supressed_img, low_threshold=50, high_threshold=100)
		tracked_img = hysteresis(threshold_img)
		return tracked_img




	


