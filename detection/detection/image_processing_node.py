#!/usr/bin/env python3
import cv2
import numpy as np
import os
import math
import random
from abc import ABC, abstractmethod
from itertools import permutations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading  # Import the threading module
from queue import Queue, Empty  # Import Queue for thread communication
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult


# --- Helper Functions and Classes ---

def calculate_angle(v1, v2):
    """Calculates the angle between two vectors."""
    dot_product = np.dot(v1, v2)
    norm_v1 = np.linalg.norm(v1)
    norm_v2 = np.linalg.norm(v2)

    if norm_v1 * norm_v2 == 0:
        return 0
    cos_angle = np.clip(dot_product / (norm_v1 * norm_v2), -1.0, 1.0)
    angle_rad = np.arccos(cos_angle)
    return np.degrees(angle_rad)

class Point:
    """Represents a 2D point."""
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def to_tuple(self):
        return (self.x, self.y)

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)  # Return a Point object

    def __repr__(self):
        return f"Point({self.x},{self.y})"
    
    def to_numpy(self):
        return np.array([self.x, self.y])

class TwoDShapeExtractor:
    """Checks if a contour is a rectangle, square, or parallelogram."""

    def __init__(self, angle_tolerance=10.0, side_angle_tolerance_parallelogram=20.0):
        self.angle_tolerance = angle_tolerance
        self.side_angle_tolerance_parallelogram = side_angle_tolerance_parallelogram

    def is_rectangle_or_square(self, ordered_points):
        """Checks if four points form a rectangle or square."""
        for i in range(4):
            p1 = ordered_points[i]
            p2 = ordered_points[(i + 1) % 4]
            p3 = ordered_points[(i + 2) % 4]

            v1 = np.array(p1) - np.array(p2)
            v2 = np.array(p3) - np.array(p2)
            angle_deg = calculate_angle(v1, v2)

            if not (abs(angle_deg - 90) <= self.angle_tolerance):
                return False
        return True

    def is_parallelogram(self, ordered_points):
        """Checks if four points form a parallelogram."""
        v1 = np.array(ordered_points[1]) - np.array(ordered_points[0])
        v2 = np.array(ordered_points[2]) - np.array(ordered_points[1])
        v3 = np.array(ordered_points[3]) - np.array(ordered_points[2])
        v4 = np.array(ordered_points[0]) - np.array(ordered_points[3])
        v3 = -v3  # Negate for opposite side comparison
        v4 = -v4

        angle1 = np.degrees(np.arctan2(v1[1], v1[0]))
        angle2 = np.degrees(np.arctan2(v2[1], v2[0]))
        angle3 = np.degrees(np.arctan2(v3[1], v3[0]))
        angle4 = np.degrees(np.arctan2(v4[1], v4[0]))

        if not (abs(angle1 - angle3) <= self.side_angle_tolerance_parallelogram or abs(abs(angle1 - angle3) - 180) <= self.side_angle_tolerance_parallelogram):
            return False
        if not (abs(angle2 - angle4) <= self.side_angle_tolerance_parallelogram or abs(abs(angle2 - angle4) - 180) <= self.side_angle_tolerance_parallelogram):
            return False
        return True

    def is_box(self, contour):
        """Checks if a contour is a rectangle/square/parallelogram."""
        approx = cv2.approxPolyDP(contour, 0.014 * cv2.arcLength(contour, True), True)
        if len(approx) != 4:
            return False

        ordered_rect_points = VertexOrderer.order_vertices_centroid_angle(contour)
        return self.is_rectangle_or_square(ordered_rect_points) or self.is_parallelogram(ordered_rect_points)

    def detect_ellipse(self, contour):

        if len(contour) >= 5:  # fitEllipse requires at least 5 points
            ellipse = cv2.fitEllipse(contour)
            return ellipse
        else:
            return None   

    def is_circle_ellipse(self, updated_contour):

        ellipse = self.detect_ellipse(updated_contour)
        if ellipse:
            (x,y), (MA, ma), angle = ellipse
            a = MA/2.0
            b = ma/2.0
            area_ellipse = np.pi*a*b
            area_contour = cv2.contourArea(updated_contour)
            area_ratio = area_ellipse / area_contour if area_contour != 0 else 0 

            if area_contour >= 150.0:
                if 0.9 <= area_ratio <= 1.111:
                    if 0.9 <= a/b <= 1.111:
                        print("valid circle found")
                    else:
                        print("valid ellipse found")
                    return True, updated_contour
            else:
                print("background noise")
        return False, updated_contour
            


class VertexOrderer:
    """Orders vertices of a contour based on centroid angle."""

    @staticmethod
    def order_vertices_centroid_angle(contour):
        """Orders vertices based on their angle relative to the centroid."""
        rect_points = np.reshape(cv2.approxPolyDP(contour, 0.014 * cv2.arcLength(contour, True), True), (4, 2))

        M = cv2.moments(contour)
        if M["m00"] == 0:
            return rect_points.tolist()  # Fallback: Return original points

        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        centroid = Point(cX, cY)

        angles = []
        initial_angle = None

        for point_arr in rect_points:
            point = Point(point_arr[0], point_arr[1]) # use Point object
            vector = point - centroid  # Use Point object subtraction
            angle = np.arctan2(vector.y, vector.x)

            if initial_angle is None:
                initial_angle = angle
            angles.append(angle)

        if initial_angle is None: # handle if initial_angle still none
             return rect_points.tolist()

        # relative_angles = [(angle - initial_angle) % (2 * np.pi) for angle in angles]
        relative_angles = []
        for angle in angles:
            relative_angle_rad = angle - initial_angle 
            normalized_angle = np.fmod(relative_angle_rad, 2 * np.pi)
            if normalized_angle < 0: 
                normalized_angle += 2 * np.pi
            relative_angles.append(normalized_angle)

        indexed_vertices = sorted(enumerate(rect_points), key=lambda iv: relative_angles[iv[0]])
        ordered_vertices = [vertex for index, vertex in indexed_vertices]
        return [vertex.tolist() for vertex in ordered_vertices]


class ContourFilter:
    """Filters contours based on various criteria."""

    def __init__(self, min_area=300.0, max_area=25000.0, min_edges=4, shape_extractor: TwoDShapeExtractor = None):
        self.min_area = min_area
        self.max_area = max_area
        self.min_edges = min_edges
        self.shape_extractor = shape_extractor or TwoDShapeExtractor() # Use composition
        self.area_with_more_edges = 3000.0 # Initialize

    def filter_contours(self, contours, rows, cols):
        """Filters contours and returns indices of those to remove."""
        filtered_contours = []
        contours_to_remove_indices = []
        contour_edge_counts = []
       
        for i, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            approx = cv2.approxPolyDP(contour, 0.014 * cv2.arcLength(contour, True), True)
            num_edges_approx = len(approx)

            is_box = True
            if num_edges_approx == 4:
                is_box = self.shape_extractor.is_box(contour)  # Use TwoDShapeExtractor

            is_border_pixel_present = self._check_border_pixel(contour, rows, cols)

            c2 = area < self.min_area
            c3 = area > self.max_area
            c4 = not is_box
            c5 = num_edges_approx < 4
            c6 = is_border_pixel_present


            if c2 or c3 or c4 or c5 or c6:
                contours_to_remove_indices.append(i)
            else:
                filtered_contours.append(contour)
                contour_edge_counts.append(num_edges_approx)
       
        return filtered_contours, contours_to_remove_indices, contour_edge_counts

    def _check_border_pixel(self, contour, rows, cols):
        """Checks if any pixel of the contour touches the image border."""
        blank = np.zeros((rows, cols), dtype=np.uint8)
        cv2.drawContours(blank, contour, -1, 255, thickness=cv2.FILLED)
        y_indices, x_indices = np.where(blank == 255)
        for y, x in zip(y_indices, x_indices):
            if x == 0 or x == cols - 1 or y == 0 or y == rows - 1:
                return True  # Border pixel found
        return False

class MaskModifier:
    """Modifies a mask by removing specified contours."""

    def generate_new_mask_by_removing_undesired_contours(self, mask, contours, contours_to_remove_indices):
        """Modifies the mask by drawing filled black contours."""
        modified_mask = mask.copy()
        for index in contours_to_remove_indices:
            cv2.drawContours(modified_mask, [contours[index]], -1, 0, thickness=cv2.FILLED)
        return modified_mask


class ContourFinder:
    """Finds contours in a binary image."""

    def __init__(self, retrieval_mode=cv2.RETR_EXTERNAL, approximation_method=cv2.CHAIN_APPROX_SIMPLE):
        self.retrieval_mode = retrieval_mode
        self.approximation_method = approximation_method

    def find_contours(self, binary_image):
        """Finds contours in the given binary image."""
        contours, _ = cv2.findContours(binary_image, self.retrieval_mode, self.approximation_method)
        return contours, _

class ImageLoader:
    """Loads an image from a file."""

    def __init__(self, image_path, ros_image):
        rows, cols = ros_image.shape[:2]
        self.image_path = image_path

    def load_image(self, ros_image):
        """Loads the image and returns it.  Returns None on error."""
        rows, cols = ros_image.shape[:2]
        # if the img is blank, image is not sent by the camera -> read img from path
        if np.sum(ros_image == 0) == rows * cols:  # blank image
            image = cv2.imread(self.image_path)
        else:
            image = ros_image
        if image is None:
            print(f"Error: Could not load image from {self.image_path}")
        return image

class ImagePreprocessor:
    """Preprocesses an image (e.g., blurring)."""

    def __init__(self, use_blur=True, kernel_size=5, sigma=1.5):
        self.use_blur = use_blur
        self.kernel_size = kernel_size
        self.sigma = sigma

    def preprocess(self, image):
        """Applies Gaussian blur if enabled."""
        if self.use_blur:
            return cv2.GaussianBlur(image, (self.kernel_size, self.kernel_size), self.sigma)
        return image


class BaseImageClusterer(ABC):
    """Abstract base class for image clustering."""

    def __init__(self, image_path, ros_image,  K=15, use_blur=True, kernel_size=5, sigma=1.5, output_dir="output", grid_size=20):
        self.image_loader = ImageLoader(image_path, ros_image)  # Composition
        self.preprocessor = ImagePreprocessor(use_blur, kernel_size, sigma)  # Composition
        self.K = K
        self.output_dir = output_dir
        self.grid_size = grid_size  # Grid size
        self.image_bgr = None
        self.ros_image = ros_image
        os.makedirs(self.output_dir, exist_ok=True) # ensure output directory exist

    def load_and_preprocess(self):
        """Loads and preprocesses the image."""
        self.image_bgr = self.image_loader.load_image(self.ros_image)
        if self.image_bgr is not None:
            self.image_bgr = self.preprocessor.preprocess(self.image_bgr)
        return self.image_bgr

    @abstractmethod
    def convert_color_space(self, image):
        """Converts the image to a specific color space (abstract)."""
        pass

    def kmeans_clustering(self, image):
        """Performs K-means clustering on the image."""
        rows, cols, _ = image.shape
        image_transformed = self.convert_color_space(image)
        image_flat = image_transformed.reshape((-1, 3)).astype(np.float32)

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1.0)
        attempts = 5
        _, labels, centers = cv2.kmeans(image_flat, self.K, None, criteria, attempts, cv2.KMEANS_PP_CENTERS)

        pixel_belongs_to_which_cluster = labels.reshape((rows, cols))
        segmented_flat = centers[labels.flatten()]
        segmented_image = segmented_flat.reshape(image.shape).astype(np.uint8) # correct shape
        return pixel_belongs_to_which_cluster, segmented_image, centers

    def run(self):
      # will be defined in the inherited classes
      pass

class BGRClusterer(BaseImageClusterer):
    """Performs clustering in the BGR color space."""

    def convert_color_space(self, image):
        return image  # No conversion needed


class HSVClusterer(BaseImageClusterer):
    """Performs clustering in the HSV color space."""

    def convert_color_space(self, image):
        return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


class LabClusterer(BaseImageClusterer):
    """Performs clustering in the Lab color space."""

    def convert_color_space(self, image):
        return cv2.cvtColor(image, cv2.COLOR_BGR2Lab)
    

class ConnectedRectangleDetector:
    """Detects connected rectangles within a contour."""

    def __init__(self, angle_threshold=20.0):
        self.angle_threshold = angle_threshold

    def detect_connected_rectangles_vertices(self, contour):
        """Detects vertices of connected rectangles within a contour."""
        approx = cv2.approxPolyDP(contour, 0.014 * cv2.arcLength(contour, True), True)
        points_list_L = [tuple(pt[0]) for pt in approx]
        n_points = len(points_list_L)

        if n_points < 4:
            return None  # Not enough points to form a rectangle

        angle_90_count = 0

        for i_origin in range(n_points):
            origin_point_P_i = points_list_L[i_origin]
            found_90_angle_for_origin = False

            other_points_indices = [idx for idx in range(n_points) if idx != i_origin]
            point_permutations = permutations(other_points_indices, 2)

            for indices_j_k in point_permutations:
                i_j, i_k = indices_j_k
                point_P_j = points_list_L[i_j]
                point_P_k = points_list_L[i_k]

                v1 = np.array(point_P_j) - np.array(origin_point_P_i)
                v2 = np.array(point_P_k) - np.array(origin_point_P_i)

                angle_deg = calculate_angle(v1, v2)
                if abs(angle_deg - 90) <= self.angle_threshold:
                    angle_90_count +=1
                    break
        
        vertices = None
        if angle_90_count >= n_points -1 or angle_90_count < n_points + 1:
            vertices = points_list_L
            return vertices

        return vertices # return vertices if found, else return None

    def draw_circle_with_vertices(self, final_cluster, vertices):
        """Draws circles at the detected vertices and finds contours."""
        final_cluster_copy = final_cluster.copy()
        radius = 3
        for vertex in vertices:
            cv2.circle(final_cluster_copy, vertex, radius, 0, -1)  # Draw black circle

        # Find contours after drawing circles
        contours, _ = cv2.findContours(final_cluster_copy, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return final_cluster_copy, contours



class ClusterScorer:
    """Scores clusters based on how well they fit rectangles."""

    @staticmethod
    def find_final_cluster_id(binary_image_list):
        """Finds the cluster ID with the highest score."""
        max_score = -1.0
        max_score_index = None
        contour_finder = ContourFinder()  # Use composition
        cluster_score = 0.0

        for i, binary_image in enumerate(binary_image_list):
            if len(binary_image.shape) != 2:
                raise ValueError("Input image must be binary (grayscale).")

            contours, _ = contour_finder.find_contours(binary_image)
            contour_score = 0.0

            for contour in contours:
                # 1. Find bounding box vertices
                contour_points = contour.reshape(-1, 2)
                min_x = np.min(contour_points[:, 0])
                min_y = np.min(contour_points[:, 1])
                max_x = np.max(contour_points[:, 0])
                max_y = np.max(contour_points[:, 1])

                # 2. Count white pixels inside contour
                contour_mask = np.zeros_like(binary_image, dtype=np.uint8)
                cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)
                n1 = np.sum(contour_mask == 255)

                # 3. Create a rectangle and count white pixels
                rect_mask = np.zeros_like(binary_image, dtype=np.uint8)
                cv2.rectangle(rect_mask, (min_x, min_y), (max_x, max_y), 255, thickness=cv2.FILLED)
                n2 = np.sum(rect_mask == 255)

                # 4. Calculate score
                if n2 > 0:
                    contour_score += (n1 / float(n2))

            if len(contours) > 0 :
                cluster_score = contour_score / len(contours)
            else:
                cluster_score = 0.0

            if cluster_score > max_score:
                max_score = cluster_score
                max_score_index = i

        return max_score_index


class ClusterAnalyzer:
    """Analyzes clusters to identify and process contours."""

    def __init__(self, image_shape, base_filename, output_dir, color_space_name, grid_size, K):
        self.image_shape = image_shape
        self.base_filename = base_filename
        self.output_dir = output_dir
        self.color_space_name = color_space_name
        self.grid_size = grid_size
        self.K = K
        self.contour_finder = ContourFinder()  # Composition
        self.contour_filter = ContourFilter()  # Composition
        self.mask_modifier = MaskModifier()    # Composition
        self.connected_rect_detector = ConnectedRectangleDetector() # Composition

    def _create_cluster_mask_and_calculate_grid_density(self, pixel_belongs_to_which_cluster, cluster_id):
        """Creates a mask for a cluster and calculates grid density."""
        rows, cols = pixel_belongs_to_which_cluster.shape
        mask = np.zeros((rows, cols), dtype=np.uint8)
        mask[pixel_belongs_to_which_cluster == cluster_id] = 255
        white_pixels = np.sum(mask == 255)

        grid_rows = rows // self.grid_size
        grid_cols = cols // self.grid_size
        grid_count = 0

        for i in range(grid_rows):
            for j in range(grid_cols):
                x1, x2 = i * self.grid_size, (i + 1) * self.grid_size
                y1, y2 = j * self.grid_size, (j + 1) * self.grid_size
                grid_patch = mask[x1:x2, y1:y2]
                if np.any(grid_patch == 255):
                    grid_count += 1

        num_of_pixels_per_grid = 0
        if grid_count > 0: 
            num_of_pixels_per_grid = float(white_pixels) / float(grid_count)
        return mask, white_pixels, grid_count, num_of_pixels_per_grid

    def analyze_clusters(self, pixel_belongs_to_which_cluster):
        """Analyzes each cluster, filters contours, and saves results."""
        rows, cols, _ = self.image_shape
        candidate_clusters = []
        fully_processed_img = None

        for cluster_id in range(self.K):
            mask, _, _, num_of_pixels_per_grid = self._create_cluster_mask_and_calculate_grid_density(pixel_belongs_to_which_cluster, cluster_id)

            if num_of_pixels_per_grid > float(self.grid_size * self.grid_size) / 4.0:
                contours, _ = self.contour_finder.find_contours(mask)
                filtered_contours, contours_to_remove_indices, _ = self.contour_filter.filter_contours(contours, rows, cols)
                modified_mask = self.mask_modifier.generate_new_mask_by_removing_undesired_contours(mask, contours, contours_to_remove_indices)

                if np.any(modified_mask == 255):  # Check if any white pixels remain
                    candidate_clusters.append(modified_mask)

        desired_cluster_id = ClusterScorer.find_final_cluster_id(candidate_clusters)
        
        if desired_cluster_id is not None:
            updated_contours, _ = self.contour_finder.find_contours(candidate_clusters[desired_cluster_id])
            
            final_image = candidate_clusters[desired_cluster_id]
            separated_contours = None
            condition = False

            for _, updated_contour in enumerate(updated_contours):
                approx = cv2.approxPolyDP(updated_contour, 0.014 * cv2.arcLength(updated_contour, True), True)
                if len(approx) > 4:
                    vertices = self.connected_rect_detector.detect_connected_rectangles_vertices(updated_contour)
                    if vertices:
                        final_image, separated_contours = self.connected_rect_detector.draw_circle_with_vertices(candidate_clusters[desired_cluster_id], vertices)
                        condition = True

            if condition:
                fully_processed_img = self._visualize_and_save_modified_mask(final_image, separated_contours)
            else:
                fully_processed_img = self._visualize_and_save_modified_mask(candidate_clusters[desired_cluster_id], updated_contours)

        else:
            print("No suitable cluster found for processing.")

        return fully_processed_img

    def _visualize_and_save_modified_mask(self, modified_mask, contours):
        """Visualizes and saves the modified mask with contours, centroids, and vectors."""
        mask_bgr = cv2.cvtColor(modified_mask, cv2.COLOR_GRAY2BGR)

        for contour in contours:
            shape_extractor = TwoDShapeExtractor()
            flag, cont = shape_extractor.is_circle_ellipse(contour)
            if flag:
                print("circle/ellipse detected, will not be visualised")
                cv2.drawContours(mask_bgr, [contour], -1, (0,0,0), thickness=cv2.FILLED)
            else:
                color = tuple(np.random.randint(0, 255, 3).tolist())
                cv2.drawContours(mask_bgr, [contour], -1, (255,255,255), thickness=cv2.FILLED)

                # Find approxPolyDP and centroid
                epsilon = 0.04 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                M = cv2.moments(approx)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    centroid = (cX, cY)

                    # --- Draw Centroid ---
                    cv2.circle(mask_bgr, centroid, 3, (0, 0, 0), -1)  # Black circle

                    # --- Vector Calculation and Drawing ---
                    p1 = tuple(approx[0 % len(approx)][0])
                    p2 = tuple(approx[1 % len(approx)][0])  
                    p3 = tuple(approx[2 % len(approx)][0])  


                    v1 = np.array(p1) - np.array(p2)
                    v2 = np.array(p3) - np.array(p2)
                    norm_v1 = np.linalg.norm(v1)
                    unit_v1 = v1 / norm_v1 if norm_v1 !=0 else np.array([0,0])
                        
                    norm_v2 = np.linalg.norm(v2)
                    unit_v2 = v2/ norm_v2 if norm_v2 != 0 else np.array([0,0])

                    scale = 40
                    scaled_v1 = (centroid[0] + int(unit_v1[0] * scale), centroid[1] + int(unit_v1[1] * scale))
                    scaled_v2 = (centroid[0] + int(unit_v2[0] * scale), centroid[1] + int(unit_v2[1] * scale))

                    # Draw vectors
                    cv2.arrowedLine(mask_bgr, centroid, scaled_v1, (0, 180, 0), 2)  # White vectors
                    cv2.arrowedLine(mask_bgr, centroid, scaled_v2, (0, 0, 255), 2)

        mask_path = os.path.join(self.output_dir, f"{self.base_filename}_{self.color_space_name}_modified.png")
        cv2.imwrite(mask_path, mask_bgr)
        return mask_bgr


# --- Concrete Clusterer Implementations ---

class BGRClusterer(BaseImageClusterer):
    def convert_color_space(self, image):
        return image
    def run(self):
        """Runs the clustering and analysis pipeline."""
        image = self.load_and_preprocess()
        if image is None:
            return  # Exit if image loading failed

        pixel_belongs_to_which_cluster, segmented_image, centers = self.kmeans_clustering(image)

        # Now, use the ClusterAnalyzer:
        base_filename = os.path.splitext(os.path.basename(self.image_loader.image_path))[0]
        analyzer = ClusterAnalyzer(image.shape, base_filename, self.output_dir, self.__class__.__name__, self.grid_size, self.K)
        fully_processed_img = analyzer.analyze_clusters(pixel_belongs_to_which_cluster)
        print(f"Saved results in '{self.output_dir}' for {self.__class__.__name__}.")
        return fully_processed_img

class HSVClusterer(BaseImageClusterer):
    def convert_color_space(self, image):
        return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    def run(self):
        """Runs the clustering and analysis pipeline."""
        image = self.load_and_preprocess()
        if image is None:
            return  # Exit if image loading failed
        pixel_belongs_to_which_cluster, segmented_image, centers = self.kmeans_clustering(image)
        # Now, use the ClusterAnalyzer:
        base_filename = os.path.splitext(os.path.basename(self.image_loader.image_path))[0]
        analyzer = ClusterAnalyzer(image.shape, base_filename, self.output_dir, self.__class__.__name__, self.grid_size, self.K)
        fully_processed_img = analyzer.analyze_clusters(pixel_belongs_to_which_cluster)
        print(f"Saved results in '{self.output_dir}' for {self.__class__.__name__}.")
        return fully_processed_img

class LabClusterer(BaseImageClusterer):
    def convert_color_space(self, image):
        return cv2.cvtColor(image, cv2.COLOR_BGR2Lab)
    def run(self):
        """Runs the clustering and analysis pipeline."""
        image = self.load_and_preprocess()
        if image is None:
            return  # Exit if image loading failed

        pixel_belongs_to_which_cluster, segmented_image, centers = self.kmeans_clustering(image)

        # Now, use the ClusterAnalyzer:
        base_filename = os.path.splitext(os.path.basename(self.image_loader.image_path))[0]
        analyzer = ClusterAnalyzer(image.shape, base_filename, self.output_dir, self.__class__.__name__, self.grid_size, self.K)
        fully_processed_img = analyzer.analyze_clusters(pixel_belongs_to_which_cluster)
        print(f"Saved results in '{self.output_dir}' for {self.__class__.__name__}.")
        return fully_processed_img


class ImageProcessing(Node):
    """ROS 2 node for image processing using clustering-based segmentation."""

    def __init__(self):
        super().__init__('image_processing_node')
        self.bridge = CvBridge()
        self.processed_img_pub_ = None  # Initialize publisher to None
        self.incoming_img_sub_ = None  # Initialize subscriber to None
        self._stop_processing = False 
        self.image_received = False
        # Quality of Service settings.  BEST_EFFORT is crucial for real-time performance.
        self.qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1  # Keep only the latest message.  Important for real-time!
        )
        # Declare and get the 'activate_detection' parameter.
        self.declare_parameter('activate_detection', False, ParameterDescriptor(
            description='Enable or disable image processing.'))
        self.activate_detection = self.get_parameter(
            'activate_detection').get_parameter_value().bool_value
        # Set up a parameter callback to handle changes to 'activate_detection'
        self.add_on_set_parameters_callback(self.parameter_callback)
        # --- Threading Setup ---
        self.image_queue = Queue(maxsize=1)  # Queue to hold incoming images.  1 size!
        self.result_queue = Queue(maxsize=1)  # Queue to hold processed images
        self.processing_thread = None
        self.lock = threading.Lock()  # Lock for thread safety
        self.output_dir = "/home/akarshan/mobile_cobot_ws/src/detection/img_processing_output"
        os.makedirs(self.output_dir, exist_ok=True)  # ensure output directory exists
        # Create publisher here, so is active from the moment the node starts up.
        self.processed_img_pub_ = self.create_publisher(Image, '/detections_visualized', self.qos)
        # if initial state of 'activate_detection is True
        if self.activate_detection is True:
            self.get_logger().info("Activating detection: creating subscription.")
            self.incoming_img_sub_ = self.create_subscription(Image, '/arm_rgbd_camera/image_raw', self.image_callback, self.qos)
            self.start_processing_thread()

    def image_callback(self, msg):
        """Callback function for image subscription."""
        self.get_logger().info("Image Received")
        if not self.image_received:  # Avoid blocking if the queue is full
            self.image_received = True
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                try:
                    self.image_queue.put_nowait((cv_image, msg.header))
                except queue.Full:  
                    self.get_logger().info("Queue is Full")
                # *Immediately* unsubscribe after receiving *one* image.
                with self.lock:
                    if self.incoming_img_sub_:
                        self.get_logger().info("Destroying Image subscription after receiving one image.")
                        self.destroy_subscription(self.incoming_img_sub_)
                        self.incoming_img_sub_ = None
            except CvBridgeError as e:
                self.get_logger().error(f"CvBridgeError: {e}")

    def processing_loop(self):
        """Thread function to process images from the queue."""
        while rclpy.ok() and not self._stop_processing:  # Keep running as long as ROS 2 is running
            try:
                cv_image, header = self.image_queue.get(timeout=1.0)  # Get image from queue (with timeout)
                print("cosumer gets the cv_image, header")
            except Empty:
                self.get_logger().info("Image Queue is Empty")
                continue  # If queue is empty, go to the next iteration
            except Exception as e:
                self.get_logger().error("Error Retrieving Image from the Queue", e)
                continue
            try:
                # *Critically*, pass a copy of the image, to avoid race condition
                clusterer = HSVClusterer("", cv_image.copy(), K=15, use_blur=False,
                                         output_dir=self.output_dir)  
                processed_image = clusterer.run()
                
                if self.processed_img_pub_:
                    try:
                        ros_image = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
                        ros_image.header = header  # Copy the header from the input image
                        self.processed_img_pub_.publish(ros_image)
                    except CvBridgeError as e:
                        self.get_logger().error(f"CvBridgeError during publish: {e}")
            except Exception as e:
                self.get_logger().error(f"Error during image processing: {e}")

            self.image_queue.task_done()  # Indicate that the task is done

    def start_processing_thread(self):
        """Starts the processing thread if it's not already running."""
        if self.processing_thread is None or not self.processing_thread.is_alive():
            self.get_logger().info("Starting processing thread.")
            self._stop_processing = False  # Reset the flag
            self.processing_thread = threading.Thread(target=self.processing_loop, daemon=True)
            self.processing_thread.start()

    def parameter_callback(self, params):
        """Callback for parameter changes, particularly for 'activate_detection'."""
        for param in params:
            if param.name == 'activate_detection':
                with self.lock:  # Use lock for thread safety
                    if param.value is True and self.incoming_img_sub_ is None:
                        self.get_logger().info("Activating detection: creating subscription.")
                        self.incoming_img_sub_ = self.create_subscription(Image, '/arm_rgbd_camera/image_raw', self.image_callback, self.qos)
                        self.image_received = False
                        self.start_processing_thread()
                    elif param.value is False:
                        self.get_logger().info("Deactivating detection: destroying subscription.")
                        if self.incoming_img_sub_:
                            self.destroy_subscription(self.incoming_img_sub_)
                            self.incoming_img_sub_ = None
                        self._stop_processing = True
                        self.processing_thread = None
                        self.image_received = False
        # Accept all parameter changes
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessing()
    rclpy.spin(image_processor)  # Runs the node until it's shut down
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()