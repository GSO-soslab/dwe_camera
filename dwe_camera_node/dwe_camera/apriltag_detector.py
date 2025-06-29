import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from pupil_apriltags import Detector

class AprilTagDetector:
    """
    A class to detect AprilTags in an image using the pupil-apriltags library,
    perform pose estimation, and draw the results on the image.
    """
    def __init__(self, family, tag_size, camera_intrinsics, camera_distortion, image_size, logger, detector_params):
        """
        Initializes the AprilTag detector.

        :param family: The family of AprilTags to detect (e.g., 'tag36h11').
        :param tag_size: The size of the tags in meters.
        :param camera_intrinsics: A dictionary with camera intrinsic parameters [fx, fy, cx, cy].
        :param camera_distortion: A list or tuple of camera distortion coefficients.
        :param image_size: A dictionary with image dimensions {'img_width', 'img_height'}.
        :param logger: A ROS 2 logger object for logging messages.
        :param detector_params: A dictionary of tunable parameters for the pupil-apriltags detector.
        """
        self.logger = logger
        self.tag_size = float(tag_size)

        # pupil-apriltags takes camera params as a simple list/tuple: [fx, fy, cx, cy]
        self.camera_params = (
            camera_intrinsics['fx'],
            camera_intrinsics['fy'],
            camera_intrinsics['cx'],
            camera_intrinsics['cy']
        )
        
        # NOTE: pupil-apriltags does not use distortion coefficients for its internal pose estimation.
        # The user should provide an undistorted image if pose accuracy is critical.
        # We will log a warning if distortion coefficients are present.
        if any(d != 0 for d in camera_distortion):
            self.logger.warn("pupil-apriltags library does not use distortion coefficients for pose estimation. "
                             "For accurate results, provide an undistorted image stream. "
                             "The provided distortion coefficients will be ignored by the detector, but used for drawing axes.")

        self.distCoeffs = np.array(camera_distortion, dtype=np.float32)
        self.camera_intrinsics_mtx = np.array([
            [self.camera_params[0], 0, self.camera_params[2]],
            [0, self.camera_params[1], self.camera_params[3]],
            [0, 0, 1]
        ], dtype=np.float32)

        self.img_width = image_size['img_width']
        self.img_height = image_size['img_height']

        # --- Create pupil-apriltags Detector ---
        try:
            self.detector = Detector(
                families=family,
                **detector_params # Pass YAML parameters directly
            )
            self.logger.info(f"pupil-apriltags detector created for family '{family}' with params: {detector_params}")
        except Exception as e:
            self.logger.error(f"Failed to create pupil-apriltags detector: {e}")
            self.detector = None

    def _rotation_matrix_to_euler_angles(self, R_matrix):
        """
        Converts a rotation matrix to Euler angles (roll, pitch, yaw) in degrees.
        """
        r = R.from_matrix(R_matrix)
        return r.as_euler('xyz', degrees=True)

    def detect_and_draw(self, image):
        """
        Detects AprilTags in the given image, estimates their pose, and draws
        visualizations on the image. This method is now robust against bad
        detections that could cause crashes.
        """
        if self.detector is None:
            self.logger.error("AprilTag detector is not initialized. Cannot process image.")
            return image # Return original image
            
        if image is None:
            self.logger.warn("Received a null image for AprilTag detection.")
            return None
        
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Detect tags. The library handles pose estimation internally if camera_params and tag_size are provided.
        detections = self.detector.detect(
            gray_image, 
            estimate_tag_pose=True, 
            camera_params=self.camera_params, 
            tag_size=self.tag_size
        )
        
        # If tags are successfully detected, process and draw them
        for tag in detections:
            corners = tag.corners.astype(int)
            
            try:
                # --- POSE ESTIMATION AND VISUALIZATION ---
                # This block is wrapped in a try-except to handle cases where pose
                # estimation results in an invalid rotation matrix (e.g., from a
                # noisy or false positive detection), which would crash scipy.
                
                # Extract pose info
                tvec = tag.pose_t.flatten()
                R_matrix = tag.pose_R
                
                # This can fail if R_matrix is not a valid 3x3 matrix
                rvec, _ = cv2.Rodrigues(R_matrix)
                
                # This is the call that was causing the crash
                roll, pitch, yaw = self._rotation_matrix_to_euler_angles(R_matrix)

                # --- DRAW SUCCESSFUL DETECTION ---
                # If all conversions are successful, draw the full pose info
                # Green bounding box for successfully identified tags with valid poses
                cv2.polylines(image, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
                cv2.drawFrameAxes(image, self.camera_intrinsics_mtx, self.distCoeffs, rvec, tvec, self.tag_size * 0.5)

                pose_str = f"({tvec[0]:.2f}, {tvec[1]:.2f}, {tvec[2]:.2f}, {roll:.0f}, {pitch:.0f}, {yaw:.0f})"
                id_str = f"ID: {tag.tag_id}"

                text_anchor = tuple(corners[0])
                cv2.putText(image, id_str, (text_anchor[0], text_anchor[1] - 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                cv2.putText(image, pose_str, (text_anchor[0], text_anchor[1] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 2)

            except (ValueError, cv2.error) as e:
                # --- DRAW FAILED DETECTION ---
                # If pose estimation fails (e.g., non-positive determinant in rotation matrix),
                # log a warning and mark the tag on the image without crashing.
                self.logger.warn(f"Could not process pose for tag {tag.tag_id}. It might be a false positive. Error: {e}")
                
                # Draw a red bounding box to indicate a detection with a bad pose
                cv2.polylines(image, [corners], isClosed=True, color=(0, 0, 255), thickness=2)
                
                # Add text to indicate the failed pose estimation
                text_anchor = tuple(corners[0])
                id_str = f"ID: {tag.tag_id} (Bad Pose)"
                cv2.putText(image, id_str, (text_anchor[0], text_anchor[1] - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                            
        return image