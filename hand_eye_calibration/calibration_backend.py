import cv2
import numpy as np

from scipy.spatial.transform import Rotation as Rot

class CalibrationBackend:
    MIN_SAMPLES = 4

    AVAILABLE_ALGORITHMS = {
        'Tsai-Lenz': cv2.CALIB_HAND_EYE_TSAI,
        'Park': cv2.CALIB_HAND_EYE_PARK,
        'Horaud': cv2.CALIB_HAND_EYE_HORAUD,
        'Andreff': cv2.CALIB_HAND_EYE_ANDREFF,
        'Daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
    }

    @staticmethod
    def list_to_opencv(transform: list()):
        """
        transform = [tx, ty, tz, qx, qy, qz, qw]
        """
        t = transform
        tr = np.array([t[0], t[1], t[2]])
        rot = Rot.from_quat([t[3], t[4], t[5], t[6]]).as_matrix()
        return rot, tr

    @staticmethod
    def get_opencv_samples(samples_robot, samples_tracking):
        """
        Returns the sample list as a rotation matrix and a translation vector.
        :rtype: (np.array, np.array)
        """
        hand_base_rot = []
        hand_base_tr = []
        marker_camera_rot = []
        marker_camera_tr = []

        for robot_tf, tracking_tf in zip(samples_robot, samples_tracking):
            (mcr, mct) = CalibrationBackend.list_to_opencv(tracking_tf)
            marker_camera_rot.append(mcr)
            marker_camera_tr.append(mct)

            (hbr, hbt) = CalibrationBackend.list_to_opencv(robot_tf)
            hand_base_rot.append(hbr)
            hand_base_tr.append(hbt)

        return (hand_base_rot, hand_base_tr), (marker_camera_rot, marker_camera_tr)

    @staticmethod
    def compute_calibration(# handeye_parameters, 
                            samples_robot, 
                            samples_tracking,
                            algorithm=None):
        """
        Computes the calibration through the OpenCV library and returns it.
        :rtype: easy_handeye.handeye_calibration.HandeyeCalibration
        """
        if algorithm is None: algorithm = 'Tsai-Lenz'
        # Update data
        opencv_samples = CalibrationBackend.get_opencv_samples(samples_robot=samples_robot, 
                                                               samples_tracking=samples_tracking)
        (hand_world_rot, hand_world_tr), (marker_camera_rot, marker_camera_tr) = opencv_samples
        method = CalibrationBackend.AVAILABLE_ALGORITHMS[algorithm]

        hand_camera_rot, hand_camera_tr = cv2.calibrateHandEye(hand_world_rot, hand_world_tr, marker_camera_rot,
                                                               marker_camera_tr, method=method)

        (hcqx, hcqy, hcqz, hcqw) = [float(i) for i in Rot.from_matrix(hand_camera_rot).as_quat()]
        (hctx, hcty, hctz) = [float(i) for i in hand_camera_tr]
        return [hctx, hcty, hctz, hcqx, hcqy, hcqz, hcqw]