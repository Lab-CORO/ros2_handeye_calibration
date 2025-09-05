import cv2
import numpy as np
import math

import transforms3d as tfs
from rospy import logerr, logwarn, loginfo

from easy_handeye.handeye_calibration import HandeyeCalibration

from scipy.spatial.transform import Rotation


import transforms3d.quaternions as tq

# Assuming prev_q and curr_q are [x, y, z, w]
def compute_rotation_difference(prev_q, curr_q):
    # Reorder quaternions for transforms3d: (w, x, y, z)
    prev_q = np.array([prev_q[3], prev_q[0], prev_q[1], prev_q[2]])
    curr_q = np.array([curr_q[3], curr_q[0], curr_q[1], curr_q[2]])

    # Compute relative quaternion: q_rel = q_prev⁻¹ * q_curr
    q_inv = tq.qinverse(prev_q)
    q_rel = tq.qmult(q_inv, curr_q)

    # Rotation angle (in radians) is: 2 * acos(q_rel[0]) = 2 * acos(w)
    angle_rad = 2 * math.acos(np.clip(q_rel[0], -1.0, 1.0))  # q_rel[0] = w
    angle_deg = math.degrees(angle_rad)
    return angle_deg


class HandeyeCalibrationBackendOpenCV(object):
    MIN_SAMPLES = 2  # TODO: correct? this is what is stated in the paper, but sounds strange
    """Minimum samples required for a successful calibration."""

    AVAILABLE_ALGORITHMS = {
        'Tsai-Lenz': cv2.CALIB_HAND_EYE_TSAI,
        'Park': cv2.CALIB_HAND_EYE_PARK,
        'Horaud': cv2.CALIB_HAND_EYE_HORAUD,
        'Andreff': cv2.CALIB_HAND_EYE_ANDREFF,
        'Daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
    }

    def __init__(self):
        self.last_calibration = None

    @staticmethod
    def _msg_to_opencv(transform_msg):
        cmt = transform_msg.translation
        tr = np.array((cmt.x, cmt.y, cmt.z))
        cmq = transform_msg.rotation
        rot = tfs.quaternions.quat2mat((cmq.w, cmq.x, cmq.y, cmq.z))
        return rot, tr

    @staticmethod
    def _get_opencv_samples(samples):
        """
        Returns the sample list as a rotation matrix and a translation vector.

        :rtype: (np.array, np.array)
        """
        hand_base_rot = []
        hand_base_tr = []
        marker_camera_rot = []
        marker_camera_tr = []

        for s in samples:
            camera_marker_msg = s['optical'].transform
            (mcr, mct) = HandeyeCalibrationBackendOpenCV._msg_to_opencv(camera_marker_msg)
            marker_camera_rot.append(mcr)
            marker_camera_tr.append(mct)

            base_hand_msg = s['robot'].transform
            (hbr, hbt) = HandeyeCalibrationBackendOpenCV._msg_to_opencv(base_hand_msg)
            hand_base_rot.append(hbr)
            hand_base_tr.append(hbt)

        return (hand_base_rot, hand_base_tr), (marker_camera_rot, marker_camera_tr)

    def compute_calibration(self, handeye_parameters, samples, algorithm=None):
        """
        Computes the calibration through the OpenCV library and returns it.
        Also logs the delta between the current and previous calibration.
        """
        if algorithm is None:
            algorithm = 'Tsai-Lenz'

        loginfo(f'OpenCV backend calibrating with algorithm {algorithm}')

        if len(samples) < self.MIN_SAMPLES:
            logwarn(f"{self.MIN_SAMPLES - len(samples)} more samples needed! Not computing the calibration")
            return

        # Update data
        opencv_samples = self._get_opencv_samples(samples)
        (hand_world_rot, hand_world_tr), (marker_camera_rot, marker_camera_tr) = opencv_samples

        if len(hand_world_rot) != len(marker_camera_rot):
            logerr("Different numbers of hand-world and camera-marker samples!")
            raise AssertionError

        loginfo(f"Computing from {len(samples)} poses...")

        method = self.AVAILABLE_ALGORITHMS[algorithm]

        # Perform calibration
        hand_camera_rot, hand_camera_tr = cv2.calibrateHandEye(
            hand_world_rot, hand_world_tr, marker_camera_rot, marker_camera_tr, method=method
        )

        # Format result
        (hcqw, hcqx, hcqy, hcqz) = tfs.quaternions.mat2quat(hand_camera_rot)
        (hctx, hcty, hctz) = hand_camera_tr
        result_tuple = ((hctx, hcty, hctz), (hcqx, hcqy, hcqz, hcqw))

        loginfo("Translation (xyz): {}".format((float(hctx), float(hcty), float(hctz))))
        loginfo("Orientation (xyzw): {}".format((hcqx, hcqy, hcqz, hcqw)))

        r = Rotation.from_quat((hcqx, hcqy, hcqz, hcqw))
        # Convert to RPY (roll, pitch, yaw) in radians
        rpy_rad = r.as_euler('xyz', degrees=False)
        loginfo("Orientation (rpy): {}".format((rpy_rad[0], rpy_rad[1], rpy_rad[2])))

        # If a previous calibration exists, compute delta
        if self.last_calibration is not None:
            prev_t, prev_q = self.last_calibration
            curr_t = np.array([hctx, hcty, hctz])
            prev_t = np.array(prev_t)

            delta_t = np.linalg.norm(curr_t - prev_t)
            loginfo(f"Δ Translation: {delta_t:.6f} m")

            curr_q = [hcqx, hcqy, hcqz, hcqw]
            prev_q = list(prev_q)  # tuple to list

            angle_deg = compute_rotation_difference(prev_q, curr_q)
            loginfo(f"Δ Rotation: {angle_deg:.6f} degrees")

        # Save this as the last calibration
        self.last_calibration = ((hctx, hcty, hctz), (hcqx, hcqy, hcqz, hcqw))

        # Return result as object
        ret = HandeyeCalibration(calibration_parameters=handeye_parameters,
                                 transformation=result_tuple)
        return ret
