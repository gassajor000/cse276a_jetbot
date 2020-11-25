"""
    created by Jordan Gassaway, 11/25/2020
    Camera: Class for managing camera functions
"""
import cv2
import numpy
import glob
from jetbot import Camera as JBCamera

class Camera:
    # Default values for camera distortion
    DEFAULT_K = numpy.array(
        [[121.72388176606911, 0.0, 152.47287927762537], [0.0, 161.6329164473156, 142.225856754917], [0.0, 0.0, 1.0]])
    DEFAULT_D = numpy.array(
        [[0.05000918770713665], [-0.16688048742835998], [0.24973285586862118], [-0.14173811443102421]])

    def __init__(self, camera_instance):
        self.K = self.DEFAULT_K
        self.D = self.DEFAULT_D
        if camera_instance:
            self.camera_instance = camera_instance
        else:
            print("Initializing Camera...")
            self.camera_instance = JBCamera.instance(width=300, height=300)

    def calibrate(self, file_path):
        """Run camera calibration on captured images"""
        print('Beginning Camera Calibration')
        CHECKERBOARD = (6, 9)
        subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
        calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW
        objp = numpy.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), numpy.float32)
        objp[0, :, :2] = numpy.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
        _img_shape = None
        objpoints = []  # 3d point in real world space
        imgpoints = []  # 2d points in image plane.

        images = glob.glob('images/*.jpg')
        for fname in images:
            img = cv2.imread(fname)
            if _img_shape == None:
                _img_shape = img.shape[:2]
            else:
                assert _img_shape == img.shape[:2], "All images must share the same size."
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,
                                                     cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)
                cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), subpix_criteria)
                imgpoints.append(corners)

                img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners, ret)
                cv2.imwrite(fname + '-processed.png', img)

        N_OK = len(objpoints)
        K = numpy.zeros((3, 3))
        D = numpy.zeros((4, 1))
        rvecs = [numpy.zeros((1, 1, 3), dtype=numpy.float64) for i in range(N_OK)]
        tvecs = [numpy.zeros((1, 1, 3), dtype=numpy.float64) for i in range(N_OK)]
        rms, _, _, _, _ = \
            cv2.fisheye.calibrate(
                objpoints,
                imgpoints,
                gray.shape[::-1],
                K,
                D,
                rvecs,
                tvecs,
                calibration_flags,
                (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
            )
        print("Found " + str(N_OK) + " valid images for calibration")
        print("DIM=" + str(_img_shape[::-1]))
        print("K=np.array(" + str(K.tolist()) + ")")
        print("D=np.array(" + str(D.tolist()) + ")")
        self.K = K
        self.D = D

        self.undistort_image(cv2.imread(images[0]))

    def close(self):
        """Clean up resources and extra threads"""
        self.camera_instance.stop()

    def correct_images(self):
        images = glob.glob('images/*.jpg')
        for fname in images:
            img = cv2.imread(fname)
            img_corrected = self.undistort_image(img)
            cv2.imwrite(fname + '-corrected.png', img_corrected)

    def undistort_image(self, image):
        """undistort the image using the calibrated camera matrix"""
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(self.K, self.D, numpy.eye(3), self.K, image.shape[:2],
                                                         cv2.CV_16SC2)
        undistorted_img = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR,
                                    borderMode=cv2.BORDER_CONSTANT)

        #         cv2.imwrite('images/calibresult.png', undistorted_img)
        return undistorted_img

    def get_image(self):
        """Returns an undistorted image from the camera"""
        return self.undistort_image(self.camera_instance.value)

    def get_instance(self):
        return self.camera_instance