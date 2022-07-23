import numpy as np
import cv2
import glob
import statistics
import os
from threading import Thread

# All user config params here
pattern_width = 9
pattern_height = 6
square_size = 0.058
# square_size = 0.028 # square size for the 5x8 board
folderpath = "/home/msardonini/calibration/"
visualize_images = False
num_images = 245
# num_images = 15
cam0_prefix = 'cam0_'
cam1_prefix = 'cam1_'
img_extension = '.png'
output_file = 'stereo_calibration.yaml'
image_shape = (1280, 720)


class ThreadWithReturnValue(Thread):

    def __init__(self,
                 group=None,
                 target=None,
                 name=None,
                 args=(),
                 kwargs={},
                 Verbose=None):
        Thread.__init__(self, group, target, name, args, kwargs)
        self._return = None

    def run(self):
        print(type(self._target))
        if self._target is not None:
            self._return = self._target(*self._args, **self._kwargs)

    def join(self, *args):
        Thread.join(self, *args)
        return self._return


class CamParams:

    def __init__(self, K, D, rvec, tvec):
        self.K = K
        self.D = D
        self.rvecs = rvec
        self.tvecs = tvec


# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((pattern_height * pattern_width, 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_width, 0:pattern_height].T.reshape(
    -1, 2) * square_size

# Arrays to store object points and image points from all the images.
imgpoints0 = []  # 2d points in image plane.
objpoints0 = []  # 3d point in real world space
points_index0 = []

imgpoints1 = []  # 2d points in image plane.
objpoints1 = []  # 3d point in real world space
points_index1 = []

objpoints_stereo = []  # 3d point in real world space
imgpoints0_stereo = []  # 2d points in image plane.
imgpoints1_stereo = []  # 2d points in image plane.


def get_single_cam_points(cam_num):
    if (cam_num == 0):
        prefix = cam0_prefix
    elif (cam_num == 1):
        prefix = cam1_prefix
    else:
        return

    for i in range(num_images):
        img_path = folderpath + prefix + str(i) + img_extension
        img = cv2.imread(img_path)
        if img is None:
            print("Could not read ", img_path)
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(
            gray, (pattern_width, pattern_height), None)

        if ret == True:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                       criteria)
            img = cv2.drawChessboardCorners(img,
                                            (pattern_width, pattern_height),
                                            corners, ret)

            if visualize_images:
                cv2.imshow('img', img)
                key = cv2.waitKey(0)
                if key == ord('y'):
                    if (cam_num == 0):
                        imgpoints0.append(corners)
                        objpoints0.append(objp)
                        points_index0.append(i)
                    else:
                        imgpoints1.append(corners)
                        objpoints1.append(objp)
                        points_index1.append(i)
                elif key == ord('n'):
                    # pass
                    os.remove(img_path)
                else:
                    print("Error! need to provide 'y' or 'n'")
            else:
                if (cam_num == 0):
                    imgpoints0.append(corners)
                    objpoints0.append(objp)
                    points_index0.append(i)
                else:
                    imgpoints1.append(corners)
                    objpoints1.append(objp)
                    points_index1.append(i)
        else:
            # pass
            os.remove(img_path)


def get_stereo_points():
    for i in range(num_images):
        if (i in points_index0 and i in points_index1):
            index0 = points_index0.index(i)
            imgpoints0_stereo.append(imgpoints0[index0])
            index1 = points_index1.index(i)
            imgpoints1_stereo.append(imgpoints1[index1])
            objpoints_stereo.append(objp)

    # for i in range(num_images):
    #     # if (i % 4 != 0):
    #     #     continue
    #     img0_path = folderpath + cam0_prefix + str(i) + img_extension
    #     img1_path = folderpath + cam1_prefix + str(i) + img_extension
    #     img0 = cv2.imread(img0_path)
    #     img1 = cv2.imread(img1_path)

    #     # Check to make sure that our images were read
    #     if img0 is None or img1 is None:
    #         if img0 is None:
    #             print("Could not read ", img0_path)
    #         else:
    #             print("Could not read ", img1_path)
    #         continue

    #     # Convert to Grayscale
    #     gray0 = cv2.cvtColor(img0,cv2.COLOR_BGR2GRAY)
    #     gray1 = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)

    #     # Find the chess board corners
    #     ret0, corners0 = cv2.findChessboardCorners(gray0, (pattern_width, pattern_height), None)
    #     ret1, corners1 = cv2.findChessboardCorners(gray1, (pattern_width, pattern_height), None)

    #     # If found, add object points, image points (after refining them)
    #     if ret0 == True and ret1 == True:
    #         corners0 = cv2.cornerSubPix(gray0 ,corners0, (11, 11),(-1, -1), criteria)
    #         corners1 = cv2.cornerSubPix(gray1 ,corners1, (11, 11),(-1, -1), criteria)

    #         # Draw and display the corners
    #         img0 = cv2.drawChessboardCorners(img0, (pattern_width, pattern_height), corners0, ret0)
    #         img1 = cv2.drawChessboardCorners(img1, (pattern_width, pattern_height), corners1, ret1)

    #         if visualize_images:
    #             show_im = cv2.hconcat([img0, img1])
    #             cv2.imshow('img',show_im)
    #             key = cv2.waitKey(0)
    #             if key == ord('y'):
    #                 objpoints.append(objp)
    #                 imgpoints0.append(corners0)
    #                 imgpoints1.append(corners1)
    #             elif key == ord('n'):
    #                 os.remove(img0_path)
    #                 os.remove(img1_path)
    #             else:
    #                 print("Error! need to provide 'y' or 'n'")
    #         else:
    #             objpoints.append(objp)
    #             imgpoints0.append(corners0)
    #             imgpoints1.append(corners1)
    # cv2.destroyAllWindows()
    #


def calibrate_intrinsics(objpoints, imgpoints, shape, cam=None):
    flags = 0
    #flags |= cv2.CALIB_FIX_ASPECT_RATIO
    #flags |= cv2.CALIB_SAME_FOCAL_LENGTH
    #flags |= cv2.CALIB_ZERO_TANGENT_DIST
    flags |= cv2.CALIB_RATIONAL_MODEL
    if cam == None:
        return cv2.calibrateCamera(objpoints,
                                   imgpoints,
                                   shape,
                                   None,
                                   None,
                                   flags=flags)
    else:
        flags |= cv2.CALIB_USE_INTRINSIC_GUESS
        return cv2.calibrateCamera(objpoints,
                                   imgpoints,
                                   shape,
                                   cam.K,
                                   cam.D,
                                   flags=flags)


def calibrate_intrinsics_parallel(objpoints0,
                                  imgpoints0,
                                  objpoints1,
                                  imgpoints1,
                                  cam0=None,
                                  cam1=None):
    print("Calibrating Intrinsics")
    th0 = ThreadWithReturnValue(target=calibrate_intrinsics,
                                args=(objpoints0, imgpoints0, image_shape,
                                      cam0))
    th1 = ThreadWithReturnValue(target=calibrate_intrinsics,
                                args=(objpoints1, imgpoints1, image_shape,
                                      cam1))
    th0.start()
    th1.start()
    # th1 = threading.Thread(target=calibrate_intrinsics, args=(objpoints, imgpoints0, image_shape, )).start()

    ret0, K0, D0, rvec0, tvecs0 = th0.join()
    ret1, K1, D1, rvec1, tvecs1 = th1.join()

    cam0 = CamParams(K0, D0, rvec0, tvecs0)
    cam1 = CamParams(K1, D1, rvec1, tvecs1)
    return cam0, cam1


def calc_reproj_error(objpoints, imgpoints, cam_params):
    mean_error = 0
    errors = []
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], cam_params.rvecs[i],
                                          cam_params.tvecs[i], cam_params.K,
                                          cam_params.D)
        error = cv2.norm(imgpoints[i], imgpoints2,
                         cv2.NORM_L2) / len(imgpoints2)
        errors.append(error)
    return errors


def save_intrinsics(cam, filename):
    fs_write = cv2.FileStorage(filename, cv2.FILE_STORAGE_WRITE)
    fs_write.write("K", cam.K)
    fs_write.write("D", cam.D)
    fs_write.release()


def save_stereo(cam0, cam1, R, T, E, F):
    fs_write = cv2.FileStorage(output_file, cv2.FILE_STORAGE_WRITE)
    fs_write.write("K0", cam0.K)
    fs_write.write("D0", cam0.D)
    fs_write.write("K1", cam1.K)
    fs_write.write("D1", cam1.D)
    fs_write.write("R", R)
    fs_write.write("T", T)
    fs_write.write("R0", cam0.R)
    fs_write.write("R1", cam1.R)
    fs_write.write("P0", cam0.P)
    fs_write.write("P1", cam1.P)
    fs_write.write("E", E)
    fs_write.write("F", F)
    fs_write.release()


if __name__ == "__main__":
    get_single_cam_points(0)
    get_single_cam_points(1)

    cam0, cam1 = calibrate_intrinsics_parallel(objpoints0, imgpoints0,
                                               objpoints1, imgpoints1)

    reprojection_errors_0 = calc_reproj_error(objpoints0, imgpoints0, cam0)
    reprojection_errors_1 = calc_reproj_error(objpoints1, imgpoints1, cam1)

    for err in reprojection_errors_0:
        print("Cam0 Frame error: ", err)
    print("Average Reprojection Error Cam0: ",
          statistics.mean(reprojection_errors_0))
    for err in reprojection_errors_1:
        print("Cam1 Frame error: ", err)
    print("Average Reprojection Error Cam1: ",
          statistics.mean(reprojection_errors_1))

    while True:
        ret = input(
            "\n\n Would you like to recalibrate using images with lower reprojection errors? (y/n): "
        )
        if (ret == 'y'):
            thresh = input("Enter reprojection error threshold: ")
            thresh = float(thresh)
            for i in range(len(reprojection_errors_0) - 1, -1, -1):
                if (reprojection_errors_0[i] > thresh):
                    imgpoints0.pop(i)
                    objpoints0.pop(i)
                    points_index0.pop(i)

            for i in range(len(reprojection_errors_1) - 1, -1, -1):
                if (reprojection_errors_1[i] > thresh):
                    imgpoints1.pop(i)
                    objpoints1.pop(i)
                    points_index1.pop(i)

            cam0, cam1 = calibrate_intrinsics_parallel(objpoints0,
                                                       imgpoints0,
                                                       objpoints1,
                                                       imgpoints1,
                                                       cam0=cam0,
                                                       cam1=cam1)

            reprojection_errors_0 = calc_reproj_error(objpoints0, imgpoints0,
                                                      cam0)
            reprojection_errors_1 = calc_reproj_error(objpoints1, imgpoints1,
                                                      cam1)

            for err in reprojection_errors_0:
                print("Cam0 Frame error: ", err)
            print("Average Reprojection Error Cam0: ",
                  statistics.mean(reprojection_errors_0))
            for err in reprojection_errors_1:
                print("Cam1 Frame error: ", err)
            print("Average Reprojection Error Cam1: ",
                  statistics.mean(reprojection_errors_1))

        elif (ret == 'n'):
            save_intrinsics(cam0, cam0_prefix + "intrinsics.yaml")
            save_intrinsics(cam1, cam1_prefix + "intrinsics.yaml")
            break
        else:
            print("Bad value, need y or n")

    get_stereo_points()
    print("Calibrating Stereo")
    print("Sizes: ", len(objpoints_stereo), len(imgpoints0_stereo),
          len(imgpoints1_stereo))
    ret_stereo, cam0.K, cam0.D, cam1.K, cam1.D, R, T, E, F = cv2.stereoCalibrate(
        objpoints_stereo,
        imgpoints0_stereo,
        imgpoints1_stereo,
        cam0.K,
        cam0.D,
        cam1.K,
        cam1.D,
        image_shape,
        flags=cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_FIX_INTRINSIC)

    cam0.R, cam1.R, cam0.P, cam1.P, Q, cam0.roi, cam1.roi = cv2.stereoRectify(
        cam0.K, cam0.D, cam1.K, cam1.D, image_shape, R, T)

    cam0.D.resize(8)
    cam1.D.resize(8)

    save_stereo(cam0, cam1, R, T, E, F)

    print("RMS reprojection error stereo:", ret_stereo)
