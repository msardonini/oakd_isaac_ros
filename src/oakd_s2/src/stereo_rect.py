import cv2
import numpy as np
import yaml
from pathlib import Path


def intrinsics_to_matrix(intrinsics):
    return np.array([[intrinsics[0], 0, intrinsics[2]],
                     [0, intrinsics[1], intrinsics[3]], [0, 0, 1]])


def main():
    yaml_file = Path(
        '/home/msardonini/workspaces/isaac_ros-dev/src/oakd_s2/src/factory_calibration.yaml'
    )

    # import the data from the yaml file
    with open(yaml_file, 'r') as stream:
        calib_data = yaml.safe_load(stream)

    K1 = intrinsics_to_matrix(np.array(calib_data['cam0']['intrinsics']))
    K2 = intrinsics_to_matrix(np.array(calib_data['cam1']['intrinsics']))

    D1 = np.array(calib_data['cam0']['distortion_coeffs'])
    D2 = np.array(calib_data['cam1']['distortion_coeffs'])

    affine = np.array(calib_data['cam1']['T_cn_cnm1'])

    R = affine[0:3, 0:3]
    T = affine[0:3, 3]

    R1, R2, P1, P2, Q, validPixl, validPixl2 = cv2.stereoRectify(
        K1, D1, K2, D2, (1280, 720), R, T)

    print(f"K1 = {K1}")
    print(f"K2 = {K2}")
    print(f"P1 = {P1}")
    print(f"P2 = {P2}")


if __name__ == '__main__':
    main()
