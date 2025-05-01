import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt
import math
from math import pi
import cv2

np.set_printoptions(
    linewidth=120,
    formatter={"float": lambda x: f"{0:8.4g}" if abs(x) < 1e-10 else f"{x:8.4g}"},
)
np.random.seed(0)
from machinevisiontoolbox.base import *
from machinevisiontoolbox import *
from spatialmath.base import *
from spatialmath import *


class ImageProcessor:

    # init contains camera params
    def __init__(self):
        self.K = np.array([[548.5, 0, 310.9], [0, 548.5, 236], [0, 0, 1]])
        self.distortion = [-0.5326, 0.371, 0.000496, 0.004691, -0.1812]
        self.u0 = self.K[0, 2]
        self.v0 = self.K[1, 2]
        self.fpixel_width = self.K[0, 0]
        self.fpixel_height = self.K[1, 1]
        self.k1, self.k2, self.p1, self.p2, self.k3 = self.distortion

    def capture_image(self):
        camera = cv2.VideoCapture(0)

        ret, frame = camera.read()
        if ret:
            cv2.imwrite("/media/img.png")
            print(f"Image captured and saved as")
            return frame
        else:
            print("Failed to capture image")

    def undistort_image(self, frame):
        U, V = frame.meshgrid()

        x = (U - self.u0) / self.fpixel_width
        y = (V - self.v0) / self.fpixel_height
        r = np.sqrt(x**2 + y**2)
        delta_x = (
            x * (self.k1 * r**2 + self.k2 * r**4 + self.k3 * r**6)
            + 2 * self.p1 * x * y
            + self.p2 * (r**2 + 2 * x**2)
        )
        delta_y = (
            y * (self.k1 * r**2 + self.k2 * r**4 + self.k3 * r**6)
            + self.p1 * (r**2 + 2 * y**2)
            + self.p2 * x * y
        )

        xd = x + delta_x
        yd = y + delta_y

        Ud = xd * self.fpixel_width + self.u0
        Vd = yd * self.fpixel_height + self.v0
        return frame.warp(Ud, Vd)

    # all the aruco stuff()

    def get_aruco_marker(self):
        frame = self.capture_image()
        markers = self.undistort_image(frame).fiducial(dict="36h11", K=self.K, side=25e-3)
        return markers
    