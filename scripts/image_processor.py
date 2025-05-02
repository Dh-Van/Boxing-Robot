import os
from tkinter import Image
import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt
import math
from math import pi
import cv2
import subprocess

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
        # --- Add ROS Kill ---
        print("[ImageProcessor] Attempting to stop ROS nodes before camera access...")
        # Construct the path relative to this script file or use absolute path
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir) # Assumes scripts/ is one level down
        stop_script_path = os.path.join(project_root, "stop_ros.sh")
        # Or use absolute path: stop_script_path = "/home/ubuntu/Documents/Boxing-Robot/stop_ros.sh"
        if not os.path.exists(stop_script_path):
            print(f"ERROR: stop_ros.sh not found at {stop_script_path}")
            # Decide how to handle: return None or try capture anyway?
            # return None # Safer option
        else:
            try:
                # Run the shell script. Inherits sudo privileges if main.py was run with sudo.
                # Timeout added to prevent hangs. check=True raises error on failure.
                result = subprocess.run(['sh', stop_script_path], check=True, capture_output=True, text=True, timeout=15)
                print("stop_ros.sh completed.")
                # Optional: print output for debugging
                # print("stdout:\n", result.stdout)
                # print("stderr:\n", result.stderr)
                # Maybe a tiny sleep is needed? Probably not effective against fast respawn.
                # time.sleep(0.5)
            except FileNotFoundError:
                 print(f"ERROR: 'sh' command not found?") # Should not happen
                 return None
            except subprocess.CalledProcessError as e:
                print(f"ERROR: stop_ros.sh failed with code {e.returncode}")
                print("stderr:\n", e.stderr)
                # Continue to attempt capture anyway? Or return None?
            except subprocess.TimeoutExpired:
                print("ERROR: stop_ros.sh timed out.")
                return None
            except Exception as e:
                print(f"ERROR: Failed to run stop_ros.sh: {e}")
                return None
        #--- End ROS Kill ---
        camera = cv2.VideoCapture(0)

        ret, frame = camera.read()
        camera.release()
        if ret:
            cv2.imwrite("/media/img.jpg", frame)
            frame_mvt = Image(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), dtype=np.uint8)
            print("we have a picture chat")
            return frame_mvt
        else:
            print("Failed to capture image")

    def undistort_image(self, frame):
        if frame is None:
            print("no frame chat")
            return None
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
        if frame is not None:
            try:
                markers = self.undistort_image(frame).fiducial(dict="36h11", K=self.K, side=150e-3)
                return markers
            except TypeError as e:
                # --- Catch the specific TypeError from len(None) ---
                if "'NoneType' object has no len()" in str(e):
                    print("[ImageProcessor] No ArUco markers found (TypeError caught).")
                    markers = [] # Ensure markers is an empty list
                else:
                    # Re-raise other unexpected TypeErrors or handle differently
                    print(f"[ImageProcessor] Unexpected TypeError in fiducial(): {e}")
                    markers = [] # Return empty list for safety
                    # raise e # Optionally re-raise if you want it to crash on other TypeErrors
            except Exception as e:
                # Catch any other unexpected errors during fiducial detection
                print(f"[ImageProcessor] Unexpected error during fiducial() call: {e}")
                import traceback
                traceback.print_exc() # Print full traceback for debugging
                markers = [] # Return empty list on other errors 
        print("jk no fiducial found, try again?")
        return None
    