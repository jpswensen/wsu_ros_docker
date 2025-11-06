from __future__ import annotations

import os
import time
import math
import threading
import argparse
import numpy as np
# cv2 can pull in system GDAL/libspatialite and fail to import on some systems.
# Make it optional at import time to avoid breaking the whole module.
try:
    import cv2  # optional; only required if vision utilities below are used
except Exception:
    cv2 = None
from scipy.spatial.transform import Rotation as R
import scipy.optimize as opt

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
# Avoid importing cv_bridge at module import time, since it pulls cv2.
# We'll implement a numpy-based conversion in _image_cb instead.
from tf_transformations import quaternion_from_euler
from transform3d import Transform


# ------------------------------
# ROS2 Node: subscribe image, publish pose
# ------------------------------
class FloatingCamClient(Node):
    def __init__(self, image_topic='/floating_camera/floating_camera/image_raw', pose_topic='/floating_camera/set_pose'):
        super().__init__('hw4_solutions_client')
        self._last_image = None
        self._last_pose = None
        self._last_stamp = None
        self._lock = threading.Lock()
        self.pose_pub = self.create_publisher(Pose, pose_topic, 1)
        self.image_sub = self.create_subscription(Image, image_topic, self._image_cb, 1)
        self.pose_sub = self.create_subscription(PoseStamped, '/floating_camera/pose', self._pose_cb, 1)  # dummy to ensure pub-sub connection
        self.cmd_vel_pub = self.create_publisher(Twist, '/floating_camera/cmd_vel', 1)  # dummy to ensure pub-sub connection

        # Background executor
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spinning = False

    def start(self):
        if not self._spinning:
            self._spinning = True
            self._thread.start()
            self.get_logger().info('Executor thread started')

    def stop(self):
        if self._spinning:
            self._executor.shutdown()
            self._spinning = False
            try:
                self._thread.join(timeout=1.0)
            except Exception:
                pass
            self.get_logger().info('Executor thread stopped')

    def _pose_cb(self, msg: PoseStamped):
        self._last_pose = msg.pose

    def get_pose(self) -> Pose | None:

        if self._last_pose is None:
            return None
        T = np.array([self._last_pose.position.x, self._last_pose.position.y, self._last_pose.position.z])
        q = np.array([self._last_pose.orientation.x, self._last_pose.orientation.y, self._last_pose.orientation.z, self._last_pose.orientation.w])  # wxyz
        return Transform(p=T, quat=q)


    def set_camera_velocity(self, linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0)):
        """Set camera velocity; robustly cast inputs to native Python floats.
        Accepts tuples/lists/numpy arrays/py objects containing 3 elements each.
        """
        vel_msg = Twist()
        try:
            lx = float(linear[0]); ly = float(linear[1]); lz = float(linear[2])
            ax = float(angular[0]); ay = float(angular[1]); az = float(angular[2])
        except Exception as e:
            raise TypeError(f"Invalid velocity inputs: {e}")
        vel_msg.linear.x = lx
        vel_msg.linear.y = ly
        vel_msg.linear.z = lz
        vel_msg.angular.x = ax
        vel_msg.angular.y = ay
        vel_msg.angular.z = az
        self.cmd_vel_pub.publish(vel_msg)

    def _image_cb(self, msg: Image):
        """Convert ROS2 Image message to numpy BGR array without cv_bridge/cv2.
        Assumes encoding 'bgr8' (as published by the floating camera)."""
        try:
            import numpy as np
            h = int(getattr(msg, 'height'))
            w = int(getattr(msg, 'width'))
            step = int(getattr(msg, 'step'))  # bytes per row
            # msg.data is a bytes-like; frombuffer avoids copy
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            # Handle potential row padding via step
            if buf.size < h * step:
                raise ValueError(f'Image buffer too small: got {buf.size}, expected at least {h*step}')
            arr2d = buf[:h*step].reshape((h, step))
            # Truncate padding to width*channels (3 for bgr8)
            arr2d = arr2d[:, :w*3]
            img = arr2d.reshape((h, w, 3))
            with self._lock:
                self._last_image = img.copy()
                self._last_stamp = getattr(msg.header, 'stamp', None)
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
    
    def try_get_image(self):
        """Return the most recent image as numpy array or None without blocking."""
        with self._lock:
            if self._last_image is None:
                return None
            return self._last_image.copy()

    def publish_pose_rpy(self, position=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        qx, qy, qz, qw = quaternion_from_euler(*rpy)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        self.pose_pub.publish(pose)

    def publish_pose_quat(self, position=(0.0,0.0,0.0), quat_xyzw=(0.0,0.0,0.0,1.0)):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        qx, qy, qz, qw = quat_xyzw
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        self.pose_pub.publish(pose)

    def publish_wait_pose_rpy(self, position=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        qx, qy, qz, qw = quaternion_from_euler(*rpy)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        self.pose_pub.publish(pose)
    
        while self._last_pose is None or position_error(self._last_pose, pose) > 1e-3:
            time.sleep(0.01)

    def publish_wait_pose_quat(self, position=(0.0,0.0,0.0), quat_xyzw=(0.0,0.0,0.0,1.0)):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        qx, qy, qz, qw = quat_xyzw
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        self.pose_pub.publish(pose)

        while self._last_pose is None or position_error(self._last_pose, pose) > 1e-3:
            time.sleep(0.01)

    def wait_for_image(self, timeout=5.0):
        start = time.time()
        last = None
        while time.time() - start < timeout:
            with self._lock:
                img = None if self._last_image is None else self._last_image.copy()
                stamp = self._last_stamp
            if img is not None:
                if stamp is None or stamp != last:
                    return img
            time.sleep(0.05)
        raise TimeoutError('No fresh image within timeout')


def position_error(pose1, pose2):
    p1 = np.array([pose1.position.x, pose1.position.y, pose1.position.z])
    p2 = np.array([pose2.position.x, pose2.position.y, pose2.position.z])
    error_vec = p2 - p1
    distance = np.linalg.norm(error_vec)
    return distance


# ------------------------------
# Image utilities: sphere detection
# ------------------------------

def find_sphere_centers(img_bgr: np.ndarray, merge_tol_px: float = 10.0) -> np.ndarray:
    if cv2 is None:
        raise ImportError('cv2 not available; find_sphere_centers requires OpenCV.')
    """Detect sphere-like blobs robustly and return centers as Nx2 array (x,y).
    Combines dual Otsu thresholding, morphology, connected components filtering, and blob detector.
    """
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    gray_blur = cv2.GaussianBlur(gray, (5,5), 0)
    # Dual Otsu
    _, tb = cv2.threshold(gray_blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    _, td = cv2.threshold(gray_blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    mask = cv2.bitwise_or(tb, td)
    # Morphology
    k = np.ones((3,3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)

    H, W = gray.shape
    # Connected components filter
    num, labels, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    comps = []
    min_area_px = 12
    min_area_frac = 2e-5
    min_area = max(min_area_px, int(min_area_frac * W * H))
    max_area = int(0.25 * W * H)
    aspect_min, aspect_max = 0.3, 2.5
    for i in range(1, num):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < min_area or area > max_area:
            continue
        ww = stats[i, cv2.CC_STAT_WIDTH]
        hh = stats[i, cv2.CC_STAT_HEIGHT]
        if hh == 0:
            continue
        asp = ww/float(hh)
        if asp < aspect_min or asp > aspect_max:
            continue
        cx, cy = cents[i]
        comps.append((float(cx), float(cy)))

    # Blob detector
    sbd = cv2.SimpleBlobDetector_Params()
    sbd.filterByArea = True
    sbd.minArea = 12.0
    sbd.maxArea = 1e6
    sbd.filterByCircularity = True
    sbd.minCircularity = 0.6
    sbd.filterByInertia = True
    sbd.minInertiaRatio = 0.2
    sbd.filterByConvexity = False
    sbd.minThreshold = 10
    sbd.maxThreshold = 220
    sbd.thresholdStep = 10
    try:
        detector = cv2.SimpleBlobDetector_create(sbd)
    except AttributeError:
        detector = cv2.SimpleBlobDetector(sbd)
    kps = detector.detect(gray_blur)
    blobs = [(kp.pt[0], kp.pt[1]) for kp in kps]

    # Merge near duplicates
    merged = list(comps)
    def near(p,q,tol=merge_tol_px):
        return (p[0]-q[0])**2 + (p[1]-q[1])**2 <= tol*tol
    for b in blobs:
        if not any(near(b, c) for c in merged):
            merged.append(b)

    if len(merged) == 0:
        return np.zeros((0,2), dtype=float)
    # deterministic ordering (by y then x)
    merged = sorted(merged, key=lambda t: (t[1], t[0]))
    return np.array(merged, dtype=float)


# ------------------------------
# Linear algebra utilities for HW4
# ------------------------------

# Convert to homogeneous pixels and normalized rays
def to_homog_xy(centers: np.ndarray) -> np.ndarray:
    if centers.shape[0] == 0:
        return np.zeros((3,0))
    return np.vstack([centers[:,0], centers[:,1], np.ones((centers.shape[0],))])

def K_intrinsic(f: float, sx: float, sy: float, st: float, ox: float, oy: float) -> np.ndarray:
    Ks = np.array([[sx, st, ox],[0.0, sy, oy],[0.0, 0.0, 1.0]])
    Kf = np.array([[f, 0.0, 0.0],[0.0, f, 0.0],[0.0, 0.0, 1.0]])
    return Ks @ Kf

def skew3(x: np.ndarray) -> np.ndarray:
    x = np.asarray(x)
    if x.shape == (3,3):
        # unskew
        return np.array([x[2,1], x[0,2], x[1,0]])
    x = x.reshape(3,)
    return np.array([[0.0, -x[2], x[1]],[x[2], 0.0, -x[0]],[-x[1], x[0], 0.0]])

def essential_from_RT(Rmat: np.ndarray, Tvec: np.ndarray) -> np.ndarray:
    return skew3(Tvec) @ Rmat

def correspondence(E: np.ndarray, x1: np.ndarray, x2: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Greedy matching by epipolar distance. x1,x2 are (3,N) in normalized coords.
    Returns (x1Matched, x2Matched) both (3,N2) where N2 = N of x2.
    """
    if x1.size == 0 or x2.size == 0:
        return x1.copy(), x2.copy()
    x1_pool = x1.copy()
    N2 = x2.shape[1]
    x1m = np.zeros((3, N2))
    x2m = x2.copy()
    for i in range(N2):
        x2p = x2[:, i]
        ell = (x2p.T @ E)  # (3,)
        denom = max(1e-9, np.linalg.norm(ell[:2]))
        d = np.abs(ell @ x1_pool) / denom
        idx = int(np.argmin(d))
        x1m[:, i] = x1_pool[:, idx]
        x1_pool = np.delete(x1_pool, idx, axis=1)
        if x1_pool.shape[1] == 0 and i < N2-1:
            # no more x1 to match; truncate x2 accordingly
            x2m = x2m[:, :i+1]
            x1m = x1m[:, :i+1]
            break
    return x1m, x2m


def findMatchedPointsByColor(imgG, img):
    if cv2 is None:
        raise ImportError('cv2 not available; findMatchedPointsByColor requires OpenCV.')

    #Convert both images to BW
    grayG = cv2.cvtColor(imgG,cv2.COLOR_BGR2GRAY).astype(np.uint8) 
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY).astype(np.uint8) 
    _, grayG = cv2.threshold(grayG, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    _, gray = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    # Use the connectedComponents function to get the centroids of all the points
    retG, labelsG, statsG, centroidsG = cv2.connectedComponentsWithStats(grayG,connectivity=8)
    ret, labels, stats, centroids = cv2.connectedComponentsWithStats(gray,connectivity=8)

    pntsG = centroidsG[1:]
    pnts = centroids[1:]

    # Use the hue at the centroids to find out which ones match
    matchIdx = np.zeros((pnts.shape[0],1))
    colorsG = imgG[pntsG[:,1].astype(int),pntsG[:,0].astype(int)].T
    colors = img[pnts[:,1].astype(int),pnts[:,0].astype(int)].T

    xGMatched = pntsG.copy()
    xTmp = pnts.copy()
    xMatched = 0*pnts.copy()
    for I in range(colors.shape[1]):
        # Compute the distance between the current color and all the colors in the goal image
        colorDist = np.linalg.norm(np.subtract(colorsG,colors[:,I].reshape((3,1))),axis=0)
        idx = np.argmin(colorDist)
        xMatched[idx,:] = pnts[I,:]


    # for I in range(len(pnts)):
    #     centroid = pnts[I]
    #     color = img[int(centroid[1]),int(centroid[0]),:]
    #     print(f'Centroid: ({centroid[0],centroid[1]})  Color: {color}')
    return xGMatched, xMatched     


# Algebraic E from matched points
def essential_algebraic(x1Matched: np.ndarray, x2Matched: np.ndarray) -> np.ndarray:
    M = x1Matched.shape[1]
    A = np.zeros((M, 9))
    for i in range(M):
        a = np.kron(x1Matched[:, i].T, x2Matched[:, i].T)
        A[i, :] = a
    Ua, Da, Va = np.linalg.svd(A)
    Ea = np.reshape(Va[-1, :], (3,3), order='F')
    Ue, De, Ve = np.linalg.svd(Ea)
    De = np.diag((1.0, 1.0, 0.0))
    E = Ue @ De @ Ve
    return E

# Optimize E to minimize Sampson-like error sum (simple algebraic residual squared)

def E_cost(Eflat: np.ndarray, x1Matched: np.ndarray, x2Matched: np.ndarray) -> float:
    Emat = np.reshape(Eflat, (3,3), order='F')
    cost = 0.0
    for i in range(x1Matched.shape[1]):
        r = float(x2Matched[:, i].T @ Emat @ x1Matched[:, i])
        cost += r*r
    return cost

def essential_minimization(x1Matched: np.ndarray, x2Matched: np.ndarray) -> np.ndarray:
    E0 = essential_from_RT(np.eye(3), np.array([1.0, 0.0, 0.0]))
    print(E0)
    E0f = E0[:]
    print(E0f.flatten())
    result = opt.minimize(E_cost, E0f.flatten(), args=(x1Matched, x2Matched), method='Powell', options={'xtol':1e-4,'ftol':1e-6,'maxiter':1e4})
    Em = np.reshape(result.x, (3,3), order='F')
    Ue, De, Ve = np.linalg.svd(Em)
    De = np.diag((1.0, 1.0, 0.0))
    E = Ue @ De @ Ve
    return E

# Triangulation by reprojection-error minimization in normalized coordinates

def triangulation_cost(Xflat: np.ndarray, x1m: np.ndarray, x2m: np.ndarray, R12: np.ndarray, T12: np.ndarray) -> float:
    N = x1m.shape[1]
    X1 = np.reshape(Xflat, (3, N), order='F')  # points in cam1 frame (homog scale arbitrary)
    # Project in cam1: normalized
    xp1 = X1 / X1[-1, :]
    # Transform to cam2: X2 = R21 @ (X1 - T12)
    R21 = R12.T
    T21 = -R12.T @ T12
    X2 = R21 @ (X1 - T12.reshape(3,1))
    xp2 = X2 / X2[-1, :]
    # Sum of squared residuals
    cost = 0.0
    for i in range(N):
        cost += np.sum((x1m[:, i] - xp1[:, i])**2) + np.sum((x2m[:, i] - xp2[:, i])**2)
    return float(cost)

def triangulate_points(x1m: np.ndarray, x2m: np.ndarray, R12: np.ndarray, T12: np.ndarray, X0: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
    N = x1m.shape[1]
    if X0 is None:
        # initialize along the first camera rays with unit depth
        X0 = []
        for i in range(N):
            d = x1m[:, i]
            if abs(d[-1]) < 1e-9:
                d[-1] = 1.0
            X0.extend((d / d[-1]).tolist())
        X0 = np.array(X0)
    bounds = []
    for i in range(N):
        bounds.extend([(-10,10), (-10,10), (0,50)])
    res = opt.minimize(triangulation_cost, X0, args=(x1m, x2m, R12, T12), method='L-BFGS-B', options={'maxiter':10}, bounds=bounds)
    X1 = np.reshape(res.x, (3, N), order='F')
    R21 = R12.T
    T21 = -R12.T @ T12
    X2 = R21 @ (X1 - T12.reshape(3,1))
    return X1, X2


def triangulate_points_algebraic(x1m: np.ndarray, x2m: np.ndarray, R12: np.ndarray, T12: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Linear DLT triangulation per point in camera-1 frame.
    x1m, x2m are normalized image coordinates (3xN) with last row ~1.
    R12, T12 map camera-1 frame to camera-2: X2 = R12 X1 + T12.
    Returns X1 (3xN) in camera-1 and X2 (3xN) in camera-2.
    """
    N = x1m.shape[1]
    X1 = np.zeros((3, N))
    # Projection matrices in normalized coordinates
    P1 = np.hstack([np.eye(3), np.zeros((3,1))])          # [I | 0]
    P2 = np.hstack([R12, T12.reshape(3,1)])               # [R | t]
    for i in range(N):
        u1, v1, w1 = x1m[:, i]
        u2, v2, w2 = x2m[:, i]
        # Build 4x4 system
        A = np.zeros((4,4))
        A[0,:] = u1*P1[2,:] - P1[0,:]
        A[1,:] = v1*P1[2,:] - P1[1,:]
        A[2,:] = u2*P2[2,:] - P2[0,:]
        A[3,:] = v2*P2[2,:] - P2[1,:]
        _, _, Vt = np.linalg.svd(A)
        Xh = Vt[-1,:]
        X1[:, i] = Xh[:3] / Xh[3]
    R21 = R12.T
    T21 = -R12.T @ T12
    X2 = R21 @ (X1 - T12.reshape(3,1))
    return X1, X2

# Recover motion (up to scale) from E by SVD disambiguation (cheirality test)

def motion_from_E(E: np.ndarray, x1m: np.ndarray, x2m: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Recover motion (R12, T12 direction) from an Essential matrix using the four canonical
    decompositions from the SVD E = U Σ V^T:

      R1 = U W  V^T,  T = ±u3
      R2 = U W^T V^T,  T = ±u3

    where W = [[0,-1,0],[1,0,0],[0,0,1]] and u3 is the third column of U.
    The valid solution is selected by cheirality: all triangulated points must have positive
    depth in both cameras. If none satisfy perfectly, the best-scoring candidate is returned.
    """
    U, S, Vt = np.linalg.svd(E)
    # Enforce proper rotations
    if np.linalg.det(U) < 0:
        U[:, -1] *= -1
    if np.linalg.det(Vt) < 0:
        Vt[-1, :] *= -1
    W = np.array([[0, -1, 0],[1, 0, 0],[0, 0, 1]])
    # Two rotation candidates
    R1 = U @ W @ Vt
    R2 = U @ W.T @ Vt
    if np.linalg.det(R1) < 0:
        R1 = -R1
    if np.linalg.det(R2) < 0:
        R2 = -R2
    # Translation direction (up to scale)
    u3 = U[:, 2]
    candidates = [
        (R1,  u3),
        (R1, -u3),
        (R2,  u3),
        (R2, -u3),
    ]

    best = None
    best_score = -1
    for Rc, tc in candidates:
        try:
            X1, X2 = triangulate_points(x1m, x2m, Rc, tc)
            ok1 = (X1[-1, :] > 0)
            ok2 = (X2[-1, :] > 0)
            score = int(np.logical_and(ok1, ok2).sum())
            if score == X1.shape[1]:
                return Rc, tc
            if score > best_score:
                best_score = score
                best = (Rc, tc)
        except Exception:
            continue
    return best if best is not None else (R1, u3)
