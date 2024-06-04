import cv2, time
import open3d as o3d
import depthai as dai
from settings import *
from utils.host_camera import HostSync
from utils.projector_3d import PointCloudVisualizer


with dai.Device(pipeline) as device:

    device.setIrLaserDotProjectorBrightness(1200)
    qs = []
    qs.append(device.getOutputQueue("depth", maxSize=1, blocking=False))
    qs.append(device.getOutputQueue("colorize", maxSize=1, blocking=False))
    qs.append(device.getOutputQueue("rectified_left", maxSize=1, blocking=False))
    qs.append(device.getOutputQueue("rectified_right", maxSize=1, blocking=False))

    calibData = device.readCalibration()
    if COLOR:
        w, h = camRgb.getIspSize()
        intrinsics = calibData.getCameraIntrinsics(dai.CameraBoardSocket.RGB, dai.Size2f(w, h))
    else:
        w, h = monoRight.getResolutionSize()
        intrinsics = calibData.getCameraIntrinsics(dai.CameraBoardSocket.RIGHT, dai.Size2f(w, h))
    pcl_converter = PointCloudVisualizer(intrinsics, w, h)

    serial_no = device.getMxId()
    sync = HostSync()
    depth_vis, color, rect_left, rect_right = None, None, None, None

    while True:
        for q in qs:
            new_msg = q.tryGet()
            if new_msg is not None:
                msgs = sync.add_msg(q.getName(), new_msg)
                if msgs:
                    depth = msgs["depth"].getFrame()
                    color = msgs["colorize"].getCvFrame()
                    rectified_left = msgs["rectified_left"].getCvFrame()
                    rectified_right = msgs["rectified_right"].getCvFrame()
                    depth_vis = cv2.normalize(depth, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
                    depth_vis = cv2.equalizeHist(depth_vis)
                    depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_HOT)
                    cv2.imshow("depth", depth_vis)
                    cv2.imshow("color", color)
                    cv2.imshow("rectified_left", rectified_left)
                    cv2.imshow("rectified_right", rectified_right)
                    rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
                    pcl_converter.rgbd_to_projection(depth, rgb)
                    pcl_converter.visualize_pcd()

        key = cv2.waitKey(1)
        if key == ord("s"):
            timestamp = str(int(time.time()))
            cv2.imwrite(OUT_DIR+f"{serial_no}_{timestamp}_depth.png", depth)
            cv2.imwrite(OUT_DIR+f"{serial_no}_{timestamp}_rgb.png", rgb)

            cv2.imwrite(OUT_DIR+f"{serial_no}_{timestamp}_depth_vis.png", depth_vis)
            cv2.imwrite(OUT_DIR+f"{serial_no}_{timestamp}_color.png", color)
            cv2.imwrite(OUT_DIR+f"{serial_no}_{timestamp}_rectified_left.png", rectified_left)
            cv2.imwrite(OUT_DIR+f"{serial_no}_{timestamp}_rectified_right.png", rectified_right)
            o3d.io.write_point_cloud(OUT_DIR+f"{serial_no}_{timestamp}.pcd", pcl_converter.pcl, compressed=True)
        elif key == ord("q"):
            break