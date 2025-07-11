import depthai as dai
import cv2
import numpy as np

# Create pipeline
pipeline = dai.Pipeline()

# Mono cameras
mono_left = pipeline.create(dai.node.MonoCamera)
mono_right = pipeline.create(dai.node.MonoCamera)

mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

# StereoDepth node
stereo = pipeline.create(dai.node.StereoDepth)
stereo.setRectifyEdgeFillColor(0)
stereo.setLeftRightCheck(False)
stereo.setSubpixel(False)
stereo.setExtendedDisparity(False)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)

mono_left.out.link(stereo.left)
mono_right.out.link(stereo.right)

# Output streams
xout_left = pipeline.create(dai.node.XLinkOut)
xout_left.setStreamName("rect_left")
stereo.rectifiedLeft.link(xout_left.input)

xout_depth = pipeline.create(dai.node.XLinkOut)
xout_depth.setStreamName("depth")
stereo.depth.link(xout_depth.input)

# ArUco/AprilTag setup
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
aruco_params = cv2.aruco.DetectorParameters()

# Start device
with dai.Device(pipeline) as device:
    q_left = device.getOutputQueue("rect_left", maxSize=4, blocking=False)
    q_depth = device.getOutputQueue("depth", maxSize=4, blocking=False)

    while True:
        frame_left = q_left.get().getCvFrame()
        frame_depth = q_depth.get().getFrame()

        # Detect AprilTags
        corners, ids, _ = cv2.aruco.detectMarkers(frame_left, aruco_dict, parameters=aruco_params)

        if ids is not None:
            for tag_id, tag_corners in zip(ids.flatten(), corners):
                try:
                    pts = tag_corners[0]
                    top_left = pts[0]
                    x, y = int(round(top_left[0])), int(round(top_left[1]))
                    print(f"[APRILTAG] ID: {tag_id}")
                    print(f"  Top-left corner: x={x}, y={y}")

                    if 0 <= y < frame_depth.shape[0] and 0 <= x < frame_depth.shape[1]:
                        depth_mm = frame_depth[y, x]
                        if depth_mm > 0:
                            print(f"  Depth at top-left: {depth_mm} mm")
                        else:
                            print("  Depth at top-left: invalid (0)")
                    else:
                        print("  Top-left corner out of bounds")

                    # Draw marker border + ID
                    cv2.polylines(frame_left, [pts.astype(int)], True, (255, 0, 0), 2)
                    cv2.putText(frame_left, f"ID:{tag_id}", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                except Exception as e:
                    print(f"[ERR] Could not process tag ID {tag_id}: {e}")

        cv2.imshow("Rectified Left + AprilTags", frame_left)
        if cv2.waitKey(1) == ord('q'):
            break

cv2.destroyAllWindows()
