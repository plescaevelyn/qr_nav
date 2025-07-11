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

# Stereo node
stereo = pipeline.create(dai.node.StereoDepth)
stereo.setRectifyEdgeFillColor(0)
stereo.setLeftRightCheck(False)
stereo.setSubpixel(False)
stereo.setExtendedDisparity(False)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)  # Align depth to rectified left

mono_left.out.link(stereo.left)
mono_right.out.link(stereo.right)

# Output: rectified left image
xout_left = pipeline.create(dai.node.XLinkOut)
xout_left.setStreamName("rect_left")
stereo.rectifiedLeft.link(xout_left.input)

# Output: depth map
xout_depth = pipeline.create(dai.node.XLinkOut)
xout_depth.setStreamName("depth")
stereo.depth.link(xout_depth.input)

# QR detector
qr = cv2.QRCodeDetector()

# Start device
with dai.Device(pipeline) as device:
    q_left = device.getOutputQueue("rect_left", maxSize=4, blocking=False)
    q_depth = device.getOutputQueue("depth", maxSize=4, blocking=False)

    while True:
        frame_left = q_left.get().getCvFrame()
        frame_depth = q_depth.get().getFrame()  # depth in millimeters (uint16)

        data, points, _ = qr.detectAndDecode(frame_left)

        if points is not None and data:
            try:
                points = points[0]
                top_left = points[0]
                x, y = int(round(top_left[0])), int(round(top_left[1]))
                print(f"[QR] Data: {data}")
                print(f"  Top-left corner: x={x}, y={y}")

                # Depth map shape check
                if 0 <= y < frame_depth.shape[0] and 0 <= x < frame_depth.shape[1]:
                    depth_mm = frame_depth[y, x]
                    if depth_mm > 0:
                        print(f"  Depth at top-left: {depth_mm} mm")
                    else:
                        print("  Depth at top-left: invalid (0)")
                else:
                    print("  Top-left corner out of depth frame bounds")

                # Draw QR box
                cv2.polylines(frame_left, [points.astype(int)], True, (0, 255, 0), 2)

            except Exception as e:
                print(f"[ERR] Could not process QR: {e}")

        cv2.imshow("Rectified Left + QR", frame_left)
        if cv2.waitKey(1) == ord('q'):
            break

cv2.destroyAllWindows()
