import depthai as dai
import cv2
import cv2.aruco as aruco
import numpy as np

print("OpenCV:", cv2.__version__)
print("Has ArUco:", hasattr(cv2, 'aruco'))

# Create DepthAI pipeline
pipeline = dai.Pipeline()

# Mono cams
mono_left = pipeline.createMonoCamera()
mono_right = pipeline.createMonoCamera()
mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

# Stereo depth
stereo = pipeline.createStereoDepth()
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
stereo.setExtendedDisparity(True)
stereo.setSubpixel(True)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)

mono_left.out.link(stereo.left)
mono_right.out.link(stereo.right)

# Output rectified image
xout_left = pipeline.createXLinkOut()
xout_left.setStreamName("rectified_left")
stereo.rectifiedLeft.link(xout_left.input)

# Output depth
xout_depth = pipeline.createXLinkOut()
xout_depth.setStreamName("depth")
stereo.depth.link(xout_depth.input)

# ArUco config (classic original dict)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
aruco_params = aruco.DetectorParameters()

# Optional tweaks
aruco_params.adaptiveThreshWinSizeMin = 3
aruco_params.adaptiveThreshWinSizeMax = 23
aruco_params.adaptiveThreshWinSizeStep = 10
aruco_params.minMarkerPerimeterRate = 0.03
aruco_params.maxErroneousBitsInBorderRate = 0.35

# Start device
with dai.Device(pipeline) as device:
    queue_left = device.getOutputQueue("rectified_left", 4, False)
    queue_depth = device.getOutputQueue("depth", 4, False)

    while True:
        frame_left = queue_left.get().getCvFrame()
        depth_frame = queue_depth.get().getFrame()

        # Detect markers
        corners, ids, _ = aruco.detectMarkers(frame_left, aruco_dict, parameters=aruco_params)

        if ids is not None:
            print(f"[INFO] Detected {len(ids)} ArUco marker(s): {ids.flatten().tolist()}")
            for corner, marker_id in zip(corners, ids.flatten()):
                pts = corner[0].astype(int)
                x, y = pts[0]  # Top-left corner

                # Draw outline
                cv2.polylines(frame_left, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

                # Get depth
                distance_text = "???"
                if 0 <= x < depth_frame.shape[1] and 0 <= y < depth_frame.shape[0]:
                    depth_mm = depth_frame[y, x]
                    if depth_mm > 0 and depth_mm < 8000:
                        distance_text = f"{depth_mm} mm"
                    else:
                        distance_text = "Out of range"
                    print(f" - ID {marker_id} @ ({x},{y}) → {distance_text}")
                else:
                    print(f" - ID {marker_id} top-left out of bounds!")

                # Overlay text
                cv2.putText(frame_left, f"ID:{marker_id}", (x, y - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame_left, distance_text, (x, y - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        else:
            print("[INFO] No ArUco markers detected.")

        cv2.imshow("Rectified Left + ArUco + Depth", frame_left)
        if cv2.waitKey(1) == ord('q'):
            break

cv2.destroyAllWindows()
