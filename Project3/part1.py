import cv2 as cv
import numpy as np
import pyrealsense2 as rs

# TODO: Make this the correct video input from your device's camera
cam = cv.VideoCapture(2)
check, frame = cam.read()
width, height, channels = frame.shape

pipeline = rs.pipeline()
config = rs.config()

tracker_type = 'KCF'
tracker = cv.TrackerKCF_create()
bbox = cv.selectROI(frame, False)
ok = tracker.init(frame, bbox)

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False

for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("Color Sensor Required")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# NOTE: Make sure that you have the opencv-contrib-python installed. If you don't uninstall opencv-cpython and install opencv-contrib-python
# Create the KCF Tracker


# Create CV windows
# cv.namedWindow("Original Video Capture", 1)
cv.namedWindow('RealSense', cv.WINDOW_AUTOSIZE)


while True:

    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        continue

    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)

    depth_colormap_dim = depth_colormap.shape
    color_colormap_dim = color_image.shape

    # Create the black image to draw on
    blackImg = np.zeros(depth_colormap_dim)

    #TODO: Use the hstack and vstack methods to add this to the 'Realsense' cv window
    cv.imshow('Tracking', blackImg)
    
            

########################################################################
    # Tracking
#######################################################################

    # Get info for the Tracker
    frame = frames.get_color_frame()
    timer = cv.getTickCount()

    # Update tracker
    ok, bbox = tracker.update(color_image)

    # Calculate Frames per second (FPS)
    fps = cv.getTickFrequency() / (cv.getTickCount() - timer)

    if ok:
            # Tracking success
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv.rectangle(color_image, p1, p2, (255,0,0), 2, 1)
    else :
        # Tracking failure
        cv.putText(color_image, "Tracking failure detected", (100,80), cv.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

    # Display tracker type on frame
    cv.putText(color_image, tracker_type + " Tracker", (100,20), cv.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)

    # Display FPS on frame
    cv.putText(color_image, "FPS : " + str(int(fps)), (100,50), cv.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)

    # NOTE: This has to be after the tracking is done. Otherwise the retracking will not show up on the screen correctly.
    if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv.INTER_AREA)
            
            # NOTE: The two lines below screw up the color image-- and I have no idea what is going on
            # resized_color_image = np.vstack((resized_color_image, blackImg))
            # depth_colormap = np.vstack((depth_colormap, blackImg))
            
            images = np.hstack(( resized_color_image, depth_colormap))

    else:
        # NOTE: The two lines below screw up the color image-- and I have no idea what is going on
        # color_image = np.vstack((color_image, blackImg))
        # depth_colormap = np.vstack((depth_colormap, blackImg))
        
        images = np.hstack((color_image, depth_colormap))

    # cv.imshow("Original Video Capture", color_image)
    cv.imshow('RealSense', images)
    key = cv.waitKey(1)
    if key == 27:
        break

