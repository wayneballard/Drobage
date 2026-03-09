import depthai as dai
import numpy as np
import cv2

pipeline = dai.Pipeline()
cam_rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
videoQueue = cam_rgb.requestOutput((640, 480)) #.createOutputQueue()

monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
stereo = pipeline.create(dai.node.StereoDepth)

jsonfile = "/home/wb/Desktop/Drobage/src/my_yolo_package/share/config/184430101153051300_09_28_25_13_00.json"

calibData = dai.CalibrationHandler(jsonfile)
pipeline.setCalibrationData(calibData)

# Linking
monoLeftOut = monoLeft.requestOutput((640,480)) #monoLeft.requestFullResolutionOutput()
monoRightOut = monoRight.requestOutput((640, 480)) #monoRight.requestFullResolutionOutput()
monoLeftOut.link(stereo.left)
monoRightOut.link(stereo.right)

colorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
colorMap[0] = [0, 0, 0]  # to make zero-disparity pixels black


sync = pipeline.create(dai.node.Sync)

monoLeftOut.link(sync.inputs["monoLeft"])
monoRightOut.link(sync.inputs["monoRight"])
videoQueue.link(sync.inputs["cam_rgb"])

disparityQueue = stereo.disparity.createOutputQueue()


outQueue = sync.out.createOutputQueue()
pipeline.start()

maxDisparity = 1
while pipeline.isRunning():
    messageGroup : dai.MessageGroup = outQueue.get()
    disparity = disparityQueue.get()

    npDisparity = disparity.getFrame() 
    maxDisparity = max(maxDisparity, np.max(npDisparity)) #stereo.initialConfig.getMaxDisparity()
    colorizedDisparity = cv2.applyColorMap(((npDisparity / maxDisparity) * 255).astype(np.uint8), colorMap)
    cv2.imshow("disparity", colorizedDisparity)
    key = cv2.waitKey(1)
    if key == ord('q'):
        pipeline.stop()
        break

    left = messageGroup["monoLeft"]
    right = messageGroup["monoRight"]
    rgb = messageGroup["cam_rgb"]
    print(f"Timestamps, message group {messageGroup.getTimestamp()}, left {left.getTimestamp()}, right {right.getTimestamp()}, rgb {rgb.getTimestamp()}, disparity {disparity.getTimestamp()}")

