from innopy.api import FileReader, FrameDataAttributes, GrabType

# Set recording file path
recording_path = ''

fr = FileReader(recording_path)

attr = [FrameDataAttributes(GrabType.GRAB_TYPE_OGM_LIDAR_DETECTIONS),
        FrameDataAttributes(GrabType.GRAB_TYPE_OGM_OBJECT_LIST),
        FrameDataAttributes(GrabType.GRAB_TYPE_OGM_GEOMETRIES_LIST)]

print("number of frames: ", fr.num_of_frames)

for i in range(fr.num_of_frames):
    res = fr.get_frame(i, attr)
    if res.success is True:
        ogm_detections = res.results['GrabType.GRAB_TYPE_OGM_LIDAR_DETECTIONS']
        ogm_objects = res.results['GrabType.GRAB_TYPE_OGM_OBJECT_LIST']
        ogm_geometries = res.results['GrabType.GRAB_TYPE_OGM_GEOMETRIES_LIST']
        print(f"Succeeded to read frame number: {res.frame_number}")

print("End of recording")

