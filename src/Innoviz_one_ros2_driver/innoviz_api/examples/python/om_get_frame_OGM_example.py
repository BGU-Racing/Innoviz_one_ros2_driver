from innopy.api import DeviceInterface, FrameDataAttributes, GrabType
import time


class NewFrameHandler:
    def __init__(self):
        self.attr = [FrameDataAttributes(GrabType.GRAB_TYPE_OGM_LIDAR_DETECTIONS),
                     FrameDataAttributes(GrabType.GRAB_TYPE_OGM_OBJECT_LIST),
                     FrameDataAttributes(GrabType.GRAB_TYPE_OGM_GEOMETRIES_LIST)]
        # Set the absolute path of the configuration file - iPEM.json found in LidarConfig folder
        config_files_path = '../lidar_configuration_files'
        self.di = DeviceInterface(config_file_name=config_files_path+'/iPEM.json', is_connect=False)

        for i in range(len(self.attr)):
            self.di.activate_buffer(self.attr[i], True)

    def callback(self, h):
        try:
            res = self.di.get_frame(self.attr)
            if res.success:
                ogm_detections = res.results['GrabType.GRAB_TYPE_OGM_LIDAR_DETECTIONS']
                ogm_objects = res.results['GrabType.GRAB_TYPE_OGM_OBJECT_LIST']
                ogm_geometries = res.results['GrabType.GRAB_TYPE_OGM_GEOMETRIES_LIST']
                print(f"Succeeded to read frame number: {res.frame_number}")

        except:
            print('NewFrameHandler: failed executing frame')

    def print_data_frames(self):
        self.di.register_new_frame_callback(self.callback)
        time.sleep(10)
        self.di.unregister_new_frame_callback()

    def finish(self):
        self.di.device_close()

def main():

    fh = NewFrameHandler()
    fh.print_data_frames()
    print("The End:)")
    fh.finish()

if __name__ == '__main__':
    main()


