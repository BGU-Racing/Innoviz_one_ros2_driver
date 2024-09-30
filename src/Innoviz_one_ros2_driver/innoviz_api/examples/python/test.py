from __future__ import print_function
from innopy.api import DeviceInterface, FrameDataAttributes, GrabType
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class LidarPointCloudViewer:
    def __init__(self):
        self.attr = [FrameDataAttributes(GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION0)]
        config_files_path = '/home/bgr/Desktop/innovizapi_5.12.0_5cdde86f/examples/lidar_configuration_files'
        self.di = DeviceInterface(config_file_name=config_files_path+'/om_config.json', is_connect=False)
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

    def callback(self, h):
        try:
            res = self.di.get_frame(self.attr)
            if res.success:
                points = res.results['GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION0']
                x = [p['x'] for p in points]
                y = [p['y'] for p in points]
                z = [p['z'] for p in points]
                self.ax.clear()
                self.ax.scatter(x, y, z, c='b', marker='o')
                plt.pause(0.01)

        except Exception as e:
            print('Error:', e)

    def display_point_cloud(self):
        self.di.register_new_frame_callback(self.callback)
        plt.show()

    def finish(self):
        self.di.device_close()

def main():
    viewer = LidarPointCloudViewer()
    viewer.display_point_cloud()
    viewer.finish()

if __name__ == '__main__':
    main()
