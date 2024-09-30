from innopy.api import DeviceInterface, FrameDataAttributes, GrabType
import threading
from datetime import datetime
import sys
import struct
import time
def main():
    # TODO INIT INNOVIZ OBJECT
    attr = [FrameDataAttributes(GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION0)]
    config_files_path = '/home/bgr/Desktop/innovizapi_5.12.0_5cdde86f/examples/lidar_configuration_files'
                
    # A = classA()  
    di = DeviceInterface(config_file_name=config_files_path+'/om_config.json', is_connect=False)

    for i in range(len(attr)):
        di.activate_buffer(attr[i], True)
            
    frame_count = 0
    # INFINITE LOOP/UNTIL FRAME IS NOT AVAILABLE 
    while True:
        try:
            res = di.get_frame(attr)
            if res.success:
                mes = res.results['GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION0']
                file_name = f"Frame_{frame_count}.bin"
                bin_time_start = datetime.now()

                # Prepare the data for binary writing
                data = bytearray()
                data.extend(struct.pack('I', frame_count))
                data.extend(struct.pack('d', bin_time_start.timestamp()))
                for m in mes:
                    if m['distance'] < 2000:
                        data.extend(struct.pack('ffff', m['reflectivity'], m['x'], m['y'], m['z']))

                # Write data to binary file
                start = time.time()
                with open(file_name, 'wb') as file:
                    print("time:",time.time()-start)
                    file.write(data)

                frame_count += 1

                bin_time_finish = datetime.now()
                bin_creation = bin_time_finish - bin_time_start
                print("bin_creation:", bin_creation)

        except Exception as e:
            print('NewFrameHandler: failed executing frame', e)
            break

    di.device_close()

    def finish(self):
        self.di.device_close()

if __name__ == '__main__':
    main()
