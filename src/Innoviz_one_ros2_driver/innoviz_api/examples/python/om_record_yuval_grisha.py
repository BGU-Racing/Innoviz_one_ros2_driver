
from innopy.api import DeviceInterface, FrameDataAttributes, GrabType
import threading
from datetime import datetime
import sys
import csv

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
        # pixel_number = 10 # consider change to pixel_number = h
        try:
            res = di.get_frame(attr)
            if res.success:
                mes = res.results['GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION0']
                file_name = f"Frame {frame_count}"
                csv_time_start = datetime.now()
                # formatted_start = csv_time_start.strftime("%H:%M:%S")

                with open(file_name, 'w', newline='') as file:
                    writer = csv.writer(file)
                    data = [[frame_count],[csv_time_start],
                    ["reflectivity", "x" , "y", "z"]]
                    writer.writerows(data)
                    for i in range(len(mes)):
                        if mes[i]['distance'] < 2000: # and mes[i]['reflectivity'] > 61:
                            curr_mes = [[(mes[i])['reflectivity'],(mes[i])['x'],(mes[i])['y'],(mes[i])['z']]]
                            writer.writerows(curr_mes)

                frame_count += 1

                csv_time_finish = datetime.now()
                formatted_finish = csv_time_finish.strftime("%H:%M:%S")
                csv_creation = csv_time_finish - csv_time_start
                print("csv_creation:",csv_creation)

        except:
            print('NewFrameHandler: failed executing frame')
            break

            
    di.device_close()

    # def print_data_frames(self):
    #     self.di.register_new_frame_callback(self.callback)
    #     time.sleep(10)
    #     self.di.unregister_new_frame_callback()

    def finish(self):
        self.di.device_close()
	# 	READ LIDAR FRAME
	# 	SAVE LIDAR FRAME TO CSV
	
	
	# IMPORTANT: 
	# measure the time for the csv creating and saving. and fix it if necessary.
	
	
	# 

if __name__ == '__main__':
    main()
