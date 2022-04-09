import os
from shutil import copyfile

input_dir = '/home/msardonini/Pictures/flight_vehicle_cal_imgs/flight_cal'
output_dir = '/home/msardonini/Pictures/flight_vehicle_cal_imgs/flight_cal_rotated'

# if __name__ == "__main__":
#   files = os.listdir(input_dir)

#   file_count = 1

#   # Copy over all the matching paris of images
#   for file in files:
#     # check if its a cam0 or cam1 image
#     if 'cam0' in file:
#       # Check if this image has a corresponding image
#       cam1_filename = file.replace('cam0', 'cam1')
#       print("cam1 filename ", cam1_filename)
#       if cam1_filename in files:
#         copyfile(input_dir + '/' + file, output_dir + '/' + 'cam0_' + str(file_count) + '.png')
#         copyfile(input_dir + '/' + cam1_filename, output_dir + '/' +  'cam1_' + str(file_count) + '.png')
#         file_count = file_count + 1

#         files.remove(file)
#         files.remove(cam1_filename)

#   # Move over all other non-matching files
#   for file in files:
#     if 'cam0' in file:
#       copyfile(input_dir + '/' + file, output_dir + '/' + 'cam0_' + str(file_count) + '.png')
#       file_count = file_count + 1
#     elif 'cam1' in file:
#       copyfile(input_dir + '/' + file, output_dir + '/' + 'cam1_' + str(file_count) + '.png')
#       file_count = file_count + 1

input_dir = '/home/msardonini/Pictures/flight_vehicle_cal_imgs/flight_cal_rotated'
output_dir = '/home/msardonini/Pictures/flight_vehicle_cal_imgs/flight_cal_rotated'

import cv2

if __name__ == "__main__":
    # Roatate all cam1 files
    files = os.listdir(input_dir)

    for file in files:
        if 'cam1' in file:
            image = cv2.imread(input_dir + '/' + file)
            img_rotate_180 = cv2.rotate(image, cv2.ROTATE_180)
            cv2.imwrite(output_dir + '/' + file, img_rotate_180)
