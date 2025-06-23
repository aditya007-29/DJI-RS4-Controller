import cv2 as cv
from cv2 import aruco
import numpy as np
import serial
import struct
from icecream import ic
from ctypes import *
import time
import can
import threading
import os
import sys
import csv

file_dir = os.path.dirname(__file__)
sys.path.append(file_dir)
from check_sum import *

class VCI_CAN_OBJ(Structure):
    _fields_ = [
        ("ID", c_uint),
        ("TimeStamp", c_uint),
        ("TimeFlag", c_ubyte),
        ("SendType", c_ubyte),
        ("RemoteFlag", c_ubyte),
        ("ExternFlag", c_ubyte),
        ("DataLen", c_ubyte),
        ("Data", c_ubyte * 8),
        ("Reserved", c_ubyte * 3),
    ]
    
class DJIController():
    def __init__(self, can_bus):
        self.header = 0xAA
        # ENC
        self.enc = 0x00
        # RES
        self.res1 = 0x00
        self.res2 = 0x00
        self.res3 = 0x00
        # SEQ
        self.Seq_Init_Data = 0x0002

        # CAN BUS
        self.bus = can.interface.Bus(interface="canserial", channel=can_bus, baudrate=2000000)
        self.send_id = int("223", 16)
        self.recv_id = int("222", 16)
        # CAN Specs
        # Data frame len
        self.FRAME_LEN = 8
        self.NORMAL_SEND = 0
        self.SINGLE_SEND = 1
        # Remote Transmission request or not
        self.DATA_FRAME = 0
        self.REMOTE_FRAME = 1
        #
        self.STD_FRAME = 0
        self.EXT_FRAME = 1
        # Data storage
        self.can_recv_msg_buffer = []
        self.can_recv_msg_len_buffer = []
        self.can_recv_buffer_len = 10
        # Standard IDs
        self.cmd_callback_posControl = 0x0E00
        self.cmd_callback_speedControl = 0x0E01
        self.cmd_callback_getGimbalInfo = 0x0E02
        self.cmd_callback_setAngleLimit = 0x0E03
        self.cmd_callback_getAngleLimit = 0x0E04
        self.cmd_callback_setMotorStrength = 0x0E05
        self.cmd_callback_getMotorStrength = 0x0E06
        self.cmd_callback_setPush = 0x0E07
        self.cmd_callback_pushData = 0x0E08
        self.cmd_callback_cameraControl = 0x0D00
        self.cmd_callback_focusControl = 0x0D01
        # Current Position
        self.yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.focus = 0
        # Multithreading CAN Callback
        self.request_rate = 0.14     #seconds until next
        self.last_msg_id = 0
        self.debug = False
        # self.f = open("debug.log", "w")
        self.msg_id = 0
        self.b = threading.Thread(name='background', target=self.can_callback)
        self.b.start()
        self.b2 = threading.Thread(name='background', target=self.request_cur_data)
        self.b2.start()

    def debugPrint(self, debug_msg):
        if self.debug:
            # self.f.write(str(debug_msg))
            # self.f.write('\n')
            ic(debug_msg)

    def request_cur_data(self):
        while True:
            self.getPosData()
            self.getFocPosData()
            time.sleep(self.request_rate)

    def seq_num(self):
        # Generate Sequence number by incrementing it
        # global Seq_Init_Data
        if self.Seq_Init_Data >= 0xFFFD:
            self.Seq_Init_Data = 0x0002
        self.Seq_Init_Data += 1
        # Seq_Init_Data = 0x1122
        seq_str = "%04x" % self.Seq_Init_Data
        return seq_str[2:] + ":" + seq_str[0:2]

    def assemble_can_msg(self,cmd_type, cmd_set, cmd_id, data):
        # Assembling a respective CAN message
        if data == "":
            can_frame_data = "{prefix}" + \
            ":{cmd_set}:{cmd_id}".format(cmd_set=cmd_set, cmd_id=cmd_id)
        else:
            can_frame_data = "{prefix}" + ":{cmd_set}:{cmd_id}:{data}".format(cmd_set=cmd_set, cmd_id=cmd_id, data=data)

        cmd_length = len(can_frame_data.split(":")) + 15
        seqnum = self.seq_num()
        # ic(seqnum)
        can_frame_header = "{header:02x}".format(header=self.header)  # SOF byte
        can_frame_header += ":" + ("%04x" % (cmd_length))[2:4]  # 1st length byte
        can_frame_header += ":" + ("%04x" % (cmd_length))[0:2]  # 2nd length byte
        can_frame_header += ":" + \
        "{cmd_type}".format(cmd_type=cmd_type)  # Command Type
        can_frame_header += ":" + "{enc:02x}".format(enc=self.enc)  # Encoding
        can_frame_header += ":" + "{res1:02x}".format(res1=self.res1)  # Reserved 1
        can_frame_header += ":" + "{res2:02x}".format(res2=self.res2)  # Reserved 2
        can_frame_header += ":" + "{res3:02x}".format(res3=self.res3)  # Reserved 3
        can_frame_header += ":" + seqnum    # Sequence number
        can_frame_header += ":" + calc_crc16(can_frame_header)
        # hex_seq = [eval("0x" + hex_num) for hex_num in can_frame_header.split(":")]
        whole_can_frame = can_frame_data.format(prefix=can_frame_header)
        whole_can_frame += ":" + calc_crc32(whole_can_frame)
        whole_can_frame = whole_can_frame.upper()
        #
        # print("Header: ", can_frame_header)
        # print("Total: ", whole_can_frame)
        return whole_can_frame

    def setPosControl(self, yaw, roll, pitch, ctrl_byte=0x01, time_for_action=0x5):
        yaw = int(yaw*10.0)
        roll = int(roll*10.0)
        pitch = int(pitch*10.0)
        # ic(yaw)
        # Turn Position Data into CAN message
        # yaw, roll, pitch in 0.1° steps (-1800,1800)
        # ctrl_byte always to 1
        # time_for_action to define speed in 0.1sec
        hex_data = struct.pack('<3h2B', yaw, roll, pitch,
                           ctrl_byte, time_for_action)
        # ic(hex_data)
        pack_data = ['{:02X}'.format(i) for i in hex_data]
        cmd_data = ':'.join(pack_data)
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E',
                           cmd_id='00', data=cmd_data)
        self.send_cmd(cmd)

    def getPosData(self):
        hex_data = struct.pack('<1B', 0x01)
        # print(hex_data)
        pack_data = ['{:02X}'.format(i) for i in hex_data]
        # print(pack_data)
        cmd_data = ':'.join(pack_data)
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E',
                           cmd_id='02', data=cmd_data)
        self.send_cmd(cmd)

    def setFocControl(self, position, cmd_sub_id=0x01, ctl_type=0x00, data_length=0x02):
        # 0-4096 absolute position values
        hex_data = struct.pack('<3B1H', cmd_sub_id, ctl_type, data_length, position)
        # ic(hex_data)
        pack_data = ['{:02X}'.format(i) for i in hex_data]
        cmd_data = ':'.join(pack_data)
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E',
                               cmd_id='12', data=cmd_data)
        self.send_cmd(cmd)

    def getFocPosData(self):
        hex_data = struct.pack('<2B', 0x15,0x00)
        # print(hex_data)
        pack_data = ['{:02X}'.format(i) for i in hex_data]
        # print(pack_data)
        cmd_data = ':'.join(pack_data)
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E',
                           cmd_id='12', data=cmd_data)
        self.send_cmd(cmd)

    def setSpeedControl(self, yaw, roll, pitch, ctrl_byte=0x80):
        yaw = int(yaw*10.0)
        roll = int(roll*10.0)
        pitch = int(pitch*10.0)
        hex_data = struct.pack('<3hB', yaw, roll, pitch, ctrl_byte)
        pack_data = ['{:02X}'.format(i) for i in hex_data]
        cmd_data = ':'.join(pack_data)
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E',
                           cmd_id='01', data=cmd_data)
        # print('cmd---data {}'.format(cmd))
        self.send_cmd(cmd)

    def enable_hand_push(self):
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E', cmd_id='07', data='01')
        self.send_cmd(cmd)

    def send_cmd(self,cmd):
        # Preparing data for bus output
        data = [int(i, 16) for i in cmd.split(":")]
        # self.lastSendData = data
        # status=False
        self.send_data(self.send_id, data)
        # send_data(send_id, cmd)

    def send_data(self,can_id, data):
        # Pushing Data out the CAN bus
        data_len = len(data)
        full_frame_num, left_len = divmod(data_len, self.FRAME_LEN)
        # ic(full_frame_num)
        # ic(left_len)
        # ic(data_len)
        # ic(data)
        if left_len == 0:
            frame_num = full_frame_num
        else:
            frame_num = full_frame_num + 1
        # ic(frame_num)
        send_buf = (VCI_CAN_OBJ * (frame_num))()
        # ic(send_buf)
        data_offset = 0
        for i in range(full_frame_num):
            send_buf[i].ID = can_id
            send_buf[i].SendType = self.NORMAL_SEND
            send_buf[i].RemoteFlag = self.DATA_FRAME
            send_buf[i].ExternFlag = self.STD_FRAME
            send_buf[i].DataLen = self.FRAME_LEN
            for j in range(self.FRAME_LEN):
                send_buf[i].Data[j] = data[data_offset + j]
            data_offset += self.FRAME_LEN

            # If there is data left over, the last frame isn't 8byte long
            if left_len > 0:
                send_buf[frame_num - 1].ID = can_id
                send_buf[frame_num - 1].SendType = self.NORMAL_SEND
                send_buf[frame_num - 1].RemoteFlag = self.DATA_FRAME
                send_buf[frame_num - 1].ExternFlag = self.STD_FRAME
                send_buf[frame_num - 1].DataLen = left_len
                for j in range(left_len):
                    send_buf[frame_num - 1].Data[j] = data[data_offset + j]

            for i in range(frame_num):
                frame = bytearray()
                for j in range(send_buf[i].DataLen):
                    frame.append(send_buf[i].Data[j])
                msg = can.Message(arbitration_id=0x223, data=frame, is_extended_id=False)
                try:
                    self.bus.send(msg)
                    time.sleep(0.001)
                    # print("Message sent on {}".format(bus.channel_info))
                except can.CanError:
                    print("Message NOT sent")

    def can_buffer_to_full_frame(self):
        # assembling individual CAN messages into whole FRAMES
        full_msg_frames = []
        full_frame_counter = 0
        for i in range(len(self.can_recv_msg_buffer)):
            msg = self.can_recv_msg_buffer[i]
            length = self.can_recv_msg_len_buffer[i]
            msg = msg[:length]
            cmd_data = ':'.join(msg)
            # print("len: " + str(length) + " - " +
            #       str(msg) + " -> " + cmd_data)
            if msg[0] == "AA":
                full_msg_frames.append(msg)
                full_frame_counter += 1
            if msg[0] != "AA" and (full_frame_counter > 0):
                # full_msg_frames[-1] += ":"
                for byte in msg:
                    full_msg_frames[-1].append(byte)
        return full_msg_frames

    def validate_api_call(self, data_frame):
        # Validating received frames
        validated = False
        check_sum = ':'.join(data_frame[-4:])
        data = ':'.join(data_frame[:-4])
        # # print(len(hex_data))
        # # print(data)
        if len(data_frame) >= 8:
            if check_sum == calc_crc32(data):
                #         # print("Approved Message: " + str(hex_data))
                header = ':'.join(data_frame[:10])
                header_check_sum = ':'.join(data_frame[10:12])
                if header_check_sum == calc_crc16(header):
                    validated = True
        return validated

    def parse_focus_motor_response(self, data_frame):
        self.debugPrint("focus Motor response?")
        self.debugPrint(data_frame)
        pos_data = data_frame[-8:-4]
        self.debugPrint("value frame")
        self.debugPrint(pos_data)
        value = int('0x'+pos_data[3] + pos_data[2]+pos_data[1] + pos_data[0], base=16)
        self.debugPrint(value)
        self.focus = value
        # print(value)

    def parse_position_response(self, data_frame):
        pos_data = data_frame[16:-4]
        yaw = int(
            '0x' + pos_data[1] + pos_data[0], base=16)
        roll = int(
            '0x' + pos_data[3] + pos_data[2], base=16)
        pitch = int(
            '0x' + pos_data[5] + pos_data[4], base=16)
        if yaw > 1800:
            yaw -= 65538
        if roll > 1800:
            roll -= 65538
        if pitch > 1800:
            pitch -= 65538
        self.yaw = yaw * 0.1 #* np.pi / 180
        self.roll = roll * 0.1 #* np.pi / 180
        self.pitch = pitch * 0.1 #* np.pi / 180

    def can_callback(self):
        while True:
            self.debugPrint(self.last_msg_id)
            if(65536 < self.last_msg_id):
                self.last_msg_id = 0
            for msg in self.bus:
                # print(msg)
                # print(msg.arbitration_id)
                if msg.arbitration_id == self.recv_id:
                    # ic(msg)
                    str_data = ['{:02X}'.format(struct.unpack('<1B', i.to_bytes(1, 'big'))[
                                                0]) for i in msg.data]
                    self.can_recv_msg_buffer.append(str_data)
                    self.can_recv_msg_len_buffer.append(msg.dlc)
                    if len(self.can_recv_msg_buffer) > self.can_recv_buffer_len:
                        # print("Pop")
                        self.can_recv_msg_buffer.pop(0)
                        self.can_recv_msg_len_buffer.pop(0)
                    full_msg_frames = self.can_buffer_to_full_frame()
                    for hex_data in full_msg_frames:
                        self.debugPrint(":".join(hex_data))
                        self.debugPrint(hex_data)
                        if self.validate_api_call(hex_data):
                            cur_msg_id = int('0x' + hex_data[9] + hex_data[8], base=16)
                            self.debugPrint(cur_msg_id)
                            if True:
                                self.last_msg_id = cur_msg_id
                                request_data = ":".join(hex_data[12:14])
                                self.debugPrint(":".join(hex_data))
                                self.debugPrint(request_data)
                                if request_data == "0E:02":
                                    self.parse_position_response(hex_data)
                                elif request_data == "0E:12":
                                    self.parse_focus_motor_response(hex_data)
                        else:
                            self.debugPrint("invalid message")
                    # ic(full_msg_frames)


# Define camera intrinsic parameters (example values, adjust as per your camera)
fx = 962.6 # Focal length in pixels (along x-axis)
fy = 1085.46 # Focal length in pixels (along y-axis)
cx = 997.89  # Principal point (x-coordinate) in pixels
cy = 505.60  # Principal point (y-coordinate) in pixels

# Define distortion coefficients (typically obtained from camera calibration)
k1 = -0.813251508
k2 = 16.1669366
p1 = -0.0218294859
p2 = 0.0727941984
k3 = -92.6641208




# Define camera matrix and distortion coefficients
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]])

dist_coeffs = np.array([k1, k2, p1, p2, k3])

# Dictionary to specify the type of the marker
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
param_markers = aruco.DetectorParameters()

# Marker size in meters (adjust according to the actual size of your marker)
marker_size = 0.1  # 10 cm in this example

# Utilize the default camera/webcam driver
# Setup camera
print("Setting up camera...")
cap = cv.VideoCapture(0)  # Adjust the index as needed

controller = DJIController("/dev/ttyUSB1")

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()
else:
    print("Camera connection established.")





while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        continue

    try:
        
        # Get the frame dimensions
        height, width, _ = frame.shape

        # Calculate the center of the frame
        frame_center_x = width // 2
        frame_center_y = height // 2

        # Draw a red dot at the center of the frame
        cv.circle(frame, (frame_center_x, frame_center_y), 5, (0, 0, 255), -1)

        # Turn the frame to grayscale-only (for efficiency)
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # Detect ArUco markers
        marker_corners, marker_IDs, _ = aruco.detectMarkers(gray_frame, marker_dict, parameters=param_markers)

        if marker_corners:
            for ids, corners in zip(marker_IDs, marker_corners):
                # Draw the detected markers
                cv.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)

                # Calculate center of the marker
                center = np.mean(corners, axis=0).astype(int)
                aruco_center_x, aruco_center_y = center

                # Calculate the difference between the frame center and the marker center
                dy = aruco_center_y - frame_center_y
                dx = aruco_center_x - frame_center_x
                distance = np.sqrt(dx**2 + dy**2)
                distance_threshold = 10
                pitch = 0
                yaw = 0


                
                if distance > distance_threshold:
                    distance_within_threshold_start_time = None
                    if dy != 0:
                        pitch += np.degrees(np.arctan2(-dy, fy))
                    if dx != 0:
                        yaw += np.degrees(np.arctan2(dx, fx))

                    controller.setSpeedControl(yaw, 0, pitch)
                    output = "Pitch: " + \
                        str(controller.pitch) + ", Yaw: " + \
                        str(controller.yaw) + ", Roll: " + str(controller.roll) + ", Focus: " + str(controller.focus)
                    print(output)

                

                # Incrementally adjust pitch and yaw based on the distance
                 
                    # pitch, yaw = set_gimbal_orientation(pitch_increment, 0, yaw_increment)
                    # print(pitch_increment)

                    
                # else:
                    
                #     if distance_within_threshold_start_time is None:
                #         distance_within_threshold_start_time = time.time()
                #     elif time.time() - distance_within_threshold_start_time > 0.5:
                #         # Calculate X and Y using the provided formula
                #         d = 500  # Example distance value; replace as needed
                #         B = np.degrees(np.arctan2(d * np.cos(np.radians(yaw_increment)), d * np.sin(np.radians(yaw_increment)) - 80))
                #         C = np.degrees(np.arctan2(d * np.cos(np.radians(yaw_increment)), d * np.sin(np.radians(yaw_increment)) + 160))
                #         X = (90 - B) % 360
                #         Y = (90 - C) % 360
                #         Z = -pitch_increment
                        

                #         message_1 = f"1 {X}\n"
                #         message_2 = f"2 {Y}\n"
                #         message3=f"{Z}\n"
                #         # try:
                #         ser.write(message_1.encode())
                #         print(f"Sent message to ESP32: {message_1.strip()}")
                #         ser.write(message_2.encode())
                #         print(f"Sent message to ESP32: {message_2.strip()}")
                        
                #         # Send message to Arduino Mega
                #         esp2.write(message3.encode())
                #         print("Sent testing message to Arduino Mega: 45")

                #         # # Reset variables to ensure re-entering the loop correctly
                #         # distance_within_threshold_start_time = None
                #         # pitch_increment = 0
                #         # yaw_increment = 0
                #         # except Exception as e:
                #         #     print(f"Failed to send message: {e}")

        # Show the frame with markers and tracking info
        cv.imshow("Frame", frame)

    except Exception as e:
        print(f"Error in processing frame: {e}")

    key = cv.waitKey(1)
    if key == ord("q"):
        break

# Release camera and close all windows
cap.release()
cv.destroyAllWindows()
print("Released the camera and destroyed all windows.")