import struct
from icecream import ic
from ctypes import *
import serial
import time
import can
import threading
import os
import sys
import csv
"""
AA 13 00 03 00 00 00 00 
93 00 7C 4E 0E 02 01 78
73 B8 B4    

    """
    
"""

AA 1A 00 03 00 00 00 00 
9B 00 11 DE 0E 00 E8 03
E8 03 E8 03 01 14 8B AE 77 2A
"""
    
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
    def __init__(self):
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
        self.request_rate = 0.1     #seconds until next
        self.last_msg_id = 0
        self.debug = True
        # self.f = open("debug.log", "w")
        self.msg_id = 0

    def debugPrint(self, debug_msg):
        if self.debug:
            # self.f.write(str(debug_msg))
            # self.f.write('\n')
            ic(debug_msg)

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

    def setPosControl(self, yaw, roll, pitch, ctrl_byte=0x01, time_for_action=0x14):
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
        # self.send_cmd(cmd)
        return cmd

    def getPosData(self):
        hex_data = struct.pack('<1B', 0x01)
        # print(hex_data)
        pack_data = ['{:02X}'.format(i) for i in hex_data]
        # print(pack_data)
        cmd_data = ':'.join(pack_data)
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E',
                           cmd_id='02', data=cmd_data)
        return cmd

    def setFocControl(self, position, cmd_sub_id=0x01, ctl_type=0x00, data_length=0x02):
        # 0-4096 absolute position values
        hex_data = struct.pack('<3B1H', cmd_sub_id, ctl_type, data_length, position)
        # ic(hex_data)
        pack_data = ['{:02X}'.format(i) for i in hex_data]
        cmd_data = ':'.join(pack_data)
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E',
                               cmd_id='12', data=cmd_data)
        return cmd
    
    def getFocPosData(self):
        hex_data = struct.pack('<2B', 0x15,0x00)
        # print(hex_data)
        pack_data = ['{:02X}'.format(i) for i in hex_data]
        # print(pack_data)
        cmd_data = ':'.join(pack_data)
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E',
                           cmd_id='12', data=cmd_data)
        return cmd

    def setSpeedControl(self, yaw, roll, pitch, ctrl_byte=0x80):
        yaw = int(yaw*10.0)
        roll = int(roll*10.0)
        pitch = int(pitch*10.0)
        hex_data = struct.pack('<3hB', yaw, roll, pitch, ctrl_byte)
        pack_data = ['{:02X}'.format(i) for i in hex_data]
        cmd_data = ':'.join(pack_data)
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E',
                           cmd_id='01', data=cmd_data)
        return cmd 
        # print('cmd---data {}'.format(cmd))

    def enable_hand_push(self):
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E', cmd_id='07', data='01')
        return cmd

    def can_buffer_to_full_frame(self):
        # assembling individual CAN messages into whole FRAMES
        full_msg_frames = []
        full_frame_counter = 0
        for i in range(len(self.can_recv_msg_buffer)):
            msg = self.can_recv_msg_buffer[i]
            length = self.can_recv_msg_len_buffer[i]
            msg = msg[:length]
            cmd_data = ':'.join(msg)
            print("len: " + str(length) + " - " +
                  str(msg) + " -> " + cmd_data)
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
        print("RECEIVED DATA",data)
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

if __name__ == "__main__":


    controller = DJIController("/dev/ttyUSB0")

    mode = int(input("CommandID: "))
    if mode == 0:
        while True:
            test_string = input("Enter a string: ")
            input_values = test_string.split(",")
            if len(input_values) == 3:
                yaw = float(input_values[0])
                roll = float(input_values[1])
                pitch = float(input_values[2])
                # ic(yaw)
                # ic(roll)
                # ic(pitch)
                if -180.0 <= yaw <= 180.0 and -180.0 <= roll <= 180.0 and -180.0 <= pitch <= 180.0:
                    controller.setPosControl(yaw, roll, pitch)
                else:
                    print("Vales out of bound")
            else:
                print("Not enough values")
    elif mode == 1:
        while True:
            test_string = input("Enter a string: ")
            input_values = test_string.split(",")
            if len(input_values) == 3:
                yaw = float(input_values[0])
                roll = float(input_values[1])
                pitch = float(input_values[2])
                # ic(yaw)
                # ic(roll)
                # ic(pitch)
                if -360 <= yaw <= 360 and -360 <= roll <= 360 and -360 <= pitch <= 360:
                    controller.setSpeedControl(yaw, roll, pitch)
                else:
                    print("Values out of bound")
            else:
                print("Not enough values")
    elif mode == 2:
        # setFocControl
        # controller.debug = True
        while True:
            test_string = input("Enter a motor position [0-4096]: ")
            input_values = test_string.split(",")
            if len(input_values) == 1:
                # print(input_values[0])
                if 0 <= int(input_values[0]) <= 4096:
                    controller.setFocControl(int(input_values[0]))
                else:
                    print("Values out of bound")
            else:
                print("Not enough values")

    elif mode ==3:
        # controller.debug = True
        # logPositions
        print("Staring to log")
        time.sleep(1)
        # os.system('rm -r dji.log')
        controller.bus.flush_tx_buffer()
        while True:
            # os.system('touch dji.log')
            output = "Pitch: " + \
                        str(controller.pitch) + ", Yaw: " + \
                        str(controller.yaw) + ", Roll: " + str(controller.roll) + ", Focus: " + str(controller.focus)
            print(output)
            #write log to file
            logFile = open("dji.log", "a+")
            logFile.write(str(int(controller.yaw)) + "," + str(int(controller.pitch)) + "," + str(int(controller.roll)) + "," + str(int(controller.focus)) + "\n")
            time.sleep(.05)

    else:
        while True:
            with open(file_dir + "/dji.log", newline="") as f:
                csvreader = csv.reader(f)
                for row in csvreader:
                    y = float(row[0])
                    p = float(row[1])
                    r = float(row[2])
                    f = int(row[3])
                    controller.setPosControl(y, r, p)
                    controller.setFocControl(f)
                    time.sleep(.1)
                break
            False

        print('playback complete, returning to home')
        controller.setPosControl(0, 0, 0)
        controller.bus.flush_tx_buffer()
    time.sleep(2)
    os._exit(0)

