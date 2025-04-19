from dji_rs2_controller_v2_dev import DJIController
from usbcana import UsbCanAdapter
import time 
import os

def read_pos_response():
    pass

def split_string(str, seg_size=12):
    str = str[2:]
    commands = [""]
    count = 0
    for ch in str:
        if count==seg_size:
            count = 0
            commands.append("")
    
        commands[-1]+=(ch)
        count+=1
    return commands

commander = DJIController()
# cmd = commander.setPosControl(45,45,45)
cmd = commander.getPosData()
cmd = cmd.replace(":","")
cmd = hex(int(cmd,16))

# adapter = UsbCanAdapter()
# adapter.adapter_init("COM9", 2000000)

frames = split_string(cmd)
print(split_string(cmd))

print("INJECTING DATA FRAME")
injected_frames = []
port = "/dev/ttyUSB2"
for i in range(2):
    for frame in frames:
        print(frame)
        command = f"./USB-CAN-A/z -d {port} -s 1000000 -i 223 -g 100  -j {frame} -n 1"
        print(command)
        os.system(command)
        # time.sleep(0.001)
    # injected_frames.append(adapter.inject_data_frame("223", frame))
    
# print(injected_frames)
# for command in 
# injected_data = adapter.inject_data_frame("223", cmd)
# print("INJECTED")
# print(injected_data)
# for i in range(10):
#     # adapter.frame_send(injected_data)
#     time.sleep(0.1)
