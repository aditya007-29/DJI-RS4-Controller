import can
import time
# bus = can.Bus(interface="canserial", channel="/dev/ttyUSB2", baudrate=2000000, bitrate=1000000, fd=True)
#['aa13000300000000', '0300104e0e020108', '9fb207']

### ['aa1a00030000', '000003007a1e', '0e00c201c201', 'c2010114f606', '53eb']

## Get Pos data ['aa1300030000', '00000300104e', '0e0201089fb2', '07']



with can.Bus(interface="canserial", channel="/dev/ttyUSB2", baudrate=2000000) as bus:
    time.sleep(5)
    for i in range(2):
        ## Send this message ['aa1300030000', '00000300104e', '0e0201089fb2', '07']
        bus.send(can.Message(arbitration_id=0x223, data=[0xaa, 0x13, 0x00, 0x03, 0x00, 0x00], is_extended_id=False))
        time.sleep(0.2)
        bus.send(can.Message(arbitration_id=0x223, data=[0x00, 0x00, 0x03, 0x00, 0x10, 0x4e], is_extended_id=False))
        time.sleep(0.2)
        bus.send(can.Message(arbitration_id=0x223, data=[0x0e, 0x02, 0x01, 0x08, 0x9f, 0xb2], is_extended_id=False))
        time.sleep(0.2)
        bus.send(can.Message(arbitration_id=0x223, data=[0x07], is_extended_id=False))
        time.sleep(0.2)
        ### Send this message['aa1a00030000', '000003007a1e', '0e00c201c201', 'c2010114f606', '53eb']
        # bus.send(can.Message(arbitration_id=0x223, data=[0xaa, 0x1a, 0x00, 0x03, 0x00, 0x00], is_extended_id=False))
        # time.sleep(0.2)
        # bus.send(can.Message(arbitration_id=0x223, data=[0x00, 0x00, 0x03, 0x00, 0x7a, 0x1e], is_extended_id=False))
        # time.sleep(0.2)
        # bus.send(can.Message(arbitration_id=0x223, data=[0x0e, 0x00, 0xc2, 0x01, 0xc2, 0x01], is_extended_id=False))
        # time.sleep(0.2)
        # bus.send(can.Message(arbitration_id=0x223, data=[0xc2, 0x01, 0x01, 0x14, 0xf6, 0x06], is_extended_id=False))
        # time.sleep(0.2)
        # bus.send(can.Message(arbitration_id=0x223, data=[0x53, 0xeb], is_extended_id=False))
        # time.sleep(0.2)
    for msg in bus:
        print(msg.arbitration_id)
        if(msg.arbitration_id == 0x222):
            print(msg)

while True:
    try:
        print(bus.recv(timeout=1.0))
    except Exception as e:
        print(e)