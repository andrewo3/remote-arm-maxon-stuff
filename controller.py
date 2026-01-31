import usb.core
import usb.util
import ctypes
from threading import Thread
import time

_POWER_A_VENDOR = 0x20d6

def find_controllers():
   return [c.idProduct for c in usb.core.find(find_all=True, idVendor=_POWER_A_VENDOR)]

def getdict(struct):
    return dict((field[0], getattr(struct, field[0])) for field in struct._fields_)


class ButtonData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("type",ctypes.c_uint8),
        ("const_0",ctypes.c_uint8),
        ("id",ctypes.c_uint16),
        ("sync",ctypes.c_uint8,1),
        ("dummy1",ctypes.c_uint8,1),
        ("start",ctypes.c_uint8,1),
        ("back",ctypes.c_uint8,1),
        ("a",ctypes.c_uint8,1),
        ("b",ctypes.c_uint8,1),
        ("x",ctypes.c_uint8,1),
        ("y",ctypes.c_uint8,1),
        ("dpad_up",ctypes.c_uint8,1),
        ("dpad_down",ctypes.c_uint8,1),
        ("dpad_left",ctypes.c_uint8,1),
        ("dpad_right",ctypes.c_uint8,1),
        ("bumper_left",ctypes.c_uint8,1),
        ("bumper_right",ctypes.c_uint8,1),
        ("stick_left_click",ctypes.c_uint8,1),
        ("stick_right_click",ctypes.c_uint8,1),
        ("trigger_left",ctypes.c_uint16),
        ("trigger_right",ctypes.c_uint16),
        ("stick_left_x",ctypes.c_int16),
        ("stick_left_y",ctypes.c_int16),
        ("stick_right_x",ctypes.c_int16),
        ("stick_right_y",ctypes.c_int16)
    ]

class PowerAController():
    def __init__(self,id):
        self.dev = usb.core.find(idVendor=_POWER_A_VENDOR, idProduct=id)
        time.sleep(0.5)
        self.dev.set_configuration()
        self.cfg = self.dev.get_active_configuration()
        self.data = []
        intf = self.cfg[(0,0)]  # First configuration/interface/endpoint
        self.guide = 0
        self.connected = False

        self.ep_in = usb.util.find_descriptor(
            intf,
            custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
        )
        self.ep_out = usb.util.find_descriptor(
            intf,
            custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT
        )
        self.button_data = ButtonData()
        self.wakeup()
        self.runningThread = Thread(target=self.operation)
        self.runningThread.daemon = True
        self.runningThread.start()
    def stop(self):
        self.connected = False
        self.runningThread.join()
        self.kill_connection()
        
    def rumble(self,num_pulses=1,pulse_length=0x80,forces=[0.1,0.1,0.1,0.1]):
        start_rumble = b'\x09\x08\x00\x08\x00'
        bitmask = 0
        for f in range(4):
            if forces[f] > 0:
                bitmask |= 1 << f
        start_rumble += bytes([bitmask])
        
        forces = [(int(i*256))>>2 for i in forces]
        start_rumble += bytes(forces)
        
        start_rumble += bytes([pulse_length])
        start_rumble += b'\x00'
        start_rumble += bytes([num_pulses])
        
        self.dev.write(self.ep_out.bEndpointAddress,start_rumble)    
    
    def operation(self):
        self.read_buttons()
        
    def wakeup(self):
        self.dev.write(self.ep_out.bEndpointAddress,b'\x05\x20',timeout=0)
        self.connected = True
        
    def kill_connection(self):
        self.dev.write(self.ep_out.bEndpointAddress,b'\x01\x20\x00\x00\x00\x00\x00\x00\x00\x0e',timeout=0)
    
    def read_buttons(self):
        while self.connected:
            try:
                self.data = self.dev.read(self.ep_in.bEndpointAddress, 256,timeout=1)
            except usb.core.USBTimeoutError:
                continue
            except usb.core.USBError:
                self.connected = False
                
            if self.data[0] == 0x20: # if button data msg
                buffer = ctypes.create_string_buffer(bytes(self.data))
                self.button_data = ButtonData.from_buffer(buffer)
            elif self.data[0] == 0x07: #guide button msg
                self.guide = self.data[4]
        
