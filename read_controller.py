import socket, controller, ctypes, threading
from multiprocessing import shared_memory
import struct


NODES = 2
SHM_NAME = "motor_velocities"

shm_size = NODES * 4
shm = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=shm_size)
shm_buf = shm.buf

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(('0.0.0.0',3001))
sock.listen(5)

def client(sock):
    def rumble(num_pulses=1,pulse_length=0x80,forces=[0.1,0.1,0.1,0.1]):
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
        sock.send(start_rumble)
    while True:
        b = ctypes.create_string_buffer(sock.recv(1024))
        cont_buttons = controller.ButtonData.from_buffer(b)
        if not b:
            sock.close()
            return
        #print(cont_buttons.stick_left_x,cont_buttons.stick_left_y)
        
        # Example mapping: left stick X/Y -> motor velocities
        v0 = int(cont_buttons.stick_left_x/32768*500)
        v1 = int(cont_buttons.stick_left_y/32768*500)
        
        print(v0,v1)

        # Write unsigned ints into shared memory
        struct.pack_into("i", shm_buf, 0, v0)
        struct.pack_into("i", shm_buf, 4, v1)
    
while True:
    c,add = sock.accept()
    print(c,"!!!")
    client_thread = threading.Thread(target=client,args=[c])
    client_thread.start()