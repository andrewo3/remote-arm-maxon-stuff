import socket, controller, ctypes, threading
from multiprocessing import shared_memory
import struct
import array


NODES = 2
SHM_NAME = "motor_velocities"

shm_size = NODES * 4
shm = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=shm_size)
shm_buf = shm.buf

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(('0.0.0.0',3001))
sock.listen(5)

def client(sock):
    while True:
        data = sock.recv(1024)
        if not data:
            sock.close()
            return
        values = struct.unpack("i"*NODES, data)
        print("Received:", values)
        values[0] //= 5
        values[1] //= 5

        # write to shared memory using array slice
        arr = array.array('i', values)
        shm_buf[:NODES*4] = arr.tobytes()
        
        print("Updated shared memory:", values)
    
try:
    while True:
        c,add = sock.accept()
        print(c,"!!!")
        client_thread = threading.Thread(target=client,args=[c])
        client_thread.start()
except KeyboardInterrupt:
    shm.close()
    sock.close()
    shm.unlink()