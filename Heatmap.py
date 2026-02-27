import serial
import struct
import numpy as np
import matplotlib.pyplot as plt

# ----- SERIAL CONFIG -----
PORT = "/dev/cu.usbserial-0001"
BAUD = 460800
NUM_VALUES = 64
GRID = 8
DATA_BYTES = NUM_VALUES * 2
PACKET_SIZE = 1 + DATA_BYTES + 1  # start + data + checksum
# -------------------------

ser = serial.Serial(PORT, BAUD)
buffer = bytearray()

# ----- Matplotlib Setup -----
plt.ion()  # interactive mode
fig, ax = plt.subplots()
heatmap = ax.imshow(np.zeros((GRID, GRID)), cmap="viridis", vmin=200, vmax=1000)
plt.colorbar(heatmap)
plt.show()

# -------------------------

while True:
    buffer += ser.read(ser.in_waiting or 1)

    while len(buffer) >= PACKET_SIZE:
        start_index = buffer.find(b'\xAA')
        if start_index == -1:
            buffer.clear()
            break

        if len(buffer) < start_index + PACKET_SIZE:
            break  # wait for full packet

        packet = buffer[start_index:start_index + PACKET_SIZE]
        buffer = buffer[start_index + PACKET_SIZE:]

        data = packet[1:1+DATA_BYTES]
        received_checksum = packet[-1]

        # Verify checksum
        checksum = 0
        for b in data:
            checksum ^= b

        if checksum != received_checksum:
            print("Checksum mismatch")
            continue

        # Convert to 8x8 array
        values = struct.unpack('<' + 'H'*NUM_VALUES, data)
        #print(values)
        frame = np.array(values, dtype=np.float32).reshape(GRID, GRID)
        #frame = np.flipud(frame)  # optional: match sensor orientation

        # Update heatmap
        heatmap.set_data(frame)
        fig.canvas.draw_idle()
        fig.canvas.flush_events()