import serial
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ------------------ SERIAL CONFIG ------------------
PORT = '/dev/serial0'
# Where your LiDAR is connected
BAUDRATE = 230400
# Communication speed

# ------------------ LIDAR BUFFERS ------------------
# Creates a 360 degree "map buffer"
angles_deg = np.arange(360)
angles_rad = np.deg2rad(angles_deg)

distances = np.zeros(360)   # mm
x = np.zeros(360)
y = np.zeros(360)
# Each index = one degree of LiDAR
# ------------------ CRC CHECK ------------------
# Checks whether the data package is valid or invalid 
def crc_check(packet):
    data = packet[:-1]
    crc = packet[-1]
    calc = 0
    for b in data:
        calc ^= b
    return calc == crc

# ------------------ PACKET PARSER ------------------
# This converts raw LiDAR bytes to readable distances and angles
def parse_packet(data):
    if len(data) != 47:
        return None
    if data[0] != 0x54:
        return None

    # Little-endian 2-byte start/end angles
    start_angle = (data[4] | (data[5] << 8)) / 100.0
    end_angle   = (data[42] | (data[43] << 8)) / 100.0

    distances_out = []
    angles_out = []
    rssi = []

    num_points = 12
    step = (end_angle - start_angle) / (num_points - 1)

    for i in range(num_points):
        idx = 6 + i * 3
        d = (data[idx] | (data[idx + 1] << 8)) / 10.0  # mm
        a = int(round(start_angle + step * i))
        distances_out.append(d)
        angles_out.append(a)
        rssi.append(data[idx + 2])

    return distances_out, angles_out, rssi



# ------------------ SERIAL STEP READER ------------------
# This function reads raw bytes, then finds the packets, then parses them, and updates the distance map
"""def read_lidar_step(ser, buffer):
    byte = ser.read(1)
    if not byte:
        return

    buffer += byte
    if len(buffer) > 48:
        buffer[:] = buffer[-48:]

    for i in range(len(buffer) - 46):
        if buffer[i] != 0x54:
            continue

        packet = buffer[i:i+47]
        #print("Candidate packet len:", len(packet))

        #if not crc_check(packet):
         #   continue

        parsed = parse_packet(packet)
        if parsed is None:
            continue

        dists, angs, _ = parsed

        for d, a in zip(dists, angs):
            if 0 <= a < 360:
                distances[a] = d
                print("Sample:", a, d)
                """
HEADER = 0x54  # replace with what your test shows

def read_lidar_step(ser, buffer):
    b = ser.read(1)
    if not b:
        return

    buffer += b
    if len(buffer) > 100:   # safe margin
        buffer[:] = buffer[-100:]

    for i in range(len(buffer) - 46):
        if buffer[i] != HEADER:
            continue

        packet = buffer[i:i+47]

        # TEMP bypass CRC
        # if not crc_check(packet):
        #     continue

        parsed = parse_packet(packet)
        if parsed is None:
            continue

        dists, angs, _ = parsed
        for d, a in zip(dists, angs):
            if 0 <= a < 360:
                distances[a] = d
                print("Sample:", a, d)   # <-- will now actually print




# ------------------ MATPLOTLIB SETUP ------------------
# Creates figure + axis
fig, ax = plt.subplots()
# Plot Settings
ax.set_aspect('equal')
ax.set_xlim(-200, 200)
ax.set_ylim(-200, 200)
ax.set_title("LD19 LiDAR (Cartesian View)")
ax.set_xlabel("X (cm)")
ax.set_ylabel("Y (cm)")
# Displays LiDAR points
scatter = ax.scatter(x, y, s=5)

# ------------------ ANIMATION UPDATE ------------------
def update(frame):
    # This runs continuously
    #print("FRAME")
    x[:] = distances * np.cos(angles_rad)
    y[:] = distances * np.sin(angles_rad)
    # Converts polar into cartesian
    mask = distances > 0
    # Ignore empty readings
    scatter.set_offsets(np.c_[x[mask], y[mask]])
    # This redraws only valid points 
    #print("Nonzero distances:", np.count_nonzero(distances))
    return scatter,
#def update(frame):
    #r = 2000
    #x[:] = r * np.cos(angles_rad)
    #y[:] = r * np.sin(angles_rad)
    #scatter.set_offsets(np.c_[x, y])
    #return scatter,

# ------------------ MAIN ------------------
buffer = bytearray()
ser = serial.Serial(PORT, BAUDRATE, timeout=0)
# Main animation loop
ani = animation.FuncAnimation(
    fig,
    update,
    interval=50,
    blit=False,
    cache_frame_data=False
)
# Serial safety timer 
def serial_timer():
    try:
        read_lidar_step(ser, buffer)
    except Exception as e:
        print("Serial read error:", e)


timer = fig.canvas.new_timer(interval=1)
# Creates a timer that runs every 1 ms
timer.add_callback(serial_timer)
# Continuosly reads serial, updates buffer, and parses packets
timer.start()


plt.show()
# Opens window and starts event loop 
ser.close()
# Closes serial connection when program ends  
                            


