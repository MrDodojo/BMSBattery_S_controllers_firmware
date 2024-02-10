import serial
from binascii import hexlify
from time import time
# start 0xaa55aa55
from statistics import median
import signal
import sys

effective = []
crossover_result = []
ctr = [[], [], [], [], [], [], []]
counter = 0
first = True
def signal_handler(sig, frame):
    a = ((median(ctr[4])) * (255.0/360.0))
    b = ((median(ctr[6])) * (255.0/360.0)) + a
    c = ((median(ctr[2])) * (255.0/360.0)) + b
    d = ((median(ctr[3])) * (255.0/360.0)) + c
    e = ((median(ctr[1])) * (255.0/360.0)) + d
    f = ((median(ctr[5])) * (255.0/360.0)) + e
    co = ((median(crossover_result) - 180) * (255.0/360.0) % 256)

    print("#define ANGLE_4_0 1")
    print("#define ANGLE_6_60 %d" % (b))
    print("#define ANGLE_2_120 %d" % (c))
    print("#define ANGLE_3_180 %d" % (d))
    print("#define ANGLE_1_240 %d" % (e))
    print("#define ANGLE_5_300 %d" % (f))
    print("#define MOTOR_ROTOR_DELTA_PHASE_ANGLE_RIGHT %d" % co) 
    sys.exit(0)

current_offset = 512
signal.signal(signal.SIGINT, signal_handler)
ob = None
ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0)
buffer = b""
update = False
SIZE = 1 + 2 + 1 + 1
start_time = time()
row = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
current = []
previous = 0
cycle = 0
order = [0, 0, 0, 0, 0, 0]
counter = 0
while True:
    data = ser.read(1024)
    buffer = buffer + data
    start = -1
   
    if len(buffer) >= SIZE:
        for i in range(0, len(buffer) - 1):
            if buffer[i] == 0xaa:
                start = i
                break

        if start < 0:
            buffer = b""
        else:
            buffer = buffer[start:]

    if len(buffer) >= SIZE + start and start >= 0:
        # check crc
        crc = 4
        for i in buffer[start:start+SIZE-1]:
            crc = crc ^ i
        try:
            if crc != buffer[start + SIZE-1]:
                buffer = buffer[SIZE:]
                continue
        except:
            print(start, SIZE, buffer)
            exit(1)
        ob = buffer[:SIZE]
        hall_current = buffer[3]
        phase_b_current = int.from_bytes(buffer[1:3], byteorder='big')
        if len(buffer) > SIZE - 1:
            buffer = buffer[SIZE:]
        else:
            buffer = b""
        #if not (phase_b_current < -100 or phase_b_current > 100):
        update = True


    t = (time() - start_time)
    if update and t > 1:
        counter = counter + 1
        if counter < 100:
            current_offset = phase_b_current
            continue
        elif counter == 100:
            print("Phase b zero: %d" % current_offset)

        current.append((time(), phase_b_current - current_offset))
        if previous != hall_current and hall_current <= 7 and hall_current >= 0:
            if hall_current == 7:
                print("Hall '7' detected, you have noisy sensors")
                continue
            row[hall_current] = time()
            previous = hall_current
            order.append(hall_current)
            if len(order) > 6:
                order = order[1:]
            for i in range(0, 6):
                if order[i] == 4:
                    start = i
            sorder = [order[x % 6] for x in range(start, start+6)]
            if sorder == [4, 5, 1, 3, 2, 6]:
                print("Hall order reversed, please edit hall order or add HALL_SENSOR_MODE_REVERSED to config.h")
                exit(1)
            elif sorder != [4, 6, 2, 3, 1, 5]:
                continue
            if hall_current == 4:
                cycle = cycle + 1

                b = row.copy()
                b.sort()
                dur = b[-1] - b[0]

                angle = [0, 0, 0, 0, 0, 0, 0]
                angle[4] = 0 # by definition
                angle[6] = (row[6] - row[0]) * (360.0/dur)
                angle[2] = (row[2] - row[6]) * (360.0/dur)
                angle[3] = (row[3] - row[2]) * (360.0/dur)
                angle[1] = (row[1] - row[3]) * (360.0/dur)
                angle[5] = (row[5] - row[1]) * (360.0/dur)

                for i in range(1, len(current)-1): #[1:]:
                    cp = current[i-1][1]
                    cv = current[i][1]
                    dc = cp + abs(cv) # only needed when cv < 0
                    tp = current[i-1][0]
                    tv = current[i][0]
                    if cp > cv and cp >= 0 and cv <= 0: # either previous or current measurement can be 0
                        crossover = tp + (((tv - tp) / dc) * cp) # take zero point

                if not first:
                    for i in range(1, 7):
                        ctr[i].append(angle[i])
                else:
                    row[0] = row[4]
                    first = False
                    continue

                if crossover != 0:
                    crossover = (crossover - row[0]) * (360.0/dur)
                    crossover_result.append(crossover)
                print("%.3f " % dur, end="")
                print("%.3f " % (row[6] - row[0]), end="")
                print("%.3f " % (row[2] - row[6]), end="")
                print("%.3f " % (row[3] - row[2]), end="")
                print("%.3f " % (row[1] - row[3]), end="")
                print("%.3f " % (row[5] - row[1]), end="")
                print("%.3f  -- " % (row[4] - row[5]), end="")
                for i in sorder:
                    print("%d " % i, end="")
                print(" -- %d " % (median(ctr[4])), end="")
                print("%d " % (median(ctr[6])), end="")
                print("%d " % (median(ctr[2])), end="")
                print("%d " % (median(ctr[3])), end="")
                print("%d " % (median(ctr[1])), end="")
                print("%d " % (median(ctr[5])), end="")
                if len(crossover_result) > 0:
                    print(" -- %d" % median(crossover_result))
                current = []

                row[0] = row[4]
