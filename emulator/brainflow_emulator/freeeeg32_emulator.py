import logging
import math
import os
import threading
import time


class Listener(threading.Thread):

    def __init__(self, port, write, read):
        # for windows write and read are methods from Serial object, for linux - os.read/write it doesnt work otherwise
        threading.Thread.__init__(self)
        self.port = port
        self.writer_process = None
        self.write = write
        self.read = read
        self.need_stop = False

    def run(self):
        # FREEEEG32_EMULATOR_EXTENDED=1 sends 134-byte frames with board ID, PPG and motion data,
        # FREEEEG32_EMULATOR_EXTENDED=121 the 121-byte frames of firmware before the motion sensor
        mode = os.environ.get('FREEEEG32_EMULATOR_EXTENDED', '0')
        extended = mode in ('1', '121', '134')
        motion = mode in ('1', '134')
        self.writer_process = FreeEEG32Writer(self.port, 0.005, self.write, extended, motion)
        self.writer_process.daemon = True
        self.writer_process.start()
        time.sleep(10)
        self.writer_process.need_data = False
        self.writer_process.join()


class FreeEEG32Writer(threading.Thread):
    # FreeEEG32 firmware frames: 0xA0, sequence number, 32 x 24-bit EEG, 6 aux bytes, 0xC0 (105 bytes).
    # Extended frames put the board ID in the 6 aux bytes and add 16 bytes before the 0xC0:
    # PPG red/IR/green (24-bit), PPG counter, sync pulse counter, ping ID, pulse offset, flags
    # (121 bytes). Current firmware adds accel X/Y/Z and gyro X/Y/Z (signed 16-bit) and a motion
    # counter (134 bytes).
    board_id = bytes([0x31, 0x5A, 0x34, 0x79, 0x33, 0x32])

    def __init__(self, port, delay, write, extended=False, motion=False):
        threading.Thread.__init__(self)
        self.port = port
        self.write = write
        self.delay = delay
        self.extended = extended
        self.motion = extended and motion
        self.package_size = (134 if self.motion else 121) if extended else 105
        self.package_num = 0
        self.frames = 0
        self.need_data = True

    def run(self):
        while self.need_data:
            if self.package_num % 256 == 0:
                self.package_num = 0

            package = list()
            package.append(0xA0)
            package.append(self.package_num)
            if self.extended:
                for i in range(2, 98):
                    package.append(i)
                package.extend(self.board_id)
                # the PPG node samples at 200 Hz, the EEG at 512 Hz: a new sample every ~2.5 frames
                sample = self.frames * 200 // 512
                for base in (100000, 110000, 8000):
                    value = base + int(1000 * math.sin(2 * math.pi * 1.2 * sample / 200.0))
                    package.extend([(value >> 16) & 0xFF, (value >> 8) & 0xFF, value & 0xFF])
                package.append(sample % 256)  # PPG counter
                package.append(0)  # sync pulse counter
                # ping ID 0xC0A0: a 0xC0 0xA0 pair inside a frame, which the parser must not take for a frame end
                package.extend([0xC0, 0xA0])
                package.extend([0, 0])  # pulse offset
                package.append(0)  # flags
                if self.motion:
                    # lying flat: 1 g on Z (0.122 mg/count), a slow wobble on X; gyro at
                    # 17.5 mdps/count, with 0xC0 0xA0 inside gyro Y
                    sample = self.frames * 104 // 512
                    ax = int(400 * math.sin(2 * math.pi * 0.5 * sample / 104.0))
                    for v in (ax, -50, 8197, 100, -16224, -3):
                        package.extend(list((v & 0xFFFF).to_bytes(2, 'big')))
                    package.append(sample % 256)  # motion counter
            else:
                for i in range(2, self.package_size - 1):
                    package.append(i)
            package.append(0xC0)
            logging.info(bytes(package))
            self.write(self.port, bytes(package))

            self.package_num = self.package_num + 1
            self.frames = self.frames + 1
            time.sleep(self.delay)
