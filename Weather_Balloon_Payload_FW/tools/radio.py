import serial
import struct
import time
import sys

MODE = 0
RADIO_STATE = 1
PA_EN = 2
LO_GEN_EN = 3
LO_PLL_R_COUNTER_H = 4
LO_PLL_R_COUNTER_L = 5
LO_PLL_N_COUNTER_H = 6
LO_PLL_N_COUNTER_L = 7
LO_PLL_LOCK = 8
CALLSIGN_0 = 9
CALLSIGN_1 = 10
CALLSIGN_2 = 11
CALLSIGN_3 = 12
CALLSIGN_4 = 13
CALLSIGN_5 = 14
TX_PERIOD_H = 15
TX_PERIOD_L = 16
LOG_INTERVAL = 17
SD_CARD_OK = 18
GPS_PPS_OK = 19
DEBUG_MODE = 20
NUM_FIELDS = 21

CMD_NONE = 0x00
CMD_READ = 0x01
CMD_READ_BLOCK = 0x02
CMD_WRITE = 0x03
CMD_WRITE_BLOCK = 0x04
CMD_COMMS_TEST = 0x05

START_BYTE = 0xA5
IND_CMD = 1
LEN_IND = (IND_CMD + 1)
BODY_IND = (LEN_IND + 1)

REF_FREQ    = 10.0e6
R_DIV       = 200
PFD_FREQ    = REF_FREQ/R_DIV

class Radio:
    def __init__(self, comport):
        self.ser = serial.Serial(comport)
        self.ser.timeout = 1.5
        self.ser.baudrate = 115200
    
    def test_comms(self):
        retval = False
        # START_BYTE | CMD | BODY_LEN
        arr = bytearray(BODY_IND + 4)
        payload = 0xEFBEADDE
        struct.pack_into("<BBBI", arr, 0, START_BYTE, CMD_COMMS_TEST, 4, payload)
        print("Sending packet; {}".format(arr))
        self.ser.write(arr)
        try:
            hddr = self.ser.read(BODY_IND)
            body = self.ser.read(hddr[LEN_IND])
            rpy_payload = struct.unpack_from("<I", body, 0)[0]
            print("Received: {}".format(body))
            retval = (hddr[0] == START_BYTE and hddr[1] == CMD_COMMS_TEST and rpy_payload == payload)
        except serial.SerialTimeoutException:
            print("Comms test timed out.")
        except IndexError:
            print("No response to comms received.")
        
        return retval
    
    def set_field(self, field, value):
        # START_BYTE | CMD | BODY_LEN
        arr = bytearray(BODY_IND + 2)
        struct.pack_into("<BBBBB", arr, 0, START_BYTE, CMD_WRITE, 2, field, value)
        self.ser.write(arr)
        try:
            hddr = self.ser.read(BODY_IND)
            body = self.ser.read(hddr[LEN_IND])
            print("Received: {}{}".format(hddr, body))
        except serial.SerialTimeoutException:
            print("Comms test timed out.")
        except IndexError:
            print("No response to comms received.")
    
    def set_N_div(self, N_div):
        # START_BYTE | CMD | BODY_LEN ...
        N_div_H = (N_div >> 8) & 0xFF
        N_div_L = N_div & 0xFF
        arr = bytearray(BODY_IND + 3)
        arr[0] = START_BYTE
        arr[1] = CMD_WRITE_BLOCK
        arr[2] = 3
        arr[BODY_IND] = LO_PLL_N_COUNTER_H
        arr[BODY_IND + 1] = N_div_H
        arr[BODY_IND + 2] = N_div_L
        self.ser.write(arr)
        try:
            hddr = self.ser.read(BODY_IND)
            body = self.ser.read(hddr[LEN_IND])
            print("Received: {}{}".format(hddr, body))
        except serial.SerialTimeoutException:
            print("Comms test timed out.")
        except IndexError:
            print("No response to comms received.")
#------------------------------------------------------------------------------
if __name__ == "__main__":
    comport = input("COMPORT: ")
    if len(comport) < 0:
        comport = "COM1"
    
    radio = Radio(comport)

    # if radio.test_comms():
    #     print("Comms test succeeded.")
    
    done = False
    while not done:
        cmd = input(">>> ")

        cmds = cmd.split(' ')

        if cmds[0].upper() == 'H':
            print("COMMANDS:")
            print("\th\t- display this menu.")
            print("\tq\t- quit.")
            print("\tm\t- set or query mode.")
            print("\tr\t- set RX audio gain.")
            print("\tt\t- set TX audio gain.")
            print("\tf\t- set LO frequency.")

        elif cmds[0].upper() == 'Q':
            done = True

        elif cmds[0].upper() == 'M':
            if 'R' in cmds[1].upper():
                print("Setting radio mode to RX...")
                radio.set_field(RADIO_MODE, 0)

            elif 'T' in cmds[1].upper():
                print("Setting radio mode to TX...")
                radio.set_field(RADIO_MODE, 1)

            else:
                print(f"Argument {cmds[1]} not understood.")

        elif cmds[0].upper() == 'T':
            try:
                gain = int(cmds[1])
                if gain > 255:
                    gain = 255
                elif gain < 0:
                    gain = 0
                print(f"Setting TX audio gain to {gain}...")
                radio.set_field(TX_AUDIO_GAIN, gain)
            except ValueError:
                print(f"Could not convert {cmds[1]} to an integer.")
                
        elif cmds[0].upper() == 'R':
            try:
                gain = int(cmds[1])
                if gain > 255:
                    gain = 255
                elif gain < 0:
                    gain = 0
                print(f"Setting RX audio gain to {gain}...")
                radio.set_field(RX_AUDIO_GAIN, gain)
            except ValueError:
                print(f"Could not convert {cmds[1]} to an integer.")
                
        elif cmds[0].upper() == 'F':
            try:
                freq = float(cmds[1]) * 1.0e6
                N_div = int(freq/PFD_FREQ + 0.5)
                radio.set_N_div(N_div)

            except ValueError:
                print(f"Could not convert {cmds[1]} to an integer.")

