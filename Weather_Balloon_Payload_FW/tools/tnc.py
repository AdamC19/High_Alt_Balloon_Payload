import serial

def return_as_binary(character):
    retval = ""
    for i in range(0, 8):
        retval += "{:d}".format((character >> i) & 1)
    return retval

if __name__ == "__main__":

    comport = "COM5"

    ser = serial.Serial(comport)
    ser.baudrate = 115200
    ser.timeout = 0.25

    done = False

    line = ""
    line_ascii = ""
    frame_sync = 0
    payload = ""
    successes = 0
    packets = 0
    while not done:
        # try:
        if len(line_ascii) >= 80:
            # print(line)
            print(line_ascii)
            line_ascii = ""
        if len(line) >= 80:
            print(line)
            line = ""

        b = ser.read(1)
        if len(b) > 0:
            # line += "{} ".format(return_as_binary(b[0]))
            if b[0] == 0x7E and frame_sync == 0:
                # analyze success rate
                if len(payload) > 0:
                    packets += 1
                    if "KE8ITF" in payload:
                        successes += 1
                    if packets % 10 == 0:
                        print("\r\nSuccess Rate: {:.1f}".format(100*successes/packets))
                    payload = ""
                
                print("") # print a newline
                frame_sync = 1
            elif b[0] < 126 and b[0] > 31:
                # line += "{} ".format(return_as_binary(b[0]))
                symbol = chr(b[0])
                payload += symbol
                print(symbol, end='')
                # line_ascii += "{:s}".format(chr(b[0]))
                frame_sync = 2
            else:
                if frame_sync > 1:
                    print('~', end="")
        else:
            frame_sync = 0

        # elif len(line_ascii) > 0:
        #     print(line_ascii, end='')
        #     line_ascii = ""

        # else:
        #     pass
            # print("Timed out")

        # except KeyboardInterrupt:
        #     done = True
    
    ser.close()