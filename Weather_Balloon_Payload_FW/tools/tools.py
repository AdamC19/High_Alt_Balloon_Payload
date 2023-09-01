import numpy as np

def within_plus_minus(value, target, tolerance):
    return (value < target + tolerance) and (value > target - tolerance)


if __name__ == '__main__':
    HIGH_FREQ = 4.608e9

    MARK_FREQ = 1200
    MARK_TOLERANCE = 300
    SPACE_FREQ = 2200
    
    MARK_SPACE_JUMP = SPACE_FREQ - MARK_FREQ
    MARK_SPACE_TOLERANCE = 300

    LO_PFD_STEP = 100.0e3

    MAX_BASEBAND_FREQ = 5.0e6 + MARK_FREQ
    MIN_BASEBAND_FREQ = 490.0e3 + MARK_FREQ

    CARRIER_FREQ = 144.39e6

    start_div = np.floor((HIGH_FREQ/MIN_BASEBAND_FREQ) + 0.5)

    steps = np.arange(MIN_BASEBAND_FREQ, MAX_BASEBAND_FREQ, LO_PFD_STEP)

    print("{:10s} {:10s} {:10s} {:10s} {:10s} {:10s}".format("DIVISION", "BASE FREQ.", "MARK FREQ.", "SPACE FREQ.", "JUMP", "LO FREQ."))
    for freq in steps:
        div = np.floor((HIGH_FREQ/freq) + 0.5)
        mark_freq = HIGH_FREQ/div
        ideal_space_freq = mark_freq + MARK_SPACE_JUMP

        if within_plus_minus(mark_freq, freq, MARK_TOLERANCE):
            space_div_diff = 1.0
            space_freq_1 = HIGH_FREQ/(div - space_div_diff)
            space_freq_2 = HIGH_FREQ/(div - (space_div_diff + 1))
            space_freq = 0.0
            while space_freq_1 < ideal_space_freq - MARK_SPACE_TOLERANCE:
                space_div_diff += 1.0
                space_freq_1 = HIGH_FREQ/(div - space_div_diff)
                space_freq_2 = HIGH_FREQ/(div - (space_div_diff + 1))
            if within_plus_minus(space_freq_1, ideal_space_freq, MARK_SPACE_TOLERANCE):
                space_freq = space_freq_1
            if within_plus_minus(space_freq_2, ideal_space_freq, MARK_SPACE_TOLERANCE):
                space_freq = space_freq_2
            if space_freq != 0.0:
                # GOT ONE
                carrier_freq = freq - MARK_FREQ
                lo_freq = CARRIER_FREQ - carrier_freq
                print("{:10d} {:10.1f} {:10.1f} {:10.1f} {:10.1f} {:10.1f}".format(int(div), carrier_freq, mark_freq - carrier_freq, space_freq - carrier_freq, space_freq - mark_freq, lo_freq))

