import pigpio
import subprocess
import time
import sys

prefix_pulse = 250;
long_pulse = 100
short_pulse = round(long_pulse/10)
inter_pulse = long_pulse
suffix_pulse = prefix_pulse
PIN = 26  # BCM GPIO 19 = Physical Pin 35

# 5x8 font subset for needed characters
font_5x8 = {
    'A': [0x7C, 0x12, 0x11, 0x12, 0x7C],
    'D': [0x7F, 0x41, 0x41, 0x22, 0x1C],
    'E': [0x7F, 0x49, 0x49, 0x49, 0x41],
    'H': [0x7F, 0x08, 0x08, 0x08, 0x7F],
    'I': [0x00, 0x41, 0x7F, 0x41, 0x00],
    'M': [0x7F, 0x02, 0x04, 0x02, 0x7F],
    'O': [0x3E, 0x41, 0x41, 0x41, 0x3E],
    'S': [0x46, 0x49, 0x49, 0x49, 0x31],
    'T': [0x01, 0x01, 0x7F, 0x01, 0x01],
    'a': [0x20, 0x54, 0x54, 0x54, 0x78],
    'd': [0x38, 0x44, 0x44, 0x44, 0x7F],
    'e': [0x38, 0x54, 0x54, 0x54, 0x18],
    'h': [0x7F, 0x08, 0x08, 0x08, 0x70],
    'i': [0x00, 0x00, 0x7A, 0x00, 0x00],
    'm': [0x7C, 0x04, 0x18, 0x04, 0x78],
    'o': [0x38, 0x44, 0x44, 0x44, 0x38],
    's': [0x48, 0x54, 0x54, 0x54, 0x24],
    ' ': [0x00, 0x00, 0x00, 0x00, 0x00],
}
def text_to_column_data(text, space=1):
    """Convert text to a flat list of column bytes (MSB top, LSB bottom)."""
    column_data = []
    for char in text:
        if char in font_5x8:
            column_data.extend(font_5x8[char])
        else:
            column_data.extend([0x00] * 5)  # unknown chars as blank
        column_data.extend([0x00] * space)
    return column_data

def get_display_rows(columns):
    """Convert 8 column bytes into 8 row bytes for 8x8 matrix."""
    rows = [0x00] * 8
    for col_index, col_byte in enumerate(columns):
        for row in range(8):
            if col_byte & (1 << (7 - row)):
                rows[row] |= (1 << (7 - col_index))
    return rows

def print_matrix(rows):
    """Prints 8x8 LED matrix to terminal."""
    for row in rows:
        print(''.join(['█' if row & (1 << (7 - col)) else ' ' for col in range(8)]))
    print()

def scroll_text(pi, pin, text, delay=0.1):
    data = text_to_column_data(text + " ")  # trailing space for spacing at end
    width = len(data)

    # Pad with 8 columns of zeros on the end to scroll fully off screen
    data.extend([0x00] * 8)

    for i in range(width + 8):
        window = data[i:i + 8]
        if len(window) < 8:
            window += [0x00] * (8 - len(window))  # pad if needed
        rows = get_display_rows(window)
        wave_id = create_encoded_wave(pi, pin, rows)
        if wave_id >= 0:
            pi.wave_send_once(wave_id)
        else:
            print("Failed to generate wave")
            pi.wave_tx_stop()
            pi.wave_delete(wave_id)
            pi.stop()
            sys.exit()
        time.sleep(delay)

def ensure_pigpiod_running():
    try:
        result = subprocess.run(['pgrep', 'pigpiod'], stdout=subprocess.DEVNULL)
        if result.returncode != 0:
            print("pigpiod not running. Starting it...")
            subprocess.run(['sudo', 'pigpiod'])
            time.sleep(0.5)
        else:
            print("pigpiod is already running.")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

def bytes_to_bits(byte_data):
    """Convert bytes to list of bits (MSB first)."""
    bits = []
    for byte in byte_data:
        for i in range(8):
            bits.append((byte >> i) & 1)
        for i in range(8): #Also send XOR for NEC encoding
            bits.append(1-(byte >> i) & 1)
    return bits

def create_encoded_wave(pi, pin, data_bytes):
    bits = bytes_to_bits(data_bytes)
    pulses = []
    pulses.append(pigpio.pulse(1 << pin, 0, prefix_pulse)) #Com start pulse
    pulses.append(pigpio.pulse(0, 1 << pin, inter_pulse))
    pulses.append(pigpio.pulse(1 << pin, 0, long_pulse)) #Com timing pulse
    pulses.append(pigpio.pulse(0, 1 << pin, inter_pulse))
    for bit in bits:
        pulse_length = short_pulse if bit == 0 else long_pulse  # µs
        # HIGH pulse for 10 or 40 µs
        pulses.append(pigpio.pulse(1 << pin, 0, pulse_length))
        # LOW period after each bit (e.g. 10 µs gap)
        pulses.append(pigpio.pulse(0, 1 << pin, inter_pulse))
    pulses.append(pigpio.pulse(0, 1 << pin, suffix_pulse))
    pi.wave_clear()
    pi.wave_add_generic(pulses)
    wave_id = pi.wave_create()
    return wave_id

def main():
    ensure_pigpiod_running()
    pi = pigpio.pi()
    if not pi.connected:
        print("Failed to connect to pigpiod.")
        sys.exit(1)

    pi.set_mode(PIN, pigpio.OUTPUT)

    print("Sending demo to pin 37... Press Ctrl+C to stop.")
    try:
        while True:
            # 👇 Replace with your actual data
            scroll_text(pi, PIN, "This is a demo", delay=0.04)
            
    except KeyboardInterrupt:
        print("\nStopping waveform...")
        pi.wave_tx_stop()
        pi.wave_delete(wave_id)
        pi.stop()

if __name__ == "__main__":
    main()
