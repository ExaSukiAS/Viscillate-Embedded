import serial
import struct
import time

ser = serial.Serial('COM12', 2000000) 

PRINT_INTERVAL = 1  # 300ms interval

last_print_time = time.time()

# Variables to keep track of the data between prints
sum_mV = 0
sample_count = 0

while True:
    # 1. Look for the sync header (0xAA, 0xBB)
    if ser.read(1) == b'\xAA':
        if ser.read(1) == b'\xBB':
        
            payload = ser.read(4)
            
            if len(payload) == 4:
                # Unpack the 4-byte integer
                mV = struct.unpack('<i', payload)[0]
                
                # 2. Add the current reading to our running total
                sum_mV += mV
                sample_count += 1
                
                # 3. Check if 300ms has passed
                current_time = time.time()
                if current_time - last_print_time >= PRINT_INTERVAL:
                    
                    # Prevent division by zero just in case
                    if sample_count > 0:
                        # Calculate the average
                        average_mV = sum_mV / sample_count
                        avg_V = (0.9684*average_mV - 421.4)/1000
                        
                        # Print the average (formatted to 2 decimal places)
                        print(f"Average Voltage: {avg_V:.2f} V  (averaged from {sample_count} readings)")
                    
                    # 4. Reset the counters and timer for the next 300ms window
                    sum_mV = 0
                    sample_count = 0
                    last_print_time = current_time