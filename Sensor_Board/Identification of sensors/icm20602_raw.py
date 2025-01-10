import smbus2
import time

# I2C address of the ICM-20602
ICM20602_ADDR = 0x68

# Registers
WHO_AM_I = 0x10
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# Scale factors (adjust based on your configuration)
ACCEL_SCALE = 16384.0  # For ±2g
GYRO_SCALE = 131.0     # For ±250°/s

def read_raw_data(bus, addr, reg):
    # Read 2 bytes of raw data from the sensor
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    # Convert to signed value
    if value > 32767:
        value -= 65536
    return value

def initialize_icm20602(bus, addr):
    # Wake up the sensor (exit sleep mode)
    bus.write_byte_data(addr, PWR_MGMT_1, 0x00)
    time.sleep(0.1)
    
    # Verify communication
    who_am_i = bus.read_byte_data(addr, WHO_AM_I)
    if who_am_i == 0x12:  # Expected value for ICM-20602
        print("ICM-20602 connected successfully!")
    else:
        raise Exception(f"Device not recognized! WHO_AM_I: 0x{who_am_i:02X}")

def read_accel_gyro(bus, addr):
    # Read raw accelerometer and gyroscope data
    accel_x = read_raw_data(bus, addr, ACCEL_XOUT_H) / ACCEL_SCALE
    accel_y = read_raw_data(bus, addr, ACCEL_XOUT_H + 2) / ACCEL_SCALE
    accel_z = read_raw_data(bus, addr, ACCEL_XOUT_H + 4) / ACCEL_SCALE

    gyro_x = read_raw_data(bus, addr, GYRO_XOUT_H) / GYRO_SCALE
    gyro_y = read_raw_data(bus, addr, GYRO_XOUT_H + 2) / GYRO_SCALE
    gyro_z = read_raw_data(bus, addr, GYRO_XOUT_H + 4) / GYRO_SCALE

    return {
        "accel": {"x": accel_x, "y": accel_y, "z": accel_z},
        "gyro": {"x": gyro_x, "y": gyro_y, "z": gyro_z},
    }

def main():
    try:
        # Initialize I2C bus
        bus = smbus2.SMBus(1)

        # Initialize the sensor
        initialize_icm20602(bus, ICM20602_ADDR)

        # Continuously read sensor data
        while True:
            sensor_data = read_accel_gyro(bus, ICM20602_ADDR)
            print("Accelerometer (g):", sensor_data["accel"])
            print("Gyroscope (°/s):", sensor_data["gyro"])
            time.sleep(0.5)

    except Exception as e:
        print("Error:", e)

if __name__ == "__main__":
    main()

