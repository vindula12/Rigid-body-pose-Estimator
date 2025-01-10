import time
from smbus2 import SMBus

# BMP388 default I2C address
BMP388_I2C_ADDR = 0x38

# BMP388 register addresses
REG_CHIP_ID = 0x00
REG_PRESS_MSB = 0x04
REG_PRESS_LSB = 0x05
REG_PRESS_XLSB = 0x06
REG_TEMP_MSB = 0x07
REG_TEMP_LSB = 0x08
REG_TEMP_XLSB = 0x09

def read_sensor(bus, address, reg, length):
    """Reads multiple bytes from the sensor."""
    return bus.read_i2c_block_data(address, reg, length)

def compensate_pressure(raw_pressure):
    """Dummy function to process raw pressure readings."""
    # Add real compensation logic from the BMP388 datasheet
    return raw_pressure / 256.0  # Example conversion

def compensate_temperature(raw_temperature):
    """Dummy function to process raw temperature readings."""
    # Add real compensation logic from the BMP388 datasheet
    return raw_temperature / 256.0  # Example conversion

def main():
    bus = SMBus(1)  # Use I2C bus 1

    try:
        # Read the chip ID to confirm communication
        chip_id = bus.read_byte_data(BMP388_I2C_ADDR, REG_CHIP_ID)
        print(f"BMP388 Chip ID: {chip_id:X}")

        while True:
            # Read raw pressure and temperature data
            raw_pressure = read_sensor(bus, BMP388_I2C_ADDR, REG_PRESS_MSB, 3)
            raw_temperature = read_sensor(bus, BMP388_I2C_ADDR, REG_TEMP_MSB, 3)

            # Convert raw data to integers
            pressure = (raw_pressure[0] << 16) | (raw_pressure[1] << 8) | raw_pressure[2]
            temperature = (raw_temperature[0] << 16) | (raw_temperature[1] << 8) | raw_temperature[2]

            # Compensate readings
            pressure = compensate_pressure(pressure)
            temperature = compensate_temperature(temperature)

            # Print readings
            print(f"Pressure: {pressure:.2f} Pa, Temperature: {temperature:.2f} °C")

            # Wait before next reading
            time.sleep(1)

    except Exception as e:
        print(f"Error: {e}")

    finally:
        bus.close()

if __name__ == "__main__":
    main()

# import smbus2
# from time import sleep

# def i2c_scan(bus_number=1):
#     bus = smbus2.SMBus(bus_number)
#     print("Scanning for I2C devices...")
#     for address in range(0x03, 0x78):
#         try:
#             bus.read_byte(address)
#             print(f"Device found at 0x{address:02X}")
#         except OSError:
#             pass
#     bus.close()

# i2c_scan()
