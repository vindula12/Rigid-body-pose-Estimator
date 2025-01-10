import smbus2

I2C_ADDR = 0x38

def read_register(bus, address, reg):
    try:
        return bus.read_byte_data(address, reg)
    except Exception as e:
        return None

try:
    bus = smbus2.SMBus(1)
    print(f"Scanning device at 0x{I2C_ADDR:02X}")
    for reg in range(0x00, 0x10):  # Adjust range as needed
        value = read_register(bus, I2C_ADDR, reg)
        if value is not None:
            print(f"Register 0x{reg:02X}: 0x{value:02X}")
    bus.close()
except Exception as e:
    print(f"Error: {e}")


