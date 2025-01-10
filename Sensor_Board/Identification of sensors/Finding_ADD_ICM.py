# import smbus2

# def i2c_scan():
#     bus = smbus2.SMBus(1)
#     print("Scanning I2C bus for devices...")
#     for addr in range(0x03, 0x78):
#         try:
#             bus.read_byte(addr)
#             print(f"Device found at 0x{addr:02X}")
#         except Exception:
#             pass
#     bus.close()

# i2c_scan()

# import smbus2

# ICM20602_ADDR = 0x38  # Change to 0x69 if needed
# WHO_AM_I = 0x75

# try:
#     bus = smbus2.SMBus(1)
#     who_am_i = bus.read_byte_data(ICM20602_ADDR, WHO_AM_I)
#     print(f"WHO_AM_I: 0x{who_am_i:02X}")
#     bus.close()
# except Exception as e:
#     print(f"Error: {e}")

# import smbus2

# I2C_ADDR = 0x38  # Update based on your setup

# def read_register(bus, address, reg):
#     try:
#         return bus.read_byte_data(address, reg)
#     except Exception:
#         return None

# def scan_device(bus, address):
#     print(f"Scanning device at 0x{address:02X}")
#     for reg in range(0x00, 0x7F):  # Adjust range as needed
#         value = read_register(bus, address, reg)
#         if value is not None:
#             print(f"Register 0x{reg:02X}: 0x{value:02X}")

# try:
#     bus = smbus2.SMBus(1)
#     scan_device(bus, I2C_ADDR)
#     bus.close()
# except Exception as e:
#     print(f"Error: {e}")

# import smbus2
# # Attempt to interpret the data from likely registers
# I2C_ADDR = 0x38

# def read_sensor(bus, addr):
#     try:
#         # Try reading a few registers and interpreting values
#         reg_data = []
#         for reg in [0x05, 0x06, 0x28, 0x2F]:
#             reg_data.append(bus.read_byte_data(addr, reg))
#         print(f"Register data: {reg_data}")
#     except Exception as e:
#         print(f"Error: {e}")

# bus = smbus2.SMBus(1)
# read_sensor(bus, I2C_ADDR)
# bus.close()

import smbus2

def dump_registers(bus, address):
    print("Register dump:")
    for reg in range(0x00, 0x7F):
        try:
            value = bus.read_byte_data(address, reg)
            print(f"Register 0x{reg:02X}: 0x{value:02X}")
        except Exception as e:
            print(f"Error reading register 0x{reg:02X}: {e}")

def main():
    bus = smbus2.SMBus(1)
    address = 0x38

    print(f"Scanning device at 0x{address:02X}")
    dump_registers(bus, address)

    bus.close()

if __name__ == "__main__":
    main()






