import smbus2

I2C_ADDR = 0x38  # Address of the detected device
REG_CHIP_ID = 0x00  # Common register to identify devices (may vary)

try:
    bus = smbus2.SMBus(1)
    chip_id = bus.read_byte_data(I2C_ADDR, REG_CHIP_ID)
    print(f"Device at 0x{I2C_ADDR:02X} responded with CHIP ID: 0x{chip_id:02X}")
    bus.close()
except Exception as e:
    print(f"Error: {e}")

