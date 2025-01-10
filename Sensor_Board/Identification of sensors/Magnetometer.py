import smbus2
import time

# QMC5883L I2C address and register mappings
QMC5883L_ADDR = 0x0D  # Default I2C address
MODE_REG = 0x09       # Mode register
X_DATA_REG = 0x00     # X-axis data register
Y_DATA_REG = 0x02     # Y-axis data register
Z_DATA_REG = 0x04     # Z-axis data register
STATUS_REG = 0x06     # Status register

# Constants for configuring the sensor
MODE_CONTINUOUS = 0x01  # Continuous measurement mode
MODE_IDLE = 0x00        # Idle mode (power-down)

def read_word(bus, address, reg):
    """Read a 16-bit word from a given register."""
    low = bus.read_byte_data(address, reg)
    high = bus.read_byte_data(address, reg + 1)
    return (high << 8) + low

def set_mode(bus, mode):
    """Set the QMC5883L operation mode (continuous or idle)."""
    bus.write_byte_data(QMC5883L_ADDR, MODE_REG, mode)

def read_data(bus):
    """Read the magnetometer data (X, Y, Z)."""
    x = read_word(bus, QMC5883L_ADDR, X_DATA_REG)
    y = read_word(bus, QMC5883L_ADDR, Y_DATA_REG)
    z = read_word(bus, QMC5883L_ADDR, Z_DATA_REG)
    
    # Handle negative values (two's complement)
    if x >= 0x8000:
        x -= 0x10000
    if y >= 0x8000:
        y -= 0x10000
    if z >= 0x8000:
        z -= 0x10000
    
    return x, y, z

def main():
    bus = smbus2.SMBus(1)  # Initialize the I2C bus
    set_mode(bus, MODE_CONTINUOUS)  # Set sensor to continuous mode

    try:
        while True:
            x, y, z = read_data(bus)
            print(f"Magnetometer Data -> X: {x}, Y: {y}, Z: {z}")
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting program.")
        bus.close()

if __name__ == "__main__":
    main()
