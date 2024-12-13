import time
import struct
import pyb

IMU_Device = ["MAG", "ACCEL", "GYRO", "SYS"]

# BNO055 I2C address (default is 0x28)
BNO055_ADDRESS = 0x28

# Register addresses for BNO055
BNO055_CHIP_ID_ADDR = 0x00
BNO055_OPR_MODE_ADDR = 0x3D
BNO055_SYS_STATUS_ADDR = 0x39
BNO055_EULER_H_ADDR = 0x1A
BNO055_EULER_R_ADDR = 0x1E
BNO055_EULER_GYRO_ADDR = 0X18
BNO055_TEMP_ADDR = 0x34
BNO055_CALIBRATION_STATUS = 0X35
BNO055_MAG_RADIUS_LSB = 0x69
BNO055_MAG_RADIUS_MSB = 0x6A
BNO055_ACCEL_RADIUS_LSB = 0x67
BNO055_ACCEL_RADIUS_MSB = 0x68
BNO055_GYRO_OFFSET_X_LSB = 0x61
BNO055_GYRO_OFFSET_X_MSB = 0x62
BNO055_GYRO_OFFSET_Y_LSB = 0x63
BNO055_GYRO_OFFSET_Y_MSB = 0x64
BNO055_GYRO_OFFSET_Z_LSB = 0x65
BNO055_GYRO_OFFSET_Z_MSB = 0x66
BNO055_MAG_OFFSET_X_LSB = 0x5B
BNO055_MAG_OFFSET_X_MSB = 0x5C
BNO055_MAG_OFFSET_Y_LSB = 0x5D
BNO055_MAG_OFFSET_Y_MSB = 0x5E
BNO055_MAG_OFFSET_Z_LSB = 0x5F
BNO055_MAG_OFFSET_Z_MSB = 0x60
BNO055_ACCEL_OFFSET_X_LSB = 0x55
BNO055_ACCEL_OFFSET_X_MSB = 0x56
BNO055_ACCEL_OFFSET_Y_LSB = 0x57
BNO055_ACCEL_OFFSET_Y_MSB = 0x58
BNO055_ACCEL_OFFSET_Z_LSB = 0x59
BNO055_ACCEL_OFFSET_Z_MSB = 0x5A

# Operation mode values (config, normal, etc.)
BNO055_OPR_MODE_CONFIG = 0x00
BNO055_OPR_MODE_NDOF = 0x0C

class BNO055:
    def __init__(self, i2c_controller, address=BNO055_ADDRESS):
        # Use the pyb I2C object to interact with the BNO055
        self.i2c = i2c_controller
        self.address = address
        self.set_operation_mode(BNO055_OPR_MODE_NDOF)
        time.sleep(0.3)  # Allow time for sensor to initializef
        print("IMU init")
        pass
    
    def try_calibration(self):
        print("IMU calibration")
        """ Attempt to calibrate by reading from calibration file. """
        # Attempt to load calibration data from file
        try:
            with open('calibration.txt', 'r') as f:
                calib_data = f.read()
                calib_values = [int(x) for x in calib_data.strip().split(',')]
                self.mag_offset = tuple(calib_values[0:3])
                self.accel_offset = tuple(calib_values[3:6])
                self.gyro_offset = tuple(calib_values[6:9])
                self.mag_radius = calib_values[9]
                self.accel_radius = calib_values[10]
                # Write these coefficients to the sensor
                self.write_calibration_coefficients()
                print("Calibration data loaded from file.")
                i = 1
        except OSError:
            # File does not exist, so perform calibration
            print("Calibration data not found. Performing calibration...")
            print("Please move the sensor in a figure 8 pattern until calibration is complete.")
            # Wait until the calibration status is fully calibrated
            i = self.read_calibration_status()
            if i:
                # Read the calibration coefficients
                self.read_calibration_coefficients()
                # Save the calibration coefficients to file
                with open('calibration.txt', 'w') as f:
                    calib_values = (
                        list(self.mag_offset)
                        + list(self.accel_offset)
                        + list(self.gyro_offset)
                        + [self.mag_radius, self.accel_radius]
                    )
                    f.write(','.join(map(str, calib_values)))
                    print("Calibration data saved to calibration.txt")
        return i

    def set_operation_mode(self, mode):
        """ Set the operation mode of the BNO055 sensor. """
        self.i2c.mem_write(mode, self.address, BNO055_OPR_MODE_ADDR)
        time.sleep(0.03)  # Wait for mode to be set
        pass

    def read_euler_angles(self):
        """ Read the Euler angles (heading, roll, pitch) from the sensor. """
        # Read 6 bytes (2 bytes per angle)
        data = bytearray([0 for n in range(6)])
        self.i2c.mem_read(data, BNO055_ADDRESS, BNO055_EULER_H_ADDR)
        heading, roll, pitch = struct.unpack('<hhh', data)
        # Convert to degrees (from 16-bit signed integer)
        heading /= 16
        roll /= 16
        pitch /= 16
        return heading

    def read_velocity(self):
        data = bytearray([0 for n in range(2)])
        self.i2c.mem_read(data, BNO055_ADDRESS, BNO055_EULER_GYRO_ADDR)
        GyroZ = struct.unpack('<h', data)
        GyroZ = GyroZ[0] / 16.384
        return GyroZ

    def read_calibration_status(self):
        data = self.i2c.mem_read(1, BNO055_ADDRESS, BNO055_CALIBRATION_STATUS)
        calibration_byte = data[0]
        sys_status = (calibration_byte >> 6) & 0x03
        gyro_status = (calibration_byte >> 4) & 0x03
        accel_status = (calibration_byte >> 2) & 0x03
        mag_status = calibration_byte & 0x03

        print("SYS:", sys_status, "GYRO:", gyro_status, "ACCEL:", accel_status, "MAG:", mag_status)

        fully_calibrated = (
            sys_status == 3 and gyro_status == 3 and accel_status == 3 and mag_status == 3
        )
        return fully_calibrated

    def read_calibration_coefficients(self):
        """ Read calibration coefficients for magnetometer, accelerometer, gyroscope, and radii. """
        data = bytearray([0 for n in range(6)])
        # Read magnetometer offsets (X, Y, Z)
        self.i2c.mem_read(data, BNO055_ADDRESS, BNO055_MAG_OFFSET_X_LSB)
        self.mag_offset = struct.unpack('<hhh', data)
        # Read accelerometer offsets (X, Y, Z)
        self.i2c.mem_read(data, BNO055_ADDRESS, BNO055_ACCEL_OFFSET_X_LSB)
        self.accel_offset = struct.unpack('<hhh', data)
        # Read gyroscope offsets (X, Y, Z)
        self.i2c.mem_read(data, BNO055_ADDRESS, BNO055_GYRO_OFFSET_X_LSB)
        self.gyro_offset = struct.unpack('<hhh', data)
        # Read magnetometer radius
        data = bytearray([0 for n in range(2)])
        self.i2c.mem_read(data, BNO055_ADDRESS, BNO055_MAG_RADIUS_LSB)
        self.mag_radius = struct.unpack('<H', data)[0]
        # Read accelerometer radius
        self.i2c.mem_read(data, BNO055_ADDRESS, BNO055_ACCEL_RADIUS_LSB)
        self.accel_radius = struct.unpack('<H', data)[0]
        # Print all calibration coefficients
        print("Magnetometer offsets:", self.mag_offset)
        print("Accelerometer offsets:", self.accel_offset)
        print("Gyroscope offsets:", self.gyro_offset)
        print("Magnetometer radius:", self.mag_radius)
        print("Accelerometer radius:", self.accel_radius)
        pass

    def write_calibration_coefficients(self):
        """ Write calibration coefficients for magnetometer, accelerometer, gyroscope, and radii. """
        self.set_operation_mode(BNO055_OPR_MODE_CONFIG)
        # Write magnetometer offsets (X, Y, Z)
        data = struct.pack('<hhh', *self.mag_offset)
        self.i2c.mem_write(data, self.address, BNO055_MAG_OFFSET_X_LSB)
        # Write accelerometer offsets (X, Y, Z)
        data = struct.pack('<hhh', *self.accel_offset)
        self.i2c.mem_write(data, self.address, BNO055_ACCEL_OFFSET_X_LSB)
        # Write gyroscope offsets (X, Y, Z)
        data = struct.pack('<hhh', *self.gyro_offset)
        self.i2c.mem_write(data, self.address, BNO055_GYRO_OFFSET_X_LSB)
        # Write magnetometer radius
        data = struct.pack('<H', self.mag_radius)
        self.i2c.mem_write(data, self.address, BNO055_MAG_RADIUS_LSB)
        # Write accelerometer radius
        data = struct.pack('<H', self.accel_radius)
        self.i2c.mem_write(data, self.address, BNO055_ACCEL_RADIUS_LSB)
        self.set_operation_mode(BNO055_OPR_MODE_NDOF)
        pass
