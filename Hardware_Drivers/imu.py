from .hardware_setup import i2c
import adafruit_lis3mdl
from  adafruit_lsm6ds.ls6mdsox import LSM6DSOX
import traceback 
def init_imu_and_mag():
            imu = None
            mag = None
            try:
                imu = LSM6DSOX(i2c, address=0x6B) # adafruit_lsm6ds.lsm6dsox.LSM6DSOX(i2c, address=0x6B) # new
                print("✅ LSM6DSOX IMU initialized at 0x6B")
            except Exception as e:
                print(f"❌ IMU init failed: {e}")
                imu = None
                traceback.print_exc()
            try:
                mag = adafruit_lis3mdl.LIS3MDL(i2c, address=0x1E)
                print("✅ LIS3MDL magnetometer initialized at 0x1E")
                traceback.print_exc()
            except Exception as e:
                print(f"❌ Magnetometer init failed: {e}")
                mag = None
            return imu, mag
# initialize at module load
imu, mag = init_imu_and_mag()
