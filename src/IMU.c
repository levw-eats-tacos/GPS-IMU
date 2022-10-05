#include <stdint.h>
#include <linux/i2c-dev.h>
#include "LSM9DS0.h"
#include "LSM9DS1.h"
#include "LSM6DSL.h"
#include "LIS3MDL.h"

int file;
int BerryIMUversion = 99;

void  readBlock(uint8_t command, uint8_t size, uint8_t *data)
{
    int result = i2c_smbus_read_i2c_block_data(file, command, size, data);
    if (result != size){
		printf("Failed to read block from I2C.");
		exit(1);
	}
}

void selectDevice(int file, int addr)
{
	if (ioctl(file, I2C_SLAVE, addr) < 0) {
		 printf("Failed to select I2C device.");
	}
}


void readACC(int  a[])
{
	uint8_t block[6];
	if (BerryIMUversion == 1){
		selectDevice(file,LSM9DS0_ACC_ADDRESS);
		readBlock(0x80 |  LSM9DS0_OUT_X_L_A, sizeof(block), block);
	}
	else if (BerryIMUversion == 2){
		selectDevice(file,LSM9DS1_ACC_ADDRESS);
		readBlock(LSM9DS1_OUT_X_L_XL, sizeof(block), block);       
	}
	else if (BerryIMUversion == 3){
		selectDevice(file,LSM6DSL_ADDRESS);
		readBlock(LSM6DSL_OUTX_L_XL, sizeof(block), block);    
	}
	// Combine readings for each axis.
	a[0] = (int16_t)(block[0] | block[1] << 8);
	a[1] = (int16_t)(block[2] | block[3] << 8);
	a[2] = (int16_t)(block[4] | block[5] << 8);
}


void readMAG(int  m[])
{
	uint8_t block[6];
    if (BerryIMUversion == 1){
		selectDevice(file,LSM9DS0_MAG_ADDRESS);
		readBlock(0x80 |  LSM9DS0_OUT_X_L_M, sizeof(block), block);
	}
	else if (BerryIMUversion == 2){
		selectDevice(file,LSM9DS1_MAG_ADDRESS);
		readBlock(LSM9DS1_OUT_X_L_M, sizeof(block), block);    
	}
	else if (BerryIMUversion == 3){
		selectDevice(file,LIS3MDL_ADDRESS);
		readBlock(LIS3MDL_OUT_X_L, sizeof(block), block);    
	}

	// Combine readings for each axis.
	m[0] = (int16_t)(block[0] | block[1] << 8);
	m[1] = (int16_t)(block[2] | block[3] << 8);
	m[2] = (int16_t)(block[4] | block[5] << 8);

}

void readGYR(int g[])
{
	uint8_t block[6];
    if (BerryIMUversion == 1){
		selectDevice(file,LSM9DS0_GYR_ADDRESS);
		readBlock(0x80 |  LSM9DS0_OUT_X_L_G, sizeof(block), block);
	}
	else if (BerryIMUversion == 2){
		selectDevice(file,LSM9DS1_GYR_ADDRESS);
		readBlock(LSM9DS1_OUT_X_L_G, sizeof(block), block);    
	}
	else if (BerryIMUversion == 3){
		selectDevice(file,LSM6DSL_ADDRESS);
		readBlock(LSM6DSL_OUTX_L_G, sizeof(block), block);   
	}

	// Combine readings for each axis.
	g[0] = (int16_t)(block[0] | block[1] << 8);
	g[1] = (int16_t)(block[2] | block[3] << 8);
	g[2] = (int16_t)(block[4] | block[5] << 8);
}


void writeAccReg(uint8_t reg, uint8_t value)
{
	if (BerryIMUversion == 1)
		selectDevice(file,LSM9DS0_ACC_ADDRESS);
	else if (BerryIMUversion == 2)
		selectDevice(file,LSM9DS1_ACC_ADDRESS);
	else if (BerryIMUversion == 3)
		selectDevice(file,LSM6DSL_ADDRESS);

	int result = i2c_smbus_write_byte_data(file, reg, value);
	if (result == -1){
		printf ("Failed to write byte to I2C Acc.");
        exit(1);
    }
}

void writeMagReg(uint8_t reg, uint8_t value)
{
	if (BerryIMUversion == 1)
		selectDevice(file,LSM9DS0_MAG_ADDRESS);
	else if (BerryIMUversion == 2)
		selectDevice(file,LSM9DS1_MAG_ADDRESS);
	else if (BerryIMUversion == 3)
		selectDevice(file,LIS3MDL_ADDRESS);

	int result = i2c_smbus_write_byte_data(file, reg, value);
	if (result == -1){
		printf("Failed to write byte to I2C Mag.");
		exit(1);
	}
}


void writeGyrReg(uint8_t reg, uint8_t value)
{
	if (BerryIMUversion == 1)
		selectDevice(file,LSM9DS0_GYR_ADDRESS);
	else if (BerryIMUversion == 2)
		selectDevice(file,LSM9DS1_GYR_ADDRESS);
	else if (BerryIMUversion == 3)
		selectDevice(file,LSM6DSL_ADDRESS);

	int result = i2c_smbus_write_byte_data(file, reg, value);
	if (result == -1){
		printf("Failed to write byte to I2C Gyr.");
		exit(1);
	}
}