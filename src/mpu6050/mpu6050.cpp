#include "mpu6050/mpu6050.h"


IMU::IMU() {
	

    sleep_ms(2000);

    // Initialize chosen serial port


    printf("Setting I2C init \n");
    //Initialize I2C port at 400 kHz
    i2c_init(i2c, 400 * 1000);


    printf("Setting I2C init gpio \n");
    // Initialize I2C pins
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    
    int status;

    soft_iron_scaling << 1.24506622, -0.09939941, -0.04428451,
                        -0.09939941,  1.20940279,  0.04576971,
                        -0.04428451,  0.04576971,  1.27723159;

    hard_iron_offset <<  1381.46265332, -3195.81090886,  538.74050546;

}



void IMU::getOffsets() {
	float gyro_off[3]; //Temporary storage
	float accel_off[3];

	gr_off = 0, gp_off = 0, gy_off = 0; //Initialize the offsets to zero
	ax_off = 0, ay_off = 0, az_off = 0; //Initialize the offsets to zero

	for (int i = 0; i < 10000; i++) { //Use loop to average offsets
		getGyroRaw(&gyro_off[0], &gyro_off[1], &gyro_off[2]); //Raw gyroscope values
		gr_off = gr_off + gyro_off[0], gp_off = gp_off + gyro_off[1], gy_off = gy_off + gyro_off[2]; //Add to sum

		getAccelRaw(&accel_off[0], &accel_off[1], &accel_off[2]); //Raw accelerometer values
		ax_off = ax_off + accel_off[0], ay_off = ay_off + accel_off[1], az_off = az_off + accel_off[2]; //Add to sum
	}

	gr_off = gr_off / 10000, gp_off = gp_off / 10000, gy_off = gy_off / 10000; //Divide by number of loops (to average)
	ax_off = ax_off / 10000, ay_off = ay_off / 10000, az_off = az_off / 10000;

	az_off = az_off - ACCEL_SENS; //Remove 1g from the value calculated to compensate for gravity)
}


void IMU::getGyroRaw(float *roll, float *pitch, float *yaw) {
	uint8_t temp_buffer[2];
    
    this->reg_read(i2c, REG_DEVID_MPU6050, 0x43, temp_buffer, 2); //Read X registers
    int16_t X = temp_buffer[0] << 8 | temp_buffer[1];    

    this->reg_read(i2c, REG_DEVID_MPU6050, 0x45, temp_buffer, 2); //Read Y registers
	int16_t Y = temp_buffer[0] << 8 | temp_buffer[1];
	
    this->reg_read(i2c, REG_DEVID_MPU6050, 0x47, temp_buffer, 2); //Read Z registers
	int16_t Z = temp_buffer[0] << 8 | temp_buffer[1];

	*roll = (float)X; //Roll on X axis
	*pitch = (float)Y; //Pitch on Y axis
	*yaw = (float)Z; //Yaw on Z axis
}

void IMU::getMagRaw(float *x, float *y, float *z)
{
    uint8_t temp_buffer[6];
    
    this->reg_read(i2c, REG_DEVID_QMC5883, 0x00, temp_buffer, 6); //Read X, Z, Y registers at once

    int16_t X = temp_buffer[1] << 8 | temp_buffer[0];    
	int16_t Y = temp_buffer[3] << 8 | temp_buffer[2];
	int16_t Z = temp_buffer[5] << 8 | temp_buffer[4];

	*x = (float)X;
	*y = (float)Y;
	*z = (float)Z;

    std::cout << *x << " " << *y << " " << *z << std::endl;
}

void IMU::getAccelRaw(float *x, float *y, float *z) {

	uint8_t temp_buffer[2];
    
    this->reg_read(i2c, REG_DEVID_MPU6050, 0x3b, temp_buffer, 2); //Read X registers
    int16_t X = temp_buffer[0] << 8 | temp_buffer[1];    

    this->reg_read(i2c, REG_DEVID_MPU6050, 0x3d, temp_buffer, 2); //Read Y registers
	int16_t Y = temp_buffer[0] << 8 | temp_buffer[1];
	
    this->reg_read(i2c, REG_DEVID_MPU6050, 0x3f, temp_buffer, 2); //Read Z registers
	int16_t Z = temp_buffer[0] << 8 | temp_buffer[1];

	*x = (float)X;
	*y = (float)Y;
	*z = (float)Z;
}


void IMU::getAccel(float *x, float *y, float *z) {
	getAccelRaw(x, y, z); //Store raw values into variables
	*x = round((*x - ax_off) * 1000.0 / ACCEL_SENS) / 1000.0; //Remove the offset and divide by the accelerometer sensetivity (use 1000 and round() to round the value to three decimal places)
	*y = round((*y - ay_off) * 1000.0 / ACCEL_SENS) / 1000.0;
	*z = round((*z - az_off) * 1000.0 / ACCEL_SENS) / 1000.0;
}


void IMU::getGyro(float *roll, float *pitch, float *yaw) {
	getGyroRaw(roll, pitch, yaw); //Store raw values into variables
	*roll = round((*roll - gr_off) * 1000.0 / GYRO_SENS) / 1000.0; //Remove the offset and divide by the gyroscope sensetivity (use 1000 and round() to round the value to three decimal places)
	*pitch = round((*pitch - gp_off) * 1000.0 / GYRO_SENS) / 1000.0;
	*yaw = round((*yaw - gy_off) * 1000.0 / GYRO_SENS) / 1000.0;
}

void IMU::getMag(float *x, float *y, float *z)
{


    getMagRaw(x, y, z); //Store raw values into variables

    Eigen::Vector3d temp;
    temp << *x, *y, *z;

    temp = soft_iron_scaling * (temp - hard_iron_offset);
    //temp = (temp - hard_iron_offset);

    temp.normalize();

    *x = temp(0);
    *y = temp(1);
    *z = temp(2); 

    
    double headingRad = atan2(*x, *y) ;
    
    std::cout << "Heading is " << headingRad * 180/M_PI<< std::endl;
    
}

/*******************************************************************************
 * Function Definitions
 */

void IMU::initialize_hmc5883() {
    uint8_t temp;

    printf("Define set/reset period \n");
    temp = 0x01;
    this->reg_write(i2c, REG_DEVID_QMC5883, 0x0B, &temp, 1); //Define set/reset period

    printf("Define OSR = 512, Full Scale Range = 2 Gauss, ODR = 200Hz, set continuous measurement mode \n");
    temp = 0x0D;
    this->reg_write(i2c, REG_DEVID_QMC5883, 0x09, &temp, 1); //Define OSR = 512, Full Scale Range = 8 Gauss, ODR = 200Hz, set continuous measurement mode

    // printf("write 0 to mode register \n");
    // temp = 0b00000000;
    // this->reg_write(i2c, REG_DEVID_QMC5883, 0x02, &temp, 1); //write 0 to mode register

    
    this->reg_read(i2c, REG_DEVID_QMC5883, 0x0D, &temp, 1); //Read ID reg A
    printf("read Chip ID : %d\n", temp);

    // this->reg_read(i2c, REG_DEVID_QMC5883, 0x0B, &temp, 1); //Read ID reg B
    // printf("read identification register B : %d\n", temp);

    // this->reg_read(i2c, REG_DEVID_QMC5883, 0x0C, &temp, 1); //Read ID reg C
    // printf("read identification register C : %d\n", temp);
}

void IMU::initialize_mpu6050() {
        
    uint8_t temp;
    
    printf("Take MPU6050 out of sleep mode \n");
    temp = 0b00000000;
    this->reg_write(i2c, REG_DEVID_MPU6050, 0x6b, &temp, 1); //Take MPU6050 out of sleep mode - see Register Map
    
    printf("Set DLPF (low pass filter) to 44Hz (so no noise above 44Hz will pass through \n");
    temp = 0b00000011;
    this->reg_write(i2c, REG_DEVID_MPU6050, 0x1a, &temp, 1); //Set DLPF (low pass filter) to 44Hz (so no noise above 44Hz will pass through)

    printf("Set sample rate divider (to 200Hz) \n");	
    temp = 0b00000100;
    this->reg_write(i2c, REG_DEVID_MPU6050, 0x19, &temp, 1); //Set sample rate divider (to 200Hz) - see Register Map
	
    printf("Configure gyroscope settings \n");
    temp = GYRO_CONFIG;
    this->reg_write(i2c, REG_DEVID_MPU6050, 0x1b, &temp, 1); //Configure gyroscope settings - see Register Map (see MPU6050.h for the GYRO_CONFIG parameter)
	
    printf("Configure accelerometer settings \n");
    temp = ACCEL_CONFIG;
    this->reg_write(i2c, REG_DEVID_MPU6050, 0x1c, &temp, 1); //Configure accelerometer settings - see Register Map (see MPU6050.h for the GYRO_CONFIG parameter)

    printf("Set offsets to zero \n");
	//Set offsets to zero
	temp = 0b00000000;
    this->reg_write(i2c, REG_DEVID_MPU6050, 0x06, &temp, 1);
    
    this->reg_write(i2c, REG_DEVID_MPU6050, 0x07, &temp, 1); 
    
    this->reg_write(i2c, REG_DEVID_MPU6050, 0x08, &temp, 1); 
    
    this->reg_write(i2c, REG_DEVID_MPU6050, 0x09, &temp, 1); 
    
    this->reg_write(i2c, REG_DEVID_MPU6050, 0x0A, &temp, 1); 
    
    this->reg_write(i2c, REG_DEVID_MPU6050, 0x0B, &temp, 1); 
    
    temp = 0b10000001;
    this->reg_write(i2c, REG_DEVID_MPU6050, 0x00, &temp, 1); 
    
    temp = 0b00000001;
    this->reg_write(i2c, REG_DEVID_MPU6050, 0x01, &temp, 1); 
    
    temp = 0b10000001;
    this->reg_write(i2c, REG_DEVID_MPU6050, 0x02, &temp, 1);

    
    printf("Getting offsets \n");
    getOffsets();
}


// Write 1 byte to the specified register
int IMU::reg_write(  i2c_inst_t *i2c, 
                const uint addr, 
                const uint8_t reg, 
                uint8_t *buf,
                const uint8_t nbytes) {

    printf("Reg write called : %d, %d, %d, %d \n", addr, reg, *buf, nbytes);


    int num_bytes_read = 0;
    uint8_t msg[nbytes + 1];

    // Check to make sure caller is sending 1 or more bytes
    if (nbytes < 1) {
        return 0;
    }

    // Append register address to front of data packet
    msg[0] = reg;
    for (int i = 0; i < nbytes; i++) {
        msg[i + 1] = buf[i];
    }

    // Write data to register(s) over I2C
    i2c_write_blocking(i2c, addr, msg, (nbytes + 1), false);

    return num_bytes_read;
}

// Read byte(s) from specified register. If nbytes > 1, read from consecutive
// registers.
int IMU::reg_read(  i2c_inst_t *i2c,
                const uint addr,
                const uint8_t reg,
                uint8_t *buf,
                const uint8_t nbytes) {

    int num_bytes_read = 0;

    // Check to make sure caller is asking for 1 or more bytes
    if (nbytes < 1) {
        return 0;
    }

    // Read data from register(s) over I2C
    i2c_write_blocking(i2c, addr, &reg, 1, true);
    num_bytes_read = i2c_read_blocking(i2c, addr, buf, nbytes, false);

    return num_bytes_read;
}

/*******************************************************************************
 * Main
 */
int main() {
    

    stdio_init_all();
    int i = 10;
    while (i--) {
        printf("Hello, world IMU example!\n");
        sleep_ms(1000);
    }
    
    IMU imu;
    
    printf("Reaching here before init mpu \n");

    imu.initialize_mpu6050();
    imu.initialize_hmc5883();
    
    printf("Reaching here after init mpu \n");


    float gyro_x_f;
    float gyro_y_f;
    float gyro_z_f;
    float acc_x_f;
    float acc_y_f;
    float acc_z_f;
    float mag_x_f;
    float mag_y_f;
    float mag_z_f;


    float roll, pitch;
    float g;
    // Loop forever
    while (true) {

        imu.getMag(&mag_x_f, &mag_y_f, &mag_z_f);
        imu.getAccel(&acc_x_f, &acc_y_f, &acc_z_f);
        imu.getGyro(&gyro_x_f, &gyro_y_f, &gyro_z_f);


        g = sqrt(acc_x_f*acc_x_f + acc_y_f*acc_y_f + acc_z_f*acc_z_f);

        roll = asin(acc_x_f / g);
        pitch = atan2(acc_y_f, acc_z_f);
        
        // Print results
        //printf("X_A: %.2f | Y_A: %.2f | Z_A: %.2f\n", acc_x_f, acc_y_f, acc_z_f);
        //printf("X_G: %.2f | Y_G: %.2f | Z_G: %.2f\r\n", gyro_x_f, gyro_y_f, gyro_z_f);
        //printf("X_M: %.2f | Y_M: %.2f | Z_M: %.2f\r\n", mag_x_f, mag_y_f, mag_z_f);    
        //printf("%.2f %.2f %.2f\n", mag_x_f, mag_y_f, mag_z_f);    

//        printf("Roll angle: %.2f , Pitch angle: %.2f\n", roll, pitch);
        
        sleep_ms(100);
    }
}