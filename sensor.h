enum e_i2c_status {
    I2C_SUCCESS,
    I2C_WRITE,
    I2C_READ,
    I2C_ERROR
};
#define HMC5883L_I2C_ADDR 0x1E

//enum e_i2c_status init_i2c(int *fd,int addr);
//enum e_i2c_status read_i2c(int *fd, short *x, short *y, short *z);

enum e_i2c_status sensor_hum_init(int * file);
enum e_i2c_status sensor_hum_read(int *file, float *humidity, float *cTemp);
enum e_i2c_status senseHat_init(int * sock, char * filename);
enum e_i2c_status senseHat_read(int *file, float *humidity, float *cTemp, float *cAngle);