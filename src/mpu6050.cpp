#include "mpu6050.h"

mpu6050::mpu6050(uint8_t addr_){

    uint8_t val = 0;
    i2c_addr = addr_;
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    //bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
    

    mpu6050::reset();

    sleep_ms(200);
    //setup, see datasheet
    mpu6050::write(0x1A,0x05);
    sleep_ms(200);
    mpu6050::write(0x1B,0x8);
    sleep_ms(200);
    mpu6050::write(0x1C,0x0);
    sleep_ms(200);



}

void mpu6050::write(uint8_t register_, uint8_t val){
    uint8_t data[2] = {register_, val};
    i2c_write_blocking(i2c_default, i2c_addr, data, 2, false);

}



void mpu6050::read(uint8_t device_address, uint8_t reg,uint8_t * value){
    i2c_write_blocking(i2c_default, device_address, &reg, 1, true); // Send register address, keep bus active
    i2c_read_blocking(i2c_default, device_address, value, 6, false); // Read 1 byte, release bus
}
void mpu6050::reset(){
    //reset the chip

    mpu6050::write(0x6B,0x88);
    sleep_ms(200);
    mpu6050::write(0x6B,0x08);
    sleep_ms(200);
    //reset the accelerometer and gyroscope
    mpu6050::write(0x68,0x07);
    sleep_ms(200);
    mpu6050::write(0x68,0x00);
    sleep_ms(200);
}

void mpu6050::std(int n,bool flag_accel){

}



void mpu6050::calibration(int n,bool flag_accel){
    float mean_accel[3] = {0.0,0.0,0.0};
    float mean_gyro[3] = {0.0,0.0,0.0};
    int n_meas = 10;
    Serial.print("Starting Accelometer Calibration Process...");
    sleep_ms(100);

    if(flag_accel){
        for(int i = 0; i < n; i++){
            Serial.print("Position ");Serial.print(": ");Serial.println(i+1);
            sleep_ms(100);
            //Take 10 measurements compute the mean value
            for(int j = 0; j < n_meas; j++){
                mpu6050::read_values();
                for(int k = 0; k < 3; k++){
                    mean_accel[k] += accel[k];
                }
                sleep_ms(10);
            }
            for(int k = 0; k < 3; k++){
                mean_accel[k] = mean_accel[k]/n_meas;
            }

            Serial.print("Acceleration X:");
            Serial.print(mean_accel[0]);
            Serial.print(" Acceleration Y:");
            Serial.print(mean_accel[1]);
            Serial.print(" Acceleration Z:");
            Serial.println(mean_accel[2]);

            for(int k = 0; k < 3; k++){
                mean_accel[k] = 0;
            }
        }
    }
    Serial.println("Hold the board still. Starting Gyro Calibration Process...");
    sleep_ms(1000);

    n_meas = 10;
    //Take 10 measurements compute the mean value
    for(int j = 0; j < n_meas; j++){
        mpu6050::read_values();
        for(int k = 0; k < 3; k++){
            mean_gyro[k] += gyro[k];
        }
        sleep_ms(10);
    }
    for(int k = 0; k < 3; k++){
        gyro_drift[k] = mean_gyro[k]/n_meas;
    }

    Serial.print("Gyro offset X:");
    Serial.print(gyro_drift[0]);
    Serial.print(" Gyro offset Y:");
    Serial.print(gyro_drift[1]);
    Serial.print(" Gyro offset Z:");
    Serial.println(gyro_drift[2]);


}
void mpu6050::read_values(){

    uint8_t buffer[6];
    int16_t read = 0;

    // Start reading acceleration registers from register 0x3B for 6 bytes
    mpu6050::write(0x1A,0x05);
    mpu6050::write(0x1C,0x0);
    mpu6050::read(0x68,0x3B,buffer);

    for (int i = 0; i < 3; i++) {

        read =   (buffer[i * 2]<< 8 | buffer[(i * 2) + 1]);
        accel[i] = (float)read/16384.0;
        accel[i] -= accel_drift[i];       
    }

    //Roll
    angle[0] = atan(accel[1]/sqrt(accel[0]*accel[0]+accel[2]*accel[2]))/(PI/180);
    //Pitch
    angle[1] = -atan(accel[0]/sqrt(accel[1]*accel[1]+accel[2]*accel[2]))/ (PI/180);

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    mpu6050::write(0x1B,0x8);
    mpu6050::read(0x68,0x43,buffer);

    for (int i = 0; i < 3; i++) {
        read =   (buffer[i * 2]<< 8 | buffer[(i * 2) + 1]);    
        gyro[i] = (float) read/65.5;
        gyro[i] -= gyro_drift[i];
        
    }




}

uint8_t mpu6050::debug(uint8_t teste){
    i2c_write_blocking(i2c_default, i2c_addr, &teste, 1, true);
    
    i2c_read_blocking(i2c_default, i2c_addr, &teste, 1, false);
    return teste;

}