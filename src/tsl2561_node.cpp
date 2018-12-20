#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Illuminance.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

using namespace std;

// ------------------------------
// Sensor control registers and data

// I2C address of the sensor
const int I2C_ADDR = 0x39;

// timing register address
const int TIMING_REGISTER = 0x01 | 0x80;
// timing register possible values (integration time for the sensor)
const int TIMING_VALUE_13MS  = 0x00;
const int TIMING_VALUE_101MS = 0x01;
const int TIMING_VALUE_402MS = 0x02;
// gain definition
const int LOW_GAIN_MODE  = 0x00;
const int HIGH_GAIN_MODE = 0x10;

// control register address
const int CONTROL_REGISTER = 0x00 | 0x80;
// control register possible values
const int POWER_UP    = 0x03;
const int POWER_DOWN = 0x00;

// data register addresses
// channel 0
const int CH0_LSB = 0x8C;
const int CH0_MSB = 0x8D;
// channel 1
const int CH1_LSB = 0x8E;
const int CH1_MSB = 0x8F;


// adafruit constants
// Clipping thresholds
#define TSL2561_CLIPPING_13MS     (4900)    ///< # Counts that trigger a change in gain/integration
#define TSL2561_CLIPPING_101MS    (37000)   ///< # Counts that trigger a change in gain/integration
#define TSL2561_CLIPPING_402MS    (65000)   ///< # Counts that trigger a change in gain/integration

#define TSL2561_LUX_LUXSCALE      (14)      ///< Scale by 2^14
#define TSL2561_LUX_RATIOSCALE    (9)       ///< Scale ratio by 2^9
#define TSL2561_LUX_CHSCALE       (10)      ///< Scale channel values by 2^10
#define TSL2561_LUX_CHSCALE_TINT0 (0x7517)  ///< 322/11 * 2^TSL2561_LUX_CHSCALE
#define TSL2561_LUX_CHSCALE_TINT1 (0x0FE7)  ///< 322/81 * 2^TSL2561_LUX_CHSCALE

// T, FN and CL package values
#define TSL2561_LUX_K1T           (0x0040)  ///< 0.125 * 2^RATIO_SCALE
#define TSL2561_LUX_B1T           (0x01f2)  ///< 0.0304 * 2^LUX_SCALE
#define TSL2561_LUX_M1T           (0x01be)  ///< 0.0272 * 2^LUX_SCALE
#define TSL2561_LUX_K2T           (0x0080)  ///< 0.250 * 2^RATIO_SCALE
#define TSL2561_LUX_B2T           (0x0214)  ///< 0.0325 * 2^LUX_SCALE
#define TSL2561_LUX_M2T           (0x02d1)  ///< 0.0440 * 2^LUX_SCALE
#define TSL2561_LUX_K3T           (0x00c0)  ///< 0.375 * 2^RATIO_SCALE
#define TSL2561_LUX_B3T           (0x023f)  ///< 0.0351 * 2^LUX_SCALE
#define TSL2561_LUX_M3T           (0x037b)  ///< 0.0544 * 2^LUX_SCALE
#define TSL2561_LUX_K4T           (0x0100)  ///< 0.50 * 2^RATIO_SCALE
#define TSL2561_LUX_B4T           (0x0270)  ///< 0.0381 * 2^LUX_SCALE
#define TSL2561_LUX_M4T           (0x03fe)  ///< 0.0624 * 2^LUX_SCALE
#define TSL2561_LUX_K5T           (0x0138)  ///< 0.61 * 2^RATIO_SCALE
#define TSL2561_LUX_B5T           (0x016f)  ///< 0.0224 * 2^LUX_SCALE
#define TSL2561_LUX_M5T           (0x01fc)  ///< 0.0310 * 2^LUX_SCALE
#define TSL2561_LUX_K6T           (0x019a)  ///< 0.80 * 2^RATIO_SCALE
#define TSL2561_LUX_B6T           (0x00d2)  ///< 0.0128 * 2^LUX_SCALE
#define TSL2561_LUX_M6T           (0x00fb)  ///< 0.0153 * 2^LUX_SCALE
#define TSL2561_LUX_K7T           (0x029a)  ///< 1.3 * 2^RATIO_SCALE
#define TSL2561_LUX_B7T           (0x0018)  ///< 0.00146 * 2^LUX_SCALE
#define TSL2561_LUX_M7T           (0x0012)  ///< 0.00112 * 2^LUX_SCALE
#define TSL2561_LUX_K8T           (0x029a)  ///< 1.3 * 2^RATIO_SCALE
#define TSL2561_LUX_B8T           (0x0000)  ///< 0.000 * 2^LUX_SCALE
#define TSL2561_LUX_M8T           (0x0000)  ///< 0.000 * 2^LUX_SCALE

// -------------------------
// i2c setting utils

int fd = 0;

uint8_t integration_time = TIMING_VALUE_101MS;
uint8_t gain = LOW_GAIN_MODE;

void i2c_setup() {
    fd = wiringPiI2CSetup(I2C_ADDR);
    if (fd == -1)
        ROS_ERROR("Cant find I2C device with addr %d", I2C_ADDR);
}

void write(int reg, int data) {
    wiringPiI2CWriteReg8(fd, reg, data);
}

uint16_t read8(int reg) {
    uint16_t val = wiringPiI2CReadReg8(fd, reg);
    return val;
}

uint16_t read16(int reg) {
    uint16_t val = wiringPiI2CReadReg16(fd, reg);
    return val;
}


// --------
// user api

void init_sensor() {
    i2c_setup();
    write(CONTROL_REGISTER, POWER_UP); // power ON mode 0x03
    write(TIMING_REGISTER, integration_time | gain); // Timing and gain definition
    usleep(5000);
}

// code extracted from the adafruit class
uint32_t calculateLux(uint16_t broadband, uint16_t ir){
    unsigned long chScale;
    unsigned long channel1;
    unsigned long channel0;

    // Make sure the sensor isn't saturated!
    uint16_t clipThreshold;
    switch (integration_time){
        case TIMING_VALUE_13MS:
            clipThreshold = TSL2561_CLIPPING_13MS;
            break;
        case TIMING_VALUE_101MS:
            clipThreshold = TSL2561_CLIPPING_101MS;
            break;
        default:
            clipThreshold = TSL2561_CLIPPING_402MS;
            break;
    }

    // Return 65536 lux if the sensor is saturated
    if ((broadband > clipThreshold) || (ir > clipThreshold)){
        return 65536;
    }

    // Get the correct scale depending on the intergration time
    switch (integration_time){
        case TIMING_VALUE_13MS:
            chScale = TSL2561_LUX_CHSCALE_TINT0;
            break;
        case TIMING_VALUE_101MS:
            chScale = TSL2561_LUX_CHSCALE_TINT1;
            break;
        default: // No scaling ... integration time = 402ms
            chScale = (1 << TSL2561_LUX_CHSCALE);
            break;
    }

    // Scale for gain (1x or 16x)
    if (! gain) chScale = chScale << 4;

    // Scale the channel values
    channel0 = (broadband * chScale) >> TSL2561_LUX_CHSCALE;
    channel1 = (ir * chScale) >> TSL2561_LUX_CHSCALE;

    // Find the ratio of the channel values (Channel1/Channel0)
    unsigned long ratio1 = 0;
    if (channel0 != 0) ratio1 = (channel1 << (TSL2561_LUX_RATIOSCALE+1)) / channel0;

    // round the ratio value
    unsigned long ratio = (ratio1 + 1) >> 1;

    unsigned int b, m;

    if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1T)){
        b=TSL2561_LUX_B1T; m=TSL2561_LUX_M1T;
    }else if (ratio <= TSL2561_LUX_K2T){
        b=TSL2561_LUX_B2T; m=TSL2561_LUX_M2T;
    }else if (ratio <= TSL2561_LUX_K3T){
        b=TSL2561_LUX_B3T; m=TSL2561_LUX_M3T;
    }else if (ratio <= TSL2561_LUX_K4T){
        b=TSL2561_LUX_B4T; m=TSL2561_LUX_M4T;
    }else if (ratio <= TSL2561_LUX_K5T){
        b=TSL2561_LUX_B5T; m=TSL2561_LUX_M5T;
    }else if (ratio <= TSL2561_LUX_K6T){
        b=TSL2561_LUX_B6T; m=TSL2561_LUX_M6T;
    }else if (ratio <= TSL2561_LUX_K7T){
        b=TSL2561_LUX_B7T; m=TSL2561_LUX_M7T;
    }else if (ratio > TSL2561_LUX_K8T){
        b=TSL2561_LUX_B8T; m=TSL2561_LUX_M8T;
    }

    unsigned long temp;
    temp = ((channel0 * b) - (channel1 * m));

    // Do not allow negative lux value
    if (temp < 0) temp = 0;

    // Round lsb (2^(LUX_SCALE-1))
    temp += (1 << (TSL2561_LUX_LUXSCALE-1));

    // Strip off fractional portion
    uint32_t lux = temp >> TSL2561_LUX_LUXSCALE;

    // Signal I2C had no errors
    return lux;
}


int main(int argc, char **argv) {
    // Setup.
    init_sensor();

    // Start ROS node stuff.
    ros::init(argc, argv, "tsl2561_node");
    ros::NodeHandle node;

    ros::Publisher pub_fullspectrum = node.advertise<sensor_msgs::Illuminance>("tsl2561/full_spectrum", 100);
    ros::Publisher pub_infrared = node.advertise<sensor_msgs::Illuminance>("tsl2561/infrared", 100);
    ros::Publisher pub_lux = node.advertise<sensor_msgs::Illuminance>("tsl2561/lux", 100);

    // ros::Publisher pub_visible = node.advertise<sensor_msgs::Illuminance>("tsl2561/visible", 100);

    sensor_msgs::Illuminance full_spectrum;
    //needed to defined the frame attached to the message
    full_spectrum.header.frame_id = "tsl2561_link";

    sensor_msgs::Illuminance infrared;
    //needed to defined the frame attached to the message
    infrared.header.frame_id = "tsl2561_link";

    sensor_msgs::Illuminance lux;
    //needed to defined the frame attached to the message
    lux.header.frame_id = "tsl2561_link";


    // sensor_msgs::Illuminance visible;
    //needed to defined the frame attached to the message
    // visible.header.frame_id = "tsl2561_link";

    // autonomously change the frequency : todo!
    int frequency = 2; // for 101ms integration time : f=1/T

    ros::Rate loop_rate(frequency);

    ROS_INFO("Starting the sensor reading at a %dHz frequency...", frequency);

    uint16_t ch0, ch1;
    while (ros::ok())
    {
        ch0 = read16(CH0_LSB);
        ch1 = read16(CH1_LSB);

        full_spectrum.illuminance = ch0;
        infrared.illuminance = ch1;
	lux.illuminance = calculateLux(ch0, ch1);
        // visible.illuminance = ch0 - ch1;

        full_spectrum.header.stamp = ros::Time::now();
        infrared.header.stamp = ros::Time::now();
	lux.header.stamp = ros::Time::now();

        pub_fullspectrum.publish(full_spectrum);
        pub_infrared.publish(infrared);
        pub_lux.publish(lux);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
