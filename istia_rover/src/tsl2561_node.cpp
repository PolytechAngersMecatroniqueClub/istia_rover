#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Illuminance.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

using namespace std;

// ------------------------------
// Sensor control registers and data

const int I2C_ADDR = 0x39;

const int CONTROL_REGISTER = 0x00 | 0x80;
const int TIMING_REGISTER = 0x01 | 0x80;

const int CH0_LSB = 0x8C;
const int CH0_MSB = 0x8D;

const int CH1_LSB = 0x8E;
const int CH1_MSB = 0x8F;

// -------------------------
// i2c setting utils

int fd = 0;

void i2c_setup() {
    fd = wiringPiI2CSetup(I2C_ADDR);
    if (fd == -1)
        ROS_ERROR("Cant find I2C device with addr %d", I2C_ADDR);
}

void write(int reg, int data) {
    wiringPiI2CWriteReg8(fd, reg, data);
}

int read8(int reg) {
    int val = wiringPiI2CReadReg8(fd, reg);
    return val;
}

int read16(int reg) {
    int val = wiringPiI2CReadReg16(fd, reg);
    return val;
}


// --------
// user api

void init_sensor() {
    i2c_setup();
    write(CONTROL_REGISTER, 0x03); // power ON mode 0x03
    write(TIMING_REGISTER, 0x02); // Nominal integration time = 402ms (0x02)
    usleep(5000);
}

int main(int argc, char **argv) {
    // Setup.
    init_sensor();

    // Start ROS node stuff.
    ros::init(argc, argv, "tsl2561_node");
    ros::NodeHandle node;

    ros::Publisher pub_fullspectrum = node.advertise<sensor_msgs::Illuminance>("tsl2561/full_spectrum", 100);
    ros::Publisher pub_infrared = node.advertise<sensor_msgs::Illuminance>("tsl2561/infrared", 100);
    ros::Publisher pub_visible = node.advertise<sensor_msgs::Illuminance>("tsl2561/visible", 100);

    sensor_msgs::Illuminance full_spectrum;
    //needed to defined the frame attached to the message
    full_spectrum.header.frame_id = "tsl2561_link";

    sensor_msgs::Illuminance infrared;
    //needed to defined the frame attached to the message
    infrared.header.frame_id = "tsl2561_link";

    sensor_msgs::Illuminance visible;
    //needed to defined the frame attached to the message
    visible.header.frame_id = "tsl2561_link";

    int frequency = 5;

    ros::Rate loop_rate(frequency);

    ROS_INFO("Starting the sensor reading at a %dHz frequency...", frequency);

    float ch0, ch1;
    while (ros::ok())
    {
        ch0 = read16(CH0_LSB);
        ch1 = read16(CH1_LSB);

        full_spectrum.illuminance = ch0;
        infrared.illuminance = ch1;
        visible.illuminance = ch0 - ch1;

        pub_fullspectrum.publish(full_spectrum);
        pub_infrared.publish(infrared);
        pub_visible.publish(visible);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
