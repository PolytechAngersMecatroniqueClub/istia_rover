/*
This file is the source code of the motor_hat_twist_node. Is is based on the motor_hat_twist_node file 
from matpalm (https://github.com/matpalm/ros-motorhat-node).
*/
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

using namespace std;

string cst_topic_name;
int cst_motor_drive;
int cst_motor_steer;
int cst_max_speed;
double cst_max_twist;


// ------------------------------
// HAT control registers and data

const int I2C_ADDR_DRIVE = 0x60;
const int I2C_ADDR_STEER = 0x40;

const int MODE1 = 0x00;
const int MODE2 = 0x01;
const int PRESCALE = 0xFE;
const int LED0_ON_L = 0x06;
const int LED0_ON_H = 0x07;
const int LED0_OFF_L = 0x08;
const int LED0_OFF_H = 0x09;
const int ALL_LED_ON_L = 0xFA;
const int ALL_LED_ON_H = 0xFB;
const int ALL_LED_OFF_L = 0xFC;
const int ALL_LED_OFF_H = 0xFD;

const int SLEEP = 0x10;
const int ALLCALL = 0x01;
const int OUTDRV = 0x04;

// --------------------------------------------------
// mappings from motorN to specific control registers
//                     m0  m1  m2  m3
const int pwmPin_drive = 8;
const int in1Pin_drive = 10;
const int in2Pin_drive = 9;

const int pwmPin_steer = 15;

int cst_min_steer = 400;
int cst_max_steer = 500;

// -------------------------
// i2c and pwm setting utils

int fd_drive = 0;
int fd_steer = 0;

void i2c_setup() {
    fd_drive = wiringPiI2CSetup(I2C_ADDR_DRIVE);
    if (fd_drive == -1)
        ROS_ERROR("Cant find I2C device with addr %d (Drive motor)", I2C_ADDR_DRIVE);
    else ROS_INFO("fd_drive %d (i2C)", fd_drive);

    fd_steer = wiringPiI2CSetup(I2C_ADDR_STEER);
    if (fd_steer == -1)
        ROS_ERROR("Cant find I2C device with addr %d (Steer motor)", I2C_ADDR_STEER);
    else ROS_INFO("fd_steer %d (i2C)", fd_steer);
}

void write(int fd, int reg, int data) {
    wiringPiI2CWriteReg8(fd, reg, data);
}

int read(int fd, int reg) {
    int val = wiringPiI2CReadReg8(fd, reg);
    return val;
}

void setPWM(int fd, int channel, int on, int off) {
    write(fd, LED0_ON_L+4*channel, on & 0xFF);
    write(fd, LED0_ON_H+4*channel, on >> 8);
    write(fd, LED0_OFF_L+4*channel, off & 0xFF);
    write(fd, LED0_OFF_H+4*channel, off >> 8);
}

void setPWMSteer(int led, int on, int off) {
    write(fd_steer, LED0_ON_L+4*(led-1), on & 0xFF);
    write(fd_steer, LED0_ON_H+4*(led-1), on >> 8);
    write(fd_steer, LED0_OFF_L+4*(led-1), off & 0xFF);
    write(fd_steer, LED0_OFF_H+4*(led-1), off >> 8);
}

void setPin(int fd, int pin, int val) {
    if (val == 0)
        setPWM(fd, pin, 0, 4096);
    else if (val == 1)
        setPWM(fd, pin, 4096, 0);
    else
        cerr << "invalid val for setPin(" << pin << "," << val << ")" << endl;
}

// --------
// user api

void init_hat() {
    // This is just cut and paste (including sleeps)
    // See original for some more explaination....
    i2c_setup();

    //init the driver hat
    write(fd_drive, ALL_LED_ON_L, 0);
    write(fd_drive, ALL_LED_ON_H, 0);
    write(fd_drive, ALL_LED_OFF_L, 0);
    write(fd_drive, ALL_LED_OFF_H, 0);
    write(fd_drive, MODE2, OUTDRV);
    write(fd_drive, MODE1, ALLCALL);
    usleep(5000);
    int mode1_drive = read(fd_drive, MODE1);
    mode1_drive &= ~SLEEP;
    write(fd_drive, MODE1, mode1_drive);
    usleep(5000);

    /*//init the steer hat (from PCA9685 documentation - github)
    write(fd_steer, ALL_LED_ON_L, 0);
    write(fd_steer, ALL_LED_ON_H, 0);
    write(fd_steer, ALL_LED_OFF_L, 0);
    write(fd_steer, ALL_LED_OFF_H, 0);
    write(fd_steer, MODE1, ALLCALL);
    write(fd_steer, MODE2, OUTDRV);
    usleep(5000);
    int mode1_steer = read(fd_steer, MODE1);
    mode1_steer &= ~SLEEP;
    write(fd_steer, MODE1, mode1_steer);
    usleep(5000);*/

    // reset
    write(fd_steer, MODE1, 0x00); // Normal mode
    write(fd_steer, MODE2, 0x04); // totem pole
    usleep(5000);
    // set frequency
    int freq = 50; // between 40Hz and 1000Hz considering a 25MHz oscillator
    float prescaleval = 25000000.0;
    prescaleval /= 4096.0;
    prescaleval /= freq;
    prescaleval -= 1.0;
    write(fd_steer, MODE1, 0x10); // Sleep
    write(fd_steer, PRESCALE, floor(prescaleval));
    write(fd_steer, MODE1, 0x80); // restart
    write(fd_steer, MODE2, 0x04); // totem pole, default
}

void setPWMFreq(int fd, int freq) {
    float prescaleval = 25000000.0;
    prescaleval /= 4096.0;
    prescaleval /= freq;
    prescaleval -= 1.0;
    float prescale = floor(prescaleval + 0.5);
    int old_mode = read(fd, MODE1);
    int new_mode = (old_mode & 0x7F) | 0x10;
    write(fd, MODE1, new_mode);
    write(fd, PRESCALE, int(floor(prescale)));
    write(fd, MODE1, old_mode);
    usleep(5000);
    write(fd, MODE1, old_mode | 0x80);
}

void getPWM(int fd, int channel) {
    int ledval = 0;
    ledval = read(fd, LED0_OFF_H+4*channel);
    ledval = ledval & 0xF;
    ledval <<= 8;
    ledval += read(fd, LED0_OFF_L+4*channel);
    ROS_INFO("Read PMW: %d", ledval);
    


/*void setPWM(int fd, int channel, int on, int off) {
    write(fd, LED0_ON_L+4*channel, on & 0xFF);
    write(fd, LED0_ON_H+4*channel, on >> 8);
    write(fd, LED0_OFF_L+4*channel, off & 0xFF);
    write(fd, LED0_OFF_H+4*channel, off >> 8);
}*/
}

void turnOffDriveMotor() {
    ROS_INFO("Stopping drive motor");
    setPin(fd_drive, in1Pin_drive, 0);
    setPin(fd_drive, in2Pin_drive, 0);
}

void setSpeed(int speed) {
    ROS_INFO("Setting speed of drive motor to %d", speed);
    if (speed < -255 || speed > 255) {
        ROS_ERROR("Invalid Speed [%d] Expected >=-255 <=255", speed);
        return;
    }

    if (speed == 0) {
        // RELEASE
        turnOffDriveMotor();
    }
    else if (speed > 0) {
        // FORWARD
        setPin(fd_drive, in2Pin_drive, 0);
        setPin(fd_drive, in1Pin_drive, 1);
        setPWM(fd_drive, pwmPin_drive, 0, speed * 16);
    }
    else { // => speed < 0
        // BACKWARD
        setPin(fd_drive, in2Pin_drive, 1);
        setPin(fd_drive, in1Pin_drive, 0);
        setPWM(fd_drive, pwmPin_drive, 0, -speed * 16);
    }
}

void setSteer(int steer) {
    ROS_INFO("Setting steer motor to %d", steer);
    setPWM(fd_steer, pwmPin_steer, 0, steer);
}

// Receive a twist message corresponding to the desired motor speeds.
void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    float x = msg->linear.x;    // linear speed
    float z = -msg->angular.z;   // angular speed (rotation)

    // we check if the twist value is lower than the max defined one
    if(x > cst_max_twist || x < -cst_max_twist){
        ROS_WARN("linear X twist value is highter than the max, max value is considered");
        x>cst_max_twist? x= cst_max_twist : x=-cst_max_twist;
    }
    if(z > cst_max_twist || z < -cst_max_twist){
        ROS_WARN("angular Z twist value is highter than the max, max value is considered");
        z>cst_max_twist? z= cst_max_twist : z=-cst_max_twist;
    }

    int valdrive=0, valsteer=0;

    valdrive = cst_max_speed * x / cst_max_twist;

    valsteer = cst_min_steer + (z+cst_max_twist)*(cst_max_steer - cst_min_steer)/(2.0*cst_max_twist);

    setSpeed(valdrive);
    setSteer(valsteer);
}

int main(int argc, char **argv) {
    // Setup.
    init_hat();
    setPWMFreq(fd_drive, 1600);
    // setPWMFreq(fd_steer, 50);
    atexit(turnOffDriveMotor);

    // Start ROS node stuff.
    ros::init(argc, argv, "motor_hat_twist_node");
    ros::NodeHandle node;

    // Get parameters so we can change these later.
    // this allows to use the launch file parameters!
    node.param<std::string>("/motor_hat_twist_node/topic_name", cst_topic_name, "/default/cmd_vel");
    node.param<int>("/motor_hat_twist_node/motor_drive", cst_motor_drive, 0);
    node.param<int>("/motor_hat_twist_node/motor_steer", cst_motor_steer, 1);
    node.param<int>("/motor_hat_twist_node/max_speed", cst_max_speed, 100);
    node.param<double>("/motor_hat_twist_node/max_twist", cst_max_twist, 1);

    ROS_INFO("considered cmd topic name: \"%s\"", cst_topic_name.c_str());

    ros::Subscriber sub = node.subscribe(cst_topic_name, 5, cmdCallback);

    // Run until exit.
    ros::spin();
    return 0;
}

