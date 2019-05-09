/*
This file is the source code of the cerema rover motors. Is is based on the motor_hat_twist_node file 
from matpalm (https://github.com/matpalm/ros-motorhat-node).
*/

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

using namespace std;

// the following constants can be initialized in the launch file
string cst_topic_name;  // constant value for the command topic name (twist)
int cst_max_speed;      // the linear maximal speed (pwm value)
double cst_max_twist;   // the max value of the considered twist


const int I2C_ADDR_DRIVE = 0x60; // i2c address of the DC motor shield
const int I2C_ADDR_STEER = 0x40; // i2c address of the servo motor shield

// ------------------------------
// HATs control registers and data

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
const int pwmPin_drive = 8;  // the pwm where the DC motor is plugged on the DC hat
const int in1Pin_drive = 10; // for forward/backward option
const int in2Pin_drive = 9;  // for forward/backward option

const int pwmPin_steer = 15; // the pwm where the servo motor is plugged on the servo hat

// maximal values left and right for the servo motor (according to the mechanical part)
int cst_min_steer = 400;
int cst_max_steer = 550;

// -------------------------
// i2c and pwm setting utils

int fd_drive = 0; // to save the id of the DC motor hat
int fd_steer = 0; // to save the id of the servo motor hat

// to get the id of the hats
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

// to write on the i2c bus
void write(int fd, int reg, int data) {
    // fd : the i2c id of the corresponding hat
    // reg: the register where to write the data
    // data: the data to write
    wiringPiI2CWriteReg8(fd, reg, data); // function from the wiringPiI2C library
}

// to read a value on the i2c bus
int read(int fd, int reg) {
    // fd : the i2c id of the corresponding hat
    // reg: the register where to read the data
    // returns the read data
    int val = wiringPiI2CReadReg8(fd, reg); // function from the wiringPiI2C library
    return val;
}

// to set a PWM value
void setPWM(int fd, int channel, int on, int off) {
    // fd : the i2c id of the corresponding hat
    // channel : the corresponding PWM to set
    // on : may be zero
    // off: the pwm value (max 4095)
    write(fd, LED0_ON_L+4*channel, on & 0xFF);
    write(fd, LED0_ON_H+4*channel, on >> 8);
    write(fd, LED0_OFF_L+4*channel, off & 0xFF);
    write(fd, LED0_OFF_H+4*channel, off >> 8);
}

// to set a PWM value as digital output (zero or one)
void setPin(int fd, int pin, int val) {
    // fd : the i2c id of the corresponding hat
    // pin : the corresponding PWM to set
    // val : 0 or 1
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
    i2c_setup(); // to get the corresponding i2c id

    // --- init the driver hat (DC motor) ---
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

    // --- init the steer hat (servo motor) ---
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

// to set the frequency of the PWM (warning: not the value, the frequency!)
// note that this function is only used for the DC motor, as the servo motor frequency is set in the init function
// it works that way so I did not changed it but may be improved...
void setPWMFreq(int fd, int freq) {
    // fd : the i2c id of the corresponding hat
    // freq : the desired frequency
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

// to read the current value of a PMW, for debugging purpose mostly
void getPWM(int fd, int channel) {
    // fd : the i2c id of the corresponding hat
    // channel : the PWM to read
    int ledval = 0;
    ledval = read(fd, LED0_OFF_H+4*channel);
    ledval = ledval & 0xF;
    ledval <<= 8;
    ledval += read(fd, LED0_OFF_L+4*channel);
    ROS_INFO("Read PMW: %d", ledval);
    // not that I only used this function for debug purposes, that is the value is only displayed and not returned...

}

// to stop the rovor by shutting down the DC motor
void turnOffDriveMotor() {
    ROS_INFO("Stopping drive motor");
    setPin(fd_drive, in1Pin_drive, 0);
    setPin(fd_drive, in2Pin_drive, 0);
}

// to set the rover speed by setting the DC motor
void setSpeed(int speed) {
    // speed: the wanted speed (pwm value)
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

// to set the robot direction by setting up the servo motor
void setSteer(int steer) {
    // steer: the servo motor value (pwm)
    setPWM(fd_steer, pwmPin_steer, 0, steer);
}

// Receive a twist message corresponding to the desired motor speeds.
void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // msg : the received twist message
    float x = msg->linear.x;     // linear speed
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

    valdrive = cst_max_speed * x / cst_max_twist; // we compute the DC pwm value from the twist  and the maximal speed
    // we compute the servo motor value from the twist and the turing bounds
    valsteer = cst_min_steer + (z+cst_max_twist)*(cst_max_steer - cst_min_steer)/(2.0*cst_max_twist);

    setSpeed(valdrive); // set the speed
    setSteer(valsteer); // set the steer
}

// main function
int main(int argc, char **argv) {
    // Setup.
    init_hat(); // init the i2c stuff
    // init the DC motor PWM frequency (the servo PWM frequency is init in the init_hat function
    setPWMFreq(fd_drive, 1600);
    atexit(turnOffDriveMotor); // to stop the robot when killing the node

    // Start ROS node stuff.
    ros::init(argc, argv, "motor_hat_twist_node");
    ros::NodeHandle node;

    // Get parameters so we can change these later.
    // this allows to use the launch file parameters!
    node.param<std::string>("/motor_hat_twist_node/topic_name", cst_topic_name, "/default/cmd_vel");
    node.param<int>("/motor_hat_twist_node/max_speed", cst_max_speed, 300);
    node.param<double>("/motor_hat_twist_node/max_twist", cst_max_twist, 1);

    // display the name of the listened topic
    ROS_INFO("considered cmd topic name: \"%s\"", cst_topic_name.c_str());

    ros::Subscriber sub = node.subscribe(cst_topic_name, 5, cmdCallback);

    // Run until exit.
    ros::spin();
    return 0;
}

