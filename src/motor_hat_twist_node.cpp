#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

using namespace std;

string cst_topic_name;
int cst_motor_bot_left;
int cst_motor_bot_right;
int cst_motor_front_left;
int cst_motor_front_right;
int cst_max_speed;
double cst_max_twist;


// ------------------------------
// HAT control registers and data

const int I2C_ADDR = 0x60;

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
const int pwmPin[] = {  8, 13,  2,  7 };
const int in1Pin[] = { 10, 11,  4,  5 };
const int in2Pin[] = {  9, 12,  3,  6 };

// -------------------------
// i2c and pwm setting utils

int fd = 0;
void i2c_setup() {
    fd = wiringPiI2CSetup(I2C_ADDR);
    if (fd == -1)
        ROS_ERROR("Cant find I2C device with addr %d", I2C_ADDR);
}

void write(int reg, int data) {
    wiringPiI2CWriteReg8(fd, reg, data);
}

int read(int reg) {
    int val = wiringPiI2CReadReg8(fd, reg);
    return val;
}

void setPWM(int channel, int on, int off) {
    write(LED0_ON_L+4*channel, on & 0xFF);
    write(LED0_ON_H+4*channel, on >> 8);
    write(LED0_OFF_L+4*channel, off & 0xFF);
    write(LED0_OFF_H+4*channel, off >> 8);
}

void setPin(int pin, int val) {
    if (val == 0)
        setPWM(pin, 0, 4096);
    else if (val == 1)
        setPWM(pin, 4096, 0);
    else
        cerr << "invalid val for setPin(" << pin << "," << val << ")" << endl;
}

// --------
// user api

void init_hat() {
    // This is just cut and paste (including sleeps)
    // See original for some more explaination....
    i2c_setup();
    write(ALL_LED_ON_L, 0);
    write(ALL_LED_ON_H, 0);
    write(ALL_LED_OFF_L, 0);
    write(ALL_LED_OFF_H, 0);
    write(MODE2, OUTDRV);
    write(MODE1, ALLCALL);
    usleep(5000);
    int mode1 = read(MODE1);
    mode1 &= ~SLEEP;
    write(MODE1, mode1);
    usleep(5000);
}

void setPWMFreq(int freq) {
    float prescaleval = 25000000.0;
    prescaleval /= 4096.0;
    prescaleval /= freq;
    prescaleval -= 1.0;
    float prescale = floor(prescaleval + 0.5);
    int old_mode = read(MODE1);
    int new_mode = (old_mode & 0x7F) | 0x10;
    write(MODE1, new_mode);
    write(PRESCALE, int(floor(prescale)));
    write(MODE1, old_mode);
    usleep(5000);
    write(MODE1, old_mode | 0x80);
}

void stop(int motor_id) {
    ROS_INFO("Stopping motor %d", motor_id);
    setPin(in1Pin[motor_id], 0);
    setPin(in2Pin[motor_id], 0);
}

void setSpeed(int motor_id, int speed) {
    ROS_INFO("Setting speed of motor %d to %d", motor_id, speed);
    if (motor_id < 0 || motor_id > 3) {
        ROS_ERROR("Invalid motor_id [%d] Expected 0,1,2,3", motor_id);
        return;
    }
    if (speed < -255 || speed > 255) {
        ROS_ERROR("Invalid Speed [%d] Expected >=-255 <=255", speed);
        return;
    }

    if (speed == 0) {
        // RELEASE
        stop(motor_id);
    }
    else if (speed > 0) {
        // FORWARD
        setPin(in2Pin[motor_id], 0);
        setPin(in1Pin[motor_id], 1);
        setPWM(pwmPin[motor_id], 0, speed * 16);
    }
    else { // => speed < 0
        // BACKWARD
        setPin(in1Pin[motor_id], 0);
        setPin(in2Pin[motor_id], 1);
        setPWM(pwmPin[motor_id], 0, -speed * 16);
    }
}

void turnOffMotors() {
    for (int i = 0; i < 4; ++i)
        stop(i);
}

// Receive a vector4 of int corresponding to the desired motor speeds.
void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    float x = msg->linear.x;    // linear speed
    float z = msg->angular.z;   // angular speed (rotation)
    int flagforward = 1;
    int flagleft = 1;

    // we get the absolute value of the command saving the direction forward/backward and left/right
    if(x<0){
        flagforward = -1;
        x = -x;
    }
    if(z<0){
        flagleft = -1;
        z = -z;
    }

    // we check if the twist value is lower than the max defined one
    if(x > cst_max_twist){
        ROS_WARN("linear X twist value is highter than the max, max value is considered");
        x = cst_max_twist;
    }
    if(z > cst_max_twist){
        ROS_WARN("angular Z twist value is highter than the max, max value is considered");
        z = cst_max_twist;
    }

    int valleft=0, valrigth=0;
    if(x > z){
        // linear command is considered
        valleft = valrigth = int(cst_max_speed*x/(cst_max_twist));  // linear speed of all the motors
        if( flagleft == 1 ){
            valleft -= int(cst_max_speed*z/cst_max_twist); // angular speed for the left motors
        }else{
            valrigth -= int(cst_max_speed*z/cst_max_twist); // angular speed for the right motors
        }
        valleft *= flagforward;
        valrigth *= flagforward;
    }else{
        // linear command is not considered, only rotation (angular)
        // valleft = -int(cst_max_speed*(z-x)/(cst_max_twist))*flagleft;    // angular speed for the right motors
	valleft = -int(cst_max_speed*(z)/(cst_max_twist))*flagleft;    // angular speed for the right motors
        valrigth = -valleft;                                    // angular speed for the left motors
    }

    setSpeed(cst_motor_bot_left, -valleft);
    setSpeed(cst_motor_bot_right, -valrigth);
    setSpeed(cst_motor_front_left, -valleft);
    setSpeed(cst_motor_front_right, -valrigth);
}

int main(int argc, char **argv) {
    // Setup.
    init_hat();
    setPWMFreq(1600);
    atexit(turnOffMotors);

    // Start ROS node stuff.
    ros::init(argc, argv, "motor_hat_twist_node");
    ros::NodeHandle node;

    // Get parameters so we can change these later.
    // this allows to use the launch file parameters!
    node.param<std::string>("/motor_hat_twist_node/topic_name", cst_topic_name, "/default/cmd_vel");
    node.param<int>("/motor_hat_twist_node/motor_bot_left", cst_motor_bot_left, 0);
    node.param<int>("/motor_hat_twist_node/motor_bot_right", cst_motor_bot_right, 1);
    node.param<int>("/motor_hat_twist_node/motor_front_left", cst_motor_front_left, 3);
    node.param<int>("/motor_hat_twist_node/motor_front_right", cst_motor_front_right, 2);
    node.param<int>("/motor_hat_twist_node/max_speed", cst_max_speed, 100);
    node.param<double>("/motor_hat_twist_node/max_twist", cst_max_twist, 1);

    ROS_INFO("considered cmd topic name: \"%s\"", cst_topic_name.c_str());

    ros::Subscriber sub = node.subscribe(cst_topic_name, 5, cmdCallback);

    // Run until exit.
    ros::spin();
    return 0;
}
