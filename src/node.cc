//
// Created by dinir on 9/14/20.
//
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "NgimuReceive.h"
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <cstdlib>
#include <iostream>
#include <thread>
// C library headers
#include <stdio.h>
#include <string.h>

ros::Publisher pub;
int mSerialPort = -1;

void ngimuSensorsCallback(const NgimuSensors ngimuSensors)
{
    sensor_msgs::Imu imuData;
    // set time
    imuData.header.stamp = ros::Time::now();

    // accelerometer
    imuData.linear_acceleration.x = ngimuSensors.accelerometerX;
    imuData.linear_acceleration.y = ngimuSensors.accelerometerY;
    imuData.linear_acceleration.z = ngimuSensors.accelerometerZ;
    // gyroscope
    imuData.angular_velocity.x = ngimuSensors.gyroscopeX;
    imuData.angular_velocity.y = ngimuSensors.gyroscopeY;
    imuData.angular_velocity.z = ngimuSensors.gyroscopeZ;

    pub.publish(imuData);
};

void initComPort()
{
    int serialPort = open("/dev/ttyACM0", O_RDWR);

    if (serialPort < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }

    struct termios tty;
    // Read in existing settings, and handle any error
    if(tcgetattr(serialPort, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag |= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

// Set in/out baud rate to be 9600
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);


    if (tcsetattr(serialPort, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }
    ::mSerialPort = serialPort;
}

void receiveImu()
{
    char readBuf;

    while (true)
    {
        int n = read(mSerialPort, &readBuf, sizeof(readBuf));

        if (n == 1)
        {
            NgimuReceiveProcessSerialByte(readBuf);
        }
    }

}

int main(int argc, char ** argv)
{
    ros::init( argc, argv, "ngimu");
    ros::NodeHandle n;
    pub = n.advertise<sensor_msgs::Imu>("/ngimu/imu", 400);

    initComPort();

    std::thread th1(receiveImu);
    th1.detach();

    // init IMU sensor
    NgimuReceiveInitialise();
    NgimuReceiveSetSensorsCallback(ngimuSensorsCallback);
    
    ros::spin();
}