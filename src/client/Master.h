#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <serial/serial.h>
#include <timing/TimeStamp.hpp>
#include "structs.h"

namespace RhobanSSL
{
class Master
{
public:
    struct Robot
    {
        Robot();

        bool present;
        Utils::Timing::TimeStamp lastUpdate;
        struct packet_robot status;

        float age();
        bool isOk();
    };

    Master(std::string port, unsigned int baudrate);
    virtual ~Master();

    // Emergency stop
    void em();

    // Stop the master
    void stop();

    // Send the packet
    void send();

    // Master packets and statuses
    struct Robot robots[MAX_ROBOTS];

    void addRobotPacket(int robot, struct packet_master robotPacket);
    void addParamPacket(int robot, struct packet_params params);

protected:
    bool running;
    bool shouldSend;
    bool shouldSendParams;
    bool receivedAnswer;
    Utils::Timing::TimeStamp lastSend;

    serial::Serial serial;
    std::thread *thread;
    std::mutex mutex;
    std::string tmpPacket;
    size_t tmpNbRobots;
    std::string packet;
    size_t nbRobots;

    void execute();
    void addPacket(int robot, int instruction, char *packet, size_t len);
    void sendPacket();
};
}
