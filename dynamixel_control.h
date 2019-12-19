//
// Created by rns on 12/18/19.
//
#ifndef DYNAMIXEL_CONTROL_DYNAMIXEL_CONTROL_H
#define DYNAMIXEL_CONTROL_DYNAMIXEL_CONTROL_H

#include <Arduino.h>

using namespace std;

class DynamixelController {
private:
    // ATTRIBUTES
    Stream *_serialport; //_serialport is set as a pointer to a Stream from Arduino.h (&Serial1)
    int _flow_control_pin;

    struct status_packet_info{
        unsigned char id;
        unsigned char error;
        unsigned char parameters[4];
    };

    // METHODS
    unsigned short update_crc (unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size); // Calculates CRC
    void CreateCRC(unsigned char *tx_packet, unsigned short blk_size);

    void TransmitPacket(unsigned char *tx_packet);
    status_packet_info ReceiveStatusPacket();
    unsigned short CreateLength(unsigned char *tx_packet, size_t parameter_size);

    template <typename T>
    void ConstructPacket(unsigned char *tx_packet, unsigned char device_id, unsigned char instruction,
                         T parameter, unsigned char address);

    template <typename T>
    void insertParameter(unsigned char *array, T value);

    template <typename T>
    T extractParameter(unsigned char *array);

    template <typename T>
    T genericGet(unsigned char id, unsigned short bytes, unsigned short address);

    template <typename T>
    void genericSet(unsigned char id, unsigned short address, T value);



public:
    // CONSTRUCTORS
    DynamixelController(int flow_control_pin);
    // UTILITY
    void begin(long baud_rate);

    void ping(unsigned char id);
    void reboot(unsigned char id);

    void idScanner(unsigned char *id_array, unsigned char device_count);
    void reset_device(unsigned char id);
    void stop_device(unsigned char id);

    // SETTERS
    void setTorqueEnable(unsigned char id, unsigned char value);
    void setPositionGainP(unsigned char id, unsigned short value);
    void setGoalPosition(unsigned char id, unsigned long value);

    // GETTERS
    unsigned short getPositionGainP(unsigned char id);
    unsigned long getGoalPosition(unsigned char id);
    unsigned char getMoving(unsigned char id);
    unsigned long getPresentPosition(unsigned char id);
    unsigned char getTemperature(unsigned char id);
};

#endif //DYNAMIXEL_CONTROL_DYNAMIXEL_CONTROL_H
