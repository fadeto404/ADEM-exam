#include "dynamixel_control.h"

// CONSTRUCTOR
DynamixelController::DynamixelController(int flow_control_pin) {
    _flow_control_pin = flow_control_pin;
    pinMode(_flow_control_pin, OUTPUT);
}

// PUBLIC
void DynamixelController::begin(long baud_rate) {
    Serial1.begin(baud_rate);
    _serialport = &Serial1;
}

void DynamixelController::idScanner(unsigned char *id_array, unsigned char device_count) {
    ping(0xFE);
    for (int i = 0; i < device_count; ++i) {
        status_packet_info status = ReceiveStatusPacket();
        id_array[i] = status.id;
    }
}

void DynamixelController::stop_device(unsigned char id) {
    unsigned long presentPosition = getPresentPosition(id);
    setGoalPosition(id, presentPosition);
}

void DynamixelController::reset_device(unsigned char id) {
    setGoalPosition(id, 2048);
    delay(50);
    while (getMoving(id)){
        delay(10);
    }
    setTorqueEnable(id, false);
}

void DynamixelController::ping(unsigned char id) {
    unsigned char tx_packet[10];
    ConstructPacket(tx_packet, id, 0x01, 0x00, 0x00);
    TransmitPacket(tx_packet);
}

void DynamixelController::reboot(unsigned char id) {
    unsigned char tx_packet[10];
    ConstructPacket(tx_packet, id, 0x08, 0x00, 0x00);
    TransmitPacket(tx_packet);
}

void DynamixelController::setTorqueEnable(unsigned char id, unsigned char value) {
    genericSet<unsigned char>(id, 0x40, value);
}

void DynamixelController::setPositionGainP(unsigned char id, unsigned short value) {
    genericSet<unsigned short>(id, 0x54, value);
}

void DynamixelController::setGoalPosition(unsigned char id, unsigned long value) {
    genericSet<unsigned long>(id, 0x74, value);
}

unsigned short DynamixelController::getPositionGainP(unsigned char id){
    return genericGet<unsigned short>(id, 2, 84);
}

unsigned long DynamixelController::getGoalPosition(unsigned char id){
    return genericGet<unsigned long>(id, 4, 116);
}

unsigned char DynamixelController::getMoving(unsigned char id){
    return genericGet<unsigned char>(id, 1, 122);
}

unsigned long DynamixelController::getPresentPosition(unsigned char id){
    return genericGet<unsigned long>(id, 4, 132);
}

unsigned char DynamixelController::getTemperature(unsigned char id){
    return genericGet<unsigned char>(id, 1, 146);
}


// PRIVATE METHODS
unsigned short DynamixelController::CreateLength(unsigned char *tx_packet, size_t parameter_size) {
    unsigned short packet_length = parameter_size + 3; // length(instruction + crc) = 3
    unsigned char length1 = ((unsigned char) packet_length & 0xFF);
    unsigned char length2;

    if (packet_length > 0xFF) {
        length2 = (packet_length >> 8) & 0xFF;
    } else {
        length2 = 0x00;
    }

    tx_packet[5] = length1;
    tx_packet[6] = length2;

    return packet_length;
}
template <typename T>
void DynamixelController::ConstructPacket(unsigned char *tx_packet,
                                          unsigned char device_id,
                                          unsigned char instruction,
                                          T parameter,
                                          unsigned char address) {
    tx_packet[0] = 0xFF;
    tx_packet[1] = 0xFF;
    tx_packet[2] = 0xFD;
    tx_packet[3] = 0x00;
    tx_packet[4] = device_id;
    tx_packet[7] = instruction;
    unsigned short packet_length;
    if (instruction == 1 or instruction == 8) {
        packet_length = CreateLength(tx_packet, 0);
    } else {
        tx_packet[8] = address & 0x00FF;
        tx_packet[9] = address >> 8 & 0x00FF;
        insertParameter<T>(tx_packet, parameter);
        packet_length = CreateLength(tx_packet, sizeof(T)+2); // size of parameter + address = sizeof(T)+2
    }

    CreateCRC(tx_packet, packet_length+5); // length(header + id + length_field - crc) = 5
    /*// Print tx-packet for debugging
    for (int i = 0; i < (tx_packet[6] << 8) + tx_packet[5] + 7; ++i) {
        Serial.print(tx_packet[i], HEX);
        Serial.print(" ");
    }
    Serial.println();*/
}

void DynamixelController::TransmitPacket(unsigned char *tx_packet) {
    digitalWrite(_flow_control_pin, HIGH);
    unsigned short bytes_in_packet = (tx_packet[6] << 8) + tx_packet[5] + 7; // length(header + id + length_field) = 7

    for (int i = 0; i < bytes_in_packet; i++) {
        _serialport->write(tx_packet[i]);
    }
    _serialport->flush();
    digitalWrite(_flow_control_pin, LOW);
}

DynamixelController::status_packet_info DynamixelController::ReceiveStatusPacket() {
    unsigned long start_time = micros();
    status_packet_info status;
    status.id = 0xFF;
    status.error = 0x00;

    while (micros()<start_time+5000){
        if (_serialport->available() >= 7) {

            // Get rid of the header
            unsigned long hdr = 0;
            for (int l = 0; l < 4; ++l) {
                hdr = (hdr << 8) + _serialport->read();
            }
            if (hdr != 0xFFFFFD00){
                status.error = 0x08;
                return status;
            }

            // Get ID, length
            unsigned char id = _serialport->read();
            unsigned char l1 = _serialport->read();
            unsigned char l2 = _serialport->read();
            unsigned short packet_length = l1 + (l2 << 8);
            status.id = id;

            // Recreate RX-packet
            unsigned char rx_packet[packet_length+7];
            rx_packet[0] = 0xFF;
            rx_packet[1] = 0xFF;
            rx_packet[2] = 0xFD;
            rx_packet[3] = 0x00;
            rx_packet[4] = id;
            rx_packet[5] = l1;
            rx_packet[6] = l2;

            while (_serialport->available() < packet_length) {
                // TODO: Create a timeout
            }

            // Populate the rest of the rx_packet with instr, err, params, crc
            for (int j = 0; j < packet_length; ++j) {
                rx_packet[j + 7] = _serialport->read();
            }

            // Set error in return value
            status.error = rx_packet[8];
            if (status.error != 0x00) return status;

            // Extract parameters
            for (int k = 0; k < packet_length - 4; ++k) {
                status.parameters[k] = rx_packet[9 + k];
            }

            unsigned short calc_crc = update_crc(0, rx_packet, packet_length+5);
            if ((rx_packet[packet_length+5] != (calc_crc & 0xFF)) | (rx_packet[packet_length+6] != ((calc_crc >> 8) & 0xFF))){
                status.error = 0x03; // CRC error
            }
            /*//Print rx_packet (for debugging)
            for (int l = 0; l < packet_length+7; ++l) {
                Serial.print(rx_packet[l], HEX);
                Serial.print(" ");
            }
            Serial.println();
            Serial.print("Error: ");
            Serial.println(status.error, HEX);*/

            return status;
        }
    }
    status.error = 0x09; // TIMEOUT ERROR
    return status;
}

unsigned short DynamixelController::update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size) {
    unsigned short i, j;
    unsigned short crc_table[256] = {
            0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
            0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
            0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
            0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
            0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
            0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
            0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
            0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
            0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
            0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
            0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
            0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
            0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
            0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
            0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
            0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
            0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
            0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
            0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
            0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
            0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
            0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
            0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
            0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
            0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
            0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
            0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
            0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
            0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
            0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
            0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
            0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for (j = 0; j < data_blk_size; j++) {
        i = ((unsigned short) (crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}

void DynamixelController::CreateCRC(unsigned char *tx_packet, unsigned short blk_size) {
    unsigned short cal_crc = update_crc(0, tx_packet, blk_size);
    tx_packet[blk_size] = (cal_crc & 0x00FF);
    tx_packet[blk_size + 1] = (cal_crc >> 8) & 0x00FF;
}

template <typename T>
T DynamixelController::extractParameter(unsigned char *array) {
    T value = 0;
    for (int i = 0; i < sizeof(T); ++i) {
        value += ((T) array[i] << i*8) & ((T) 0xFF << i*8);
    }
    return value;
}

template <typename T>
void DynamixelController::insertParameter(unsigned char *array, T value) {
    for (int i = 0; i < sizeof(T); ++i) {
        array[i+10] = value >> i*8 & ((T) 0xFF);
    }
}

template <typename T>
T DynamixelController::genericGet(unsigned char id, unsigned short bytes, unsigned short address) {
    unsigned char tx_packet[14];

    ConstructPacket<unsigned short>(tx_packet, id, 0x02, sizeof(T), address);
    TransmitPacket(tx_packet);

    status_packet_info status = ReceiveStatusPacket();
    //Serial.write(status.error); // For debugging
    T receivedData = (T) extractParameter<T>(status.parameters);

    return receivedData;
}

template <typename T>
void DynamixelController::genericSet(unsigned char id, unsigned short address, T value) {
    unsigned char tx_packet[10 + sizeof(T)];

    ConstructPacket<T>(tx_packet, id, 0x03, value, address);
    TransmitPacket(tx_packet);
    ReceiveStatusPacket();
}
