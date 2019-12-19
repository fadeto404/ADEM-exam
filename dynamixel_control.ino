#include "dynamixel_control.h"

const unsigned char motor_count = 5;
const unsigned char temperature_limit = 25;

DynamixelController controller = DynamixelController(13);

unsigned char id_array[motor_count];
unsigned char the_chosen_one = 0;
unsigned char present_temperature;
unsigned long start_time;

void setup(){
    Serial.begin(57600);
    controller.begin(57600);
    controller.idScanner(id_array, motor_count);
    for (int i = 0; i < motor_count; i++){
        Serial.println(id_array[i]);
    }
    the_chosen_one = id_array[1];
    Serial.println(the_chosen_one, HEX);
    controller.setTorqueEnable(the_chosen_one, true);
    controller.setPositionGainP(the_chosen_one, 150);
    unsigned long start_time = micros();
}

void loop(){
    if (micros() >= start_time + 100000){
        start_time = micros();
        present_temperature = controller.getTemperature(the_chosen_one);
        if (present_temperature > temperature_limit){
            controller.reset_device(the_chosen_one);
            //controller.stop_device(the_chosen_one);
            //controller.reboot(the_chosen_one);
        }
        Serial.println(present_temperature);
    }
}