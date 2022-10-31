
#include <Arduino.h>        // Core Arduino

#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>



#include <BatterySocData.h> // Contains Configurable parameters

#include <Wire.h>   //  I2C communication SDA,SCL
#include <Ticker.h> //  Tickers which can call repeating functions. Replaces delay() with non-blocking functions

#ifdef ADS1115_SENSOR
#include <Adafruit_ADS1X15.h> //  ADS1115 16-bit ADC
#endif

#ifdef INA219_SENSOR
#include <Adafruit_INA219.h> //  INA218  12-bit current sensing IC using shunt
#endif

#ifdef LCD_16X2_I2C
#include <LiquidCrystal_I2C.h> //  LCD 16x2
#endif

#ifdef LED_7SEG_TM1637_I2C
#include <TM1637Display.h> //  4 Digit 7 segment display I2C
#endif

// #include <ArduinoJson.h>         //  Easy handling of JSON data in C/C++

#ifndef BatterySoc_h
#define BatterySoc_h

// Varialbles
uint8_t lcd_screen_no = 0;

float battery_voltage = BATTERY_VOLTAGE;                    //  Present voltage of battery V
float battery_capacity_ah = BATTERY_CAPACITY_AH;            //  Present Capacity of battery Ah
double battery_capacity_coulomb = BATTERY_CAPACITY_COULOMB; //  Present Capacity of battery Col

// TODO : REMOVE IN PRODUCTION / 2 PART
// TODO : In production it will coulmb left from EEPROM/FRAM
uint32_t battery_capacity_milli_coulomb = BATTERY_CAPACITY_MILLI_COULOMB / 2; // Present Capacity of battery mCol

const uint32_t battery_capacity_milli_coulomb_const = BATTERY_CAPACITY_MILLI_COULOMB; // Present Capacity of battery mCol

float shunt_voltage = 0; // Present voltage across shunt mV

unsigned long last_read_micros = 0;
unsigned long current_read_micros = 0;

bool flag_battery_fully_charged = 0;
bool flag_battery_fully_drained = 0;

// Methods

#ifdef LCD_16X2_I2C

/*Set Message on LCD screen*/
void lcd_set_msg(String message, uint8_t lcd_row = 0, uint8_t lcd_col = 0, bool clear_screen = true, bool align_centre = true);

/*Setup LCD display*/
void setup_lcd_i2c();

#endif

#ifdef LED_7SEG_TM1637_I2C

/*Setup 7 Segment display*/
void setup_7seg_i2c();

#endif

/*Setup push buttons*/
void setup_push_buttons();

#ifdef ANALOG_INPUT_POT

/*Setup Analog Read : For 10K pot , if user is using dial as input*/
void setup_analog_read_pot();

#endif

/*Setup Analog Read : For V-bat (64V)*/
void setup_analog_read_vbat();

#ifdef ADS1115_SENSOR

/*Setup ADS1115*/
void setup_ads1115();

#endif

#ifdef INA219_SENSOR

/*Setup INA219*/
void setup_ina219();

#endif

#ifdef DEBUG_CODE

/*Only for debugging purposes , not for production*/
void print_BatterySocData_h();

#endif

/*Only Execute once while power on*/
void setup();

/*Calculate the mC tranafferd from/to (-/+) battery using shunt_voltage , SHUNT_RESISTANCE_MILLI_OHM ,dt(time between two readings)*/
double calculte_milli_coulombs_transffred();

/*Amount of charge(Coulomb) left in battery*/
void show_coulomb_lcd();

/*Calculate and show Battery percent using 'battery_capacity_coulomb' and 'BATTERY_CAPACITY_COULOMB'*/
void show_battery_percent_lcd();

/*Read the ADC at PIN_ANALOG_BATTERY_VOLTAGE(A0) pin value through voltage divider , to calculate the Voltage of battery */
void analog_read_vbat();

/*Display V bat on 16x 2 LCD , after reading from analog_read_vbat();*/
void show_vbat_lcd();

/*Reading differential voltage between A0 and A1 pin of the ads115 to give voltage drop across shunt resistance*/
void differential_analog_read_ads_vshunt();

/*Display Voltage(mV) drop across SHUNT_RESISTANCE on 16x 2 LCD , after calulating from differential_analog_read_ads_vshunt(); */
void show_vshunt_lcd();

/*Display latest data on LCD , and keep track of the what to display i.e. screens*/
void update_lcd_display();

/*Update the present capacity(C/mC) of battery on the basis of charge transfered  ,and also update the full charged/ full drain flags */
void update_charge_transfer_data();

/*Call all the other methods which will do the caluclations and update the respective data of variables*/
void process_and_update_data();

/*Read all sensor data and update respective variables*/
void read_sensor_data();

/*Update all running Tickers/Timers , and keep their track*/
void ticker_loop();

/*Infinite loop*/
void loop();


int16_t raw_a0_read;
int16_t raw_ads_read;

#endif