#ifndef SOFTWARE_DEFINES_H
#define SOFTWARE_DEFINES_H

/// Almost all numbers and constants should end up here

/// Motor defines (define JSON_SIZE if more than 128 total)
#define MAX_I2C_MOTORS  64
#define MAX_LEDC_MOTORS 64
#define MAX_MOTORS MAX_I2C_MOTORS + MAX_LEDC_MOTORS

// pwm frequency of pca motors
#define PCA_FREQUENCY 1500 
#define PCA_1 0x40 
#define PCA_2 0x41

/// parameters to drive direct pins at
#define LEDC_FREQUENCY 200
#define LEDC_RESOLUTION 8 // Don't just change this. Reimplemnt the array and scaling too.

/// Temperature controls
#define MAX_TEMP 120.0
#define MIN_TEMP_COOLDOWN 80.0

/// Wireless defines
#define WIRELESS_TICK_MS 7 /*atleast this much time must have passed before next wireless tick*/
#define RADIO_KEEPALIVE_OFF 0
#define RADIO_KEEPALIVE_BALANCED 1
#define RADIO_KEEPALIVE_BALANCED_MS 100
#define RADIO_KEEPALIVE_AGGRESSIVE 2
#define RADIO_KEEPALIVE_AGGRESSIVE_MS 50
#define WIRELESS_DISCONNECT_THRESH_MS 1000 /*assumes we have lost connection after this long*/
#define OSC_MOTOR_CHAR_NUM 4
#define RECIEVE_PORT 1027
#define SEND_PORT 1037
#define MULTICAST_PORT 6868
#define MULTICAST_GROUP 239,0,0,1
#define AP_NAME "Haptics-Connect-To-Me"
#define OTA_PASS "Haptics-OTA"
/// Whether OTA should be disabled 1minute after boot or not.
#define OTA_TIMEOUT true

#define OTA_UPDATE_MS 1000

#define HEARTBEAT_ADDRESS "/hrtbt"
#define PING_ADDRESS "/ping"
#define COMMAND_ADDRESS "/command"
#define MOTOR_ADDRESS "/h"
#define DONOTHING_ADDRESS "/"

// internal (calculated for 64 motors on each)
#define JSON_SIZE 4096
#define NODE_LOCATION_DIGITS 4 
#define MAX_NODE_GROUPS 10

#define CONFIG_VERSION 1

#endif // Software defines