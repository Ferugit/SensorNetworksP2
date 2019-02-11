/**
 * Copyright (c) 2017, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdio.h>

#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "events/EventQueue.h"

// Application helpers
#include "trace_helper.h"
#include "lora_radio_helper.h"
#include "mbed.h"

// Measurements
#include "Si7021.h"
#include "definitions.h"

using namespace events;


// Max payload size can be LORAMAC_PHY_MAXPAYLOAD.
// This example only communicates with much shorter messages (<30 bytes).
// If longer messages are used, these buffers must be changed accordingly.
uint8_t tx_buffer[30];
uint8_t rx_buffer[30];


/*
 * Sets up an application dependent transmission timer in ms. Used only when Duty Cycling is off for testing
 */
#define TX_TIMER                        10000

/**
 * Maximum number of events for the event queue.
 * 10 is the safe number for the stack events, however, if application
 * also uses the queue for whatever purposes, this number should be increased.
 */
#define MAX_NUMBER_OF_EVENTS            10

/**
 * Maximum number of retries for CONFIRMED messages before giving up
 */
#define CONFIRMED_MSG_RETRY_COUNTER     3


/**
* This event queue is the global event queue for both the
* application and stack. To conserve memory, the stack is designed to run
* in the same thread as the application and the application is responsible for
* providing an event queue to the stack that will be used for ISR deferment as
* well as application information event queuing.
*/
static EventQueue ev_queue(MAX_NUMBER_OF_EVENTS * EVENTS_EVENT_SIZE);

/**
 * Event handler.
 *
 * This will be passed to the LoRaWAN stack to queue events for the
 * application which in turn drive the application.
 */
static void lora_event_handler(lorawan_event_t event);

/**
 * Constructing Mbed LoRaWANInterface and passing it down the radio object.
 */
static LoRaWANInterface lorawan(radio);

/**
 * Application specific callbacks
 */
static lorawan_app_callbacks_t callbacks;

////Threads definition
extern Thread threadGNSS;

//Code of the extern threads
extern void gnss_thread();

//Values of the sensors
extern float lon, lat; 				//GNSS RX
float temperature, humidity;	//Floats

//TO GET MEASUREMENTS OF SOIL MOSITURE AND LIGH SENSOR
AnalogIn soil_moisture(PA_0);
AnalogIn light_sensor(PA_4);

// Flags
bool flagTHSensor = false;
bool flagGNSSReceiver = true;

// Functions
void int2Bytes(uint8_t bytes_temp[], float float_variable, int index);
void float2Bytes(float float_variable, int index);
void extractElements(uint8_t subArray[], int index);

//RGB Led
BusOut RGB(PB_13, PH_1, PH_0);//Blue, Green and Red: active with low level

//Objects of sensors
Si7021 THsensor;														//Temperature and humidity sensor

/**
 * Entry point for application
 */
int main (void)
{
    // setup tracing
    setup_trace();

    // stores the status of a call to LoRaWAN protocol
    lorawan_status_t retcode;

    // Initialize LoRaWAN stack
    if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
        printf("\r\n LoRa initialization failed! \r\n");
        return -1;
    }

    printf("\r\n Mbed LoRaWANStack initialized \r\n");

    // prepare application callbacks
    callbacks.events = mbed::callback(lora_event_handler);
    lorawan.add_app_callbacks(&callbacks);

    // Set number of retries in case of CONFIRMED messages
    if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER)
                                          != LORAWAN_STATUS_OK) {
        printf("\r\n set_confirmed_msg_retries failed! \r\n\r\n");
        return -1;
    }

    printf("\r\n CONFIRMED message retries : %d \r\n",
           CONFIRMED_MSG_RETRY_COUNTER);

    // Enable adaptive data rate
    if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
        printf("\r\n enable_adaptive_datarate failed! \r\n");
        return -1;
    }

    printf("\r\n Adaptive data  rate (ADR) - Enabled \r\n");
		

    retcode = lorawan.connect();

    if (retcode == LORAWAN_STATUS_OK ||
        retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
    } else {
        printf("\r\n Connection error, code = %d \r\n", retcode);
        return -1;
    }

    printf("\r\n Connection - In Progress ...\r\n");
		
		// Start Threads
		threadGNSS.start(gnss_thread);
		 	
		//Initialization of TH sensor
		if(THsensor.check()){
			flagTHSensor = true;
		}
		
		RGB = ALL_OFF;
		
    // make your event queue dispatching events forever
    ev_queue.dispatch_forever();
		
    return 0;
}


/**
 * Sends a message to the Network Server
 */
static void send_message()
{
    uint16_t packet_len;
    int16_t retcode;
    float sensor_value;

		float2Bytes(lat, 0);//lat
		float2Bytes(lon, 4);//long
	
		float2Bytes(light_sensor*100, 8);
		float2Bytes(soil_moisture*100, 12);
		
		//TEMPERATURE AND HUMIDITY SENSOR
		if(flagTHSensor){
			//Perform the measurement
			if(THsensor.measure()){
				//Get the values of the relative humidity and the temperature 
					temperature   =    THsensor.get_temperature();
					humidity   		=    THsensor.get_humidity();
			}
		}
		
		float2Bytes(temperature, 16);
		float2Bytes(humidity, 20);
		
		//Check if the value is witted right in the buffer
		
		uint8_t recovered_float_lat[4];
		uint8_t recovered_float_long[4];
		uint8_t recovered_float_light[4];
		uint8_t recovered_float_moisture[4];
		uint8_t recovered_float_temp[4];
		uint8_t recovered_float_hum[4];
		extractElements(recovered_float_lat, 0);
		extractElements(recovered_float_long, 4);
		extractElements(recovered_float_light, 8);
		extractElements(recovered_float_moisture, 12);
		extractElements(recovered_float_temp, 16);
		extractElements(recovered_float_hum, 20);
		printf("\r\n Result Latitude: %.5f", *(float*)recovered_float_lat);
		printf("\r\n Result Longitude: %.5f", *(float*)recovered_float_long);
		printf("\r\n Result Light: %.5f", *(float*)recovered_float_light);
		printf("\r\n Result Moisture: %.5f", *(float*)recovered_float_moisture);
		printf("\r\n Result Temperature: %.5f", *(float*)recovered_float_temp);
		printf("\r\n Result Humidity: %.5f", *(float*)recovered_float_hum);
			
    packet_len = 24;
		

    retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, packet_len,
                           MSG_CONFIRMED_FLAG);

    if (retcode < 0) {
        retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("send - WOULD BLOCK\r\n")
                : printf("\r\n send() - Error code %d \r\n", retcode);
        return;
    }

    printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
    memset(tx_buffer, 0, sizeof(tx_buffer));
}

/**
 * Receive a message from the Network Server
 */
static void receive_message()
{
    int16_t retcode;
    retcode = lorawan.receive(MBED_CONF_LORA_APP_PORT, rx_buffer,
                              sizeof(rx_buffer),
                              MSG_CONFIRMED_FLAG|MSG_UNCONFIRMED_FLAG);

    if (retcode < 0) {
        printf("\r\n receive() - Error code %d \r\n", retcode);
        return;
    }

    printf(" Data:");

    for (uint8_t i = 0; i < retcode; i++) {
        printf("%x", rx_buffer[i]);
				if (rx_buffer[i] == 0x52){
				}
    }
		
		switch(rx_buffer[0]){
			case 0x4F://Character O: OFF
				RGB = ALL_OFF; 
				break;
			case 0x47://Character G: GREEN
				RGB = GREEN_ON;
				break;
			case 0x52://Character R: RED
				RGB = RED_ON;
				break;
			case 0x59://Character Y: YELLOW
				RGB = YELLOW_ON;
				break;
			case 0x4D://Character M: MAGENTA
				RGB = MAGENTA_ON;
				break;
			case 0x43://Character C: CYAN
				RGB = CYAN_ON;
				break;
			case 0x57://Character W: WHITE
				RGB = WHITE_ON;
				break;
		}

    printf("\r\n Data Length: %d\r\n", retcode);

    memset(rx_buffer, 0, sizeof(rx_buffer));
}

/**
 * Event handler
 */
static void lora_event_handler(lorawan_event_t event)
{
    switch (event) {
        case CONNECTED:
            printf("\r\n Connection - Successful \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            } else {
                ev_queue.call_every(TX_TIMER, send_message);
            }

            break;
        case DISCONNECTED:
            ev_queue.break_dispatch();
            printf("\r\n Disconnected Successfully \r\n");
            break;
        case TX_DONE:
            printf("\r\n Message Sent to Network Server \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case TX_TIMEOUT:
        case TX_ERROR:
        case TX_CRYPTO_ERROR:
        case TX_SCHEDULING_ERROR:
            printf("\r\n Transmission Error - EventCode = %d \r\n", event);
            // try again
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case RX_DONE:
            printf("\r\n Received message from Network Server \r\n");
            receive_message();
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            printf("\r\n Error in reception - Code = %d \r\n", event);
            break;
        case JOIN_FAILURE:
            printf("\r\n OTAA Failed - Check Keys \r\n");
            break;
        default:
            MBED_ASSERT("Unknown Event");
    }
}

void float2Bytes(float float_variable, int index){ 
  memcpy(tx_buffer+index, ( char*) (&float_variable), 4);
}

void int2Bytes(uint8_t bytes_temp[], int int_variable, int index){ 
  memcpy(bytes_temp+index, ( char*) (&int_variable), 4);
}

void extractElements(uint8_t subArray[], int index)
{
    for (int i = 0; i < 4; i++)
        subArray[i] = tx_buffer[index+i];
}
// EOF
