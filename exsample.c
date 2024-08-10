#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"


#include "ATM90E36.h"


void app_main(void)

{
//Initialise SPI
start_spi();

//Initialise ATM90E36
begin_ATM90E36();



//while loop to keep printing the data
while(1){


//Reterive and print the data   

// Get and print Line Voltage A
double LineVoltageA = GetLineVoltageA();
printf("Line Voltage A: %f\n", LineVoltageA);

// Get and print Line Voltage B
double LineVoltageB = GetLineVoltageB();
printf("Line Voltage B: %f\n", LineVoltageB);

// Get and print Line Voltage C
double LineVoltageC = GetLineVoltageC();
printf("Line Voltage C: %f\n", LineVoltageC);

// Get and print Line Current A
double LineCurrentA = GetLineCurrentA();
printf("Line Current A: %f\n", LineCurrentA);

// Get and print Line Current B
double LineCurrentB = GetLineCurrentB();
printf("Line Current B: %f\n", LineCurrentB);

// Get and print Line Current C
double LineCurrentC = GetLineCurrentC();
printf("Line Current C: %f\n", LineCurrentC);

// Get and print Neutral Line Current
double LineCurrentN = GetLineCurrentN();
printf("Line Current N: %f\n", LineCurrentN);

// Get and print Total Active Power
double TotalActivePower = GetTotalActivePower();
printf("Total Active Power: %f\n", TotalActivePower);

// Get and print Total Reactive Power
double TotalReactivePower = GetTotalReactivePower();
printf("Total Reactive Power: %f\n", TotalReactivePower);

// Get and print Apparent Power A
double ApparentPowerA = GetApparentPowerA();
printf("Apparent Power A: %f\n", ApparentPowerA);

// Get and print Temperature
double Temperature = GetTemperature();
printf("Temperature: %f\n", Temperature);


vTaskDelay(100); 

}
}
