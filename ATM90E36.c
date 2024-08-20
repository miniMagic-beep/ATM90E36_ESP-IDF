// MIT License

// Copyright (c) 2016 whatnick and Ryzee

// Copyright (c) 2024 Mihiran Wickramarathne

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#include "ATM90E36.h"
#include <stdio.h>
#include <string.h>
#include "driver/spi_master.h"


//SPI Pins
//Change these to the pins you use
#define GPIO_MOSI 5
#define GPIO_MISO 6
#define GPIO_SCLK 4
#define GPIO_CS 7

#define WRITE 0 // WRITE SPI
#define READ 1 	// READ SPI



spi_transaction_t t;
spi_device_handle_t spi;   
esp_err_t ret;




void start_spi(void){
spi_host_device_t spi_host = SPI3_HOST;


spi_bus_config_t buscfg;
memset(&buscfg, 0, sizeof(buscfg));
buscfg.mosi_io_num = GPIO_MOSI;
buscfg.miso_io_num = GPIO_MISO;
buscfg.sclk_io_num = GPIO_SCLK;
buscfg.quadwp_io_num = -1;
buscfg.quadhd_io_num = -1;
buscfg.max_transfer_sz = 1;
buscfg.flags = SPICOMMON_BUSFLAG_MASTER ; /*| SPICOMMON_BUSFLAG_IOMUX_PINS ;*/
buscfg.intr_flags = 0;

spi_device_interface_config_t devcfg;
memset(&devcfg, 0, sizeof(devcfg));
devcfg.command_bits = 0;
devcfg.address_bits = 16;
devcfg.dummy_bits = 0;
devcfg.mode = (uint8_t) 3; //SPI_MODE;
devcfg.duty_cycle_pos = 0;
devcfg.cs_ena_pretrans = 0;
devcfg.cs_ena_posttrans = (uint8_t) 0; //CS_ENA_POSTTRANS;
devcfg.clock_speed_hz = 300 * 1000; //SPI_HZ;
devcfg.input_delay_ns = 50; // INPUT_DELAY_NS;
devcfg.spics_io_num = GPIO_CS;
devcfg.flags = SPI_DEVICE_NO_DUMMY ;
devcfg.queue_size = 1;
devcfg.pre_cb = 0;
devcfg.post_cb = 0;
devcfg.input_delay_ns = 400;

ret = spi_bus_initialize(spi_host, &buscfg, 0); // No DMA 
ESP_ERROR_CHECK(ret);
ret =  spi_bus_add_device(spi_host, &devcfg, &spi);
ESP_ERROR_CHECK(ret);


memset(&t, 0, sizeof(t));       
t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA  ; 

t.length = 32;
t.rxlength = 16;  
             

//ret =  spi_device_transmit(spi, &t);
}



unsigned short CommEnergyIC(unsigned char RW, unsigned short address, unsigned short val) 
    {
        //Sets the adress and read and write
        uint16_t addrss;
        uint16_t response;
        uint16_t data;
        if(RW){
            addrss = 0x1<<15 | address; // Read
            data = 0x0;
        }
        else{
            addrss = 0x0<<15 | address;//Write
            data = val;
            t.length = 16;
        }
        spi_device_acquire_bus(spi, portMAX_DELAY);
        
  
        
        *((uint16_t*)t.tx_data) = SPI_SWAP_DATA_TX(data,16); // Writing the data to the TX buffer
        t.addr = addrss;// Writing the address to the address buffer

        
        ret =  spi_device_polling_transmit(spi, &t);// Transmitting the data
        response = SPI_SWAP_DATA_RX(*((uint16_t*)t.rx_data), 16);// Reading the data from the RX buffer
        spi_device_release_bus(spi);
        vTaskDelay(10);
        return response;
    }



/* Parameters Functions*/
/*
- Gets main electrical parameters,
such as: Voltage, Current, Power, Energy,
and Frequency
- Also gets the temperature
*/
// VOLTAGE
double  GetLineVoltageA() {
  unsigned short voltage = CommEnergyIC(READ, UrmsA, 0xFFFF);
  return (double)voltage / 100;
}

double  GetLineVoltageB() {
  unsigned short voltage = CommEnergyIC(READ, UrmsB, 0xFFFF);
  return (double)voltage / 100;
}

double GetLineVoltageC() {
  unsigned short voltage = CommEnergyIC(READ, UrmsC, 0xFFFF);
  return (double)voltage / 100;
}

// CURRENT
double GetLineCurrentA() {
  unsigned short current = CommEnergyIC(READ, IrmsA, 0xFFFF);
  return (double)current / 1000;
}
double GetLineCurrentB() {
  unsigned short current = CommEnergyIC(READ, IrmsB, 0xFFFF);
  return (double)current / 1000;
}
double GetLineCurrentC() {
  unsigned short current = CommEnergyIC(READ, IrmsC, 0xFFFF);
  return (double)current / 1000;
}
double GetLineCurrentN() {
  unsigned short current = CommEnergyIC(READ, IrmsN0, 0xFFFF);
  return (double)current / 1000;
}

// ACTIVE POWER
double GetActivePowerA() {
  signed short apower = (signed short) CommEnergyIC(READ, PmeanA, 0xFFFF); 
  if (apower & 0x8000) {
    apower= (apower & 0x7FFF) * -1;
  }
  return (double)apower / 1000;
}
double GetActivePowerB() {
  signed short apower = (signed short) CommEnergyIC(READ, PmeanB, 0xFFFF); 
  if (apower & 0x8000) {
    apower= (apower & 0x7FFF) * -1;
  }
  return (double)apower / 1000;
}
double GetActivePowerC() {
  signed short apower = (signed short) CommEnergyIC(READ, PmeanC, 0xFFFF); 
  if (apower & 0x8000) {
    apower= (apower & 0x7FFF) * -1;
  }
  return (double)apower / 1000;
}
double GetTotalActivePower() {
  signed short apower = (signed short) CommEnergyIC(READ, PmeanT, 0xFFFF); 
  if (apower & 0x8000) {
    apower= (apower & 0x7FFF) * -1;
  }
  return (double)apower / 250;
}

// REACTIVE POWER
double GetReactivePowerA() {
  signed short apower = (signed short) CommEnergyIC(READ, QmeanA, 0xFFFF); 
  if (apower & 0x8000) {
    apower= (apower & 0x7FFF) * -1;
  }
  return (double)apower / 1000;
}
double GetReactivePowerB() {
  signed short apower = (signed short) CommEnergyIC(READ, QmeanB, 0xFFFF); 
  if (apower & 0x8000) {
    apower= (apower & 0x7FFF) * -1;
  }
  return (double)apower / 1000;
}
double GetReactivePowerC() {
  signed short apower = (signed short) CommEnergyIC(READ, QmeanC, 0xFFFF);
  if (apower & 0x8000) {
    apower= (apower & 0x7FFF) * -1;
  }
  return (double)apower / 1000;
}
double GetTotalReactivePower() {
  signed short apower = (signed short) CommEnergyIC(READ, QmeanT, 0xFFFF); 
  if (apower & 0x8000) {
    apower= (apower & 0x7FFF) * -1;
  }
  return (double)apower / 250;
}

// APPARENT POWER
double GetApparentPowerA() {
  signed short apower = (signed short) CommEnergyIC(READ, SmeanA, 0xFFFF); 
  if (apower & 0x8000) {
    apower= (apower & 0x7FFF) * -1;
  }
  return (double)apower / 1000;
}
double GetApparentPowerB() {
 signed short apower = (signed short) CommEnergyIC(READ, SmeanB, 0xFFFF);
  if (apower & 0x8000) {
    apower= (apower & 0x7FFF) * -1;
  }
  return (double)apower / 1000;
}
double GetApparentPowerC() {
  signed short apower = (signed short) CommEnergyIC(READ, SmeanC, 0xFFFF); 
  if (apower & 0x8000) {
    apower= (apower & 0x7FFF) * -1;
  }
  return (double)apower / 1000;
}
double GetTotalApparentPower() {
  signed short apower = (signed short) CommEnergyIC(READ, SmeanT, 0xFFFF); 
  if (apower & 0x8000) {
    apower= (apower & 0x7FFF) * -1;
  }
  return (double)apower / 250;
}

// FREQUENCY
double GetFrequency() {
  unsigned short freq = CommEnergyIC(READ, Freq, 0xFFFF);
  return (double)freq / 100;
}

// POWER FACTOR
double GetPowerFactorA() {
  short pf = (short) CommEnergyIC(READ, PFmeanA, 0xFFFF); 
  //if negative
  if (pf & 0x8000) {
    pf = (pf & 0x7FFF) * -1;
  }
  return (double)pf / 1000;
}
double GetPowerFactorB() {
  short pf = (short) CommEnergyIC(READ, PFmeanB, 0xFFFF); 
  if (pf & 0x8000) {
    pf = (pf & 0x7FFF) * -1;
  }
  return (double)pf / 1000;
}
double GetPowerFactorC() {
  short pf = (short) CommEnergyIC(READ, PFmeanC, 0xFFFF); 
  //if negative
  if (pf & 0x8000) {
    pf = (pf & 0x7FFF) * -1;
  }
  return (double)pf / 1000;
}
double GetTotalPowerFactor() {
  short pf = (short) CommEnergyIC(READ, PFmeanT, 0xFFFF); 
  //if negative
  if (pf & 0x8000) {
    pf = (pf & 0x7FFF) * -1;
  }
  return (double)pf / 1000;
}

// PHASE ANGLE
double GetPhaseA() {
  signed short apower = (signed short) CommEnergyIC(READ, PAngleA, 0xFFFF);
  return (double)apower / 10;
}
double GetPhaseB() {
  signed short apower = (signed short) CommEnergyIC(READ, PAngleB, 0xFFFF);
  return (double)apower / 10;
}
double GetPhaseC() {
  signed short apower = (signed short) CommEnergyIC(READ, PAngleC, 0xFFFF);
  return (double)apower / 10;
}

// TEMPERATURE
double GetTemperature() {
  short int apower = (short int) CommEnergyIC(READ, Temp, 0xFFFF); 
  return (double)apower;
}


// /* Gets the Register Value if Desired */
// // REGISTER
// unsigned short GetValueRegister(unsigned short registerRead) {
//   return (CommEnergyIC(READ, registerRead, 0xFFFF)); //returns value register
// }

// ENERGY MEASUREMENT
double GetImportEnergy() {
  unsigned short ienergyT = CommEnergyIC(READ, APenergyT, 0xFFFF);
  // unsigned short ienergyA = CommEnergyIC(READ, APenergyA, 0xFFFF);
  // unsigned short ienergyB = CommEnergyIC(READ, APenergyB, 0xFFFF);
  // unsigned short ienergyC = CommEnergyIC(READ, APenergyC, 0xFFFF);

  // unsigned short renergyT = CommEnergyIC(READ, RPenergyT, 0xFFFF);
  // unsigned short renergyA = CommEnergyIC(READ, RPenergyA, 0xFFFF);
  // unsigned short renergyB = CommEnergyIC(READ, RPenergyB, 0xFFFF);
  // unsigned short renergyC = CommEnergyIC(READ, RPenergyC, 0xFFFF);

  // unsigned short senergyT = CommEnergyIC(READ, SAenergyT, 0xFFFF);
  // unsigned short senergyA = CommEnergyIC(READ, SenergyA, 0xFFFF);
  // unsigned short senergyB = CommEnergyIC(READ, SenergyB, 0xFFFF);
  // unsigned short senergyC = CommEnergyIC(READ, SenergyC, 0xFFFF);

  return (double)ienergyT / 100 / 3200; //returns kWh
}

// double GetExportEnergy() {

//   unsigned short eenergyT = CommEnergyIC(READ, ANenergyT, 0xFFFF);
//   // unsigned short eenergyA = CommEnergyIC(READ, ANenergyA, 0xFFFF);
//   // unsigned short eenergyB = CommEnergyIC(READ, ANenergyB, 0xFFFF);
//   // unsigned short eenergyC = CommEnergyIC(READ, ANenergyC, 0xFFFF);

//   // unsigned short reenergyT = CommEnergyIC(READ, RNenergyT, 0xFFFF);
//   // unsigned short reenergyA = CommEnergyIC(READ, RNenergyA, 0xFFFF);
//   // unsigned short reenergyB = CommEnergyIC(READ, RNenergyB, 0xFFFF);
//   // unsigned short reenergyC = CommEnergyIC(READ, RNenergyC, 0xFFFF);

//   return (double)eenergyT / 100 / 3200; //returns kWh 
// }

// /* System Status Registers */
// unsigned short GetSysStatus0() {    
//   return CommEnergyIC(READ, SysStatus0, 0xFFFF);
// }
// unsigned short GetSysStatus1() {
//   return CommEnergyIC(READ, SysStatus1, 0xFFFF);
// }
// unsigned short  GetMeterStatus0() {
//   return CommEnergyIC(READ, EnStatus0, 0xFFFF);
// }
// unsigned short  GetMeterStatus1() {
//   return CommEnergyIC(READ, EnStatus1, 0xFFFF);
// }




// /* BEGIN FUNCTION */

void begin_ATM90E36(void)
{  

  CommEnergyIC(WRITE, SoftReset, 0x789A);   // Perform soft reset
  CommEnergyIC(WRITE, FuncEn0, 0x0000);     // Voltage sag
  CommEnergyIC(WRITE, FuncEn1, 0x0000);     // Voltage sag
  CommEnergyIC(WRITE, SagTh, 0x0001);       // Voltage sag threshold

  /* SagTh = Vth * 100 * sqrt(2) / (2 * Ugain / 32768) */
  
  //Set metering config values (CONFIG)
  CommEnergyIC(WRITE, ConfigStart, 0x5678); // Metering calibration startup 
  CommEnergyIC(WRITE, PLconstH, 0x0861);    // PL Constant MSB (default)
  CommEnergyIC(WRITE, PLconstL, 0xC468);    // PL Constant LSB (default)
  CommEnergyIC(WRITE, MMode0, 0x1087);      // Mode Config (60 Hz, 3P4W)
  CommEnergyIC(WRITE, MMode1, 0x1500);      // 0x5555 (x2) // 0x0000 (1x)
  CommEnergyIC(WRITE, PStartTh, 0x0000);    // Active Startup Power Threshold
  CommEnergyIC(WRITE, QStartTh, 0x0000);    // Reactive Startup Power Threshold
  CommEnergyIC(WRITE, SStartTh, 0x0000);    // Apparent Startup Power Threshold
  CommEnergyIC(WRITE, PPhaseTh, 0x0000);    // Active Phase Threshold
  CommEnergyIC(WRITE, QPhaseTh, 0x0000);    // Reactive Phase Threshold
  CommEnergyIC(WRITE, SPhaseTh, 0x0000);    // Apparent  Phase Threshold
  CommEnergyIC(WRITE, CSZero, 0x4741);      // Checksum 0
  
  //Set metering calibration values (CALIBRATION)
  CommEnergyIC(WRITE, CalStart, 0x5678);    // Metering calibration startup 
  CommEnergyIC(WRITE, GainA, 0x0000);       // Line calibration gain
  CommEnergyIC(WRITE, PhiA, 0x0000);        // Line calibration angle
  CommEnergyIC(WRITE, GainB, 0x0000);       // Line calibration gain
  CommEnergyIC(WRITE, PhiB, 0x0000);        // Line calibration angle
  CommEnergyIC(WRITE, GainC, 0x0000);       // Line calibration gain
  CommEnergyIC(WRITE, PhiC, 0x0000);        // Line calibration angle
  CommEnergyIC(WRITE, PoffsetA, 0x0000);    // A line active power offset
  CommEnergyIC(WRITE, QoffsetA, 0x0000);    // A line reactive power offset
  CommEnergyIC(WRITE, PoffsetB, 0x0000);    // B line active power offset
  CommEnergyIC(WRITE, QoffsetB, 0x0000);    // B line reactive power offset
  CommEnergyIC(WRITE, PoffsetC, 0x0000);    // C line active power offset
  CommEnergyIC(WRITE, QoffsetC, 0x0000);    // C line reactive power offset
  CommEnergyIC(WRITE, CSOne, 0x0000);       // Checksum 1
  
  //Set metering calibration values (HARMONIC)
  CommEnergyIC(WRITE, HarmStart, 0x5678);   // Metering calibration startup 
  CommEnergyIC(WRITE, POffsetAF, 0x0000);   // A Fund. active power offset
  CommEnergyIC(WRITE, POffsetBF, 0x0000);   // B Fund. active power offset
  CommEnergyIC(WRITE, POffsetCF, 0x0000);   // C Fund. active power offset
  CommEnergyIC(WRITE, PGainAF, 0x0000);     // A Fund. active power gain
  CommEnergyIC(WRITE, PGainBF, 0x0000);     // B Fund. active power gain
  CommEnergyIC(WRITE, PGainCF, 0x0000);     // C Fund. active power gain
  CommEnergyIC(WRITE, CSTwo, 0x0000);       // Checksum 2 

  //Set measurement calibration values (ADJUST)
  CommEnergyIC(WRITE, AdjStart, 0x5678);    // Measurement calibration
  CommEnergyIC(WRITE, UgainA, 0x0002);      // A SVoltage rms gain
  CommEnergyIC(WRITE, IgainA, 0xFD7F);      // A line current gain
  CommEnergyIC(WRITE, UoffsetA, 0x0000);    // A Voltage offset
  CommEnergyIC(WRITE, IoffsetA, 0x0000);    // A line current offset
  CommEnergyIC(WRITE, UgainB, 0x0002);      // B Voltage rms gain
  CommEnergyIC(WRITE, IgainB, 0xFD7F);      // B line current gain
  CommEnergyIC(WRITE, UoffsetB, 0x0000);    // B Voltage offset
  CommEnergyIC(WRITE, IoffsetB, 0x0000);    // B line current offset
  CommEnergyIC(WRITE, UgainC, 0x0002);      // C Voltage rms gain
  CommEnergyIC(WRITE, IgainC, 0xFD7F);      // C line current gain
  CommEnergyIC(WRITE, UoffsetC, 0x0000);    // C Voltage offset
  CommEnergyIC(WRITE, IoffsetC, 0x0000);    // C line current offset
  CommEnergyIC(WRITE, IgainN, 0xFD7F);      // C line current gain
  CommEnergyIC(WRITE, CSThree, 0x02F6);     // Checksum 3

  // Done with the configuration
  //Uncomment these if you want to reset the IC
  //CommEnergyIC(WRITE, ConfigStart, 0x5678);
  // CommEnergyIC(WRITE, CalStart, 0x5678);    // 0x6886 //0x5678 //8765);
  // CommEnergyIC(WRITE, HarmStart, 0x5678);   // 0x6886 //0x5678 //8765);    
  // CommEnergyIC(WRITE, AdjStart, 0x5678);    // 0x6886 //0x5678 //8765);  

  // CommEnergyIC(WRITE, SoftReset, 0x789A);   // Perform soft reset  
}
