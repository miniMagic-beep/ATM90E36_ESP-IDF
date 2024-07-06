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






#define ATM90E36_h

#define WRITE 0 // WRITE SPI
#define READ 1 	// READ SPI
#define DEBUG_SERIAL

/* STATUS REGISTERS */
#define SoftReset 0x00 		// Software Reset
#define SysStatus0 0x01 	// System Status0
#define SysStatus1 0x02 	// System Status1
#define FuncEn0 0x03 		// Function Enable0
#define FuncEn1 0x04 		// Function Enable1
#define ZXConfig 0x07 		// Zero-Crossing Config
#define SagTh 0x08 			// Voltage Sag Th
#define PhaseLossTh 0x09 	// Voltage Phase Losing Th
#define INWarnTh0 0x0A 		// N Current Line Th
#define INWarnTh1 0x0B 		// Voltage ADC Th
#define THDNUTh 0x0C		// Voltage THD Th
#define THDNITh 0x0D		// Current THD Th
#define DMACtrl 0x0E		// DMA Int. Control 
#define LastSPIData 0x0F 	// Last Read/Write SPI Value

/* LOW POWER MODE REGISTERS - NOT USED */
#define DetectCtrl 0x10
#define DetectTh1 0x11
#define DetectTh2 0x12
#define DetectTh3 0x13
#define	PMOffsetA 0x14
#define	PMOffsetB 0x15
#define	PMOffsetC 0x16
#define	PMPGA 0x17
#define	PMIrmsA 0x18
#define	PMIrmsB 0x19
#define	PMIrmsC 0x1A
#define	PMConfig 0x10B
#define	PMAvgSamples 0x1C
#define PMIrmsLSB 0x1D

/* CONFIGURATION REGISTERS */
#define ConfigStart 0x30 	// Configuration Start
#define PLconstH 0x31 		// High Word of PL_Constant
#define PLconstL 0x32 		// Low Word of PL_Constant
#define MMode0 0x33 		// Metering Mode Config
#define MMode1 0x34 		// Metering Mode Config
#define PStartTh 0x35 		// Startup Power Th (P)
#define QStartTh 0x36 		// Startup Power Th (Q)
#define SStartTh 0x37		// Startup Power Th (S)
#define PPhaseTh 0x38 		// Startup Power Accum Th (P)
#define QPhaseTh 0x39		// Startup Power Accum Th (Q)
#define SPhaseTh 0x3A		// Startup Power Accum Th (S)
#define CSZero 0x3B			// Checksum 0

/* CALIBRATION REGISTERS */
#define CalStart 0x40 		// Cal Start
#define PoffsetA 0x41 		// A Line Power Offset (P)
#define QoffsetA 0x42 		// A Line Power Offset (Q)
#define PoffsetB 0x43 		// B Line Power Offset (P)
#define QoffsetB 0x44 		// B Line Power Offset (Q)
#define PoffsetC 0x45 		// C Line Power Offset (P)
#define QoffsetC 0x46 		// C Line Power Offset (Q)
#define GainA 0x47 			// A Line Calibration Gain
#define PhiA 0x48  			// A Line Calibration Angle
#define GainB 0x49 			// B Line Calibration Gain
#define PhiB 0x4A  			// B Line Calibration Angle
#define GainC 0x4B 			// C Line Calibration Gain
#define PhiC 0x4C  			// C Line Calibration Angle
#define CSOne 0x4D 			// Checksum 1

/* HARMONIC & ENERGY REGISTERS */
#define HarmStart 0x50		// Harmonic Cal Start
#define	POffsetAF 0x51		// A Fund Power Offset (P)
#define POffsetBF 0x52		// B Fund Power Offset (P)
#define	POffsetCF 0x53		// C Fund Power Offset (P)
#define PGainAF	0x54		// A Fund Power Gain (P)
#define	PGainBF	0x55		// B Fund Power Gain (P)
#define PGainCF	0x56		// C Fund Power Gain (P)
#define CSTwo 0x57 			// Checksum 2

/* MEASUREMENT CALIBRATION REGISTERS */
#define AdjStart 0x60 		// Measurement Cal Start
#define UgainA 0x61 		// A Voltage RMS Gain
#define IgainA 0x62 		// A Current RMS Gain
#define UoffsetA 0x63 		// A Voltage Offset
#define IoffsetA 0x64 		// A Current Offset
#define UgainB 0x65 		// B Voltage RMS Gain
#define IgainB 0x66 		// B Current RMS Gain
#define UoffsetB 0x67 		// B Voltage Offset
#define IoffsetB 0x68 		// B Current Offset
#define UgainC 0x69 		// C Voltage RMS Gain
#define IgainC 0x6A 		// C Current RMS Gain
#define UoffsetC 0x6B 		// C Voltage Offset
#define IoffsetC 0x6C 		// C Current Offset
#define IgainN 0x6D 		// N Current Gain
#define IoffsetN 0x6E 		// N Current Offset
#define CSThree 0x6F 		// Checksum 3

/* ENERGY REGISTERS */
#define APenergyT 0x80 		// Total Forward Active	
#define APenergyA 0x81 		// A Forward Active
#define APenergyB 0x82 		// B Forward Active
#define APenergyC 0x83 		// C Forward Active
#define ANenergyT 0x84 		// Total Reverse Active	
#define ANenergyA 0x85 		// A Reverse Active
#define ANenergyB 0x86 		// B Reverse Active
#define ANenergyC 0x87 		// C Reverse Active
#define RPenergyT 0x88 		// Total Forward Reactive
#define RPenergyA 0x89 		// A Forward Reactive
#define RPenergyB 0x8A 		// B Forward Reactive
#define RPenergyC 0x8B 		// C Forward Reactive
#define RNenergyT 0x8C 		// Total Reverse Reactive
#define RNenergyA 0x8D 		// A Reverse Reactive
#define RNenergyB 0x8E 		// B Reverse Reactive
#define RNenergyC 0x8F 		// C Reverse Reactive

#define SAenergyT 0x90 		// Total Apparent Energy
#define SenergyA 0x91  		// A Apparent Energy		
#define SenergyB 0x92 		// B Apparent Energy
#define SenergyC 0x93 		// C Apparent Energy
#define SVenergyT 0x94 		// Total Apparent Energy (Arit)

#define EnStatus0 0x95 		// Metering Status 0
#define EnStatus1 0x96 		// Metering Status 1
///////////////// 0x97	    // Reserved Register 
#define SVmeanT 0x98  		// Total Apparent Energy (Vect)
#define SVmeanTLSB 0x99		// LSB of Vector Sum

/* FUNDAMENTAL / HARMONIC ENERGY REGISTERS */
#define APenergyTF 0xA0 	// Total Forward Fund. Energy
#define	APenergyAF 0xA1 	// A Forward Fund. Energy
#define APenergyBF 0xA2 	// B Forward Fund. Energy
#define APenergyCF 0xA3 	// C Forward Fund. Energy
#define ANenergyTF 0xA4  	// Total Reverse Fund Energy
#define ANenergyAF 0xA5 	// A Reverse Fund. Energy
#define ANenergyBF 0xA6 	// B Reverse Fund. Energy 
#define ANenergyCF 0xA7 	// C Reverse Fund. Energy
#define APenergyTH 0xA8 	// Total Forward Harm. Energy
#define APenergyAH 0xA9 	// A Forward Harm. Energy
#define	APenergyBH 0xAA 	// B Forward Harm. Energy
#define APenergyCH 0xAB 	// C Forward Harm. Energy
#define ANenergyTH 0xAC 	// Total Reverse Harm. Energy
#define ANenergyAH 0xAD  	// A Reverse Harm. Energy
#define ANenergyBH 0xAE  	// B Reverse Harm. Energy
#define ANenergyCH 0xAF  	// C Reverse Harm. Energy

/* POWER & P.F. REGISTERS */
#define PmeanT 0xB0 		// Total Mean Power (P)
#define PmeanA 0xB1 		// A Mean Power (P)
#define PmeanB 0xB2 		// B Mean Power (P)
#define PmeanC 0xB3 		// C Mean Power (P)
#define QmeanT 0xB4 		// Total Mean Power (Q)
#define QmeanA 0xB5 		// A Mean Power (Q)
#define QmeanB 0xB6 		// B Mean Power (Q)
#define QmeanC 0xB7 		// C Mean Power (Q)
#define SmeanT 0xB8 		// Total Mean Power (S)
#define SmeanA 0xB9 		// A Mean Power (S)
#define SmeanB 0xBA 		// B Mean Power (S)
#define SmeanC 0xBB 		// C Mean Power (S)
#define PFmeanT 0xBC 	 	// Mean Power Factor
#define PFmeanA 0xBD 		// A Power Factor
#define PFmeanB 0xBE 		// B Power Factor
#define PFmeanC 0xBF 		// C Power Factor

#define PmeanTLSB 0xC0 		// Lower Word (Tot. Act. Power)
#define	PmeanALSB 0xC1 		// Lower Word (A Act. Power)
#define PmeanBLSB 0xC2 		// Lower Word (B Act. Power)
#define PmeanCLSB 0xC3 		// Lower Word (C Act. Power)
#define QmeanTLSB 0xC4 		// Lower Word (Tot. React. Power)
#define QmeanALSB 0xC5 		// Lower Word (A React. Power)
#define QmeanBLSB 0xC6 		// Lower Word (B React. Power)
#define QmeanCLSB 0xC7 		// Lower Word (C React. Power)
#define SAmeanTLSB 0xC8 	// Lower Word (Tot. App. Power)
#define	SmeanALSB 0xC9 		// Lower Word (A App. Power)
#define SmeanBLSB 0xCA 		// Lower Word (B App. Power)
#define SmeanCLSB 0xCB 		// Lower Word (C App. Power)

/* FUND/HARM POWER & V/I RMS REGISTERS */
#define PmeanTF 0xD0 		// Total Active Fund. Power
#define PmeanAF 0xD1 		// A Active Fund. Power
#define PmeanBF 0xD2 		// B Active Fund. Power
#define PmeanCF 0xD3 		// C Active Fund. Power
#define PmeanTH 0xD4 		// Total Active Harm. Power
#define	PmeanAH 0xD5 		// A Active Harm. Power
#define PmeanBH 0xD6 		// B Active Harm. Power
#define PmeanCH 0xD7 		// C Active Harm. Power
#define IrmsN1 0xD8 		// N Sampled Current
#define UrmsA 0xD9 			// A RMS Voltage
#define UrmsB 0xDA 			// B RMS Voltage
#define UrmsC 0xDB 			// C RMS Voltage
#define IrmsN0 0xDC 		// N Calculated Current (USE)
#define IrmsA 0xDD 			// A RMS Current
#define IrmsB 0xDE 			// B RMS Current
#define IrmsC 0xDF 			// C RMS Current

#define PmeanTFLSB 0xE0		// Lower Word (Tot. Act. Fund. Power)
#define PmeanAFLSB 0xE1		// Lower Word (A Act. Fund. Power) 
#define PmeanBFLSB 0xE2		// Lower Word (B Act. Fund. Power)
#define PmeanCFLSB 0xE3		// Lower Word (C Act. Fund. Power)
#define PmeanTHLSB 0xE4		// Lower Word (Tot. Act. Harm. Power)
#define PmeanAHLSB 0xE5		// Lower Word (A Act. Harm. Power)
#define PmeanBHLSB 0xE6		// Lower Word (B Act. Harm. Power)
#define PmeanCHLSB 0xE7		// Lower Word (C Act. Harm. Power)
///////////////// 0xE8	    // Reserved Register 
#define UrmsALSB 0xE9		// Lower Word (A RMS Voltage)
#define UrmsBLSB 0xEA		// Lower Word (B RMS Voltage)
#define UrmsCLSB 0xEB		// Lower Word (C RMS Voltage)
///////////////// 0xEC	    // Reserved Register	
#define IrmsALSB 0xED		// Lower Word (A RMS Current)
#define IrmsBLSB 0xEE		// Lower Word (B RMS Current)
#define IrmsCLSB 0xEF		// Lower Word (C RMS Current)

/* THD, FREQUENCY, ANGLE & TEMP REGISTERS*/
#define THDNUA 0xF1 		// A Voltage THD+N
#define THDNUB 0xF2 		// B Voltage THD+N
#define THDNUC 0xF3 		// C Voltage THD+N
///////////////// 0xF4	    // Reserved Register	
#define THDNIA 0xF5 		// A Current THD+N
#define THDNIB 0xF6 		// B Current THD+N
#define THDNIC 0xF7 		// V Current THD+N
#define Freq 0xF8 			// Frequency
#define PAngleA 0xF9 		// A Mean Phase Angle
#define PAngleB 0xFA 		// B Mean Phase Angle
#define PAngleC 0xFB 		// C Mean Phase Angle
#define Temp 0xFC			// Measured Temperature
#define UangleA 0xFD		// A Voltage Phase Angle
#define UangleB 0xFE		// B Voltage Phase Angle
#define UangleC 0xFF		// C Voltage Phase Angle
		/* Main Electrical Parameters (GET)*/
		double GetLineVoltageA();
		double GetLineVoltageB();
		double GetLineVoltageC();

		double GetLineCurrentA();
		double GetLineCurrentB();
		double GetLineCurrentC();
		double GetLineCurrentN();

		double GetActivePowerA();
		double GetActivePowerB();
		double GetActivePowerC();
		double GetTotalActivePower();

		double GetReactivePowerA();
		double GetReactivePowerB();
		double GetReactivePowerC();
		double GetTotalReactivePower();

		double GetApparentPowerA();
		double GetApparentPowerB();
		double GetApparentPowerC();
		double GetTotalApparentPower();

		double GetFrequency();

		double GetPowerFactorA();
		double GetPowerFactorB();
		double GetPowerFactorC();
		double GetTotalPowerFactor();

		double GetPhaseA();
		double GetPhaseB();
		double GetPhaseC();

		double GetTemperature();

		/* Gain Parameters (GET)*/
		unsigned short GetValueRegister(unsigned short registerRead);

		/* Energy Consumption */
		double GetImportEnergy();
		double GetExportEnergy();

		/* System Status */
		unsigned short GetSysStatus0();
		unsigned short GetSysStatus1();
		unsigned short GetMeterStatus0();
		unsigned short GetMeterStatus1();


//Communication Functions
unsigned short CommEnergyIC(unsigned char RW, unsigned short address, unsigned short val);

//SPI starting Function
void start_spi(void);

//Starting ATM90E36
void begin_ATM90E36(void);