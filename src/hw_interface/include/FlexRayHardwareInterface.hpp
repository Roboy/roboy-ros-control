/* 
 * File:   flexRayCommunication.hpp
 * Author: letrend
 *
 * Created on November 18, 2015, 4:04 PM
 */

#pragma once

// comment if no hardware available
#define HARDWARE

#include <ftd2xx.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include "CommunicationData.h"
#include <ros/console.h>

#define NUM_SPI_FRAMES 310
/*! \def DATASETSIZE 
 * \brief number of words to exchange per SPI frame (taken from CommunicationData.h)
 */
#define DATASETSIZE NUM_SPI_FRAMES 
/*! \def USBOUTSIZE 
 * \brief size of USB out buffer in bytes (64 byte aligned)
 */
#define USBOUTSIZE (((DATASETSIZE*17)/64)+1)*64
/*! \def USBINSIZE 
 * \brief size of USB in buffer in bytes (64 byte aligned)
 */
#define USBINSIZE (((DATASETSIZE*2)/64)+1)*64

class FlexRayHardwareInterface{
public:
    /**
     * Constructor 
     */
    FlexRayHardwareInterface(){        
        std::cout << "----------------------------" << std::endl;
#ifdef HARDWARE
        ROS_INFO("Trying to connect to FlexRay");
        while(!connect()){
            ROS_INFO("retry? [y/n]");
            std::string s;
            std::cin >> s;
            if(strcmp("y",s.c_str())==0){
                ROS_INFO("retrying...");
            }else if(strcmp("n",s.c_str())==0){
                ROS_FATAL("abort");
                break;
            }
        }
        initializeMotors();
#else
        ROS_INFO( "No Hardware mode enabled");
        activeGanglionsMask = 0b111111;
        numberOfGanglionsConnected = 6;
#endif
    };
    /**
     * connect to flexray
     * @return status
     */
    bool connect(){
        if(CheckDeviceConnected(&m_numberOfConnectedDevices)==true)
        {
            if(GetDeviceInfo(&m_numberOfConnectedDevices)==true)
            {
                if(OpenPortAndConfigureMPSSE(&m_ftHandle,USBINSIZE,USBOUTSIZE)==true)
                {
                    if(TestMPSSE(&m_ftHandle)==true)
                    {
                        if(ConfigureSPI(&m_ftHandle, m_clockDivisor)==true)
                        {
                            ROS_INFO("Configuration OK");
                            m_FTDIReady = true;
                            return true;
                        }
                    }//TestMPSSE
                    else
                    {
                        ROS_ERROR("device test failed");
                    }
                }//OpenPortAndConfigureMPSSE
                else
                {
                    ROS_ERROR("open port failed\n--Perhaps the kernel automatically loaded another driver for the FTDI USB device, from command line try: \nsudo rmmod ftdi_sio \n sudo rmmod usbserial\n or maybe run with sudo");
                }
            }//GetDeviceInfo
            else
            {
                ROS_ERROR("device info failed");
            }
        } //CheckDeviceConnected
        else
        {
            ROS_ERROR("device not connected");
        }
        return false;
    };
    /**
     * use this to initialize the motor control
     * @return status
     */
    void initializeMotors(){
        controlparams.tag = 0;			// sint32
        controlparams.outputPosMax = 1000;	// sint32			
        controlparams.outputNegMax = -1000;		// sint32
        controlparams.spPosMax = 10000.0;		// float32
        controlparams.spNegMax = -10000.0;		// float32
        controlparams.timePeriod = 100;		// float32		//in us	// set time period to avoid error case
        controlparams.radPerEncoderCount = 2*3.14159265359/(2000.0*53.0);	// float32
        controlparams.polyPar[0] = 0;		// float32
        controlparams.polyPar[1] = 1;
        controlparams.polyPar[2] = 0;
        controlparams.polyPar[3] = 0;
        controlparams.torqueConstant = 1.0;	// float32

        controlparams.params.pidParameters.integral = 0;	// float32
        controlparams.params.pidParameters.pgain = 10000.0;	// float32
        controlparams.params.pidParameters.igain = 0.0; 	// float32
        controlparams.params.pidParameters.dgain = 0.0;	// float32
        controlparams.params.pidParameters.forwardGain = 0;	// float32
        controlparams.params.pidParameters.deadBand = 0;	// float32
        controlparams.params.pidParameters.lastError = 0;	// float32
        controlparams.params.pidParameters.IntegralPosMax = 0;	// float32
        controlparams.params.pidParameters.IntegralNegMax = 0;	// float32

        // initialize PID controller in motordriver boards
        for(uint i=0;i<3;i++){
            for(uint j=0;j<4;j++)
            {
                commandframe0[i].ControlMode[j] = Position;
                commandframe0[i].OperationMode[j] = Initialise;
                commandframe0[i].sp[j] = 0; 
                commandframe1[i].ControlMode[j] = Position;
                commandframe1[i].OperationMode[j] = Initialise;
                commandframe1[i].sp[j] = 0; 
            }
        }
        updateCommandFrame();
        exchangeData();
        for(uint i=0;i<3;i++){
            for(uint j=0;j<4;j++)
            {
                commandframe0[i].ControlMode[j] = Position;
                commandframe0[i].OperationMode[j] = Run;
                commandframe0[i].sp[j] = 0;
                commandframe1[i].ControlMode[j] = Position;
                commandframe1[i].OperationMode[j] = Run;
                commandframe1[i].sp[j] = 0;
            }
        }
        updateCommandFrame();
        exchangeData();
        // check how many ganglions are connected via the activeGanglionsMask
//        numberOfGanglionsConnected = NumberOfSetBits(activeGanglionsMask);
        numberOfGanglionsConnected = 1;
        ROS_INFO("%d ganglions are connected via flexray, activeGanglionsMask %c", numberOfGanglionsConnected,activeGanglionsMask);
    };
    
    void exchangeData(){
        m_ftStatus = SPI_WriteBuffer(m_ftHandle, &dataset[0], DATASETSIZE);		// send data
	if (m_ftStatus != FT_OK){
	    ROS_ERROR_STREAM("Failed to Send a byte through SPI, Error Code: " << m_ftStatus);
	    //FT_SetBitMode(ftHandle, 0x0, 0x00); 			// Reset the port to disable MPSSE
	    //FT_Close(ftHandle);					// Close the USB port
	}else{
            DWORD dwNumInputBuffer=0;
            DWORD dwNumBytesRead;
            
            // WAIT FOR DATA TO ARRIVE
            ROS_DEBUG("waiting for data");
            while(dwNumInputBuffer!=DATASETSIZE*2){
                m_ftStatus = FT_GetQueueStatus(m_ftHandle, &dwNumInputBuffer); // get the number of bytes in the device receive buffer
            }
            ROS_DEBUG("reading data");
            FT_Read(m_ftHandle, &InputBuffer[0], dwNumInputBuffer, &dwNumBytesRead); 	// read bytes into word locations
            
            // now make a copy of relevant signal
            memcpy( GanglionData, InputBuffer, sizeof(GanglionData)); //ganglion data
            
            activeGanglionsMask=InputBuffer[sizeof(GanglionData)>>1];   //active ganglions, generated from usbFlexRay interface
        }
    }
    
    void updateCommandFrame(){
        memcpy((void *)&dataset[0], commandframe0, sizeof(comsCommandFrame)*GANGLIONS_PER_CONTROL_FRAME );
        memcpy((void *)&dataset[sizeof(comsCommandFrame)*GANGLIONS_PER_CONTROL_FRAME / 2], commandframe1, sizeof(comsCommandFrame)*GANGLIONS_PER_CONTROL_FRAME );
        memcpy((void *)&dataset[72], &controlparams, sizeof(control_Parameters_t));
    };
    
    uint32_t NumberOfSetBits(uint32_t i){
        i = i - ((i >> 1) & 0x55555555);
        i = (i & 0x33333333) + ((i >> 2) & 0x33333333);
        return (((i + (i >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
    }

    //! upstream from ganglions to PC
    ganglionData_t GanglionData[NUMBER_OF_GANGLIONS]; 
    unsigned short activeGanglionsMask;
    uint numberOfGanglionsConnected;
    //! command frames containing motor control parameters for 3 ganglia, respectively
    comsCommandFrame commandframe0[3];
    comsCommandFrame commandframe1[3];
    //! control parameters for motor initialization run disable
    control_Parameters_t controlparams;
private:
    //! status code
    char m_status;
    //! Handle of the FTDI device
    FT_HANDLE m_ftHandle;
    //! Result of each D2XX call
    FT_STATUS m_ftStatus; 
    //! number of devices connected
    uint m_numberOfConnectedDevices;
    //! Value of clock divisor, SCL Frequency = 60/((1+value)*2) = MHz i.e., value of 2 = 10MHz, or 29 = 1Mhz
    uint m_clockDivisor = 2;	
    //! FTDI ready
    bool m_FTDIReady;
    //! this will be send via flexray
    WORD dataset[DATASETSIZE];	
    //! this will contain data coming from flexray
    WORD InputBuffer[DATASETSIZE];	
    
    /**
     * routine is used to enable slave
     * @param OutputBuffer points to output buffer
     * @param NumBytesToSend number of bytes to send
     * @return NumBytesToSend number of bytes to send
     */
    DWORD SPI_CSEnable(BYTE* OutputBuffer, DWORD* NumBytesToSend);
    /**
     * routine is used to disable slave
     * @param OutputBuffer points to output buffer
     * @param NumBytesToSend number of bytes to send
     * @param end (true) disable slave
     * @return NumBytesToSend number of bytes to send
     */
    DWORD SPI_CSDisable(BYTE* OutputBuffer, DWORD* NumBytesToSend, bool end);
    /**
     * find connected devices
     * @param NumDevs gets filled with number of devices
     * @return (true) if any devices found (false) if no devices found
     */
    bool CheckDeviceConnected(DWORD* NumDevs);
    /**
     * get device info
     * @param NumDevs devices to get info on
     * @return (true) if devices found (false) if devices not found
     */
    bool GetDeviceInfo(DWORD* NumDevs);
    /**
     * Open device and configure the MPSSE controller
     * @param ftHandle Handle of the device
     * @param InTransferSize  Transfer size for USB IN request.
     * @param OutTransferSize Transfer size for USB OUT request
     * @return (true) on success (false) on failure
     */
    bool OpenPortAndConfigureMPSSE(FT_HANDLE* ftHandle, DWORD InTransferSize, DWORD OutTransferSize);
    /**
     * test the MPSSE controller
     * @param ftHandle Handle of the device
     * @return (true) MPSSE synchronized (false) Error in synchronizing the MPSSE
     */
    bool TestMPSSE(FT_HANDLE* ftHandle);
    /**
     * Configure the MPSSE controller into an SPI module
     * @param ftHandle Handle of the device
     * @param dwClockDivisor 
     * @return (true) SPI initialization successful (false) Error configuring SPI
     */
    bool ConfigureSPI(FT_HANDLE* ftHandle, DWORD dwClockDivisor);
    /**
     * Send a single byte through the SPI bus
     * @param ftHandle Handle of the device
     * @param bdata Byte to be send
     * @return (true) success (false) failure
     */
    BOOL SPI_WriteByte(FT_HANDLE ftHandle, WORD bdata);
    /**
     * Write a number of words to the SPI bus
     * @param ftHandle Handle of the device
     * @param buffer data
     * @param numwords number of words
     * @return FT_STATUS
     */
    FT_STATUS SPI_WriteBuffer(FT_HANDLE ftHandle, WORD* buffer, DWORD numwords);
    
    // necessary for MPSSE command
    const BYTE MSB_RISING_EDGE_CLOCK_BYTE_OUT = '\x10';             
    const BYTE MSB_FALLING_EDGE_CLOCK_BYTE_OUT = '\x11';            
    const BYTE MSB_RISING_EDGE_CLOCK_BIT_OUT = '\x12';              
    const BYTE MSB_FALLING_EDGE_CLOCK_BIT_OUT = '\x13';             
    const BYTE MSB_RISING_EDGE_CLOCK_BYTE_IN = '\x20';              
    const BYTE MSB_RISING_EDGE_CLOCK_BIT_IN = '\x22';               
    const BYTE MSB_FALLING_EDGE_CLOCK_BYTE_IN = '\x24';             
    const BYTE MSB_FALLING_EDGE_CLOCK_BIT_IN = '\x26';              
    const BYTE MSB_FALLING_EDGE_OUT_RISING_EDGE_IN_BYTE = '\x31';   
    const BYTE MSB_RISING_EDGE_OUT_FALLING_EDGE_IN_BYTE = '\x34';   
    const BYTE MSB_RISING_EDGE_OUT_FALLING_EDGE_IN_BIT = '\x36';    
    const BYTE MSB_FALLING_EDGE_OUT_RISING_EDGE_IN_BIT = '\x33';    
};

