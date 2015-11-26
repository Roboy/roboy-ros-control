/*
 *  Copyright (c) 2012, MYOROBOTICS consortium
 *  Author: Alexander Lenz and Paul Bremner
 *  All rights reserved
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification is governed by the MYOROBOTICS Non-Commercial Software
 *  License Agreement. See LICENSE file distributed with this work for
 *  additional information.
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under this license is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either expressed or
 *  implied. See the License for the specific language governing permissions
 *  and limitations under the License.
 *
 */
#pragma once

#define NUMBER_OF_GANGLIONS 6 //make sure to adjust the value below too when changing here
#define MAX_ACTIVE_GANGLIONS_MASK 0x3F //6 ganglions can only set the 6 LSBs
#define NUMBER_OF_MUSCLES_PER_GANGLION 4
#define NUMBER_OF_JOINTS_PER_GANGLION 4
#define NUMBER_OF_RECEPTORS_PER_GANGLION 12
#define GANGLIONS_PER_CONTROL_FRAME 3

//sample time value is used by myode to cyclicly communicate with the USB-FlexRay driver
#define USB_FLEXRAY_SAMPLE_TIME_MS 2

#ifndef __SYS_COMMON_H__//if not using the ganglion definitions then define the types for PC implementation.
//To use this file make sure sys_common.h is included first

typedef unsigned long long uint64;
typedef unsigned int uint32;
typedef unsigned short  uint16;
//typedef unsigned char boolean;
//typedef unsigned char boolean_t;
typedef signed long long sint64;
typedef signed int sint32;
typedef signed short sint16;
typedef signed char sint8;
typedef float float32;
typedef double float64;

#endif//__SYS_COMMON_H__

#define EMBEDDED
#define NUM_SPI_FRAMES 310
#define MINIMUM_TIMEPERIOD_CONTROLLER_MICROSECONDS 400
/* The types in this file need to be in functions called from the ISR functions written in C, so namespace can't be used on the embedded system */

//ERROR from SVN plugin: Get log messages for 'https://svnknoll.informatik.tu-muenchen.de/myorobotics/src/trunk/myode/src/eu.myode.Myorobot/robot/CommunicationData.h' failed.
//svn: Malformed reply from SOCKS server
//svn: PROPFIND request failed on '/myorobotics/src/trunk/myode/src/eu.myode.Myorobot/robot/CommunicationData.h'

//TODO: define data types for embbedded and Linux project
//use svn-externals to include this header in the embedded project as well as here!
//define

typedef enum comsControllerMode
{
    Raw=0,
	    Torque=1,
	    Velocity=2,
	    Position=3,
	    Force=4,
	    NumberOfControllers=5   //not a usable control mode, but used on the ganglion to set up the array of controllers
}comsControllerMode;

typedef enum comsOperationMode
{
    Disable=0,
	    Initialise=1, //! expects control parameter update in the dynamic frame
	    Run	=2	  //! runs the controller and receives new reference values
	    
}comsOperationMode;

/** An array of 6 of these will be transmitted in the mode frame on the flexray bus.
 * The difference in endianess is dealt with on the embedded system.
 *
 */
typedef struct
{
    /** \brief a mode word is needed for each motor that can be connected to a ganglion
     * @see comsControllerMode
     */
    sint8 ControlMode[4];
    /** \brief operation mode for each motor of a ganglion
     * @see comsOperationMode
     */
    sint8 OperationMode[4];
    /** \brief setpoint for 4 muscles of each Ganglion, merging of static frames 1 and 2 */
    float32 sp[4];				
}comsCommandFrame;

typedef struct
{
    float32 integral;/*!<Integral of the error*/
    float32 pgain;/*!<Gain of the proportional component*/
    float32 igain;/*!<Gain of the integral component*/
    float32 dgain;/*!<Gain of the differential component*/
    float32 forwardGain; /*!<Gain of  the feed-forward term*/
    float32 deadBand;/*!<Optional deadband threshold for the control response*/
    float32 lastError;/*!<Error in previous time-step, used to calculate the differential component*/
    float32 IntegralPosMax; /*!<Integral positive component maximum*/
    float32 IntegralNegMax; /*!<Integral negative component maximum*/
}pid_Parameters_t;


/*! \brief Raw mode has a limited set of control parameters - just those used in error checking.
 *
 */
/*
 typedef struct
 {
 
 }raw_Parameters_t;
 */

/*! \brief To allow a single set of bridge functions to be used with all controllers a union is used to store the parameters.
 *
 * This union can be extended with additional structs that contain the parameters used by additional controllers.
 * The initial 4 members of each struct must be the same (see raw_Parameters_t) to ensure compatibility.
 */
typedef union
{
    pid_Parameters_t pidParameters;
    //raw_Parameters_t rawParameters;
}parameters_t;

typedef struct
{
    uint32 tag;/*!<Tag to indicate data type when passing the union*/
    sint32 outputPosMax; /*!< maximum control output in the positive direction in counts, max 4000*/
    sint32 outputNegMax; /*!< maximum control output in the negative direction in counts, max -4000*/
    float32 spPosMax;/*<!Positive limit for the set point.*/
    float32 spNegMax;/*<!Negative limit for the set point.*/
    float32 timePeriod;/*!<Time period of each control iteration in microseconds.*/
    float32 radPerEncoderCount; /*!output shaft rotation (in rad) per encoder count */
    float32 polyPar[4]; /*! polynomial fit from displacement (d)  to tendon force (f)
			 f=polyPar[0]+polyPar[1]*d +polyPar[2]*d^2+ +polyPar[3]*d^3/ //*/
    float32 torqueConstant; /*!motor torque constant in Nm/A */
    
    parameters_t params;
    
}control_Parameters_t;

typedef struct
{
    control_Parameters_t controlParameters;
    int ganglionId;
    int muscleId;
    comsControllerMode controllerMode;
}queueableControlParameters_t;

/* Data that is returned for each muscle, different ordering is used to compensate for different endianess of PC and embedded system.
 * Due to word invarience, this only affects the ordering of the data for the 16-bit values.
 */

#ifdef EMBEDDED
typedef struct muscleState
{
    sint32 jointPos;//position of the controlled joint
    sint32 actuatorPos;//position of the actuator in encoder counts
    //sint8 test1;sint8 test2;sint8 test3;sint8 test4;
    sint32 actuatorVel;//velocity of the actuator in counts/s
    uint16 actuatorCurrent;//current drawn by the actuator
    sint16 tendonDisplacement;//displacement of the SE element
}muscleState_t;

/*Semaphores for memory arbitration between asynchronous SPI and FlexRay comms*/
typedef union
{
    unsigned char allflags;
    struct
    {
	unsigned newdata :1U;
	unsigned busy	 :1U;
	unsigned :6U;
    }flags_s;
}flags_t;

#else
typedef struct muscleState
{
    sint32 jointPos;//position of the controlled joint
    sint32 actuatorPos;//position of the actuator in encoder counts
    //sint8 test1;sint8 test2;sint8 test3;sint8 test4;
    sint32 actuatorVel;//velocity of the actuator in counts/s
    sint16 tendonDisplacement;//displacement of the SE element
    uint16 actuatorCurrent;//current drawn by the actuator
}muscleState_t;
#endif
//data from each ganglion - there will be an array of 6 of these to store the incoming data from the flexray bus.
typedef struct ganglionData
{
    muscleState_t muscleState[4];
    sint16 extSensor[12];//there is space for 12 16-bit sensor values from external sensors
}ganglionData_t;
