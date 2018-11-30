/*
 * LeerSensorHumo.c
 *
 *  Created on: Nov 29, 2018
 *      Author: leandro
 */
//#include "main.h"
//#include "stm32f0xx_hal.h"
#include <string.h>
#include "stdbool.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "SmokeConfig.h"

float LPGCurve[3]={2.3, 0.2, -0.45};

float SmokeCurve[3] ={2.3,0.53,-0.43};


float  GetPercentage(float rs_ro_ratio, float *pcurve);
float GetGasPercentage(float rs_ro_ratio, int gas_id);
float ReadSensor(float valor);
float ResistanceCalculation(float raw_adc);
//float SensorCalibration(float adcv);
float ReadMQ (float crudo,int cual, float di);


float ReadMQ (float crudo,int cual, float di)
{
		switch(cual)
		{
		case 1:
			return (GetGasPercentage(ReadSensor(crudo)/di,LPG));
		case 2:
			return (GetGasPercentage(ReadSensor(crudo)/di,SMOKE));
		default:
			return crudo;

		}
}

float ReadSensor(float raw)
{                                 // take multiple readings and average it.
    return ResistanceCalculation((raw));   // rs changes according to gas concentration.
}

float ResistanceCalculation(float raw_adc)
{
	return (10*(4096-raw_adc)/raw_adc);
}

float GetGasPercentage(float rs_ro_ratio, int gas_id)
{
	if (gas_id == LPG)
	{
		return GetPercentage(rs_ro_ratio,LPGCurve);
	}
	else if (gas_id == SMOKE)
	{
		return GetPercentage(rs_ro_ratio,SmokeCurve);
	}
	return 0;
}

float GetPercentage(float rs_ro_ratio, float *curve)
{
	//return rs_ro_ratio;
	return ( pow(10,(((log10(rs_ro_ratio)+curve[1])/curve[2])+ curve[0])));
}
