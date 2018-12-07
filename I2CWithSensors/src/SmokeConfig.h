/*
 * SmokeConfig.h
 *
 *  Created on: Dec 6, 2018
 *      Author: leandro
 */

#ifndef SMOKECONFIG_H_
#define SMOKECONFIG_H_

#define RL_Value 10
#define RO_CLEAN_AIR_FACTOR 9.83

#define LPG 0
#define SMOKE 1
#define CO 2

float  GetPercentage(float rs_ro_ratio, float *pcurve);
float GetGasPercentage(float rs_ro_ratio, int gas_id);
float ReadSensor(float valor);
float ResistanceCalculation(float raw_adc);
float ReadMQ (float crudo,int cual, float di);

float LPGCurve[3]={2.3, 0.2, -0.45};

float SmokeCurve[3] ={2.3,0.53,-0.43};

float COCurve[3] = {1.7,0.255,-0.67}; // Formato: { x, y , media}

float ReadMQ (float crudo,int cual, float di)
{
		switch(cual)
		{
		case 1:
			return (GetGasPercentage(ReadSensor(crudo)/di,LPG));
		case 2:
			return (GetGasPercentage(ReadSensor(crudo)/di,SMOKE));
		default:
			return (GetGasPercentage(ReadSensor(crudo)/di,CO));

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
	switch (gas_id)
	{
	case 0:
		return GetPercentage(rs_ro_ratio,LPGCurve);
	case 1:
		return GetPercentage(rs_ro_ratio,SmokeCurve);
	default:
		return GetPercentage(rs_ro_ratio,COCurve);
	}
}

float GetPercentage(float rs_ro_ratio, float *curve)
{
	//return rs_ro_ratio;
	return ( pow(10,(((log10(rs_ro_ratio)+curve[1])/curve[2])+ curve[0])));
}


#endif /* SMOKECONFIG_H_ */
