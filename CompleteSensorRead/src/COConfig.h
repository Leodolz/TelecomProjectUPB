/*
 * COConfig.h
 *
 *  Created on: Nov 30, 2018
 *      Author: leandro
 */

#ifndef COCONFIG_H_
#define COCONFIG_H_

#include <string.h>
#include "stdbool.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

float CalibrarCO(float raw);
float ValorCO (float raww, float di);

float CalibrarCO (float raw)
{
	float inter;
	float rs_air;
	inter = raw/4096*3.3;
	rs_air = (3.3-inter)/inter;
	return rs_air/(26+(1/3));
}
float ValorCO (float raww, float di)
{
	float rs_gas;
	float ratio;
	rs_gas = (1-(raww/4096))/(raww/4096);
	ratio= rs_gas/di;
	return 100*pow((log10(40))/(log10(0.09)),ratio);

}

#endif /* COCONFIG_H_ */
