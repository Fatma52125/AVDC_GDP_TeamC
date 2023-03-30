#pragma once
#include <stdint.h>

void Calculate_Fit(float v[9], float ref_radius, float off[3], float rotM[9]);

void Correct_DataPoint(float XYZ[3], float off[3], float rotM[9]);

void Recursive_Ellipsoid_Fit(float P[81], float theta[9], float XYZ[3], uint8_t ini, uint8_t rotation);