/* -------------------------------------------------------------------------------------------------
 * Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved
 * Licensed under the GPLv3. See LICENSE in the project root for license information.
 * Author(s): Daniel Schneider
 * -------------------------------------------------------------------------------------------------
 */

#include "tools.h"
#include <iostream>
#include <assert.h>
#include <cmath>

namespace TLS{

double vec3_squared_dist(const vec3& a, const vec3& b){
	return (a.x1-b.x1)*(a.x1-b.x1) + (a.x2-b.x2)*(a.x2-b.x2) + (a.x3-b.x3)*(a.x3-b.x3);
}

double vec3_dist(const vec3& a, const vec3& b){
	return std::sqrt(vec3_squared_dist(a,b));
}

double vec3_len(const vec3& a){
	return vec3_dist(a, {0.0, 0.0, 0.0});	
}

bool vec3_eq(const vec3& a, const vec3& b){
	if (a.x1 == b.x1 && a.x2 == b.x2 && a.x3 == b.x3){
		return true;
	}
	else{
		return false;
	}
}

bool my_fmod(double x, double y){
	const double a = x/y;
	const double b = static_cast<double>(static_cast<int>(x/y));

	return (std::abs(a-b) < 1e-14);
}

void set_vec3(vec3& target, const vec3& source){
	target.x1 = source.x1;
	target.x2 = source.x2;
	target.x3 = source.x3; 
} 

vec3 vec3_sum(const vec3& a, const vec3& b){
	return {a.x1 + b.x1, a.x2 + b.x2, a.x3 + b.x3};
}

vec3 vec3_difference(const vec3& a, const vec3& b){
	return {a.x1 - b.x1, a.x2 - b.x2, a.x3 - b.x3};
} 
vec3 vec3_scal_mult(const double& lambda, const vec3& a){
	return {lambda * a.x1, lambda * a.x2, lambda * a.x3};
} 
int faculty(const unsigned int n){
	int res = 1.0;
	for (unsigned int i = 1; i<= n; i++){
		res *= i;
	}

	return res;
} 


double qfunction(const double x){
	return 0.5 * (1.0 - erf(x/sqrt(2.0)));
} 


/*
double qfunction(const double){
	return 1.0;
}
*/

double erf(double x)
{
    /* taken from https://www.johndcook.com/blog/cpp_erf/ */

    // constants
    double a1 =  0.254829592;
    double a2 = -0.284496736;
    double a3 =  1.421413741;
    double a4 = -1.453152027;
    double a5 =  1.061405429;
    double p  =  0.3275911;

    // Save the sign of x
    int sign = 1;
    if (x < 0)
        sign = -1;
    x = fabs(x);

    // A&S formula 7.1.26
    // A&S refers to Handbook of Mathematical Functions by Abramowitz and Stegun.
    double t = 1.0/(1.0 + p*x);
    double y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*exp(-x*x);

    return sign*y;
}
}//end namespace

void testErf()
{
    /* taken from https://www.johndcook.com/blog/cpp_erf/ */

    // Select a few input values
    double x[] = 
    {
        -3, 
        -1, 
        0.0, 
        0.5, 
        2.1 
    };

    // Output computed by Mathematica
    // y = Erf[x]
    double y[] = 
    { 
        -0.999977909503, 
        -0.842700792950, 
        0.0, 
        0.520499877813, 
        0.997020533344 
    };

    int numTests = sizeof(x)/sizeof(double);

    double maxError = 0.0;
    for (int i = 0; i < numTests; ++i)     {         
        double error = fabs(y[i] - TLS::erf(x[i]));         
        if (error > maxError)
            maxError = error;
    }

    std::cout << "Maximum error: " << maxError << "\n";
}  
