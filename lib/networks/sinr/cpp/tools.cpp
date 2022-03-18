/* -------------------------------------------------------------------------------------------------
 * Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved
 * Licensed under the GPLv3. See LICENSE in the project root for license information.
 * Author(s): Daniel Schneider
 * -------------------------------------------------------------------------------------------------
 */

#include "tools.h"
#include <iostream>
#include <assert.h>
 
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

}//end namespace








