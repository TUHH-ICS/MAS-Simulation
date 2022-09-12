#pragma once
#include <vector>
#include <cmath>
#include <assert.h>
#include <tuple>

 /* *****************************************************
 *  tools.h
 *  Author: Daniel Schneider [ds] (schneiderd@uni-koblenz.de)
 *
 *  Comments:
 *
 *  *****************************************************
*/

namespace TLS{
// Modern C++ Design, A. Alexandrescu, p. 25
template<bool> struct CompileTimeError;
template<> struct CompileTimeError<true> {}; 

#define STATIC_CHECK(expr) (TLS::CompileTimeError<(expr) != 0>())
 



template <int v>
struct Int2Type{
	// Modern C++ Design, A. Alexandrescu, p. 29
	enum { value = v };
};

template <typename T>
struct Type2Type{
	// Modern C++ Design, A. Alexandrescu, p. 32
	typedef T OriginalType;
};
struct vec3{
	double x1;
	double x2;
	double x3;
};

void set_vec3(vec3& target, const vec3& source);


double vec3_squared_dist(const vec3& a, const vec3& b);

double vec3_dist(const vec3& a, const vec3& b);
bool vec3_eq(const vec3& a, const vec3& b);

vec3 vec3_sum(const vec3& a, const vec3& b);
vec3 vec3_difference(const vec3& a, const vec3& b);
vec3 vec3_scal_mult(const double& lambda, const vec3& a);

double vec3_len(const vec3& a);


bool my_fmod(double x, double y);



 
int faculty(const unsigned int n);


template <typename T>
std::tuple<T, T, T, T> sort_four(const T& a,const T& b,const T& c,const T& d){
    T low1, low2, high1, high2, lowest, middle1, middle2, highest;
    if (a < b){
        low1 = a;
        high1 = b;
    }
    else {
        low1 = b;
        high1 = a;
    }
     
    if (c < d){
        low2 = c;
        high2 = d;
    }
    else{
        low2 = d;
        high2 = c;
    }

    if (low1 < low2){
        lowest = low1;
        middle1 = low2;
    }
    else{
        lowest = low2;
        middle1 = low1;
    }

    if (high1 > high2){
        highest = high1;
        middle2 = high2;
    }
    else{
        highest = high2;
        middle2 = high1;
    }

    if (middle1 < middle2){
        return std::make_tuple(lowest, middle1, middle2, highest);
    }
    else{
        return std::make_tuple(lowest, middle2, middle1, highest);
    }
}
  
template <typename T>
T my_build_reduced_index(const T& low, const T& mid1, const T& mid2, const T& high, const T& theInd){
	T pos;
	if (theInd < mid1){
		if (theInd < low){
			pos = 0;
		}
		else{
			pos = 1;
		}
	}
	else{
		if (theInd < mid2){
			pos = 2;
		}
		else if (theInd < high){
			pos = 3;
		}
		else{
			pos = 4;
		}
	}
	return theInd - pos;
}
     
double Qfunction(const double x);
double erf(double x);


}//end namespace TLS

