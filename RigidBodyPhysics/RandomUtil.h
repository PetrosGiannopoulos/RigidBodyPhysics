#ifndef RANDOMUTIL_H
#define RANDOMUTIL_H
#endif

#include <random>
#include <time.h>

class RandomUtil{

public:
	RandomUtil(){srand(time(NULL));}


	float randFloat(float min,float max){
		float random = ((float) rand()) / (float) RAND_MAX;
		float diff = max - min;
		float r = random * diff;
		return min + r;
	}
};