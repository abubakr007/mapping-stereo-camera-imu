#ifndef FPSPRINTER_H_
#define FPSPRINTER_H_

#include <ros/ros.h>
#include <iostream>
#include <iomanip>

using namespace ros;
using namespace std;

class FPSPrinter
{

public:

	FPSPrinter();
	virtual ~FPSPrinter();

	void print();

private:
    bool first;
	Time begin;
	Duration total;
	int frames;

};

#endif /* FPSPRINTER_H_ */
