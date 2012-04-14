#include "IMUSensor/FPSPrinter.h"

FPSPrinter::FPSPrinter()
{
    first = true;
	frames = 0;
}

FPSPrinter::~FPSPrinter()
{
}

void FPSPrinter::print()
{
    if(first)
    {
        begin = Time::now();
        total = Duration(0);
        first = false;
        cout << "Time    Frames  FPSm  FPSi" << endl;
        cout << "----  --------  ----  ----" << endl;
    }
    else
    {
        Time now = Time::now();
	    Duration diff = now - begin;
	    total += diff;
	    frames++;

	    int ttime = (int) total.toSec();
	    int fpsm = (int) (frames/total.toSec());
	    int fpsi = (int) (1/diff.toSec());

		cout << "\r" << setw(4) << right << ttime << "  " << setw(8) << right << frames
			 << "  " << setw(4) << right << fpsm << "  " << setw(4) << right << fpsi << flush;

	    begin = Time::now();
    }
}

