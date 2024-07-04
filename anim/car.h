
#include <GLModel/GLModel.h>
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include "BaseSimulator.h"
#include "BaseSystem.h"

#include <string>

// a sample simulator

class car : public BaseSimulator
{
public:

	car(const std::string& name, BaseSystem* spline);

	int step(double time);
	int init(double time);

	int command(int argc, myCONST_SPEC char** argv) { return TCL_OK; }

protected:
	double t; //current t value
	BaseSystem* spline_object;
};
