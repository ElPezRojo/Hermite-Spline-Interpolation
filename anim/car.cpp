#include "car.h"
#include <windows.h>

car::car(const std::string& name, BaseSystem* spline) :
	BaseSimulator(name),
	spline_object(spline),
	t(0)
{
}	// SampleGravitySimulator


int car::init(double time) {
	return 0;


}
int car::step(double time)
{
	t += time;
	animTcl::OutputMessage("t = %02f", t);
	spline_object->setState(&t);
	//   pos[1]= 2 * sin(2*3.14*time) ;

	//pos = spline_object->getState(&t);
	//car_object->setState(pos);

	return 0;

}	// SampleGravitySimulator::step