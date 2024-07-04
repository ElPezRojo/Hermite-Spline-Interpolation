#include "HSpline.h"
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <math.h>
#include <sstream>
#include <cmath>

	
HSpline::HSpline(const std::string& name) :
	BaseSystem(name),
	len(0),
	isInitialized(false),
	p1(false),
	p2(false),
	offset(0.0)
	{
		eval_at_value(0.0L, coords);
	}

	//Use Catmull-Rom spline initiation
	void HSpline::catmullRom() {
		for (int d = 0; d < 3; d++) {
			CP[0][d + 3] = 2 * (CP[1][d] - CP[0][d]) - (CP[2][d] - CP[0][d]) / 2;
			CP[len - 1][d + 3] = 2 * (CP[len - 2][d] - CP[len - 1][d]) - (CP[len - 3][d] - CP[len - 2][d]) / 2;
		}
		for (int i = 1; i < len - 1; i++) {
			for (int d = 0; d < 3; d++) {
				CP[i][d + 3] = (CP[i + 1][d] - CP[i - 1][d]) / 2;
			}
		}
		animTcl::OutputMessage("Finished initiation of Spline");
	}

	//set part1 and part2 variables to choose the display mode
	void HSpline::part1() {
		p1 = true;
	}

	void HSpline::part2() {
		p2 = true;

	}


	//main display function, draws the spline and car if in display mode 2
	void HSpline::display(GLenum mode) {

		//do not draw if spline is not initialized
		if (!isInitialized) {
			return;
		}


		glEnable(GL_LIGHTING);
		glMatrixMode(GL_MODELVIEW);
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glEnable(GL_COLOR_MATERIAL);

		//draw control points of spline
		glPointSize(5);
		glBegin(GL_POINTS);
		glColor3f(1.0, 0.0, 0.0);
		for (int i = 0; i < len; i++) {
			glVertex3f(CP[i][0], CP[i][1], CP[i][2]);
		}
		glEnd();

		// draw interpolated line between control points
		glLineWidth(3);
		glBegin(GL_LINE_STRIP);
		glColor3f(0.0, 0.0, 0.0);
		float cur_point[3];
		for (int i = 0; i < len - 1; i++) {
			for (int k = 0; k < 100; k++) {
				double t = (double)i + ((double)k / 100.0);
				eval_at_value(t, cur_point);
				glVertex3f(cur_point[0], cur_point[1], cur_point[2]);
			}
		}

		glPopAttrib();
		glEnd();

		//only draws car if in display mode 2
		if (p2) {

			// draw porsche
			m_model.ReadOBJ("data/porsche.obj");
			glmFacetNormals(&m_model);
			glmVertexNormals(&m_model, 90);

			
			glEnable(GL_LIGHTING);
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glPushAttrib(GL_ALL_ATTRIB_BITS);

			//scale, rotate, and line up car with track
			glTranslated(coords[0], coords[1], coords[2]);
			glScalef(0.02, 0.02, 0.02);
			glRotatef(90, 0, 1, 0);
			glRotatef(90, 0, 0, 1);


			//apply dynamic rotations depending on the tangent which the car should be facing at the certain point in time
			glRotatef(rotation[0], 0, 1, 0);
			glRotatef(rotation[1], 1, 0, 0);


			if (m_model.numvertices > 0)
				glmDraw(&m_model, GLM_SMOOTH | GLM_MATERIAL);
			else
				glutSolidSphere(1.0, 20, 20);
			glPopAttrib();
			glPopMatrix();
		}

		return;
	}

	void HSpline::getName(double* p) {
		//name already handled
	}


	void HSpline::getState(double* p) {
		//No state to get
	}


	//updates the animation with time counter p
	void HSpline::setState(double* p) {

		//use the time as the parameter to evaluate the spline on
		double tVal = *p;
		tVal /= 10;

		//stop updating if the car is already at the end of the track
		if (tVal > len-1) {
			return;
		}

		//evaluate the spline for the position of the car, and calculate the proper rotation depending on tangent
		lastT = tVal;
		eval_at_value(tVal - offset, coords);
		float curTangent[3] = { 0.0, 0.0, 0.0 };
		getTangent(tVal, curTangent);
		calculate_rotation(lastDir, curTangent, rotation); 

	}


	//resets the animation to the start
	void HSpline::reset(double time) {
		coords[0] = CP[0][0];
		coords[1] = CP[0][1];
		coords[2] = CP[0][2];
		float curDir[3] = { 1.0, 1.0, 1.0 };
		float firstTangent[3] = { 0.0, 0.0, 0.0 };
		getTangent(0, firstTangent);
		offset = lastT;
		lastDir[0] = 0.0;
		lastDir[1] = 0.0;
		lastDir[2] = 0.0;

	}


	//handles TCL commands from the terminal provided
	int HSpline::command(int argc, myCONST_SPEC char** argv) {

		//initiates Catmull-Rom splines
		if (strcmp(argv[0], "cr") == 0) {
			catmullRom();
			return TCL_OK;
		}

		//modifies already-set points or tangents
		else if (strcmp(argv[0], "set") == 0) {
			if (strcmp(argv[1], "point") == 0) {

				setPoint(std::stof(argv[2]), std::stof(argv[3]), std::stof(argv[4]), std::stof(argv[5]));
				return TCL_OK;
			}else if (strcmp(argv[1], "tangent") == 0) {

				setTangent(std::stof(argv[2]), std::stof(argv[3]), std::stof(argv[4]), std::stof(argv[5]));
				return TCL_OK;
			}
			animTcl::OutputMessage("parameters missing / inaccurate");
			return TCL_OK;

		}

		//adds additional points or tangents
		else if (strcmp(argv[0], "add") == 0 && strcmp(argv[1], "point") == 0) {
			setCP(std::stof(argv[2]), std::stof(argv[3]), std::stof(argv[4]), std::stof(argv[5]), std::stof(argv[6]), std::stof(argv[7]), std::stof(argv[8]));
			return TCL_OK;
		}

		//calculates and outputs the arc length of the track
		else if (strcmp(argv[0], "getArcLength") == 0) {
			double t = std::stod(argv[1]);
			double arcLen = calcArcLen(t);
			animTcl::OutputMessage("arcLen: %l", arcLen);
			return TCL_OK;

		}

		//loads a spline from a file with standard spline notation
		else if (strcmp(argv[0], "load") == 0) {
			readFromFile(argv[1]);
			return TCL_OK;

		//exports spline as a file with standard spline notation
		}else if (strcmp(argv[0], "export") == 0) {
			writeToFile(argv[1]);
			return TCL_OK;
		}

		//catch badly-formatted commands
		else {
			animTcl::OutputMessage("could not recognize command");
			return TCL_OK;
		}
		


	}

	//modifies a point's coordinates
	void HSpline::setPoint(float x, float y, float z, int index){

		CP[index][0] = x;
		CP[index][1] = y;
		CP[index][2] = z;
	}

	//modifies a tangent's coordinates
	void HSpline::setTangent(float x, float y, float z, int index){
		CP[index][3] = x;
		CP[index][4] = y;
		CP[index][5] = z;


	}

	//sets a control point (point and tangent)
	void HSpline::setCP(float px, float py, float pz, float tx, float ty, float tz, int index) {
		CP[index][0] = px;
		CP[index][1] = py;
		CP[index][2] = pz;
		CP[index][3] = tx;
		CP[index][4] = ty;
		CP[index][5] = tz;
	}

	//evaluates the current spline at value t, and stores result into location[3]
	void HSpline::eval_at_value(double t, float location[3]) {
		int ti = floor(t);
		
		//catch edge cases
		if (t > len - 1) {
			location[0] = CP[len - 1][0];
			location[1] = CP[len - 1][1];
			location[2] = CP[len - 1][2];
			return;
		}else if(t == 0) {
			location[0] = CP[0][0];
			location[1] = CP[0][1];
			location[2] = CP[0][2];
			return;
		}

		//main spline evaluation part
		float noPow = t - ti;
		float pow2 = pow(noPow, 2);
		float pow3 = pow(noPow, 3);
		for (int i = 0; i < 3; i++) {
			float y_i = CP[ti][i] * (2.0f * pow3 - 3.0f * pow2 + 1.0f);
			float y_next = CP[ti + 1][i] * (-2 * pow3 + 3 * pow2);
			float s_i = CP[ti][i + 3] * (pow3 - 2 * pow2 + noPow);
			float s_next = CP[ti + 1][i + 3] * (pow3 - pow2);
			location[i] = y_i + y_next + s_i + s_next;
		}
		return;
	}


	//calculates and returns the arc length of track up to the given t-value
	double HSpline::calcArcLen(double t) {
		double cur_t = 0.0;
		t *= 10.0L;
		double totalDist = 0.0;
		float last[3] = { 0.0, 0.0, 0.0 };
		float cur[3] = { 0.0, 0.0, 0.0 };
		eval_at_value(0.0, last);
		while (cur_t < t) {
			cur_t += 0.01;
			eval_at_value(cur_t, cur);
			float a = pow((last[0] - cur[0]), 2);
			float b = pow((last[1] - cur[1]), 2);
			float c = pow((last[2] - cur[2]), 2);
			totalDist += sqrt(a+b+c);
			last[0] = cur[0];
			last[1] = cur[1];
			last[2] = cur[2];

		}
		return totalDist;

	}


	//writes the current spline to a file with standard formatting
	void HSpline::writeToFile(std::string fn) {
		if (!isInitialized) {
			return;
		}
		std::ofstream file(fn);
		std::string in = name + std::string(" ") + std::to_string(len);
		file << in;
		file << "\n";
		for (int i = 0; i < len; i++) {
			file << (std::to_string(CP[i][0]) + std::to_string(CP[i][1]) + std::to_string(CP[i][2]) + std::to_string(CP[i][3]) + std::to_string(CP[i][4]) + std::to_string(CP[i][5]));
			file << "\n";
		}
		return;
	}


	//calculates the correct rotation to apply depending on the direction vector, and the target direction vector
	//and stores the yaw in rot[0], and pitch in rot[1]
	void HSpline::calculate_rotation(float cur[3], float target[3], float rot[2]) {

		//default rotation is always facing x-axis
		cur[0] = 1, cur[1] = 0, cur[2] = 0;
		float angle = 0.0;


		//Calculate yaw

		//normalize the target vector and use Arc-Cosine to get angle
		float norm_f = sqrtf((powf(target[0], 2) + powf(target[1], 2)));
		float a = cur[0] * (target[0] / norm_f);
		float b = cur[1] * (target[1] / norm_f);
		angle = acosf(a + b);
		angle *= (180 / PI);

		//correct the angle if the target angle is negative to avoid turning in the opposite direction
		if (target[1] < 0) {
			angle *= -1;
		}
		rotation[0] = angle;


		//calculate pitch using same method as yaw
		norm_f = sqrtf((powf(target[0], 2) + powf(target[2], 2)));
		if (target[0] < 0) {
			target[0] *= -1;
		}
		a = cur[0] * (target[0] / norm_f);
		b = cur[2] * (target[2] / norm_f);
		angle = acosf(a + b);
		angle *= (180 / PI);
		if (target[2] > 0) {
			angle *= -1;
		}
		rotation[1] = angle;
	}
	

	//calculates the tangent of the car at any given time on the spline by approximation, stores result in tangent[3]
	void HSpline::getTangent(double t, float tangent[3]) {
		float cur[3] = { 0.0, 0.0, 0.0 };
		float next[3] = { 0.0, 0.0, 0.0 };

		//evaluate the spline at current point and a very very close approaching point, subtracting to get the tangent
		 
		eval_at_value(t, cur);
		eval_at_value(t + .25, next);

		tangent[0] = next[0] - cur[0];
		tangent[1] = next[1] - cur[1];
		tangent[2] = next[2] - cur[2];

	}

	//reads a spline from file with standard notation
	void HSpline::readFromFile(std::string fn) {
		std::ifstream file(fn);
		std::string curLine;

		if (!file) {
			animTcl::OutputMessage("file could not be found");
			return;
		}
		getline(file, curLine);
		std::stringstream stream(curLine);
		stream >> name;
		std::string nString;
		stream >> nString;
		len = std::stoi(nString);
		for (int i = 0; i < len; i++) {
			getline(file, curLine);
			std::stringstream stream(curLine);
			std::string temp;
			for (int k = 0; k < 6; k++) {
				stream >> temp;
				CP[i][k] = std::stod(temp);
			}
		}

		//set up initial coordinates of the start of the track for car
		coords[0] = CP[0][0];
		coords[1] = CP[0][1];
		coords[2] = CP[0][2];
		initArcTable();
		float curDir[3] = { 0.0, 1.0, 0.0 };
		getTangent(0, lastDir);
		prevRoty = 0.0;
		calculate_rotation(curDir, lastDir, rotation);
		isInitialized = true;
		return;
	}


	void HSpline::initArcTable() {
		double prevVal = 0.0;
		double totalVal = 0.0;
		for (int i = 1; i < len - 1; i++) {
			double curVal = calcArcLen(i / 10.0);
			totalVal += curVal;
			arcTable[i][0] = curVal - prevVal;
			arcTable[i][1] = totalVal;
			prevVal = curVal;
			totalArcLen += curVal;
		}
	}




