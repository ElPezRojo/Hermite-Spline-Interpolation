
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include <GLmodel/GLmodel.h>
#include "shared/opengl.h"
#include "BaseSystem.h"

class HSpline : public BaseSystem {
	public:
		bool p1;
		bool p2;
		float coords[3];
		float lastCoords[3];
		float lastDir[3];
		HSpline(const std::string& name);
		virtual void getState(double* p);
		virtual void setState(double* p);
		void reset(double time);
		void catmullRom();

		void display(GLenum mode = GL_RENDER);

		void readModel(char* fname) { m_model.ReadOBJ(fname); }
		void flipNormals(void) { glmReverseWinding(&m_model); }
		int command(int argc, myCONST_SPEC char** argv);
		void getName(double* p);

		void setPoint(float x, float y, float z, int index);
		void setTangent(float x, float y, float z, int index);
		void setCP(float px, float py, float pz, float tx, float ty, float tz, int index);
		void eval_at_value(double t, float location[3]);

		
		void readFromFile(std::string fn);
		void writeToFile(std::string fn);
		void calculate_rotation(float cur[3], float target[3], float rot[2]);
		double calcArcLen(double t);
		void  getTangent(double t, float tangent[3]);
		void initArcTable();

		void part1();
		void part2();

	protected:
		double arcTable[40][2];
		double totalArcLen;
		double offset;
		float prevRoty;
		double lastT;
		//float rot[3];
		float rotation[2];
		char name[20];
		float CP[40][6];
		int len;
		GLMmodel m_model;
		bool isInitialized;

};

