#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

class Grid {
	// to be implemented
public:
	Grid()
	{
		length = 16;
		width = 16;
		points = new point[length * width];
	}
	struct point
	{
		double value;
	};
	point* getPoints()
	{
		return points;
	}
	void setSize(int len, int wid)
	{
		delete points;
		length = len;
		width = wid;
		points = new point[length * width];
		for (int i = 0; i < length * width; i++)
		{
			points[i].value = 0;
		}
		for (int i = 0; i < 5 * length; i++)
		{
			//points[i].value = rand() % 100 - 50;
			points[i].value = 25.0;
		}
	}
	int* getSize()
	{
		int size[2];
		size[0] = length;
		size[1] = width;
		return size;
	}
	point* points;
private:
	int length;
	int width;


};



class DiffusionSimulator:public Simulator{
public:
	// Construtors
	DiffusionSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();

	// Feel free to change the signature of these functions, add arguments, etc.
	void diffuseTemperatureExplicit(float timeStep);
	void diffuseTemperatureImplicit(float timeStep);

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	Grid T;
	double alpha;
	int length;
	int width;
};

#endif