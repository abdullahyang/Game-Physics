#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"
#include <vector>

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change


class MassSpringSystemSimulator :public Simulator {
public:
	// Construtors
	MassSpringSystemSimulator();

	// Structs
	struct MassPoint
	{
		Vec3 position;
		Vec3 Velocity;
		bool isFixed;
		Vec3 force;
	};
	struct Spring
	{
		int masspoint1;
		int masspoint2;
		float initialLength;
		float length;
	};

	// UI Functions
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);

	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;
	const Vec3 gravity = Vec3(0, -0.1, 0);
	bool wallCollision{ true };
	bool initLeapFrog{ true };
	float initLeapFrogModifier{ 0.5 };
	std::vector<Spring> springVector;
	std::vector<MassPoint> massPointVector;
	std::vector<std::tuple<int, int>> connections;

	std::vector<std::tuple<int, int>> connectGrid(const std::vector<MassPoint>& massPointVector, int gridWidth);

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif