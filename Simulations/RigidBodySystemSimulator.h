#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "util/matrixbase.h"
#include "util/vectorbase.h"
#include <DirectXMath.h>
//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2

class RigidBodySystemSimulator :public Simulator {
public:
	// Construtors
	RigidBodySystemSimulator();

	// Structs
	//struct RigidBodyPoint
	//{
	//	int id;
	//	Vec3 pos;
	//};
	struct RigidBody
	{
		int id;
		Vec3 cm_position;
		int mass;
		Vec3 size;
		std::vector<Vec3> vertices;
		Vec3 linearVelocity;
		Vec3 angularVelocity;
		Vec3 angularMomentum;
		Quat orientation;
		Vec3 force;
		Vec3 torque;
		matrix4x4<double> inertiaMatrix;
		matrix4x4<double> rotationMatrix;
		matrix4x4<double> scaleMatrix;
		matrix4x4<double> translateMatrix;
		matrix4x4<double> obj2WorldMatrix;

		RigidBody(int _id, Vec3 _cm_position, int _mass, Vec3 _size, std::vector<Vec3> _vertices, Vec3 _linearVelocity, Vec3 _angularVelocity, Vec3 _angularMomentum, Quat _orientation, Vec3 _force, Vec3 _torque) {
			id = _id;
			cm_position = _cm_position;
			mass = _mass;
			size = _size;
			vertices = _vertices;
			linearVelocity = _linearVelocity;
			angularVelocity = _angularVelocity;
			angularMomentum = _angularMomentum;
			orientation = _orientation;
			force = _force;
			torque = _torque;
			inertiaMatrix = matrix4x4<double>{
				_mass * (size[1] * size[1] + size[2] * size[2]) / 12, 0.0f, 0.0f, 0.0f,
				0.0f, _mass * (size[0] * size[0] + size[2] * size[2]) / 12, 0.0f, 0.0f,
				0.0f, 0.0f, _mass * (size[0] * size[0] + size[1] * size[1]) / 12, 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f
			};
			inertiaMatrix = inertiaMatrix.inverse();
			rotationMatrix = _orientation.getRotMat();
			scaleMatrix = matrix4x4<double>{
				size[0], 0.0f, 0.0f, 0.0f,
				0.0f, size[1], 0.0f, 0.0f,
				0.0f, 0.0f, size[2], 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f
			};
			translateMatrix = matrix4x4<double>{
				1.0f, 0.0f, 0.0f, 0.0f,
				0.0f, 1.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 1.0f, 0.0f,
				_cm_position[0], _cm_position[1], _cm_position[2], 1.0f
			};
			obj2WorldMatrix = scaleMatrix * rotationMatrix * translateMatrix;
		}
	};

	// Functions
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void resetRigidBody();
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, float mass);
	void setOrientationOf(int i, Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);
	void addStaticWall(Vec3 position, Vec3 size);


private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	Vec3 m_externalForce;
	std::vector<RigidBody> rigidBodies;
	std::vector<RigidBody> wallBodies;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif