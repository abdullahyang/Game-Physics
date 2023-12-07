#include "RigidBodySystemSimulator.h" 

#define TESTCASEUSEDTORUNTEST 2

// Construtors
RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	rigidBodyArray = new RigidBody[100];
	rigidBodyNum = 0;
}

// Functions
const char* RigidBodySystemSimulator::getTestCasesStr();
void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC);
void RigidBodySystemSimulator::reset();
void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
void RigidBodySystemSimulator::notifyCaseChanged(int testCase);
void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed);
void RigidBodySystemSimulator::simulateTimestep(float timeStep);
void RigidBodySystemSimulator::onClick(int x, int y);
void RigidBodySystemSimulator::onMouse(int x, int y);

// ExtraFunctions
int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return rigidBodyNum;
}
Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return rigidBodyArray[i].pos;
}
Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return rigidBodyArray[i].LinearVelocity;
}
Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return rigidBodyArray[i].AngularVelocity;
}
void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force);
void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	rigidBodyArray[rigidBodyNum].pos = position;
	rigidBodyArray[rigidBodyNum].size = size;
	rigidBodyArray[rigidBodyNum].mass = mass;
	rigidBodyArray[rigidBodyNum].id = rigidBodyNum;
	rigidBodyNum++;
}
void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	rigidBodyArray[i].orientation = orientation;
}
void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	rigidBodyArray[i].LinearVelocity = velocity;
}
