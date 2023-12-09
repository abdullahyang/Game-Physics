#include "RigidBodySystemSimulator.h" 
#include "Simulator.h"
#define TESTCASEUSEDTORUNTEST 2

bool continueSimulation{ false };
float timeFactor{ 0.0f };
bool userInteraction{ false };
bool gravityInfluence{ false };

std::vector<Vec3> calculateVertices(Vec3 center, Vec3 size) {
	std::vector<Vec3> vertices;
	int id{ 0 };
	for (int i = -1; i <= 1; i += 2) {
		for (int j = -1; j <= 1; j += 2) {
			for (int k = -1; k <= 1; k += 2) {
				float x = center[0] + i * size[0] / 2.0f;
				float y = center[1] + j * size[1] / 2.0f;
				float z = center[2] + k * size[2] / 2.0f;

				vertices.emplace_back(Vec3(x, y, z));
			}
		}
	}
	return vertices;
}

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_externalForce = Vec3(1, 1, 0);

	rigidBodies.reserve(1);
	RigidBody rigidBodyA = RigidBody(
		1,
		Vec3(0, 0, 0),
		2,
		Vec3(1, 0.6, 0.5),
		calculateVertices(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5)),
		Vec3(0, 0, 0),
		Vec3(0, 0, 0),
		Vec3(0, 0, 0),
		Quat(
			GamePhysics::vector3Dim<Real>(0.0f, 0.0f, 1.0f), // Z-axis
			DirectX::XMConvertToRadians(90.0f) // 90 degrees
		),
		Vec3(0, 0, 0),
		Vec3(0, 0, 0)
	);
	rigidBodies.emplace_back(rigidBodyA);
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "Start, Demo 1,Demo 2,Demo 3,Demo 4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Time Factor", TW_TYPE_FLOAT, &timeFactor, "");
	TwAddVarRW(DUC->g_pTweakBar, "Continue Simulation", TW_TYPE_BOOL8, &continueSimulation, "");
	TwAddVarRW(DUC->g_pTweakBar, "User Interaction", TW_TYPE_BOOL8, &userInteraction, "");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_BOOL8, &gravityInfluence, "");
}

void RigidBodySystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
	for (auto& rigidBody : rigidBodies) {
		rigidBody.obj2WorldMatrix = rigidBody.scaleMatrix * rigidBody.rotationMatrix * rigidBody.translateMatrix;
		DUC->drawRigidBody(rigidBody.obj2WorldMatrix);
	}
}

void printDetails(RigidBodySystemSimulator::RigidBody& rigidBody) 
{
	std::cout << "Xcm: " << rigidBody.cm_position << std::endl;
	std::cout << "Vcm: " << rigidBody.linearVelocity << std::endl;
	std::cout << "Orientation: " << rigidBody.orientation << std::endl;
	std::cout << "Torque: " << rigidBody.torque << std::endl;
	std::cout << "Angular Momentum: " << rigidBody.angularMomentum << std::endl;
	std::cout << "Rotation: " << rigidBody.rotationMatrix << std::endl;
	std::cout << "Inertia: " << rigidBody.inertiaMatrix << std::endl;
	std::cout << "Translation: " << rigidBody.translateMatrix << std::endl;
}

void RigidBodySystemSimulator::resetRigidBody()
{
	rigidBodies.clear();
	RigidBody rigidBodyA = RigidBody(
		1,
		Vec3(0, 0, 0),
		2,
		Vec3(1, 0.6, 0.5),
		calculateVertices(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5)),
		Vec3(0, 0, 0),
		Vec3(0, 0, 0),
		Vec3(0, 0, 0),
		Quat(
			Vec3(0.0f, 0.0f, 1.0f), // Z-axis
			DirectX::XMConvertToRadians(90.0f) // 90 degrees
		),
		Vec3(0, 0, 0),
		Vec3(0, 0, 0)
	);
	rigidBodies.emplace_back(rigidBodyA);
}
void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
		case 0:
			resetRigidBody();
			break;
		case 1:
			applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
			continueSimulation = true;
			simulateTimestep(2);
			continueSimulation = false;
			std::cout << "Point position: " << rigidBodies.at(0).cm_position + (rigidBodies.at(0).rotationMatrix * Vec3(0.3, 0.5, 0.25)) << std::endl;
			printDetails(rigidBodies.at(0));
			break;
		case 2:
			continueSimulation = true;
			applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
			printDetails(rigidBodies.at(0));
			break;
		case 3:
			break;
		default:
			std::cout << "Empty\n";
			break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	Vec3 gravity(0, 0, 0);
	Vec3 pullforce(0, 0, 0);
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 forceView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 forceWorld = worldViewInv.transformVectorNormal(forceView);
		float forceScale = 0.00001f;
		pullforce = pullforce + (forceWorld * forceScale);
	}
	//pullforce -=  pullforce * 5.0f * timeElapsed;

	if (userInteraction) {
		if (gravityInfluence) {
			Vec3 gravity = Vec3(0, -9.81f, 0);
		}
		applyForceOnBody(0, Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0), gravity + pullforce);
	}
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep) {
	timeStep += timeFactor * timeStep;
	if (continueSimulation)
		for (auto& rigidBody : rigidBodies) {
			rigidBody.force = m_externalForce;
			rigidBody.linearVelocity += timeStep * (rigidBody.force / rigidBody.mass);
			m_externalForce = 0;
			rigidBody.cm_position += timeStep * rigidBody.linearVelocity;

			rigidBody.orientation += (timeStep / 2) * Quat(0, rigidBody.angularVelocity[0], rigidBody.angularVelocity[1], rigidBody.angularVelocity[2]) * rigidBody.orientation;
			// Normalize the quaternion to prevent drift
			rigidBody.orientation = rigidBody.orientation.unit();

			// Update the rotation matrix
			rigidBody.rotationMatrix = rigidBody.orientation.getRotMat();

			rigidBody.angularMomentum += timeStep * rigidBody.torque;
			rigidBody.torque = Vec3(0, 0, 0);
			auto temporaryMatrix = rigidBody.rotationMatrix;
			rigidBody.rotationMatrix.transpose();
			rigidBody.inertiaMatrix = temporaryMatrix * rigidBody.inertiaMatrix * rigidBody.rotationMatrix;
			rigidBody.rotationMatrix.transpose();
			rigidBody.angularVelocity = rigidBody.inertiaMatrix * rigidBody.angularMomentum;
			
			rigidBody.translateMatrix.value[3][0] = rigidBody.cm_position[0];
			rigidBody.translateMatrix.value[3][1] = rigidBody.cm_position[1];
			rigidBody.translateMatrix.value[3][2] = rigidBody.cm_position[2];

			//printDetails(rigidBody);
		}
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

// ExtraFunctions
int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return rigidBodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return rigidBodies.at(i).cm_position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return rigidBodies.at(i).linearVelocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return rigidBodies.at(i).angularVelocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
	m_externalForce = force;
	rigidBodies.at(i).torque = cross(loc, force);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	RigidBody newRigidBody(
		rigidBodies.size(),
		position,
		mass,
		size,
		calculateVertices(position, size),
		Vec3(0, 0, 0),
		Vec3(0, 0, 0),
		Vec3(0, 0, 0),
		Quat(
			GamePhysics::vector3Dim<Real>(0.0f, 0.0f, 1.0f), // Z-axis
			DirectX::XMConvertToRadians(90.0f) // 90 degrees
		),
		Vec3(0, 0, 0),
		Vec3(0, 0, 0)
	);

	rigidBodies.emplace_back(newRigidBody);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	rigidBodies.at(i).orientation = orientation;
}
void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	rigidBodies.at(i).linearVelocity = velocity;
}