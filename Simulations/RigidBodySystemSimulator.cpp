#include "RigidBodySystemSimulator.h" 
#include "Simulator.h"
#include "collisionDetect.h"
#define TESTCASEUSEDTORUNTEST 2

bool continueSimulation{ false };
float timeFactor{ 1.0f };
bool userInteraction{ false };
bool gravityInfluence{ false };
bool allowCollisions{ false };
bool wallCollisions{ false };

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
	m_externalForce = Vec3(0, 0, 0);
	reset();
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "Start,Demo 1,Demo 2,Demo 3,Demo 4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Time Factor", TW_TYPE_FLOAT, &timeFactor, "");
	TwAddVarRW(DUC->g_pTweakBar, "Continue Simulation", TW_TYPE_BOOL8, &continueSimulation, "");
	TwAddVarRW(DUC->g_pTweakBar, "User Interaction", TW_TYPE_BOOL8, &userInteraction, "");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_BOOL8, &gravityInfluence, "");
	TwAddVarRW(DUC->g_pTweakBar, "Collisions", TW_TYPE_BOOL8, &allowCollisions, "");
	TwAddVarRW(DUC->g_pTweakBar, "Wall", TW_TYPE_BOOL8, &wallCollisions, "");
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
	addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
	setOrientationOf(0, Quat(Vec3(0.0f, 0.0f, 0.1f), (float)(M_PI) * 0.5f));

}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
		case 0:
			break;
		case 1:
			resetRigidBody();
			applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
			continueSimulation = true;
			simulateTimestep(2);
			continueSimulation = false;
			std::cout << "Point position: " << rigidBodies.at(0).cm_position + (rigidBodies.at(0).rotationMatrix * Vec3(0.3, 0.5, 0.25)) << std::endl;
			printDetails(rigidBodies.at(0));
			break;
		case 2:
			resetRigidBody();
			userInteraction = true;
			continueSimulation = true;
			applyForceOnBody(0, Vec3(0.8, 0.3, 0.4), Vec3(1, 1, 1));
			printDetails(rigidBodies.at(0));
			break;
		case 3:
			rigidBodies.clear();
			cout << "3!" << endl;
			allowCollisions = true;
			continueSimulation = true;
			addRigidBody(Vec3(0, 0, 0), Vec3(0.3, 0.4, 0.5), 2);
			setOrientationOf(0, Quat(Vec3(0.0f, 0.0f, 0.1f), (float)(M_PI) * 0.5f));
			addRigidBody(Vec3(0.7, 0, 0), Vec3(0.5, 0.5, 0.25), 2);
			setOrientationOf(1, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.0f));
			applyForceOnBody(0, Vec3(0.2, 0, 0), Vec3(70, 10, 40));
			applyForceOnBody(1, Vec3(0.3, 0.5, 0.25), Vec3(-40, 10, 0));
			printDetails(rigidBodies.at(0));
			break;
		case 4:
			allowCollisions = true;
			rigidBodies.clear();

			// Create multiple rigid bodies

			addRigidBody(Vec3(-0.5, 0, 0), Vec3(0.6, 0.2, 0.25), 1);
			addRigidBody(Vec3(0.5, 0, 0), Vec3(0.8, 0.3, 0.25), 25);
			addRigidBody(Vec3(0, 0.5, 0), Vec3(0.2, 0.2, 0.25), 1);
			addRigidBody(Vec3(0, -0.5, 0), Vec3(0.2, 0.2, 0.7), 1);
			addRigidBody(Vec3(0, -0.5, -0.5), Vec3(0.1, 0.1, 0.4), 2);
			addRigidBody(Vec3(0.5, 0.5, 0), Vec3(0.5, 0.2, 0.1), 1);
			setOrientationOf(0, Quat(Vec3(0.0f, 0.8f, 0.6f), (float)(M_PI) * 0.25f));
			setOrientationOf(1, Quat(Vec3(0.8f, 0.0f, 0.6f), (float)(M_PI) * 0.2f));
			setOrientationOf(2, Quat(Vec3(0.6f, 0.8f, 0.0f), (float)(M_PI) * 0.75f));
			setOrientationOf(3, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.3f));
			setOrientationOf(4, Quat(Vec3(0.6f, 0.8f, 0.0f), (float)(M_PI) * 0.21f));
			setOrientationOf(5, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.76f));
			applyForceOnBody(0, Vec3(0.5, 0.1, 0.2), Vec3(30, 10, 0));
			applyForceOnBody(1, Vec3(0.7, 0.1, 0.2), Vec3(-100, -200, 0));
			applyForceOnBody(2, Vec3(0.04, 0.1, 0.2), Vec3(-8, -6, 4));
			applyForceOnBody(3, Vec3(0.15, 0, 0.5), Vec3(-10, 10, 10));
			applyForceOnBody(4, Vec3(0.05, 0.05, 0.53), Vec3(-10, 10, 10));
			applyForceOnBody(5, Vec3(0.4, 0, 0.05), Vec3(-20, -10, 10));

			// Enable wall collision
			wallCollisions = true;
			continueSimulation = true;

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
		float forceScale = 0.001f;
		pullforce = pullforce + (forceWorld * forceScale);
	}
	//pullforce -=  pullforce * 5.0f * timeElapsed;

	if (userInteraction) {
		// added support for arbitary number of rigid bodies
		for (int i = 0; i < getNumberOfRigidBodies(); i++)
		{
			applyForceOnBody(i, Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0), pullforce);
		}
	}

}

void collisionResponse(CollisionInfo collisionTest, RigidBodySystemSimulator::RigidBody& rigidBodyA, RigidBodySystemSimulator::RigidBody& rigidBodyB, float c = 0.8f) {
	// Calculate collision points in local space
	Vec3 collisionPointA = collisionTest.collisionPointWorld - rigidBodyA.cm_position;
	Vec3 collisionPointB = collisionTest.collisionPointWorld - rigidBodyB.cm_position;
	
	// Calculate velocity at collision points for both bodies
	Vec3 linearVelocityAtCollisionPointA = rigidBodyA.linearVelocity + cross(rigidBodyA.angularVelocity, collisionPointA);
	Vec3 linearVelocityAtCollisionPointB = rigidBodyB.linearVelocity + cross(rigidBodyB.angularVelocity, collisionPointB);

	// Calculate relative velocity
	Vec3 relativeVelocity = linearVelocityAtCollisionPointA - linearVelocityAtCollisionPointB;
	Vec3 collisionNormal = collisionTest.normalWorld;
	if (dot(relativeVelocity, collisionNormal) < 0) {
		// Calculate impulse
		float totalMass = rigidBodyA.mass + rigidBodyB.mass;
		float impulseScalar = -(1 + c) * dot(relativeVelocity, collisionNormal) /
			((1 / rigidBodyA.mass) + (1 / rigidBodyB.mass) +
				dot((cross((rigidBodyA.inertiaMatrix * cross(collisionPointA, collisionNormal)), collisionPointA) +
					(cross((rigidBodyB.inertiaMatrix * cross(collisionPointB, collisionNormal)), collisionPointB))), collisionNormal));

		Vec3 dampingFactor = Vec3(0.8f, 0.8f, 0.8f);
		Vec3 impulse = impulseScalar * collisionNormal * dampingFactor;

		// Apply impulse to rigid bodies
		rigidBodyA.linearVelocity += impulse / rigidBodyA.mass;
		rigidBodyA.angularVelocity += cross(collisionPointA, impulse);

		rigidBodyB.linearVelocity -= impulse / rigidBodyB.mass;
		rigidBodyB.angularVelocity -= cross(collisionPointB, impulse);

		rigidBodyA.cm_position += 0.001f * rigidBodyA.linearVelocity;
		rigidBodyB.cm_position += 0.001f * rigidBodyB.linearVelocity;
	}
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep) {
	// timeStep += timeFactor * timeStep;
	if (continueSimulation) {
		for (auto& rigidBody : rigidBodies) {
			if (gravityInfluence) {
				rigidBody.force += Vec3(0, -9.8, 0) * rigidBody.mass * 0.01;
			}
			// Apply force and update position
			rigidBody.force += m_externalForce;
			rigidBody.cm_position += timeStep * rigidBody.linearVelocity;
			rigidBody.linearVelocity += timeStep * (rigidBody.force / rigidBody.mass);
			rigidBody.force = 0;

			// Update the orientation quaternion
			rigidBody.orientation += (timeStep / 2) * Quat(0, rigidBody.angularVelocity[0], rigidBody.angularVelocity[1], rigidBody.angularVelocity[2]) * rigidBody.orientation;
			// Normalize the quaternion to prevent drift
			rigidBody.orientation = rigidBody.orientation.unit();
			// Update the rotation matrix
			rigidBody.rotationMatrix = rigidBody.orientation.getRotMat();
			// Apply torque
			rigidBody.angularMomentum += timeStep * rigidBody.torque;
			// Remove torque after changing linear momentum
			rigidBody.torque = Vec3(0, 0, 0);
			// transpose() function changes matrix in-place instead of returning a transpose
			auto temporaryMatrix = rigidBody.rotationMatrix;
			rigidBody.rotationMatrix.transpose();
			rigidBody.inertiaMatrix = temporaryMatrix * rigidBody.inertiaMatrix * rigidBody.rotationMatrix;
			rigidBody.rotationMatrix.transpose();
			rigidBody.angularVelocity = rigidBody.inertiaMatrix * rigidBody.angularMomentum;
			// Update the translate matrix for world-space conversion
			rigidBody.translateMatrix.value[3][0] = rigidBody.cm_position[0];
			rigidBody.translateMatrix.value[3][1] = rigidBody.cm_position[1];
			rigidBody.translateMatrix.value[3][2] = rigidBody.cm_position[2];

			//printDetails(rigidBody);
		}

		// Remove all external forces after influencing linear velocity
		m_externalForce = 0;
	}

	// added inter-collision support for arbitary number of rigid bodies
	if (allowCollisions) {
		for (int i = 0; i < getNumberOfRigidBodies()-1; i++)
		{
			for (int j = i + 1; j < getNumberOfRigidBodies(); j++)
			{
				CollisionInfo collisionTest = checkCollisionSAT(rigidBodies.at(i).obj2WorldMatrix, rigidBodies.at(j).obj2WorldMatrix);
				if (collisionTest.isValid) {
					collisionResponse(collisionTest, rigidBodies.at(i), rigidBodies.at(j));
				}
			}
		}

	}

	if (wallCollisions) {
		addStaticWall(Vec3(0, 0, -1), Vec3(2, 2, 0.1));  // Wall at the back (negative z)
		addStaticWall(Vec3(0, 0, 1), Vec3(2, 2, 0.1));   // Wall at the front (positive z)
		addStaticWall(Vec3(-1, 0, 0), Vec3(0.1, 2, 2));   // Wall on the left (negative x)
		addStaticWall(Vec3(1, 0, 0), Vec3(0.1, 2, 2));    // Wall on the right (positive x)
		addStaticWall(Vec3(0, -1, 0), Vec3(2, 0.1, 2));   // Floor (negative y)
		addStaticWall(Vec3(0, 1, 0), Vec3(2, 0.1, 2));    // Ceiling (positive y)
		for (int i = 0; i < getNumberOfRigidBodies(); i++)
		{
//			for (int j = 0; j < 6; j++)
//			{
//				CollisionInfo collisionTest = checkCollisionSAT(rigidBodies.at(i).obj2WorldMatrix, wallBodies.at(j).obj2WorldMatrix);
//				if (collisionTest.isValid) {
//					collisionResponse(collisionTest, rigidBodies.at(i), wallBodies.at(j), 1.0f);
//				}
//			}
			
			for (int j = 0; j < 3; j++)
			{
				// bouncing back when hitting walls
				if (rigidBodies.at(i).cm_position[j] < -0.8 || rigidBodies.at(i).cm_position[j] > 0.8)
				{
					int sign = rigidBodies.at(i).cm_position[j] / abs(rigidBodies.at(i).cm_position[j]);
					rigidBodies.at(i).cm_position[j] = sign * 0.8 - sign * abs(rigidBodies.at(i).cm_position[j] - (sign * 0.8));
					rigidBodies.at(i).linearVelocity[j] = -rigidBodies.at(i).linearVelocity[j];
					rigidBodies.at(i).linearVelocity *= 0.9;
				}
			}
		}
	}
	// Sleep(3);
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
	rigidBodies.at(i).force += force;
	rigidBodies.at(i).torque += cross(loc, force);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, float mass)
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
			Vec3(1.0f, 0.0f, 0.0f),
			DirectX::XMConvertToRadians(90.0f) // 90 degrees
		),
		Vec3(0, 0, 0),
		Vec3(0, 0, 0)
	);

	rigidBodies.emplace_back(newRigidBody);
}

void RigidBodySystemSimulator::addStaticWall(Vec3 position, Vec3 size) {
	RigidBody wallBody(
		wallBodies.size(),
		position,
		std::numeric_limits<float>::infinity(),  // Infinite mass for static objects
		size,
		calculateVertices(position, size),
		Vec3(0, 0, 0),
		Vec3(0, 0, 0),
		Vec3(0, 0, 0),
		Quat(),  // No initial rotation for walls
		Vec3(0, 0, 0),
		Vec3(0, 0, 0)
	);

	wallBody.obj2WorldMatrix = wallBody.scaleMatrix * wallBody.rotationMatrix * wallBody.translateMatrix;
	wallBodies.emplace_back(wallBody);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	rigidBodies.at(i).orientation = orientation;
	rigidBodies.at(i).rotationMatrix = orientation.getRotMat();
}
void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	rigidBodies.at(i).linearVelocity = velocity;
}