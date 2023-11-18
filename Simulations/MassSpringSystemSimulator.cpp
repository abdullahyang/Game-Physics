#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_fMass = 10;
	m_fStiffness = 40;
	m_fDamping = 0;
	m_iIntegrator = 0;
	m_externalForce = Vec3(0, 0, 0);

	// Limit number to 100
	springVector.reserve(100);
	massPointVector.reserve(100);

	// Generate 8 mass points with 16 springs (a cube)
	massPointVector.emplace_back(MassPoint{ Vec3(-0.25, 0.25, 0.25), Vec3(0, 0, 0), false, Vec3(0, 0, 0) });
	massPointVector.emplace_back(MassPoint{ Vec3(0.25, 0.25, 0.25), Vec3(0, 0, 0), false, Vec3(0, 0, 0) });
	massPointVector.emplace_back(MassPoint{ Vec3(-0.25, -0.25, 0.25), Vec3(0, 0, 0), false, Vec3(0, 0, 0) });
	massPointVector.emplace_back(MassPoint{ Vec3(0.25, -0.25, 0.25), Vec3(0, 0, 0), false, Vec3(0, 0, 0) });
	massPointVector.emplace_back(MassPoint{ Vec3(-0.25, 0.25, -0.25), Vec3(0, 0, 0), false, Vec3(0, 0, 0) });
	massPointVector.emplace_back(MassPoint{ Vec3(0.25, 0.25, -0.25), Vec3(0, 0, 0), false, Vec3(0, 0, 0) });
	massPointVector.emplace_back(MassPoint{ Vec3(-0.25, -0.25, -0.25), Vec3(0, 0, 0), false, Vec3(0, 0, 0) });
	massPointVector.emplace_back(MassPoint{ Vec3(0.25, -0.25, -0.25), Vec3(0, 0, 0), false, Vec3(0, 0, 0) });

	connections.reserve(16);
	connections = { { {0, 1}, {0, 2}, {0, 4}, {1, 5}, {1, 3}, {2, 3}, {2, 6}, {3, 7}, {4, 5}, {4, 6}, {5, 7}, {6, 7}, {0, 7}, {4, 3}, {5, 2}, {1, 6} } };

	// Either set initial length to a constant (0.5) to form an inconsistent cube or randomise
	for (auto& connection : connections)
	{
		springVector.emplace_back(Spring{ connection.first, connection.second, /*0.5*/(float)rand() / RAND_MAX, 0 });
	}
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "Euler,Leap-Frog,Midpoint";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=10");
	TwAddVarRW(DUC->g_pTweakBar, "Spring Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=40 step=10");
	TwAddVarRW(DUC->g_pTweakBar, "Damping Factor", TW_TYPE_FLOAT, &m_fDamping, "min=0 step=0.01");
	TwAddVarRW(DUC->g_pTweakBar, "Collision", TW_TYPE_BOOL8, &wallCollision, "");
}

void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	for (MassPoint& point : massPointVector)
	{
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
		DUC->drawSphere(point.position, Vec3(0.05, 0.05, 0.05));
	}
	for (Spring& spring : springVector)
	{
		DUC->beginLine();
		DUC->drawLine(massPointVector.at(spring.masspoint1).position, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)), massPointVector.at(spring.masspoint2).position, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
		DUC->endLine();
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case EULER:
		std::cout << "Euler\n";
		setIntegrator(EULER);
		break;
	case LEAPFROG:
		std::cout << "Leap-Frog\n";
		setIntegrator(LEAPFROG);
		break;
	case MIDPOINT:
		std::cout << "Midpoint\n";
		setIntegrator(MIDPOINT);
		break;
	default:
		std::cout << "Empty\n";
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// TODO
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		float inputScale = 0.0001f;
		inputWorld = inputWorld * inputScale;
		for (MassPoint& massPoint : massPointVector)
		{
			massPoint.position += inputWorld;
		}
	}
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;

	// For the purposes of testing a single case
	//massPointVector.emplace_back(MassPoint{ Vec3(0, 0, 0), Vec3(-1, 0, 0), false, Vec3() });
	//massPointVector.emplace_back(MassPoint{ Vec3(0, 2, 0), Vec3(1, 0, 0), false, Vec3() });
	//springVector.emplace_back(Spring{ 0, 1, 1, 0 });
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	// Create vectors to store initial positions and velocities
	std::vector<Vec3> initialPositions;
	std::vector<Vec3> initialVelocities;
	switch (m_iIntegrator)
	{
	case EULER:
		for (Spring& spring : springVector)
		{
			// Create reference to mass points
			MassPoint& massPoint1 = massPointVector.at(spring.masspoint1);
			MassPoint& massPoint2 = massPointVector.at(spring.masspoint2);

			// Calculate distances
			Vec3 distanceVector = massPoint1.position - massPoint2.position;
			spring.length = sqrt(distanceVector[0] * distanceVector[0] + distanceVector[1] * distanceVector[1] + distanceVector[2] * distanceVector[2]);

			// Calculate spring force acting on both involved mass points
			// F = -k * (l - L) * d_normalised
			massPoint1.force += -m_fStiffness * (spring.length - spring.initialLength) * (distanceVector / spring.length);
			massPoint2.force += -m_fStiffness * (spring.length - spring.initialLength) * (-distanceVector / spring.length);

			// Account for external forces
			massPoint1.force += m_externalForce - m_fDamping * massPoint1.Velocity;
			massPoint2.force += m_externalForce - m_fDamping * massPoint2.Velocity;
		}
		for (MassPoint& massPoint : massPointVector)
		{
			// Update position using old velocity
			// Update velocity based on old acceleration
			if (!massPoint.isFixed)
			{
				massPoint.position += timeStep * massPoint.Velocity;

				// Deal with collision; simply have the point bounce back by the same amount it exceeded boundary
				if (wallCollision)
				{
					for (int axis = 0; axis < 3; axis++)
					{
						if (massPoint.position[axis] > 0.5 || massPoint.position[axis] < -0.5)
						{
							int sign = abs(massPoint.position[axis]) / massPoint.position[axis];
							massPoint.position[axis] = sign * (1 - abs(massPoint.position[axis]));
							massPoint.Velocity[axis] = -massPoint.Velocity[axis];
						}
					}
				}

				// Add gravity to acceleration
				massPoint.Velocity += timeStep * (massPoint.force / m_fMass + gravity);
			}

			//std::cout << "Force: " << massPoint.force << endl;
			//std::cout << "Position: " << massPoint.position << endl;
			//std::cout << "Velocity: " << massPoint.Velocity << endl;

			// Clear force
			massPoint.force = Vec3(0, 0, 0);
		}
		break;

	case LEAPFROG:
		//std::cout << "Init: " << initLeapFrog << initLeapFrogModifier << endl;
		for (Spring& spring : springVector)
		{
			// Create reference to mass points
			MassPoint& massPoint1 = massPointVector.at(spring.masspoint1);
			MassPoint& massPoint2 = massPointVector.at(spring.masspoint2);

			// Calculate distances
			Vec3 distanceVector = massPoint1.position - massPoint2.position;
			spring.length = sqrt(distanceVector[0] * distanceVector[0] + distanceVector[1] * distanceVector[1] + distanceVector[2] * distanceVector[2]);

			// Calculate spring force acting on both involved mass points
			// F = -k * (l - L) * d_normalised
			massPoint1.force += -m_fStiffness * (spring.length - spring.initialLength) * (distanceVector / spring.length);
			massPoint2.force += -m_fStiffness * (spring.length - spring.initialLength) * (-distanceVector / spring.length);

			// Account for external forces
			massPoint1.force += m_externalForce - m_fDamping * massPoint1.Velocity;
			massPoint2.force += m_externalForce - m_fDamping * massPoint2.Velocity;
		}
		for (MassPoint& massPoint : massPointVector)
		{
			// Update position using old velocity
			// Update velocity based on old acceleration
			if (!massPoint.isFixed)
			{
				// Modifier set to 0.5 only during first pass to desynchronize velocity and position
				// Add gravity to acceleration
				massPoint.Velocity += initLeapFrogModifier * timeStep * (massPoint.force / m_fMass + gravity);
				massPoint.position += timeStep * massPoint.Velocity;

				// Deal with collision; simply have the point bounce back by the same amount it exceeded boundary
				if (wallCollision)
				{
					for (int axis = 0; axis < 3; axis++)
					{
						if (massPoint.position[axis] > 0.5 || massPoint.position[axis] < -0.5)
						{
							int sign = abs(massPoint.position[axis]) / massPoint.position[axis];
							massPoint.position[axis] = sign * (1 - abs(massPoint.position[axis]));
							massPoint.Velocity[axis] = -massPoint.Velocity[axis];
						}
					}
				}
			}

			//std::cout << "Force: " << massPoint.force << endl;
			//std::cout << "Position: " << massPoint.position << endl;
			//std::cout << "Velocity: " << massPoint.Velocity << endl;

			// Clear force
			massPoint.force = Vec3(0, 0, 0);

			// Signal completion of initialization
			initLeapFrog = false;
		}
		if (!initLeapFrog)
		{
			initLeapFrogModifier = 1;
			initLeapFrog = false;
		}
		break;

	case MIDPOINT:
		// Accumulate forces
		for (Spring& spring : springVector)
		{
			// Create reference to mass points
			MassPoint& massPoint1 = massPointVector.at(spring.masspoint1);
			MassPoint& massPoint2 = massPointVector.at(spring.masspoint2);

			//// Create temporary copies of mass points to take half-step
			//MassPoint massPointTmp1 = massPointVector.at(spring.masspoint1);
			//MassPoint massPointTmp2 = massPointVector.at(spring.masspoint2);

			// Calculate distances
			Vec3 distanceVector = massPoint1.position - massPoint2.position;
			spring.length = sqrt(distanceVector[0] * distanceVector[0] + distanceVector[1] * distanceVector[1] + distanceVector[2] * distanceVector[2]);

			// Calculate spring force acting on both involved mass points
			// F = -k * (l - L) * d_normalised
			massPoint1.force += -m_fStiffness * (spring.length - spring.initialLength) * (distanceVector / spring.length);
			massPoint2.force += -m_fStiffness * (spring.length - spring.initialLength) * (-distanceVector / spring.length);

			// Account for external forces
			massPoint1.force += m_externalForce - m_fDamping * massPoint1.Velocity;
			massPoint2.force += m_externalForce - m_fDamping * massPoint2.Velocity;
		}

		// Perform half-step
		for (MassPoint& massPoint : massPointVector)
		{
			// Store initial values
			initialPositions.emplace_back(massPoint.position);
			initialVelocities.emplace_back(massPoint.Velocity);

			// Make half-step for position
			massPoint.position += timeStep / 2 * massPoint.Velocity;

			// Make half-step for velocity
			// Add gravity to acceleration
			massPoint.Velocity += timeStep / 2 * (massPoint.force / m_fMass + gravity);

			// Clear forces
			massPoint.force = Vec3(0, 0, 0);
		}

		// Accumulate forces at half-step
		for (Spring& spring : springVector)
		{
			// Create reference to mass points
			MassPoint& massPoint1 = massPointVector.at(spring.masspoint1);
			MassPoint& massPoint2 = massPointVector.at(spring.masspoint2);

			// Calculate distances
			Vec3 distanceVector = massPoint1.position - massPoint2.position;
			spring.length = sqrt(distanceVector[0] * distanceVector[0] + distanceVector[1] * distanceVector[1] + distanceVector[2] * distanceVector[2]);

			// Calculate spring force acting on both involved mass points
			// F = -k * (l - L) * d_normalised
			massPoint1.force += -m_fStiffness * (spring.length - spring.initialLength) * (distanceVector / spring.length);
			massPoint2.force += -m_fStiffness * (spring.length - spring.initialLength) * (-distanceVector / spring.length);

			// Account for external forces
			massPoint1.force += m_externalForce - m_fDamping * massPoint1.Velocity;
			massPoint2.force += m_externalForce - m_fDamping * massPoint2.Velocity;
		}

		// Perform full-step
		for (int i = 0; i < massPointVector.size(); ++i)
		{
			if (!massPointVector[i].isFixed)
			{
				// Make full-step for position using velocity at half-step
				massPointVector[i].position = initialPositions[i] + timeStep * massPointVector[i].Velocity;

				// Deal with collision; simply have the point bounce back by the same amount it exceeded boundary
				if (wallCollision)
				{
					for (int axis = 0; axis < 3; axis++)
					{
						if (massPointVector[i].position[axis] > 0.5 || massPointVector[i].position[axis] < -0.5)
						{
							int sign = abs(massPointVector[i].position[axis]) / massPointVector[i].position[axis];
							massPointVector[i].position[axis] = sign * (1 - abs(massPointVector[i].position[axis]));
							initialVelocities[i][axis] = -initialVelocities[i][axis];
						}
					}
				}

				// Make full-step for velocity using accelaration at half-step
				// Add gravity to acceleration
				massPointVector[i].Velocity = initialVelocities[i] + timeStep * (massPointVector[i].force / m_fMass + gravity);
			}

			//std::cout << "Force: " << massPointVector[i].force << endl;
			//std::cout << "Position: " << massPointVector[i].position << endl;
			//std::cout << "Velocity: " << massPointVector[i].Velocity << endl;

			// Clear forces
			massPointVector[i].force = Vec3(0, 0, 0);
		}
		break;

	default:
		std::cout << "This will never trigger, but I'm trying to be good at what I do." << endl;
		break;
	}
}

void MassSpringSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	massPointVector.emplace_back(MassPoint{ position, Velocity, isFixed, Vec3(0, 0, 0) });
	return massPointVector.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	springVector.emplace_back(Spring{ masspoint1, masspoint2, initialLength, 0 });
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return massPointVector.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return springVector.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return massPointVector.at(index).position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return massPointVector.at(index).Velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	m_externalForce += force;
}