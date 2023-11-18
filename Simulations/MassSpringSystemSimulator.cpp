#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = 0;
	m_fMass = 1;
	m_fStiffness = 50;
	m_fDamping = 0.2;
	m_iIntegrator = EULER;
	// Limit max. 100 MassPoints
	massPointArray = new MassPoint[100];
	// Limit max. 100 Springs
	springArray = new Spring[100];
	numMassPoint = 0;
	numSpring = 0;
	wallCollision = FALSE;
}

// TODO: Complete UI Functions to allow UI interaction, see Demo 4 in PDF document
// UI Functions
const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "Euler,Midpoint,Leapfrog";
}
void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	delete massPointArray;
	delete springArray;
	massPointArray = new MassPoint[100];
	springArray = new Spring[100];
	numMassPoint = 0;
	numSpring = 0;
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=1");
	TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=0");
	TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "step=0.1 min=0");
	TwAddVarRW(DUC->g_pTweakBar, "Collision", TW_TYPE_BOOL8, &wallCollision, "");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_BOOL8, &gravity, "");

	std::mt19937 eng;
	std::uniform_real_distribution<float> randVel(-0.5f, 0.5f);
	std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
	for (int i = 0; i < 20; i++)
	{
		Vec3 newPos = Vec3(randPos(eng), randPos(eng), randPos(eng));
		Vec3 newVel = Vec3(randVel(eng), randVel(eng), randVel(eng));
		addMassPoint(newPos, newVel, (i%3==0));
	}
	addSpring(0, 1, 0.3);
	addSpring(0, 2, 0.5);
	addSpring(2, 3, 0.8);
	addSpring(4, 5, 0.2);
	addSpring(5, 6, 0.1);
	addSpring(7, 8, 0.3);
	addSpring(8, 9, 0.3);
	addSpring(10, 11, 0.7);
	addSpring(11, 12, 0.1);
	addSpring(12, 13, 0.9);
	addSpring(13, 14, 0.8);
	addSpring(15, 1, 0.2);
	addSpring(16, 13, 0.4);
	addSpring(17, 18, 0.6);
	addSpring(18, 19, 0.2);
	addSpring(19, 20, 0.5);
	addSpring(20, 6, 0.8);
	addSpring(18, 9, 0.1);
	addSpring(14, 7, 0.5);

	if (gravity)
	{
		applyExternalForce(Vec3(0, -9.8, 0));
	}
	else
	{
		applyExternalForce(Vec3(0, 0, 0));
	}
	//TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_fSphereSize, "min=0.01 step=0.01");
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
	for (int idx = 0; idx < numMassPoint; idx++)
	{
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
		DUC->drawSphere(massPointArray[idx].position, Vec3(0.05f, 0.05f, 0.05f));
	}
	DUC->beginLine();
	for (int idx = 0; idx < numSpring; idx++)
	{
		if (springArray[idx].length > springArray[idx].initialLength)
		{
			DUC->drawLine(massPointArray[springArray[idx].masspoint1].position, Vec3(255, 0, 0), massPointArray[springArray[idx].masspoint2].position, Vec3(255, 0, 0));
		}
		else
		{
			DUC->drawLine(massPointArray[springArray[idx].masspoint1].position, Vec3(0, 255, 0), massPointArray[springArray[idx].masspoint2].position, Vec3(0, 255, 0));
		}
	}
	DUC->endLine();

}
void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	switch (testCase)
	{
	case 0:
		m_iIntegrator = EULER;
		break;
	case 1:
		m_iIntegrator = MIDPOINT;
		break;
	case 2:
		m_iIntegrator = LEAPFROG;
		break;
	default:
		break;
	}
}
void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	
}

// TODO: Add Midpoint method and Leap-Frog method
void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	switch (m_iIntegrator)
	{
	case EULER:
		// Below are Euler Method
		// Add spring force on mass points
		for (int idx = 0; idx < numSpring; idx++)
		{
			Vec3 pos1 = massPointArray[springArray[idx].masspoint1].position;
			Vec3 pos2 = massPointArray[springArray[idx].masspoint2].position;
			Vec3 distanceVec = pos1 - pos2;
			// Calculate length of spring
			springArray[idx].length = sqrt(distanceVec[0] * distanceVec[0] + distanceVec[1] * distanceVec[1] + distanceVec[2] * distanceVec[2]);

			// Calculate force of springs
			float springForce = -m_fStiffness * (springArray[idx].length - springArray[idx].initialLength);

			// Direction of force
			Vec3 dir1 = distanceVec / springArray[idx].length;

			massPointArray[springArray[idx].masspoint1].force += springForce * dir1;
			massPointArray[springArray[idx].masspoint2].force += -springForce * dir1;
		}

		// Update position with old velocity
		for (int idx = 0; idx < numMassPoint; idx++)
		{
			if (massPointArray[idx].isFixed)
			{
				// No position change to fixed mass points
			}
			else
			{
				massPointArray[idx].position += massPointArray[idx].Velocity * timeStep;
			}
			// Deal with collision
			if (wallCollision)
			{
				for (int axis = 0; axis < 3; axis++)
				{
					if (massPointArray[idx].position[axis] > 0.5 || massPointArray[idx].position[axis] < -0.5)
					{
						int sign = abs(massPointArray[idx].position[axis]) / massPointArray[idx].position[axis];
						massPointArray[idx].position[axis] = sign * (1 - abs(massPointArray[idx].position[axis]));
						massPointArray[idx].Velocity[axis] = -massPointArray[idx].Velocity[axis];
					}
				}
			}
		}
		//cout << "*************** New Timestep of " << timeStep << " ***************" << endl;

		// Update velocity
		for (int idx = 0; idx < numMassPoint; idx++)
		{
			cout << "Mass Point No. " << idx << ":" << endl;
			cout << "Force: " << massPointArray[idx].force << endl;
			cout << "Position: " << massPointArray[idx].position << endl;
			cout << "Old Velocity" << massPointArray[idx].Velocity << endl;

			// Sum of force
			massPointArray[idx].force -= m_fDamping * massPointArray[idx].Velocity;
			massPointArray[idx].force += m_externalForce;
			Vec3 forceSum = massPointArray[idx].force;
			// Acceleration (Newton's 2nd Law)
			Vec3 acc = forceSum / m_fMass;
			if (massPointArray[idx].isFixed)
			{
				massPointArray[idx].Velocity = Vec3(0, 0, 0);
			}
			else
			{
				massPointArray[idx].Velocity += (acc * timeStep);
			}
			// Clear force
			massPointArray[idx].force = Vec3(0, 0, 0);
			cout << "New Velocity" << massPointArray[idx].Velocity << endl;
		}
		break;
	case LEAPFROG:
		// Add spring force on mass points
		for (int idx = 0; idx < numSpring; idx++)
		{
			Vec3 pos1 = massPointArray[springArray[idx].masspoint1].position;
			Vec3 pos2 = massPointArray[springArray[idx].masspoint2].position;
			Vec3 distanceVec = pos1 - pos2;
			// Calculate length of spring
			springArray[idx].length = sqrt(distanceVec[0] * distanceVec[0] + distanceVec[1] * distanceVec[1] + distanceVec[2] * distanceVec[2]);

			// Calculate force of springs
			float springForce = -m_fStiffness * (springArray[idx].length - springArray[idx].initialLength);

			// Direction of force
			Vec3 dir1 = distanceVec / springArray[idx].length;

			massPointArray[springArray[idx].masspoint1].force += springForce * dir1;
			massPointArray[springArray[idx].masspoint2].force += -springForce * dir1;
		}
		// Update velocity
		for (int idx = 0; idx < numMassPoint; idx++)
		{
			cout << "Mass Point No. " << idx << ":" << endl;
			cout << "Force: " << massPointArray[idx].force << endl;
			cout << "Position: " << massPointArray[idx].position << endl;
			cout << "Old Velocity" << massPointArray[idx].Velocity << endl;

			// Sum of force
			massPointArray[idx].force -= m_fDamping * massPointArray[idx].Velocity;
			massPointArray[idx].force += m_externalForce;
			Vec3 forceSum = massPointArray[idx].force;
			// Acceleration (Newton's 2nd Law)
			Vec3 acc = forceSum / m_fMass;
			if (!massPointArray[idx].isFixed)
			{
				// Update position
				massPointArray[idx].position += massPointArray[idx].Velocity * timeStep + 0.5 * acc * timeStep * timeStep;

				// Calculate half-step velocity
				Vec3 v_half = massPointArray[idx].Velocity + 0.5 * acc * timeStep;

				// Update velocity
				massPointArray[idx].Velocity = v_half + 0.5 * acc * timeStep;
			}

			// Deal with collision
			if (wallCollision)
			{
				for (int axis = 0; axis < 3; axis++)
				{
					if (massPointArray[idx].position[axis] > 0.5 || massPointArray[idx].position[axis] < -0.5)
					{
						int sign = abs(massPointArray[idx].position[axis]) / massPointArray[idx].position[axis];
						massPointArray[idx].position[axis] = sign * (1 - abs(massPointArray[idx].position[axis]));
						massPointArray[idx].Velocity[axis] = -massPointArray[idx].Velocity[axis];
					}
				}
			}

			// Clear force
			massPointArray[idx].force = Vec3(0, 0, 0);
			cout << "New Velocity" << massPointArray[idx].Velocity << endl;
		}
		cout << "*************** New Timestep of " << timeStep << " ***************" << endl;
		break;

	case MIDPOINT:
		Vec3 xtmp[100];
		Vec3 vtmp[100];
		Vec3 ftmp[100];
		for (int idx = 0; idx < 100; idx++)
		{
			xtmp[idx] = Vec3(0, 0, 0);
			vtmp[idx] = Vec3(0, 0, 0);
			ftmp[idx] = Vec3(0, 0, 0);
		}
		// Compute midpoint position with old velocity
		for(int idx = 0; idx < numMassPoint; idx++)
		{
			xtmp[idx] = massPointArray[idx].position + timeStep / 2 * massPointArray[idx].Velocity;
			
		}
		for (int idx = 0; idx < numSpring; idx++)
		{
			Vec3 pos1 = massPointArray[springArray[idx].masspoint1].position;
			Vec3 pos2 = massPointArray[springArray[idx].masspoint2].position;
			Vec3 distanceVec = pos1 - pos2;
			// Calculate length of spring
			springArray[idx].length = sqrt(distanceVec[0] * distanceVec[0] + distanceVec[1] * distanceVec[1] + distanceVec[2] * distanceVec[2]);

			// Calculate force of springs
			float springForce = -m_fStiffness * (springArray[idx].length - springArray[idx].initialLength);

			// Direction of force
			Vec3 dir1 = distanceVec / springArray[idx].length;

			massPointArray[springArray[idx].masspoint1].force += springForce * dir1;
			massPointArray[springArray[idx].masspoint2].force += -springForce * dir1;
		}
		
		// Update velocity for Midpoints
		for (int idx = 0; idx < numMassPoint; idx++)
		{
			// Sum of force
			massPointArray[idx].force -= m_fDamping * massPointArray[idx].Velocity;
			massPointArray[idx].force += m_externalForce;
			Vec3 forceSum = massPointArray[idx].force;
			// Acceleration (Newton's 2nd Law)
			Vec3 acc = forceSum / m_fMass;
			if (massPointArray[idx].isFixed)
			{
				vtmp[idx] = Vec3(0, 0, 0);
			}
			else
			{
				vtmp[idx] = massPointArray[idx].Velocity + (acc * timeStep/2);
			}
			// Clear force
			massPointArray[idx].force = Vec3(0, 0, 0);
		}

		// Update position using Midpoint velocity
		for (int idx = 0; idx < numMassPoint; idx++)
		{
			massPointArray[idx].position += vtmp[idx] * timeStep;
		}

		// Update force for Midpoints
		for (int idx = 0; idx < numSpring; idx++)
		{
			Vec3 pos1 = xtmp[springArray[idx].masspoint1];
			Vec3 pos2 = xtmp[springArray[idx].masspoint2];
			Vec3 distanceVec = pos1 - pos2;
			// Calculate length of spring
			springArray[idx].length = sqrt(distanceVec[0] * distanceVec[0] + distanceVec[1] * distanceVec[1] + distanceVec[2] * distanceVec[2]);

			// Calculate force of springs
			float springForce = -m_fStiffness * (springArray[idx].length - springArray[idx].initialLength);

			// Direction of force
			Vec3 dir1 = distanceVec / springArray[idx].length;

			ftmp[springArray[idx].masspoint1] += springForce * dir1;
			ftmp[springArray[idx].masspoint2] += -springForce * dir1;
		}

		// Update velocity for Points
		for (int idx = 0; idx < numMassPoint; idx++)
		{
			// Sum of force
			ftmp[idx] -= m_fDamping * vtmp[idx];
			ftmp[idx] += m_externalForce;
			Vec3 forceSum = ftmp[idx];
			// Acceleration (Newton's 2nd Law)
			Vec3 acc = forceSum / m_fMass;
			if (massPointArray[idx].isFixed)
			{
				massPointArray[idx].Velocity = Vec3(0, 0, 0);
			}
			else
				massPointArray[idx].Velocity += (acc * timeStep);
			}
			// Clear force
			ftmp[idx] = Vec3(0, 0, 0);

			// Deal with collision
			if (wallCollision)
			{
				for (int axis = 0; axis < 3; axis++)
				{
					if (massPointArray[idx].position[axis] > 0.5 || massPointArray[idx].position[axis] < -0.5)
					{
						int sign = abs(massPointArray[idx].position[axis]) / massPointArray[idx].position[axis];
						massPointArray[idx].position[axis] = sign * (1 - abs(massPointArray[idx].position[axis]));
						massPointArray[idx].Velocity[axis] = -massPointArray[idx].Velocity[axis];
					}
				}
			}
		}
		break;
	//default:
		//break;
	}
	

}
void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

// Specific Functions
void MassSpringSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
	return;
}
void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
	return;
}
void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
	return;
}
int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	MassPoint newMassPoint;
	newMassPoint.position = position;
	newMassPoint.Velocity = Velocity;
	newMassPoint.isFixed = isFixed;
	newMassPoint.force = Vec3(0,0,0);
	massPointArray[numMassPoint] = newMassPoint;
	numMassPoint++;
	return numMassPoint - 1;
}
void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	Spring newSpring;
	newSpring.masspoint1 = masspoint1;
	newSpring.masspoint2 = masspoint2;
	newSpring.initialLength = initialLength;
	newSpring.length = 0;

	Vec3 pos1 = massPointArray[masspoint1].position;
	Vec3 pos2 = massPointArray[masspoint2].position;

	springArray[numSpring] = newSpring;
	numSpring++;
	return;
}
int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return numMassPoint;
}
int MassSpringSystemSimulator::getNumberOfSprings()
{
	return numSpring;
}
Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return massPointArray[index].position;
}
Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return massPointArray[index].Velocity;
}
void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	m_externalForce = force;
}
