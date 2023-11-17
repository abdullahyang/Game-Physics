#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	//m_iTestCase = 0;
	m_fMass = 1;
	m_fStiffness = 0.5;
	m_fDamping = 0;
	m_iIntegrator = EULER;
	// Limit max. 100 MassPoints
	massPointArray = new MassPoint[100];
	// Limit max. 100 Springs
	springArray = new Spring[100];
	numMassPoint = 0;
	numSpring = 0;
}

// TODO: Complete UI Functions to allow UI interaction, see Demo 4 in PDF document
// UI Functions
const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "test";
}
void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{

}
void MassSpringSystemSimulator::reset()
{

}
void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{

}
void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{

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
		}
		cout << "*************** New Timestep of " << timeStep << " ***************" << endl;

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
		// TODO
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
			{
				massPointArray[idx].Velocity += (acc * timeStep);
			}
			// Clear force
			ftmp[idx] = Vec3(0, 0, 0);
		}
		break;
	//default:
		//break;
	}
	

}
void MassSpringSystemSimulator::onClick(int x, int y)
{

}
void MassSpringSystemSimulator::onMouse(int x, int y)
{

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
