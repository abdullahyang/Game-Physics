#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	// rest to be implemented
	length = 15;
	width = 15;
	alpha = 0.2;
}

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	// to be implemented
	switch (m_iTestCase)
	{
	case 0:
		TwAddVarRW(DUC->g_pTweakBar, "length", TW_TYPE_INT32, &length, "min=1");
		TwAddVarRW(DUC->g_pTweakBar, "width", TW_TYPE_INT32, &width, "min=1");
		TwAddVarRW(DUC->g_pTweakBar, "alpha", TW_TYPE_FLOAT, &alpha, "min=0.1 step=0.1");
		break;
	case 1:

		break;
	case 2:break;
	default:break;
	}
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	//
	// to be implemented
	//
	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		length = 15;
		width = 15;
		alpha = 0.2;
		T.setSize(length, width);
		break;
	case 1:
		cout << "Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void DiffusionSimulator::diffuseTemperatureExplicit(float timeStep) {
	Grid T_old = T;
	int length = T.getSize()[0];
	int width = T.getSize()[1];
	for (int idx = 0; idx < length * width; idx++)
	{
		// set zero for boundary cells
		if (idx % length == 0 || (idx + 1) % length == 0 || idx / width == 0 || idx / width == length - 1)
		{
			T.points[idx].value = 0;
		}
		else
		{
			// central difference
			double x_diff = T_old.points[idx + 1].value - 2 * T_old.points[idx].value + T_old.points[idx - 1].value;
			double y_diff = T_old.points[idx + width].value - 2 * T_old.points[idx].value + T_old.points[idx - width].value;
			T.points[idx].value = T_old.points[idx].value + timeStep * alpha * (x_diff + y_diff);
		}
	}
	
}


void DiffusionSimulator::diffuseTemperatureImplicit() {
	// solve A T = b

	// This is just an example to show how to work with the PCG solver,
	const int nx = 5;
	const int ny = 5;
	const int nz = 5;
	const int N = nx * ny * nz;

	SparseMatrix<Real> A(N);
	std::vector<Real> b(N);

	// This is the part where you have to assemble the system matrix A and the right-hand side b!

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(A, b, x, ret_pcg_residual, ret_pcg_iterations, 0);

	// Final step is to extract the grid temperatures from the solution vector x
	// to be implemented
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// Reset grid if size changes
	if (T.getSize()[0] != length || T.getSize()[1] != width)
	{
		T.setSize(length, width);
	}

	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		// feel free to change the signature of this function
		diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		// feel free to change the signature of this function
		diffuseTemperatureImplicit();
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	//visualization
	double scale = 1.0 / length;
	for (int i = 0; i < length * width; i++)
	{
		float val = T.points[i].value;
		if (val < 0)
		{
			DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, -val * 0.05 * Vec3(1.0f, 0.0f, 0.0f));
		}
		else
		{
			DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, val * 0.05 * Vec3(1.0f, 1.0f, 1.0f));
		}
		DUC->drawSphere(Vec3(i%width, i/width, 0) * scale - Vec3(0.5, 0.5, 0), Vec3(0.05, 0.05, 0.05));
	}
	
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
