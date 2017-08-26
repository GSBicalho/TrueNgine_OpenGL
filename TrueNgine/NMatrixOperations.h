#pragma once

#include <Eigen/Dense>
#include <stdexcept>
#include <assert.h>
#include <string.h>
#include <vector>


Eigen::VectorXf cross4(Eigen::VectorXf U, Eigen::VectorXf V, Eigen::VectorXf W) {
	float A, B, C, D, E, F;       // Intermediate Values

								  // Calculate intermediate values.

	A = (V(0) * W(1)) - (V(1) * W(0));
	B = (V(0) * W(2)) - (V(2) * W(0));
	C = (V(0) * W(3)) - (V(3) * W(0));
	D = (V(1) * W(2)) - (V(2) * W(1));
	E = (V(1) * W(3)) - (V(3) * W(1));
	F = (V(2) * W(3)) - (V(3) * W(2));

	// Calculate the result-vector components.
	Eigen::VectorXf result(4);

	result(0) = (U(1) * F) - (U(2) * E) + (U(3) * D);
	result(1) = -(U(0) * F) + (U(2) * C) - (U(3) * B);
	result(2) = (U(0) * E) - (U(1) * C) + (U(3) * A);
	result(3) = -(U(0) * D) + (U(1) * B) - (U(2) * A);

	return result;
}

// This is working
// And by working I mean I tested with an equivalent method in 4D and the same results came up
// TODO: Remake it using Eigen methods
Eigen::VectorXf getNormalVector(Eigen::MatrixXf vectors) {
	assert(vectors.rows() == vectors.cols() + 1); //Matrix size invalid. Must be (N, N-1)

	Eigen::MatrixXf pV = vectors.transpose();
	const unsigned int d = vectors.rows();

	//std::cout << "Normal -> pV:\n" << pV << "\n";

	Eigen::MatrixXf pM = Eigen::MatrixXf::Zero(d, d);
	Eigen::MatrixXf baseVectors = Eigen::MatrixXf::Zero(d, d);

	for (unsigned int i = 0; i < (d - 1); i++) {
		for (unsigned int j = 0; j < d; j++) {
			pM(i, j) = pV(i, j);
			baseVectors(i, j) = ((i == j) ? 1.f : 0.f);
		}
	}

	for (unsigned int j = 0; j < d; j++) {
		const unsigned int i = d - 1;
		baseVectors(i, j) = ((i == j) ? 1.f : 0.f);
	}

	//std::cout << "Normal -> pM:\n" << pM << "\n";

	Eigen::VectorXf rv = Eigen::VectorXf::Zero(d);

	for (unsigned int i = 0; i < d; i++) {
		Eigen::MatrixXf pS(d - 1, d - 1);

		for (unsigned int j = 0, r = 0; j < (d - 1); j++, r++) {
			for (unsigned int k = 0, c = 0; k < d; k++) {
				if (k == i) {
					continue;
				}

				pS(r, c) = pM(j, k);

				c++;
			}
		}

		if ((i % 2) == 0) {
			rv += baseVectors.row(i) * pS.determinant();
		} else {
			rv -= baseVectors.row(i) * pS.determinant();
		}

		//std::cout << "Normal -> pS " << i << ":\n" << pS << "\n";
		//std::cout << "Normal -> pS" << i << " deteminant:\n" << pS.determinant() << "\n";
		//std::cout << "Normal -> rV " << i << ":\n" << rv << "\n";
	}

	//std::cout << "Normal -> Result:\n" << rv << "\n";
	return rv;

	/*
	std::cout << "Getting Normal Vector...\n";

	std::cout << "Normal -> Vectors:\n" << vectors << "\n";

	const int N = vectors.rows();

	//Copy vectors to m and initialize its last column with zeroes
	Eigen::MatrixXf m(N, N);
	m.topLeftCorner(N, N - 1) = vectors;
	m.rightCols(1) = Eigen::VectorXf::Zero(N);
	std::cout << "Normal -> Starting Matrix:\n" << m << "\n";

	//Initialize baseVectors with the Identity matrix
	Eigen::MatrixXf baseVectors = Eigen::MatrixXf::Identity(N, N);

	Eigen::VectorXf returnVector = Eigen::VectorXf::Zero(N);
	for (int i = 0; i < N; i++) {
	Eigen::MatrixXf pS(N - 1, N - 1);

	for (unsigned int j = 0, r = 0; j < (N - 1); j++, r++) {
	for (unsigned int k = 0, c = 0; k < N; k++) {
	if (k == i) {
	continue;
	}

	pS(r, c) = m(j, k);

	c++;
	}
	}

	std::cout << "Normal -> pS" << i << " deteminant:\n" << pS.determinant() << "\n";

	if ((i % 2) == 0) {
	returnVector += baseVectors.col(i) * pS.determinant();
	} else {
	returnVector -= baseVectors.col(i) * pS.determinant();
	}

	//pS.leftCols(i) = m.topLeftCorner(N - 1, i);
	//pS.rightCols(N - i - 1) = m.topRightCorner(N - 1, N - i - 1);
	//returnVector += ((i % 2 ? -1 : 1) * pS.determinant()) * baseVectors.col(i);

	std::cout << "Normal -> pS " << i << ":\n" << pS << "\n";
	std::cout << "Normal -> returnVector " << i << ":\n" << returnVector << "\n";
	}

	std::cout << "Normal -> Result:\n" << returnVector << "\n";
	return returnVector;

	*/
}

Eigen::MatrixXf translateMatrixN(int N, Eigen::VectorXf point) {
	assert(N > 0);

	Eigen::MatrixXf m = Eigen::MatrixXf::Identity(N + 1, N + 1);
	m.topRightCorner(N, 1) = point;
	return m;
}

Eigen::MatrixXf Calc4Matrix(
	Eigen::VectorXf to,
	Eigen::VectorXf from,
	Eigen::MatrixXf ups) {

	Eigen::VectorXf Up = ups.col(0);
	Eigen::VectorXf Over = ups.col(1);

	// Get the normalized Wd column-vector.
	Eigen::VectorXf Wd = (to - from).normalized();

	// Calculate the normalized Wa column-vector.
	Eigen::VectorXf Wa = cross4(Up, Over, Wd).normalized();

	// Calculate the normalized Wb column-vector.
	Eigen::VectorXf Wb = cross4(Over, Wd, Wa).normalized();

	// Calculate the Wc column-vector.
	Eigen::VectorXf Wc = cross4(Wd, Wa, Wb).normalized();

	Eigen::MatrixXf m(4, 4);
	m.col(0) = Wa;
	m.col(1) = Wb;
	m.col(2) = Wc;
	m.col(3) = Wd;

	Eigen::MatrixXf temp = Eigen::MatrixXf::Identity(4 + 1, 4 + 1);
	temp.topLeftCorner(4, 4) = m;
	return temp.transpose();
}

// This was based on https://ef.gy/linear-algebra:perspective-projections
// However, he makes this in a different way than every other LookAt I could find
// Therefor it has been silightly adjusted to conform to the others
Eigen::MatrixXf lookAtMatrixN(const int N, Eigen::VectorXf from, Eigen::VectorXf to, Eigen::MatrixXf ups) {
	assert(N > 2);

	//std::cout << "LookAt Calculation:\n";

	Eigen::MatrixXf m(N, N);

	m.rightCols(1) = (to - from).normalized();

	//std::cout << "Starting Matrix:\n" << m << "\n";

	int numLoops = 0;
	for (int currentColumn = N - 2; currentColumn > 0; currentColumn--) {
		Eigen::MatrixXf vectorsToCross(N, N - 1);
		int currentColumnOnVectorsToCross = 1;

		//First, cross product all ups, in order
		vectorsToCross.col(0) = ups.col(numLoops);
		for (int i = 1; i < currentColumn; i++) {
			vectorsToCross.col(currentColumnOnVectorsToCross) = ups.col(numLoops + i);
			currentColumnOnVectorsToCross++;
		}

		numLoops++;
		for (int i = 0; i < numLoops; i++) {
			vectorsToCross.col(currentColumnOnVectorsToCross) = m.col(currentColumn + i + 1);
			currentColumnOnVectorsToCross++;
		}

		//std::cout << "vectorsToCross\n" << vectorsToCross << "\n";

		auto normal = getNormalVector(vectorsToCross);
		//std::cout << "Normal:\n" << normal << "\n";

		m.col(currentColumn) = normal.normalized();

		//std::cout << "Current Matrix:\n" << m << "\n";
	}

	m.col(0) = getNormalVector(m.rightCols(N - 1)).normalized();

	Eigen::MatrixXf temp = Eigen::MatrixXf::Identity(N + 1, N + 1);
	m.leftCols(N - 1).rowwise().reverseInPlace();
	temp.topLeftCorner(N, N) = -m;

	return temp.transpose();
}

Eigen::MatrixXf perspectiveMatrixN(const int N, float eye_radians_angle, float nearPlane, float farPlane, float aspectRatio) {
	assert(N > 2);

	if (N == 3) {
		Eigen::MatrixXf m = Eigen::MatrixXf::Zero(N + 1, N + 1);

		float f_tan = 1 / tan(eye_radians_angle / 2);
		m(0, 0) = f_tan / aspectRatio;
		m(1, 1) = f_tan;
		m(2, 2) = (nearPlane + farPlane) / (nearPlane - farPlane);
		m(2, 3) = -1.f;
		m(3, 2) = 2 * (nearPlane*farPlane) / (nearPlane - farPlane);

		return m.transpose();
	} else {
		Eigen::MatrixXf m = Eigen::MatrixXf::Identity(N + 1, N + 1);

		double f_tan = 1 / tan(eye_radians_angle / 2);
		m = m*f_tan;
		m(N - 1, N - 1) = 1;
		m(N, N) = 1;

		return m;
	}
}

Eigen::VectorXf project4Dto3D(
	Eigen::VectorXf to,
	Eigen::VectorXf from,
	Eigen::MatrixXf ups,
	float eye_radians_angle,
	Eigen::VectorXf point,
	bool print) {

	Eigen::VectorXf pointWith1(point.rows() + 1);
	pointWith1.topRightCorner(point.rows(), 1) = point;
	pointWith1(point.rows()) = 1;

	auto tr = translateMatrixN(4, -from);
	auto la = Calc4Matrix(to, from, ups);
	auto pr = perspectiveMatrixN(4, eye_radians_angle, 0, 0, 0);

	Eigen::MatrixXf viewMatrix = pr * la * tr;
	if (print) {
		std::cout << "4tr\n" << tr << "\n";
		std::cout << "4la\n" << la << "\n";
		std::cout << "4pr\n" << pr << "\n";
		std::cout << "viewMatrix\n" << viewMatrix << "\n";
	}
	Eigen::VectorXf matrixMethod = viewMatrix * pointWith1;
	//matrixMethod = la * matrixMethod;
	//matrixMethod = pr * matrixMethod;

	//TODO: Check if this if should exist
	if (matrixMethod(3)) {
		matrixMethod /= matrixMethod(3);
	}

	return matrixMethod;
}

// N > 3
Eigen::MatrixXf viewMatrixN(
	const int N,
	Eigen::VectorXf from,
	Eigen::VectorXf to,
	Eigen::MatrixXf ups,
	float eyeRadiansAngle,
	float nearPlane,
	float farPlane,
	float aspectRatio) {

	//std::cout << "N: " << N << "\n";
	//std::cout << "to\n" << to << "\n";
	//std::cout << "from\n" << from << "\n";
	//std::cout << "ups\n" << ups << "\n";
	//std::cout << "eye_angle: " << eyeRadiansAngle << "\n";

	auto tr = translateMatrixN(N, -from);
	//std::cout << "Ntr\n" << tr << "\n";

	auto la = lookAtMatrixN(N, from, to, ups);
	//std::cout << "Nla\n" << la << "\n";
	//la = Calc4Matrix(to, from, ups);
	//std::cout << "newLookAt\n" << la << "\n";

	auto pm = perspectiveMatrixN(N, eyeRadiansAngle, nearPlane, farPlane, aspectRatio);
	//std::cout << "Npm\n" << pm << "\n";

	auto result = pm * la * tr;

	//std::cout << "Nresult\n" << result << "\n";

	return result;
}

//This assumes that point is an N+1 dimensional vector and that point(N) == 1
Eigen::VectorXf projectPointLosingDimension(Eigen::VectorXf point, Eigen::MatrixXf m) {
	std::cout << "point\n" << point << "\n";

	Eigen::VectorXf pointWith1(point.rows() + 1);
	pointWith1.topRightCorner(point.rows(), 1) = point;
	pointWith1(point.rows()) = 1;

	std::cout << "pointWith1\n" << pointWith1 << "\n";

	Eigen::VectorXf v = m * pointWith1;
	std::cout << "v\n" << v << "\n";
	v = v / v(v.rows() - 2, 0);
	std::cout << "v\n" << v << "\n";
	auto result = v.topLeftCorner(v.rows() - 1, 1);
	std::cout << "result\n" << result << "\n";

	return result;
}

//This assumes that each point is an N+1 dimensional column in a matrix and that point(N) == 1
Eigen::MatrixXf projectPointsLosingDimension(Eigen::MatrixXf points, Eigen::MatrixXf m) {
	//std::cout << "point\n" << points << "\n";

	Eigen::MatrixXf pointWith1(points.rows() + 1, points.cols());
	pointWith1.topRightCorner(points.rows(), points.cols()) = points;
	pointWith1.row(points.rows()) = Eigen::VectorXf::Ones(points.cols());

	//std::cout << "pointWith1\n" << pointWith1 << "\n";

	Eigen::MatrixXf v = m * pointWith1;
	//std::cout << "v\n" << v << "\n";
	for (int i = 0; i < v.cols(); i++) {
		v.col(i) = v.col(i) / v.col(i)(v.rows() - 2, 0);
	}

	//std::cout << "v\n" << v << "\n";
	Eigen::MatrixXf result = v.topLeftCorner(v.rows() - 1, v.cols());
	//std::cout << "result\n" << result << "\n";

	return result;
}

Eigen::MatrixXf rotateMatrixN(const unsigned int N, unsigned int axis1, unsigned int axis2, float radians_angle) {
	assert(axis1 != axis2);

	Eigen::MatrixXf rot_aa = Eigen::MatrixXf::Identity(N, N);

	rot_aa(axis1, axis1) = cos(radians_angle);
	rot_aa(axis1, axis2) = sin(radians_angle);
	rot_aa(axis2, axis1) = -sin(radians_angle);
	rot_aa(axis2, axis2) = cos(radians_angle);

	return rot_aa.transpose();
}

std::string generateNDimensionalShader(int n) {
	std::stringstream ss;
	ss << "#version 330 core\nlayout (location = 0) in float[" << n << "] position;\n";

	for (int i = n; i >= 3; i--) {
		ss << "uniform float m" << i << "[" << ((i + 1) * (i + 1)) << "];\n";
	}

	ss << "\nvoid main() {\n";

	ss << "	float[" << (n + 1) << "] positionWithOne;\n";
	ss << "	for (int i = 0; i < " << n << "; i++){\n";
	ss << "		positionWithOne[i] = position[i];\n";
	ss << "	}\n";
	ss << "	positionWithOne[" << n << "] = 1;\n\n";

	ss << "	float[" << (n + 1) << "] newPos" << n << ";\n";

	for (int i = n; i >= 3; i--) {
		ss << "	for (int i = 0; i <= " << i << "; i++) {\n";
		ss << "		float newVal = 0;\n";
		ss << "		for (int j = 0; j <= " << i << "; j++){\n";
		ss << "			newVal += m" << i << "[j * " << i << " + i] * positionWithOne[j];\n";
		ss << "		}\n";
		ss << "		newPos" << i << "[i] = newVal;\n";
		ss << "	}\n";
		ss << "	\n";
		if (i != 3) {
			ss << "	for (int i = 0; i < " << i << "; i++) {\n";
			ss << "		positionWithOne[i] = newPos" << i << "[i] / newPos" << i << "[" << (i - 1) << "];\n";
			ss << "	}\n";
			ss << "	\n";
			ss << "	float[" << i << "] newPos" << i - 1 << ";\n";
		}
	}

	ss << "	gl_Position.x = newPos3[0]/newPos3[3];\n";
	ss << "	gl_Position.y = newPos3[1]/newPos3[3];\n";
	ss << "	gl_Position.z = newPos3[2]/newPos3[3];\n";
	ss << "	gl_Position.w = newPos3[3]/newPos3[3];\n";
	ss << "}\n";

	std::string str = ss.str();
	return str;
}
