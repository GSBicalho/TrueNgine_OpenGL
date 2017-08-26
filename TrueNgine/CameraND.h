#pragma once

#include <Eigen/Dense>
#include <assert.h>

#include "NMatrixOperations.h"

const double CAMERA_N_EYE_ANGLE = 45.0f;

class CameraND {
public:
	unsigned int N;

	//This is relative to the world
	Eigen::VectorXf Position;
	Eigen::VectorXf Direction;
	Eigen::MatrixXf Ups; //for everything else

	float EyeAngle;

public:
	CameraND() {}

	CameraND(
		unsigned int n,
		Eigen::VectorXf position,
		Eigen::VectorXf direction,
		float eyeAngle = CAMERA_N_EYE_ANGLE) {
		assert(position.rows() == n);
		assert(n >= 4);

		this->N = n;

		this->Position = position;
		this->Direction = direction;
		this->EyeAngle = eyeAngle;

		this->Ups = Eigen::MatrixXf::Zero(N, N - 2);
		this->Ups(1, 0) = 1.f;
		this->Ups.bottomRightCorner(N - 3, N - 3) = Eigen::MatrixXf::Identity(N - 3, N - 3);

	}

	// Returns the view matrix calculated using Eular Angles and the LookAt Matrix
	Eigen::MatrixXf GetViewProjectionModelMatrix() {
		/*if (N == 3) {
		float nearPlane = 0.1f;
		float farPlane = 1000.0f;
		float aspect = this->ScreenWidth / this->ScreenHeight;

		Eigen::VectorXf to = this->Position.topLeftCorner(3,1);
		to.topLeftCorner(3, 1) += this->Front;

		return viewMatrixN(N, this->Position.topLeftCorner(3, 1), to, this->Up, EyeAngle, nearPlane, farPlane, aspect);
		} else {*/
		Eigen::VectorXf to = this->Position + this->Direction;

		return viewMatrixN(N, this->Position, to, this->Ups, EyeAngle, 0, 0, 0);
		//}
	}
};