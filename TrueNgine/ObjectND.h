#pragma once

#include <Eigen/Dense>
#include <assert.h>

#include <Qhull.h>

#include "NMatrixOperations.h"

class ObjectND {
public:
	unsigned int N;

	Eigen::MatrixXf vertices;
	int num_edges;
	int* edges;

	//Each vertex location as a column in the matrix
	ObjectND(unsigned int N, Eigen::MatrixXf vertices) {
		assert(vertices.rows() == N);

		this->vertices = vertices;

		orgQhull::Qhull qhull;
		//qhull.
	}
};