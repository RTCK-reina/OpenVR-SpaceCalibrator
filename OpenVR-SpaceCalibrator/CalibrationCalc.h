#pragma once

#include <Eigen/Dense>
#include <openvr.h>
#include <vector>
#include <deque>
#include <iostream>
#include <memory>

// New solver (pure Eigen, no platform deps)
#include <spacecal/core/calibration_solver.h>
#include <spacecal/core/calibration_policy.h>

// ---------------------------------------------------------------------------
// Legacy types kept for Calibration.cpp / CalibrationDebug.cpp compatibility
// ---------------------------------------------------------------------------

struct Pose
{
	Eigen::Matrix3d rot;
	Eigen::Vector3d trans;

	Pose() { }
	Pose(const Eigen::AffineCompact3d& transform) {
		rot = transform.rotation();
		trans = transform.translation();
	}

	Pose(vr::HmdMatrix34_t hmdMatrix)
	{
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				rot(i, j) = hmdMatrix.m[i][j];
			}
		}
		trans = Eigen::Vector3d(hmdMatrix.m[0][3], hmdMatrix.m[1][3], hmdMatrix.m[2][3]);
	}
	Pose(vr::HmdQuaternion_t rot, const double *trans) {
		this->rot = Eigen::Matrix3d(Eigen::Quaterniond(rot.w, rot.x, rot.y, rot.z));
		this->trans = Eigen::Vector3d(trans[0], trans[1], trans[2]);
	}
	Pose(double x, double y, double z) : trans(Eigen::Vector3d(x, y, z)) { }

	Eigen::Matrix4d ToAffine() const {
		Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				matrix(i, j) = rot(i, j);
			}
			matrix(i, 3) = trans(i);
		}

		return matrix;
	}
};

struct Sample
{
	Pose ref, target;
	bool valid;
	Sample() : valid(false) { }
	Sample(Pose ref, Pose target) : valid(true), ref(ref), target(target) { }
};

// ---------------------------------------------------------------------------
// CalibrationCalc: thin wrapper around KabschCalibrationSolver.
// Public API is unchanged so Calibration.cpp requires no modification.
// ---------------------------------------------------------------------------

class CalibrationCalc {
public:
	/// Axis variance threshold (mirrors KabschCalibrationSolver::AxisVarianceThreshold)
	static const double AxisVarianceThreshold;

	bool enableStaticRecalibration;

	const Eigen::AffineCompact3d Transformation() const;
	const Eigen::Vector3d EulerRotation() const;
	bool isValid() const;

	void PushSample(const Sample& sample);
	void Clear();

	bool ComputeOneshot();
	bool ComputeIncremental(bool &lerp, double threshold);

	size_t SampleCount() const;
	void ShiftSample();

	CalibrationCalc();

	// Debug fields (populated after each Compute* call, read by CalibrationDebug.cpp)
	Eigen::Vector3d m_posOffset;
	double m_newCalRMS, m_oldCalRMS, m_axisVariance;
	long m_calcCycle;

private:
	spacecal::KabschCalibrationSolver m_solver;
};
