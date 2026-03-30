/**
 * CalibrationCalc.cpp
 *
 * Thin wrapper around spacecal::KabschCalibrationSolver.
 * All calibration math is now in spacecalibrator/core/src/kabsch_solver.cpp.
 * This file handles:
 *   - Type conversion between legacy Pose/Sample and spacecal:: types
 *   - Forwarding diagnostic messages to CalCtx.Log()
 *   - Updating Metrics time-series
 *   - Exposing debug fields (m_posOffset, m_newCalRMS, etc.) for the debug UI
 */

#include "CalibrationCalc.h"
#include "Calibration.h"
#include "CalibrationMetrics.h"
#include "..\Protocol.h"

// ---------------------------------------------------------------------------
// Type conversion helpers
// ---------------------------------------------------------------------------

namespace {

inline spacecal::Pose toSpacecalPose(const Pose& p)
{
	return spacecal::Pose(p.rot, p.trans);
}

inline spacecal::Sample toSpacecalSample(const Sample& s)
{
	// timestamp left at 0.0 → temporal weighting returns 1.0 (uniform),
	// matching legacy behaviour. Supply real timestamps here to activate decay.
	return spacecal::Sample(
		toSpacecalPose(s.ref),
		toSpacecalPose(s.target)
	);
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// Statics
// ---------------------------------------------------------------------------

// Use the new solver's tuned constant (0.0005) instead of the old 0.001.
const double CalibrationCalc::AxisVarianceThreshold =
	spacecal::KabschCalibrationSolver::AxisVarianceThreshold;

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

CalibrationCalc::CalibrationCalc()
	: enableStaticRecalibration(true)
	, m_newCalRMS(0.0)
	, m_oldCalRMS(0.0)
	, m_axisVariance(0.0)
	, m_calcCycle(0)
{
	m_posOffset.setZero();
}

// ---------------------------------------------------------------------------
// Sample management
// ---------------------------------------------------------------------------

void CalibrationCalc::PushSample(const Sample& sample)
{
	m_solver.pushSample(toSpacecalSample(sample));
}

void CalibrationCalc::Clear()
{
	m_solver.clear();
}

size_t CalibrationCalc::SampleCount() const
{
	return m_solver.sampleCount();
}

void CalibrationCalc::ShiftSample()
{
	m_solver.shiftOldest(1);
}

// ---------------------------------------------------------------------------
// State accessors
// ---------------------------------------------------------------------------

const Eigen::AffineCompact3d CalibrationCalc::Transformation() const
{
	return m_solver.currentTransformation();
}

const Eigen::Vector3d CalibrationCalc::EulerRotation() const
{
	return m_solver.currentEulerRotation();
}

bool CalibrationCalc::isValid() const
{
	return m_solver.isValid();
}

// ---------------------------------------------------------------------------
// Helper: sync debug fields from solver and log diagnostic messages
// ---------------------------------------------------------------------------

namespace {

void syncDebugFields(
	CalibrationCalc& cc,
	const spacecal::KabschCalibrationSolver& solver,
	const spacecal::CalibrationOutcome& outcome)
{
	cc.m_posOffset    = solver.posOffset();
	cc.m_newCalRMS    = solver.newCalRMS();
	cc.m_oldCalRMS    = solver.oldCalRMS();
	cc.m_axisVariance = solver.axisVariance();
	cc.m_calcCycle    = solver.calcCycle();

	for (const auto& msg : outcome.diagnosticMessages) {
		if (msg != "lerp:true") {
			CalCtx.Log((msg + "\n").c_str());
		}
	}
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// One-shot calibration
// ---------------------------------------------------------------------------

bool CalibrationCalc::ComputeOneshot()
{
	auto outcome = m_solver.computeOneshot();
	syncDebugFields(*this, m_solver, outcome);
	return outcome.applied;
}

// ---------------------------------------------------------------------------
// Incremental (continuous) calibration
// ---------------------------------------------------------------------------

bool CalibrationCalc::ComputeIncremental(bool &lerp, double threshold)
{
	Metrics::RecordTimestamp();

	spacecal::CalibrationParams params;
	params.enableStaticRecalibration = enableStaticRecalibration;
	// threshold was previously: keep old if priorError < newError * threshold
	// new solver:               adopt new if newError < priorError * threshold
	// semantics are equivalent.
	params.hysteresisAdoptThreshold = threshold;

	// Capture validity before the call so we can derive the lerp flag.
	const bool wasValid = m_solver.isValid();

	auto outcome = m_solver.computeIncremental(params);
	syncDebugFields(*this, m_solver, outcome);

	// lerp = "there was a valid calibration before, and we just adopted a new one"
	lerp = wasValid && outcome.applied;

	// Push available metrics (series not listed here will simply have no new point)
	Metrics::axisIndependence.Push(m_axisVariance);
	Metrics::error_rawComputed.Push(m_newCalRMS * 1000.0);
	Metrics::error_currentCal.Push(m_oldCalRMS * 1000.0);

	if (outcome.applied) {
		Metrics::posOffset_rawComputed.Push(m_posOffset * 1000.0);
		Metrics::calibrationApplied.Push(true);
	}

	return outcome.applied;
}
