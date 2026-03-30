#include "stdafx.h"
#include "UserInterface.h"
#include "Calibration.h"
#include "Configuration.h"
#include "VRState.h"
#include "CalibrationMetrics.h"
#include "../Version.h"

#include <thread>
#include <string>
#include <vector>
#include <algorithm>
#include <imgui/imgui.h>

// Forward declarations
void TextWithWidth(const char *label, const char *text, float width);
VRState LoadVRState();
void BuildSystemSelection(const VRState &state);
void BuildDeviceSelections(const VRState &state);
void BuildProfileEditor();
void BuildMenu(bool runningInOverlay);

static const ImGuiWindowFlags bareWindowFlags =
	ImGuiWindowFlags_NoTitleBar |
	ImGuiWindowFlags_NoResize |
	ImGuiWindowFlags_NoMove |
	ImGuiWindowFlags_NoScrollbar |
	ImGuiWindowFlags_NoScrollWithMouse |
	ImGuiWindowFlags_NoCollapse;

void BuildContinuousCalDisplay();
void ShowVersionLine();
void CCal_BasicInfo();
void CCal_AlignParams();

static bool runningInOverlay;

// ============================================================
// Color palette constants
// ============================================================
static const ImVec4 kAccent        = { 0.32f, 0.78f, 0.70f, 1.0f };
static const ImVec4 kAccentBright  = { 0.48f, 0.95f, 0.85f, 1.0f };
static const ImVec4 kStatusOK      = { 0.28f, 0.82f, 0.65f, 1.0f };
static const ImVec4 kStatusErr     = { 1.0f,  0.38f, 0.28f, 1.0f };
static const ImVec4 kStatusWarn    = { 0.95f, 0.65f, 0.20f, 1.0f };
static const ImVec4 kTextDim       = { 0.50f, 0.53f, 0.57f, 1.0f };
static const ImVec4 kTextMuted     = { 0.38f, 0.40f, 0.44f, 1.0f };

// ============================================================
// UI Helpers
// ============================================================

// Styled section header with left accent bar and tinted background
static void SectionHeader(const char* label) {
	ImGui::Spacing();
	float avail = ImGui::GetContentRegionAvail().x;
	ImVec2 p = ImGui::GetCursorScreenPos();
	float textH = ImGui::GetTextLineHeight();
	float barH  = textH + 10.0f;

	auto* dl = ImGui::GetWindowDrawList();
	// Background
	dl->AddRectFilled(p, ImVec2(p.x + avail, p.y + barH), IM_COL32(20, 58, 54, 215), 3.0f);
	// Left accent bar
	dl->AddRectFilled(p, ImVec2(p.x + 3.5f,  p.y + barH), IM_COL32(82, 200, 175, 255), 2.0f);

	// Text inside the bar
	ImGui::SetCursorScreenPos(ImVec2(p.x + 13.0f, p.y + 5.0f));
	ImGui::TextColored(kAccentBright, "%s", label);

	// Advance cursor past the bar
	ImVec2 afterText = ImGui::GetCursorScreenPos();
	float barBottom  = p.y + barH + 3.0f;
	if (afterText.y < barBottom)
		ImGui::SetCursorScreenPos(ImVec2(p.x, barBottom));
	ImGui::Spacing();
}

// Push danger (red) button colors
static void PushDanger() {
	ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.50f, 0.18f, 0.18f, 1.0f));
	ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.62f, 0.24f, 0.24f, 1.0f));
	ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.70f, 0.30f, 0.30f, 1.0f));
}
static void PopDanger() { ImGui::PopStyleColor(3); }

// Push muted/secondary button colors
static void PushMuted() {
	ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.20f, 0.22f, 0.26f, 1.0f));
	ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.27f, 0.29f, 0.34f, 1.0f));
	ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.33f, 0.35f, 0.41f, 1.0f));
}
static void PopMuted() { ImGui::PopStyleColor(3); }

// Push selected/active toggle button colors
static void PushSelected() {
	ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.22f, 0.52f, 0.47f, 1.0f));
	ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.28f, 0.60f, 0.54f, 1.0f));
	ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.34f, 0.66f, 0.60f, 1.0f));
}
static void PopSelected() { ImGui::PopStyleColor(3); }

// ============================================================
// Tooltip helper - draws (?) icon with hover text
// ============================================================
static void HelpTip(const char* text) {
	ImGui::SameLine();
	ImGui::TextColored(kTextMuted, "(?)");
	if (ImGui::IsItemHovered()) {
		ImGui::BeginTooltip();
		ImGui::PushTextWrapPos(ImGui::GetFontSize() * 22.0f);
		ImGui::TextUnformatted(text);
		ImGui::PopTextWrapPos();
		ImGui::EndTooltip();
	}
}

// ============================================================
// Calibration Parameter Presets
// ============================================================
enum class CalPreset { Quick, Balanced, Precision, Custom };
static CalPreset sCurrentPreset = CalPreset::Balanced;

struct PresetValues {
	float continuousCalibrationThreshold;
	double thr_trans_tiny, thr_trans_small, thr_trans_large;
	double thr_rot_tiny, thr_rot_small, thr_rot_large;
	double align_speed_tiny, align_speed_small, align_speed_large;
};

static const PresetValues kPresetQuick = {
	3.0f,
	0.5 / 1000.0, 2.0 / 1000.0, 25.0 / 1000.0,
	0.8 * (EIGEN_PI / 180.0), 1.0 * (EIGEN_PI / 180.0), 8.0 * (EIGEN_PI / 180.0),
	1.5, 1.5, 1.5
};

static const PresetValues kPresetBalanced = {
	1.5f,
	0.2 / 1000.0, 1.0 / 1000.0, 20.0 / 1000.0,
	0.49 * (EIGEN_PI / 180.0), 0.5 * (EIGEN_PI / 180.0), 5.0 * (EIGEN_PI / 180.0),
	1.0, 1.0, 1.0
};

static const PresetValues kPresetPrecision = {
	1.1f,
	0.1 / 1000.0, 0.5 / 1000.0, 10.0 / 1000.0,
	0.25 * (EIGEN_PI / 180.0), 0.3 * (EIGEN_PI / 180.0), 3.0 * (EIGEN_PI / 180.0),
	0.6, 0.7, 0.8
};

static void ApplyPreset(const PresetValues& p) {
	CalCtx.continuousCalibrationThreshold = p.continuousCalibrationThreshold;
	CalCtx.alignmentSpeedParams.thr_trans_tiny  = p.thr_trans_tiny;
	CalCtx.alignmentSpeedParams.thr_trans_small = p.thr_trans_small;
	CalCtx.alignmentSpeedParams.thr_trans_large = p.thr_trans_large;
	CalCtx.alignmentSpeedParams.thr_rot_tiny    = p.thr_rot_tiny;
	CalCtx.alignmentSpeedParams.thr_rot_small   = p.thr_rot_small;
	CalCtx.alignmentSpeedParams.thr_rot_large   = p.thr_rot_large;
	CalCtx.alignmentSpeedParams.align_speed_tiny  = p.align_speed_tiny;
	CalCtx.alignmentSpeedParams.align_speed_small = p.align_speed_small;
	CalCtx.alignmentSpeedParams.align_speed_large = p.align_speed_large;
}

static bool MatchesPreset(const PresetValues& p) {
	auto& ap = CalCtx.alignmentSpeedParams;
	const float eps = 0.001f;
	return std::abs(CalCtx.continuousCalibrationThreshold - p.continuousCalibrationThreshold) < eps
		&& std::abs(ap.thr_trans_tiny  - p.thr_trans_tiny)  < 1e-8
		&& std::abs(ap.thr_trans_small - p.thr_trans_small) < 1e-8
		&& std::abs(ap.thr_trans_large - p.thr_trans_large) < 1e-8
		&& std::abs(ap.thr_rot_tiny    - p.thr_rot_tiny)    < 1e-8
		&& std::abs(ap.thr_rot_small   - p.thr_rot_small)   < 1e-8
		&& std::abs(ap.thr_rot_large   - p.thr_rot_large)   < 1e-8
		&& std::abs(ap.align_speed_tiny  - p.align_speed_tiny)  < 1e-6
		&& std::abs(ap.align_speed_small - p.align_speed_small) < 1e-6
		&& std::abs(ap.align_speed_large - p.align_speed_large) < 1e-6;
}

static void DetectCurrentPreset() {
	if (MatchesPreset(kPresetQuick))         sCurrentPreset = CalPreset::Quick;
	else if (MatchesPreset(kPresetBalanced)) sCurrentPreset = CalPreset::Balanced;
	else if (MatchesPreset(kPresetPrecision))sCurrentPreset = CalPreset::Precision;
	else                                     sCurrentPreset = CalPreset::Custom;
}

// Draw a colored status badge at cursor position
static void DrawStatusBadge(bool found, bool tracking) {
	float w  = ImGui::GetContentRegionAvail().x;
	ImVec2 p = ImGui::GetCursorScreenPos();
	float bh = ImGui::GetTextLineHeight() + 8.0f;
	auto* dl = ImGui::GetWindowDrawList();

	if (!found) {
		dl->AddRectFilled(p, ImVec2(p.x + w, p.y + bh), IM_COL32(140, 50, 20, 170), 3.0f);
		ImGui::SetCursorScreenPos(ImVec2(p.x + 9.0f, p.y + 4.0f));
		ImGui::TextColored(kStatusWarn, "  NOT FOUND");
	} else if (!tracking) {
		dl->AddRectFilled(p, ImVec2(p.x + w, p.y + bh), IM_COL32(160, 40, 30, 160), 3.0f);
		ImGui::SetCursorScreenPos(ImVec2(p.x + 9.0f, p.y + 4.0f));
		ImGui::TextColored(kStatusErr, "  NOT TRACKING");
	} else {
		dl->AddRectFilled(p, ImVec2(p.x + w, p.y + bh), IM_COL32(20, 90, 65, 160), 3.0f);
		ImGui::SetCursorScreenPos(ImVec2(p.x + 9.0f, p.y + 4.0f));
		ImGui::TextColored(kStatusOK, "  TRACKING");
	}

	float bottom = p.y + bh + 2.0f;
	if (ImGui::GetCursorScreenPos().y < bottom)
		ImGui::SetCursorScreenPos(ImVec2(p.x, bottom));
}

// ============================================================
// Main Window
// ============================================================

void BuildMainWindow(bool runningInOverlay_)
{
	runningInOverlay = runningInOverlay_;
	bool continuousCalibration = CalCtx.state == CalibrationState::Continuous
		|| CalCtx.state == CalibrationState::ContinuousStandby;

	auto &io = ImGui::GetIO();

	ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);
	ImGui::SetNextWindowSize(io.DisplaySize, ImGuiCond_Always);

	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(18.0f, 16.0f));
	if (!ImGui::Begin("OpenVRSpaceCalibrator", nullptr, bareWindowFlags))
	{
		ImGui::End();
		ImGui::PopStyleVar();
		return;
	}

	ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImGui::GetStyleColorVec4(ImGuiCol_Button));

	if (continuousCalibration) {
		BuildContinuousCalDisplay();
	}
	else {
		auto state = LoadVRState();

		ImGui::BeginDisabled(CalCtx.state == CalibrationState::Continuous);
		BuildSystemSelection(state);
		ImGui::Spacing();
		BuildDeviceSelections(state);
		ImGui::EndDisabled();
		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		BuildMenu(runningInOverlay);
	}

	ShowVersionLine();

	ImGui::PopStyleColor();
	ImGui::End();
	ImGui::PopStyleVar();
}

// ============================================================
// Footer / Version Line
// ============================================================

void ShowVersionLine() {
	float footerH = ImGui::GetFrameHeightWithSpacing() * 1.9f;
	ImGui::SetNextWindowPos(
		ImVec2(16.0f, ImGui::GetWindowHeight() - footerH));
	if (!ImGui::BeginChild("##footer",
		ImVec2(ImGui::GetWindowWidth() - 32.0f, footerH), false)) {
		ImGui::EndChild();
		return;
	}

	ImGui::Separator();
	ImGui::Spacing();
	ImGui::TextColored(kTextDim,
		"OpenVR Space Calibrator  v" SPACECAL_VERSION_STRING "   by tach / pushrax / bd_");
	if (runningInOverlay) {
		ImGui::SameLine();
		ImGui::TextColored(kTextMuted, " |  close VR overlay to use mouse");
	}
	ImGui::EndChild();
}

// ============================================================
// Continuous Calibration Display
// ============================================================

void BuildContinuousCalDisplay() {
	ImGui::SetNextWindowPos(ImVec2(0, 0));
	ImGui::SetNextWindowSize(ImGui::GetWindowSize());
	ImGui::SetNextWindowBgAlpha(1);
	if (!ImGui::Begin("Continuous Calibration", nullptr,
		bareWindowFlags & ~ImGuiWindowFlags_NoTitleBar))
	{
		ImGui::End();
		return;
	}

	ImVec2 contentRegion;
	contentRegion.x = ImGui::GetWindowContentRegionWidth();
	contentRegion.y = ImGui::GetWindowHeight() - ImGui::GetFrameHeightWithSpacing() * 2.1f;

	if (!ImGui::BeginChild("CCalDisplayFrame", contentRegion, false)) {
		ImGui::EndChild();
		return;
	}

	if (ImGui::BeginTabBar("CCalTabs", 0)) {
		if (ImGui::BeginTabItem("  Status  ")) {
			ImGui::Spacing();
			CCal_BasicInfo();
			ImGui::EndTabItem();
		}

		if (ImGui::BeginTabItem("  Graphs  ")) {
			ImGui::Spacing();
			ShowCalibrationDebug(2, 3);
			ImGui::EndTabItem();
		}

		if (ImGui::BeginTabItem("  Parameters  ")) {
			ImGui::Spacing();
			CCal_AlignParams();
			ImGui::EndTabItem();
		}

		ImGui::EndTabBar();
	}

	ImGui::EndChild();
	ShowVersionLine();
	ImGui::End();
}

// ============================================================
// Alignment Parameters Tab
// ============================================================

static void ScaledDragFloat(const char* label, double& f, double scale,
	double min, double max, int flags = ImGuiSliderFlags_AlwaysClamp) {
	float v = (float)(f * scale);
	ImGui::SliderFloat(label, &v, (float)min, (float)max, "%1.2f", flags);
	f = v / scale;
}

void CCal_AlignParams() {
	// --- Preset Mode Selector ---
	DetectCurrentPreset();
	SectionHeader("Parameter Preset");

	ImGui::TextColored(kTextDim,
		"Choose a preset to configure all parameters at once.");
	HelpTip("Quick: low CPU, fewer samples, faster convergence.\n"
		"Balanced: default settings, good for most setups.\n"
		"Precision: highest accuracy, more samples, higher CPU load.");
	ImGui::Spacing();

	{
		ImGuiStyle& pstyle = ImGui::GetStyle();
		float presetBtnH = ImGui::GetTextLineHeight() * 1.7f;
		float presetW = (ImGui::GetContentRegionAvail().x - pstyle.ItemSpacing.x * 3.0f) / 4.0f;

		auto PresetBtn = [&](const char* label, CalPreset preset, const PresetValues& vals, float w) {
			bool sel = (sCurrentPreset == preset);
			if (sel) PushSelected(); else PushMuted();
			if (ImGui::Button(label, ImVec2(w, presetBtnH))) {
				ApplyPreset(vals);
				sCurrentPreset = preset;
			}
			if (sel) PopSelected(); else PopMuted();
		};

		PresetBtn("Quick",     CalPreset::Quick,     kPresetQuick,     presetW);
		ImGui::SameLine();
		PresetBtn("Balanced",  CalPreset::Balanced,   kPresetBalanced,  presetW);
		ImGui::SameLine();
		PresetBtn("Precision", CalPreset::Precision,  kPresetPrecision, presetW);
		ImGui::SameLine();

		// Custom indicator (not clickable, just shows state)
		bool isCust = (sCurrentPreset == CalPreset::Custom);
		if (isCust) {
			ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.35f, 0.30f, 0.15f, 1.0f));
			ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.40f, 0.35f, 0.18f, 1.0f));
			ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.45f, 0.40f, 0.22f, 1.0f));
		} else {
			PushMuted();
		}
		ImGui::Button("Custom", ImVec2(-1, presetBtnH));
		if (isCust) ImGui::PopStyleColor(3); else PopMuted();
	}

	ImGui::Spacing();

	// --- Recalibration Threshold ---
	SectionHeader("Recalibration Threshold");

	ImGui::TextColored(kTextDim,
		"Confidence required before applying a new calibration result.");
	ImGui::Spacing();
	ImGui::SetNextItemWidth(-1);
	ImGui::SliderFloat("##RecalThreshold",
		&CalCtx.continuousCalibrationThreshold, 1.01f, 10.0f, "%.2f", 0);
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip(
			"Higher values calibrate less frequently.\n"
			"Useful on systems with lots of tracker drift.");
	}

	// --- Drift Detection Thresholds ---
	SectionHeader("Drift Detection Thresholds");

	ImGui::TextColored(kTextDim,
		"Amount of drift required before switching to a faster correction speed:");
	HelpTip("Translation thresholds are in millimeters.\n"
		"Rotation thresholds are in degrees.\n"
		"Lower values = more sensitive to drift but may cause jitter.");
	ImGui::Spacing();

	ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, ImVec2(9.0f, 6.0f));
	if (ImGui::BeginTable("SpeedThresholds", 3,
		ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg))
	{
		ImGui::TableSetupColumn("Speed",            ImGuiTableColumnFlags_WidthFixed, 80.0f);
		ImGui::TableSetupColumn("Translation (mm)", ImGuiTableColumnFlags_WidthStretch);
		ImGui::TableSetupColumn("Rotation (deg)",   ImGuiTableColumnFlags_WidthStretch);
		ImGui::TableHeadersRow();

		// Decel row
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::TextColored(kAccent, "Decel");
		ImGui::TableSetColumnIndex(1);
		ImGui::SetNextItemWidth(-1);
		ScaledDragFloat("##TrD",
			CalCtx.alignmentSpeedParams.thr_trans_tiny, 1000.0, 0, 20.0);
		ImGui::TableSetColumnIndex(2);
		ImGui::SetNextItemWidth(-1);
		ScaledDragFloat("##RoD",
			CalCtx.alignmentSpeedParams.thr_rot_tiny, 180.0 / EIGEN_PI, 0, 5.0);

		// Slow row
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::TextColored(kAccent, "Slow");
		ImGui::TableSetColumnIndex(1);
		ImGui::SetNextItemWidth(-1);
		ScaledDragFloat("##TrS",
			CalCtx.alignmentSpeedParams.thr_trans_small, 1000.0,
			CalCtx.alignmentSpeedParams.thr_trans_tiny * 1000.0, 20.0);
		ImGui::TableSetColumnIndex(2);
		ImGui::SetNextItemWidth(-1);
		ScaledDragFloat("##RoS",
			CalCtx.alignmentSpeedParams.thr_rot_small, 180.0 / EIGEN_PI,
			CalCtx.alignmentSpeedParams.thr_rot_tiny * (180.0 / EIGEN_PI), 10.0);

		// Fast row
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::TextColored(kAccent, "Fast");
		ImGui::TableSetColumnIndex(1);
		ImGui::SetNextItemWidth(-1);
		ScaledDragFloat("##TrF",
			CalCtx.alignmentSpeedParams.thr_trans_large, 1000.0,
			CalCtx.alignmentSpeedParams.thr_trans_small * 1000.0, 50.0);
		ImGui::TableSetColumnIndex(2);
		ImGui::SetNextItemWidth(-1);
		ScaledDragFloat("##RoF",
			CalCtx.alignmentSpeedParams.thr_rot_large, 180.0 / EIGEN_PI,
			CalCtx.alignmentSpeedParams.thr_rot_small * (180.0 / EIGEN_PI), 20.0);

		ImGui::EndTable();
	}
	ImGui::PopStyleVar();

	// --- Correction Speeds ---
	SectionHeader("Correction Speed Multipliers");

	ImGui::TextColored(kTextDim,
		"Speed at which calibration is dragged back when drift is detected:");
	HelpTip("Higher multiplier = faster correction but may overshoot.\n"
		"Lower multiplier = smoother correction but slower response.\n"
		"Values above 1.0 are more aggressive, below 1.0 are conservative.");
	ImGui::Spacing();

	ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, ImVec2(9.0f, 6.0f));
	if (ImGui::BeginTable("AlignSpeeds", 4,
		ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg))
	{
		ImGui::TableSetupColumn("",      ImGuiTableColumnFlags_WidthFixed,   70.0f);
		ImGui::TableSetupColumn("Decel", ImGuiTableColumnFlags_WidthStretch);
		ImGui::TableSetupColumn("Slow",  ImGuiTableColumnFlags_WidthStretch);
		ImGui::TableSetupColumn("Fast",  ImGuiTableColumnFlags_WidthStretch);
		ImGui::TableHeadersRow();

		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::TextColored(kTextDim, "Speed");

		ImGui::TableSetColumnIndex(1);
		ImGui::SetNextItemWidth(-1);
		ScaledDragFloat("##AS_D",
			CalCtx.alignmentSpeedParams.align_speed_tiny, 1.0, 0, 2.0, 0);

		ImGui::TableSetColumnIndex(2);
		ImGui::SetNextItemWidth(-1);
		ScaledDragFloat("##AS_S",
			CalCtx.alignmentSpeedParams.align_speed_small, 1.0, 0, 2.0, 0);

		ImGui::TableSetColumnIndex(3);
		ImGui::SetNextItemWidth(-1);
		ScaledDragFloat("##AS_F",
			CalCtx.alignmentSpeedParams.align_speed_large, 1.0, 0, 2.0, 0);

		ImGui::EndTable();
	}
	ImGui::PopStyleVar();

	// --- Reset Button ---
	ImGui::Spacing();
	ImGui::Spacing();
	PushDanger();
	if (ImGui::Button("Reset All Parameters to Defaults",
		ImVec2(-1, ImGui::GetTextLineHeight() * 1.7f)))
	{
		CalCtx.ResetConfig();
	}
	PopDanger();
}

// ============================================================
// Continuous Calibration Status Tab
// ============================================================

void CCal_BasicInfo() {
	ImGuiStyle& style = ImGui::GetStyle();
	float halfW = (ImGui::GetContentRegionAvail().x - style.ItemSpacing.x) * 0.5f;

	// Calculate card height to fit device info + status badge
	float cardH = ImGui::GetTextLineHeightWithSpacing() * 5.5f + style.WindowPadding.y * 2 + 28.0f;

	bool refFound    = CalCtx.referenceID >= 0;
	bool refTracking = CalCtx.ReferencePoseIsValid();
	bool tgtFound    = CalCtx.targetID >= 0;
	bool tgtTracking = CalCtx.TargetPoseIsValid();

	// --- Device Status Cards ---
	auto DrawDeviceCard = [&](const char* cardId, const char* title,
		const StandbyDevice& dev, bool found, bool tracking)
	{
		ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.13f, 0.15f, 0.18f, 1.0f));
		if (ImGui::BeginChild(cardId, ImVec2(halfW, cardH), true)) {
			// Card title
			ImGui::TextColored(kTextDim, "%s", title);
			ImGui::Separator();
			ImGui::Spacing();

			// Device info
			if (!dev.model.empty())
				ImGui::TextWrapped("%s", dev.model.c_str());
			else
				ImGui::TextColored(kTextMuted, "(no device selected)");

			if (!dev.serial.empty())
				ImGui::TextColored(kTextDim, "%s", dev.serial.c_str());
			if (!dev.trackingSystem.empty())
				ImGui::TextColored(kTextDim, "System: %s", dev.trackingSystem.c_str());

			ImGui::Spacing();
			DrawStatusBadge(found, tracking);
		}
		ImGui::EndChild();
		ImGui::PopStyleColor();
	};

	DrawDeviceCard("##RefCard", "Reference Device",
		CalCtx.referenceStandby, refFound, refTracking);
	ImGui::SameLine();
	DrawDeviceCard("##TgtCard", "Target Device",
		CalCtx.targetStandby, tgtFound, tgtTracking);

	ImGui::Spacing();

	// --- Action Buttons ---
	bool hasDebugLog = Metrics::enableLogs;
	int  nCols       = hasDebugLog ? 3 : 2;
	float btnH = ImGui::GetTextLineHeight() * 2.0f;

	if (ImGui::BeginTable("##ActionBtns", nCols, 0,
		ImVec2(ImGui::GetContentRegionAvail().x, btnH + style.FramePadding.y * 2)))
	{
		ImGui::TableNextRow();

		ImGui::TableSetColumnIndex(0);
		PushDanger();
		if (ImGui::Button("Cancel Continuous Calibration",
			ImVec2(-FLT_MIN, 0.0f)))
			EndContinuousCalibration();
		PopDanger();

		ImGui::TableSetColumnIndex(1);
		PushMuted();
		if (ImGui::Button("Debug: Force Break Cal",
			ImVec2(-FLT_MIN, 0.0f)))
			DebugApplyRandomOffset();
		PopMuted();

		if (hasDebugLog) {
			ImGui::TableSetColumnIndex(2);
			PushMuted();
			if (ImGui::Button("Debug: Mark Logs",
				ImVec2(-FLT_MIN, 0.0f)))
				Metrics::WriteLogAnnotation("MARK LOGS");
			PopMuted();
		}

		ImGui::EndTable();
	}

	ImGui::Spacing();

	// --- Options Row ---
	ImGui::Checkbox("Hide target from application",
		&CalCtx.quashTargetInContinuous);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Hides the target device from SteamVR apps.\nUseful to prevent duplicate tracker interference.");
	ImGui::SameLine();
	ImGui::Checkbox("Static recalibration",
		&CalCtx.enableStaticRecalibration);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Recalibrate even when devices are stationary.\nMay improve drift correction but uses more CPU.");
	ImGui::SameLine();
	ImGui::Checkbox("Debug logs", &Metrics::enableLogs);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Write detailed calibration debug logs to file.\nUseful for troubleshooting issues.");

	ImGui::Spacing();

	// --- Message Log ---
	float logH = ImGui::GetTextLineHeightWithSpacing() * 4.0f
		+ style.WindowPadding.y * 2;
	ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.09f, 0.10f, 0.12f, 1.0f));
	if (ImGui::BeginChild("##MsgLog", ImVec2(-1, logH), true)) {
		for (const auto& msg : CalCtx.messages) {
			if (msg.type == CalibrationContext::Message::String) {
				ImGui::PushStyleColor(ImGuiCol_Text, kTextDim);
				ImGui::TextWrapped("> %s", msg.str.c_str());
				ImGui::PopStyleColor();
			}
		}
	}
	ImGui::EndChild();
	ImGui::PopStyleColor();

	ImGui::Spacing();

	// --- Calibration Debug Graphs ---
	ShowCalibrationDebug(1, 3);
}

// ============================================================
// Main Menu (None / Editing / In-Progress States)
// ============================================================

void BuildMenu(bool runningInOverlay)
{
	auto &io = ImGui::GetIO();
	ImGuiStyle &style = ImGui::GetStyle();

	if (CalCtx.state == CalibrationState::None)
	{
		// Warning: profile disabled
		if (CalCtx.validProfile && !CalCtx.enabled)
		{
			ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.28f, 0.09f, 0.07f, 0.55f));
			ImGui::BeginChild("##WarnBox",
				ImVec2(-1, ImGui::GetTextLineHeightWithSpacing() + style.FramePadding.y * 2 + 4.0f),
				true);
			ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.4f, 1.0f),
				"Reference (%s) HMD not detected - profile disabled",
				CalCtx.referenceTrackingSystem.c_str());
			ImGui::EndChild();
			ImGui::PopStyleColor();
			ImGui::Spacing();
		}

		// --- Primary Action Buttons ---
		float totalW = ImGui::GetContentRegionAvail().x;
		float btnH   = ImGui::GetTextLineHeight() * 2.2f;

		if (CalCtx.validProfile) {
			// 4 buttons: Start | Continuous | Edit | Clear
			float btnW = (totalW - style.ItemSpacing.x * 3.0f) / 4.0f;

			if (ImGui::Button("Start Calibration", ImVec2(btnW, btnH))) {
				ImGui::OpenPopup("Calibration Progress");
				StartCalibration();
			}
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("Run a one-shot calibration with the selected speed.\nCollects samples and computes alignment once.");

			ImGui::SameLine();
			if (ImGui::Button("Continuous Cal", ImVec2(btnW, btnH)))
				StartContinuousCalibration();
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("Continuously recalibrate in the background.\nKeeps alignment accurate over time. Uses more CPU.");

			ImGui::SameLine();
			PushMuted();
			if (ImGui::Button("Edit Cal", ImVec2(btnW, btnH)))
				CalCtx.state = CalibrationState::Editing;
			PopMuted();

			ImGui::SameLine();
			PushDanger();
			if (ImGui::Button("Clear Cal", ImVec2(-1, btnH))) {
				CalCtx.Clear();
				SaveProfile(CalCtx);
			}
			PopDanger();
		}
		else {
			// 2 buttons: Start | Continuous
			float halfW = (totalW - style.ItemSpacing.x) * 0.5f;
			if (ImGui::Button("Start Calibration", ImVec2(halfW, btnH))) {
				ImGui::OpenPopup("Calibration Progress");
				StartCalibration();
			}
			ImGui::SameLine();
			if (ImGui::Button("Continuous Calibration", ImVec2(-1, btnH)))
				StartContinuousCalibration();
		}

		// --- Chaperone Section ---
		SectionHeader("Chaperone Bounds");
		HelpTip("Copy/paste SteamVR chaperone boundaries between profiles.\n"
			"Useful when switching between play spaces.");

		float chapW  = ImGui::GetContentRegionAvail().x;
		float chapBH = ImGui::GetTextLineHeight() * 1.8f;

		if (CalCtx.chaperone.valid) {
			float bw = (chapW - style.ItemSpacing.x) * 0.5f;
			if (ImGui::Button("Copy Bounds to Profile", ImVec2(bw, chapBH))) {
				LoadChaperoneBounds();
				SaveProfile(CalCtx);
			}
			ImGui::SameLine();
			if (ImGui::Button("Paste Chaperone Bounds", ImVec2(-1, chapBH)))
				ApplyChaperoneBounds();
			if (ImGui::Checkbox(
				"Auto-paste Chaperone Bounds when geometry resets",
				&CalCtx.chaperone.autoApply))
				SaveProfile(CalCtx);
		}
		else {
			if (ImGui::Button("Copy Chaperone Bounds to Profile",
				ImVec2(chapW, chapBH))) {
				LoadChaperoneBounds();
				SaveProfile(CalCtx);
			}
		}

		// --- Calibration Speed ---
		SectionHeader("Calibration Speed");

		auto speed = CalCtx.calibrationSpeed;
		float speedBtnH = ImGui::GetTextLineHeight() * 1.7f;
		float speedW    = (ImGui::GetContentRegionAvail().x - style.ItemSpacing.x * 2.0f) / 3.0f;

		auto SpeedButton = [&](const char* label, CalibrationContext::Speed s, float w) {
			bool sel = (speed == s);
			if (sel) PushSelected(); else PushMuted();
			if (ImGui::Button(label, ImVec2(w, speedBtnH)))
				CalCtx.calibrationSpeed = s;
			if (sel) PopSelected(); else PopMuted();
		};

		SpeedButton("Fast",      CalibrationContext::FAST,      speedW);
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("~100 samples. Fast convergence, lower accuracy.\nBest for quick alignment checks.");
		ImGui::SameLine();
		SpeedButton("Slow",      CalibrationContext::SLOW,      speedW);
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("~250 samples. Good balance of speed and accuracy.\nRecommended for most setups.");
		ImGui::SameLine();
		SpeedButton("Very Slow", CalibrationContext::VERY_SLOW, -1);
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("~500 samples. Highest accuracy, takes longer.\nBest for precise multi-tracker rigs.");
	}
	else if (CalCtx.state == CalibrationState::Editing)
	{
		SectionHeader("Edit Calibration Profile");
		BuildProfileEditor();

		ImGui::Spacing();
		if (ImGui::Button("Save Profile",
			ImVec2(-1, ImGui::GetTextLineHeight() * 2.0f)))
		{
			SaveProfile(CalCtx);
			CalCtx.state = CalibrationState::None;
		}
	}
	else
	{
		// Calibration running placeholder
		PushMuted();
		ImGui::Button("Calibration in progress...",
			ImVec2(-1, ImGui::GetTextLineHeight() * 2.0f));
		PopMuted();
	}

	// --- Calibration Progress Modal ---
	ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f), ImGuiCond_Always);
	ImGui::SetNextWindowSize(
		ImVec2(io.DisplaySize.x - 40.0f, io.DisplaySize.y - 40.0f),
		ImGuiCond_Always);
	if (ImGui::BeginPopupModal("Calibration Progress", nullptr, bareWindowFlags))
	{
		ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.10f, 0.11f, 0.13f, 1.00f));

		for (auto &message : CalCtx.messages)
		{
			switch (message.type)
			{
			case CalibrationContext::Message::String:
				ImGui::TextWrapped(message.str.c_str());
				break;
			case CalibrationContext::Message::Progress: {
				float fraction = (float)message.progress / (float)message.target;
				ImGui::Spacing();

				// Progress bar with teal accent
				ImGui::PushStyleColor(ImGuiCol_PlotHistogram, kAccent);
				float barH = ImGui::GetTextLineHeight() + style.FramePadding.y * 2;
				ImGui::ProgressBar(fraction, ImVec2(-1.0f, barH), "");
				ImGui::PopStyleColor();

				// Overlaid percentage text
				ImGui::SetCursorPosY(ImGui::GetCursorPosY() - barH - style.FramePadding.y);
				ImGui::Text("  %d%%", (int)(fraction * 100));
				break;
			}
			}
		}
		ImGui::PopStyleColor();

		if (CalCtx.state == CalibrationState::None)
		{
			ImGui::Spacing();
			if (ImGui::Button("Close",
				ImVec2(ImGui::GetWindowContentRegionWidth(),
					ImGui::GetTextLineHeight() * 2.0f)))
				ImGui::CloseCurrentPopup();
		}

		ImGui::EndPopup();
	}
}

// ============================================================
// System Selection (Reference / Target space dropdowns)
// ============================================================

void BuildSystemSelection(const VRState &state)
{
	if (state.trackingSystems.empty())
	{
		ImGui::TextColored(kStatusWarn, "No tracked devices are present");
		return;
	}

	ImGuiStyle &style = ImGui::GetStyle();
	float paneWidth = ImGui::GetWindowContentRegionWidth() / 2 - style.FramePadding.x;

	// Column headers
	ImGui::TextColored(kAccent, "Reference Space");
	ImGui::SameLine(paneWidth + style.FramePadding.x * 2);
	ImGui::TextColored(kAccent, "Target Space");

	int currentReferenceSystem = -1;
	int currentTargetSystem    = -1;
	int firstReferenceSystemNotTargetSystem = -1;

	std::vector<const char *> referenceSystems;
	for (auto &str : state.trackingSystems)
	{
		if (str == CalCtx.referenceTrackingSystem)
			currentReferenceSystem = (int)referenceSystems.size();
		else if (firstReferenceSystemNotTargetSystem == -1
			&& str != CalCtx.targetTrackingSystem)
			firstReferenceSystemNotTargetSystem = (int)referenceSystems.size();
		referenceSystems.push_back(str.c_str());
	}

	if (currentReferenceSystem == -1 && CalCtx.referenceTrackingSystem == "")
	{
		if (CalCtx.state == CalibrationState::ContinuousStandby) {
			auto iter = std::find(state.trackingSystems.begin(),
				state.trackingSystems.end(),
				CalCtx.referenceStandby.trackingSystem);
			if (iter != state.trackingSystems.end())
				currentReferenceSystem = (int)(iter - state.trackingSystems.begin());
		}
		else {
			currentReferenceSystem = firstReferenceSystemNotTargetSystem;
		}
	}

	ImGui::PushItemWidth(paneWidth);
	ImGui::Combo("##ReferenceTrackingSystem", &currentReferenceSystem,
		&referenceSystems[0], (int)referenceSystems.size());

	if (currentReferenceSystem != -1
		&& currentReferenceSystem < (int)referenceSystems.size())
	{
		CalCtx.referenceTrackingSystem =
			std::string(referenceSystems[currentReferenceSystem]);
		if (CalCtx.referenceTrackingSystem == CalCtx.targetTrackingSystem)
			CalCtx.targetTrackingSystem = "";
	}

	if (CalCtx.targetTrackingSystem == "") {
		if (CalCtx.state == CalibrationState::ContinuousStandby) {
			auto iter = std::find(state.trackingSystems.begin(),
				state.trackingSystems.end(),
				CalCtx.targetStandby.trackingSystem);
			if (iter != state.trackingSystems.end())
				currentTargetSystem = (int)(iter - state.trackingSystems.begin());
		}
		else {
			currentTargetSystem = 0;
		}
	}

	std::vector<const char *> targetSystems;
	for (auto &str : state.trackingSystems)
	{
		if (str != CalCtx.referenceTrackingSystem)
		{
			if (str != "" && str == CalCtx.targetTrackingSystem)
				currentTargetSystem = (int)targetSystems.size();
			targetSystems.push_back(str.c_str());
		}
	}

	ImGui::SameLine();
	ImGui::Combo("##TargetTrackingSystem", &currentTargetSystem,
		&targetSystems[0], (int)targetSystems.size());

	if (currentTargetSystem != -1
		&& currentTargetSystem < (int)targetSystems.size())
	{
		CalCtx.targetTrackingSystem =
			std::string(targetSystems[currentTargetSystem]);
	}

	ImGui::PopItemWidth();
}

// ============================================================
// Device Selection helpers
// ============================================================

void AppendSeparated(std::string &buffer, const std::string &suffix)
{
	if (!buffer.empty()) buffer += " | ";
	buffer += suffix;
}

std::string LabelString(const VRDevice &device)
{
	std::string label;
	AppendSeparated(label, device.model);
	AppendSeparated(label, device.serial);
	return label;
}

std::string LabelString(const StandbyDevice& device) {
	std::string label("< ");
	label += device.model;
	AppendSeparated(label, device.serial);
	label += " >";
	return label;
}

void BuildDeviceSelection(const VRState &state, int &initialSelected,
	const std::string &system, StandbyDevice &standbyDevice)
{
	int selected = initialSelected;
	ImGui::TextColored(kTextDim, "System: %s", system.c_str());

	if (selected != -1)
	{
		bool matched = false;
		for (auto &device : state.devices)
		{
			if (device.trackingSystem != system) continue;
			if (selected == device.id) { matched = true; break; }
		}
		if (!matched) selected = -1;
	}

	bool standby = CalCtx.state == CalibrationState::ContinuousStandby;

	if (selected == -1 && !standby)
	{
		for (auto &device : state.devices)
		{
			if (device.trackingSystem != system) continue;
			if (device.controllerRole == vr::TrackedControllerRole_LeftHand) {
				selected = device.id;
				break;
			}
		}
		if (selected == -1) {
			for (auto& device : state.devices) {
				if (device.trackingSystem != system) continue;
				selected = device.id;
				break;
			}
		}
	}

	if (selected == -1 && standby) {
		bool present = false;
		for (auto& device : state.devices) {
			if (device.trackingSystem != system) continue;
			if (standbyDevice.model != device.model) continue;
			if (standbyDevice.serial != device.serial) continue;
			present = true;
			break;
		}
		if (!present) {
			auto label = LabelString(standbyDevice);
			ImGui::Selectable(label.c_str(), true);
		}
	}

	for (auto &device : state.devices)
	{
		if (device.trackingSystem != system) continue;
		auto label = LabelString(device);
		if (ImGui::Selectable(label.c_str(), selected == device.id))
			selected = device.id;
	}

	if (selected != initialSelected) {
		const auto& device = std::find_if(state.devices.begin(), state.devices.end(),
			[&](const auto& d) { return d.id == selected; });
		if (device == state.devices.end()) return;

		initialSelected              = selected;
		standbyDevice.trackingSystem = system;
		standbyDevice.model          = device->model;
		standbyDevice.serial         = device->serial;
	}
}

void BuildDeviceSelections(const VRState &state)
{
	ImGuiStyle &style = ImGui::GetStyle();
	ImVec2 paneSize(
		ImGui::GetWindowContentRegionWidth() / 2 - style.FramePadding.x,
		ImGui::GetTextLineHeightWithSpacing() * 5 + style.ItemSpacing.y * 4);

	ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
	ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.14f, 0.15f, 0.18f, 1.00f));

	ImGui::BeginChild("left device pane", paneSize, true);
	BuildDeviceSelection(state, CalCtx.referenceID,
		CalCtx.referenceTrackingSystem, CalCtx.referenceStandby);
	ImGui::EndChild();

	ImGui::SameLine();

	ImGui::BeginChild("right device pane", paneSize, true);
	BuildDeviceSelection(state, CalCtx.targetID,
		CalCtx.targetTrackingSystem, CalCtx.targetStandby);
	ImGui::EndChild();

	ImGui::PopStyleColor();
	ImGui::PopStyleVar();

	ImGui::Spacing();
	PushMuted();
	if (ImGui::Button(
		"Identify selected devices (blinks LED / vibrates)",
		ImVec2(-1, ImGui::GetTextLineHeightWithSpacing() + 6.0f)))
	{
		for (unsigned i = 0; i < 100; ++i)
		{
			vr::VRSystem()->TriggerHapticPulse(CalCtx.targetID,    0, 2000);
			vr::VRSystem()->TriggerHapticPulse(CalCtx.referenceID, 0, 2000);
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}
	}
	PopMuted();
}

// ============================================================
// VR State loader (injects standby devices for continuous cal)
// ============================================================

VRState LoadVRState() {
	VRState state = VRState::Load();
	auto& trackingSystems = state.trackingSystems;

	if (CalCtx.state == CalibrationState::ContinuousStandby) {
		auto existing = std::find(trackingSystems.begin(),
			trackingSystems.end(), CalCtx.referenceTrackingSystem);
		if (existing == trackingSystems.end())
			trackingSystems.push_back(CalCtx.referenceTrackingSystem);

		existing = std::find(trackingSystems.begin(),
			trackingSystems.end(), CalCtx.targetTrackingSystem);
		if (existing == trackingSystems.end())
			trackingSystems.push_back(CalCtx.targetTrackingSystem);
	}

	return state;
}

// ============================================================
// Profile Editor
// ============================================================

void BuildProfileEditor()
{
	ImGuiStyle& style = ImGui::GetStyle();
	float colW  = ImGui::GetWindowContentRegionWidth() / 3.0f - style.FramePadding.x;
	float inputW = colW - style.FramePadding.x;

	// Rotation card
	float cardH = ImGui::GetTextLineHeightWithSpacing() * 3.2f
		+ style.WindowPadding.y * 2 + style.FramePadding.y * 2;

	ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.13f, 0.14f, 0.17f, 1.0f));

	if (ImGui::BeginChild("##RotCard", ImVec2(-1, cardH), true)) {
		ImGui::TextColored(kAccent, "Rotation");
		HelpTip("Euler angles (Yaw/Pitch/Roll) in degrees.\nManual fine-tuning of the calibrated rotation.");
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::PushItemWidth(inputW);
		TextWithWidth("YawLabel",   "Yaw",   colW); ImGui::SameLine();
		TextWithWidth("PitchLabel", "Pitch", colW); ImGui::SameLine();
		TextWithWidth("RollLabel",  "Roll",  colW);

		ImGui::InputDouble("##Yaw",   &CalCtx.calibratedRotation(1), 0.1, 1.0, "%.8f");
		ImGui::SameLine();
		ImGui::InputDouble("##Pitch", &CalCtx.calibratedRotation(2), 0.1, 1.0, "%.8f");
		ImGui::SameLine();
		ImGui::InputDouble("##Roll",  &CalCtx.calibratedRotation(0), 0.1, 1.0, "%.8f");
		ImGui::PopItemWidth();
	}
	ImGui::EndChild();

	ImGui::Spacing();

	// Translation card
	if (ImGui::BeginChild("##TransCard", ImVec2(-1, cardH), true)) {
		ImGui::TextColored(kAccent, "Translation");
		HelpTip("Position offset (X/Y/Z) in centimeters.\nManual fine-tuning of the calibrated position.");
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::PushItemWidth(inputW);
		TextWithWidth("XLabel", "X", colW); ImGui::SameLine();
		TextWithWidth("YLabel", "Y", colW); ImGui::SameLine();
		TextWithWidth("ZLabel", "Z", colW);

		ImGui::InputDouble("##X", &CalCtx.calibratedTranslation(0), 1.0, 10.0, "%.8f");
		ImGui::SameLine();
		ImGui::InputDouble("##Y", &CalCtx.calibratedTranslation(1), 1.0, 10.0, "%.8f");
		ImGui::SameLine();
		ImGui::InputDouble("##Z", &CalCtx.calibratedTranslation(2), 1.0, 10.0, "%.8f");
		ImGui::PopItemWidth();
	}
	ImGui::EndChild();

	ImGui::PopStyleColor(); // ChildBg

	// Scale (inline, no card)
	ImGui::Spacing();
	ImGui::TextColored(kAccent, "Scale");
	HelpTip("Uniform scale factor between tracking spaces.\n1.0 = no scaling. Rarely needs adjustment.");
	ImGui::Spacing();
	ImGui::PushItemWidth(inputW);
	ImGui::InputDouble("##Scale", &CalCtx.calibratedScale, 0.0001, 0.01, "%.8f");
	ImGui::PopItemWidth();
}

// ============================================================
// TextWithWidth helper
// ============================================================

void TextWithWidth(const char *label, const char *text, float width)
{
	ImGui::BeginChild(label, ImVec2(width, ImGui::GetTextLineHeightWithSpacing()));
	ImGui::Text(text);
	ImGui::EndChild();
}
