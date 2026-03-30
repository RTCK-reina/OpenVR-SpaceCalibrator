#include "stdafx.h"
#include "Calibration.h"
#include "Configuration.h"
#include "EmbeddedFiles.h"
#include "UserInterface.h"

#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>
#include <implot/implot.h>
#include <GL/gl3w.h>
#include <GLFW/glfw3.h>
#include <openvr.h>
#include <direct.h>

#pragma comment(linker,"\"/manifestdependency:type='win32' \
name='Microsoft.Windows.Common-Controls' version='6.0.0.0' \
processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")

#define OPENVR_APPLICATION_KEY "pushrax.SpaceCalibrator"

extern "C" __declspec(dllexport) DWORD NvOptimusEnablement = 0x00000001;
extern "C" __declspec(dllexport) DWORD AmdPowerXpressRequestHighPerformance = 0x00000001;

void CreateConsole()
{
	static bool created = false;
	if (!created)
	{
		AllocConsole();
		FILE *file = nullptr;
		freopen_s(&file, "CONIN$", "r", stdin);
		freopen_s(&file, "CONOUT$", "w", stdout);
		freopen_s(&file, "CONOUT$", "w", stderr);
		created = true;
	}
}

//#define DEBUG_LOGS

void GLFWErrorCallback(int error, const char* description)
{
	fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

void openGLDebugCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar *message, const void *userParam)
{
	fprintf(stderr, "OpenGL Debug %u: %.*s\n", id, length, message);
}

static void HandleCommandLine(LPWSTR lpCmdLine);

static GLFWwindow *glfwWindow = nullptr;
static vr::VROverlayHandle_t overlayMainHandle = 0, overlayThumbnailHandle = 0;
static GLuint fboHandle = 0, fboTextureHandle = 0;
static int fboTextureWidth = 0, fboTextureHeight = 0;

static char cwd[MAX_PATH];

void CreateGLFWWindow()
{
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, false);

#ifdef DEBUG_LOGS
	glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);
#endif

	fboTextureWidth = 1200;
	fboTextureHeight = 800;

	glfwWindow = glfwCreateWindow(fboTextureWidth, fboTextureHeight, "OpenVR-SpaceCalibrator", NULL, NULL);
	if (!glfwWindow)
		throw std::runtime_error("Failed to create window");

	glfwMakeContextCurrent(glfwWindow);
	glfwSwapInterval(1);
	gl3wInit();

	glfwIconifyWindow(glfwWindow);

#ifdef DEBUG_LOGS
	glDebugMessageCallback(openGLDebugCallback, nullptr);
	glEnable(GL_DEBUG_OUTPUT);
#endif

	ImGui::CreateContext();
	ImPlot::CreateContext();
	ImGuiIO &io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
	io.IniFilename = nullptr;
	io.Fonts->AddFontFromMemoryCompressedTTF(DroidSans_compressed_data, DroidSans_compressed_size, 24.0f);

	ImGui_ImplGlfw_InitForOpenGL(glfwWindow, true);
	ImGui_ImplOpenGL3_Init("#version 330");

	// Modern dark theme (refined)
	{
		ImGuiStyle &style = ImGui::GetStyle();

		// Rounding - slightly more rounded for modern look
		style.WindowRounding   = 6.0f;
		style.ChildRounding    = 6.0f;
		style.FrameRounding    = 5.0f;
		style.PopupRounding    = 5.0f;
		style.ScrollbarRounding= 4.0f;
		style.GrabRounding     = 4.0f;
		style.TabRounding      = 5.0f;

		// Spacing & padding - slightly more breathing room
		style.WindowPadding     = ImVec2(12.0f, 12.0f);
		style.FramePadding      = ImVec2(9.0f,  5.0f);
		style.ItemSpacing       = ImVec2(8.0f,  7.0f);
		style.ItemInnerSpacing  = ImVec2(6.0f,  4.0f);
		style.ScrollbarSize     = 13.0f;
		style.GrabMinSize       = 13.0f;
		style.IndentSpacing     = 20.0f;

		// Borders
		style.WindowBorderSize  = 1.0f;
		style.ChildBorderSize   = 1.0f;
		style.FrameBorderSize   = 0.0f;
		style.PopupBorderSize   = 1.0f;
		style.TabBorderSize     = 0.0f;
		// Alignment
		style.WindowTitleAlign  = ImVec2(0.5f, 0.5f);

		// Colors - modern muted dark palette with teal accent
		ImVec4 *colors = style.Colors;

		// Backgrounds - deep muted dark
		colors[ImGuiCol_WindowBg]             = ImVec4(0.10f, 0.11f, 0.13f, 1.00f);
		colors[ImGuiCol_ChildBg]              = ImVec4(0.12f, 0.13f, 0.16f, 1.00f);
		colors[ImGuiCol_PopupBg]              = ImVec4(0.12f, 0.13f, 0.17f, 0.97f);

		// Borders
		colors[ImGuiCol_Border]               = ImVec4(0.22f, 0.24f, 0.28f, 0.60f);
		colors[ImGuiCol_BorderShadow]         = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);

		// Frame (input fields, checkboxes)
		colors[ImGuiCol_FrameBg]              = ImVec4(0.16f, 0.17f, 0.20f, 1.00f);
		colors[ImGuiCol_FrameBgHovered]       = ImVec4(0.20f, 0.22f, 0.26f, 1.00f);
		colors[ImGuiCol_FrameBgActive]        = ImVec4(0.24f, 0.26f, 0.30f, 1.00f);

		// Title bar
		colors[ImGuiCol_TitleBg]              = ImVec4(0.09f, 0.10f, 0.12f, 1.00f);
		colors[ImGuiCol_TitleBgActive]        = ImVec4(0.11f, 0.12f, 0.14f, 1.00f);
		colors[ImGuiCol_TitleBgCollapsed]     = ImVec4(0.09f, 0.10f, 0.12f, 0.60f);

		// Menu bar
		colors[ImGuiCol_MenuBarBg]            = ImVec4(0.14f, 0.15f, 0.18f, 1.00f);

		// Scrollbar
		colors[ImGuiCol_ScrollbarBg]          = ImVec4(0.12f, 0.13f, 0.15f, 0.60f);
		colors[ImGuiCol_ScrollbarGrab]        = ImVec4(0.26f, 0.28f, 0.32f, 1.00f);
		colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.32f, 0.34f, 0.38f, 1.00f);
		colors[ImGuiCol_ScrollbarGrabActive]  = ImVec4(0.38f, 0.40f, 0.44f, 1.00f);

		// Checkmark
		colors[ImGuiCol_CheckMark]            = ImVec4(0.32f, 0.78f, 0.70f, 1.00f);

		// Slider
		colors[ImGuiCol_SliderGrab]           = ImVec4(0.32f, 0.78f, 0.70f, 0.80f);
		colors[ImGuiCol_SliderGrabActive]     = ImVec4(0.40f, 0.88f, 0.80f, 1.00f);

		// Buttons - teal accent
		colors[ImGuiCol_Button]               = ImVec4(0.18f, 0.42f, 0.40f, 1.00f);
		colors[ImGuiCol_ButtonHovered]        = ImVec4(0.24f, 0.52f, 0.48f, 1.00f);
		colors[ImGuiCol_ButtonActive]         = ImVec4(0.30f, 0.60f, 0.56f, 1.00f);

		// Header (collapsing headers, selectable items)
		colors[ImGuiCol_Header]               = ImVec4(0.18f, 0.42f, 0.40f, 0.50f);
		colors[ImGuiCol_HeaderHovered]        = ImVec4(0.24f, 0.52f, 0.48f, 0.60f);
		colors[ImGuiCol_HeaderActive]         = ImVec4(0.30f, 0.60f, 0.56f, 0.70f);

		// Separator
		colors[ImGuiCol_Separator]            = ImVec4(0.22f, 0.24f, 0.28f, 0.60f);
		colors[ImGuiCol_SeparatorHovered]     = ImVec4(0.32f, 0.78f, 0.70f, 0.60f);
		colors[ImGuiCol_SeparatorActive]      = ImVec4(0.32f, 0.78f, 0.70f, 1.00f);

		// Resize grip
		colors[ImGuiCol_ResizeGrip]           = ImVec4(0.32f, 0.78f, 0.70f, 0.20f);
		colors[ImGuiCol_ResizeGripHovered]    = ImVec4(0.32f, 0.78f, 0.70f, 0.50f);
		colors[ImGuiCol_ResizeGripActive]     = ImVec4(0.32f, 0.78f, 0.70f, 0.80f);

		// Tabs
		colors[ImGuiCol_Tab]                  = ImVec4(0.15f, 0.16f, 0.19f, 1.00f);
		colors[ImGuiCol_TabHovered]           = ImVec4(0.24f, 0.52f, 0.48f, 0.80f);
		colors[ImGuiCol_TabActive]            = ImVec4(0.18f, 0.42f, 0.40f, 1.00f);
		colors[ImGuiCol_TabUnfocused]         = ImVec4(0.13f, 0.14f, 0.16f, 1.00f);
		colors[ImGuiCol_TabUnfocusedActive]   = ImVec4(0.16f, 0.34f, 0.32f, 1.00f);

		// Plot
		colors[ImGuiCol_PlotLines]            = ImVec4(0.32f, 0.78f, 0.70f, 1.00f);
		colors[ImGuiCol_PlotLinesHovered]     = ImVec4(0.40f, 0.88f, 0.80f, 1.00f);
		colors[ImGuiCol_PlotHistogram]        = ImVec4(0.32f, 0.78f, 0.70f, 1.00f);
		colors[ImGuiCol_PlotHistogramHovered] = ImVec4(0.40f, 0.88f, 0.80f, 1.00f);

		// Table
		colors[ImGuiCol_TableHeaderBg]        = ImVec4(0.15f, 0.16f, 0.19f, 1.00f);
		colors[ImGuiCol_TableBorderStrong]    = ImVec4(0.22f, 0.24f, 0.28f, 0.80f);
		colors[ImGuiCol_TableBorderLight]     = ImVec4(0.20f, 0.22f, 0.25f, 0.50f);
		colors[ImGuiCol_TableRowBg]           = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
		colors[ImGuiCol_TableRowBgAlt]        = ImVec4(1.00f, 1.00f, 1.00f, 0.02f);

		// Text
		colors[ImGuiCol_Text]                 = ImVec4(0.90f, 0.92f, 0.94f, 1.00f);
		colors[ImGuiCol_TextDisabled]         = ImVec4(0.45f, 0.48f, 0.52f, 1.00f);
		colors[ImGuiCol_TextSelectedBg]       = ImVec4(0.32f, 0.78f, 0.70f, 0.30f);

		// Nav
		colors[ImGuiCol_NavHighlight]         = ImVec4(0.32f, 0.78f, 0.70f, 1.00f);
		colors[ImGuiCol_NavWindowingHighlight]= ImVec4(1.00f, 1.00f, 1.00f, 0.70f);
		colors[ImGuiCol_NavWindowingDimBg]    = ImVec4(0.80f, 0.80f, 0.80f, 0.20f);

		// Drag/drop & modal
		colors[ImGuiCol_DragDropTarget]       = ImVec4(0.32f, 0.78f, 0.70f, 0.90f);
		colors[ImGuiCol_ModalWindowDimBg]     = ImVec4(0.05f, 0.06f, 0.07f, 0.60f);
	}

	glGenTextures(1, &fboTextureHandle);
	glBindTexture(GL_TEXTURE_2D, fboTextureHandle);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, fboTextureWidth, fboTextureHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

	glGenFramebuffers(1, &fboHandle);
	glBindFramebuffer(GL_FRAMEBUFFER, fboHandle);
	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, fboTextureHandle, 0);

	GLenum drawBuffers[1] = { GL_COLOR_ATTACHMENT0 };
	glDrawBuffers(1, drawBuffers);

	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	{
		throw std::runtime_error("OpenGL framebuffer incomplete");
	}
}

void TryCreateVROverlay()
{
	if (overlayMainHandle || !vr::VROverlay())
		return;

	vr::VROverlayError error = vr::VROverlay()->CreateDashboardOverlay(
		"pushrax.SpaceCalibrator", "Space Cal",
		&overlayMainHandle, &overlayThumbnailHandle
	);

	if (error == vr::VROverlayError_KeyInUse)
	{
		throw std::runtime_error("Another instance of OpenVR Space Calibrator is already running");
	}
	else if (error != vr::VROverlayError_None)
	{
		throw std::runtime_error("Error creating VR overlay: " + std::string(vr::VROverlay()->GetOverlayErrorNameFromEnum(error)));
	}

	vr::VROverlay()->SetOverlayWidthInMeters(overlayMainHandle, 3.0f);
	vr::VROverlay()->SetOverlayInputMethod(overlayMainHandle, vr::VROverlayInputMethod_Mouse);
	vr::VROverlay()->SetOverlayFlag(overlayMainHandle, vr::VROverlayFlags_SendVRDiscreteScrollEvents, true);

	std::string iconPath = cwd;
	iconPath += "\\icon.png";
	vr::VROverlay()->SetOverlayFromFile(overlayThumbnailHandle, iconPath.c_str());
}

void ActivateMultipleDrivers()
{
	vr::EVRSettingsError vrSettingsError;
	bool enabled = vr::VRSettings()->GetBool(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_ActivateMultipleDrivers_Bool, &vrSettingsError);

	if (vrSettingsError != vr::VRSettingsError_None)
	{
		std::string err = "Could not read \"" + std::string(vr::k_pch_SteamVR_ActivateMultipleDrivers_Bool) + "\" setting: "
			+ vr::VRSettings()->GetSettingsErrorNameFromEnum(vrSettingsError);

		throw std::runtime_error(err);
	}

	if (!enabled)
	{
		vr::VRSettings()->SetBool(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_ActivateMultipleDrivers_Bool, true, &vrSettingsError);
		if (vrSettingsError != vr::VRSettingsError_None)
		{
			std::string err = "Could not set \"" + std::string(vr::k_pch_SteamVR_ActivateMultipleDrivers_Bool) + "\" setting: "
				+ vr::VRSettings()->GetSettingsErrorNameFromEnum(vrSettingsError);

			throw std::runtime_error(err);
		}

		std::cerr << "Enabled \"" << vr::k_pch_SteamVR_ActivateMultipleDrivers_Bool << "\" setting" << std::endl;
	}
	else
	{
		std::cerr << "\"" << vr::k_pch_SteamVR_ActivateMultipleDrivers_Bool << "\" setting previously enabled" << std::endl;
	}
}

void InitVR()
{
	auto initError = vr::VRInitError_None;
	vr::VR_Init(&initError, vr::VRApplication_Other);
	if (initError != vr::VRInitError_None)
	{
		auto error = vr::VR_GetVRInitErrorAsEnglishDescription(initError);
		throw std::runtime_error("OpenVR error:" + std::string(error));
	}

	if (!vr::VR_IsInterfaceVersionValid(vr::IVRSystem_Version))
	{
		throw std::runtime_error("OpenVR error: Outdated IVRSystem_Version");
	}
	else if (!vr::VR_IsInterfaceVersionValid(vr::IVRSettings_Version))
	{
		throw std::runtime_error("OpenVR error: Outdated IVRSettings_Version");
	}
	else if (!vr::VR_IsInterfaceVersionValid(vr::IVROverlay_Version))
	{
		throw std::runtime_error("OpenVR error: Outdated IVROverlay_Version");
	}

	ActivateMultipleDrivers();
}

static char textBuf[0x400];

static bool immediateRedraw;
void RequestImmediateRedraw() {
	immediateRedraw = true;
}

void RunLoop()
{
	while (!glfwWindowShouldClose(glfwWindow))
	{
		TryCreateVROverlay();

		double time = glfwGetTime();
		CalibrationTick(time);

		bool dashboardVisible = false;
		int width, height;
		glfwGetFramebufferSize(glfwWindow, &width, &height);

		if (overlayMainHandle && vr::VROverlay())
		{
			auto &io = ImGui::GetIO();
			dashboardVisible = vr::VROverlay()->IsActiveDashboardOverlay(overlayMainHandle);

			static bool keyboardOpen = false, keyboardJustClosed = false;

			// After closing the keyboard, this code waits one frame for ImGui to pick up the new text from SetActiveText
			// before clearing the active widget. Then it waits another frame before allowing the keyboard to open again,
			// otherwise it will do so instantly since WantTextInput is still true on the second frame.
			if (keyboardJustClosed && keyboardOpen)
			{
				ImGui::ClearActiveID();
				keyboardOpen = false;
			}
			else if (keyboardJustClosed)
			{
				keyboardJustClosed = false;
			}
			else if (!io.WantTextInput)
			{
				// User might close the keyboard without hitting Done, so we unset the flag to allow it to open again.
				keyboardOpen = false;
			}
			else if (io.WantTextInput && !keyboardOpen && !keyboardJustClosed)
			{
				int id = ImGui::GetActiveID();
				auto textInfo = ImGui::GetInputTextState(id);

				textBuf[0] = 0;
				int len = WideCharToMultiByte(CP_ACP, 0, (LPCWCH)textInfo->TextW.Data, textInfo->TextW.Size, textBuf, sizeof(textBuf), NULL, NULL);
				textBuf[min(len, sizeof(textBuf) - 1)] = 0;
				
				uint32_t unFlags = 0; // EKeyboardFlags 

				vr::VROverlay()->ShowKeyboardForOverlay(
					overlayMainHandle, vr::k_EGamepadTextInputModeNormal, vr::k_EGamepadTextInputLineModeSingleLine,
					unFlags, "Space Calibrator Overlay", sizeof textBuf, textBuf, 0
				);
				keyboardOpen = true;
			}

			vr::VREvent_t vrEvent;
			while (vr::VROverlay()->PollNextOverlayEvent(overlayMainHandle, &vrEvent, sizeof(vrEvent)))
			{
				switch (vrEvent.eventType) {
				case vr::VREvent_MouseMove:
					io.AddMousePosEvent(vrEvent.data.mouse.x, vrEvent.data.mouse.y);
					break;
				case vr::VREvent_MouseButtonDown:
					io.AddMouseButtonEvent(vrEvent.data.mouse.button == vr::VRMouseButton_Left ? 0 : 1, true);
					break;
				case vr::VREvent_MouseButtonUp:
					io.AddMouseButtonEvent(vrEvent.data.mouse.button == vr::VRMouseButton_Left ? 0 : 1, false);
					break;
				case vr::VREvent_ScrollDiscrete:
				{
					float x = vrEvent.data.scroll.xdelta * 360.0f * 8.0f;
					float y = vrEvent.data.scroll.ydelta * 360.0f * 8.0f;
					io.AddMouseWheelEvent(x, y);
					break;
				}
				case vr::VREvent_KeyboardDone: {
					vr::VROverlay()->GetKeyboardText(textBuf, sizeof textBuf);

					int id = ImGui::GetActiveID();
					auto textInfo = ImGui::GetInputTextState(id);
					int bufSize = MultiByteToWideChar(CP_ACP, 0, textBuf, -1, NULL, 0);
					textInfo->TextW.resize(bufSize);
					MultiByteToWideChar(CP_ACP, 0, textBuf, -1, (LPWSTR)textInfo->TextW.Data, bufSize);
					textInfo->CurLenW = bufSize;
					textInfo->CurLenA = WideCharToMultiByte(CP_UTF8, 0, (LPCWCH)textInfo->TextW.Data, textInfo->TextW.Size, NULL, 0, NULL, NULL);
					
					keyboardJustClosed = true;
					break;
				}
				case vr::VREvent_Quit:
					return;
				}
			}
		}

		auto &io = ImGui::GetIO();
		io.DisplaySize = ImVec2((float) fboTextureWidth, (float) fboTextureHeight);
		io.DisplayFramebufferScale = ImVec2(1.0f, 1.0f);

		io.ConfigFlags = io.ConfigFlags & ~ImGuiConfigFlags_NoMouseCursorChange;
		if (dashboardVisible) {
			io.ConfigFlags = io.ConfigFlags | ImGuiConfigFlags_NoMouseCursorChange;
		}
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		BuildMainWindow(dashboardVisible);

		ImGui::Render();

		glBindFramebuffer(GL_FRAMEBUFFER, fboHandle);
		glViewport(0, 0, fboTextureWidth, fboTextureHeight);
		glClearColor(0, 0, 0, 1);
		glClear(GL_COLOR_BUFFER_BIT);

		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glBindFramebuffer(GL_FRAMEBUFFER, 0);

		if (width && height)
		{
			glBindFramebuffer(GL_READ_FRAMEBUFFER, fboHandle);
			glBlitFramebuffer(0, 0, width, height, 0, 0, width, height, GL_COLOR_BUFFER_BIT, GL_NEAREST);
			glfwSwapBuffers(glfwWindow);
		}

		if (dashboardVisible)
		{
			vr::Texture_t vrTex;
			vrTex.eType = vr::TextureType_OpenGL;
			vrTex.eColorSpace = vr::ColorSpace_Auto;

			vrTex.handle = (void *)
#if defined _WIN64 || defined _LP64
			(uint64_t)
#endif
				fboTextureHandle;

			vr::HmdVector2_t mouseScale = { (float) fboTextureWidth, (float) fboTextureHeight };

			vr::VROverlay()->SetOverlayTexture(overlayMainHandle, &vrTex);
			vr::VROverlay()->SetOverlayMouseScale(overlayMainHandle, &mouseScale);
		}

		const double dashboardInterval = 1.0 / 90.0; // fps
		double waitEventsTimeout = CalCtx.wantedUpdateInterval;

		if (dashboardVisible && waitEventsTimeout > dashboardInterval)
			waitEventsTimeout = dashboardInterval;

		if (immediateRedraw) {
			waitEventsTimeout = 0;
			immediateRedraw = false;
		}

		glfwWaitEventsTimeout(waitEventsTimeout);
	}
}

int APIENTRY wWinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPWSTR lpCmdLine, _In_ int nCmdShow)
{
	_getcwd(cwd, MAX_PATH);
	HandleCommandLine(lpCmdLine);

#ifdef DEBUG_LOGS
	CreateConsole();
#endif

	if (!glfwInit())
	{
		MessageBox(nullptr, L"Failed to initialize GLFW", L"", 0);
		return 0;
	}

	glfwSetErrorCallback(GLFWErrorCallback);

	try {
		InitVR();
		CreateGLFWWindow();
		InitCalibrator();
		LoadProfile(CalCtx);
		RunLoop();

		vr::VR_Shutdown();

		if (fboHandle)
			glDeleteFramebuffers(1, &fboHandle);

		if (fboTextureHandle)
			glDeleteTextures(1, &fboTextureHandle);

		ImGui_ImplOpenGL3_Shutdown();
		ImGui_ImplGlfw_Shutdown();
		ImPlot::DestroyContext();
		ImGui::DestroyContext();
	}
	catch (std::runtime_error &e)
	{
		std::cerr << "Runtime error: " << e.what() << std::endl;
		wchar_t message[1024];
		swprintf(message, 1024, L"%hs", e.what());
		MessageBox(nullptr, message, L"Runtime Error", 0);
	}

	if (glfwWindow)
		glfwDestroyWindow(glfwWindow);

	glfwTerminate();
	return 0;
}

static void HandleCommandLine(LPWSTR lpCmdLine)
{
	if (lstrcmp(lpCmdLine, L"-openvrpath") == 0)
	{
		auto vrErr = vr::VRInitError_None;
		vr::VR_Init(&vrErr, vr::VRApplication_Utility);
		if (vrErr == vr::VRInitError_None)
		{
			char cruntimePath[MAX_PATH] = { 0 };
			unsigned int pathLen;
			vr::VR_GetRuntimePath(cruntimePath, MAX_PATH, &pathLen);

			printf("%s", cruntimePath);
			vr::VR_Shutdown();
			exit(0);
		}
		fprintf(stderr, "Failed to initialize OpenVR: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(vrErr));
		vr::VR_Shutdown();
		exit(-2);
	}
	else if (lstrcmp(lpCmdLine, L"-installmanifest") == 0)
	{
		auto vrErr = vr::VRInitError_None;
		vr::VR_Init(&vrErr, vr::VRApplication_Utility);
		if (vrErr == vr::VRInitError_None)
		{
			if (vr::VRApplications()->IsApplicationInstalled(OPENVR_APPLICATION_KEY))
			{
				char oldWd[MAX_PATH] = { 0 };
				auto vrAppErr = vr::VRApplicationError_None;
				vr::VRApplications()->GetApplicationPropertyString(OPENVR_APPLICATION_KEY, vr::VRApplicationProperty_WorkingDirectory_String, oldWd, MAX_PATH, &vrAppErr);
				if (vrAppErr != vr::VRApplicationError_None)
				{
					fprintf(stderr, "Failed to get old working dir, skipping removal: %s\n", vr::VRApplications()->GetApplicationsErrorNameFromEnum(vrAppErr));
				}
				else
				{
					std::string manifestPath = oldWd;
					manifestPath += "\\manifest.vrmanifest";
					std::cout << "Removing old manifest path: " << manifestPath << std::endl;
					vr::VRApplications()->RemoveApplicationManifest(manifestPath.c_str());
				}
			}
			std::string manifestPath = cwd;
			manifestPath += "\\manifest.vrmanifest";
			std::cout << "Adding manifest path: " << manifestPath << std::endl;
			auto vrAppErr = vr::VRApplications()->AddApplicationManifest(manifestPath.c_str());
			if (vrAppErr != vr::VRApplicationError_None)
			{
				fprintf(stderr, "Failed to add manifest: %s\n", vr::VRApplications()->GetApplicationsErrorNameFromEnum(vrAppErr));
			}
			else
			{
				vr::VRApplications()->SetApplicationAutoLaunch(OPENVR_APPLICATION_KEY, true);
			}
			vr::VR_Shutdown();
			exit(-2);
		}
		fprintf(stderr, "Failed to initialize OpenVR: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(vrErr));
		vr::VR_Shutdown();
		exit(-2);
	}
	else if (lstrcmp(lpCmdLine, L"-removemanifest") == 0)
	{
		auto vrErr = vr::VRInitError_None;
		vr::VR_Init(&vrErr, vr::VRApplication_Utility);
		if (vrErr == vr::VRInitError_None)
		{
			if (vr::VRApplications()->IsApplicationInstalled(OPENVR_APPLICATION_KEY))
			{
				std::string manifestPath = cwd;
				manifestPath += "\\manifest.vrmanifest";
				std::cout << "Removing manifest path: " << manifestPath << std::endl;
				vr::VRApplications()->RemoveApplicationManifest(manifestPath.c_str());
			}
			vr::VR_Shutdown();
			exit(0);
		}
		fprintf(stderr, "Failed to initialize OpenVR: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(vrErr));
		vr::VR_Shutdown();
		exit(-2);
	}
	else if (lstrcmp(lpCmdLine, L"-activatemultipledrivers") == 0)
	{
		int ret = -2;
		auto vrErr = vr::VRInitError_None;
		vr::VR_Init(&vrErr, vr::VRApplication_Utility);
		if (vrErr == vr::VRInitError_None)
		{
			try
			{
				ActivateMultipleDrivers();
				ret = 0;
			}
			catch (std::runtime_error &e)
			{
				std::cerr << e.what() << std::endl;
			}
		}
		else
		{
			fprintf(stderr, "Failed to initialize OpenVR: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(vrErr));
		}
		vr::VR_Shutdown();
		exit(ret);
	}
}
