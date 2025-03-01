#include "util/base.h";
#include <d3d11.h>
#include <ImGui/imgui.h>
#include <ImGui/imgui_impl_dx11.h>
#include <ImGui/imgui_impl_win32.h>
#include <dwmapi.h>
#include <set>
#include <fstream>
#include <DMALibrary/Memory/Memory.h>
#include "util/Offset.h"
#include "util/Data.h"
#include "util/VectorHelper.h"
#include "util/Utils.h"
#include "util/LineTrace.h"
#include <util/VisibleCheck.h>

FGameData GameData;

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
void Raycast(FRotator& playerRotation, FVector& playerLocation, float fov);
void RenderTriangleMesh(FRotator& playerRotation, FVector& playerLocation, float fov);
void ExportMeshAsObj(TriangleMeshData& mesh);
TriangleMeshData* HitMeshPtr = nullptr;

LRESULT CALLBACK window_procedure(HWND window, UINT message, WPARAM w_param, LPARAM l_param) {
    if (ImGui_ImplWin32_WndProcHandler(window, message, w_param, l_param)) {
        return 0L;
    }

    if (message == WM_DESTROY) {
        PostQuitMessage(0);
        return 0L;
    }
    return DefWindowProc(window, message, w_param, l_param);
}


void ReleaseResource(
    const HWND window,
    IDXGISwapChain* swap_chain,
    ID3D11DeviceContext* device_context,
    ID3D11Device* device,
    ID3D11RenderTargetView* render_target_view
) {
    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();

    if (swap_chain) {
        swap_chain->Release();
    }

    if (device_context) {
        device_context->Release();
    }

    if (device) {
        device->Release();
    }

    if (render_target_view) {
        render_target_view->Release();
    }

    DestroyWindow(window);
}

typedef struct {
    ImU32 R;
    ImU32 G;
    ImU32 B;
    ImU32 A;
} RGB;

ImU32 Color(RGB color) {
    return IM_COL32(color.R, color.G, color.B, color.A);
}


static auto DecFunction = reinterpret_cast<uint64_t(*)(uint64_t key, uint64_t base)>(0);

namespace Decrypt {

    uint64_t Xe(uint64_t addr)
    {
        try
        {
            static uint64_t Ptr = 0;
            if (DecFunction == nullptr)
            {
                int64_t UWorld = 0x0;
                int64_t DecryptPtr = 0x0;
                while (!UWorld || !DecryptPtr)
                {
                    DecryptPtr = mem.Read<uint64_t>(GameData.GameBase + Offset::XenuineDecrypt);
                    UWorld = mem.Read<uint64_t>(GameData.GameBase + Offset::UWorld);
                    Sleep(1000);
                }

                int32_t Tmp1Add = mem.Read<uint32_t>(DecryptPtr + 3);
                Ptr = Tmp1Add + DecryptPtr + 7;
                unsigned char ShellcodeBuff[1024] = { NULL };
                ShellcodeBuff[0] = 0x90;
                ShellcodeBuff[1] = 0x90;
                mem.Read(DecryptPtr, &ShellcodeBuff[2], sizeof(ShellcodeBuff) - 2);
                ShellcodeBuff[2] = 0x48;
                ShellcodeBuff[3] = 0x8B;
                ShellcodeBuff[4] = 0xC1;
                ShellcodeBuff[5] = 0x90;
                ShellcodeBuff[6] = 0x90;
                ShellcodeBuff[7] = 0x90;
                ShellcodeBuff[8] = 0x90;
                DecFunction = reinterpret_cast<decltype(DecFunction)> (VirtualAlloc(nullptr, 4096, MEM_RESERVE | MEM_COMMIT, PAGE_EXECUTE_READWRITE));
                RtlCopyMemory((LPVOID)DecFunction, (LPVOID)ShellcodeBuff, sizeof(ShellcodeBuff));
            }

            return  DecFunction(Ptr, addr);
        }
        catch (const std::exception&)
        {
            return 0;
        }
    }
}

void InitHotKey(){
    if (!mem.GetKeyboard()->InitKeyboard())
    {
        Utils::Log(2, "Failed to initialize keyboard hotkeys through kernel.");
    }
    else {
        Utils::Log(1, "Keyboard initialize success");
    }
}

void ListenHotKey() {

    while (true) {
        mem.GetKeyboard()->UpdateKeys();

        if (mem.GetKeyboard()->WasKeyPressed(VK_F1) && HitMeshPtr != nullptr) {
            ExportMeshAsObj(*HitMeshPtr);
        }

        Sleep(10);
    }
}


void StartLoadMapModel() {
    Utils::Log(1, "Starting load map model...");
    Utils::Log(1, "[Debug] Initializing Dynamic Load Scene...");
    GameData.DynamicLoadScene = new Physics::VisibleScene<PrunerPayload, PrunerPayloadHash>(Physics::prunerPayloadExtractor);
    Utils::Log(1, "[Debug] Initializing Height Field Scene...");
    GameData.HeightFieldScene = new Physics::VisibleScene<uint64_t, Int64Hash>(Physics::int64Extractor);
    Utils::Log(1, "[Debug] Initializing Dynamic Rigid Scene...");
    GameData.DynamicRigidScene = new Physics::VisibleScene<PrunerPayload, PrunerPayloadHash>(Physics::prunerPayloadExtractor);

    Utils::Log(1, "[Debug] Starting scene update threads...");
    std::thread LoadDynamicHeightFieldThread(VisibleCheck::UpdateDynamicHeightField);
    std::thread LoadDynamicRigidThread(VisibleCheck::UpdateDynamicRigid);
    std::thread LoadSceneByRangeThread(VisibleCheck::UpdateSceneByRange);
    LoadDynamicHeightFieldThread.detach();
    LoadDynamicRigidThread.detach();
    LoadSceneByRangeThread.detach();
    Utils::Log(1, "[Debug] All scene update threads started successfully");
}

void RenderGameData() {

    uintptr_t UWorld                = Decrypt::Xe(mem.Read<uint64_t>(GameData.GameBase + Offset::UWorld));
    uintptr_t GameInstance          = Decrypt::Xe(mem.Read<uint64_t>(UWorld + Offset::GameInstance));
    uintptr_t LocalPlayer           = Decrypt::Xe(mem.Read<uint64_t>(mem.Read<uint64_t>(GameInstance + Offset::LocalPlayer)));
    uintptr_t PlayerController      = Decrypt::Xe(mem.Read<uint64_t>(LocalPlayer + Offset::PlayerController));
    uint64_t PlayerCameraManager    = mem.Read<uint64_t>(PlayerController + Offset::PlayerCameraManager);
    FVector playerLocation          = mem.Read<FVector>(PlayerCameraManager + Offset::CameraCacheLocation);
    FRotator playerRotation         = mem.Read<FRotator>(PlayerCameraManager + Offset::CameraCacheRotation);
    float fov                       = mem.Read<float>(PlayerCameraManager + Offset::CameraCacheFOV);


    FIntVector2D WorldOrigin = mem.Read<FIntVector2D>(UWorld + Offset::WorldToMap);

    GameData.WorldOriginLocation = FVector(
        static_cast<float>(WorldOrigin.X),
        static_cast<float>(WorldOrigin.Y),
        0
    );
    GameData.CameraLocation = playerLocation;

    playerLocation = playerLocation + GameData.WorldOriginLocation;

    // Raycast Mode
    Raycast(playerRotation, playerLocation, fov);
    // Near Mode
    //RenderTriangleMesh(playerRotation, playerLocation, fov);
}

void RenderTriangleMesh(FRotator& playerRotation, FVector& playerLocation, float fov)
{
    // dynamic rigid and static rigid
    std::vector<TriangleMeshData*> meshs;
    auto dynamicMeshs = GameData.DynamicRigidScene->GetNearMesh(playerLocation, 1000.0 * 1000.0);
    auto staticMeshs = GameData.DynamicLoadScene->GetNearMesh(playerLocation, 1000.0 * 1000.0);
    meshs.insert(meshs.end(), dynamicMeshs.begin(), dynamicMeshs.end());
    meshs.insert(meshs.end(), staticMeshs.begin(), staticMeshs.end());
    for (size_t i = 0; i < meshs.size(); i++)
    {
        const TriangleMeshData& mesh_data = *meshs[i];

        for (const auto meshPtr : meshs) {
            TriangleMeshData& mesh = *meshPtr;
            try {

                if (!mesh.Vertices.empty()
                    && !mesh.Indices.empty() && static_cast<uint32_t>(mesh.Type) < 10
                    && static_cast<uint32_t>(mesh.Type) >= 0
                    && mesh.Vertices.size() < 1000000
                    && mesh.Indices.size() < 1000000)
                {

                    for (size_t i = 0; i < mesh.Indices.size(); i += 3)
                    {
                        if (i + 2 >= mesh.Indices.size())
                            break;

                        Vector3 v0 = mesh.Vertices[mesh.Indices[i]];
                        Vector3 v1 = mesh.Vertices[mesh.Indices[i + 1]];
                        Vector3 v2 = mesh.Vertices[mesh.Indices[i + 2]];

                        auto p0 = VectorHelper::WorldToScreen((FVector&)v0, playerRotation, playerLocation, fov);
                        auto p1 = VectorHelper::WorldToScreen((FVector&)v1, playerRotation, playerLocation, fov);
                        auto p2 = VectorHelper::WorldToScreen((FVector&)v2, playerRotation, playerLocation, fov);

                        if (i == 0) {
                            // convert flags to binary
                            std::string binary;
                            for (int bit = 31; bit >= 0; bit--) {
                                binary += ((mesh.Flags >> bit) & 1) ? '1' : '0';
                            }

                            // Debug info
                            /*ImGui::GetForegroundDrawList()->AddText(
                                ImVec2(p0.X, p0.Y - 60),
                                ImColor(255, 255, 255, 255),
                                (
                                    "Binary: " + binary + "\nDecimal: " + std::to_string(mesh.Flags) +
                                    "\nQuery Filter Data: " +
                                    std::to_string(mesh.QueryFilterData.word0) + "," +
                                    std::to_string(mesh.QueryFilterData.word1) + "," +
                                    std::to_string(mesh.QueryFilterData.word2) + "," +
                                    std::to_string(mesh.QueryFilterData.word3) +
                                    "\nSim Filter Data: " +
                                    std::to_string(mesh.SimulationFilterData.word0) + "," +
                                    std::to_string(mesh.SimulationFilterData.word1) + "," +
                                    std::to_string(mesh.SimulationFilterData.word2) + "," +
                                    std::to_string(mesh.SimulationFilterData.word3) +
                                    "\nVerties/Indices Num: " +
                                    std::to_string(mesh.Vertices.size()) + "," +
                                    std::to_string(mesh.Indices.size()) +
                                    "\nUniqueKey: " +
                                    std::to_string(mesh.UniqueKey1.Shape) + "," +
                                    std::to_string(mesh.UniqueKey1.Actor)
                                    ).c_str()
                            );*/
                        }

                        if (VectorHelper::IsInScreen(p0) || VectorHelper::IsInScreen(p1) || VectorHelper::IsInScreen(p2))
                        {
                            auto A = ImVec2(p0.X, p0.Y);
                            auto B = ImVec2(p1.X, p1.Y);
                            auto C = ImVec2(p2.X, p2.Y);
                            ImGui::GetForegroundDrawList()->AddTriangle(ImVec2(p0.X, p0.Y), ImVec2(p1.X, p1.Y), ImVec2(p2.X, p2.Y), ImColor(255, 255, 255, 200), 0.5f);
                        }
                    }
                }
            }
            catch (...) {
                Utils::Log(2, "Wild pointer error");
            }
        }
    }
}

void ExportMeshAsObj(TriangleMeshData& mesh) {
    std::ofstream outFile(std::to_string(mesh.Vertices.size()) + "_" + std::to_string(mesh.Indices.size()) + ".obj");
    if (!outFile.is_open()) {
        return;
    }

    // 写入顶点数据
    for (const auto& vertex : mesh.Vertices) {
        outFile << "v " << vertex.x << " " << vertex.y << " " << vertex.z << "\n";
    }

    // 写入面数据
    for (size_t i = 0; i < mesh.Indices.size(); i += 3) {
        // OBJ文件的索引从1开始,所以要+1
        outFile << "f " << mesh.Indices[i] + 1 << " " 
                       << mesh.Indices[i + 1] + 1 << " "
                       << mesh.Indices[i + 2] + 1 << "\n";
    }

    outFile.close();
    Utils::Log(1, "[Debug] Exported mesh with %zu vertices and %zu indices", mesh.Vertices.size(), mesh.Indices.size());

}

void Raycast(FRotator& playerRotation, FVector& playerLocation, float fov)
{

    FVector forwardVector;
    forwardVector.X = cos(playerRotation.Yaw * M_PI / 180.0f) * cos(playerRotation.Pitch * M_PI / 180.0f);
    forwardVector.Y = sin(playerRotation.Yaw * M_PI / 180.0f) * cos(playerRotation.Pitch * M_PI / 180.0f);
    forwardVector.Z = sin(playerRotation.Pitch * M_PI / 180.0f);
    FVector target = playerLocation + forwardVector * 100000.0f; // Project 100000 units in view direction
    auto screenTarget = VectorHelper::WorldToScreen(target, playerRotation, playerLocation, fov);

    TriangleMeshData* mesh_ptr = LineTrace::getNextHint(playerRotation, playerLocation, fov);

    if (mesh_ptr == nullptr) {
        //Utils::Log(1, "[Debug] No mesh data found from line trace");
        HitMeshPtr = nullptr;
        return;
    }

    const TriangleMeshData& mesh = *mesh_ptr;

    try {

        if (!mesh.Vertices.empty()
            && !mesh.Indices.empty() && static_cast<uint32_t>(mesh.Type) < 10
            && static_cast<uint32_t>(mesh.Type) >= 0
            && mesh.Vertices.size() < 1000000
            && mesh.Indices.size() < 1000000)
        {
            for (size_t i = 0; i < mesh.Indices.size(); i += 3)
            {
                if (i + 2 >= mesh.Indices.size())
                    break;

                Vector3 v0 = mesh.Vertices[mesh.Indices[i]];
                Vector3 v1 = mesh.Vertices[mesh.Indices[i + 1]];
                Vector3 v2 = mesh.Vertices[mesh.Indices[i + 2]];

                auto p0 = VectorHelper::WorldToScreen((FVector&)v0, playerRotation, playerLocation, fov);
                auto p1 = VectorHelper::WorldToScreen((FVector&)v1, playerRotation, playerLocation, fov);
                auto p2 = VectorHelper::WorldToScreen((FVector&)v2, playerRotation, playerLocation, fov);

                if (i == 0) {
                    // convert flags to binary
                    std::string binary;
                    for (int bit = 31; bit >= 0; bit--) {
                        binary += ((mesh.Flags >> bit) & 1) ? '1' : '0';
                    }

                    //// Debug info
                    //ImGui::GetForegroundDrawList()->AddText(
                    //    ImVec2(p0.X, p0.Y - 60),
                    //    ImColor(255, 0, 0, 255),
                    //    (
                    //        "Binary: " + binary + "\nDecimal: " + std::to_string(mesh.Flags) +
                    //        "\nQuery Filter Data: " +
                    //        std::to_string(mesh.QueryFilterData.word0) + "," +
                    //        std::to_string(mesh.QueryFilterData.word1) + "," +
                    //        std::to_string(mesh.QueryFilterData.word2) + "," +
                    //        std::to_string(mesh.QueryFilterData.word3) +
                    //        "\nSim Filter Data: " +
                    //        std::to_string(mesh.SimulationFilterData.word0) + "," +
                    //        std::to_string(mesh.SimulationFilterData.word1) + "," +
                    //        std::to_string(mesh.SimulationFilterData.word2) + "," +
                    //        std::to_string(mesh.SimulationFilterData.word3) +
                    //        "\nVerties/Indices Num: " +
                    //        std::to_string(mesh.Vertices.size()) + "," +
                    //        std::to_string(mesh.Indices.size()) +
                    //        "\nUniqueKey: " +
                    //        std::to_string(mesh.UniqueKey1.Shape) + "," +
                    //        std::to_string(mesh.UniqueKey1.Actor)
                    //    ).c_str()
                    //);
                }

                if (VectorHelper::IsInScreen(p0) || VectorHelper::IsInScreen(p1) || VectorHelper::IsInScreen(p2))
                {
                    auto A = ImVec2(p0.X, p0.Y);
                    auto B = ImVec2(p1.X, p1.Y);
                    auto C = ImVec2(p2.X, p2.Y);
                    ImGui::GetForegroundDrawList()->AddTriangle(ImVec2(p0.X, p0.Y), ImVec2(p1.X, p1.Y), ImVec2(p2.X, p2.Y), ImColor(255, 255, 255, 200), 0.5f);
                }
            }
            HitMeshPtr = mesh_ptr;
        }
        else {
            HitMeshPtr = nullptr;
        }
    }
    catch (...) {
        Utils::Log(2, "Wild pointer error");
    }
}

int Render(const HWND& window, ID3D11RenderTargetView*& render_target_view,
    ID3D11DeviceContext* device_context, IDXGISwapChain* swap_chain) {
    while (TRUE) {
        MSG msg;
        while (PeekMessage(&msg, nullptr, 0U, 0U, PM_REMOVE)) {
            TranslateMessage(&msg);
            DispatchMessage(&msg);

            if (msg.message == WM_QUIT) {
                return 0;
            }

            ImGui_ImplWin32_WndProcHandler(window, msg.message, msg.wParam, msg.lParam);
        }

        // Prepare new frame
        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        RenderGameData();

        // Complete ImGui rendering
        ImGui::Render();

        // Clear background
        constexpr float color[4]{ 0.f, 0.f, 0.f, 0.f };

        if (render_target_view) {
            device_context->OMSetRenderTargets(1U, &render_target_view, nullptr);
            device_context->ClearRenderTargetView(render_target_view, color);
        }

        // Display rendering result
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
        swap_chain->Present(0U, 0U);

        Sleep(1000 / 60);
    }
}

int main() {
    if (!mem.Init("TslGame.exe", true, false))
    {
        Utils::Log(2, "Failed to initialize DMA");
        system("pause");
    }
    else
    {
        Utils::Log(1, "DMA initialization successful");
    }
    mem.GetTslGamePID();

    InitHotKey();
    std::thread hotKeyThread(ListenHotKey);

    WNDCLASSEXW wc = {};
    wc.cbSize = sizeof(WNDCLASSEXW);
    wc.style = CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc = window_procedure;
    wc.lpszClassName = L"Extern Overlay Class";

    RegisterClassExW(&wc);

    const HWND hwnd = CreateWindowExW(
        WS_EX_TOPMOST | WS_EX_TRANSPARENT | WS_EX_LAYERED,
        wc.lpszClassName,
        L"PhysX Demo",
        WS_POPUP,
        0, 0, 1920, 1080,
        nullptr, nullptr,
        wc.hInstance, nullptr
    );

    SetLayeredWindowAttributes(hwnd, RGB(0, 0, 0), BYTE(255), LWA_ALPHA);

    {
        RECT client_area{};
        GetClientRect(hwnd, &client_area);

        RECT window_area{};
        GetClientRect(hwnd, &window_area);

        POINT diff{};
        ClientToScreen(hwnd, &diff);

        const MARGINS margins{
            window_area.left + (diff.x - window_area.left),
            window_area.top + (diff.y - window_area.top),
            client_area.right,
            client_area.bottom
        };

        DwmExtendFrameIntoClientArea(hwnd, &margins);
        Utils::Log(1, "[Debug] Window attributes configured");
    }

    DXGI_SWAP_CHAIN_DESC sd{};

    HMONITOR hMonitor = MonitorFromWindow(GetDesktopWindow(), MONITOR_DEFAULTTOPRIMARY);

    MONITORINFOEXW monitorInfo;
    monitorInfo.cbSize = sizeof(MONITORINFOEXW);
    GetMonitorInfoW(hMonitor, &monitorInfo);

    DEVMODEW devMode;
    devMode.dmSize = sizeof(DEVMODEW);
    devMode.dmDriverExtra = 0;
    EnumDisplaySettingsW(monitorInfo.szDevice, ENUM_CURRENT_SETTINGS, &devMode);

    // Configure swap chain
    sd.BufferDesc.RefreshRate.Numerator = 144;
    sd.BufferDesc.RefreshRate.Denominator = 1;
    sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.SampleDesc.Count = 1U;
    sd.OutputWindow = hwnd;
    sd.Windowed = TRUE;
    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.BufferCount = 2U;
    sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;
    sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;

    // Initialize DirectX device and context
    constexpr D3D_FEATURE_LEVEL levels[2]{
        D3D_FEATURE_LEVEL_11_0,
        D3D_FEATURE_LEVEL_10_0
    };

    ID3D11Device* device{ nullptr };
    ID3D11DeviceContext* device_context{ nullptr };
    IDXGISwapChain* swap_chain{ nullptr };
    ID3D11RenderTargetView* render_target_view{ nullptr };
    D3D_FEATURE_LEVEL level{};

    D3D11CreateDeviceAndSwapChain(
        nullptr,
        D3D_DRIVER_TYPE_HARDWARE,
        nullptr,
        0U,
        levels,
        2U,
        D3D11_SDK_VERSION,
        &sd,
        &swap_chain,
        &device,
        &level,
        &device_context
    );

    ID3D11Texture2D* back_buffer{ nullptr };
    swap_chain->GetBuffer(0U, IID_PPV_ARGS(&back_buffer));

    if (back_buffer) {
        device->CreateRenderTargetView(back_buffer, nullptr, &render_target_view);
        back_buffer->Release();
    }
    else {
        Utils::Log(2, "[Error] Failed to create back buffer");
        return 1;
    }

    ShowWindow(hwnd, SW_SHOWDEFAULT);
    UpdateWindow(hwnd);

    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX11_Init(device, device_context);

    Utils::Log(1, "[Debug] Starting map model loading...");
    StartLoadMapModel();

    while (TRUE) {
        Render(hwnd, render_target_view, device_context, swap_chain);
    }

    Utils::Log(1, "[Debug] Cleaning up resources...");
    ReleaseResource(hwnd, swap_chain, device_context, device, render_target_view);
    UnregisterClassW(wc.lpszClassName, wc.hInstance);
    Utils::Log(1, "[Debug] Application shutdown complete");

    return 0;
}