#pragma once
#include <cstddef>
#include <cstdint>

namespace Offset {
    constexpr ptrdiff_t XenuineDecrypt = 0xE874628;
    constexpr ptrdiff_t UWorld = 0x102dcb68;
    constexpr ptrdiff_t GameInstance = 0x160;
    constexpr ptrdiff_t LocalPlayer = 0x38;
    constexpr ptrdiff_t PlayerController = 0x30;
    constexpr ptrdiff_t PlayerCameraManager = 0x4c0;
    constexpr ptrdiff_t CameraCacheLocation = 0xa60;
    constexpr ptrdiff_t CameraCacheRotation = 0x4d4;
    constexpr ptrdiff_t CameraCacheFOV = 0xa6c;
    constexpr ptrdiff_t WorldToMap = 0x9b4;

    constexpr uint64_t Physx = 0xFD07848;
}

