#pragma once
#include <DMALibrary/Memory/Memory.h>
#include "./Data.h"
#include "./VisibleScene.h"

namespace LineTrace
{
    /// <summary>
    /// 判断是否可视
    /// </summary>
    /// <param name="TraceStart"></param>
    /// <param name="TraceEnd"></param>
    /// <returns></returns>
    static bool LineTraceSingle(FVector TraceStart, FVector TraceEnd)
    {
        if (GameData.DynamicLoadScene == nullptr || GameData.HeightFieldScene == nullptr || GameData.DynamicRigidScene == nullptr) {
            return true;
        }
        FVector origin = TraceStart + GameData.WorldOriginLocation;
        FVector target = TraceEnd + GameData.WorldOriginLocation;
        auto dynamicRayHit = GameData.DynamicLoadScene->Raycast(origin, target);
        if (dynamicRayHit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
            return false;
        }
        auto heightFieldRayHit = GameData.HeightFieldScene->Raycast(origin, target);
        if (heightFieldRayHit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
            return false;
        }
        auto globalSceneRayHit = GameData.DynamicRigidScene->Raycast(origin, target);
        if (globalSceneRayHit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
            return false;
        }
        return true;
    }

    static TriangleMeshData* getNextHint(FRotator& playerRotation, FVector& playerLocation, float fov) {
        if (GameData.DynamicLoadScene == nullptr || GameData.HeightFieldScene == nullptr || GameData.DynamicRigidScene == nullptr) {
            return nullptr;
        }
        FVector forwardVector;
        forwardVector.X = cos(playerRotation.Yaw * M_PI / 180.0f) * cos(playerRotation.Pitch * M_PI / 180.0f);
        forwardVector.Y = sin(playerRotation.Yaw * M_PI / 180.0f) * cos(playerRotation.Pitch * M_PI / 180.0f);
        forwardVector.Z = sin(playerRotation.Pitch * M_PI / 180.0f);
        FVector origin = playerLocation;
        FVector target = playerLocation + forwardVector * 100000.0f; // 向视线方向1000米
        auto dynamicRayHit = GameData.DynamicLoadScene->Raycast(origin, target);
        auto heightFieldRayHit = GameData.HeightFieldScene->Raycast(origin, target);
        auto globalSceneRayHit = GameData.DynamicRigidScene->Raycast(origin, target);

        if (dynamicRayHit.hit.geomID == RTC_INVALID_GEOMETRY_ID && 
            heightFieldRayHit.hit.geomID == RTC_INVALID_GEOMETRY_ID && 
            globalSceneRayHit.hit.geomID == RTC_INVALID_GEOMETRY_ID) {
            return nullptr;
        }
        
        // return near
        float minDist = FLT_MAX;
        TriangleMeshData* result = nullptr;
        
        if (dynamicRayHit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
            if (dynamicRayHit.ray.tfar < minDist) {
                minDist = dynamicRayHit.ray.tfar;
                result = GameData.DynamicLoadScene->GetGeomeoryData(dynamicRayHit.hit.geomID);
            }
        }
        
        if (heightFieldRayHit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
            if (heightFieldRayHit.ray.tfar < minDist) {
                minDist = heightFieldRayHit.ray.tfar;
                result = GameData.HeightFieldScene->GetGeomeoryData(heightFieldRayHit.hit.geomID);
            }
        }
        
        if (globalSceneRayHit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
            if (globalSceneRayHit.ray.tfar < minDist) {
                minDist = globalSceneRayHit.ray.tfar;
                result = GameData.DynamicRigidScene->GetGeomeoryData(globalSceneRayHit.hit.geomID);
            }
        }
        
        return result;
    }
}