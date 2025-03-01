#pragma once
#include "./VisibleScene.h"
#include "./Data.h"
#include "./Utils.h"
#include "./Throttler.h"

namespace VisibleCheck {

	void UpdateSceneByRange() {
		std::unordered_map<PrunerPayload, PxTransformT, PrunerPayloadHash> cache{};
		std::set<PrunerPayload> currentSceneObjects{};
		std::unordered_map<PrunerPayload, uint64_t, PrunerPayloadHash> alwaysCheckShape{};
		uint32_t lastUpdateTimestamp = 0;
		Throttler Throttlered;
		while (true) {
			Throttlered.executeTaskWithSleep("UpdateSceneByRangeSleep", std::chrono::milliseconds(3000), [&cache, &currentSceneObjects, &lastUpdateTimestamp, &alwaysCheckShape] {
				std::set<PrunerPayload> willRemoveObjects{};
				FVector currentPosition = GameData.CameraLocation + GameData.WorldOriginLocation;
				if (currentPosition.IsNearlyZero()) {
					return;
				}
#ifdef _PHYSX_DEBUG
				auto start = std::chrono::high_resolution_clock::now();
#endif
				auto Meshs = physx::LoadShapeByRange(
					lastUpdateTimestamp,
					cache,
					currentSceneObjects,
					willRemoveObjects,
					alwaysCheckShape,
					{currentPosition.X, currentPosition.Y, currentPosition.Z},
					GameData.LoadRadius * 100.f,
					GameData.PhysxRefreshLimit
				);

				if (!Meshs.empty() || !willRemoveObjects.empty()) {
#ifdef _PHYSX_DEBUG
					auto end = std::chrono::high_resolution_clock::now();
					auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
					Utils::Log(1, "Static: Load %d meshs cost %d ms", Meshs.size(), duration.count());
#endif
					try {
						GameData.DynamicLoadScene->UpdateMesh(Meshs, willRemoveObjects);
					}
					catch (...) {
						Utils::Log(3, "Static: Update Mesh error");
					}
#ifdef _PHYSX_DEBUG
					end = std::chrono::high_resolution_clock::now();
					duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
					Utils::Log(3, "Static: Update embree scene cost %d ms", duration.count());
#endif
				}
			});

		}

	}

	void UpdateDynamicHeightField() {
		std::set<PrunerPayload> UniqueSet{};
		std::set<PrunerPayload> HeightFieldSet{};
		std::set<uint64_t> HeightFieldSamplePtrSet{};
		uint32_t lastUpdateTimestamp = 0;
		Throttler Throttlered;
		while (true) {
			Throttlered.executeTaskWithSleep("HeightFieldUpdateSleep", std::chrono::milliseconds(3000), [&UniqueSet, &HeightFieldSet, &HeightFieldSamplePtrSet, &lastUpdateTimestamp] {
				std::set<uint64_t> RemoveHeightFieldKey{};
#ifdef _PHYSX_DEBUG
				auto start = std::chrono::high_resolution_clock::now();
#endif
				auto Meshs = physx::RefreshDynamicLoadHeightField(
					lastUpdateTimestamp, UniqueSet,
					HeightFieldSet,
					HeightFieldSamplePtrSet,
					RemoveHeightFieldKey
				);

				if (!Meshs.empty() || !RemoveHeightFieldKey.empty()) {
#ifdef _PHYSX_DEBUG
					auto end = std::chrono::high_resolution_clock::now();
					auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
					Utils::Log(1, "Load %d meshs cost %d ms", Meshs.size(), duration.count());
#endif
					try {
						GameData.HeightFieldScene->UpdateMesh(Meshs, RemoveHeightFieldKey);
					}
					catch (...) {
						Utils::Log(3, "Update Height Field Mesh error");
					}

#ifdef _PHYSX_DEBUG
					end = std::chrono::high_resolution_clock::now();
					duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
					Utils::Log(3, "Update embree scene cost %d ms", duration.count());
#endif
				}
			});
		}
	}

	void UpdateDynamicRigid() {
		Throttler Throttlered;
		std::unordered_map<PrunerPayload, PxTransformT, PrunerPayloadHash> cache{};
		std::unordered_map<PrunerPayload, uint64_t, PrunerPayloadHash> ptrCache{};
		std::set<PrunerPayload> currentSceneObjects{};
		while (true) {
			Throttlered.executeTaskWithSleep("DynamicRigidUpdateSleep", std::chrono::milliseconds(300), [&currentSceneObjects, &cache, &ptrCache] {
				FVector currentPosition = GameData.CameraLocation + GameData.WorldOriginLocation;
				if (currentPosition.IsNearlyZero()) {
					return;
				}
				std::set<PrunerPayload> willRemoveShape{};
#ifdef _PHYSX_DEBUG
				auto start = std::chrono::high_resolution_clock::now();
#endif
				auto Meshs = physx::LoadDynamicRigidShape(
					currentSceneObjects,
					cache,
					ptrCache,
					willRemoveShape,
					{ currentPosition.X, currentPosition.Y, currentPosition.Z },
					GameData.LoadRadius * 100.f
				);

				if (!Meshs.empty() || !willRemoveShape.empty()) {
#ifdef _PHYSX_DEBUG
					auto end = std::chrono::high_resolution_clock::now();
					auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
					Utils::Log(1, "Dynamic: Load %d meshs cost %d ms", Meshs.size(), duration.count());
#endif
					try {
						GameData.DynamicRigidScene->UpdateMesh(Meshs, willRemoveShape);
					}
					catch (...) {
						Utils::Log(3, "Dynamic: Update Mesh error");
					}

#ifdef _PHYSX_DEBUG
					end = std::chrono::high_resolution_clock::now();
					duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
					Utils::Log(3, "Dynamic: Update embree scene cost %d ms", duration.count());
#endif
				}
				});
		}
	}

}