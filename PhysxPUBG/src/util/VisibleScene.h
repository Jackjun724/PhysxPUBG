#pragma once
#include <embree4/rtcore.h>
#include <embree4/rtcore_ray.h>
#include <mutex>
#include <vector>
#include "math.h"
#include "../visible/Physx.h"
#include "./Utils.h"

namespace Physics {

using namespace physx;
using namespace std;

static auto prunerPayloadExtractor = [](const TriangleMeshData& mesh) -> PrunerPayload {
    return mesh.UniqueKey1;
};

static auto int64Extractor = [](const TriangleMeshData& mesh) -> uint64_t {
    return mesh.UniqueKey2;
};

static auto normal64Extractor = [](const TriangleMeshData& mesh) -> uint64_t {
    return (uint64_t)rand();
};

static void embreeErrorFunction(void* userPtr, RTCError code, const char* str) {
    Utils::Log(0, "[DEBUG] Embree Error [%d]: %s", code, str);
}

template <typename T, typename Hash>
class VisibleScene
{
public:
    using KeyExtractor = T(*)(const TriangleMeshData&);

    VisibleScene(KeyExtractor keyExtractor) : 
        getKey(keyExtractor) {
        this->device = rtcNewDevice(nullptr);
        rtcSetDeviceErrorFunction(device, embreeErrorFunction, nullptr);
        this->scene = rtcNewScene(device);
        rtcSetDeviceProperty(device, RTC_DEVICE_PROPERTY_TASKING_SYSTEM, 1); // USE TBB
        rtcSetSceneBuildQuality(scene, RTC_BUILD_QUALITY_LOW);
        rtcSetSceneFlags(scene, RTC_SCENE_FLAG_DYNAMIC);
        rtcCommitScene(this->scene);
    }

    ~VisibleScene() {
        mesh_datas.clear();
        if (scene) {
            rtcReleaseScene(scene);
            scene = nullptr; // 避免重复释放
        }
        if (device) {
            rtcReleaseDevice(device);
            device = nullptr; // 避免重复释放
        }
    }

    void UpdateMesh(
        const vector<TriangleMeshData>& willAddMeshs,
        const set<T>& RemoveKey
    ){
        // remove geometry use disable
        vector<RTCGeometry> willRemoveGeometry;
        for (auto& key : RemoveKey) {
            if (geometry_id_map.find(key) != geometry_id_map.end()) {
                auto geometry_id = geometry_id_map[key];
                auto geometry = rtcGetGeometry(scene, geometry_id);
                rtcDisableGeometry(geometry);
                disabled_geometry_ids.insert(geometry_id);
                geometry_id_map.erase(key);
            }
        }

        // remove mesh data
        if (!mesh_datas.empty()) {
            mesh_datas.erase(
                remove_if(
                    mesh_datas.begin(), mesh_datas.end(),
                    [this, &RemoveKey](const shared_ptr<TriangleMeshData>& mesh) {
                        return RemoveKey.find(this->getKey(*mesh)) != RemoveKey.end();
                    }
                ), 
                mesh_datas.end()
            );
        }


        // add geometry or enable
        for (auto& mesh : willAddMeshs) {
            if (mesh.Vertices.size() == 0 || mesh.Indices.size() == 0) {
                continue;
            }
            RTCGeometry geom;
            bool should_release = false;
            uint32_t geometry_id = 0;
            auto mesh_copy = make_shared<TriangleMeshData>(mesh);
            mesh_datas.push_back(mesh_copy);
            if (!disabled_geometry_ids.empty()) {
                // use disabled geometry
                geometry_id = *disabled_geometry_ids.begin();
                disabled_geometry_ids.erase(disabled_geometry_ids.begin());
                geom = rtcGetGeometry(scene, geometry_id);
                rtcEnableGeometry(geom);
            } else {
                // create new geometry
                geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
                should_release = true;
            }


            // set vertices buffer
            float* vertices = (float*)rtcSetNewGeometryBuffer(geom,
                RTC_BUFFER_TYPE_VERTEX,
                0,
                RTC_FORMAT_FLOAT3,
                3 * sizeof(float),
                mesh.Vertices.size());

            if (!vertices) {
                Utils::Log(2, "Error: Failed to allocate vertex buffer");
                if (should_release) {
                    rtcReleaseGeometry(geom);
                } else {
                    rtcDisableGeometry(geom);
                    disabled_geometry_ids.insert(geometry_id);
                }
                break;
            }

            // Copie data
            for (size_t i = 0; i < mesh.Vertices.size(); i++) {
                vertices[i * 3] = mesh.Vertices[i].x;
                vertices[i * 3 + 1] = mesh.Vertices[i].y;
                vertices[i * 3 + 2] = mesh.Vertices[i].z;
            }

            auto bufferSize = mesh.Indices.size() / 3;
            // set indices buffer
            unsigned int* indices = (unsigned int*)rtcSetNewGeometryBuffer(geom,
                RTC_BUFFER_TYPE_INDEX,
                0,
                RTC_FORMAT_UINT3,
                3 * sizeof(unsigned int),
                bufferSize);

            if (!indices) {
                Utils::Log(2, "Error: Failed to allocate index buffer");
                if (should_release) {
                    rtcReleaseGeometry(geom);
                } else {
                    rtcDisableGeometry(geom);
                    disabled_geometry_ids.insert(geometry_id);
                }
                break;
            }


            // copy indices buffer
            memcpy(indices, mesh.Indices.data(), mesh.Indices.size() * sizeof(uint32_t));

            rtcSetGeometryUserData(geom, mesh_copy.get());
            rtcCommitGeometry(geom);

            if (should_release) {
                try {
                    geometry_id = rtcAttachGeometry(scene, geom);
                }
                catch (...) {
                    Utils::Log(2, "Error attach geom.");
                    rtcReleaseGeometry(geom);
                    return;
                }
                rtcReleaseGeometry(geom);
            }
            // 使用key提取器获取key
            T key = getKey(mesh);
            geometry_id_map.insert({key, geometry_id});
        }

        // commit scene
        rtcCommitScene(scene);
    }

    RTCRayHit Raycast(FVector& origin, FVector& target)
    {
        try {
            // define ray
            RTCRay ray;
            ray.org_x = origin.X;
            ray.org_y = origin.Y;
            ray.org_z = origin.Z;
            ray.dir_x = target.X - origin.X;
            ray.dir_y = target.Y - origin.Y;
            ray.dir_z = target.Z - origin.Z;

            // norm
            float dir_length = std::sqrt(ray.dir_x * ray.dir_x + ray.dir_y * ray.dir_y + ray.dir_z * ray.dir_z);
            ray.dir_x /= dir_length;
            ray.dir_y /= dir_length;
            ray.dir_z /= dir_length;

            ray.tnear = 0.0f;       // start distance
            ray.tfar = dir_length;  // total distance
            ray.mask = -1;
            ray.flags = 0;

            // init result var
            RTCRayHit rayhit;
            rayhit.ray = ray;
            rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;

            RTCRayQueryContext context;
            rtcInitRayQueryContext(&context);

            RTCIntersectArguments intersectArgs;
            rtcInitIntersectArguments(&intersectArgs);
            intersectArgs.context = &context;

            if (this) {
                rtcIntersect1(this->scene, &rayhit, &intersectArgs);
            }
            return rayhit;
        }
        catch (...) {
            Utils::Log(2, "Raycast error");
            RTCRayHit rayhit;
            rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
            return rayhit;
        }
    }

    TriangleMeshData* GetGeomeoryData(uint32_t geomId) {
        return (TriangleMeshData*)rtcGetGeometryUserData(rtcGetGeometry(scene, geomId));
    }

    vector<TriangleMeshData*> GetNearMesh(FVector Position, double RadiusSquare) const {
       
        vector<TriangleMeshData*> result;
        
        // 遍历所有网格数据
        for (const auto& meshData : mesh_datas) {
            // 计算网格中心点到Position的距离
            auto meshCenter = meshData->Transform.mPosition;
            Vector3 position = { Position.X, Position.Y, Position.Z };
            auto distance = (meshCenter - position).Length2DSquare();
            
            // 如果在指定半径内,加入结果集
            if (distance <= RadiusSquare) {
                result.push_back(meshData.get());
            }
        }
        return result;
    }

private:
    RTCDevice device;
    RTCScene  scene;
    std::unordered_map<T, uint32_t, Hash> geometry_id_map = {};
    set<uint32_t> disabled_geometry_ids = {};
    vector<shared_ptr<TriangleMeshData>> mesh_datas = {};
    KeyExtractor getKey;

};

} // namespace Physics
