#pragma once
#include <cstdint>
#include <functional>
#include "../math/math.h"

namespace Physics {
	template<typename T, typename Hash>
	class VisibleScene;
}

enum class PxGeometryType : int32_t
{

	eSPHERE,	   // 0
	ePLANE,		   // 1
	eCAPSULE,	   // 2
	eBOX,		   // 3
	eCONVEXMESH,   // 4
	eTRIANGLEMESH, // 5
	eHEIGHTFIELD,  // 6

	eGEOMETRY_COUNT, //!< internal use only!
	eINVALID = -1	 //!< internal use only!
};

struct FIntVector2D {
	int X, Y;
};

struct FilterDataT
{
	uint32_t word0;
	uint32_t word1;
	uint32_t word2;
	uint32_t word3;
};

struct PrunerPayload
{
	uintptr_t Shape;
	uintptr_t Actor;

	bool operator==(const PrunerPayload& other) const {
		return Shape == other.Shape && Actor == other.Actor;
	}

	bool operator<(const PrunerPayload& other) const {
		if (Shape != other.Shape)
			return Shape < other.Shape;
		return Actor < other.Actor;
	}
};

struct PrunerPayloadHash {
	size_t operator()(const PrunerPayload& p) const {
		return std::hash<uintptr_t>()(p.Shape) ^ (std::hash<uintptr_t>()(p.Actor) << 1);
	}
};


struct Int64Hash {
	size_t operator()(const uint64_t& p) const {
		return std::hash<uint64_t>()(p);
	}
};

struct PxTransformT
{

	Vector4 mRotation{};
	Vector3 mPosition{};

	PxTransformT() = default;

	PxTransformT(const Vector4& rotation, const Vector3& position)
		: mRotation(rotation), mPosition(position)
	{
	}

	// Transform a vector using the transform's rotation and translation
	Vector3 transform(const Vector3& input) const
	{
		return mRotation.rotate(input) + mPosition;
	}

	PxTransformT operator*(const PxTransformT& other) const
	{
		Vector4 newRotation = mRotation * other.mRotation;
		Vector3 newPosition = mRotation.rotate(other.mPosition) + mPosition;
		return PxTransformT(newRotation, newPosition);
	}

	bool isNearlyZero(float value, float epsilon = 1e-6f)
	{
		return fabs(value) < epsilon;
	}

	PxTransformT getInverse() const
	{
		Vector4 invRotation = mRotation.conjugate();
		Vector3 invPosition = mRotation.rotateInv(-mPosition);
		return PxTransformT(invRotation, invPosition);
	}

	bool IsNearlyEqual(const PxTransformT& other, float tolerance = 0.1f) const {
		return mRotation.IsNearlyEqual(other.mRotation, tolerance) &&
			mPosition.IsNearlyEqual(other.mPosition, tolerance);
	}


};

struct TriangleMeshData
{
	std::vector<Vector3> Vertices{};
	std::vector<uint32_t> Indices{};
	uint8_t Flags{};
	FilterDataT QueryFilterData{};
	FilterDataT SimulationFilterData{};
	PrunerPayload UniqueKey1;
	uint64_t UniqueKey2;
	PxGeometryType Type{};
	PxTransformT Transform;
};

struct FGameData {
	uint64_t GameBase;
	DWORD PID;
	uint64_t PhysxInstancePtr;
	FVector WorldOriginLocation;
	FVector CameraLocation;

	// 刷新距离 300m
	float LoadRadius = 300.f;
	// 刷新幅度，只有场景变化大于这个数量才更新
	uint32_t PhysxRefreshLimit = 0;

	Physics::VisibleScene<PrunerPayload, PrunerPayloadHash>* DynamicLoadScene;
	Physics::VisibleScene<PrunerPayload, PrunerPayloadHash>* DynamicRigidScene;
	Physics::VisibleScene<uint64_t, Int64Hash>* HeightFieldScene;
	TriangleMeshData* NextHintMeshData;


};



extern FGameData GameData;
