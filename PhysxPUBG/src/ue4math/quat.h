#pragma once
#include "ue4math.h"

struct FVector;
struct FMatrix;

// ScriptStruct CoreUObject.Quat
// 0x0010
struct alignas(16) FQuat {
public:
	float                                              X;                                                        // 0x0000(0x0004) (CPF_Edit, CPF_BlueprintVisible, CPF_ZeroConstructor, CPF_SaveGame, CPF_IsPlainOldData)
	float                                              Y;                                                        // 0x0004(0x0004) (CPF_Edit, CPF_BlueprintVisible, CPF_ZeroConstructor, CPF_SaveGame, CPF_IsPlainOldData)
	float                                              Z;                                                        // 0x0008(0x0004) (CPF_Edit, CPF_BlueprintVisible, CPF_ZeroConstructor, CPF_SaveGame, CPF_IsPlainOldData)
	float                                              W;

	FQuat(float X = 0.0, float Y = 0.0, float Z = 0.0, float W = 1.0) : X(X), Y(Y), Z(Z), W(W) {}

	static void VectorQuaternionMultiply(void* Result, const void* Quat1, const void* Quat2) {
		typedef float Float4[4];
		const Float4& A = *((const Float4*)Quat1);
		const Float4& B = *((const Float4*)Quat2);
		Float4& R = *((Float4*)Result);

		const float T0 = (A[2] - A[1]) * (B[1] - B[2]);
		const float T1 = (A[3] + A[0]) * (B[3] + B[0]);
		const float T2 = (A[3] - A[0]) * (B[1] + B[2]);
		const float T3 = (A[1] + A[2]) * (B[3] - B[0]);
		const float T4 = (A[2] - A[0]) * (B[0] - B[1]);
		const float T5 = (A[2] + A[0]) * (B[0] + B[1]);
		const float T6 = (A[3] + A[1]) * (B[3] - B[2]);
		const float T7 = (A[3] - A[1]) * (B[3] + B[2]);
		const float T8 = T5 + T6 + T7;
		const float T9 = 0.5f * (T4 + T8);

		R[0] = T1 + T9 - T8;
		R[1] = T2 + T9 - T7;
		R[2] = T3 + T9 - T6;
		R[3] = T0 + T9 - T5;
	}

	FQuat operator*(const FQuat& Q) const {
		FQuat Result;
		VectorQuaternionMultiply(&Result, this, &Q);
		return Result;
	}

	void Normalize(float Tolerance = SMALL_NUMBER)
	{
		const float SquareSum = X * X + Y * Y + Z * Z + W * W;

		if (SquareSum >= Tolerance)
		{
			const float Scale = InvSqrt(SquareSum);

			X *= Scale;
			Y *= Scale;
			Z *= Scale;
			W *= Scale;
		}
		else
		{
			*this = FQuat();
		}
	}



	FVector rotateInv(const FVector& v) const;

	float SizeSquared() const { return (X * X + Y * Y + Z * Z + W * W); }
	bool IsNormalized() const { return (fabs(1.0 - SizeSquared()) < THRESH_QUAT_NORMALIZED); }
	FQuat Inverse() const { return FQuat(-X, -Y, -Z, W); }

	FQuat(const FMatrix& M);

	FVector RotateVector(const FVector& V) const;
	FVector operator*(const FVector& V) const;
};

static_assert(sizeof(FQuat) == 0x10, "FQuat");