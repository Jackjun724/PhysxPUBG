#pragma once
#include "ue4math.h"
#include "vector.h"
#include "quat.h"

struct FMatrix;

// ScriptStruct CoreUObject.Transform
// 0x0030
struct alignas(16) FTransform {
public:
	struct FQuat                                       Rotation;                                                 // 0x0000(0x0010) (CPF_Edit, CPF_BlueprintVisible, CPF_SaveGame, CPF_IsPlainOldData)
	struct FVector                                     Translation;                                              // 0x0010(0x000C) (CPF_Edit, CPF_BlueprintVisible, CPF_SaveGame, CPF_IsPlainOldData)
private:	unsigned char                              UnknownData00[0x4];                                       // 0x001C(0x0004) MISSED OFFSET
public:		struct FVector                             Scale3D;                                                  // 0x0020(0x000C) (CPF_Edit, CPF_BlueprintVisible, CPF_SaveGame, CPF_IsPlainOldData)

	FTransform();
	FTransform(const FQuat& Rotation, const FVector& Translation, const FVector& Scale3D);
	static bool AnyHasNegativeScale(const FVector& InScale3D, const FVector& InOtherScale3D);
	static void Multiply(FTransform* OutTransform, const FTransform* A, const FTransform* B);

	static void MultiplyUsingMatrixWithScale(FTransform* OutTransform, const FTransform* A, const FTransform* B);
	static void ConstructTransformFromMatrixWithDesiredScale(const FMatrix& AMatrix, const FMatrix& BMatrix, const FVector& DesiredScale, FTransform& OutTransform);

	FMatrix ToMatrixWithScale() const;

	FTransform operator*(const FTransform& A);

	static FVector GetSafeScaleReciprocal(const FVector& InScale, float Tolerance = SMALL_NUMBER);
	FTransform GetRelativeTransform(const FTransform& Other) const;

	static void GetRelativeTransformUsingMatrixWithScale(FTransform* OutTransform, const FTransform* Base, const FTransform* Relative);
};

static_assert(sizeof(FTransform) == 0x30, "FTransform");
static_assert(offsetof(FTransform, Scale3D) == 0x20, "FTransform");