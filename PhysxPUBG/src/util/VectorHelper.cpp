#pragma once
#define NOMINMAX
#include "VectorHelper.h"

bool VectorHelper::IsInScreen(const FVector2D WorldToScreen) {
	if ((WorldToScreen.X < -100 || WorldToScreen.X > 1920 + 100 || WorldToScreen.Y < -100 || WorldToScreen.Y > 1080 + 100))
	{
		return false;
	}
	return true;
}

FVector2D VectorHelper::WorldToScreen(const FVector& WorldLocation, const FRotator& rotation, const FVector& location, float FOV) {
	FVector2D ScreenLocation;

	FMatrix RotationMatrix = rotation.GetMatrix();

	FVector AxisX = RotationMatrix.GetScaledAxisX();
	FVector AxisY = RotationMatrix.GetScaledAxisY();
	FVector AxisZ = RotationMatrix.GetScaledAxisZ();

	FVector vDelta(WorldLocation - location);
	FVector vTransformed(vDelta | AxisY, vDelta | AxisZ, vDelta | AxisX);

	if (vTransformed.Z == 0.0f)
		vTransformed.Z = -0.001f;

	auto VieW = vTransformed.Z;

	if (vTransformed.Z < 0.0f)
		vTransformed.Z = -vTransformed.Z;

	float ScreenCenterX = 1920 / 2.0f;
	float ScreenCenterY = 1080 / 2.0f;
	float TangentFOV = tanf(ConvertToRadians(FOV / 2.0f));

	ScreenLocation.X = (ScreenCenterX + vTransformed.X * (ScreenCenterX / TangentFOV) / vTransformed.Z);
	ScreenLocation.Y = (ScreenCenterY - vTransformed.Y * (ScreenCenterX / TangentFOV) / vTransformed.Z);

	if (VieW != INFINITY && VieW > 0.0f)
		return ScreenLocation;
	return FVector2D(INFINITY, INFINITY);
}