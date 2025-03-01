#include "ue4math.h"

#include "rotator.h"
#include "vector.h"
#include "quat.h"
#include "matrix.h"

FRotator::FRotator(const FQuat& q) {
	const float SingularitYTest = q.Z * q.X - q.W * q.Y;
	const float YawY = 2.0f * (q.W * q.Z + q.X * q.Y);
	const float YawX = (1.0f - 2.0f * (q.Y * q.Y + q.Z * q.Z));

	const float SINGULARITY_THRESHOLD = 0.4999995f;
	const float RAD_TO_DEG = (180.0f) / PI;

	if (SingularitYTest < -SINGULARITY_THRESHOLD) {
		Pitch = -90.0f;
		Yaw = atan2f(YawY, YawX) * RAD_TO_DEG;
		Roll = NormalizeAxis(-Yaw - (2.0f * atan2f(q.X, q.W) * RAD_TO_DEG));
	}
	else if (SingularitYTest > SINGULARITY_THRESHOLD) {
		Pitch = 90.0f;
		Yaw = atan2f(YawY, YawX) * RAD_TO_DEG;
		Roll = NormalizeAxis(Yaw - (2.0f * atan2f(q.X, q.W) * RAD_TO_DEG));
	}
	else {
		Pitch = asinf(2.0f * (SingularitYTest)) * RAD_TO_DEG;
		Yaw = atan2f(YawY, YawX) * RAD_TO_DEG;
		Roll = atan2f(-2.0f * (q.W * q.X + q.Y * q.Z), (1.0f - 2.0f * (q.X * q.X + q.Y * q.Y))) * RAD_TO_DEG;
	}
}

FQuat FRotator::GetQuaternion() const {
	const float DEG_TO_RAD = PI / (180.0f);
	const float RADS_DIVIDED_BY_2 = DEG_TO_RAD / 2.0f;
	float SP, SY, SR;
	float CP, CY, CR;

	const float PitchNoWinding = fmodf(Pitch, 360.0);
	const float YawNoWinding = fmodf(Yaw, 360.0);
	const float RollNoWinding = fmodf(Roll, 360.0);

	SP = sinf(PitchNoWinding * RADS_DIVIDED_BY_2);
	CP = cosf(PitchNoWinding * RADS_DIVIDED_BY_2);
	SY = sinf(YawNoWinding * RADS_DIVIDED_BY_2);
	CY = cosf(YawNoWinding * RADS_DIVIDED_BY_2);
	SR = sinf(RollNoWinding * RADS_DIVIDED_BY_2);
	CR = cosf(RollNoWinding * RADS_DIVIDED_BY_2);

	FQuat RotationQuat;
	RotationQuat.X = CR * SP * SY - SR * CP * CY;
	RotationQuat.Y = -CR * SP * CY - SR * CP * SY;
	RotationQuat.Z = CR * CP * SY - SR * SP * CY;
	RotationQuat.W = CR * CP * CY + SR * SP * SY;
	return RotationQuat;
}

FRotator::operator FQuat() const {
	return GetQuaternion();
}

FMatrix FRotator::GetMatrix(FVector origin) const {
	float radPitch = ConvertToRadians(Pitch);
	float radYaw = ConvertToRadians(Yaw);
	float radRoll = ConvertToRadians(Roll);

	float SP = sinf(radPitch);
	float CP = cosf(radPitch);
	float SY = sinf(radYaw);
	float CY = cosf(radYaw);
	float SR = sinf(radRoll);
	float CR = cosf(radRoll);

	FMatrix matriX;
	matriX.M[0][0] = CP * CY;
	matriX.M[0][1] = CP * SY;
	matriX.M[0][2] = SP;
	matriX.M[0][3] = 0.0;

	matriX.M[1][0] = SR * SP * CY - CR * SY;
	matriX.M[1][1] = SR * SP * SY + CR * CY;
	matriX.M[1][2] = -SR * CP;
	matriX.M[1][3] = 0.0;

	matriX.M[2][0] = -(CR * SP * CY + SR * SY);
	matriX.M[2][1] = CY * SR - CR * SP * SY;
	matriX.M[2][2] = CR * CP;
	matriX.M[2][3] = 0.0;

	matriX.M[3][0] = origin.X;
	matriX.M[3][1] = origin.Y;
	matriX.M[3][2] = origin.Z;
	matriX.M[3][3] = 1.0;

	return matriX;
}

FVector FRotator::GetUnitVector() const {
	float radPitch = ConvertToRadians(Pitch);
	float radYaw = ConvertToRadians(Yaw);

	float SP = sinf(radPitch);
	float CP = cosf(radPitch);
	float SY = sinf(radYaw);
	float CY = cosf(radYaw);

	return FVector(CP * CY, CP * SY, SP);
}


