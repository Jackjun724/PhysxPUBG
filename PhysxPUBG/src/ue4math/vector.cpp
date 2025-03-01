#include "vector.h"
#include "rotator.h"

FRotator FVector::GetDirectionRotator() const {
	FRotator r;
	r.Pitch = ConvertToDegrees(atan2f(Z, sqrtf(X * X + Y * Y)));
	r.Yaw = ConvertToDegrees(atan2f(Y, X));
	r.Roll = 0.0;
	return r;
}