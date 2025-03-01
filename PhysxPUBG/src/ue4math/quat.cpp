#include "quat.h"
#include "vector.h"
#include "matrix.h"

FVector FQuat::RotateVector(const FVector& V) const
{
	// http://people.csail.mit.edu/bkph/articles/Quaternions.pdf
	// V' = V + 2w(Q x V) + (2Q x (Q x V))
	// refactor:
	// V' = V + w(2(Q x V)) + (Q x (2(Q x V)))
	// T = 2(Q x V);
	// V' = V + w*(T) + (Q x T)

	const FVector Q(X, Y, Z);
	const FVector T = (Q ^ V) * 2.0;
	const FVector Result = V + (T * W) + (Q ^ T);
	return Result;
}

FVector FQuat::operator*(const FVector& V) const { return RotateVector(V); }

FQuat::FQuat(const FMatrix& M) {
	// If Matrix is NULL, return Identity quaternion. If any of them is 0, you won't be able to construct rotation
	// if you have two plane at least, we can reconstruct the frame using cross product, but that's a bit expensive op to do here
	// for now, if you convert to matrix from 0 scale and convert back, you'll lose rotation. Don't do that. 
	if (M.GetScaledAxisX().IsNearlyZero() || M.GetScaledAxisY().IsNearlyZero() || M.GetScaledAxisZ().IsNearlyZero())
	{
		*this = FQuat();
		return;
	}

	//const MeReal *const t = (MeReal *) tm;
	float s;

	// Check diagonal (trace)
	const float tr = M.M[0][0] + M.M[1][1] + M.M[2][2];

	if (tr > 0)
	{
		float InvS = InvSqrt(tr + 1.f);
		this->W = 0.5f * (1.f / InvS);
		s = 0.5f * InvS;

		this->X = (M.M[1][2] - M.M[2][1]) * s;
		this->Y = (M.M[2][0] - M.M[0][2]) * s;
		this->Z = (M.M[0][1] - M.M[1][0]) * s;
	}
	else
	{
		// diagonal is negative
		int i = 0;

		if (M.M[1][1] > M.M[0][0])
			i = 1;

		if (M.M[2][2] > M.M[i][i])
			i = 2;

		const int nxt[3] = { 1, 2, 0 };
		const int j = nxt[i];
		const int k = nxt[j];

		s = M.M[i][i] - M.M[j][j] - M.M[k][k] + 1.0f;

		float InvS = InvSqrt(s);

		float qt[4];
		qt[i] = 0.5f * (1.f / InvS);

		s = 0.5f * InvS;

		qt[3] = (M.M[j][k] - M.M[k][j]) * s;
		qt[j] = (M.M[i][j] + M.M[j][i]) * s;
		qt[k] = (M.M[i][k] + M.M[k][i]) * s;

		this->X = qt[0];
		this->Y = qt[1];
		this->Z = qt[2];
		this->W = qt[3];
	}
}

FVector FQuat::rotateInv(const FVector& v) const
{
	float vx = 2.0f * v.X;
	float vy = 2.0f * v.Y;
	float vz = 2.0f * v.Z;
	float w2 = W * W - 0.5f;
	float dot2 = (X * vx + Y * vy + Z * vz);
	return FVector((vx * w2 - (Y * vz - Z * vy) * W + X * dot2), (vy * w2 - (Z * vx - X * vz) * W + Y * dot2),
		(vz * w2 - (X * vy - Y * vx) * W + Z * dot2));
}