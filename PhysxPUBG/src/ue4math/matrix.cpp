#include "matrix.h"
#include "vector.h"
#include "rotator.h"
#include "transform.h"

FVector FMatrix::GetScaledAxisX() const { return FVector(M[0][0], M[0][1], M[0][2]); }
FVector FMatrix::GetScaledAxisY() const { return FVector(M[1][0], M[1][1], M[1][2]); }
FVector FMatrix::GetScaledAxisZ() const { return FVector(M[2][0], M[2][1], M[2][2]); }

FVector FMatrix::GetOrigin() const { return FVector(_41, _42, _43); }

FRotator FMatrix::GetRotator() const {
    const FVector XAxis = GetScaledAxisX();
    const FVector YAxis = GetScaledAxisY();
    const FVector ZAxis = GetScaledAxisZ();

    FRotator r = FRotator(
        atan2f(XAxis.Z, sqrtf(XAxis.X * XAxis.X + XAxis.Y * XAxis.Y)) * 180.0f / PI,
        atan2f(XAxis.Y, XAxis.X) * 180.0f / PI,
        0
    );

    const FVector SYAxis = GetScaledAxisY();

    r.Roll = atan2f(ZAxis | SYAxis, YAxis | SYAxis) * 180.0f / PI;

    return r;
}

FMatrix& FMatrix::operator=(const FTransform& t) { return *this = FTransform(t).ToMatrixWithScale(); }
FMatrix::FMatrix(const FTransform& t) { operator=(t); }

void FMatrix::SetAxis0(const FVector& Axis)
{
    M[0][0] = Axis.X;
    M[0][1] = Axis.Y;
    M[0][2] = Axis.Z;
}

void FMatrix::SetAxis1(const FVector& Axis)
{
    M[1][0] = Axis.X;
    M[1][1] = Axis.Y;
    M[1][2] = Axis.Z;
}

void FMatrix::SetAxis2(const FVector& Axis)
{
    M[2][0] = Axis.X;
    M[2][1] = Axis.Y;
    M[2][2] = Axis.Z;
}

FMatrix FMatrix::Inverse() const
{
    FMatrix Result;

    // Check for zero scale matrix to invert
    if (GetScaledAxisX().IsNearlyZero(SMALL_NUMBER) &&
        GetScaledAxisY().IsNearlyZero(SMALL_NUMBER) &&
        GetScaledAxisZ().IsNearlyZero(SMALL_NUMBER))
    {
        // just set to zero - avoids unsafe inverse of zero and duplicates what QNANs were resulting in before (scaling away all children)
        Result = FMatrix();
    }
    else
    {
        const float	Det = Determinant();

        if (Det == 0.0f)
        {
            Result = FMatrix();
        }
        else
        {
            VectorMatrixInverse(&Result, this);
        }
    }

    return Result;
}