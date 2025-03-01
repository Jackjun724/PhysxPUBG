#pragma once
#include "ue4math.h"

struct FRotator;

// ScriptStruct CoreUObject.Vector
// 0x000C
struct FVector
{
public:
	float                                              X;                                                        // 0x0000(0x0004) (CPF_Edit, CPF_BlueprintVisible, CPF_ZeroConstructor, CPF_SaveGame, CPF_IsPlainOldData)
	float                                              Y;                                                        // 0x0004(0x0004) (CPF_Edit, CPF_BlueprintVisible, CPF_ZeroConstructor, CPF_SaveGame, CPF_IsPlainOldData)
	float                                              Z;                                                        // 0x0008(0x0004) (CPF_Edit, CPF_BlueprintVisible, CPF_ZeroConstructor, CPF_SaveGame, CPF_IsPlainOldData)

	FVector() : X(0.0), Y(0.0), Z(0.0) {}
	FVector(float X, float Y, float Z) :X(X), Y(Y), Z(Z) {}

	float DotProduct(const FVector& v) const {
		return (X * v.X) + (Y * v.Y) + (Z * v.Z);
	}

	FVector CrossProduct(const FVector& v) const {
		FVector output;
		output.X = (Y * v.Z) - (Z * v.Y);
		output.Y = (Z * v.X) - (X * v.Z);
		output.Z = (X * v.Y) - (Y * v.X);
		return output;
	}

	FVector Min(const FVector& v) const {
		FVector output;
		output.X = X < v.X ? X : v.X;
		output.Y = Y < v.Y ? Y : v.Y;
		output.Z = Z < v.Z ? Z : v.Z;
		return output;
	}

	FVector Max(const FVector& v) const {
		FVector output;
		output.X = X > v.X ? X : v.X;
		output.Y = Y > v.Y ? Y : v.Y;
		output.Z = Z > v.Z ? Z : v.Z;
		return output;
	}

	bool operator == (const FVector& v) const {
		return X == v.X && Y == v.Y && Z == v.Z;
	}

	bool operator != (const FVector& v) const {
		return !(*this == v);
	}

	FVector& operator*=(float s) {
		X *= s;
		Y *= s;
		Z *= s;
		return *this;
	}

	FVector operator - () const {
		return FVector(-X, -Y, -Z);
	}

	FVector operator + (const FVector& v) const {
		return FVector(X + v.X, Y + v.Y, Z + v.Z);
	}

	FVector operator - (const FVector& v) const {
		return FVector(X - v.X, Y - v.Y, Z - v.Z);
	}

	FVector operator * (const FVector& v) const {
		return FVector(X * v.X, Y * v.Y, Z * v.Z);
	}

	FVector operator * (float Value) const {
		return FVector(X * Value, Y * Value, Z * Value);
	}

	FVector GetNormalizedVector() const {
		return operator*(1.0f / sqrtf(X * X + Y * Y + Z * Z));
	}

	void Normalize() {
		*this = GetNormalizedVector();
	}

	float Length() const {
		return sqrtf(X * X + Y * Y + Z * Z);
	}

	float Distance(const FVector& v) const {
		return (v - *this).Length();
	}

	FVector operator ^ (const FVector& v) const {
		return CrossProduct(v);
	}

	float operator | (const FVector& v) const {
		return DotProduct(v);
	}

	FVector GetSignVector() const
	{
		return FVector
		(
			Select(X, 1.0, -1.0),
			Select(Y, 1.0, -1.0),
			Select(Z, 1.0, -1.0)
		);
	}

	bool IsNearlyZero(float Tolerance = KINDA_SMALL_NUMBER) const {
		return fabs(X) <= Tolerance && fabs(Y) <= Tolerance && fabs(Z) <= Tolerance;
	}

	FRotator GetDirectionRotator() const;
};

static FVector operator * (float Value, const FVector& v) {
	return v.operator*(Value);
}

static_assert(sizeof(FVector) == 0xC, "FVector");
class FVector2D
{
public:

	float                                              X;                                                         // 0x0000(0x0004) (Edit, BlueprintVisible, ZeroConstructor, SaveGame, IsPlainOldData, NoDestructor, HasGetValueTypeHash, NativeAccessSpecifierPublic)
	float                                              Y;                                                         // 0x0004(0x0004) (Edit, BlueprintVisible, ZeroConstructor, SaveGame, IsPlainOldData, NoDestructor, HasGetValueTypeHash, NativeAccessSpecifierPublic)

	inline FVector2D() : X(0), Y(0) {}

	inline FVector2D(float x, float y) : X(x), Y(y) {}
	inline bool Zero() const
	{
		return (X > -0.1f && X < 0.1f && Y > -0.1f && Y < 0.1f);
	}

	inline float Distance(const FVector2D& V1, const FVector2D& V2) const
	{
		return sqrtf(powf(V2.X - V1.X, 2) + powf(V2.Y - V1.Y, 2));
	}

	inline float DistanceTo(const FVector2D& V) const
	{
		return Distance(*this, V);
	}

	inline FVector2D operator + (const FVector2D& other) const { return FVector2D(X + other.X, Y + other.Y); }

	inline FVector2D operator - (const FVector2D& other) const { return FVector2D(X - other.X, Y - other.Y); }

	inline FVector2D operator * (float scalar) const { return FVector2D(X * scalar, Y * scalar); }

	inline FVector2D operator * (const FVector2D& other) const { return FVector2D(X * other.X, Y * other.Y); }

	inline FVector2D operator / (float scalar) const { return FVector2D(X / scalar, Y / scalar); }

	inline FVector2D operator / (const FVector2D& other) const { return FVector2D(X / other.X, Y / other.Y); }

	inline FVector2D& operator=  (const FVector2D& other) { X = other.X; Y = other.Y; return *this; }

	inline FVector2D& operator+= (const FVector2D& other) { X += other.X; Y += other.Y; return *this; }

	inline FVector2D& operator-= (const FVector2D& other) { X -= other.X; Y -= other.Y; return *this; }

	inline FVector2D& operator*= (const float other) { X *= other; Y *= other; return *this; }

	
};