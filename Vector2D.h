#pragma once

typedef struct tagVector2D
{
	double x;
	double y;

	tagVector2D() : x(0.0), y(0.0) {}
	tagVector2D(double _x, double _y) : x(_x), y(_y) {}
}Vector2D;

struct Segment {
	Vector2D s;
	Vector2D v;
};

Vector2D operator+ (const Vector2D & v1, const Vector2D & v2)
{
	return Vector2D(v1.x + v2.x, v1.y + v2.y);
}

Vector2D operator- (const Vector2D& v1, const Vector2D& v2)
{
	return Vector2D(v1.x - v2.x, v1.y - v2.y);
}

template<typename T>
Vector2D operator* (const Vector2D &v,  T scalar)
{
	return Vector2D(v.x * scalar, v.y * scalar);
}

//2Dベクトルの外積
template<typename T>
T Vector2Cross(const Vector2D* v1, const Vector2D* v2)
{
	return v1->x * v2->y - v1->y * v2->x;
}

//2Dベクトルの内積
template<typename T>
T Vector2Dot(const Vector2D& v1, const Vector2D& v2)
{
	return v1.x * v2.x + v1.y * v2.y;
}

template<typename T>
bool ColSegments(Segment& seg1, Segment& seg2, float* outT1 = 0, float* outT2 = 0, Vector2D* outPos = 0) {
	Vector2D w = seg2.s - seg1.s;
	float Crs_v1_v2 = Vector2Cross(&seg1.v, &seg2.v);
	if (Crs_v1_v2 == 0.0F) {
		return false;
	}

	float Crs_v_v1 = Vector2Cross(&w, &seg1.v);
	float Crs_v_v2 = Vector2Cross(&w, &seg2.v);

	float t1 = Crs_v_v2 / Crs_v1_v2;
	float t2 = Crs_v_v1 / Crs_v1_v2;

	if (outT1)
		*outT1 = t1;
	if (outT2)
		*outT2 = t2;

	const float eps = 0.00001f;
	if (t1 >= -eps && t1 <= 1.0f + eps && t2 >= -eps && t2 <= 1.0f + eps)
	{
		if (outPos)
		{
			*outPos = seg1.s + seg1.v * t1;
		}
		return true;
	}
	return false;
}