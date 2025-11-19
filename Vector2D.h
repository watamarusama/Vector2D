#pragma once
#include <cmath>

typedef struct tagVector2D
{
	double x;
	double y;

	tagVector2D() : x(0.0), y(0.0) {}
	tagVector2D(double _x, double _y) : x(_x), y(_y) {}
}Vector2D;

struct Segment {
	Vector2D s; //始点
	Vector2D v; //終点
};

//ベクター+ベクター
Vector2D operator+ (const Vector2D & v1, const Vector2D & v2)
{
	return Vector2D(v1.x + v2.x, v1.y + v2.y);
}

//ベクター-ベクター
Vector2D operator- (const Vector2D& v1, const Vector2D& v2)
{
	return Vector2D(v1.x - v2.x, v1.y - v2.y);
}

//ベクター*スカラー(float)
Vector2D operator* (const Vector2D &v, float scalar)
{
	return Vector2D(v.x * scalar, v.y * scalar);
}
//ベクター*スカラー(double)
Vector2D operator* (const Vector2D& v, double scalar)
{
	return Vector2D(v.x * scalar, v.y * scalar);
}
//スカラー(float)*ベクター
Vector2D operator* (float scalar , const Vector2D& v)
{
	return Vector2D(v.x * scalar, v.y * scalar);
}
//スカラー(double)*ベクター
Vector2D operator* (double scalar, const Vector2D& v)
{
	return Vector2D(v.x * scalar, v.y * scalar);
}

//2Dベクトルの外積
inline float Vector2Cross(const Vector2D* v1, const Vector2D* v2)
{
	return (float)v1->x * v2->y - v1->y * v2->x;
}

//2Dベクトルの内積
inline float Vector2Dot(const Vector2D& v1, const Vector2D& v2)
{
	return (float)v1.x * v2.x + v1.y * v2.y;
}

bool ColSegments(Segment& seg1, Segment& seg2, float* outT1 = 0, float* outT2 = 0, Vector2D* outPos = 0)
{
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

bool Line_vs_Box(Segment& line, Vector2D& box, double size, float* outT = 0, Vector2D* outPos = 0)
{
	//矩形の4点の位置をベクトル表現
	Vector2D P_top_left = box;
	Vector2D P_top_right = Vector2D(box.x + size, box.y);
	Vector2D P_bottom_right = Vector2D(box.x + size, box.y + size);
	Vector2D P_bottom_left = Vector2D(box.x, box.y + size);
	
	//矩形の4辺をセグメント表現
	Segment box_top = { P_top_left, P_top_right };
	Segment box_right = {P_top_right, P_bottom_right};
	Segment box_bottom = { P_bottom_right, P_bottom_left };
	Segment box_left = { P_bottom_left, P_top_left };

	//線分と矩形の各辺との当たり判定
	if (ColSegments(line, box_top, outT, nullptr, outPos) ||
		ColSegments(line, box_right, outT, nullptr, outPos) ||
		ColSegments(line, box_bottom, outT, nullptr, outPos) ||
		ColSegments(line, box_left, outT, nullptr, outPos))
	{
		return true;
	}

	return false;
}