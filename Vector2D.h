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
inline Vector2D operator+ (const Vector2D & v1, const Vector2D & v2)
{
	return Vector2D(v1.x + v2.x, v1.y + v2.y);
}

//ベクター-ベクター
inline Vector2D operator- (const Vector2D& v1, const Vector2D& v2)
{
	return Vector2D(v1.x - v2.x, v1.y - v2.y);
}

//ベクター*スカラー(float)
inline Vector2D operator* (const Vector2D &v, float scalar)
{
	return Vector2D(v.x * scalar, v.y * scalar);
}
//ベクター*スカラー(double)
inline Vector2D operator* (const Vector2D& v, double scalar)
{
	return Vector2D(v.x * scalar, v.y * scalar);
}
//スカラー(float)*ベクター
inline Vector2D operator* (float scalar , const Vector2D& v)
{
	return Vector2D(v.x * scalar, v.y * scalar);
}
//スカラー(double)*ベクター
inline Vector2D operator* (double scalar, const Vector2D& v)
{
	return Vector2D(v.x * scalar, v.y * scalar);
}

//2Dベクトルの外積
inline double Vector2Cross(const Vector2D* v1, const Vector2D* v2)
{
	return v1->x * v2->y - v1->y * v2->x;
}

//2Dベクトルの内積
inline double Vector2Dot(const Vector2D& v1, const Vector2D& v2)
{
	return v1.x * v2.x + v1.y * v2.y;
}

inline bool ColSegments(Segment& seg1, Segment& seg2, double* outT1 = 0, double* outT2 = 0, Vector2D* outPos = 0)
{
	Vector2D w = seg2.s - seg1.s;
	double Crs_v1_v2 = Vector2Cross(&seg1.v, &seg2.v);
	if (Crs_v1_v2 == 0.0F) {
		return false;
	}

	double Crs_v_v1 = Vector2Cross(&w, &seg1.v);
	double Crs_v_v2 = Vector2Cross(&w, &seg2.v);

	double t1 = Crs_v_v2 / Crs_v1_v2;
	double t2 = Crs_v_v1 / Crs_v1_v2;

	if (outT1)
		*outT1 = t1;
	if (outT2)
		*outT2 = t2;

	const double eps = 0.00001f;
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

inline bool Line_vs_Box(Segment& line, Vector2D& box, double size, double* outT = 0, Vector2D* outPos = 0)
{
	//矩形の4点の位置をベクトル表現
	Vector2D P_top_left = box;
	Vector2D P_top_right = { box.x + size, box.y };
	Vector2D P_bottom_right = { box.x + size, box.y + size };
	Vector2D P_bottom_left = { box.x, box.y + size };
	
	//矩形の4辺をセグメント表現
	Segment box_top = { P_top_left, P_top_right - P_top_left };
	Segment box_right = {P_top_right, P_bottom_right - P_top_right};
	Segment box_bottom = { P_bottom_right, P_bottom_left - P_bottom_right };
	Segment box_left = { P_bottom_left, P_top_left - P_bottom_left };

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

//線分と円形
inline bool Line_vs_Circle(Segment& line, Vector2D& circle, double size, double* outT = 0, Vector2D* outPos = 0)
{
	//矩形の4点の位置をベクトル表現
	Vector2D P_left = { circle.x - size, circle.y };
	Vector2D P_right = { circle.x + size, circle.y };
	Vector2D P_top = { circle.x, circle.y - size };
	Vector2D P_bottom = { circle.x, circle.y + size };

	//矩形の4辺をセグメント表現
	Segment box_top = { circle, P_top - circle };
	Segment box_right = { circle, P_right - circle };
	Segment box_bottom = { circle, P_bottom - circle };
	Segment box_left = { circle, P_left - circle };

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

//矩形と矩形
inline bool Box_vs_Box(Vector2D& box1, double size1, Vector2D& box2, double size2)
{
	if (box1.x + size1 < box2.x) return false; //box1がbox2の左側
	if (box2.x + size2 < box1.x) return false; //box1がbox2の右側
	if (box1.y + size1 < box2.y) return false; //box1がbox2の上側
	if (box2.y + size2 < box1.y) return false; //box1がbox2の下側
	return true;
}

//円形と円形
inline bool Circle_vs_Circle(Vector2D& circle1, double size1, Vector2D& circle2, double size2)
{
	double distX = circle1.x - circle2.x;
	double distY = circle1.y - circle2.y;
	double radiiSum = size1 + size2;
	return (distX * distX + distY * distY) <= (radiiSum * radiiSum);
}

//矩形と円形
inline bool Box_vs_Circle(Vector2D& box, double boxSize, Vector2D& circle, double circleSize)
{
	//矩形の中心点を求める
	Vector2D boxCenter = { box.x + boxSize / 2.0, box.y + boxSize / 2.0 };
	//矩形の中心点と円の中心点の距離を求める
	double distX = std::abs(circle.x - boxCenter.x);
	double distY = std::abs(circle.y - boxCenter.y);
	//矩形の半分のサイズ
	double halfBoxSize = boxSize / 2.0;
	//距離が矩形の半分のサイズを超えている場合、衝突していない
	if (distX > (halfBoxSize + circleSize)) return false;
	if (distY > (halfBoxSize + circleSize)) return false;
	//距離が矩形の半分のサイズ以内の場合、衝突している
	if (distX <= halfBoxSize) return true;
	if (distY <= halfBoxSize) return true;
	//コーナー部分での衝突判定
	double cornerDistSq = (distX - halfBoxSize) * (distX - halfBoxSize) +
		(distY - halfBoxSize) * (distY - halfBoxSize);
	return cornerDistSq <= (circleSize * circleSize);
}

//円形と矩形
inline bool Circle_vs_Box(Vector2D& circle, double circleSize, Vector2D& box, double boxSize)
{
	//矩形の中心点を求める
	Vector2D boxCenter = { box.x + boxSize / 2.0, box.y + boxSize / 2.0 };
	//矩形の中心点と円の中心点の距離を求める
	double distX = std::abs(circle.x - boxCenter.x);
	double distY = std::abs(circle.y - boxCenter.y);
	//矩形の半分のサイズ
	double halfBoxSize = boxSize / 2.0;
	//距離が矩形の半分のサイズを超えている場合、衝突していない
	if (distX > (halfBoxSize + circleSize)) return false;
	if (distY > (halfBoxSize + circleSize)) return false;
	//距離が矩形の半分のサイズ以内の場合、衝突している
	if (distX <= halfBoxSize) return true;
	if (distY <= halfBoxSize) return true;
	//コーナー部分での衝突判定
	double cornerDistSq = (distX - halfBoxSize) * (distX - halfBoxSize) +
		(distY - halfBoxSize) * (distY - halfBoxSize);
	return cornerDistSq <= (circleSize * circleSize);
}