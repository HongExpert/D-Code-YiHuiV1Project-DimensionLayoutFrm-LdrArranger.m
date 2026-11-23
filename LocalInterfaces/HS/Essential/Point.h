#pragma once
#include "LdrArranger.h"
#include <cmath>
#include <limits>
#include <algorithm>
#include <array>
#include <map>
#include <vector>

// Preserve the upstream algorithm; isolate its types and exports in HS for the CAA module. [Rectified/Added by Leo 2026/11/19]
namespace HS
{
class ExportedByLdrArranger Point;
using Vector2d = Point;
const double ZERO = 1e-3;
const double Pi = 3.1415926;
const double Nan = std::numeric_limits<double>::quiet_NaN();

	enum Region
	{
		Quadrant1,
		Quadrant2,
		Quadrant3,
		Quadrant4,

		XAxisP,
		XAxisN,
		YAxisP,
		YAxisN,

		Center,
		Unknown
	};

	typedef enum LineRelation
	{
		UnknownRelation,
		Touch,
		Link,
		CollinearAdjacent,
		CollinearOverlap,
		CollinearSeparate,
		Separate,

		Intersect	
	}LR;

	typedef enum PointRelationToBBox
	{
		inward,
		outward,
		onedge,
		unkonwn
	}PRB;

	enum Orient
	{
		Left,
		Up,
		Right,
		Down,
		Other
	};

	typedef enum SearchingOrient
	{
		UL,
		UR,
		LU,
		LD,
		DL,
		DR,
		RU,
		RD,
		NO,
	}SrOrient;
class ExportedByLdrArranger Point
{
public:
	Point(
		double x,
		double y);

	Point(
		const Point& other);

	Point();

	~Point();

	double X() const;

	double& sX();

	double Y() const;

	double& sY();

	Vector2d Norm();				 //convert the  Vector2d to unitVector

	Vector2d GetPerpendicular(int type) const;

	bool ParallelTo(const Vector2d& other) const;

	bool PerpendicularTo(const Vector2d& other) const;

	static double Distance(
		const Point& PointA,
		const Point& PointB);

	static double Angle(
		const Vector2d& v1,
		const Vector2d& v2);

	double Len() const;

	double operator*(				//dot product
		const Point& other) const;

	Point operator/(
		double value) const;

	Point operator*(
		double value) const;

	double operator^(				//cross product
		const Point& other) const;

	Point operator+(                 //it will create a new instance 
		const Point& other) const;

	void operator+=(
		const Point& other);

	Point operator-(
		const Point& other) const;

	Point operator-() const;

	void operator-=(
		const Point& other);

	bool operator==(
		const Point& other) const;

	bool valid() const;

private:
	double _x, _y;
};
} // namespace HS
