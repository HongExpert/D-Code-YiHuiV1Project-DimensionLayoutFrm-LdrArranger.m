#pragma once
#include "LdrArranger.h"
#include "Point.h"

// Preserve the upstream algorithm; isolate its types and exports in HS for the CAA module. [Rectified/Added by Leo 2026/11/19]
namespace HS
{
class ExportedByLdrArranger Line
{
	friend class BBox;
public:

	Line();
	Line(Point* endPoints);
	Line(const Point(&endPoints)[2]);
	Line(const Line& other);

	~Line();

	bool Valid() const;

	std::array<Point, 2> GetEndPoints() const;

	Vector2d& AccessParallelNorm();

	bool PointOnLine(const Point& point) const; 

	double CalPerpendicularDistanceToPoint(
		const Point& point) const;

	bool HasProjectedOn(const Line& other) const;

	bool HasProjectedOn(const Point& point) const;

	Point CalCoordForProjectedPoint(const Point& point) const;

	static HS::LR CalLineRelation(
		const Line& Line1, 
		const Line& Line2);

	double Len() const;

	void SetEndPoints(const Point(&endPoints)[2]);

	bool Move(const Vector2d& offset);

private:

	void Initialize(const Point(&endPoints)[2]);

private:
	Point _endPoints[2];
	Vector2d _parallelNorm;
	double _len;
	bool _valid;

};
} // namespace HS
