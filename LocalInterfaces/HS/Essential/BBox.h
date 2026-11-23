#pragma once
#include "LdrArranger.h"
#include "Line.h"
#include <array>
#include <tuple>
#include <map>

// Preserve the upstream algorithm; isolate its types and exports in HS for the CAA module. [Rectified/Added by Leo 2026/11/19]
namespace HS
{
class ExportedByLdrArranger BBox
{
	friend class Arranger;
public:
	BBox();
	BBox(Point* vertices);
	BBox(const BBox& other);

	void GetVertices(
		std::array<Point, 4> &vertices) const;

	Point GetCenter() const;

	bool Intersect(const BBox& other) const;

	HS::Region CalPointRegion(const Point& point) const;

	bool IntersectWithLine(const Line& line) const;

	HS::PRB CalPointRelation(const Point& point) const;

	const std::tuple<Vector2d, double>* AccessBoxParam() const;

	Point FindCornerForBox(const Vector2d& dir) const;

	bool Move(const Vector2d& offset);

	bool Valid() const;

	BBox operator+(const Point& point);

	BBox operator+(const BBox& other);

	BBox& operator+=(const BBox& other);

private:
	void Initialize();
	void Initialize(Point* vertices);
	void Initialize(std::array<Point, 4>& vertices);

	//std::map<HS::Orient, size_t> CalEdgeOrient();

	void CalBoxParams();

private:
	Point _vertices[4];			//_vertices[0]: locating point.
	Line _edges[4];
	//2 norm vector2d along the BBox edge, and relevant length of this edge.
	std::tuple<Vector2d, double> _param[2];
	double _area;
	bool _valid;

};
} // namespace HS
