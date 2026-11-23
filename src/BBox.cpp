#include "HS/Essential/BBox.h"

// Preserve the upstream algorithm; isolate its types and exports in HS for the CAA module. [Rectified/Added by Leo 2026/11/19]
namespace HS
{
BBox::BBox()
	:_valid(false)
{
	Initialize();
}

BBox::BBox(Point* vertices)
{
	Initialize(vertices);
}

BBox::BBox(const BBox& other)
{
	std::array<Point, 4> vertices;
	other.GetVertices(vertices);
	Initialize(vertices);
}

void BBox::GetVertices(
	std::array<Point, 4>& vertices) const
{
	for (size_t i = 0; i < sizeof(_vertices) / sizeof(_vertices[0]); i++)
	{
		vertices[i] = _vertices[i];
	}
}

Point BBox::GetCenter() const
{
	return (_vertices[0]+std::get<0>(_param[0]) * std::get<1>(_param[0])) / 2 + 
		(_vertices[0] + std::get<0>(_param[1]) * std::get<1>(_param[1])) / 2;
}

bool BBox::Intersect(const BBox& other) const
{
	if (!_valid || !other._valid)
		return false;

	Vector2d axes[4] = {
		std::get<0>(_param[0]), std::get<0>(_param[1]),
		std::get<0>(other._param[0]), std::get<0>(other._param[1])
	};

	for (size_t k = 0; k < 4; k++)
	{
		double minA = 1e300, maxA = -1e300;
		double minB = 1e300, maxB = -1e300;

		for (size_t i = 0; i < sizeof(_vertices) / sizeof(_vertices[0]); i++)
		{
			double d = _vertices[i] * axes[k];
			minA = (std::min)(minA, d);
			maxA = (std::max)(maxA, d);
		}

		for (size_t i = 0; i < sizeof(other._vertices) / sizeof(other._vertices[0]); i++)
		{
			double d = other._vertices[i] * axes[k];
			minB = (std::min)(minB, d);
			maxB = (std::max)(maxB, d);
		}

		if (maxA < minB - ZERO || maxB < minA - ZERO)
			return false;
	}

	return true;
}

HS::Region BBox::CalPointRegion(const Point& point) const
{
	if (!_valid || !point.valid())
		return HS::Unknown;

	// The first box edge is local Y and the second parameter is local X. Keep the existing quadrant IDs used by corner ordering. [Rectified/Added by Leo_GPT 2026/11/19]
	const Vector2d offset = point - GetCenter();
	const double x = offset * std::get<0>(_param[1]);
	const double y = offset * std::get<0>(_param[0]);
	if (!std::isfinite(x) || !std::isfinite(y))
		return HS::Unknown;

	const bool onXAxis = std::abs(y) <= ZERO;
	const bool onYAxis = std::abs(x) <= ZERO;
	if (onXAxis && onYAxis)
		return HS::Center;
	if (onXAxis)
		return x > 0.0 ? HS::XAxisP : HS::XAxisN;
	if (onYAxis)
		return y > 0.0 ? HS::YAxisP : HS::YAxisN;
	if (x > 0.0)
		return y > 0.0 ? HS::Quadrant1 : HS::Quadrant2;
	return y > 0.0 ? HS::Quadrant3 : HS::Quadrant4;
}

bool BBox::IntersectWithLine(const Line& line) const
{
	if (!_valid || !line.Valid() || line._len < ZERO)
		return false;

	/*detect the case where the line segments are collinear or have 
	intersected with one edge of BBox.*/
	HS::LR curRelation = HS::UnknownRelation;
	for (size_t i = 0; i < 4; i++) 
	{
		curRelation = Line::CalLineRelation(_edges[i], line);
		switch (curRelation)
		{
		case HS::CollinearAdjacent:			
		case HS::CollinearOverlap:
		case HS::CollinearSeparate:
			return false;

		case HS::Intersect:					
			return true;
		}
	}

	/*when the line traverses exactly the diagonal vertices of BBox.
	if 2 vertices exist on the line, it indicates the line has traversed the diagonal vertices.*/
	//size_t onLineVertices = 0;		
	std::vector<Point> onLineVertices;
	for (auto vertex : _vertices)
	{
		if (line.PointOnLine(vertex))		
			onLineVertices.push_back(vertex);
	}

	if (onLineVertices.size() > 1)
		return true;

	/*detect whether the line segments are encapsulated in the BBox or not*/
	std::map<HS::PRB, size_t> pointRelationCount;
	std::vector<Point> verticesOnEdge;
	for (auto point : line.GetEndPoints())
	{
		HS::PRB curRelation = CalPointRelation(point);
		if (curRelation == HS::onedge)
			verticesOnEdge.push_back(point);

		pointRelationCount[curRelation]++;
	}

	if (pointRelationCount[HS::onedge] == 2 || pointRelationCount[HS::inward] > 0)
		return true;

	/*the Point is located in the corner and another is outside of box*/
	if (onLineVertices.size() == 1 && pointRelationCount[HS::onedge] == 1)
		return Point::Distance(onLineVertices[0], verticesOnEdge[0]) > ZERO;
	
	return false;
}

HS::PRB BBox::CalPointRelation(const Point& point) const
{
	if (!_valid || !point.valid())
		return HS::unkonwn;

	double refDots[2] = { 
		(point - _vertices[0])* std::get<0>(_param[0]), 
		(point - _vertices[0])* std::get<0>(_param[1]) };

	if (refDots[0] < -ZERO || refDots[1] < -ZERO)
		return HS::outward;

	for (size_t i = 0; i < 2; i++)
	{
		if ((fabs(refDots[i]) < ZERO || fabs(refDots[i] - std::get<1>(_param[i])) < ZERO) &&
			(refDots[1-i] > -ZERO && refDots[1 - i] < std::get<1>(_param[1 - i]) + ZERO))
			return HS::onedge;
	}

	if (refDots[0] < std::get<1>(_param[0]) &&
		refDots[1] < std::get<1>(_param[1]))
		return HS::inward;

	return HS::outward;
}

const std::tuple<Vector2d, double>* BBox::AccessBoxParam() const
{
	return _param;
}

Point BBox::FindCornerForBox(const Vector2d& dir) const
{
	if (!Valid())
		return Point();

	double maxDist = 0, curDist = 0;
	bool Start = false;
	size_t index = 0;
	for (size_t i = 0; i < 4; i++)
	{
		curDist = dir * _vertices[i];
		if (!Start || curDist > maxDist)
		{
			Start = true;
			maxDist = curDist;
			index = i;
		}
	}

	return _vertices[index];
}

bool BBox::Move(const Vector2d& offset)
{
	if (!_valid || !offset.valid() ||
		!std::isfinite(offset.X()) || !std::isfinite(offset.Y()))
		return false;

	if (offset.X() == 0.0 && offset.Y() == 0.0)
		return true;

	for (auto& vertex : _vertices)
	{
		vertex += offset;
	}

	for (auto& edge : _edges)
	{
		edge.Move(offset);
	}

	return true;
}


bool BBox::Valid() const
{
	return _valid;
}

BBox BBox::operator+(const Point& point)
{
	if (!_valid)
		return BBox();

	Vector2d offSetDirs[4] = 
	{	std::get<0>(_param[0]) * std::get<1>(_param[0]),
		Vector2d(0,0),
		std::get<0>(_param[1]) * std::get<1>(_param[1]),
		Vector2d(0,0)
	};//up:0 down:1 right:2 left:3

	Point vertices[4];

	for (size_t i = 0; i < sizeof(_param) / sizeof(_param[0]); i++)
	{
		double dot = (point - _vertices[0]) * std::get<0>(_param[i]);
		if (dot > std::get<1>(_param[i]))
			offSetDirs[i * 2] = std::get<0>(_param[i]) * dot;

		if (dot < 0.)
			offSetDirs[2 * i + 1] = std::get<0>(_param[i]) * dot;
	}

	vertices[0] = _vertices[0] + (offSetDirs[1] + offSetDirs[3]);
	vertices[1] = _vertices[0] + (offSetDirs[0] + offSetDirs[3]);
	vertices[2] = _vertices[0] + (offSetDirs[2] + offSetDirs[0]);
	vertices[3] = _vertices[0] + (offSetDirs[2] + offSetDirs[1]);

	return BBox(vertices);
}

BBox BBox::operator+(const BBox& other)
{
	BBox result = *this;

	if (!_valid || !other._valid)
		return result;

	for (auto vertex : other._vertices)
	{
		result = result + vertex;
	}

	return result;
}

BBox& BBox::operator+=(const BBox& other)
{
	*this = *this + other;
	return *this;
}

void BBox::Initialize()
{
	std::fill_n(_vertices, 4, Point());
}

void BBox::Initialize(Point* vertices)
{
	_valid = true;

	for (size_t i = 0; i < sizeof(_vertices) / sizeof(_vertices[0]); i++)
	{
		if (!(vertices + i)->valid())
			_valid = false;
		_vertices[i] = *(vertices + i);
	}
	CalBoxParams();
}

void BBox::Initialize(std::array<Point, 4>& vertices)
{
	_valid = true;

	for (size_t i = 0; i < sizeof(_vertices) / sizeof(_vertices[0]); i++)
	{
		if (!vertices[i].valid())
			_valid = false;
		_vertices[i] = vertices[i];
	}
	CalBoxParams();
}

//std::map<HS::Orient,size_t > BBox::CalEdgeOrient()
//{
//	std::map<HS::Orient, size_t> edgeOrients;
//	if (!_valid)
//		return edgeOrients;
//
//	for (size_t i = 0; i < 2; i++)
//	{
//		for (auto edge:_edges)
//		{
//			if (edge._parallelNorm.PerpendicularTo(std::get<0>(_param[i])))
//			{
//				if (fabs((edge._endPoints[0] - _vertices[0])* std::get<0>(_param[i])) < std::get<1>(_param[i]) - ZERO &&
//					)
//
//			}
//		}
//	}
//
//
//
//}

void BBox::CalBoxParams()
{
	Vector2d vectors[3];
	double lens[3];

	size_t i = 1;
	Vector2d cur;
	for (; i < sizeof(_vertices) / sizeof(_vertices[0]); i++)
	{
		cur = _vertices[i] - _vertices[0];
		vectors[i - 1] = cur.Norm();
		lens[i - 1] = cur.Len(); 
	}

	bool end = false;
	for (i = 0; i < sizeof(vectors) / sizeof(vectors[0]); i++)
	{
		for (size_t j = 0; j < sizeof(vectors) / sizeof(vectors[0]); j++)
		{
			if (j == i)
				continue;

			if (fabs(Point::Angle(vectors[i], vectors[j]) - Pi / 2) < ZERO)
			{
				size_t oppositeIndex = [&]()->size_t {
					for (size_t k = 0; k < 3; k++)
					{
						if (k != i && k != j)
							return k + 1;
					}
					return SIZE_MAX;
					}();

				if (oppositeIndex == SIZE_MAX)
					return;

				_param[0] = std::make_tuple(vectors[i], lens[i]);
				_param[1] = std::make_tuple(vectors[j], lens[j]);

				_edges[0] = Line({ _vertices[0], _vertices[i + 1] });				//left
				_edges[1] = Line({ _vertices[i + 1], _vertices[oppositeIndex] });	//Up
				_edges[2] = Line({ _vertices[oppositeIndex], _vertices[j + 1] });	//Right
				_edges[3] = Line({ _vertices[j + 1], _vertices[0] });			    //Donw

				_area = lens[i] * lens[j];
				end = true;
				break;
			}
		}

		if (end)
			break;
	}

	if (!end)
		_valid = false;			//this bounding box is invalid
}
} // namespace HS
