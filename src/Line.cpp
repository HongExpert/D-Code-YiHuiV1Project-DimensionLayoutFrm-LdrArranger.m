#include "HS/Essential/Line.h"

// Preserve the upstream algorithm; isolate its types and exports in HS for the CAA module. [Rectified/Added by Leo 2026/11/19]
namespace HS
{
Line::Line()
	:_parallelNorm(0, 0), 
	_len(0),
	_valid(false)
{}

Line::Line(Point* endPoints)
	:_valid(true)
{
	Point endPnts[2] = {endPoints[0],endPoints[1]};
	Initialize(endPnts);
}

Line::Line(const Point(&endPoints)[2])
	:_valid(true)
{
	Initialize(endPoints);
}

Line::Line(const Line& other)
	:_valid(true)
{
	Initialize(other._endPoints);
}

Line::~Line()
{
}

void Line::Initialize(const Point(&endPoints)[2])
{
	for (size_t i = 0; i < 2; i++)
	{
		if (!endPoints[i].valid())
			_valid = false;

		_endPoints[i] = endPoints[i];
	}

	if (!_valid)
		return;

	_parallelNorm = (_endPoints[1] - _endPoints[0]).Norm();
	_len = Point::Distance(_endPoints[0], _endPoints[1]);

	_valid = (_len < ZERO) ? false : true;
}

bool Line::Valid() const
{
	return _valid;
}

std::array<Point, 2> Line::GetEndPoints() const
{
	std::array<Point, 2> EndPoints;

	for (size_t i = 0; i < 2; i++)
	{
		EndPoints[i] = _endPoints[i];
	}

	return EndPoints;
}

Vector2d& Line::AccessParallelNorm()
{
	return _parallelNorm;
}

bool Line::PointOnLine(const Point& point) const
{
	if (!_valid || !point.valid() || _len < ZERO)
		return false;

	return CalPerpendicularDistanceToPoint(point) < ZERO &&
		(point - _endPoints[0]) * _parallelNorm > -ZERO &&
		(point - _endPoints[0]) * _parallelNorm < _len + ZERO;
}

double Line::CalPerpendicularDistanceToPoint(const Point& point) const
{
	if (!_valid || !(_len > ZERO) || !point.valid())
		return Nan;

	return fabs((point - _endPoints[0]) ^ _parallelNorm);
}

bool Line::HasProjectedOn(const Line& other) const
{
	double projectedLens[2] = {
		(_endPoints[0] - other._endPoints[0]) * other._parallelNorm,
		(_endPoints[1] - other._endPoints[0]) * other._parallelNorm,
	};
	
	if ((projectedLens[0] < ZERO && projectedLens[1] < ZERO) ||
		(projectedLens[0] > other._len - ZERO && projectedLens[1] > other._len - ZERO))
		return false;
	else
		return true;
}

Point Line::CalCoordForProjectedPoint(const Point& point) const
{
	if (_len < ZERO)			//its projection is not on the line.
		return Point();

	double projDot = (point - _endPoints[0]) * _parallelNorm;
	return _endPoints[0] + _parallelNorm * projDot;
}

//bool Line::HasProjectedOn(const Point& point) const
//{
//	double projectedDot = (point - _endPoints[0]) * _parallelNorm;
//
//	if ()
//}

HS::LR Line::CalLineRelation(const Line& Line1, const Line& Line2)
{
	if (!Line1._valid || !Line2._valid || Line1._len < ZERO || Line2._len < ZERO)
		return HS::UnknownRelation;

	Vector2d perpendicularNorms[2] = {
	Line1._parallelNorm.GetPerpendicular(1),
	Line2._parallelNorm.GetPerpendicular(1),
	};

	const Line* lines[2] = { &Line1 ,&Line2};

	size_t counts[2][3] = { /*first line*/{ 0,0,0},
							/*second line*/{0,0,0} };//0:ZERO on the line 1:positive 2:negative

	for (size_t i = 0; i < 2; i++)			// the i_th line
	{
		for (size_t j = 0; j < 2; j++)		//the j_th endpoint
		{
			if (fabs(perpendicularNorms[i] * (lines[1 - i]->_endPoints[j] - lines[i]->_endPoints[0])) < ZERO)
				counts[i][0]++;

			if (perpendicularNorms[i] * (lines[1 - i]->_endPoints[j] - lines[i]->_endPoints[0]) > 0)
				counts[i][1]++;

			if (perpendicularNorms[i] * (lines[1 - i]->_endPoints[j] - lines[i]->_endPoints[0]) < 0)
				counts[i][2]++;
		}
	}

	if (counts[0][0] == 2 || counts[1][0] == 2)		//they are colinear.
	{
		double maxLength = [&]()->double {
			double curLen = Nan, maxLen = Nan;
			for (size_t i = 0; i < 2; i++)
			{
				for (size_t j = 0; j < 2; j++)
				{
					curLen = Point::Distance(lines[0]->_endPoints[i], lines[1]->_endPoints[j]);
					if (std::isnan(maxLen) || curLen > maxLen)
					{
						maxLen = curLen;
					}
				}
			}
			return maxLen;
			}();

		if (fabs(maxLength - (lines[0]->_len + lines[1]->_len)) < ZERO)
			return HS::CollinearAdjacent;
		else if (maxLength > (lines[0]->_len + lines[1]->_len))
			return HS::CollinearSeparate;
		else
			return HS::CollinearOverlap;
	}

	if (counts[0][1] == 2 || counts[0][2] == 2 || counts[1][1] == 2 || counts[1][2] == 2)
		return HS::Separate;

	if (counts[0][0] == 1 && counts[1][0] == 1)		//they are connected with one endpoint.
		return HS::Link;
	
	if (counts[0][0] == 1 || counts[1][0] == 1)		//one endpoint of a line on another.
		return HS::Touch;
	
	return HS::Intersect;
}

void Line::SetEndPoints(const Point(&endPoints)[2])
{
	if (!endPoints[0].valid() || !endPoints[1].valid())
		return;

	Initialize(endPoints);
}

double Line::Len() const
{
	return _len;
}


bool Line::Move(const Vector2d& offset)
{
	if (!_valid || !offset.valid() ||
		!std::isfinite(offset.X()) || !std::isfinite(offset.Y()))
		return false;

	if (offset.X() == 0.0 && offset.Y() == 0.0)
		return true;

	for (auto& endPoint : _endPoints)
	{
		endPoint += offset;
	}

	return true;
}
} // namespace HS
