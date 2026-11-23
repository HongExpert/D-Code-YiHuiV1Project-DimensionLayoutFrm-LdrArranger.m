#include "HS/Essential/Point.h"

// Preserve the upstream algorithm; isolate its types and exports in HS for the CAA module. [Rectified/Added by Leo 2026/11/19]
namespace HS
{
Point::Point(double x, double y)
	:_x(x), _y(y)
{}

Point::Point(const Point& other)
	:_x(other._x),
	_y(other._y)
{}

Point::Point()
	:_x(Nan),
	_y(Nan)
{}

Point::~Point()
{}

double Point::X() const
{
	return _x;
}

double& Point::sX()
{
	return _x;
}

double Point::Y() const
{
	return _y;
}

double& Point::sY()
{
	return _y;
}

Vector2d Point::Norm()
{
	double length = sqrt(_x * _x + _y * _y);
	if (length == 0.0)
		return Vector2d(0.0, 0.0);            // avoid division by zero
	return Vector2d(_x / length, _y / length);
}

Vector2d Point::GetPerpendicular(int type) const
{
	if (!valid())
		return Vector2d();

	switch (type)
	{
	case -1:
		return Vector2d(-_y, _x);
	case 1:
	default:
		return Vector2d(_y, -_x);
	}
}

bool Point::ParallelTo(const Vector2d& other) const
{
	return fabs(*this ^ other) < ZERO;
}

bool Point::PerpendicularTo(const Vector2d& other) const
{
	return fabs(*this * other) < ZERO;
}

double Point::Distance(const Point& PointA, const Point& PointB)
{
	double dx = PointA.X() - PointB.X();
	double dy = PointA.Y() - PointB.Y();
	return sqrtl(dx * dx + dy * dy);
}

double Point::Angle(	
	const Vector2d& v1,
	const Vector2d& v2)
{

	if (!(v1.Len() > ZERO) || !(v2.Len() > ZERO))
		return 0;	// <--（改成 !(len > ZERO)：原来的 len < ZERO 对 NaN 恒为 false，无效向量会一路带 NaN 进 acos；同时 Len() 只算一次，原来算了三遍）

	double cosine = (v1 * v2) / (v1.Len() * v2.Len());
	cosine = (std::max)(-1.0, (std::min)(1.0, cosine));	// <--（浮点误差会让余弦略大于 1 或略小于 -1，acos 返回 NaN；该角度被 CalSrOrientOrder 当作 std::sort 比较器，NaN 会破坏严格弱序）

	return acos(cosine);	// <--（原来这里少一个右括号，acos 的括号没闭合，整个文件无法编译）
}

double Point::Len() const
{
	return sqrt(_x * _x + _y * _y);
}

double Point::operator*(
	const Point& other) const
{
	return (_x * other._x + _y * other._y);
}

Point Point::operator*(
	double value) const
{
	return Point(_x * value, _y * value);
}

Point Point::operator/(
	double value) const
{
	if (value == 0)
		return *this;

	return Point(_x / value, _y / value);
}

double Point::operator^(
	const Point& other) const
{
	return (_x * other._y - _y * other._x);
}

Point Point::operator+(const Point& other) const
{
	return Point(_x + other._x, _y + other._y);
}

void Point::operator+=(const Point& other)
{
	_x = _x + other._x;
	_y = _y + other._y;
}

void Point::operator-=(const Point& other)
{
	_x = _x - other._x;
	_y = _y - other._y;
}

Point Point::operator-(const Point& other) const
{
	return Point(_x - other._x, _y - other._y);
}

Point Point::operator-() const
{
	return Point(-_x,  -_y);
}

bool Point::operator==(const Point& other) const
{
	if (abs(other._x - _x) < ZERO && abs(other._y - _y) < ZERO)
		return true;
	else
		return false;
}

bool Point::valid() const
{
	if (std::isnan(_x) || std::isnan(_y))
		return false;
	else
		return true;
}
} // namespace HS
