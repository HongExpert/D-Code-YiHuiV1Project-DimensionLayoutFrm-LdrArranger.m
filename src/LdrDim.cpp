#include "HS/Leader/LdrDim.h"

// Preserve the upstream algorithm; isolate its types and exports in HS for the CAA module. [Rectified/Added by Leo 2026/11/19]
namespace HS
{
LdrDim::LdrDim(const std::string& text,const BBox& annoBox, const Point(&vertices)[3])
	:Dim(text,annoBox),
	_region(HS::Unknown),
	_hasFixed(false)
{
	SetVertices(vertices);
}

LdrDim::LdrDim(const LdrDim& other)
	:Dim(other),
	_region(other._region),
	_hasFixed(false)
{
	SetVertices(other._vertices);
}

LdrDim::~LdrDim()
{
}

void LdrDim::GetVertices(Point(&vertices)[3]) const
{
	for (size_t i = 0; i < 3; i++)
	{
		vertices[i] = _vertices[i];
	}
}

void LdrDim::SetVertices(const Point(&vertices)[3])
{
	for (size_t i = 0; i < 3; i++)
	{
		_vertices[i] = vertices[i];
	}

	_lines[0].SetEndPoints({_vertices[0],_vertices[1]});
	_lines[1].SetEndPoints({_vertices[1],_vertices[2]});
	_srOrients[0] = HS::NO;
	_srOrients[1] = HS::NO;
}

void LdrDim::SetStatus(bool status)
{
	_hasFixed = status;
}

bool LdrDim::GetStatus() const
{
	return _hasFixed;
}

bool LdrDim::HasIntersectWith(const LdrDim* other) const
{
	if (!other)
		return false;

	for (auto thisLine : _lines)
	{
		if (!thisLine.Valid())
			continue;

		if (other->GetAnnoBox().IntersectWithLine(thisLine))
			return true;

		for (auto otherLine : other->_lines)
		{
			if (!otherLine.Valid())
				continue;

			if (_annoBox.IntersectWithLine(otherLine))
				return true;

			if (Line::CalLineRelation(thisLine, otherLine) == HS::Intersect ||
				Line::CalLineRelation(thisLine, otherLine) == HS::CollinearOverlap)
				return true;
		}


	}
	
	return false;
}

void LdrDim::SetSearchOrient(const HS::SrOrient(&srOrients)[2])
{
	_srOrients[0] = srOrients[0];
	_srOrients[1] = srOrients[1];
}

void LdrDim::SetRegion(HS::Region region)
{
	_region = region;
}

void LdrDim::SetPosition(const Point& pos)
{
	if ( !pos.valid())
		return;

	Translate(pos - _vertices[1]);
}

HS::Region LdrDim::GetRegion() const
{
	return _region;
}

void LdrDim::CalRegionInBox(const BBox& box)
{
	if (!box.Valid())
		return;

	_region = box.CalPointRegion(_vertices[0]);
}

std::array<Point, 3> LdrDim::GetVertices() const
{
	return { _vertices[0] ,_vertices[1] ,_vertices[2] };
}

void LdrDim::Translate(const Vector2d& offset)
{
	if (offset.Len() < ZERO || !offset.valid())
		return;

	_vertices[1] += offset;		//elbow and endpoint move along offset direction.
	_vertices[2] += offset;

	_lines[0] = Line({ _vertices[0],_vertices[1] });
	_lines[1] = Line({ _vertices[1],_vertices[2] });

	Dim::Translate(offset);
}

void LdrDim::AddBox(const BBox& extraBox)
{
	_annoBox += extraBox;
}
} // namespace HS
