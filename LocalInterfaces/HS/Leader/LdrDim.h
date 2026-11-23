#pragma once
#include "LdrArranger.h"
#include "Dim.h"

// Preserve the upstream algorithm; isolate its types and exports in HS for the CAA module. [Rectified/Added by Leo 2026/11/19]
namespace HS
{
class ExportedByLdrArranger LdrDim : public Dim
{
	friend class Arranger;
public:
	LdrDim(const std::string& text, const BBox& annoBox, const Point(&vertices)[3]);
	LdrDim(const LdrDim& other);
	virtual ~LdrDim();

	void GetVertices(Point(&vertices)[3]) const;
	void SetVertices(const Point(&vertices)[3]);

	void SetStatus(bool status);

	bool GetStatus() const;

	bool HasIntersectWith(const LdrDim* other) const;

	void SetSearchOrient(const HS::SrOrient(&srOrients)[2]);

	void SetRegion(HS::Region region);

	void SetPosition(const Point& pos);

	HS::Region GetRegion() const;

	void CalRegionInBox(const BBox& box);

	std::array<Point, 3> GetVertices() const;

	void Translate(const Vector2d& offset) override;

	void AddBox(const BBox& extraBox) override;

protected:
	bool _hasFixed;
	Point _vertices[3]; //0:anchor 1:elbow 2:endpoint
	Line _lines[2];		//0:leader(slant) 1:dimension segment(only horizontal)
	HS::Region _region;
	HS::SrOrient _srOrients[2]; //0 :priority first
};
} // namespace HS
