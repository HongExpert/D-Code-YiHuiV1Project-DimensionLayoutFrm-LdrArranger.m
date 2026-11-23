#pragma once
#include "LdrArranger.h"
#include "Dim.h"

// Preserve the upstream algorithm; isolate its types and exports in HS for the CAA module. [Rectified/Added by Leo 2026/11/19]
namespace HS
{
class ExportedByLdrArranger LenDim : public Dim
{
	friend class Arranger;
public:
	LenDim(const std::string& text, const BBox& annoBox, const Line(&lines)[3]);
	LenDim(const LenDim& other);
	virtual ~LenDim();

	void SetPermutation(const Vector2d& dir);

	void SetLines(const Line(&lines)[3]);

	const std::vector<BBox>& GetExtraBoxes() const;

	HS::Orient CalOrientInBox(const BBox& box) const;

	void GetDimensionLines(Line(&lines)[3]) const;
	std::array<Line, 3> GetDimensionLines() const;

	void AddBox(const BBox& extraBox) override;

	void Translate(const Vector2d& offset) override; //deprecated:there is defect here, but it's fine due to LenDim objects being fixed

protected:
	std::vector<BBox> _extraBoxes;					//it may have been attached by roughness symbols
	Vector2d _perDir;								//permutation
	Line _lines[3];									//0:dimension segements, 1 and 2 are boundaries
};
} // namespace HS
