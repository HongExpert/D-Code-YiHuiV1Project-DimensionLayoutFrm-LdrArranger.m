#include "HS/Leader/LenDim.h"

// Preserve the upstream algorithm; isolate its types and exports in HS for the CAA module. [Rectified/Added by Leo 2026/11/19]
namespace HS
{
LenDim::LenDim(const std::string& text, const BBox& annoBox, const Line(&lines)[3])
	:Dim(text,annoBox)
{
	SetLines(lines);
}

LenDim::LenDim(const LenDim& other)
	:Dim(other)
{
	SetLines(other._lines);
}

LenDim::~LenDim()
{
}

void LenDim::SetPermutation(const Vector2d& dir)
{
	_perDir = dir;
}

void LenDim::SetLines(const Line(&lines)[3])
{
	for (size_t i = 0; i < 3; i++)
	{
		_lines[i] = lines[i];
	}
}

const std::vector<BBox>& LenDim::GetExtraBoxes() const
{
	return _extraBoxes;
}

HS::Orient LenDim::CalOrientInBox(const BBox& box) const
{
	const Vector2d refNorms[2] = {
		std::get<0>(box.AccessBoxParam()[0]),
		std::get<0>(box.AccessBoxParam()[1])
	};

	if (_perDir.ParallelTo(refNorms[0]))
		return (_perDir * refNorms[0] > 0) ? HS::Up : HS::Down;

	if (_perDir.ParallelTo(refNorms[1]))
		return (_perDir * refNorms[1] > 0) ? HS::Right : HS::Left;

	return HS::Other;
}

void LenDim::GetDimensionLines(Line(&lines)[3]) const
{
	for (size_t i = 0; i < 3; i++)
	{
		lines[i] = _lines[i];
	}
}

std::array<Line, 3> LenDim::GetDimensionLines() const
{
	return { _lines[0],_lines[1],_lines[2] };
}

void LenDim::AddBox(const BBox& extraBox)
{
	if (extraBox.Valid())
		_extraBoxes.push_back(extraBox);
}

void LenDim::Translate(const Vector2d& offset)
{
	/*if (offset.Len() < ZERO || !offset.valid())
		return;

	Dim::Translate(offset);*/
}
} // namespace HS
