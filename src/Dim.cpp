#include "HS/Leader/Dim.h"

// Preserve the upstream algorithm; isolate its types and exports in HS for the CAA module. [Rectified/Added by Leo 2026/11/19]
namespace HS
{
Dim::Dim(const std::string& text,const BBox& annoBox)
	:_annoBox(annoBox),
	_text(text)
{
}

Dim::Dim(const Dim& other)
	:_annoBox(other._annoBox),
	_text(other._text)
{
}

Dim::~Dim()
{
}

const BBox& Dim::GetAnnoBox() const
{
	return _annoBox;
}

void Dim::SetAnnoBox(const BBox& annoBox)
{
	_annoBox = annoBox;
}

//void Dim::AddBox(const BBox& extraBox)
//{
//	_extraBox = extraBox;
//}

void Dim::Translate(const Vector2d& offset)
{
	if (offset.Len() < ZERO || !offset.valid())
		return;

	_annoBox.Move(offset);
}
} // namespace HS
