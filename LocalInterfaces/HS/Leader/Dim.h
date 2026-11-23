#pragma once
#include "LdrArranger.h"
#include <string>
#include "../Essential/BBox.h"

// Preserve the upstream algorithm; isolate its types and exports in HS for the CAA module. [Rectified/Added by Leo 2026/11/19]
namespace HS
{
class ExportedByLdrArranger Dim
{
	friend class Arranger;
public:
	Dim(const std::string& text,const BBox& annoBox);
	Dim(const Dim& other);
	virtual ~Dim();

	const BBox& GetAnnoBox() const;
	void SetAnnoBox(const BBox& annoBox);

	virtual void AddBox(const BBox& extraBox) = 0;

	void SetText(const std::string& text);

	virtual void Translate(const Vector2d& offset);

protected:
	BBox _annoBox;
	std::string _text;		//indicates the value text of dimension
};
} // namespace HS
