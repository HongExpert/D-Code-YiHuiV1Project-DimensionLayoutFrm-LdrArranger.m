#include <algorithm>
#include <iostream>
#include "HS/Arranger.h"
// Reuse the project INI reader and parse numbers independently of the system locale. [Rectified/Added by Leo_GPT 2026/11/20]
#include "INIReader.h"
#include <locale>
#include <sstream>

// Preserve the upstream algorithm; isolate its types and exports in HS for the CAA module. [Rectified/Added by Leo 2026/11/19]
namespace HS
{
Arranger& Arranger::Instance()
{
	static Arranger instance;
	return instance;
}

namespace
{
	// Preserve the original values when a caller has no project configuration. [Rectified/Added by Leo_GPT 2026/11/20]
	constexpr double DefaultSearchAngleDegrees = 20.0;
	constexpr double DefaultSearchStep = 1.0;
}

// Initialize the search parameters for callers that omit the INI. [Rectified/Added by Leo_GPT 2026/11/20]
Arranger::Arranger()
	: _srAngle(DefaultSearchAngleDegrees * Pi / 180.0), _srDist(DefaultSearchStep)
{
}

Arranger::~Arranger()
{
}

// Reload each key independently so an invalid or absent value restores its default. [Rectified/Added by Leo_GPT 2026/11/20]
void Arranger::LoadSearchParameters(const INIReader& config)
{
	const auto readValue = [&](const char* key, double fallback)
	{
		std::istringstream input(config.Get("UserProjectUISet", key, ""));
		input.imbue(std::locale::classic());
		double value = 0.0;
		return (input >> value) && input.eof() && std::isfinite(value) ? value : fallback;
	};

	const double degrees = readValue("SrAngle", DefaultSearchAngleDegrees);
	_srAngle = (degrees > 0.0 && degrees < 90.0 ? degrees : DefaultSearchAngleDegrees) * Pi / 180.0;
	const double step = readValue("SrDist", DefaultSearchStep);
	_srDist = step > ZERO ? step : DefaultSearchStep;
}

void Arranger::Setup(double ldrAltitude,const std::string& viewName, const BBox& partBox, const std::vector<Dim*>& dims)
{
	_viewName = viewName;
	_partBox = partBox;
	_dims = dims;
	_ldrAltitude = ldrAltitude;
	//_LenDimCompositeBoxes.clear();
	_LenDimsWithOrients.clear();
	_LdrDimsWithSrOrients.clear();
	_LdrDimsClassifiedViaFourQuadrants.clear();
	_SearchOrients.clear();
	_fixedDims.clear();

	Initialize();
}

// Dump the part box and every dimension's geometry for data monitoring. [Added by Leo 2026/11/19]
void Arranger::OutputData() const
{
	std::cout << "<-------------------------" << _viewName << "------------------------------->" << std::endl;
	auto outVertices = [](const std::array<Point, 4>& vertices, const std::string& tag)
	{
		std::cout << tag << " = [";
		for (size_t i = 0; i < vertices.size(); ++i)
			std::cout << (i == 0 ? "" : ", ") << "(" << vertices[i].X() << ", " << vertices[i].Y() << ")";
		std::cout << "]\n";
	};

	auto outBox = [&outVertices](const BBox& box, const std::string& tag)
	{
		std::array<Point, 4> vertices{};
		box.GetVertices(vertices);
		outVertices(vertices, tag);
	};

	auto outLines = [](const auto& lines, const std::string& tag)
	{
		const size_t count = sizeof(lines) / sizeof(lines[0]);
		for (size_t i = 0; i < count; ++i)
		{
			const auto endPoints = lines[i].GetEndPoints();
			std::cout << tag << "[" << i << "] = (" << endPoints[0].X() << ", " << endPoints[0].Y()
				<< ") -> (" << endPoints[1].X() << ", " << endPoints[1].Y() << ")\n";
		}
	};

	outBox(_partBox, "PartBox");

	size_t lenDimCount = 0;
	size_t ldrDimCount = 0;
	for (const auto dim : _dims)
	{
		if (dynamic_cast<const LenDim*>(dim))
			++lenDimCount;
		else if (dynamic_cast<const LdrDim*>(dim))
			++ldrDimCount;
	}

	std::cout << "LenDim count = " << lenDimCount << "\n";
	size_t lenDimIndex = 0;
	for (const auto dim : _dims)
	{
		const LenDim* lenDim = dynamic_cast<const LenDim*>(dim);
		if (!lenDim)
			continue;

		const std::string prefix = "LenDim[" + std::to_string(lenDimIndex++) + "]";
		std::cout << prefix << " : \"" << lenDim->_text << "\"\n";
		outBox(lenDim->_annoBox, prefix + " annoBox");
		outLines(lenDim->_lines, prefix + " lines");

		const std::vector<BBox>& extraBoxes = lenDim->GetExtraBoxes();
		if (!extraBoxes.empty())
		{
			std::cout << prefix << " extraBoxes count = " << extraBoxes.size() << "\n";
			for (size_t i = 0; i < extraBoxes.size(); ++i)
				outBox(extraBoxes[i], prefix + " extraBoxes[" + std::to_string(i) + "]");
		}
	}

	std::cout << "LdrDim count = " << ldrDimCount << "\n";
	size_t ldrDimIndex = 0;
	for (const auto dim : _dims)
	{
		const LdrDim* ldrDim = dynamic_cast<const LdrDim*>(dim);
		if (!ldrDim)
			continue;

		const std::string prefix = "LdrDim[" + std::to_string(ldrDimIndex++) + "]";
		std::cout << prefix << " : \"" << ldrDim->_text << "\"\n";
		outBox(ldrDim->_annoBox, prefix + " annoBox");
		outLines(ldrDim->_lines, prefix + " lines");
	}

	std::cout << "<----------------------" << _viewName << " END------------------------------>" << std::endl;
}

// Expose each input leader's current elbow without modifying geometry or placement status. [Rectified/Added by Leo 2026/11/19]
std::map<const LdrDim*, Point> Arranger::GetLdrDimElbowPositions() const
{
	std::map<const LdrDim*, Point> positions;
	for (const auto dim : _dims)
	{
		const LdrDim* leader = dynamic_cast<const LdrDim*>(dim);
		if (leader)
			positions.emplace(leader, leader->GetVertices()[1]);
	}
	return positions;
}

std::map<HS::Region, std::vector<LdrDim*>> Arranger::CollectLdrDimGroups()
{
	std::map<HS::Region, std::vector<LdrDim*>> groups;
	if (_dims.empty())
		return groups;

	for (auto dim : _dims)
	{
		LdrDim* ldrDim = nullptr;
		if (ldrDim = dynamic_cast<LdrDim*>(dim))
		{
			ldrDim->CalRegionInBox(_partBox);
			groups[ldrDim->GetRegion()].push_back(ldrDim);
		}
	}

	return groups;
}

void Arranger::CollectLdrDimsOnlyForQuadrant()
{
	_LdrDimsClassifiedViaFourQuadrants.clear();

	if (_dims.empty() || !_partBox.Valid())
		return;

	for (auto dim : _dims)
	{
		LdrDim* ldrDim = nullptr;
		if (!(ldrDim = dynamic_cast<LdrDim*>(dim)))
			continue;

		HS::Region region = HS::Unknown;

		switch (ldrDim->_srOrients[0])
		{
		case HS::UR:
		case HS::RU:
			region = HS::Quadrant1;
			break;

		case HS::DR:
		case HS::RD:
			region = HS::Quadrant2;
			break;

		case HS::UL:
		case HS::LU:
			region = HS::Quadrant3;
			break;

		case HS::DL:
		case HS::LD:
			region = HS::Quadrant4;
			break;

		default:
			continue;
		}

		if (ldrDim->_srOrients[1] == HS::NO)
			continue;

		_LdrDimsClassifiedViaFourQuadrants[region].push_back(ldrDim);
	}
}

void Arranger::Initialize()
{
	ClassifyLenDims();
	//InitLenDimCompositeBoxes();
	InitSearchOrients();
	std::map<HS::Region, std::vector<LdrDim*>> LdrDimsWithRegion = CollectLdrDimGroups();
	InitLdrDimSearchOrients(LdrDimsWithRegion);
	CollectLdrDimsOnlyForQuadrant();
}

void Arranger::InitSearchOrients()
{
	_SearchOrients.clear();

	Vector2d u = std::get<0>(_partBox._param[0]); //relatively parallel to local Y axis
	Vector2d v = std::get<0>(_partBox._param[1]); //relatively parallel to local X axis

	if (!_partBox.Valid() || !u.valid() || !v.valid())
	{
		u = Vector2d(1.0, 0.0);
		v = Vector2d(0.0, 1.0);
	}

	_SearchOrients = {
		{ HS::UR, v },
		{ HS::UL, -v },
		{ HS::DR, v },
		{ HS::DL, -v },
		{ HS::RU, u },
		{ HS::RD, -u },
		{ HS::LU, u },
		{ HS::LD, -u }
	};

}

void Arranger::ClassifyLenDims()
{
	_LenDimsWithOrients.clear();
	_fixedDims.clear();

	if (_dims.empty())
		return;

	for (auto dim : _dims)
	{
		LenDim* lenDim = nullptr;
		if (lenDim = dynamic_cast<LenDim*>(dim))
		{
			_LenDimsWithOrients[lenDim->CalOrientInBox(_partBox)].push_back(lenDim);
			_fixedDims.insert(lenDim);
		}
	}
}

//void Arranger::CalLocatingRegions()
//{
//	for (auto LdrDimSrGroup : _LdrDimsWithSrOrients)
//	{
//
//	}
//}


BBox Arranger::CalEntireBoxToOneOrientLenDims(HS::Orient orient, std::vector<LenDim*>& LenDims)
{
	BBox entireBox;
	if (LenDims.empty())
		return entireBox;

	auto AddGeometircElementsToExpandBox = [](const LenDim* lenDim)->BBox {
		
		BBox box;
		if (!lenDim)
			return box;
		else
			box = lenDim->GetAnnoBox();
		
		Line lines[3];
		lenDim->GetDimensionLines(lines);

		for (auto line: lines)
		{
			if (!line.Valid())
				continue;

			auto endPoints = line.GetEndPoints();
			box = box + endPoints[0];
			box = box + endPoints[1];
		}

		for (auto extra : lenDim->GetExtraBoxes())
		{
			box += extra;
		}

		return box;
		};

	for (auto lenDim : LenDims)
	{
		if (!lenDim)
			continue;

		BBox box = AddGeometircElementsToExpandBox(lenDim);

		if (!box.Valid())
			continue;

		if (!entireBox.Valid())
			entireBox = box;
		else
			entireBox += box;
	}

	return entireBox;
}

Point Arranger::FindVerticalSrPointOnPartBox(HS::Orient side, const Vector2d& dir)
{
	auto FindArrowProjectsLimitPoint = [&](const std::vector<LenDim*>& lenDims)->Point {

		double maxDot = 0., curDot = 0.;
		bool hasAdded = false;
		Point pnt;
		for (auto ldim : lenDims)
		{
			if (!ldim)
				continue;

			for (size_t i = 0; i < 2; i++)
			{
				curDot = ldim->_lines[0].GetEndPoints()[i] * dir;
				if (!hasAdded || curDot > maxDot)
				{
					hasAdded = true;
					maxDot = curDot;
					pnt = ldim->_lines[0].GetEndPoints()[i];
				}
			}
		}

		return pnt;
		};

	Point pnt = _partBox._edges[side].CalCoordForProjectedPoint(FindArrowProjectsLimitPoint(_LenDimsWithOrients[side]));
	Point cen_PartBox = _partBox.GetCenter();

	if (!pnt.valid())
		pnt = _partBox._edges[side].CalCoordForProjectedPoint(cen_PartBox + dir * _ldrAltitude);

	return pnt;
}

//void Arranger::InitLenDimCompositeBoxes()
//{
//	if (_LenDimsWithOrients.empty())
//		return;
//
//	for (auto group : _LenDimsWithOrients)
//	{
//		if (group.second.empty())
//			continue;
//
//		BBox compositeBox = CalEntireBoxToOneOrientLenDims(group.first, group.second);
//		if (compositeBox.Valid())
//			_LenDimCompositeBoxes.emplace(group.first, compositeBox);
//	}
//}

void Arranger::InitLdrDimSearchOrients(std::map<HS::Region, std::vector<LdrDim*>>& ldrGroups)
{
	_LdrDimsWithSrOrients.clear();
	if (ldrGroups.empty())
		return;

	// Fixed leaders are obstacles before the first candidate is searched. [Rectified/Added by Leo 2026/11/19]
	for (const auto& group : ldrGroups)
		for (auto ldrDim : group.second)
			if (ldrDim->GetStatus())
				_fixedDims.insert(ldrDim);

	const HS::SrOrient fallback[2] = { HS::UR, HS::RD };
	if (!_partBox.Valid())
	{
		for (const auto& group : ldrGroups)
		{
			for (auto ldrDim : group.second)
			{
				ldrDim->SetSearchOrient(fallback);
				_LdrDimsWithSrOrients[fallback[0]].push_back(ldrDim);
			}
		}
		return;
	}

	// V11 section 2.2 ranks the geometrically eligible sides by distance, then by length count for axis anchors, and finally by compass order. [Rectified/Added by LeoLeo_GPT 2026/11/19]
	struct SideChoice
	{
		HS::Orient side;
		int edge;
		HS::SrOrient positive;
		HS::SrOrient negative;
	};
	const SideChoice sides[] = {
		{ HS::Up, 1, HS::UR, HS::UL },
		{ HS::Right, 2, HS::RU, HS::RD },
		{ HS::Down, 3, HS::DR, HS::DL },
		{ HS::Left, 0, HS::LU, HS::LD }
	};
	constexpr size_t sideCount = sizeof(sides) / sizeof(sides[0]);
	const Point center = _partBox.GetCenter();
	const Vector2d xAxis = std::get<0>(_partBox._param[1]);
	const Vector2d yAxis = std::get<0>(_partBox._param[0]);
	const auto lenCount = [this](HS::Orient side) {
		const auto it = _LenDimsWithOrients.find(side);
		return it == _LenDimsWithOrients.end() ? size_t(0) : it->second.size();
	};

	for (const auto& group : ldrGroups)
	{
		for (auto ldrDim : group.second)
		{
			const Point& anchor = ldrDim->_vertices[0];
			const Vector2d offset = anchor - center;
			const double x = offset * xAxis;
			const double y = offset * yAxis;
			const bool onAxis = std::abs(x) <= ZERO || std::abs(y) <= ZERO;
			const bool eligible[] = { y >= -ZERO, x >= -ZERO, y <= ZERO, x <= ZERO };
			double distances[] = { Nan, Nan, Nan, Nan };
			double nearest = std::numeric_limits<double>::infinity();
			for (size_t i = 0; i < sideCount; ++i)
			{
				if (!eligible[i])
					continue;
				distances[i] = _partBox._edges[sides[i].edge].CalPerpendicularDistanceToPoint(anchor);
				if (std::isfinite(distances[i]))
					nearest = (std::min)(nearest, distances[i]);
			}

			const SideChoice* best = nullptr;
			for (size_t i = 0; i < sideCount; ++i)
			{
				if (!std::isfinite(distances[i]) || distances[i] > nearest + ZERO)
					continue;
				if (!best || (onAxis && lenCount(sides[i].side) < lenCount(best->side)))
					best = &sides[i];
			}

			HS::SrOrient directions[2] = { fallback[0], fallback[1] };
			if (best)
			{
				const bool horizontal = best->side == HS::Up || best->side == HS::Down;
				const bool positiveFirst = (horizontal ? x : y) >= -ZERO;
				directions[0] = positiveFirst ? best->positive : best->negative;
				directions[1] = positiveFirst ? best->negative : best->positive;
			}
			ldrDim->SetSearchOrient(directions);
			_LdrDimsWithSrOrients[directions[0]].push_back(ldrDim);
		}
	}
}

//BBox Arranger::GenerateCompositeBoxInSameSrOrient(HS::SrOrient srOrient, std::vector<LdrDim*>& ldrDims)
//{
//	BBox compositeBox;
//
//	if (_viewName != "Front view")
//	{
//		CalSrOrientOrder(srOrient, ldrDims);
//		AlignWithElbowToGenerateBox(_SearchOrients[srOrient], compositeBox, ldrDims);
//	}
//	else
//	{
//		Vector2d u = std::get<0>(_partBox._param[0]); //relatively parallel to local Y axis
//
//		CalVerticalSrOrientOrder(srOrient, ldrDims);
//		switch (srOrient)
//		{
//		case HS::UL:
//		case HS::UR:
//		case HS::RU:
//		case HS::LU:
//			AlignWithElbowToGenerateBox(u,  compositeBox, ldrDims); break;
//		case HS::DL:
//		case HS::DR:
//		case HS::RD:
//		case HS::LD:
//			AlignWithElbowToGenerateBox(-u, compositeBox, ldrDims); break;
//		default:
//			return compositeBox;
//		}
//
//
//	}
//
//	return compositeBox;
//}

void Arranger::CalSrOrientOrder(HS::SrOrient srOrient, std::vector<LdrDim*>& ldrDims)
{
	if (ldrDims.empty())
		return;

	std::array<Point, 4>vertices;
	_partBox.GetVertices(vertices);

	auto setElbowPoint = [&](const Point& point) {
		for (auto ldrDim : ldrDims)
		{
			if (ldrDim->GetStatus())
				continue;

			ldrDim->Translate(point - ldrDim->_vertices[1]);
		}
	};

	Vector2d u = std::get<0>(_partBox._param[0]);	//relatively parallel to local Y axis
	Vector2d v = std::get<0>(_partBox._param[1]);	//relatively parallel to local X axis
	Vector2d axis = v;

	auto maxAngularOrder = [&](const LdrDim* ldr1, const LdrDim* ldr2)->bool {
		return (Point::Angle(ldr1->_vertices[0] - ldr1->_vertices[1], axis) >
			Point::Angle(ldr2->_vertices[0] - ldr2->_vertices[1], axis));
		};

	auto minAngularOrder = [&](const LdrDim* ldr1, const LdrDim* ldr2)->bool {
		return (Point::Angle(ldr1->_vertices[0] - ldr1->_vertices[1], axis) <
			Point::Angle(ldr2->_vertices[0] - ldr2->_vertices[1], axis));
		};

	switch (srOrient)
	{
	case HS::UR:
	{
		setElbowPoint(vertices[2]);
		axis = v;
		std::sort(ldrDims.begin(), ldrDims.end(), maxAngularOrder);
	}
	break;

	case HS::UL:
	{
		setElbowPoint(vertices[1]);
		axis = -v;
		std::sort(ldrDims.begin(), ldrDims.end(), maxAngularOrder);
	}
	break;

	case HS::RU:
	{
		setElbowPoint(vertices[2]);
		axis = u;
		std::sort(ldrDims.begin(), ldrDims.end(), minAngularOrder);
	}
	break;

	case HS::RD:
	{
		setElbowPoint(vertices[3]);
		axis = -u;
		std::sort(ldrDims.begin(), ldrDims.end(), minAngularOrder);
	}
	break;

	case HS::LU:
	{
		setElbowPoint(vertices[1]);
		axis = u;
		std::sort(ldrDims.begin(), ldrDims.end(), minAngularOrder);
	}
	break;

	case HS::LD:
	{
		setElbowPoint(vertices[0]);
		axis = -u;
		std::sort(ldrDims.begin(), ldrDims.end(), minAngularOrder);
	}
	break;

	case HS::DL:
	{
		setElbowPoint(vertices[0]);
		axis = -v;
		std::sort(ldrDims.begin(), ldrDims.end(), maxAngularOrder);
	}
	break;

	case HS::DR:
	{
		setElbowPoint(vertices[3]);
		axis = v;
		std::sort(ldrDims.begin(), ldrDims.end(), maxAngularOrder);
	}
	break;

	default:
		break;
	}
}

void Arranger::CalVerticalSrOrientOrder(HS::SrOrient srOrient, std::vector<LdrDim*>& ldrDims)
{
	if (ldrDims.empty())
		return;

	std::array<Point, 4>vertices;
	_partBox.GetVertices(vertices);

	auto setElbowPoint = [&](const Point& point) {		//move all elbow vertex: _vertices[1] to the corresponding Quadrant Corner of _partBox.
		for (auto ldrDim : ldrDims)
		{
			ldrDim->Translate(point - ldrDim->_vertices[1]);
		}
		};

	Vector2d u = std::get<0>(_partBox._param[0]);

	auto PMaxAngularOrder = [&](const LdrDim* ldr1, const LdrDim* ldr2)->bool {
		//relatively parallel to local Y axis
		return (Point::Angle(ldr1->_vertices[0] - ldr1->_vertices[1], u) >
			Point::Angle(ldr2->_vertices[0] - ldr2->_vertices[1], u));
		};

	auto NMaxAngularOrder = [&](const LdrDim* ldr1, const LdrDim* ldr2)->bool {
		//relatively parallel to local Y axis
		return (Point::Angle(ldr1->_vertices[0] - ldr1->_vertices[1], -u) >
			Point::Angle(ldr2->_vertices[0] - ldr2->_vertices[1], -u));
		};

	switch (srOrient)
	{
	case HS::UR:
	case HS::RU:
		setElbowPoint(vertices[2]);
		std::sort(ldrDims.begin(), ldrDims.end(), PMaxAngularOrder);
		break;

	case HS::UL:
	case HS::LU:
		setElbowPoint(vertices[1]);
		std::sort(ldrDims.begin(), ldrDims.end(), PMaxAngularOrder);
		break;

	case HS::LD:
	case HS::DL:

		setElbowPoint(vertices[0]);
		std::sort(ldrDims.begin(), ldrDims.end(), NMaxAngularOrder);
		break;

	case HS::DR:
	case HS::RD:

		setElbowPoint(vertices[3]);
		std::sort(ldrDims.begin(), ldrDims.end(), NMaxAngularOrder);
		break;

	default:
		break;
	}

}

void Arranger::SearchLdrDimVerticalPositions(std::vector<LdrDim*>& ldrDims)//bug exists.
{

	Vector2d u = std::get<0>(_partBox._param[0]);	//relatively parallel to local Y axis
	Vector2d v = std::get<0>(_partBox._param[1]);	//relatively parallel to local X axis

	auto searchPosition = [&](
		LdrDim* ldrDim,
		HS::Orient partBoxSide,
		HS::SrOrient srOrient,
		const Vector2d& projdir)
		{
		const Point originalElbow = ldrDim->_vertices[1];

		bool finalRound = false;
		bool success = false;
		double sumOffset = 0;
		Vector2d srVec,offSetVec;

		Point s_Pnt = FindVerticalSrPointOnPartBox(partBoxSide, projdir);

		if (!s_Pnt.valid())
			return;

		switch (srOrient)
		{
		case HS::UR:
		case HS::RU:
			srVec = u;
			offSetVec = v;
			s_Pnt += (v + u).Norm() * _ldrAltitude / 2; break;
		case HS::UL:
		case HS::LU:
			srVec = u;
			offSetVec = -v;
			s_Pnt += (-v + u).Norm() * _ldrAltitude / 2; break;
		case HS::LD:
		case HS::DL:
			srVec = -u;
			offSetVec = -v;
			s_Pnt += (-v + -u).Norm() * _ldrAltitude / 2; break;
		case HS::DR:
		case HS::RD:
			srVec = -u;
			offSetVec = v;
			s_Pnt += (v + -u).Norm() * _ldrAltitude / 2; break;
		default:
			return;
		}

		ldrDim->SetPosition(s_Pnt);

		while (true)
		{
			success = SearchPositionForCurSrOrient(ldrDim, srVec, _fixedDims);
			if (success || finalRound)
				break;

			s_Pnt = s_Pnt + offSetVec * _ldrAltitude * 1.5;

			sumOffset += _ldrAltitude;
			ldrDim->SetPosition(s_Pnt);

			if (sumOffset > 8 * _ldrAltitude)
				finalRound = true;
		}

		if (!success)
			ldrDim->SetPosition(originalElbow);

		ldrDim->SetStatus(success);
		_fixedDims.insert(ldrDim);
		};

	for (auto ldrDim : ldrDims)
	{
		if (!ldrDim)
			continue;

		switch (ldrDim->_srOrients[0])	//only search its first preferred 
		{
		case HS::UR:
			searchPosition(ldrDim, HS::Up, HS::UR, v);break;
		case HS::RU:
			searchPosition(ldrDim, HS::Right, HS::RU, u); break;
		case HS::UL:
			searchPosition(ldrDim, HS::Up, HS::UL, -v); break;
		case HS::LU:
			searchPosition(ldrDim, HS::Left, HS::LU, u); break;
		case HS::DL:
			searchPosition(ldrDim, HS::Down, HS::DL, -v); break;
		case HS::LD:
			searchPosition(ldrDim, HS::Left, HS::LD, -u); break;
		case HS::DR:
			searchPosition(ldrDim, HS::Down, HS::DR, v); break;
		case HS::RD:
			searchPosition(ldrDim, HS::Right, HS::RD, -u); break;
		default:
			break;

		}
	}
}

void Arranger::CalVerticalRegionOrder(HS::Region region, std::vector<LdrDim*>& ldrDims)
{
	if (ldrDims.empty())
		return;

	std::array<Point, 4>vertices;
	_partBox.GetVertices(vertices);

	auto setElbowPoint = [&](const Point& point) {
		for (auto ldrDim : ldrDims)
		{
			ldrDim->SetPosition(point);
		}
		};

	auto minAngularOrder = [&](const LdrDim* ldr1, const LdrDim* ldr2)->bool {
		//relatively parallel to local Y axis
		return (Point::Angle(ldr1->_vertices[0] - ldr1->_vertices[1], std::get<0>(_partBox._param[1])) <
			Point::Angle(ldr2->_vertices[0] - ldr2->_vertices[1], std::get<0>(_partBox._param[1])));
		};

	//_vertices:0 left-down 1 left-up 2 right-up 3 right-down
	//each region collapses its elbows onto the corner it grows from.
	switch (region)
	{
	case HS::Quadrant1:
		setElbowPoint(vertices[2]); break;	//right-up
	case HS::Quadrant2:
		setElbowPoint(vertices[3]); break;	//right-down
	case HS::Quadrant3:
		setElbowPoint(vertices[1]); break;	//left-up
	case HS::Quadrant4:
		setElbowPoint(vertices[0]); break;	//left-down
	default:
		break;
	}

	std::sort(ldrDims.begin(), ldrDims.end(), minAngularOrder);
}

void Arranger::PerformLocatingLdrDims()
{
	if (_LdrDimsWithSrOrients.empty())
		return;

	for (auto srGroup : _LdrDimsWithSrOrients)
	{
		if (srGroup.first == HS::NO)
			continue;

		// Accept the native CATIA front-view name and its compact spelling. [Rectified/Added by Leo_GPT 2026/11/20]
		if (_viewName == "Front view" || _viewName == "FrontView")
		{
			// Sort only movable leaders and restore their elbows after the temporary corner alignment. [Rectified/Added by Leo_GPT 2026/11/20]
			srGroup.second.erase(std::remove_if(srGroup.second.begin(), srGroup.second.end(),
				[](const LdrDim* ldrDim) { return !ldrDim || ldrDim->GetStatus(); }), srGroup.second.end());
			std::vector<std::pair<LdrDim*, Point>> originalElbows;

			for (auto ldrDim : srGroup.second)
				originalElbows.emplace_back(ldrDim, ldrDim->_vertices[1]);
			CalVerticalSrOrientOrder(srGroup.first, srGroup.second);
			for (const auto& original : originalElbows)
				original.first->SetPosition(original.second);
			SearchLdrDimVerticalPositions(srGroup.second);
		}
		else
		{
			CalSrOrientOrder(srGroup.first, srGroup.second);
			SearchLdrDimPositions(srGroup.first, srGroup.second);
		}
	}
}

void Arranger::SearchLdrDimPositions(HS::SrOrient srOrient, std::vector<LdrDim*>& ldrDims)
{
	if (ldrDims.empty() || !_partBox.Valid() || srOrient == HS::NO)
		return;

	//const double a = _srAngle;//start searching angle;
	Vector2d refVector;
	Line alEdge;				//the edge with which it searches along

	Vector2d u = std::get<0>(_partBox._param[0]);	//relatively parallel to local Y axis
	Vector2d v = std::get<0>(_partBox._param[1]);	//relatively parallel to local X axis

	switch (srOrient)
	{
	case HS::UR:
	case HS::UL:
		refVector = u;
		alEdge = _partBox._edges[1];
		break;

	case HS::DR:
	case HS::DL:
		refVector = -u;
		alEdge = _partBox._edges[3];
		break;

	case HS::LU:
	case HS::LD:
		refVector = -v;
		alEdge = _partBox._edges[0];
		break;

	case HS::RU:
	case HS::RD:
		refVector = v;
		alEdge = _partBox._edges[2];
		break;

	default:
		return;
	}

	auto LiftUpFixedLdrDimsAltitude = [&]() {

		Vector2d u = std::get<0>(_partBox._param[0]);	//relatively parallel to local Y axis

		for (auto ldrDim : ldrDims)
		{
			if (!ldrDim->GetStatus() || !ldrDim)
				continue;

			switch (ldrDim->_srOrients[0])
			{
			case HS::UR:
			case HS::UL:
				ldrDim->Translate(u * _ldrAltitude); break;
			case HS::DR:
			case HS::DL:
				ldrDim->Translate(-u * _ldrAltitude); break;

			default:
				continue;
			}
		}	
		};

	for (auto ldrDim : ldrDims)
	{
		if (!ldrDim || 
			ldrDim->_srOrients[0] ==HS::NO || ldrDim->_srOrients[1] == HS::NO ||
			ldrDim->GetStatus())
			continue;

		bool success = false;
		size_t tryTimes = 0;
		while(!success && tryTimes++ <= 3)
		{
			LiftUpFixedLdrDimsAltitude();

			double perpDistToAnchor = alEdge.CalPerpendicularDistanceToPoint(ldrDim->GetVertices()[0]) + 1.2 * _ldrAltitude;  //calculate the perpendicular distance from current LdrDim object anchor Point to current _parbBox edge. 
			if (!std::isfinite(perpDistToAnchor))	// <--Translate silently swallows NaN. As a result, the annotation remains unmoved, yet the operation is incorrectly evaluated as successful.
				continue;

			double offset = perpDistToAnchor * tan(_srAngle);

			Point s_Pnt = ldrDim->_vertices[0] + refVector * perpDistToAnchor + _SearchOrients[ldrDim->_srOrients[0]] * offset;
			ldrDim->SetPosition(s_Pnt);

			bool finalRound = false;
			double sumOffsetDis = _ldrAltitude;

			while (true)
			{
				// <--Elevation layers have a budget limit. If exhausted, finalRound accepts the current point directly, guaranteeing the outer loop always terminates.
				if (sumOffsetDis > _ldrAltitude * 6)
					finalRound = true;

				success = SearchPositionForCurSrOrient(ldrDim, _SearchOrients[ldrDim->_srOrients[0]], _fixedDims);
				if (!success)
				{
					ldrDim->Translate(s_Pnt - ldrDim->_vertices[1] + _SearchOrients[ldrDim->_srOrients[1]] * offset * 2);
					success = SearchPositionForCurSrOrient(ldrDim, _SearchOrients[ldrDim->_srOrients[1]], _fixedDims);
				}

				if (success || finalRound)
					break;

				sumOffsetDis += _ldrAltitude;
				s_Pnt = s_Pnt + refVector * _ldrAltitude * 1.5;
				ldrDim->SetPosition(s_Pnt);
			}

			if (ldrDim->_srOrients[0] == HS::LU ||
				ldrDim->_srOrients[0] == HS::LD ||
				ldrDim->_srOrients[0] == HS::RU ||
				ldrDim->_srOrients[0] == HS::RD)
				break;
		}

		ldrDim->SetStatus(true);
		_fixedDims.insert(ldrDim);
	}
}

bool Arranger::SearchPositionForCurSrOrient(LdrDim* ldrDim, const Vector2d& dir, const std::set<Dim*>& fixedDims)
{
	if (!ldrDim || !dir.valid() || fixedDims.empty())
		return true;

	bool success = true;
	while (HasConflictedWithGeometricElements(ldrDim, fixedDims))
	{
		if (fabs((ldrDim->_vertices[1] - ldrDim->_vertices[0]) * dir) > _ldrAltitude * 8)
		{
			success = false;
			break;
		}

		ldrDim->Translate(dir * _srDist); // Advance by the configured search step. [Rectified/Added by Leo_GPT 2026/11/20]
	}

	if (success)						  // for the purpose to make a gap between LdrDim objects.
		ldrDim->Translate(dir * _ldrAltitude * 0.5);

	return success;
}

bool Arranger::HasConflictedWithGeometricElements(LdrDim* ldrDim, const std::set<Dim*>& fixedDims)
{
	if (fixedDims.empty() || !ldrDim)
		return false;

	LdrDim* otherLdrDim = nullptr;
	LenDim* otherLenDim = nullptr;
	for (auto dim : fixedDims)
	{
		if (!dim || dim == ldrDim)
			continue;

		if (ldrDim->GetAnnoBox().Intersect(_partBox))
			return true;

		if (otherLdrDim = dynamic_cast<LdrDim*>(dim))
		{
			if (ldrDim->GetAnnoBox().Intersect(otherLdrDim->GetAnnoBox()) ||
				ldrDim->HasIntersectWith(otherLdrDim))
				return true;
		}

		if (otherLenDim = dynamic_cast<LenDim*>(dim))
		{
			if (ldrDim->GetAnnoBox().Intersect(otherLenDim->GetAnnoBox()))
				return true;

			for (auto extra : otherLenDim->GetExtraBoxes())
			{
				if (!extra.Valid())
					continue;

				if (ldrDim->GetAnnoBox().Intersect(extra))
					return true;
			}

			for (auto line : otherLenDim->_lines)
			{
				if (!line.Valid())
					continue;

				if (ldrDim->GetAnnoBox().IntersectWithLine(line))
					return true;		
			}
		}
	}

	return false;
}

} // namespace HS
