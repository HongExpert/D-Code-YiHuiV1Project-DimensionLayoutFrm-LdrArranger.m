#pragma once
#include "LdrArranger.h"
#include <cstddef>
#include <utility>
#include <vector>
#include <map>
#include <set>
#include <algorithm>
#include <cmath>
#include "Leader/LenDim.h"
#include "Leader/LdrDim.h"

// Read the existing project configuration through the Arranger interface. [Rectified/Added by Leo_GPT 2026/11/20]
class INIReader;

// Preserve the upstream algorithm; isolate its types and exports in HS for the CAA module. [Rectified/Added by Leo 2026/11/19]
namespace HS
{

class ExportedByLdrArranger Arranger
{
public:
	typedef std::pair<size_t, size_t> OverlapPair;

	static Arranger& Instance();

	// Load SrAngle in degrees and SrDist in drawing-view units. [Rectified/Added by Leo_GPT 2026/11/20]
	void LoadSearchParameters(const INIReader& config);

	void Setup(double ldrAltitude, const std::string& viewName, const BBox& partBox, const std::vector<Dim*>& dims);

	void OutputData() const;

	// Return current elbow coordinates; keys refer to caller-owned inputs and are never transferred or deleted. [Rectified/Added by Leo 2026/11/19]
	std::map<const LdrDim*, Point> GetLdrDimElbowPositions() const;

	void CalVerticalRegionOrder(HS::Region region, std::vector<LdrDim*>& ldrDims);

	void PerformLocatingLdrDims();

private:
	Arranger();
	~Arranger();

	Arranger(const Arranger&) = delete;
	Arranger& operator=(const Arranger&) = delete;

	/*Essential methods to initialize necessary data*/
	void Initialize();

	void ClassifyLenDims();

	void InitSearchOrients();											//initialize member _SearchOrients, which determines where current LdrDim object moves along _partBox

	std::map<HS::Region, std::vector<LdrDim*>> CollectLdrDimGroups();

	void CollectLdrDimsOnlyForQuadrant();								//classify the LdrDim objects into only 4 quadrants.

	void InitLdrDimSearchOrients(std::map<HS::Region, std::vector<LdrDim*>>& ldrGroups);

	/*this section is used to build the Composite boxes for 4 direction LenDim objects*/
	void InitLenDimCompositeBoxes();    //deprecated

	BBox CalEntireBoxToOneOrientLenDims(HS::Orient orient, std::vector<LenDim*>& LenDims);

	Point FindVerticalSrPointOnPartBox(HS::Orient side, const Vector2d& dir);

	/*adjust the order of LdrDim objext in std::vector, which helps to determine which LdrDim object
	is to be firstly located.*/
	void CalSrOrientOrder(HS::SrOrient srOrient, std::vector<LdrDim*>& ldrDims);

	/*this section processes the case where the program allows LdrDim objects to be located between LenDim objects.*/
	void SearchLdrDimPositions(HS::SrOrient srOrient, std::vector<LdrDim*>& ldrDims);

	/*adjust the order of LdrDim objext in std::vector, this method is similar to CalSrOrientOrder,
	but this method only processes the case where all LdrDim objects should be located along the vertical orientation.*/
	void CalVerticalSrOrientOrder(HS::SrOrient srOrient, std::vector<LdrDim*>& ldrDims);

	void SearchLdrDimVerticalPositions(std::vector<LdrDim*>& ldrDims);

	bool SearchPositionForCurSrOrient(			
		LdrDim* ldrDim,
		const Vector2d& dir,
		const std::set<Dim*>& fixedDims);

	bool HasConflictedWithGeometricElements(LdrDim* ldrDim, const std::set<Dim*>& fixedDims);
	/*<-------------------------------END--------------------------------->*/

private:
	BBox _partBox;  
	std::vector<Dim*> _dims;
	std::set<Dim*> _fixedDims;
	std::string _viewName;

	std::map<HS::SrOrient, Vector2d> _SearchOrients;
	std::map<HS::Orient, std::vector<LenDim*>> _LenDimsWithOrients;
	std::map<HS::SrOrient, std::vector<LdrDim*>> _LdrDimsWithSrOrients;
	std::map<HS::Region, std::vector<LdrDim*>> _LdrDimsClassifiedViaFourQuadrants;

	double _ldrAltitude;

	// Keep the configured angle in radians for tan() and use the step in view units. [Rectified/Added by Leo_GPT 2026/11/20]
	double _srAngle;
	double _srDist;
};
} // namespace HS
