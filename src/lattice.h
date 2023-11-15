#include <lwmeshedt.h>
#include <lwserver.h>
#include <lwxpanel.h>
#include <lwdyna.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <lwmeshes.h>
#include <lwtxtr.h>
#include <lwtxtred.h>
#include <lwmath.h>
#include <lwmodtool.h>
#include <lwrender.h>
#include "sharedGlobals.h"
#include <tbb/tbb.h>

#include <vector>
#include <set>
#include <map>

typedef struct st_PointCache
{
    std::vector <float> weights;
    LWPntID id;
    LWDVector pos;
    LWDVector destPos;
    int layerNum;
    
    int minX, maxX, minY, maxY, minZ, maxZ;
    double sLocal, tLocal, uLocal;
    
} PointCache;

typedef struct st_SymmPoint
{
    LWPntID id;
    int pcIndex;
    int layerNum;
} SymmPoint;

typedef struct st_LatticeTool
{
	int active,update, dirty;
    
    int x_div, y_div, z_div;
    int latticeDivControlsEnabled;
    int showWires;
    double handleRadius;
    int interpolation;
    
    bool togglingSelection;
    bool addingSelection;
    
    int majorDraggingAxis;
    
    int numpoints;

    Vector3F minBBox, maxBBox;
    std::vector<Vector3F> ctlPoints;
    std::set<int> selection;
    std::map<int, Vector3F> initialPositions;
    
    bool justActivated;
    std::set<LWPntID> points;
    
    bool hasCache;
    std::vector<PointCache> pointCache;
    
    std::vector<SymmPoint> symmCache;
    
    bool multipleSelection;
    
    // we need to keep track of this...
    double pixelScale;
    
    //for drawing the multiple selection box and managing points movement
    int clickedAxis;
    Vector3F initialDraggingPosition, currentDraggingPosition, draggingAY, draggingAX;
    
    void refreshLatticeCtlPoints();
    int getIndex(int x, int y, int z);
    bool isX0Handle(int index);
    
} LatticeTool;


class LatticeTBBData
{
public:
    
	struct ThreadData
	{
        LatticeTool *tool;
	};
    
    LatticeTBBData(){};
    
	LatticeTBBData(LatticeTool *tool)
	{
        m_data.tool = tool;
	}
    
	void operator()( const tbb::blocked_range<size_t>& r ) const;
    
private:
    
    struct ThreadData m_data;
};

/* global functions typedef function */

int Lattice(MeshEditOp *op, LatticeTool *);

LWXPanelID Lattice_Panel(LatticeTool *tool );

int Lattice_Count(LatticeTool *tool,LWToolEvent *event);
LWError Lattice_Build (LatticeTool	*tool,MeshEditOp *op);
int Lattice_Test (LatticeTool *tool);
void Lattice_End (LatticeTool *tool,int keep);
const char *Lattice_Help (LatticeTool *tool, LWToolEvent *event);
int Lattice_Down (LatticeTool *tool,LWToolEvent *event);
void Lattice_Move (LatticeTool	*tool,LWToolEvent *event);
void Lattice_Up (LatticeTool *tool,LWToolEvent *event);
void Lattice_Event (LatticeTool *tool,int code);
void Lattice_Draw(LatticeTool *tool, LWWireDrawAccess *draw);
int Lattice_Dirty(LatticeTool *tool );
void Lattice_Done (LatticeTool	*tool);
