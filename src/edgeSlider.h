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
#include <vector>
#include <map>
#include <set>
#include "sharedGlobals.h"
#include <tbb/tbb.h>

typedef struct st_EdgeSegment
{
    LWPntID p1;
	LWPntID p2;
} EdgeSegment;

typedef struct st_EdgePoint
{
    LWPntID id;
    LWPntID targetPt1;
    LWPntID targetPt2;
    
    Vector3F pos, sourcePos;
    
    LWEdgeID edge;
} EdgePoint;

typedef struct st_Edge
{
    LWEdgeID id;
    LWPntID p1;
    LWPntID p2;
    
    LWPolID pol1;
    LWPolID pol2;
    
    Vector3F pos;
    Vector3F directionVector;
}Edge;


typedef struct st_EdgeSliderTool
{
	int active, update, dirty;
    
    bool loopMode;
	double edgeSlideFactor;

    bool isFirst;
	Vector3F firstDirection;
	
	void resetTool();
	void clearTempData();
    
    std::set<LWEdgeID> edges;
    std::set<LWPntID> edgePointsSet;
	std::map<LWPntID, EdgePoint> edgePoints;
    
    std::set<LWPntID> loopPointsSet;
    std::map<LWEdgeID, Edge> loopEdges;
    std::map<LWPntID, EdgePoint> loopPoints;
    
    std::vector<EdgeSegment> cachedEdgeForDrawing;
    
    std::map<LWPntID, LWPntID> symmMap;
    
} EdgeSliderTool;

int EdgeSlider(MeshEditOp *op, EdgeSliderTool *);

LWXPanelID EdgeSlider_Panel(EdgeSliderTool *tool );

int EdgeSlider_Count(EdgeSliderTool *tool,LWToolEvent *event);
LWError EdgeSlider_Build (EdgeSliderTool	*tool,MeshEditOp *op);
int EdgeSlider_Test (EdgeSliderTool *tool);
void EdgeSlider_End (EdgeSliderTool *tool,int keep);
const char *EdgeSlider_Help (EdgeSliderTool *tool, LWToolEvent *event);
int EdgeSlider_Down (EdgeSliderTool *tool,LWToolEvent *event);
void EdgeSlider_Move (EdgeSliderTool	*tool,LWToolEvent *event);
void EdgeSlider_Up (EdgeSliderTool *tool,LWToolEvent *event);
void EdgeSlider_Event (EdgeSliderTool *tool,int code);
void EdgeSlider_Draw(EdgeSliderTool *tool, LWWireDrawAccess *draw);
int EdgeSlider_Dirty(EdgeSliderTool *tool );
void EdgeSlider_Done (EdgeSliderTool	*tool);

