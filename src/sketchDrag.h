#include <lwmeshedt.h>
#include <lwserver.h>
#include <lwxpanel.h>
#include <lwdyna.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <lwmath.h>
#include <math.h>
#include <lwmeshes.h>
#include <lwtxtr.h>
#include <lwtxtred.h>
#include <lwmodtool.h>
#include <lwrender.h>
#include "sharedGlobals.h"

#include <vector>
#include <set>
#include <map>


typedef struct st_SketchDragTool
{
	int active,update, dirty;
    int numberOfCurvePoints;
    std::vector<Vector3F> worldCurvePoints;
    std::vector<Vector3F> projCurvePoints;
    
    int mode;
    int iterations;
    double propagate;
    
    int portAxis;
    Vector3F planeNormal, ax, ay;
} SketchDragTool;


int SketchDrag(MeshEditOp *op, SketchDragTool *);

LWXPanelID SketchDrag_Panel(SketchDragTool *tool );

int SketchDrag_Count(SketchDragTool *tool,LWToolEvent *event);
LWError SketchDrag_Build (SketchDragTool	*tool,MeshEditOp *op);
int SketchDrag_Test (SketchDragTool *tool);
void SketchDrag_End (SketchDragTool *tool,int keep);
const char *SketchDrag_Help (SketchDragTool *tool, LWToolEvent *event);
int SketchDrag_Down (SketchDragTool *tool,LWToolEvent *event);
void SketchDrag_Move (SketchDragTool	*tool,LWToolEvent *event);
void SketchDrag_Up (SketchDragTool *tool,LWToolEvent *event);
void SketchDrag_Event (SketchDragTool *tool,int code);
void SketchDrag_Draw(SketchDragTool *tool, LWWireDrawAccess *draw);
int SketchDrag_Dirty(SketchDragTool *tool );
void SketchDrag_Done (SketchDragTool	*tool);
int SketchDrag(MeshEditOp *op, SketchDragTool *);

LWXPanelID SketchDrag_Panel(SketchDragTool *tool );

