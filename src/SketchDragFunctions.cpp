/*
======================================================================
wdbtool.c

The Windows debug version of tool.c.  Writes a trace of each callback
call to a text file.  Only one of these (wdbtool.c, tool.c) should be
included in your project, not both.

Ernie Wright  5 Jul 01
====================================================================== */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <lwserver.h>
#include "sketchDrag.h"

#define TR_DONE      0
#define TR_HELP      1
#define TR_COUNT     2
#define TR_HANDLE    3
#define TR_ADJUST    4
#define TR_START     5
#define TR_DRAW      6
#define TR_DIRTY     7
#define TR_EVENT     8
#define TR_PANEL     9
#define TR_BUILD     10
#define TR_TEST      11
#define TR_END       12
#define TR_MDOWN     13
#define TR_MMOVE     14
#define TR_MUP       15
#define TR_PANSET    16
#define TR_PANGET    17


int SketchDrag_Count(SketchDragTool *tool,LWToolEvent *event)
{
	return 0;
}


LWError SketchDrag_Build (SketchDragTool	*tool,MeshEditOp *op)
{
	EDError	err;
	//trace( TR_BUILD, tool, NULL );
	err = SketchDrag(op, tool);
	tool->update = LWT_TEST_ACCEPT;

    tool->worldCurvePoints.clear();
    tool->projCurvePoints.clear();
    tool->dirty = 1;
    
	return (err ? "Failed" : NULL);
}

int SketchDrag_Test (SketchDragTool *tool)
{
	//trace( TR_TEST, tool, NULL );
	return tool->update;
}


void SketchDrag_End (SketchDragTool *tool,int keep)
{
	tool->update = LWT_TEST_NOTHING;
}


const char *SketchDrag_Help (SketchDragTool *tool, LWToolEvent *event)
{
	return "Click on the viewport and drag to make a path.";
}

int SketchDrag_Down (SketchDragTool *tool,LWToolEvent *event)
{
	if (!tool->active) tool->active = 1;	
    
    tool->planeNormal.setValues(event->az[0], event->az[1], event->az[2]);
    tool->planeNormal.normalize();
    
    tool->portAxis = event->portAxis;
    
    tool->worldCurvePoints.clear();
    tool->projCurvePoints.clear();
    Vector3F worldVec, projVec;
    worldVec.setValues(event->posRaw);
    tool->worldCurvePoints.push_back(worldVec);
    
    tool->numberOfCurvePoints = 1;
    
    projVec.setValues(0,0,0);
    tool->projCurvePoints.push_back(projVec);
    
    tool->ax.setValues(event->ax[0], event->ax[1], event->ax[2]);
    tool->ay.setValues(event->ay[0], event->ay[1], event->ay[2]);
    tool->ay.normalize(); tool->ax.normalize();
    
	return 1;
}	

void SketchDrag_Move (SketchDragTool	*tool,LWToolEvent *event)
{
    Vector3F worldVec, projVec;
    worldVec.setValues(event->posRaw);
    
    projVec = worldVec - tool->worldCurvePoints[0];
    projVec.setValues(projVec * tool->ax, projVec * tool->ay, 0);
    
    if ((projVec - tool->projCurvePoints[tool->numberOfCurvePoints-1]).length() > 0.2)
    {
        tool->projCurvePoints.push_back(projVec);
        tool->worldCurvePoints.push_back(worldVec);
        tool->numberOfCurvePoints++;
    }
    
 	tool->dirty = 1;
}

void SketchDrag_Up (SketchDragTool *tool,LWToolEvent *event)
{
	tool->update = LWT_TEST_UPDATE;
    tool->dirty = 0;
}


void SketchDrag_Event (SketchDragTool *tool,int code)
{
	switch (code) {
	    case LWT_EVENT_DROP:
			tool->active = 0;
			tool->dirty = 1;
			tool->update = LWT_TEST_REJECT;
			break;

	    case LWT_EVENT_RESET:
			tool->active = 0;
			tool->dirty = 1;
			tool->update = LWT_TEST_REJECT;
			break;

	    case LWT_EVENT_ACTIVATE:
			tool->active = 0;
			tool->dirty = 0;
			break;
	}
}


void SketchDrag_Draw(SketchDragTool *tool, LWWireDrawAccess *draw)
{
	if (!tool->active) return;
    
    if (tool->portAxis != draw->axis)
    {
      	tool->dirty = 0;
        return;
    }

	if (tool->worldCurvePoints.size() < 2)
		return;
    
    LWFVector pt;
    pt[0] = (float)tool->worldCurvePoints[0].x();
    pt[1] = (float)tool->worldCurvePoints[0].y();
    pt[2] = (float)tool->worldCurvePoints[0].z();
    draw->moveTo(draw->data, pt, LWWIRE_SOLID);
    
    for (int i = 1; i < tool->worldCurvePoints.size(); i++)
    {
        pt[0] = (float)tool->worldCurvePoints[i].x();
        pt[1] = (float)tool->worldCurvePoints[i].y();
        pt[2] = (float)tool->worldCurvePoints[i].z();
        draw->lineTo(draw->data, pt, LWWIRE_ABSOLUTE);
    }
    
  	tool->dirty = 0;
}

int SketchDrag_Dirty(SketchDragTool *tool )
{
	return tool->dirty ? LWT_DIRTY_WIREFRAME | LWT_DIRTY_HELPTEXT : 0;
}


void SketchDrag_Done (SketchDragTool	*tool)
{
	if (tool)	delete tool;
}

