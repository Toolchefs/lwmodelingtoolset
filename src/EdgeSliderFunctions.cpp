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
#include "edgeSlider.h"
#include "sharedGlobals.h"

/* this is where the trace is written */

#define TR_FILENAME  "/Users/piddu/Documents/LW_Development/EdgeSlider/trace.txt"

//OldHandlePosition
LWDVector oldHpos, oldAllPos[19];

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


int EdgeSlider_Count(EdgeSliderTool *tool,LWToolEvent *event)
{
	return 0;
}


LWError EdgeSlider_Build (EdgeSliderTool	*tool,MeshEditOp *op)
{
	EDError	err;
	//trace( TR_BUILD, tool, NULL );
	err = EdgeSlider(op, tool);
	tool->update = LWT_TEST_NOTHING;

	return (err ? "Failed" : NULL);
}

int EdgeSlider_Test (EdgeSliderTool *tool)
{
	//trace( TR_TEST, tool, NULL );
	return tool->update;
}


void EdgeSlider_End (EdgeSliderTool *tool,int keep)
{
	tool->update = LWT_TEST_NOTHING;
}


const char *EdgeSlider_Help (EdgeSliderTool *tool, LWToolEvent *event)
{
	return "Click on the viewport and drag. Left click for selection / Right click looks for loops.";
}

int EdgeSlider_Down (EdgeSliderTool *tool,LWToolEvent *event)
{
	if (!tool->active) tool->active = 1;	

	//trace( TR_MDOWN, tool, NULL );
	tool->loopMode = (event->flags & LWTOOLF_ALT_BUTTON);
	tool->update = LWT_TEST_UPDATE;
    
    tool->edgeSlideFactor = 0;
    
	return 1;
}	

void EdgeSlider_Move (EdgeSliderTool	*tool,LWToolEvent *event)
{
	tool->dirty = 1;
    
	tool->edgeSlideFactor = double(event->dx) / 200.0;
    if (tool->edgeSlideFactor > 1)
        tool->edgeSlideFactor = 1;
    else if (tool->edgeSlideFactor < -1)
        tool->edgeSlideFactor = -1;
	
	tool->update = LWT_TEST_UPDATE;
}

void EdgeSlider_Up (EdgeSliderTool *tool,LWToolEvent *event)
{
	tool->loopMode = false;
	tool->update = LWT_TEST_ACCEPT;
	tool->dirty = 0;
}


void EdgeSlider_Event (EdgeSliderTool *tool,int code)
{
	switch (code) {
	    case LWT_EVENT_DROP:		
			tool->resetTool();
			tool->active = 0;
			tool->dirty = 1;
			tool->update = LWT_TEST_REJECT;
			break;

	    case LWT_EVENT_RESET:
			tool->resetTool();
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


void EdgeSlider_Draw(EdgeSliderTool *tool, LWWireDrawAccess *draw)
{
	if (!tool->active) return;
	//trace(TR_DRAW, tool, NULL);
	
    if (tool->loopMode)
	{
		LWFVector pos;
		int size = tool->cachedEdgeForDrawing.size();
		for (int i = 0; i < size; ++i)
		{
            tool->loopPoints[tool->cachedEdgeForDrawing[i].p1].pos.getValues(pos);
            draw->moveTo(draw->data, pos, LWWIRE_SOLID);
			
			tool->loopPoints[tool->cachedEdgeForDrawing[i].p2].pos.getValues(pos);
			draw->lineTo(draw->data, pos, LWWIRE_SOLID);
		}
	}
	
	tool->dirty = 0;
}


int EdgeSlider_Dirty(EdgeSliderTool *tool )
{
	return tool->dirty ? LWT_DIRTY_WIREFRAME | LWT_DIRTY_HELPTEXT : 0;
}


void EdgeSlider_Done (EdgeSliderTool	*tool)
{
	if (tool)
		delete tool;
}

