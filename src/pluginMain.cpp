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
#include <lwserver.h>
#include <lwpanel.h>
#include <fstream>

#ifdef __APPLE__
#include <CoreFoundation/CFBundle.h>
#include <ApplicationServices/ApplicationServices.h>
#else
#define _WINSOCKAPI_ // stops windows.h including winsock.h
#include <windows.h>
#endif

#include "lattice.h"
#include "edgeSlider.h"
#include "sketchDrag.h"
#include "sharedGlobals.h"

LWMessageFuncs *msgf;
LWStateQueryFuncs *query;
LWXPanelFuncs *xpanf;
unsigned int serialno;


int activate(long	version, GlobalFunc	*global, LWMeshEditTool	*local, void	*serverData)
{
	if (version != LWMESHEDITTOOL_VERSION)
		return AFUNC_BADVERSION;

	if (!msgf)
	{
		msgf = static_cast<LWMessageFuncs*>(global(LWMESSAGEFUNCS_GLOBAL, GFUSE_TRANSIENT));
		if (!msgf) return AFUNC_BADGLOBAL;
	}

	if (!xpanf)
	{
		xpanf = static_cast<LWXPanelFuncs*>(global(LWXPANELFUNCS_GLOBAL, GFUSE_TRANSIENT));
		if (!xpanf) return AFUNC_BADGLOBAL;
	}

	if (!query)
	{
		query = static_cast<LWStateQueryFuncs*>(global( LWSTATEQUERYFUNCS_GLOBAL, GFUSE_TRANSIENT ));
		if (!msgf) return AFUNC_BADGLOBAL;
	}
    
    void *sys = global( LWSYSTEMID_GLOBAL, GFUSE_TRANSIENT );
    if (!sys)
        return AFUNC_BADGLOBAL;
	unsigned int sysid = PTR2UINT(sys);
	serialno = sysid & LWSYS_SERIALBITS;

	return AFUNC_OK;
}


XCALL_(int)
LatticeActivate (long	version,GlobalFunc	*global,LWMeshEditTool	*local,void	*serverData)
{
	int result = activate(version, global, local, serverData);
	if (result != AFUNC_OK)
		return result;    
    
    LatticeTool *tool = new LatticeTool;
	if (!tool) return AFUNC_BADGLOBAL;
    
    tool->x_div = 3;
    tool->y_div = 3;
    tool->z_div = 3;
    
	tool->latticeDivControlsEnabled = 1;
    tool->showWires = 1;
    tool->handleRadius = 5.0;
    tool->multipleSelection = false;
    tool->pixelScale = 1.0;
    tool->clickedAxis = -2;
    tool->hasCache = false;
    tool->interpolation = 1;
    
    tool->justActivated = true;
    tool->update = LWT_TEST_UPDATE;
    
	local->instance   = tool;
	local->tool->done  = (done_function)Lattice_Done;
	local->tool->count = (count_function)Lattice_Count;
	local->tool->draw  = (draw_function)Lattice_Draw;
    local->tool->dirty = (dirty_function)Lattice_Dirty;
	local->tool->up	   = (up_function)Lattice_Up;
	local->tool->help  = (help_function)Lattice_Help;
	local->tool->move  = (move_function)Lattice_Move;
	local->tool->down  = (down_function)Lattice_Down;
	local->tool->panel = (panel_function)Lattice_Panel;
	local->tool->event = (event_function)Lattice_Event;
	local->build       = (build_function)Lattice_Build;
	local->test        = (test_function)Lattice_Test;
	local->end         = (end_function)Lattice_End;
    
	return AFUNC_OK;
}

XCALL_(int)
EdgeSliderActivate (long	version,GlobalFunc	*global,LWMeshEditTool	*local,void	*serverData)
{
	int result = activate(version, global, local, serverData);
	if (result != AFUNC_OK)
		return result;
	
	EdgeSliderTool *tool = new EdgeSliderTool;
	if (!tool)
		return AFUNC_OK;
	
	tool->resetTool();
	
	local->instance   = tool;
	local->tool->done  = (done_function)EdgeSlider_Done;
	local->tool->count = (count_function)EdgeSlider_Count;
	local->tool->draw  = (draw_function)EdgeSlider_Draw;
    local->tool->dirty = (dirty_function)EdgeSlider_Dirty;
	local->tool->up	   = (up_function)EdgeSlider_Up;
	local->tool->help  = (help_function)EdgeSlider_Help;
	local->tool->move  = (move_function)EdgeSlider_Move;
	local->tool->down  = (down_function)EdgeSlider_Down;
	local->tool->panel = (panel_function)EdgeSlider_Panel;
	local->tool->event = (event_function)EdgeSlider_Event;
	local->build       = (build_function)EdgeSlider_Build;
	local->test        = (test_function)EdgeSlider_Test;
	local->end         = (end_function)EdgeSlider_End;
    
    
	return AFUNC_OK;
}

XCALL_(int)
SketchDragActivate (long	version,GlobalFunc	*global,LWMeshEditTool	*local,void	*serverData)
{
	int result = activate(version, global, local, serverData);
	if (result != AFUNC_OK)
		return result;
	
	SketchDragTool *tool = new SketchDragTool;
	if (!tool)
		return AFUNC_OK;
    tool->mode = 1;
    tool->iterations = 3;
    tool->propagate = 0.6;
    
	local->instance   = tool;
	local->tool->done  = (done_function)SketchDrag_Done;
	local->tool->count = (count_function)SketchDrag_Count;
	local->tool->draw  = (draw_function)SketchDrag_Draw;
    local->tool->dirty = (dirty_function)SketchDrag_Dirty;
	local->tool->up	   = (up_function)SketchDrag_Up;
	local->tool->help  = (help_function)SketchDrag_Help;
	local->tool->move  = (move_function)SketchDrag_Move;
	local->tool->down  = (down_function)SketchDrag_Down;
	local->tool->panel = (panel_function)SketchDrag_Panel;
	local->tool->event = (event_function)SketchDrag_Event;
	local->build       = (build_function)SketchDrag_Build;
	local->test        = (test_function)SketchDrag_Test;
	local->end         = (end_function)SketchDrag_End;
    
	return AFUNC_OK;
}

static ServerTagInfo EdgeSliderSrvtag[] = {
	{ "tcEdgeSlider", SRVTAG_USERNAME | LANGID_USENGLISH },
	{ "Modify", SRVTAG_CMDGROUP },
	{ "Translate", SRVTAG_MENU },
	{ "tcEdgeSlider", SRVTAG_BUTTONNAME },
	{ "sedge", SRVTAG_ENABLE },
	{ "", 0 }
};

static ServerTagInfo LatticeSrvtag[] = {
	{ "tcLatticeModeler", SRVTAG_USERNAME | LANGID_USENGLISH },
	{ "Modify", SRVTAG_CMDGROUP },
	{ "Translate", SRVTAG_MENU },
	{ "tcLatticeModeler", SRVTAG_BUTTONNAME },
	{ "pnt", SRVTAG_ENABLE },
	{ "", 0 }
};

static ServerTagInfo SketchDragSrvtag[] = {
	{ "tcSketchDrag", SRVTAG_USERNAME | LANGID_USENGLISH },
	{ "Modify", SRVTAG_CMDGROUP },
	{ "Translate", SRVTAG_MENU },
	{ "tcSketchDrag", SRVTAG_BUTTONNAME },
    { "edge", SRVTAG_ENABLE },
	{ "", 0 }
};



ServerRecord ServerDesc[] = {
   { LWMESHEDITTOOL_CLASS, "tcLatticeModeler", (activate_function)LatticeActivate, LatticeSrvtag },
   { LWMESHEDITTOOL_CLASS, "tcEdgeSlider", (activate_function)EdgeSliderActivate, EdgeSliderSrvtag },
   { LWMESHEDITTOOL_CLASS, "tcSketchDrag", (activate_function)SketchDragActivate, SketchDragSrvtag },
   { NULL }
};


void *Startup (void)
{
    return (void *) 4;
}


void Shutdown (void *serverData)
{

}