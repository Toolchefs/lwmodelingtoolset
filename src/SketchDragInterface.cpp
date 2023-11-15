//////////////Add Sphere-Dented-Ellispoid
//////////////Add push-pull


#include "sketchDrag.h"
#include <stdlib.h>
#include <string.h>
#include <lwmodtool.h>
#include <lwvparm.h>


LWXPanelID sketchDragPanel;

enum { MODE = 0x8001, ITERATION, PROPAGATE};

static void *Get(SketchDragTool *tool, unsigned long vid)
{
	switch (vid)
	{
        case MODE:	return &tool->mode;
        case ITERATION: return &tool->iterations;
        case PROPAGATE: return &tool->propagate;
		default:	return NULL;			
	}
}

static int Set( SketchDragTool *tool, unsigned long vid, void *value )
{
   switch ( vid )
   {
       case MODE:
           tool->mode = *(int *)value;
           break;
           
       case ITERATION:
           tool->iterations = *(int *)value;
           break;
           
       case PROPAGATE:
           tool->propagate = *(double *)value;
           if (tool->propagate == 0)
               tool->propagate = 0.01;
           break;
           
       default:
		   return LWXPRC_NONE;
   }

	tool->dirty = 1;

	xpanf->viewRefresh(sketchDragPanel);
	return LWXPRC_DRAW;
}


LWXPanelID SketchDrag_Panel( SketchDragTool *tool )
{
    static char *modes[] = {"Spread", "Closest", NULL};
    
	static LWXPanelControl ctl[] = {
        {MODE, "Mode:", "iChoice"},
        {PROPAGATE, "Propagate:", "percent"},
        {ITERATION, "Iterations Allowed:", "iSliderText"},
        { 0 }
	};

	static LWXPanelDataDesc cdata[] = {
        {MODE, "Spread Lenght:", "integer"},
        {PROPAGATE, "Propagate:", "float"},
        {ITERATION, "Iteration Allowed:", "integer"},
        { 0 }
	};

	LWXPanelHint hints[] = {
        XpSTRLIST(MODE, modes),
        XpDIVADD(MODE),
        XpMIN(ITERATION, 0),
        XpMAX(ITERATION, 10),
        XpMIN(PROPAGATE, 0),
        XpEND
	};
	
	sketchDragPanel = xpanf->create( LWXP_VIEW, ctl );
 	if ( !sketchDragPanel ) return NULL;

 	xpanf->describe( sketchDragPanel, cdata, (get_function)Get, (set_function)Set );
 	xpanf->hint( sketchDragPanel, 0, hints );

 	return sketchDragPanel;
}