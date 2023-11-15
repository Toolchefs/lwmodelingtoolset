//////////////Add Sphere-Dented-Ellispoid
//////////////Add push-pull


#include "edgeSlider.h"
#include <stdlib.h>
#include <string.h>
#include <lwmodtool.h>
#include <lwvparm.h>
#include "sharedGlobals.h"


#define GRAD_PARM_NUM_PARAMDESC 2

LWXPanelID edgeSliderPanel;
EdgeSliderTool *edgeSliderTl;

enum { ID_EDGE_SLIDING_FACTOR = 0x8001};

static void *Get(EdgeSliderTool *tool, unsigned long vid)
{
	switch (vid)
	{
		case ID_EDGE_SLIDING_FACTOR:	return &tool->edgeSlideFactor;
		default:	return NULL;			
	}
}

static int Set( EdgeSliderTool *tool, unsigned long vid, void *value )
{
   switch ( vid )
   {
     case ID_EDGE_SLIDING_FACTOR:
	 	   tool->edgeSlideFactor = *((double *)value);
           if (tool->edgeSlideFactor > 1)
               tool->edgeSlideFactor = 1;
           else if (tool->edgeSlideFactor < -1)
               tool->edgeSlideFactor = -1;
           
           tool->update = LWT_TEST_UPDATE;
	 	   break;
	 default:
		   return LWXPRC_NONE;
   }

	tool->dirty = 1;

	xpanf->viewRefresh(edgeSliderPanel);
	return LWXPRC_DRAW;
}


LWXPanelID EdgeSlider_Panel( EdgeSliderTool *tool )
{
    //static char *modes[] = {"Vector", "Radial", NULL};
    
	static LWXPanelControl ctl[] = {
      { ID_EDGE_SLIDING_FACTOR, "Factor", "percent"},
	  { 0 }
	};

	static LWXPanelDataDesc cdata[] = {
      { ID_EDGE_SLIDING_FACTOR, "Factor", "percent"},
	  { 0 }
	};

	LWXPanelHint hints[] = {
      XpMIN(ID_EDGE_SLIDING_FACTOR, -1),
      XpMAX(ID_EDGE_SLIDING_FACTOR, 1),
      //XpSTRLIST( ID_INTERACTIVE_MODE, modes),
	  XpEND
	};

	edgeSliderTl = tool;
	
	edgeSliderPanel = xpanf->create( LWXP_VIEW, ctl );
 	if ( !edgeSliderPanel ) return NULL;

 	xpanf->describe( edgeSliderPanel, cdata, (get_function)Get, (set_function)Set );
 	xpanf->hint( edgeSliderPanel, 0, hints );

 	return edgeSliderPanel;
}