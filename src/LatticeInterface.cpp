//////////////Add Sphere-Dented-Ellispoid
//////////////Add push-pull


#include "lattice.h"
#include "sharedGlobals.h"
#include <stdlib.h>
#include <string.h>
#include <lwmodtool.h>

#include <lwvparm.h>


#define GRAD_PARM_NUM_PARAMDESC 2

LWXPanelID latticePanel;
LatticeTool *latticeTl;

enum { X_DIV = 0x8001, Y_DIV, Z_DIV, ID_STARTENABLE, ID_SHOWWIRES, ID_HANDLERADIUS, ID_INTERPOLATION};

static void *Get(LatticeTool *tool, unsigned long vid)
{
	switch (vid)
	{
        case X_DIV:	return &tool->x_div;
		case Y_DIV:	return &tool->y_div;
		case Z_DIV:	return &tool->z_div;
        case ID_STARTENABLE:	return &tool->latticeDivControlsEnabled;
        case ID_SHOWWIRES: return &tool->showWires;
        case ID_HANDLERADIUS: return &tool->handleRadius;
        case ID_INTERPOLATION: return &tool->interpolation;
		default:	return NULL;			
	}
}

static int Set( LatticeTool *tool, unsigned long vid, void *value )
{
   switch ( vid )
   {
       case X_DIV:
           tool->x_div = *(int *)value;
           tool->refreshLatticeCtlPoints();
           break;
       
       case Y_DIV:
           tool->y_div = *(int*)value;
           tool->refreshLatticeCtlPoints();
           break;
           
       case Z_DIV:
           tool->z_div = *(int*)value;
           tool->refreshLatticeCtlPoints();
           break;
           
       case ID_SHOWWIRES:
           tool->showWires = *(int*)value;
           break;
           
       case ID_HANDLERADIUS:
           tool->handleRadius = *(double *)value;
           break;
        
       case ID_INTERPOLATION:
           tool->interpolation = *(int*)value;
           tool->update = LWT_TEST_UPDATE;
           break;
           
       default:
		   return LWXPRC_NONE;
   }

	tool->dirty = 1;

	xpanf->viewRefresh(latticePanel);
	return LWXPRC_DRAW;
}


LWXPanelID Lattice_Panel( LatticeTool *tool )
{
    static char *interpolations[] = {"Linear", "Bezier", NULL};
    
	static LWXPanelControl ctl[] = {
      { X_DIV,	"X Division",    "iSliderText"  },
      { Y_DIV,	"Y Division",    "iSliderText"  },
      { Z_DIV,	"Z Division",    "iSliderText"  },
      { ID_INTERPOLATION,	"Interpolation", "iChoice"},
      { ID_HANDLERADIUS,	"Handle Radius", "float"},
      { ID_SHOWWIRES,	"Show wires", "iBoolean"},
	  { 0 }
	};

	static LWXPanelDataDesc cdata[] = {
      { X_DIV,	"X Division",    "integer"  },
      { Y_DIV,	"Y Division",    "integer"  },
      { Z_DIV,	"Z Division",    "integer"  },
      { ID_INTERPOLATION,	"Interpolation", "integer"},
      { ID_STARTENABLE, "StartEnable", "integer"},
      { ID_HANDLERADIUS,"Handle Radius", "float"},
      { ID_SHOWWIRES,	"Show wires", "integer"},
	  { 0 }
	};

	LWXPanelHint hints[] = {
      XpLABEL( 0, "Lattice Tool" ),
      XpMIN(X_DIV, 2),
	  XpMIN(Y_DIV, 2),
      XpMIN(Z_DIV, 2),
      XpMAX(X_DIV, 20),
      XpMAX(Y_DIV, 20),
      XpMAX(Z_DIV, 20),
      XpSTRLIST( ID_INTERPOLATION, interpolations),
      XpDIVADD(Z_DIV),
      XpMIN(ID_HANDLERADIUS, 1),
      XpENABLEMSG_ (ID_STARTENABLE, "Div controls get disabled after you start modifying the lattice."), XpH(X_DIV), XpH(Y_DIV), XpH(Z_DIV), XpEND,
	  XpEND
	};

	
	latticeTl = tool;
	
	latticePanel = xpanf->create( LWXP_VIEW, ctl );
 	if ( !latticePanel ) return NULL;

 	xpanf->describe( latticePanel, cdata, (get_function)Get, (set_function)Set );
 	xpanf->hint( latticePanel, 0, hints );

 	return latticePanel;
}