//
//  LatticeFunctions.cpp
//  LatticeModeler
//
//  Created by Daniele Federico on 24/03/15.
//
//

#include "lattice.h"

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


int Lattice_Count(LatticeTool *tool,LWToolEvent *event)
{
	return tool->x_div * tool->y_div * tool->z_div;
}


LWError Lattice_Build (LatticeTool	*tool,MeshEditOp *op)
{
	EDError	err;
	//trace( TR_BUILD, tool, NULL );
	err = Lattice(op, tool);
    tool->update = LWT_TEST_NOTHING;
    
	return (err ? "Failed" : NULL);
}

int Lattice_Test (LatticeTool *tool)
{
	//trace( TR_TEST, tool, NULL );
	return tool->update;
}


 void Lattice_End (LatticeTool *tool,int keep)
{
  	tool->update = LWT_TEST_NOTHING;
}


 const char *Lattice_Help (LatticeTool *tool, LWToolEvent *event)
{
	return "Click on the viewport and drag to make a path.";
}


void singleSelectInPerspective(LatticeTool *tool, LWToolEvent *event, std::set<int> &newSelection)
{
    int index, selectedIndex = -1;
    double dot;
    bool hasDot = false;
    
    Vector3F clickedPosition(event->posRaw[0], event->posRaw[1], event->posRaw[2]);
    Vector3F axis(event->axis[0], event->axis[1], event->axis[2]);
    Vector3F vec;
    double sqrHandleRadius = tool->handleRadius * tool->handleRadius * tool->pixelScale * tool->pixelScale;
    
    
    for (int x=0; x < tool->x_div; ++x)
        for (int y=0; y < tool->y_div; ++y)
            for (int z=0; z < tool->z_div; ++z)
            {
                index = tool->getIndex(x, y, z);
                vec = clickedPosition - tool->ctlPoints[index];
                
                double b = vec * axis;
                double c = vec * vec - sqrHandleRadius;
                double discr = b * b - c;
                
                // the handle with greatest dot is the one closest to the camera ;)
                if ((!hasDot || b > dot) && discr >= 0)
                {
                    hasDot = true;
                    dot = b;
                    selectedIndex = index;
                }
                
            }
    
    if (selectedIndex != -1)
        newSelection.insert(selectedIndex);
}

void singleSelectInOrthogonal(LatticeTool *tool,LWToolEvent *event, std::set<int> &newSelection)
{
    Vector3F clickedPosition(event->posRaw[0], event->posRaw[1], event->posRaw[2]);
    Vector3F vec;
    
    double sqrHandleRadius = tool->handleRadius * tool->handleRadius * tool->pixelScale * tool->pixelScale;
    
    int index;
    bool isX = abs(event->axis[0]) == 1.0;
    bool isY = abs(event->axis[1]) == 1.0;
    bool isZ = abs(event->axis[2]) == 1.0;
    
    for (int x=0; x < tool->x_div; ++x)
        for (int y=0; y < tool->y_div; ++y)
            for (int z=0; z < tool->z_div; ++z)
            {
                index = tool->getIndex(x, y, z);
                vec = clickedPosition - tool->ctlPoints[index];
                
                if ((isX && (vec.y() * vec.y() + vec.z() * vec.z() <= sqrHandleRadius)) ||
                    (isY && (vec.x() * vec.x() + vec.z() * vec.z() <= sqrHandleRadius)) ||
                    (isZ && (vec.x() * vec.x() + vec.y() * vec.y() <= sqrHandleRadius)))
                    newSelection.insert(index);
            }
}

bool toolSelectionContainsNewSelection(LatticeTool *tool, std::set<int> newSelection)
{
    std::set<int>::iterator it;
    for (it=newSelection.begin(); it!=newSelection.end(); ++it)
    {
        if (tool->selection.find(*it) != tool->selection.end())
            return true;
    }
    
    return false;
}

void cacheSelectionPositions(LatticeTool *tool)
{
    std::set<int>::iterator it;
    for (it=tool->selection.begin(); it!=tool->selection.end(); ++it)
        tool->initialPositions[*it] = tool->ctlPoints[*it];
}

 int Lattice_Down (LatticeTool *tool,LWToolEvent *event)
{
	if (!tool->active) tool->active = 1;
    
    std::set<int> newSelection;
    if (event->portAxis == -1)
        singleSelectInPerspective(tool, event, newSelection);
    else
        singleSelectInOrthogonal(tool, event, newSelection);
    
    tool->clickedAxis = -2;
    tool->majorDraggingAxis = -1;
    
    tool->initialDraggingPosition.setValues(event->posRaw[0], event->posRaw[1], event->posRaw[2]);
    tool->currentDraggingPosition.setValues(event->posRaw[0], event->posRaw[1], event->posRaw[2]);
    
    tool->draggingAY.setValues(event->ay[0], event->ay[1], event->ay[2]);
    tool->draggingAX.setValues(event->ax[0], event->ax[1], event->ax[2]);
    
    tool->draggingAY.normalize(); tool->draggingAX.normalize();
    
    tool->initialPositions.clear();
    
    tool->togglingSelection = false;
    tool->addingSelection = false;
    
    //multiple selection
    if (newSelection.empty())
    {
        tool->togglingSelection = bool(event->flags & LWTOOLF_CTRL);
        tool->addingSelection = bool(event->flags & LWTOOLF_SHIFT);
        if (!tool->togglingSelection && !tool->addingSelection)
            tool->selection.clear();
        tool->clickedAxis = event->portAxis;
    }
    //dragging multiple selection
    else if (tool->multipleSelection && toolSelectionContainsNewSelection(tool, newSelection))
    {
        cacheSelectionPositions(tool);
    }
    else
    {
        tool->multipleSelection = false;
        tool->selection.swap(newSelection);
        
        cacheSelectionPositions(tool);
        
    }
    
	return 1;
}

int getAxisContrained(Vector3F move)
{
    float x = fabs(move.x());
    float y = fabs(move.y());
    float z = fabs(move.z());
    if (x > y && x > z)
        return 0;
    else if (y > z)
        return 1;
    else
        return 2;
}

void getConstrainedVector(int axis, Vector3F &move)
{
    switch (axis)
    {
        case 0:
            move.setY(0); move.setZ(0);
            break;
        case 1:
            move.setX(0); move.setZ(0);
            break;
        case 2:
            move.setX(0); move.setY(0);
            break;
    }
}

void Lattice_Move (LatticeTool	*tool,LWToolEvent *event)
{
    tool->currentDraggingPosition.setValues(event->posRaw[0], event->posRaw[1], event->posRaw[2]);
    
    //dragging selection
    if (!tool->selection.empty() && !tool->togglingSelection && !tool->addingSelection)
    {
        //disabling the division controls in the numeric panel
        tool->latticeDivControlsEnabled = 0;
        
        //changing the movement in screen space
        Vector3F diff = tool->currentDraggingPosition - tool->initialDraggingPosition;
        Vector3F move;
        if(event->flags&LWTOOLF_CONSTRAIN)
        {
            if(event->portAxis>=0)
            {
                if(event->flags&LWTOOLF_CONS_Y)
                    move = tool->draggingAY * (tool->draggingAY * diff);
                else if(event->flags&LWTOOLF_CONS_X)
                    move = tool->draggingAX * (tool->draggingAX * diff);
            }
            else
            {
                move = tool->draggingAY * (tool->draggingAY * diff) + tool->draggingAX * (tool->draggingAX * diff);
                if (tool->majorDraggingAxis == -1)
                    tool->majorDraggingAxis = getAxisContrained(move);
                getConstrainedVector(tool->majorDraggingAxis, move);
            }
        }
        else
            move = tool->draggingAY * (tool->draggingAY * diff) + tool->draggingAX * (tool->draggingAX * diff);
        
        Vector3F moveNoX(0, move.y(), move.z());
        
        for (std::set<int>::iterator it=tool->selection.begin(); it!=tool->selection.end(); ++it)
        {
            if (query->mode(LWM_MODE_SYMMETRY) && tool->isX0Handle(*it))
                tool->ctlPoints[*it] = tool->initialPositions[*it] + moveNoX;
            else
                tool->ctlPoints[*it] = tool->initialPositions[*it] + move;
        }
        
        tool->update = LWT_TEST_UPDATE;
    }
    tool->dirty = 1;
    
}

void multipleSelect(LatticeTool *tool,LWToolEvent *event)
{
    // calcola vetto
    Vector3F diff = tool->currentDraggingPosition - tool->initialDraggingPosition;
    double screenDot1 = tool->draggingAY * diff;
    double screenDot2 = tool->draggingAX * diff;
    
    int index;
    Vector3F vec;
    for (int x=0; x < tool->x_div; ++x)
        for (int y=0; y < tool->y_div; ++y)
            for (int z=0; z < tool->z_div; ++z)
            {
                index = tool->getIndex(x, y, z);
                vec = tool->ctlPoints[index] - tool->initialDraggingPosition;
                
                double dot1 = (vec * tool->draggingAY);
                double dot2 = (vec * tool->draggingAX);
                if ( ((screenDot1 > 0 && dot1 > 0 && dot1 < screenDot1) ||
                      (screenDot1 < 0 && dot1 < 0 && dot1 > screenDot1)) &&
                    ((screenDot2 > 0 && dot2 > 0 && dot2 < screenDot2) ||
                     (screenDot2 < 0 && dot2 < 0 && dot2 > screenDot2))
                    )
                {
                    if (tool->togglingSelection)
                    {
                        std::set<int>::iterator it = tool->selection.find(index);
                        if (it != tool->selection.end())
                        {
                            tool->selection.erase(it);
                            continue;
                        }
                    }

                    tool->selection.insert(index);
                }
                
            }
}

 void Lattice_Up (LatticeTool *tool,LWToolEvent *event)
{
    if (tool->selection.empty() || tool->togglingSelection || tool->addingSelection)
    {
        //try to get the selection from the selection box
        tool->currentDraggingPosition.setValues(event->posRaw[0], event->posRaw[1], event->posRaw[2]);
        multipleSelect(tool, event);
        tool->multipleSelection = !tool->selection.empty();
    }
    else
    {
        // if we only have one selected handle we unselect it
        // otherwise we keep the selection
        if (!tool->multipleSelection)
            tool->selection.clear();
    }
    
    tool->clickedAxis = -2;
    
    tool->dirty = 1;
}


 void Lattice_Event (LatticeTool *tool,int code)
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
            
        case LWT_EVENT_DEACTIVATE:
			tool->active = 0;
			tool->dirty = 0;
			break;
	}
}

void drawLine(LatticeTool *tool, LWWireDrawAccess *draw, const int index, LWFVector initialPos)
{
    LWFVector endPos;
    draw->moveTo(draw->data, initialPos, LWWIRE_DASH);
    tool->ctlPoints[index].getValues(endPos);
    draw->lineTo(draw->data, endPos, LWWIRE_ABSOLUTE);
}

 void Lattice_Draw(LatticeTool *tool, LWWireDrawAccess *draw)
{
    LWFVector pos;
    tool->pixelScale = draw->pxScale;
    
    for (int x=0; x < tool->x_div; ++x)
        for (int y=0; y < tool->y_div; ++y)
            for (int z=0; z < tool->z_div; ++z)
            {
                int index = tool->getIndex(x,y,z);
                tool->ctlPoints[index].getValues(pos);
                
                if (tool->selection.find(index) == tool->selection.end())
                    draw->moveTo(draw->data, pos, LWWIRE_DASH);
                else
                    draw->moveTo(draw->data, pos, LWWIRE_SOLID);
                
                draw->circle(draw->data, tool->handleRadius * draw->pxScale, LWWIRE_ABSOLUTE);
                if (tool->showWires)
                {
                    if (x < tool->x_div - 1)
                        drawLine(tool, draw, tool->getIndex(x+1,y,z), pos);
                    
                    if (y < tool->y_div - 1)
                        drawLine(tool, draw, tool->getIndex(x,y+1,z), pos);
                    
                    if (z < tool->z_div - 1)
                        drawLine(tool, draw, tool->getIndex(x,y,z+1), pos);
                }
                
            }
    
    //showing the selection box
    if (tool->clickedAxis == draw->axis)
    {
        Vector3F diff = tool->currentDraggingPosition - tool->initialDraggingPosition;
        Vector3F p1 = tool->initialDraggingPosition + tool->draggingAY * (tool->draggingAY * diff);
        Vector3F p2 = tool->initialDraggingPosition + tool->draggingAX * (tool->draggingAX * diff);
        
        //draw selection box
        tool->initialDraggingPosition.getValues(pos);
        draw->moveTo(draw->data, pos, LWWIRE_DASH);
        
        p1.getValues(pos);
        draw->lineTo(draw->data, pos, LWWIRE_ABSOLUTE);
        
        tool->currentDraggingPosition.getValues(pos);
        draw->lineTo(draw->data, pos, LWWIRE_ABSOLUTE);
        
        p2.getValues(pos);
        draw->lineTo(draw->data, pos, LWWIRE_ABSOLUTE);
        
        tool->initialDraggingPosition.getValues(pos);
        draw->lineTo(draw->data, pos, LWWIRE_ABSOLUTE);
    }
    
    tool->dirty = 0;
}


 int Lattice_Dirty(LatticeTool *tool )
{
	return tool->dirty ? LWT_DIRTY_WIREFRAME : 0;
}


 void Lattice_Done (LatticeTool	*tool)
{
	if (tool)
		delete tool;
}

