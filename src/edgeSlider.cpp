#include "edgeSlider.h"
#include <lwmodtool.h>
#include <lwserver.h>
#include "sharedGlobals.h"


/*
 * Local information packet.  This includes the mesh edit context, monitor,
 * and polygon count.  Also holds the variable spike factor.
 */
typedef struct st_EdgeSliderData {
	MeshEditOp		*op;
	unsigned int	count;
	EdgeSliderTool	*tool;
	float			len;
} EdgeSliderData;

void EdgeSliderTool::resetTool()
{
	active = 0;
	update = 0;
	dirty = 1;
	edgeSlideFactor = 0.0;
	loopMode = 0;
    isFirst = true;

	clearTempData();
}

void EdgeSliderTool::clearTempData()
{
    edges.clear();
    loopPointsSet.clear();
    edgePointsSet.clear();
    edgePoints.clear();
    loopPoints.clear();
    loopEdges.clear();
    cachedEdgeForDrawing.clear();
}

void getPointPos(const MeshEditOp *op, const LWPntID id, Vector3F &pointPos)
{
    double tempPos[3];
    op->pointPos(op->state, id, tempPos);
    pointPos.setValues(tempPos);
}

void setPointPos(MeshEditOp *op, const LWPntID id, const Vector3F &pointPos)
{
    double tempPos[3];
    pointPos.getValues(tempPos);
    op->pntMove(op->state, id, tempPos);
}

void getPolyCenter(MeshEditOp *op, const LWPolID &polID, Vector3F &center)
{
	EDPolygonInfo *info = op->polyInfo(op->state, polID);
	Vector3F tempPos;
	for (int i = 0; i < info->numPnts; ++i)
	{
		getPointPos(op, info->points[i], tempPos);
		center += tempPos;
	}
	center /= info->numPnts;
}

void getEdgeCenter(MeshEditOp *op, const EDEdgeInfo *edge, Vector3F &center)
{
	Vector3F pos1, pos2;
	getPointPos(op, edge->p1, pos1);
	getPointPos(op, edge->p2, pos2);
	center = (pos2 - pos1) * 0.5 + pos1;
}

void cacheLoopEdge(MeshEditOp *op, EdgeSliderTool *tool, const EDEdgeInfo *edge)
{
    if (tool->loopEdges.find(edge->edge) != tool->loopEdges.end())
        return;
    
    Edge e;
    e.id = edge->edge;
    e.p1 = edge->p1;
    e.p2 = edge->p2;
    e.pol1 = edge->pols[0];
    e.pol2 = edge->numPols > 1 ? edge->pols[1] : NULL;
    getEdgeCenter(op, edge, e.pos);
    
    tool->loopEdges[edge->edge] = e;
    
    tool->loopPointsSet.insert(edge->p1);
    tool->loopPointsSet.insert(edge->p2);
}


void getPolyFromPoints(MeshEditOp *op, const EDPointInfo *point, std::set<LWPolID> &polys)
{
	polys.clear();
	EDEdgeInfo *eInfo;
	for (int i = 0; i < point->numEdges; ++i)
	{
		eInfo = op->edgeInfo(op->state, point->edges[i]);
		for (int j = 0; j < eInfo->numPols; ++j)
			polys.insert(eInfo->pols[j]);
	}
}

LWPntID getNextVertexOnPoly(MeshEditOp *op, const LWPolID polyID, const LWPntID thisPoint, const std::set<LWPntID> otherPoints)
{
    EDPolygonInfo *polyInfo = op->polyInfo(op->state, polyID);
    
	// get polygon points which are not belonging to this edge
	for (int i = 0; i < polyInfo->numPnts; ++i)
	{
        if (otherPoints.find(polyInfo->points[i]) != otherPoints.end())
            continue;
        
        LWPntID nextId = polyInfo->points[i == polyInfo->numPnts - 1 ? 0 : i + 1];
        LWPntID previousId = polyInfo->points[i == 0 ? polyInfo->numPnts - 1 : i - 1];
		if (nextId == thisPoint || previousId == thisPoint)
            return polyInfo->points[i];
	}
	
	return NULL;
}

bool isEdgeSharedByPolys(const EDEdgeInfo *eInfo, const LWPolID &pol1, const LWPolID &pol2)
{
	return (eInfo->pols[1] == pol1 && eInfo->pols[0] == pol2) || (eInfo->pols[1] == pol2 && eInfo->pols[0] == pol1);
}

bool getLoopEdge(MeshEditOp *op, EDPointInfo *pInfo, const LWPolID &pol1, const LWPolID &pol2, const LWEdgeID &currentEdge, LWEdgeID &newEdge)
{
	std::set<LWPolID>::iterator it_polys;
	std::set<LWPolID> polys;
	
	EDEdgeInfo *eInfo = op->edgeInfo(op->state, currentEdge);
	
	switch (pInfo->numEdges)
	{
		case 1:
			return false;
		case 2:
			//if only two edges take the other edge
			newEdge = pInfo->edges[0] == currentEdge ? pInfo->edges[1] : pInfo->edges[0];
			return true;
		case 3:
            return false;
		case 4:
			//sprintf(FString,"numEdges: %d\n", pInfo->numEdges);
			getPolyFromPoints(op, pInfo, polys);
			// if point has 3 polygons then exit if edge is shared by two, result edge is the one which is not shared by more than one poly
			if (polys.size() == 3) 
			{
				it_polys = polys.begin();
				LWPolID pol1 = *it_polys; ++it_polys;
				LWPolID pol2 = *it_polys; ++it_polys;
				LWPolID pol3 = *it_polys;
				if (isEdgeSharedByPolys(eInfo, pol1, pol2) || isEdgeSharedByPolys(eInfo, pol1, pol3) || isEdgeSharedByPolys(eInfo, pol3, pol2))	
					//if this edge is shared by both polygons we skip
					return false;
				for (int i = 0; i < pInfo->numEdges; ++i)
				{
					eInfo = op->edgeInfo(op->state, pInfo->edges[i]);
					if (eInfo->edge == currentEdge) continue;
					if (!isEdgeSharedByPolys(eInfo, pol1, pol2) && !isEdgeSharedByPolys(eInfo, pol1, pol3) && !isEdgeSharedByPolys(eInfo, pol3, pol2))	
					{
						newEdge = eInfo->edge;
						return true;
					}
				}
			}
			
			if (polys.size() == 4) 
			{
				// if 4 polygons share this point take the edge which is not shared by the polygons of the current edge
				for (int i = 0; i< pInfo->numEdges; ++i)
				{
					if(pInfo->edges[i] == currentEdge) continue;
					eInfo = op->edgeInfo(op->state, pInfo->edges[i]);
					if (eInfo->pols[0] == pol1 || eInfo->pols[0] == pol2 || eInfo->pols[1] == pol1 || eInfo->pols[1] == pol2)
						continue;
					newEdge = eInfo->edge;
					return true;
				}
			}
			break;
		default:
			return false;
	}
	
	return false;
}


void addLoopEdgeToCache(MeshEditOp *op, EdgeSliderTool *tool, const EDEdgeInfo *eInfo)
{
	EdgeSegment segment;
	segment.p1 = eInfo->p1;
	segment.p2 = eInfo->p2;
	tool->cachedEdgeForDrawing.push_back(segment);
}

void getLoopsForSelectedEdges(MeshEditOp *op, EdgeSliderTool *tool)
{
    std::set<LWEdgeID> tempEdgeCache;
    for (std::map<LWEdgeID, Edge>::iterator it = tool->loopEdges.begin(); it != tool->loopEdges.end(); ++it)
        tempEdgeCache.insert(it->first);
    
	std::set<LWEdgeID>::iterator it;
	
	EDEdgeInfo *eInfo;
	LWEdgeID edge, newEdge;
	while (!tempEdgeCache.empty())
	{
		edge = *tempEdgeCache.begin();
		tempEdgeCache.erase(edge);
		
		eInfo = op->edgeInfo(op->state, edge);
		//get the edge points
		LWPntID p1 = eInfo->p1;
		LWPntID p2 = eInfo->p2;
		LWPolID pol1 = eInfo->pols[0];
		LWPolID pol2 = eInfo->pols[1];
		
		//for both points get the edges
		bool res = getLoopEdge(op, op->pointInfo(op->state, p1), pol1, pol2, edge, newEdge);
		if (res)
		{
			if (tool->loopEdges.find(newEdge) == tool->loopEdges.end())
			{
                eInfo = op->edgeInfo(op->state, newEdge);
                if (eInfo->numPols > 2)
                    continue;
                
				tempEdgeCache.insert(newEdge);
				addLoopEdgeToCache(op, tool, eInfo);
                cacheLoopEdge(op, tool, eInfo);
			}
		}
		
		res = getLoopEdge(op, op->pointInfo(op->state, p2), pol1, pol2, edge, newEdge);
		if (res)
		{
			if (tool->loopEdges.find(newEdge) == tool->loopEdges.end())
			{
                eInfo = op->edgeInfo(op->state, newEdge);
                if (eInfo->numPols > 2)
                    continue;

				tempEdgeCache.insert(newEdge);
                addLoopEdgeToCache(op, tool, eInfo);
                cacheLoopEdge(op, tool, eInfo);
			}
		}
    }
}

int getSharedPointsNumBetweenPolys(const MeshEditOp *op, const LWPolID &pol1, const LWPolID &pol2)
{
	int numPoints = 0;
	
	EDPolygonInfo *info = op->polyInfo(op->state, pol1);
	std::vector <LWPntID> points;
	points.reserve(info->numPnts);
	for (int i = 0; i < info->numPnts; ++i)
		points.push_back(info->points[i]);
	
	info = op->polyInfo(op->state, pol2);
	int points_size = points.size();
	for (int i = 0; i < info->numPnts; ++i)
	{
		for (int j = 0; j < points_size; ++j)
		{
			if (points[j] == info->points[i])
			{
				++numPoints;
				break;
			}
		}
	}
	
	return numPoints;
}

LWPolID getPolyNextTo(MeshEditOp *op, EdgeSliderTool *tool, const LWPolID &polID, const EDEdgeInfo *edge, const LWEdgeID &previousEdge)
{
	// get the two polys for edge
	LWPolID pol1 = edge->pols[0];
	LWPolID pol2 = edge->pols[1];
	
	// if one of the edge polys is the same as the previous poly then return it
	if (pol1 == polID)	return polID;
	if (pol2 == polID)	return polID;
	
	int numPoints1 = getSharedPointsNumBetweenPolys(op, pol1, polID);
	int numPoints2 = getSharedPointsNumBetweenPolys(op, pol2, polID);

    // if pol2 shares two points with polID and pol1 doesn't return pol2
    if (numPoints2 > 1 && numPoints1 < 2)
        return pol2;
    
	return pol1;
}


void getNeighbors(MeshEditOp *op, EdgeSliderTool *tool, const EDEdgeInfo *edge, const std::set<LWEdgeID> &edgeCache, std::vector <LWEdgeID> &edges)
{
	LWPntID p1 = edge->p1;
	LWPntID p2 = edge->p2;
	
	for (std::set<LWEdgeID>::iterator it = edgeCache.begin() ; it != edgeCache.end(); ++it)
	{
        EDEdgeInfo *info = op->edgeInfo(op->state, *it);
		//is edge connected?
		if (p1 == info->p1 || p1 == info->p2 || p2 == info->p1 || p2 == info->p2)
        {
			edges.push_back(*it);
            
        }
	}
}

LWEdgeID getNextLoopEdge(MeshEditOp *op, std::set<LWEdgeID> &allEdges, EdgeSliderTool *tool, const Edge &thisEdge, LWPntID &sharingPoint, LWPntID &nextPoint)
{
    std::set<LWEdgeID>::iterator it;
    LWEdgeID newEdge;
    LWPntID sp;
    bool res;
    
    if (sharingPoint == NULL)
    {
        res = getLoopEdge(op, op->pointInfo(op->state, thisEdge.p1), thisEdge.pol1, thisEdge.pol2, thisEdge.id, newEdge);
        sp = thisEdge.p1;
        if (res)
            it = allEdges.find(newEdge);
        
        if (!res || it == allEdges.end())
        {
            res = getLoopEdge(op, op->pointInfo(op->state, thisEdge.p2), thisEdge.pol1, thisEdge.pol2, thisEdge.id, newEdge);
            sp = thisEdge.p2;
            if (res)
                it = allEdges.find(newEdge);
        }
        
        if (res && it != allEdges.end())
            sharingPoint = sp;
        
    }
    else
    {
        res = getLoopEdge(op, op->pointInfo(op->state, sharingPoint), thisEdge.pol1, thisEdge.pol2, thisEdge.id, newEdge);
        if (res)
            it = allEdges.find(newEdge);
    }
    
    if (res && it != allEdges.end())
    {
        Edge *edge = &tool->loopEdges[*it];
        nextPoint = edge->p1 == sharingPoint ? edge->p2 : edge->p1;
        return newEdge;
    }
    
    return NULL;
}

void rotateVector(Vector3F &vecToRotate, const Vector3F &v1, const Vector3F &v2)
{
    if (v1.dot(v2) > 0.99999)
        return;
    
    if (v1.dot(v2) < -0.99999)
    {
        vecToRotate *= -1;
        return;
    }
    
    double angle = acos(v1.dot(v2));
    Vector3F axis = v1.cross(v2);
    axis.normalize();
    
    double r = cos(angle/2);
    Vector3F v = axis * sin(angle/2);
    
    double Q[3][3];
    Q[0][0] = 1. - 2.0 * (v.y() * v.y() + v.z() * v.z());
    Q[0][1] = 2.0 * (v.x() * v.y() + v.z() * r);
    Q[0][2] = 2.0 * (v.z() * v.x() - v.y() * r);
    
    Q[1][0] = 2.0 * (v.x() * v.y() - v.z() * r);
    Q[1][1] = 1. - 2.0 * (v.z() * v.z() + v.x() * v.x());
    Q[1][2] = 2.0 * (v.y() * v.z() + v.x() * r);
    
    Q[2][0] = 2.0 * (v.z() * v.x() + v.y() * r);
    Q[2][1] = 2.0 * (v.y() * v.z() - v.x() * r);
    Q[2][2] = 1. - 2.0 * (v.y() * v.y() + v.x() * v.x());
    
    double x = vecToRotate.x() * Q[0][0] + vecToRotate.y() * Q[1][0] + vecToRotate.z() * Q[2][0];
    double y = vecToRotate.x() * Q[0][1] + vecToRotate.y() * Q[1][1] + vecToRotate.z() * Q[2][1];
    double z = vecToRotate.x() * Q[0][2] + vecToRotate.y() * Q[1][2] + vecToRotate.z() * Q[2][2];
    
    vecToRotate.setValues(x, y, z);
    vecToRotate.normalize();
}

void getDirectionVector(MeshEditOp *op, Edge &edge, const LWPntID firstPoint)
{
    Vector3F firstPos, secondPos;
    LWPntID secondPoint = edge.p1 == firstPoint ? edge.p2 : edge.p1;
    getPointPos(op, firstPoint, firstPos);
    getPointPos(op, secondPoint, secondPos);
    
    edge.directionVector = secondPos - firstPos;
    edge.directionVector.normalize();
}

void cacheEdgePoints(MeshEditOp *op, EdgeSliderTool *tool, const Vector3F &referenceVector, const Edge &edge, const Vector3F &edgeNormal1, const Vector3F &edgeNormal2)
{
    
    LWPntID p11 = getNextVertexOnPoly(op, edge.pol1, edge.p1, tool->edgePointsSet);
    LWPntID p21 = getNextVertexOnPoly(op, edge.pol1, edge.p2, tool->edgePointsSet);
    
    LWPntID p12=NULL, p22=NULL;
    if (edge.pol2)
    {
        p12 = getNextVertexOnPoly(op, edge.pol2, edge.p1, tool->edgePointsSet);
        p22 = getNextVertexOnPoly(op, edge.pol2, edge.p2, tool->edgePointsSet);
    }
    
    EdgePoint point1, point2;
    point1.id = edge.p1;
    point2.id = edge.p2;
    point1.edge = edge.id;
    point2.edge = edge.id;
    getPointPos(op, point1.id, point1.sourcePos);
    getPointPos(op, point2.id, point2.sourcePos);
    
    if (edgeNormal1 * referenceVector >= edgeNormal2 * referenceVector)
    {
        point1.targetPt1 = p11;
        point1.targetPt2 = p12;
        point2.targetPt1 = p21;
        point2.targetPt2 = p22;
    }
    else
    {
        point1.targetPt1 = p12;
        point1.targetPt2 = p11;
        point2.targetPt1 = p22;
        point2.targetPt2 = p21;
    }
    
    if (tool->edgePoints.find(edge.p1) == tool->edgePoints.end())
        tool->edgePoints[edge.p1] = point1;
    
    if (tool->edgePoints.find(edge.p2) == tool->edgePoints.end())
        tool->edgePoints[edge.p2] = point2;
}

void cacheLoopPoints(MeshEditOp *op, EdgeSliderTool *tool, const Vector3F &referenceVector, const Edge &edge, Vector3F &nextReferenceVector)
{
    Vector3F polCenter1, polCenter2, edgeNormal1, edgeNormal2, crossVector;
    getPolyCenter(op, edge.pol1, polCenter1);
    edgeNormal1 = polCenter1 - edge.pos;
    edgeNormal1.normalize();
    crossVector = edgeNormal1.cross(edge.directionVector);
    if (crossVector.length() > 0.0001)
    {
        crossVector.normalize();
        edgeNormal1 = edge.directionVector.cross(crossVector);
        edgeNormal1.normalize();
    }
    
    LWPntID p11 = getNextVertexOnPoly(op, edge.pol1, edge.p1, tool->loopPointsSet);
    LWPntID p21 = getNextVertexOnPoly(op, edge.pol1, edge.p2, tool->loopPointsSet);
    
    LWPntID p12=NULL, p22=NULL;
    if (edge.pol2)
    {
        getPolyCenter(op, edge.pol2, polCenter2);
        edgeNormal2 = polCenter2 - edge.pos;
        edgeNormal2.normalize();
        crossVector = edgeNormal2.cross(edge.directionVector);
        if (crossVector.length() > 0.0001)
        {
            crossVector.normalize();
            edgeNormal2 = edge.directionVector.cross(crossVector);
            edgeNormal2.normalize();
        }
        
        p12 = getNextVertexOnPoly(op, edge.pol2, edge.p1, tool->loopPointsSet);
        p22 = getNextVertexOnPoly(op, edge.pol2, edge.p2, tool->loopPointsSet);
    }
    else
        edgeNormal2.setValues(-edgeNormal1.x(), -edgeNormal1.y(), -edgeNormal1.z());
    
    EdgePoint point1, point2;
    point1.id = edge.p1;
    point2.id = edge.p2;
    point1.edge = edge.id;
    point2.edge = edge.id;
    getPointPos(op, point1.id, point1.sourcePos);
    getPointPos(op, point2.id, point2.sourcePos);
    
    if (edgeNormal1 * referenceVector >= edgeNormal2 * referenceVector)
    {
        nextReferenceVector = edgeNormal1;
        point1.targetPt1 = p11;
        point1.targetPt2 = p12;
        point2.targetPt1 = p21;
        point2.targetPt2 = p22;
    }
    else
    {
        nextReferenceVector = edgeNormal2;
        point1.targetPt1 = p12;
        point1.targetPt2 = p11;
        point2.targetPt1 = p22;
        point2.targetPt2 = p21;
    }

    if (tool->loopPoints.find(edge.p1) == tool->loopPoints.end())
        tool->loopPoints[edge.p1] = point1;
    
    if (tool->loopPoints.find(edge.p2) == tool->loopPoints.end())
        tool->loopPoints[edge.p2] = point2;
    
    if (tool->edges.find(edge.id) != tool->edges.end())
        cacheEdgePoints(op, tool, referenceVector, edge, edgeNormal1, edgeNormal2);
}

LWEdgeID getMinEdge(EdgeSliderTool *tool, const std::set<LWEdgeID> &edges)
{
    double minX;
    LWEdgeID edge;
    
    std::set<LWEdgeID>::iterator it = edges.begin();
    minX = tool->loopEdges[*it].pos.x();
    edge = *it;

    ++it;
    
    for (; it != edges.end(); ++it)
    {
        if (minX > tool->loopEdges[*it].pos.x())
        {
            minX = tool->loopEdges[*it].pos.x();
            edge = *it;
        }
    }
    
    return edge;
}

void cacheEdgePointsForGroups(MeshEditOp *op, EdgeSliderTool *tool, std::vector<std::vector<LWEdgeID> > &edgeGroups)
{
    bool isFirst = true;
    Vector3F edgeNormal(0,1,0), firstEdgeNormal, thisEdgeNormal, prevDirectionVector, firstDirectionalVector;
    LWEdgeID edge, nextEdge;
    
    
    for (unsigned i = 0; i < edgeGroups.size(); ++i)
    {
        std::vector<LWEdgeID> group = edgeGroups[i];
        unsigned int groupSize = group.size();
        
        std::vector<LWEdgeID> group1, group2;
        
        if (!isFirst && group.size() > 1)
        {
            double maxDot;
            int index;

            for (unsigned j = 0; j < groupSize; ++j)
            {
                double dot = fabs(firstDirectionalVector * tool->loopEdges[group[j]].directionVector);
                if (j == 0 || dot > maxDot)
                {
                    maxDot = dot;
                    index = j;
                }
            }
            
            std::vector<LWEdgeID>::iterator it = group.begin();
            it = it + index;
            group2.insert(group2.begin(), group.begin(), it);
            group1.insert(group1.begin(), it, group.end());
            
            std::reverse(group2.begin(), group2.end());
        }
        else
            group1 = group;
        
        edge = group1[0];
        
        cacheLoopPoints(op, tool, edgeNormal, tool->loopEdges[edge], thisEdgeNormal);
        firstEdgeNormal = thisEdgeNormal;
        if (isFirst)
        {
            edgeNormal = thisEdgeNormal;
            firstDirectionalVector = tool->loopEdges[edge].directionVector;
            isFirst = false;
        }
        
        if (group.size() == 1)
            continue;
        
        //storing previous direction vector
        prevDirectionVector = tool->loopEdges[edge].directionVector;
        for (unsigned j = 1; j < group1.size(); ++j)
        {
            nextEdge = group1[j];
            
            rotateVector(thisEdgeNormal, prevDirectionVector, tool->loopEdges[nextEdge].directionVector);
            cacheLoopPoints(op, tool, thisEdgeNormal, tool->loopEdges[nextEdge], thisEdgeNormal);
            
            //storing previous direction vector
            prevDirectionVector = tool->loopEdges[nextEdge].directionVector;
        }
        
        prevDirectionVector = -tool->loopEdges[edge].directionVector;
        thisEdgeNormal = firstEdgeNormal;
        for (unsigned j = 0; j < group2.size(); ++j)
        {
            nextEdge = group2[j];
            
            rotateVector(thisEdgeNormal, prevDirectionVector, -tool->loopEdges[nextEdge].directionVector);
            cacheLoopPoints(op, tool, thisEdgeNormal, tool->loopEdges[nextEdge], thisEdgeNormal);
            
            //storing previous direction vector
            prevDirectionVector = -tool->loopEdges[nextEdge].directionVector;
        }
    }
}

LWPntID getEdgeLimitPoint(const Edge &limitEdge, const Edge &otherEdge)
{
    if (limitEdge.p1 == otherEdge.p1 || limitEdge.p1 == otherEdge.p2)
        return limitEdge.p2;
    return limitEdge.p1;
}

void getEdgeGroupLimitPoints(EdgeSliderTool *tool, const std::vector<LWEdgeID> &group, LWPntID &first, LWPntID &last)
{
    int groupSize = group.size();
    const Edge &firstEdge = tool->loopEdges[group[0]];
    if (groupSize == 1)
    {
        first = firstEdge.p1;
        last = firstEdge.p2;
        return;
    }

    const Edge &secondEdge = tool->loopEdges[group[groupSize - 1]];
    first = getEdgeLimitPoint(firstEdge, tool->loopEdges[group[1]]);
    last = getEdgeLimitPoint(secondEdge, tool->loopEdges[group[groupSize-2]]);
}

void mergeGroups(MeshEditOp *op, EdgeSliderTool *tool, std::vector<std::vector<LWEdgeID> > &edgeGroups)
{
    int groupSize = edgeGroups.size();
    
    std::vector< std::vector<LWPntID> > groupLimits;
    groupLimits.reserve(groupSize);
    for (int i = 0; i < groupSize; ++i)
    {
        std::vector<LWPntID> pnts;
        pnts.resize(2);
        LWPntID first, last;
        getEdgeGroupLimitPoints(tool, edgeGroups[i], first, last);
        pnts[0] = first;
        pnts[1] = last;
        groupLimits.push_back(pnts);
    }
    
    for (int i = groupSize - 1; i > 0; --i)
    {
        for (int j = 0; j < i; ++j)
        {
            // first - first
            if (groupLimits[i][0] == groupLimits[j][0])
            {
                for (int ii = 0; ii < edgeGroups[i].size(); ++ii)
                    tool->loopEdges[edgeGroups[i][ii]].directionVector.invert();
                std::reverse(edgeGroups[i].begin(), edgeGroups[i].end());
                
                edgeGroups[j].insert(edgeGroups[j].begin(), edgeGroups[i].begin(), edgeGroups[i].end());
                groupLimits[j][0] = groupLimits[i][1];
                
                edgeGroups.erase(edgeGroups.begin() + i);
                
                break;
            }
            //last - first
            else if (groupLimits[i][1] == groupLimits[j][0])
            {
                edgeGroups[j].insert(edgeGroups[j].begin(), edgeGroups[i].begin(), edgeGroups[i].end());
                groupLimits[j][0] = groupLimits[i][0];
                
                edgeGroups.erase(edgeGroups.begin() + i);
                break;
            }
            //first - last
            else if (groupLimits[i][0] == groupLimits[j][1])
            {
                edgeGroups[j].insert(edgeGroups[j].end(), edgeGroups[i].begin(), edgeGroups[i].end());
                groupLimits[j][1] = groupLimits[i][1];
                
                edgeGroups.erase(edgeGroups.begin() + i);
                break;
            }
            //last - last
            else if (groupLimits[i][1] == groupLimits[j][1])
            {
                for (int ii = 0; ii < edgeGroups[i].size(); ++ii)
                    tool->loopEdges[edgeGroups[i][ii]].directionVector.invert();
                std::reverse(edgeGroups[i].begin(), edgeGroups[i].end());
                
                edgeGroups[j].insert(edgeGroups[j].end(), edgeGroups[i].begin(), edgeGroups[i].end());
                groupLimits[j][1] = groupLimits[i][0];
                
                edgeGroups.erase(edgeGroups.begin() + i);
                break;
            }
        }
    }
    
}

void groupEdges(MeshEditOp *op, EdgeSliderTool *tool, std::set<LWEdgeID> &limitEdges, std::vector<std::vector<LWEdgeID> > &edgeGroups)
{
    std::set<LWEdgeID> allEdges;
    for (std::map<LWEdgeID, Edge>::iterator it = tool->loopEdges.begin(); it != tool->loopEdges.end(); ++it)
        allEdges.insert(it->first);
    
	std::set<LWEdgeID>::iterator it;
    LWPntID sharingPoint, nextPoint;
    
	while (!limitEdges.empty())
	{
        std::vector<LWEdgeID> group;
        
        LWEdgeID edge = getMinEdge(tool, limitEdges);
        group.push_back(edge);
 
        limitEdges.erase(limitEdges.find(edge));
        allEdges.erase(allEdges.find(edge));
        
        sharingPoint = NULL;
        LWEdgeID nextEdge = getNextLoopEdge(op, allEdges, tool, tool->loopEdges[edge], sharingPoint, nextPoint);
        if (!nextEdge)
        {
            edgeGroups.push_back(group);
            getDirectionVector(op, tool->loopEdges[edge], tool->loopEdges[edge].p1);
            continue;
        }
        
        //get the edge vector
        getDirectionVector(op, tool->loopEdges[edge], sharingPoint == tool->loopEdges[edge].p1 ? tool->loopEdges[edge].p2 : tool->loopEdges[edge].p1);
        getDirectionVector(op, tool->loopEdges[nextEdge], sharingPoint);
        
        group.push_back(nextEdge);
        
        //removing for all edges set
        allEdges.erase(allEdges.find(nextEdge));
        it = limitEdges.find(nextEdge);
        
        sharingPoint = nextPoint;
        while (it == limitEdges.end() && !allEdges.empty())
        {
            nextEdge = getNextLoopEdge(op, allEdges, tool, tool->loopEdges[nextEdge], sharingPoint, nextPoint);
            //this could happen when encountering a fork, if the edge was already taken out of here
            if (!nextEdge)
                break;
    
            group.push_back(nextEdge);
            
            //get the edge vector
            getDirectionVector(op, tool->loopEdges[nextEdge], sharingPoint);

            sharingPoint = nextPoint;
            
            //removing for all edges set
            allEdges.erase(allEdges.find(nextEdge));
            it = limitEdges.find(nextEdge);
        }
        
        edgeGroups.push_back(group);
        
        if (it != limitEdges.end())
            limitEdges.erase(it);
        
        // this will happen when we had closed loops
        if (!allEdges.empty() && limitEdges.empty())
            limitEdges.insert(getMinEdge(tool, allEdges));
    }
}

void findLimitEdges(MeshEditOp *op, EdgeSliderTool *tool, std::set<LWEdgeID> &limitEdges)
{
    limitEdges.clear();
    for (std::map<LWEdgeID, Edge>::iterator it1 = tool->loopEdges.begin(); it1 != tool->loopEdges.end(); ++it1)
    {
        LWPntID p1 = it1->second.p1;
        LWPntID p2 = it1->second.p2;
        bool shareP1 = false, shareP2 = false;
        
        for (std::map<LWEdgeID, Edge>::iterator it2 = tool->loopEdges.begin(); it2 != tool->loopEdges.end(); ++it2)
        {
            if (it1->second.id == it2->second.id)   continue;
            
            if (p1 == it2->second.p1 || p1 == it2->second.p2)
                shareP1 = true;
            if (p2 == it2->second.p1 || p2 == it2->second.p2)
                shareP2 = true;
            
            if (shareP1 && shareP2)
                break;
        }
        
        if (!shareP1 || !shareP2)
            limitEdges.insert(it1->second.id);
    }
    
    if (limitEdges.size() == 0)
        limitEdges.insert(tool->loopEdges.begin()->second.id);
}

int evaluatePoint(MeshEditOp *op, EdgeSliderTool *tool, EdgePoint &ePoint, std::map<LWPntID, EdgePoint> &busyPoints)
{
    Vector3F otherPos;
    double factor = tool->edgeSlideFactor;
    
    ePoint.pos = ePoint.sourcePos;
    if (tool->edgeSlideFactor > 0)
    {
        if (ePoint.targetPt1 == NULL)
            return 0;
        
        // HERE WE CHECK IF THE TARGET POINTS ARE IN THE MAP, IF THEY ARE WE DON'T MOVE THIS POINTS
        if (busyPoints.find(ePoint.targetPt1) != busyPoints.end())
            return 0;
        
        getPointPos(op, ePoint.targetPt1, otherPos);
        
        ePoint.pos = (otherPos - ePoint.sourcePos) * fabs(factor) + ePoint.sourcePos;
        setPointPos(op, ePoint.id, ePoint.pos);
    }
    else
    {
        if (ePoint.targetPt2 == NULL)
            return 0;
        
        // HERE WE CHECK IF THE TARGET POINTS ARE IN THE MAP, IF THEY ARE WE DON'T MOVE THIS POINTS
        if (busyPoints.find(ePoint.targetPt2) != busyPoints.end())
            return 0;
        
        getPointPos(op, ePoint.targetPt2, otherPos);
        
        ePoint.pos = (otherPos - ePoint.sourcePos) * fabs(factor) + ePoint.sourcePos;
        setPointPos(op, ePoint.id, ePoint.pos);
    }
	
	return EDERR_NONE;
}

int cacheEdges(EdgeSliderData *dat, const EDEdgeInfo *edge)
{
	if (!(edge->flags & EDDF_SELECT) || edge->numPols > 2)
		return EDERR_NONE;
    
    cacheLoopEdge(dat->op, dat->tool, edge);
    addLoopEdgeToCache(dat->op, dat->tool, edge);
    
    dat->tool->edges.insert(edge->edge);
    dat->tool->edgePointsSet.insert(edge->p1);
    dat->tool->edgePointsSet.insert(edge->p2);
    
	return EDERR_NONE;
}

void findPointsSymmetry(EdgeSliderTool *tool)
{
    //cache if the points were symmetrical or not
    std::map<LWPntID, EdgePoint>::iterator it1, it2;
    for (it1 = tool->loopPoints.begin(); it1 != tool->loopPoints.end(); ++it1)
    {
        EdgePoint *p1 = &it1->second;
        
        if (p1->sourcePos.x() >= 0)
            continue;
            
        for (it2 = tool->loopPoints.begin(); it2 != tool->loopPoints.end(); ++it2)
        {
            EdgePoint *p2 = &it2->second;
            if (p2->sourcePos.x() < 0)
                continue;
            
            if ((p1->sourcePos.x() + p2->sourcePos.x()) * (p1->sourcePos.x() + p2->sourcePos.x()) <= 0.0000025  &&
                p1->sourcePos.y() == p2->sourcePos.y() && p1->sourcePos.z() == p2->sourcePos.z())
            {
                tool->symmMap[p1->id] = p2->id;
                break;
            }
        }
    }
}

int EdgeSlider(MeshEditOp *op, EdgeSliderTool *tool)
{	
	EDError err = EDERR_NONE;
	
	if (query->mode(LWM_MODE_SELECTION) != 3) return err;
	
	int numedges = op->edgeCount(op->state, OPLYR_FG, EDCOUNT_SELECT);
	if (numedges == 0) return err;
    
    EdgeSliderData dat;
	dat.op = op;
	dat.tool = tool;
    
	if (tool->edgePoints.empty())
	{
        //getting selected edges
        err = (*op->edgeScan) (op->state, (edgeScan_function)cacheEdges, &dat, OPLYR_FG);
        
        //getting all loop edges, unfortunately we can't group them here as we have no idea about the directions and
        //the limit edges
        getLoopsForSelectedEdges(op, tool);
        
        std::set<LWEdgeID> limitEdges;
        findLimitEdges(op, tool, limitEdges);
        
        //grouping the edges
        std::vector<std::vector<LWEdgeID> > edgeGroups;
        edgeGroups.clear();
        groupEdges(op, tool, limitEdges, edgeGroups);
        
        //merge groups
        mergeGroups(op, tool, edgeGroups);
        
        //caching the points
        cacheEdgePointsForGroups(op, tool, edgeGroups);
        
        //find the simmetry in case we need it later
        findPointsSymmetry(tool);
    }
    
    
    // when looping here if the group is negative and simmetry is on, then negate the edgeSlideFactor
    //      otherwise check edge centers
    std::map<LWPntID, EdgePoint> &pointsMap = tool->loopMode ? tool->loopPoints: tool->edgePoints;
    for (std::map<LWPntID, EdgePoint>::iterator it = pointsMap.begin(); it != pointsMap.end(); ++it)
        evaluatePoint(op, tool, it->second, pointsMap);
    
    //final ovverride for simmetry, if the simmetry is on and the points were identical simmetrical then we mirror the offset
    if (query->mode(LWM_MODE_SYMMETRY))
    {
        for (std::map<LWPntID, LWPntID>::iterator it = tool->symmMap.begin(); it != tool->symmMap.end(); ++it)
        {
            if (pointsMap.find(it->second) != pointsMap.end() && pointsMap.find(it->first) != pointsMap.end())
            {
                Vector3F p = pointsMap[it->second].pos;
                pointsMap[it->first].pos.setValues(-p.x(), p.y(), p.z());
                setPointPos(op, pointsMap[it->first].id, pointsMap[it->first].pos);
            }
        }
    }
    
    //set all the final positions here
    
	(*op->done) (op->state, err, 0);

	return err;
}
