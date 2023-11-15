#include "sketchDrag.h"
#include <lwmodtool.h>
#include <lwserver.h>

/*
 * Local information packet.  This includes the mesh edit context, monitor,
 * and polygon count.  Also holds the variable spike factor.
 */

#define EPSILON 0.00001

enum { X = 0x8001, minusX, Y, minusY};

typedef struct st_Line
{
	LWDVector direction;
	LWDVector origin;
}Line;

typedef struct st_LWPoint
{
    float x;
    float y;
    float z;
} Point3D;

typedef struct st_SketchDragPoint
{
    LWPntID id;
    Vector3F initialPosition;
    Vector3F projectedPosition;
    Vector3F closestPosition;
    Vector3F finalPosition;
    Vector3F delta;

    Vector3F iterationPosition;
} SketchDragPoint;

typedef  struct st_IterationPoint
{
    Vector3F initialPosition;
    double sqrDistance;
    LWPntID otherPoint;
} IterationPoint;

typedef struct st_SketchDragData {
	MeshEditOp		*op;
	SketchDragTool	*tool;
    std::map<LWPntID, SketchDragPoint> points;
    
    bool hasClosestPoint;
    double sqrDistance;
    LWPntID closestPointId;

    std::set<LWEdgeID> edges;
    
    std::map<LWPntID, SketchDragPoint> movedPoints;
    
    std::vector<SketchDragPoint> pointsToMove;
    std::set<LWPntID> evaluatedPoints;
    
    SketchDragPoint bufferedPoints[3];
    int bufferIndex;

} SketchDragData;

float PointLineDistanceSquared3D(Point3D q, Line l, int normalized, float *t)
{
	float distanceSquared;
	LWDVector temp, vec, tempq;
	Point3D qPrime;
	float tt;
    
	tempq[0] = q.x;
	tempq[1] = q.y;
	tempq[2] = q.z;
    
	VSUB3(temp, tempq, l.origin);
	tt = VDOT(l.direction, temp);
    
	if (!normalized)
		tt /= VDOT(l.direction, l.direction);
    
	qPrime.x = l.origin[0] + tt * l.direction[0];
	qPrime.y = l.origin[1] + tt * l.direction[1];
	qPrime.z = l.origin[2] + tt * l.direction[2];
    
	*t = tt;
    
	vec[0] = q.x - qPrime.x;
	vec[1] = q.y - qPrime.y;
	vec[2] = q.z - qPrime.z;
	distanceSquared = VDOT(vec, vec);
    
	return distanceSquared;
}

float PointPolylineDistance3D(Point3D p, std::vector<Vector3F> vertices, int nSegments, int *ind, float *tt)
{
	float dSq;
	float finalDist;
	float xMinusA, yMinusB, zMinusC;
	float xNextMinusA, yNextMinusB, zNextMinusC;
    
	float xMinusASq, yMinusBSq, zMinusCSq;
	float xNextMinusASq, yNextMinusBSq, zNextMinusCSq;
	
	int i;
	Line l;
	float t;
    
	t = 0;
    
	xMinusA = vertices[0].x() - p.x;
	yMinusB = vertices[0].y() - p.y;
	zMinusC = vertices[0].z() - p.z;
	xMinusASq = xMinusA * xMinusA;
	yMinusBSq = yMinusB * yMinusB;
	zMinusCSq = zMinusC * zMinusC;
	
	xNextMinusA = vertices[1].x() - p.x;
	yNextMinusB = vertices[1].y() - p.y;
	zNextMinusC = vertices[1].z() - p.z;
	xNextMinusASq = xNextMinusA * xNextMinusA;
	yNextMinusBSq = yNextMinusB * yNextMinusB;
	zNextMinusCSq = zNextMinusC * zNextMinusC;
	
	// Compute distance to first segment
	l.origin[0] = vertices[0].x();
	l.origin[1] = vertices[0].y();
	l.origin[2] = vertices[0].z();
    
	l.direction[0] = vertices[1].x() - vertices[0].x();
    l.direction[1] = vertices[1].y() - vertices[0].y();
    l.direction[2] = vertices[1].z() - vertices[0].z();
    
	dSq = PointLineDistanceSquared3D(p, l, 0, &t);
	
	// If closest point not on segment, check appropriate end point
	if (t < 0)
		dSq = xMinusASq + yMinusBSq + zMinusCSq;
	else
        if (t > 1)
            dSq = xNextMinusASq + yNextMinusBSq + zNextMinusCSq;
    
	finalDist = dSq;
	*tt = t;
	*ind = 0;
	
	// Go through each successive segment, rejecting if possible,
	// and computing the distance squared if not rejected.
	for (i = 1; i < nSegments -1; i++) {
        
		if (i != nSegments - 1) {
			xMinusASq = xNextMinusASq;
			yMinusBSq = yNextMinusBSq;
			zMinusCSq = zNextMinusCSq;
            
			xNextMinusA = vertices[i + 1].x() - p.x;
			yNextMinusB = vertices[i + 1].y() - p.y;
			zNextMinusC = vertices[i + 1].z() - p.z;
            
			xNextMinusASq = xNextMinusA * xNextMinusA;
			yNextMinusBSq = yNextMinusB * yNextMinusB;
			zNextMinusCSq = zNextMinusC * zNextMinusC;
		}
        
		// Rejection test failed - check distance to line
		l.origin[0] = vertices[i].x();
		l.origin[1] = vertices[i].y();
		l.origin[2] = vertices[i].z();
		
        l.direction[0] = vertices[i+1].x() - vertices[i].x();
        l.direction[1] = vertices[i+1].y() - vertices[i].y();
        l.direction[2] = vertices[i+1].z() - vertices[i].z();

		dSq = PointLineDistanceSquared3D(p, l, 0, &t);
        
		// If closest point not on segment, check appropriate end point
		if (t < 0)
			dSq = xMinusASq + yMinusBSq + zMinusCSq;
		else
            if (t > 1)
                dSq = xNextMinusASq + yNextMinusBSq + zNextMinusCSq;
        
		if (finalDist > dSq)
		{
			finalDist = dSq;
			*tt = t;
			*ind = i;
		}
	}
    
	return sqrt(finalDist);
}

int fastProcessEdges(SketchDragData *dat, const LWEdgeID edge)
{
    dat->edges.insert(edge);
    return EDERR_NONE;
}

int fastSymmPnts(SketchDragData *dat, const LWPntID pnt)
{
    LWDVector pntPos;
    dat->op->pointPos(dat->op->state, pnt, pntPos);
    if (pntPos[0] > 0)
        return EDERR_NONE;
    
    std::map<LWPntID, SketchDragPoint>::iterator it;
    if (pntPos[0] == 0)
    {
        it = dat->movedPoints.find(pnt);
        if (it != dat->movedPoints.end())
        {
            it->second.finalPosition.getValues(pntPos);
            pntPos[0] = 0;
            dat->op->pntMove(dat->op->state, pnt, pntPos);
        }
    }
    
    for (it = dat->movedPoints.begin(); it != dat->movedPoints.end(); ++it)
    {
        if (it->second.initialPosition.x() < 0)
            continue;
        
        if ((pntPos[0] + it->second.initialPosition.x()) * (pntPos[0] + it->second.initialPosition.x()) <= 0.0000025  &&
            pntPos[1] == it->second.initialPosition.y() && pntPos[2] == it->second.initialPosition.z())
        {
            it->second.finalPosition.getValues(pntPos);
            pntPos[0] = -pntPos[0];
            dat->op->pntMove(dat->op->state, pnt, pntPos);
        }
    }
    
    return EDERR_NONE;
}

int processEdges(SketchDragData *dat, const EDEdgeInfo *edge)
{
    if (edge->numPols > 2)
        return EDERR_NONE;
    
    if (edge->numPols != 1)
    {
        LWDVector normal1, normal2;
        dat->op->polyNormal(dat->op->state, edge->pols[0], normal1);
        dat->op->polyNormal(dat->op->state, edge->pols[1], normal2);
        
        LWDVector planeNormal;
        dat->tool->planeNormal.getValues(planeNormal);
        
        double dot1 = VDOT(planeNormal, normal1);
        double dot2 = VDOT(planeNormal, normal2);
        
        // these are the edges on the silhouette with 2 polys
        if ((dot1 >= 0 && dot2 <= 0) || (dot2 >= 0 && dot1 <= 0))
            dat->edges.insert(edge->edge);
    }
    else
        dat->edges.insert(edge->edge);

    return EDERR_NONE;
}

void projectPoint(Vector3F &result, const SketchDragTool *tool, const Vector3F &pos)
{
    Vector3F vec = pos - tool->worldCurvePoints[0];
    result.setValues(vec * tool->ax, vec * tool->ay, 0);
}

void projectAllEdgePoints(SketchDragData *dat)
{
    LWPntID pnts[2];
    LWDVector pntPos;
    double sqrDistance;
    
    for (std::set<LWEdgeID>::iterator it=dat->edges.begin(); it!=dat->edges.end(); ++it)
    {
        pnts[0] = dat->op->edgePoint1(dat->op->state, *it);
        pnts[1] = dat->op->edgePoint2(dat->op->state, *it);
        
        for (int i=0; i < 2; ++i)
        {
            if (dat->points.find(pnts[i]) != dat->points.end())
                continue;
            
            SketchDragPoint point;
            point.id = pnts[i];

            dat->op->pointPos(dat->op->state, pnts[i], pntPos);
            point.initialPosition.setValues(pntPos);
            projectPoint(point.projectedPosition, dat->tool, point.initialPosition);
            
            sqrDistance = point.projectedPosition * point.projectedPosition;
            if (!dat->hasClosestPoint || sqrDistance < dat->sqrDistance)
            {
                dat->sqrDistance = sqrDistance;
                dat->closestPointId = pnts[i];
                dat->hasClosestPoint = true;
            }
            
            dat->points[pnts[i]] = point;
         }
    }
}

void processNextEdgeFromPoint(SketchDragData *dat, SketchDragPoint &point, const Vector3F &directionalVector, const bool selectedEdges)
{
    
    const LWEdgeID *edges;
    int num_edges = dat->op->pointEdges(dat->op->state, point.id, &edges);

    bool hasEdge = false;
    
    LWEdgeID nextEdge;
    LWPntID otherPoint;
    double dot, thisDot;
    for (int i=0; i < num_edges; ++i)
    {
        if (dat->edges.find(edges[i]) != dat->edges.end())
        {
            LWPntID pointID = dat->op->edgePoint1(dat->op->state, edges[i]);
            if (point.id == pointID)
                pointID = dat->op->edgePoint2(dat->op->state, edges[i]);
            
            if (dat->evaluatedPoints.find(pointID) != dat->evaluatedPoints.end())
                continue;
            
            Vector3F thisDirection = dat->points[pointID].projectedPosition - point.projectedPosition;
            
            thisDot = thisDirection * directionalVector;
            if(!hasEdge || dot < thisDot)
            {
                nextEdge = edges[i];
                dot = thisDot;
                otherPoint = pointID;
                
                hasEdge = true;
            }
        }
    }
    
    // this means there was no edge on the silhouette following the direction of stroke
    if (!hasEdge)
        return;
    
    dat->evaluatedPoints.insert(otherPoint);
    
    Vector3F vec = dat->tool->projCurvePoints[dat->tool->numberOfCurvePoints-1] - dat->points[otherPoint].projectedPosition;
    double sqrDistance = vec * vec;
    
	if (dat->sqrDistance > sqrDistance || selectedEdges)
    {
        if (dat->bufferIndex > 0)
            for (int i=0; i<dat->bufferIndex; ++i)
                dat->pointsToMove.push_back(dat->bufferedPoints[i]);
        
        dat->sqrDistance  = sqrDistance;
        dat->bufferIndex = 0;
        dat->pointsToMove.push_back(dat->points[otherPoint]);
    }
	//if the distance is not close enough, we buffer two points, just in case the mesh shape is not following exactly the stroke
    else
    {
        dat->bufferedPoints[dat->bufferIndex] = dat->points[otherPoint];
        ++dat->bufferIndex;
    }
    
    if (dat->bufferIndex > 2)
        return;
    
	processNextEdgeFromPoint(dat, dat->points[otherPoint], directionalVector, selectedEdges);
    
    
    //  for each edge in startPoint.edges:
    //      if edge in silhouette.edges && thisEdgeSquaredAngle_is min_with_vector_described_by_path:
    //          get next point from this edge
    //          if next point was already evaluated
    //              break
    //          if any of the following conditions happen then keep going
    //              if the sqrDistance is closer
    //          else
    //              add to a buffer and make three attempts before breaking
}

void movePointToT(SketchDragData *dat, const double t, const int minCurveIndex, const int maxCurveIndex, const int currentPointIndex)
{
    dat->pointsToMove[currentPointIndex].closestPosition = dat->tool->projCurvePoints[minCurveIndex] + (dat->tool->projCurvePoints[maxCurveIndex] - dat->tool->projCurvePoints[minCurveIndex]) * t;
    
    Vector3F projectedDeltaVector = dat->pointsToMove[currentPointIndex].closestPosition - dat->pointsToMove[currentPointIndex].projectedPosition;
    
    dat->pointsToMove[currentPointIndex].delta = dat->tool->ax * projectedDeltaVector.x() + dat->tool->ay * projectedDeltaVector.y();
    
    dat->pointsToMove[currentPointIndex].finalPosition = dat->pointsToMove[currentPointIndex].initialPosition + dat->pointsToMove[currentPointIndex].delta;
    
    dat->pointsToMove[currentPointIndex].iterationPosition = dat->pointsToMove[currentPointIndex].finalPosition;
    
    LWDVector finalPos;
    dat->pointsToMove[currentPointIndex].finalPosition.getValues(finalPos);
    dat->op->pntMove(dat->op->state, dat->pointsToMove[currentPointIndex].id, finalPos);
}


double getSqrProjectedSketchLenght(SketchDragTool *tool)
{
    double sqrEdgesLenght = 0;
    Vector3F vec;
    for (int i=1; i < tool->numberOfCurvePoints; ++i)
    {
        vec = tool->projCurvePoints[i] - tool->projCurvePoints[i-1];
        sqrEdgesLenght += vec * vec;
    }
    return sqrEdgesLenght;
}

void processNeighborPoint(MeshEditOp *op, const LWPntID pID, const LWPntID otherPoint, std::map<LWPntID, SketchDragPoint> &movedPoints, std::map<LWPntID, IterationPoint> &neighborPoints)
{
    if (movedPoints.find(pID) == movedPoints.end())
    {
        LWDVector pntPos;
        Vector3F pointPos;
        op->pointPos(op->state, pID, pntPos);
        pointPos.setValues(pntPos);
        Vector3F toPoint1 = pointPos - movedPoints[otherPoint].initialPosition;

        double thisSquareDistance = toPoint1 * toPoint1;
        if (neighborPoints.find(pID) == neighborPoints.end())
        {
            IterationPoint ip;
            ip.initialPosition = pointPos;
            ip.otherPoint = otherPoint;
            ip.sqrDistance = thisSquareDistance;
            
            neighborPoints[pID] = ip;
        }
        else
        {
            if (thisSquareDistance < neighborPoints[pID].sqrDistance)
            {
                neighborPoints[pID].sqrDistance = thisSquareDistance;
                neighborPoints[pID].otherPoint = otherPoint;
            }
        }
    }
}

void getNextNeighborPoints(MeshEditOp *op, std::map<LWPntID, SketchDragPoint> &movedPoints, std::map<LWPntID, IterationPoint> &neighborPoints)
{
    std::set<LWPntID> lastNeighborPoints;
    for (std::map<LWPntID, IterationPoint>::iterator it = neighborPoints.begin(); it != neighborPoints.end(); ++it)
        lastNeighborPoints.insert(it->first);
    neighborPoints.clear();
    
    for (std::set<LWPntID>::iterator it = lastNeighborPoints.begin(); it != lastNeighborPoints.end(); ++it)
    {
        const LWEdgeID *edges;
        int numEdges = op->pointEdges(op->state, *it, &edges);
        
        for (int i=0; i < numEdges; ++i)
        {
            LWPntID p1 = op->edgePoint1(op->state, edges[i]);
            LWPntID p2 = op->edgePoint2(op->state, edges[i]);
            
            processNeighborPoint(op, p1, *it, movedPoints, neighborPoints);
            processNeighborPoint(op, p2, *it, movedPoints, neighborPoints);
        }
    }
}

int SketchDrag(MeshEditOp *op, SketchDragTool *tool)
{	
	EDError err = EDERR_NONE;
    
    if (tool->numberOfCurvePoints == 1)
        return err;
    
	SketchDragData dat;
	dat.op = op;
	dat.tool = tool;
    dat.hasClosestPoint = false;
    dat.bufferIndex = 0;
	dat.edges.clear();

    
    //fast edge scanning to see if any edge is selected
    err = (*op->fastEdgeScan) (op->state, (fastEdgeScan_function)fastProcessEdges, &dat, OPLYR_FG, true);

	bool selectedEdges = dat.edges.size() != 0;
    if (!selectedEdges)
    {
        //for all edges get those which are on the silhouette (poly normals < and > 0) or (both dots are <> then e = 0.000  1)
        //      and get the closest point to the initial path points
        // else
        //    get the closest edge points to the initial path point
        err = (*op->edgeScan) (op->state, (edgeScan_function)processEdges, &dat, OPLYR_FG);
    }


    // project all points of the silhouette edges on the camera plane
    projectAllEdgePoints(&dat);
    
    // ************* filter the edges on the inside of the projected mesh
    
    // start from the point closest to the start point
    Vector3F vec = dat.tool->projCurvePoints[tool->numberOfCurvePoints-1] - dat.points[dat.closestPointId].projectedPosition;
    dat.sqrDistance = vec * vec;
    
    dat.pointsToMove.push_back(dat.points[dat.closestPointId]);
    dat.evaluatedPoints.insert(dat.closestPointId);

    //finding directional vector, following the stroke
    Vector3F directionalVector(0,0,0);
	for (int i = 1; i < tool->projCurvePoints.size(); ++i)
		directionalVector += tool->projCurvePoints[i] - tool->projCurvePoints[0];
	directionalVector /= tool->projCurvePoints.size() - 1;
	directionalVector.normalize();
    
	processNextEdgeFromPoint(&dat, dat.points[dat.closestPointId], directionalVector, selectedEdges);
    
    // calculate entire lenght of edges
    double strokeLenght = 0;
	std::vector<double> segmentLenghts;
	segmentLenghts.resize(tool->numberOfCurvePoints - 1);
	for (int i = 1; i<tool->numberOfCurvePoints; ++i)
    {
		segmentLenghts[i-1] = (tool->projCurvePoints[i] - tool->projCurvePoints[i - 1]).length();
		strokeLenght += segmentLenghts[i - 1];
    }

    // Mode 1) project on current t of polyline
    // Mode 2) spread along the polyline
    // Then unproject in 3D and set the new positions for the points, keep the delta movement for those points
    
	int currentSegmentIndex = 0;
	double currentSegmentLenght = 0;
    int pointSize = dat.pointsToMove.size();
    if (dat.pointsToMove.size() == 1)
        movePointToT(&dat, 0, 0, 0, 0);
    else
    {
        for (int i=0; i < pointSize; ++i)
        {
			int minIndex;
			int maxIndex;
            float t;
            
            if (tool->mode == 1)
            {
                Point3D p;
                p.x = dat.pointsToMove[i].projectedPosition.x();
                p.y = dat.pointsToMove[i].projectedPosition.y();
                p.z = dat.pointsToMove[i].projectedPosition.z();
                PointPolylineDistance3D(p, tool->projCurvePoints, tool->numberOfCurvePoints, &minIndex, &t);
				if (t > 1) t = 1;
				if (t < 0) t = 0;
                maxIndex = minIndex + 1;
            }
            else
            {
				if (i == 0)
				{
					minIndex = 0;
					maxIndex = 1;
					t = 0;
				}
				else if (i == pointSize - 1)
				{
					minIndex = tool->numberOfCurvePoints - 2;
					maxIndex = tool->numberOfCurvePoints - 1;
					t = 1;
				}
				else
				{
					
					float tl = (((float)i) / ((float)pointSize - 1)) * strokeLenght;
					
					for (int j = currentSegmentIndex; j < tool->numberOfCurvePoints - 1; ++j)
					{
						if (tl > currentSegmentLenght && tl < currentSegmentLenght + segmentLenghts[j])
						{
							currentSegmentIndex = j;
							break;
						}
						else
							currentSegmentLenght += segmentLenghts[j];
					}

					minIndex = currentSegmentIndex;
					maxIndex = currentSegmentIndex + 1;
					t = (tl - currentSegmentLenght) / (segmentLenghts[currentSegmentIndex]);
				}
            }

            movePointToT(&dat, t, minIndex, maxIndex, i);
        }
    }
    
    std::set<LWPntID> initialPoints;
    std::map<LWPntID, IterationPoint> neighborPoints;
    for (int i=0; i < pointSize; ++i)
    {
        IterationPoint ip;
        dat.movedPoints[dat.pointsToMove[i].id] = dat.pointsToMove[i];
        neighborPoints[dat.pointsToMove[i].id] = ip;
        
        initialPoints.insert(dat.pointsToMove[i].id);
    }
    
    //iterations
    if (tool->iterations > 0)
    {
        //FINDING NEIGHBORS AND EVALUATING THEIR NEW POSITION (EACH NEIGHBOR IS AFFECTED ONLY BY ONE POINT) - ITERATION
        int currentIteration = 0;
        while (currentIteration < tool->iterations && neighborPoints.size() > 0)
        {
            double falloff = (1 - float(currentIteration+1) / 10.0) * tool->propagate;
            
            getNextNeighborPoints(op, dat.movedPoints, neighborPoints);
            for (std::map<LWPntID, IterationPoint>::iterator it = neighborPoints.begin(); it != neighborPoints.end(); ++it)
            {
                SketchDragPoint p;
                p.id = it->first;
                p.initialPosition = it->second.initialPosition;
                p.delta = dat.movedPoints[it->second.otherPoint].delta * falloff;
                p.iterationPosition = p.initialPosition + p.delta;
                
                dat. movedPoints[it->first]= p;
            }
            
            currentIteration++;
        }
        
        //BOX FILTERING
        for (std::map<LWPntID, SketchDragPoint>::iterator it = dat.movedPoints.begin(); it != dat.movedPoints.end(); ++it)
        {
            if (initialPoints.find(it->first) != initialPoints.end())
                continue;
            
            const LWEdgeID *edges;
            int numEdges = op->pointEdges(op->state, it->first, &edges);
            
            Vector3F vec = it->second.iterationPosition;
            
            LWDVector pos;
            for (int i=0; i < numEdges; ++i)
            {
                LWPntID p1 = op->edgePoint1(op->state, edges[i]);
                LWPntID p2 = op->edgePoint2(op->state, edges[i]);
                
                LWPntID p = p1 != it->first ? p1 : p2;
                if (dat.movedPoints.find(p) == dat.movedPoints.end())
                {
                    op->pointPos(op->state, p, pos);
                    vec += Vector3F(pos[0], pos[1], pos[2]);
                }
                else
                    vec += dat.movedPoints[p].iterationPosition;
            }
            
            vec /= numEdges + 1;
            it->second.finalPosition = vec;
            
            vec.getValues(pos);
            op->pntMove(op->state, it->first, pos);
        }
    }

    if (query->mode(LWM_MODE_SYMMETRY))
        err = (*op->fastPointScan) (op->state, (fastPntScan_function)fastSymmPnts, &dat, OPLYR_FG, false);
    
	(*op->done) (op->state, err, 0);

	return err;
}
