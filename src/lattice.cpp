#include "lattice.h"
#include <lwmodtool.h>
#include <lwserver.h>
#include "sharedGlobals.h"


typedef struct st_LatticeData {
	MeshEditOp		*op;
	LatticeTool     *tool;
    Vector3F S, T, U;
    int meshPointCounter;
} LatticeData;


void LatticeTool::refreshLatticeCtlPoints()
{
    ctlPoints.clear();
    
    Vector3F direction = maxBBox - minBBox;
    
    ctlPoints.resize(x_div * y_div * z_div);
    for (int x=0; x < x_div; ++x)
        for (int y=0; y < y_div; ++y)
            for (int z=0; z < z_div; ++z)
            {
                int index = getIndex(x, y, z);
                ctlPoints[index].setValues(direction.x() * ((float) x) / (x_div - 1) + minBBox.x(),
                                                       direction.y() * ((float) y) / (y_div - 1) + minBBox.y(),
                                                       direction.z() * ((float) z) / (z_div - 1) + minBBox.z());
            }

}

bool LatticeTool::isX0Handle(int index)
{
    for (int y=0; y < y_div; ++y)
        for (int z=0; z < z_div; ++z)
        {
            if (index == y * x_div + z * x_div * y_div)
                return true;
        }
    return false;
}

int LatticeTool::getIndex(int x, int y, int z)
{
    return x + y * x_div + z * x_div * y_div;
}

long double fac(int n)
{
	if (n == 0 || n == 1)
		return 1;
	else
		return n * fac(n-1) ;
}

float C(int i, int l)
{
	return fac(l) / (fac(i) * fac(l - i));
}

float B(int i, int l, float s)
{
	return C(i, l) * pow(s,i) * pow(1-s,l-i);
}

void findBoundaryCells(const double w, const int D, int &min, int &max)
{
	double step = 1.0/(D - 1);
	
	for (int i = 0; i < D - 1; i++)
	{
		double thisStep = step * i;
		if ((w < thisStep + step) && (w >= thisStep))
		{
			min = i;
			max = i+1;
			return;
		}
	}
    
    //in case we are outside the case (LEFT)
    if (w < 0)
    {
        min = 0;
        max = 1;
        return;
    }
    
    //in case we are outside the case (RIGHT)
	if (D > 2)
		min = D - 2;
	else
		min = D - 1;
	
	max = D - 1;
}

int fastPolyControlCache(LatticeData *dat, LWPolID id)
{
    const LWPntID *points;
    int count = dat->op->polyPoints(dat->op->state, id, &points);
    
    LWDVector pos;
    for (int i = 0; i < count; ++i)
    {
        dat->op->pointPos(dat->op->state, points[i], pos);
        
        if (query->mode(LWM_MODE_SYMMETRY) && pos[0] < 0)
            continue;

        dat->tool->points.insert(points[i]);
    }
    
    return EDERR_NONE;
}

int fastPointControlCache(LatticeData *dat, LWPntID id)
{
    LWDVector pos;
    dat->op->pointPos(dat->op->state, id, pos);
    
    if (query->mode(LWM_MODE_SYMMETRY) && pos[0] < 0)
        return EDERR_NONE;
    
    dat->tool->points.insert(id);
    return EDERR_NONE;
}

void cachePoint(LatticeData *dat, LWPntID id)
{
    LatticeTool *tool = dat->tool;
    
    LWDVector pos;
    dat->op->pointPos(dat->op->state, id, pos);
    
    double s = dat->S.x() == 0 ? 0 : (pos[0] - tool->minBBox.x()) / dat->S.x();
    double t = dat->T.y() == 0 ? 0 : (pos[1] - tool->minBBox.y()) / dat->T.y();
    double u = dat->U.z() == 0 ? 0 : (pos[2] - tool->minBBox.z()) / dat->U.z();
    
    //BEZIER
    std::vector<float> weights;
    weights.resize(tool->x_div * tool->y_div * tool->z_div);
    
    float X, Y;
    int index;
    for ( unsigned i = 0; i < tool->x_div; i++ )
    {
        X = B(i, tool->x_div - 1, s);
        for ( unsigned j = 0; j < tool->y_div; j++ )
        {
            Y = B(j, tool->y_div - 1, t);
            for ( unsigned k = 0; k < tool->z_div; k++ )
            {
                index = tool->getIndex(i, j, k);
                weights[index] = X * Y * B(k, tool->z_div - 1, u);
            }
        }
    }
    
    PointCache *pc = &tool->pointCache[dat->meshPointCounter];
    
    pc->layerNum = dat->op->pointLayer(dat->op->state, id);
    pc->weights = weights;
    pc->id = id;
    pc->pos[0] = pos[0];
    pc->pos[1] = pos[1];
    pc->pos[2] = pos[2];
    
    //LINEAR
    findBoundaryCells(s, dat->tool->x_div, pc->minX, pc->maxX);
    findBoundaryCells(t, dat->tool->y_div, pc->minY, pc->maxY);
    findBoundaryCells(u, dat->tool->z_div, pc->minZ, pc->maxZ);
    
    int factorX = tool->x_div - 1;
    int factorY = tool->y_div - 1;
    int factorZ = tool->z_div - 1;
    pc->sLocal = (s - float(pc->minX) / factorX) / (float(pc->maxX) / factorX - float(pc->minX) / factorX);
    pc->tLocal = (t - float(pc->minY) / factorY) / (float(pc->maxY) / factorY - float(pc->minY) / factorY);
    pc->uLocal = (u - float(pc->minZ) / factorZ) / (float(pc->maxZ) / factorZ - float(pc->minZ) / factorZ);
    
    dat->meshPointCounter++;
}

void LatticeTBBData::operator()( const tbb::blocked_range<size_t>& r ) const
{
    LatticeTool *tool = m_data.tool;
    
    for( size_t i=r.begin(); i!=r.end(); ++i )
    {
        PointCache *pc = &tool->pointCache[i];
        pc->destPos[0] = 0; pc->destPos[1] = 0; pc->destPos[2] = 0;
        
        if (tool->interpolation == 0)
        {
            Vector3F *p1 = &tool->ctlPoints[tool->getIndex(pc->minX, pc->minY, pc->minZ)];
            Vector3F *p2 = &tool->ctlPoints[tool->getIndex(pc->minX, pc->maxY, pc->minZ)];
            Vector3F *p3 = &tool->ctlPoints[tool->getIndex(pc->maxX, pc->minY, pc->minZ)];
            Vector3F *p4 = &tool->ctlPoints[tool->getIndex(pc->maxX, pc->maxY, pc->minZ)];
            
            Vector3F p21 = (*p2 - *p1) * pc->tLocal + *p1;
            Vector3F p43 = (*p4 - *p3) * pc->tLocal + *p3;
            Vector3F firstPoint = (p43 - p21) * pc->sLocal + p21;
            
            p1 = &tool->ctlPoints[tool->getIndex(pc->minX, pc->minY, pc->maxZ)];
            p2 = &tool->ctlPoints[tool->getIndex(pc->minX, pc->maxY, pc->maxZ)];
            p3 = &tool->ctlPoints[tool->getIndex(pc->maxX, pc->minY, pc->maxZ)];
            p4 = &tool->ctlPoints[tool->getIndex(pc->maxX, pc->maxY, pc->maxZ)];
            
            p21 = (*p2 - *p1) * pc->tLocal + *p1;
            p43 = (*p4 - *p3) * pc->tLocal + *p3;
            Vector3F secondPoint = (p43 - p21) * pc->sLocal + p21;
            
            Vector3F finalPoint = (secondPoint - firstPoint) * pc->uLocal + firstPoint;
            
            pc->destPos[0] = finalPoint.x();
            pc->destPos[1] = finalPoint.y();
            pc->destPos[2] = finalPoint.z();
            
        }
        else
        {
            const std::vector<float>& weights = tool->pointCache[i].weights;
            for (int index = 0; index < tool->x_div * tool->y_div * tool->z_div; index++ )
            {
                pc->destPos[0] += tool->ctlPoints[index].x() * weights[index];
                pc->destPos[1] += tool->ctlPoints[index].y() * weights[index];
                pc->destPos[2] += tool->ctlPoints[index].z() * weights[index];
            }
        }
    }
};

EDError initialize(LatticeData &data)
{
    EDError err = EDERR_NONE;
    switch (query->mode(LWM_MODE_SELECTION))
    {
            //points
        case 0:
            err = (data.op->fastPointScan) (data.op->state, (fastPntScan_function)fastPointControlCache, &data, OPLYR_FG, true);
            break;
            //polys
        case 1:
            err = (data.op->fastPolyScan) (data.op->state, (fastPolyScan_function)fastPolyControlCache, &data, OPLYR_FG, true);
            break;
        default:
            break;
    }
    
    if (data.tool->points.size() == 0)
        err = (data.op->fastPointScan) (data.op->state, (fastPntScan_function)fastPointControlCache, &data, OPLYR_FG, false);
    data.tool->numpoints = data.tool->points.size();
    
    bool first = true;
    LWDVector pos;
    for (std::set<LWPntID>::iterator it = data.tool->points.begin(); it != data.tool->points.end(); ++it)
    {
        data.op->pointPos(data.op->state, *it, pos);
        
        if (first)
        {
            data.tool->minBBox.setValues(pos[0], pos[1], pos[2]);
            data.tool->maxBBox.setValues(pos[0], pos[1], pos[2]);
            first = false;
            continue;
        }
        
        if (pos[0] < data.tool->minBBox.x())
            data.tool->minBBox.setX(pos[0]);
        
        if (pos[1] < data.tool->minBBox.y())
            data.tool->minBBox.setY(pos[1]);
        
        if (pos[2] < data.tool->minBBox.z())
            data.tool->minBBox.setZ(pos[2]);
        
        if (pos[0] > data.tool->maxBBox.x())
            data.tool->maxBBox.setX(pos[0]);
        
        if (pos[1] > data.tool->maxBBox.y())
            data.tool->maxBBox.setY(pos[1]);
        
        if (pos[2] > data.tool->maxBBox.z())
            data.tool->maxBBox.setZ(pos[2]);
    }
    
    data.tool->refreshLatticeCtlPoints();
    
    return err;
}

int fastSymmPnts(LatticeData *dat, const LWPntID pnt)
{
    LWDVector pntPos;
    dat->op->pointPos(dat->op->state, pnt, pntPos);
    if (pntPos[0] >= 0)
        return EDERR_NONE;
    
    for (int i = 0; i < dat->tool->numpoints; ++i)
    {
        PointCache *pc = &dat->tool->pointCache[i];
        if (pc->pos[0] < 0 && pnt != pc->id)
            continue;
        
        if ((pntPos[0] + pc->pos[0]) * (pntPos[0] + pc->pos[0]) <= 0.0000025  && pntPos[1] == pc->pos[1] && pntPos[2] == pc->pos[2])
        {
            SymmPoint sp;
            sp.id = pnt;
            sp.pcIndex = i;
            sp.layerNum = dat->op->pointLayer(dat->op->state, pnt);
            dat->tool->symmCache.push_back(sp);
            return EDERR_NONE;
        }
    }
    
    return EDERR_NONE;
}

int Lattice(MeshEditOp *op, LatticeTool *tool)
{
	EDError err = EDERR_NONE;
    
    LatticeData data;
    data.op = op;
    data.tool = tool;
    data.meshPointCounter = 0;
    
    if (tool->justActivated)
    {
        err = initialize(data);
        tool->justActivated = false;
        return err;
    }
    
    Vector3F direction = tool->maxBBox - tool->minBBox;
    data.S.setValues(direction.x(), 0, 0);
    data.T.setValues(0, direction.y(), 0);
    data.U.setValues(0, 0, direction.z());

    if (!tool->hasCache)
    {
        tool->pointCache.resize(tool->numpoints);
        for (std::set<LWPntID>::iterator it = tool->points.begin(); it != tool->points.end(); ++it)
            cachePoint(&data, *it);
        
        if (query->mode(LWM_MODE_SYMMETRY))
            err = (*op->fastPointScan) (op->state, (fastPntScan_function)fastSymmPnts, &data, OPLYR_FG, false);
        
        tool->hasCache = true;
    }
    
    LatticeTBBData dataObj(tool);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, tool->numpoints), dataObj);

    for (unsigned int i = 0; i < tool->pointCache.size(); ++i)
    {
        op->setLayer( op->state,  tool->pointCache[i].layerNum);
        op->pntMove(op->state, tool->pointCache[i].id, tool->pointCache[i].destPos);
    }
    
    if (query->mode(LWM_MODE_SYMMETRY))
    {
        for (unsigned int i = 0; i < tool->symmCache.size(); ++i)
        {
            LWDVector destPos;
            VCPY(destPos, tool->pointCache[tool->symmCache[i].pcIndex].destPos);
            destPos[0] = -destPos[0];
            
            op->setLayer( op->state, tool->symmCache[i].layerNum);
            op->pntMove(op->state, tool->symmCache[i].id, destPos);
        }
    }
    
	(*op->done) (op->state, err, 0);

	return err;
}
