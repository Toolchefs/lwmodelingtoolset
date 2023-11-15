/*
 *  Vector3F.h
 *  StretchDrag
 *
 *  Created by Daniele Federico on 21/03/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */
#include <sstream>
#include <string>

class Vector3F{
private:
	LWFVector vs;
public:
	Vector3F(float x=0, float y=0, float z=0){vs[0]=x; vs[1]=y; vs[2]=z;}
	void getValues(Vector3F &v) const {v.setX(vs[0]); v.setY(vs[1]); v.setZ(vs[2]);}
	void setValues(const Vector3F &v) {vs[0]=v.x(); vs[1]=v.y(); vs[2]=v.z();}
	void setValues(double v[3]) {vs[0]=v[0]; vs[1]=v[1]; vs[2]=v[2];}
    void setValues(float v[3]) {vs[0]=v[0]; vs[1]=v[1]; vs[2]=v[2];}
	void getValues(double v[3])  const {v[0]=vs[0]; v[1]=vs[1]; v[2]=vs[2];}	
	void getValues(float v[3])  const {v[0]=vs[0]; v[1]=vs[1]; v[2]=vs[2];}	
	double x() const {return vs[0];}
	double y() const {return vs[1];}
	double z() const {return vs[2];}
	void setX(double x){vs[0] = x;}
	void setY(double y){vs[1] = y;}
	void setZ(double z){vs[2] = z;}
	void setValues(double x, double y, double z){vs[0]=x; vs[1]=y; vs[2]=z;}
	const double length() const {return (sqrt(vs[0]*vs[0] + vs[1]*vs[1] + vs[2]*vs[2]));}
	const double dot(const Vector3F &v) const {return vs[0]*v.x() + vs[1]*v.y() + vs[2]*v.z();}
	void normalize(){double lenght = this->length(); vs[0] /= lenght; vs[1] /= lenght; vs[2] /= lenght;}
    void invert(){vs[0] = -vs[0]; vs[1] = -vs[1]; vs[2] = -vs[2];};
    
    Vector3F cross (const Vector3F &v) const{ return Vector3F (vs[1] * v.z() - vs[2] * v.y(), vs[2] * v.x() - vs[0] * v.z(), vs[0] * v.y() - vs[1] * v.x());}
	
	void add(const double x, const double y, const double z){vs[0]+=x; vs[1]+=y; vs[2]+=z;}
	void add(const Vector3F &vec){vs[0]+=vec.x(); vs[1]+=vec.y(); vs[2]+=vec.z();}
	void add(const double offset[3]){vs[0]+=offset[0]; vs[1]+=offset[1]; vs[2]+=offset[2];}
	
    Vector3F operator- () const {return Vector3F(-vs[0], -vs[1], -vs[2]);};
	Vector3F operator- (const Vector3F &v) const {return Vector3F(vs[0]-v.x(), vs[1]-v.y(), vs[2]-v.z());}
	Vector3F operator* (const double factor) const {return Vector3F(vs[0]*factor, vs[1]*factor, vs[2]*factor);}
    double operator* (const Vector3F &v) const {return vs[0]*v.x() + vs[1]*v.y() + vs[2]*v.z();}
	Vector3F operator+ (const Vector3F &v) const {return Vector3F(vs[0]+v.x(), vs[1]+v.y(), vs[2]+v.z());}
	void operator*= (const double factor) {vs[0]*=factor; vs[1]*=factor; vs[2]*=factor;}
	void operator/=(const double factor) {vs[0]/=factor; vs[1]/=factor; vs[2]/=factor;}
	void operator+=(const Vector3F &v) {vs[0]+=v.x(); vs[1]+=v.y(); vs[2]+=v.z();}
    void operator-= (const Vector3F &v) {vs[0]-=v.x(); vs[1]-=v.y(); vs[2]-=v.z();}
	
	std::string asString() const {std::stringstream ss;	ss << "("; ss << vs[0]; ss << ", "; ss << vs[1]; ss << ", "; ss << vs[2]; ss << ")"; return ss.str();}
};