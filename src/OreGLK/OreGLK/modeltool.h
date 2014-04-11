#ifndef __MODELTOOL_H__
#define __MODELTOOL_H__

#define _USE_MATH_DEFINES
#include <math.h>
#include <cstdarg>
#include <vector>
#include "ModelType.h"

#define GRAVITY_CONST	9.8
#define BIG_NUMBER		((MODELFLOAT)1e5)
#define SMALL_NUMBER	((MODELFLOAT)1e-5)
#define IS_POSITIVE(x)	(x >= 0.0)
#define SIGN(x)			((x)<0.0?-1.0:1.0)
#define IS_ALMOST_ZERO(x)	(fabs(x) < SMALL_NUMBER)
#define ZEROMEMORY(x)	memset(&x, 0, sizeof(x))
#define SAFE_DELETE(p)  { delete p; p=NULL; }

namespace osakanaengine {

	MODELVEC3D MODELVEC3DMake(MODELFLOAT x, MODELFLOAT y, MODELFLOAT z);

	static const MODELVEC3D G = { 0.0, -GRAVITY_CONST, 0.0};
	static const MODELVEC3D XAXIS = {1.0, 0.0, 0.0};
	static const MODELVEC3D YAXIS = {0.0, 1.0, 0.0};
	static const MODELVEC3D ZAXIS = {0.0, 0.0, 1.0};
    static const MODELVEC3D ZEROVEC3D = {0.0, 0.0, 0.0};

    MODELPOINT MODELPOINTMake(MODELFLOAT x, MODELFLOAT y);
    MODELPOINT* SubtPoint(const MODELPOINT* a, const MODELPOINT* b, MODELPOINT* c);
    MODELFLOAT AngleBetween(const MODELPOINT* a, const MODELPOINT* b);
    MODELFLOAT DotProduct(const MODELPOINT* a, const MODELPOINT* b);
    MODELFLOAT CrossProduct(const MODELPOINT* a, const MODELPOINT* b);
    
	MODELVEC3D* NormalForce(const MODELFLOAT mass, const MODELVEC3D *normal, MODELVEC3D *ret);
	MODELVEC3D* ProjBOntoA(const MODELVEC3D *a, const MODELVEC3D *b, MODELVEC3D *ret);
	MODELFLOAT Magnitude(const MODELVEC3D *a);
	MODELFLOAT DotProduct(const MODELVEC3D *a, const MODELVEC3D *b);
	MODELVEC3D* CrossProduct(const MODELVEC3D *a, const MODELVEC3D *b, MODELVEC3D *ret);
	MODELVEC3D* ScaleVec(const MODELFLOAT f, const MODELVEC3D *v, MODELVEC3D *ret);
	MODELVEC3D* AddVec(const MODELVEC3D *a, const MODELVEC3D *b, MODELVEC3D *ret);
	MODELVEC3D* SubtVec(const MODELVEC3D *a, const MODELVEC3D *b, MODELVEC3D *ret);
	MODELVEC3D* RotateY(const MODELVEC3D *src, const MODELFLOAT angle, MODELVEC3D *ret);
	MODELVEC3D* FreeRotate(const MODELVEC3D *src, const MODELVEC3D *axis, const MODELFLOAT angle, MODELVEC3D *ret);
	MODELVEC3D* Normalize(const MODELVEC3D *src, MODELVEC3D *ret);
	MODELFLOAT SubtFriction(const MODELFLOAT force, const MODELFLOAT friction);
	MODELVEC3D* SubtFriction(const MODELVEC3D *force, const MODELVEC3D *friction, MODELVEC3D *ret);
	MODELVEC3D* SubtFriction(const MODELVEC3D *force, const MODELFLOAT friction, MODELVEC3D *ret);
	MODELFLOAT GripTurn(const MODELFLOAT centripetal, const MODELFLOAT grip);
	MODELVEC3D* GripTurn(const MODELVEC3D *centripetal, const MODELVEC3D* grip, MODELVEC3D *ret);
	MODELFLOAT AngleBetween(const MODELVEC3D *a, const MODELVEC3D *b);
	MODELVEC3D* EulerMethod(const MODELVEC3D *Rhs, const MODELVEC3D *Xcur, const MODELFLOAT dt, MODELVEC3D *Xnext);
	MODELVEC3D* GetTorque(const MODELVEC3D *R, const MODELFLOAT r, const MODELVEC3D *F, MODELVEC3D *ret);
	MODELVEC3D* MatTrans(const MODELVEC3D *X, const MODELFLOAT MAT[3][3], MODELVEC3D *ret);
	MODELVEC3D* RotateAroundX(const MODELVEC3D *X, const MODELFLOAT theta, MODELVEC3D *ret);
	MODELVEC3D* RotateAroundY(const MODELVEC3D *X, const MODELFLOAT theta, MODELVEC3D *ret);
	MODELVEC3D* RotateAroundZ(const MODELVEC3D *X, const MODELFLOAT theta, MODELVEC3D *ret);
    MODELVEC3D* RotateAroundXYZ(const MODELVEC3D *X, const MODELVEC3D* angle, MODELVEC3D *ret);
	MODELVEC3D* AngleToDirY(const MODELVEC3D *Angle, MODELVEC3D *dir);
	MODELVEC3D* AddMultiVec(MODELVEC3D *ret, const int num, ...);
	MODELVEC3D* SubtMultiVec(MODELVEC3D *ret, const int num, ...);
	MODELFLOAT NormalizeAngle(const MODELFLOAT original);
	MODELVEC3D* NormalizeAngle(MODELVEC3D *angle);
	bool IsInsideBox(const MODELVEC3D* pos, const MODELVEC3D rect[4]);
	MODELFLOAT Distance(const MODELVEC3D *a, const MODELVEC3D *b);
	bool GetIntersection(const MODELVEC3D *a, const MODELVEC3D *b, const MODELVEC3D *c, const MODELVEC3D *d, MODELVEC3D* ret);
	void ApplyReflection(const MODELVEC3D *prevPos, const MODELVEC3D *nextPos, const MODELVEC3D *is, const MODELVEC3D *a, const MODELVEC3D *b, MODELVEC3D *vel);
	void RotateLine(const MODELFLOAT angle, const MODELVEC3D *center, MODELLINE* line);
	void RotateRect(const MODELFLOAT angle, const MODELVEC3D *center, MODELRECT* rect);

    template<class T>
    void DeletePointerVectors(std::vector<T> &vec)
    {
        while (!vec.empty()) {
            T p = vec.back();
            vec.pop_back();
            delete p;
        }
    }

    template<class T>
    bool RemoveFromVector(std::vector<T> &vec, T value)
    {
        typename std::vector<T>::iterator it = vec.end();
        
        it = std::find(vec.begin(), vec.end(), value);
        if(it != vec.end()) {
            vec.erase(it);
            return true;
        }

        return false;
    }
    
    bool StringEndsWith(std::string const &fullString, std::string const &ending);
}

#endif
