#include "stdafx.h"
#include "modeltool.h"
#include <cstring>
#include <algorithm>
#include <string>

#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;

namespace osakanaengine 
{
	static const MODELFLOAT TWO_M_PI = 2.0 * M_PI;

    MODELPOINT MODELPOINTMake(MODELFLOAT x, MODELFLOAT y)
    {
        MODELPOINT p = {x,y};
        return p;
    }
    
    MODELPOINT* SubtPoint(const MODELPOINT* a, const MODELPOINT* b, MODELPOINT* c)
    {
        c->x = a->x - b->x;
        c->y = a->y - b->y;
        
        return c;
    }
    
    MODELFLOAT AngleBetween(const MODELPOINT* a, const MODELPOINT* b)
    {
        MODELFLOAT dot = DotProduct(a, b);
        MODELFLOAT cross = CrossProduct(a, b);
        
        return atan2f(cross, dot);
    }
    
    MODELFLOAT DotProduct(const MODELPOINT* a, const MODELPOINT* b)
    {
        return a->x * b->x + a->y * b->y;
    }
    
    MODELFLOAT CrossProduct(const MODELPOINT* a, const MODELPOINT* b)
    {
        return a->x * b->y - a->y * b->x;
    }
    
	MODELVEC3D MODELVEC3DMake(MODELFLOAT x, MODELFLOAT y, MODELFLOAT z)
	{
		MODELVEC3D vec3d;
		vec3d.x = x;
		vec3d.y = y;
		vec3d.z = z;

		return vec3d;
	}

	MODELVEC3D* NormalForce(const MODELFLOAT mass, const MODELVEC3D *normal, MODELVEC3D *ret)
	{
		MODELVEC3D projmGOntoS = {0.0};
		MODELVEC3D mGup = {0.0};

		ScaleVec(-mass, &G, &mGup);
		ProjBOntoA( normal, &mGup, &projmGOntoS);

		memcpy(ret, &projmGOntoS, sizeof(projmGOntoS));

		return ret;
	}

	MODELVEC3D* ProjBOntoA(const MODELVEC3D *a, const MODELVEC3D *b, MODELVEC3D *ret)
	{
		// projection of B onto A : projA_B = (B dot A)/|A|^2 * A
		MODELFLOAT ab = DotProduct(a, b);
		if(ab == 0.0)
		{
			memcpy(ret, b, sizeof(G));
		}
		else
		{
			MODELFLOAT aa = DotProduct(a, a);
			ScaleVec( ab/aa, a, ret);
		}
		return ret;
	}

	MODELFLOAT Magnitude(const MODELVEC3D *a)
	{
		return sqrt(pow(a->x, 2) + pow(a->y, 2) + pow(a->z, 2));
	}

	MODELFLOAT DotProduct(const MODELVEC3D *a, const MODELVEC3D *b)
	{
		return a->x * b->x + a->y * b->y + a->z * b->z;
	}

	MODELVEC3D* CrossProduct(const MODELVEC3D *a, const MODELVEC3D *b, MODELVEC3D *ret)
	{
		ret->x = a->y * b->z - b->y * a->z;//y1*z2 - y2*z1;
		ret->y = a->z * b->x - b->z * a->x;//z1*x2 - z2*x1;
		ret->z = a->x * b->y - b->x * a->y;//x1*y2 - x2*y1;

		return ret;
	}

	MODELVEC3D* ScaleVec(const MODELFLOAT f, const MODELVEC3D *v, MODELVEC3D *ret)
	{
		ret->x = f * v->x;
		ret->y = f * v->y;
		ret->z = f * v->z;

		return ret;
	}

	MODELVEC3D* AddVec(const MODELVEC3D *a, const MODELVEC3D *b, MODELVEC3D *ret)
	{
		ret->x = a->x + b->x;
		ret->y = a->y + b->y;
		ret->z = a->z + b->z;

		return ret;
	}

	MODELVEC3D* AddMultiVec(MODELVEC3D *ret, const int num, ...)
	{
		MODELVEC3D temp = {0.0};

		va_list ap;
		va_start(ap, num);
		for( int i=0; i < num; i++)
		{
			MODELVEC3D *p = va_arg(ap, MODELVEC3D*);
			temp.x += p->x;
			temp.y += p->y;
			temp.z += p->z;
		}

		memcpy(ret, &temp, sizeof(temp));

		return ret;
	}

	MODELVEC3D* SubtMultiVec(MODELVEC3D *ret, const int num, ...)
	{
		if(num == 0)
			return ret;

		MODELVEC3D temp = {0.0};
		va_list ap;
		va_start(ap, num);

		MODELVEC3D *p = va_arg(ap, MODELVEC3D*);
		memcpy(&temp, p, sizeof(temp));

		for( int i=1; i < num; i++)
		{
			p = va_arg(ap, MODELVEC3D*);
			temp.x -= p->x;
			temp.y -= p->y;
			temp.z -= p->z;
		}

		memcpy(ret, &temp, sizeof(temp));

		return ret;
	}

	MODELVEC3D* SubtVec(const MODELVEC3D *a, const MODELVEC3D *b, MODELVEC3D *ret)
	{
		ret->x = a->x - b->x;
		ret->y = a->y - b->y;
		ret->z = a->z - b->z;

		return ret;
	}

	MODELVEC3D* RotateY(const MODELVEC3D *src, const MODELFLOAT angle, MODELVEC3D *ret)
	{
		MODELFLOAT c = cos(angle);
		MODELFLOAT s = sin(angle);

		MODELFLOAT m00 = c;
		MODELFLOAT m01 = 0.0;
		MODELFLOAT m02 = s;

		MODELFLOAT m10 = 0.0;
		MODELFLOAT m11 = 1.0;
		MODELFLOAT m12 = 0.0;

		MODELFLOAT m20 = -s;
		MODELFLOAT m21 = 0.0;
		MODELFLOAT m22 = c;

		ret->x = m00 * src->x + m01 * src->y + m02 * src->z;
		ret->y = m10 * src->x + m11 * src->y + m12 * src->z;
		ret->z = m20 * src->x + m21 * src->y + m22 * src->z;

		return ret;
	}

	MODELVEC3D* FreeRotate(const MODELVEC3D *src, const MODELVEC3D *axis, const MODELFLOAT angle, MODELVEC3D *ret)
	{
		// get rotation matrix
		MODELFLOAT cosine = cos(angle);
		MODELFLOAT one_sub_cos = 1 - cosine;
		MODELFLOAT sine = sin(angle);
		MODELFLOAT x = axis->x;
		MODELFLOAT y = axis->y;
		MODELFLOAT z = axis->z;
		MODELFLOAT xx = pow(x,2);
		MODELFLOAT xy = x * y;
		MODELFLOAT xz = x * z;
		MODELFLOAT yy = pow(y,2);
		MODELFLOAT yz = y * z;
		MODELFLOAT zz = pow(z,2);
		MODELFLOAT xsine = x * sine;
		MODELFLOAT ysine = y * sine;
		MODELFLOAT zsine = z * sine;

		MODELFLOAT m00 = xx * one_sub_cos + cosine;
		MODELFLOAT m01 = xy * one_sub_cos - zsine;
		MODELFLOAT m02 = xz * one_sub_cos + ysine;

		MODELFLOAT m10 = xy * one_sub_cos + zsine;
		MODELFLOAT m11 = yy * one_sub_cos + cosine;
		MODELFLOAT m12 = yz * one_sub_cos - xsine;

		MODELFLOAT m20 = xz * one_sub_cos - ysine;
		MODELFLOAT m21 = yz * one_sub_cos + xsine;
		MODELFLOAT m22 = zz * one_sub_cos + cosine;

		ret->x = m00 * src->x + m01 * src->y + m02 * src->z;
		ret->y = m10 * src->x + m11 * src->y + m12 * src->z;
		ret->z = m20 * src->x + m21 * src->y + m22 * src->z;

		return ret;
	}

	MODELVEC3D* Normalize(const MODELVEC3D *src, MODELVEC3D *ret)
	{
		memcpy(ret, src, sizeof(G));
		MODELFLOAT mag = Magnitude(src);
		if(mag == 0.0)
			return NULL;

		ret->x = src->x / mag;
		ret->y = src->y / mag;
		ret->z = src->z / mag;

		return ret;
	}

	MODELFLOAT SubtFriction(const MODELFLOAT force, const MODELFLOAT friction)
	{
		bool wasPositive = true;
		MODELFLOAT subt = 0.0;
		
		wasPositive = IS_POSITIVE(force);
		subt = force - friction;
		if(IS_POSITIVE(subt) == wasPositive)
			return subt;

		return (MODELFLOAT)0.0;
	}

	MODELVEC3D* SubtFriction(const MODELVEC3D *force, const MODELVEC3D *friction, MODELVEC3D *ret)
	{
		ret->x = SubtFriction(force->x, friction->x);
		ret->y = SubtFriction(force->y, friction->y);
		ret->z = SubtFriction(force->z, friction->z);

		return ret;
	}

	MODELVEC3D* SubtFriction(const MODELVEC3D *force, const MODELFLOAT friction, MODELVEC3D *ret)
	{
		MODELFLOAT fmag = Magnitude(force);
		if(fmag < friction)
		{
			memset(ret, 0, 3);
		}
		else
		{
			MODELFLOAT rate = (fmag-friction) / fmag;
			ScaleVec(rate, force, ret);
		}

		return ret;
	}

	MODELFLOAT GripTurn(const MODELFLOAT centripetal, const MODELFLOAT grip)
	{
		if(fabs(centripetal) <= fabs(grip))
			return centripetal;

		return grip - centripetal;
	}

	/**
	* centripetal and friction must be same direction
	*/
	MODELVEC3D* GripTurn(const MODELVEC3D *centripetal, const MODELVEC3D *grip, MODELVEC3D *ret)
	{
		ret->x = GripTurn(centripetal->x, grip->x);
		ret->y = GripTurn(centripetal->y, grip->y);
		ret->z = GripTurn(centripetal->z, grip->z);

		return ret;
	}

	MODELFLOAT AngleBetween(const MODELVEC3D *a, const MODELVEC3D *b)
	{
		MODELFLOAT a_dot_b = DotProduct(a, b);
		MODELFLOAT magA = Magnitude(a);
		MODELFLOAT magB = Magnitude(b);
		MODELFLOAT denom = magA * magB;
		if(denom == 0.0)
			return 0.0;

		return acos(a_dot_b / denom);
	}

	MODELVEC3D* EulerMethod(const MODELVEC3D *Rhs, const MODELVEC3D *Xcur, const MODELFLOAT dt, MODELVEC3D *Xnext)
	{
		Xnext->x = Rhs->x * dt + Xcur->x;
		Xnext->y = Rhs->y * dt + Xcur->y;
		Xnext->z = Rhs->z * dt + Xcur->z;

		return Xnext;
	}

	/**
	* R is unit vector
	*/
	MODELVEC3D* GetTorque(const MODELVEC3D *R, const MODELFLOAT r, const MODELVEC3D *F, MODELVEC3D *ret)
	{
		MODELVEC3D Rvec = {0.0};
		ScaleVec(r, R, &Rvec);
		CrossProduct(&Rvec, F, ret);

		return ret;
	}

	MODELVEC3D* MatTrans(const MODELVEC3D *X, const MODELFLOAT MAT[3][3], MODELVEC3D *ret)
	{
		ret->x = X->x * MAT[0][0] + X->y * MAT[0][1] + X->z * MAT[0][2] ;
		ret->y = X->x * MAT[1][0] + X->y * MAT[1][1] + X->z * MAT[1][2] ;
		ret->z = X->x * MAT[2][0] + X->y * MAT[2][1] + X->z * MAT[2][2] ;

		return ret;
	}

	MODELVEC3D* RotateAroundX(const MODELVEC3D *X, const MODELFLOAT theta, MODELVEC3D *ret)
	{
		MODELFLOAT cosine = cos(theta);
		MODELFLOAT sine = sin(theta);
		MODELFLOAT MAT[3][3] = {
			{1.0,	0.0,		0.0},
			{0.0,	cosine,		-sine},
			{0.0,	sine,		cosine}
		};

		MatTrans(X, MAT, ret);

		return ret;
	}

	MODELVEC3D* RotateAroundY(const MODELVEC3D *X, const MODELFLOAT theta, MODELVEC3D *ret)
	{
		MODELFLOAT cosine = cos(theta);
		MODELFLOAT sine = sin(theta);
		MODELFLOAT MAT[3][3] = {
			{cosine,	0.0,	sine},
			{0.0,		1.0,	0.0},
			{-sine,		0.0,	cosine}
		};

		MatTrans(X, MAT, ret);

		return ret;
	}

	MODELVEC3D* RotateAroundZ(const MODELVEC3D *X, const MODELFLOAT theta, MODELVEC3D *ret)
	{
		MODELFLOAT cosine = cos(theta);
		MODELFLOAT sine = sin(theta);
		MODELFLOAT MAT[3][3] = {
			{cosine,	-sine,		0.0},
			{sine,		cosine,		0.0},
			{0.0,		0.0,		1.0}
		};

		MatTrans(X, MAT, ret);

		return ret;
	}

    MODELVEC3D* RotateAroundXYZ(const MODELVEC3D *X, const MODELVEC3D* angle, MODELVEC3D *ret)
    {
        MODELVEC3D rotX, rotY;
        RotateAroundX(X,     angle->x, &rotX);
        RotateAroundY(&rotX, angle->y, &rotY);
        RotateAroundZ(&rotY, angle->z, ret);
    
        return ret;
    }

	MODELVEC3D* AngleToDirY(const MODELVEC3D *Angle, MODELVEC3D *dir)
	{
		RotateAroundY(&ZAXIS, Angle->y, dir);

		return dir;
	}

	MODELFLOAT NormalizeAngle(const MODELFLOAT original)
	{
		MODELFLOAT a = fmod(original, TWO_M_PI);

		MODELFLOAT pinum = (double)(int)(a / M_PI);
		MODELFLOAT ans = (-TWO_M_PI) * pinum + a;

		return ans;
	}

	MODELVEC3D* NormalizeAngle(MODELVEC3D *angle)
	{
		angle->x = NormalizeAngle(angle->x);
		angle->y = NormalizeAngle(angle->y);
		angle->z = NormalizeAngle(angle->z);

		return angle;
	}

	bool IsInsideBox(const MODELVEC3D* pos, const MODELVEC3D box[4])
	{
		int numPositive = 0;

		for(int i = 0; i < 4; i++)
		{
			MODELVEC3D ab, ap;
			SubtVec(&(box[(i+1)%4]), &(box[i]), &ab);
			SubtVec(pos, &(box[i]), &ap);

			MODELFLOAT ab_ap = ab.x * ap.z - ap.x * ab.z;
			if(IS_POSITIVE(ab_ap))
				numPositive++;
		}

		return (numPositive == 0 || numPositive == 4);
	}

	MODELFLOAT Distance(const MODELVEC3D *a, const MODELVEC3D *b)
	{
		MODELVEC3D c;
		SubtVec( b, a, &c);
		return sqrt(pow(c.x, 2)+pow(c.y,2)+pow(c.z,2));
	}

	bool GetIntersection(const MODELVEC3D *a, const MODELVEC3D *b, const MODELVEC3D *c, const MODELVEC3D *d, MODELVEC3D* ret)
	{
		MODELFLOAT bxmax = b->x - a->x;
		MODELFLOAT dxmcx = d->x - c->x;
		MODELVEC3D cand;
		if(!IS_ALMOST_ZERO(bxmax) && !IS_ALMOST_ZERO(dxmcx))
		{
			// Ax-y+C=0
			MODELFLOAT A = (b->z - a->z)/bxmax;
			MODELFLOAT C = (a->z * b->x - a->x * b->z)/bxmax;

			// Dx-y+F=0
			MODELFLOAT D = (d->z - c->z)/dxmcx;
			MODELFLOAT F = (c->z * d->x - c->x * d->z)/dxmcx;

			if(A-D == 0.0)
				return false;

			cand.x = (F-C)/(A-D);
			cand.y = 0.0;
			cand.z = cand.x * A + C;
		}
		else if(IS_ALMOST_ZERO(bxmax) && !IS_ALMOST_ZERO(dxmcx))
		{
			// line A is perpendicular to X axis
			MODELFLOAT x1 = b->x;

			// Dx-y+F=0
			MODELFLOAT D = (d->z - c->z)/dxmcx;
			MODELFLOAT F = (c->z * d->x - c->x * d->z)/dxmcx;

			cand.x = x1;
			cand.y = 0.0;
			cand.z = cand.x * D + F;
		}
		else if(!IS_ALMOST_ZERO(bxmax) && IS_ALMOST_ZERO(dxmcx))
		{
			MODELFLOAT x2 = c->x;

			// Ax-y+C=0
			MODELFLOAT A = (b->z - a->z)/bxmax;
			MODELFLOAT C = (a->z * b->x - a->x * b->z)/bxmax;

			cand.x = x2;
			cand.y = 0.0;
			cand.z = cand.x * A + C;
		}
		else
		{
			return false;
		}

		if(min(a->x ,b->x) <= cand.x && cand.x <= max(a->x, b->x))
		{
			if(min(a->z, b->z) <= cand.z && cand.z <= max(a->z, b->z))
			{
				*ret = cand;
				return true;
			}
		}

		return false;
	}

	void ApplyReflection(const MODELVEC3D *prevPos, const MODELVEC3D *nextPos, const MODELVEC3D *is, const MODELVEC3D *a, const MODELVEC3D *b, MODELVEC3D *vel)
	{
        MODELVEC3D posVec = {0.0};
        SubtVec(nextPos, prevPos, &posVec);
        
        MODELVEC3D ab = {0.0};
        SubtVec(a, b, &ab);
        
        MODELVEC3D prj = { 0.0};
        ProjBOntoA(&ab, &posVec, &prj);
        
        MODELVEC3D xp = {0.0};
        CrossProduct(&prj, &posVec, &xp);
        
        // decompose vec into posVec direction and something else
        MODELVEC3D veloIntoPosVec = {0.0};
        MODELVEC3D veloIntoTheOther = {0.0};
        ProjBOntoA(&posVec, vel, &veloIntoPosVec);
        SubtVec(vel, &veloIntoPosVec, &veloIntoTheOther);
        
        // if posVec is left of prj, rotate negative angle
        MODELFLOAT angle = -2.0*SIGN(xp.y);
        MODELVEC3D veloIntoPosVecRef = {0.0};
        RotateY(&veloIntoPosVec, angle, &veloIntoPosVecRef);
        
        AddVec(&veloIntoPosVecRef, &veloIntoTheOther, vel);
	}

	void RotateLine(const MODELFLOAT angle, const MODELVEC3D *center, MODELLINE* line)
	{
		MODELLINE rotated;
		for(int i = 0; i < 2; i++)
		{
			MODELVEC3D rel_vec = {0.0};
			SubtVec(&line->v[i], center, &rel_vec);

			MODELVEC3D rel_rot = {0.0};
			RotateAroundY(&rel_vec, angle, &rel_rot);
			AddVec(&rel_rot, center, &line->v[i]);
		}
		memcpy(&rotated, line, sizeof(rotated));
	}

	void RotateRect(const MODELFLOAT angle, const MODELVEC3D *center, MODELRECT* rect)
	{
		MODELRECT rotated;
		for(int i = 0; i < 4; i++)
		{
			MODELVEC3D rel_vec = {0.0};
			SubtVec(&rect->v[i], center, &rel_vec);

			MODELVEC3D rel_rot = {0.0};
			RotateAroundY(&rel_vec, angle, &rel_rot);
			AddVec(&rel_rot, center, &rect->v[i]);
		}
		memcpy(&rotated, rect, sizeof(rotated));
	}
    
    bool StringEndsWith(std::string const &fullString, std::string const &ending)
    {
        if (fullString.length() >= ending.length()) {
            return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
        } else {
            return false;
        }
    }
}