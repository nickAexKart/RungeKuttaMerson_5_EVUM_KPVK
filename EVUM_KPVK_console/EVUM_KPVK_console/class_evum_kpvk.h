////////////////////////////////////////////////////////////////////////////////~80
#ifndef CL_EVUM_KPVK_H
#define CL_EVUM_KPVK_H

#include <math.h>
#include <stdio.h>
#include <conio.h>
#include <string>
#include <iostream>

enum EQType {
	EQ_GROUND   = 0,
	EQ_PISTON_1 = 1,
	EQ_PISTON_2 = 2
};

enum EQComponent {
    EQ_COORD = 0,
    EQ_SPEED = 1
};

#define U32 unsigned int
#define U16 unsigned short int
#define U8  unsigned char

#define S32 signed int
#define S16 signed short int
#define S8  signed char

#define ERROR_RETURNS -999999999
#define MAX_TRY_COUNT 21
#define ZERO 0.00000000

/* ODE: X`` + 2*h*X` + (L^2)*X = -p , X=X(t)*/
/* ODS: { X` = V ; V` = -(2*h*V + (L^2)*X + p)}*/
struct resTable
{
	double _x;
	double _x1;
	double _x2;
	double _t;
	resTable() : _x(0.0), _x1(0.0), _x2(0.0), _t(0.0) {}
};

class CL_EVUM_KPVK
{
private:
	/* allowable error */
	double eps;
	/* system parameters */
	double l; /* ~lambda */
	double h;
	double h1;
	double p;
	double mu;
	double mu0;
	double phi;
	double gam;
    double R;
	/* step size */
	double H;
	/* speeds */
	double v;
	double v1;
	double v2;
	/* coordinates */
	double x;
	double x1;
	double x2;
	/* time */
	double t;
	double t0;
	double tn;
	/* result */
	resTable* resA = nullptr;
    U32 resArrLength;

public:
	/* Constructors */
    CL_EVUM_KPVK(double _x, double _x1, double _x2,
                 double _v, double _v1, double _v2,
                 double _t, double _t0, double _tn,
                 double _h, double _p, double _l,
                 double _h1, double mu, double mu0,
                 double _gam, double _eps, double _phi,
                 double _R);
    CL_EVUM_KPVK(void);
	/* ----- METHODS ----- */
    double motion_f(double _x, double _v, double _t, EQType pistonIx);
    double RKM_step(EQType pistonIx, EQComponent component, double* dif);
	resTable* extendArray(resTable* resA);
	void extendArray(void);
	void dynamicComputation(void);
	/* ----- INIT ----- */
	void initResTable(void);
	void initSysytem(void);
	/* ----- GET ----- */
	/* coordinates */
	double getX()  const{ return x; };
	double getX1() const{ return x1; };
	double getX2() const{ return x2; };
	/* speeds */
	double getV()  const{ return v; };
	double getV1() const{ return v1; };
	double getV2() const{ return v2; };
	/* params */
	double getH()   const{ return h; };
	double getH1()  const{ return h1; };
	double getP()   const{ return p; };
	double getL()   const{ return l; };
	double getMu()  const{ return mu; };
	double getMu0() const{ return mu0; };
	double getPhi() const{ return phi; };
	double getGam() const{ return gam; };
	/* time */
	double getT()      const{ return t; };
	double getBeginT() const{ return t0; };
	double getEndT()   const{ return tn; };
	/* ----- SET ----- */
	/* coordinates */
	void setX(double _x)	 { x = _x; };
	void setX1(double _x1)	 { x1 = _x1; };
	void setX2(double _x2)	 { x2 = _x2; };
	/* speeds */
	void setV(double _v)	 { v = _v; };
	void setV1(double _v1)	 { v1 = _v1; };
	void setV2(double _v2)	 { v2 = _v2; };
	/* params */
	void setH(double _h)     { h = _h; };
	void setH1(double _h1)    { h1 = _h1; };
	void setP(double _p)     { p = _p; };
	void setL(double _l)     { l = _l; };
	void setMu(double _mu)   { mu = _mu; };
	void setMu0(double _mu0) { mu0 = _mu0; };
	void setPhi(double _phi) { phi = _phi; };
	void setGam(double _gam) { gam = _gam; };
	void setEps(double _eps) { eps = _eps; };
	/* time */
	void setT(double _t)	 { t = _t; };
	void setT0(double _t0)	 { t0 = _t0; };
	void setTn(double _tn)	 { tn = _tn; };

};

#endif