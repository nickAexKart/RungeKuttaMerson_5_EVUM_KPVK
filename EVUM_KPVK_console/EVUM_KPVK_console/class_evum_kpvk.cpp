////////////////////////////////////////////////////////////////////////////////~80
#include "class_evum_kpvk.h"
using namespace std;
CL_EVUM_KPVK::CL_EVUM_KPVK(double _x, double _x1, double _x2,
                           double _v, double _v1, double _v2,
                           double _t, double _t0, double _tn,
                           double _h, double _p,  double _l,
                           double _h1, double _mu, double _mu0,
                           double _gam, double _eps, double _phi,
                           double _R)
{
	x  = _x;
    x1 = _x1;
    x2 = _x2;
	v  = _v;
    v1 = _v1;
    v2 = _v2;
    h = _h;
    p = _p;
    l = _l;
    t = _t;
    t0 = _t0;
    tn = _tn;
    h1 = _h1;
    mu = _mu;
    mu0 = _mu0;
    gam = _gam;
    eps = _eps;
    phi = _phi;
    R = _R;	 
    H = 2*_eps;
    resArrLength = 0;
    initResTable();
}

CL_EVUM_KPVK::CL_EVUM_KPVK(void)
{
    x   = 0;
    x1  = 1;
    x2  = 1.2;
    v   = 0;
    v1  = 0.001;
    v2  = 0.002;
    h   = 0.1;
    h1  = 0.2;
    p   = 1.9;
    l   = 100.0;
    mu  = 0.1;
    mu0 = 0.2;
    t0  = ZERO;
    tn  = 4.0;
    eps = 0.1;
    gam = 1;
    phi = 0.785;
    R   = 0.8;
    H   = 2.0*eps;
    resArrLength;
    initResTable();
}

double
CL_EVUM_KPVK::motion_f(double _x, double _v, double _t, EQType pistonIx)
{
	switch(pistonIx)
	{
	case EQ_GROUND:
		return -(2.0*h1*_v + l*l*_x + p);
	case EQ_PISTON_1:
		return -(2.0*h*_v + mu*(cos(_t) + 2.0*h*sin(_t)) + p);
	case EQ_PISTON_2:
		return -(2.0*h*_v + mu*gam*(cos(_t - phi) + 2.0*h*sin(_t - phi)) + p);
	default:
	{
		std::cout << "ERROR: class_evum_kpvk.cpp: piston_f: invalid pistonIx="
				  << pistonIx;
		return ERROR_RETURNS;
	}
	}
};

/* double
CL_EVUM_KPVK::RKM_step(EQType pistonIx, EQComponent component, double* dif)
{
    double m1 = 0;
    double m2 = 0;
    double m3 = 0;
    double m4 = 0;
    double m5 = 0;

    if (component == EQ_COORD)
    {
        double _x;
        double _v;
        double fl = 0.0;
        double delta = 0.0;
        switch (pistonIx)
        {
        case EQ_GROUND:   { _x = x;  _v = v;  fl = 0.0; break; }
        case EQ_PISTON_1: { _x = x1; _v = v1; fl = 1.0; break; }
        case EQ_PISTON_2: { _x = x2; _v = v2; fl = 1.0; break; }
        default:
        {
            std::cout
                 << "ERROR: class_evum_kpvk.cpp: RKM_step: invalid pistonIx="
                 << pistonIx;
            return ERROR_RETURNS;
        }
        }

        m1 = motion_f(_x, _v, fl*t, pistonIx);
        delta = 1.5*m1;
        m2 = motion_f(_x + delta,
                      _v + delta,
                      fl*t,
                      pistonIx);
        delta = 0.16666667*H*m1 + 0.16666667*H*m2;
        m3 = motion_f(_x + delta,
                      _v + delta,
                      fl*(t + delta),
                      pistonIx);
        delta = 0.125*H*m1 + 1.5*H*m2;
        m4 = motion_f(_x + delta,
                      _v + delta,
                      fl*(t + delta),
                      pistonIx);
        delta = 0.5*H*m1 - 1.5*H*m3 + 2.0*H*m4;
        m5 = motion_f(_x + delta,
                      _v + delta,
                      fl*(t + delta),
                      pistonIx);
    }
    else if (EQ_SPEED)
    {
        double _v;

        switch (pistonIx)
        {
        case EQ_GROUND:   { _v = v;  break; }
        case EQ_PISTON_1: { _v = v1; break; }
        case EQ_PISTON_2: { _v = v2; break; }
        default:
        {
            std::cout
                 << "ERROR: class_evum_kpvk.cpp: RKM_step: invalid pistonIx="
                 << pistonIx;
            return ERROR_RETURNS;
        }
        }

        m1 = _v;
        m2 = _v + 1.5*m1;
        m3 = _v + 0.16666667*H*m1 + 0.16666667*H*m2;
        m4 = _v + 0.125*H*m1 + 1.5*H*m2;
        m5 = _v + 0.5*H*m1 - 1.5*H*m3 + 2.0*H*m4;
    }
    
    return abs(H*(0.16666667*(m1 + 4.0*m3 + m5) - 0.5*(m1 - 3.0*m3 + 4.0*m4)));    
}; */

void CL_EVUM_KPVK::initResTable(void)
{
    resArrLength = (U32)(abs((tn - t0) / pow(eps, 2)));
    std::cout << "initialising array: length=" << resArrLength << endl;
    std::cout << "initialising array: DONE" << endl;
    resA = new resTable[resArrLength];
	return;
}

void CL_EVUM_KPVK::extendArray(void)
{
	resTable* tmp = nullptr;
    U32 resLengthOld = resArrLength;
    U8  tryIx = 1;

    cout << "trying to extend array: curLength=" << resArrLength << endl;
	do{
        cout << "trying to extend array[" << tryIx
             << "]: curLength=" << resArrLength << endl;
        resArrLength = (U32)(1.5*resArrLength);
        tmp = new resTable[resArrLength];
        tryIx++;
	} while (tmp == nullptr);

    cout << "array is extended: newLength=" << resArrLength << endl;
    for (U32 i = 0; i < resLengthOld; i++)
	{
		tmp[i] = resA[i];
	}
    cout << "array is filled with data " << resLengthOld << endl;
	if (resA != nullptr)
	{
		delete[] resA;
		resA = nullptr;
	}
    cout << "extended array: DONE" << endl;
	resA = tmp;
}

void CL_EVUM_KPVK::dynamicComputation(void)
{   /* initialisation of RKM procedure */
    double mX0 = 0; double mV0 = 0;
    double mX1 = 0; double mV1 = 0;
    double mX2 = 0; double mV2 = 0;

    double dx  = 0; double dv  = 0;
    double dx1 = 0; double dv1 = 0;
    double dx2 = 0; double dv2 = 0;

    double m1 = 0; /* double tt1 = t */ 
    double m2 = 0; double tt2 = t + H/3.0;
    double m3 = 0; double tt3 = t + H/3.0;
    double m4 = 0; double tt4 = t + H*0.5;
    double m5 = 0; double tt5 = t + H;

    double lowEps =   eps / 64.0;
    double hightEps = eps * 5.0;
    double delta = 0;
    U32 ix = 0;
    U8 count = 1; /* = 0 - good Merson major numbers;
                   * > 0 - correction of step H is needed */
	H = eps*2.0;
	t = t0;

    resA[ix]._t  = t;
    resA[ix]._x  = x;
    resA[ix]._x1 = x1;
    resA[ix]._x2 = x2;
    std::cout << "    t         x         x1        x2\n";
    std::cout << "========= ========= ========= =========\n";
    printf("%.8lf %.8lf %.8lf %.8lf : %6u\n", t, x, x1, x2, ix);
	do
	{
        count = 1;
        do{
            /* ----- ground ----- */
            /* coord */
            m1 = motion_f(x, v, 0, EQ_GROUND);
            delta = 1.5*m1;
            m2 = motion_f(x + delta, v + delta, 0, EQ_GROUND);
            delta = 0.16666667*H*m1 + 0.16666667*H*m2;
            m3 = motion_f(x + delta, v + delta, 0, EQ_GROUND);
            delta = 0.125*H*m1 + 1.5*H*m2;
            m4 = motion_f(x + delta, v + delta, 0, EQ_GROUND);
            delta = 0.5*H*m1 - 1.5*H*m3 + 2.0*H*m4;
            m5 = motion_f(x + delta, v + delta, 0, EQ_GROUND);

            mX0 = abs(H*(0.16666667*(m1 + 4.0*m3 + m5) -
                         0.5*(m1 - 3.0*m3 + 4.0*m4)));
            dx  = 0.16666667*H*(m1 + 4 * m3 + m5);
            /* speed */
            m1 = v;
            m2 = v + 1.5*m1;
            m3 = v + 0.16666667*H*m1 + 0.16666667*H*m2;
            m4 = v + 0.125*H*m1 + 1.5*H*m2;
            m5 = v + 0.5*H*m1 - 1.5*H*m3 + 2.0*H*m4;

            mV0 = abs(H*(0.16666667*(m1 + 4.0*m3 + m5) -
                      0.5*(m1 - 3.0*m3 + 4.0*m4)));
            dv  = 0.16666667*H*(m1 + 4 * m3 + m5);
            /* ----- piston 1  -----*/
            /* coord */
            m1 = motion_f(x1, v1, t, EQ_PISTON_1);
            delta = 1.5*m1;
            m2 = motion_f(x1 + delta, v1 + delta, tt2, EQ_PISTON_1);
            delta = 0.16666667*H*m1 + 0.16666667*H*m2;
            m3 = motion_f(x1 + delta, v1 + delta, tt3, EQ_PISTON_1);
            delta = 0.125*H*m1 + 1.5*H*m2;
            m4 = motion_f(x1 + delta, v1 + delta, tt4, EQ_PISTON_1);
            delta = 0.5*H*m1 - 1.5*H*m3 + 2.0*H*m4;
            m5 = motion_f(x1 + delta, v1 + delta, tt5, EQ_PISTON_1);

            mX1 = abs(H*(0.16666667*(m1 + 4.0*m3 + m5) -
                      0.5*(m1 - 3.0*m3 + 4.0*m4)));
            dx1 = 0.16666667*H*(m1 + 4 * m3 + m5);
            /* speed */
            m1 = v1;
            m2 = v1 + 1.5*m1;
            m3 = v1 + 0.16666667*H*m1 + 0.16666667*H*m2;
            m4 = v1 + 0.125*H*m1 + 1.5*H*m2;
            m5 = v1 + 0.5*H*m1 - 1.5*H*m3 + 2.0*H*m4;

            mV1 = abs(H*(0.16666667*(m1 + 4.0*m3 + m5) -
                0.5*(m1 - 3.0*m3 + 4.0*m4)));
            dv1 = 0.16666667*H*(m1 + 4 * m3 + m5);
            /* ----- piston 2 ----- */
            /* coord */
            m1 = motion_f(x2, v2, t, EQ_PISTON_2);
            delta = 1.5*m1;
            m2 = motion_f(x2 + delta, v2 + delta, tt2, EQ_PISTON_2);
            delta = 0.16666667*H*m1 + 0.16666667*H*m2;
            m3 = motion_f(x2 + delta, v2 + delta, tt3, EQ_PISTON_2);
            delta = 0.125*H*m1 + 1.5*H*m2;
            m4 = motion_f(x2 + delta, v2 + delta, tt4, EQ_PISTON_2);
            delta = 0.5*H*m1 - 1.5*H*m3 + 2.0*H*m4;
            m5 = motion_f(x2 + delta, v2 + delta, tt5, EQ_PISTON_2);

            mX2 = abs(H*(0.16666667*(m1 + 4.0*m3 + m5) -
                      0.5*(m1 - 3.0*m3 + 4.0*m4)));
            dx2 = 0.16666667*H*(m1 + 4 * m3 + m5);
            /* speed */
            m1 = v2;
            m2 = v2 + 1.5*m1;
            m3 = v2 + 0.16666667*H*m1 + 0.16666667*H*m2;
            m4 = v2 + 0.125*H*m1 + 1.5*H*m2;
            m5 = v2 + 0.5*H*m1 - 1.5*H*m3 + 2.0*H*m4;

            mV2 = abs(H*(0.16666667*(m1 + 4.0*m3 + m5) -
                  0.5*(m1 - 3.0*m3 + 4.0*m4)));
            dv2 = 0.16666667*H*(m1 + 4 * m3 + m5);
            /* Control of H */
            if (mX0 >= hightEps || mV0 >= hightEps ||
                mX1 >= hightEps || mV1 >= hightEps ||
                mX2 >= hightEps || mV2 >= hightEps)
            {
                H = H*0.5;
                count++;
            }
            else
            {
                count = 0;
            }
        } while (count > 0 && count < MAX_TRY_COUNT);
        /* Next step values */
        x = x + dx;
        x1 = x1 + dx1;
        x2 = x2 + dx2;
        if ((x1-x <= ZERO) && (x2-x >= ZERO))
        {
            v1 = ((mu0 - R)*(v1 + mu*sin(t)) + (1 + R)*v + (1 + R - mu0)*mu*sin(t)) / (1 + mu0) - mu*sin(t);
            v =  (mu0*(1 + R)*(v1 + 2*mu*sin(t)) + (R*mu0 + 1)*v) /
                 (1 + mu0);
            v2 = v2 + dv2;
            printf("%.8lf %.8lf %.8lf %.8lf : %6u\n", t, x, x1, x2, ix);
        }
        else if ((x2-x <= ZERO) && (x1-x >= ZERO))
        {
            v2 = ((mu0 - R)*(v2 + mu*gam*sin(t - phi)) + (1 + R)*v + (1 + R - mu0)*mu*gam*sin(t - phi)) /
                (1 + mu0) - mu*gam*sin(t - phi);
            v =  (mu0*(1 + R)*(v1 + 2 * mu*sin(t)) + (R*mu0 + 1)*v) /
                 (1 + mu0);
            v1 = v1 + dv1;
            printf("%.4lf %.4lf %.4lf %.4lf : %8u\n", t, x, x1, x2, ix);
        }
        else
        {
            v = v + dv;
            v1 = v1 + dv1;
            v2 = v2 + dv2;
        }
        t = t + H;
        /* Control of H */
		if (mX0 < lowEps || mV0 < lowEps ||
			mX1 < lowEps || mV1 < lowEps ||
			mX2 < lowEps || mV2 < lowEps)
		{
			H = 2*H;
		}
        /* Save result */
        if (ix >= resArrLength)
        {
            extendArray();
            std::cout << "array extended : " << ix;
        }
        resA[ix]._t  = t;
        resA[ix]._x  = x;
        resA[ix]._x1 = x1;
        resA[ix]._x2 = x2;
        /* printf("%.8lf %.8lf %.8lf %.8lf : %6u\n", t, x, x1, x2, ix); */ 
        ix++;
    } while (t <= tn);

}
