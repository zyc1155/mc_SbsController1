#include "Tra_Gen.h"

Tra_Gen::Tra_Gen()
{
	Set_Func(1.0, 1.0, 2.0);
}

Tra_Gen::Tra_Gen(double _te, double _sb, double _se)
{
	Set_Func(_te, _sb, _se);
}

Tra_Gen::~Tra_Gen()
{
}

void Tra_Gen::Set_Func(double _te, double _sb, double _se)
{
	te = _te;
	sb = _sb;
	se = _se;

	Omega = 2 * M_PI / te;
	C = sb;
	B = (se - sb) / te;
	A = B * Omega;
}

void Tra_Gen::Cal_Func(double tt)
{

	if (tt<0.0)
	{
		for(int i=0;i<3;i++)
			s[i]=0;
	}
	else if(tt<=te)
	{
		s[2] = A * sin(Omega * tt);
		s[1] = -(A / Omega) * cos(Omega * tt) + B;
		s[0] = -(A / (Omega * Omega)) * sin(Omega * tt) + B * tt + C;
	}

}
