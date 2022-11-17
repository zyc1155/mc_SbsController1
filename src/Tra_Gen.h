#ifndef TRAGEN_H
#define TRAGEN_H
#include <math.h>

class Tra_Gen
{
 public:
	 Tra_Gen();
	 Tra_Gen(double _te, double _sb, double _se);
	 ~Tra_Gen();
public:
	double te;
	double sb, se;
	double A, B, C, Omega;
	double  s[3];//s,ds,dds
public:
	void Set_Func(double _te, double _sb, double _se);
	void Cal_Func(double tt);
};
#endif