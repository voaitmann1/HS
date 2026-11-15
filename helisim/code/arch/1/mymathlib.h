#ifndef MYMATHLIB_H
#define MYMATHLIB_H

#include<iostream>//for NULL, so!
#include<math.h>
#include<vector>
#include<QString>

//class MyMathLib
//{
//public:
//    MyMathLib();
//};

struct TValsShowHide{
  //public:
    int Show1Hide0;
    bool ConsoleInterface;
 //   QTextBrowser*TxtFld;
    FILE*f;
    //
    //TValsShowHide();
    //~TValsShowHide();
    //void Assign(const TValsShowHide&obj);
    //void EnableWriting();
    //void DisableWriting();
    //void SetShow1Hide0(int Show1Hide0);
    //void write(char*s);
    //void write(QString str);
    //void writeln(char*s);
    //void writeln(QString str);
};

class MathApprox{
public:
    double precision;//=1E-9;
    MathApprox(double precision=0);
    bool ET(double x1, double x2);

    bool GT(double x1, double x2);
    bool LT(double x1, double x2);
    bool NE(double x1, double x2);
    bool GE(double x1, double x2);
    bool LE(double x1, double x2);
};

bool fApprET(double x1, double x2, double precision);
bool fApprGT(double x1, double x2, double precision);
bool fApprLT(double x1, double x2, double precision);
bool fApprNE(double x1, double x2, double precision);
bool fApprGE(double x1, double x2, double precision);
bool fApprLE(double x1, double x2, double precision);

bool fApprZero(double x1, double precision);
bool fApprNonZero(double x1, double precision);
bool fApprPositive(double x1, double precision);
bool fApprNegative(double x1, double precision);



struct TPosInSucc{
     bool isLess, isWithin, isGreater;
     int EqualN, lessN;
};

TPosInSucc PosInSucc1(double*X, int Q, double x, double eps=0);
TPosInSucc PosInSucc(std::vector<double>X, double x, double eps=0);

double LinInterp(double*X, double*Y, int Q, double x, double eps=0);
double LinInterp(std::vector<double>X, std::vector<double>Y, double x, double eps=0);
double LinInterp2D(std::vector<std::vector<double>>Y, std::vector<double>Xext, std::vector<double>Xine, double xe, double xi,  double eps=0);

double Integr_Trapez_SimpleByElementsArray(std::vector<double>X, std::vector<double>Y);
double Integr_Trapez_SimpleByElementsArray(double*X, double*Y, int Q);

std::vector<double>GetSectionsBounds(double LB, double HB, int Q, double SectL=0, int SectL1Q2And3Or4=2, bool QOfSectsNotSectsBounds=true);

//void CalcDigits(int num, int digits[], /*int&order=0,*/ int SysBase=10, bool AsInNum=false);
void CalcDigits(int num, int digits[], int&order, int SysBase=10, TValsShowHide*vsh=NULL);//, bool AsInNum=false);

double IntPower(double a, int b);
int NaturalPower(int a, int b);

QString ComplexNumberToString(double ReX, double ImX);

double MyArctan(double x, double y, double eps = 0);
//
void EquationQuadratic(std::vector<double>C, std::vector<double>&ReX, std::vector<double>&ImX);
void EquationQuadratic(double a, double b, double c, std::vector<double>&ReX, std::vector<double>&ImX);
void EquationQuadratic(double C[], std::vector<double>&ReX, std::vector<double>&ImX);
void EquationQuadratic(double BToA, double CToA, std::vector<double>&ReX, std::vector<double>&ImX);

void EquationCubic(std::vector<double>C, std::vector<double>&ReX, std::vector<double>&ImX);
void EquationCubic(double C[], std::vector<double>&ReX, std::vector<double>&ImX);
void EquationCubic(double c3, double c2, double c1, double c0, std::vector<double>&ReX, std::vector<double>&ImX);
void EquationCubic(double C2ToC3, double C1ToC3, double C0ToC3, std::vector<double>&ReX, std::vector<double>&ImX);

int EquationOf4thPower(std::vector<double>C, std::vector<double>&ReX, std::vector<double>&ImX);
int EquationOf4thPower(double C[], std::vector<double>&ReX, std::vector<double>&ImX);
int EquationOf4thPower(double C3To4, double C2To4, double C1To4, double C0To4, std::vector<double>&ReX, std::vector<double>&ImX);
int EquationOf4thPower(double c4, double c3, double c2, double c1, double c0, std::vector<double>&ReX, std::vector<double>&ReY);

class PolyEq{
    //std::vector<double> C, ReX, ImX;
public:
    std::vector<double> C, ReX, ImX;
    //
    PolyEq();
    PolyEq(std::vector<double>C);
    PolyEq(const PolyEq&obj);
    ~PolyEq();
    //
    void Assign(const PolyEq&obj);
    PolyEq operator =(const PolyEq&obj);
    //
    void SetEquationAndCalcSolution(std::vector<double>C);
    void StartGenerator(std::vector<double>ReX);
    //void StartSolution()
    std::vector<double> GetReX();
    std::vector<double> GetImX();
    std::vector<double> GetCoefs();
    void SetOrderAndClear(int order);
    int GetOrder();
    QString EquationToString();
    std::string EquationToStdString();
    QString SolutionToString();
    std::string SolutionToToStdString();
};


void LinearApproximation(double*X, double*Y, int Q, double&a, double&b);
void LinearApproximation(const std::vector<double>&X, const std::vector<double>&Y, double&a, double&b);


double arctg2(double x, double y);

class ComplexNum{
    double re, im;
public:
    ComplexNum(double re=0, double im=0);
    ComplexNum(const ComplexNum&obj);
    void Assign(const ComplexNum&obj);
    ~ComplexNum();
    ComplexNum& operator = (const ComplexNum&obj);
    double SetDecart(double re=0, double im=0);
    double SetPolar(double fi=0, double r=0);
    double GetRe();
    double GetIm();
    double GetFi();
    double GetR();
    ComplexNum GetConjugated();
    void GetPolar(double&r, double&fi);
    void ChangeRe(double val);
    void ChangeIm(double val);
    void ChangeFi(double val);
    void ChangeR(double val);
    ComplexNum& operator + (const ComplexNum&obj);
    ComplexNum& operator - (const ComplexNum&obj);
    ComplexNum& operator * (const ComplexNum&obj);
    ComplexNum& operator / (const ComplexNum&obj);
    ComplexNum& operator * (double val);
    bool operator ==(const ComplexNum&obj);
    bool operator !=(const ComplexNum&obj);
    bool IsConjugated(const ComplexNum&obj);
};


#endif // MYMATHLIB_H
