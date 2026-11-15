#include "mymathlib.h"

//MyMathLib::MyMathLib()
//{
//
//}




TPosInSucc PosInSucc1(double*X, int Q, double x, double eps){
    TPosInSucc pos;
    MathApprox appr(eps);
    pos.isLess=false;
    pos.isGreater=false;
    pos.isWithin-false;
    pos.EqualN=0;
    pos.lessN=0;
    //
    if(x<X[1-1]){
        pos.isLess=true;
    }else if(x>X[Q-1]){
       pos.isGreater=true;
    }else{
        pos.isWithin=true;
        for(int i=1; i<=Q; i++){
            if(appr.ET(X[i-1], x)){
                pos.EqualN=i;
                break;
            }
        }
        if(pos.EqualN==0){
            for(int i=1; i<=Q-1; i++){
                if(appr.LT(X[i-1], x) && appr.GT(X[i+1-1], x)){
                    pos.lessN=i;
                    break;
                }
            }
        }
    }
    return pos;
}

TPosInSucc PosInSucc(std::vector<double>X, double x, double eps){
    TPosInSucc pos;
    MathApprox appr(eps);
    int Q=X.size();
    pos.isLess=false;
    pos.isGreater=false;
    pos.isWithin-false;
    pos.EqualN=0;
    pos.lessN=0;
    //
    if(x<X[1-1]){
        pos.isLess=true;
    }else if(x>X[Q-1]){
       pos.isGreater=true;
    }else{
        pos.isWithin=true;
        for(int i=1; i<=Q; i++){
            if(appr.ET(X[i-1], x)){
                pos.EqualN=i;
                break;
            }
        }
        if(pos.EqualN==0){
            for(int i=1; i<=Q-1; i++){
                if(appr.LT(X[i-1], x) && appr.GT(X[i+1-1], x)){
                    pos.lessN=i;
                    break;
                }
            }
        }
    }
    return pos;
}

double LinInterp(double*X, double*Y, int Q, double x, double eps){
    TPosInSucc pos=PosInSucc1(X, Q, x, eps);
    double y=0, y1, y2, x1, x2, k;
    if(pos.EqualN>0){
        y=Y[pos.EqualN-1];
    }else{
        if(pos.isLess){
            x1=X[1-1];
            x2=X[2-1];
            y1=Y[1-1];
            y2=Y[2-1];
        }else if(pos.isGreater){
            x1=X[Q-1-1];
            x2=X[Q-1];
            y1=Y[Q-1-1];
            y2=Y[Q-1];
        }else if(pos.lessN>0){
            x1=X[pos.lessN-1];
            x2=X[pos.lessN+1-1];
            y1=X[pos.lessN-1];
            y2=X[pos.lessN+1-1];
        }
        k=(y2-y1)/(x2-x1);
        y=k*(x-x1)+y1;
    }
    return y;
}

double LinInterp(std::vector<double>X, std::vector<double>Y, double x, double eps){
    int Q=X.size();
    TPosInSucc pos=PosInSucc(X, x, eps);
    double y=0, y1, y2, x1, x2, k;
    if(pos.EqualN>0){
        y=Y[pos.EqualN-1];
    }else{
        if(pos.isLess){
            x1=X[1-1];
            x2=X[2-1];
            y1=Y[1-1];
            y2=Y[2-1];
        }else if(pos.isGreater){
            x1=X[Q-1-1];
            x2=X[Q-1];
            y1=Y[Q-1-1];
            y2=Y[Q-1];
        }else if(pos.lessN>0){
            x1=X[pos.lessN-1];
            x2=X[pos.lessN+1-1];
            y1=X[pos.lessN-1];
            y2=X[pos.lessN+1-1];
        }
        k=(y2-y1)/(x2-x1);
        y=k*(x-x1)+y1;
    }
    return y;
}

double LinInterp2D(std::vector<std::vector<double>>Arr2D, std::vector<double>Xext, std::vector<double>Xine, double xe, double xi, double eps){
    TPosInSucc posExt=PosInSucc(Xext, xe, eps), posIne=PosInSucc(Xine, xi, eps);
    double y0, y;
    std::vector<double>X0, X, Y0, Y;
    int Q=Arr2D.size(), L=Arr2D[1-1].size();
    X=Xine;
    if(posExt.EqualN>0){
        Y=Arr2D[posExt.EqualN-1];
    }else{
        if(posExt.isLess){
            X0.clear();
            X0.push_back(Xext[1-1]);
            X0.push_back(Xext[2-1]);
            for(int i=1; i<=L; i++){
                Y0.clear();
                Y0.push_back(Arr2D[1-1][i-1]);
                Y0.push_back(Arr2D[2-1][i-1]);
                //
                y0=LinInterp(X0, Y0, xe, eps);
                Y.push_back(y0);
            }
        }else if(posExt.isGreater){
            X0.clear();
            X0.push_back(Xext[Q-1-1]);
            X0.push_back(Xext[Q-1]);
            for(int i=1; i<=L; i++){
                Y0.clear();
                Y0.push_back(Arr2D[Q-1-1][i-1]);
                Y0.push_back(Arr2D[Q-1][i-1]);
                //
                y0=LinInterp(X0, Y0, xe, eps);
                Y.push_back(y0);
            }
        }else if(posExt.lessN>0){
            X0.clear();
            X0.push_back(Xext[posExt.lessN-1]);
            X0.push_back(Xext[posExt.lessN+1-1]);
            for(int i=1; i<=L; i++){
                Y0.clear();
                Y0.push_back(Arr2D[posExt.lessN-1][i-1]);
                Y0.push_back(Arr2D[posExt.lessN+1-1][i-1]);
                //
                y0=LinInterp(X0, Y0, xe, eps);
                Y.push_back(y0);
            }
        }
        y=LinInterp(X, Y, xi, eps);
    }
    return y;
}

std::vector<double>GetSectionsBounds(double LB, double HB, int Q, bool QOfSectsNotSectsBounds=true){
    std::vector<double>Y;
    int QSects, QSectsBounds;
    double SectL, y;
    QSects = QOfSectsNotSectsBounds==true ? Q : Q-1;
    QSectsBounds=QSects+1;
    SectL=(HB-LB)/QSects;
    Y.push_back(LB);
    for(int SectBoundN=1; SectBoundN<=QSectsBounds; SectBoundN++){
        y=LB+SectL*(SectBoundN-1);
        if(SectBoundN>1 && SectBoundN<QSectsBounds){
            Y.push_back(y);
        }
    }
    Y.push_back(HB);
    return Y;
}


double Integr_Trapez_SimpleByElementsArray(std::vector<double>X, std::vector<double>Y){
    double ys=0, c1, c2, dx;
    int Q=Y.size();
    for(int i=1; i<=Q-1; i++){
        dx=X[i+1-1]-X[i-1];
        c1=(Y[i+1-1]+Y[i-1])/2;
        c2=c1*dx;
        ys+=c2;
    }
    return ys;
}

double Integr_Trapez_SimpleByElementsArray(double*X, double*Y, int Q){
    double ys=0, c1, c2, dx;
    //int Q=Y.size();
    for(int i=1; i<=Q-1; i++){
        dx=X[i+1-1]-X[i-1];
        c1=(Y[i+1-1]+Y[i-1])/2;
        c2=c1*dx;
        ys+=c2;
    }
    return ys;
}

//void CalcDigits(int num, int digits[], /*int&order,*/ int SysBase, bool AsInNum){
void CalcDigits(int num, int digits[], int&order, int SysBase, TValsShowHide*vsh){//, bool AsInNum){
    int digit, z;
    z=num>=0?num:-num;
    //if(z>SysBase)
    order=-1;
    //writeln(vsh,"CalcDigits starts working. Try number "+IntToStr(z));
    while(z>=SysBase){
        order++;
        digit=z%SysBase;
        digits[order]=digit;
        //writeln(vsh,"z="+IntToStr(z)+" > Sys base="+IntToStr(SysBase)+" => digit="+IntToStr(digit));
        z=(z-digit)/SysBase;
    }
    order++;
    digit=z;
    digits[order]=digit;
    //writeln(vsh,"z="+IntToStr(z)+" > Sys base="+IntToStr(SysBase)+" => digit="+IntToStr(digit)+" order="+IntToStr(order));
}

double IntPower(double a, int b){
    double y=1;
    if(b>=0){
        for(int i=1; i<=b; i++){
            y*=a;
        }
    }else{
        for(int i=1; i<=b; i++){
            y/=a;
        }
    }
    return y;
}

int NaturalPower(int a, int b){
   int y=1;
   for(int i=1; i<=b; i++){
       y*=a;
   }
   return y;
}

QString ComplexNumberToString(double ReX, double ImX){
    QString s, sRe, sIm;
    sRe.setNum(ReX);
    sIm.setNum(ImX);
    if(ImX!=0){
        s+="(";
        s+=sRe;
        if(ImX>=0){
            s+=" + ";
        }
        s+=sIm;
        s+="*i)";
    }else{
        s=sRe;
    }
    return s;
}


//class MathApprox{
//public:
//    double precision=1E-9;
    MathApprox::MathApprox(double precision){
        this->precision=precision;
    }
    bool MathApprox::ET(double x1, double x2){
        bool verdict=false;
        if(fabs(x1-x2)<this->precision){
            verdict=true;
        }
        return verdict;
    }
    bool MathApprox::GT(double x1, double x2){
        bool verdict=false;
        if(x1>x2 && fabs(x1-x2)>this->precision){
            verdict=true;
        }
        return verdict;
    }
    bool MathApprox::LT(double x1, double x2){
        bool verdict=false;
        if(x1<x2 && fabs(x2-x1)>this->precision){
            verdict=true;
        }
        return verdict;
    }
    bool MathApprox::NE(double x1, double x2){ return(!(this->ET(x1, x2)));}
    bool MathApprox::GE(double x1, double x2){ return(!(this->LT(x1, x2)));}
    bool MathApprox::LE(double x1, double x2){ return(!(this->GT(x1, x2)));}

    //


    bool fApprET(double x1, double x2, double precision){
        bool verdict=false;
        if(fabs(x1-x2)<precision){
            verdict=true;
        }
        return verdict;
    }
    bool fApprGT(double x1, double x2, double precision){
        bool verdict=false;
        if(x1>x2 && fabs(x1-x2)>precision){
            verdict=true;
        }
        return verdict;
    }
    bool fApprLT(double x1, double x2, double precision){
        bool verdict=false;
        if(x1<x2 && fabs(x2-x1)>precision){
            verdict=true;
        }
        return verdict;
    }
    bool fApprNE(double x1, double x2, double precision){ return(!(fApprET(x1, x2, precision)));}
    bool fApprGE(double x1, double x2, double precision){ return(!(fApprLT(x1, x2, precision)));}
    bool fApprLE(double x1, double x2, double precision){ return(!(fApprGT(x1, x2, precision)));}

    bool fApprZero(double x1, double precision){
        return((fApprET(x1, 0, precision)));
    }
    bool fApprNonZero(double x1, double precision){
        return((fApprNE(x1, 0, precision)));
    }
    bool fApprPositive(double x1, double precision){
        return((fApprGT(x1, 0, precision)));
    }
    bool fApprNegative(double x1, double precision){
        return((fApprLT(x1, 0, precision)));
    }


    void EquationQuadratic(std::vector<double>C, std::vector<double>&ReX, std::vector<double>&ImX){
        double a, b, c, d, re_x, im_x;
        ReX.clear();
        ImX.clear();
        a=C[2];
        b=C[1];
        c=C[0];
        d=b*b-4*a*c;
        if(a!=0){
            if(d<0){
                re_x=-b/2/a;
                ReX.push_back(re_x);
                ReX.push_back(re_x);
                im_x=sqrt(-d)/2/a;
                ImX.push_back(-im_x);
                ImX.push_back(im_x);
            }
        }
    }
    void EquationQuadratic(double C[], std::vector<double>&ReX, std::vector<double>&ImX){
        double a, b, c;//, d, re_x, im_x;
        ReX.clear();
        ImX.clear();
        a=C[2];
        b=C[1];
        c=C[0];
        EquationQuadratic(a, b, c, ReX, ImX);
    }
    void EquationQuadratic(double BToA, double CToA, std::vector<double>&ReX, std::vector<double>&ImX){
        double a, b, c;//, d, re_x, im_x;
        ReX.clear();
        ImX.clear();
        a=1;
        b=BToA;
        c=CToA;
        EquationQuadratic(a, b, c, ReX, ImX);
    }
    void EquationQuadratic(double a, double b, double c, std::vector<double>&ReX, std::vector<double>&ImX){
        //double a, b, c;
        double d, re_x, im_x;
        ReX.clear();
        ImX.clear();
        //a=C[2];
        //b=C[1];
        //c=C[0];
        d=b*b-4*a*c;
        if(a!=0){
            if(d<0){
                re_x=-b/2/a;
                ReX.push_back(re_x);
                ReX.push_back(re_x);
                im_x=sqrt(-d)/2/a;
                ImX.push_back(-im_x);
                ImX.push_back(im_x);
            }else{
                re_x=(-b-sqrt(d))/2/a;
                ReX.push_back(re_x);
                re_x=(-b+sqrt(d))/2/a;
                ReX.push_back(re_x);
                ImX.push_back(0);
                ImX.push_back(0);
            }
        }
    }
    void EquationCubic(std::vector<double>C, std::vector<double>&ReX, std::vector<double>&ImX){
        double c3, c2, c1, c0;
        c0=C[0];
        c1=C[1];
        c2=C[2];
        c3=C[3];
        EquationCubic(c3, c2, c1, c0, ReX, ImX);
    }
    void EquationCubic(double C[], std::vector<double>&ReX, std::vector<double>&ImX){
        double c3, c2, c1, c0;
        c0=C[0];
        c1=C[1];
        c2=C[2];
        c3=C[3];
        EquationCubic(c3, c2, c1, c0, ReX, ImX);
    }
void EquationCubic(double c3, double c2, double c1, double c0, std::vector<double>&ReX, std::vector<double>&ImX){
    double d, re_x, im_x, Re1, Re2, Im2, Re3, Im3;
    //s=new string[6];
    //s[1-1]="c3="+c3.ToString()+" c2="+c2.ToString()+" c1="+c1.ToString()+" c0="+c0.ToString();
    double a, b, c, A, B, p, q, Q, alfa, CosAlfa, ReY1, ReY2, ReY3;
    a=c2/c3; b=c1/c3; c=c0/c3;
    p=-a*a/3+b;
    q=2*a*a*a/27-a*b/3+c;
    Q=p*p*p/27+q*q/4;
    B = 0;
    CosAlfa = 0;
    alfa = 0;
    A = 0;
    //s[2-1]="p="+p.ToString()+" q="+q.ToString()+" Q="+Q.ToString();
    if(Q<0){
       // s[3-1]="Equation has 3 real not equal roots";
        CosAlfa=-q/(2*sqrt(-(p/3)*(p/3)*(p/3)));
        alfa=acos(CosAlfa);
        ReY1 = 2.0 * sqrt(-(p / 3.0)) * cos(alfa / 3.0);
        ReY2 = 2 * sqrt(-(p / 3)) * cos(alfa / 3 + 2 *M_PI / 3);
        ReY3 = 2 * sqrt(-(p / 3)) * cos(alfa / 3 - 2 *M_PI / 3);
        Im2=0;
        Im3=0;
    }else{
        //if(Q>0)s[3-1]="Equation has 1 real and 2 complex roots";
        //else  s[3-1]="Equation has 3 real roots, min 2 of them are equal";
        if ((-q / 2 + sqrt(Q)) >= 0){
            A = pow((-q / 2 + sqrt(Q)), 1 / 3.0);
        }else{
            A = -pow(-(-q / 2 + sqrt(Q)), 1 / 3.0);
        }
        if ((-q / 2 - sqrt(Q)) >= 0){
            B = pow((-q / 2 - sqrt(Q)), 1 / 3.0);
        }else{
            B = -pow(-(-q / 2 - sqrt(Q)), 1 / 3.0);
        }
        ReY1=A+B;
        //Im1=0;
        ReY2=-(A+B)/2;
        Im2 = (A - B) / 2.0 * sqrt(3);
        ReY3=ReY2;
        Im3=-Im2;
    }
    //s[4-1]="-(p/3)*(p/3)*(p/3)="+(-(p/3)*(p/3)*(p/3)).ToString()+" ";
    //if(!(Q<0))s[4-1]+=" A="+A.ToString()+" B="+B.ToString();
    //else s[4-1]+=" can't calculate A & B ";
    //if (!(Q > 0)) s[4 - 1] += " Cos(alfa)= " + (CosAlfa).ToString() + " alfa=" + (alfa).ToString();
    //else s[4-1]+=" can't calculate cos & alfa ";
    //s[5 - 1] += " ReY1=" + (ReY1).ToString() + " ReY2=" + (ReY2).ToString() + " ReY3=" + (ReY3).ToString();
    Re1=ReY1-a/3;
    Re2=ReY2-a/3;
    Re3=ReY3-a/3;
    //s[6 - 1] += " Re1=" + (Re1).ToString() + " Re2=" + (Re2).ToString() + " Re3=" + (Re3).ToString();
    ReX.push_back(Re1);
    ImX.push_back(0);
    ReX.push_back(Re2);
    ImX.push_back(Im2);
    ReX.push_back(Re3);
    ImX.push_back(Im3);
}
void EquationCubic(double C2ToC3, double C1ToC3, double C0ToC3, std::vector<double>&ReX, std::vector<double>&ImX){

}

int EquationOf4thPower(std::vector<double>C, std::vector<double>&ReX, std::vector<double>&ImX){
    double c4, c3, c2, c1, c0;
    c0=C[0];
    c1=C[1];
    c2=C[2];
    c3=C[3];
    c4=C[4];
    int ErrN;
    ErrN= EquationOf4thPower(c4, c3, c2, c1, c0, ReX, ImX);
    return ErrN;
}
int EquationOf4thPower(double C[], std::vector<double>&ReX, std::vector<double>&ImX){
    double c4, c3, c2, c1, c0;
    c0=C[0];
    c1=C[1];
    c2=C[2];
    c3=C[3];
    c4=C[4];
    int ErrN;
    ErrN= EquationOf4thPower(c4, c3, c2, c1, c0, ReX, ImX);
    return ErrN;
}
int EquationOf4thPower(double C3To4, double C2To4, double C1To4, double C0To4, std::vector<double>&ReX, std::vector<double>&ImX){
    double c4, c3, c2, c1, c0;
    c0=C0To4;
    c1=C1To4;
    c2=C2To4;
    c3=C3To4;
    c4=1;
    int ErrN;
    ErrN= EquationOf4thPower(c4, c3, c2, c1, c0, ReX, ImX);
    return ErrN;
}
int EquationOf4thPower(double c4, double c3, double c2, double c1, double c0, std::vector<double>&ReX, std::vector<double>&ImX){
            double e2pD;
            bool AllRight=true;
            int ErrN=0;
            std::vector<double>ResolvRe, ResolvIm;
            std::vector<double> EqPow2N1Re, EqPow2N1Im, EqPow2N2Re,EqPow2N2Im;
            //string[] si=new string[27];
            //string []ResolveStr=null;
            double Re1, Im1, Re2, Im2, Re3, Im3, Re4, Im4;
            double ResolvC3, ResolvC2, ResolvC1, ResolvC0;//, CubicReRoot1;
            double Eq1C2,  Eq1C1, Eq1C0, Eq2C2,  Eq2C1, Eq2C0;
            double Root1Eq1Re1, Root1Eq1Im1, Root2Eq1Re2, Root2Eq1Im2,
               Root3Eq2Re1, Root3Eq2Im1, Root4Eq2Re2, Root4Eq2Im2,
               ResolvRe1, ResolvRe2, ResolvIm2, ResolvRe3, ResolvIm3;
            double p,q,r;
            double a=c3/c4, b=c2/c4, c=c1/c4, d=c0/c4;
            //
            e2pD = 0; ResolvRe1 = 0; ResolvRe2 = 0; ResolvIm2 = 0; ResolvRe3 = 0;
            ResolvIm3 = 0; Root3Eq2Re1 = 0; Root3Eq2Im1 = 0; Root4Eq2Re2 = 0; Root4Eq2Im2 = 0;
            Root1Eq1Re1 = 0; Root1Eq1Im1 = 0; Root2Eq1Re2 = 0; Root2Eq1Im2 = 0;
            //Root1Eq2Re1 = 0; Root1Eq2Im1 = 0; Root2Eq2Re2 = 0; Root2Eq2Im2 = 0;
            //
            p=b-3*a*a/8;
            q=a*a*a/8-a*b/2+c;
            r=-3*a*a*a*a/256+a*a*b/16-c*a/4+d;
            ResolvC3=2;
            ResolvC2=-p;
            ResolvC1=-2*r;
            ResolvC0=r*p-q*q/4;
            //si[1-1]="Supplementary Coefficients p="+p.ToString()+" q="+q.ToString()+" r="+r.ToString();
           // si[2-1]="Cubic resolvent: "+ResolvC3.ToString()+"*y^3+"+ResolvC2.ToString()+"*y^2+"+ResolvC1.ToString()+"*y+"+ResolvC0.ToString()+"=0";
            //CubicEquation(ResolvC3, ResolvC2, ResolvC1, ResolvC0, ref ReolvRe1, ref ResolvRe2, ref ResolvIm2, ref ResolvRe3, ref ResolvIm3, ref ResolveStr);
            EquationCubic(ResolvC3, ResolvC2, ResolvC1, ResolvC0, ResolvRe, ResolvIm);
            ResolvRe1=ResolvRe[1-1];
            ResolvRe2=ResolvRe[2-1];
            ResolvIm2=ResolvIm[2-1];
            ResolvRe3=ResolvRe[3-1];
            ResolvIm3=ResolvIm[3-1];
            //si[3-1]="1st Solution of Cubic Resolvent: "+ResolvRe1.ToString();
            //si[4-1]="Checking: "+(ResolvC3*ResolvRe1*ResolvRe1*ResolvRe1+ResolvC2*ResolvRe1*ResolvRe1+ResolvC1*ResolvRe1+ResolvC0).ToString();
            //si[5-1]=ResolveStr[1-1];
            //si[6-1]=ResolveStr[2-1];
            //    si[7-1]=ResolveStr[3-1];
            //si[8-1]=ResolveStr[4-1];
            //si[9-1]=ResolveStr[5-1];
            //    si[10-1]=ResolveStr[6-1];
            //si[11-1]="It was info from cubic resolvent. Now again equation of 4th power";
            if(ResolvRe1<=p/2){
                //MessageBox.Show("Equation can't be replaced by equivalent quadratic equations with real coefs!");
                AllRight=false;
                Root1Eq1Re1=666; Root1Eq1Im1=666; Root2Eq1Re2=666; Root2Eq1Im2=666;
                Root3Eq2Re1=666; Root3Eq2Im1=666; Root4Eq2Re2=666; Root4Eq2Im2=666;
                //si[12-1]="can't calculate the real coefs of equivalent cuadratic equations";
                //si[13-1]="2y1-p=2*"+ResolvRe1.ToString()+"-"+p.ToString()+"="+(2*ResolvRe1-p).ToString();
                //for(int i=14; i<=27; i++){
                //    si[14-1]=" ";
                //}
            }else{
                //si[12-1]="No problem with solution";
                Eq1C2=1;
                Eq1C1=-sqrt(2*ResolvRe1-p);
                Eq1C0 = q / (2 * sqrt(2 * ResolvRe1 - p)) + ResolvRe1;
                //si[13-1]="First quadratic equation: "+Eq1C2.ToString()+"*x^2+"+Eq1C1.ToString()+"*x+"+Eq1C0.ToString()+"=0";
                Eq2C2=Eq1C2;
                Eq2C1=-Eq1C1;
                Eq2C0=-q/(2*sqrt(2*ResolvRe1-p))+ResolvRe1;
                //si[14-1]="Second quadratic equation: "+Eq2C2.ToString()+"*x^2+"+Eq2C1.ToString()+"*x+"+Eq2C0.ToString()+"=0";
                //QuadraticEquation(Eq1C2, Eq1C1, Eq1C0, ref e2pD,  ref Root1Eq1Re1, ref Root1Eq1Im1, ref Root2Eq1Re2, ref Root2Eq1Im2);
                //QuadraticEquation(Eq2C2, Eq2C1, Eq2C0, ref e2pD,  ref Root3Eq2Re1, ref Root3Eq2Im1, ref Root4Eq2Re2, ref Root4Eq2Im2);
                EquationQuadratic(Eq1C2, Eq1C1, Eq1C0, EqPow2N1Re, EqPow2N1Im);
                EquationQuadratic(Eq2C2, Eq2C1, Eq2C0, EqPow2N2Re, EqPow2N2Im);
                Root1Eq1Re1=EqPow2N1Re[1-1];
                Root1Eq1Im1=EqPow2N1Im[1-1];
                Root2Eq1Re2=EqPow2N1Re[2-1];
                Root2Eq1Im2=EqPow2N1Im[2-1];
                //
                Root3Eq2Re1=EqPow2N2Re[1-1];
                Root3Eq2Im1=EqPow2N2Im[1-1];
                Root4Eq2Re2=EqPow2N2Re[2-1];
                Root4Eq2Im2=EqPow2N2Im[2-1];
                //si[15-1]="ANSWER:";
                //si[16-1]="1st Y: ";
                //if(Root1Eq1Im1==0){
                //    si[16-1]+=Root1Eq1Re1.ToString();
                //}else{
                //    si[16-1]+=Root1Eq1Re1.ToString()+"+i*"+Root1Eq1Im1.ToString();
                //}
                //si[17-1]="2nd Y: ";
                //if(Root2Eq1Im2==0){
                //    si[17-1]+=Root2Eq1Re2.ToString();
                //}else{
                //    si[17-1]+=Root2Eq1Re2.ToString()+"+i*"+Root2Eq1Im2.ToString();
                //}
                //si[18-1]="3rd Y: ";
                //if (Root3Eq2Im1 == 0)
                //{
                //    si[18 - 1] += Root3Eq2Re1.ToString();
                //}
                //else
                //{
                //    si[18 - 1] += Root3Eq2Re1.ToString() + "+i*" + Root3Eq2Im1.ToString();
                //}
                //si[19-1]="4th Y: ";
                //if(Root4Eq2Im2==0){
                //    si[19-1]+=Root4Eq2Re2.ToString();
                //}else{
                //    si[19-1]+=Root4Eq2Re2.ToString()+"+i*"+Root4Eq2Im2.ToString();
                //}
                //
                Root1Eq1Re1-=a/4;
                Root2Eq1Re2-=a/4;
                Root3Eq2Re1-=a/4;
                Root4Eq2Re2-=a/4;
                //si[20-1]="1st Root: ";
                //if(Root1Eq1Im1==0){
                //    si[20-1]+=Root1Eq1Re1.ToString();
                //}else{
                //    si[20-1]+=Root1Eq1Re1.ToString()+"+i*"+Root1Eq1Im1.ToString();
                //}
                //si[21-1]="2nd Root: ";
                //if(Root2Eq1Im2==0){
                //    si[21-1]+=Root2Eq1Re2.ToString();
                //}else{
                //    si[21 - 1] += Root2Eq1Re2.ToString() + "+i*" + Root2Eq1Im2.ToString();
                //}
                //si[22-1]="3rd Root: ";
                //if(Root3Eq2Im1==0){
                //    si[22 - 1] += Root3Eq2Re1.ToString();
                //}else{
                //    si[22-1]+=Root3Eq2Re1.ToString()+"+i*"+Root3Eq2Im1.ToString();
                //}
                //si[23-1]="4th Root: ";
                //if(Root4Eq2Im2==0){
                //    si[23-1]+=Root4Eq2Re2.ToString();
                //}else{
                //    si[23 - 1] += Root4Eq2Re2.ToString() + "+i*" + Root4Eq2Im2.ToString();
                //}
                //
                //if(Root1Eq1Im1==0){
                //    si[24-1]="Checking Root 1: "+(Root1Eq1Re1*Root1Eq1Re1*Root1Eq1Re1*Root1Eq1Re1+a*Root1Eq1Re1*Root1Eq1Re1*Root1Eq1Re1+b*Root1Eq1Re1*Root1Eq1Re1+c*Root1Eq1Re1+d).ToString();
                //}else{
                //    si[24-1]="  ";
                //}
                //if(Root2Eq1Im2==0){
                //    si[25 - 1] = "Checking Root 2: " + (Root2Eq1Re2 * Root2Eq1Re2 * Root2Eq1Re2 * Root2Eq1Re2 + a * Root2Eq1Re2 * Root2Eq1Re2 * Root2Eq1Re2 + b * Root2Eq1Re2 * Root2Eq1Re2 + c * Root2Eq1Re2 + d).ToString();
                //}else{
                //    si[25-1]="  ";
                //}
                //if(Root3Eq2Im1==0){
                //    si[26 - 1] = "Checking Root 3: " + (Root3Eq2Re1 * Root3Eq2Re1 * Root3Eq2Re1 * Root3Eq2Re1 + a * Root3Eq2Re1 * Root3Eq2Re1 * Root3Eq2Re1 + b * Root3Eq2Re1 * Root3Eq2Re1 + c * Root3Eq2Re1 + d).ToString();
                //}else{
                //    si[26-1]="  ";
                //}
                //if(Root4Eq2Im2==0){
                //    si[27 - 1] = "Checking Root 4: " + (Root4Eq2Re2 * Root4Eq2Re2 * Root4Eq2Re2 * Root4Eq2Re2 + a * Root4Eq2Re2 * Root4Eq2Re2 * Root4Eq2Re2 + b * Root4Eq2Re2 * Root4Eq2Re2 + c * Root4Eq2Re2 + d).ToString();
                //}else{
                //    si[27-1]="  ";
                //}
            }
            Re1=Root1Eq1Re1; Im1=Root1Eq1Im1; Re2=Root2Eq1Re2; Im2=Root2Eq1Im2;
            Re3=Root3Eq2Re1; Im3=Root3Eq2Im1; Re4=Root4Eq2Re2; Im4=Root4Eq2Im2;
            //s=si;
            ReX.push_back(Re1);
            ReX.push_back(Re2);
            ReX.push_back(Re3);
            ReX.push_back(Re4);
            ImX.push_back(Im1);
            ImX.push_back(Im2);
            ImX.push_back(Im3);
            ImX.push_back(Im4);
            if(AllRight)ErrN=0;
            return ErrN;
        }

//class PolyEq{
//       std::vector<double>C, ReX, ImX;
  //
PolyEq::PolyEq(){}
PolyEq::PolyEq(std::vector<double>C){
    this->SetEquationAndCalcSolution(C);
}
PolyEq::PolyEq(const PolyEq&obj){
    this->C=obj.C;
}
PolyEq::~PolyEq(){}
void PolyEq::Assign(const PolyEq&obj){
    this->C=obj.C;
    this->ReX=obj.ReX;
    this->ImX=obj.ImX;
}
PolyEq PolyEq::operator =(const PolyEq&obj){
    this->Assign(obj);
    return *this;
}
void PolyEq::SetEquationAndCalcSolution(std::vector<double>C){
    int Q=C.size(), order=Q-1;
    this->ReX.clear();
    this->ImX.clear();
    double val;
    int ErrN;
    switch(order){
        case 1:
            if(this->C[1]!=0){
                val=-this->C[1]/this->C[0];
                ReX.push_back(val);
                ImX.push_back(0);
            }
        break;
        case 2:
            EquationQuadratic(this->C, this->ReX, this->ImX);
        break;
        case 3:
            EquationCubic(this->C, this->ReX, this->ImX);
        break;
        case 4:
            ErrN=EquationOf4thPower(this->C, this->ReX, this->ImX);
        break;
    }
}
void PolyEq::StartGenerator(std::vector<double>ReX){
    int Q=ReX.size(), order=Q-0;
    double K=1,
           c11, c10,
           c22, c21, c20,
           c33, c32, c31, c30,
           c44, c43, c42, c41, c40,
           c55, c54, c53, c52, c51, c50;
    this->ReX=ReX;
    this->C.clear();
    this->ImX.clear();
    for(int i=1; i<=order; i++){
       this->ImX.push_back(0);
    }
    c11=K;
    c10=-ReX[1-1];
    //
    c20=ReX[1-1]*ReX[2-1];
    c21=-(ReX[1-1]+ReX[2-1]);
    c22=K;
    //
    c33 = K * 1;
    c32 = -K * (ReX[1 - 1] + ReX[2 - 1] + ReX[3 - 1]);
    c31 = K * (ReX[1 - 1] + ReX[2 - 1]) * +ReX[3 - 1] + ReX[1 - 1] * ReX[2 - 1];
    c30 = -K * (ReX[1 - 1] * ReX[2 - 1] * ReX[3 - 1]);
    //
    c44 = c33;
    c43 = c32 - ReX[4 - 1] * c33;
    c43 = -K * (ReX[1 - 1] + ReX[2 - 1] + ReX[3 - 1] + ReX[4 - 1]);
    c42 = c31 - ReX[4 - 1] * c32;
    c42 = K * ((ReX[1 - 1] + ReX[2 - 1]) * ReX[3 - 1] + ReX[1 - 1] * ReX[2 - 1] + ReX[4 - 1] * (ReX[1 - 1] + ReX[2 - 1] + ReX[3 - 1]));
    c41 = c30 - ReX[4 - 1] * c31;
    c40 = -ReX[4 - 1] * c30;
    c40 = K * ReX[1 - 1] * ReX[2 - 1] * ReX[3 - 1] * ReX[4 - 1];
    //try by analogy
    c55=c44;
    c54=c43-ReX[5-1]*c44;
    c53=c42-ReX[5-1]*c43;
    c52=c41-ReX[5-1]*c42;
    c51=c40-ReX[5-1]*c41;
    c50= K * ReX[1 - 1] * ReX[2 - 1] * ReX[3 - 1] * ReX[4 - 1]* ReX[5 - 1];
    //
    switch(order){
        case 1:
            if(c11!=0){
                this->ReX.push_back(-c10/c11);
                //this->ReX.push_back(c11);
                this->C.push_back(c10);
                this->C.push_back(c11);
            }
        break;
        case 2:
            if(c22>0){
                this->C.push_back(c20);
                this->C.push_back(c21);
                this->C.push_back(c22);
            }
        break;
        case 3:
            if(c33>0){
                this->C.push_back(c30);
                this->C.push_back(c31);
                this->C.push_back(c32);
                this->C.push_back(c33);
            }
        break;
        case 4:
            if(c44>0){
                this->C.push_back(c40);
                this->C.push_back(c41);
                this->C.push_back(c42);
                this->C.push_back(c43);
                this->C.push_back(c44);
            }
        break;
        case 5:

        break;
    }
    this->SetEquationAndCalcSolution(this->C);
}
std::vector<double> PolyEq::GetReX(){return this->ReX;}
std::vector<double> PolyEq::GetImX(){return this->ImX;}
std::vector<double> PolyEq::GetCoefs(){return this->C;}
void PolyEq::SetOrderAndClear(int order){
    this->ReX.clear();
    this->ImX.clear();
    this->C.clear();
    for(int i=1; i<order; i++){
        this->C.push_back(0);
        this->ReX.push_back(0);
        this->ImX.push_back(0);
    }
    this->C.push_back(0);
}
int PolyEq::GetOrder(){
    int order=0;
    order=this->ReX.size();
    return order;
}
QString PolyEq::EquationToString(){
    int powN, order=this->ReX.size(), Q=order+1;
    QString s="", sC, sPow, snum;
    double val, absVal;
    for(int i=0; i<=order; i++){
        powN=order-i;
        val=this->C[powN];
        absVal=fabs(val);
        //snum.setNum(absVal);
        snum.setNum(val);
        sPow.setNum(powN);
        if(val!=0){
            if(powN>1){//if(powN>0){
                if(powN<order){
                    if(val>=0){
                        s+="+";
                    }
                    s+=snum;
                    s+="*X^";
                    s+=sPow;
                }else{
                    s+=snum;
                    s+="*X^";
                    s+=sPow;
                }
        }else if(powN==1){
            if(powN<order){
                if(val>=0){
                    s+="+";
                }
                s+=snum;
                s+="*X";
                //s+="^";
                 //s+=sPow;
            }else{
                s+=snum;
                s+="*X";
                //s+="^";
                //s+=sPow;
            }
        }else{//nur free member
            if(val>=0){
                s+="+";
            }
            s+=snum;
            //s+="*X^";
            //s+=sPow;
            }
        }
    }
    s+="=0";
    return s;
}
std::string PolyEq::EquationToStdString(){
    std::string ss;
    QString qs;
    qs=this->EquationToString();
    ss=qs.toStdString();
    return ss;
}
QString PolyEq::SolutionToString(){
    QString ss, scur, sn;
    int order=this->ReX.size();
    for(int i=1; i<=order-1; i++){
        scur="";
        sn.setNum(i);
        scur="X["+sn+"]=";
        scur+=ComplexNumberToString(this->ReX[i-1], this->ImX[i-1]);
        ss+=scur;
        ss+="; ";
    }
    sn.setNum(order);
    scur="X["+sn+"]=";
    scur+=ComplexNumberToString(this->ReX[order-1], this->ImX[order-1]);
    ss+=scur;
    return ss;
}
std::string PolyEq::SolutionToToStdString(){
    std::string ss;
    QString qs;
    qs=this->SolutionToString();
    ss=qs.toStdString();
    return ss;
}



void LinearApproximation(double*X, double*Y, int Qini, double&a, double&b){
    double meanY=0, sumSquaredDiff=0, stdDevY;
    int Q;
    std::vector<double> newX, newY;
    double sumX=0;
    double sumY=0;
    double sumXY = 0.0;
    double sumX2 = 0.0;
    for(int i=1; i<=Qini; i++){
        meanY+=Y[i-1];
    }
    meanY/=Q;
    for(int i=1; i<=Qini; i++){
        sumSquaredDiff+=(Y[i-1]-meanY)*(Y[i-1]-meanY);
    }
    stdDevY = sqrt(sumSquaredDiff / Qini);
    const double threshold = 2.0 * stdDevY;
    for(int i=1; i<=Qini; i++){
        if(abs(Y[i] - meanY) <= threshold){
            newX.push_back(X[i-1]);
            newY.push_back(Y[i-1]);
        }
    }
    Q=newX.size();
    for(int i=1; i<=Q; i++){
        sumX+=newX[i-1];
        sumY+=newY[i-1];
        sumXY+=newX[i-1]*newY[i-1];
        sumX2+=newX[i-1]*newX[i-1];
    }
    a = (Q * sumXY - sumX * sumY) / (Q * sumX2 - sumX * sumX);
    b = (sumY - a * sumX) / Q;

}

void LinearApproximation(const std::vector<double>&X, const std::vector<double>&Y, double&a, double&b){
    double meanY, sumSquaredDiff;
    std::vector<double>X1=X, Y1=Y;
    int Q=X.size();
    if(X.size()!=Y.size() || X.empty()){
        //NOp
        a=0;
        b=0;
    }else{
        //void LinearApproximation(double*&X, double*Y, int Qini, double&a, double&b)
        LinearApproximation(X1.data(), Y1.data(), X.size(), a, b);
    }
}

double arctg2(double x, double y){
    double R=0, absval;
    int QuadrantN=0;
    if(x=0 && y!=0){
        //std::th
    }else if(y==0 &&x!=0){
        R=0;
    }else{
        if(x>0 && y>0){
            QuadrantN=1;
        }else if(x<0 && y>0){
            QuadrantN=2;
        }else if(x<0 && y<0){
            QuadrantN=3;
        }else if(x>0 && y<0){
            QuadrantN=4;
        }
        absval=fabs(y/x);
        switch(QuadrantN){
            case 1:
                R=absval;
            break;
            case 2:
                R=M_PI-absval;
            break;
            case 3:
                R=M_PI+absval;
            break;
            case 4:
                R=2*M_PI-absval;
            break;
        }
    }
    return R;
}


//class ComplexNum{
//    double re, im;
//public:
    ComplexNum::ComplexNum(double re, double im){
        this->SetDecart(re, im);
    }

    ComplexNum::ComplexNum(const ComplexNum&obj){
        this->re=re;
        this->im=im;
    }

    void ComplexNum::Assign(const ComplexNum&obj){
        this->SetDecart(re, im);
    }

    ComplexNum::~ComplexNum(){}
    ComplexNum& ComplexNum::operator = (const ComplexNum&obj){
        this->Assign(obj);
        return(*(this));
    }

    double ComplexNum::SetDecart(double re, double im){
        this->re=re;
        this->im=im;
    }

    double ComplexNum::SetPolar(double fi, double r){
        this->re=r*cos(fi);
        this->im=r*sin(fi);
    }

    double ComplexNum::GetRe(){
        return this->re;
    }

    double ComplexNum::GetIm(){
        return this->im;
    }
    void ComplexNum::GetPolar(double&r, double&fi){
        r=sqrt(this->re*this->re + this->im*this->im);
        fi=arctg2(this->re, this->im);
    }

    double ComplexNum::GetFi(){
        double r=0, fi=0;
        this->GetPolar(r, fi);
        return fi;
    }

    double ComplexNum::GetR(){
        double r=0, fi=0;
        this->GetPolar(r, fi);
        return r;
    }
    void ComplexNum::ChangeRe(double val){
        this->re=val;
    }

    void ComplexNum::ChangeIm(double val){
        this->im=val;
    }
    void ComplexNum::ChangeFi(double val){
        double r=GetR();
        this->SetPolar(val, r);
    }
    ComplexNum ComplexNum::GetConjugated(){
        ComplexNum y(this->re, -this->im);
        return y;
    }

    void ComplexNum::ChangeR(double val){
        double fi=GetFi();
        this->SetPolar(fi, val);
    }
    ComplexNum& ComplexNum::operator + (const ComplexNum&obj){
        this->re+=obj.re;
        this->im+=obj.im;
        return*this;
    }

    ComplexNum& ComplexNum::operator - (const ComplexNum&obj){
        this->re-=obj.re;
        this->im-=obj.im;
        return*this;
    }
    ComplexNum& ComplexNum::operator * (const ComplexNum&obj){
        this->re = this->re * obj.re + this->im * obj.im;
        this->im = this->re * obj.im - this->im * obj.re;
        return*this;
    }
    ComplexNum& ComplexNum::operator / (const ComplexNum&obj){
        double denominator=this->re*this->re+this->im*this->im;
        this->re = this->re * obj.re - this->im * obj.im;
        this->im = this->re * obj.im + this->im * obj.re;
        this->re/=denominator;
        this->im/=denominator;
        return*this;
    }
    ComplexNum& ComplexNum::operator * (double val){
        this->re*=val;
        this->im*=val;
        return (*(this));
    }

    bool ComplexNum::operator ==(const ComplexNum&obj){
        return (this->re == obj.re && this->im == obj.im);
    }

    bool ComplexNum::operator !=(const ComplexNum&obj){
        return !(this->re == obj.re && this->im == obj.im);
    }

    bool ComplexNum::IsConjugated(const ComplexNum&obj){
        return(this->re==obj.re && this->im==-obj.im);
    }

//};
