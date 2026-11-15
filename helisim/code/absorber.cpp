#pragma hdrstop
#pragma package(smart_init)
#include "absorber.h"

IAbsorber::~IAbsorber()
{
}
AbsorberEq::~AbsorberEq()
{
}
Absorber1cam::~Absorber1cam()
{
}
Absorber2cam::~Absorber2cam()
{
}
AbsorberMi2::~AbsorberMi2()
{
}
double IAbsorber::min(double x1, double x2)
{
    if ( x1 < x2 ) return x1; else return x2;
}
double IAbsorber::max(double x1, double x2)
{
    if ( x1 > x2 ) return x1; else return x2;
}
void IAbsorber::setNull()
{

}

void IAbsorber::Preprocess()//ОПРЕДЕЛЕНИЕ ОСНОВНЫХ НЕИЗМЕНЯЕМЫХ ПАРАМЕТРОВ АМОРТИЗАТОРА (и изменяемых на шаге 0)
{
    t_= 8;//используется только для изолированного амортизатора (испытываемого) в SolveSingleTest()
    S = c1.s + c2.s;//суммарный максимальный ход штоков
    kgds = 1.0/(mu*mu);//коэф.гидродинамического сопротивления
    //начальный объем газа
       V =    V_=    V0;
    c1.V = c1.V_= c1.V0;
    c2.V = c2.V_= c2.V0;

    //шток 1
    c1.Fc = 0.25* M_PI* c1.Dc*c1.Dc;//площадь по диаметру цилиндра 1
    c1.Fv = 0.25* M_PI* c1.Dv*c1.Dv;//площадь по внутреннему диаметру штока 1
    c1.Fs = 0.25* M_PI* c1.Ds*c1.Ds;//площадь по наружному диаметру штока 1
    c1.Fb = c1.Fc - c1.Fs;          //площадь буксы штока 1,
    c1.F = (c1.fb)?(c1.Fs):(c1.Fc);
    //шток 2
    c2.Fc = 0.25* M_PI* c2.Dc*c2.Dc;//площадь по диаметру цилиндра 2
    c2.Fv = 0.25* M_PI* c2.Dv*c2.Dv;//площадь по внутреннему диаметру штока 1
    c2.Fs = 0.25* M_PI* c2.Ds*c2.Ds;//площадь по наружному диаметру штока 1
    c2.Fb = c2.Fc - c2.Fs;          //площадь буксы штока 1,
    c2.F = (c2.fb)?(c2.Fs):(c2.Fc);

    ha=ha_= 0;//ход амортизатора начальный
    c1.v  = c2.v  = va = 0;//
    c1.h  = c1.h_ = c2.h  = c2.h_ = 0;//ход амортизатора
    c1.dh = c2.dh = dha = 0;//изменение хода
    c1.p  = c1.p_ = c1.p0;// давление в демпфере
    c2.p  = c2.p_ = c2.p0;// давление в амортизаторе
    c1.Pg = c1.p*c1.F;             // сила сжатия газа в демпфере
    c1.Pf = c1.Pg*c1.ktr;          // сила механического трения в демпфере
    c1.Ph = 0;                     // сила гидравлического сопротивления демпфера
    c1.P  = c1.Ph + c1.Pg + c1.Pf; // усилие на демпфере Н
    c2.Pg = c2.p*c2.F;             // сила сжатия газа   Н
    c2.Pf = c2.Pg*c2.ktr;          // сила механического трения амортизатора
    c2.Ph = 0;                     // сила гидравлического сопротивления амортизатора
    c2.P  = c2.Ph + c2.Pg + c2.Pf; // усилие на амортизаторе
    Pg = c1.Pg;                 //
    Pa = 0;//(c2.s==0)?(c1.P):(min(c2.P,c1.P)); //min    // усилие на амортизаторе <modify 2020.02
    Ph = 0;
    dA = 0;
    status=0;
    Pmin = 0;//(c2.s==0)?(c1.Pg-c1.Pf):(min( c2.Pg-c2.Pf, c1.Pg-c1.Pf )); <modify 2020.02
    dh12 = 0;
    q = 0;
    //old:
    //c1.sigma  = (ro * c1.Fn * c1.Fn * c1.Fn )/( 2* mu*mu * c1.f * c1.f );
    //c1.sigmar = (ro * c1.Fn * c1.Fn * c1.Fn )/( 2* mu*mu * c1.fr* c1.fr);
    //c1.sigmab = (ro * c1.Fb * c1.Fb * c1.Fb )/( 2* mu*mu * c1.fb* c1.fb);
    //c2.sigma  = (ro * c2.Fc * c2.Fc * c2.Fc )/( 2* mu*mu * c2.f * c2.f );
    //c2.sigmar = (ro * c2.Fc * c2.Fc * c2.Fc )/( 2* mu*mu * c2.fr* c2.fr);
    //c1.sigmap = (ro * c1.Fv * c1.Fv * c1.Fv )/( 2* mu*mu * c1.f * c1.f );
}

void IAbsorber::cam1force()//сила на штоке амортизатора низкого давления
{
    // сила сжатия газа, Н
    c1.Pg = c1.p * c1.F;

    // сила механического трения, Н
    c1.Pf = c1.Pg * c1.ktr;

    // сила гидравлического сопротивления
    if(c1.dh>0)
    {
        if(c1.Dv) c1.Ph = c1.dpa * c1.Fv + c1.dpb * c1.Fb;
        else      c1.Ph = c1.dpa * c1.F  + c1.dpb * c1.Fb;
    }
    else//revers
    {
        if(c1.Dv) c1.Ph = c1.dpar* c1.Fv + c1.dpbr * c1.Fb;
        else      c1.Ph = c1.dpar* c1.F  + c1.dpbr * c1.Fb;
    }

    // суммарная сила на штоке демпфера
    if(c1.dh > 0)   c1.P = c1.Pg + c1.Ph + c1.Pf;
    if(c1.dh < 0)   c1.P = c1.Pg - c1.Ph - c1.Pf;
    if(c1.dh== 0)
    {
        if(status<0) c1.P = c1.Pg - c1.Pf;
        else         c1.P = c1.Pg + c1.Pf;
    }
}

void IAbsorber::cam2force()//сила на штоке амортизатора высокого давления
{
    // сила сжатия газа, Н
    c2.Pg = c2.p * c2.F;

    // сила механического трения, Н
    c2.Pf = c2.Pg * c2.ktr;

    // сила сопротивления жидкости, Н
    if(c2.dh>0)
    {
        if(c2.Dv) c2.Ph = c2.dpa * c2.Fv + c2.dpb * c2.Fb;
        else      c2.Ph = c2.dpa * c2.F  + c2.dpb * c2.Fb;
    }
    else//revers
    {
        if(c2.Dv) c2.Ph = c2.dpar* c2.Fv + c2.dpbr * c2.Fb;
        else      c2.Ph = c2.dpar* c2.F  + c2.dpbr * c2.Fb;
    }

    // суммарная сила на штоке аморт, Н
    if(c2.dh > 0)  c2.P = c2.Pg + c2.Ph + c2.Pf;
    if(c2.dh < 0)  c2.P = c2.Pg - c2.Ph - c2.Pf;
    if(c2.dh== 0)
    {
        if(status<0) c2.P = c2.Pg - c2.Pf;
        else         c2.P = c2.Pg + c2.Pf;
    }
}

double IAbsorber::get_restriction(vector<double> &restriction, vector<double> &rodposition, double rp)
{
    double f=0;
    double x1,y1, x2,y2, a,b;

    if(rp<=rodposition[0])
    {
        f = restriction[0];
    }
    else if(rp>=rodposition[rodposition.size()-1])
    {
        f = restriction[rodposition.size()-1];
    }
    else
    for(int i=1; i<rodposition.size(); i++)
    {
        x1 = rodposition[i-1];
        x2 = rodposition[i];
        if (rp>=x1 && rp<x2)
        {
            y1 = restriction[i-1];
            y2 = restriction[i];
            a = (y2-y1)/(x2-x1);
            b = y1-a*x1;
            f = a*rp+b;
            break;
        }
    }
return f;
}



// НЕЛИНЕЙНАЯ ПРУЖИНА ////////////////////////////////////////////////////////////////////////////////////

int  AbsorberEq::Solve()
{
    warning = 0;
    //c12SetNull();
    va = dha / dt;// скорость штока(штоков):
    c1.dh = dha;

    if ( ha_+c1.dh >= c1.s+c2.s)    status = 2; // обжат
    if ( c1.dh      > 0 )           status = 1; // обжимается
    if ((c1.dh==0)&&(ha<c1.s+c2.s)) status = 0; // неподвижен
    if ( c1.dh      < 0 )           status =-1; // разжимается
    if ( ha_+c1.dh <= 0 )           status = 0; //-2/ разжат

    abInside();

return status;
}

void AbsorberEq::abInside()// сила + перемещение штока однокамерного амортизатора
{
    double P12, h1, h2;
    // скорость деформации
    va = dha / dt;
    c1.v = va;

    if (va >= 0)/// прямой ход (сжатие)
    {
        // в пределах упругости
        if ((ha_-dh12 + dha) <= hu)
        {
            ha = c1.h = ha_ + dha;
            Pa = c1.P = Ke*(ha-dh12);///
            c2.P = c1.P;
            c2.h = c1.h;
        }
        else// пластическая деформация
        {
            ha = c2.h = ha_ + dha;
            Pa = c2.P = Ki*(ha-dh12) + Ci;///
            c1.h = hu;
            c1.P = Ke*hu;
        }
        //потери на внутреннее трение
        //AddDamping();
    }
    else/// обратный ход (разжатие)
    {
        q++;
        // разжатие по упругой характеристике
        if(q==1 && ha_-dh12 +dha>hu)//-dh12
        {// определяем разность смещения по оси деформации
            h2   = ha_;//c2.h_;
            P12  = Ki * h2 + Ci;
            h1   = P12 / Ke;
            dh12 = h2 - h1;
        }
        ha = c2.h = ha_ + dha;
        Pa = c2.P = Ke *(ha - dh12);

        //потери на внутреннее трение
        AddDamping();

        if(Pa<0)
        {
            status = 0;//разжат
            Pa = 0;
            ha = c2.h = ha_;
        }
        c1.h = ha;            //c1.h = ha;
        c1.P = Pa;
    }

    //работа за обжатие dha амортизатора:
    dA = Pa * dha;
}

void AbsorberEq::AddDamping()//учет демпфирования от внутреннего трения
{
    //усилие демпфирования (от внутреннего трения)//kdp - коэф.демпфирования пневматика [H*c^2/mm]
    //double kdp = 0.05;
    //double Pdmp = fabs(dha * Ke);
    double Pdmp = dha * Ke;/// *10

    Pa = Pa + Pdmp;//(va<0) ? () : (Pa)
}

int  AbsorberEq::SolveFree(double Plimit)// расчет разгруженного амортизатора (при отрыве пневматика)
{
    //c12SetNull();
    isFree = true;
    //c1.dh = dha;
    status = -2;

    abInsideFree(Plimit);///добавил 22.10.2020

    //уточнение состояния
    if(Plimit==0)
    {
        isFree = true;
        if ( dha <  0 )                 status =-2; // разжимается
        if ( ha  <= 0 )                 status = 0; //-2 разжат
    }
    else
    {
        isFree = false;
        if ( dha <  0 )                 status =-1; // разжимается
        if ( dha >  0 )                 status = 1; // cжимается
        if ( ha  <= 0 )                 status = 0; //-2 разжат
    }

    //status = 0;//разжат
    //Pa = 0;
    //ha = ha_;
    isFree = false;
return status;
}

void AbsorberEq::abInsideFree(double limit)// свободная стойка, однокамерный амортизатор: сила + перемещение
{
    bool  c1free=1;
    double na_free = kra*0.000001;
    dha = 0;

    do
    {
        abInside();
        if (Pa <= limit) c1free = false;
        else             dha -= na_free;

    } while ( c1free );/// && ha>0

    //if (ha < 0)
    //{
    //    c1.h = 0;   ha = 0;
    //    c1.v = 0;   va = 0;
    //}
}

void AbsorberEq::c12SetNull()
{//
    c2.s =0;
    c2.Dc=0;
    c2.Dv=0;
    c2.Ds=0;
    c2.f =0;
    c2.fb=0;
    c2.fr=0;
    c2.Fc=0;
    c2.Fb=0;
    c2.Fv=0;
    //c2.h =0;
    //c2.h_=0;
    c2.dh=0;
    c2.dV=0;
    c2.v =0;
    c2.p =0;
    c2.Ph=0;
    c2.Pg=0;
    c2.Pf=0;
    c2.P =0;
}

void AbsorberEq::SolveSingleTest()// расчет изолированного амортизатора (испытываемого)
{
    warning = 0;
    //c12SetNull();
    dha = va*dt;
    c1.dh = dha;
    if ( ha_+c1.dh <= 0 )           status = 2; // обжат
    if ( c1.dh      > 0 )           status = 1; // обжимается
    if ((c1.dh==0)&&(ha<c1.s+c2.s)) status = 0; // неподвижен
    if ( c1.dh      < 0 )           status =-1; // разжимается
    if ( ha_+c1.dh >= c1.s+c2.s)    status = 0; //-2 разжат

    abInside();

    if (status==0 && Pa==0)  status = -2;
}



//  1  ШТОК ////////////////////////////////////////////////////////////////////////////////////

int  Absorber1cam::Solve()//   ОДНОКАМЕРНЫЙ
{
    warning = 0;
    c2SetNull();
    va = dha / dt;// скорость штока(штоков):
    c1.dh = dha;

    if ( ha_+c1.dh >= c1.s+c2.s)    status = 2; // обжат
    if ( c1.dh      > 0 )           status = 1; // обжимается
    if ((c1.dh==0)&&(ha<c1.s+c2.s)) status = 0; // неподвижен
    if ( c1.dh      < 0 )           status =-1; // разжимается
    if ( ha_+c1.dh <= 0 )           status = 0; //-2 разжат

    abInside();

return status;
}

void Absorber1cam::abInside()// сила + перемещение штока однокамерного амортизатора
{
    double _p;
    // ход штока
    c1.h =  c1.h_ + c1.dh;
    // скорость перемещения штока
    c1.v = c1.dh/dt;
    // изменение объема, м^3
    c1.dV = c1.F * c1.dh;
    dV = c1.dV;
    // текущий объем газовой полости
    V = V_ - dV;
    c1.V = V;
    c2.V = 0;
    // давление в газовой полости, Па
    if(V<=0) _p = p0 * pow( V0/V_, kpoly);
    else
    {
        if(c1.dV>0) _p = p0 * pow( V0/V, kpoly );///origin
        else        _p = c1.p_*pow(c1.V_/c1.V, kpolyr);///_p = p0 * pow( V0/V, kpolyr);
    }
    c1.p = _p;

    //c1.f как функция положения штока
    if(c1.v_h.size()>1)
    {
        c1.f = get_restriction(c1.v_f, c1.v_h, c1.h);//forward
        c1.fr= get_restriction(c1.v_fr,c1.v_h, c1.h);//backward
    }

    // сила на штоке
    abRodForce();
    //
    Pa = c1.P;
    // ход штока амортизатора
    ha = c1.h;
    // скорость штока:
    va = c1.v;
    //работа за обжатие dha амортизатора:
    dA = c1.P * c1.dh;
}

void Absorber1cam::abRodForce()// сила на штоке однокамерного амортизатора
{
    //скорости перетекания жидкости через отверстия дросселей
    if(c1.fb!=0)  c1.wb  = c1.v * c1.Fb / c1.fb;
    else c1.wb = 0;
    if(c1.fbr!=0) c1.wbr = c1.v * c1.Fb / c1.fbr;
    else c1.wbr = 0;
    if(c1.f !=0)
    {
        if(c1.Dv) c1.w  = c1.v * c1.Fv/ c1.f ;
        else      c1.w  = c1.v * c1.F / c1.f ;
    }
    else c1.w = 0;
    if(c1.fr!=0)
    {
        if(c1.Dv) c1.wr = c1.v * c1.Fv/ c1.fr;
        else      c1.wr = c1.v * c1.F / c1.fr;
    }
    else c1.w = 0;

    //перепады давлений на дросселях
    c1.dpa = 0.5*ro*kgds *c1.w *c1.w ;
    c1.dpar= 0.5*ro*kgds *c1.wr*c1.wr;
    c1.dpb = 0.5*ro*kgds *c1.wb*c1.wb;
    c1.dpbr= 0.5*ro*kgds *c1.wbr*c1.wbr;

    //усилие на штоке
    cam1force();

    c2.P  = 0;
    c2.Pg = 0;
    c2.Ph = 0;
    c2.Pf = 0;

    /*old
    // сила сжатия газа, Н
    c1.Pg = c1.p * c1.F;
    // сила механического трения, Н
    c1.Pf = c1.Pg * c1.ktr;

    // сила гидравлического сопротивления
    if(c1.dh>0)
    {
        if(c1.Dv) c1.Ph = c1.dpa * c1.Fv + c1.dpb * c1.Fb;
        else      c1.Ph = c1.dpa * c1.F  + c1.dpb * c1.Fb;
    }
    else//revers
    {
        if(c1.Dv) c1.Ph = c1.dpar* c1.Fv + c1.dpbr * c1.Fb;
        else      c1.Ph = c1.dpar* c1.F  + c1.dpbr * c1.Fb;
    }
    // суммарная сила на штоке демпфера,Н
    if (c1.dh > 0)  c1.P =  c1.Pg + c1.Ph + c1.Pf;
    if (c1.dh < 0)  c1.P =  c1.Pg - c1.Ph - c1.Pf;
    if (c1.dh== 0)
    {
        if (status<0) c1.P = c1.Pg - c1.Pf; //Н
        else          c1.P = c1.Pg + c1.Pf; //Н
    }
    */
    /*/old:
        double _Pw1, _Pw2;
        // сила гидравлического сопротивления на центральном жеклере
        if (c1.dh > 0) _Pw1 = c1.sigma * c1.v * c1.v;
        else           _Pw1 = c1.sigmar* c1.v * c1.v;
        // сила гидравлического сопротивления на жеклерах в буксе
        _Pw2 =  c1.sigmab* c1.v * c1.v;
        // суммарная сила гидравлического сопротивления
        c1.Ph = _Pw1 + _Pw2;
    //
    //для совпадения со старой версией : c1.Fv = c1.Fn;
        //if(c1.f !=0)  c1.w  = c1.v * c1.Fv / c1.f ;  else c1.w  = 0;
        //if(c1.fr!=0)  c1.wr = c1.v * c1.Fv / c1.fr;  else c1.wr = 0;

    //if (c1.dh > 0) c1.Ph = c1.dp * c1.Fv + c1.dpb * c1.Fb;
    //else           c1.Ph = c1.dpr* c1.Fv + c1.dpb * c1.Fb;
// */
}

int  Absorber1cam::SolveFree(double Plimit)// расчет разгруженного амортизатора (при отрыве пневматика)
{
    c2SetNull();
    isFree = true;
    //c1.dh = dha;
    status = -2;

    abInsideFree(Plimit);
    //уточнение состояния
    if(Plimit==0)
    {
        isFree = true;
        if ( dha <  0 )                 status =-2; // разжимается
        if ( ha  <= 0 )                 status = 0; //-2 разжат
    }
    else
    {
        isFree = false;
        if ( dha <  0 )                 status =-1; // разжимается
        if ( dha >  0 )                 status = 1; // cжимается
        if ( ha  <= 0 )                 status = 0; //-2 разжат
    }

    isFree = false;
return status;
}

void Absorber1cam::abInsideFree(double limit)// свободная стойка, однокамерный амортизатор: сила + перемещение
{
    bool  c1free=1;
    double na_free = kra*0.000001;
    c1.dh = 0;

    do// цикл до  c1.P < 0
    {
        abInside();
        if (Pa <= limit) c1free = false;
        else             c1.dh -= na_free;

    } while ( c1free && c1.h>0 );//(c1.P >= 0) && (c1.h >= 0) //добавил &&(c1.h >= 0)     >= весa "значительной" части шасси

    if (c1.h < 0)
    {
        c1.h = 0;   ha = 0;
        c1.v = 0;   va = 0;
    }
}

void Absorber1cam::c2SetNull()
{//
      c2.s=0;
      c2.Dc=0;
      c2.Dv=0;
      c2.Ds=0;
      c2.f=0;
      c2.fb=0;
      c2.fr=0;
      c2.Fc=0;
      c2.Fb=0;
      c2.Fv=0;
      c2.h=0;
      c2.h_=0;
      c2.dh=0;
      c2.dV=0;
      c2.v =0;
      c2.p =0;
      c2.Ph=0;
      c2.Pg=0;
      c2.Pf=0;
      c2.P =0;
}

void Absorber1cam::SolveSingleTest()// расчет изолированного амортизатора (испытываемого)
{
    dha = va*dt;
    c1.dh =    dha;
    if ( ha_+c1.dh <= 0 )           status = 2; // обжат
    if ( c1.dh      > 0 )           status = 1; // обжимается
    if ((c1.dh==0)&&(ha<c1.s+c2.s)) status = 0; // неподвижен
    if ( c1.dh      < 0 )           status =-1; // разжимается
    if ( ha_+c1.dh >= c1.s+c2.s)    status = 0; //-2 разжат

    abInside();
}



//  2  ШТОКА ////////////////////////////////////////////////////////////////////////////////////

/// раздельные газ.камеры // type Mi-8

int  Absorber2cam::Solve()//   ДВУХКАМЕРНЫЙ
{
    warning = 0;
    va = dha / dt; // скорость штока(штоков):

    if ( ha_+dha >= c1.s+c2.s )  status = 2; // обжат
    if ( dha     >  0 )          status = 1; // обжимается
    if ((dha==0))                status = 0; // неподвижен    &&(ha_+dha<c1.s+c2.s) //2017.09.20
    if ( dha     <  0 )          status =-1; // разжимается
    if ( ha_+dha <= 0)           status = 0; //-2 разжат

    abInsideBalance();// баланс демпфера и амортизатора:

return status;
}

void Absorber2cam::abInsideBalance()// определение силы всего амортизатора
{
    c1.dh= 0;    c2.dh= 0;
    c1.P = 0;    c2.P = 0;
    _Pa = 0;
    double  daf = 0,     naa, nad;
    int     zdaf_= 1,    zdaf = 1;
    int     Adirect;

    if (dha == 0)// нет перемещений штоков амортизатора /////////////////
    {
        Adirect = 0;
        c1.dh = 0;
        c2.dh = 0;
        abRodsForces();
        if (status>=0) _Pa = min( c1.P, c2.P );//было ">"
        else           _Pa = max( c1.P, c2.P );//max-min не вижу разницы;  было "max" до 2017.09.19 !!!
    }
    else
    {// dha != 0
        if (dha>0) Adirect =+1;
        if (dha<0) Adirect =-1;

        nad = 0.5;
        naa = 0.5;
        c1.dh = dha*nad;
        c2.dh = dha*naa;

        abRodsForces();

        daf = (c1.P - c2.P) / fabs(0.5*(c1.P + c2.P));// ?
        if(daf>0) zdaf = 1;
        else     zdaf = -1;
        zdaf_ = zdaf;
        int q=0;
        while (zdaf_ == zdaf)
        {// цикл до смены знака zdaf
            q++;
            zdaf_ = zdaf;
            if(daf*Adirect < 0)// c1.P < c2.P
            {
                nad = nad + kra;
                naa = 1 - nad;
            }
            else  // c1.P >= c2.P
            {
                naa = naa + kra;
                nad = 1 - naa;
            }
            c1.dh = dha*nad;
            c2.dh = dha*naa;

            abRodsForces();

            if ((c2.h<=0)||(c2.h>=c2.s))// a- обжат/разжат до упора
            {//a-
                if ((c1.h>0)&&(c1.h<c1.s))// d+ работает
                {//d+   // заходит на несколько шагов в самом начале
                        if (c2.h <= 0)// a- разжат до упора
                        {
                            c2.dh = 0 - c2.h_;
                            c1.dh = dha - c2.dh;
                        }
                        if (c2.h >= c2.s)// a- обжат до упора
                        {
                            c2.dh = c2.s - c2.h_;
                            c1.dh = dha - c2.dh;
                        }
                        abRodsForces();
                        _Pa = c1.P;
                        goto END_AB_BALANCE;
                }//end d+
                else // d- обжат/разжат до упора ///////////////
                {//d-
                        if (c2.h <= 0)// a- разжат до упора
                        {
                            c2.dh = 0 - c2.h_;
                            c1.dh = dha - c2.dh;
                        }
                        if (c2.h >= c2.s)// a- обжат до упора
                        {
                            c2.dh = c2.s - c2.h_;
                            c1.dh = dha - c2.dh;
                        }
                        if (c1.h <= 0)// d- разжат до упора
                        {
                            c1.dh = 0 - c1.h_;
                            c2.dh = dha - c1.dh;
                        }
                        if (c1.h >= c1.s)// d- обжат до упора
                        {
                            c1.dh = c1.s - c1.h_;
                            c2.dh = dha - c1.dh;
                        }
                        abRodsForces();
                        _Pa = max( c1.P, c2.P );//max-min не вижу разницы;  было "max" до 2017.09.19 !!!
                        goto END_AB_BALANCE;
                }//end d-
            }// end a-
            else // a+ работает ////////////////////////////////////
            {//a+
                if ((c1.h <= 0)||(c1.h >= c1.s))// d- обжат/разжат до упора
                {//d-
                    if (c1.h <= 0)// d- разжат до упора
                    {
                        c1.dh = 0 - c1.h_;
                        c2.dh = dha - c1.dh;
                    }
                    if (c1.h >= c1.s)// d- обжат до упора
                    {
                        c1.dh = c1.s - c1.h_;
                        c2.dh = dha - c1.dh;
                    }
                    abRodsForces();
                    _Pa = c2.P;
                    goto END_AB_BALANCE;
                }//end d-
                else // d+ работает ////////////////////////////
                {//d+
                    if(c1.P!=-c2.P)
                            daf = (c1.P - c2.P) / fabs(0.5*(c1.P + c2.P));
                    if(daf>=0) zdaf = 1;
                    else       zdaf =-1;

                    if(nad>=1)//
                    {
                        c1.dh = dha;
                        c2.dh = 0;
                        abRodsForces();
                        _Pa = c1.P;
                        goto END_AB_BALANCE;
                    }
                    if(naa>=1)//
                    {
                        c1.dh = 0;
                        c2.dh = dha;
                        abRodsForces();
                        _Pa = c2.P;
                        goto END_AB_BALANCE;
                    }
                        _Pa = max( c1.P, c2.P );//max-min не вижу разницы;   было "max" до 2017.09.19 !!!
                }//end d+ //////////////////////////////////////
            }// end a+
            if (q>9999) {  warning=210;  break; }//"abInsideBalance(): q>999"
        }// end while
    }// end if dha!=0
END_AB_BALANCE:
    // усилие на штоке амортизатора:
    Pa = _Pa;
    // ход штока амортизатора:
    ha  = c1.h + c2.h;
    // работа за обжатие dha амортизатора:
    dA = c1.P*c1.dh + c2.P*c2.dh;
}

void Absorber2cam::abRodsForces()//организация расчета сил + перемещений штоков двухкамерного амортизатора
{
    //положения штоков
    c1.h = c1.h_ + c1.dh;
    c2.h = c2.h_ + c2.dh;

    //c1.f как функция положения штока камеры низкого давления
    if(c1.v_h.size()>1)
    {
        c1.f = get_restriction(c1.v_f, c1.v_h, c1.h);//forward
        c1.fr= get_restriction(c1.v_fr,c1.v_h, c1.h);//backward
    }
    //c2.f как функция положения штока камеры высокого давления
    if(c2.v_h.size()>1)
    {
        c2.f = get_restriction(c2.v_f, c2.v_h, c2.h);//forward
        c2.fr= get_restriction(c2.v_fr,c2.v_h, c2.h);//backward
    }

    //скорости штоков
    c1.v = c1.dh / dt;
    c2.v = c2.dh / dt;

    //объем газа, м^3
    {
        V = 0;  dV = 0;
        //изменение объемов газовых камер
        c1.dV = c1.F * c1.dh;
        c2.dV = c2.F * c2.dh;
        //объемы газовых камер
        c1.V = c1.V_ - c1.dV;
        c2.V = c2.V_ - c2.dV;
    }

    //давление газа, Па
    if(c1.V<=0) c1.p=0;
    else
    {
        if(c1.dV>0)  c1.p = c1.p0 *pow(c1.V0/c1.V, kpoly );//сжатие
        else         c1.p = c1.p_ *pow(c1.V_/c1.V, kpolyr);//c1.p0 *pow(c1.V0/c1.V, kpolyr);//разжатие
        //c1.p = (c1.V) ? ( c1.p0 *pow(c1.V0/c1.V, kpoly) ) : (0);
    }
    if(c2.V<=0) c2.p=0;
    else
    {
        if(c2.dV>0)  c2.p = c2.p0 *pow(c2.V0/c2.V, kpoly );//сжатие
        else         c2.p = c2.p_ *pow(c2.V_/c2.V, kpolyr);//c2.p = c2.p0 *pow(c2.V0/c2.V, kpolyr);//_p = c1.p_*pow(V_/(V_-dV), kpolyr);//разжатие
        //c2.p = (c2.V) ? ( c2.p0 *pow(c2.V0/c2.V, kpoly) ) : (0);
    }

    //скорости перетекания жидкости через отверстия дросселей
    {
        //скорости перетекания жидкости через отверстия дросселей
        //if(c1.f !=0)  c1.w  = c1.v * c1.Fs / c1.f ;  else c1.w = 0;
        //if(c1.fr!=0)  c1.wr = c1.v * c1.Fs / c1.fr;  else c1.wr= 0;
        //if(c2.f !=0)  c2.w  = c2.v * c2.Fs / c2.f ;  else c2.w = 0;
        //if(c2.fr!=0)  c2.wr = c2.v * c2.Fs / c2.fr;  else c2.wr= 0;

        //шток 1
        if(c1.fb!=0)  c1.wb = c1.v * c1.Fb/c1.fb;   else c1.wb= 0;
        if(c1.fbr!=0) c1.wbr= c1.v * c1.Fb/c1.fbr;  else c1.wbr = 0;
        if(c1.f !=0)
        {
            if(c1.Dv) c1.w  = c1.v * c1.Fv/c1.f;
            else      c1.w  = c1.v * c1.F /c1.f;
        }
        else c1.w = 0;
        if(c1.fr!=0)
        {
            if(c1.Dv) c1.wr = c1.v * c1.Fv/c1.fr;
            else      c1.wr = c1.v * c1.F /c1.fr;
        }
        else c1.w = 0;

        //шток 2
        if(c2.fb!=0)  c2.wb = c2.v * c2.Fb/c2.fb;   else c2.wb= 0;
        if(c2.fbr!=0) c2.wbr= c2.v * c2.Fb/c2.fbr;  else c2.wbr = 0;
        if(c2.f!=0)
        {
            if(c2.Dv) c2.w  = c2.v * c2.Fv/c2.f;
            else      c2.w  = c2.v * c2.F /c2.f;
        }
        else c2.w = 0;
        if(c2.fr!=0)
        {
            if(c2.Dv) c2.wr = c2.v * c2.Fv/c2.fr;
            else      c2.wr = c2.v * c2.F /c2.fr;
        }
        else c2.w = 0;
    }

    //перепады давлений на дросселях
    c1.dpa = 0.5*ro*kgds *pow(c1.w  ,2.0);
    c1.dpar= 0.5*ro*kgds *pow(c1.wr ,2.0);
    c1.dpb = 0.5*ro*kgds *pow(c1.wb ,2.0);
    c1.dpbr= 0.5*ro*kgds *pow(c1.wbr,2.0);
    c2.dpa = 0.5*ro*kgds *pow(c2.w  ,2.0);
    c2.dpar= 0.5*ro*kgds *pow(c2.wr ,2.0);
    c2.dpb = 0.5*ro*kgds *pow(c2.wb ,2.0);
    c2.dpbr= 0.5*ro*kgds *pow(c2.wbr,2.0);

    //усилия на штоках
    cam1force();
    cam2force();
}

int  Absorber2cam::SolveFree(double Plimit)// расчет разгруженного амортизатора (при отрыве пневматика)
{
    isFree = true;
    status = -2;

    abInsideBalanceFree(Plimit);
    //уточнение состояния
    if(Plimit==0)
    {
        isFree = true;
        if ( dha <  0 )                 status =-2; // разжимается
        if ( ha  <= 0 )                 status = 0; //-2 разжат
    }
    else
    {
        isFree = false;
        if ( dha <  0 )                 status =-1; // разжимается
        if ( dha >  0 )                 status = 1; // cжимается
        if ( ha  <= 0 )                 status = 0; //-2 разжат
    }

return status;
}

void Absorber2cam::abInsideBalanceFree(double limit)// определение силы свободного амортизатора
{
    c1.dh= 0;    c2.dh= 0;
    c1.P = 0;    c2.P = 0;
    bool c1_change, c2_change;
    double na_free = kra*0.00001;// kra*0.000001
    int q=0;

    abRodsForces();
    ha = c1.h + c2.h;

    while (( max(c1.P,c2.P) > limit ) && (ha>0))
    {
        q++;
        if ( ( c1.P >= limit ) && (c1.h >= 0) )
        {
            c1.dh -= na_free;
            c1_change = true;
        }
        else c1_change = false;
        if ( ( c2.P >= limit ) && (c2.h >= 0) )
        {
            c2.dh -= na_free;
            c2_change = true;
        }
        else c2_change = false;

        abRodsForces();
        dha = c1.dh + c2.dh;

        if (!c1_change && !c2_change)
        {
            abInsideBalance();
            break;///c1.P = 0; c2.P = 0; <modify 2020.02
        }
        if (q>9999) {  warning=211;  break; }//"abInsideBalanceFree(): q>999"
    }//endwhile

    if (c1.h < 0)  c1.h = 0;
    if (c2.h < 0)  c2.h = 0;
    Pa = max( c1.P, c2.P );
    // скорость штока:
    va = dha / dt;
    // ход штока амортизатора:
    ha = c1.h + c2.h;
    // работа за обжатие dha амортизатора:
    dA = c2.P * c2.dh + c1.P * c1.dh;
}

void Absorber2cam::SolveSingleTest()// расчет изолированного амортизатора (испытываемого)
{
    dha = va*dt;
    if ( ha_+dha <= 0 )                 status = 2; // обжат
    if ( dha     >  0 )                 status = 1; // обжимается
    if ((dha==0)&&(ha_+dha<c1.s+c2.s))  status = 0; // неподвижен
    if ( dha     <  0 )                 status =-1; // разжимается
    if ( ha_+dha >= c1.s+c2.s)          status = 0; //-2 разжат
    // баланс демпфера и амортизатора:
    abInsideBalance();
}


/// общая газ.камера // type Mi-2

int  AbsorberMi2::Solve()//   ДВУХКАМЕРНЫЙ
{
    warning = 0;
    va = dha / dt; // скорость штока(штоков):

    if ( ha_+dha >= c1.s+c2.s )         status = 2; // обжат
    if ( dha     >  0 )                 status = 1; // обжимается
    if ((dha==0))  status = 0; // неподвижен    &&(ha_+dha<c1.s+c2.s) //2017.09.20
    if ( dha     <  0 )                 status =-1; // разжимается
    if ( ha_+dha <= 0)                  status = 0; //-2 разжат

    abInsideBalance();// баланс демпфера и амортизатора:

return status;
}

void AbsorberMi2::abInsideBalance()// определение силы всего амортизатора
{
    c1.dh= 0;    c2.dh= 0;
    c1.P = 0;    c2.P = 0;
    _Pa = 0;
    double  daf = 0,     naa, nad;
    int     zdaf_= 1,    zdaf = 1;
    int     Adirect;

    if (dha == 0)// нет перемещений штоков амортизатора /////////////////
    {
        Adirect = 0;
        c1.dh = 0;
        c2.dh = 0;
        abRodsForces();
        if (status>=0) _Pa = min( c1.P, c2.P );//было ">"
        else           _Pa = max( c1.P, c2.P );//max-min не вижу разницы;  было "max" до 2017.09.19 !!!
    }
    else
    {// dha != 0
        if (dha>0) Adirect =+1;
        if (dha<0) Adirect =-1;

        nad = 0.5;
        naa = 0.5;
        c1.dh = dha*nad;
        c2.dh = dha*naa;

        abRodsForces();

        daf = (c1.P - c2.P) / fabs(0.5*(c1.P + c2.P));// ?
        if(daf>0) zdaf = 1;
        else     zdaf = -1;
        zdaf_ = zdaf;
        int q=0;
        while (zdaf_ == zdaf)
        {// цикл до смены знака zdaf
            q++;
            zdaf_ = zdaf;
            if(daf*Adirect < 0)// c1.P < c2.P
            {
                nad = nad + kra;
                naa = 1 - nad;
            }
            else  // c1.P >= c2.P
            {
                naa = naa + kra;
                nad = 1 - naa;
            }
            c1.dh = dha*nad;
            c2.dh = dha*naa;

            abRodsForces();

            if ((c2.h<=0)||(c2.h>=c2.s))// a- обжат/разжат до упора
            {//a-
                if ((c1.h>0)&&(c1.h<c1.s))// d+ работает
                {//d+   // заходит на несколько шагов в самом начале
                        if (c2.h <= 0)// a- разжат до упора
                        {
                            c2.dh = 0 - c2.h_;
                            c1.dh = dha - c2.dh;
                        }
                        if (c2.h >= c2.s)// a- обжат до упора
                        {
                            c2.dh = c2.s - c2.h_;
                            c1.dh = dha - c2.dh;
                        }
                        abRodsForces();
                        _Pa = c1.P;
                        goto END_AB_BALANCE;
                }//end d+
                else // d- обжат/разжат до упора ///////////////
                {//d-
                        if (c2.h <= 0)// a- разжат до упора
                        {
                            c2.dh = 0 - c2.h_;
                            c1.dh = dha - c2.dh;
                        }
                        if (c2.h >= c2.s)// a- обжат до упора
                        {
                            c2.dh = c2.s - c2.h_;
                            c1.dh = dha - c2.dh;
                        }
                        if (c1.h <= 0)// d- разжат до упора
                        {
                            c1.dh = 0 - c1.h_;
                            c2.dh = dha - c1.dh;
                        }
                        if (c1.h >= c1.s)// d- обжат до упора
                        {
                            c1.dh = c1.s - c1.h_;
                            c2.dh = dha - c1.dh;
                        }
                        abRodsForces();
                        _Pa = max( c1.P, c2.P );//max-min не вижу разницы;  было "max" до 2017.09.19 !!!
                        goto END_AB_BALANCE;
                }//end d-
            }// end a-
            else // a+ работает ////////////////////////////////////
            {//a+
                if ((c1.h <= 0)||(c1.h >= c1.s))// d- обжат/разжат до упора
                {//d-
                    if (c1.h <= 0)// d- разжат до упора
                    {
                        c1.dh = 0 - c1.h_;
                        c2.dh = dha - c1.dh;
                    }
                    if (c1.h >= c1.s)// d- обжат до упора
                    {
                        c1.dh = c1.s - c1.h_;
                        c2.dh = dha - c1.dh;
                    }
                    abRodsForces();
                    _Pa = c2.P;
                    goto END_AB_BALANCE;
                }//end d-
                else // d+ работает ////////////////////////////
                {//d+
                    if(c1.P!=-c2.P)
                            daf = (c1.P - c2.P) / fabs(0.5*(c1.P + c2.P));
                    if(daf>=0) zdaf = 1;
                    else       zdaf =-1;

                    if(nad>=1)//
                    {
                        c1.dh = dha;
                        c2.dh = 0;
                        abRodsForces();
                        _Pa = c1.P;
                        goto END_AB_BALANCE;
                    }
                    if(naa>=1)//
                    {
                        c1.dh = 0;
                        c2.dh = dha;
                        abRodsForces();
                        _Pa = c2.P;
                        goto END_AB_BALANCE;
                    }
                        _Pa = max( c1.P, c2.P );//max-min не вижу разницы;   было "max" до 2017.09.19 !!!
                }//end d+ //////////////////////////////////////
            }// end a+
            if (q>9999) {  warning=210;  break; }//"abInsideBalance(): q>999"
        }// end while
    }// end if dha!=0
END_AB_BALANCE:
    // усилие на штоке амортизатора:
    Pa = _Pa;
    // ход штока амортизатора:
    ha  = c1.h + c2.h;
    // работа за обжатие dha амортизатора:
    dA = c1.P*c1.dh + c2.P*c2.dh;
}

void AbsorberMi2::abRodsForces()// организация расчета сил + перемещений штоков двухкамерного амортизатора
{
    double _p=0;
    v1to2=0, w12r=0, w12=0;
    //положения штоков
    c1.h = c1.h_ + c1.dh;
    c2.h = c2.h_ + c2.dh;
    //скорости штоков
    c1.v = c1.dh / dt;
    c2.v = c2.dh / dt;

    //скорости перетекания жидкости через отверстия дросселей
    {
    //шток 1
    if(c1.fb !=0)  c1.wb = c1.v * c1.Fb / c1.fb ;  else c1.wb= 0;
    if(c1.fbr!=0)  c1.wbr= c1.v * c1.Fb / c1.fbr;  else c1.wbr=0;
    if(c1.f !=0)
    {
        if(c1.Dv)  c1.w  = c1.v * c1.Fv / c1.f;
        else       c1.w  = c1.v * c1.F  / c1.f;
    }
    else c1.w = 0;
    if(c1.fr!=0)
    {
        if(c1.Dv)  c1.wr = c1.v * c1.Fv / c1.fr  /valve_damp(c1.f,c1.fr);
        else       c1.wr = c1.v * c1.F  / c1.fr  /valve_damp(c1.f,c1.fr);
    }
    else c1.w = 0;
    //шток 2
    if(c2.fb !=0)  c2.wb = c2.v * c2.Fb / c2.fb ;  else c2.wb= 0;
    if(c2.fbr!=0)  c2.wbr= c2.v * c2.Fb / c2.fbr;  else c2.wbr=0;
    if(c2.f!=0)
    {
        if(c2.Dv)  c2.w  = c2.v * c2.Fv / c2.f;
        else       c2.w  = c2.v * c2.F  / c2.f;
    }
    else c2.w = 0;
    if(c2.fr!=0)
    {
        if(c2.Dv)  c2.wr = c2.v * c2.Fv / c2.fr  /valve_damp(c2.f,c2.fr);
        else       c2.wr = c2.v * c2.F  / c2.fr  /valve_damp(c2.f,c2.fr);
    }
    else c2.w = 0;
    }

    //дополнительная скорость на дросселе 2го штока от вытеснения масла 1 штоком
    if(c2.F !=0)   v1to2 = c1.v * c1.F / c2.F;   else v1to2= 0; //c1.Fs / c2.Fc;
    if(c2.f !=0)   w12  = v1to2 * c2.F / c2.f;   else w12  = 0; //c2.Fc
    if(c2.fr!=0)   w12r = v1to2 * c2.F / c2.fr;  else w12r = 0; //c2.Fc

    //перепады давлений на дросселях
    c1.dpa = 0.5*ro*kgds *pow(c1.w  -w12 , 2.0);                        ///-w12
    c1.dpar= 0.5*ro*kgds *pow(c1.wr -w12r, 2.0)*kgds_corr(c1.p, 0.5*ro*kgds*pow(c1.wr -w12r,2));///-w12r
    c1.dpb = 0.5*ro*kgds *pow(c1.wb ,2.0);
    c1.dpbr= 0.5*ro*kgds *pow(c1.wbr,2.0);
    c2.dpb = 0.5*ro*kgds *pow(c2.wb ,2.0);
    c2.dpbr= 0.5*ro*kgds *pow(c2.wbr,2.0);
    c2.dpa = 0.5*ro*kgds *pow(c2.w  +w12 , 2.0);                        ///-w12
    c2.dpar= 0.5*ro*kgds *pow(c2.wr +w12r, 2.0)*kgds_corr(c2.p, 0.5*ro*kgds*pow(c2.wr +w12r,2));///-w12r

    //объем газа, м^3
    {
        c1.dV = c1.F * c1.dh;//c1.Fs
        c2.dV = c2.F * c2.dh;//c2.Fc
        dV = c1.dV + c2.dV;
        V  = V_ - dV;
        c1.V = V;
        c2.V = V;
    }

    //давление газа, Па
    {
        if(V_-dV == 0) _p = 0;
        else
        {
            if(dV>0)
                _p = p0 * pow( V0/V, kpoly );//сжатие
            else  // _p = p0 * pow( V0/(V_-dV), kpolyr);//разжатие
                _p = c1.p_*pow(V_/V, kpolyr);//разжатие
        }
        c1.p = _p;
        c2.p = _p;
    }

    //усилия на штоках
    cam1force();
    cam2force();
}

double IAbsorber::kgds_corr(double p, double dpa)
{
    double corr = 1;
    double X = fabs(dpa/p);

    //if     (X<=0.6)  corr = 1;
    //else if(X>=0.9)  corr = 0.2;
    //else  corr = 2.6-2.666667*X;//0.6...0.9    //corr =  -2.75722241401672+29.336784362793*X-84.674560546875*pow(X,2)+107.146202087402*pow(X,3)-50.2491226196289*pow(X,4);//0.6...0.9

    ///1.0Hz
    //if     (X<=0.50)  corr = 1.0;
    //else if(X>=0.75)  corr = 0.2;
    //else    corr = 2.6 - 3.2 * X;
    ///0.6Hz
    //if     (X<=0.4)  corr = 1.0;
    //else if(X>=0.6)  corr = 0.6;
    //else    corr = 1.8 - 2.0 *X;

    if     (X<=0.4)  corr = 1.0;
    else if (X>0.4 && X<=0.6)  corr = 1.8 - 2.0 *X;   //corr от 1 до 0.6;
    else if (X>0.6 && X<=0.9)  corr = 2.6-2.666667*X; //corr от 0.6 до 0.2;
    else  corr = 0.2;

return corr;
}
double IAbsorber::valve_damp(double f, double fr)
{
    double delay = 0.1;//cекунд
    double Y = 0, X = t_;

    if (dt < delay && t_ < 1.01*delay)
        //Y=0.99987-32.91886*X+274.013153*pow(X,2)-54.824562*pow(X,3);//delay = 0.06
        //Y=1-30.833334*X+312.5*pow(X,2)-1041.66662597656*pow(X,3);//delay = 0.08
        Y=0.992381-29.2281742*X+288.5416565*pow(X,2)-954.861084*pow(X,3);//delay = 0.10

return ( Y*f + (1-Y)*fr )/fr;
}

int  AbsorberMi2::SolveFree(double Plimit)// расчет разгруженного амортизатора (при отрыве пневматика)
{
    isFree = true;
    status = -2;

    abInsideBalanceFree(Plimit);
    //уточнение состояния
    if(Plimit==0)
    {
        isFree = true;
        if ( dha <  0 )                 status =-2; // разжимается
        if ( ha  <= 0 )                 status = 0; //-2 разжат
    }
    else
    {
        isFree = false;
        if ( dha <  0 )                 status =-1; // разжимается
        if ( dha >  0 )                 status = 1; // cжимается
        if ( ha  <= 0 )                 status = 0; //-2 разжат
    }

return status;
}

void AbsorberMi2::abInsideBalanceFree(double limit)// определение силы свободного амортизатора
{
    c1.dh= 0;    c2.dh= 0;
    c1.P = 0;    c2.P = 0;
    bool c1_change, c2_change;
    double na_free = kra*0.00001;// kra*0.000001
    int q=0;

    abRodsForces();
    ha = c1.h + c2.h;

    while (( max(c1.P,c2.P) > limit ) && (ha>0))//while (( c1.P > limit) || (c2.P > limit))//    <modify 2020.02
    {
        q++;
        if ( ( c1.P >= limit ) && (c1.h >= 0) )
        {
            c1.dh -= na_free;
            c1_change = true;
        }
        else c1_change = false;
        if ( ( c2.P >= limit ) && (c2.h >= 0) )
        {
            c2.dh -= na_free;
            c2_change = true;
        }
        else c2_change = false;

        abRodsForces();
        dha = c1.dh + c2.dh;

        if (!c1_change && !c2_change)
        {
            abInsideBalance();
            break;///c1.P = 0; c2.P = 0; <modify 2020.02
        }
        if (q>9999) {  warning=211;  break; }//"abInsideBalanceFree(): q>999"
    }//endwhile

    if (c1.h < 0)  c1.h = 0;
    if (c2.h < 0)  c2.h = 0;
    Pa = max( c1.P, c2.P );//max() [2020.02]
    // скорость штока:
    va = dha / dt;
    // ход штока амортизатора:
    ha = c1.h + c2.h;
    // работа за обжатие dha амортизатора:
    dA = c2.P * c2.dh + c1.P * c1.dh;
}

void AbsorberMi2::SolveSingleTest()// расчет изолированного амортизатора (испытываемого)
{
    dha = va*dt;
    if ( ha_+dha <= 0 )                 status = 2; // обжат
    if ( dha     >  0 )                 status = 1; // обжимается
    if ((dha==0)&&(ha_+dha<c1.s+c2.s))  status = 0; // неподвижен
    if ( dha     <  0 )                 status =-1; // разжимается
    if ( ha_+dha >= c1.s+c2.s)          status = 0; //-2 разжат
    // баланс демпфера и амортизатора:
    abInsideBalance();
}


