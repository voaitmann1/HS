#pragma hdrstop
#pragma package(smart_init)
#include <QtMath>
#include "pneumatic.h"

void Pneumatic::setNull()
{
    h.x  = 0;   P.x  = 0;
    h.y  = 0;   P.y  = 0;
    h.z  = 0;   P.z  = 0;
    h1.x = 0;   P1.x = 0;
    h1.y = 0;   P1.y = 0;
    h1.z = 0;   P1.z = 0;
    h2.x = 0;   P2.x = 0;
    h2.y = 0;   P2.y = 0;
    h2.z = 0;   P2.z = 0;
    status=0;
}

void Pneumatic::Preprocess()
{
    status= 0;
    dppo =  dpmd/0.95;//обжатие пневматика полное
    dA = 0;
    ang_sx = 0;
    //изменение обжатия пневматика
    dhx = 0;    dhy = 0;    dhz = 0;
    //скорость изменения обжатия пневматика
    v.x = 0;    v.y = 0;    v.z = 0;
    //обжатия пневматика
    h.x  = 0;   h.y  = 0;   h.z  = 0;
    h_.x = 0;   h_.y = 0;   h_.z = 0;
    h1.x = 0;   h1.y = 0;   h1.z = 0;
    h2.x = 0;   h2.y = 0;   h2.z = 0;
    //усилие на пневматике
    P.x  = 0;   P.y  = 0;   P.z  = 0;
    P1.x = 0;   P1.y = 0;   P1.z = 0;
    P2.x = 0;   P2.y = 0;   P2.z = 0;
}

void Pneumatic::get_pncompress(int nwh, xyz kh)//
{
    //обжатия в новом положении
    h.y = h_.y + dhy;

    h.x = h_.x - dhx;
    if (h_.x > 0.5*dpmd)  h.x = 0.5*dpmd;
    if (h_.x <-0.5*dpmd)  h.x =-0.5*dpmd;

    h.z = h_.z - dhz;
    if (h_.z > 0.5*dpmd)  h.z = 0.5*dpmd;
    if (h_.z <-0.5*dpmd)  h.z =-0.5*dpmd;

    if (nwh==2)
    {
        h1.x = h.x + kh.x;
        h1.y = h.y + kh.y;
        h1.z = h.z + kh.z;
        h2.x = h.x - kh.x;
        h2.y = h.y - kh.y;
        h2.z = h.z - kh.z;
    }
}

void Pneumatic::Solve(int nwh)// расчет пневматика
{
    warning = 0;
    if (dhy > 0)  status = status1 = status2 = 1; // обжимается
    if (dhy < 0)  status = status1 = status2 =-1; // разжимается
    //скорость обжатия пневматика
    v.x = dhx / dt;
    v.y = dhy / dt;
    v.z = dhz / dt;
    //усилие на пневматиках
    status = PneumaForce(h,P);
    Pdf_ = Pdf;
    Pd_ = Pdmp;
    if(nwh==2)
    {
        status1 = PneumaForce(h1,P1);
        status2 = PneumaForce(h2,P2);
        if ((status1== 0)&&(status2== 0)) status = 0;
        if ((status1== 2)||(status2== 2)) status = 2;
        P.x = P1.x + P2.x;
        P.y = P1.y + P2.y;
        P.z = P1.z + P2.z;
    }
    //работа за обжатие dhy
    dA = P.y * dhy;
    //работа сил трения при проскальзывании пневматика
    if(fabs(ang_sx)>0.98*0.5*M_PI)
        dAtr = 0;
    else
        dAtr = fabs(dhy) * Ptr * tan(ang_sx);//dhtrfabs(
    //проекция на вертикаль усилия на пневматике
    ///if(status<0 && dhtr!=0)// && dA>dAtr
    ///    P.y -= 1;//0.1*Ptr;
    ///    P.y =(dA+dAtr)/dhy;//

}

int Pneumatic::PneumaForce(xyz &h, xyz &P)//усилие от сжатия
{
    int status=0;
    dhtr   = 0;    Ptr = 0;//вертикальное усилие пропорциональное работе сил трения
    Pdmp.x = 0;    Pdmp.y = 0;    Pdmp.z = 0;
    Pdf.x  = 0;    Pdf.y  = 0;    Pdf.z  = 0;
    //вычисление усилий для 1 колеса ///////////////////////////////////////////////////////////////////
    if (h.y <= 0)//пневматик разгружен
    {
        status = 0;
        kpy = 0;
        h.x = 0;    h.y = 0;    h.z = 0;
        P.x = 0;    P.y = 0;    P.z = 0;
        v.x = 0;    v.y = 0;    v.z = 0;
    }
    else//h.y > 0 //пневматик нагружен
    {
        //усилие без учета условий контакта и демпфирования
        if (h.y <= dppo)//нормальный режим
        {
            if (dhy > 0) status= 1;
            if (dhy < 0) status=-1;
            //коэф.при вертикальной проекции усилия
            kpy = pow(1-0.999*h.y/dppo, -alfanp);
            //усилие без демпфирования
            Pdf.y = cpy*h.y *kpy;
            Pdf.x = cpx*h.x;
            Pdf.z = cpz*h.z;
        }
        else//h.y > dppo деформация резины и деталей шасси
        {
            status = 2;//обжат полностью
            //коэф.при вертикальной проекции усилия
            kpy = pow(1-0.999, -alfanp);
            //усилие без демпфирования
            Pdf.y = cpy*h.y *kpy;//Pdf.y = cpy*h.y *pow(1-0.999, -alfanp) + pow(cpy*fabs(h.y-dppo),1.5);//,1.75
            Pdf.x = cpx*h.x;
            Pdf.z = cpz*h.z;
            //деформация деталей шасси //,1.75зависимость подобрана под общий вид диаграмм обжатия, с попыткой учесть деформацию деталей шасси
            if ((h.y>dpmd)&&(dhy>0)) Pdf.y += pow(cpy*fabs(h.y-dppo),1.5);
            if ((h.y>dpmd)&&(dhy<0)) Pdf.y += pow(cpy*fabs(h.y-dppo),1.5);///pow(cpy*fabs(h.y-dppo),1.2);
        }

        P.x = Pdf.x;
        P.y = Pdf.y/kpy;
        P.z = Pdf.z;

        //учет потерь на внутреннее трение
        if(kdp) AddDamping();

        //проверка по трению в контакте пневматика с землей
        FrictionalCheck( h, P );

        //отсечение отрицательных значений
        P.y = (P.y<0)? (0): (P.y*kpy);
    }
return status;
}

void Pneumatic::FrictionalCheck(xyz &h, xyz &P)//сила трения пневматика с землей
{
    Pf0 = fabs(P.y) *fs;//f0 сила трения покоя  ...fs чтобы избежать дрожания вектора силы на пневматике
    Pfs = fabs(P.y) *fs;//сила трения скольжения
    Pfk = fabs(P.y) *fk;//сила трения качения
    //dz ----------------------------------------------------------------------
    if(P.z>=0)
    {
        if( P.z > Pf0)
        {
            if(fabs(h.z)>=fabs(h_.z))
            {//проскальзывание
                dhtr = fabs(h.z) - fabs(h_.z);//величина проскальзывания
                h.z  = h_.z;
                Ptr  = Pfs;//P.z - Pfs;
                //dAtr = Ptr * dhtr;//работа за dhtr проскальзывание пневматика
                //P.y = 0.9*P.y;
            }
            P.z = Pfs;
        }//  dhz = 0;   hz = hz_;
    }
    else
    {
        if(-P.z > Pf0)
        {
            if(fabs(h.z)>=fabs(h_.z))
            {//проскальзывание
                dhtr = fabs(h.z) - fabs(h_.z);//величина проскальзывания
                h.z  = h_.z;
                Ptr  = Pfs;//-P.z - Pfs;
                //dAtr = Ptr * dhtr;//работа за dhtr проскальзывание пневматика
                //P.y = 0.9*P.y;
            }
            P.z =-Pfs;
        }//  dhz = 0;   hz = hz_;
    }
    //dx ----------------------------------------------------------------------
    if(P.x>=0)
    {
        switch ( isbrake )
        {
        case 0:
            if( P.x > Pfk) { P.x = Pfk; if(fabs(h.x)>=fabs(h_.x)) h.x = h_.x; }
            break;//колесо свободно      dhx = 0;   hx = hx_;0
        case 1:
            if( P.x > Pf0) { P.x = Pfs; if(fabs(h.x)>=fabs(h_.x)) h.x = h_.x; }
            break;//колесо заторможено   dhx = 0;   hx = hx_;0
        }
    }
    else
    {
        switch ( isbrake )
        {
        case 0:
            if(-P.x > Pfk) { P.x =-Pfk; if(fabs(h.x)>=fabs(h_.x)) h.x = h_.x; }
            break;//колесо свободно      dhx = 0;   hx = hx_;0
        case 1:
            if(-P.x > Pf0) { P.x =-Pfs; if(fabs(h.x)>=fabs(h_.x)) h.x = h_.x; }
            break;//колесо заторможено   dhx = 0;   hx = hx_;0
        }
    }//------------------------------------------------------------------------
}

void Pneumatic::AddDamping()//учет демпфирования от внутреннего трения
{
    ///wheel
    ///if (fabs(v.x)<0.01) Pdmp.x = kdp*Pdf.x *fabs(v.x)/0.01;   else Pdmp.x = kdp*Pdf.x;
    ///if (fabs(v.y)<0.01) Pdmp.y = kdp*Pdf.y *fabs(v.y)/0.01;   else Pdmp.y = kdp*Pdf.y;
    ///if (fabs(v.z)<0.01) Pdmp.z = kdp*Pdf.z *fabs(v.z)/0.01;   else Pdmp.z = kdp*Pdf.z;

    ///skid
    ///double Pdmp = fabs(kdp * dha * Ke);
    Pdmp.x = 0;//Pdf.x *dha *kdp *fs *1000000;
    Pdmp.y = 0;//Pdf.y *dha *kdp *fs *1000000;
    Pdmp.z = 0;//Pdf.z *dha *kdp *fs *1000000;

    //усилие демпфирования (от внутреннего трения)//kdp - коэф.демпфирования пневматика [H*c^2/mm]
    //Pdmp.x = (fabs(v.x)<0.001) ? (0) : (kdp*Pdf.x);//fabs(
    //Pdmp.y = (fabs(v.y)<0.001) ? (0) : (kdp*Pdf.y);//fabs(
    //Pdmp.z = (fabs(v.z)<0.001) ? (0) : (kdp*Pdf.z);//fabs(

    P.x = (v.x*h.x>0) ? (P.x-Pdmp.x) : (P.x+Pdmp.x);
    P.y = (v.y    <0) ? (P.y-Pdmp.y) : (P.y+Pdmp.y);
    P.z = (v.z*h.z>0) ? (P.z-Pdmp.z) : (P.z+Pdmp.z);
}


