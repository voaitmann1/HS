#ifndef shassiH
#define shassiH
#include "absorber.h"
#include "pneumatic.h"
#include "types.h"

class IShassi
{
public:
        virtual ~IShassi();
        virtual void get_pk(double dhay) =0;
        virtual void Preprocess() =0;
        IAbsorber *ab;
        Pneumatic *pn;

        void ShassiSolver();
        xyz  SKo2SKS(xyz A);
        xyz  SKS2SKo(xyz Asks);
        xyz  SKS2SKN(xyz A);
        xyz  SKN2SKS(xyz A);
        xyz  SKsh2SKo(xyz A, double ang_s_x, double localerotate);
        xyz  SKo2SKsh(xyz A, double ang_s_x, double localerotate);
        void delta_correction();
        xyz  corrD(xyz D_);
        xyz  getC();

        int     warning;
        float   ground;//отклонение земли от номинальной(нулевой) поверхности
        int     nwh;   //количество колес в стойке
        int     maycont,contact;//индикатор контакта (maybe/not;есть/нет)
        int     status,//изменение геометрии шасси: сжимается-разжимается
                stype, ptype, atype,//типы стойки,пневматика,амортизатора
                direct;//расположение стойки (справа-слева)
        bool    kostyl;
        double  yrot;
        double  dhpy, dhay;
        double  psih, psif,//передаточные коэффициенты
                pk_p, pkfp,//передаточные коэффициенты
                pk_a, pkfa,//передаточные коэффициенты
                krp,       //коэф.разжатия пневматика
                bpa,       //точность баланса сил на пневматике и амортизаторе
                ang_wh,    //отклонение плоскости колеса;
                Ppy,Pay,   //проекция усилия на вертикальную ось вертолета:
                Aa,Ap,Aptr,//работа стойки на данный момент
                CGfree,CG;
        double  AB,AB_,ABfull,  AByzfull, ABxyfull,  ABtest,            //ABB,BWtest,
                lambda,
                whd,//половина расстояния между колесами = смещение колеса относительно плоскости качания рычага стойки
                ang_sx,ang_sz, ang_ax, ang_az, ang_a, ang_pn,
                ang_sx_full,ang_sz_full, ang_a_full, ang_pn_full,
                BOWxy,BOWyz,
                OV,OVxy, BV,BVxy, VW,VWxy,  BOVxy,VOWxy, dOx,dOy,OQ,
                OAyz, OByz, OWyz, BWyz, AByz, AByz_,
                OAxy, OBxy, OWxy, BWxy, ABxy, ABxy_,
                AOH, OA,OB,OW,BW,
                WD, Rmd;
        xyz     deltan,  delta,
                O, A, B, C, D, W, V, Q,
                O_,A_,B_,C_,D_,W_,V_,Q_,
                C1,D1,W1, C2,D2,W2,
                Dloc,Wloc,Bloc,
                saC,saC_, Cfree, Cfree_abs, C_sks,C1sks,C2sks,
                snC, snCteor,  snD0,dD, ssD0,
                Bfull,Bfree, Wfree,Wfull,
                Dfree,
                Dfree_skn,Cfree_skn,
                Dfree_sks,Cfree_sks,
                kh2wh;//for 2 wheels on stojka
        double  df = 0;
        int     zdf= 1;
        bool    stopbalance;
        xyzkrt  orient;
        xyz     Pp_s,Pp1s,Pp2s;//для нормального к земле направления

protected:
           void commonPre();
           int  isContact();
           xyz  getDelta(xyz snC);
           void paBalance();
           void paBalanceOLD(int contact);
           bool getForces(double np, double na);
           bool CheckLimit(double dy);

           void pk_initial(double AB, double pk_a, double pk_p);
           void pk_return(double AB);
           QVector<double> v_ab, v_pk_a, v_pk_p;
};

class PyramidType : public IShassi
{
public:
        PyramidType(int pneumID, int absorbID);
        virtual ~PyramidType();
        virtual void get_pk(double dhay);
        virtual void Preprocess();
private:
        void set_pk(double dhay);
};

class LeverType : public IShassi
{
public:
        LeverType(int pneumID, int absorbID);
        virtual ~LeverType();
        virtual void get_pk(double dhay);
        virtual void Preprocess();
private:
        xyz getW(double ang_s_z);
        void set_pk(double dhay);
};

class SkidType : public IShassi
{
public:
        SkidType(int pneumID, int absorbID);
        virtual ~SkidType();
        virtual void get_pk(double dhay);
        virtual void Preprocess();
private:
        //xyz getW(double ang_s_z);
};

/*class Stojka_LeverType : public IShassi
{
public:
        Stojka_LeverType(int pneumID, int absorbID);
        virtual ~Stojka_LeverType();
        virtual void get_pk(double dhay);
        virtual void Preprocess();
private:
        xyz getW(double ang_s_z);
};*/

#endif


