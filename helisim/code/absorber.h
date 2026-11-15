#ifndef absorberH
#define absorberH
#include <QtMath>
#include <QVector>
using namespace std;

class IAbsorber
{
public:
        virtual ~IAbsorber();
                void setNull();
                void Preprocess();
                void cam1force();
                void cam2force();
                double get_restriction(vector<double> &restriction, vector<double> &rodposition, double rp);
                double kgds_corr(double p, double dpa);
                double valve_damp(double f, double fr);
                double min(double x1, double x2);
                double max(double x1, double x2);
        virtual int  Solve()=0;
        virtual int  SolveFree(double Plimit)=0;
        virtual void SolveSingleTest()=0;

        int     status,status_,
                warning;
        bool    isFree;//isReverse, isTrouble,
        double  dA,
                Pa, Pmin,// суммарное усилие на амортизаторе
                ha, ha_, // ход амортизатора общий
                dha,     // измение хода амортизатора общее
                dt,va,kra,
                t_,//только для изолированного амортизатора (испытываемого) SolveSingleTest()
                ro, mu, kgds, kpoly,kpolyr,
                S, // ход амортизатора общий максимальный
                p0, V0,//давление и объем газа в разжатом состоянии
                _Pa,Ph,Pg,Pf,
                dV,V_,V;// суммарное изменение объема, объемы до и после

        struct  absorber_camera
        {
        double  s,  //макс.ход
                Dc, //Внутренний диаметр цилиндра = Наружный диаметр буксы   Dbyk_d
                Dv, //внутр.диаметр плунжера                           Dwt_d_vn
                Ds, //наруж.диаметр плунжера                           Dwt_a / Dwt_d
                f, //сумм.площадь центральных отверстий на прямом ходу    fw_a / fw_d
                fr,//сумм.площадь центральных отверстий в штоке на обратном ходу  fw_a_rev
                fb, //сумм.площадь отверстий в буксе на прямом ходу    fw_d_byk
                fbr,//сумм.площадь отверстий в буксе на обратном ходу   fw_d_byk
                F,  //рабочая площадь штока
                Fc, //площадь по внутреннему диаметру цилиндра = наружному диаметру буксы     Fw_a / Fw_d
                Fb, //площадь буксы                                    Fbyk_d
                Fv, //площадь по внутр.диаметру штока                  Fwt_d_vn
                Fs, //площадь штока по наруж.диаметру штока
                ktr;//Коэффициент трения в буксах и уплотнениях;
        vector <double> v_h,v_f,v_fr;
        double  h,h_,//ход штока
                dh,  //изменение хода штока
                v,      //скорость штока
                dV,V_,V,//изменение объема газ.камеры, объем до и после
                p0,V0,//давление и объем газа в разжатом состоянии
                p, p_,   //давление
                w,wr,wb,wbr,//скорость перетекания жидкости через отверстия дросселя на прямом и обатном ходу
                dpa,dpar,//перепады давлений на центральных дросселях на прямом и обратном ходу
                dpb,dpbr,//перепады давлений на буксовых дросселях на прямом и обратном ходу
                Ph,  //сила гидравлического сопротивления
                Pg,  //сила сжатия газа
                Pf,  //сила механического трения
                P ;  //результирующее усилие на штоке
        } c1, c2;

        double v1to2, w12, w12r;

        /// НЕЛИНЕЙНАЯ ПРУЖИНА
        int     q;
        double  hu,
                Ke,
                Ki,
                Ci,
                dh12, hmax;
protected:
};

class Absorber1cam : public IAbsorber
{
public:
    virtual ~Absorber1cam();
    virtual  int Solve();
    virtual  int SolveFree(double Plimit);
    virtual void SolveSingleTest();
private:
            void abInside();
            void abRodForce();
            void abInsideFree(double limit);
            void c2SetNull();
};

class Absorber2cam : public IAbsorber
{
public:
    virtual ~Absorber2cam();
    virtual  int Solve();
    virtual  int SolveFree(double Plimit);
    virtual void SolveSingleTest();
private:
            void abInsideBalance();
            void abRodsForces();
            void abInsideBalanceFree(double limit);
            void abForceFree();
};

class AbsorberMi2 : public IAbsorber
{
public:
    virtual ~AbsorberMi2();
    virtual  int Solve();
    virtual  int SolveFree(double Plimit);
    virtual void SolveSingleTest();
private:
            void abInsideBalance();
            void abRodsForces();
            void abInsideBalanceFree(double limit);
            void abForceFree();

};

class AbsorberEq : public IAbsorber
{
public:
    virtual ~AbsorberEq();
    virtual  int Solve();
    virtual  int SolveFree(double Plimit);
    virtual void SolveSingleTest();
private:
            void abInside();
            void abInsideFree(double limit);
            void c12SetNull();
            void AddDamping();//учет демпфирования от внутреннего трения
};

#endif
