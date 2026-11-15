#ifndef rotorsH
#define rotorsH
#include <vector>
#include <QString>
#include "types.h"

#include"mymathlib.h"
#include"fluttermodels.h"

#define mbl 6
#define Ca 50
#define CM 20

using namespace std;


struct segment
{
    double  Ux, Uy, U, Fi, fi,fl, alfar, M, cy,cx,mz, xd,
            lambda, dT_dr, dQ_dr, dMa_dr,   Uywz, Uywx;
};


struct Blades//параметры лопасти и шарниров // для моделирования по азимуту
{
        double sd,sd_,vd;//параметры демпфера лопасти

        //распределенные по лопасти параметры
        vector <vector <double>> alfar;     //угол атаки в сечении, градусы
        vector <vector <double>> dT_dr;     //погонная сила тяги, Н/м
        vector <vector <double>> dQ_dr;     //погонная сила сопротивления, Н/м
        vector <vector <double>> dMa_dr;    //погонный аэродинамический момент отн.ОШ, Н*м
        vector <vector <double>> fi;        //угол установки сечения, градусы
        vector <vector <double>> fl;        //угол закрутки сечения, градусы
        vector <vector <double>> Fi;        //угол притекания в сечении, градусы
        vector <vector <double>> M;         //числа Маха в сечении
        vector <vector <double>> cy;        //коэф. подъемной силы в сечении
        vector <vector <double>> cx;        //коэф. профильного сопротивления в сечении
        vector <vector <double>> mz;
        vector <vector <double>> Ux;        //скорость обтекания в плоскости вращения, м/с
        vector <vector <double>> Uy;        //скорость обтекания в плоскости взмаха, м/с
        vector <vector <double>> Uywx,Uywz;
        vector <vector <double>> U;         //суммарная скорость обтекания в сечении, м/с
        vector <vector <double>> lambda;    //коэф. протекания в сечении
            vector <vector <double>> dH_dr;     //
            vector <vector <double>> dS_dr;     //

        vector <double> Tv;         //индуктивная мощность в конкретном азимуте, Вт
        vector <double> XaU;        //профильная мощность в конкретном азимуте, Вт

        //параметры движений лопасти
        double psi0;
        vector <double> psi,   //азимут лопасти (СКПС), грд
                        //dfidt,
                        fi7, fi7_,     //угол установки, грд
                        dfi7,dfi7_,
                        d2fi7,d2fi7_,   //угл.ускорение закрутки, грд[взм]/грд2[об]
                        beta,       //угол взмаха, грд
                        dbeta,      //угл.скорость взмаха, грд[взм]/грд[об]
                        d2beta,     //угл.ускорение взмаха, грд[взм]/грд2[об]
                        ksi,        //угол качания, грд
                        dksi,       //угл.скорость качания, грд[взм]/грд[об]
                        d2ksi,      //угл.ускорение качания, грд[взм]/грд2[об]
                        khi,        //угол "качания ЦТ отн.оси винта", грд
                        gama,gama_,    //угол закрутки, грд
                        dgama,dgama_,  //угл.скорость закрутки, грд[взм]/грд[об]
                        d2gama,d2gama_;//угл.ускорение закрутки, грд[взм]/грд2[об]

        vector <double> Fx, Fy, Fz, //проекции суммарной силы в шарнирах (СКЛ), Н
                        //Fx_,Fy_,Fz_,//только аэродинамические силы в шарнирах (СКЛ), Н
                        FNx,FNz;
        vector <double> T,          //сила тяги лопасти (СКЛ), Н
                        Q,          //сила сопротивления лопасти (СКЛ), Н
                        Ma,         //аэродинамический момент отн.ОШ, Н*м
                        K1,         //сила Кориолиса в плоскости вращения от взмаха (СКЛ), Н
                        //K2,         //сила Кориолиса от качания
                        Nr,         //центробежная сила от вращения винта (СКЛ), Н
                        Nf,         //центробежная сила от маховых движений (СКЛ), Н
                        P1,         //сила инерции взмаха (СКЛ), Н
                        P2,         //сила инерции качания (СКЛ), Н
                        Paxz,//сила инерции в плоскости винта (от линейного перемещения вдоль продольной и поперечной осей CKПС)
                        Payy,//общая cила инерции в плоскости тяги
                        Pay, //cила инерции от линейного перемещения вдоль оси НВ
                        Pexz,//сила инерции от вращения НВ вокруг продольной и поперечной осей CKПС
                        Pkxz;//силы инерции Кориолиса, возникающие от вращения НВ вокруг продольной и поперечной осей CKПС на азимуте psi

        //ГШ, СКЛ,
        vector <double> M1T,        //момент отн.ГШ силы тяги (СКЛ), Н*м
                        M1Q,        //момент отн.ГШ силы сопротивления лопасти dQ (СКЛ), Н*м
                        M1K,        //момент отн.ГШ силы Кориолиса (СКЛ), Н*м
                        M1Gx,M1Gy,  //момент отн.ГШ веса лопасти (СКЛ), Н*м
                        M1Px,M1Py,  //момент отн.ГШ силы инерции взмаха (СКЛ), Н*м
                        M1Cx,M1Cy,  //момент отн.ГШ центробежной силы (СКЛ), Н*м
                        M1Cz,
                        hc1,
                        M1x,M1y,M1z;//суммарный момент сил в ГШ отн.X/Y/Z СКЛ, Н*м

        //ВШ, СКЛ,
        vector <double> M2T,        //момент отн.ВШ силы тяги (СКЛ), Н*м
                        M2Q,        //момент отн.ВШ силы сопротивления лопасти dQ (СКЛ), Н*м
                        M2K,        //момент отн.ВШ силы Кориолиса (СКЛ), Н*м
                        M2Gx,M2Gy,  //момент отн.ВШ веса лопасти (СКЛ), Н*м
                        M2Py,M2Px,  //момент отн.ВШ силы инерции качания (СКЛ), Н*м
                        M2Cx,M2Cy,  //момент отн.ВШ центробежной силы (СКЛ), Н*м
                        M2Cz,
                        hc2,
                        Fdmp,       //сила на штоке демпфера лопасти, Н
                        M2D,        //момент отн.ВШ силы демпфера, Н*м
                        M2x,M2y,M2z;//суммарный момент в ВШ отн.X/Y/Z СКЛ, Н*м

        //ОШ, СКЛ,
        vector <double> delta_pov,
                        M3Ma,
                        M3Cz,
                        M3su,Fpov,
                        M3tr,M3dmp,
                        M3iz,
                        M3z,M3y,M3x;//суммарный момент в OШ отн.X/Y/Z СКЛ, Н*м

        //ГШ, СКПС,
        vector <double> Ty,         //проекция силы тяги лопасти на оY винта (СКПС), Н
                        H,          //продольная сила лопасти (СКПС), Н
                        S;          //поперечная сила лопасти (СКПС), Н
        //Центр втулки, СКПС,
        vector <double> MQ,         //момент сил сопротивления лопасти dQ отн.оY винта (СКПС), Н*м
                        Mxa,        //момент отн.оX винта (СКПС) аэродинамических сил, Н*м
                        Mza,        //момент отн.оZ винта (СКПС) аэродинамических сил, Н*м
                        Mxj,        //момент отн.оX винта (СКПС) инерционных сил, Н*м
                        Mzj,        //момент отн.оZ винта (СКПС) инерционных сил, Н*м
                        Fhx,Fhy,Fhz,//проекции суммарной силы в центре втулкиp (СКПС) от одной лопасти, Н
                        Mhx,Mhy,Mhz;//проекции суммарного момента в центре втулки (СКПС) от одной лопасти, Н*м
                    //Fxa_,Fya_,Fza_,//проекции силы в центре втулки (СКПС) от аэродинамических сил от одной лопасти, Н
                    //Mxa_,Mya_,Mza_;//проекции момента в центре втулки (СКПС) от аэродинамических сил от одной лопасти, Н*м

};


struct iBlades// параметры лопасти и шарниров /// для моделирования по времени ////////////////////////////
{
        //распределенные по лопасти параметры
        vector <double> alfar;   //угол атаки в сечении, градусы
        vector <double> dT_dr;   //погонная сила тяги, Н/м
        vector <double> dQ_dr;   //погонная сила сопротивления, Н/м
        vector <double> dMa_dr;  //погонный аэродинамический момент отн.ОШ, Н*м
        vector <double> fi;      //угол установки сечения, градусы
        vector <double> fl;      //угол закрутки сечения, градусы
        vector <double> Fi;      //угол притекания в сечении, градусы
        vector <double> M;       //числа Маха в сечении
        vector <double> cy;      //коэф. подъемной силы профиля
        vector <double> cx;      //коэф. профильного сопротивления
        vector <double> mz;      //коэф. крутящего момента профиля
        vector <double> Ux;      //скорость обтекания в плоскости вращения, м/с
        vector <double> Uy;      //скорость обтекания в плоскости взмаха, м/с
        vector <double> Uywx,Uywz;
        vector <double> U;       //суммарная скорость обтекания в сечении, м/с
        vector <double> lambda;  //коэф. протекания в сечении
        vector <double> dTv_dr; //
        vector <double> dXaU_dr;//
        double ni;//Tv;          //индуктивная мощность в конкретном азимуте, Вт
        double np;//XaU;         //профильная мощность в конкретном азимуте, Вт

        double sd,sd_,vd;//параметры демпфера лопасти

        //параметры движения лопасти
        double  psi0;
        double  psi,psi_,      //азимут лопасти (СКПС), грд
                    deltagama,deltabeta,deltaksi,
                xd7,           //центр давления, %
                fi7,fi7_,      //угол установки, грд
                dfi7,dfi7_,    //скорость изменения угла установки, грд/грд[об]
                d2fi7,d2fi7_,   //угл.ускорение закрутки, грд[взм]/грд2[об]
                gama,gama_,    //угол закрутки, грд
                dgama,dgama_,  //угл.скорость закрутки, грд[взм]/грд[об]
                d2gama,d2gama_,//угл.ускорение закрутки, грд[взм]/грд2[об]
                beta,beta_,    //угол взмаха, грд
                dbeta,dbeta_,  //угл.скорость взмаха, грд[взм]/грд[об]
                d2beta,d2beta_,//угл.ускорение взмаха, грд[взм]/грд2[об]
                ksi,ksi_,      //угол качания, грд
                dksi,dksi_,    //угл.скорость качания, грд[взм]/грд[об]
                d2ksi,d2ksi_,  //угл.ускорение качания, грд[взм]/грд2[об]
                khi;           //угол "на ЦТ отн.оси винта", грд

        double  Fx,Fy,Fz,  FNx,FNz;//проекции суммарной силы в шарнирах (СКЛ), Н
        double  T,          //сила тяги лопасти (СКЛ), Н
                Q,          //сила сопротивления лопасти (СКЛ), Н
                Ma,         //аэродинамический момент отн.ОШ, Н*м
                K1,         //сила Кориолиса в плоскости вращения от взмаха (СКЛ), Н      //K2 - сила Кориолиса от качания
                Nr,         //центробежная сила от вращения винта (СКЛ), Н
                Nf,         //центробежная сила от маховых движений (СКЛ), Н
                P1,         //сила инерции взмаха (СКЛ), Н
                P2,         //сила инерции качания (СКЛ), Н
                Paxz,//сила инерции в плоскости винта (от линейного перемещения вдоль продольной и поперечной осей CKПС)
                Payy,//общая cила инерции в плоскости тяги
                Pay, //cила инерции от линейного перемещения вдоль оси НВ
                Pexz,//сила инерции от вращения НВ вокруг продольной и поперечной осей CKПС
                Pkxz;//силы инерции Кориолиса, возникающие от вращения НВ вокруг продольной и поперечной осей CKПС на азимуте psi

        ///ГШ, СКЛ,
        double  M1T,        //момент отн.ГШ силы тяги (СКЛ), Н*м
                M1Q,        //момент отн.ГШ силы сопротивления лопасти dQ (СКЛ), Н*м
                M1Gx,M1Gy,  //момент отн.ГШ веса лопасти (СКЛ), Н*м
                M1K,        //момент отн.ГШ силы Кориолиса (СКЛ), Н*м
                M1Px,M1Py,  //момент отн.ГШ силы инерции взмаха (СКЛ), Н*м
                M1Cx,M1Cy,  //момент отн.ГШ центробежной силы (СКЛ), Н*м
                M1Cz,
                hc1,
                M1x,M1y,M1z;//суммарный момент сил в ГШ отн.X/Y/Z СКЛ, Н*м

        ///ВШ, СКЛ,
        double  M2T,        //момент отн.ВШ силы тяги (СКЛ), Н*м
                M2Q,        //момент отн.ВШ силы сопротивления лопасти dQ (СКЛ), Н*м
                M2Gx,M2Gy,  //момент отн.ВШ веса лопасти (СКЛ), Н*м
                M2K,        //момент отн.ВШ силы Кориолиса (СКЛ), Н*м
                M2Py,M2Px,  //момент отн.ВШ силы инерции качания (СКЛ), Н*м
                M2Cx,M2Cy,  //момент отн.ВШ центробежной силы (СКЛ), Н*м
                M2Cz,
                hc2,
                Fdmp,       //сила на штоке демпфера лопасти, Н
                M2D,        //момент отн.ВШ силы демпфера, Н*м
                M2x,M2y,M2z;//суммарный момент в ВШ отн.X/Y/Z СКЛ, Н*м

        ///ОШ, СКЛ,
        double  delta_pov,
                M3Ma,
                M3Cz,
                M3su,Fpov,
                M3tr,M3dmp,
                M3iz,
                M3z,M3y,M3x;//суммарный момент в OШ отн.X/Y/Z СКЛ, Н*м

        ///ГШ, СКПС,
        double  Ty,         //проекция силы тяги лопасти на оY винта (СКПС), Н
                H,          //продольная сила лопасти (СКПС), Н
                S;          //поперечная сила лопасти (СКПС), Н
        ///Центр втулки, СКПС,
        double  MQ,         //момент сил сопротивления лопасти dQ отн.оY винта (СКПС), Н*м
                Mxa,        //момент отн.оX винта (СКПС) аэродинамических сил, Н*м
                Mza,        //момент отн.оZ винта (СКПС) аэродинамических сил, Н*м
                Mxj,        //момент отн.оX винта (СКПС) инерционных сил, Н*м
                Mzj,        //момент отн.оZ винта (СКПС) инерционных сил, Н*м
                Fhx,Fhy,Fhz,//проекции суммарной силы в центре втулки (СКПС) от одной лопасти, Н
                Mhx,Mhy,Mhz;//проекции суммарного момента в центре втулки (СКПС) от одной лопасти, Н*м

};


class RotorBase
{
public:
        RotorBase();
        ~RotorBase();

        enum { MR, TR, PR }      ROTORID;   //идентификатор винта: 0-mainrotor, 1-tailrotor, 2-propulsiverotor
        enum {EQROTOR, FORCES}   CALCMODE;  //Режим расчета: EQROTOR - поиск параметров эквивалентного винта по потребным силам; FORCES - расчет сил по заданным параметрам управления;
        enum {AXIAL, HORIZONTAL} FLIGHTMODE;//Режим полета:  AXIAL - осевой полет; HORIZONTAL - горизонтальный полет

        void Preparation();//подготовка данных для расчета
        void ClassicSpecified(double _alpha = 0, double _fi0 = 0);//поиск сил по заданным условиям
        void ClassicForceSynth();//Расчет всех сил и моментов на винте при текущих параметрах управления
        QString Pre2Str();//текст данных для расчета
        QString Res2Str(int log);//текст результатов расчета
        vector<double> shiftAz(vector<double> T, int currBlade);
        double normAng(double angle);
        void toHubSystem(double psi, double xl, double zl, double &xh, double &zh);
        void RotorIterate(double _T, double _H, double _S, double _V, double _alpha, double _beta_sk, double _fi0, double _eta, double _chi);
        QString getAverage();
        void getIniFlow();//Расчет проекций скорости полета на оси расчетной СК


        double chord, xt, xo, xd;
        double r_pov, k_pov, stiff_su, f_pod, r_pod, sigma3;
        bool zakrut;

        ///лопасти
        Blades  b[mbl];//1fullrev
        iBlades B[mbl];//iter

        int     hubtype;       //тип втулки винта
        int     rdirect;  //направление вращения винта
        bool    damper;
        int     warning;
        double  prec;
        double  dt,
                a_x, a_y, a_z,//лин.ускорение втулки НВ
                e_x, e_y, e_z,//угл.ускорение втулки НВ
                v_x, v_y, v_z,//лин.скорость втулки НВ
                w_x, w_y, w_z;//угл.скорость втулки НВ
                //shx, shy, shz;
        QString logstr, title, model, ss,sss, outres;
        int     logkey;
        int     nAz;        //кол-во расчетных азимутов за оборот
        int     nSec;       //кол-во сечений = r.size(),
        int     krd;         //коэф. учета направления вращения: винтов левого вращения = 1, правого вращения = -1
        int     zl;         //кол-во лопастей
        int     sec07;      //номер сечения 0,7R
        int     revcount, itcount; //счетчик полных оборотов и накопитель азимутов   ob,
        double  R;          //максимальный радиус винта, м
        double  rpm;       //частота вращения винта, об/мин
        double  w;          //угловая скорость вращения винта, 1/с
        double  ml,mlp;     //масса лопасти, кг:
        double  Bkp;         //коэффициент концевых потерь <=1
        double  alpha,alpha_;//угол атаки винта, градусы
        double  r0;         //радиус комлевого сечения лопасти, м =  r.back()
        double  rB;         //радиус рабочей ? части лопасти
        double  _k;         //коэф. компенсатора взмаха
        double  b7;         //хорда в отн. сечении .7*R
        double  krutka;     //общая крутка лопасти
        double  sigma;      //коэф. заполнения винта
        double  F;          //ометаемая площадь винта
        double  Sb0;       //статический момент лопасти отн.оси винта, кг*м:
        double  Ib0;       //момент инерции лопасти отн.оси винта, кг*м^2
        double  l_osh;      //разнос OШ, м
        double  _l_gsh;    //отн. разнос ГШ, м
        double  l_gsh;     //разнос ГШ, м
        double  S_gsh;     //статический момент лопасти отн. ГШ, кг*м:
        double  I_gsh;     //момент инерции лопасти отн. ГШ, кг*м^2
        double  l_vsh;      //разнос ВШ, м
        double  s_vsh;      //
        double  S_vsh;      //статический момент лопасти отн. ВШ, кг*м:
        double  I_vsh;      //момент инерции лопасти отн. ВШ, кг*м^2
        double  I_osh,Iop;  //момент инерции лопасти отн. ОШ, кг*м^2  и погонный момент инерции отн. ОШ, кг*м^2/м
        double  Icb;        //Icb = Sb0 *(xo-xt)*chord;
        double  Jy;         //сумм.момент инерции бесшарнирных лопастей отн.оси вращения, кг*м^2
        double  wR;         //окружная скорость концов лопастей м/с
        double  dCxp;       //поправка к коэф. профильного сопротивления
        double  beta_sk;    //угол скольжения, градусы
        double  fi0_min,fi0_max;    //пределы угла ОШ
        double  rpm_min,rpm_max;
        double  ksi_min,ksi_max;//пределы угла качания
        double  bet_min,bet_max;//пределы угла взмаха
        double  gam_min,gam_max;//пределы угла закрутки
        double  ksi_nom;  //
        double   V;         //скорость полета, м/с
        double  _V;         //отн.скорость полета = (V / wR)
        double  _Vx;        //проекция отн. скорости полета на ось OXh
        double  _Vy;        //проекция отн. скорости полета на ось OYh
        double  _Vz;        //проекция отн. скорости полета на ось OZh
        double  ro;         //плотность воздуха, кг/м^3
        double  p0,t0;      //давление и температура на уровне моря
        double  sV;         //скорость звука, м/с
        double  g;          //ускорение свободного падения м/с/с
        double  delta;      //угол наклона равнодействующей силы, градусы
        double  delta_need; //поребный угол наклона равнодействующей силы, градусы
        double  mu;         //коэф. режима работы винта
        double  f1, f2;     //коэфф. в формуле индуктивной скорости
        double  d2r,r2d;    //коэф. перевода градусов в радианы и наоборот
        double  KA1, KB1, KC1;//Коэф. уравнения равновесия лопасти отн.ГШ
        double  KA2, KB2, KE; //Коэф. уравнения равновесия лопасти отн.ВШ
        double  dpsi;       //шаг по азимуту, радианы
        double  lambda_sr;  //ср. коэф. протекания
        double  K_mu;       //
        double  Hy;         //
        int  rpmORfi0; //определение частоты вращения или угла ОШ для ЭВ
        double  fi0;        //общий шаг, градусы
        double  Xa_need;    //потребная пропульсивная сила, Н
        double  Ya_need;    //потребная подъемная сила, Н
        double  Xa;         //пропульсивная сила, Н
        double  Ya;         //подъемная сила, Н
        double  Vx,Vy,Vz;   //проекции скорости полета на оси OXh,OYh,OZh
        double  a0, a1, b1, a2, b2;

        double  Fbs, Fbsxr, Fbszr, Fbsx, Fbsz, dctbs,act, ct1bx,ct1bz, ctMRxr,ctMRzr, ctMRx,ctMRz;
        //от всех лопастей на текущем азимуте
        double  Fhy_sum, Fhx_sum, Fhz_sum, Mhy_sum, Mhx_sum, Mhz_sum;
        double  Fhx2sum, FxhQ, FxhT, FxhNr, FxhNf, FxhP1, FxhK1, FxhP2;
        double  Fhz2sum, FzhQ, FzhT, FzhNr, FzhNf, FzhP1, FzhK1, FzhP2;
        //от всех лопастей на текущем азимуте при полном обороте винта!
        vector <double> Fhx_S, Fhy_S, Fhz_S;//проекции сумм.силы в центре втулки (СКПС) от всех лопастей, Н
        vector <double> Mhx_S, Mhy_S, Mhz_S;//проекции сумм.момента в центре втулки (СКПС) от всех лопастей, Н*м
        vector <double> xhQ, zhQ, xhT, zhT, xhNr, zhNr, xhNf, zhNf, xhP1, zhP1, xhP2, zhP2, xhK1, zhK1, xhK2, zhK2, xhSum, zhSum;

        //средние за оборот от всех лопастей на втулке от аэродин.сил, СКПС
        bool   nextrev; //сигнал нового оборота
        double ksi_sr;
        double T_sr;    //средняя за оборот аэродинамическая тяга винта (СКПС), Н
        double H_sr;    //средняя за оборот аэродинамическая продольная сила винта (СКПС), Н
        double S_sr;    //средняя за оборот аэродинамическая поперечная сила винта (СКПС), Н
        double My_sr;   //средний за оборот крутящий момент на валу винта (СКПС), Н*м
        double Mz_sr;   //средний за оборот суммарный момент отн.оZ винта (СКПС), Н*м
        double Mx_sr;   //средний за оборот суммарный момент отн.оX винта (СКПС), Н*м
        double Mzasr;   //средний за оборот момент аэродин.сил отн.оZ винта (СКПС), Н*м
        double Mxasr;   //средний за оборот момент аэродин.сил отн.оX винта (СКПС), Н*м
        double Mzjsr;   //средний за оборот момент инерционных сил отн.оZ винта (СКПС), Н*м
        double Mxjsr;   //средний за оборот момент инерционных сил отн.оX винта (СКПС), Н*м
        double Mgyrx;   //
        double Mgyrz;   //

        double Fhx_sr, Fhy_sr, Fhz_sr;//проекции средней за оборот сумм.силы в ц.втулки (СКПС) от всех лопастей, Н
        double Mhx_sr, Mhy_sr, Mhz_sr;//проекции среднего за оборот сумм.момента в ц.втулки (СКПС) от всех лопастей, Н*м
        double xhSum_sr,zhSum_sr,  //Fbsx_sr,Fbsz_sr,
         psi_med,  psimin,  psimax,
         fi7_med,  fi7min,  fi7max,
        gama_med,gama_min,gama_max,
        beta_med,beta_min,beta_max,
         ksi_med,  ksimin,  ksimax,
           T_med,   T_min,   T_max,
           Q_med,   Q_min,   Q_max,
          Ma_med,  Ma_min,  Ma_max,
           K_med,   K_min,   K_max,
          Nr_med,  Nr_min,  Nr_max,
          Nf_med,  Nf_min,  Nf_max,
          P1_med,  P1_min,  P1_max,
          P2_med,  P2_min,  P2_max,
          Fy_med,  Fy_min,  Fy_max,
          Fx_med,  Fx_min,  Fx_max,
          Fz_med,  Fz_min,  Fz_max,
        Fpov_med,Fpov_min,Fpov_max,
        Fdmp_med,Fdmp_min,Fdmp_max;

        double N_need;      //потребная мощность, кВт
        double N_i;         //индуктивная мощность, кВт
        double N_p;         //профильная мощность, кВт
        double N_vr;        //мощность вредного сопротивления, кВт
        double m_i;         //коэф. индуктивной мощности
        double m_p;         //коэф. профильной мощности
        double m_k;         //коэф. крутящего момента
        double Ct,Ch,Cs;          //коэф. тяги,,
        double Ct_test, Ch_test;
        double ty;          //коэф. подъемной силы винта в полусвязанной СК
        double tx;          //коэф. пропульсивной силы винта в полусвязанной СК
        double del_xy;      //погрешность определения результирующей силы винта
        double del_Y;       //погрешность определения подъемной силы винта
        double del_X;       //погрешность определения пропульсивной силы винта
        double vt;          //осевая индуктивная скорость, м/с
        double vh;          //продольная индуктивная скорость, м/с
        double vs;          //боковая индуктивная скорость, м/с
        double R_need;      //потребная результирующая сила, Н
        double Ra;          //результирующая сила, Н
        double Rxy;         //результирующая сила в продольной плоскости, Н
        double T_i;         //тяга винта, исп. в расчете инд.скорости, Н
        double H_i;         //продольная сила, исп. в расчете инд.скорости, Н
        double S_i;         //поперечная сила, исп. в расчете инд.скорости, Н

        double  V_sr    ,        Fbsx_sr ,        Fbsz_sr ,
                psi_sr  ,        bT_sr   ,        bQ_sr   ,        bK1_sr  ,
                bNr_sr  ,        bNf_sr  ,        bP1_sr  ,        bP2_sr  ,
                bF1y_sr ,        bF2x_sr ,        bF3z_sr ,        bFdmp_sr,
                alfa_sr ,        fi0_sr  ,        eta_sr  ,        chi_sr  ,
                ni_sum  ,        np_sum  ;

        //демпфер лопасти
        struct bladedampher
        {
         double h    ,//плечо силы на демпфере
                x2   ,//коорд.шарнира демпфера на втулке отн.оси НВ
                z2   ,//коорд.шарнира демпфера на втулке отн.оси НВ
                x3   ,//коорд.шарнира демпфера на лопасти отн.оси НВ
                z3   ,//коорд.шарнира демпфера на лопасти отн.оси НВ
                fik  ,//корректировка угла fi "к отсчету от вертикали"
                A_,C_,D_,//геометрические коэф.демпфера (как кулисы)
                pmax ,//давление срабатывания клапана демпфера
                sigma,sigma2,//
                F    ,//активная площадь поршня демпфера
                dn   ,//
                dv   ,//
                //fimax,fimin,
                f,f2,
                mu,ro;
        } d;

        QString profiles[5];//имя профиля
        int       nA[5][3], //колво углов атаки: 3 - кол-во профилей; 2 - для Су, Сх, Mz
                  nM[5][3]; //колво Чисел Маха: 3 - кол-во профилей; 2 - для Су, Сх, Mz
        double ALPHA[5][Ca][3], //угол атаки: 3- кол-во профилей; Са - кол-во углов; 2 - для Су, Сх, Mz
                MACH[5][CM][3],
                 _Cy[5][CM][Ca],
                 _Cx[5][CM][Ca],
                 _Mz[5][CM][Ca],
             _Cx_Cy0[5][CM], //Сх при Су=0
              _a_Cy0[5][CM]; //углы атаки при Сy=0

        vector <float>   r_is;//исходный массив радиусов сечений, м
        vector <float>  bh_is;//исходный массив хорд сечений, м
        vector <float> dfi_is;//исходный массив отн. крутки сечений, градусы
        vector <int>  prof_is;//исходный массив ##профилей сечений
        vector <double>     r;//массив радиусов сечений, м  r_is + концевые потери(B*R)
        vector <double>    bh;//массив хорд сечений, м
        vector <double>   dfi;//массив отн. крутки сечений, градусы
        vector <int>     prof;//массив ##профилей сечений
        vector <double> _r;  // массив отн. радиусов сечений = r/R
        vector <double> psi0;//массив расчетных азимутов, градусы
        double  psih, psih_;  //расчетный азимут втулки  и приращение, градусы

        //MR
        double D1; //передаточное отношение
        double D2; //передаточное отношение
        double chi; //угол отклонения АП в продольном канале, градусы
        double eta; //угол отклонения АП в поперечном канале, градусы
        double alpha_min; //минимальный угол атаки винта
        double alpha_max; //максимальный угол атаки винта
        //TR
        double T_need; //потребная тяга, Н
        //
        //
        TValsShowHide vsh, vsh1;
        //
        Flutter_IsolatedBladeNoFriction_Data_v1 fltrModel;
        Flutter_IsolatedBladeNoFriction_Data_v1_ParamsIniForCalcEquations fltrEquationsParams;
        IncomingParamsFlutter dataIncomingFlutter;
        BladeParamsSupplFlutter dataBladeSupplFlutter;
        HelicParamsSupplFlutter dataHelicSupplFlutter;
        FlutterModeParams dataFlutterMode;
        //void SetIncomingParams(const IncomingParamsFlutter&data);
        //void SetBladeSupplData(const BladeParamsSupplFlutter&data);
        //void SetHelicSupplData(const HelicParamsSupplFlutter&data);
        //void SetFlutterModeParams(const FlutterModeParams&data);
        //
        double AzimuthDuration;
        std::vector<double>flKynDevParams;
        //
        double get__xFocus(int profN, int M);
        //
        int CalcCalls1;//f' kyn_az
        int CalcCalls2;//f' flutter fun
        //

protected:
        virtual void Core() = 0;//Расчет интегральных сил и маховых движений при текущих значениях alpha и fi0 винта
        void generalCore();
        double get_dRa(double _fi0);
        void getHingeForces();
        void blade_kinematic_dt(int l);//Расчет параметров движения лопасти
        //void blade_kinematic_az(int n, bool krut);//commented 25.11.2024
        void blade_kinematic_az(int n, int krutMode);//changed 25.11.2024
        double get_d2beta_hinge(double psi, double beta, double figa, double d2figa, double Int_MdT);//расчет ускорения взмаха для типа "шарнир"
        double get_d2beta_kardan(double psi, double beta, double figa, double d2figa, double Int_MdT1, double Int_MdT2);//расчет ускорения взмаха для типа "кардан"
        double get_d2ksi_hinge(double psi, double beta, double dbeta, double ksi, double dksi);//расчет ускорения качания
        double get_d2ksi_kardan(double psi, double beta, double ksi, double Int_MdQ1, double Int_MdQ2);
        double get_d2gama(double beta, double d2beta, double figa, double d2fi, double gama, double dgama, double dfi, double Int_MdMa);
        double gamaDamph(double dgama);
        double f_xf(double M);
        void getInducedFlow();//Расчет индуктивной скорости в зависимости от T_i, H_i, S_i
        double get_fi0();//Расчет общего шага, исходя из угла атаки и потребной результирующей силы
        double get_rpm();
        double get_dTsr(double _rpm);
        double get_f2(double mu);//Определение коэф. f2 по коэф. режима работы mu
        segment Segment(int i, double psi, double beta, double dbeta, double gama, double dgama, double dfi7);//расчет массива параметров
        double get_Ux(double _r, double psi, double beta);//расчет проекции скорости обтекания сечения на его ось X
        double get_Uy(double _r, double lambda ,double psi, double beta, double dbeta);//расчет проекции скорости обтекания сечения на его ось Y
        double get_lambda(double _r, double psi);//расчет коэф. протекания в сечении
        double get_Fi(double Ux, double Uy);//расчет угла притекания в сечении
        double get_fi(int i, double psi, double beta);//расчет шага лопасти в сечении
        double get_alfa_r(double fi, double Fi);//расчет угла атаки в сечении
        double get_M(double U);//расчет числа Маха
        double get_Cy(int prof, double M, double alpha);//расчет коэф. подъемной силы Cy в сечении
        double get_Cx(int prof, double M, double alpha);//расчет коэф. профильного сопротивления Cxp в сечении
        double get_Cxp0(int prof, double M);//расчет коэф. профильного сопротивления Cxp в сечении при (Су = 0)
        double get_Mz(int prof, double M, double alpha);//расчет коэф. mz
        double get_Integral(vector<double> T,vector<double> r);//интегрирование трапециями
        double get_IntegralMdT1(double psi, double beta, double dbeta, double gama, double dgama, double dfi7);//расчет момента тяги лопасти отн. ГШ
        double get_IntegralMdT2(double psi, double beta, double dbeta, double gama, double dgama, double dfi7);//расчет момента тяги лопасти отн. ВШ
        double get_IntegralMdQ1(double psi, double beta, double dbeta, double gama, double dgama, double dfi7);//расчет момента сопр.лопасти отн. ГШ
        double get_IntegralMdQ2(double psi, double beta, double dbeta, double gama, double dgama, double dfi7);//расчет момента сопр.лопасти отн. ВШ
        double get_IntegralMdMa(double psi, double beta, double dbeta, double gama, double dgama, double dfi7);//расчет момента аэродин.момента отн.ОШ
        //double get_IntegralMdT3(double psi, double beta, double dbeta, double gama, double dgama, double dfi7);//расчет момента аэродин.силы  отн.ОШ
        double DampherSpeed(double ksi, double dksi);
        double DampherForce(double vd);
        double get_max_deviation(vector<double> v1, vector<double> v2);//определение макс. погрешности между углами взмаха за два оборота
        double _getCy(int profil, double Mach, double angle);
        double _getCx(int profil, double Mach, double angle);
        double _getMz(int profil, double Mach, double angle);
        double _getAlfa(int profil, double Mach, double Cy);
        double _getCxCy0(int profil, double Mach);
        void getCxCy0Array();
        void checkVectors();//проверяет и выравнивает длину векторов  r_is, b_is, dfi_is, prof_is
        void get_minmax(vector<double> V, double med, double &min, double &max);
        double KatToAng(double pro, double pri);

        vector <int>    iter;   //номер итерации
        vector <double> t;      //
        vector <double> Psi;    //вектор азимутов, градусы
        vector <double> Fi7;    //вектор углов , градусы
        vector <double> dFi7;   //вектор скоростей , грд[взмаха]/грд[об]
        vector <double> d2Fi7;  //вектор
        vector <double> Beta;   //вектор углов взмаха, градусы
        vector <double> dBeta;  //вектор скоростей взмаха, грд[взмаха]/грд[об]
        vector <double> d2Beta; //вектор ускорений взмаха, грд[взмаха]/грд[об]/грд[об]
        vector <double> Ksi;    //вектор углов качания, градусы
        vector <double> dKsi;   //вектор скоростей качания, грд[взмаха]/грд[об]
        vector <double> d2Ksi;  //вектор ускорений качания, грд[взмаха]/грд[об]/грд[об]
        vector <double> Gama;   //вектор углов закрутки, градусы
        vector <double> dGama;  //вектор скоростей закрутки, грд[взмаха]/грд[об]
        vector <double> d2Gama; //вектор ускорений закрутки, грд[взмаха]/грд[об]/грд[об]

        vector <double> deltaBeta,deltabeta; //вектор погрешности при итерационном определении углов,
        vector <double> deltaGama,deltagama; //вектор погрешности при итерационном определении углов,
        vector <double> deltaKsi, deltaksi;  //вектор погрешности при итерационном определении углов,

        vector <double> dMQ_dr1;    //погонный момент силы сопротивления отн.оси винта, Н/м
        vector <double> dM_y1_Q_dr1;//погонный момент силы сопротивления отн.ГШ, Н/м
        vector <double> dTv_dr1;    //погонная индуктивная мощность, Вт/м
        vector <double> dXaU_dr1;   //погонная профильная мощность, Вт/м
        vector <vector <double>> dMQ_dr;    //погонный момент силы сопротивления отн.оси винта, Н/м
        vector <vector <double>> dM_y1_Q_dr;//погонный момент силы сопротивления отн.ГШ, Н/м
        vector <vector <double>> dTv_dr;    //погонная индуктивная мощность, Вт/м
        vector <vector <double>> dXaU_dr;   //погонная профильная мощность, Вт/м

        segment seg;//
        double dpsih;

        double _D1;
        double _D2;
        double Theta1; //угол установки лопасти отн. КПВ в азимуте 270
        double Theta2; //угол установки лопасти отн. КПВ в азимуте 180
        double del_Ra; //погешность расчета по результирующей силе, %
        double del_Ya; //погрешность по подъемной силе, %
        double del_Xa; //погрешность по пропульсивной силе, %
        double del_T; //погрешность расчета по тяге, %
};


class MainRotor : public RotorBase
{
public:
        MainRotor();
        void Preparation();//подготовка данных для расчета
        void ClassicEquivalent(double _Xa = 0, double _Ya = 0, double _eta = 0, double _chi = 0);//поиск параметров эквивалентного винта
        void ClassicSpecified(double _alpha = 0, double _fi0 = 0, double _eta = 0, double _chi = 0);//поиск сил по заданным условиям
        void ClassicForceSynth();//Расчет всех сил и ммоментов на винте при текущих параметрах управления
        QString Pre2Str(int log);//текст данных для расчета
        QString Res2Str(int log);//текст результатов расчета
        void fillOutAz(outstructure &out, int az);
        void fillOut(outstructure &out);
        double alpha_min, alpha_max;

protected:
        void Core();//Расчет интегральных сил и маховых движений при текущих значениях alpha и fi0 винта
        double get_dDelta(double _alpha);//Расчет отличия между текущим угом наклона равнодействующей delta и потребным углом
};


class TailRotor : public RotorBase
{
public:
        TailRotor();
        void Preparation();//подготовка данных для расчета
        void ClassicEquivalent(double _alpha = 0, double _T = 0);//поиск параметров эквивалентного винта (fi0)
        void ClassicForceSynth();//Расчет всех сил и моментов на винте при текущих параметрах управления
        QString Pre2Str(int log);//текст данных для расчета
        QString Res2Str(int log);//текст результатов расчета
        void fillOutAz(outstructure &out, int az);
        void fillOut(outstructure &out);

protected:
        void Core();//Расчет интегральных сил и маховых движений при текущих значениях alpha и fi0 винта
};


class PushRotor : public RotorBase
{
public:
        PushRotor();
        void Preparation();//подготовка данных для расчета
        void ClassicEquivalent(double _alpha = 0, double _T = 0);//поиск параметров эквивалентного винта (fi0)
        void ClassicForceSynth();//Расчет всех сил и моментов на винте при текущих параметрах управления
        QString Pre2Str(int log);//текст данных для расчета
        QString Res2Str(int log);//текст результатов расчета
        void fillOutAz(outstructure &out, int az);
        void fillOut(outstructure &out);

protected:
        void Core();//Расчет интегральных сил и маховых движений при текущих значениях alpha и fi0 винта
};


#endif


