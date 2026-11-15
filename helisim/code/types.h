#ifndef types_H
#define types_H
#include <QVector>
#define  mst  5  //макс.кол.амортстоек
#define  mbl  6  //макс.кол.лопастей винтов
#define  mass1  111 //101
#define  mass2  10001
#define  Ca   50
#define  CM   20

using namespace std;

struct duo
{
    double a,b;
};

struct tre
{
    double a,b,c;
};

struct quat
{
    double a,b,c,d;
};

struct pent
{
    double a,b,c,d,e;
};

struct six
{
    double a,b,c,d,e,f;
};

struct rparams
{
    double a,b,c,d;
};

struct forcemoment
{
    double Fx,Fy,Fz, Mx,My,Mz;
};

struct xyzkrt
{
    double x,y,z, k,r,t;
};

struct floatxyz
{
    float x,y,z;
};

struct xyz
{
    double x, y, z;
};

struct extloadstruct//внешняя нагрузка
{
    int i;
    float t, Fx,Fy,Fz;//, Mx,My,Mz;
};

struct ManualControl
{
    double fi0mr;
    double etamr;
    double chimr;
    //double T_tr;
    double fi0tr;
    double fi0pr;
};

struct instructure//структура считанных исходных данных
{
    QString datafolder, reportfolder;
    int     logkey;
    int     steps, start, logstep;
    int     calcmode,   //тип расчета
            makebalance,//выполнять балансировку
            nAz,        //число расчетных шагов по азимуту (в отдельных расчетах винтов)
            nstabrev;   //количество оборотов стабилизации винта
    bool    fit;        //выполнять подгонку расчета в некоторых функциях
    bool    autorot;    //указатель режима авторотации
    bool    autogyro;   //указатель autogyro
    //const
    float   g, t0, p0, P_p, ro, Lpt, RjmK, RjkgK, Mm;
    float   dt,
            bpa, kra, krp, rotprec,
            snlf6prc, snlf6lim, snlf2prc, snlf2lim,
            precXan,  precYan,  precChi,  precEta,
            //ПАРАМЕТРЫ ВЕРТОЛЕТА
            Hx,  Hy,  Hz,  kren_,rysk_,tang_,//начальное положение
            vHx, vHy, vHz, wHx, wHy, wHz,   //начальная скорость
            mH,
            Ix, Iy, Iz,
            Iyz,Ixz,Ixy,
            xctmr,yctmr,zctmr,//центровка относительно втулки НВ
            x_f,  y_f,  z_f, //характерная точка приложения аэродинамических сил на фюзеляже
            lmodel, Smodel, Mmodel,//Продувочная модель
            //СТАБИЛИЗАТОР
            x_st, y_st, z_st,
            est_min, est_max, fi0_min, fi0_max,//пределы установки стабилизатора
            S_st, l_st, K_st, //площадь и коэф.торможения перед стабилизатором
            //КИЛЬ
            x_k,  y_k,  z_k,
            S_keel, K_keel, eps_keel,
            //КРЫЛО
            x_w,  y_w,  z_w,
            S_wing, l_wing, K_wing, eps_wing,
            //ГРУЗ НА ВНЕШНЕЙ ТРОСОВОЙ ПОДВЕСКЕ
            xcg, ycg, zcg, mcg, lcg;
            //
    //учитывать аэродинамические нагрузки
    int     solvePlaner, solveStab, solveKeel, solveWing;
    float   Fx_f,   Fy_f,   Fz_f,   Mx_f,   My_f,   Mz_f,
            Fx_st,  Fy_st,  Fz_st,  Mx_st,  My_st,  Mz_st,
            Fx_keel,Fy_keel,Fz_keel,Mx_keel,My_keel,Mz_keel,
            Fx_wing,Fy_wing,Fz_wing,Mx_wing,My_wing,Mz_wing;

    //КЛЮЧИ ДЛЯ ОГРАНИЧЕНИЯ ПЕРЕМЕЩЕНИЙ ПО ОСНОВНЫМ НАПРАВЛЕНИЯМ
    int     fix_org_x, fix_org_y, fix_org_z,
            fix_org_k, fix_org_r, fix_org_t;

    //ПОСАДОЧНАЯ ПОВЕРХНОСТЬ по отклонениям от номинальной плоскости под колесами вертолета
    float   ground[mst];

    //ПАРАМЕТРЫ ИЗ КЛАСИЧЕСКОЙ ТЕОРИИ
    float   tetra,
            beta_,
            dchi,//отклонение тарелки АП по продольному каналу от расчитаного в аэродин., рад
            deta,//отклонение тарелки АП по поперечному каналу от расчитаного в аэродин., рад
            kren;//угол крена вертолета, рад (kren=gama)


    //ВИНТ НЕСУЩИЙ
    bool mrsolve;//ключ для включения НВ в расчет
    bool mrtilt;//ключ для включения в расчет наклона втулки НВ назад и влево
    //float eta_hub,chi_hub;
    float kh_mr;
    struct mainrotor_in //СТРУКТУРА ДЛЯ НВ
    {
        int hubtype;//тип втулки винта
        int rdirect;//направление вращения винта
        int     rpmORfi0;//определение частоты вращения или угла ОШ для ЭВ
        bool    vgsolve;//ключ для включения в расчет виброгасителя
        bool    damper; //ключ для включения в расчет демпфер лопасти
        float   Ox, Oy, Oz,
                Fx_,Fy_,Fz_,
                Mx_,My_,Mz_;
        float   ax,az;//ускорение внешнего воздействия (на одной итерации ihit), м/с2
        float   vs_, skos;//для расчета планера
        int     zl;
        float   D,  //диаметр несущего винта, мм;
                rpm,//частота вращения НВ, об/мин
                mlp,//масса лопасти НВ
                Sb0, Ib0, Iop,
                l_osh,//разнос OШ
                s_vsh,//смещение ВШ
                l_vsh,//разнос ВШ
                l_gsh;//
        float   alfa_min,alfa_max,
                fi0_min, fi0_max,
                rpm_min, rpm_max,
                chi_min, chi_max,
                eta_min, eta_max,
                bet_min, bet_max,
                ksi_min, ksi_max,
                Xa,Ya,
                chi_,
                eta_,
                fi0_,
                Bkp, dCxp;
        //for flatter
        float xt, xo, k_pov, r_pov, sigma3, f_pod, r_pod;

        //АВТОМАТ ПЕРЕКОСА
        float   kvz, Rap, ll, dgsh, dpsi_cont,  D1_, D2_;

        struct bdampher//демпфер лопасти
        {
          float h   ,//плечо силы на демпфере
                x2  ,//коорд.шарнира демпфера на втулке отн.оси НВ
                z2  ,//коорд.шарнира демпфера на втулке отн.оси НВ
                x3  ,//коорд.шарнира демпфера на лопасти отн.оси НВ
                z3  ,//коорд.шарнира демпфера на лопасти отн.оси НВ
                pmax,//давление срабатывания клапана демпфера
                dn  ,//диаметр цилиндра демпфера внутренний, мм:
                dv  ,//диаметр штока демпфера, мм:
                fimax,fimin,//предельные углы поворота лопасти отн.ВШ, рад;
                f,f2,//суммарная площадь отверстий в демпфере, мм^2:
                mu  ,//коэф. истечения жидкости в демпфере
                ro  ;//плотность жидкости в демпфере, кг/м^3:
        }d;

        struct vibrogas//виброгаситель
        {
            int n;//количество маятников виброгасителя
          float m,//масса 1-го маятника
                h,//расстояние от оси вала НВ до центра масс маятника
                d,//диаметр роликов
                D;//диаметр сверлений
        }vg;

        struct helistand//стенд
        {
          float m, //масса
                mu,//коэф. истечения жидкости в демпфере
                ro,//плотность жидкости в демпфере, кг/м^3:
                fx,//суммарная площадь отверстий в демпфере стенда, мм^2;
                dnx,//диаметр цилиндра демпфера стенда внутренний, мм;
                dvx,//диаметр штока демпфера стенда, мм
                dnz,//диаметр цилиндра демпфера стенда внутренний, мм;
                dvz,//диаметр штока демпфера стенда, мм
                fz,//суммарная площадь отверстий в демпфере стенда, мм^2;
                kx, kz; //жесткость пружины стенда, Н/м;
        }s;

    } mr;


    //ВИНТ РУЛЕВОЙ
    bool TRmount;
    bool trsolve;//ключ для включения PВ в расчет
    float kh_tr;
    struct tailrotor_in //СТРУКТУРА ДЛЯ PВ
    {
        int zl; //кол-во лопастей
        int hubtype;//тип втулки винта
        int rdirect;//направление вращения: 0-левое 1-правое
        int rpmORfi0;//определение частоты вращения или угла ОШ для ЭВ
       bool damper;
      float D,
            rpm,//частота вращения винта, об/мин
            mlp,
            Iop,    Ib0,    Sb0,
            l_gsh,  l_vsh,  l_osh,
            s_vsh,//смещение ВШ
            kvz,
            dCxp,
            Bkp,
            alfa_, vt_, skos,
            fi0_,
            fi0_min, fi0_max,
            rpm_min, rpm_max,
            bet_min, bet_max,
            ksi_min, ksi_max,
            Ox, Oy, Oz,
            Fx_,Fy_,Fz_,
            Mx_,My_,Mz_;
      //for flatter
      float xt, xo, k_pov, r_pov, sigma3, f_pod, r_pod;
    } tr;


    //ВИНТ толкающий
    bool PRmount;
    bool prsolve;//ключ для включения TВ в расчет
    float kh_pr;
    struct pushrotor_in //СТРУКТУРА ДЛЯ TВ
    {
        int zl; //кол-во лопастей
        int hubtype;//тип втулки винта
        int rdirect;//направление вращения: 0-левое 1-правое
        int rpmORfi0;//определение частоты вращения или угла ОШ для ЭВ
       bool damper;
      float D,
            rpm,//частота вращения винта, об/мин
            mlp,
            Iop,    Ib0,    Sb0,
            l_gsh,  l_vsh,  l_osh,
            s_vsh,//смещение ВШ
            kvz,
            dCxp,
            Bkp,
            Kt,
            alfa_, vt_,
            fi0_,
            fi0_min, fi0_max,
            rpm_min, rpm_max,
            bet_min, bet_max,
            ksi_min, ksi_max,
            Ox, Oy, Oz,
            Fx_,Fy_,Fz_,
            Mx_,My_,Mz_;
      //for flatter
      float xt, xo, k_pov, r_pov, sigma3, f_pod, r_pod;
    } pr;


    //СТРУКТУРА ДЛЯ РАСЧЕТА ШАССИ
    struct shassi_in//
    {
        int     nst,  //количество стоек
                nwh;  //количество колес в стойке
        int     stype,//тип шасси: 1 - , 2 -
                atype,//тип амортизатора ( 1 - однокамерный, 2 - две раздельные газ.камеры, 3 - общая газ.камера на два штока );
                ptype;//тип пневматика
       floatxyz O_,    //т.O - начало локальной системы координат в СКС вертолета
                A_, B_, W_, V_, Q_;
        float   lambda,
                ang_wh,// отклонение плоскости колеса
                //ground,//Y-отклонение посад.поверхности
                    gama,// ?
                whd;//половина расстояния между колесами
        struct  shassi_in_pneumatic
        {
            float
            dpmd        ,//  Максимально допустимое обжатие, мм;
            cpx,cpy,cpz ,//  Жесткость пневматика, Н/мм; (393000) 424038
            alfanp      ,//  коэффициент нелинейности диаграммы обжатия пневматика (0.025)
            kdp         ,//  коэффициент демпфирования пневматика; (0.02)
            D,B         ,//  Диаметр&Ширина пневматика, мм;
            f0,fs,fk    ;//коэф.трения
            int isbrake ;//заторможенность колеса
        } pn;
        struct  shassi_in_absorber
        {
            float
            p0   ,//давление газа в разжатом состоянии, кГ/cм^2;
            Vsum, Voil,  //V0 -объем газовой полости в разжатом состоянии, л(дм^3);
            ro   ,//Плотность жидкости, кг/м^3;
            mu   ,//Коэффициент истечения; (0.65...0.75)
            kpoly,kpolyr;//Показатель политропы;
            //kci  ;//Коэффициент трения в буксах и уплотнениях;
            struct  in_absorber_camera
            {
                float
                p0   ,//давление газа в разжатом состоянии, кГ/cм^2;
                Vsum, Voil,  //V0 -объем газовой полости в разжатом состоянии, л(дм^3);
                s  ,//Максимальный ход штока, мм:
                Dc ,//Наружный диаметр буксы - внутренний диаметр цилиндра, мм:
                Dv ,//Внутренний диаметр штока - диаметр плунжера, мм:
                Ds ,//Наружный диаметр штока, мм:
                ktr,//Коэффициент трения в буксах и уплотнениях;
                kdf,//коэффициент экв.увеличения сечения дросселя (0...30)
                fb,fbr,//Суммарная площадь отверстий в буксе, мм^2: 27*3mm
                f  ,//Суммарная площадь отверстий в штоке(плунжере) (прямой ход), мм^2: 1mm+1.5mm
                fr ;//Суммарная площадь отверстий в штоке(плунжере) (обратный ход), мм^2: 1mm
                vector <float> v_h,v_f,v_fr;
            } c1,c2;//c1 - шток демпфера, //c2 - основной шток

            //atype=0 (нелинейная пружина)
            float hu,
                  Ke,
                  Ki,
                  Ci;
        } ab;
    } st[3];//0-fore,1-rear,2-tail


    //ВНЕШНЯЯ НАГРУЗКА
    bool extloads;
    xyz  load1pos,load2pos;
    QVector <extloadstruct> load1, load2;

    bool isPointSelected;
    floatxyz selpoint;

    //readed aidyncharacter of helicopters fuselage
    double _alfa_f[mass1];        int _alfa_f_col;
    double _cx_f[mass1];          int _cx_f_col;
    double _cy_f[mass1];          int _cy_f_col;
    double _cz_f[mass1];          int _cz_f_col;
    double _mx_f[mass1];          int _mx_f_col;
    double _my_f[mass1];          int _my_f_col;
    double _mz_f[mass1];          int _mz_f_col;
    double _mz_fst[mass2];        int _mz_fst_row, _mz_fst_col;
    double __alfa_f[mass1];       int __alfa_f_col;
    double __eps_st[mass1];       int __eps_st_col;
    double __cx_st[mass2];        int __cx_st_row, __cx_st_col;
    double __cy_st[mass2];        int __cy_st_row, __cy_st_col;
    double __mz_st[mass2];        int __mz_st_row, __mz_st_col;
    double _Vvol_mr[mass1];       int _Vvol_mr_col;
    double _delta_mr[mass1];      int _delta_mr_col;
    double _uvol_mr[mass2];       int _uvol_mr_row, _uvol_mr_col;
    double _Ksi3n_[mass1];        int _Ksi3n_col;
    double _h_[mass1];            int _h_col;
    double __eps_st0[mass1];
    double _dalf_stf[mass1];    int _dalf_stf_col;

    double _cx_st[mass1];       int       _cx_st_col;
    double _cy_st[mass1];       int       _cy_st_col;
    double _alfa_st[mass1];     int     _alfa_st_col;
    double _alfa_fs[mass1];     int     _alfa_fs_col;
    double _dalfa_stf[mass1];   int   _dalfa_stf_col;

    int    _alfa_wing_col,      _cx_wing_col,   _cy_wing_col;
    double _alfa_wing[mass1],   _cx_wing[mass1],_cy_wing[mass1];
    float  _dalf_wingf;
    int    _alfa_keel_col,      _cx_keel_col,   _cy_keel_col;
    double _alfa_keel[mass1],   _cx_keel[mass1],_cy_keel[mass1];


    //blades
    vector <float> r_mr;
    vector <float> bh_mr;
    vector <float> dfi_mr;
    vector <int> prof_mr;

    vector <float> r_tr;
    vector <float> bh_tr;
    vector <float> dfi_tr;
    vector <int> prof_tr;

    vector <float> r_pr;
    vector <float> bh_pr;
    vector <float> dfi_pr;
    vector <int> prof_pr;

    //sections
    int ProfilesAmount;//количество профилей для считывания
    QString profiles[5];
    int      nA[5][3], //колво углов атаки: 5 - кол-во профилей; 2 - для Су, Сх, Mz
             nM[5][3]; //колво Чисел Маха: 5 - кол-во профилей; 2 - для Су, Сх, Mz
    float ALPHA[5][Ca][3], //угол атаки: 5 - кол-во профилей; Са - кол-во углов; 2 - для Су, Сх, Mz
           MACH[5][CM][3],
            _Cy[5][CM][Ca],
            _Cx[5][CM][Ca],
            _Mz[5][CM][Ca],
        _Cx_Cy0[5][CM], //Сх при Су=0
         _a_Cy0[5][CM]; //углы атаки при Сy=0
};

struct outstructure//структура данных результатов расчета i-й итерации
{
    double t;
    int iter;
    int  nst;
    bool isLanding;

    struct copter_out
    {
         xyzkrt org; //
       floatxyz ocw,
                vn,wn,an,en,//скорость  ЦТ вертолета в нормальной СК
                v, w, a, e, //скорость ускорение ЦТ вертолета в СКС
                n,          //перегрузка в начале СКС
                Fadp,Madp, fus_F,fus_M, stab_F,stab_M, keel_F,keel_M, wing_F,wing_M,//airdyn
                Fmr, Mmr, //rotor
                Ftr, Mtr, //rotor
                Fpr, Mpr, //rotor
                Fhg, Mhg, //gravity
                Fcg, Mcg, //cargovtp
                Fsh, Msh, //shassi
                F1,  F1pos,//external load 1
                F2,  F2pos,//external load 2
                Fsum,Msum;//сумма сил/моментов в СКС
       floatxyz fus, stab, keel;
         double xct, yct, zct;//положение центра тяжести вертолета в СКС
       floatxyz selpoint,  selpoint_Q,//координаты заданной точки для пересчета параметров
                selpoint_v,selpoint_a,selpoint_n;

    } fus;

    struct cargo_out
    {
        float m;          //масса груза на ВТП
        float Ox, Oy, Oz; //точка зацепления троса в СКС
        float CTx,CTy,CTz;//положение центра тяжести груза в СКС
        float Gx, Gy, Gz; //проекции вектора веса груза в СКС
    } cargo;

    struct shassi_out
    {
        int     type,
                status;//состояние шасси: 0-свободно, 1-сжимается/разжимается, 2-обж.до упора
        int     nwh;//количество колес
        double  Bwh,Dwh,rwh, AB, ground;//Rwh,
       floatxyz O, Q, A, B, V,
                W, W1,W2, Wfull,Wfree,
                D, D1,D2,
                C, C1,C2, saCfree,saC,//ssC,ssC1,ssC2,
                F, F1,F2;//проекции силы в СКС
        double  Aap,Aa,Ap,Pay,Ppy;//работы и приведенные усилия
        double  psih,psif, //pk_a, pkfa, pk_p, pkfp, //передаточные числа
                ang_sx,ang_sz, ang_ax,ang_az, ang_pn;//важнейшие углы в кинематике стойки

        struct  stojka_out_absorber
        {
            int     status;
            double  S, ha,ha1,ha2, Pa,Pa1,Pa2, Pg1,Ph1,Pf1, Pg2,Ph2,Pf2, V1,V2,p1,p2, v1,v2, dh12;
        } a;

        struct  stojka_out_pneumatic
        {
            int     status;
           floatxyz h,h1,h2, P,P1,P2;
            double  S;
        } p;

    } st[mst];

    struct mrotor_out
    {
           bool solve,
                vgsolve;
            int nb, //количество лопастей
                nSec,//количество сечений
                sec07,
                az;
            int revcount; //счетчик полных оборотов
       floatxyz O,//положение начала локальной системы координат отн.СКС
                eh, wh, ah, vh;//, sh
         double psih,
                alfa,beta,//углы атаки и скольжения
                fi0, chi, eta,
                D, //диаметр винта
                eps_fz, eps_fx, //угол заклинения фюзеляжа
                chi_hub,  eta_hub,  //углы наклона втулки НВ
                l_vsh, //Lb, l_gsh,
                Faxe,
                dctbs,
                angbs;
         xyz dct;

         double Fhx_sr,  Fhy_sr,  Fhz_sr,  Mhx_sr,  Mhy_sr,  Mhz_sr;//средние за оборот сумм.усилия на втулке НВ (CKПС)
         double Fhx_sum, Fhy_sum, Fhz_sum, Mhx_sum, Mhy_sum, Mhz_sum;//суммарные силы и моменты на втулке от всех лопастей (CKПС)

        ///усилия на шарнирах и параметры по сечениям
        struct blades //лопасти
        {
            vector <float> _r, r, bh, dfi, prof;
            vector <float> alfar,fi,fl, Ux, Uy, U, Fi, lambda, M, cy, cx, mz, dT_dr, dQ_dr, dMa_dr, Uywz,Uywx;

            double  psi,  psi0,
                    fi7,  dfi7,  d2fi7,
                    beta, dbeta, d2beta,
                    gama, dgama, d2gama,
                    ksi,  dksi,  d2ksi;
                    //результирующие усилия в шарнирах
            double  Fx,  Fy,  Fz,//проекция суммарной силы на ось х в с-ме кординат лопасти в конкретном азимуте, Н
                    M1x, M1y, M1z,//момент сил на ГШ, Н*м
                    M2x, M2y, M2z,//момент сил на ВШ, Н*м
                    M3x, M3y, M3z,//момент сил на ОШ, Н*м
                    Fhx, Fhy, Fhz,//
                    Mhx, Mhy, Mhz;//

            double //H, T, S, Max, May, Maz;//средние за оборот сумм.усилия на втулке НВ в локальной СК
                    T,Q,Ma,H,S,
                    //Mxa,Mza,//момент аэрод. сил отн. оси хн винта в конкретном азимуте, Н*м
                    //Mxj,Mzj,//момент инерционных сил отн. оси хн винта в конкретном азимуте, Н*м
                    //MQ,     //момент силы сопротивления лопасти отн. оси винта, Н*м
                    //моменты в шарнирах от внешних сил
                    M1T,
                    M1Q,   //момент силы сопротивления лопасти отн. оси у1, Н*м
                    M1K,   //момент Кориолисовых сил лопасти отн. y1, Н*м
                    M1Gx,M1Gy,
                    M1Px,M1Py,
                    M1Cx,M1Cy,
                    M2T,
                    M2Q,   //момент силы сопротивления лопасти отн.ВШ в конкретном азимуте, Н
                    M2K,   //момент силы Кориолиса отн. ВШ, Н*м
                    M2Gx,M2Gy,
                    M2Px,M2Py,
                    M2Cx,M2Cy,   //момент центробежной силы
                    M3Ma,
                    M3Cz,
                    M3su,Fpov,
                    M3tr,
                    M3iz,
                    //инерционные силы
                    Nr,    //центробежная сила в конкретном азимуте, Н
                    Nf,    //центробежная сила от маховых движений в конкретном азимуте, Н
                    P1,     //сила инерции взмаха в конкретном азимуте, Н
                    P2,     //сила инерции качания в конкретном азимуте, Н
                    K1,  //сила Кориолиса в плоскости вращения от взмаха и от качания, Н
                    //внешнее возмущение
                    Paxz,   //сила инерции от линейного перемещения вдоль продольной и поперечной осей CKПС
                    Payy,   //cила инерции от линейного перемещения вдоль оси НВ
                    //Pexz,   //сила инерции от вращения НВ вокруг продольной и поперечной осей CKПС
                    //Pkxz,   //силы инерции Кориолиса, возникающие от вращения НВ вокруг продольной и поперечной осей CKПС на азимуте psi
                    //демпфер
                    sd, vd, Fdmp, M2D;//перемещение и скорость штока демпфера, сила и момент силы демпфера отн.ВШ
        } b[mbl];

        struct vibrogas //виброгаситель
        {
          double psi, khi, eps;//относительное и абсолютное угловое перемещение, угловое ускорение
          double Mn, Mt, Mk, Mi;
        } vg;
    } mr;

    struct trotor_out
    {
       floatxyz O;//расположение начала локальной системы координат отн.СКС
       bool solve,mount;
        int nb, //количество лопастей
            nSec,//количество сечений
            sec07,
            az,
            revcount; //счетчик полных оборотов
     double psih,
            alfa,beta,//углы атаки и скольжения
            fi0,
            D,//диаметр винта
            l_vsh,
            Fhx_sr,  Fhy_sr,  Fhz_sr,  Mhx_sr,  Mhy_sr,  Mhz_sr,//средние за оборот сумм.усилия на втулке НВ (CKПС)
            Fhx_sum, Fhy_sum, Fhz_sum, Mhx_sum, Mhy_sum, Mhz_sum;//суммарные силы и моменты на втулке от всех лопастей (CKПС)
            //Fax_sum, Fay_sum, Faz_sum, Max_sum, May_sum, Maz_sum;//airdyn силы и моменты на втулке от всех лопастей (CKПС)

       ///усилия на шарнирах и параметры по сечениям
       struct blades //лопасти
       {
           vector <float> _r, r, bh, dfi, prof;
           vector <float> alfar,fi,fl, Ux, Uy, U, Fi, lambda, M, cy, cx, mz, dT_dr, dQ_dr, dMa_dr, Uywz,Uywx;

           double  psi,  psi0,
                   fi7,  dfi7,  d2fi7,
                   beta, dbeta, d2beta,
                   gama, dgama, d2gama,
                   ksi,  dksi,  d2ksi;
                   //результирующие усилия в шарнирах
           double  Fx,  Fy,  Fz,//проекция суммарной силы на ось х в с-ме кординат лопасти в конкретном азимуте, Н
                   M1x, M1y, M1z,//момент сил на ГШ, Н*м
                   M2x, M2y, M2z,//момент сил на ВШ, Н*м
                   M3x, M3y, M3z,//момент сил на ОШ, Н*м
                   Fhx, Fhy, Fhz,//
                   Mhx, Mhy, Mhz;//

           double //H, T, S, Max, May, Maz;//средние за оборот сумм.усилия на втулке НВ в локальной СК
                   T,Q,Ma,H,S,
                   //Mxa,Mza,//момент аэрод. сил отн. оси хн винта в конкретном азимуте, Н*м
                   //Mxj,Mzj,//момент инерционных сил отн. оси хн винта в конкретном азимуте, Н*м
                   //MQ,     //момент силы сопротивления лопасти отн. оси винта, Н*м
                   //моменты в шарнирах от внешних сил
                   M1T,
                   M1Q,   //момент силы сопротивления лопасти отн. оси у1, Н*м
                   M1K,   //момент Кориолисовых сил лопасти отн. y1, Н*м
                   M1Gx,M1Gy,
                   M1Px,M1Py,
                   M1Cx,M1Cy,
                   M2T,
                   M2Q,   //момент силы сопротивления лопасти отн.ВШ в конкретном азимуте, Н
                   M2K,   //момент силы Кориолиса отн. ВШ, Н*м
                   M2Gx,M2Gy,
                   M2Px,M2Py,
                   M2Cx,M2Cy,   //момент центробежной силы
                   M3Ma,
                   M3Cz,
                   M3su,Fpov,
                   M3tr,
                   M3iz,
                   //инерционные силы
                   Nr,    //центробежная сила в конкретном азимуте, Н
                   Nf,    //центробежная сила от маховых движений в конкретном азимуте, Н
                   P1,     //сила инерции взмаха в конкретном азимуте, Н
                   P2,     //сила инерции качания в конкретном азимуте, Н
                   K1,  //сила Кориолиса в плоскости вращения от взмаха и от качания, Н
                   //внешнее возмущение
                   Paxz,   //сила инерции от линейного перемещения вдоль продольной и поперечной осей CKПС
                   Payy,   //cила инерции от линейного перемещения вдоль оси НВ
                   //Pexz,   //сила инерции от вращения НВ вокруг продольной и поперечной осей CKПС
                   //Pkxz,   //силы инерции Кориолиса, возникающие от вращения НВ вокруг продольной и поперечной осей CKПС на азимуте psi
                   //демпфер
                   sd, vd, Fdmp, M2D;//перемещение и скорость штока демпфера, сила и момент силы демпфера отн.ВШ
       } b[mbl];

    } tr;

    struct protor_out
    {
        floatxyz O;//расположение начала локальной системы координат отн.СКС
        bool solve,mount;
         int nb, //количество лопастей
             nSec,//количество сечений
             sec07,
             az,
             revcount; //счетчик полных оборотов
      double psih,
             alfa,beta,//углы атаки и скольжения
             fi0,
             D,//диаметр винта
             l_vsh,
             Fhx_sr,  Fhy_sr,  Fhz_sr,  Mhx_sr,  Mhy_sr,  Mhz_sr,//средние за оборот сумм.усилия на втулке НВ (CKПС)
             Fhx_sum, Fhy_sum, Fhz_sum, Mhx_sum, Mhy_sum, Mhz_sum;//суммарные силы и моменты на втулке от всех лопастей (CKПС)
             //Fax_sum, Fay_sum, Faz_sum, Max_sum, May_sum, Maz_sum;//airdyn силы и моменты на втулке от всех лопастей (CKПС)

        ///усилия на шарнирах и параметры по сечениям
        struct blades //лопасти
        {
            vector <float> _r, r, bh, dfi, prof;
            vector <float> alfar,fi,fl, Ux, Uy, U, Fi, lambda, M, cy, cx, mz, dT_dr, dQ_dr, dMa_dr, Uywz,Uywx;

            double  psi,  psi0,
                    fi7,  dfi7,  d2fi7,
                    beta, dbeta, d2beta,
                    gama, dgama, d2gama,
                    ksi,  dksi,  d2ksi;
                    //результирующие усилия в шарнирах
            double  Fx,  Fy,  Fz,//проекция суммарной силы на ось х в с-ме кординат лопасти в конкретном азимуте, Н
                    M1x, M1y, M1z,//момент сил на ГШ, Н*м
                    M2x, M2y, M2z,//момент сил на ВШ, Н*м
                    M3x, M3y, M3z,//момент сил на ОШ, Н*м
                    Fhx, Fhy, Fhz,//
                    Mhx, Mhy, Mhz;//

            double //H, T, S, Max, May, Maz;//средние за оборот сумм.усилия на втулке НВ в локальной СК
                    T,Q,Ma,H,S,
                    //Mxa,Mza,//момент аэрод. сил отн. оси хн винта в конкретном азимуте, Н*м
                    //Mxj,Mzj,//момент инерционных сил отн. оси хн винта в конкретном азимуте, Н*м
                    //MQ,     //момент силы сопротивления лопасти отн. оси винта, Н*м
                    //моменты в шарнирах от внешних сил
                    M1T,
                    M1Q,   //момент силы сопротивления лопасти отн. оси у1, Н*м
                    M1K,   //момент Кориолисовых сил лопасти отн. y1, Н*м
                    M1Gx,M1Gy,
                    M1Px,M1Py,
                    M1Cx,M1Cy,
                    M2T,
                    M2Q,   //момент силы сопротивления лопасти отн.ВШ в конкретном азимуте, Н
                    M2K,   //момент силы Кориолиса отн. ВШ, Н*м
                    M2Gx,M2Gy,
                    M2Px,M2Py,
                    M2Cx,M2Cy,   //момент центробежной силы
                    M3Ma,
                    M3Cz,
                    M3su,Fpov,
                    M3tr,
                    M3iz,
                    //инерционные силы
                    Nr,    //центробежная сила в конкретном азимуте, Н
                    Nf,    //центробежная сила от маховых движений в конкретном азимуте, Н
                    P1,     //сила инерции взмаха в конкретном азимуте, Н
                    P2,     //сила инерции качания в конкретном азимуте, Н
                    K1,  //сила Кориолиса в плоскости вращения от взмаха и от качания, Н
                    //внешнее возмущение
                    Paxz,   //сила инерции от линейного перемещения вдоль продольной и поперечной осей CKПС
                    Payy,   //cила инерции от линейного перемещения вдоль оси НВ
                    //Pexz,   //сила инерции от вращения НВ вокруг продольной и поперечной осей CKПС
                    //Pkxz,   //силы инерции Кориолиса, возникающие от вращения НВ вокруг продольной и поперечной осей CKПС на азимуте psi
                    //демпфер
                    sd, vd, Fdmp, M2D;//перемещение и скорость штока демпфера, сила и момент силы демпфера отн.ВШ
        } b[mbl];

    } pr;

};

#endif

