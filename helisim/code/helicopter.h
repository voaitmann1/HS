#ifndef helicopter_H
#define helicopter_H
#include <QString>
#include <QtMath>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_multiroots.h>
#include <gsl/gsl_errno.h>//для уравнения
#include <gsl/gsl_math.h> //для уравнения
#include <gsl/gsl_roots.h>//для уравнения
#include <gsl/gsl_linalg.h>//для СЛАУ
#include <gsl/gsl_spline.h>//для SplineXY
#include <gsl/gsl_interp2d.h>//для SplineXYZ
#include <gsl/gsl_spline2d.h>//для SplineXYZ
#include "types.h"
#include "rotors.h"
#include "shassi.h"
//---------------------------------------------------------------------------
#define mst 5 //макс.кол.амортстоек
#define mbl 6 //макс.кол.лопастей винтов
//---------------------------------------------------------------------------
// СКЗ  - Система координат Земная(абсолютная)
// СКН  - Нормальная система координат
// СКС  - Система координат Связанная
// СКР  - Система координат Расчетная (параллельна СКС, совмещенная с ЦТ вертолета)
// СКНВ - Система координат несущего винта
// СКРВ - Система координат рулевого винта
//---------------------------------------------------------------------------

class Helicopter//класс вертолета
{
public:
           Helicopter();
          ~Helicopter();
    static Helicopter *phc;//указатель класса на себя (нужно для статических функций)
            MainRotor *mr;//класс винта несущего
            TailRotor *tr;//класс винта рулевого
            PushRotor *pr;//класс винта толкающего
              IShassi *st[mst];//класс стойки шасси
        ManualControl manual;

    void    Pre();
    void    PreRotors(outstructure &out);
    void    PreStabOrBalance();
    void    PreFlying(outstructure &out);
    int     SolveFlying(outstructure &out);
    int     FixFlying(outstructure &out);
    QString Post2log();
    void    fillOut( outstructure &out);
    QString CurrentInfo_Airframe();
    QString CurrentInfo_MainRotor();
    QString CurrentInfo_TailRotor();
    QString CurrentInfo_Shassi();

    void    PreBalance();
    int     SolveSpecified(outstructure &out);
    int     SolveBalance0();
    int     SolveBalance(outstructure &out);
    void    PostBalance();
    double  get_dMkr(double vy);//double Mkr
    double  get_dMkr_autogyro(double Tpr_);

    void    inFlow();
    void    SetClassPointer();
    void    setNull();

    // н а с т р о й к и  р а с ч е т а
    int     logkey,//ключ для вывода информации по расчету,
            iter,  //номер рассчитываемой итерации
            snl6iter,snl2iter,
            nstabrev,//количество оборотов стабилизации винта
            fix_org_x, fix_org_y, fix_org_z,
            fix_org_k, fix_org_r, fix_org_t;
    QString log_ogl,logstr, //ing строка-буфер для записи состояния и др. данных
            ss,sss, snl2log,snl6log;
    QString preout,solvout,solvout2,balancereport;//строки-буфера для балансировки
    int     warning;
    bool    warnings;
    bool    flyerror;
    bool    gofly,
            stop, // прервать основной цикл расчета
            balanced,
            autorot, // указатель режима авторотации
            autogyro,// указатель автожира
            shassishow,mrshow;
    double  pi, g, d2r,r2d; //const

    // п а р а м е т р ы  о к р у ж а ю щ е й  с р е д ы
    double Lpt;   // скорость падения температуры, К/м
    double RjmK;  // газовая постоянная, Дж/(мол*К)
    double RjkgK; // газовая постоянная, Дж/(кг*К)
    double Mm;	  // молярная масса сухого воздуха, кг/мол
    double t0;	  // температура воздуха на уровне моря,°С
    double p0;	  // давление воздуха на уровне моря, Па
    double ro;	  // плотность воздуха (МСА), кг/м3
    double roh;   // плотность воздуха на высоте, Нh, кг/м3
    double TK;	  // температура на высоте Нh, К
    double P_;	  // давление на высоте Нh, Па
    double vsound;

    // п а р а м е т р ы  п о л е т а
    double  dt,t,t_;          // время
    double  snlf6prc,snlf6lim;// точность и предел для решателя СНЛУ 6 порядка
    double  snlf2prc,snlf2lim;// точность и предел для решателя СНЛУ 2 порядка
    xyzkrt  ocw,ocw_;         // положение ЦТ вертолета в СКЗ и угол ориентации осей СКС в нормальной СК
    xyzkrt  org,org_;         // положение начала СКС в СКЗ и угол ориентации осей СКС в нормальной СК
    xyzkrt  vn;               // скорость (линейная и угловая) в нормальной СК
    xyz     v,v_, w,w_, a, e; // скорость ускорениe ЦТ вертолета в СКС
    xyz     an,en;            // ускорениe в нормальной СК

    // п а р а м е т р ы  в е р т о л е т а
    bool    mrtilt;
    double  mh, Gh, Ghb;      // расчетная масса, кг //расчетный вес вертолета, Н
    double  Ix, Iy, Iz;       // моменты инерции вертолета
    double  Iyz,Ixz,Ixy;      // моменты инерции вертолета
    double  eps_fz,eps_fx;    // углы заклинения фюзеляжа, рад
    double  eta_hub,chi_hub,eta_hub_,chi_hub_; // углы наклона втулки НВ
    double  l_mr;             // расстояние по оси НВ от КПНВ до строительной горизонтали, мм;
    double  xctmr,yctmr,zctmr;// центровка вертолета отн. НВ, м
    double  xct,  yct,  zct;  // положение ЦТ вертолета в СКС, м
    double  x_mr, y_mr, z_mr; // положение центра втулки НВ в CКС, м
    double  x_tr, y_tr, z_tr; // расстояние до оси РВ от начала СКС, м
    double  x_pr, y_pr, z_pr; // расстояние до оси РВ от начала СКС, м
    double  x_f,  y_f,  z_f;  // точка на фюзеляже для определения скоса от НВ в СКС, м
    double  x_st, y_st, z_st; // координаты стабилизатора в СКС, м
    double  x_k,  y_k,  z_k;  // расстояние до центра давления киля (25% хорды bсах) от начала СКС по оси Ох, м
    double  x_w,  y_w,  z_w;  // координаты крыла в СКС, м
    double  mcg,  Gcg,  lcg;  // масса, вес внешнего груза, длина троса //xcg,  ycg,  zcg,  //координаты точки крепления троса внешнего груза
    xyz     zacep,crgct,Gcrg; // координаты и вес внешнего груза

    double  Hh;			      // высота полета, м
    double  Vhx,Vhy,Vhz;      // компоненты начальной скорости полета, м/с
    double  Vi, Vxy,Vxz;      // скорость набегающего потока, м/с	(Vhx/3.6)
    double  tetra;		      // угол наклона траектории, рад		(Teta_tr)
    double  lst_mr;		      // расстояние от 25% хорды стабилизатора до точки пересечения КПНВ и оси вращения НВ, м
    double  dlt_st;		      // угол между КПНВ и линией соединяющей точку пересечения КПНВ с точкой на 25% хорды стабилизатора, рад
    double  Lctr;             // Расстояние от ЦТ вертолета до оси РВ, м
    double  Lst;		      // расстояние от оси НВ до 25% хорды стабилизатора, м
    double  fi0_min,fi0_max;  // диапазон общего шага, рад
    double  est_min,est_max;  //
    //продувка модели
    double  lmodel;		      // линейный размер на модели, м
    double  Smodel;		      // площадь миделя модели, м2
    double  Mmodel;		      // масштаб модели
    double  Lmod;             // условный линейный размер при определении mxf/myf/mzf, м2
    double  Smod;             // условная площадь к которой отнесены моменты фюзеляжа при определении mxf/myf/mzf, м2

    //SolveBalance()
    double	dchi,deta;		// отклонение тарелки АП от аэродин.расcчитаного, рад
    double	kren,tang,rysk;
    double	CxS;		//
    double	fx,fy;		// сумма проекций сил без НВ
    double	Xa, Xa_need,Xa_need_;
    double	Ya, Ya_need,Ya_need_;
    double	Ra, Ra_need;
    double	errRan;
    double	errFx,  errFy;
    double	errXan, precXan;
    double	errYan, precYan;
    double	errChi, precChi;
    double	errEta, precEta;
    double	errTtr;

    //SolveFlying
    double  Fx,Fy,Fz,Mx,My,Mz;//усилия результирующие, приведенные к ЦТ (в начало СКС)



    ///MainRotor
    int     mrsolve;    // 1:пересчитать НВ  0:использовать готовые силы(неизменные)
    double  alfa_mr;    // угол между набегающим потоком и КПВ, рад, alfa_mr_
    double  eta_ap, chi_ap;
    double  fi0_mr, rpm_mr; //
    double  vs_mr,vs_mr_;//индуктивная скорость
    double  skos_mr_;   //заданный скос потока от НВ
    double  Dmr,Rmr,Fmr;//диаметр,радиус,площадь несущего винта, м
    double  Fx_mr;		// пропульсивная сила НВ в СКНВ
    double  Fy_mr;		// тяга         НВ в СКНВ
    double  Fz_mr;		// боковая сила НВ в СКНВ
    double  Mx_mr;		// момент крена НВ    = проекция на Оx в СКНВ результирующего момента на НВ
    double  My_mr;		// крутящий момент НВ = проекция на Оy в СКНВ результирующего момента на НВ
    double  Mz_mr;		// момент тангажа НВ  = проекция на Оz в СКНВ результирующего момента на НВ
    forcemoment FMmr;	// проекции в СКС силы и момента, действующих на вертолет от НВ
    double  dH_mr;		// приращение продольной силы НВ за счет дополнительного отклонения АП
    double  dS_mr;		// приращение поперечной силы НВ за счет дополнительного отклонения АП
    double  w_mr;		// угловая скорость вращения НВ при заданом режиме, рад/с. 1/с
    double  zl_mr;		// количество лопастей несущего винта
    double  Sgsh_mr;	// стат. момент инерции лопасти относительно г.ш
    double  Bmr;		// коэф. концевых потерь
    double  k_;         // компенсатор взмаха
    double  Rap;		// радиус крепления тяг, м
    double  ll;         // расстояние от оси ОШ до т.крепления тяги управления к поводку, м
    double  lgsh;		// разнос горизонтальных шарниров, м
    double  lvsh;		// разнос вертикальных шарниров, м
    double  dgsh;		// вынос горизонтального шарнира (смещение середины горизонтального шарнира), м
    double  dpsi_cont;	// угол опережения управления, рад
    double  D_2;		// передаточные числа управления циклическим шагом, отношение между углом установки лопасти и углом отклонения АП
    double  D_1;		// передаточные числа управления циклическим шагом, отношение между углом установки лопасти и углом отклонения АП
    double  D1_mr,D1_mr_;// передаточное отношение между углом отклонения равнодействующей и углом отклонения АП
    double  D2_mr,D2_mr_;// передаточное отношение между углом отклонения равнодействующей и углом отклонения АП
    double  zswlmr;	     // вспомогательная величина для расчета сил-моментов от НВ (0.5*zl_mr*Sgsh_mr*w_mr^2*lgsh)
    double  beta_sk;     // угол скольжения, рад
    double  chi_ap_;     // угол отклонения кольца АП НВ в продольном направлении, рад
    double  eta_ap_;     // угол отклонения кольца АП НВ в поперечном направлении, рад
    double  fi0_mr_;     // угол общего шага НВ, рад
    double  Fy_mr_ ;     //      НВ, Н            [положительное направление - вверх]
    double  Fx_mr_ ;     // продольная сила НВ, Н [положительное направление - назад]
    double  Fz_mr_ ;     // боковая сила НВ, Н	  [положительное направление - влево]
    double  Mx_mr_ ;     // момент крена НВ, Нм
    double  My_mr_,Mkr_; // момент кручения НВ, Нм
    double  Mz_mr_ ;     // момент тангажа НВ, Нм

    ///TailRotor
    bool    trmount;
    int     trsolve;     // 1:пересчитать НВ  0:использовать готовые силы(неизменные)
    double  fi0_tr, rpm_tr;
    double  alfa_tr;
    double  skos_tr_rsk, //скос потока у PВ от рыскания, рад
            skos_tr_;    //скос потока у PВ
    double  u_tr_,u_tr;  //индуктивная скорость РВ
    double  Dtr;	     //
    double  Fx_tr;	     // пропульсивная сила РВ в СКРВ
    double  Fy_tr,Ttr_need;// тяга РВ в СКРВ
    double  Fz_tr;	 // боковая сила РВ в СКРВ
    double  Mx_tr;	 // продольный момент РВ
    double  My_tr;	 // крутящий момент РВ
    double  Mz_tr;	 // поперечный момент РВ
    forcemoment FMtr;// проекции в СКС силы и момента, действующих на вертолет от РВ
    double  Fx_tr_;  // продольная сила РВ, Н
    double  Fy_tr_;  // тяга РВ, Н
    double  Fz_tr_;  // поперечная сила РВ, Н
    double  Mx_tr_;  // продольный момент РВ, Н*м
    double  My_tr_;  // вертикальный момент РВ, Н*м
    double  Mz_tr_;  // поперечный момент РВ, Н*м

    ///PushRotor
    bool    prmount;
    int     prsolve;       // 1:пересчитать НВ  0:использовать готовые силы(неизменные)
    double  fi0_pr, rpm_pr;
    double  alfa_pr;
    double  u_pr_,u_pr;    //индуктивная скорость РВ
    double  Kt_pr;         // коэффициент торможения потока перед ТВ
    double  Dpr;		   // диаметр ТВ
    double  Fx_pr;	       // пропульсивная сила РВ в СКРВ
    double  Fy_pr,Tpr_need;// тяга РВ в СКРВ
    double  Fz_pr;	       // боковая сила РВ в СКРВ
    double  Mx_pr;	       // продольный момент РВ
    double  My_pr;	       // крутящий момент РВ
    double  Mz_pr;	       // поперечный момент РВ
    forcemoment FMpr;      // проекции в СКС силы и момента, действующих на вертолет от РВ
    double  Fx_pr_;        //
    double  Fy_pr_;        // тяга TВ, Н
    double  Fz_pr_;        //
    double  Mx_pr_;        //
    double  My_pr_;        //
    double  Mz_pr_;        //

    ///Planer
    int     solvePlaner;// рассчитывать аэродинамические нагрузки
    forcemoment FMadp;	// проекции в СКС аэродинамических силы и момента действующих на весь планер
    forcemoment FMadf;	// проекции в СКС аэродинамических силы и момента действующих на фюзеляж (без оперения)
    double  Vx_mr;		// проекция скорости набегающего потока на связаную ось Ox, м/с
    double  Vy_mr;		// проекция скорости набегающего потока на связаную ось Oy, м/с
    double  delta_mr;	// угол между вектором равнодействующей а.д.сил винта и нормалью к скорости набегающего потока, рад
    double  i1_mr;		// угол между направлением потока за винтом и осью Ox в расч.точке у фюзеляжа, рад
    double  Vvol_mr;	// относительная скорость набегающего потока для рис.3.
    double  uvol_mr;	// относительная индуктивная скорость
    double  u0_mr;		// скорость индуктивного потока на висении, м/с
    double  u_mr;		// средняя индуктивная скорость от НВ, м/с
    double  uh_mr;		// средняя индуктивная скорость, спроецированная на ось Ox, м/с
    double  ut_mr;		// средняя индуктивная скорость, спроецированная на ось Oy, м/с
    double  utau_mr;	// средняя индуктивная скорость, спроецированная на ось потока за винтом, м/с
    double  unor_mr;	// средняя индуктивная скорость, спроецированная на ось перпенд. оси потока за винтом, м/с
    double  ut_f_mr;	// индуктивная скорость в расч.точке у фюзеляжа, спроецированная на ось Oy, м/с
    double  uh_f_mr;	// индуктивная скорость в расч.точке у фюзеляжа, спроецированная на ось Ox, м/с
    double  Ksi1f; 		// коэффициент, учитывающий влияние сужения ???
    double  Ksi2; 		// коэффициент, учитывающий изменение скоса при удалении от крыла по направлению скорости невозмущенного потока;
    double  Ksi3n;		// коэффициент, хар. изменение скоса при удалении от крыла в направлении, перпенд. к скорости невозмущенного потока. - коэффициент взаимоиндукции?
    double  Ksi3t;		// коэффициент, хар. изменение скоса при удалении от крыла в направлении, ........ .......... невозмущенного потока
    double  V_f;		// скорость набегающего потока в расч.точке у фюзеляжа, м/с
    double  q_f;		// Скоростной напор в расч.точке у фюзеляжа, Па
    double  i1_mr_f;	// угол между направлением потока за винтом и осью Ox у фюзеляжа, рад
    double  skos_f_mr,skos_f_mr_coarse;  // скос потока в расч.точке у фюзеляжа, рад
    double  alfa_f;	    // угол атаки фюзеляжа, рад
    double  alfa0f;     // -//- без скоса потока
    double  cxaf, cyaf, czaf, mxaf, myaf, mzaf;
    double  cx_f, cy_f, cz_f, mx_f, my_f, mz_f;
    double  Fx_f;		// сила сопротивления фюзеляжа, Н
    double  Fy_f;		// подъемная сила фюзеляжа, Н
    double  Fz_f;		//
    double  Mx_f;		//
    double  My_f;		//
    double  Mz_f;		//

    ///Stabilizer
    int solveStab;// рассчитывать аэродинамические нагрузки
    double	S_st,l_st;  // площадь и размах стабилизатора
    double	_alfafst;	// = __alfa_f[i]
    double  Ksi1st;	    // коэффициент, учитывающий влияние сужения ???
    double  ut_st_mr;	// индуктивная скорость в расч.точке у стабилизатора, спроецированная на ось Oy, м/с
    double  uh_st_mr;	// индуктивная скорость в расч.точке у стабилизатора, спроецированная на ось Ox, м/с
    double  V_st;		// скорость набегающего потока в расч.точке у стабилизатора, м/с
    double  q_st;		// Скоростной напор у стабилизатора, Па
    double  i1_mr_st;	// угол между направлением потока за винтом и осью Ox у стабилизатора, рад
    double  skos_st_mr,skos_st_mr_coarse;// скос потока у стабилизатора от НВ, рад
    double  skos_st_f;	// скос потока у стабилизатора от фюзеляжа, рад
    double  skos_st_wing, lmbd_wing, Lstw, a_stw, alfa_stw, _y_;
    double  skos_st;	// скос потока у стабилизатора, рад
    double  dalf_st_tng;// изменение угла атаки стабилизатора от изменения тангажа,
    double  eps_st0mr;	// угол установки стабилизатора относительно КПНВ при котором mz.st=0, рад
    double  eps_st;     // Угол установки стабилизатора относительно СГ фюзеляжа, рад//double  eps_st_mr; // Угол установки стабилизатора относительно КПНВ, рад
    double  alfa_st;	// угол атаки стабилизатора, рад
    double  eps_st_pr,eps_st_pr_; // угол установки стабилизатора в продувке, рад
    double  cx_st, cy_st, cxast, cyast;
    double	K_st;       // коэф.торможения потока перед стабилизатором
    double  X_st,Y_st;  // силы на стабилизаторе в локальной нормальной СК, Н
    double  Fx_st;		// сила сопротивления стабилизатора, Н
    double  Fy_st;		// подъемная сила стабилизатора, Н
    double  Fz_st;		//
    double  Mx_st;		//
    double  My_st;		//
    double  Mz_st;		// момент, от стабилизатора, Н*м
    forcemoment FMads;	// проекции в СКС силы и момента, действующих на вертолет от стабилизатора

    ///Keel
    int     solveKeel;  // рассчитывать аэродинамические нагрузки
    double  alfa_keel;
    double  Ksi1keel;	// коэффициент, учитывающий влияние сужения ???
    double  ut_keel_mr;	// индуктивная скорость в расч.точке, спроецированная на ось Oy, м/с
    double  uh_keel_mr;	// индуктивная скорость в расч.точке, спроецированная на ось Ox, м/с
    double  V_keel, q_keel;
    double  skos_k_mr,  //Скос потока у киля от НВ, рад
            skos_k_tr,  //Скос потока у киля от РВ, рад
            skos_k_f,   //скос потока у киля от фюзеляжа
            dalf_k_rsk; //изменение угла атаки киля от рыскания, рад
    double  xi_co;      //поправка, зависит от отн.расстояния (z_tr-z_k)/Rtr и следом().
    double  cxakeel, cyakeel, cx_keel, cy_keel;
    double  X_keel, Y_keel;
    double  eps_keel, S_keel, K_keel;
    double  Fx_keel;	// сила сопротивления , Н
    double  Fy_keel;	// подъемная сила , Н
    double  Fz_keel;	//
    double  Mx_keel;	//
    double  My_keel;	//
    double  Mz_keel;	// момент, от , Н*м
    forcemoment FMadk;	// проекции в СКС силы и момента, действующих на вертолет от

    ///Wing
    int     solveWing;// рассчитывать аэродинамические нагрузки
    double  alfa_wing;
    double   eps_wing;
    double  skos_wing_mr,skos_wing_mr_coarse, skos_wing_f, i1_mr_wing;
    double  q_wing, V_wing, uh_wing_mr, ut_wing_mr, Ksi1wing, K_wing, S_wing, l_wing;
    double  cxawing, cyawing, cx_wing, cy_wing, X_wing, Y_wing;
    double  Fx_wing;	// сила сопротивления , Н
    double  Fy_wing;	// подъемная сила , Н
    double  Fz_wing;	//
    double  Mx_wing;	//
    double  My_wing;	//
    double  Mz_wing;	// момент, от , Н*м
    forcemoment FMadw;	// проекции в СКС силы и момента, действующих на вертолет от

    ///HelicopterGravity()
    forcemoment FMhg;	// проекции в СКС веса вертолета и момента

    ///OutboardCargo()
    forcemoment FMcg;	// проекции в СКС веса груза на ВТП

    ///Shassi
    int nst;
    forcemoment FMsh, FMw[mst],FMw1[mst],FMw2[mst];	// проекции в СКС силы и момента, действующих на вертолет от НВ,FMw_[mst]

    ///ExternalLoad()
    forcemoment FM1,FM2; // проекции в СКС

    //f_uth()			// функция для FuselageForces() и StabilizerForces() со своими аргументами
    double	Xmr_;		// расстояние от оси НВ до расч.точки у фюзеляжа или стабилизатора по оси Oxh, м
    double	Ymr_;		// расстояние от КПНВ до расч.точки у фюзеляжа или стабилизатора по оси Oyh, м
    double	tau_mr;		//
    double	nor_mr;		//
    double	x1_mr;		//
    double	R1_mr;		//
    double	hflow_mr;	//

    //оцифрованные графики для аэродинамических коеффициентов
    double _Vvol_mr[mass1];	 int _Vvol_mr_col;
    double _delta_mr[mass1]; int _delta_mr_col;
    double _h_[mass1];		 int _h_col;
    double _Ksi3n_[mass1];
    double _uvol_mr[mass2];

    //airdyncharacters massives
    double _alfa_f[mass1];	 int _alfa_f_col;
    double _cx_f[mass1];
    double _cy_f[mass1];
    double _cz_f[mass1];
    double _mx_f[mass1];
    double _my_f[mass1];
    double _mz_f[mass1];
    double _cx_st[mass1];
    double _cy_st[mass1];
    double _alfa_st[mass1];     int  _alfa_st_col;
    double _alfa_fs[mass1];     int  _alfa_fs_col;
    double _dalf_stf[mass1];    int _dalf_stf_col;
    double _cx_keel[mass1];
    double _cy_keel[mass1];
    double _alfa_keel[mass1];   int _alfa_keel_col;
    double _cx_wing[mass1];
    double _cy_wing[mass1];
    double _alfa_wing[mass1];   int _alfa_wing_col;

    //only for Mi-2
    double __alfa_f[mass1];     int __alfa_f_col;
    double __eps_st[mass1];     int __eps_st_col;
    double _mz_fst[mass2];      int _mz_fst_row, _mz_fst_col;
    double __cx_st[mass2];      int __cx_st_row, __cx_st_col;
    double __cy_st[mass2];      int __cy_st_row, __cy_st_col;
    double __mz_st[mass2];      int __mz_st_row, __mz_st_col;
    double __eps_st0[mass1]; // углы установки стабилизатора при которых момент варианта с стабилизатором и без стабилизатора равны, рад

    //ExternalLoads
    xyz  load1pos,load2pos;
    QVector <extloadstruct> load1, load2;


protected:

private:
    void    HelicopterDynamic(double x0,double x1,double x2,double x3,double x4,double x5);
    void    HelicopterKinematic(double x0, double x1, double x2, double x3, double x4, double x5);
    void	HelicopterGravity(double tang, double kren);
    void    MainRotorForces();
    void    MainRotorForceCorrection(double dchi, double deta);
    void    TailRotorForces();
    void    TailRotorForceCorrection(double T_tr_);
    void    PushRotorForces();
    void    PushRotorForceCorrection(double T_pr_);//СИЛЫ/МОМЕНТЫ ОТ пропульсивного винта
    void    ShassiForces();
    void    PlanerAirDynForces();
    void	WingAirDynForces();//крыло
    void	StabAirDynForces();//стабилизатор
    void	KeelAirDynForces();//киль
    void	OutboardCargo(double tang, double kren);
    void    ExternalLoads(float t);
    float   inter(float x, float x1, float x2, float y1, float y2);
    void    correctedCWparam();
    xyzkrt  org2ocw(xyzkrt &org);
    xyzkrt  ocw2org(xyzkrt &ocw);
    xyz     SKStoSKN(xyz &A);       //из связанной СК в нормальную
    xyz     SKNtoSKS(xyz &A);       //из нормальной СК в связанную
    xyz     SKS0toSKS1(xyz &A);
    void    SKMRtoSKS();
    //xyz     CKHB2CKC(xyz &A);

    void    initial_airdyn_characters();
    double  f_cx_f(double alfa);
    double  f_cy_f(double alfa);
    double  f_cz_f(double alfa);
    double  f_mx_f(double alfa);
    double  f_my_f(double alfa);
    double  f_mz_f(double alfa);
    double  f_cx_keel(double ak);
    double  f_cy_keel(double ak);
    double  f_cx_wing(double aw);
    double  f_cy_wing(double aw);
    double  f_cx_st(double ast);
    double  f_cy_st(double ast);
    double  f_cx_st2(double est, double alfa);
    double  f_cy_st2(double est, double alfa);
    double  f_mz_st2(double est, double alfa);
    double  f_mz_fst(double est, double alfa);
    double  f_eps_st(double fi0);
    double  f_uvol_mr(double V_vol, double delta);
    double  f_Ksi3n(double h);
    double  f_Ksi2(double x);
    duo     f_uth(double x,double y,double Ksi1);
    double  f_R1(double x);
    double  f_xc(double si, double a,double b,double c,double d);
    double  f_yc(double si, double a,double b,double c,double d);
    double  f_dalf_stf(double afus);
    double  f_dalf_stf2(double afus);
    double  f_eps_st0(double afus);
    double  f_Xco(double _y_);

    //уравнения систем
    double  f_Fx();
    double  f_Fy();
    double  f_Fz();
    double  f_Mx();
    double  f_My();
    double  f_Mz();
    void    RecalcAngles(double x0,double x1);
    double  f_dkren();
    double  f_drysk();
    double  f_dtang();
    void	HeliBalanceDynamic(double x0, double x1, double x2, double x3);
    //void	HeliBalanceDynamic5(double x0, double x1, double x2, double x3, double x4);
    double  b_Fx();
    double  b_Fy();
    double  b_Fz();
    double  b_Mx();
    double  b_My();
    double  b_Mz();

    //решатели
    six     SolveSystemNL6();
    pent    SolveSystemNL5();
    quat	SolveSystemNL4();
    quat	SolveSystemNL4ag();
    tre     SolveSystemNL3();
    duo 	SolveSystemNL2();
    double  SplineXY(double xi, double x_arr[], double y_arr[], int n);
    double  SplineXYZ(double xi, double yi, double x_ar[], double y_ar[], double z_arr[], int nx, int ny);
    double  RegressXY(int sp, double xi, double x_arr[], double y_arr[], int n);
    double  SolvEquation(int fkey, double x1_lo, double x1_hi);
    quat	SolveSystem4x4(pent e1, pent e2, pent e3, pent e4);
    gsl_vector* slae_solve(double A_data[],double B_data[], int ne, int nx);
    static int system2nlf(const gsl_vector *x, void *params, gsl_vector *fstr);
    static int system4nlf(const gsl_vector *x, void *params, gsl_vector *fstr);
    static int system4ag (const gsl_vector *x, void *params, gsl_vector *fstr);
    static int system5nlf(const gsl_vector *x, void *params, gsl_vector *fstr);
    static int system6nlf(const gsl_vector *x, void *params, gsl_vector *fstr);
    static double f_x1root1(double x, void *params);
    static double f_x1root2(double x, void *params);
    static double f_e0root3(double x, void *params);

    //var
    double xc000, xc015, yc000, yc015, a0yb1;
};
//---------------------------------------------------------------------------

//for Impulse
#define mas  801
#define mas_ 301
#define npr  8

class Impulse//класс вертолета
{
protected:
private:
public:
           Impulse();
          ~Impulse();

           //void  PreImpulse();
           void  SolveImpulse();
           //void N_hover(float H,int i);  //потребная мощность на висении
           //void N_gor(float H,float V,int i,int j);//потребная мощность для гор. полета
           //void CheckGmax(); //проверка предельной нагрузки
           //void FirstMethod();
           //void  N_gor2(float H,float V,int i,int j);//потребная мощность для гор. полета по 2му методу
           void  N_gor_Vildgrube(float H,float V,int i,int j);//потребная мощность для гор. полета по 2му методу
           float getRo(float H);
           float getKsi(float V);
           float getCy_pr_max(float M);  //находит максимальный Cy
           float GetAlfaForPull(float ro, float G);
           float GetAlfOfCy(float cy);
           float GetCxOfAlfa(float alfa);
           float GetCx(float M,float a);
           float GetA_sr(float M);
           float get_n(float mu);//коэффициент n для проф. сопротивления (Джонсон)
           float GetAlfa(float cy,float M);//возвращает угол в гадусах
           float getNvzl(float h,float v); //взлетная мощность дв. на заданной высоте
           float getNnom(float h,float v); //ном. мощность дв. на заданной высоте
           float getNkr(float h,float v); //крейсерская мощность дв. на заданной высоте
           float getN(vector<float> H, vector<float> V,vector< vector<float> > N,float h,float v); //мощность дв. на заданной высоте
           void  StaticCeiling();  //статический потолок
           void  DynCeiling();  //динамический потолок
           void  DynCeiling_nom();  //динамический потолок (на номинальной мощности)
           bool  DataRead();
           bool  ProfilDataRead();
           void  Calc();
           void  Calc_max_step();
           float get_Ct_kr(float mu); //
           float get_Ct_kr_KhAI(float mu); //
           float get_Ct_kr_GT(float mu); //
           void  N_max_step(float H,float V,int i,int j);//потребная мощность для гор. полета при максимальном шаге НВ
           float MaxSpeed(int i); //максимальная скорость на i-й высоте
           float MinSpeed(int i);
           float stallingSpeed_Ct(int i);  //скорость срыва потока по Сt
           float stallingSpeed_a_kr(int i);//скорость срыва потока по критическому альфа
           float stallingSpeed_Cy(int i);  //скорость срыва потока по критическому Cy
           float MaxSpeedNom(int i); //максимальная скорость на i-й высоте
           float MinSpeedNom(int i);
           float EkSpeed(int i); //экономическая скорость на i-й высоте
           float NaivSpeed(int i); //наивыгоднейшая скорость на i-й высоте
           float NaivN(int i); //наивыгоднейшая мощность на i-й высоте
           float Vy_gor(int i); //максимальная скороподъемность на i-й высоте
           float Vy_gor_nom(int i); //максимальная скороподъемность на i-й высоте на номинальной мощности
           float Vy_vert(int i); //максимальная вертикальная скороподъемность на i-й высоте
           float Vy_vert2(int i); //максимальная вертикальная скороподъемность на i-й высоте
           void  setV500(); //определяет макс. скорость с учетом срыва
           float getAzv(float H); //скорость звука на высоте Н
           float get_Cy_kr(float mu); //
           float getI(float V);
           float getI0(float dfi);
           void  check_Vector(vector<float> H, vector<float> V, vector< vector<float> >* N);
           void  check_Vektor_Ksi(vector<float> V_ksi, vector<float>* ksi);
           float get_Np(float V, float c_); //профильная мощность по Вильдгрубе
           float getCe(float N); //удельный расход по мощности
           //void  DrawChart();
           //void  RezWrite();

           vector<float> V_Ksi;
           vector<float> Ksi;
           vector<float> HNvzlet;
           vector<float> VNvzlet;
           vector< vector<float> > Nv;
           vector<float> HNnom;
           vector<float> VNnom;
           vector<float> t_vec;
           vector< vector<float> > Nn;
           vector<float> HNk;
           vector<float> VNk;
           vector< vector<float> > Nk;

           char  prim[10][1000];
           int   i_, j_, in, __ind[mas_][mas];
           float Cy_p[npr],Cx_p[npr],ALfa_p[npr], alfaP[11],MP[6],CYp[6][11],CXp[6][11], a_sr[6];
           float F,D_nv,b_07,z,p,G,sigma,H[mas],N[mas_][mas],wR,alfa7[mas_][mas],B_[mas], //N[V][H]
                 Ct_vis[mas], Vi[mas],Ni[mas_][mas],alfa[mas_][mas],Cy[mas_][mas],
                 Cx[mas_][mas],Cx_0[mas_][mas],Np[mas_][mas],
                 N_vzl[3][3],N_nom[3][3],H_[3],Hst,Hdyn,V[mas_],ksi,Hst_sryv,Hdyn_sryv,
                 Xvr[mas_][mas],Ndvige[mas_][mas],Cx_vr,T_g[mas_][mas],Vi_g[mas_][mas],
                 B_g[mas_][mas],Ct[mas_][mas],N_kr[3][3],Vmax[mas],Vmin[mas],Vy_g[mas],Vy_v[mas],
                 tau[mas_][mas],V1[mas_],VmaxNom[mas],VminNom[mas],e1,t_obd,V_[3],Vnaiv[mas], Nmin_[mas],
                 Vek[mas],n,t,p_,g,Vmax_sr[mas],N__[mas_][mas],Cy_pr_max,mu[mas_][mas],
                 alfa_k[mas_][mas],Vy_g_nom[mas], Hdyn_nom, Hdyn_sryv_nom, mk[mas_][mas],
                 eta0[mas],sryv[mas_][mas],
                 M270[mas_][mas],CY_sryv[mas_][mas],sryv_Cy[mas_][mas],Vsr_a[mas],Vsr_Cy[mas],V500v,V500n,
                 D_rv,b_rv,z_rv,sigma_rv,F_rv,popravka_RV [mas_][mas],
                 N_i[mas_][mas],N_p[mas_][mas],N_prop[mas_][mas],k, Nsp[mas],Cy_MAX,
                 a,gama,fi_is;
           float V_sryv_Ct[mas],_k,Igs,B,db,m_i[mas_][mas],m_pr[mas_][mas],m_vr[mas_][mas],
                 a0[mas_][mas],fi0_e[mas_][mas],fi0[mas_][mas],lambda[mas_][mas],dfi,fi0_v[mas_][mas],
                 fi0_e_j[mas_][mas],fi0_j[mas_][mas],popr_RV,m_i_rv[mas_][mas],m_pr_rv[mas_][mas],
                 N_i_rv[mas_][mas],N_p_rv[mas_][mas],N_potr[mas_][mas], N_rv[mas_][mas], N_nv[mas_][mas],
                 ksi_rv[mas_][mas],NNaiv[mas],xi,I0,I[mas_];
           float Ce[mas_][mas], Q[mas_][mas],q[mas_][mas]; //для дальности

};
//---------------------------------------------------------------------------*/

#endif
