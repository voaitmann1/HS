//ROTORS.MATHEMATICAL.MODEL
#include <QtMath>
#include <QCoreApplication>
#include <vector>
#include "rotors.h"
#include "helicopter.h"
#define appname "helisim"
//
#include<QMessageBox>
//-----------------------------------------------------------------------------

/// RotorBase /////////////////////////////////////////////////////////////////

RotorBase::RotorBase()
{
}
RotorBase::~RotorBase()
{
}

void RotorBase::Preparation()//подготовка считанных исходных данных к расчету
{
    warning = 0;   itcount = 0;   revcount= 0;   logstr = "";   outres = "";   title = "";
    zakrut = false;////////////////////////////

    switch (ROTORID)//винт
    {
    case MR:
        title = "Винт несущий, ";
        break;
    case TR:
        title = "Винт рулевой, ";
        break;
    case PR:
        title = "Винт толкающий, ";
        break;
    }
    switch (hubtype)//тип втулки
    {
    case 0://hinge=шарнирный
        title += "втулка шарнирная (ОШ+ГШ+ВШ), ";
        break;
    case 1://hinge=шарнирный
        title += "втулка шарнирная (ОШ+ГШ), ";
        break;
    case 2://teeter=качалка на кардане
        title += "втулка карданного типа (ОШ+СГШ), ";
        break;
    case 3://hingeles=бесшарнирный
        title += "втулка бесшарнирная (ОШ), ";
        break;
    }
    switch (rdirect)//коэф. учета направления вращения: винтов левого вращения = 1, правого вращения = -1
    {
    case 0://левого вращения (отечественные НВ)
        krd = 1;
        title += "направление вращения: против ЧС (взгляд по оси Y);";
        break;
    case 1://правого вращения
        krd = -1;
        title += "направление вращения: по ЧС (взгляд по оси Y);";
        break;
    }

    //константы
    d2r = M_PI/180.0;//коэф. перевода из градусов в радианы
    r2d = 180.0/M_PI;//коэф. перевода из радиан в градусы
    double Lpt = 0.0065;//скорость падения температуры, К/м
    double RjmK = 8.31447;//газовая постоянная, Дж/(мол*К)
    double Mm = 0.0289644;//молярная масса сухого воздуха, кг/мол
    double RjkgK = 287.0;//газовая постоянная, Дж/(кг*К)
    double TK = t0 + 273.15 - Lpt*Hy;//температура на расчетной высоте
    double P_ = p0 * pow((1 - Lpt*Hy /(15.0 + 273.15)), (g*Mm /(RjmK*Lpt)));//давление на расчетной высоте
    //плотность воздуха как функция высоты
    if (ro<0.1) ro = P_* Mm /(RjmK*TK);//плотность воздуха
    //скорость звука
    sV = sqrt(1.4*TK*RjkgK);//=vsound

    //проверка (и выравнивание) длин векторов r_is, b_is, dfi_is, prof_is
    checkVectors();
    //наполняет массив Сх при Су=0
    getCxCy0Array();

    //кол-во участков по длине лопасти: +2 для вставки сечений на радиусе B*R
    nSec = r_is.size() + 2;
    //радиус винта
    R = r_is.at(r_is.size() - 1);
    //радиус комлевого сечения лопасти
    r0 = r_is.at(0);
    //
    (Bkp < 1) ? rB = Bkp*R : rB = R;
    //ометаемая площадь НВ
    F = M_PI * R*R;
    //отн. разнос ГШ, м
    _l_gsh = l_gsh / R;
    //отклонение лопасти от линии 'ось винта - ось ВШ'
    ksi_nom = (l_vsh==0||s_vsh==0)?(0):(atan(fabs(s_vsh)/l_vsh));

    //Расчет проекций скорости полета на оси расчетной СК
    alpha = alpha_;//добавлено 20200701
    getIniFlow();

    //вектор радиусов
    for (int i = 0; i < r_is.size(); i++)
    {
        r.push_back(r_is.at(i));
    }

    int ii; //insert_index
    for (int i = (r_is.size() - 1); i; --i)
    {
        if (r.at(i) <= rB) {  ii = i + 1;  break; }
    }
    r.insert(r.begin()+ii,   rB);
    r.insert(r.begin()+ii+1, rB+0.000001);

    //вектор крутки
    double _dfi;
    _dfi = dfi_is.at(ii-1) + (rB - r_is.at(ii-1)) * (dfi_is.at(ii) - dfi_is.at(ii-1)) / (r_is.at(ii) - r_is.at(ii-1));
    for (int i = 0; i < dfi_is.size(); i++)
    {
        dfi.push_back(dfi_is.at(i) *d2r);
    }
    dfi.insert( dfi.begin()+ii  , _dfi *d2r);
    dfi.insert( dfi.begin()+ii+1, _dfi *d2r);
    krutka = dfi.at(dfi.size()-1) - dfi.at(0);

    //вектор хорд
    if (bh_is.at(ii - 1) == bh_is.at(ii)) //если хорда постоянна
    {
        chord = bh_is.at(ii);
    }
    else
    {
        chord = bh_is.at(ii - 1) + (rB - r_is.at(ii - 1)) * (bh_is.at(ii) - bh_is.at(ii - 1)) / (r_is.at(ii) - r_is.at(ii - 1)) ;
    }
    for (int i = 0; i < bh_is.size(); i++)
    {
        bh.push_back(bh_is.at(i));
    }
    bh.insert(bh.begin()+ii  ,chord);
    bh.insert(bh.begin()+ii+1,chord);

    //вектор профилей
    int _prof = prof_is.at(ii);
    prof = prof_is;
    prof.insert(prof.begin()+ii  , _prof);
    prof.insert(prof.begin()+ii+1, _prof);

    //хорда b7
    for (int i=r_is.size()-1; i; --i)
    {
        if (r.at(i) <= 0.7*R)
        {
            ii = i + 1;
            sec07 = i;
            break;
        }
    }
    if (bh_is.at(ii-1) == bh_is.at(ii)) //если хорда постоянна
    {
        b7 = bh_is.at(ii);
    }
    else
    {
        b7 = bh_is.at(ii - 1) + (0.7*R - r_is.at(ii - 1)) * (bh_is.at(ii) - bh_is.at(ii - 1)) / (r_is.at(ii) - r_is.at(ii - 1)) ;
    }

    //формирование вектора отн. радиусов
    vector <double>::iterator pos;//_r.reserve(CU);
    for (pos = r.begin(); pos != r.end(); ++pos)
    {
        _r.push_back(*pos / R);
    }

    //формирование вектора азимутов
    psih = psih_ = 0;
    if (dt==0)
    {
        dpsi = 2*M_PI / double(nAz);
        dt = dpsi / w;//интервал времени между двумя расчетными азимутами
        psi0.reserve(nAz + 1);
    }
    else
    {
        dpsi = dt * w;
        nAz = 0;//1
    }
    for (int i = 0; i <= nAz; i++)
    {
        iter.push_back(i);
        t.push_back(i*dt);//i*dt
        psi0.push_back(i*dpsi);
    }
    for (int l = 0; l < zl; l++)
    {
        //full rotor revolution
        b[l].psi.reserve(nAz + 1);
        b[l].psi = shiftAz(psi0, l);
        b[l].psi0 = b[l].psi.at(0);
        //real move
        B[l].psi_ = B[l].psi = B[l].psi0 = normAng( psih + 2*M_PI*(zl-l)/double(zl) );
        B[l].fi7_ = fi0 + dfi.at(sec07);
        B[l].dfi7_  = 0;
        B[l].d2fi7_ = 0;
        B[l].gama_  = 0;
        B[l].dgama_ = 0;
        B[l].d2gama_= 0;
        B[l].beta_  = 0;//5*d2r;
        B[l].dbeta_ = 0;
        B[l].d2beta_= 0;
        if(hubtype) B[l].ksi_ = 0;
        else  B[l].ksi_ = ksi_nom;
        B[l].dksi_  = 0;
        B[l].d2ksi_ = 0;
        if(nAz==0) b[l].psi.at(0) = B[l].psi0;
    }

    //коэф. заполнения винта
    sigma = zl * b7 / (M_PI * R);
    //масса лопасти
    ml    = mlp *(R-r0);
    //стат.момент и момент инерции отн.шарниров
    I_osh = Iop *(R-r0);
    I_gsh = Ib0 - ml*l_gsh*l_gsh;
    S_gsh = Sb0 - ml*l_gsh;
    I_vsh = Ib0 - ml*l_vsh*l_vsh;
    S_vsh = Sb0 - ml*l_vsh;
    //сумм.момент инерции бесшарнирных лопастей отн.оси вращения
    Jy = zl*ml*R*R/3.0;
    //центробежный момент инерции лопасти
    Icb = Sb0 *(xo-xt)*chord;//[mil:359]
    //коэф. уравнения равновесия отн. ГШ
    KA1 = Sb0 - ml*l_gsh;
    KB1 = Ib0 - 2*Sb0*l_gsh + ml*l_gsh*l_gsh;
    KC1 = Ib0 - Sb0*l_gsh;
    //коэф. уравнения равновесия отн. ВШ
    KA2 = Sb0 - ml*l_vsh;
    KB2 = Ib0 - 2*Sb0*l_vsh + ml*l_vsh*l_vsh;
    KE  = Ib0 - Sb0*l_gsh - Sb0*l_vsh + ml*l_gsh*l_vsh;

    //параметры демпфера-кулисы
    double Ak,Bk;
    d.fik = KatToAng( (d.z3-l_vsh), (d.x3-s_vsh) );//корректировка угла ksi "к отсчету от вертикали"
    Ak  = s_vsh - d.x2;
    Bk  = l_vsh - d.z2;
    d.C_ = Ak*Ak + Bk*Bk + d.h*d.h;
    d.D_ = sqrt( Ak*Ak + Bk*Bk );
    if(Bk!=0) d.A_ = atan( Ak / Bk );  else d.A_ = 0.5*M_PI;
    d.F = 0.25* M_PI * ( d.dn*d.dn - d.dv*d.dv );
    if (d.mu && d.f)
         d.sigma = 0.5* d.ro * d.F*d.F*d.F / ( d.mu*d.mu * d.f*d.f);
    else d.sigma = 0;
    if (d.mu && d.f2)
         d.sigma2 = 0.5* d.ro * d.F*d.F*d.F / ( d.mu*d.mu * (d.f2+d.f)*(d.f2+d.f));
    else d.sigma2 = 0;
    //
    //
    vsh.EnableWriting();  //vsh.DisableWriting();//
    vsh.debugConsole=false;
    vsh.SetFullFileName1_qs("c:\\temp\\hs_rot_prot.txt");
    vsh1.EnableWriting(); //vsh1.DisableWriting();
    vsh1.debugConsole=true;
    //vsh1.SetFullFileName1_qs("c:\\temp\\hs_rot_prot.txt");
    //
    this->CalcCalls1=0;//2026/10/26
    this->CalcCalls2=0;//2026/10/26
}



// динамика в реальном времени: ////////////////////////////////////////

void RotorBase::RotorIterate(double _T, double _H, double _S, double _V, double _alpha, double _beta_sk, double _fi0, double _eta, double _chi)//параметры винта на текущей итерации
{
    logstr = "";
    warning = 0;
    double fx, fz;
    vector <double> tmp2(nSec), dMQ_dr1(nSec);
    for (int l = 0; l < zl; l++)
    {
        B[l].fi = B[l].fl = B[l].Fi = B[l].alfar = B[l].M = B[l].cy = B[l].cx = B[l].mz = tmp2;
        B[l].dT_dr = B[l].dQ_dr = B[l].dMa_dr = B[l].dXaU_dr = B[l].dTv_dr = tmp2;
        B[l].lambda = B[l].U = B[l].Ux = B[l].Uy = B[l].Uywx = B[l].Uywz = tmp2;
        B[l].beta = B[l].dbeta = B[l].d2beta = B[l].ksi = B[l].dksi = B[l].d2ksi = B[l].khi = 0;
        B[l].fi7 = B[l].dfi7 = B[l].d2fi7 = B[l].gama = B[l].dgama = B[l].d2gama = B[l].xd7 = 0;
        B[l].T = B[l].Ty = B[l].Q = B[l].H = B[l].S = B[l].Ma = 0;
        B[l].Nr = B[l].Nf = B[l].P1 = B[l].P2 = B[l].K1 = B[l].Fx = B[l].Fy = B[l].Fz = B[l].FNx = B[l].FNz = 0;
        B[l].Fhx = B[l].Fhz = B[l].Fhy = B[l].Mhx = B[l].Mhy = B[l].Mhz = 0;
        B[l].MQ = B[l].Mza = B[l].Mxa = B[l].Mzj = B[l].Mxj =  B[l].M2D = B[l].Fdmp = B[l].ni = B[l].np = 0;
        B[l].M1T = B[l].M1Q = B[l].M1Px = B[l].M1Py = B[l].M1K = B[l].M1Gx = B[l].M1Gy = B[l].M1Cx = B[l].M1Cy = B[l].M1Cz = 0;
        B[l].M2T = B[l].M2Q = B[l].M2Px = B[l].M2Py = B[l].M2K = B[l].M2Gx = B[l].M2Gy = B[l].M2Cx = B[l].M2Cy = B[l].M2Cz = 0;
        B[l].M3Ma = B[l].M3Cz = B[l].M3su = B[l].M3tr = B[l].M3dmp = B[l].M3iz = 0;
        B[l].M1x = B[l].M1y = B[l].M1z = B[l].M2x = B[l].M2y = B[l].M2z = B[l].M3x = B[l].M3y = B[l].M3z = 0;
        B[l].Paxz = B[l].Payy = B[l].Pay = B[l].Pexz = B[l].Pkxz = 0;
    }
    FxhQ=0;    FxhT=0;    FxhNr=0;   FxhNf=0;   FxhP1=0;   FxhP2=0;   FxhK1=0;   //FxhK2=0;
    FzhQ=0;    FzhT=0;    FzhNr=0;   FzhNf=0;   FzhP1=0;   FzhP2=0;   FzhK1=0;   //FzhK2=0;
    Fhy_sum=0; Fhx_sum=0; Fhz_sum=0; Mhy_sum=0; Mhx_sum=0; Mhz_sum=0; np_sum=0;
    ctMRx=0;   ctMRz=0;
    //полученные параметры
    alpha = _alpha;   beta_sk = _beta_sk;
    T_i   = _T;         fi0   = _fi0;
    H_i   = _H;         eta   = _eta;
    S_i   = _S;         chi   = _chi;
    V     = _V;
    //проекции скорости полета, относительные скорости
    getIniFlow();
    //режим
    CALCMODE = FORCES;
    //углы установки лопасти отн. КПВ в азимутах 270 и 180 соотв.; для get_fi()
    if (ROTORID==MR)
    {
        Theta1 = _D2*eta - _D1*chi;
        Theta2 = _D1*eta + _D2*chi;
    }
    //Расчет индуктивной скорости
    RotorBase::getInducedFlow();
    //коэфф. в формуле индуктивной скорости
    f2 = get_f2(mu);
    f1 = 1.0 - f2;
    if (FLIGHTMODE==AXIAL)
    {
        if(fabs(_Vy)<0.01)
            logstr += "flightmode= Hovering\n";
        else if(_Vy>0)
            logstr += "flightmode= AxialUp\n";
        else
            logstr += "flightmode= AxialDown\n";
    }
    else //HORIZONTAL
    {
        if(fabs(_Vy)<0.01)
            logstr += "flightmode= HorizontFly\n";
        else if(_Vy>0)
            logstr += "flightmode= Fly&Up\n";
        else
            logstr += "flightmode= Fly&Down\n";
    }

    ///1: ПАРАМЕТРЫ ДВИЖЕНИЯ ЛОПАСТЕЙ /////////////////////////////////////////////////////////////////////////////////////////////////////////
    psih = normAng(psih_ + dpsi);// normAng()
    for (int l = 0; l < zl; l++)
    {
        B[l].psi = normAng( B[l].psi_ + dpsi );
        blade_kinematic_dt(l);//B[l].beta
        //get_ksi_(l); //B[l].ksi
        B[l].khi = B[l].ksi *(1 - l_vsh/(0.505*R));
        //get_gama_(l);//B[l].gama

        ////положение центра тяжести лопасти во вращающейся с втулкой СК
        //ct1bx = 0.5*R *sin( B[l].psi - B[l].khi );
        //ct1bz = 0.5*R *cos( B[l].psi - B[l].khi );
        ////сумма проекций центров тяжести лопастей во вращающейся с втулкой СК
        //ctMRxr += ct1bx;
        //ctMRzr += ct1bz;
    }
    // //центр тяжести системы лопастей во вращающейся с втулкой СК
    //ctMRxr /= zl;
    //ctMRzr /= zl;
    //toHubSystem(psih, ctMRxr, ctMRzr, ctMRx, ctMRz);
    // //отклонение центра тяжести системы лопастей относит. оси НВ
    //dctbs = sqrt( ctMRxr*ctMRxr + ctMRzr*ctMRzr );
    // //направление неуравновешенной (центробежной) силы на оси НВ //угол между вектором смещения ЦТ НВ и осью Х НВ паралл.оси Х фюзеляжа
    //act = KatToAng( ctMRzr, ctMRxr );
    // //центробежная сила системы лопастей на оси НВ во вращающейся с втулкой СК
    //Fbs = zl*ml*w*w*dctbs;//Faxe
    //Fbszr = Fbs *sin( act );//проекция на ось Z
    //Fbsxr = Fbs *cos( act );//проекция на ось X
    // //центробежная сила системы лопастей в СКПС
    //toHubSystem(psih, Fbsxr, Fbszr, Fbsx, Fbsz);

    ///2: УСИЛИЯ РАСПРЕДЕЛЕННЫЕ И ИНТЕГРАЛЬНЫЕ ////////////////////////////////////////////////////////////////////////////////////////////////
    for (int l = 0; l < zl; l++)
    {
        //проход по сечениям лопасти
        for (int i = 0; i < nSec; i++)
        {
            seg = Segment(i, B[l].psi, B[l].beta, B[l].dbeta, B[l].gama, B[l].dgama, B[l].dfi7);
            //... параметров распределенных по лопасти
            B[l].alfar .at(i) = seg.alfar;
            B[l].fi    .at(i) = seg.fi;
            B[l].fl    .at(i) = seg.fl;
            B[l].U     .at(i) = seg.U;
            B[l].Ux    .at(i) = seg.Ux;
            B[l].Uy    .at(i) = seg.Uy;
            B[l].Uywx  .at(i) = seg.Uywx;
            B[l].Uywz  .at(i) = seg.Uywz;
            B[l].lambda.at(i) = seg.lambda;
            B[l].Fi    .at(i) = seg.Fi;
            B[l].M     .at(i) = seg.M;
            B[l].cy    .at(i) = seg.cy;
            B[l].cx    .at(i) = seg.cx;
            B[l].mz    .at(i) = seg.mz;
            B[l].dT_dr .at(i) = seg.dT_dr;
            B[l].dQ_dr .at(i) = seg.dQ_dr;
            B[l].dMa_dr.at(i) = seg.dMa_dr;
            if(i==sec07)
            {
                B[l].fi7 = seg.fi;
                B[l].xd7 = seg.xd;
            }

            //массивы распределенных по лопасти удельных моментов
            dMQ_dr1.at(i) = B[l].dQ_dr.at(i) * ((r.at(i)-l_gsh)*cos(B[l].beta) + l_gsh);//= tmp_dMQ_dr //dM_y1_Q_dr1.at(i) = B[l].dQ_dr.at(i) * ((r.at(i)-l_gsh)*cos(B[l].beta));//= tmp_dM_y1_Q_dr//не используется
            //for инд. мощности Ni
            B[l].dTv_dr.at(i) = B[l].dT_dr.at(i) * cos(B[l].beta) * vt *
                          ( sqrt(1.5)*pow(_r.at(i),0.5)*f1 + (1 + K_mu*_r.at(i)*cos(B[l].psi))*f2 );//= tmp_dTv_dr
            //for проф. мощности Np
            B[l].dXaU_dr.at(i) = 0.5*ro*bh.at(i) * B[l].cx.at(i) * pow(B[l].U.at(i),3);//= tmp_dXaU_dr
            if (B[l].Ux.at(i) < 0)  B[l].dXaU_dr.at(i) = -1*B[l].dXaU_dr.at(i);
        }
        //для расчета компонентов мощностей
        B[l].ni = get_Integral(B[l]. dTv_dr, r)/1000.0; //B[l].Tv  for инд. мощности Ni
        B[l].np = get_Integral(B[l].dXaU_dr, r)/1000.0; //B[l].XaU for проф. мощности Np
        //элементарная мощность за dt от всех лопастей
        ni_sum += B[l].ni;//инд.
        np_sum += B[l].np;//проф.

        //Расчет аэродинамических сил (T,Q,H,S) ГШ,СКЛ
        B[l].T  = get_Integral(B[l].dT_dr, r);
        B[l].Q  = krd*get_Integral(B[l].dQ_dr, r);
        B[l].Ma = get_Integral(B[l].dMa_dr, r);
        B[l].Ty = B[l].T *cos(B[l].beta);
        B[l].H  =   -1* B[l].T *sin(B[l].beta) *cos(B[l].psi) + krd*B[l].Q *sin(B[l].psi);//20211006 добавил krd*
        B[l].S  = -krd* B[l].T *sin(B[l].beta) *sin(B[l].psi) - B[l].Q *cos(B[l].psi);

        ///Расчет инерционных и центробежных сил (N,P,K) ГШ (СКЛ)
        {
        //центробежная сила от вращения винта Н
        B[l].Nr = w*w* ( KA1 *cos(B[l].beta) + ml*l_gsh );//+
        //центробежная сила от маховых движений Н
        B[l].Nf = KA1 *w*w* B[l].dbeta*B[l].dbeta;//+
        //силa Кориолиса от взмаха
        B[l].K1 = -2*krd*KA1 *w*w* B[l].dbeta *sin(B[l].beta);//+KA1
        //силa Кориолиса от качания
        //B[l].K2 = 0;//-2*KA2 *w*w* B[l].dksi * sin(B[l].ksi);// не используется  и сам придумал:)
        //сила инерции, Н
        B[l].P1 = -KA1 *w*w* B[l].d2beta;//+ взмах
        B[l].P2 = -KA2 *w*w* B[l].d2ksi;//+ качание

            //сила инерции в плоскости винта от линейного перемещения вдоль продольной и поперечной осей CKПС
            B[l].Paxz = -ml*( a_x*sin(B[l].psi) - a_z*cos(B[l].psi) );
            //cила инерции от линейного перемещения вдоль оси НВ
            B[l].Pay  = -ml*a_y;
            //сила инерции от вращения НВ вокруг продольной и поперечной осей CKПС
            B[l].Pexz = S_gsh*( e_x*sin(B[l].psi) - e_z*cos(B[l].psi) );
            //силы инерции Кориолиса, возникающие от вращения НВ вокруг продольной и поперечной осей CKПС на азимуте psi
            B[l].Pkxz = 2*S_gsh*w*( krd*w_x*cos(B[l].psi) + w_z*sin(B[l].psi) );
            //общая cила инерции в плоскости тяги
            B[l].Payy = B[l].Pay + B[l].Pexz + B[l].Pkxz;

            //момент силы инерции от линейного перемещения вдоль оси НВ
            double MPay  = -a_y*KA1*cos(B[l].beta);
            //момент силы инерции от вращения НВ вокруг продольной и поперечной осей CKПС
            double MPexz = KC1*cos(B[l].beta)*( e_x*sin(B[l].psi) - e_z*cos(B[l].psi) );
            //момент силы инерции Кориолиса, возникающие от вращения НВ вокруг продольной и поперечной осей CKПС на азимуте psi
            double MPkxz = KC1*cos(B[l].beta)*( krd*w_x*cos(B[l].psi) + w_z*sin(B[l].psi) )*2*w;
            //момент силы инерции в плоскости винта от линейного перемещения вдоль продольной и поперечной осей CKПС
            double MPaxz = -KA2*cos(B[l].beta)*( a_x*sin(B[l].psi) - a_z*cos(B[l].psi) );


        ///ГШ (СКЛ)
        B[l].M1T  = -get_IntegralMdT1( B[l].psi, B[l].beta, B[l].dbeta, B[l].gama, B[l].dgama, B[l].dfi7 );
        //момент силы сопротивления лопасти отн. y1 (Yл)
        B[l].M1Q  =  krd*get_IntegralMdQ1( B[l].psi, B[l].beta, B[l].dbeta, B[l].gama, B[l].dgama, B[l].dfi7 );//не используется
        //момент силы Кориолиса отн.ГШ, Н*м
        B[l].M1K  = -krd*KB1 *w*w* B[l].dbeta *sin(2*B[l].beta);
        //момент центробежной силы
        B[l].M1Cx =  0.5*KB1 *w*w *sin(2*B[l].beta) + KA1*l_gsh *w*w *sin(B[l].beta);
        B[l].M1Cy = -B[l].Nr *l_gsh *sin(B[l].khi);//не используется
        //момент инерционных+центробежных сил
        //if ( I_osh && k_pov && zakrut )
            B[l].M1Cz = ( B[l].d2fi7+B[l].d2gama + B[l].gama) *w*w *Icb;   //B[l].M1Cz = (B[l].d2fi7+B[l].d2gama + B[l].fi7+B[l].gama) *w*w *Icb;
        //else            B[l].M1Cz = 0;
        //момент инерционной силы
        B[l].M1Px = KB1 *w*w* B[l].d2beta  + MPay + MPexz + MPkxz;
        B[l].M1Py = -KE *w*w* B[l].d2ksi   + MPaxz;//не используется

        ///ВШ (СКЛ)
        B[l].M2T  = -get_IntegralMdT2( B[l].psi, B[l].beta, B[l].dbeta, B[l].gama, B[l].dgama, B[l].dfi7 );
        //момент силы сопротивления лопасти отн. ВШ, Н*м
        B[l].M2Q  =  krd*get_IntegralMdQ2( B[l].psi, B[l].beta, B[l].dbeta, B[l].gama, B[l].dgama, B[l].dfi7 );
        //момент силы Кориолиса отн. ВШ, Н*м
        B[l].M2K  = -krd*KE *w*w* sin(2*B[l].beta) *B[l].dbeta;
        //момент центробежной силы
        B[l].M2Cx =  0.5*KE*w*w *sin(2*B[l].beta) + KA2*l_gsh*w*w *sin(B[l].beta);
        B[l].M2Cy = -B[l].Nr *l_vsh *sin(B[l].khi);
        //момент инерционных+центробежных сил
        B[l].M2Cz = B[l].M1Cz;
        //момент инерционной силы
        B[l].M2Px =  KE *w*w* B[l].d2beta  + MPay + MPexz + MPkxz;
        B[l].M2Py = -KB2*w*w* B[l].d2ksi   + MPaxz;

        ///OШ (СКЛ)
          if(zakrut)
            {
            //момент аэродинамических сил
            B[l].M3Ma = get_IntegralMdMa(B[l].psi, B[l].beta, B[l].dbeta, B[l].gama, B[l].dgama, B[l].dfi7);
            //момент силы в тяге системы управления
            B[l].delta_pov = B[l].gama*r_pov;
            //stiff_su = k_pov *r_pov*r_pov;//Жорсткість системи управлніння кутова навколо осьового шарніра
            B[l].Fpov = k_pov * B[l].delta_pov;//B[l].M3su / r_pov;
            B[l].M3su = -B[l].Fpov *r_pov;//-B[l].gama * stiff_su;
            //момент инерционных+центробежных сил        double Icb = Sb0 *(xo-xt)*chord;
            //double Int_Idfidr = I_osh*krutka;  //get_Integral(_Jmdfi_dr, r);
            B[l].M3Cz = -(B[l].d2beta+B[l].beta) *w*w *Icb ;///+ w*w *I_osh*krutka;
            //момент сил трения        double Nr = w*w* Sb0;
            B[l].M3tr = (B[l].dfi7+B[l].dgama>=0)?( -f_pod * r_pod * B[l].Nr ):( f_pod * r_pod * B[l].Nr );
            B[l].M3dmp = gamaDamph(B[l].dgama);//B[l].dfi7+
            //момент сил инерции закрутки
            B[l].M3iz = -( B[l].d2fi7 + B[l].d2gama + B[l].fi7 + B[l].gama )*(w*w*I_osh);
            }
            else
            { B[l].M3su = B[l].M3Cz = B[l].M3iz = B[l].M3tr = B[l].M3dmp = B[l].Fpov = 0; }//B[l].M3Ma =
        }

        //момент веса лопасти (СКЛ)
        switch (ROTORID)
          {
          case MR://неплохо бы учесть перераспределение от крена+тангажа...
            B[l].M1Gx = g*KA1 *cos(B[l].beta);
            B[l].M2Gx = g*KA2 *cos(B[l].beta);
            B[l].M1Gy = 0;
            B[l].M2Gy = 0;
            break;
          case TR:
            B[l].M1Gx = -krd*g*KA1 *sin(B[l].beta) *sin(B[l].psi);
            B[l].M2Gx = -krd*g*KA2 *sin(B[l].beta) *sin(B[l].psi);
            B[l].M1Gy =      g*KA1 *sin(B[l].beta) *cos(B[l].psi);
            B[l].M2Gy =      g*KA2 *sin(B[l].beta) *cos(B[l].psi);
            break;
          case PR:
            B[l].M1Gx = -krd*g*KA1 *sin(B[l].beta) *sin(B[l].psi);
            B[l].M2Gx = -krd*g*KA2 *sin(B[l].beta) *sin(B[l].psi);
            B[l].M1Gy =      g*KA1 *sin(B[l].beta) *cos(B[l].psi);
            B[l].M2Gy =      g*KA2 *sin(B[l].beta) *cos(B[l].psi);
            break;
          }

        //моменты в центре втулки (СКB)
        //момент силы сопротивления лопасти отн.оси винта (он же крутящий момент)
        B[l].MQ  = krd*get_Integral(dMQ_dr1, r);
        //моменты аэрод. сил
        B[l].Mza =      B[l].T *cos(B[l].beta) *l_gsh *cos(B[l].psi);//продольный
        B[l].Mxa = -krd*B[l].T *cos(B[l].beta) *l_gsh *sin(B[l].psi);//боковой
        //момент силы инерции взмаха
        B[l].Mzj =     B[l].P1 *cos(B[l].beta) *l_gsh *cos(B[l].psi);//продольный
        B[l].Mxj =-krd*B[l].P1 *cos(B[l].beta) *l_gsh *sin(B[l].psi);//боковой

        ///силы и моменты от 1 лопасти
        {
            //Результирующие силы в шарнирах, СКЛ
            switch (ROTORID)
            {
            case MR:
                //origin
                B[l].Fx = B[l].Q
                        + B[l].K1;

                B[l].Fy = B[l].Ty - g*ml
                        + B[l].Nf *sin(B[l].beta)
                        + B[l].P1 *cos(B[l].beta);

                B[l].Fz = B[l].Nr
                        + B[l].Nf *cos(B[l].beta)
                        - B[l].T  *sin(B[l].beta)
                        - B[l].P1 *sin(B[l].beta);
                /*//new:
                B[l].Fx =   B[l].Q  *cos(B[l].ksi)//ok
                                + B[l].Nr *sin(B[l].khi ) *cos(ksi_sr*M_PI/180.0)
                                + B[l].P2 *cos(B[l].ksi)
                          + B[l].K1 *cos(B[l].ksi);//ok

                B[l].Fy =   B[l].T  *cos(B[l].beta)//массу лопастей нужно вычесть из массы вертолета
                          + B[l].P1 *cos(B[l].beta)//ok
                          + B[l].Nf *sin(B[l].beta)
                          - g*ml;//ok

                B[l].Fz = - B[l].T  *sin(B[l].beta)// *cos(B[l].ksi)
                          - B[l].P1 *sin(B[l].beta)// *cos(B[l].ksi)
                                - B[l].Q  *sin(B[l].ksi)
                                - B[l].K1 *sin(B[l].ksi)
                                - B[l].P2 *sin(B[l].ksi)
                          + B[l].Nr *cos(B[l].khi ) *sin(ksi_sr*M_PI/180.0)
                          + B[l].Nf *cos(B[l].beta);// *cos(B[l].khi);
                  //  */
                break;

            case TR:
                B[l].Fx = B[l].Q
                        + B[l].K1
                        + g*ml *cos(B[l].psi);

                B[l].Fy = B[l].T  *cos(B[l].beta)
                        + B[l].P1 *cos(B[l].beta)
                        + B[l].Nf *sin(B[l].beta);

                B[l].Fz = B[l].Nr
                        + B[l].Nf *cos(B[l].beta)
                        - B[l].T  *sin(B[l].beta)
                        - B[l].P1 *sin(B[l].beta)
                        - krd*g*ml*sin(B[l].psi);
                break;

            case PR:
                B[l].Fx = B[l].Q
                        + B[l].K1
                        + g*ml *cos(B[l].psi);

                B[l].Fy = B[l].T  *cos(B[l].beta)
                        + B[l].P1 *cos(B[l].beta)
                        + B[l].Nf *sin(B[l].beta);

                B[l].Fz = B[l].Nr
                        + B[l].Nf *cos(B[l].beta)
                        - B[l].T  *sin(B[l].beta)
                        - B[l].P1 *sin(B[l].beta)
                        - krd*g*ml*sin(B[l].psi);
                break;
            }

            //Результирующие моменты в шарнирах, СКЛ
            //сила на штоке демпфера лопасти и момент силы отн.ВШ
            if(damper) B[l].Fdmp = DampherForce( DampherSpeed( B[l].ksi, B[l].dksi) );  else B[l].Fdmp = 0;
            B[l].M2D = B[l].Fdmp *d.h;
            //ГШ (СКЛ)
            if(damper)
                 B[l].M1y = B[l].Fx*(l_vsh-l_gsh) + B[l].M2D;
            else B[l].M1y = B[l].M1Q + B[l].M1K + B[l].M1Gy + B[l].M1Cy + B[l].M1Py;
            B[l].M1x = B[l].M1T + B[l].M1Gx + B[l].M1Cx + B[l].M1Px + B[l].M1Cz;//control:mustbe=0
            B[l].M1z = 0;
            //ВШ (СКЛ)
            B[l].M2x = B[l].M2T + B[l].M2Gx + B[l].M2Cx + B[l].M2Px + B[l].M2Cz;
            B[l].M2y = B[l].M2Q + B[l].M2K + B[l].M2Gy + B[l].M2Cy + B[l].M2Py + B[l].M2D;//control:mustbe=0
            B[l].M2z = 0;
            //ОШ (СКЛ): нет расчета шарнирного момента
            B[l].M3x = 0;////////?
            B[l].M3y = 0;////////?
            B[l].M3z = B[l].M3Ma + B[l].M3su + B[l].M3Cz + B[l].M3tr + B[l].M3dmp + B[l].M3iz;//control:mustbe=0

            //I: Результирующие силы и моменты от 1 лопасти в центре втулки, СКB
            B[l].Fhx =  B[l].Fz *cos(B[l].psi) + krd*B[l].Fx *sin(B[l].psi);
            B[l].Fhz =  krd*B[l].Fz *sin(B[l].psi) - B[l].Fx *cos(B[l].psi);
            B[l].Fhy =      B[l].Fy;
            B[l].Mhx = -krd*B[l].Fy *l_gsh *sin(B[l].psi);
            B[l].Mhz =      B[l].Fy *l_gsh *cos(B[l].psi);
            B[l].Mhy =      B[l].Fx *l_vsh    + B[l].M2Q;
        }

        /*/силы от 1 лопасти в центре втулки, СКB
        {
        //Q
        toHubSystem( B[l].psi,  B[l].Q*cos(B[l].ksi), -B[l].Q*sin(B[l].ksi),  fx, fz );// *cos(B[l].ksi)
        FxhQ += fx;
        FzhQ += fz;
        //FhT
        toHubSystem( B[l].psi,  0, -B[l].T*sin(B[l].beta),  fx, fz );//
        FxhT += fx;
        FzhT += fz;
        //FhP1
        //toHubSystem( B[l].psi,  0, -B[l].P1*sin(B[l].beta),  fx, fz );//
        toHubSystem( B[l].psi,  0, -B[l].P1*sin(B[l].beta),  fx, fz );//
        FxhP1 += fx;
        FzhP1 += fz;
        //FhP2
        //toHubSystem( B[l].psi,  B[l].P2, 0,  fx, fz );//
        toHubSystem( B[l].psi,  B[l].P2*cos(B[l].ksi), -B[l].P2*sin(B[l].ksi),  fx, fz );//
        FxhP2 += fx;
        FzhP2 += fz;
        //FhK1
        //toHubSystem( B[l].psi,  B[l].K1, 0,  fx, fz );//*cos(B[l].ksi)
        toHubSystem( B[l].psi,  B[l].K1*cos(B[l].ksi), -B[l].K1*sin(B[l].ksi),  fx, fz );//*cos(B[l].ksi)
        FxhK1 += fx;
        FzhK1 += fz;
        //FhNr
        //toHubSystem( B[l].psi,  0, B[l].Nr,  fx, fz );
        toHubSystem( B[l].psi,  B[l].Nr*sin(B[l].khi), B[l].Nr*cos(B[l].khi),  fx, fz );//B[l].M1Cy/(0.4*R-l_vsh)
        FxhNr += fx;
        FzhNr += fz;
        //FhNf
        //toHubSystem( B[l].psi,  0, B[l].Nf*cos(B[l].beta),  fx, fz );//
        toHubSystem( B[l].psi,  0, B[l].Nf*cos(B[l].beta),  fx, fz );//
        FxhNf += fx;
        FzhNf += fz;
        }//    */
        //суммарные усилия от всех лопастей в центре втулки, СКB
        Fhx_sum += B[l].Fhx;
        Fhy_sum += B[l].Fhy;
        Fhz_sum += B[l].Fhz;
        Mhx_sum += B[l].Mhx;
        Mhy_sum += B[l].Mhy;
        Mhz_sum += B[l].Mhz;
    }

    //корректировка на вес лопастей
    switch (ROTORID)
      {//так как вес лопастей не вычитается из взлетной массы
      case MR:  Fhy_sum += g*zl*ml;  break;
      case TR:  Fhz_sum += g*zl*ml;  break;
      case PR:  Fhz_sum += g*zl*ml;  break;
      }
    ////сумм.усилия на втулке в СКB
    //Fhx2sum = FxhQ + FxhT + FxhNr + FxhNf + FxhP1 + FxhK1 + FxhP2;//+ xhK2
    //Fhz2sum = FzhQ + FzhT + FzhNr + FzhNf + FzhP1 + FzhK1 + FzhP2;//+ zhK2
}

void RotorBase::blade_kinematic_dt(int l)//расчет параметров движения лопасти
{
    int q = 0;
    bool brk = false;
    B[l].deltabeta = 1;
    B[l].deltagama = 1;
    B[l].deltaksi  = 1;

    double beta, dbeta, d2beta, beta1, dbeta1;
    double gama, dgama, d2gama, gama1, dgama1;
    double ksi, dksi, d2ksi, dksi1, ksi1;
    double dfi7,dfi7_,fi7,fi7_, d2fi7;
    double psi = B[l].psi;
    //параметры на предыдущем шаге
    double   bet0 = B[l].beta_;  //  bet0 - угол взмаха на предыдущем шаге, рад
    double  dbet0 = B[l].dbeta_; // dbet0 - скорость взмаха на предыдущем шаге, рад [взмаха]/рад [оборота]
    double d2bet0 = B[l].d2beta_;//d2bet0 - ускорение взмаха на предыдущем шаге, рад [взмаха]/рад [оборота]^2
    double   ksi0 = B[l].ksi_;   //  ksi0 - угол качания на предыдущем шаге, рад
    double  dksi0 = B[l].dksi_;  // dksi0 - скорость качания на предыдущем шаге, рад [качания]/рад [оборота]
    double d2ksi0 = B[l].d2ksi_; //d2ksi0 - ускорение качания на предыдущем шаге, рад [качания]/рад [оборота]^2
    double   gam0 = B[l].gama_;  //  gam0 - угол закрутки на предыдущем шаге, рад
    double  dgam0 = B[l].dgama_; // dgam0 - скорость закрутки на предыдущем шаге, рад[закрутки]/рад [оборота]
    double d2gam0=d2gama= B[l].d2gama_;//d2gam0 - ускорение закрутки на предыдущем шаге, рад[закрутки]/рад [оборота]^2
    fi7 = fi7_ = B[l].fi7_;
    dfi7= dfi7_= B[l].dfi7_;

    //углы и скорости в первом приближении
    dbeta = dbet0 + d2bet0 *dpsi;
     beta  = bet0 + 0.5*(dbet0 + dbeta)*dpsi;
    dgama = dgam0 + d2gam0 *dpsi;
     gama  = gam0 + 0.5*(dgam0 + dgama)*dpsi;
    dksi  = dksi0 + d2ksi0 *dpsi;
     ksi   = ksi0 + 0.5*(dksi0 + dksi) *dpsi;
    //определение изменения угла установки сечения 07R
    for (int rr=0; rr<nSec; rr++)
    {
        seg = Segment(rr, psi, beta, dbeta, gama, dgama, dfi7);
        if(rr==sec07)  fi7 = seg.fi;
    }
    dfi7  = ( fi7 - fi7_)/dpsi;
    d2fi7 = (fi7 - fi7_ - dfi7_*dpsi)*2/dpsi/dpsi;//[СМ1,с.372(ф47)]
    //d2fi7 = (dfi7 - dfi7_)/dpsi;

    //ускорения в первом приближении
    switch (hubtype)
      {
      case 0://шарнирный(ОШ+ГШ+ВШ)
      case 1://шарнирный(ОШ+ГШ)
        d2beta = get_d2beta_hinge(psi, beta, gama, d2fi7+d2gama, get_IntegralMdT1(psi, beta, dbeta, gama, dgama, dfi7));
        d2ksi  = get_d2ksi_hinge( psi, beta, dbeta, ksi, dksi);
        break;
      case 2://качалка (2 лопасти) карданного типа (ОШ+СГШ)
        //d2beta = get_d2beta_kardan(psi, beta, 0, 0, get_IntegralMdT1(psi, beta, dbeta, 0, 0, 0), get_IntegralMdT1(psi+M_PI, -beta, -dbeta, 0, 0, 0));
        d2beta = get_d2beta_kardan(psi, beta, gama, d2fi7+d2gama, get_IntegralMdT1(psi, beta, dbeta, gama, dgama, dfi7), get_IntegralMdT1(psi+M_PI, -beta, -dbeta, -gama, -dgama, -dfi7));
        d2ksi  = 0;
        break;
      case 3://
        d2beta = 0;   dbeta = 0;   beta = 0;   B[l].deltabeta = 0;
        d2ksi = 0;   B[l].deltaksi = 0;
        break;
      }
    d2gama = get_d2gama( beta, d2beta, fi7, d2fi7, gama, dgama, dfi7, get_IntegralMdMa(psi, beta, dbeta, gama, dgama, dfi7) );

    //итерационное нахождение угла и скoрости взмаха
    while (B[l].deltabeta > 0.0001*prec)
    {
        dbeta1 = dbet0 + 0.5*(d2beta + d2bet0) *dpsi;
         beta1 =  bet0 + 0.5*( dbeta1 + dbet0) *dpsi;
        dgama1 = dgam0 + 0.5*(d2gama + d2gam0) *dpsi;
         gama1 =  gam0 + 0.5*( dgama1 + dgam0) *dpsi;
         dksi1 = dksi0 + 0.5*( d2ksi + d2ksi0) *dpsi;
          ksi1 =  ksi0 + 0.5*(  dksi1 + dksi0) *dpsi;
        //угол установки сечения 07R и скорость его изменения
        for (int rr=0; rr<nSec; rr++)
        {
            seg = Segment(rr, psi, beta1, dbeta1, gama1, dgama1, dfi7);
            if(rr==sec07)  fi7 = seg.fi;
        }
        dfi7  = ( fi7 - fi7_)/dpsi;
        d2fi7 = (fi7 - fi7_ - dfi7_*dpsi)*2/dpsi/dpsi;//[СМ1,с.372(ф47)]
        //d2fi7 = (dfi7 - dfi7_)/dpsi;

        switch (hubtype)
          {
          case 0://шарнирный(ОШ+ГШ+ВШ)
          case 1://шарнирный(ОШ+ГШ)
            d2beta = get_d2beta_hinge(psi, beta1, gama1, d2fi7+d2gama, get_IntegralMdT1(psi, beta1, dbeta1, gama1, dgama1, dfi7));
            d2ksi  = get_d2ksi_hinge( psi, beta, dbeta, ksi1, dksi1);
            break;
          case 2://качалка (2 лопасти) карданного типа (ОШ+СГШ)
            //d2beta = get_d2beta_kardan(psi, beta1, 0, 0, get_IntegralMdT1(psi, beta1, dbeta1, 0, 0, 0), get_IntegralMdT1(psi+M_PI, -beta1, -dbeta1, 0, 0, 0));
            d2beta = get_d2beta_kardan(psi, beta1, gama1, d2fi7+d2gama, get_IntegralMdT1(psi, beta1, dbeta1, gama1, dgama1, dfi7), get_IntegralMdT1(psi+M_PI, -beta1, -dbeta1, gama1, dgama1, dfi7));
            d2ksi = 0;
            break;
          case 3://
            d2beta = 0;
            d2ksi = 0;
            break;
          }
        d2gama = get_d2gama( beta1, d2beta, fi7, d2fi7, gama1, dgama1, dfi7, get_IntegralMdMa(psi, beta1, dbeta1, gama1, dgama1, dfi7) );

        B[l].deltabeta = fabs(beta1 - beta);
        B[l].deltagama = fabs(gama1 - gama);
        B[l].deltaksi  = fabs( ksi1 - ksi );

        beta   = beta1;
        dbeta  = dbeta1;
        if (beta < bet_min) {  beta = bet_min;  brk=true; }//  dbeta = 88;  d2beta = 888;
        if (beta > bet_max) {  beta = bet_max;  brk=true; }//  dbeta = 88;  d2beta = 888;

        gama   = gama1;
        dgama  = dgama1;
        if (gama < gam_min) {  gama = gam_min;  brk=true; }//  dgama = 88;  d2gama = 888;
        if (gama > gam_max) {  gama = gam_max;  brk=true; }//  dgama = 88;  d2gama = 888;

        ksi  = ksi1;
        dksi = dksi1;
        if (ksi < ksi_min+ksi_nom) {  ksi = ksi_min+ksi_nom;  brk=true; }//  dksi = 88;  d2ksi = 888;
        if (ksi > ksi_max+ksi_nom) {  ksi = ksi_max+ksi_nom;  brk=true; }//  dksi = 88;  d2ksi = 888;

        q++;
        if(q>999) brk=true;
        if(brk) break;
    }
    if(B[l].deltabeta > 0.0001*prec) warning=311;
    if(B[l].deltagama >  0.001*prec) warning=310;
    if(B[l].deltaksi  >  0.001*prec) warning=312;

    /*/параметры на текущем шаге
    dbeta = dbet0 + d2beta *dpsi;
     beta =  bet0 +  dbeta *dpsi;
    dgama = dgam0 + d2gama *dpsi;
     gama =  gam0 +  dgama *dpsi;
     dksi = dksi0 +  d2ksi *dpsi;
      ksi =  ksi0 +   dksi *dpsi;
      //угол установки сечения 07R и скорость его изменения
    for (int rr=0; rr<nSec; rr++)
    {
        seg = Segment(rr, psi, beta, dbeta, gama, dgama, dfi7);
        if(rr==sec07)  fi7 = seg.fi;
    }
     dfi7 = ( fi7 - fi7_)/dpsi;
    d2fi7 = (dfi7 -dfi7_)/dpsi;*/
    //сохраняем окончательные значения параметров
      B[l].beta   = beta;
     B[l].dbeta  = dbeta;
    B[l].d2beta = d2beta;
      B[l].gama   = gama;
     B[l].dgama  = dgama;
    B[l].d2gama = d2gama;
       B[l].ksi   =  ksi;
      B[l].dksi  =  dksi;
     B[l].d2ksi =  d2ksi;
       B[l].fi7   =  fi7;
      B[l].dfi7  =  dfi7;
     B[l].d2fi7 =  d2fi7;
}

QString RotorBase::getAverage()//средние значения выбранных параметров
{
    itcount++;
    psi_sr  += B[0].psi;
    ksi_sr  += B[0].ksi;
    bT_sr   += B[0].T ;
    bQ_sr   += B[0].Q ;
    bK1_sr  += B[0].K1;
    bNr_sr  += B[0].Nr;
    bNf_sr  += B[0].Nf;
    bP1_sr  += B[0].P1;
    bP2_sr  += B[0].P2;
    bF1y_sr += B[0].Fy;
    bF2x_sr += B[0].Fx;
    bF3z_sr += B[0].Fz;
    bFdmp_sr+= B[0].Fdmp;
    Fhx_sr  += Fhx_sum;    Fhy_sr  += Fhy_sum;    Fhz_sr  += Fhz_sum;
    Mhx_sr  += Mhx_sum;    Mhy_sr  += Mhy_sum;    Mhz_sr  += Mhz_sum;
    N_i     += ni_sum;
    N_p     += np_sum;
    alfa_sr += alpha;
    fi0_sr  += fi0;
    eta_sr  += eta;
    chi_sr  += chi;
    //Fbsx_sr += 0;//Fbsx;
    //Fbsz_sr += 0;//Fbsz;
    //средние значения за прошедший полный оборот
    if ( psih < psih_ )//end of full rotation   // B[0].psi0 < B[0].psi_
    {
        revcount++;//счетчик полных оборотов
        nextrev= 1;//
        //определяем средние   /itcount;
        psi_sr /=itcount;      alfa_sr /=itcount;
        eta_sr /=itcount;      chi_sr  /=itcount;
        fi0_sr /=itcount;      ksi_sr  /=itcount;
        bT_sr  /=itcount;      bQ_sr   /=itcount;
        bK1_sr /=itcount;      bFdmp_sr/=itcount;
        bNr_sr /=itcount;      bNf_sr  /=itcount;
        bP1_sr /=itcount;      bP2_sr  /=itcount;
        bF1y_sr/=itcount;      bF2x_sr /=itcount;      bF3z_sr /=itcount;
        Fhx_sr /=itcount;      Fhy_sr  /=itcount;      Fhz_sr  /=itcount;
        Mhx_sr /=itcount;      Mhy_sr  /=itcount;      Mhz_sr  /=itcount;
        N_i    /=itcount;      N_p     /=itcount;
        V_sr = V;        //Fbsx_sr/=itcount;        Fbsz_sr /=itcount;
        //остальные средние
        H_sr  = Fhx_sr;
        T_sr  = Fhy_sr;
        S_sr  = Fhz_sr;
        Mx_sr = Mhx_sr;
        My_sr = Mhy_sr;
        Mz_sr = Mhz_sr;
        Rxy   = sqrt(T_sr*T_sr + H_sr*H_sr);
        Ra    = sqrt(T_sr*T_sr + H_sr*H_sr + S_sr*S_sr);
        Ya    = T_sr*cos(alfa_sr) - H_sr*sin(alfa_sr);
        Xa    = T_sr*sin(alfa_sr) + H_sr*cos(alfa_sr);
        delta = atan2(Xa,Ya);//отклонение равнодействующей
        N_need= krd *My_sr *w /1000.0;//потр. мощность кВт
        N_vr  = -Xa *V_sr /1000.0;
        m_i   = N_i   *2000.0/(sigma*ro*F *pow(wR,3));
        m_p   = N_p   *2000.0/(sigma*ro*F *pow(wR,3));
        m_k   = N_need*2000.0/(sigma*ro*F *pow(wR,3));
        Ct    = T_sr*2/(ro*F*wR*wR );
        Ch    = H_sr*2/(ro*F*wR*wR );
        Cs    = S_sr*2/(ro*F*wR*wR );
        ty    =  Ya *2/(ro*F*wR*wR*sigma);
        tx    =  Xa *2/(ro*F*wR*wR*sigma);

        outres="";
        outres += "средние за оборот #" + ss.setNum(revcount)+ ":\n";
        outres += "psi= "    + ss.setNum(psi_sr *r2d,'f',1) + " грд\n";
        //outres += "alfa= "   + ss.setNum(alfa_sr*r2d,'f',1) + " грд\n";
        //outres += "fi0= "    + ss.setNum(fi0_sr *r2d,'f',1) + "\t";
        outres += "H= "      + ss.setNum(H_sr       ,'f',0) + "\t";                 //outres += "Fhx_sr   = " + ss.setNum(Fhx_sr     ,'f',1) + " Н\t(продольная сила)\n";
        outres += "T= "      + ss.setNum(T_sr       ,'f',0) + "\t";                            //outres += "Fhy_sr   = " + ss.setNum(Fhy_sr     ,'f',1) + " Н\t(тяга)\n";
        outres += "S= "      + ss.setNum(S_sr       ,'f',0) + " Н\n";                 //outres += "Fhz_sr   = " + ss.setNum(Fhz_sr     ,'f',1) + " Н\t(поперечная сила)\n";
        outres += "Mx= "     + ss.setNum(Mx_sr      ,'f',0) + "\t";//outres += "Mhx_sr   = " + ss.setNum(Mhx_sr     ,'f',1) + " Н*м\t(момент отн.продольной оси СКПС)\n";
        outres += "My= "     + ss.setNum(My_sr      ,'f',0) + "\t";               //outres += "Mhy_sr   = " + ss.setNum(Mhy_sr     ,'f',1) + " Н*м\t(крутящий момент)\n";
        outres += "Mz= "     + ss.setNum(Mz_sr      ,'f',0) + " Н*м\n";//outres += "Mhz_sr   = " + ss.setNum(Mhz_sr     ,'f',1) + " Н*м\t(момент отн.поперечной оси СКПС)\n";
        /*незадействованный вывод на экран
        //outres += "Fxa= "    + ss.setNum(Fxa_sr     ,'f',0) + "\t";
        //outres += "Fya= "    + ss.setNum(Fya_sr     ,'f',0) + "\t";
        //outres += "Fza= "    + ss.setNum(Fza_sr     ,'f',0) + " Н\n";
        //outres += "Mxa= "    + ss.setNum(Mxa_sr     ,'f',0) + "\t";
        //outres += "Mya= "    + ss.setNum(Mya_sr     ,'f',0) + "\t";
        //outres += "Mza= "    + ss.setNum(Mza_sr     ,'f',0) + " Н*м\n";
        //outres += "Rxy= "    + ss.setNum(Rxy        ,'f',0) + " Н \n";
        //outres += "Ra= "     + ss.setNum(Ra         ,'f',0) + " Н \n";
        //outres += "Ya= "     + ss.setNum(Ya         ,'f',0) + " Н \n";
        //outres += "Xa= "     + ss.setNum(Xa         ,'f',0) + " Н \n";
        //outres += "N_need= " + ss.setNum(N_need     ,'f',0) + " кВт\n";
        //outres += "N_vr= "   + ss.setNum(N_vr       ,'f',0) + "\t";
        //outres += "N_i= "    + ss.setNum(N_i        ,'f',0) + "\t";
        //outres += "N_p= "    + ss.setNum(N_p        ,'f',0) + " кВт\n";
        //outres += "Tv(ni)   = " + ss.setNum(B[0].ni    ,'f',3) + " кВт\t\n";
        //outres += "XaU(np)  = " + ss.setNum(B[0].np    ,'f',3) + " кВт\t\n";
        //outres += "m_i      = " + ss.setNum(m_i        ,'f',6) + " \t(коэффициент индуктивной мощности)\n";
        //outres += "m_p      = " + ss.setNum(m_p        ,'f',6) + " \t(коэффициент профильной мощности)\n";
        //outres += "m_k      = " + ss.setNum(m_k        ,'f',6) + " \t(коэффициент крутящего момента)\n";
        //outres += "Ct       = " + ss.setNum(Ct         ,'f',6) + " \t(коэффициент тяги)\n";
        //outres += "Kt       = " + ss.setNum(Kt         ,'f',6) + " \t(коэффициент тяги)\n";
        //outres += "ty       = " + ss.setNum(ty         ,'f',6) + " \t(коэффициент подъемной силы tya в полусвязанной СК)\n";
        //outres += "tx       = " + ss.setNum(tx         ,'f',6) + " \t(коэффициент пропульсивной силы txa в полусвязанной СК)\n";
        //outres += "Fbsx_sr  = " + ss.setNum(Fbsx_sr    ,'f',1) + " Н \t\n";
        //outres += "Fbsz_sr  = " + ss.setNum(Fbsz_sr    ,'f',1) + " Н \t\n";
        //outres += "b.T_sr= "  + ss.setNum(bT_sr      ,'f',1) + " Н\n";
        //outres += "b.Q_sr= "  + ss.setNum(bQ_sr      ,'f',1) + " Н\n";
        //outres += "b.Nr_sr= " + ss.setNum(bNr_sr     ,'f',1) + " Н\n";
        //outres += "b.K1_sr  = " + ss.setNum(bK1_sr     ,'f',1) + " Н\t\n";
        //outres += "b.Nf_sr  = " + ss.setNum(bNf_sr     ,'f',1) + " Н\t\n";
        //outres += "b.P1_sr  = " + ss.setNum(bP1_sr     ,'f',1) + " Н\t\n";
        //outres += "b.P2_sr  = " + ss.setNum(bP2_sr     ,'f',1) + " Н\t\n";
        //outres += "b.Fy_sr = " + ss.setNum(bF1y_sr    ,'f',1) + " Н\t\n";
        //outres += "b.Fx_sr = " + ss.setNum(bF2x_sr    ,'f',1) + " Н\t\n";
        //outres += "b.Fz_sr = " + ss.setNum(bF3z_sr    ,'f',1) + " Н\t\n";
        //outres += "b.Fdmpsr = " + ss.setNum(bFdmp_sr   ,'f',1) + " Н\t\n";

        //outres += "Mxa=    " + ss.setNum(Mxa_sr    ,'f',1) + " Н*м    \t- момент аэродинамических сил отн. оси Xн (в полусвязанной CK);\n";
        //outres += "Mza=    " + ss.setNum(Mza_sr    ,'f',1) + " Н*м    \t- момент аэродинамических сил отн. оси Zн (в полусвязанной CK);\n";
        //outres += "Mxj=    " + ss.setNum(Mxj_sr    ,'f',1) + " Н*м    \t- момент инерционных сил отн. оси Xн (в полусвязанной CK);\n";
        //outres += "Mzj=    " + ss.setNum(Mzj_sr    ,'f',1) + " Н*м    \t- момент инерционных сил отн. оси Zн (в полусвязанной CK);\n";
        //outres += "vt=     " + ss.setNum(vt        ,'f',3) + " м/с    \t- средняя индуктивная скорость осевая;\n";
        //outres += "vh=     " + ss.setNum(vh        ,'f',3) + " м/с    \t- средняя индуктивная скорость продольная;\n";
        //outres += "vs=     " + ss.setNum(vs        ,'f',3) + " м/с    \t- средняя индуктивная скорость боковая;\n";
        //outres += "delta=  " + ss.setNum(delta*r2d ,'f',5) + " грд    \t- угол наклона равнодействующей силы;\n";
        //outres += "lambda= " + ss.setNum(lambda_sr ,'f',5) + "        \t- ср. коэф. протекания;\n";
        //outres += "K_mu=   " + ss.setNum(K_mu      ,'f',5) + "        \t- \n";
        //outres += "mksigma=" + ss.setNum(m_k*sigma ,'f',5) + "        \t- коэффициент крутящего момента;\n";
        //outres += "\n";
     /* //Распределенные параметры лопасти (по сечению и по азимуту)
        {
        outres += "\nРаспределенные параметры лопасти (по сечению и по азимуту):\n";

        outres += "\nскорость обтекания Ux, м/с\n";
        outres += "_r\\psi";
        for (int az = 0; az <= nAz; az++)
            outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n";
        for (int i = 0; i < nSec; i++)
        {
            outres += ss.setNum(_r.at(i),'f',3);
            for (int az = 0; az <= nAz; az++)
                outres += "\t" + ss.setNum(b[0].Ux.at(az).at(i),'f',1);
            outres += "\n";
        }

        outres += "\nскорость обтекания Uy, м/с\n";
        outres += "_r\\psi";
        for (int az = 0; az <= nAz; az++)
            outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n";
        for (int i = 0; i < nSec; i++)
        {
            outres += ss.setNum(_r.at(i),'f',3);
            for (int az = 0; az <= nAz; az++)
                outres += "\t" + ss.setNum(b[0].Uy.at(az).at(i),'f',1);
            outres += "\n";
        }

        outres += "\nРезультирующая скорость обтекания U, м/с\n";
        outres += "_r\\psi";
        for (int az = 0; az <= nAz; az++)
            outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n";
        for (int i = 0; i < nSec; i++)
        {
            outres += ss.setNum(_r.at(i),'f',3);
            for (int az = 0; az <= nAz; az++)
                outres += "\t" + ss.setNum(b[0].U.at(az).at(i),'f',1);
            outres += "\n";
        }

        outres += "\nУгол установки сечения fi, градусы\n";
        outres += "_r\\psi";
        for (int az = 0; az <= nAz; az++)
            outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n";
        for (int i = 0; i < nSec; i++)
        {
            outres += ss.setNum(_r.at(i),'f',3);
            for (int az = 0; az <= nAz; az++)
                outres += "\t" + ss.setNum(b[0].fi.at(az).at(i)*r2d,'f',3);
            outres += "\n";
        }

        outres += "\nУгол притекания Fi, градусы\n";
        outres += "_r\\psi";
        for (int az = 0; az <= nAz; az++)
            outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n";
        for (int i = 0; i < nSec; i++)
        {
            outres += ss.setNum(_r.at(i),'f',3);
            for (int az = 0; az <= nAz; az++)
                outres += "\t" + ss.setNum(b[0].Fi.at(az).at(i)*r2d,'f',3);
            outres += "\n";
        }

        outres += "\nУгол атаки alpha_r, градусы\n";
        outres += "_r\\psi";
        for (int az = 0; az <= nAz; az++)
            outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n";
        for (int i = 0; i < nSec; i++)
        {
            outres += ss.setNum(_r.at(i),'f',3);
            for (int az = 0; az <= nAz; az++)
                outres += "\t" + ss.setNum(b[0].alfar.at(az).at(i)*r2d,'f',3);
            outres += "\n";
        }

        outres += "\nЧисло Маха M\n";
        outres += "_r\\psi";
        for (int az = 0; az <= nAz; az++)
            outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n";
        for (int i = 0; i < nSec; i++)
        {
            outres += ss.setNum(_r.at(i),'f',3);
            for (int az = 0; az <= nAz; az++)
                outres += "\t" + ss.setNum(b[0].M.at(az).at(i),'f',3);
            outres += "\n";
        }

        outres += "\nКоэффициент подъемной силы Cy\n";
        outres += "_r\\psi";
        for (int az = 0; az <= nAz; az++)
            outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n";
        for (int i = 0; i < nSec; i++)
        {
            outres += ss.setNum(_r.at(i),'f',3);
            for (int az = 0; az <= nAz; az++)
                outres += "\t" + ss.setNum(b[0].Cy.at(az).at(i),'f',3);
            outres += "\n";
        }

        outres += "\nКоэффициент профильного сопротивления Cx\n";
        outres += "_r\\psi";
        for (int az = 0; az <= nAz; az++)
            outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n";
        for (int i = 0; i < nSec; i++)
        {
            outres += ss.setNum(_r.at(i),'f',3);
            for (int az = 0; az <= nAz; az++)
                outres += "\t" + ss.setNum(b[0].Cx.at(az).at(i),'f',3);
            outres += "\n";
        }

        outres += "\nРаспределение погонной силы тяги dT/dr, [Н/м]\n";
        outres += "_r\\psi";
        for (int az = 0; az <= nAz; az++)
            outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n";
        for (int i = 0; i < nSec; i++)
        {
            outres += ss.setNum(_r.at(i),'f',3);
            for (int az = 0; az <= nAz; az++)
                outres += "\t" + ss.setNum(b[0].dT_dr.at(az).at(i),'f',1);
            outres += "\n";
        }

        outres += "\nРаспределение погонной силы сопротивления вращению dQ/dr, [Н/м]\n";

        outres += "_r\\psi";
        for (int az = 0; az <= nAz; az++)
            outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n";
        for (int i = 0; i < nSec; i++)
        {
            outres += ss.setNum(_r.at(i),'f',3);
            for (int az = 0; az <= nAz; az++)
                outres += "\t" + ss.setNum(b[0].dQ_dr.at(az).at(i),'f',1);
            outres += "\n";
        }
        }
    */
        /*//Параметры лопасти по азимуту
        {
        outres += "\nПараметры лопасти по азимуту";
        outres += "\n\tpsi, грд:";                          for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n\tbeta, грд[взм]:";                    for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].beta.at(az)*r2d,'f',3);
        outres += "\n\tdbeta/dpsi, рад[взм]/рад[об]:";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].dbeta.at(az),'f',3);
        outres += "\n\td2beta/dpsi^2, рад[взм]/рад^2[об]:"; for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].d2beta.at(az),'f',3);
        outres += "\n\tksi, грд[кач]:";                     for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].ksi.at(az)*r2d,'f',3);
        outres += "\n\tdksi/dpsi, рад[кач]/рад[об]:";       for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].dksi.at(az),'f',3);
        outres += "\n\td2ksi/dpsi^2, рад[кач]/рад^2[об]:";  for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].d2ksi.at(az),'f',3);

        outres += "\n\tT, Н:    ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].T.at(az),'f',1);
        outres += "\n\tTy, Н:   ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Ty.at(az),'f',1);
        outres += "\n\tQ, Н:    ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Q.at(az),'f',1);
        outres += "\n\tH, Н:    ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].H.at(az),'f',1);
        outres += "\n\tS, Н:    ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].S.at(az),'f',1);
        outres += "\n\tK1, Н:   ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].K1.at(az),'f',1);
        outres += "\n\tK2, Н:   ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].K2.at(az),'f',1);
        outres += "\n\tNr, Н:   ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Nr.at(az),'f',1);
        outres += "\n\tNf, Н:   ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Nf.at(az),'f',1);
        outres += "\n\tP1, Н:   ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].P1.at(az),'f',1);
        outres += "\n\tP2, Н:   ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].P2.at(az),'f',1);
        outres += "\n\tFdmp, Н: ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Fdmp.at(az),'f',1);
        outres += "\n\tMdmp, Н*м: ";    for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].M2D.at(az),'f',1);
        outres += "\n\tMxa+Mxj, Н*м: "; for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Mxa.at(az) + b[0].Mxj.at(az),'f',1);
        outres += "\n\tMza+Mzj, Н*м: "; for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Mza.at(az) + b[0].Mzj.at(az),'f',1);

        outres += "\n\nУсилия в шарнирах (СКЛ)";
        outres += "\n\tF2x, Н:  ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Fx.at(az),'f',1);
        outres += "\n\tF1y, Н:  ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Fy.at(az),'f',1);
        outres += "\n\tF3z, Н:  ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Fz.at(az),'f',1);

        outres += "\nМоменты в ГШ (СКЛ)";
        outres += "\n\tM1x, Н*м:";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].M1x.at(az),'f',1);
        outres += "\n\tM1y, Н*м:";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].M1y.at(az),'f',1);
        outres += "\n\tM1z, Н*м:";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].M1z.at(az),'f',1);

        outres += "\nМоменты в ВШ (СКЛ)";
        outres += "\n\tM2x, Н*м:";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].M2x.at(az),'f',1);
        outres += "\n\tM2y, Н*м:";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].M2y.at(az),'f',1);
        outres += "\n\tM2z, Н*м:";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].M2z.at(az),'f',1);

        outres += "\nМоменты в ОШ (СКЛ)";
        outres += "\n\tM3x, Н*м:";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].M3x.at(az),'f',1);
        outres += "\n\tM3y, Н*м:";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].M3y.at(az),'f',1);
        outres += "\n\tM3z, Н*м:";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].M3z.at(az),'f',1);

        outres += "\n\nCилы и моменты от одной лопасти за один оборот (в полусвязанной CK)";
        outres += "\n\tазимут, грд:";   for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n\tFxh_0, Н:  ";    for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Fhx.at(az),'f',1);
        outres += "\n\tFyh_0, Н:  ";    for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Fhy.at(az),'f',1);
        outres += "\n\tFzh_0, Н:  ";    for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Fhz.at(az),'f',1);
        outres += "\n\tMxh_0, Н*м:";    for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Mhx.at(az),'f',1);
        outres += "\n\tMyh_0, Н*м:";    for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Mhy.at(az),'f',1);
        outres += "\n\tMzh_0, Н*м:";    for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Mhz.at(az),'f',1);

        outres += "\n\nСуммарные силы и моменты на втулке от всех лопастей за один оборот (в полусвязанной СК):";
        outres += "\n\tазимут, грд:";   for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n\tFxh_sum, Н:";    for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(Fhx_S.at(az),'f',1);
        outres += "\n\tFyh_sum, Н:";    for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(Fhy_S.at(az),'f',1);
        outres += "\n\tFzh_sum, Н:";    for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(Fhz_S.at(az),'f',1);
        outres += "\n\tMxh_sum, Н*м:";  for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(Mhx_S.at(az),'f',1);
        outres += "\n\tMyh_sum, Н*м:";  for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(Mhy_S.at(az),'f',1);
        outres += "\n\tMzh_sum, Н*м:";  for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(Mhz_S.at(az),'f',1);
        outres += "\n";
        outres += "\n\txhSum, Н:";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(xhSum.at(az),'f',1);////////////////////////
        outres += "\n\tzhSum, Н:";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(zhSum.at(az),'f',1);////////////////////////
        //outres += "\n\tFbsx, Н: ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(Fbsx .at(az),'f',1);////////////////////////
        //outres += "\n\tFbsz, Н: ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(Fbsz .at(az),'f',1);////////////////////////
        }
        ///
        {
        outres += "\n\nСилы от всех лопастей на втулке винта (в полусвязанной СК):";
        outres += "\npsi: ";   outres += "\t" + ss.setNum(psi0 *r2d,'f',1);
        outres += "\nxhQ: ";   outres += "\t" + ss.setNum(xhQ  ,'f',1);
        outres += "\nxhT: ";   outres += "\t" + ss.setNum(xhT  ,'f',1);
        outres += "\nxhNr:";   outres += "\t" + ss.setNum(xhNr ,'f',1);
        outres += "\nxhNf:";   outres += "\t" + ss.setNum(xhNf ,'f',1);
        outres += "\nxhP1:";   outres += "\t" + ss.setNum(xhP1 ,'f',1);
        outres += "\nxhP2:";   outres += "\t" + ss.setNum(xhP2 ,'f',1);
        outres += "\nxhK1:";   outres += "\t" + ss.setNum(xhK1 ,'f',1);
        outres += "\n";
        outres += "\npsi: ";   outres += "\t" + ss.setNum(psi0 *r2d,'f',1);
        outres += "\nzhQ: ";   outres += "\t" + ss.setNum(zhQ  ,'f',1);
        outres += "\nzhT: ";   outres += "\t" + ss.setNum(zhT  ,'f',1);
        outres += "\nzhNr:";   outres += "\t" + ss.setNum(zhNr ,'f',1);
        outres += "\nzhNf:";   outres += "\t" + ss.setNum(zhNf ,'f',1);
        outres += "\nzhP1:";   outres += "\t" + ss.setNum(zhP1 ,'f',1);
        outres += "\nzhP2:";   outres += "\t" + ss.setNum(zhP2 ,'f',1);
        outres += "\nzhK1:";   outres += "\t" + ss.setNum(zhK1 ,'f',1);
        }
        outres += "\n\nСредние за оборот от всех лопастей (в начале полусвязанной СК (центр втулки винта)):";
        outres += "\n\tFxh=  " + ss.setNum( Fhx_sr,'f',1) + "\tсредняя за оборот продольная сумм.сила на втулке, Н";
        outres += "\n\tFyh=  " + ss.setNum( Fhy_sr,'f',1) + "\tсредняя за оборот вертикальная сумм.сила на втулке, Н";
        outres += "\n\tFzh=  " + ss.setNum( Fhz_sr,'f',1) + "\tсредняя за оборот поперечная сумм.сила на втулке, Н";
        outres += "\n\tMxh=  " + ss.setNum( Mhx_sr,'f',1) + "\tсредний за оборот сумм.момент на втулке отн.продольной оси, Н*м";
        outres += "\n\tMyh=  " + ss.setNum( Mhy_sr,'f',1) + "\tсредний за оборот сумм.момент на втулке отн.вертикальной оси, Н*м";
        outres += "\n\tMzh=  " + ss.setNum( Mhz_sr,'f',1) + "\tсредний за оборот сумм.момент на втулке отн.поперечной оси, Н*м";
        outres += "\n";
        outres += "\n\txhSum= "+ ss.setNum(xhSum_sr,'f',1) + "\tсредняя за оборот продольная сумм.сила на втулке, Н";
        outres += "\n\tzhSum= "+ ss.setNum(zhSum_sr,'f',1) + "\tсредняя за оборот вертикальная сумм.сила на втулке, Н";
        //outres += "\n\tFbsx= " + ss.setNum(Fbsx_sr,'f',1) + "\tсредняя за оборот продольная компонента центробежной силы от смещения цт лопастей, Н";
        //outres += "\n\tFbsz= " + ss.setNum(Fbsz_sr,'f',1) + "\tсредняя за оборот поперечная компонента центробежной силы от смещения цт лопастей, Н";
        outres += "\n--------------------------------------------------------------------------------";
    // */
        //обнуление выбранных параметров
        itcount =0;
        psi_sr  =0;        alfa_sr =0;
        eta_sr  =0;        chi_sr  =0;
        fi0_sr  =0;        ksi_sr  =0;
        bT_sr   =0;        bQ_sr   =0;
        bK1_sr  =0;        bNr_sr  =0;        bNf_sr  =0;
        bP1_sr  =0;        bP2_sr  =0;        bFdmp_sr=0;
        bF1y_sr =0;        bF2x_sr =0;        bF3z_sr =0;
        Fhx_sr  =0;        Fhy_sr  =0;        Fhz_sr  =0;
        Mhx_sr  =0;        Mhy_sr  =0;        Mhz_sr  =0;
        N_i     =0;        N_p     =0;
    }
    //в данный момент
    logstr += "alfa= "+ ss.setNum(alpha *r2d,'f',1) + " грд\n";
    logstr += "Fhx= " + ss.setNum(Fhx_sum,'f',0) + "\t";
    logstr += "Fhy= " + ss.setNum(Fhy_sum,'f',0) + "\t";
    logstr += "Fhz= " + ss.setNum(Fhz_sum,'f',0) + " Н\n";
    logstr += "Mhx= " + ss.setNum(Mhx_sum,'f',0) + "\t";
    logstr += "Mhy= " + ss.setNum(Mhy_sum,'f',0) + "\t";
    logstr += "Mhz= " + ss.setNum(Mhz_sum,'f',0) + " Н·м\n";
    logstr += "\n";
    logstr += outres;
    logstr += "\n";
    logstr += "blade#1:\n";
    logstr += "fi7=  " + ss.setNum(B[0].fi7   *180/M_PI,'f',4) + "°\n"; // logstr += "d2fi7=  " + ss.setNum(B[0].d2fi7 *w*w*180/M_PI,'f',1) + "°/c2\n";
    logstr += "gama= " + ss.setNum(B[0].gama  *180/M_PI,'f',4) + "°\n"; // logstr += "d2gama= " + ss.setNum(B[0].d2gama*w*w*180/M_PI,'f',1) + "°/c2\n";
    logstr += "beta= " + ss.setNum(B[0].beta  *180/M_PI,'f',4) + "°\n"; // logstr += "d2beta= " + ss.setNum(B[0].d2beta*w*w*180/M_PI,'f',1) + "°/c2\n";
    logstr += "ksi=  " + ss.setNum(B[0].ksi   *180/M_PI,'f',4) + "°\n"; // logstr += "d2ksi=  " + ss.setNum(B[0].d2ksi *w*w*180/M_PI,'f',1) + " грд/c2\n";
    //logstr += "dfi7=  " + ss.setNum(B[0].dfi7           ,'f',3) + "\n";
    //logstr += "dgama= " + ss.setNum(B[0].dgama *180/M_PI,'f',3) + "\n";
    //logstr += "d2beta= "+ ss.setNum(B[0].d2beta*180/M_PI,'f',3) + "\n";
    logstr += "T=    "  + ss.setNum(B[0].T              ,'f',1) + " N\n";
    //logstr += "Ma=   "  + ss.setNum(B[0].Ma             ,'f',1) + " N*m\n";
    logstr += "Ma=   "  + ss.setNum(B[0].M3Ma           ,'f',1) + " N*m\n";
    logstr += "M3Cz= "  + ss.setNum(B[0].M3Cz           ,'f',1) + " N*m\n";
    logstr += "M3tr= "  + ss.setNum(B[0].M3tr           ,'f',1) + " N*m\n";
    logstr += "M3dmp="  + ss.setNum(B[0].M3dmp          ,'f',1) + " N*m\n";
    logstr += "M3iz= "  + ss.setNum(B[0].M3iz           ,'f',1) + " N*m\n";
    logstr += "M3su= "  + ss.setNum(B[0].M3su           ,'f',1) + " N*m\n";
    logstr += "Fpov= "  + ss.setNum(B[0].Fpov           ,'f',1) + " N\n";
    logstr += "dpov= "  + ss.setNum(B[0].delta_pov*1000 ,'f',3) + " mm\n";
    logstr += "\n";
    //logstr += "Icb  = " + ss.setNum(Icb            ,'f',6) + "\n";
    //logstr += "Iosh = " + ss.setNum(I_osh          ,'f',6) + "\n";
    if(B[0].beta)
    logstr += "delta_beta= " + ss.setNum(B[0].deltabeta*100/B[0].beta,'f',5) + "%\n"; else logstr += "delta_beta= 0.00000%\n";
    if(B[0].gama)
    logstr += "delta_gama= " + ss.setNum(B[0].deltagama*100/B[0].gama,'f',5) + "%\n"; else logstr += "delta_gama= 0.00000%\n";
    if(B[0].ksi)
    logstr += "delta_ksi = " + ss.setNum(B[0].deltaksi *100/B[0].ksi ,'f',5) + "%\n"; else logstr += "delta_ksi = 0.00000%\n";

    //logstr += "\n";
    //logstr += "beta1= " + ss.setNum(B[0].beta  *180/M_PI,'f',4) + "°\n"; // logstr += "d2beta= " + ss.setNum(B[0].d2beta*w*w*180/M_PI,'f',1) + "°/c2\n";
    //logstr += "beta2= " + ss.setNum(B[1].beta  *180/M_PI,'f',4) + "°\n"; // logstr += "d2beta= " + ss.setNum(B[0].d2beta*w*w*180/M_PI,'f',1) + "°/c2\n";
    //if(zl>2)
    //logstr += "beta3= " + ss.setNum(B[2].beta  *180/M_PI,'f',4) + "°\n"; // logstr += "d2beta= " + ss.setNum(B[0].d2beta*w*w*180/M_PI,'f',1) + "°/c2\n";

    //stiff_su = k_pov *r_pov*r_pov;
    //double potEnergy = 0.5 *B[0].gama*B[0].gama *stiff_su;///0.5*(alfa-alfa0)*rotStiff*(alfa-alfa0) + flapStiff*pow(y,2)/2;
    //double kinEnergy = I_osh *pow(B[0].dgama*w,2)/2;  ///mPog* pow(velocY,2)/2 + momInerPog*pow(rotVeloc,2)/2;
    //logstr += "potE= "  + ss.setNum(potEnergy          ,'f',3) + " Дж\n";
    //logstr += "kinE= "  + ss.setNum(kinEnergy          ,'f',3) + " Дж\n";
    //logstr += "Energy= "  + ss.setNum(potEnergy+kinEnergy,'f',3) + " Дж\n";
    //logstr += "\n";

    //logstr += "M3Cz_1= "  + ss.setNum(-(B[0].d2beta+B[0].beta) *w*w *Icb    ,'f',3) + " N*m\n";
    //logstr += "M3Cz_2= "  + ss.setNum( w*w *I_osh*krutka           ,'f',3) + " N*m\n";
    //logstr += "Icb= "  + ss.setNum( Icb    ,'f',8) + "\n";
    ///
    //logstr += "zakrut= " + ss.setNum(zakrut   ) + "\n";

    //logstr += "\n";
    //logstr += "bladesystem:\n";
    //logstr += "psih= "  + ss.setNum(psih*180/M_PI,'f',1) + " грд\n";
    //logstr += "alfact= "+ ss.setNum(act *180/M_PI,'f',1) + " грд\n";
    //logstr += "lct= "   + ss.setNum(dctbs   *1000,'f',1) + " мм\n";
    //logstr += "ctMRx= " + ss.setNum(ctMRx   *1000,'f',1) + " мм\n";
    //logstr += "ctMRz= " + ss.setNum(ctMRz   *1000,'f',1) + " мм\n";
    //logstr += "Fbs= "   + ss.setNum(Fbs          ,'f',0) + "\n";
    //logstr += "Fbsxr= " + ss.setNum(Fbsxr        ,'f',0) + "\t";
    //logstr += "Fbszr= " + ss.setNum(Fbszr        ,'f',0) + "\n";
    //logstr += "Fbsx= "  + ss.setNum(Fbsx         ,'f',0) + "\t";
    //logstr += "Fbsz= "  + ss.setNum(Fbsz         ,'f',0) + "\n";
    logstr += "\n\n";

return outres;
}



// расчет одного оборота (по заданному углу dpsi): /////////////////////////////////////////

void RotorBase::ClassicSpecified(double _alpha, double _fi0)//поиск сил по заданным условиям
{
    CALCMODE = FORCES;
    alpha  = _alpha;
    fi0  =  _fi0;
    //(V == 0 || alpha == 0.5*M_PI || alpha == -0.5*M_PI) ? (FLIGHTMODE = AXIAL) : (FLIGHTMODE = HORIZONTAL);//осевой режим
    getIniFlow();//добавил 20210716

    int q = 0;
    double T1, T2, deltaT = 50;
    if (fi0<0) T_sr = -500;
    else T_sr = 500;///500;
    H_sr = 0;  S_sr = 0;  T1 = T_sr;
    //
    QString qsn;
    writeln(&this->vsh,  "ClassicSpecified starts working");
    writeln(&this->vsh1, "ClassicSpecified starts working");
    //
    while (deltaT > prec)///0.01...0.001
    {
        q++;
        //
        qsn.setNum(q);
        writeln(&this->vsh, "q="+qsn);
        writeln(&this->vsh1, "q="+qsn);
        //
        Core();
        T2 = T_sr;
        deltaT = fabs((T2-T1)/T2);
        if(deltaT > prec) T1 = T_sr = 0.2*(T2+4*T1);
        if(q>99)
        {
            T_sr = 0;
            Core();
            deltaT = 0;
            warning = 313;
        }
    }
    //
    writeln(&this->vsh,  "ClassicSpecified finishes working");
    writeln(&this->vsh1, "ClassicSpecified finishes working");
    //
}

void RotorBase::ClassicForceSynth()//расчет сил и моментов на винте по известному закону движения лопасти
{
    double _psi_,_beta_,_dbeta_,_d2beta_,_d2ksi_,  _gama_,_dgama_,_d2gama_, _fi7_,_dfi7_,_d2fi7_,  v,  integral;
    vector<double> tmp_dMQ_dr(nSec), tmp_dM_y1_Q_dr(nSec), tmp_dTv_dr(nSec), tmp_dXaU_dr(nSec);
    vector<double> tmp(nAz+1);
    vector<vector<double>> tmp2(nAz+1);
    dMQ_dr = dM_y1_Q_dr = dTv_dr = dXaU_dr = tmp2;
    Fhx_S = Fhz_S = Fhy_S = Mhx_S = Mhy_S = Mhz_S = tmp;
    xhQ= zhQ= xhT= zhT= xhNr= zhNr= xhNf= zhNf= xhP1= zhP1= xhP2= zhP2= xhK1= zhK1= xhSum= zhSum= tmp;
    for (int l = 0; l < zl; l++)
    {
        b[l].Nr  = b[l].Nf  = b[l].P1  = b[l].P2  = b[l].K1  = tmp;
        b[l].Paxz = b[l].Payy = b[l].Pay  = b[l].Pexz = b[l].Pkxz = tmp;
        b[l].Fx  = b[l].Fy  = b[l].Fz  = b[l].FNx = b[l].FNz = tmp;
        b[l].MQ  = b[l].Tv  = b[l].XaU = b[l].Mza = b[l].Mxa = b[l].Mzj = b[l].Mxj = b[l].M2D = b[l].Fdmp= tmp;
        b[l].M1T = b[l].M1Q = b[l].M1Px= b[l].M1Py= b[l].M1K = b[l].M1Gx= b[l].M1Gy= b[l].M1Cx= b[l].M1Cy= b[l].M1Cz= tmp;
        b[l].M2T = b[l].M2Q = b[l].M2Px= b[l].M2Py= b[l].M2K = b[l].M2Gx= b[l].M2Gy= b[l].M2Cx= b[l].M2Cy= b[l].M2Cz= tmp;
        b[l].M3Ma= b[l].M3Cz= b[l].M3su= b[l].M3tr= b[l].M3dmp=b[l].M3iz= b[l].Fpov= b[l].delta_pov= tmp;
        b[l].M1x = b[l].M1y = b[l].M1z = b[l].M2x = b[l].M2y = b[l].M2z = b[l].M3x = b[l].M3y = b[l].M3z = tmp;
        b[l].Fhx = b[l].Fhz = b[l].Fhy = b[l].Mhx = b[l].Mhy = b[l].Mhz = tmp;
    }

    //сосредоточенные силы и моменты на азимуте по известному закону движения лопасти
    for (int l = 0; l < zl; l++)
    {
        for (int az = 0; az <= nAz; az++)
        {
            _psi_   = b[l].psi.at(az);
            _fi7_   = b[l].fi7.at(az);
            _dfi7_  = b[l].dfi7.at(az);
            _d2fi7_ = b[l].d2fi7.at(az);
            _beta_  = b[l].beta.at(az);
            _dbeta_ = b[l].dbeta.at(az);
            _d2beta_= b[l].d2beta.at(az);
            _d2ksi_ = b[l].d2ksi.at(az);
            _gama_  = b[l].gama.at(az);
            _dgama_ = b[l].dgama.at(az);
            _d2gama_= b[l].d2gama.at(az);
            b[l].khi.at(az) =  b[l].ksi.at(az) * (1-l_vsh/(0.505*R));

            for (int sec = 0; sec < nSec; sec++)
            {
                v = vt *( sqrt(1.5)*pow(_r.at(sec),0.5)*f1 + (1 + K_mu*_r.at(sec)*cos(_psi_))*f2 );
                //T*v для инд. мощности Ni
                tmp_dTv_dr.at(sec) = fabs( b[l].dT_dr.at(az).at(sec) *cos(_beta_) *v);
                //Xa*U для проф. мощности Np
                tmp_dXaU_dr.at(sec) = 0.5*ro*bh.at(sec) *b[l].cx.at(az).at(sec) *pow(b[l].U.at(az).at(sec),3);
                if (b[l].Ux.at(az).at(sec) < 0)  tmp_dXaU_dr.at(sec) = -1*tmp_dXaU_dr.at(sec);
                //массивы распределенных удельных моментов
                tmp_dMQ_dr.at(sec)     = b[l].dQ_dr.at(az).at(sec) *((r.at(sec)-l_gsh)*cos(_beta_) + l_gsh);
                tmp_dM_y1_Q_dr.at(sec) = b[l].dQ_dr.at(az).at(sec) *((r.at(sec)-l_gsh)*cos(_beta_));
            }
            dMQ_dr.at(az)    = tmp_dMQ_dr;
            dM_y1_Q_dr.at(az)= tmp_dM_y1_Q_dr;//не используется
            dTv_dr.at(az)    = tmp_dTv_dr;
            dXaU_dr.at(az)   = tmp_dXaU_dr;
            //для расчета компонентов мощностей
            b[l].Tv .at(az)  = get_Integral( dTv_dr.at(az), r);//for инд. мощности Ni
            b[l].XaU.at(az)  = get_Integral(dXaU_dr.at(az), r);//for проф. мощности Np

            //компоненты усилий в шарнирах:
            //центробежная сила от вращения винта Н
            b[l].Nr.at(az)  = w*w* ( KA1 *cos(_beta_) + ml*l_gsh );//+
            //центробежная сила от маховых движений Н
            b[l].Nf.at(az)  = KA1 *w*w* _dbeta_*_dbeta_;//+
            //силa Кориолиса от взмаха
            b[l].K1.at(az)  = -2*krd*KA1 *w*w* _dbeta_ *sin(_beta_);//+
            //силa Кориолиса от качания
            //b[l].K2.at(az)  = -2*KA2 *w*w* b[l].dksi.at(az) *sin(b[l].ksi.at(az));
            //сила инерции, Н
            b[l].P1.at(az)  = -KA1 *w*w* _d2beta_;//+ взмах
            b[l].P2.at(az)  = -KA2 *w*w* _d2ksi_;//+ качание

            //сила инерции в плоскости винта от линейного перемещения вдоль продольной и поперечной осей CKПС
            b[l].Paxz.at(az) = -ml*( a_x*sin(_psi_) - a_z*cos(_psi_) );
            //cила инерции от линейного перемещения вдоль оси НВ
            b[l].Pay.at(az)  = -ml*a_y;
            //сила инерции от вращения НВ вокруг продольной и поперечной осей CKПС
            b[l].Pexz.at(az) = S_gsh*( e_x*sin(_psi_) - e_z*cos(_psi_) );
            //силы инерции Кориолиса, возникающие от вращения НВ вокруг продольной и поперечной осей CKПС на азимуте psi
            b[l].Pkxz.at(az) = 2*S_gsh*w*( krd*w_x*cos(_psi_) + w_z*sin(_psi_) );
            //общая cила инерции в плоскости тяги
            b[l].Payy.at(az) = b[l].Pay.at(az) + b[l].Pexz.at(az) + b[l].Pkxz.at(az);

            //момент силы инерции от линейного перемещения вдоль оси НВ
            double MPay  = -a_y*KA1*cos(_beta_);
            //момент силы инерции от вращения НВ вокруг продольной и поперечной осей CKПС
            double MPexz = KC1*cos(_beta_)*( e_x*sin(_psi_) - e_z*cos(_psi_) );
            //момент силы инерции Кориолиса, возникающие от вращения НВ вокруг продольной и поперечной осей CKПС на азимуте psi
            double MPkxz = KC1*cos(_beta_)*( krd*w_x*cos(_psi_) + w_z*sin(_psi_) )*2*w;
            //момент силы инерции в плоскости винта от линейного перемещения вдоль продольной и поперечной осей CKПС
            double MPaxz = -KA2*cos(_beta_)*( a_x*sin(_psi_) - a_z*cos(_psi_) );


            ///ГШ (СКЛ)
            //момент тяги лопасти отн.ГШ
            b[l].M1T.at(az)  = -get_IntegralMdT1( _psi_, _beta_, _dbeta_, _gama_, _dgama_, _dfi7_);//+
            //момент силы сопротивления лопасти отн.ГШ (Yл)
            b[l].M1Q.at(az)  =  krd*get_IntegralMdQ1( _psi_, _beta_, _dbeta_, _gama_, _dgama_, _dfi7_);//+ не используется
            //момент силы Кориолиса отн.ГШ, Н*м
            b[l].M1K.at(az)  = -krd*KB1 *w*w* _dbeta_ *sin(2*_beta_);//+
            //момент центробежной силы
            b[l].M1Cx.at(az) =  0.5*KB1*w*w*sin(2*_beta_) + KA1*l_gsh*w*w*sin(_beta_);//+
            b[l].M1Cy.at(az) = -b[l].Nr.at(az) *l_gsh *sin(b[l].khi.at(az));//+ не используется
            //момент инерционных+центробежных сил
            //if ( I_osh && k_pov && zakrut )
            b[l].M1Cz.at(az) = (_d2fi7_+_d2gama_ + _gama_) *w*w *Icb;// _fi7_+
            //else b[l].M1Cz.at(az) = 0;
            //момент инерционной силы
            b[l].M1Px.at(az) = KB1 *w*w*_d2beta_  + MPay + MPexz + MPkxz;
            b[l].M1Py.at(az) = -KE *w*w* _d2ksi_  + MPaxz;//+ не используется

            ///ВШ (СКЛ)
            //момент тяги лопасти отн.ВШ
            b[l].M2T.at(az)  = -get_IntegralMdT2( _psi_, _beta_, _dbeta_, _gama_, _dgama_, _dfi7_);
            //момент силы сопротивления лопасти отн.ВШ, Н*м
            b[l].M2Q.at(az)  =  krd*get_IntegralMdQ2( _psi_, _beta_, _dbeta_, _gama_, _dgama_, _dfi7_);//+
            //момент силы Кориолиса отн.ВШ, Н*м
            b[l].M2K.at(az)  = -krd*KE *w*w* sin(2*_beta_) *_dbeta_;//+
            //момент центробежной силы
            b[l].M2Cx.at(az) =  0.5*KE*w*w*sin(2*_beta_) + KA2*l_gsh*w*w*sin(_beta_);//+
            b[l].M2Cy.at(az) = -b[l].Nr.at(az) *l_vsh *sin(b[l].khi.at(az));//+
            //момент инерционных+центробежных сил
            b[l].M2Cz.at(az) = b[l].M1Cz.at(az);
            //момент инерционной силы
            b[l].M2Px.at(az) =  KE *w*w*_d2beta_  + MPay + MPexz + MPkxz;
            b[l].M2Py.at(az) = -KB2*w*w* _d2ksi_  + MPaxz;//+

            ///OШ (СКЛ)
            //момент аэродинамических сил отн.ОШ
            b[l].M3Ma.at(az) = get_IntegralMdMa(_psi_, _beta_, _dbeta_, _gama_, _dgama_, _dfi7_);
            //момент силы в тяге системы управления
            b[l].delta_pov.at(az) = _gama_*r_pov;
            //stiff_su = k_pov*r_pov*r_pov;//Жорсткість системи управління кутова навколо осьового шарніра
            //b[l].M3su.at(az) = -_gama_* stiff_su;
            //b[l].Fpov.at(az) = b[l].M3su.at(az) / r_pov;
            b[l].Fpov.at(az) = k_pov * b[l].delta_pov.at(az);
            b[l].M3su.at(az) = -b[l].Fpov.at(az) * r_pov;
            //момент инерционных+центробежных сил
            b[l].M3Cz.at(az) = -(_d2beta_+_beta_) *w*w *Icb ;//+ w*w *I_osh*krutka; //I_osh*krutka=get_Integral(_Jmdfi_dr, r);
            //момент сил трения
            b[l].M3tr.at(az) = (_dfi7_+_dgama_>=0)?( -f_pod * r_pod * b[l].Nr.at(az) ):( f_pod * r_pod * b[l].Nr.at(az) );
            b[l].M3dmp.at(az) = gamaDamph(_dgama_);//_dfi7_+
            //момент сил инерции закрутки
            b[l].M3iz.at(az) = -( _d2fi7_+_d2gama_ + _fi7_+_gama_ )*(w*w*I_osh);
            //}
            if ( I_osh==0 || k_pov==0 )
            {
                b[l].Fpov.at(az) = 0;
                b[l].M3su.at(az) = 0;
                b[l].M3Cz.at(az) = 0;
                b[l].M3tr.at(az) = 0;
                b[l].M3dmp.at(az)= 0;
                b[l].M3iz.at(az) = 0;
            }

            //момент веса лопасти
            switch (ROTORID)
              {
              case MR://неплохо бы учесть перераспределение от крена+тангажа...
                b[l].M1Gx.at(az) = g*KA1 *cos(_beta_);
                b[l].M2Gx.at(az) = g*KA2 *cos(_beta_);
                b[l].M1Gy.at(az) = 0;
                b[l].M2Gy.at(az) = 0;
                break;
              case TR:
                b[l].M1Gx.at(az) = -krd*g*KA1 *sin(_beta_) *sin(_psi_);
                b[l].M2Gx.at(az) = -krd*g*KA2 *sin(_beta_) *sin(_psi_);
                b[l].M1Gy.at(az) =      g*KA1 *sin(_beta_) *cos(_psi_);
                b[l].M2Gy.at(az) =      g*KA2 *sin(_beta_) *cos(_psi_);
                break;
              case PR:
                b[l].M1Gx.at(az) = -krd*g*KA1 *sin(_beta_) *sin(_psi_);
                b[l].M2Gx.at(az) = -krd*g*KA2 *sin(_beta_) *sin(_psi_);
                b[l].M1Gy.at(az) =      g*KA1 *sin(_beta_) *cos(_psi_);
                b[l].M2Gy.at(az) =      g*KA2 *sin(_beta_) *cos(_psi_);
                break;
              }

            //СКЛ, отн.центра втулки
            //момент силы сопротивления лопасти отн.оси винта (он же крутящий момент)
            b[l].MQ.at(az)  =  krd*get_Integral(dMQ_dr.at(az), r);

            //СКВ, отн.центра втулки
            //моменты аэрод. сил
            b[l].Mza.at(az) =      b[l].T.at(az) *cos(_beta_) *l_gsh *cos(_psi_);//продольный
            b[l].Mxa.at(az) = -krd*b[l].T.at(az) *cos(_beta_) *l_gsh *sin(_psi_);//боковой
            //момент силы инерции взмаха
            b[l].Mzj.at(az) =     b[l].P1.at(az) *cos(_beta_) *l_gsh *cos(_psi_);//продольный -1*
            b[l].Mxj.at(az) =-krd*b[l].P1.at(az) *cos(_beta_) *l_gsh *sin(_psi_);//боковой
        }
    }

    //расчет нагрузок в шарнирах и в центре втулки с учетом инерционных усилий
    getHingeForces();

    //суммарные силы и моменты от всех лопастей в центре втулки НА АЗИМУТЕ az!
    for (int az = 0; az <= nAz; az++)
    {
        Fhy_S.at(az) = 0;    Mhy_S.at(az) = 0;
        Fhx_S.at(az) = 0;    Mhx_S.at(az) = 0;
        Fhz_S.at(az) = 0;    Mhz_S.at(az) = 0;
        for (int l = 0; l < zl; l++)
        {
            //Центр втулки, СКПС
            Fhy_S.at(az) += b[l].Fhy.at(az);
            Fhx_S.at(az) += b[l].Fhx.at(az);
            Fhz_S.at(az) += b[l].Fhz.at(az);
            Mhy_S.at(az) += b[l].Mhy.at(az);
            Mhx_S.at(az) += b[l].Mhx.at(az);
            Mhz_S.at(az) += b[l].Mhz.at(az);
        }
    }

    //средние значения и амплитуды, Конус лопастей
    {
     beta_med=0; beta_min=0; beta_max=0;
      ksi_med=0;   ksimin=0;   ksimax=0;
     gama_med=0; gama_min=0; gama_max=0;
       Fx_med=0;   Fx_min=0;   Fx_max=0;
       Fy_med=0;   Fy_min=0;   Fy_max=0;
       Fz_med=0;   Fz_min=0;   Fz_max=0;
        T_med=0;    T_min=0;    T_max=0;
        Q_med=0;    Q_min=0;    Q_max=0;
       Ma_med=0;   Ma_min=0;   Ma_max=0;
        K_med=0;    K_min=0;    K_max=0;
       Nr_med=0;   Nr_min=0;   Nr_max=0;
       P1_med=0;   P1_min=0;   P1_max=0;
       Nf_med=0;   Nf_min=0;   Nf_max=0;
       P2_med=0;   P2_min=0;   P2_max=0;
       Fhx_sr=0;   Fhy_sr=0;   Fhz_sr=0;
       Mhx_sr=0;   Mhy_sr=0;   Mhz_sr=0;
     Fdmp_med=0; Fdmp_min=0; Fdmp_max=0;
        My_sr=0;      N_i=0;      N_p=0;
        Mzasr=0;    Mxasr=0;    Mzjsr=0;
        Mxjsr=0;    Mgyrx=0;    Mgyrz=0;

        for (int l = 0; l < zl; l++)
        {
            Mzasr  += (0.5/M_PI)* get_Integral(b[l].Mza, psi0);
            Mxasr  += (0.5/M_PI)* get_Integral(b[l].Mxa, psi0);
            Mzjsr  += (0.5/M_PI)* get_Integral(b[l].Mzj, psi0);
            Mxjsr  += (0.5/M_PI)* get_Integral(b[l].Mxj, psi0);
            My_sr  += (0.5/M_PI)* get_Integral(b[l].MQ , psi0);

            Fhx_sr += (0.5/M_PI)* get_Integral(b[l].Fhx, psi0);
            Fhy_sr += (0.5/M_PI)* get_Integral(b[l].Fhy, psi0);
            Fhz_sr += (0.5/M_PI)* get_Integral(b[l].Fhz, psi0);
            Mhx_sr += (0.5/M_PI)* get_Integral(b[l].Mhx, psi0);
            Mhy_sr += (0.5/M_PI)* get_Integral(b[l].Mhy, psi0);
            Mhz_sr += (0.5/M_PI)* get_Integral(b[l].Mhz, psi0);

            N_i    += (0.5/M_PI)* get_Integral(b[l].Tv , psi0) / 1000.0; //инд. мощность кВт
            N_p    += (0.5/M_PI)* get_Integral(b[l].XaU, psi0) / 1000.0; //проф. мощность кВт
        }

        switch (ROTORID)
          {//так как вес лопастей не вычитается из взлетной массы
          case MR:  Fhy_sr += g*zl*ml;  break;
          case TR:  Fhz_sr += g*zl*ml;  break;
          case PR:  Fhz_sr += g*zl*ml;  break;
          }

        switch (hubtype)//гироскопический момент для бесшарнирной втулки
        {
          case 0://hinge=шарнирный
          case 1://hinge=шарнирный
          case 2://teeter=качалка на кардане
            Mgyrx=0;  Mgyrz=0;
            break;
          case 3://hingeles=бесшарнирный
            Mgyrx = -krd*Jy*w*w_z;
            Mgyrz =  krd*Jy*w*w_x;
            break;
        }
        Mhx_sr += Mgyrx;
        Mhz_sr += Mgyrz;

        psi_med = (0.5/M_PI)* get_Integral(b[0].psi, psi0);
        fi7_med = (0.5/M_PI)* get_Integral(b[0].fi7, psi0);
        ksi_med = (0.5/M_PI)* get_Integral(b[0].ksi, psi0);
       beta_med = (0.5/M_PI)* get_Integral(b[0].beta,psi0);
       gama_med = (0.5/M_PI)* get_Integral(b[0].gama,psi0);

       xhSum_sr = (0.5/M_PI)* get_Integral(xhSum, psi0);
       zhSum_sr = (0.5/M_PI)* get_Integral(zhSum, psi0);
      //Fbsx_sr = (0.5/M_PI)* get_Integral(Fbsx, psi0);
      //Fbsz_sr = (0.5/M_PI)* get_Integral(Fbsz, psi0);

        //Конус лопастей винта
        //a0
        integral = 0;
        for (int i = 1; i <= nAz; i++)  integral += 0.5* (b[0].beta.at(i) + b[0].beta.at(i-1)) *dpsi;
        a0 = 0.5*integral/M_PI;
        //a1
        integral = 0;
        for (int i = 1; i <= nAz; i++)  integral += 0.5*( b[0].beta.at(i)*cos(b[0].psi.at(i)) + b[0].beta.at(i-1)*cos(b[0].psi.at(i-1)) ) *dpsi;
        a1 = -integral/M_PI;
        //b1
        integral = 0;
        for (int i = 1; i <= nAz; i++)  integral += 0.5*( b[0].beta.at(i)*sin(b[0].psi.at(i)) + b[0].beta.at(i-1)*sin(b[0].psi.at(i-1)) ) *dpsi;
        b1 = -integral/M_PI;
        //a2
        integral = 0;
        for (int i = 1; i <= nAz; i++)  integral += 0.5*( b[0].beta.at(i)*cos(2*b[0].psi.at(i)) + b[0].beta.at(i-1)*cos(2*b[0].psi.at(i-1)) ) *dpsi;
        a2 = -integral/M_PI;
        //b2
        integral = 0;
        for (int i = 1; i <= nAz; i++)  integral += 0.5*( b[0].beta.at(i)*sin(2*b[0].psi.at(i)) + b[0].beta.at(i-1)*sin(2*b[0].psi.at(i-1)) ) *dpsi;
        b2 = -integral/M_PI;

       T_med = (0.5/M_PI)* get_Integral(b[0].T   , psi0);
       Q_med = (0.5/M_PI)* get_Integral(b[0].Q   , psi0);
      Ma_med = (0.5/M_PI)* get_Integral(b[0].Ma  , psi0);
       K_med = (0.5/M_PI)* get_Integral(b[0].K1  , psi0);
      Nr_med = (0.5/M_PI)* get_Integral(b[0].Nr  , psi0);
      Nf_med = (0.5/M_PI)* get_Integral(b[0].Nf  , psi0);
      P1_med = (0.5/M_PI)* get_Integral(b[0].P1  , psi0);
      P2_med = (0.5/M_PI)* get_Integral(b[0].P2  , psi0);
      Fy_med = (0.5/M_PI)* get_Integral(b[0].Fy  , psi0);
      Fx_med = (0.5/M_PI)* get_Integral(b[0].Fx  , psi0);
      Fz_med = (0.5/M_PI)* get_Integral(b[0].Fz  , psi0);
    Fpov_med = (0.5/M_PI)* get_Integral(b[0].Fpov, psi0);
    Fdmp_med = (0.5/M_PI)* get_Integral(b[0].Fdmp, psi0);

    //мин-макс
    get_minmax( b[0].psi,  psi_med,  psimin,  psimax );
    get_minmax( b[0].fi7,  fi7_med,  fi7min,  fi7max );
    get_minmax( b[0].ksi,  ksi_med,  ksimin,  ksimax );
    get_minmax( b[0].beta,beta_med,beta_min,beta_max );
    get_minmax( b[0].gama,gama_med,gama_min,gama_max );
    get_minmax( b[0].T,      T_med,   T_min,   T_max );
    get_minmax( b[0].Q,      Q_med,   Q_min,   Q_max );
    get_minmax( b[0].Ma,    Ma_med,  Ma_min,  Ma_max );
    get_minmax( b[0].K1,     K_med,   K_min,   K_max );
    get_minmax( b[0].Nr,    Nr_med,  Nr_min,  Nr_max );
    get_minmax( b[0].Nf,    Nf_med,  Nf_min,  Nf_max );
    get_minmax( b[0].P1,    P1_med,  P1_min,  P1_max );
    get_minmax( b[0].P2,    P2_med,  P2_min,  P2_max );
    get_minmax( b[0].Fy,    Fy_med,  Fy_min,  Fy_max );
    get_minmax( b[0].Fx,    Fx_med,  Fx_min,  Fx_max );
    get_minmax( b[0].Fz,    Fz_med,  Fz_min,  Fz_max );
    get_minmax( b[0].Fpov,Fpov_med,Fpov_min,Fpov_max );
    get_minmax( b[0].Fdmp,Fdmp_med,Fdmp_min,Fdmp_max );
    }

    Mz_sr = Mzasr + Mzjsr + Mgyrz;//боковой момент
    Mx_sr = Mxasr + Mxjsr + Mgyrx;//продольный момент
    N_need= krd *My_sr *w /1000.0;//потр. мощность, кВт
    N_vr  = -Xa *V /1000.0;
    m_i   = N_i   *2000.0/( pow(wR,3) *ro*F*sigma );
    m_p   = N_p   *2000.0/( pow(wR,3) *ro*F*sigma );
    m_k   = N_need*2000.0/( pow(wR,3) *ro*F*sigma );
    Ct    = T_sr*2/(ro*F*wR*wR);
    Ch    = H_sr*2/(ro*F*wR*wR);
    Cs    = S_sr*2/(ro*F*wR*wR);
    ty    =  Ya *2/(ro*F*wR*wR*sigma);
    tx    =  Xa *2/(ro*F*wR*wR*sigma);
    //оперативный вывод (не в отчет)
    logstr += "\talfa=   " + ss.setNum(alpha *r2d,'f',3) + " грд\n";
    logstr += "\tfi0=    " + ss.setNum(fi0   *r2d,'f',3) + " грд\n";
    logstr += "\trpm=    " + ss.setNum(rpm       ,'f',1) + " об/мин\n";
    if(ROTORID==MR)
    {
    logstr += "\tchi=    " + ss.setNum(chi   *r2d,'f',3) + " грд\n";
    logstr += "\teta=    " + ss.setNum(eta   *r2d,'f',3) + " грд\n";
    }
    logstr += "\tT=      " + ss.setNum(T_sr      ,'f',1) + " Н\n";
    logstr += "\tMkr=    " + ss.setNum(My_sr     ,'f',1) + " Н·м\n";
    logstr += "\tNsum=   " + ss.setNum(N_need    ,'f',1) + " кВт\n";
    logstr += "\tvt=     " + ss.setNum(vt        ,'f',3) + " м/с\n";
}

void RotorBase::getHingeForces()//расчет нагрузок в шарнирах и в центре втулки с учетом инерционных усилий
{
    switch (ROTORID)
    {
    case MR:///////////////////////////// MAIN ROTOR /////////////////////////////////////////////////////////
        for (int az = 0; az <= nAz; az++)
        {
            xhQ.at(az) =0;   zhQ.at(az) =0;
            xhT.at(az) =0;   zhT.at(az) =0;
            xhNr.at(az)=0;   zhNr.at(az)=0;
            xhNf.at(az)=0;   zhNf.at(az)=0;
            xhP1.at(az)=0;   zhP1.at(az)=0;
            xhP2.at(az)=0;   zhP2.at(az)=0;
            xhK1.at(az)=0;   zhK1.at(az)=0;
            for (int l = 0; l < zl; l++)
            {
                //old (ovchar version)
                {
                b[l].Fx.at(az) = b[l].Q.at(az)
                               + b[l].K1.at(az);

                b[l].Fy.at(az) = b[l].Ty.at(az) - g*ml
                               + b[l].Nf.at(az) *sin(b[l].beta.at(az))
                               + b[l].P1.at(az) *cos(b[l].beta.at(az));

                b[l].Fz.at(az) = b[l].Nr.at(az)
                               + b[l].Nf.at(az) *cos(b[l].beta.at(az))
                               - b[l].T.at(az)  *sin(b[l].beta.at(az))
                               - b[l].P1.at(az) *sin(b[l].beta.at(az));
                }
                /*//new full version:
                {
                b[l].Fx.at(az)  =  b[l].Q.at(az) *cos(b[l].ksi.at(az))
                                     + b[l].Nr.at(az) *sin(b[l].khi.at(az) ) *cos(ksi_sr*M_PI/180.0)//new7.6
                                     + b[l].P2.at(az) *cos(b[l].ksi.at(az))//+new+
                                 + b[l].K1.at(az) *cos(b[l].ksi.at(az));//

                b[l].Fy.at(az) =   b[l].Ty.at(az) - g*ml//-> внимание! массу лопастей нужно вычесть из массы вертолета
                                 + b[l].P1.at(az)*cos(b[l].beta.at(az))//ok
                                 + b[l].Nf.at(az)*sin(b[l].beta.at(az));//ok

                b[l].Fz.at(az) = - b[l].T.at(az)  *sin(b[l].beta.at(az))// *cos(b[l].ksi.at(az))/////- ok
                                 - b[l].P1.at(az) *sin(b[l].beta.at(az))// *cos(b[l].ksi.at(az))/////- ok
                                     - b[l].K1.at(az) *sin(b[l].ksi.at(az))//new
                                     - b[l].P2.at(az) *sin(b[l].ksi.at(az))//- new
                                     - b[l].Q.at(az)  *sin(b[l].ksi.at(az))//new
                                 + b[l].Nr.at(az) *cos(b[l].khi.at(az) ) *sin(ksi_sr*M_PI/180.0)//7.6
                                 + b[l].Nf.at(az) *cos(b[l].beta.at(az));// *cos(b[l].khi.at(az));//;//ok
                }//
                ///компоненты усилия на втулке в СКПС (дублирование расчета сил)
                {
                //Q
                toHubSystem( b[l].psi.at(az),  b[l].Q.at(az)*cos(b[l].ksi.at(az)), -b[l].Q.at(az)*sin(b[l].ksi.at(az)),  fx, fz );// *cos(b[l].ksi.at(az))
                xhQ.at(az) += fx;
                zhQ.at(az) += fz;
                //FhT
                toHubSystem( b[l].psi.at(az),  0, -b[l].T.at(az)*sin(b[l].beta.at(az)),  fx, fz );//
                xhT.at(az) += fx;
                zhT.at(az) += fz;
                //FhP1
                //toHubSystem( b[l].psi.at(az),  0, -b[l].P1.at(az)*sin(b[l].beta.at(az)),  fx, fz );//
                toHubSystem( b[l].psi.at(az),  0, -b[l].P1.at(az)*sin(b[l].beta.at(az)),  fx, fz );//
                xhP1.at(az) += fx;
                zhP1.at(az) += fz;
                //FhP2
                //toHubSystem( b[l].psi.at(az),  b[l].P2.at(az), 0,  fx, fz );//
                toHubSystem( b[l].psi.at(az),  b[l].P2.at(az)*cos(b[l].ksi.at(az)), -b[l].P2.at(az)*sin(b[l].ksi.at(az)),  fx, fz );//
                xhP2.at(az) += fx;
                zhP2.at(az) += fz;
                //FhK1
                //toHubSystem( b[l].psi.at(az),  b[l].K1.at(az), 0,  fx, fz );//*cos(b[l].ksi.at(az))
                toHubSystem( b[l].psi.at(az),  b[l].K1.at(az)*cos(b[l].ksi.at(az)), -b[l].K1.at(az)*sin(b[l].ksi.at(az)),  fx, fz );//*cos(b[l].ksi.at(az))
                xhK1.at(az) += fx;
                zhK1.at(az) += fz;
                //FhNr
                //toHubSystem( b[l].psi.at(az),  0, b[l].Nr.at(az),  fx, fz );
                toHubSystem( b[l].psi.at(az),  b[l].Nr.at(az)*sin(b[l].khi.at(az)), b[l].Nr.at(az)*cos(b[l].khi.at(az)),  fx, fz );//b[l].M1Cy.at(az)/(0.4*R-l_vsh)
                xhNr.at(az) += fx;
                zhNr.at(az) += fz;
                //FhNf
                //toHubSystem( b[l].psi.at(az),  0, b[l].Nf.at(az)*cos(b[l].beta.at(az)),  fx, fz );//
                toHubSystem( b[l].psi.at(az),  0, b[l].Nf.at(az)*cos(b[l].beta.at(az)),  fx, fz );//
                xhNf.at(az) += fx;
                zhNf.at(az) += fz;
                //FhK2
                //toHubSystem( b[l].psi.at(az),  b[l].K2.at(az)*sin(b[l].ksi.at(az)), b[l].K2.at(az)*cos(b[l].ksi.at(az)),  fx, fz );//
                //xhK2.at(az) += fx;
                //zhK2.at(az) += fz;
                }
                //положение центра тяжести лопасти
                //ctX_1blade = 0.5*R *sin( b[l].khi.at(az) );/// - ksi_sr  - 0.133
                //ctZ_1blade = 0.5*R *cos( b[l].khi.at(az) );/// - ksi_sr  - 0.133
                //toHubSystem(b[l].psi.at(az), ctX_1blade, ctZ_1blade, ct1bx, ct1bz);
                //положение центра тяжести НВ относит. оси НВ (оси координат параллельны осям фюзеляжа)
                //MRctX += ct1bx;//ctX_1blade
                //MRctZ += ct1bz;//ctZ_1blade*/
            }
            //усилия на втулке в СКПС (дублирование расчета сил)
            //xhSum.at(az) = xhQ.at(az) + xhT.at(az) + xhNr.at(az) + xhNf.at(az) + xhP1.at(az) + xhK1.at(az) + xhP2.at(az);/// + xhK2.at(az)
            //zhSum.at(az) = zhQ.at(az) + zhT.at(az) + zhNr.at(az) + zhNf.at(az) + zhP1.at(az) + zhK1.at(az) + zhP2.at(az);/// + zhK2.at(az)
            //положение центра тяжести системы лопастей
            {
            //MRctX /= zl;
            //MRctZ /= zl;
            //отклонение центра тяжести системы лопастей относит. оси НВ
            //dct  = sqrt( MRctX*MRctX + MRctZ*MRctZ );
            //направление неуравновешенной (центробежной) силы на оси НВ //угол между вектором смещения ЦТ НВ и осью Х НВ паралл.оси Х фюзеляжа
            //act = KatToAng( MRctZ, MRctX );///
            //неуравновешенная (центробежная) сила на оси НВ
            //Faxe =  ml*zl *w*w *dct;
            //Fbsz.at(az) = Faxe *sin( act );///проекция на ось Z
            //Fbsx.at(az) = Faxe *cos( act );///проекция на ось X
            }
        }
        break;
    case TR:///////////////////////////// TAIL ROTOR /////////////////////////////////////////////////////////
        for (int az = 0; az <= nAz; az++)
        {
            for (int l = 0; l < zl; l++)
            {
                //проекции результирующей силы на шарнирах каждой лопасти
                b[l].Fx.at(az) = b[l].Q.at(az)
                               + b[l].K1.at(az)
                               + g*ml *cos(b[l].psi.at(az));

                b[l].Fy.at(az) = b[l].T.at(az)  *cos(b[l].beta.at(az))
                               + b[l].P1.at(az) *cos(b[l].beta.at(az))
                               + b[l].Nf.at(az) *sin(b[l].beta.at(az));

                b[l].Fz.at(az) = b[l].Nr.at(az)
                               + b[l].Nf.at(az) *cos(b[l].beta.at(az))
                               - b[l].T.at(az)  *sin(b[l].beta.at(az))
                               - b[l].P1.at(az) *sin(b[l].beta.at(az))
                               - krd*g*ml *sin(b[l].psi.at(az));
            }
        }
        break;
    case PR:///////////////////////////// PUSH ROTOR /////////////////////////////////////////////////////////
        for (int az = 0; az <= nAz; az++)
        {
            for (int l = 0; l < zl; l++)
            {
                //проекции результирующей силы на шарнирах каждой лопасти
                b[l].Fx.at(az) = b[l].Q.at(az)
                               + g*ml *cos(b[l].psi.at(az));

                b[l].Fy.at(az) = b[l].T.at(az);

                b[l].Fz.at(az) = b[l].Nr.at(az)
                               - krd*g*ml *sin(b[l].psi.at(az));
            }
        }
        break;
    }
    //общие
    for (int az = 0; az <= nAz; az++)
    {
        for (int l = 0; l < zl; l++)
        {
            //сила на штоке демпфера лопасти и момент силы отн.ВШ
            if(damper) b[l].Fdmp.at(az) = DampherForce( DampherSpeed(b[l].ksi.at(az),b[l].dksi.at(az)) );  else b[l].Fdmp.at(az) = 0;
            b[l].M2D.at(az) = b[l].Fdmp.at(az) *d.h;
            //ГШ (СКЛ) ---------------------------------------------------------------
            if(damper)
                 b[l].M1y.at(az) = b[l].Fx.at(az)*(l_vsh-l_gsh) + b[l].M2D.at(az);//krd*
            else b[l].M1y.at(az) = b[l].M1Q.at(az) + b[l].M1K.at(az) + b[l].M1Gy.at(az) + b[l].M1Cy.at(az) + b[l].M1Py.at(az);//krd*
            //b[l].M1y.at(az) = b[l].M1Q.at(az) + b[l].M1K.at(az);//Овчар,MR
            //b[l].M1y.at(az) = krd*(b[l].M1Q.at(az) + b[l].M1K.at(az)) + b[l].M1Gy.at(az);//Овчар,TR
            //b[l].M1G.at(az) = g *cos(b[l].beta.at(az)) *cos(b[l].psi.at(az)) *(S_gsh - ml*l_gsh);//Овчар,TR
            b[l].M1x.at(az) = b[l].M1T.at(az) + b[l].M1Gx.at(az) + b[l].M1Cx.at(az) + b[l].M1Px.at(az) + b[l].M1Cz.at(az);//control:mustbe=0
            b[l].M1z.at(az) = 0;
            //ВШ (СКЛ)
            b[l].M2x.at(az) = b[l].M2T.at(az) + b[l].M2Gx.at(az) + b[l].M2Cx.at(az) + b[l].M2Px.at(az) + b[l].M2Cz.at(az);
            b[l].M2y.at(az) = b[l].M2Q.at(az) + b[l].M2K.at(az) + b[l].M2Gy.at(az) + b[l].M2Cy.at(az) + b[l].M2Py.at(az) + b[l].M2D.at(az);//control:mustbe=0
            b[l].M2z.at(az) = 0;
            //ОШ (СКЛ): нет расчета шарнирного момента
            b[l].M3x.at(az) = 0;////////?
            b[l].M3y.at(az) = 0;////////?
            b[l].M3z.at(az) = b[l].M3Ma.at(az) + b[l].M3su.at(az) + b[l].M3Cz.at(az) + b[l].M3tr.at(az) + b[l].M3dmp.at(az) + b[l].M3iz.at(az);//control:mustbe=0

            //центр втулки (СКН)
            b[l].Fhx.at(az) = b[l].Fz.at(az) *cos(b[l].psi.at(az)) + krd*b[l].Fx.at(az) *sin(b[l].psi.at(az));
            b[l].Fhz.at(az) = krd*b[l].Fz.at(az) *sin(b[l].psi.at(az)) - b[l].Fx.at(az) *cos(b[l].psi.at(az));
            b[l].Fhy.at(az) =     b[l].Fy.at(az);
            b[l].Mhx.at(az) =-krd*b[l].Fy.at(az) *l_gsh *sin(b[l].psi.at(az));
            b[l].Mhz.at(az) =     b[l].Fy.at(az) *l_gsh *cos(b[l].psi.at(az));
            b[l].Mhy.at(az) =    (b[l].Fx.at(az) *l_vsh    + b[l].M2Q.at(az));
        }
    }
}

double RotorBase::get_rpm()//Расчет ***, исходя из угла атаки и потребной результирующей силы
{
    //метод половинного деления
    double a = rpm_min,
           b = rpm_max,
           d = fabs(b - a),
           x = 0.5*(a + b),
           d1 = get_dTsr(a),
           d2 = get_dTsr(b),
           d3 = get_dTsr(x);
    int q = 0;
    if(d1*d2>0)
    {
        warning = 317;
        logstr += ss.setNum(ROTORID);
        logstr += "::get_rpm(): неверно задана область поиска частоты вращения [rpm_min...rpm_max]";
    }
    else
    while (d > prec)
    {
        q++;
        ((d1 >= 0 && d3 >= 0) || (d1 < 0 && d3 < 0)) ? a = x : a;
        ((d2 >= 0 && d3 >= 0) || (d2 < 0 && d3 < 0)) ? b = x : b;
        (a == x) ? d1 = d3 : d1;
        (b == x) ? d2 = d3 : d2;
        x = 0.5*(a + b);///fi0
        d3 = get_dTsr(x);
        d = fabs(b - a);
        if(q>99) {
        logstr += "get_rpm(): #" + ss.setNum(q);
        logstr +=    ",\t rpm=" + ss.setNum(x*r2d,'f',4);
        logstr +=      ",\t dR=" + ss.setNum(d3,'f',2) +"\n";
        warning=304; break; }
    }
    return x;//угол ОШ, рад
}

double RotorBase::get_dTsr(double _rpm)
{//_rpm    частота вращения винта
    rpm = _rpm;
    getIniFlow();
    Core();
    switch (ROTORID)
    {
    case MR://MainRotor
        return 100* (R_need - Ra) / R_need;
        break;
    case TR://TailRotor
        return 100* (T_need-T_sr) / T_need;
        break;
    case PR://PushRotor
        return 100* (T_need-T_sr) / T_need;
        break;
    }
}

double RotorBase::get_fi0()//Расчет общего шага, исходя из угла атаки и потребной результирующей силы
{
    //метод половинного деления
    double a = fi0_min,
           b = fi0_max,
           d = fabs(b - a),
           x = 0.5*(a + b),
           d1 = get_dRa(a),
           d2 = get_dRa(b),
           d3 = get_dRa(x);
    int q = 0;
    if(d1*d2>0)
    {
        warning = 316;
        logstr += ss.setNum(ROTORID);
        logstr += "get_fi0(): неверно задана область поиска угла общего шага [fi0_min...fi0_max]";
    }
    else
    while (d > 0.02*prec)///0.02
    {
        q++;
        ((d1 >= 0 && d3 >= 0) || (d1 < 0 && d3 < 0)) ? a = x : a;
        ((d2 >= 0 && d3 >= 0) || (d2 < 0 && d3 < 0)) ? b = x : b;
        (a == x) ? d1 = d3 : d1;
        (b == x) ? d2 = d3 : d2;
        x = 0.5*(a + b);///fi0
        d3 = get_dRa(x);
        d = fabs(b - a);
        if(q>99) {
        logstr += "get_fi0(): #" + ss.setNum(q);
        logstr +=     ",\t fi0=" + ss.setNum(x*r2d,'f',4);
        logstr +=     ",\t dfi=" + ss.setNum(d,'f',4);
        logstr +=      ",\t dR=" + ss.setNum(d3,'f',2);
        logstr +=    ",\t T_sr=" + ss.setNum(T_sr,'f',1) +"\n";
        warning=303; break; }
    }
    return x;//угол ОШ, рад
}

double RotorBase::get_dRa(double _fi0)//Расчет разницы между результирующей силой винта и потребной результирующей силой; вызывается функцией get_fi0()
{//_fi0    общий шаг винта
    fi0 = _fi0;
    Core();
    switch (ROTORID)
    {
    case MR://MainRotor
        return 100* (R_need - Ra) / R_need;
        break;
    case TR://TailRotor
        return 100* (T_need-T_sr) / T_need;
        break;
    case PR://PushRotor
        return 100* (T_need-T_sr) / T_need;
        break;
    }
}

void RotorBase::getIniFlow()//расчет проекций скорости полета на оси расчетной СК
{
    //Расчет проекций скорости потока
    switch (ROTORID)
      {
      case MR:
        Vx =  V *cos(beta_sk) *cos(alpha);
        Vy =  V *cos(beta_sk) *sin(alpha);
        Vz =  V *sin(beta_sk);
        break;
      case TR:
        Vx =  V *cos(beta_sk) *cos(alpha);
        Vy =  V *cos(beta_sk) *sin(alpha);
        Vz =  V *sin(beta_sk);///-
        break;
      case PR:
        Vz =  V *cos(beta_sk) *cos(alpha);
        Vy =  V *cos(beta_sk) *sin(alpha);
        Vx = -V *sin(beta_sk);
        break;
      }
    //угловая скорость вала винта, рад/с
    w = rpm*M_PI/30.0;
    //окружная скорость концов лопастей
    wR = w * R;
    //относительная скорость потока
    _V  = V / wR;
    //коэф. режима работы винта
    _Vx = Vx / wR;    mu = _Vx;
    //проекция отн.скорости на ось OYh
    _Vy = Vy / wR;
    //проекция отн.скорости на ось OZh
    _Vz = Vz / wR;
}

void RotorBase::getInducedFlow()//расчет индуктивной скорости в зависимости от T_i, H_i, S_i
{// предварительно должны быть определены T_i, H_i, S_i
    int q = 0;
    double Ch=0, Ct=0, Cs=0, _vt=0, _vh=0, _vs=0, _Vsum=0, _vt0=0, _v_=0,_V_=0, err=1;
    //коэффициенты тяги и остальных сил на втулке
    Ct = 2* T_i / (ro*F*wR*wR);
    Ch = 2* H_i / (ro*F*wR*wR);
    Cs = 2* S_i / (ro*F*wR*wR);

    //относительная индуктивная скорость в зависимости от режима полета
    /*if (mu<0.05 && mu>-0.05)///висение
    {
        //отн.инд.скорость включающая общий поток через винт //Гессоу+Мейерс,с.138
        _vt0 = sqrt(fabs(Ct)) / (2.0*Bkp);
        //относительная индуктивная скорость
        if(Ct<0) _vt =  0.5*_V - sqrt(0.25*_V*_V + _vt0*_vt0);//для РВ с отрицательной тягой
        else     _vt = -0.5*_V + sqrt(0.25*_V*_V + _vt0*_vt0);
        //if (Ct<0) _vt =  0.5*_Vy - sqrt(0.25*_Vy*_Vy - 0.25*Ct/Bkp);///для толкающего/тянущего РВ
        //else      _vt = -0.5*_Vy + sqrt(0.25*_Vy*_Vy + 0.25*Ct/Bkp);///заменил _V на _Vy
        _vh = 0;
        _vs = 0;
    }// */
    if (mu>-0.1 && mu<0.1)/// малые скорости полета
    {
        //отн.инд.скорость включающая общий поток через винт //Гессоу+Мейерс,с.138
        _vt0 = sqrt(fabs(Ct)) / (2.0*Bkp);
        //относительная индуктивная скорость
        _vh  = Ch *_vt0 / Ct;//_vh = 0;
        _vs  = Cs *_vt0 / Ct;//_vs = 0;
        switch (ROTORID)
          {
          case MR:
            _V_ = (Ct)?(1.96 *_V / sqrt(fabs(Ct))):(0);
            _v_ = (_V_>=0 && _V_<5)?(1.0-0.0722*_V_-0.257*pow(_V_,2)+0.12126*pow(_V_,3)-0.02184*pow(_V_,4)+0.0014359*pow(_V_,5)):(1);
            //относительная индуктивная скорость
            _vt = _v_*_vt0;
            break;
          case TR: case PR:
            if(Ct<0) _vt =  0.5*_V - sqrt(0.25*_V*_V +_vt0*_vt0);//для РВ с отрицательной тягой
            else     _vt = -0.5*_V + sqrt(0.25*_V*_V +_vt0*_vt0);
            break;
          }
    }// */
    else /// режим горизонтального полета
    {
        //FLIGHTMODE = HORIZONTAL;
        _vt = Ct / (4.0*Bkp*Bkp*_V);
        if (Ct)
        {
            _vh = Ch *_vt / Ct;
            _vs = Cs *_vt / Ct;
            while (err > 0.001)
            {
                q++;
                _Vsum = sqrt(fabs( (_Vx-_vh)*(_Vx-_vh) + (_Vy-_vt)*(_Vy-_vt) + _vs*_vs ));

                _vt0 = Ct / (4.0*Bkp*Bkp*_Vsum);//Гессоу+Мейерс,с.138
                _vt  = 0.5*(_vt0 +_vt);//origin _vt  = _vt0;
                _vh  = Ch *_vt0 / Ct;
                _vs  = Cs *_vt0 / Ct;

                err  = fabs((_vt0 -_vt)/_vt);
                if (q > 99)
                {
                    logstr += "getInducedFlow(): q>999, _Vsum=" + ss.setNum(_Vsum,'f',3);
                    logstr += ", _vt0=" + ss.setNum(_vt0,'f',6);
                    logstr += ", _vt="  + ss.setNum(_vt, 'f',6);
                    logstr += ", err="  + ss.setNum(err, 'f',6) + "\n";
                    warning=305; break;
                }
            }
        }
        else {  _vh = 0;  _vs = 0;  }
    }

    //индуктивная скорость
    vt = _vt *wR;
    vh = _vh *wR;
    vs = _vs *wR;
    //средний коэф.протекания
    lambda_sr = _Vy - _vt;
    //коэффициент K_mu
    if ((_Vx -_vh + 1.2*(lambda_sr))==0 || (_Vx -_vh - 1.2*(lambda_sr))==0)
        K_mu = 0;
    else
    {
        if(Ct<0)
            K_mu = ((_Vx - _vh) / (_Vx - _vh + 1.2*(lambda_sr) ) )*4.0/3.0;
        else
            K_mu = ((_Vx - _vh) / (_Vx - _vh - 1.2*(lambda_sr) ) )*4.0/3.0;//origin
    }
    //коэфф. в формуле индуктивной скорости
    f2 = get_f2(mu);//~1
    f1 = 1.0 - f2;
    //проверка
    Ct_test = 4*Bkp*Bkp*_vt*sqrt(mu*mu+lambda_sr*lambda_sr);
    Ch_test = 4*Bkp*Bkp*_vh*sqrt(mu*mu+lambda_sr*lambda_sr);
}

void RotorBase::generalCore()//Расчет интегральных сил и маховых движений при текущих значениях alpha и fi0 винта
{
    QCoreApplication::processEvents();
    zakrut = true;
    int begin, end, i, _i, ob;
    //int     nAz;        //кол-во расчетных азимутов за оборот
    double DeltaBeta=1;
    //инициализация векторов
    vector<vector<double>> tmp2(nAz+1);            dMQ_dr = tmp2;//
    vector<double> tmp_dT(nSec), tmp_dQ(nSec), tmp_dMa(nSec),  tmp_dMQ(nSec), tmp_dH(nSec), tmp_dS(nSec),
                   tmp_Ux(nSec), tmp_Uy(nSec), tmp_Uywx(nSec), tmp_Uywz(nSec), tmp_U(nSec),//
                   tmp_lambda(nSec), tmp_fi(nSec), tmp_fl(nSec), tmp_Fi(nSec), tmp_alpha(nSec), tmp_M(nSec), tmp_Cy(nSec), tmp_Cx(nSec), tmp_Mz(nSec);
    vector<double> tmp(nAz+1), _fi7(nAz+1),_dfi7(nAz+1),_d2fi7(nAz+1),
                   _beta(nAz+1),_dbeta(nAz+1),_d2beta(nAz+1),_beta1(nAz+1),_beta2(nAz+1),
                   _gama(nAz+1),_dgama(nAz+1),_d2gama(nAz+1),  _ksi(nAz+1), _dksi(nAz+1), _d2ksi(nAz+1);//_gama1(nAz+1),_gama2(nAz+1),//, _ksi1(nAz+1), _ksi2(nAz+1), _dksi1(nAz+1), _dksi2(nAz+1)
    deltabeta = deltagama = deltaksi = tmp;
    //
    QString qs, qsn;
    //
    writeln(&this->vsh, "RotorBase::generalCore starts working");
    writeln(&this->vsh1, "RotorBase::generalCore starts working");
    //
    for (int l=0; l<zl; l++)
    {
        b[l].fi7 = b[l].dfi7 = b[l].d2fi7 = b[l].gama = b[l].dgama = b[l].d2gama = tmp;
        b[l].beta = b[l].dbeta = b[l].d2beta = b[l].ksi = b[l].dksi = b[l].d2ksi = b[l].khi = tmp;
        b[l].T = b[l].Ty = b[l].Q = b[l].H = b[l].S = b[l].Ma = b[l].MQ = b[l].Nr = b[l].Nf = tmp;
        b[l].dT_dr = b[l].dQ_dr = b[l].dMa_dr = b[l].dH_dr = b[l].dS_dr =  tmp2;
        b[l].lambda = b[l].U = b[l].Ux = b[l].Uy = b[l].Uywx = b[l].Uywz = tmp2;
        b[l].fi = b[l].fl = b[l].Fi = b[l].alfar = b[l].M = b[l].cy = b[l].cx = b[l].mz = tmp2;
    }

    //параметры движения лопасти в начальном положении
    int n = 0;//текущий азимут
       _fi7.at(n) = fi0 + dfi.at(sec07);
      _dfi7.at(n) = 0;
      _gama.at(n) = 0;
     _dgama.at(n) = 0;
      _beta.at(n) = 5*d2r;
     _dbeta.at(n) = 0;
      _dksi.at(n) = 0;
     if (hubtype) _ksi.at(n) = 0;
     else _ksi.at(n) = ksi_nom;
     _d2gama.at(n)= 0;
     switch (hubtype)
       {
       case 0://шарнирный(ОШ+ГШ+ВШ)
       case 1://шарнирный(ОШ+ГШ)
           _d2beta.at(n)= get_d2beta_hinge(0,0,_fi7.at(0),0,get_IntegralMdT1(0,0,0,0,0,0));
           _d2ksi.at(n) = get_d2ksi_hinge(0,_beta.at(n),_dbeta.at(n),_ksi.at(n),_dksi.at(n));
           break;
       case 2://качалка карданного типа (ОШ+СГШ)
           _d2beta.at(n) = get_d2beta_kardan(0,0,_fi7.at(0),0,get_IntegralMdT1(0,0,0,0,0,0),get_IntegralMdT1(M_PI,0,0,0,0,0));
           _d2ksi.at(n) = 0;//get_d2ksi_kardan(0, _beta.at(n),_ksi.at(n), get_IntegralMdQ2(0, 0, 0), get_IntegralMdQ2(0, 0, M_PI));
           break;
       case 3://hingeles=бесшарнирный
           _d2beta.at(n) = 0;
           _d2ksi.at(n) = 0;
           break;
       }
       Fi7.clear();      Fi7.push_back(   _fi7.at(n));
      dFi7.clear();     dFi7.push_back(  _dfi7.at(n));
     d2Fi7.clear();    d2Fi7.push_back( _d2fi7.at(n));
      Beta.clear();     Beta.push_back(  _beta.at(n));
     dBeta.clear();    dBeta.push_back( _dbeta.at(n));
    d2Beta.clear();   d2Beta.push_back(_d2beta.at(n));
       Ksi.clear();      Ksi.push_back(   _ksi.at(n));
      dKsi.clear();     dKsi.push_back(  _dksi.at(n));
     d2Ksi.clear();    d2Ksi.push_back( _d2ksi.at(n));
      Gama.clear();     Gama.push_back(  _gama.at(n));
     dGama.clear();    dGama.push_back( _dgama.at(n));
    d2Gama.clear();   d2Gama.push_back(_d2gama.at(n));
    deltaBeta.clear();  deltaBeta.push_back(0);
    deltaGama.clear();  deltaGama.push_back(0);
     deltaKsi.clear();   deltaKsi.push_back(0);

     //vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
     //rad / (rad/s) =s
     /*
        IncomingParamsFlutter dataForFlutter;
        HelicParamsSupplFlutter dataHelicSupplFlutter;
        BladeParamsSupplFlutter dataBladeSupplFlutter;
     */
     AzimuthDuration=2*M_PI/this->w/nAz; //tn=T/n
     //
     //this->fltrModel.SetIniExtData(this->flIniExtData);
     //this->fltrModel.SetIniKynemConds(this->flKynDevParams);
     //
     //IncomingParamsFlutter dataIncomingFlutter;
     //BladeParamsSupplFlutter dataBladeSupplFlutter;
     //HelicParamsSupplFlutter dataHelicSupplFlutter;
     //FlutterModeParams dataFlutterMode;
     //void SetIncomingParams(const IncomingParamsFlutter&data);
     //void SetBladeSupplData(const BladeParamsSupplFlutter&data);
     //void SetHelicSupplData(const HelicParamsSupplFlutter&data);
     //void SetFlutterModeParams(const FlutterModeParams&data);
     //
     //this->fltrModel.SetIncomingParams(dataIncomingFlutter);//je azimuth=>je Mach
     //Initialization flutter model
     if(true){
         //this->fltrModel.SetIniDataByTextFileReadingBySearch(fileName);//fileName msb gi'd
     }
     this->fltrModel.SetBladeSupplData(dataBladeSupplFlutter);
     this->fltrModel.SetHelicSupplData(dataHelicSupplFlutter);
     this->fltrModel.SetFlutterModeParams(dataFlutterMode);
     //
     this->fltrModel.SetIniKynemConds(flKynDevParams);
     //
     //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    //расчет параметров движения лопасти
    Psi.clear();
    Psi.push_back(0);
    _beta1.at(0) = Beta.at(0);
    //
    //первый оборот
    writeln(&this->vsh,  "1st rotatiom");
    writeln(&this->vsh1, "1st rotatiom");
    //
    for (n = 1; n <= nAz; n++)
    {

        Psi.push_back(n*dpsi);
        //
        qsn.setNum(n);
        qs="AzimuthN="+qsn;
        qsn.setNum(nAz);
        qs=qs+" of "+qsn+" (psi=";
        qsn.setNum(n*dpsi);
        qs=qs+qsn+")";
        writeln(&this->vsh, qs);
        writeln(&this->vsh1, qs);
        //
        blade_kinematic_az(n,0);//0- без закрутки отн.ОШ
        _beta1.at(n) = Beta.at(n);
    }
    //еще минимум четыре следующих оборота
    writeln(&this->vsh,  "Next rotations (min.4)");
    writeln(&this->vsh1, "Next rotations (min.4)");
    ob = 1;
    while ((ob<5)||(DeltaBeta>prec && ob<100))//0.01...0.001
    {
        ob++;//номер оборота
        begin = 1 + (ob-1)*nAz;//начало расчетного интервала на данном обороте
        end   = ob*nAz;         //конец расчетного интервала на данном обороте
        _beta2.at(0) = Beta.at(begin-1);
        //
        qsn.setNum(ob);
        qs="Rotation: "+qsn;
        qsn.setNum(begin);
        qs=qs+" Azimuths: ("+qsn+"..";
        qsn.setNum(end);
        qs=qs+qsn;
        //qs=qs+" Cur AzimuthN:";
        writeln(&this->vsh, qs);
        writeln(&this->vsh1, qs);
        //
        //qs=qs+", Cur AzimuthN:";
        //writeln(&this->vsh, qs);
        //
        for (i = begin; i <= end; i++)//cycle of azimuths
        {
            Psi.push_back(i*dpsi);
            //
            //
            this->fltrModel.b7=this->b7;
            this->fltrModel.chi_param=this->chi_sr;//or mab chi?
            //this->fltrModel.c_ctrl
            //this->fltrModel.dist_focus_ridg
            //this->fltrModel.d_frict
            //this->fltrModel.gamma0_param
            this->fltrModel.I_gsh=this->I_gsh;
            this->fltrModel.I_osh=this->I_osh;
            //this->fltrModel.lagerKFr
            this->fltrModel.mlp=this->mlp;//or mab ml?
            this->fltrModel.R=this->R;
            this->fltrModel.ro=this->ro;
            this->fltrModel.Sb0=this->Sb0;
            //this->fltrModel.RigidityCentersLine
            //this->fltrModel.RigidityCentersLineTypeN
            //this->fltrModel.sigmav // ne this->sigma!
            //this->fltrModel.sigma_f
            this->fltrModel.w=this->w;
            //this->fltrModel.WeightCentersLine
            //this->fltrModel.WeightCentersLineTypeN;
            //this->fltrModel.x0
            //this->fltrModel._b_focus
            //this->fltrModel._b_weight
            //this->fltrModel._b_press
            //this->fltrModel._b_ridg
            //this->fltrModel._x_focus
            //this->fltrModel._x_rig
            //this->fltrModel._x_weight
            //this->fltrModel.
            //
            //blade_kinematic_az(i,1);//1- с закруткой отн.ОШ //I commented ut be real vals
            blade_kinematic_az(i,0);//1- с закруткой отн.ОШ //I gi alt val ut be real vals
            _beta2.at(i-begin+1) = Beta.at(i);
        }
        DeltaBeta = get_max_deviation(_beta1, _beta2);
        _beta1 = _beta2;
        if(ob>99) warning = 301;
        //
        if(ob>6) break;//my tmp f'debug
    }
    //QString rotNum;
    //QMessageBox msg;
    //rotNum.setNum(ob);
    //msg.setText("Quantity of rotations done for calc: "+rotNum);
    //msg.show();
    //для дальнейшего расчета принимаем параметры махового движения с последней "сошедшейся" итерации
    _i = 0;
    for (n = (ob-1)*nAz; n <= ob*nAz; n++)
    {
          _beta.at(_i) =   Beta.at(n);
         _dbeta.at(_i) =  dBeta.at(n);
        _d2beta.at(_i) = d2Beta.at(n);
          _gama.at(_i) =   Gama.at(n);
         _dgama.at(_i) =  dGama.at(n);
        _d2gama.at(_i) = d2Gama.at(n);
           _ksi.at(_i) =    Ksi.at(n);
          _dksi.at(_i) =   dKsi.at(n);
         _d2ksi.at(_i) =  d2Ksi.at(n);
           _fi7.at(_i) =    Fi7.at(n);
          _dfi7.at(_i) =   dFi7.at(n);
         _d2fi7.at(_i) =  d2Fi7.at(n);
      deltabeta.at(_i) = deltaBeta.at(n);
      deltagama.at(_i) = deltaGama.at(n);
       deltaksi.at(_i) =  deltaKsi.at(n);
       _i++;
    }
    //распределение beta, ksi, gama на каждую лопасть
    for (int l = 0; l < zl; l++)
    {
           b[l].fi7 = shiftAz(   _fi7, l);
          b[l].dfi7 = shiftAz(  _dfi7, l);
         b[l].d2fi7 = shiftAz( _d2fi7, l);
          b[l].gama = shiftAz(  _gama, l);
         b[l].dgama = shiftAz( _dgama, l);
        b[l].d2gama = shiftAz(_d2gama, l);
          b[l].beta = shiftAz(  _beta, l);
         b[l].dbeta = shiftAz( _dbeta, l);
        b[l].d2beta = shiftAz(_d2beta, l);
           b[l].ksi = shiftAz(   _ksi, l);
          b[l].dksi = shiftAz(  _dksi, l);
         b[l].d2ksi = shiftAz( _d2ksi, l);
    }
    //среднее за оборот значение угла качания
    ksi_sr = (0.5/M_PI)* get_Integral(b[0].ksi, psi0);

    //аэродинамические параметры лопастей
    for (int l = 0; l < zl; l++)
    {
        for (int az = 0; az <= nAz; az++)//проход по азимутам  //qob < zl (cycle ef), < nSec (cycle af), ma <= (n'<) nAz (amda)?
        {
            for (i = 0; i < nSec; i++)//проход по сечениям
            {
                seg = Segment(i, b[l].psi.at(az), b[l].beta.at(az), b[l].dbeta.at(az), b[l].gama.at(az), b[l].dgama.at(az), b[l].dfi7.at(az));
                //заполнение массивов параметров распределенных по лопасти
                tmp_alpha.at(i) = seg.alfar;
                tmp_fi.at(i)    = seg.fi;
                tmp_fl.at(i)    = seg.fl;
                tmp_U.at(i)     = seg.U;
                tmp_Ux.at(i)    = seg.Ux;
                tmp_Uy.at(i)    = seg.Uy;
                tmp_Uywx.at(i)  = seg.Uywx;
                tmp_Uywz.at(i)  = seg.Uywz;
                tmp_lambda.at(i)= seg.lambda;
                tmp_Fi.at(i)    = seg.Fi;
                tmp_M.at(i)     = seg.M;
                tmp_Cy.at(i)    = seg.cy;
                tmp_Cx.at(i)    = seg.cx;
                tmp_Mz.at(i)    = seg.mz;
                tmp_dT.at(i)    = seg.dT_dr;
                tmp_dQ.at(i)    = seg.dQ_dr;
                tmp_dMa.at(i)   = seg.dMa_dr;
                    tmp_dMQ.at(i)= seg.dQ_dr *((r.at(i)-l_gsh)*cos(b[l].beta.at(az)) + l_gsh);
                    //for second method:
                    tmp_dH.at(i) = sin(b[l].psi.at(az)) * tmp_dMQ.at(i)/r.at(i) - cos(b[l].psi.at(az)) * b[l].beta.at(az) * tmp_dT.at(i);
                    tmp_dS.at(i) =-cos(b[l].psi.at(az)) * tmp_dMQ.at(i)/r.at(i) - sin(b[l].psi.at(az)) * b[l].beta.at(az) * tmp_dT.at(i);
                if(i==sec07)
                {
                    b[l].fi7.at(az) = seg.fi;
                    //b[l].xd7.at(az) = seg.xd;
                }
            }
            //заполнение массивов параметров распределенных по диску винта
            b[l].alfar.at(az) = tmp_alpha;
            b[l].fi.at(az)    = tmp_fi;
            b[l].fl.at(az)    = tmp_fl;
            b[l].U.at(az)     = tmp_U;
            b[l].Ux.at(az)    = tmp_Ux;
            b[l].Uy.at(az)    = tmp_Uy;
            b[l].Uywx.at(az)  = tmp_Uywx;
            b[l].Uywz.at(az)  = tmp_Uywz;
            b[l].lambda.at(az)= tmp_lambda;
            b[l].Fi.at(az)    = tmp_Fi;
            b[l].M.at(az)     = tmp_M;
            b[l].cy.at(az)    = tmp_Cy;
            b[l].cx.at(az)    = tmp_Cx;
            b[l].mz.at(az)    = tmp_Mz;
            b[l].dT_dr.at(az) = tmp_dT;
            b[l].dQ_dr.at(az) = tmp_dQ;
            b[l].dMa_dr.at(az)= tmp_dMa;
                dMQ_dr.at(az)     = tmp_dMQ;
                b[l].dH_dr.at(az) = tmp_dH;//for second method
                b[l].dS_dr.at(az) = tmp_dS;//for second method

            //центробежная сила от вращения винта Н
            b[l].Nr.at(az) = w*w* ( KA1 *cos(b[l].beta.at(az)) + ml*l_gsh );
            //центробежная сила от маховых движений Н
            b[l].Nf.at(az) = KA1 *w*w* b[l].dbeta.at(az)*b[l].dbeta.at(az);

            //ГШ, СКЛ
            b[l].T.at(az)  = get_Integral(b[l].dT_dr.at(az), r);
            b[l].Q.at(az)  = krd*get_Integral(b[l].dQ_dr.at(az), r);
            b[l].Ma.at(az) = get_Integral(b[l].dMa_dr.at(az), r);
            b[l].Ty.at(az) = b[l].T.at(az) *cos(b[l].beta.at(az));
            b[l].H.at(az)  =   -1*b[l].T.at(az) *sin(b[l].beta.at(az)) *cos(b[l].psi.at(az)) + krd*b[l].Q.at(az) *sin(b[l].psi.at(az));
            b[l].S.at(az)  = -krd*b[l].T.at(az) *sin(b[l].beta.at(az)) *sin(b[l].psi.at(az)) - b[l].Q.at(az) *cos(b[l].psi.at(az));//krd*
            //b[l].H.at(az)  = get_Integral(b[l].dH_dr.at(az), r);//second method
            //b[l].S.at(az)  = get_Integral(b[l].dS_dr.at(az), r);//second method
            //b[l].MQ.at(az) = krd*get_Integral(dMQ_dr.at(az), r);//момент силы сопротивления лопасти отн.оси винта (он же крутящий момент)
        }//end of "проход по азимутам"
    }
    //средние за оборот АЭРОДИНАМИЧЕСКИЕ !!! ПАРАМЕТРЫ ВИНТА
    T_sr = 0;   H_sr = 0;   S_sr = 0;
    Mx_sr= 0;   My_sr= 0;   Mz_sr= 0;
    for (int l = 0; l < zl; l++)
    {
        //средние за оборот от всех лопастей, СКПС
        T_sr  += (0.5/M_PI)* get_Integral(b[l].Ty, psi0);/// вес лопастей учтен далее
        H_sr  += (0.5/M_PI)* get_Integral(b[l].H , psi0);/// учесть отклонение лопастей!
        S_sr  += (0.5/M_PI)* get_Integral(b[l].S , psi0);/// учесть отклонение лопастей!
    }
    //равнодействующая сила на втулке
    Rxy= sqrt(T_sr*T_sr + H_sr*H_sr);
    Ra = sqrt(T_sr*T_sr + H_sr*H_sr + S_sr*S_sr);
    //проекции равнодействующей в скоростной СК
    Ya = T_sr*cos(alpha) - H_sr*sin(alpha);
    Xa = T_sr*sin(alpha) + H_sr*cos(alpha);
    //отклонение равнодействующей
    delta = atan2(Xa,Ya);
    //так надо для индуктивной скорости
    if(hubtype==2) H_i = H_sr;
    //
    writeln(&this->vsh, "RotorBase::generalCore finishes working");
    writeln(&this->vsh1, "RotorBase::generalCore finishes working");
    //
}

//void RotorBase::blade_kinematic_az(int n, bool krut)//расчет параметров движения лопасти //commented 25.11.2024
void RotorBase::blade_kinematic_az(int n, int krutMode)//расчет параметров движения лопасти //changed 25.11.2024 alt commented
{
    //  bet0 - угол взмаха на предыд. шаге, рад
    // dbet0 - скорость взмаха на предыд. шаге, рад [взмаха]/рад [оборота]
    //d2bet0 - ускорение взмаха на предыд. шаге, рад [взмаха]/рад [оборота]^2
    //d2beta - ускорение взмаха на текущем шаге, рад [взмаха]/рад [оборота]^2
    //  gam0 - угол закрутки на предыдущем шаге, рад
    // dgam0 - скорость закрутки на предыдущем шаге, рад[закрутки]/рад [оборота]
    //d2gam0 - ускорение закрутки на предыдущем шаге, рад[закрутки]/рад [оборота]^2
    //   psi - азимут, рад
    //     n - номер азимутального положения //considering Umdrehung Nr
    //
    //vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
    // krutMode:
    // 0 - not considering flutter or div
    // 2 - flutter or div by axial flow mode
    // 3 - ak 1 ma incog l'frict in axial lager
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    //
    this->CalcCalls1+=1;//2026/10/26
    //qDbg<<"blade_kinematic_az call N " <<this->CalcCalls1;//2026/10/26
    //
    //
    int q=0;
    bool brk = false;
    double bet0, dbet0, d2bet0, d2beta, beta, dbeta, beta1, dbeta1, deltab=1;
    double d2gam0, gam0, dgam0, d2gama, gama, dgama, gama1, dgama1, deltag=1;
    double ksi0, dksi0, d2ksi0, dksi, d2ksi, dksi1, ksi, ksi1, deltak=1;
    double fi7, fi7_, dfi7, dfi7_, d2fi7;
    double psi = n*dpsi;
    //
    QString qs, qsn;
    //
    //writeln(&this->vsh,  "blade_kinematic_az starts working");
    qsn.setNum(psi);
    qs="blade_kinematic_az starts working";
    qs=qs+" psi= ";
    qs=qs+qsn;
    qs=qs+" (rot.part: ";
    qsn.setNum(psi/2/M_PI);
    qs=qs+qsn;
    qs=qs+") ";
    qs=qs+" Call N ";
    qsn.setNum(this->CalcCalls1);
    qs+=qsn;
    writeln(&this->vsh, qs);
    writeln(&this->vsh1, qs);
    qsn="";
    qs="";
    //writeln(&this->vsh, ( "blade_kinematic_az starts working"+" psi= "+QString::setNum(psi)));
    //writeln(&this->vsh,  "blade_kinematic_az starts working");
    //writeln(&this->vsh,  "blade_kinematic_az starts working - Call N "+QString::setNum(this->CalcCalls1));
    //writeln(&this->vsh1, "blade_kinematic_az starts working");
    //
    //параметры на предыдущем азимуте
      bet0   = Beta.at(n-1);
     dbet0  = dBeta.at(n-1);
    d2bet0 = d2Beta.at(n-1);
      ksi0    = Ksi.at(n-1);
     dksi0   = dKsi.at(n-1);
    d2ksi0  = d2Ksi.at(n-1);
      gam0   = Gama.at(n-1);
     dgam0  = dGama.at(n-1);
    d2gam0=d2gama=d2Gama.at(n-1);
   fi7 = fi7_ = Fi7.at(n-1);
   dfi7=dfi7_= dFi7.at(n-1);

    //углы и скорости в первом приближении
    dbeta = dbet0 + d2bet0 *dpsi;
    beta  = bet0 + 0.5* (dbet0 + dbeta) *dpsi;
    dksi  = dksi0 + d2ksi0 *dpsi;
    ksi   = ksi0 + 0.5* (dksi0 + dksi) *dpsi;
    dgama = dgam0 + d2gam0 * dpsi;
    gama  = gam0 + 0.5* (dgam0 + dgama) *dpsi;
    //if(!krut) { dgama=0; gama=0; }//commented 25.11.2024
    if(krutMode!=1) { dgama=0; gama=0; }//changed 25.11.2024 alt commented
    //угол установки сечения 07R в первом приближении
    for (int rr=0; rr<nSec; rr++)
    {
        seg = Segment(rr, psi, beta, dbeta, gama, dgama, dfi7);
        if(rr==sec07)  fi7 = seg.fi;
    }
    //if(krut) {
    dfi7 = ( fi7 -  fi7_)/dpsi;
    d2fi7= (dfi7 - dfi7_)/dpsi;
    //} else { dfi7 = d2fi7 = 0; }
    //ускорения в первом приближении
    switch (hubtype)
      {
      case 0://шарнирный(ОШ+ГШ+ВШ)
      case 1://шарнирный(ОШ+ГШ)
        //d2beta = get_d2beta_hinge(psi, beta, fi7+gama, d2fi7+d2gama, get_IntegralMdT1(psi, beta, dbeta, gama, dgama, dfi7));
        d2beta = get_d2beta_hinge(psi, beta, gama, d2fi7+d2gama, get_IntegralMdT1(psi, beta, dbeta, gama, dgama, dfi7));
        d2ksi  = get_d2ksi_hinge(psi, beta, dbeta, ksi, dksi);
        break;
      case 2://качалка карданного типа (ОШ+СГШ)
        //d2beta = get_d2beta_kardan(psi, beta, fi7+gama, d2fi7+d2gama, get_IntegralMdT1(psi, beta, dbeta, gama, dgama, dfi7), get_IntegralMdT1(psi+M_PI, -beta, -dbeta, -gama, -dgama, -dfi7));
        d2beta = get_d2beta_kardan(psi, beta, gama, d2fi7+d2gama, get_IntegralMdT1(psi, beta, dbeta, gama, dgama, dfi7), get_IntegralMdT1(psi+M_PI, -beta, -dbeta, -gama, -dgama, -dfi7));
        d2ksi = 0;   dksi = 0;   ksi = 0;   deltak = 0;
        break;
      case 3://бесшарнирный(ОШ)
        d2beta = 0;  dbeta = 0;  beta = 0;  deltab = 0;
        d2ksi = 0;   dksi = 0;   ksi = 0;   deltak = 0;
        break;
      }
    d2gama = get_d2gama( beta, d2beta, fi7, d2fi7, gama, dgama, dfi7, get_IntegralMdMa(psi, beta, dbeta, gama, dgama, dfi7));
    //if(!krut) d2gama=0; //commented 25.11.2024
    if(krutMode==0) d2gama=0; //changed 25.11.2024
    //
    qs="Starting iterations: ";
    qsn.setNum(deltab);
    qs=qs+" deltab=";
    qs=qs+qsn;
    qsn.setNum(0.0001*prec);
    qs=qs+" 0.0001*prec=";
    qs=qs+qsn;
    writeln(&this->vsh, qs);
    //writeln(&this->vsh1, qs);
    //
    //Итерационное нахождение угла и скoрости взмаха
    while (deltab > 0.0001*prec)//0.00001...0.0000001 //для балансировки достаточно 0.00001
    {
        dbeta1 = dbet0 + 0.5* (d2beta + d2bet0) *dpsi;
         beta1 =  bet0 + 0.5* ( dbeta1 + dbet0) *dpsi;
         dksi1 = dksi0 + 0.5* (d2ksi  + d2ksi0) *dpsi;
          ksi1 =  ksi0 + 0.5* ( dksi1  + dksi0) *dpsi;
        dgama1 = dgam0 + 0.5* (d2gama + d2gam0) *dpsi;
         gama1 =  gam0 + 0.5* ( dgama1 + dgam0) *dpsi;
        //if(!krut) { dgama1=0; gama1=0; }//commented 25.11.2024
        if(krutMode!=1) { dgama1=0; gama1=0; }

        //угол установки сечения 07R и скорость его изменения
        for (int rr=0; rr<nSec; rr++)
        {
            seg = Segment(rr, psi, beta1, dbeta1, gama1, dgama1, dfi7);
            if(rr==sec07)  fi7 = seg.fi;
        }
        // so lif'te R. Qob ne so: seg=Segment(sec07,beta1, dbeta1, gama1, dgama1, dfi7);
        // fi7 = seg.fi;
        // Mab sec07 s'reserv'd, ma ne real'd? Ma tams ==, so S'real'd!
        //if(krut) {
        dfi7 = ( fi7 -  fi7_)/dpsi;
        d2fi7= (dfi7 - dfi7_)/dpsi;
        //} else { dfi7 = d2fi7 = 0; }

        switch (hubtype)
          {
          case 0://шарнирный(ОШ+ГШ+ВШ)
          case 1://шарнирный(ОШ+ГШ)
            //d2beta = get_d2beta_hinge(psi, beta1, fi7+gama1, d2fi7+d2gama, get_IntegralMdT1(psi, beta1, dbeta1, gama1, dgama1, dfi7));
            d2beta = get_d2beta_hinge(psi, beta1, gama1, d2fi7+d2gama, get_IntegralMdT1(psi, beta1, dbeta1, gama1, dgama1, dfi7));
            d2ksi  = get_d2ksi_hinge(psi, beta, dbeta, ksi1, dksi1);
            break;
          case 2://качалка карданного типа (ОШ+СГШ)
            //d2beta = get_d2beta_kardan(psi, beta1, fi7+gama1, d2fi7+d2gama, get_IntegralMdT1(psi, beta1, dbeta1, gama1, dgama1, dfi7), get_IntegralMdT1(psi+M_PI, -beta1, -dbeta1, -gama, -dgama, -dfi7));
            d2beta = get_d2beta_kardan(psi, beta1, gama1, d2fi7+d2gama, get_IntegralMdT1(psi, beta1, dbeta1, gama1, dgama1, dfi7), get_IntegralMdT1(psi+M_PI, -beta1, -dbeta1, -gama, -dgama, -dfi7));
            d2ksi  = 0;
            break;
          case 3://
            d2beta = 0;
            d2ksi  = 0;
            break;
          }
        d2gama = get_d2gama( beta1, d2beta, fi7, d2fi7, gama1, dgama1, dfi7, get_IntegralMdMa(psi, beta1, dbeta1, gama1, dgama1, dfi7));
        //if(!krut) d2gama=0;//commented 25.11.2024
        if(krutMode!=1) d2gama=0; //changed 25.11.2024

        deltab = fabs(beta1 - beta);// / beta1
        deltag = fabs(gama1 - gama);
        deltak = fabs( ksi1 - ksi );

        beta   = beta1;
        dbeta  = dbeta1;
        if (beta < bet_min) {  beta = bet_min;  brk=true; }//  dbeta = 88;  d2beta = 888;
        if (beta > bet_max) {  beta = bet_max;  brk=true; }//  dbeta = 88;  d2beta = 888;

        gama   = gama1;
        dgama  = dgama1;
        if (gama < gam_min) {  gama = gam_min;  brk=true; }//  dgama = 88;  d2gama = 888;
        if (gama > gam_max) {  gama = gam_max;  brk=true; }//  dgama = 88;  d2gama = 888;

        ksi  = ksi1;
        dksi = dksi1;
        if (ksi < ksi_min+ksi_nom) {  ksi = ksi_min+ksi_nom;  brk=true; }//  dksi = 88;  d2ksi = 888;
        if (ksi > ksi_max+ksi_nom) {  ksi = ksi_max+ksi_nom;  brk=true; }//  dksi = 88;  d2ksi = 888;

        q++;
        if(q>999) brk=true;
        if(brk) break;
        //
        qs="It was IterN: ";
        qsn.setNum(q);
        //qs=qs+"q=";
        qs=qs+qsn;
        qsn.setNum(deltab);
        qs=qs+" deltab=";
        qs=qs+qsn;
        qsn.setNum(0.0001*prec);
        qs=qs+" 0.0001*prec=";
        qs=qs+qsn;
        writeln(&this->vsh, qs);
        //writeln(&this->vsh1, qs);
    }//while
    if(deltab > 0.0001*prec) warning=311;
    if(deltag >  0.001*prec) warning=310;
    if(deltak >  0.001*prec) warning=312;

    (beta!=0)?(deltaBeta.push_back(deltab/beta)):(deltaBeta.push_back(0));
    (gama!=0)?(deltaGama.push_back(deltag/gama)):(deltaGama.push_back(0));
    ( ksi!=0)?( deltaKsi.push_back(deltak/ ksi)):( deltaKsi.push_back(0));

    /*/окончательные значения параметров на текущем шаге
    dbeta = dbet0 + d2beta * dpsi;
     beta =  bet0 +  dbeta * dpsi;
    dgama = dgam0 + d2gama * dpsi;
     gama =  gam0 +  dgama * dpsi;
     dksi = dksi0 +  d2ksi * dpsi;
      ksi =  ksi0 +   dksi * dpsi;
    for (int rr=0; rr<nSec; rr++){
        seg = Segment(rr, psi, beta, dbeta, gama, dgama, dfi7);
        if(rr==sec07) fi7 = seg.fi;
    }
     dfi7 = ( fi7 - fi7_)/dpsi;
    d2fi7 = (dfi7 -dfi7_)/dpsi;*/
    //сохраняем окончательные значения параметров
      Beta.push_back(  beta);
     dBeta.push_back( dbeta);
    d2Beta.push_back(d2beta);
      Gama.push_back(  gama);
     dGama.push_back( dgama);
    d2Gama.push_back(d2gama);
       Ksi.push_back(   ksi);
      dKsi.push_back(  dksi);
     d2Ksi.push_back( d2ksi);
       Fi7.push_back(   fi7);
      dFi7.push_back(  dfi7);
     d2Fi7.push_back( d2fi7);

     //vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
     // I reloc'te S ad fin ob I not beta et theta. Ma qwoe theta0?
     //params from Mil flutter, added 25.11.2024
     //const double _PI=3.1415927;
     //this->Theta1, this->Theta2
     //Ini params
     //double sigma_param,  //p 359 - dist btwn axe o'ax'l hinge et weight ebnter - ms'b ini param uz blade
     //       sigma_f_param, // p 356 - dist btwn focus et rigidity axis - ms'b ini param uz blade
     //       theta0, // p 360 angle of setting o'blade's butt => r_=min (or spec)
     //       d_frict=0.35,//frict coef, steel-steel, ob tams Lager => ms'b steel-steel rolling
     //       c_ctrl; //p 360 - rigidity coef o'controlling sys
     double here_r, here_r_;
     int profN_cur, profN_next, N_M_prev, N_M_next;
     //TPosInSucc M_Pos_cur, M_Pos_next;
     //double alfa_cur, alfa_next, v_cur, v_next, //no not ob es Segment()
     double M_cur, M_next,
            b_param_cur, b_param_next;//b wa ud
     segment seg_cur, seg_next;

     // for (int rr=0; rr<nSec; rr++) //ak in neto loc lif'd
     this->fltrModel.SetIntegrationDataNull();//sums mub assign'd 0 ef integr
     for(int secN=0; secN<nSec-1; secN++){//integral cycle
         b_param_cur=bh[secN];   // b_param_next, b_param_cur
         b_param_next=bh[secN+1];  //b_param_next, b_param_cur
         //
         //  def'n dCy/dalfa for cur et next sects
         profN_cur=prof[secN];
         profN_next=prof[secN+1];

         seg_cur=this->Segment(secN, psi, beta1, dbeta1, gama1, dgama1, dfi7);
         seg_next=this->Segment(secN+1, psi, beta1, dbeta1, gama1, dgama1, dfi7);
         //calc dCy_dalfa uz cur
         M_cur=seg_cur.M;
         //
         M_next=seg_next.M;
         //
         this->dataIncomingFlutter.b7=this->b7;//this->flIniData.b7=this->b7;
         this->dataIncomingFlutter.chi_param=this->chi_sr;//this->flIniData.chi_param=this->chi_sr;//es ety chi
         //
         //this->dataIncomingFlutter.I_gsh=this->I_gsh;//this->flIniData.I_gsh=this->I_gsh;
         //this->dataIncomingFlutter.I_osh=this->I_osh;//this->flIniData.I_osh=this->I_osh;
         //this->dataIncomingFlutter.mlp=this->mlp;// this->flIniData.mlp=this->mlp;//es ety ml
         //this->dataIncomingFlutter.R=this->R;//this->flIniData.R=this->R;
         //this->dataIncomingFlutter.ro=this->ro;//this->flIniData.ro=this->ro;
         //this->dataIncomingFlutter.sigma=this->sigma;//this->flIniData.sigma=this->sigma;
         //
         //this->dataIncomingFlutter.w=this->w;//this->flIniData.w=this->w;
         //
         this->fltrModel.SetIncomingParams(this->dataIncomingFlutter);
         //
         //void ...::Calc1_IntegrateBySects(int SecN, double M_cur, double M_next, int QMs, double MACHs[5][CM][3], int QAlfas, double ALPHAs[5][Ca][3], double _Cy[5][CM][Ca], double _Cx[5][CM][Ca], double _Mz[5][CM][Ca], int QSects, const int*prof_N, const double*bh, const double*r, double _x_ref,   TValsShowHide*vsh)
         //vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
         if(krutMode!=0){
            fltrModel.Calc1_IntegrateBySects( secN,        M_cur,        M_next,     CM,                MACH,      Ca,                    ALPHA,               this->_Cy,     this->_Cx,              this->_Mz,                  nSec, this->prof.data(),this->bh.data(), this->r.data(),       0,             &this->vsh);
         }
         //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
     }//intrgral cycle
     //
     //flParamsForEquations=fltrModel.GetParamsForEquations();
     //
      fltrModel.Calc2_char(); //1111111111111111111111111111111111111111111111111111111111111
     //
     //fltrModel.CalcDeviations();//ma ei, deco'd vats or vect o'all geq?
     //
     //integrating diff.equatioms!
     //
     //gamma0_param=dCy_dalfa*ro*b7*R*R*R*R/(2*I_gsh);
     //
     //c21=-1/I_osh*c21_SubInt;
     //c12=i0_param*c21;
     //d11=dCy_dalfa/2*ro/I_gsh*d11_SubInt;
     //d12=-dCy_dalfa/2*ro/I_gsh*d12_SubInt;
     //d21=-dCy_dalfa/2*ro/I_gsh*d21_SubInt;
     //d22=1/I_osh*(M_PI/16*ro*d22_SubInt1 + dCy_dalfa*ro/2*d22_SubInt2) + d_frict;
     //b12=-dCy_dalfa/2*ro/I_gsh*b12_SubInt+i0_param*c21;
     //b22=1+dCy_dalfa/2*ro/I_gsh*b22_SubInt;
     //
     //A1=1/(1-i0_param*c21*c21)*(d11+d12-c12*d21-c21*d12);//pg363
     //B1=1/(1-i0_param*c21*c21)*(1+b22+d11*d22-b12*c21-d12*d21-c12*c21);
     //B2=1/(1-i0_param*c21*c21)*(1-chi_param*c12);
     //C1=1/(1-i0_param*c21*c21)*(d22+d11*b22-b12*d21-c21*d12);
     //C2=1/(1-i0_param*c21*c21)*(d11-chi_param*d12);
     //D1=1/(1-i0_param*c21*c21)*(b22-b12*c21);
     //D2=1/(1-i0_param*c21*c21)*(1-chi_param*b12);
     //
     //Cp4=1;
     //Cp3=A1*w_;
     //Cp2=B1*w_*w_+B2;
     //Cp1=(C1*w_*w_+C2)*w_;
     //Cp0=D1*w_*w_*w_*w_+D2*w_*w_;
     //
     //int EquationOf4thPower(double c4, double c3, double c2, double c1, double c0, std::vector<double>&ReX, std::vector<double>&ReY);
     //EquationOf4thPower(Cp4, Cp3, Cp2, Cp1, Cp0, ReX, ImX);
     //
     ////double __lambdaI_Re[2], __lambdaI_Im[2],  __lambdaII_Re[2], __lambdaII_Im[2],
     ////         lambdaI_Re[2],   lambdaI_Im[2],    lambdaII_Re[2],   lambdaII_Im[2],
     ////         _q1, _p1, _q2, _p2;
     //
     ////Analyzing roots, ob V nved, qly S'solv'd, qics're, qics'cpx
     ////_q1=ReX[1-1];
     ////_p1=ImX[1-1];
     ////__lambdaI_Re[1-1]=_q1;
     ////__lambdaI_Re[2-1]=_q1;
     ////__lambdaI_Im[1-1]=_p1;
     ////__lambdaI_Im[2-1]=_p1*(-1);
     //not so, pair1 can be not complex
     //if(fApprNonZero(ImX[1-1], 0.000001)){
         //1st root s'complex
     //    _q2=ReX[1-1];
     //    _p2=ImX[1-1];
     //    __lambdaII_Re[1-1]=_q2;
     //    __lambdaII_Re[2-1]=_q2;
     //    __lambdaII_Im[1-1]=_p2;
     //    __lambdaII_Im[2-1]=_p2*(-1);//Ak I int pg 365 et 366, II s'alum cpx, et I ablb cpx or re, so V start amda ab II
     //    if(fApprNonZero(ImX[2-1], 0.000001)){
     //        //2nd root s'cpx
     //        if(fApprET(ReX[1-1], ReX[2-1], 0.000001) &&  fApprET(ImX[1-1], -ImX[2-1], 0.000001)){
                 //2nd root s'conjugated, no omcor. Nu 3rd et 4th.
     //            if(fApprNonZero(ImX[3-1], 0.000001)){
     //                //3rd s'cpx =>  4th s'conjug'd, no ato vrns
     //                _q1=ReX[3-1];
     //                _p1=ImX[3-1];
     //                __lambdaI_Re[1-1]=_q1;
     //                __lambdaI_Re[2-1]=_q1;
     //                __lambdaI_Im[1-1]=_p1;
     //                __lambdaI_Im[2-1]=_p1*(-1);
     //            }else{
                     //3rd s're => 4th s're => iqv qnS'diverg!
     //                _q1=ReX[3-1];
     //                _p1=ImX[3-1];
     //                __lambdaI_Re[1-1]=ReX[3-1];
     //                __lambdaI_Im[1-1]=0;
     //                __lambdaI_Re[2-1]=ReX[4-1];
     //                __lambdaI_Im[2-1]=0;
     //            }
     //        }else{
                 //2nd root s'cpx, ma n'conjug'd co 1st=> 1 of 3 et 4 s' conjug'd co 1st, et 1 - co 2nd.
     //            if(fApprET(ReX[1-1], ReX[3-1], 0.000001) &&  fApprET(ImX[1-1], -ImX[3-1], 0.000001)){
                     //3rd root s'conjug'd co 1st => 4th s'co 2nd
      //               _q1=ReX[3-1];
      //               _p1=ImX[3-1];
      //               __lambdaI_Re[1-1]=_q1;
      //               __lambdaI_Re[2-1]=_q1;
      //               __lambdaI_Im[1-1]=_p1;
      //               __lambdaI_Im[2-1]=_p1*(-1);
      //           }else{
                     //4th root s'conjug'd co 1st => 3rd s'co 2nd
      //               _q1=ReX[2-1];
      //               _p1=ImX[2-1];
      //               __lambdaI_Re[1-1]=_q1;
      //               __lambdaI_Re[2-1]=_q1;
      //               __lambdaI_Im[1-1]=_p1;
      //               __lambdaI_Im[2-1]=_p1*(-1);
      //           }
      //       }
      //   }else{
             //2nd root ns'cpx => conjug'd s'id cmx, ato s're
      //       if(fApprNonZero(ImX[3-1], 0.000001)){
                 //3rd s'cpx=> conjug'd co 1st et ns'omcor
      //           if(fApprET(ReX[1-1], ReX[3-1], 0.000001) &&  fApprET(ImX[1-1], -ImX[3-1], 0.000001)){
                     //report error
      //           }else{
      //               _q1=ReX[2-1];
      //               _p1=ImX[2-1];
      //               __lambdaI_Re[1-1]=ReX[2-1];
      //               __lambdaI_Im[1-1]=0;
      //               __lambdaI_Re[2-1]=ReX[4-1];
      //               __lambdaI_Im[2-1]=0;
      //           }
      //       }else{
                 //3rde s're => 4th s'cpx => conjug'd co 1st et ns'omcor
      //           if(fApprET(ReX[1-1], ReX[4-1], 0.000001) &&  fApprET(ImX[1-1], -ImX[4-1], 0.000001)){
                     //report error
      //           }else{
      //               _q1=ReX[3-1];
      //               _p1=ImX[3-1];
      //               __lambdaI_Re[1-1]=ReX[2-1];
      //               __lambdaI_Im[1-1]=0;
      //               __lambdaI_Re[2-1]=ReX[3-1];
      //               __lambdaI_Im[2-1]=0;
      //           }
     //        }
     //    }
     //}else{
     //    //1st s're
     //    _q1=ReX[1-1];
     //    _p1=ImX[1-1];
     //    __lambdaI_Re[1-1]=ReX[1-1];
    //     __lambdaI_Im[1-1]=0;
     //    if(fApprNonZero(ImX[2-1], 0.000001)){
             //2nd s'cpx
    //         _q2=ReX[2-1];
     //        _p2=ImX[2-1];
     //        __lambdaII_Re[1-1]=_q2;
     //        __lambdaII_Re[2-1]=_q2;
     //        __lambdaII_Im[1-1]=_p2;
     //        __lambdaII_Im[2-1]=_p2*(-1);
     //        if(fApprNonZero(ImX[3-1], 0.000001)){
                 //3nd s'cpx => conjug'd co 2 et ns'omcor => 4th s're
     //            __lambdaI_Re[2-1]=ReX[4-1];
     //            __lambdaI_Im[2-1]=0;
     //        }else{
                 //1st et 3rd s're, 2nd s'cpx => 4th s'conjug'd et ns'omcor =>  3rd s're
     //            if(fApprNonZero(ImX[4-1], 0.000001)==false){
                     //report error: min 1 pair s'cpx
     //            }
     //            __lambdaI_Re[2-1]=ReX[3-1];
     //            __lambdaI_Im[2-1]=0;
     //        }
     //    }else{
             //2nd s're =>
     //        __lambdaI_Re[2-1]=ReX[2-1];
     //        __lambdaI_Im[2-1]=0;
     //        if(fApprNonZero(__lambdaI_Im[2-1], 0.000001)==false || fApprNonZero(ImX[4-1], 0.000001)==false){
                 //report error: min 1 pair s'cpx.
     //        }else{
                 //3rd s' cpx => 4th s'p et conjug'd et ns'omcor
     //            _q2=ReX[3-1];
     //            _p2=ImX[3-1];
     //            __lambdaII_Re[1-1]=_q2;
     //            __lambdaII_Im[1-1]=_p2;
     //            __lambdaII_Re[2-1]=_q2;
     //            __lambdaII_Im[2-1]=_p2*(-1);
     //        }
     //    }
     //}
     //cfL=(C1*(2*C2-A1*B2)+A1*(A1*D2-B2*C2))/(2*(C1*C1+A1*(A1*D1-B1*C1)));
     //cfM=C2*(C2-A1*B2)/(C1*C1+A1*(A1*D1-B1*C1));
     //
     // flutter s'if _q2>0 - p.364
     // n_crit_flutter = n(MR)|_q2=0 - p.365
     // n_crit_diverg = n(MR)|_lambdaI1=0 - p.366, Im(lambdaI==0), 2 real roots
     //
     //if(fApprNonZero( __lambdaI_Im[1-1], 0.000001)==false || fApprNonZero( __lambdaI_Im[2-1], 0.000001)==false){
         //diverg
     //    _w_diverg = sqrt((_k*b12-1)/(b22-b12*c21));
     //}else if(_q2>=0){
         //flutter
     //    _w_fl_cr_1 = sqrt(cfL-sqrt(cfL*cfL-cfM));
     //    _w_fl_cr_2 = sqrt(cfL+sqrt(cfL*cfL-cfM));
     //    _p_fl_cr_1 = sqrt((C1*_w_fl_cr_1*_w_fl_cr_1  +C2)/A1);
     //    _p_fl_cr_2 = sqrt((C1*_w_fl_cr_2*_w_fl_cr_2 + C2)/A1);
     //}
     //nu question. S'calc'd


     //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      //
      writeln(&this->vsh, "blade_kinematic_az finishes working");
      //writeln(&this->vsh1, "blade_kinematic_az finishes working");
      //
}//kyn_az

double RotorBase::get_d2beta_hinge(double psi, double beta, double figa, double d2figa, double Int_MdT)// расчет ускорения взмаха
{
    //Int_MdT   момент силы тяги отн. ГШ, Н*м
    //beta      угол взмаха, рад
    //return ускорение взмаха d2beta/dpsi^2, рад[взмаха] / рад[оборота] / рад[оборота]
    double M1Gx = 0;//момент отн.ГШ веса лопасти
    double M1Cx = 0;//момент отн.ГШ центробежной силы
    double MPay = 0;//Момент отн.ГШ силы инерции от перемещения вдоль оси НВ:
    double MPexz= 0;//Момент отн.ГШ силы инерции от вращения НВ вокруг продольной и поперечной осей CKНВ
    double MPkxz= 0;//Момент отн.ГШ силы Кориолиса от вращения НВ вокруг осей СКНВ
    double M1Cz = 0;//момент инерционных+центробежных сил отн.OШ

    //if ( I_osh && k_pov && zakrut )
        M1Cz = (d2figa+figa) *w*w *Icb ;

    switch (ROTORID)
    {
    case MR://шарнирная втулка на НВ
        MPay = -a_y*KA1*cos(beta);
        MPexz= KC1*cos(beta)*( e_x*sin(psi) - e_z*cos(psi) );
        MPkxz= KC1*cos(beta)*( krd*w_x*cos(psi) + w_z*sin(psi) )*2*w;
        M1Gx = -g*KA1*cos(beta);
        M1Cx = -0.5*KB1*w*w*sin(2*beta) -KA1*l_gsh*w*w*sin(beta);
        break;
    case TR://шарнирная втулка на РВ
        M1Gx = krd*g*KA1*sin(beta)*sin(psi);
        M1Cx = -0.5*KB1*w*w*sin(2*beta) -KA1*l_gsh*w*w*sin(beta);
        break;
    case PR://не проверял   шарнирная втулка на TВ
        M1Gx = krd*g*KA1*sin(beta)*sin(psi);
        M1Cx = -0.5*KB1*w*w*sin(2*beta) -KA1*l_gsh*w*w*sin(beta);
        break;
    }
    return ( Int_MdT + M1Gx + M1Cx + MPay + MPexz + MPkxz + M1Cz ) / (w*w*KB1);
}
double RotorBase::get_d2beta_kardan(double psi, double beta, double figa, double d2figa, double Int_MdT1, double Int_MdT2)// расчет ускорения взмаха для втулки-качалки на 2 лопасти
{
    //MdT1 момент силы тяги 1й лопасти отн. ГШ, Н*м
    //MdT2 момент силы тяги противоположной лопасти отн. ГШ, Н*м
    //beta угол взмаха, рад
    //return {double} ускорение взмаха d2beta/dpsi^2, рад[взмаха] / рад[оборота] / рад[оборота]
    double M1Cx = 0;//момент центробежной силы отн.ГШ
    double MPexz= 0;//Момент отн.ГШ силы инерции от вращения НВ вокруг продольной и поперечной осей CKПС
    double MPkxz= 0;//Момент отн.ГШ силы Кориолиса от вращения НВ вокруг его осей
    double M1Cz = 0;//момент инерционных+центробежных сил отн.OШ

    //if ( I_osh && k_pov && zakrut )
        M1Cz = (d2figa+figa) *w*w *Icb;

    switch (ROTORID)
    {
    case MR://карданная втулка на НВ
        MPexz= I_gsh*cos(beta)*( e_x*sin(psi) - e_z*cos(psi) );
        MPkxz= I_gsh*cos(beta)*( krd*w_x*cos(psi) + w_z*sin(psi) )*2*w;
        M1Cx = 0.5*I_gsh *w*w *sin(2*beta);
        break;
    case TR://карданная втулка на РВ
        M1Cx = 0.5*I_gsh *w*w *sin(2*beta);
        break;
    case PR://карданная втулка на ТВ
        M1Cx = 0.5*I_gsh *w*w *sin(2*beta);
        break;
    }
    return (Int_MdT2 - Int_MdT1 + 2*M1Cx + 2*MPexz + 2*MPkxz + M1Cz ) / (-2*w*w*I_gsh);
}

double RotorBase::get_d2ksi_hinge(double psi, double beta, double dbeta, double ksi, double dksi)//расчет ускорения качания
{
    double M2K  = 0;//момент отн.ВШ силы Кориолиса, Н*м
    double M2Q  = 0;//момент отн.ВШ силы сопротивления лопасти отн. ВШ, Н*м
    double M2Cy = 0, Nr = 0, khi = 0;//момент отн.ВШ центробежной силы
    double M2D  = 0, Fdmp = 0;//момент отн.ВШ и сила на штоке демпфера лопасти
    double MPaxz= 0;//момент отн.ВШ силы инерции в плоскости винта от линейного перемещения вдоль продольной и поперечной осей CKПС
    double M2Gy = 0;//момент отн.ВШ веса лопасти
    //return ускорение качания d2ksi/dpsi^2, рад[качания] / рад[оборота] / рад[оборота]
    switch (ROTORID)
    {
    case MR://шарнирная втулка на НВ
        M2K = -KE *w*w* sin(2*beta) * dbeta;
        M2Q = get_IntegralMdQ2(psi, beta, dbeta, 0, 0, 0);
        Nr = w*w* ( KA1*cos(beta) + ml*l_gsh );
        khi = ksi * (1-l_vsh/(0.5*R));
        M2Cy = -Nr * l_vsh * sin(khi);
        Fdmp = (damper)?(DampherForce( DampherSpeed(ksi,dksi) ) ):( 0 );
        M2D = Fdmp * d.h;
        MPaxz = -KA2*cos(beta)*( a_x*sin(psi) - a_z*cos(psi) );
        break;
    case TR:///шарнирную втулку на РВ еще не проверял
        M2K = -KE *w*w* sin(2*beta) *dbeta;
        M2Q = get_IntegralMdQ2(psi, beta, dbeta, 0, 0, 0);
        Nr  = w*w* ( KA1*cos(beta) + ml*l_gsh );
        khi = ksi *(1-l_vsh/(0.5*R));
        M2Cy = -Nr *l_vsh *sin(khi);
        Fdmp = (damper)?(DampherForce( DampherSpeed(ksi,dksi) ) ):( 0 );
        M2D = Fdmp *d.h;
        M2Gy = g*KA2 *sin(beta) *cos(psi);
        //return ( M2Q + M2K + M2Cy + M2D + M2Gy )/(w*w*KB2);
        break;
    case PR:/// шарнирную втулку на TВ еще не проверял
        M2K = -KE *w*w* sin(2*beta) *dbeta;
        M2Q = get_IntegralMdQ2(psi, beta, dbeta, 0, 0, 0);
        Nr  = w*w* ( KA1*cos(beta) + ml*l_gsh );
        khi = ksi *(1-l_vsh/(0.5*R));
        M2Cy = -Nr *l_vsh *sin(khi);
        Fdmp = (damper)?(DampherForce( DampherSpeed(ksi,dksi) ) ):( 0 );
        M2D = Fdmp *d.h;
        M2Gy = g*KA2 *sin(beta) *cos(psi);
        //return ( M2Q + M2K + M2Cy + M2D + M2Gy )/(w*w*KB2);
        break;
    }
    return ( M2Q + M2K + M2Cy + M2D + M2Gy + MPaxz ) / (w*w*KB2);///////////////////////
}
double RotorBase::get_d2ksi_kardan(double psi, double beta, double ksi, double Int_MdQ1, double Int_MdQ2)//расчет ускорения качания
{
    /*double M2kr = 0;//момент от закручивания вала
    double _d2ksi_ = 0;
    switch (ROTORID)
    {
    case MR://карданная втулка на НВ

        return 0;
        break;
    case TR:
        M2kr = -cy * ksi;
        _d2ksi_ = (Int_MdQ2 + Int_MdQ1 +  M2kr) / (2*w*w*I_vsh);///поменял
        if(cy) return _d2ksi_;
        else   return 0;
        break;
    }*/
return 0;
}

double RotorBase::get_d2gama(double beta, double d2beta, double fi, double d2fi, double gama, double dgama, double dfi, double Int_MdMa)// расчет ускорения закрутки
{
    //Int_MdMa  момент аэродин.сил профиля отн.ОШ, Н*м
    //gama     угол закрутки, рад
    //return  ускорение закрутки d2gama/dpsi^2, рад / рад[оборота] / рад[оборота]
    double M3su = 0, M3Cz = 0, M3tr = 0, M3dmp = 0, M3iz = 0;
    double d2figa = 0;

    ///момент силы в тяге системы управления
    stiff_su = k_pov *r_pov*r_pov;//Жорсткість системи управління кутова навколо осьового шарніра
    M3su = -gama *stiff_su;
    //B[l].delta_pov = B[l].gama*r_pov;
    //B[l].Fpov = k_pov * B[l].delta_pov;//B[l].M3su / r_pov;
    //B[l].M3su = -B[l].Fpov *r_pov;//-B[l].gama * stiff_su;

    ///момент инерционных+центробежных сил        //double Icb = Sb0 *(xo-xt)*chord;
    //double Int_Idfidr = I_osh*krutka;//get_Integral(_Jmdfi_dr, r);
    M3Cz = -(d2beta+beta)* w*w*Icb ;///+ w*w* Int_Idfidr;

    double Nr = w*w* Sb0;
    double dfiga = dgama + dfi;
    ///момент сил трения
    if( dfiga==0 || f_pod==0 ) M3tr = 0; else
    M3tr = (dfiga>=0)?( -f_pod * r_pod * Nr ):( f_pod * r_pod * Nr );
    /// и демпфирования
    M3dmp = gamaDamph(dgama);

    //момент сил инерции закрутки
    M3iz =  Int_MdMa + M3su + M3Cz + M3tr+M3dmp - (fi+gama)*(w*w*I_osh);

    if ( I_osh && k_pov && zakrut )
    {
        //ускорение установки+закрутки
        d2figa = M3iz / (w*w*I_osh);
        //ускорение закрутки
        return d2figa - d2fi;//d2gama;
    }
    else
        return 0;
}
double RotorBase::gamaDamph(double dgama)//момент демпфирующих сил на ОШ
{
    double Mdmp=0;
    //момент демпфирующих сил на ОШ
    if (dgama < 0)  Mdmp = 0 + sigma3 *dgama*dgama *w*w;
    if (dgama > 0)  Mdmp = 0 - sigma3 *dgama*dgama *w*w;
    if (dgama ==0 || sigma3==0)  Mdmp = 0;
return Mdmp;
}

segment RotorBase::Segment(int i, double psi, double beta, double dbeta, double gama, double dgama, double dfi7)//расчет массива параметров в сечении
{
    //i       номер сечения
    //psi     азимут, рад
    //beta    угол взмаха, рад
    //dbeta   скорость взмаха,  рад[взмаха]/рад[оборота]
    //gama    угол закрутки, рад
    //dgama   угл.скорость закрутки сечения, рад[закрутки]/c
    //return  структура seg
    segment seg;
    double __r = _r.at(i), cy, cx, mz;

    double lambda = get_lambda(__r, psi);
           seg.Uywz = -w_z*__r*R*cos(psi);
           seg.Uywx =  w_x*__r*R*sin(psi);
    double Uy    = wR*get_Uy(__r, lambda, psi, beta, dbeta) +seg.Uywz +seg.Uywx;
    double Ux    = wR*get_Ux(__r, psi, beta);
    double U     = sqrt(Ux*Ux + Uy*Uy);
    double M     = get_M(U);
    double fi    = get_fi(i, psi, beta);
    double Fi    = get_Fi(Ux, Uy);
    double alfar = get_alfa_r(fi + gama, Fi);
    double dfidt = ( dfi7 + dgama ) *w;//(dfi7 + dgama) *w;
    if (r.at(i) <= rB)
    {
        cx = get_Cx(prof.at(i), M, alfar) + dCxp;
        cy = get_Cy(prof.at(i), M, alfar);
        mz = get_Mz(prof.at(i), M, alfar);
    }
    else
    {
        cx = get_Cxp0(prof.at(i), M) + dCxp;
        cy = 0;
        mz = 0;
        dfidt = 0;
    }
    seg.lambda = lambda;
    seg.Ux     = Ux;
    seg.Uy     = Uy;
    seg.U      = U;
    seg.Fi     = Fi;
    seg.fi     = fi;
    seg.fl     = gama;
    seg.alfar  = alfar;
    seg.M      = M;
    seg.cy     = cy;
    seg.cx     = cx;
    seg.mz     = mz;
    //погонное сопротивление на участке
    seg.dQ_dr  = 0.5*ro*bh[i]*U*( cx*Ux - cy*Uy );
    //погонная тяга на участке
    seg.dT_dr  = 0.5*ro*bh[i]*U*( cy*Ux + cx*Uy ) + M_PI*ro*bh[i]*U*dfidt*( 0.75*bh[i] - xo*bh[i] );
    //погонный момент аэродин.сил
    ///seg.dMa_dr = seg.dT_dr*xo*bh[i] + 0.5*ro*bh[i]*U*( mz*bh[i]*U ) - 0.5*ro*bh[i]*U*( 0.125*M_PI*bh[i]*bh[i]*dfidt );
    double xf  = f_xf(M);//0.2;
    double sigf= (xf - xo)*bh[i];
    seg.dMa_dr = -seg.dT_dr*sigf -0.0625*M_PI*ro*pow(bh[i],3)*Ux*dfidt;//[Мil1,с.357]
    //центр давления
    seg.xd = xo + seg.dMa_dr / (seg.dT_dr*cos(seg.fi) + seg.dQ_dr*sin(seg.fi)) / bh[i];
return seg;
}

double RotorBase::f_xf(double M)//
{
    double xf = 0.25;
    //if(M<0.28)               xf = 0.25;
    if((M>0.28)&&(M<1.01))  xf = 1.075274-8.3263235*M+32.48714*pow(M,2)-61.10135*pow(M,3)+53.87526*pow(M,4)-17.6282*pow(M,5);
    if(M>=1.01)             xf = 0.39;
return xf;
}

double RotorBase::get_lambda(double _r, double psi)//расчет коэф. протекания в сечении
{
    //_r   отн. радиус сечения
    //psi  азимут, рад
    double v = vt*( sqrt(1.5)*pow(_r,0.5)*f1 + (1 + K_mu*_r*cos(psi))*f2 );
    return  _Vy - (v / wR);//коэф. протекания в сечении
}

double RotorBase::get_Ux(double _r, double psi, double beta)//расчет проекции скорости обтекания сечения на его ось X
{
    //_r     отн. радиус сечения
    //psi    азимут, рад
    //beta   угол взмаха, рад
    //return проекция скорости обтекания сечения на ось X, м/с
    return (_r -_l_gsh)*cos(beta) + _l_gsh + _Vx*sin(psi) - _Vz*cos(psi);
}

double RotorBase::get_Uy(double _r, double lambda, double psi, double beta, double dbeta)//расчет проекции скорости обтекания сечения на его ось Y
{
    //_r     отн. радиус сечения
    //lambda коэф. протекания в сечении
    //psi    азимут, рад
    //beta   угол взмаха, рад
    //dbeta  скорость взмаха,  рад [взмаха]/ рад [оборота]
    //return проекция скорости обтекания сечения на ось Y, м/с
    return lambda*cos(beta) - _Vx*cos(psi)*sin(beta) - _Vz*sin(psi)*sin(beta) - (_r -_l_gsh)*dbeta;
}

double RotorBase::get_M(double U)//расчет числа Маха
{
    //U суммарная скорость обтекания в сечении, м/с
    //return число Маха
    return U / sV;
}

double RotorBase::get_fi(int i, double psi, double beta)//расчет шага лопасти в сечении
{
    //i     номер сечения
    //psi   азимут, рад
    //beta  угол взмаха, рад
    //return  шаг лопасти в сечении, рад
    switch (ROTORID)
      {
      case MR://
        return fi0 + dfi.at(i) - _k*beta - Theta1*sin(psi-beta_sk) - Theta2*cos(psi-beta_sk);
        break;
      case TR://
        return fi0 + dfi.at(i) - _k*beta;
        break;
      case PR://
        return fi0 + dfi.at(i) - _k*beta;
        break;
      }
}

double RotorBase::get_Fi(double Ux, double Uy)//расчет угла притекания в сечении
{
    //Ux    проекция скорости обтекания сечения на ось X, м/с
    //Uy    проекция скорости обтекания сечения на ось Y, м/с
    double F,
           f = atan( fabs(Uy/Ux) );
    (Uy <= 0) ?
        (Ux <= 0) ? F = -1*M_PI + f : F = -1*f :
        (Ux <= 0) ? F =    M_PI - f : F = f;
    return F;//угол притекания в сечении, рад
}

double RotorBase::get_alfa_r(double fi, double Fi)//расчет угла атаки в сечении
{
    //fi шаг лопасти в сечении, рад
    //Fi угол притекания в сечении, рад
    //return угол атаки в сечении, рад
    double a = fi + Fi;
        (a > M_PI) ? (a = a - 2*M_PI) : (a);
        (a <-M_PI) ? (a = a + 2*M_PI) : (a);
    return a;
}

double RotorBase::get_Cy(int prof, double M, double alpha)//расчет коэф. подъемной силы Cy в сечении
{
    //prof  номер профиля
    //М     число Маха
    //alpha угол атаки в сечении, рад
    //return коэф. подъемной силы Cy в сечении
    return _getCy(prof, M, alpha);
}

double RotorBase::get_Mz(int prof, double M, double alpha)//расчет коэф. moment mz
{
    //prof  номер профиля
    //М     число Маха
    //alpha угол атаки в сечении, рад
    //return коэф. mz
    return _getMz(prof, M, alpha);
}

double RotorBase::_getMz(int profil, double Mach, double angle)
{
    profil--;
    double __Mz[CM],  k = 0,  k1 = 0,  Mz=0;
    int m = nM[profil][0], //кол-во значений М
        n = nA[profil][0]; //кол-во значений углов атаки

    //интерполяция по углу атаки
    for (int i = 0; i < m; i++) //по М
    {
        k = 0;
        for (int j = 0; j < n; j++)
        {
            if ((ALPHA[profil][j][0] <= angle) && (angle <= ALPHA[profil][j+1][0]))
            {
                k = (_Mz[profil][i][j+1] - _Mz[profil][i][j]) / (ALPHA[profil][j+1][0] - ALPHA[profil][j][0]);
                __Mz[i] = k*(angle - ALPHA[profil][j][0]) + _Mz[profil][i][j];
                break;
            }
            //else warning = 0;
        }
    }

    //интерполяция по Маху
    if (Mach <= MACH[profil][0][0])
    {
        Mz = __Mz[0];
    }
    else
    {
        for (int i = 0; i < m; i++)
        {
            if ((MACH[profil][i][0] <= Mach) && (Mach <= MACH[profil][i+1][0]))
            {
                k1 = (__Mz[i+1] - __Mz[i]) / (MACH[profil][i+1][0] - MACH[profil][i][0]);
                Mz = k1*(Mach - MACH[profil][i][0]) + __Mz[i];
                break;
            }
        }
    }
    if (Mach>MACH[profil][m-1][0])
    {
        warning = 309;
        Mz = __Mz[m-1];
    }

    return Mz;
}

double RotorBase::get_Cx(int prof, double M, double alpha)//расчет коэф. профильного сопротивления Cxp в сечении
{
    //prof  номер профиля
    //М     число Маха
    //alpha угол атаки в сечении, рад
    //return коэф. профильного сопротивления Cxp в сечении
    return _getCx(prof, M, alpha);
}

double RotorBase::get_Cxp0(int prof, double M)//расчет коэф. профильного сопротивления Cxp в сечении при (Су = 0)
{
    //prof  номер профиля
    //М     число Маха
    //return коэф. профильного сопротивления Cxp в сечении при (Су = 0)
    return _getCxCy0(prof, M);
}

double RotorBase::_getCy(int profil, double Mach, double angle)
{
    profil--;
    double __Cy[CM],  k = 0,  k1 = 0,  Cy=0;
    int m = nM[profil][0], //кол-во значений М
        n = nA[profil][0]; //кол-во значений углов атаки

    //интерполяция по углу атаки
    for (int i = 0; i < m; i++) //по М
    {
        k = 0;
        for (int j = 0; j < n; j++)
        {
            if ((ALPHA[profil][j][0] <= angle) && (angle <= ALPHA[profil][j+1][0]))
            {
                k = (_Cy[profil][i][j+1] - _Cy[profil][i][j]) / (ALPHA[profil][j+1][0] - ALPHA[profil][j][0]);
                __Cy[i] = k*(angle - ALPHA[profil][j][0]) + _Cy[profil][i][j];
                break;
            }
            //else warning = 0;
        }
    }

    //интерполяция по Маху
    if (Mach <= MACH[profil][0][0])
    {
        Cy = __Cy[0];
    }
    else
    {
        for (int i = 0; i < m; i++)
        {
            if ((MACH[profil][i][0] <= Mach) && (Mach <= MACH[profil][i+1][0]))
            {
                k1 = (__Cy[i+1] - __Cy[i]) / (MACH[profil][i+1][0] - MACH[profil][i][0]);
                Cy = k1*(Mach - MACH[profil][i][0]) + __Cy[i];
                break;
            }
        }
    }
    if (Mach>MACH[profil][m-1][0])
    {
        warning = 309;
        Cy = __Cy[m-1];
    }

    return Cy;
}

double RotorBase::_getCx(int profil, double Mach, double angle)
{
    profil--;
    double __Cx[CM],  k = 0,  k1 = 0,  Cx = 0;
    int m = nM[profil][1], //кол-во значений М
        n = nA[profil][1]; //кол-во значений углов атаки

    //интерполяция по углу атаки
    for (int i = 0; i < m; i++) //по М
    {
        k = 0;
        for (int j = 0; j < n; j++)
        {
            if ((ALPHA[profil][j][1] <= angle) && (angle <= ALPHA[profil][j+1][1]))
            {
                k = (_Cx[profil][i][j+1] - _Cx[profil][i][j]) / (ALPHA[profil][j+1][1] - ALPHA[profil][j][1]);
                __Cx[i] = k*(angle - ALPHA[profil][j][1]) + _Cx[profil][i][j];
                break;
            }
            //else warning = 0;
        }
    }

    //интерполяция по Маху
    if (Mach <= MACH[profil][0][1])
    {
        Cx = __Cx[0];
    }
    else
    {
        for (int i = 0; i < m; i++)
        {
            if ((MACH[profil][i][1] <= Mach) && (Mach <= MACH[profil][i+1][1]))
            {
                k1 = (__Cx[i+1] - __Cx[i]) / (MACH[profil][i+1][1] - MACH[profil][i][1]);
                Cx = k1*(Mach - MACH[profil][i][1]) + __Cx[i];
                break;
            }
        }
    }
    if (Mach > MACH[profil][m-1][1])
    {
        warning = 308;
        Cx = __Cx[m-1];
    }

    return Cx;
}

double RotorBase::_getCxCy0(int profil, double Mach)
{
    profil--;
    double Cx, k1;
    int m = nM[profil][1]; //кол-во значений М

    if (Mach <= MACH[profil][0][1])
    {
        Cx = _Cx_Cy0[profil][0];
    }
    else
    {
        for (int i = 0; i < m; i++)
        {
            if ((MACH[profil][i][1] <= Mach) && (Mach <= MACH[profil][i+1][1]))
            {
                k1 = (_Cx_Cy0[profil][i+1] - _Cx_Cy0[profil][i]) / (MACH[profil][i+1][1] - MACH[profil][i][1]);
                Cx = k1*(Mach - MACH[profil][i][1]) + _Cx_Cy0[profil][i];
                break;
            }
        }
    }
    if (Mach>MACH[profil][m-1][1])
    {
        warning = 307;
        Cx = _Cx_Cy0[profil][m-1];
    }

    return Cx;
}

double RotorBase::_getAlfa(int profil, double Mach, double Cy)
{
    double a = -7.0;///gradus
    double del = 1;
    int q=0;
    while (del > 0.005)///0.5*prec
    {
        q++;
        a += 0.01;///prec+///gradus
        del = fabs(Cy - _getCy(profil+1, Mach, a*d2r));
        if(q>999) { logstr += "_getAlfa(): q>999\n"; warning=306; break; }
    }
    return a*d2r;//rad
}

void RotorBase::getCxCy0Array()//наполняет массив Сх при Су=0
{
    for (int ii = 0; ii < 3; ii++)//по всем 5 профилям
    {
        for (int jj = 0; jj < nM[ii][1]; jj++)//по всем числам Маха
        {
            _a_Cy0[ii][jj] = _getAlfa(ii, MACH[ii][jj][1], 0);
            _Cx_Cy0[ii][jj]= _getCx((ii + 1), MACH[ii][jj][1], _a_Cy0[ii][jj] );
        }
    }
}

double RotorBase::get_IntegralMdT1(double psi, double beta, double dbeta, double gama, double dgama, double dfi7)//расчет момента тяги лопасти отн.ГШ
{
    //beta угол взмаха, рад
    //dbeta скорость взмаха,  рад [взмаха]/ рад [оборота]
    //psi азимут, рад
    //return момент тяги лопасти отн. ГШ, Н*м
    segment seg;
    vector<double> _MdT_dr(nSec);

    for (int rr=0; rr<nSec; rr++)
    {
        seg = Segment(rr, psi, beta, dbeta, gama, dgama, dfi7);
        _MdT_dr.at(rr) = seg.dT_dr * (r.at(rr) - l_gsh);
    }
    return get_Integral(_MdT_dr, r);//момент тяги лопасти отн.ГШ, Н*м
}
double RotorBase::get_IntegralMdT2(double psi, double beta, double dbeta, double gama, double dgama, double dfi7)//расчет момента тяги лопасти отн.ВШ
{
    //beta угол взмаха, рад
    //dbeta скорость взмаха,  рад [взмаха]/ рад [оборота]
    //psi азимут, рад
    //return момент тяги лопасти отн. ГШ, Н*м
    segment seg;
    vector<double> _MdT_dr(nSec);

    for (int rr=0; rr<nSec; rr++)
    {
        seg = Segment(rr, psi, beta, dbeta, gama, dgama, dfi7);
        _MdT_dr.at(rr) = seg.dT_dr * (r.at(rr) - l_vsh);
    }
    return get_Integral(_MdT_dr, r);//момент тяги лопасти отн.ВШ, Н*м
}
double RotorBase::get_IntegralMdQ1(double psi, double beta, double dbeta, double gama, double dgama, double dfi7)//расчет момента силы возд.сопротивления лопасти отн.ГШ
{
    segment seg;
    vector<double> _MdQ_dr(nSec);

    for (int rr=0; rr<nSec; rr++)
    {
        seg = Segment(rr, psi, beta, dbeta, gama, dgama, dfi7);
        _MdQ_dr.at(rr) = seg.dQ_dr * (r.at(rr) - l_gsh)*cos(beta);
    }
    return get_Integral(_MdQ_dr, r);//момент силы сопротивления лопасти отн.ГШ, Н*м
}
double RotorBase::get_IntegralMdQ2(double psi, double beta, double dbeta, double gama, double dgama, double dfi7)//расчет момента силы возд.сопротивления лопасти отн.ВШ
{
    segment seg;
    vector<double> _MdQ_dr(nSec);

    for (int rr=0; rr<nSec; rr++)
    {
        seg = Segment(rr, psi, beta, dbeta, gama, dgama, dfi7);
        _MdQ_dr.at(rr) = seg.dQ_dr * (r.at(rr) - l_vsh)*cos(beta);
    }
    return get_Integral(_MdQ_dr, r);//момент силы сопротивления лопасти отн.ВШ, Н*м
}
double RotorBase::get_IntegralMdMa(double psi, double beta, double dbeta, double gama, double dgama, double dfi7)//расчет момента аэродин.сил профиля отн.ОШ
{
    //psi   азимут, рад
    //beta   угол взмаха, рад
    //dbeta   скорость взмаха,  рад[взмаха]/рад[оборота]
    //gama     угол закрутки, рад
    //dgama   угл.скорость закрутки, рад/рад[оборота]
    //dfi7   угл.скорость установки, рад/рад[оборота]
    //return момент аэродин.сил профиля отн.ОШ, Н*м
    segment seg;
    vector<double> _MdMa_dr(nSec);

    for (int rr=0; rr<nSec; rr++)
    {
        seg = Segment(rr, psi, beta, dbeta, gama, dgama, dfi7);
        _MdMa_dr.at(rr) = seg.dMa_dr;
    }
    return get_Integral(_MdMa_dr, r);
}

double RotorBase::get_Integral(vector<double> T, vector<double> r)//интегрирование трапециями
{
    //T значения функции
    //r значения аргумента
    //return интеграл
    double integral = 0,  delta = 0;
    int count = min(T.size(), r.size());
    for (int i = 1; i < count; i++)
    {
        delta = r[i] - r[i-1];
        integral += 0.5 * (T.at(i) + T.at(i-1)) * delta;
    }
    return integral;
}

double RotorBase::DampherSpeed(double ksi, double dksi)// определение скорости перемещения штока демпфера
{
    double dsd, vd, sd, sd_;
    double ksi0 = ksi - dksi*dpsi;//
    //положение штока демпфера (как кулисы)
    sd_ = sqrt( d.C_ + 2*d.D_*d.h *sin(ksi0-ksi_nom  + d.fik + d.A_) );///
    sd  = sqrt( d.C_ + 2*d.D_*d.h *sin(ksi -ksi_nom  + d.fik + d.A_) );///
    //смещение на текущей итерации
    dsd = sd - sd_;
    //скорость перемещения штока демпфера
    if (dt==0)  vd = 0;
    else vd = dsd / dt;
return vd;
}

double RotorBase::DampherForce(double vd)// определение силы на штоке демпфера
{
    double pd, Ph, Pf, _Fdmp=0;
    //сила механического трения, Н
    Pf = ( vd==0 )?( 0 ):( 0 );//111
    //сила гидравлического сопротивления (жеклер в корпусе демпфера)
    Ph = d.sigma *vd*vd;
    //давление в демпфере
    pd = Ph / d.F / 98066.5;
    //срабатывание клапана
    if (pd > d.pmax)
    {
        Ph = 98066.5*d.pmax*d.F + d.sigma2 *vd*vd;
    }
    //суммарная сила на штоке демпфера
    if (vd < 0)  _Fdmp = 0 + Ph + Pf;
    if (vd > 0)  _Fdmp = 0 - Ph - Pf;
    if (vd ==0)  _Fdmp = 0;
return _Fdmp;
}

double RotorBase::get_f2(double mu)//определение коэф. f2 по коэф. режима работы mu
{
    const double _mu[] = { 0.0, 0.015, 0.030, 0.047, 0.067, 0.077, 0.087, 0.100, 0.130, 0.15, 0.20, 0.30 },
                 _f2[] = { 0.0, 0.210, 0.400, 0.605, 0.800, 0.890, 0.936, 0.970, 0.995, 1.00, 1.00, 1.00 };
    int n = sizeof(_mu) / sizeof(_mu[0]); //размер массива
    double f2, k = 0;

    if (mu <= _mu[0])
    {
        f2 = _f2[0];
    }
    else
    {
        if (mu > _mu[n-1])
        {
            f2 = _f2[n-1];
        }
        else
        {
            for (int i = 0; i < n; i++)
            {
                if ((_mu[i] < mu) && (mu <= _mu[i+1]))
                {
                    k = (_f2[i+1] - _f2[i]) / (_mu[i+1] - _mu[i]);
                    f2 = k * (mu - _mu[i]) + _f2[i];
                }
            }
        }
    }
    return f2;
}

void RotorBase::toHubSystem(double psi, double xl, double zl, double &xh, double &zh)
{
    xh = zl *cos(psi) + xl *sin(psi);
    zh = zl *sin(psi) - xl *cos(psi);
}

void RotorBase::get_minmax(vector<double> V, double med, double &min, double &max)
{
    min = med;
    max = med;
    for (int i = 0; i < V.size(); i++)
    {
        if (V.at(i) < min)  min = V.at(i);
        if (V.at(i) > max)  max = V.at(i);
    }
}

double RotorBase::get_max_deviation(vector<double> v1, vector<double> v2)//определение макс. погрешности между углами взмаха за два оборота
{
    //beta1 углы взмаха за первый оборот, рад
    //beta2 углы взмаха за второй оборот, рад
    //return макс. погрешности между углами взмаха за два оборота
    double delta, maxDelta = 0;

    for (int az = 0; az <= nAz; az++)
    {
        if ( fabs(v1.at(az) + v2.at(az)) ==0 )  delta = 0;
        else delta = 0.5*fabs( (fabs(v2.at(az))-fabs(v1.at(az))) / (fabs(v1.at(az))+fabs(v2.at(az))) );
        (delta > maxDelta) ? (maxDelta = delta) : (maxDelta);
    }
    return maxDelta;
}

void RotorBase::checkVectors()//проверяет и выравнивает длину векторов  r_is, b_is, dfi_is, prof_is
{
    int min_size = min(min(r_is.size(), bh_is.size()), min(dfi_is.size(), prof_is.size()));
    r_is.resize(min_size);
    bh_is.resize(min_size);
    dfi_is.resize(min_size);
    prof_is.resize(min_size);
}

vector<double> RotorBase::shiftAz(vector<double> T, int currBlade)//
{
    vector<double> Tnew;
    Tnew.clear();
    //если шаг расчета не кратен шагу между лопастями возвращет 0, т.к. для корректного расчета нужна будет интерполяция
    //if ( nAz % zl != 0) return 0;
    int index;
    int step = -nAz/double(zl);
    for (int az = 0; az <= nAz; az++)
    {
        index = az + currBlade*step;
        if (index < 0)  index = index + nAz;//if (index > nAz)  index = index - nAz;
        Tnew.push_back( T.at(index) );
    }
    return Tnew;
}

double RotorBase::KatToAng(double pro, double pri)
{
    double ang;
    if (pri!=0)
    {
        if ( pro>0 && pri>0 ) ang =          atan( fabs(pro/pri) );
        if ( pro>0 && pri<0 ) ang =   M_PI - atan( fabs(pro/pri) );
        if ( pro<0 && pri>0 ) ang = 2*M_PI - atan( fabs(pro/pri) );
        if ( pro<0 && pri<0 ) ang =   M_PI + atan( fabs(pro/pri) );
    }
    else//pri==0
    {
        if (pro>0) ang = 0.5*M_PI;
        else       ang = 1.5*M_PI;
    }
    //ang = normAng(ang_);
    return ang;
}

double RotorBase::normAng(double angle)
{
    double normangle = angle;
    if ( angle >= 2*M_PI ) normangle = angle - 2*M_PI;
    if ( angle < 0 )      normangle = angle + 2*M_PI;
    return normangle;
}

QString RotorBase::Pre2Str()//текст данных для расчета
{
    QString outpre="";
    outpre += "Hy=           " + ss.setNum(Hy         ,'f',1) + " м  \t\tвысота полета;\n";
    outpre += "Vx=           " + ss.setNum(Vx         ,'f',1) + " м/с (";// м/с -> км/ч
    outpre +=                    ss.setNum(Vx *3.6    ,'f',1) + " км/ч)  проекция скорости потока в плоскости винта;\n";// м/с -> км/ч
    outpre += "Vy=           " + ss.setNum(Vy         ,'f',1) + " м/с\t\tпроекция скорости потока на ось винта;\n";
    outpre += "Vz=           " + ss.setNum(Vz         ,'f',1) + " м/с\t\tбоковая проекция скорости потока;\n";
    outpre += "wx=           " + ss.setNum(w_x        ,'f',3) + " рад/с  \tугл.скорость вращения втулки винта;\n";
    outpre += "wy=           " + ss.setNum(w_y        ,'f',3) + " рад/с  \tугл.скорость вращения втулки винта;\n";
    outpre += "wz=           " + ss.setNum(w_z        ,'f',3) + " рад/с  \tугл.скорость вращения втулки винта;\n";
    outpre += "g=            " + ss.setNum(g          ,'f',5) + " м/с2  \tускорение свободного падения;\n";
    outpre += "ro=           " + ss.setNum(ro         ,'f',5) + " кг/м3\tплотность воздуха;\n";
    outpre += "sV=           " + ss.setNum(sV         ,'f',1) + " м/с\t\tскорость звука;\n";
    outpre += "azCount=      " + ss.setNum(nAz              ) +"     \t\tколичество шагов лопасти по азимуту;\n";
    outpre += "dpsi.g=       " + ss.setNum(dpsi*r2d   ,'f',3) + " грд     \tшаг по азимуту за dt;\n";
    outpre += "dpsi.r=       " + ss.setNum(dpsi       ,'f',6) + " рад   \tшаг по азимуту за dt;\n";
    outpre += "dt=           " + ss.setNum(dt         ,'f',6) + " с     \tприращение времени на одном шаге;\n";
    outpre += "dpsih=        " + ss.setNum(dpsih*r2d  ,'f',3) + " грд     \tугол между лопастями;\n";
    outpre += "psi0blades:   ";
    for (int l = 0; l < zl; l++)
    {
        outpre += ss.setNum(b[l].psi0*r2d,'f',1);       //psi0.at(az)*r2d
        outpre += ";  ";
    }
    outpre += "\n";

    outpre +="\nПараметры лопасти:";
    outpre += "\nnSec=         " + ss.setNum(nSec) + "        \tколичество расчетных участков лопасти;";
    outpre += "\n№sec        - порядковый номер сечения";
    outpre += "\n_r          - радиус сечения относительный";
    outpre += "\nr           - радиус сечения, м";
    outpre += "\nbh          - хорда лопасти, м";
    outpre += "\ndfi         - крутка лопасти, грд";
    outpre += "\nprof        - номер профиля в исходных данных";
    outpre += "\nprofname    - наименование профиля";
    outpre += "\n №sec";
    outpre += "\t_r  ";
    outpre += "\tr, м";
    outpre += "\tbh, м";
    outpre += "\tdfi,грд";
    outpre += "\t prof";
    outpre += "\tprofname";
    outpre += "\n ";
    for (int i=0; i<nSec; i++)
    {
        outpre += ss.setNum(i);                     outpre += "\t";
        outpre += ss.setNum(_r.at(i), 'f',3);       outpre += "\t";
        outpre += ss.setNum( r.at(i), 'f',3);       outpre += "\t";
        outpre += ss.setNum( bh.at(i),'f',3);       outpre += "\t";
        outpre += ss.setNum(dfi.at(i)*r2d,'f',3);   outpre += "\t  ";
        outpre += ss.setNum(prof.at(i));            outpre += "\t";
        outpre +=  profiles[prof.at(i)-1];          outpre += "\n ";
    }

    outpre += "\nПараметры винта:\n";
    outpre += "zl=           " + ss.setNum(zl               ) +"    \t\tколичество лопастей винта;\n";
    outpre += "ml=           " + ss.setNum(ml         ,'f',3) + " кг\t\tмасса лопасти;\n";
    outpre += "F=            " + ss.setNum(F          ,'f',1) + " м2\t\tометаемая площадь винта;\n";
    outpre += "R=            " + ss.setNum(R          ,'f',3) + " м \t\tрадиус винта;\n";
    outpre += "r0=           " + ss.setNum(r0         ,'f',3) + " м \t\tрадиус комлевой части лопасти;\n";
    outpre += "rB=           " + ss.setNum(rB         ,'f',3) + " м \t\tрадиус рабочей части лопасти;\n";
    outpre += "sec07=        " + ss.setNum(sec07            ) + "   \t\tномер сечения на радиусе 0.7R;\n";
    outpre += "krd=          " + ss.setNum(krd              ) + "   \t\tкоэф.направления вращения винта ('1' против ЧС; '-1' по ЧС (взгляд по оси Y));\n";
    outpre += "rpm=          " + ss.setNum(rpm        ,'f',1) + " об/мин  \tчастота вращения винта;\n";
    outpre += "w=            " + ss.setNum(w          ,'f',3) + " рад/с   \tугловая скорость вращения винта;\n";
    outpre += "wR=           " + ss.setNum(wR         ,'f',1) + " м/с\t\tокружная скорость концов лопастей;\n";
    outpre += "mu=           " + ss.setNum(mu         ,'f',3) + "    \t\tкоэф. режима работы винта;\n";
    outpre += "V=            " + ss.setNum(V          ,'f',3) + " м/с  \tскорость полета;\n";
    outpre += "Vx=           " + ss.setNum(Vx         ,'f',3) + " м/с  \tпроекция скорости потока на ось X СКВ;\n";
    outpre += "Vy=           " + ss.setNum(Vy         ,'f',3) + " м/с  \tпроекция скорости потока на ось Y СКВ;\n";
    outpre += "Vz=           " + ss.setNum(Vz         ,'f',3) + " м/с  \tпроекция скорости потока на ось Z СКВ;\n";
    outpre += "_V=           " + ss.setNum(_V         ,'f',3) + "   \t\tотносительная скорость полета (=V/wR);\n";
    outpre += "_Vx=          " + ss.setNum(_Vx        ,'f',3) + "   \t\tпроекция отн.скорости потока на ось X СКВ;\n";
    outpre += "_Vy=          " + ss.setNum(_Vy        ,'f',3) + "   \t\tпроекция отн.скорости потока на ось Y СКВ;\n";
    outpre += "_Vz=          " + ss.setNum(_Vz        ,'f',3) + "   \t\tпроекция отн.скорости потока на ось Z СКВ;\n";
    outpre += "sigma=        " + ss.setNum(sigma      ,'f',3) + "   \t\tкоэф. заполнения;\n";
    outpre += "l_gsh=        " + ss.setNum(l_gsh      ,'f',3) + " м \t\tразнос ГШ;\n";
    outpre += "l_vsh=        " + ss.setNum(l_vsh      ,'f',3) + " м \t\tразнос ВШ;\n";
    outpre += "s_vsh=        " + ss.setNum(s_vsh      ,'f',3) + " м \t\tсмещение ВШ;\n";
    outpre += "krutka=       " + ss.setNum(krutka *r2d,'f',3) + " грд   \tкрутка лопасти геометрическая;\n";
    outpre += "_k=           " + ss.setNum(_k         ,'f',3) + "   \t\tкоэф. компенсатора взмаха;\n";
    outpre += "dCxp=         " + ss.setNum(dCxp       ,'f',3) + "   \t\tпоправка к коэф. профильного сопротивления;\n";
    outpre += "B=            " + ss.setNum(Bkp        ,'f',3) + "   \t\tкоэф. концевых потерь;\n";
    outpre += "b7=           " + ss.setNum(b7         ,'f',3) + " м \t\tхорда в отн.сечении 0.7*R;\n";
    outpre += "Sb0=          " + ss.setNum(Sb0        ,'f',1) + " кг·м2   \tстатический момент лопасти отн.оси винта;\n";
    outpre += "Sb1=          " + ss.setNum(S_gsh      ,'f',1) + " кг·м    \tстатический момент лопасти отн.ГШ;\n";
    outpre += "Sb2=          " + ss.setNum(S_vsh      ,'f',1) + " кг·м    \tстатический момент лопасти отн.ВШ;\n";
    outpre += "Ib0=          " + ss.setNum(Ib0        ,'f',1) + " кг·м2   \tмомент инерции лопасти отн.оси винта;\n";
    outpre += "Ib1=          " + ss.setNum(I_gsh      ,'f',1) + " кг·м2   \tмомент инерции лопасти отн.ГШ;\n";
    outpre += "Ib2=          " + ss.setNum(I_vsh      ,'f',1) + " кг·м2   \tмомент инерции лопасти отн.ВШ;\n";
    outpre += "Iosh=         " + ss.setNum(I_osh      ,'f',1) + " кг·м2   \tмомент инерции лопасти отн.OШ;\n";
    outpre += "bet_min=      " + ss.setNum(bet_min *r2d,'f',1) + " грд\t\tнижний предел  угла взмаха;\n";
    outpre += "bet_max=      " + ss.setNum(bet_max *r2d,'f',1) + " грд\t\tверхний предел угла взмаха;\n";
    outpre += "ksi_min=      " + ss.setNum((ksi_min+ksi_nom)*r2d,'f',1) + " грд\t\tпредел угла качания от линии 'ось винта - ось ВШ' (опережение);\n";
    outpre += "ksi_max=      " + ss.setNum((ksi_max+ksi_nom)*r2d,'f',1) + " грд\t\tпредел угла качания от линии 'ось винта - ось ВШ' (отставание);\n";
    outpre += "ksi_nom=      " + ss.setNum(         ksi_nom *r2d,'f',3) + " грд\t\tугол между линией 'ось винта - ось ВШ' и перпендикуляром к оси ГШ;\n";
    outpre += "rpm_min=      " + ss.setNum(rpm_min     ,'f',0) + " об/мин   \tграница области поиска частоты вращения;\n";
    outpre += "rpm_max=      " + ss.setNum(rpm_max     ,'f',0) + " об/мин  \tграница области поиска частоты вращения;\n";
    outpre += "fi0_min=      " + ss.setNum(fi0_min *r2d,'f',1) + " грд\t\tграница области поиска общего шага;\n";
    outpre += "fi0_max=      " + ss.setNum(fi0_max *r2d,'f',1) + " грд\t\tграница области поиска общего шага;\n";
    outpre += "fi0_=         " + ss.setNum(fi0     *r2d,'f',3) + " грд   \tобщий шаг заданный;\n";
    outpre += "alfa_=        " + ss.setNum(alpha_  *r2d,'f',3) + " грд   \tугол атаки ( alfa_mr_= tang +eps_f -asin(Vy/V);  alfa_tr_= -beta_sk -skos_tr_rsk -skos_tr_; )\n";
    outpre += "beta_=        " + ss.setNum(beta_sk *r2d,'f',3) + " грд   \tугол скольжения заданный;\n";
    return outpre;
}

QString RotorBase::Res2Str(int log)//текст результатов расчета
{
    QString s1,s2,s3;
    QString outres="";
    outres += "fi0=          " + ss.setNum(fi0   *r2d,'f',3) + " грд   \tобщий шаг;\n";
    outres += "rpm=          " + ss.setNum(rpm       ,'f',1) + " об/мин  \tчастота вращения винта;\n";
    outres += "w=            " + ss.setNum(w         ,'f',1) + " рад/с   \tугловая скорость вращения винта;\n";
    outres += "\nИнтегральные характеристики винта:\n";
    outres += "Nпотр=        " + ss.setNum(N_need    ,'f',1) + " кВт      \tпотребная мощность;\n";//"[ ",N_need / 0.7355," л.с.]");
    outres += "  Nпроф=      " + ss.setNum(N_p       ,'f',1) + " кВт      \tпрофильная мощность;\n";
    outres += "  Nинд=       " + ss.setNum(N_i       ,'f',1) + " кВт      \tиндуктивная мощность;\n";
    outres += "  Nвр=        " + ss.setNum(N_vr      ,'f',1) + " кВт      \tмощность движения (пропульсивная);\n";
    outres += "H=            " + ss.setNum(H_sr      ,'f',1) + " Н        \tпродольная сила (=Fhx.cp);\n";
    outres += "T=            " + ss.setNum(T_sr      ,'f',1) + " Н       \tтяга винта      (=Fhy.cp);\n";
    outres += "S=            " + ss.setNum(S_sr      ,'f',1) + " Н         \tбоковая сила    (=Fhz.cp);\n";
    outres += "Mx=           " + ss.setNum(Mx_sr     ,'f',1) + " Н·м      \tмомент отн.продольной оси СКВ   (=Mhx.cp);\n";
    outres += "My=           " + ss.setNum(My_sr     ,'f',1) + " Н·м      \tмомент отн.оси винта (реактивный =Mhy.cp);\n";
    outres += "Mz=           " + ss.setNum(Mz_sr     ,'f',1) + " Н·м      \tмомент отн.поперечной оси СКВ   (=Mhz.cp);\n";
    //outres += "\n\nПроекции на оси СКВ средних за оборот результирующих силы и момента от всех лопастей на втулке:";
    outres += "Fhx.cp=       " + ss.setNum(Fhx_sr    ,'f',1) + " Н        \tсредняя за оборот продольная сила на втулке (СКВ)\n";
    outres += "Fhy.cp=       " + ss.setNum(Fhy_sr    ,'f',1) + " Н       \tсредняя за оборот вертикальная сила на втулке (СКВ)\n";
    outres += "Fhz.cp=       " + ss.setNum(Fhz_sr    ,'f',1) + " Н         \tсредняя за оборот боковая сила на втулке (СКВ)\n";
    outres += "Mhx.cp=       " + ss.setNum(Mhx_sr    ,'f',1) + " Н·м      \tсредний за оборот момент на втулке отн.оси X(СКВ)\n";
    outres += "Mhy.cp=       " + ss.setNum(Mhy_sr    ,'f',1) + " Н·м      \tсредний за оборот момент на втулке отн.оси Y(СКВ)\n";
    outres += "Mhz.cp=       " + ss.setNum(Mhz_sr    ,'f',1) + " Н·м      \tсредний за оборот момент на втулке отн.оси Z(СКВ)\n";

    outres += "Rxy=          " + ss.setNum(Rxy       ,'f',1) + " Н      \tрезультирующая сила на втулке в плоскости XY СКВ;\n";
    outres += "Ra=           " + ss.setNum(Ra        ,'f',1) + " Н      \tрезультирующая сила на втулке винта;\n";
    outres += "delta=        " + ss.setNum(delta *r2d,'f',3) + " грд   \tугол наклона равнодействующей силы;\n";
    outres += "vt=           " + ss.setNum(vt        ,'f',3) + " м/с    \tсредняя индуктивная скорость осевая;\n";
    outres += "vh=           " + ss.setNum(vh        ,'f',3) + " м/с    \tсредняя индуктивная скорость продольная;\n";
    outres += "vs=           " + ss.setNum(vs        ,'f',3) + " м/с    \tсредняя индуктивная скорость боковая;\n";
    outres += "lambda=       " + ss.setNum(lambda_sr ,'f',5) + "      \tсредний коэффициент протекания;\n";
    outres += "Ct=           " + ss.setNum(Ct        ,'f',5) + "      \tкоэффициент тяги  (Сt'=";
    outres +=                    ss.setNum(Ct_test   ,'f',5) + ");\n";
    outres += "Ch=           " + ss.setNum(Ch        ,'f',5) + "      \tкоэффициент продольной силы (Сh'=";
    outres +=                    ss.setNum(Ch_test   ,'f',5) + ");\n";
    outres += "Cs=           " + ss.setNum(Cs        ,'f',5) + "      \tкоэффициент поперечной силы;\n";
    outres += "mk=           " + ss.setNum(m_k       ,'f',5) + "      \tкоэффициент крутящего момента;\n";
    outres += "mi=           " + ss.setNum(m_i       ,'f',5) + "      \tкоэффициент индуктивной мощности;\n";
    outres += "mp=           " + ss.setNum(m_p       ,'f',5) + "      \tкоэффициент профильной мощности;\n";
    outres += "f1=           " + ss.setNum(f1        ,'f',3) + "        \tкоэффициент в формуле индуктивной скорости;\n";
    outres += "f2=           " + ss.setNum(f2        ,'f',3) + "        \tкоэффициент в формуле индуктивной скорости;\n";
    outres += "Mgyrx=        " + ss.setNum(Mgyrx     ,'f',1) + " Н·м      \tгироскопический момент на втулке;\n";
    outres += "Mgyrz=        " + ss.setNum(Mgyrz     ,'f',1) + " Н·м      \tгироскопический момент на втулке;\n";

    outres += "\nКонус лопастей винта:\n";
    outres += "beta(psi)= a0 - a1·cos(psi) - b1·sin(psi) - a2·cos(2·psi) - b2·sin(2·psi);";
    outres += "\n       a0=    " + s1.setNum(a0   ,'f',6) + " рад\t( " + s2.setNum(a0*r2d,'f',4) + " грд )";
    outres += "\n       a1=    " + s1.setNum(a1   ,'f',6) + " рад\t( " + s2.setNum(a1*r2d,'f',4) + " грд )";
    outres += "\n       b1=    " + s1.setNum(b1   ,'f',6) + " рад\t( " + s2.setNum(b1*r2d,'f',4) + " грд )";
    outres += "\n       a2=    " + s1.setNum(a2   ,'f',6) + " рад\t( " + s2.setNum(a2*r2d,'f',4) + " грд )";
    outres += "\n       b2=    " + s1.setNum(b2   ,'f',6) + " рад\t( " + s2.setNum(b2*r2d,'f',4) + " грд )";
    outres += "\n";
    outres += "\nДиапазоны параметров лопасти:";
    outres += "\n----------------------------------------------------------------------";
    outres += "\n\t\tminimum\t\tmedium\t\tmaximum";
    outres += "\n----------------------------------------------------------------------";
    outres += "\n     psi: \t" + s1.setNum(  psimin*r2d,'f',3) + " грд \t" + s2.setNum( psi_med*r2d,'f',3) + " грд \t" + s3.setNum(  psimax*r2d,'f',3) + " грд\t";
    outres += "\n     fi7: \t" + s1.setNum(  fi7min*r2d,'f',3) + " грд \t" + s2.setNum( fi7_med*r2d,'f',3) + " грд \t" + s3.setNum(  fi7max*r2d,'f',3) + " грд\t";
    outres += "\n    gama: \t" + s1.setNum(gama_min*r2d,'f',3) + " грд \t" + s2.setNum(gama_med*r2d,'f',3) + " грд \t" + s3.setNum(gama_max*r2d,'f',3) + " грд\t";
    outres += "\n    beta: \t" + s1.setNum(beta_min*r2d,'f',3) + " грд \t" + s2.setNum(beta_med*r2d,'f',3) + " грд \t" + s3.setNum(beta_max*r2d,'f',3) + " грд\t";
    outres += "\n     ksi: \t" + s1.setNum(  ksimin*r2d,'f',3) + " грд \t" + s2.setNum( ksi_med*r2d,'f',3) + " грд \t" + s3.setNum(  ksimax*r2d,'f',3) + " грд\t";
    outres += "\n       T: \t" + s1.setNum(   T_min    ,'f',1)+" Н     \t" + s2.setNum(   T_med    ,'f',1)+" Н     \t" + s3.setNum(   T_max    ,'f',1) + " Н  \t";
    outres += "\n       Q: \t" + s1.setNum(   Q_min    ,'f',1)+" Н     \t" + s2.setNum(   Q_med    ,'f',1)+" Н     \t" + s3.setNum(   Q_max    ,'f',1) + " Н  \t";
    outres += "\n      Ma: \t" + s1.setNum(  Ma_min    ,'f',1)+" Н·м   \t" + s2.setNum(  Ma_med    ,'f',1)+" Н·м   \t" + s3.setNum(  Ma_max    ,'f',1) + " Н·м\t";
    outres += "\n       K: \t" + s1.setNum(   K_min    ,'f',1)+" Н     \t" + s2.setNum(   K_med    ,'f',1)+" Н     \t" + s3.setNum(   K_max    ,'f',1) + " Н  \t";
    outres += "\n      Nr: \t" + s1.setNum(  Nr_min    ,'f',1)+" Н     \t" + s2.setNum(  Nr_med    ,'f',1)+" Н     \t" + s3.setNum(  Nr_max    ,'f',1) + " Н  \t";
    outres += "\n      Nf: \t" + s1.setNum(  Nf_min    ,'f',1)+" Н     \t" + s2.setNum(  Nf_med    ,'f',1)+" Н     \t" + s3.setNum(  Nf_max    ,'f',1) + " Н  \t";
    outres += "\n      P1: \t" + s1.setNum(  P1_min    ,'f',1)+" Н     \t" + s2.setNum(  P1_med    ,'f',1)+" Н     \t" + s3.setNum(  P1_max    ,'f',1) + " Н  \t";
    outres += "\n      P2: \t" + s1.setNum(  P2_min    ,'f',1)+" Н     \t" + s2.setNum(  P2_med    ,'f',1)+" Н     \t" + s3.setNum(  P2_max    ,'f',1) + " Н  \t";
    outres += "\n    Fdmp: \t" + s1.setNum(Fdmp_min    ,'f',1)+" Н     \t" + s2.setNum(Fdmp_med    ,'f',1)+" Н     \t" + s3.setNum(Fdmp_max    ,'f',1) + " Н  \t";
    outres += "\n    Fpov: \t" + s1.setNum(Fpov_min    ,'f',1)+" Н     \t" + s2.setNum(Fpov_med    ,'f',1)+" Н     \t" + s3.setNum(Fpov_max    ,'f',1) + " Н  \t";
    outres += "\n      Fx: \t" + s1.setNum(  Fx_min    ,'f',1)+" Н     \t" + s2.setNum(  Fx_med    ,'f',1)+" Н     \t" + s3.setNum(  Fx_max    ,'f',1) + " Н  \t";
    outres += "\n      Fy: \t" + s1.setNum(  Fy_min    ,'f',1)+" Н     \t" + s2.setNum(  Fy_med    ,'f',1)+" Н     \t" + s3.setNum(  Fy_max    ,'f',1) + " Н  \t";
    outres += "\n      Fz: \t" + s1.setNum(  Fz_min    ,'f',1)+" Н     \t" + s2.setNum(  Fz_med    ,'f',1)+" Н     \t" + s3.setNum(  Fz_max    ,'f',1) + " Н  \t";
    outres += "\n----------------------------------------------------------------------";
    outres += "\n";
    outres += "\nПараметры лопасти по азимуту:";
    outres += "\n\tpsi,  грд:";     for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);       outres += "\tградус";
    outres += "\n\tfi7,  грд:";     for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].fi7.at(az)*r2d,'f',3);   outres += "\tградус[уст]";
    outres += "\n\tgama, грд:";     for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].gama.at(az)*r2d,'f',3);  outres += "\tградус[закр]";
    outres += "\n\tbeta, грд:";     for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].beta.at(az)*r2d,'f',3);  outres += "\tградус[взм]";
    /*outres += "\n\tbeta*,грд:";
    for (int az=0; az<=nAz; az++)
    {
        bet = a0 - a1*cos(psi0.at(az)) - b1*sin(psi0.at(az)) - a2*cos(2*psi0.at(az)) - b2*sin(2*psi0.at(az));
        outres += "\t" + ss.setNum(bet*r2d,'f',3);
    }   outres += "\tградус[взм]";*/
    outres += "\n\tksi,  грд:";     for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].ksi.at(az)*r2d,'f',3);  outres += "\tградус[кач]";
    //outres += "\n\tdbeta/dpsi:";    for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].dbeta.at(az),'f',3);    outres += "\tрад[взм]/рад[об]";
    //outres += "\n\td2beta/dpsi^2:"; for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].d2beta.at(az),'f',3);   outres += "\tрад[взм]/рад^2[об]";
    //outres += "\n\tdksi/dpsi:";     for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].dksi.at(az),'f',3);     outres += "\tрад[кач]/рад[об]";
    //outres += "\n\td2ksi/dpsi^2:";  for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].d2ksi.at(az),'f',3);    outres += "\tрад[кач]/рад^2[об]";
    //outres += "\n";
    outres += "\n\tT,  Н:   ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].T.at(az)  ,'f',1);
    outres += "\n\tQ,  Н:   ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Q.at(az)  ,'f',1);
    outres += "\n\tMa, Н·м: ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Ma.at(az) ,'f',1);
    outres += "\n\tH,  Н:   ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].H.at(az)  ,'f',1);
    outres += "\n\tS,  Н:   ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].S.at(az)  ,'f',1);
    outres += "\n\tNr, Н:   ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Nr.at(az) ,'f',0);
    outres += "\n\tNf, Н:   ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Nf.at(az) ,'f',1);
    outres += "\n\tK,  Н:   ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].K1.at(az) ,'f',1);
    outres += "\n\tP1, Н:   ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].P1.at(az) ,'f',1);
    outres += "\n\tP2, Н:   ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].P2.at(az) ,'f',1);
    outres += "\n\tFd, Н:   ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Fdmp.at(az),'f',1);
    outres += "\n\tMd, Н·м: ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].M2D.at(az),'f',1);
    outres += "\n\tFpov, Н: ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Fpov.at(az),'f',1);
    outres += "\n";
    outres += "\nУсилия в шарнирах (СКЛ):";
    outres += "\n\tFx, Н:   ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Fx.at(az) ,'f',1);
    outres += "\n\tFy, Н:   ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Fy.at(az) ,'f',1);
    outres += "\n\tFz, Н:   ";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Fz.at(az) ,'f',0);
    outres += "\nМоменты в ГШ (СКЛ):";
    outres += "\n\tM1x, Н·м:";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].M1x.at(az),'f',1);
    outres += "\n\tM1y, Н·м:";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].M1y.at(az),'f',1);
    outres += "\n\tM1z, Н·м:";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].M1z.at(az),'f',1);
    outres += "\nМоменты в ВШ (СКЛ):";
    outres += "\n\tM2x, Н·м:";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].M2x.at(az),'f',1);
    outres += "\n\tM2y, Н·м:";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].M2y.at(az),'f',1);
    outres += "\n\tM2z, Н·м:";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].M2z.at(az),'f',1);
    outres += "\nМоменты в ОШ (СКЛ):";
    outres += "\n\tM3x, Н·м:";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].M3x.at(az),'f',1);
    outres += "\n\tM3y, Н·м:";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].M3y.at(az),'f',1);
    outres += "\n\tM3z, Н·м:";      for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].M3z.at(az),'f',1);
    outres += "\n\nCилы и моменты на втулке от одной лопасти (СКНВ):";
    outres += "\n\tазимут, грд:";   for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
    outres += "\n\tFhx(1), Н:  ";   for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Fhx.at(az),'f',1);
    outres += "\n\tFhy(1), Н:  ";   for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Fhy.at(az),'f',1);
    outres += "\n\tFhz(1), Н:  ";   for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Fhz.at(az),'f',1);
    outres += "\n\tMhx(1), Н·м:";   for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Mhx.at(az),'f',1);
    outres += "\n\tMhy(1), Н·м:";   for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Mhy.at(az),'f',1);
    outres += "\n\tMhz(1), Н·м:";   for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(b[0].Mhz.at(az),'f',1);
    outres += "\n\nСуммарные силы и моменты на втулке от всех лопастей (СКНВ):";
    outres += "\n\tазимут, грд:";   for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
    outres += "\n\tFhx(S), Н: ";    for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(Fhx_S.at(az)   ,'f',1);
    outres += "\n\tFhy(S), Н: ";    for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(Fhy_S.at(az)   ,'f',1);
    outres += "\n\tFhz(S), Н: ";    for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(Fhz_S.at(az)   ,'f',1);
    outres += "\n\tMhx(S), Н·м:";   for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(Mhx_S.at(az)   ,'f',1);
    outres += "\n\tMhy(S), Н·м:";   for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(Mhy_S.at(az)   ,'f',1);
    outres += "\n\tMhz(S), Н·м:";   for (int az=0; az<=nAz; az++)  outres += "\t" + ss.setNum(Mhz_S.at(az)   ,'f',1);
    outres += "\n";

    if(log)//Распределенные параметры лопасти (по сечению и по азимуту)
    {
        outres += "\n\nРаспределенные параметры лопасти (по сечению и по азимуту):\n";

        outres += "\nскорость обтекания Ux, м/с\n";
        outres += "_r\\psi";
        for (int az = 0; az <= nAz; az++)
            outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n";
        for (int i = 0; i < nSec; i++)
        {
            outres += ss.setNum(_r.at(i),'f',3);
            for (int az = 0; az <= nAz; az++)
                outres += "\t" + ss.setNum(b[0].Ux.at(az).at(i),'f',1);
            outres += "\n";
        }

        outres += "\nскорость обтекания Uy, м/с\n";
        outres += "_r\\psi";
        for (int az = 0; az <= nAz; az++)
            outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n";
        for (int i = 0; i < nSec; i++)
        {
            outres += ss.setNum(_r.at(i),'f',3);
            for (int az = 0; az <= nAz; az++)
                outres += "\t" + ss.setNum(b[0].Uy.at(az).at(i),'f',1);
            outres += "\n";
        }

        outres += "\nРезультирующая скорость обтекания U, м/с\n";
        outres += "_r\\psi";
        for (int az = 0; az <= nAz; az++)
            outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n";
        for (int i = 0; i < nSec; i++)
        {
            outres += ss.setNum(_r.at(i),'f',3);
            for (int az = 0; az <= nAz; az++)
                outres += "\t" + ss.setNum(b[0].U.at(az).at(i),'f',1);
            outres += "\n";
        }

        outres += "\nУгол установки сечения fi, градусы\n";
        outres += "_r\\psi";
        for (int az = 0; az <= nAz; az++)
            outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n";
        for (int i = 0; i < nSec; i++)
        {
            outres += ss.setNum(_r.at(i),'f',3);
            for (int az = 0; az <= nAz; az++)
                outres += "\t" + ss.setNum(b[0].fi.at(az).at(i)*r2d,'f',3);
            outres += "\n";
        }

        //outres += "\nУгол закрутки сечения gama, градусы\n";
        //outres += "_r\\psi";
        //for (int az = 0; az <= nAz; az++)
        //    outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        //outres += "\n";
        //for (int i = 0; i < nSec; i++)
        //{
        //    outres += ss.setNum(_r.at(i),'f',3);
        //    for (int az = 0; az <= nAz; az++)
        //        outres += "\t" + ss.setNum(b[0].gama.at(az)*r2d,'f',4);
        //    outres += "\n";
        //}

        outres += "\nУгол притекания Fi, градусы\n";
        outres += "_r\\psi";
        for (int az = 0; az <= nAz; az++)
            outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n";
        for (int i = 0; i < nSec; i++)
        {
            outres += ss.setNum(_r.at(i),'f',3);
            for (int az = 0; az <= nAz; az++)
                outres += "\t" + ss.setNum(b[0].Fi.at(az).at(i)*r2d,'f',3);
            outres += "\n";
        }

        outres += "\nУгол атаки сечения alpha_r, градусы\n";
        outres += "_r\\psi";
        for (int az = 0; az <= nAz; az++)
            outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n";
        for (int i = 0; i < nSec; i++)
        {
            outres += ss.setNum(_r.at(i),'f',3);
            for (int az = 0; az <= nAz; az++)
                outres += "\t" + ss.setNum(b[0].alfar.at(az).at(i)*r2d,'f',3);
            outres += "\n";
        }

        outres += "\nЧисло Маха M\n";
        outres += "_r\\psi";
        for (int az = 0; az <= nAz; az++)
            outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n";
        for (int i = 0; i < nSec; i++)
        {
            outres += ss.setNum(_r.at(i),'f',3);
            for (int az = 0; az <= nAz; az++)
                outres += "\t" + ss.setNum(b[0].M.at(az).at(i),'f',3);
            outres += "\n";
        }

        outres += "\nКоэффициент подъемной силы cy\n";
        outres += "_r\\psi";
        for (int az = 0; az <= nAz; az++)
            outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n";
        for (int i = 0; i < nSec; i++)
        {
            outres += ss.setNum(_r.at(i),'f',3);
            for (int az = 0; az <= nAz; az++)
                outres += "\t" + ss.setNum(b[0].cy.at(az).at(i),'f',3);
            outres += "\n";
        }

        outres += "\nКоэффициент профильного сопротивления cx\n";
        outres += "_r\\psi";
        for (int az = 0; az <= nAz; az++)
            outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n";
        for (int i = 0; i < nSec; i++)
        {
            outres += ss.setNum(_r.at(i),'f',3);
            for (int az = 0; az <= nAz; az++)
                outres += "\t" + ss.setNum(b[0].cx.at(az).at(i),'f',3);
            outres += "\n";
        }

        outres += "\nКоэффициент аэродинамического момента профиля mz\n";
        outres += "_r\\psi";
        for (int az = 0; az <= nAz; az++)
            outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n";
        for (int i = 0; i < nSec; i++)
        {
            outres += ss.setNum(_r.at(i),'f',3);
            for (int az = 0; az <= nAz; az++)
                outres += "\t" + ss.setNum(b[0].mz.at(az).at(i),'f',3);
            outres += "\n";
        }

        outres += "\nРаспределение погонной силы тяги dT/dr, [Н/м]\n";
        outres += "_r\\psi";
        for (int az = 0; az <= nAz; az++)
            outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n";
        for (int i = 0; i < nSec; i++)
        {
            outres += ss.setNum(_r.at(i),'f',3);
            for (int az = 0; az <= nAz; az++)
                outres += "\t" + ss.setNum(b[0].dT_dr.at(az).at(i),'f',1);
            outres += "\n";
        }

        outres += "\nРаспределение погонной силы сопротивления вращению dQ/dr, [Н/м]\n";
        outres += "_r\\psi";
        for (int az = 0; az <= nAz; az++)
            outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n";
        for (int i = 0; i < nSec; i++)
        {
            outres += ss.setNum(_r.at(i),'f',3);
            for (int az = 0; az <= nAz; az++)
                outres += "\t" + ss.setNum(b[0].dQ_dr.at(az).at(i),'f',1);
            outres += "\n";
        }

        outres += "\nРаспределение погонного момента аэродин.сил dMa/dr, [Н/м]\n";
        outres += "_r\\psi";
        for (int az = 0; az <= nAz; az++)
            outres += "\t" + ss.setNum(psi0.at(az)*r2d,'f',1);
        outres += "\n";
        for (int i = 0; i < nSec; i++)
        {
            outres += ss.setNum(_r.at(i),'f',3);
            for (int az = 0; az <= nAz; az++)
                outres += "\t" + ss.setNum(b[0].dMa_dr.at(az).at(i),'f',1);
            outres += "\n";
        }

        outres += "\nПогрешности при итерационном определении углов отклонения лопасти:\n";
        outres += "    psi, грд\t";
        outres += "delta_beta,%\t";
        outres +=  "delta_ksi,%\t";
        outres += "delta_gama,%\n";
        for (int az = 0; az <= nAz; az++)
        {
            outres += "     "   + ss.setNum(psi0.at(az)*r2d,'f',1);
            outres += "\t" + s1.setNum(deltabeta.at(az)*100,'f',6) ;
            outres += "\t" + s1.setNum( deltaksi.at(az)*100,'f',6) ;
            outres += "\t" + s1.setNum(deltagama.at(az)*100,'f',6) + "\n";
        }
    }
    outres += "\n\n";
    return outres;
}



double RotorBase::get__xFocus(int profN, int M){
    double _xFocus;
    double MPrev, MNext, CyPrev, CyNext, MzPrev, MzNext;
    std::vector<double>alfas, Cys, Mzs;
    double dCy_dAlfa, dMz_dAlfa, bLinEq;
    for(int i=1; i<=Ca; i++){
        //alfas.push_back();
    }
    return _xFocus;
}



/// MainRotor /////////////////////////////////////////////////////////////////

MainRotor::MainRotor()
{
    //title = "Винт несущий";
    ROTORID = MR;
}

void MainRotor::Preparation()//подготовка данных для расчета
{
    RotorBase::Preparation();
    //
    _D2 = _k*D1 + D2;
    _D1 = D1*(1 +_k*_k) -_k*_D2;

    //средние
    itcount =0;
    psi_sr  =0;        alfa_sr =0;
    eta_sr  =0;        chi_sr  =0;
    fi0_sr  =0;        ksi_sr  =0;
    bT_sr   =0;        bQ_sr   =0;
    bK1_sr  =0;        bNr_sr  =0;        bNf_sr  =0;
    bP1_sr  =0;        bP2_sr  =0;        bFdmp_sr=0;
    bF1y_sr =0;        bF2x_sr =0;        bF3z_sr =0;
    Fhx_sr  =0;        Fhy_sr  =0;        Fhz_sr  =0;
    Mhx_sr  =0;        Mhy_sr  =0;        Mhz_sr  =0;
    N_i     =0;        N_p     =0;
}

void MainRotor::ClassicEquivalent(double _Xa, double _Ya, double _eta, double _chi)//поиск параметров эквивалентного винта
{
    //_Xa потребная пропульсивная сила, Н
    //_Ya потребная подъемная сила, Н
    double a,b,x,d,d1,d2,d3;    int q = 0;    logstr="";    alpha = 0;    S_sr = 0;
    CALCMODE= EQROTOR;//режим расчета
    Xa_need = _Xa;
    Ya_need = _Ya;
    eta    = _eta;
    chi    = _chi;
    /*/установка области поиска угла атаки НВ
    //if(_alfarough<0.2)//большие скорости полета (<0.2 ~ >90km/h)
    //{
    //    alpha_max = _alfarough +0.2;//+0.2;(оптимальный диапазон на 27.04.2021)
    //    alpha_min = _alfarough -0.2;//-0.5;(оптимальный диапазон на 27.04.2021)
    //}
    //else//малые скорости полета
    //{
    //    alpha_max = _alfarough +0.5;//+0.6;(оптимальный диапазон на 26.04.2021) +0.6 на 27.04.2021
    //    alpha_min = _alfarough -0.8;//-0.8;(оптимальный диапазон на 26.04.2021)
    //}*/
    //Углы установки лопасти отн. КПВ в азимутах 270 и 180 соотв.
    Theta1 = _D2*eta - _D1*chi;
    Theta2 = _D1*eta + _D2*chi;
    //потребная результирующая сила
    R_need = sqrt(Xa_need*Xa_need + Ya_need*Ya_need);
    delta_need = atan2(Xa_need, Ya_need);

    //"ручная" коррекция для осевого режима
    if (Xa_need >= -111 && Xa_need <= 111)//(Xa_need!=0 && Ya_need==0)
    {
        //FLIGHTMODE = AXIAL;
        //if(Xa_need>=0) alpha =-0.5*M_PI;
        //else           alpha = 0.5*M_PI;
        T_i = Ya_need *cos(alpha) + Xa_need *sin(alpha);
        H_i = Xa_need *cos(alpha) - Ya_need *sin(alpha);

        getIniFlow();
        if(rpmORfi0) get_fi0();
        else         get_rpm();
    }

    //подбор alpha
     a = alpha_max;
     b = alpha_min;
     x = 0.5*(a + b),
     d = fabs(b - a),
    d2 = get_dDelta(b),
    d1 = get_dDelta(a),
    d3 = get_dDelta(x);
    if(d1*d2>0)
    {
        warning = 315;
        logstr += "MR::ClassicEquivalent(): неверно задана область поиска угла атаки [alpha_min...alpha_max]";
    }
    else
    while (d > 0.02*prec)//0.01*prec
    {
        q++;
        if ((d1>0 && d3>0) || (d1<0 && d3<0))
            { a = x;  d1 = d3; }
        if ((d2>0 && d3>0) || (d2<0 && d3<0))
            { b = x;  d2 = d3; }
        x = 0.5*(a + b);//alpha
        d3 = get_dDelta(x);
        d  = fabs(b - a);
        if(q>99) { warning = 314; break; }
    }
}

void MainRotor::ClassicSpecified(double _alpha, double _fi0, double _eta, double _chi)//поиск сил по заданным условиям
{
    eta = _eta;
    chi = _chi;//не изменил знак
    //Углы установки лопасти отн. КПВ в азимутах 270 и 180 соотв.
    Theta1 = _D2*eta - _D1*chi;
    Theta2 = _D1*eta + _D2*chi;

    RotorBase::ClassicSpecified(_alpha, _fi0);
}

void MainRotor::ClassicForceSynth()//Расчет всех сил и моментов на винте при текущих параметрах управления
{
    RotorBase::ClassicForceSynth();
    if (CALCMODE == EQROTOR)
    {
        del_Ra = 100* (R_need - Ra) / R_need;
        (Ya_need == 0) ? del_Ya = 0 : del_Ya = (100 * (Ya_need - Ya) / Ya_need);
        (Xa_need == 0) ? del_Xa = 0 : del_Xa = (100 * (Xa_need - Xa) / Xa_need);
    }
}

void MainRotor::Core()//Расчет интегральных сил и маховых движений при текущих значениях alpha и fi0 винта
{
    //усилие на втулке для расчета индуктивной скорости
    if (CALCMODE == FORCES)
    {
        T_i = T_sr;
        H_i = H_sr;
        S_i = S_sr;
    }
    if (CALCMODE == EQROTOR)
    {
        T_i = Ya_need *cos(alpha) + Xa_need *sin(alpha);
        H_i = Xa_need *cos(alpha) - Ya_need *sin(alpha);
        S_i = S_sr;
    }
    //Расчет индуктивной скорости
    RotorBase::getInducedFlow();
    //Расчет аэродинамических характеристик лопастей и параметров качания и махового движения при текущих значениях alpha и fi0 винта
    RotorBase::generalCore();
}

double MainRotor::get_dDelta(double _alpha)//Расчет отличия между текущим угом наклона равнодействующей delta и потребным углом
{//_alpha  угол атаки винта, рад
    alpha = _alpha;
    getIniFlow();
    if(rpmORfi0) get_fi0();
    else         get_rpm();
    //TODO: delta_need может быть == 0 !
    if(delta_need==0)
    {
        if(delta==0) return 0;
        else return  100* (delta_need - delta) / delta;
    }
    else  return  100* (delta_need - delta) / delta_need;
}

QString MainRotor::Pre2Str(int log)//текст данных для расчета
{
    QString outpre = appname;
    outpre += ".results.mainrotor\nmodel: " + model + "\nnotes: " + title + "\n";
    if(log)
    {
        outpre += "\n\nИсходные данные:\n\n";
        outpre += RotorBase::Pre2Str();
        outpre += "chi_=         " + ss.setNum(chi    *r2d,'f',3) + " грд    \tзаданный угол отклонения АП в продольном канале;\n";
        outpre += "eta_=         " + ss.setNum(eta    *r2d,'f',3) + " грд    \tзаданный угол отклонения АП в поперечном канале;\n";
        outpre += "D1=           " + ss.setNum(D1         ,'f',5) + "        \tпередаточное отношение между углом отклонения равнодействующей и углом отклонения АП;\n";
        outpre += "D2=           " + ss.setNum(D2         ,'f',5) + "        \tпередаточное отношение между углом отклонения равнодействующей и углом отклонения АП;\n";
        outpre += "_D1=          " + ss.setNum(_D1        ,'f',5) + "        \tпередаточное число управления цикл.шагом, отношение между углом установки лопасти и углом отклонения АП;\n";
        outpre += "_D2=          " + ss.setNum(_D2        ,'f',5) + "        \tпередаточное число управления цикл.шагом, отношение между углом установки лопасти и углом отклонения АП;\n";
        if(CALCMODE==EQROTOR)
        {
        outpre += "Ya_need=      " + ss.setNum(Ya_need      ,'f',1) + " Н  \t\tпотребная подъемная сила в скоростной CK;\n";
        outpre += "Xa_need=      " + ss.setNum(Xa_need      ,'f',1) + " Н  \t\tпотребная пропульсивная сила в скоростной CK;\n";
        outres += "R_need=       " + ss.setNum(R_need        ,'f',1)+ " Н      \tпотребная результирующая сила;\n";
        outres += "delta_need=   " + ss.setNum(delta_need*r2d,'f',3)+ " грд    \tпотребный угол наклона равнодействующей силы;\n";
        outpre += "alfa_min=     " + ss.setNum(alpha_min*r2d,'f',1) + " грд\t\tграница области поиска угла атаки;\n";
        outpre += "alfa_max=     " + ss.setNum(alpha_max*r2d,'f',1) + " грд\t\tграница области поиска угла атаки;\n";
        }
    }
    return outpre;
}

QString MainRotor::Res2Str(int log)//текст результатов расчета
{
    QString outres="";//"\nwarning=" + ss.setNum(warning);
    if (CALCMODE == EQROTOR)
    {
        outres += "\n\nРезультаты расчета эквивалентного винта по классической теории:\n\n";
        outres += "delta_Xa=     " + ss.setNum(del_Xa      ,'f',3) + " %    \tпогрешность по силе сопротивления/пропульсивной;\n";
        outres += "delta_Ya=     " + ss.setNum(del_Ya      ,'f',3) + " %    \tпогрешность по подъемной силе;\n";
        outres += "delta_Ra=     " + ss.setNum(del_Ra      ,'f',3) + " %    \tпогрешность по результирующей силе;\n";
    }
    if (CALCMODE == FORCES)
    {
        outres += "\n\nРезультаты расчета усилий на втулке НВ с заданным управлением по классической теории:\n\n";
    }
    outres += "Xa=           " + ss.setNum(Xa            ,'f',1) + " Н      \tсила сопротивления/пропульсивная в скоростной СК;\n";
    outres += "Ya=           " + ss.setNum(Ya            ,'f',1) + " Н      \tподъемная сила в скоростной СК;\n";
    outres += "Ra=           " + ss.setNum(Ra            ,'f',1) + " Н      \tравнодействующая сила;\n";
    outres += "delta=        " + ss.setNum(delta     *r2d,'f',3) + " грд    \tугол наклона равнодействующей силы;\n";
    outres += "alfaэ=        " + ss.setNum((alpha+D1*chi-D2*eta)*r2d,'f',3) + " грд    \tугол атаки эквивалентного винта;\n";
    outres += "alfa=         " + ss.setNum(alpha     *r2d,'f',3) + " грд    \tугол атаки винта;\n";
    outres += "fi0=          " + ss.setNum(fi0       *r2d,'f',3) + " грд    \tугол общего шага;\n";
    outres += "chi=          " + ss.setNum(chi       *r2d,'f',3) + " грд    \tугол отклонения АП в продольном канале;\n";
    outres += "eta=          " + ss.setNum(eta       *r2d,'f',3) + " грд    \tугол отклонения АП в поперечном канале;\n";
    outres += "teta1=        " + ss.setNum(Theta1    *r2d,'f',3) + " грд    \tугол установки лопасти отн.КПВ в азимуте 270;\n";
    outres += "teta2=        " + ss.setNum(Theta2    *r2d,'f',3) + " грд    \tугол установки лопасти отн.КПВ в азимуте 180;\n";
    outres += RotorBase::Res2Str(log);
    return outres;
}

void MainRotor::fillOutAz(outstructure &out, int az)//Classic
{
    //out.iter = iter;
    //out.t    = t;
    out.mr.nb    = zl;
    out.mr.nSec  = nSec;
    out.mr.D     = 2000*R;
    out.mr.l_vsh = 1000*l_vsh;
    out.mr.dct.x = 0;
    out.mr.dct.y = 0;
    out.mr.dct.z = 0;
    out.mr.az    = az;
    out.mr.sec07 = sec07;
    out.mr.revcount=0;
    out.mr.alfa  = alpha*r2d;
    out.mr.beta  = 0;
    out.mr.fi0   = fi0*r2d;
    out.mr.chi   = chi*r2d;
    out.mr.eta   = eta*r2d;

 if (az <= nAz)
  {
    out.iter    = iter.at(az);
    out.t       = t.at(az);
    out.mr.psih = psi0.at(az)*r2d;
    //параметры возмущенного движения
    out.mr.eh.x = e_x;    out.mr.eh.y = e_y;    out.mr.eh.z = e_z;
    out.mr.wh.x = w_x;    out.mr.wh.y = w_y;    out.mr.wh.z = w_z;
    out.mr.ah.x = a_x;    out.mr.ah.y = a_y;    out.mr.ah.z = a_z;
    out.mr.vh.x = v_x;    out.mr.vh.y = v_y;    out.mr.vh.z = v_z;
    out.mr.dctbs = 0;
    out.mr.dct.x = 0;
    out.mr.dct.z = 0;
    out.mr.Faxe  = 0;
    out.mr.angbs = 0;
    //суммарные силы и моменты на втулке от всех лопастей (в полусвязанной CK):";
    out.mr.Fhx_sum = Fhx_S.at(az);
    out.mr.Fhy_sum = Fhy_S.at(az);
    out.mr.Fhz_sum = Fhz_S.at(az);
    out.mr.Mhx_sum = Mhx_S.at(az);
    out.mr.Mhy_sum = Mhy_S.at(az);
    out.mr.Mhz_sum = Mhz_S.at(az);
    out.mr.Fhx_sr  = Fhx_sr;//H_sr ;//
    out.mr.Fhy_sr  = Fhy_sr;//T_sr ;//
    out.mr.Fhz_sr  = Fhz_sr;//S_sr ;//
    out.mr.Mhx_sr  = Mhx_sr;//Mx_sr;//
    out.mr.Mhy_sr  = Mhy_sr;//My_sr;//
    out.mr.Mhz_sr  = Mhz_sr;//Mz_sr;//
    //Параметры лопастей на азимуте
    for (int l = 0; l < zl; l++)
    {
        out.mr.b[l].psi0    = b[l].psi0 *r2d;//массив расчетных азимутов, градусы
        out.mr.b[l].psi     = b[l].psi   .at(az)*r2d;//массив расчетных азимутов, градусы
        out.mr.b[l].fi7     = b[l].fi7   .at(az)*r2d;// номер сечения 0,7R
        out.mr.b[l].dfi7    = b[l].dfi7  .at(az);//
        out.mr.b[l].d2fi7   = b[l].d2fi7 .at(az);//
        out.mr.b[l].gama    = b[l].gama  .at(az)*r2d;
        out.mr.b[l].dgama   = b[l].dgama .at(az);
        out.mr.b[l].d2gama  = b[l].d2gama.at(az);
        out.mr.b[l].beta    = b[l].beta  .at(az)*r2d;//вектор углов взмаха, градусы
        out.mr.b[l].dbeta   = b[l].dbeta .at(az);// скорость взмаха
        out.mr.b[l].d2beta  = b[l].d2beta.at(az);// ускорение взмаха
        out.mr.b[l].ksi     = b[l].ksi   .at(az)*r2d;//вектор углов качания, градусы
        out.mr.b[l].dksi    = b[l].dksi  .at(az);// скорость качания
        out.mr.b[l].d2ksi   = b[l].d2ksi .at(az);// ускорение качания

        out.mr.b[l].T       = b[l].T    .at(az);//сила тяги лопасти в конкретном азимуте, Н
        out.mr.b[l].Q       = b[l].Q    .at(az);//сила сопротивления лопасти в конкретном азимуте, Н
        out.mr.b[l].Ma      = b[l].Ma   .at(az);//
        out.mr.b[l].Nr      = b[l].Nr   .at(az);//центробежная сила в конкретном азимуте, Н
        out.mr.b[l].Nf      = b[l].Nf   .at(az);//центробежная сила от маховых движений в конкретном азимуте, Н
        out.mr.b[l].P1      = b[l].P1   .at(az);//сила инерции взмаха в конкретном азимуте, Н
        out.mr.b[l].P2      = b[l].P2   .at(az);//сила инерции качания в конкретном азимуте, Н
        out.mr.b[l].K1      = b[l].K1   .at(az);//сила Кориолиса в плоскости вращения в конкретном азимуте, Н
        out.mr.b[l].Fdmp    = b[l].Fdmp .at(az);
        out.mr.b[l].Fy      = b[l].Fy   .at(az);//проекция суммарной силы на ось у в с-ме кординат лопасти в конкретном азимуте, Н
        out.mr.b[l].Fx      = b[l].Fx   .at(az);//проекция суммарной силы на ось х в с-ме кординат лопасти в конкретном азимуте, Н
        out.mr.b[l].Fz      = b[l].Fz   .at(az);//проекция суммарной силы на ось z в с-ме кординат лопасти в конкретном азимуте, Н
        out.mr.b[l].Paxz    = b[l].Paxz .at(az);//сила инерции в плоскости винта (от линейного перемещения вдоль продольной и поперечной осей CKПС)
        out.mr.b[l].Payy    = b[l].Payy .at(az);//общая cила инерции в плоскости тяги
        ///ГШ, СКПС,
        out.mr.b[l].H       = b[l].H    .at(az);//продольная сила лопасти в конкретном азимуте, Н
        out.mr.b[l].S       = b[l].S    .at(az);//поперечная сила лопасти в конкретном азимуте, Н
        ///ГШ, СКЛ,
        out.mr.b[l].M1T     = b[l].M1T  .at(az);
        out.mr.b[l].M1Q     = b[l].M1Q  .at(az);//момент силы сопротивления лопасти отн. оси у1, Н*м
        out.mr.b[l].M1Gx    = b[l].M1Gx .at(az);
        out.mr.b[l].M1Gy    = b[l].M1Gy .at(az);
        out.mr.b[l].M1K     = b[l].M1K  .at(az);//момент Кориолисовых сил лопасти отн. y1, Н*м
        out.mr.b[l].M1Cx    = b[l].M1Cx .at(az);
        out.mr.b[l].M1Cy    = b[l].M1Cy .at(az);
        out.mr.b[l].M1Px    = b[l].M1Px .at(az);
        out.mr.b[l].M1Py    = b[l].M1Py .at(az);
        out.mr.b[l].M1x     = b[l].M1x  .at(az);//суммарный момент сил отн.ГШ, Н*м
        out.mr.b[l].M1z     = b[l].M1z  .at(az);//суммарный момент сил отн.ГШ, Н*м
        out.mr.b[l].M1y     = b[l].M1y  .at(az);//суммарный момент сил отн.ГШ, Н*м
        ///ВШ, СКЛ,
        out.mr.b[l].M2T     = b[l].M2T  .at(az);
        out.mr.b[l].M2Q     = b[l].M2Q  .at(az);
        out.mr.b[l].M2Gx    = b[l].M2Gx .at(az);
        out.mr.b[l].M2Gy    = b[l].M2Gy .at(az);
        out.mr.b[l].M2K     = b[l].M2K  .at(az);
        out.mr.b[l].M2Cx    = b[l].M2Cx .at(az);
        out.mr.b[l].M2Cy    = b[l].M2Cy .at(az);
        out.mr.b[l].M2Px    = b[l].M2Px .at(az);
        out.mr.b[l].M2Py    = b[l].M2Py .at(az);
        out.mr.b[l].M2D     = b[l].M2D  .at(az);
        out.mr.b[l].M2x     = b[l].M2x  .at(az);//суммарный момент сил отн.ВШ, Н*м
        out.mr.b[l].M2z     = b[l].M2z  .at(az);//суммарный момент сил отн.ВШ, Н*м
        out.mr.b[l].M2y     = b[l].M2y  .at(az);//суммарный момент сил отн.ВШ, Н*м
        ///OШ, СКЛ,
        out.mr.b[l].M3Ma    = b[l].M3Ma .at(az);//
        out.mr.b[l].M3Cz    = b[l].M3Cz .at(az);//
        out.mr.b[l].M3su    = b[l].M3su .at(az);//
        out.mr.b[l].Fpov    = b[l].Fpov .at(az);//
        out.mr.b[l].M3tr    = b[l].M3tr .at(az) + b[l].M3dmp.at(az);//
        out.mr.b[l].M3iz    = b[l].M3iz .at(az);//
        out.mr.b[l].M3x     = b[l].M3x  .at(az);//суммарный момент сил отн.OШ, Н*м
        out.mr.b[l].M3z     = b[l].M3z  .at(az);//суммарный момент сил отн.OШ, Н*м
        out.mr.b[l].M3y     = b[l].M3y  .at(az);//суммарный момент сил отн.OШ, Н*м
        ///Центр втулки, СКПС,
        out.mr.b[l].Fhx     = b[l].Fhx  .at(az);//на втулке от одной лопасти (в полусвязанной CK)
        out.mr.b[l].Fhy     = b[l].Fhy  .at(az);//на втулке от одной лопасти (в полусвязанной CK)
        out.mr.b[l].Fhz     = b[l].Fhz  .at(az);//на втулке от одной лопасти (в полусвязанной CK)
        out.mr.b[l].Mhx     = b[l].Mhx  .at(az);//на втулке от одной лопасти (в полусвязанной CK)
        out.mr.b[l].Mhy     = b[l].Mhy  .at(az);//на втулке от одной лопасти (в полусвязанной CK)
        out.mr.b[l].Mhz     = b[l].Mhz  .at(az);//на втулке от одной лопасти (в полусвязанной CK)
        ///Распределенные параметры лопасти по сечению и по азимуту)
        out.mr.b[l]._r    .clear();
        out.mr.b[l].r     .clear();
        out.mr.b[l].alfar .clear();
        out.mr.b[l].Ux    .clear();
        out.mr.b[l].Uy    .clear();
        out.mr.b[l].Uywz  .clear();
        out.mr.b[l].Uywx  .clear();
        out.mr.b[l].U     .clear();
        out.mr.b[l].fi    .clear();
        out.mr.b[l].fl    .clear();
        out.mr.b[l].Fi    .clear();
        out.mr.b[l].lambda.clear();
        out.mr.b[l].M     .clear();
        out.mr.b[l].cy    .clear();
        out.mr.b[l].cx    .clear();
        out.mr.b[l].mz    .clear();
        out.mr.b[l].dT_dr .clear();
        out.mr.b[l].dQ_dr .clear();
        out.mr.b[l].dMa_dr.clear();
        for (int sec = 0; sec < nSec; sec++)
        {
            out.mr.b[l]._r    .push_back(_r.at(sec) );
            out.mr.b[l].r     .push_back( r.at(sec) );
            out.mr.b[l].alfar .push_back( b[l].alfar.at(az).at(sec)*r2d);//Угол атаки alpha_r, градусы
            out.mr.b[l].fi    .push_back( b[l].fi.at(az).at(sec)   *r2d);//Угол установки сечения fi, градусы
            out.mr.b[l].fl    .push_back( b[l].fl.at(az).at(sec)   *r2d);//Угол
            out.mr.b[l].Ux    .push_back( b[l].Ux.at(az).at(sec)     );//скорость обтекания Ux, м/с
            out.mr.b[l].Uy    .push_back( b[l].Uy.at(az).at(sec)     );//скорость обтекания Uy, м/с
            out.mr.b[l].Uywx  .push_back( b[l].Uywx.at(az).at(sec)   );//
            out.mr.b[l].Uywz  .push_back( b[l].Uywz.at(az).at(sec)   );//
            out.mr.b[l].U     .push_back( b[l].U.at(az).at(sec)      );//Результирующая скорость обтекания U, м/с
            out.mr.b[l].Fi    .push_back( b[l].Fi.at(az).at(sec)  *r2d);//Угол притекания Fi, градусы
            out.mr.b[l].lambda.push_back( b[l].lambda.at(az).at(sec) );//коэф. протекания в сечении
            out.mr.b[l].M     .push_back( b[l].M.at(az).at(sec)      );//Число Маха M
            out.mr.b[l].cy    .push_back( b[l].cy.at(az).at(sec)     );//Коэффициент подъемной силы Cy
            out.mr.b[l].cx    .push_back( b[l].cx.at(az).at(sec)     );//Коэффициент профильного сопротивления Cxp
            out.mr.b[l].mz    .push_back( b[l].mz.at(az).at(sec)     );//
            out.mr.b[l].dT_dr .push_back( b[l].dT_dr.at(az).at(sec)  );//Распределение погонной силы тяги dT/dr, [Н/м]
            out.mr.b[l].dQ_dr .push_back( b[l].dQ_dr.at(az).at(sec)  );//Распределение погонной силы сопротивления вращению dQ/dr, [Н/м]
            out.mr.b[l].dMa_dr.push_back( b[l].dMa_dr.at(az).at(sec) );//
        }
    }
  }
}

void MainRotor::fillOut(outstructure &out)//out-структура результатов расчета FlyDynamic
{
    out.mr.nb    = zl;
    out.mr.nSec  = nSec;
    out.mr.D     = 2000*R;
    out.mr.l_vsh = 1000*l_vsh;
    out.mr.sec07 = sec07;
    out.mr.alfa  = alpha*r2d;
    out.mr.beta  = 0;
    out.mr.fi0   = fi0*r2d;
    out.mr.chi   = chi*r2d;
    out.mr.eta   = eta*r2d;
    //параметры возмущенного движения
    out.mr.eh.x  = e_x;         out.mr.eh.y  = e_y;         out.mr.eh.z   = e_z;
    out.mr.wh.x  = w_x;         out.mr.wh.y  = w_y;         out.mr.wh.z   = w_z;
    out.mr.ah.x  = a_x;         out.mr.ah.y  = a_y;         out.mr.ah.z   = a_z;
    out.mr.vh.x  = v_x;         out.mr.vh.y  = v_y;         out.mr.vh.z   = v_z;
    out.mr.dct.x = ctMRx*1000;  out.mr.dct.y = 0;           out.mr.dct.z = ctMRz*1000;
    out.mr.dctbs = dctbs*1000;  out.mr.angbs = act*r2d;
    out.mr.Faxe  = Fbs;
    //суммарные силы и моменты на втулке от всех лопастей (в полусвязанной CK)
    out.mr.Fhx_sum = Fhx_sum;
    out.mr.Fhy_sum = Fhy_sum;
    out.mr.Fhz_sum = Fhz_sum;
    out.mr.Mhx_sum = Mhx_sum;
    out.mr.Mhy_sum = Mhy_sum;
    out.mr.Mhz_sum = Mhz_sum;
    //Fh*_sr не использовать, оно может использоваться только после завершения оборота
    out.mr.Fhx_sr  = H_sr ;
    out.mr.Fhy_sr  = T_sr ;
    out.mr.Fhz_sr  = S_sr ;
    out.mr.Mhx_sr  = Mx_sr;
    out.mr.Mhy_sr  = My_sr;
    out.mr.Mhz_sr  = Mz_sr;

    out.mr.revcount=revcount;
    out.mr.psih = psih*r2d;//азимут-положение втулки винта
    for (int l = 0; l < out.mr.nb; l++)
    {
        out.mr.b[l].psi0  = B[l].psi0  *r2d;//азимут лопасти в начале расчета, градусы
        out.mr.b[l].psi   = B[l].psi   *r2d;//текущий азимут лопасти, градусы
        out.mr.b[l].fi7   = B[l].fi7   *r2d;//угол установки сечения 0,7R
        out.mr.b[l].dfi7  = B[l].dfi7  ;
        out.mr.b[l].d2fi7 = B[l].d2fi7 ;
        out.mr.b[l].gama  = B[l].gama  *r2d;
        out.mr.b[l].dgama = B[l].dgama ;
        out.mr.b[l].d2gama= B[l].d2gama;
        out.mr.b[l].beta  = B[l].beta  *r2d;//вектор углов взмаха, градусы
        out.mr.b[l].dbeta = B[l].dbeta ;// скорость взмаха
        out.mr.b[l].d2beta= B[l].d2beta;// ускорение взмаха
        out.mr.b[l].ksi   = B[l].ksi   *r2d;//вектор углов качания, градусы
        out.mr.b[l].dksi  = B[l].dksi  ;// скорость качания
        out.mr.b[l].d2ksi = B[l].d2ksi ;// ускорение качания

        out.mr.b[l].T     = B[l].T    ;//сила тяги лопасти в конкретном азимуте, Н
        out.mr.b[l].Q     = B[l].Q    ;//сила сопротивления лопасти в конкретном азимуте, Н
        out.mr.b[l].Ma    = B[l].Ma   ;//
        out.mr.b[l].Nr    = B[l].Nr   ;//центробежная сила в конкретном азимуте, Н
        out.mr.b[l].Nf    = B[l].Nf   ;//центробежная сила от маховых движений в конкретном азимуте, Н
        out.mr.b[l].P1    = B[l].P1   ;//сила инерции взмаха в конкретном азимуте, Н
        out.mr.b[l].P2    = B[l].P2   ;//сила инерции качания в конкретном азимуте, Н
        out.mr.b[l].K1    = B[l].K1   ;//сила Кориолиса в плоскости вращения в конкретном азимуте, Н
        out.mr.b[l].Fdmp  = B[l].Fdmp ;
        out.mr.b[l].Paxz  = B[l].Paxz ;//сила инерции в плоскости винта (от линейного перемещения вдоль продольной и поперечной осей CKПС)
        out.mr.b[l].Payy  = B[l].Payy ;//общая cила инерции в плоскости тяги

        out.mr.b[l].Fy   = B[l].Fy  ;//проекция суммарной силы на ось у в с-ме кординат лопасти в конкретном азимуте, Н
        out.mr.b[l].Fx   = B[l].Fx  ;//проекция суммарной силы на ось х в с-ме кординат лопасти в конкретном азимуте, Н
        out.mr.b[l].Fz   = B[l].Fz  ;//проекция суммарной силы на ось z в с-ме кординат лопасти в конкретном азимуте, Н
        //ГШ, СКПС,
        out.mr.b[l].H     = B[l].H    ;//продольная сила лопасти в конкретном азимуте, Н
        out.mr.b[l].S     = B[l].S    ;//поперечная сила лопасти в конкретном азимуте, Н
        //ГШ, СКЛ,
        out.mr.b[l].M1T   = B[l].M1T  ;
        out.mr.b[l].M1Q   = B[l].M1Q  ;//момент силы сопротивления лопасти отн. оси у1, Н*м
        out.mr.b[l].M1Gx  = B[l].M1Gx ;
        out.mr.b[l].M1Gy  = B[l].M1Gy ;
        out.mr.b[l].M1K   = B[l].M1K  ;//момент Кориолисовых сил лопасти отн. y1, Н*м
        out.mr.b[l].M1Cx  = B[l].M1Cx ;
        out.mr.b[l].M1Cy  = B[l].M1Cy ;
        out.mr.b[l].M1Px  = B[l].M1Px ;
        out.mr.b[l].M1Py  = B[l].M1Py ;
        out.mr.b[l].M1x   = B[l].M1x  ;//суммарный момент сил отн.ГШ, Н*м
        out.mr.b[l].M1z   = B[l].M1z  ;//суммарный момент сил отн.ГШ, Н*м
        out.mr.b[l].M1y   = B[l].M1y  ;//суммарный момент сил отн.ГШ, Н*м
        //ВШ, СКЛ,
        out.mr.b[l].M2T   = B[l].M2T  ;
        out.mr.b[l].M2Q   = B[l].M2Q  ;
        out.mr.b[l].M2Gx  = B[l].M2Gx ;
        out.mr.b[l].M2Gy  = B[l].M2Gy ;
        out.mr.b[l].M2K   = B[l].M2K  ;
        out.mr.b[l].M2Cx  = B[l].M2Cx ;
        out.mr.b[l].M2Cy  = B[l].M2Cy ;
        out.mr.b[l].M2Px  = B[l].M2Px ;
        out.mr.b[l].M2Py  = B[l].M2Py ;
        out.mr.b[l].M2D   = B[l].M2D  ;
        out.mr.b[l].M2x   = B[l].M2x  ;//суммарный момент сил отн.ВШ, Н*м
        out.mr.b[l].M2z   = B[l].M2z  ;//суммарный момент сил отн.ВШ, Н*м
        out.mr.b[l].M2y   = B[l].M2y  ;//суммарный момент сил отн.ВШ, Н*м
        //OШ, СКЛ,
        out.mr.b[l].M3Ma  = B[l].M3Ma ;//
        out.mr.b[l].M3Cz  = B[l].M3Cz ;//
        out.mr.b[l].M3su  = B[l].M3su ;//
        out.mr.b[l].M3tr  = B[l].M3tr + B[l].M3dmp;//
        out.mr.b[l].M3iz  = B[l].M3iz ;//
        out.mr.b[l].Fpov  = B[l].Fpov ;//
        out.mr.b[l].M3x   = B[l].M3x  ;//суммарный момент сил отн.OШ, Н*м
        out.mr.b[l].M3z   = B[l].M3z  ;//суммарный момент сил отн.OШ, Н*м
        out.mr.b[l].M3y   = B[l].M3y  ;//суммарный момент сил отн.OШ, Н*м
        //Центр втулки, СКПС,
        out.mr.b[l].Fhx   = B[l].Fhx  ;//на втулке от одной лопасти (в полусвязанной CK)
        out.mr.b[l].Fhy   = B[l].Fhy  ;//на втулке от одной лопасти (в полусвязанной CK)
        out.mr.b[l].Fhz   = B[l].Fhz  ;//на втулке от одной лопасти (в полусвязанной CK)
        out.mr.b[l].Mhx   = B[l].Mhx  ;//на втулке от одной лопасти (в полусвязанной CK)
        out.mr.b[l].Mhy   = B[l].Mhy  ;//на втулке от одной лопасти (в полусвязанной CK)
        out.mr.b[l].Mhz   = B[l].Mhz  ;//на втулке от одной лопасти (в полусвязанной CK)
        //Распределенные по сечению параметры лопасти
        out.mr.b[l]._r    .clear();
        out.mr.b[l].r     .clear();
        out.mr.b[l].alfar .clear();
        out.mr.b[l].Ux    .clear();
        out.mr.b[l].Uy    .clear();
        out.mr.b[l].Uywz  .clear();
        out.mr.b[l].Uywx  .clear();
        out.mr.b[l].U     .clear();
        out.mr.b[l].fi    .clear();
        out.mr.b[l].fl    .clear();
        out.mr.b[l].Fi    .clear();
        out.mr.b[l].lambda.clear();
        out.mr.b[l].M     .clear();
        out.mr.b[l].cy    .clear();
        out.mr.b[l].cx    .clear();
        out.mr.b[l].mz    .clear();
        out.mr.b[l].dT_dr .clear();
        out.mr.b[l].dQ_dr .clear();
        out.mr.b[l].dMa_dr.clear();
        for (int sec = 0; sec < nSec; sec++)
        {
            out.mr.b[l]._r    .push_back(_r.at(sec) );
            out.mr.b[l].r     .push_back( r.at(sec) );
            out.mr.b[l].alfar .push_back( B[l].alfar .at(sec) *r2d);//Угол атаки alpha_r, градусы
            out.mr.b[l].fi    .push_back( B[l].fi    .at(sec) *r2d);//Угол установки сечения fi, градусы
            out.mr.b[l].fl    .push_back( B[l].fl    .at(sec) *r2d);//Угол
            out.mr.b[l].Ux    .push_back( B[l].Ux    .at(sec) );//скорость обтекания Ux, м/с
            out.mr.b[l].Uy    .push_back( B[l].Uy    .at(sec) );//скорость обтекания Uy, м/с
            out.mr.b[l].Uywx  .push_back( B[l].Uywx  .at(sec) );//
            out.mr.b[l].Uywz  .push_back( B[l].Uywz  .at(sec) );//
            out.mr.b[l].U     .push_back( B[l].U     .at(sec) );//Результирующая скорость обтекания U, м/с
            out.mr.b[l].Fi    .push_back( B[l].Fi    .at(sec) *r2d);//Угол притекания Fi, градусы
            out.mr.b[l].lambda.push_back( B[l].lambda.at(sec) );//коэф. протекания в сечении
            out.mr.b[l].M     .push_back( B[l].M     .at(sec) );//Число Маха M
            out.mr.b[l].cy    .push_back( B[l].cy    .at(sec) );//Коэффициент подъемной силы Cy
            out.mr.b[l].cx    .push_back( B[l].cx    .at(sec) );//Коэффициент профильного сопротивления Cxp
            out.mr.b[l].mz    .push_back( B[l].mz    .at(sec) );//
            out.mr.b[l].dT_dr .push_back( B[l].dT_dr .at(sec) );//Распределение погонной силы тяги dT/dr, [Н/м]
            out.mr.b[l].dQ_dr .push_back( B[l].dQ_dr .at(sec) );//Распределение погонной силы сопротивления вращению dQ/dr, [Н/м]
            out.mr.b[l].dMa_dr.push_back( B[l].dMa_dr.at(sec) );//
        }
    }
}





/// TailRotor /////////////////////////////////////////////////////////////////

TailRotor::TailRotor()
{
    //title = "Винт рулевой";
    ROTORID = TR;
}

void TailRotor::Preparation()//подготовка данных для расчета
{
    RotorBase::Preparation();
    //kp = -1;
}

void TailRotor::ClassicEquivalent(double _alpha, double _T)//поиск параметров эквивалентного винта (fi0)
{
    CALCMODE = EQROTOR;//режим расчета
    alpha = _alpha;
    T_sr = T_i = T_need = _T;//потребная тяга, Н
    H_sr = 0;
    S_sr = 0;
    if(rpmORfi0) get_fi0();
    else         get_rpm();
}

void TailRotor::ClassicForceSynth()//Расчет всех сил и моментов на винте при текущих параметрах управления
{
    RotorBase::ClassicForceSynth();

    if (CALCMODE == EQROTOR)
    {
        del_T = 100*(T_need-T_sr)/T_need;
    }
}

void TailRotor::Core()//Расчет интегральных сил и маховых движений при текущих значениях alpha и fi0 винта
{
    //усилие на втулке для расчета индуктивной скорости
    H_i = H_sr;
    S_i = S_sr;
    if (CALCMODE == FORCES)
    {
        T_i = T_sr;
    }
    //Расчет индуктивной скорости
    RotorBase::getInducedFlow();
    //Расчет аэродинамических характеристик лопастей и параметров качания и махового движения при текущих значениях alpha и fi0 винта
    RotorBase::generalCore();
}

QString TailRotor::Pre2Str(int log)//текст данных для расчета
{
    QString outpre = appname;
    outpre += ".results.tailrotor\nmodel: " + model + "\nnotes: " + title + "\n";
    if(log)
    {
        outpre += "\n\nИсходные данные:\n\n";
        outpre += RotorBase::Pre2Str();
        outpre += "T_need=       " + ss.setNum(T_need,'f',1) + " Н     \tтяга винта заданная;\n";
    }
    return outpre;
}

QString TailRotor::Res2Str(int log)//текст результатов расчета
{
    QString outres="";//"\nwarning=" + ss.setNum(warning);
    if (CALCMODE == EQROTOR)
    {
        outres += "\n\nРезультаты расчета эквивалентного винта по классической теории:\n\n";
        outres += "delta_T=      " + ss.setNum(del_T     ,'f',3) + " %      \tпогрешность по тяге;\n";
    }
    if (CALCMODE == FORCES)
    {
        outres += "\n\nРезультаты расчета усилий на втулке РВ с заданным управлением по классической теории:\n\n";
    }
    outres += "alfa=         " + ss.setNum(alpha     *r2d,'f',3) + " грд    \tугол атаки винта;\n";
    outres += "fi0=          " + ss.setNum(fi0       *r2d,'f',3) + " грд    \tугол общего шага;\n";
    outres += "Xa=           " + ss.setNum(Xa            ,'f',1) + " Н      \tсила сопротивления в скоростной СК;\n";
    outres += "Ya=           " + ss.setNum(Ya            ,'f',1) + " Н      \tподъемная сила в скоростной СК;\n";
    outres += RotorBase::Res2Str(log);
    return outres;
}

void TailRotor::fillOutAz(outstructure &out, int az)//Classic
{
    //out.iter = iter;
    //out.t    = t;
    out.tr.nb    = zl;
    out.tr.nSec  = nSec;
    out.tr.D     = 2000*R;
    out.tr.l_vsh = 1000*l_vsh;
    //out.tr.dct.x= 0;
    //out.tr.dct.y= 0;
    //out.tr.dct.z= 0;
    out.tr.az    = az;
    out.tr.sec07 = sec07;
    out.tr.revcount=0;
    out.tr.alfa  = alpha*r2d;
    out.tr.beta  = 0;
    out.tr.fi0   = fi0*r2d;

 if (az <= nAz)
  {
    //out.iter    = iter.at(az);
    //out.t       = t.at(az);
    out.tr.psih = psi0.at(az)*r2d;
    ///параметры возмущенного движения
    //out.tr.eh.x = ehx;    out.tr.eh.y = ehy;    out.tr.eh.z = ehz;
    //out.tr.wh.x = whx;    out.tr.wh.y = why;    out.tr.wh.z = whz;
    //out.tr.ah.x = ahx;    out.tr.ah.y = ahy;    out.tr.ah.z = ahz;
    //out.tr.vh.x = vhx;    out.tr.vh.y = vhy;    out.tr.vh.z = vhz;
    //out.tr.sh.x = shz;    out.tr.sh.y = shy;    out.tr.sh.z = shz;
    ///суммарные силы и моменты на втулке от всех лопастей (в полусвязанной CK):";
    out.tr.Fhx_sum = Fhx_S.at(az);
    out.tr.Fhy_sum = Fhy_S.at(az);
    out.tr.Fhz_sum = Fhz_S.at(az);
    out.tr.Mhx_sum = Mhx_S.at(az);
    out.tr.Mhy_sum = Mhy_S.at(az);
    out.tr.Mhz_sum = Mhz_S.at(az);
    out.tr.Fhx_sr  = Fhx_sr;//H_sr ;//
    out.tr.Fhy_sr  = Fhy_sr;//T_sr ;//
    out.tr.Fhz_sr  = Fhz_sr;//S_sr ;//
    out.tr.Mhx_sr  = Mhx_sr;//Mx_sr;//
    out.tr.Mhy_sr  = Mhy_sr;//My_sr;//
    out.tr.Mhz_sr  = Mhz_sr;//Mz_sr;//

    for (int l = 0; l < zl; l++)
    {
        ///Параметры лопасти по азимуту
        out.tr.b[l].psi0    = b[l].psi0 *r2d;//массив расчетных азимутов, градусы
        out.tr.b[l].psi     = b[l].psi   .at(az)*r2d;//массив расчетных азимутов, градусы
        out.tr.b[l].fi7     = b[l].fi7   .at(az)*r2d;// номер сечения 0,7R
        out.tr.b[l].dfi7    = b[l].dfi7  .at(az)  ;//
        out.tr.b[l].d2fi7   = b[l].d2fi7 .at(az)  ;//
        out.tr.b[l].gama    = b[l].gama  .at(az)*r2d;
        out.tr.b[l].dgama   = b[l].dgama .at(az);
        out.tr.b[l].d2gama  = b[l].d2gama.at(az);
        out.tr.b[l].beta    = b[l].beta  .at(az)*r2d;//вектор углов взмаха, градусы
        out.tr.b[l].dbeta   = b[l].dbeta .at(az);// скорость взмаха
        out.tr.b[l].d2beta  = b[l].d2beta.at(az);// ускорение взмаха
        out.tr.b[l].ksi     = b[l].ksi   .at(az)*r2d;//вектор углов качания, градусы
        out.tr.b[l].dksi    = b[l].dksi  .at(az);// скорость качания
        out.tr.b[l].d2ksi   = b[l].d2ksi .at(az);// ускорение качания

        out.tr.b[l].T       = b[l].T    .at(az);//сила тяги лопасти в конкретном азимуте, Н
        out.tr.b[l].Q       = b[l].Q    .at(az);//сила сопротивления лопасти в конкретном азимуте, Н
        out.tr.b[l].Ma      = b[l].Ma   .at(az);//
        out.tr.b[l].Nr      = b[l].Nr   .at(az);//центробежная сила в конкретном азимуте, Н
        out.tr.b[l].Nf      = b[l].Nf   .at(az);//центробежная сила от маховых движений в конкретном азимуте, Н
        out.tr.b[l].P1      = b[l].P1   .at(az);//сила инерции взмаха в конкретном азимуте, Н
        out.tr.b[l].P2      = b[l].P2   .at(az);//сила инерции качания в конкретном азимуте, Н
        out.tr.b[l].K1      = b[l].K1   .at(az);//сила Кориолиса в плоскости вращения в конкретном азимуте, Н
        out.tr.b[l].Fdmp    = b[l].Fdmp .at(az);
        out.tr.b[l].Fy      = b[l].Fy   .at(az);//проекция суммарной силы на ось у в с-ме кординат лопасти в конкретном азимуте, Н
        out.tr.b[l].Fx      = b[l].Fx   .at(az);//проекция суммарной силы на ось х в с-ме кординат лопасти в конкретном азимуте, Н
        out.tr.b[l].Fz      = b[l].Fz   .at(az);//проекция суммарной силы на ось z в с-ме кординат лопасти в конкретном азимуте, Н
        ///ГШ, СКПС,
        //out.tr.b[l].Ty      = b[l].Ty   .at(az);//проекция на ось Y винта силы тяги лопасти в конкретном азимуте, Н
        out.tr.b[l].H       = b[l].H    .at(az);//продольная сила лопасти в конкретном азимуте, Н
        out.tr.b[l].S       = b[l].S    .at(az);//поперечная сила лопасти в конкретном азимуте, Н
        ///ГШ, СКЛ,
        out.tr.b[l].M1T     = b[l].M1T  .at(az);
        out.tr.b[l].M1Q     = b[l].M1Q  .at(az);//момент силы сопротивления лопасти отн. оси у1, Н*м
        out.tr.b[l].M1Gx    = b[l].M1Gx .at(az);
        out.tr.b[l].M1Gy    = b[l].M1Gy .at(az);
        out.tr.b[l].M1K     = b[l].M1K  .at(az);//момент Кориолисовых сил лопасти отн. y1, Н*м
        out.tr.b[l].M1Cx    = b[l].M1Cx .at(az);
        out.tr.b[l].M1Cy    = b[l].M1Cy .at(az);
        out.tr.b[l].M1Px    = b[l].M1Px .at(az);
        out.tr.b[l].M1Py    = b[l].M1Py .at(az);
        out.tr.b[l].M1x     = b[l].M1x  .at(az);//суммарный момент сил отн.ГШ, Н*м
        out.tr.b[l].M1z     = b[l].M1z  .at(az);//суммарный момент сил отн.ГШ, Н*м
        out.tr.b[l].M1y     = b[l].M1y  .at(az);//суммарный момент сил отн.ГШ, Н*м
        ///ВШ, СКЛ,
        out.tr.b[l].M2T     = b[l].M2T  .at(az);
        out.tr.b[l].M2Q     = b[l].M2Q  .at(az);
        out.tr.b[l].M2Gx    = b[l].M2Gx .at(az);
        out.tr.b[l].M2Gy    = b[l].M2Gy .at(az);
        out.tr.b[l].M2K     = b[l].M2K  .at(az);
        out.tr.b[l].M2Cx    = b[l].M2Cx .at(az);
        out.tr.b[l].M2Cy    = b[l].M2Cy .at(az);
        out.tr.b[l].M2Px    = b[l].M2Px .at(az);
        out.tr.b[l].M2Py    = b[l].M2Py .at(az);
        out.tr.b[l].M2D     = b[l].M2D  .at(az);
        out.tr.b[l].M2x     = b[l].M2x  .at(az);//суммарный момент сил отн.ВШ, Н*м
        out.tr.b[l].M2z     = b[l].M2z  .at(az);//суммарный момент сил отн.ВШ, Н*м
        out.tr.b[l].M2y     = b[l].M2y  .at(az);//суммарный момент сил отн.ВШ, Н*м
        ///OШ, СКЛ,
        out.tr.b[l].M3Ma    = b[l].M3Ma .at(az);//
        out.tr.b[l].M3Cz    = b[l].M3Cz .at(az);//
        out.tr.b[l].M3su    = b[l].M3su .at(az);//
        out.tr.b[l].Fpov    = b[l].Fpov .at(az);//
        out.tr.b[l].M3tr    = b[l].M3tr .at(az) + b[l].M3dmp.at(az);//
        out.tr.b[l].M3iz    = b[l].M3iz .at(az);//
        out.tr.b[l].M3x     = b[l].M3x  .at(az);//суммарный момент сил отн.OШ, Н*м
        out.tr.b[l].M3z     = b[l].M3z  .at(az);//суммарный момент сил отн.OШ, Н*м
        out.tr.b[l].M3y     = b[l].M3y  .at(az);//суммарный момент сил отн.OШ, Н*м
        ///Центр втулки, СКПС,
        //out.tr.b[l].MQ      = b[l].MQ   .at(az);//момент силы сопротивления лопасти отн. оси винта, Н*м
        //out.tr.b[l].Mxa     = b[l].Mxa  .at(az);//момент аэрод. сил отн. оси хн винта в конкретном азимуте, Н*м
        //out.tr.b[l].Mza     = b[l].Mza  .at(az);//момент аэрод. сил отн. оси zн винта в конкретном азимуте, Н*м
        //out.tr.b[l].Mxj     = b[l].Mxj  .at(az);//момент инерционных сил отн. оси хн винта в конкретном азимуте, Н*м
        //out.tr.b[l].Mzj     = b[l].Mzj  .at(az);//момент инерционных сил отн. оси zн винта в конкретном азимуте, Н*м
        out.tr.b[l].Fhx     = b[l].Fhx  .at(az);//на втулке от одной лопасти (в полусвязанной CK)
        out.tr.b[l].Fhy     = b[l].Fhy  .at(az);//на втулке от одной лопасти (в полусвязанной CK)
        out.tr.b[l].Fhz     = b[l].Fhz  .at(az);//на втулке от одной лопасти (в полусвязанной CK)
        out.tr.b[l].Mhx     = b[l].Mhx  .at(az);//на втулке от одной лопасти (в полусвязанной CK)
        out.tr.b[l].Mhy     = b[l].Mhy  .at(az);//на втулке от одной лопасти (в полусвязанной CK)
        out.tr.b[l].Mhz     = b[l].Mhz  .at(az);//на втулке от одной лопасти (в полусвязанной CK)
        ///Распределенные параметры лопасти по сечению и по азимуту)
        out.tr.b[l]._r    .clear();
        out.tr.b[l].r     .clear();
        out.tr.b[l].alfar .clear();
        out.tr.b[l].Ux    .clear();
        out.tr.b[l].Uy    .clear();
        out.tr.b[l].U     .clear();
        out.tr.b[l].Uywz  .clear();
        out.tr.b[l].Uywx  .clear();
        out.tr.b[l].fi    .clear();
        out.tr.b[l].fl    .clear();
        out.tr.b[l].Fi    .clear();
        out.tr.b[l].lambda.clear();
        out.tr.b[l].M     .clear();
        out.tr.b[l].cy    .clear();
        out.tr.b[l].cx    .clear();
        out.tr.b[l].mz    .clear();
        out.tr.b[l].dT_dr .clear();
        out.tr.b[l].dQ_dr .clear();
        out.tr.b[l].dMa_dr.clear();
        for (int sec = 0; sec < nSec; sec++)
        {
            out.tr.b[l]._r    .push_back(_r.at(sec) );
            out.tr.b[l].r     .push_back( r.at(sec) );
            out.tr.b[l].alfar .push_back( b[l].alfar.at(az).at(sec)*r2d);//Угол атаки alpha_r, градусы
            out.tr.b[l].U     .push_back( b[l].U.at(az).at(sec)      );//Результирующая скорость обтекания U, м/с
            out.tr.b[l].Ux    .push_back( b[l].Ux.at(az).at(sec)     );//скорость обтекания Ux, м/с
            out.tr.b[l].Uy    .push_back( b[l].Uy.at(az).at(sec)     );//скорость обтекания Uy, м/с
            out.tr.b[l].Uywx  .push_back( b[l].Uywx.at(az).at(sec) );//
            out.tr.b[l].Uywz  .push_back( b[l].Uywz.at(az).at(sec) );//
            out.tr.b[l].fi    .push_back( b[l].fi.at(az).at(sec) *r2d);//Угол установки сечения fi, градусы
            out.tr.b[l].fl    .push_back( b[l].fl.at(az).at(sec) *r2d);//Угол
            out.tr.b[l].Fi    .push_back( b[l].Fi.at(az).at(sec) *r2d);//Угол притекания Fi, градусы
            out.tr.b[l].lambda.push_back( b[l].lambda.at(az).at(sec) );//коэф. протекания в сечении
            out.tr.b[l].M     .push_back( b[l].M.at(az).at(sec)      );//Число Маха M
            out.tr.b[l].cy    .push_back( b[l].cy.at(az).at(sec)     );//Коэффициент подъемной силы Cy
            out.tr.b[l].cx    .push_back( b[l].cx.at(az).at(sec)     );//Коэффициент профильного сопротивления Cxp
            out.tr.b[l].mz    .push_back( b[l].mz.at(az).at(sec)     );//
            out.tr.b[l].dT_dr .push_back( b[l].dT_dr.at(az).at(sec)  );//Распределение погонной силы тяги dT/dr, [Н/м]
            out.tr.b[l].dQ_dr .push_back( b[l].dQ_dr.at(az).at(sec)  );//Распределение погонной силы сопротивления вращению dQ/dr, [Н/м]
            out.tr.b[l].dMa_dr.push_back( b[l].dMa_dr.at(az).at(sec) );//
        }
    }
  }
}

void TailRotor::fillOut(outstructure &out)//out-структура результатов расчета FlyDynamic
{
    out.tr.nb    = zl;
    out.tr.nSec  = nSec;
    out.tr.D     = 2000*R;
    out.tr.l_vsh = 1000*l_vsh;
    out.tr.sec07 = sec07;
    out.tr.alfa  = alpha*r2d;
    out.tr.beta  = 0;
    out.tr.fi0   = fi0*r2d;
    //суммарные силы и моменты на втулке от всех лопастей (в полусвязанной CK)
    out.tr.Fhx_sum = Fhx_sum;
    out.tr.Fhy_sum = Fhy_sum;
    out.tr.Fhz_sum = Fhz_sum;
    out.tr.Mhx_sum = Mhx_sum;
    out.tr.Mhy_sum = Mhy_sum;
    out.tr.Mhz_sum = Mhz_sum;
    //Fh*_sr не использовать, оно может использоваться только после завершения оборота
    out.tr.Fhx_sr  = H_sr ;
    out.tr.Fhy_sr  = T_sr ;
    out.tr.Fhz_sr  = S_sr ;
    out.tr.Mhx_sr  = Mx_sr;
    out.tr.Mhy_sr  = My_sr;
    out.tr.Mhz_sr  = Mz_sr;

    out.tr.revcount=revcount;
    out.tr.psih = psih*r2d;//азимут-положение втулки винта
    for (int l = 0; l < out.tr.nb; l++)
    {
        out.tr.b[l].psi0  = B[l].psi0  *r2d;//азимут лопасти в начале расчета, градусы
        out.tr.b[l].psi   = B[l].psi   *r2d;//текущий азимут лопасти, градусы
        out.tr.b[l].fi7   = B[l].fi7   *r2d;//угол установки сечения 0,7R
        out.tr.b[l].dfi7  = B[l].dfi7  ;//
        out.tr.b[l].d2fi7 = B[l].d2fi7 ;//
        out.tr.b[l].gama  = B[l].gama  *r2d;
        out.tr.b[l].dgama = B[l].dgama ;
        out.tr.b[l].d2gama= B[l].d2gama;
        out.tr.b[l].beta  = B[l].beta  *r2d;//вектор углов взмаха, градусы
        out.tr.b[l].dbeta = B[l].dbeta ;// скорость взмаха
        out.tr.b[l].d2beta= B[l].d2beta;// ускорение взмаха
        out.tr.b[l].ksi   = B[l].ksi   *r2d;//вектор углов качания, градусы
        out.tr.b[l].dksi  = B[l].dksi  ;// скорость качания
        out.tr.b[l].d2ksi = B[l].d2ksi ;// ускорение качания
        out.tr.b[l].T     = B[l].T    ;//сила тяги лопасти в конкретном азимуте, Н
        out.tr.b[l].Q     = B[l].Q    ;//сила сопротивления лопасти в конкретном азимуте, Н
        out.tr.b[l].Ma    = B[l].Ma   ;
        out.tr.b[l].Nr    = B[l].Nr   ;//центробежная сила в конкретном азимуте, Н
        out.tr.b[l].Nf    = B[l].Nf   ;//центробежная сила от маховых движений в конкретном азимуте, Н
        out.tr.b[l].P1    = B[l].P1   ;//сила инерции взмаха в конкретном азимуте, Н
        out.tr.b[l].P2    = B[l].P2   ;//сила инерции качания в конкретном азимуте, Н
        out.tr.b[l].K1    = B[l].K1   ;//сила Кориолиса в плоскости вращения в конкретном азимуте, Н
        //out.tr.b[l].K2    = 0;//B[l].K2;//сила Кориолиса в плоскости вращения в конкретном азимуте, Н
        out.tr.b[l].Fdmp  = B[l].Fdmp ;
        //out.tr.b[l].Paxz  = B[l].Paxz;////сила инерции в плоскости винта (от линейного перемещения вдоль продольной и поперечной осей CKПС)
        //out.tr.b[l].Payy  = B[l].Payy;////общая cила инерции в плоскости тяги
        //out.tr.b[l].Pexz  = B[l].Pexz;//сила инерции от вращения НВ вокруг продольной и поперечной осей CKПС
        //out.tr.b[l].Pkxz  = B[l].Pkxz;//силы инерции Кориолиса, возникающие от вращения НВ вокруг продольной и поперечной осей CKПС на азимуте psi
        out.tr.b[l].Fy   = B[l].Fy  ;//проекция суммарной силы на ось у в с-ме кординат лопасти в конкретном азимуте, Н
        out.tr.b[l].Fx   = B[l].Fx  ;//проекция суммарной силы на ось х в с-ме кординат лопасти в конкретном азимуте, Н
        out.tr.b[l].Fz   = B[l].Fz  ;//проекция суммарной силы на ось z в с-ме кординат лопасти в конкретном азимуте, Н
        ///ГШ, СКПС,
        //out.tr.b[l].Ty    = B[l].Ty   ;//проекция на ось Y винта силы тяги лопасти в конкретном азимуте, Н
        out.tr.b[l].H     = B[l].H    ;//продольная сила лопасти в конкретном азимуте, Н
        out.tr.b[l].S     = B[l].S    ;//поперечная сила лопасти в конкретном азимуте, Н
        ///ГШ, СКЛ,
        out.tr.b[l].M1T   = B[l].M1T  ;
        out.tr.b[l].M1Q   = B[l].M1Q  ;//момент силы сопротивления лопасти отн. оси у1, Н*м
        out.tr.b[l].M1Gx  = B[l].M1Gx ;
        out.tr.b[l].M1Gy  = B[l].M1Gy ;
        out.tr.b[l].M1K   = B[l].M1K  ;//момент Кориолисовых сил лопасти отн. y1, Н*м
        out.tr.b[l].M1Cx  = B[l].M1Cx ;
        out.tr.b[l].M1Cy  = B[l].M1Cy ;
        out.tr.b[l].M1Px  = B[l].M1Px ;
        out.tr.b[l].M1Py  = B[l].M1Py ;
        out.tr.b[l].M1x   = B[l].M1x  ;//суммарный момент сил отн.ГШ, Н*м
        out.tr.b[l].M1z   = B[l].M1z  ;//суммарный момент сил отн.ГШ, Н*м
        out.tr.b[l].M1y   = B[l].M1y  ;//суммарный момент сил отн.ГШ, Н*м
        ///ВШ, СКЛ,
        out.tr.b[l].M2T   = B[l].M2T  ;
        out.tr.b[l].M2Q   = B[l].M2Q  ;
        out.tr.b[l].M2Gx  = B[l].M2Gx ;
        out.tr.b[l].M2Gy  = B[l].M2Gy ;
        out.tr.b[l].M2K   = B[l].M2K  ;
        out.tr.b[l].M2Cx  = B[l].M2Cx ;
        out.tr.b[l].M2Cy  = B[l].M2Cy ;
        out.tr.b[l].M2Px  = B[l].M2Px ;
        out.tr.b[l].M2Py  = B[l].M2Py ;
        out.tr.b[l].M2D   = B[l].M2D  ;
        out.tr.b[l].M2x   = B[l].M2x  ;//суммарный момент сил отн.ВШ, Н*м
        out.tr.b[l].M2z   = B[l].M2z  ;//суммарный момент сил отн.ВШ, Н*м
        out.tr.b[l].M2y   = B[l].M2y  ;//суммарный момент сил отн.ВШ, Н*м
        ///OШ, СКЛ,
        out.tr.b[l].M3Ma  = B[l].M3Ma ;//
        out.tr.b[l].M3Cz  = B[l].M3Cz ;//
        out.tr.b[l].M3su  = B[l].M3su ;//
        out.tr.b[l].M3tr  = B[l].M3tr + B[l].M3dmp;//
        out.tr.b[l].M3iz  = B[l].M3iz ;//
        out.tr.b[l].Fpov  = B[l].Fpov ;//
        out.tr.b[l].M3x   = B[l].M3x  ;//суммарный момент сил отн.OШ, Н*м
        out.tr.b[l].M3z   = B[l].M3z  ;//суммарный момент сил отн.OШ, Н*м
        out.tr.b[l].M3y   = B[l].M3y  ;//суммарный момент сил отн.OШ, Н*м
        ///Центр втулки, СКПС,
        //out.tr.b[l].MQ    = B[l].MQ   ;//момент силы сопротивления лопасти отн. оси винта, Н*м
        //out.tr.b[l].Mxa   = B[l].Mxa  ;//момент аэрод. сил отн. оси хн винта в конкретном азимуте, Н*м
        //out.tr.b[l].Mza   = B[l].Mza  ;//момент аэрод. сил отн. оси zн винта в конкретном азимуте, Н*м
        //out.tr.b[l].Mxj   = B[l].Mxj  ;//момент инерционных сил отн. оси хн винта в конкретном азимуте, Н*м
        //out.tr.b[l].Mzj   = B[l].Mzj  ;//момент инерционных сил отн. оси zн винта в конкретном азимуте, Н*м
        out.tr.b[l].Fhx   = B[l].Fhx  ;//на втулке от одной лопасти (в полусвязанной CK)
        out.tr.b[l].Fhy   = B[l].Fhy  ;//на втулке от одной лопасти (в полусвязанной CK)
        out.tr.b[l].Fhz   = B[l].Fhz  ;//на втулке от одной лопасти (в полусвязанной CK)
        out.tr.b[l].Mhx   = B[l].Mhx  ;//на втулке от одной лопасти (в полусвязанной CK)
        out.tr.b[l].Mhy   = B[l].Mhy  ;//на втулке от одной лопасти (в полусвязанной CK)
        out.tr.b[l].Mhz   = B[l].Mhz  ;//на втулке от одной лопасти (в полусвязанной CK)
        ///Распределенные по сечению параметры лопасти
        out.tr.b[l]._r    .clear();
        out.tr.b[l].r     .clear();
        out.tr.b[l].alfar .clear();
        out.tr.b[l].Ux    .clear();
        out.tr.b[l].Uy    .clear();
        out.tr.b[l].Uywz  .clear();
        out.tr.b[l].Uywx  .clear();
        out.tr.b[l].U     .clear();
        out.tr.b[l].fi    .clear();
        out.tr.b[l].fl    .clear();
        out.tr.b[l].Fi    .clear();
        out.tr.b[l].lambda.clear();
        out.tr.b[l].M     .clear();
        out.tr.b[l].cy    .clear();
        out.tr.b[l].cx    .clear();
        out.tr.b[l].mz    .clear();
        out.tr.b[l].dT_dr .clear();
        out.tr.b[l].dQ_dr .clear();
        out.tr.b[l].dMa_dr.clear();
        for (int sec = 0; sec < nSec; sec++)
        {
            out.tr.b[l]._r    .push_back(_r.at(sec) );
            out.tr.b[l].r     .push_back( r.at(sec) );
            out.tr.b[l].alfar .push_back( B[l].alfar .at(sec) *r2d);//Угол атаки alpha_r, градусы
            out.tr.b[l].Ux    .push_back( B[l].Ux    .at(sec) );//скорость обтекания Ux, м/с
            out.tr.b[l].Uy    .push_back( B[l].Uy    .at(sec) );//скорость обтекания Uy, м/с
            out.tr.b[l].Uywx  .push_back( B[l].Uywx  .at(sec) );//
            out.tr.b[l].Uywz  .push_back( B[l].Uywz  .at(sec) );//
            out.tr.b[l].U     .push_back( B[l].U     .at(sec) );//Результирующая скорость обтекания U, м/с
            out.tr.b[l].fi    .push_back( B[l].fi    .at(sec) *r2d);//Угол установки сечения fi, градусы
            out.tr.b[l].fl    .push_back( B[l].fl    .at(sec) *r2d);//Угол
            out.tr.b[l].Fi    .push_back( B[l].Fi    .at(sec) *r2d);//Угол притекания Fi, градусы
            out.tr.b[l].lambda.push_back( B[l].lambda.at(sec) );//коэф. протекания в сечении
            out.tr.b[l].M     .push_back( B[l].M     .at(sec) );//Число Маха M
            out.tr.b[l].cy    .push_back( B[l].cy    .at(sec) );//Коэффициент подъемной силы Cy
            out.tr.b[l].cx    .push_back( B[l].cx    .at(sec) );//Коэффициент профильного сопротивления Cxp
            out.tr.b[l].mz    .push_back( B[l].mz    .at(sec) );//
            out.tr.b[l].dT_dr .push_back( B[l].dT_dr .at(sec) );//Распределение погонной силы тяги dT/dr, [Н/м]
            out.tr.b[l].dQ_dr .push_back( B[l].dQ_dr .at(sec) );//Распределение погонной силы сопротивления вращению dQ/dr, [Н/м]
            out.tr.b[l].dMa_dr.push_back( B[l].dMa_dr.at(sec) );//
        }
    }
}





/// PushRotor /////////////////////////////////////////////////////////////////

PushRotor::PushRotor()
{
    //title = "Винт толкающий";
    ROTORID = PR;
}

void PushRotor::Preparation()//подготовка данных для расчета
{
    RotorBase::Preparation();
    //kp = -1;
}

void PushRotor::ClassicEquivalent(double _alpha, double _T)//поиск параметров эквивалентного винта (fi0)
{
    CALCMODE = EQROTOR;//режим расчета
    alpha = _alpha;
    T_sr = T_i = T_need = _T;//потребная тяга, Н
    H_sr = 0;
    S_sr = 0;
    if(rpmORfi0) get_fi0();
    else         get_rpm();
}

void PushRotor::ClassicForceSynth()//Расчет всех сил и моментов на винте при текущих параметрах управления
{
    RotorBase::ClassicForceSynth();

    if (CALCMODE == EQROTOR)
    {
        del_T = 100*(T_need-T_sr)/T_need;
    }
}

void PushRotor::Core()//Расчет интегральных сил и маховых движений при текущих значениях alpha и fi0 винта
{
    //усилие на втулке для расчета индуктивной скорости
    H_i = H_sr;
    S_i = S_sr;
    if (CALCMODE == FORCES)
    {
        T_i = T_sr;
    }
    //Расчет индуктивной скорости
    getInducedFlow();//RotorBase::
    //Расчет аэродинамических характеристик лопастей и параметров качания и махового движения при текущих значениях alpha и fi0 винта
    generalCore();//RotorBase::
}

QString PushRotor::Pre2Str(int log)//текст данных для расчета
{
    QString outpre = appname;
    outpre += ".results.pushrotor\nmodel: " + model + "\nnotes: " + title + "\n";
    if(log)
    {
        outpre += "\n\nИсходные данные:\n\n";
        outpre += RotorBase::Pre2Str();
        outpre += "Tpr_need=     " + ss.setNum(T_need,'f',1) + " Н     \tтяга винта заданная;\n";
    }
    return outpre;
}

QString PushRotor::Res2Str(int log)//текст результатов расчета
{
    QString outres="";//"\nwarning=" + ss.setNum(warning);
    if (CALCMODE == EQROTOR)
    {
        outres += "\n\nРезультаты расчета эквивалентного винта по классической теории:\n\n";
        outres += "delta_T=      " + ss.setNum(del_T     ,'f',3) + " %      \tпогрешность по тяге;\n";
    }
    if (CALCMODE == FORCES)
    {
        outres += "\n\nРезультаты расчета усилий на втулке ТВ с заданным управлением по классической теории:\n\n";
    }
    outres += "pr.alfa=      " + ss.setNum(alpha     *r2d,'f',3) + " грд    \tугол атаки винта;\n";
    outres += "pr.fi0=       " + ss.setNum(fi0       *r2d,'f',3) + " грд    \tугол общего шага;\n";
    outres += "pr.rpm=       " + ss.setNum(rpm           ,'f',1) + " об/мин  \tчастота вращения толкающего винта\n";
    outres += RotorBase::Res2Str(log);
    return outres;
}

void PushRotor::fillOutAz(outstructure &out, int az)//Classic
{
    //out.iter = iter;
    //out.t    = t;
    out.pr.nb    = zl;
    out.pr.nSec  = nSec;
    out.pr.D     = 2000*R;
    out.pr.l_vsh = 1000*l_vsh;
    //out.pr.dct.x= 0;
    //out.pr.dct.y= 0;
    //out.pr.dct.z= 0;
    out.pr.az    = az;
    out.pr.sec07 = sec07;
    out.pr.revcount=0;
    out.pr.alfa  = alpha*r2d;
    out.pr.beta  = 0;
    out.pr.fi0   = fi0*r2d;

 if (az <= nAz)
  {
    //out.iter    = iter.at(az);
    //out.t       = t.at(az);
    out.pr.psih = psi0.at(az)*r2d;
    ///параметры возмущенного движения
    //out.pr.eh.x = ehx;    out.pr.eh.y = ehy;    out.pr.eh.z = ehz;
    //out.pr.wh.x = whx;    out.pr.wh.y = why;    out.pr.wh.z = whz;
    //out.pr.ah.x = ahx;    out.pr.ah.y = ahy;    out.pr.ah.z = ahz;
    //out.pr.vh.x = vhx;    out.pr.vh.y = vhy;    out.pr.vh.z = vhz;
    //out.pr.sh.x = shz;    out.pr.sh.y = shy;    out.pr.sh.z = shz;
    ///суммарные силы и моменты на втулке от всех лопастей (в полусвязанной CK):";
    out.pr.Fhx_sum = Fhx_S.at(az);
    out.pr.Fhy_sum = Fhy_S.at(az);
    out.pr.Fhz_sum = Fhz_S.at(az);
    out.pr.Mhx_sum = Mhx_S.at(az);
    out.pr.Mhy_sum = Mhy_S.at(az);
    out.pr.Mhz_sum = Mhz_S.at(az);
    out.pr.Fhx_sr  = Fhx_sr;//H_sr ;//
    out.pr.Fhy_sr  = Fhy_sr;//T_sr ;//
    out.pr.Fhz_sr  = Fhz_sr;//S_sr ;//
    out.pr.Mhx_sr  = Mhx_sr;//Mx_sr;//
    out.pr.Mhy_sr  = Mhy_sr;//My_sr;//
    out.pr.Mhz_sr  = Mhz_sr;//Mz_sr;//

    for (int l = 0; l < zl; l++)
    {
        ///Параметры лопасти по азимуту
        out.pr.b[l].psi0    = b[l].psi0 *r2d;//массив расчетных азимутов, градусы
        out.pr.b[l].psi     = b[l].psi   .at(az)*r2d;//массив расчетных азимутов, градусы
        out.pr.b[l].fi7     = b[l].fi7   .at(az)*r2d;// номер сечения 0,7R
        out.pr.b[l].dfi7    = b[l].dfi7  .at(az);//
        out.pr.b[l].d2fi7   = b[l].d2fi7 .at(az);//
        out.pr.b[l].gama    = b[l].gama  .at(az)*r2d;
        out.pr.b[l].dgama   = b[l].dgama .at(az);
        out.pr.b[l].d2gama  = b[l].d2gama.at(az);
        out.pr.b[l].beta    = b[l].beta  .at(az)*r2d;//вектор углов взмаха, градусы
        out.pr.b[l].dbeta   = b[l].dbeta .at(az);// скорость взмаха
        out.pr.b[l].d2beta  = b[l].d2beta.at(az);// ускорение взмаха
        out.pr.b[l].ksi     = b[l].ksi   .at(az)*r2d;//вектор углов качания, градусы
        out.pr.b[l].dksi    = b[l].dksi  .at(az);// скорость качания
        out.pr.b[l].d2ksi   = b[l].d2ksi .at(az);// ускорение качания

        out.pr.b[l].T       = b[l].T    .at(az);//сила тяги лопасти в конкретном азимуте, Н
        out.pr.b[l].Q       = b[l].Q    .at(az);//сила сопротивления лопасти в конкретном азимуте, Н
        out.pr.b[l].Ma      = b[l].Ma   .at(az);//
        out.pr.b[l].Nr      = b[l].Nr   .at(az);//центробежная сила в конкретном азимуте, Н
        out.pr.b[l].Nf      = b[l].Nf   .at(az);//центробежная сила от маховых движений в конкретном азимуте, Н
        out.pr.b[l].P1      = b[l].P1   .at(az);//сила инерции взмаха в конкретном азимуте, Н
        out.pr.b[l].P2      = b[l].P2   .at(az);//сила инерции качания в конкретном азимуте, Н
        out.pr.b[l].K1      = b[l].K1   .at(az);//сила Кориолиса в плоскости вращения в конкретном азимуте, Н
        out.pr.b[l].Fdmp    = b[l].Fdmp .at(az);
        out.pr.b[l].Fy      = b[l].Fy   .at(az);//проекция суммарной силы на ось у в с-ме кординат лопасти в конкретном азимуте, Н
        out.pr.b[l].Fx      = b[l].Fx   .at(az);//проекция суммарной силы на ось х в с-ме кординат лопасти в конкретном азимуте, Н
        out.pr.b[l].Fz      = b[l].Fz   .at(az);//проекция суммарной силы на ось z в с-ме кординат лопасти в конкретном азимуте, Н
        ///ГШ, СКПС,
        //out.pr.b[l].Ty      = b[l].Ty   .at(az);//проекция на ось Y винта силы тяги лопасти в конкретном азимуте, Н
        out.pr.b[l].H       = b[l].H    .at(az);//продольная сила лопасти в конкретном азимуте, Н
        out.pr.b[l].S       = b[l].S    .at(az);//поперечная сила лопасти в конкретном азимуте, Н
        ///ГШ, СКЛ,
        out.pr.b[l].M1T     = b[l].M1T  .at(az);
        out.pr.b[l].M1Q     = b[l].M1Q  .at(az);//момент силы сопротивления лопасти отн. оси у1, Н*м
        out.pr.b[l].M1Gx    = b[l].M1Gx .at(az);
        out.pr.b[l].M1Gy    = b[l].M1Gy .at(az);
        out.pr.b[l].M1K     = b[l].M1K  .at(az);//момент Кориолисовых сил лопасти отн. y1, Н*м
        out.pr.b[l].M1Cx    = b[l].M1Cx .at(az);
        out.pr.b[l].M1Cy    = b[l].M1Cy .at(az);
        out.pr.b[l].M1Px    = b[l].M1Px .at(az);
        out.pr.b[l].M1Py    = b[l].M1Py .at(az);
        out.pr.b[l].M1x     = b[l].M1x  .at(az);//суммарный момент сил отн.ГШ, Н*м
        out.pr.b[l].M1z     = b[l].M1z  .at(az);//суммарный момент сил отн.ГШ, Н*м
        out.pr.b[l].M1y     = b[l].M1y  .at(az);//суммарный момент сил отн.ГШ, Н*м
        ///ВШ, СКЛ,
        out.pr.b[l].M2T     = b[l].M2T  .at(az);
        out.pr.b[l].M2Q     = b[l].M2Q  .at(az);
        out.pr.b[l].M2Gx    = b[l].M2Gx .at(az);
        out.pr.b[l].M2Gy    = b[l].M2Gy .at(az);
        out.pr.b[l].M2K     = b[l].M2K  .at(az);
        out.pr.b[l].M2Cx    = b[l].M2Cx .at(az);
        out.pr.b[l].M2Cy    = b[l].M2Cy .at(az);
        out.pr.b[l].M2Px    = b[l].M2Px .at(az);
        out.pr.b[l].M2Py    = b[l].M2Py .at(az);
        out.pr.b[l].M2D     = b[l].M2D  .at(az);
        out.pr.b[l].M2x     = b[l].M2x  .at(az);//суммарный момент сил отн.ВШ, Н*м
        out.pr.b[l].M2z     = b[l].M2z  .at(az);//суммарный момент сил отн.ВШ, Н*м
        out.pr.b[l].M2y     = b[l].M2y  .at(az);//суммарный момент сил отн.ВШ, Н*м
        ///OШ, СКЛ,
        out.pr.b[l].M3Ma    = b[l].M3Ma .at(az);//
        out.pr.b[l].M3Cz    = b[l].M3Cz .at(az);//
        out.pr.b[l].M3su    = b[l].M3su .at(az);//
        out.pr.b[l].Fpov    = b[l].Fpov .at(az);//
        out.pr.b[l].M3tr    = b[l].M3tr .at(az) + b[l].M3dmp.at(az);//
        out.pr.b[l].M3iz    = b[l].M3iz .at(az);//
        out.pr.b[l].M3x     = b[l].M3x  .at(az);//суммарный момент сил отн.OШ, Н*м
        out.pr.b[l].M3z     = b[l].M3z  .at(az);//суммарный момент сил отн.OШ, Н*м
        out.pr.b[l].M3y     = b[l].M3y  .at(az);//суммарный момент сил отн.OШ, Н*м
        ///Центр втулки, СКПС,
        //out.pr.b[l].MQ      = b[l].MQ   .at(az);//момент силы сопротивления лопасти отн. оси винта, Н*м
        //out.pr.b[l].Mxa     = b[l].Mxa  .at(az);//момент аэрод. сил отн. оси хн винта в конкретном азимуте, Н*м
        //out.pr.b[l].Mza     = b[l].Mza  .at(az);//момент аэрод. сил отн. оси zн винта в конкретном азимуте, Н*м
        //out.pr.b[l].Mxj     = b[l].Mxj  .at(az);//момент инерционных сил отн. оси хн винта в конкретном азимуте, Н*м
        //out.pr.b[l].Mzj     = b[l].Mzj  .at(az);//момент инерционных сил отн. оси zн винта в конкретном азимуте, Н*м
        out.pr.b[l].Fhx     = b[l].Fhx  .at(az);//на втулке от одной лопасти (в полусвязанной CK)
        out.pr.b[l].Fhy     = b[l].Fhy  .at(az);//на втулке от одной лопасти (в полусвязанной CK)
        out.pr.b[l].Fhz     = b[l].Fhz  .at(az);//на втулке от одной лопасти (в полусвязанной CK)
        out.pr.b[l].Mhx     = b[l].Mhx  .at(az);//на втулке от одной лопасти (в полусвязанной CK)
        out.pr.b[l].Mhy     = b[l].Mhy  .at(az);//на втулке от одной лопасти (в полусвязанной CK)
        out.pr.b[l].Mhz     = b[l].Mhz  .at(az);//на втулке от одной лопасти (в полусвязанной CK)
        ///Распределенные параметры лопасти по сечению и по азимуту)
        out.pr.b[l]._r    .clear();
        out.pr.b[l].r     .clear();
        out.pr.b[l].alfar .clear();
        out.pr.b[l].Ux    .clear();
        out.pr.b[l].Uy    .clear();
        out.pr.b[l].U     .clear();
        out.pr.b[l].Uywz  .clear();
        out.pr.b[l].Uywx  .clear();
        out.pr.b[l].fi    .clear();
        out.pr.b[l].fl    .clear();
        out.pr.b[l].Fi    .clear();
        out.pr.b[l].lambda.clear();
        out.pr.b[l].M     .clear();
        out.pr.b[l].cy    .clear();
        out.pr.b[l].cx    .clear();
        out.pr.b[l].mz    .clear();
        out.pr.b[l].dT_dr .clear();
        out.pr.b[l].dQ_dr .clear();
        out.pr.b[l].dMa_dr .clear();
        for (int sec = 0; sec < nSec; sec++)
        {
            out.pr.b[l]._r    .push_back(_r.at(sec) );
            out.pr.b[l].r     .push_back( r.at(sec) );
            out.pr.b[l].alfar .push_back( b[l].alfar.at(az).at(sec)*r2d);//Угол атаки alpha_r, градусы
            out.pr.b[l].U     .push_back( b[l].U.at(az).at(sec)      );//Результирующая скорость обтекания U, м/с
            out.pr.b[l].Ux    .push_back( b[l].Ux.at(az).at(sec)     );//скорость обтекания Ux, м/с
            out.pr.b[l].Uy    .push_back( b[l].Uy.at(az).at(sec)     );//скорость обтекания Uy, м/с
            out.pr.b[l].Uywx  .push_back( b[l].Uywx.at(az).at(sec) );//
            out.pr.b[l].Uywz  .push_back( b[l].Uywz.at(az).at(sec) );//
            out.pr.b[l].fi    .push_back( b[l].fi.at(az).at(sec) *r2d);//Угол установки сечения fi, градусы
            out.pr.b[l].fl    .push_back( b[l].fl.at(az).at(sec) *r2d);//Угол
            out.pr.b[l].Fi    .push_back( b[l].Fi.at(az).at(sec) *r2d);//Угол притекания Fi, градусы
            out.pr.b[l].lambda.push_back( b[l].lambda.at(az).at(sec) );//коэф. протекания в сечении
            out.pr.b[l].M     .push_back( b[l].M.at(az).at(sec)      );//Число Маха M
            out.pr.b[l].cy    .push_back( b[l].cy.at(az).at(sec)     );//Коэффициент подъемной силы Cy
            out.pr.b[l].cx    .push_back( b[l].cx.at(az).at(sec)     );//Коэффициент профильного сопротивления Cxp
            out.pr.b[l].mz    .push_back( b[l].mz.at(az).at(sec)     );//
            out.pr.b[l].dT_dr .push_back( b[l].dT_dr.at(az).at(sec)  );//Распределение погонной силы тяги dT/dr, [Н/м]
            out.pr.b[l].dQ_dr .push_back( b[l].dQ_dr.at(az).at(sec)  );//Распределение погонной силы сопротивления вращению dQ/dr, [Н/м]
            out.pr.b[l].dMa_dr.push_back( b[l].dMa_dr.at(az).at(sec) );//
        }
    }
  }
}

void PushRotor::fillOut(outstructure &out)//out-структура результатов расчета FlyDynamic
{
    out.pr.nb    = zl;
    out.pr.nSec  = nSec;
    out.pr.D     = 2000*R;
    out.pr.l_vsh = 1000*l_vsh;
    out.pr.sec07 = sec07;
    out.pr.alfa  = alpha*r2d;
    out.pr.beta  = 0;
    out.pr.fi0   = fi0*r2d;
    //суммарные силы и моменты на втулке от всех лопастей (в полусвязанной CK)
    out.pr.Fhx_sum = Fhx_sum;
    out.pr.Fhy_sum = Fhy_sum;
    out.pr.Fhz_sum = Fhz_sum;
    out.pr.Mhx_sum = Mhx_sum;
    out.pr.Mhy_sum = Mhy_sum;
    out.pr.Mhz_sum = Mhz_sum;
    //Fh*_sr не использовать, оно может использоваться только после завершения оборота
    out.pr.Fhx_sr  = H_sr ;
    out.pr.Fhy_sr  = T_sr ;
    out.pr.Fhz_sr  = S_sr ;
    out.pr.Mhx_sr  = Mx_sr;
    out.pr.Mhy_sr  = My_sr;
    out.pr.Mhz_sr  = Mz_sr;

    out.pr.revcount=revcount;
    out.pr.psih = psih*r2d;//азимут-положение втулки винта
    for (int l = 0; l < out.pr.nb; l++)
    {
        out.pr.b[l].psi0  = B[l].psi0  *r2d;//азимут лопасти в начале расчета, градусы
        out.pr.b[l].psi   = B[l].psi   *r2d;//текущий азимут лопасти, градусы
        out.pr.b[l].fi7   = B[l].fi7   *r2d;//угол установки сечения 0,7R
        out.pr.b[l].dfi7  = B[l].dfi7  ;//
        out.pr.b[l].d2fi7 = B[l].d2fi7 ;//
        out.pr.b[l].gama  = B[l].gama  *r2d;
        out.pr.b[l].dgama = B[l].dgama ;
        out.pr.b[l].d2gama= B[l].d2gama;
        out.pr.b[l].beta  = B[l].beta  *r2d;//вектор углов взмаха, градусы
        out.pr.b[l].dbeta = B[l].dbeta ;// скорость взмаха
        out.pr.b[l].d2beta= B[l].d2beta;// ускорение взмаха
        out.pr.b[l].ksi   = B[l].ksi   *r2d;//вектор углов качания, градусы
        out.pr.b[l].dksi  = B[l].dksi  ;// скорость качания
        out.pr.b[l].d2ksi = B[l].d2ksi ;// ускорение качания
        out.pr.b[l].T     = B[l].T    ;//сила тяги лопасти в конкретном азимуте, Н
        out.pr.b[l].Q     = B[l].Q    ;//сила сопротивления лопасти в конкретном азимуте, Н
        out.pr.b[l].Ma    = B[l].Ma   ;
        out.pr.b[l].Nr    = B[l].Nr   ;//центробежная сила в конкретном азимуте, Н
        out.pr.b[l].Nf    = B[l].Nf   ;//центробежная сила от маховых движений в конкретном азимуте, Н
        out.pr.b[l].P1    = B[l].P1   ;//сила инерции взмаха в конкретном азимуте, Н
        out.pr.b[l].P2    = B[l].P2   ;//сила инерции качания в конкретном азимуте, Н
        out.pr.b[l].K1    = B[l].K1   ;//сила Кориолиса в плоскости вращения в конкретном азимуте, Н
        //out.pr.b[l].K2    = 0;//B[l].K2;//сила Кориолиса в плоскости вращения в конкретном азимуте, Н
        out.pr.b[l].Fdmp  = B[l].Fdmp ;
        //out.pr.b[l].Paxz  = B[l].Paxz;////сила инерции в плоскости винта (от линейного перемещения вдоль продольной и поперечной осей CKПС)
        //out.pr.b[l].Payy  = B[l].Payy;////общая cила инерции в плоскости тяги
        //out.pr.b[l].Pexz  = B[l].Pexz;//сила инерции от вращения НВ вокруг продольной и поперечной осей CKПС
        //out.pr.b[l].Pkxz  = B[l].Pkxz;//силы инерции Кориолиса, возникающие от вращения НВ вокруг продольной и поперечной осей CKПС на азимуте psi

        out.pr.b[l].Fy   = B[l].Fy  ;//проекция суммарной силы на ось у в с-ме кординат лопасти в конкретном азимуте, Н
        out.pr.b[l].Fx   = B[l].Fx  ;//проекция суммарной силы на ось х в с-ме кординат лопасти в конкретном азимуте, Н
        out.pr.b[l].Fz   = B[l].Fz  ;//проекция суммарной силы на ось z в с-ме кординат лопасти в конкретном азимуте, Н
        ///ГШ, СКПС,
        //out.pr.b[l].Ty    = B[l].Ty   ;//проекция на ось Y винта силы тяги лопасти в конкретном азимуте, Н
        out.pr.b[l].H     = B[l].H    ;//продольная сила лопасти в конкретном азимуте, Н
        out.pr.b[l].S     = B[l].S    ;//поперечная сила лопасти в конкретном азимуте, Н
        ///ГШ, СКЛ,
        out.pr.b[l].M1T   = B[l].M1T  ;
        out.pr.b[l].M1Q   = B[l].M1Q  ;//момент силы сопротивления лопасти отн. оси у1, Н*м
        out.pr.b[l].M1Gx  = B[l].M1Gx ;
        out.pr.b[l].M1Gy  = B[l].M1Gy ;
        out.pr.b[l].M1K   = B[l].M1K  ;//момент Кориолисовых сил лопасти отн. y1, Н*м
        out.pr.b[l].M1Cx  = B[l].M1Cx ;
        out.pr.b[l].M1Cy  = B[l].M1Cy ;
        out.pr.b[l].M1Px  = B[l].M1Px ;
        out.pr.b[l].M1Py  = B[l].M1Py ;
        out.pr.b[l].M1x   = B[l].M1x  ;//суммарный момент сил отн.ГШ, Н*м
        out.pr.b[l].M1z   = B[l].M1z  ;//суммарный момент сил отн.ГШ, Н*м
        out.pr.b[l].M1y   = B[l].M1y  ;//суммарный момент сил отн.ГШ, Н*м
        ///ВШ, СКЛ,
        out.pr.b[l].M2T   = B[l].M2T  ;
        out.pr.b[l].M2Q   = B[l].M2Q  ;
        out.pr.b[l].M2Gx  = B[l].M2Gx ;
        out.pr.b[l].M2Gy  = B[l].M2Gy ;
        out.pr.b[l].M2K   = B[l].M2K  ;
        out.pr.b[l].M2Cx  = B[l].M2Cx ;
        out.pr.b[l].M2Cy  = B[l].M2Cy ;
        out.pr.b[l].M2Px  = B[l].M2Px ;
        out.pr.b[l].M2Py  = B[l].M2Py ;
        out.pr.b[l].M2D   = B[l].M2D  ;
        out.pr.b[l].M2x   = B[l].M2x  ;//суммарный момент сил отн.ВШ, Н*м
        out.pr.b[l].M2z   = B[l].M2z  ;//суммарный момент сил отн.ВШ, Н*м
        out.pr.b[l].M2y   = B[l].M2y  ;//суммарный момент сил отн.ВШ, Н*м
        ///OШ, СКЛ,
        out.pr.b[l].M3Ma  = B[l].M3Ma ;//
        out.pr.b[l].M3Cz  = B[l].M3Cz ;//
        out.pr.b[l].M3su  = B[l].M3su ;//
        out.pr.b[l].Fpov  = B[l].Fpov ;//
        out.pr.b[l].M3tr  = B[l].M3tr + B[l].M3dmp;//
        out.pr.b[l].M3iz  = B[l].M3iz ;//
        out.pr.b[l].M3x   = B[l].M3x  ;//суммарный момент сил отн.OШ, Н*м
        out.pr.b[l].M3z   = B[l].M3z  ;//суммарный момент сил отн.OШ, Н*м
        out.pr.b[l].M3y   = B[l].M3y  ;//суммарный момент сил отн.OШ, Н*м
        ///Центр втулки, СКПС,
        //out.pr.b[l].MQ    = B[l].MQ   ;//момент силы сопротивления лопасти отн. оси винта, Н*м
        //out.pr.b[l].Mxa   = B[l].Mxa  ;//момент аэрод. сил отн. оси хн винта в конкретном азимуте, Н*м
        //out.pr.b[l].Mza   = B[l].Mza  ;//момент аэрод. сил отн. оси zн винта в конкретном азимуте, Н*м
        //out.pr.b[l].Mxj   = B[l].Mxj  ;//момент инерционных сил отн. оси хн винта в конкретном азимуте, Н*м
        //out.pr.b[l].Mzj   = B[l].Mzj  ;//момент инерционных сил отн. оси zн винта в конкретном азимуте, Н*м
        out.pr.b[l].Fhx   = B[l].Fhx  ;//на втулке от одной лопасти (в полусвязанной CK)
        out.pr.b[l].Fhy   = B[l].Fhy  ;//на втулке от одной лопасти (в полусвязанной CK)
        out.pr.b[l].Fhz   = B[l].Fhz  ;//на втулке от одной лопасти (в полусвязанной CK)
        out.pr.b[l].Mhx   = B[l].Mhx  ;//на втулке от одной лопасти (в полусвязанной CK)
        out.pr.b[l].Mhy   = B[l].Mhy  ;//на втулке от одной лопасти (в полусвязанной CK)
        out.pr.b[l].Mhz   = B[l].Mhz  ;//на втулке от одной лопасти (в полусвязанной CK)
        ///Распределенные по сечению параметры лопасти
        out.pr.b[l]._r    .clear();
        out.pr.b[l].r     .clear();
        out.pr.b[l].alfar .clear();
        out.pr.b[l].Ux    .clear();
        out.pr.b[l].Uy    .clear();
        out.pr.b[l].Uywz  .clear();
        out.pr.b[l].Uywx  .clear();
        out.pr.b[l].U     .clear();
        out.pr.b[l].fi    .clear();
        out.pr.b[l].fl    .clear();
        out.pr.b[l].Fi    .clear();
        out.pr.b[l].lambda.clear();
        out.pr.b[l].M     .clear();
        out.pr.b[l].cy    .clear();
        out.pr.b[l].cx    .clear();
        out.pr.b[l].mz    .clear();
        out.pr.b[l].dT_dr .clear();
        out.pr.b[l].dQ_dr .clear();
        out.pr.b[l].dMa_dr.clear();
        for (int sec = 0; sec < nSec; sec++)
        {
            out.pr.b[l]._r    .push_back(_r.at(sec) );
            out.pr.b[l].r     .push_back( r.at(sec) );
            out.pr.b[l].alfar .push_back( B[l].alfar .at(sec) *r2d);//Угол атаки alpha_r, градусы
            out.pr.b[l].Ux    .push_back( B[l].Ux    .at(sec) );//скорость обтекания Ux, м/с
            out.pr.b[l].Uy    .push_back( B[l].Uy    .at(sec) );//скорость обтекания Uy, м/с
            out.pr.b[l].Uywx  .push_back( B[l].Uywx  .at(sec) );//
            out.pr.b[l].Uywz  .push_back( B[l].Uywz  .at(sec) );//
            out.pr.b[l].U     .push_back( B[l].U     .at(sec) );//Результирующая скорость обтекания U, м/с
            out.pr.b[l].fi    .push_back( B[l].fi    .at(sec) *r2d);//Угол установки сечения fi, градусы
            out.pr.b[l].fl    .push_back( B[l].fl    .at(sec) *r2d);//Угол
            out.pr.b[l].Fi    .push_back( B[l].Fi    .at(sec) *r2d);//Угол притекания Fi, градусы
            out.pr.b[l].lambda.push_back( B[l].lambda.at(sec) );//коэф. протекания в сечении
            out.pr.b[l].M     .push_back( B[l].M     .at(sec) );//Число Маха M
            out.pr.b[l].cy    .push_back( B[l].cy    .at(sec) );//Коэффициент подъемной силы Cy
            out.pr.b[l].cx    .push_back( B[l].cx    .at(sec) );//Коэффициент профильного сопротивления Cxp
            out.pr.b[l].mz    .push_back( B[l].mz    .at(sec) );//
            out.pr.b[l].dT_dr .push_back( B[l].dT_dr .at(sec) );//Распределение погонной силы тяги dT/dr, [Н/м]
            out.pr.b[l].dQ_dr .push_back( B[l].dQ_dr .at(sec) );//Распределение погонной силы сопротивления вращению dQ/dr, [Н/м]
            out.pr.b[l].dMa_dr.push_back( B[l].dMa_dr.at(sec) );//
        }
    }
}





///////////////////////////////////////////////////////////////////////////////
