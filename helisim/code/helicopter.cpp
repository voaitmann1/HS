//HELICOPTER.MATHEMATICAL.MODEL
#include "helicopter.h"
#include "solver_hc.cpp"
#include "airframe.cpp"
//-----------------------------------------------------------------------------

Helicopter* Helicopter::phc;
Helicopter::Helicopter()
{
}
Helicopter::~Helicopter()
{
    delete mr;
    for(int k=0; k<nst; k++)
        delete st[k];
}
void Helicopter::SetClassPointer()
{
    phc = this;
}

void Helicopter::Pre()
{//универсальный препроцесс, полностью обеспечивающий pre_Балансировку вертолета и частично pre_Динамику_полета
    flyerror = 0;
    t = t_ = 0;
    pi  = M_PI;    d2r = M_PI/180.0;   r2d = 180.0/M_PI;
    gofly = false;

    //ПЛОТНОСТЬ ВОЗДУХА КАК ФУНКЦИЯ ВЫСОТЫ
    if (ro==0)
    {
        //float L = 0.0065, //скорость падения температуры К/м
        //R = 8.31447, //газовая постоянная Дж/(мол*К)
        //M = 0.0289644, //молярная масса сухого воздуха кг/мол
        //g = 9.80665,
        //T = t + 273.15 - L * H,
        //P_ = p * pow((1 - L * H / (15.0 + 273.15)), (g * M / (R * L)));
        //ro = P_ * M / (R * T);

        TK = t0 + 273.15 - Lpt*Hh;
        P_ = p0 * pow((1 - Lpt*Hh /(15.0 + 273.15)), (g*Mm /(RjmK*Lpt)));
        roh = P_* Mm /(RjmK*TK);//плотность воздуха
    }
    else roh = ro;
    vsound = sqrt(1.4*TK*RjkgK);//скорость звука

    //вес вертолета, Н
    Gh = Ghb = mh*g;

    //расстояние по оси НВ от втулки до строительной горизонтали
    l_mr = sqrt(x_mr*x_mr+y_mr*y_mr+z_mr*z_mr);
    //углы заклинения (постоянный наклон оси НВ)
    if(y_mr)
    {
        eps_fz =-atan(x_mr/y_mr);
        eps_fx = atan(z_mr/y_mr);
    }
    else { eps_fx = 0; eps_fz = 0; }

    //положение ЦТ вертолета в CКС, м
    yct = (l_mr-yctmr)*cos(-eps_fz)*cos(eps_fx) - xctmr*sin(-eps_fz)  - zctmr*sin(eps_fx);
    xct = (l_mr-yctmr)*sin(-eps_fz) + xctmr*cos(-eps_fz);
    zct = (l_mr-yctmr)*sin( eps_fx) + zctmr*cos( eps_fx);
    //Расстояние от ЦТ вертолета до оси РВ, м
    Lctr = xct-x_tr;
    //Расстояние от оси НВ до 25% хорды стабилизатора, м
    //Lst = x_mr-x_st;
    //Расстояние от 25% хорды стабилизатора до точки пересечения констр.плоскости НВ и оси вращения НВ, м
    //lst_mr = sqrt( pow(x_mr-x_st,2) + pow(y_mr-y_st,2) );
    //Угол между констр.плоскостью НВ и линией соединяющей точку пересечения констр.плоскости НВ и оси НВ с точкой на 25% хорды стабилизатора, рад
    //if (lst_mr!=0) dlt_st = acos(Lst/lst_mr); else dlt_st = 0;

    ///кинематические параметры планера
    //положение СКО (конструктивной) в земной СК
    org = org_;
          kren = org.k;
          tang = org.t;
    //положение начала СКС в земной СК (начало СКС совпадает с ЦТ вертолета)
    ocw = org;//для работы org2ocw() нужно чтобы углы в ocw были определены (углы там одинаковы с org)
    ocw = ocw_ = org2ocw(org);
    //в нормальной СК
    vn.x = v_.x;
    vn.y = v_.y;
    vn.z = v_.z;
            //если угловые скорости заданы в СКН:
            //vn.k = w_.x;
            //vn.r = w_.y;
            //vn.t = w_.z;
    //в связанной СК
    v = v_ = SKNtoSKS(v_);
    w = w_;//а если задаем угловые скорости в СКН: w = w_ = SKNtoSKS(w_);
    //если угловые скорости заданы в СКС:
    w = SKStoSKN(w);//получили угловую скорость в СКН
    vn.k = w.x;
    vn.r = w.y;
    vn.t = w.z;
    w = w_;//возвращаем угловую скорость в СКС

    //усилия в ЦТ вертолета в первом приближении
    Fx = 0;           Mx = 0;      a.x = Fx/mh;
    Fy = Fy_mr_-Gh;   Mz = 0;      a.y = Fy/mh;
    Fz = 0;           My = 0;      a.z = Fz/mh;

    ///параметры управления вертолетом
    fi0_mr = fi0_mr_ = manual.fi0mr;
    fi0_pr = manual.fi0pr;//fi0_pr_
    fi0_tr = manual.fi0tr;
    if(mrtilt)//наклон втулки НВ
    {
        chi_hub = chi_hub_= manual.chimr;//ini.mr.chi_*M_PI/180.;
        eta_hub = eta_hub_= manual.etamr;//ini.mr.eta_*M_PI/180.;
        chi_ap  = chi_ap_ = 0;
        eta_ap  = eta_ap_ = 0;
    }
    else//наклон тарелки АПНВ
    {
        chi_ap  = chi_ap_ = manual.chimr;
        eta_ap  = eta_ap_ = manual.etamr;
        chi_hub = chi_hub_= 0;
        eta_hub = eta_hub_= 0;
    }

    //параметры набегающего потока
    inFlow();// Vi, tetra, beta_

    //TAILROTOR
    if(trsolve)
    {
        tr->V       = (Vi)?(Vi):(0.01);
        tr->alpha_  = alfa_tr;
        tr->beta_sk = -tetra;///0; //20210804
        tr->logkey  = logkey;
        tr->Preparation();
    }
    //PUSHROTOR
    if(prsolve)
    {
        pr->V       = v.x *Kt_pr;//Vi
        pr->alpha_  = alfa_pr; //20210804
        pr->beta_sk = 0;
        pr->logkey  = logkey;
        pr->Preparation();
    }
    //MAINROTOR
    if(mrsolve)
    {
        mr->V       = Vi;
        mr->alpha_  = alfa_mr;
        mr->beta_sk = beta_sk;
        mr->logkey  = logkey;
        mr->Preparation();
    }
    Ttr_need = -My_mr_/Lctr;
    Rmr = 0.5*Dmr;
    Fmr = M_PI*Rmr*Rmr;
    zswlmr = 0.5*zl_mr*Sgsh_mr*w_mr*w_mr*lgsh;//вспомогательная величина для сил-моментов на втулке НВ
    D1_mr = D1_mr_;//передаточное отношение между углом отклонения равнодействующей и углом отклонения АП
    D2_mr = D2_mr_;//передаточное отношение между углом отклонения равнодействующей и углом отклонения АП

    //параметры модели планера
    initial_airdyn_characters();
    //Условный линейный размер при определении аэродин коэффициентов, м2
    Lmod = lmodel*Mmodel;
    //Условная площадь к которой отнесены моменты фюзеляжа при определении аэродин коэф., м2
    Smod = Smodel*Mmodel*Mmodel;

    //ГРУЗ НА ВНЕШНЕЙ ТРОСОВОЙ ПОДВЕСКЕ
    Gcg = mcg*g;
    Gcrg.x =  0;
    Gcrg.y = -Gcg;
    Gcrg.z =  0;
    crgct.x = zacep.x;
    crgct.y = zacep.y -lcg;
    crgct.z = zacep.z;

    //ВНЕШНЯЯ НАГРУЗКА
    ExternalLoads(0);
}

void Helicopter::inFlow()
{
    //скорость набегающего потока, м/с
    Vi  = sqrt( pow(v.x,2) + pow(v.y,2) + pow(v.z,2) );//без разницы sqrt( pow(vn.x,2) + pow(vn.y,2) + pow(vn.z,2) );
    Vxy = sqrt( pow(v.x,2) + pow(v.y,2) );
    Vxz = sqrt( pow(v.x,2) + pow(v.z,2) );//sqrt( pow(vn.x,2) + pow(vn.z,2) );

    //угол наклона траектории, рад
    tetra = (Vi!=0)?(asin(vn.y/Vi)):(0);//vn.y/Vxy  здесь нужна проекция vn.y в норм.СК

    //угол скольжения, рад
    beta_sk = (Vxz!=0)?(asin(v.z/Vxz)):(0);

    //угол атаки НВ, рад
    alfa_mr = org.t + eps_fz + chi_hub - tetra;

    //изменение угла атаки PВ от рыскания, рад
    double signwy = (w.y)?(w.y/fabs(w.y)):(0);
    skos_tr_rsk = (Vi)?( -w.y*(x_tr-xct)/Vi ):(90*d2r*signwy);//x_tr отрицательная
    //скос потока у PВ
    //skos_tr_ = 0;
    //угол атаки PВ, рад
    alfa_tr = (trmount)?( 0 -beta_sk -skos_tr_rsk +skos_tr_ ):(0);

    //угол атаки TВ, рад
    if(prmount)
    {
        alfa_pr = (Vxy)?(-asin( v.x / Vxy )):(-0.5*M_PI);//alfa_pr_;
    }
    else alfa_pr = 0;
}



/// Динамика полета (и посадки) вертолета

void Helicopter::PreFlying(outstructure &out)//подготовка к расчету ДИНАМИКИ ВЕРТОЛЕТА
{
    preout = "Flying.Prepare:\n\n";
    Pre();
    //TAILROTOR
    if(trsolve) tr->RotorIterate( Fy_tr_, 0, 0, Vi, alfa_tr, 0, fi0_tr, 0, 0 );
    //PUSHROTOR
    if(prsolve) pr->RotorIterate( Fy_pr_, 0, 0, Vi, alfa_pr, 0, fi0_pr, 0, 0 );
    //MAINROTOR
    if(mrsolve)
    {
        mr->H_sr = Fx_mr_;
        mr->T_sr = Fy_mr_;
        mr->S_sr = Fz_mr_;
        if(mrtilt) mr->RotorIterate( Fy_mr_,Fx_mr_,Fz_mr_, Vi, alfa_mr, beta_sk, fi0_mr, 0, 0 );
        else       mr->RotorIterate( Fy_mr_,Fx_mr_,Fz_mr_, Vi, alfa_mr, beta_sk, fi0_mr, eta_ap, chi_ap );
    }
    //ШАССИ
    for (int k=0; k<nst; k++)
    {
        st[k]->ab->dt = dt;
        st[k]->pn->dt = dt;
        st[k]->orient = org;
        st[k]->Preprocess();
        st[k]->Ppy = st[k]->pn->P.y;
        st[k]->Pay = st[k]->pn->P.y;
        xyz D_sks_ = st[k]->SKo2SKS( st[k]->D_ );
        xyz D_skn_ = st[k]->SKS2SKN( D_sks_ );
        st[k]->C   = st[k]->getC();// turnCpoint( st[k]->C, st[k]->D ); //hf,jnf yt djkr,ghjhzljr
        st[k]->saC_.x = org_.x + D_skn_.x;
        st[k]->saC_.y = org_.y + D_skn_.y - st[k]->pn->dpmd;
        st[k]->saC_.z = org_.z + D_skn_.z;
    }
    //Вывод
    fillOut(out);
    if(trsolve)  tr->fillOut(out);
    if(prsolve)  pr->fillOut(out);
    if(mrsolve)  mr->fillOut(out);
    if(logkey)
    {
        preout += "dt=       " + ss.setNum(dt             ,'f',6) + " с  \t\t- приращение времени на одном шаге;\n";
        preout += "g=        " + ss.setNum(g              ,'f',5) + " м/с2\t\t- ускорение свободного падения;\n";
        preout += "fix_x=    " + ss.setNum(fix_org_x            ) + "      \t\t- принудительное ограничение перемещения по оси x;\n";
        preout += "fix_y=    " + ss.setNum(fix_org_y            ) + "      \t\t- принудительное ограничение перемещения по оси y;\n";
        preout += "fix_z=    " + ss.setNum(fix_org_z            ) + "      \t\t- принудительное ограничение перемещения по оси z;\n";
        preout += "fix_k=    " + ss.setNum(fix_org_k            ) + "      \t\t- принудительное ограничение поворота отн.оси x (крена);\n";
        preout += "fix_r=    " + ss.setNum(fix_org_r            ) + "      \t\t- принудительное ограничение поворота отн.оси y (рыскания);\n";
        preout += "fix_t=    " + ss.setNum(fix_org_t            ) + "      \t\t- принудительное ограничение поворота отн.оси z (тангажа);\n";

        preout += "\nначальное положение:\n";
        preout += "Hx=       " + ss.setNum(org.x   *1000  ,'f',1) + " мм  \t\t- продольное положение начала связанной СК;\n";
        preout += "Hy=       " + ss.setNum(org.y   *1000  ,'f',1) + " мм  \t\t- положение по высоте  начала связанной СК;\n";
        preout += "Hz=       " + ss.setNum(org.z   *1000  ,'f',1) + " мм  \t\t- поперечное положение начала связанной СК;\n";
        preout += "kren=     " + ss.setNum(org.k   *180/pi,'f',4) + " грд \t\t- угол крена   (gama-поворот связанной СК);\n";
        preout += "rysk=     " + ss.setNum(org.r   *180/pi,'f',4) + " грд \t\t- угол рыскания (psi-поворот связанной СК);\n";
        preout += "tang=     " + ss.setNum(org.t   *180/pi,'f',4) + " грд \t\t- угол тангажа (teta-поворот связанной СК);\n";
        preout += "tetra=    " + ss.setNum(tetra   *180/pi,'f',4) + " грд \t\t- угол наклона траектории\n";
        preout += "betsk=    " + ss.setNum(beta_sk *180/pi,'f',4) + " грд \t\t- угол скольжения;\n";
        preout += "Vnx=      " + ss.setNum(vn.x           ,'f',4) + " м/с \t\t- продольная составляющая линейной скорости   (в земной СК);\n";
        preout += "Vny=      " + ss.setNum(vn.y           ,'f',4) + " м/с \t\t- вертикальная составляющая линейной скорости (в земной СК);\n";
        preout += "Vnz=      " + ss.setNum(vn.z           ,'f',4) + " м/с \t\t- боковая составляющая линейной скорости      (в земной СК);\n";
        //preout += "Wnx=      " + ss.setNum(vn.k    *180/pi,'f',4) + " грд/с\t\t- угловая скорость вертолета отн. оX земной СК;\n";
        //preout += "Wny=      " + ss.setNum(vn.r    *180/pi,'f',4) + " грд/с\t\t- угловая скорость вертолета отн. оY земной СК;\n";
        //preout += "Wnz=      " + ss.setNum(vn.t    *180/pi,'f',4) + " грд/с\t\t- угловая скорость вертолета отн. оZ земной СК;\n";
        preout += "v.x=      " + ss.setNum(v.x            ,'f',4) + " м/с \t\t- продольная составляющая линейной скорости   (в связанной СК);\n";
        preout += "v.y=      " + ss.setNum(v.y            ,'f',4) + " м/с \t\t- вертикальная составляющая линейной скорости (в связанной СК);\n";
        preout += "v.z=      " + ss.setNum(v.z            ,'f',4) + " м/с \t\t- боковая составляющая линейной скорости      (в связанной СК);\n";
        preout += "w.x=      " + ss.setNum(w.x     *180/pi,'f',4) + " грд/с\t\t- угловая скорость вертолета отн. оX связанной СК;\n";
        preout += "w.y=      " + ss.setNum(w.y     *180/pi,'f',4) + " грд/с\t\t- угловая скорость вертолета отн. оY связанной СК;\n";
        preout += "w.z=      " + ss.setNum(w.z     *180/pi,'f',4) + " грд/с\t\t- угловая скорость вертолета отн. оZ связанной СК;\n";

        preout += "\nпланер:\n";
        preout += "Gh=       " + ss.setNum(Gh             ,'f',1) + " Н   \t\t- расчетный вес вертолета;\n";
        preout += "eps_fz=   " + ss.setNum(eps_fz  *180/pi,'f',4) + " грд \t\t- угол заклинения фюзеляжа в продольной плоскости (+назад);\n";
        preout += "eps_fx=   " + ss.setNum(eps_fx  *180/pi,'f',4) + " грд \t\t- угол заклинения фюзеляжа в поперечной плоскости (+вправо);\n";
        preout += "l_mr=     " + ss.setNum(l_mr    *1000  ,'f',1) + " мм  \t\t- расстояние по оси НВ от КПНВ до строительной горизонтали;\n";
        preout += "xctmr=    " + ss.setNum(xctmr   *1000  ,'f',1) + " мм  \t\t- x-центровка вертолета от оси НВ;\n";
        preout += "yctmr=    " + ss.setNum(yctmr   *1000  ,'f',1) + " мм  \t\t- y-центровка вертолета от КПНВ;\n";
        preout += "zctmr=    " + ss.setNum(zctmr   *1000  ,'f',1) + " мм  \t\t- z-центровка вертолета от оси НВ;\n";
        preout += "xct=      " + ss.setNum(xct     *1000  ,'f',1) + " мм  \t\t- положение ЦТ вертолета в CКС;\n";
        preout += "yct=      " + ss.setNum(yct     *1000  ,'f',1) + " мм  \t\t- положение ЦТ вертолета в CКС;\n";
        preout += "zct=      " + ss.setNum(zct     *1000  ,'f',1) + " мм  \t\t- положение ЦТ вертолета в CКС;\n";
        preout += "x_f=      " + ss.setNum(x_f     *1000  ,'f',1) + " мм  \t\t- характерная точка для определения скоса на фюзеляже от НВ по оси Ох;\n";
        preout += "y_f=      " + ss.setNum(y_f     *1000  ,'f',1) + " мм  \t\t- характерная точка для определения скоса на фюзеляже от НВ по оси Оу;\n";
        preout += "z_f=      " + ss.setNum(z_f     *1000  ,'f',1) + " мм  \t\t- характерная точка для определения скоса на фюзеляже от НВ по оси Оz;\n";
        preout += "Ix=       " + ss.setNum(Ix             ,'f',1) + " кг*м2  \t- момент инерции вертолета относительно оси OХ;\n";
        preout += "Iy=       " + ss.setNum(Iy             ,'f',1) + " кг*м2  \t- момент инерции вертолета относительно оси OY;\n";
        preout += "Iz=       " + ss.setNum(Iz             ,'f',1) + " кг*м2  \t- момент инерции вертолета относительно оси OZ;\n";
        preout += "Iyz=      " + ss.setNum(Iyz            ,'f',1) + " кг*м2\t\t- момент инерции вертолета;\n";
        preout += "Ixz=      " + ss.setNum(Ixz            ,'f',1) + " кг*м2\t\t- момент инерции вертолета;\n";
        preout += "Ixy=      " + ss.setNum(Ixy            ,'f',1) + " кг*м2\t\t- момент инерции вертолета;\n";

        preout += "\nнесущий винт:\n";
        preout += "mrsolve=  " + ss.setNum(mrsolve              ) + "      \t\t- пересчитывать усилия на втулке\n";
        preout += "mr.O.x=   " + ss.setNum(x_mr      *1000,'f',1) + " мм \t\t- положение центра втулки НВ в CКС;\n";
        preout += "mr.O.y=   " + ss.setNum(y_mr      *1000,'f',1) + " мм \t\t- положение центра втулки НВ в CКС;\n";
        preout += "mr.O.z=   " + ss.setNum(z_mr      *1000,'f',1) + " мм \t\t- положение центра втулки НВ в CКС;\n";
        preout += "mr.Fx=    " + ss.setNum(Fx_mr          ,'f',1) + " Н  \t\t- (X СКНВ) продольная сила НВ   [(+)назад];\n";
        preout += "mr.Fy=    " + ss.setNum(Fy_mr          ,'f',1) + " Н  \t\t- (Y СКНВ) вертикальная сила НВ [(+)вверх];\n";
        preout += "mr.Fz=    " + ss.setNum(Fz_mr          ,'f',1) + " Н  \t\t- (Z СКНВ) боковая сила НВ      [(+)влево];\n";
        preout += "mr.Mx=    " + ss.setNum(Mx_mr          ,'f',1) + " Н*м\t\t- момент крена несущего винта;\n";
        preout += "mr.My=    " + ss.setNum(My_mr          ,'f',1) + " Н*м\t\t- момент кручения НВ;\n";
        preout += "mr.Mz=    " + ss.setNum(Mz_mr          ,'f',1) + " Н*м\t\t- момент тангажа несущего винта;\n";
        if(trmount)
        {
        preout += "\nрулевой винт:\n";
        preout += "trsolve=  " + ss.setNum(trsolve              ) + "      \t\t- пересчитывать усилия на втулке\n";
        preout += "tr.O.x=   " + ss.setNum(x_tr      *1000,'f',1) + " мм \t\t- положение центра втулки РВ в CКС;\n";
        preout += "tr.O.y=   " + ss.setNum(y_tr      *1000,'f',1) + " мм \t\t- положение центра втулки РВ в CКС;\n";
        preout += "tr.O.z=   " + ss.setNum(z_tr      *1000,'f',1) + " мм \t\t- положение центра втулки РВ в CКС;\n";
        preout += "tr.Fx=    " + ss.setNum(Fx_tr          ,'f',1) + " Н  \t\t- (X СКРВ) продольная сила рулевого винта [(+)назад];\n";
        preout += "tr.Fy=    " + ss.setNum(Fy_tr          ,'f',1) + " Н  \t\t- (Y СКРВ) тяга рулевого винта           [(+)вправо];\n";
        preout += "tr.Fz=    " + ss.setNum(Fz_tr          ,'f',1) + " Н  \t\t- (Z СКРВ) поперечная сила рулевого винта [(+)вверх];\n";
        preout += "tr.Mx=    " + ss.setNum(Mx_tr          ,'f',1) + " Н*м\t\t- продольный момент рулевого винта;\n";
        preout += "tr.My=    " + ss.setNum(My_tr          ,'f',1) + " Н*м\t\t- вертикальный момент рулевого винта;\n";
        preout += "tr.Mz=    " + ss.setNum(Mz_tr          ,'f',1) + " Н*м\t\t- поперечный момент рулевого винта;\n";
        }
        if(prmount)
        {
        preout += "\nтолкающий винт:\n";
        preout += "prsolve=  " + ss.setNum(prsolve              ) + "      \t\t- пересчитывать усилия на втулке\n";
        preout += "pr.O.x=   " + ss.setNum(x_pr      *1000,'f',1) + " мм \t\t- положение центра втулки ТВ в CКС;\n";
        preout += "pr.O.y=   " + ss.setNum(y_pr      *1000,'f',1) + " мм \t\t- положение центра втулки ТВ в CКС;\n";
        preout += "pr.O.z=   " + ss.setNum(z_pr      *1000,'f',1) + " мм \t\t- положение центра втулки ТВ в CКС;\n";
        preout += "pr.Fx=    " + ss.setNum(Fx_pr          ,'f',1) + " Н  \t\t- (X СКРВ) продольная сила толкающего винта [(+)вправо];\n";
        preout += "pr.Fy=    " + ss.setNum(Fy_pr          ,'f',1) + " Н  \t\t- (Y СКРВ) тяга толкающего винта            [(+)вперед];\n";
        preout += "pr.Fz=    " + ss.setNum(Fz_pr          ,'f',1) + " Н  \t\t- (Z СКРВ) поперечная сила толкающего винта [(+)вверх];\n";
        preout += "pr.Mx=    " + ss.setNum(Mx_pr          ,'f',1) + " Н*м\t\t- продольный момент толкающего винта;\n";
        preout += "pr.My=    " + ss.setNum(My_pr          ,'f',1) + " Н*м\t\t- вертикальный момент толкающего винта;\n";
        preout += "pr.Mz=    " + ss.setNum(Mz_pr          ,'f',1) + " Н*м\t\t- поперечный момент толкающего винта;\n";
        }

        preout += "\nшасси: ";
        preout += "nst= "  + ss.setNum(nst) + "\n";
        for (int k=0; k<nst; k++)
        {
            xyz sks_A  =  st[k]->SKo2SKS(st[k]->A);
            xyz sks_B  =  st[k]->SKo2SKS(st[k]->B);
            xyz sks_Q  =  st[k]->SKo2SKS(st[k]->Q);
            xyz sks_V  =  st[k]->SKo2SKS(st[k]->V);
            xyz sks_W  =  st[k]->SKo2SKS(st[k]->W);
            xyz sks_D  =  st[k]->SKo2SKS(st[k]->D);
            xyz sks_C  =  st[k]->SKo2SKS(st[k]->C);
            xyz sks_W1 =  st[k]->SKo2SKS(st[k]->W1);
            xyz sks_D1 =  st[k]->SKo2SKS(st[k]->D1);
            xyz sks_C1 =  st[k]->SKo2SKS(st[k]->C1);
            xyz sks_W2 =  st[k]->SKo2SKS(st[k]->W2);
            xyz sks_D2 =  st[k]->SKo2SKS(st[k]->D2);
            xyz sks_C2 =  st[k]->SKo2SKS(st[k]->C2);
            xyz sks_Bfull=st[k]->SKo2SKS(st[k]->Bfull);
            xyz sks_Wfull=st[k]->SKo2SKS(st[k]->Wfull);
            //xyz sks_Wfree=st[k]->SKo2SKS(st[k]->Wfree);

            preout += "стойка №" + ss.setNum(k) + ":\n";
            preout += "s"+sss.setNum(k)+".stype=     " + ss.setNum(st[k]->stype                  ) + "  \t\t- тип стойки;\n";
            preout += "s"+sss.setNum(k)+".ptype=     " + ss.setNum(st[k]->ptype                  ) + "  \t\t- тип пневматика;\n";
            preout += "s"+sss.setNum(k)+".atype=     " + ss.setNum(st[k]->atype                  ) + "  \t\t- тип амортизатора;\n";
            preout += "s"+sss.setNum(k)+".nwh=       " + ss.setNum(st[k]->nwh                    ) + "  \t\t- количество колес;\n";
            preout += "s"+sss.setNum(k)+".brake=     " + ss.setNum(st[k]->pn->isbrake            ) + "  \t\t- блокировка колеса тормозом;\n";
            preout += "s"+sss.setNum(k)+".O.x=       " + ss.setNum(st[k]->O.x         *1000,'f',1) + " мм    \t- положение начала локальной СК отн.начала СКС;\n";
            preout += "s"+sss.setNum(k)+".O.y=       " + ss.setNum(st[k]->O.y         *1000,'f',1) + " мм    \t- положение начала локальной СК отн.начала СКС;\n";
            preout += "s"+sss.setNum(k)+".O.z=       " + ss.setNum(st[k]->O.z         *1000,'f',1) + " мм    \t- положение начала локальной СК отн.начала СКС;\n";
            preout += "s"+sss.setNum(k)+".A.x=       " + ss.setNum(sks_A.x            *1000,'f',1) + " мм    \t- координаты верхней точки крепления амортизатора;\n";
            preout += "s"+sss.setNum(k)+".A.y=       " + ss.setNum(sks_A.y            *1000,'f',1) + " мм    \t- координаты верхней точки крепления амортизатора;\n";
            preout += "s"+sss.setNum(k)+".A.z=       " + ss.setNum(sks_A.z            *1000,'f',1) + " мм    \t- координаты верхней точки крепления амортизатора;\n";
            preout += "s"+sss.setNum(k)+".B.x=       " + ss.setNum(sks_B.x            *1000,'f',1) + " мм    \t- координаты нижней точки крепления амортизатора в разжатом состоянии;\n";
            preout += "s"+sss.setNum(k)+".B.y=       " + ss.setNum(sks_B.y            *1000,'f',1) + " мм    \t- координаты нижней точки крепления амортизатора в разжатом состоянии;\n";
            preout += "s"+sss.setNum(k)+".B.z=       " + ss.setNum(sks_B.z            *1000,'f',1) + " мм    \t- координаты нижней точки крепления амортизатора в разжатом состоянии;\n";
            preout += "s"+sss.setNum(k)+".W.x=       " + ss.setNum(sks_W.x            *1000,'f',1) + " мм    \t- координаты центра колеса в разжатом состоянии;\n";
            preout += "s"+sss.setNum(k)+".W.y=       " + ss.setNum(sks_W.y            *1000,'f',1) + " мм    \t- координаты центра колеса в разжатом состоянии;\n";
            preout += "s"+sss.setNum(k)+".W.z=       " + ss.setNum(sks_W.z            *1000,'f',1) + " мм    \t- координаты центра колеса в разжатом состоянии;\n";
            preout += "s"+sss.setNum(k)+".D.x=       " + ss.setNum(sks_D.x            *1000,'f',1) + " мм    \t- координаты ближайшей к земле точки на ободе колеса (в плоскости симм.колеса) соотв.макс.допустимому обжатию пневматика;\n";
            preout += "s"+sss.setNum(k)+".D.y=       " + ss.setNum(sks_D.y            *1000,'f',1) + " мм    \t- координаты ближайшей к земле точки на ободе колеса (в плоскости симм.колеса) соотв.макс.допустимому обжатию пневматика;\n";
            preout += "s"+sss.setNum(k)+".D.z=       " + ss.setNum(sks_D.z            *1000,'f',1) + " мм    \t- координаты ближайшей к земле точки на ободе колеса (в плоскости симм.колеса) соотв.макс.допустимому обжатию пневматика;\n";
            preout += "s"+sss.setNum(k)+".C.x=       " + ss.setNum(sks_C.x            *1000,'f',1) + " мм    \t- координаты нижней точки пневматика (ближайшей к точке контакта или центр пятна контакта);\n";
            preout += "s"+sss.setNum(k)+".C.y=       " + ss.setNum(sks_C.y            *1000,'f',1) + " мм    \t- координаты нижней точки пневматика (ближайшей к точке контакта или центр пятна контакта);\n";
            preout += "s"+sss.setNum(k)+".C.z=       " + ss.setNum(sks_C.z            *1000,'f',1) + " мм    \t- координаты нижней точки пневматика (ближайшей к точке контакта или центр пятна контакта);\n";
            if(st[k]->nwh==2)
            {
            preout += "s"+sss.setNum(k)+".W1.x=      " + ss.setNum(sks_W1.x           *1000,'f',1) + " мм    \t- координаты центра колеса в разжатом состоянии;\n";
            preout += "s"+sss.setNum(k)+".W1.y=      " + ss.setNum(sks_W1.y           *1000,'f',1) + " мм    \t- координаты центра колеса в разжатом состоянии;\n";
            preout += "s"+sss.setNum(k)+".W1.z=      " + ss.setNum(sks_W1.z           *1000,'f',1) + " мм    \t- координаты центра колеса в разжатом состоянии;\n";
            preout += "s"+sss.setNum(k)+".D1.x=      " + ss.setNum(sks_D1.x           *1000,'f',1) + " мм    \t- координаты ближайшей к земле точки на ободе колеса (в плоскости симм.колеса) соотв.макс.допустимому обжатию пневматика;\n";
            preout += "s"+sss.setNum(k)+".D1.y=      " + ss.setNum(sks_D1.y           *1000,'f',1) + " мм    \t- координаты ближайшей к земле точки на ободе колеса (в плоскости симм.колеса) соотв.макс.допустимому обжатию пневматика;\n";
            preout += "s"+sss.setNum(k)+".D1.z=      " + ss.setNum(sks_D1.z           *1000,'f',1) + " мм    \t- координаты ближайшей к земле точки на ободе колеса (в плоскости симм.колеса) соотв.макс.допустимому обжатию пневматика;\n";
            preout += "s"+sss.setNum(k)+".C1.x=      " + ss.setNum(sks_C1.x           *1000,'f',1) + " мм    \t- координаты нижней точки пневматика (ближайшей к точке контакта или центр пятна контакта);\n";
            preout += "s"+sss.setNum(k)+".C1.y=      " + ss.setNum(sks_C1.y           *1000,'f',1) + " мм    \t- координаты нижней точки пневматика (ближайшей к точке контакта или центр пятна контакта);\n";
            preout += "s"+sss.setNum(k)+".C1.z=      " + ss.setNum(sks_C1.z           *1000,'f',1) + " мм    \t- координаты нижней точки пневматика (ближайшей к точке контакта или центр пятна контакта);\n";
            preout += "s"+sss.setNum(k)+".W2.x=      " + ss.setNum(sks_W2.x           *1000,'f',1) + " мм    \t- координаты центра колеса в разжатом состоянии;\n";
            preout += "s"+sss.setNum(k)+".W2.y=      " + ss.setNum(sks_W2.y           *1000,'f',1) + " мм    \t- координаты центра колеса в разжатом состоянии;\n";
            preout += "s"+sss.setNum(k)+".W2.z=      " + ss.setNum(sks_W2.z           *1000,'f',1) + " мм    \t- координаты центра колеса в разжатом состоянии;\n";
            preout += "s"+sss.setNum(k)+".D2.x=      " + ss.setNum(sks_D2.x           *1000,'f',1) + " мм    \t- координаты ближайшей к земле точки на ободе колеса (в плоскости симм.колеса) соотв.макс.допустимому обжатию пневматика;\n";
            preout += "s"+sss.setNum(k)+".D2.y=      " + ss.setNum(sks_D2.y           *1000,'f',1) + " мм    \t- координаты ближайшей к земле точки на ободе колеса (в плоскости симм.колеса) соотв.макс.допустимому обжатию пневматика;\n";
            preout += "s"+sss.setNum(k)+".D2.z=      " + ss.setNum(sks_D2.z           *1000,'f',1) + " мм    \t- координаты ближайшей к земле точки на ободе колеса (в плоскости симм.колеса) соотв.макс.допустимому обжатию пневматика;\n";
            preout += "s"+sss.setNum(k)+".C2.x=      " + ss.setNum(sks_C2.x           *1000,'f',1) + " мм    \t- координаты нижней точки пневматика (ближайшей к точке контакта или центр пятна контакта);\n";
            preout += "s"+sss.setNum(k)+".C2.y=      " + ss.setNum(sks_C2.y           *1000,'f',1) + " мм    \t- координаты нижней точки пневматика (ближайшей к точке контакта или центр пятна контакта);\n";
            preout += "s"+sss.setNum(k)+".C2.z=      " + ss.setNum(sks_C2.z           *1000,'f',1) + " мм    \t- координаты нижней точки пневматика (ближайшей к точке контакта или центр пятна контакта);\n";
            }
            if(st[k]->stype==2)
            {
            preout += "s"+sss.setNum(k)+".Q.x=       " + ss.setNum(sks_Q.x            *1000,'f',1) + " мм    \t- координаты крепления звена OQ;\n";
            preout += "s"+sss.setNum(k)+".Q.y=       " + ss.setNum(sks_Q.y            *1000,'f',1) + " мм    \t- координаты крепления звена OQ;\n";
            preout += "s"+sss.setNum(k)+".Q.z=       " + ss.setNum(sks_Q.z            *1000,'f',1) + " мм    \t- координаты крепления звена OQ;\n";
            preout += "s"+sss.setNum(k)+".OQ=        " + ss.setNum(st[k]->OQ          *1000,'f',1) + " мм\n";
            }
            if(st[k]->stype==8)
            {
            preout += "s"+sss.setNum(k)+".V.x=       " + ss.setNum(sks_V.x             *1000,'f',1) + " мм    \t- координаты центра колеса в разжатом состоянии;\n";
            preout += "s"+sss.setNum(k)+".V.y=       " + ss.setNum(sks_V.y             *1000,'f',1) + " мм    \t- координаты центра колеса в разжатом состоянии;\n";
            preout += "s"+sss.setNum(k)+".V.z=       " + ss.setNum(sks_V.z             *1000,'f',1) + " мм    \t- координаты центра колеса в разжатом состоянии;\n";
            preout += "s"+sss.setNum(k)+".OV=        " + ss.setNum(st[k]->OVxy         *1000,'f',1) + " мм    \t- длина теоретического стержня шасси;\n";
            preout += "s"+sss.setNum(k)+".BV=        " + ss.setNum(st[k]->BVxy         *1000,'f',1) + " мм    \t- длина теоретического стержня шасси;\n";
            preout += "s"+sss.setNum(k)+".VW=        " + ss.setNum(st[k]->VWxy         *1000,'f',1) + " мм    \t- длина теоретического стержня шасси;\n";
            preout += "s"+sss.setNum(k)+".BOV=       " + ss.setNum(st[k]->BOVxy      *180/pi,'f',4) + " грд   \t- угол между стержнями шасси OB и OV;\n";
            preout += "s"+sss.setNum(k)+".VOW=       " + ss.setNum(st[k]->VOWxy      *180/pi,'f',4) + " грд   \t- угол между стержнями шасси OV и OW;\n";
            }
            preout += "s"+sss.setNum(k)+".OA=        " + ss.setNum(st[k]->OA           *1000,'f',1) + " мм    \t- длина теоретического стержня шасси;\n";
            preout += "s"+sss.setNum(k)+".OAxy=      " + ss.setNum(st[k]->OAxy         *1000,'f',1) + " мм    \t- длина теоретического стержня шасси;\n";
            preout += "s"+sss.setNum(k)+".OAyz=      " + ss.setNum(st[k]->OAyz         *1000,'f',1) + " мм    \t- длина теоретического стержня шасси;\n";
            preout += "s"+sss.setNum(k)+".OB=        " + ss.setNum(st[k]->OB           *1000,'f',1) + " мм    \t- длина теоретического стержня шасси;\n";
            preout += "s"+sss.setNum(k)+".OBxy=      " + ss.setNum(st[k]->OBxy         *1000,'f',1) + " мм    \t- длина теоретического стержня шасси;\n";
            preout += "s"+sss.setNum(k)+".OByz=      " + ss.setNum(st[k]->OByz         *1000,'f',1) + " мм    \t- длина теоретического стержня шасси;\n";
            preout += "s"+sss.setNum(k)+".OW=        " + ss.setNum(st[k]->OW           *1000,'f',1) + " мм    \t- длина теоретического стержня шасси;\n";
            preout += "s"+sss.setNum(k)+".OWxy=      " + ss.setNum(st[k]->OWxy         *1000,'f',1) + " мм    \t- длина теоретического стержня шасси;\n";
            preout += "s"+sss.setNum(k)+".OWyz=      " + ss.setNum(st[k]->OWyz         *1000,'f',1) + " мм    \t- длина теоретического стержня шасси;\n";
            preout += "s"+sss.setNum(k)+".BW=        " + ss.setNum(st[k]->BW           *1000,'f',1) + " мм    \t- длина теоретического стержня шасси;\n";
            preout += "s"+sss.setNum(k)+".BWxy=      " + ss.setNum(st[k]->BWxy         *1000,'f',1) + " мм    \t- длина теоретического стержня шасси;\n";
            preout += "s"+sss.setNum(k)+".BWyz=      " + ss.setNum(st[k]->BWyz         *1000,'f',1) + " мм    \t- длина теоретического стержня шасси;\n";
            preout += "s"+sss.setNum(k)+".WD=        " + ss.setNum(st[k]->WD           *1000,'f',1) + " мм    \t- радиус колеса, соответствующий максимально допустимому обжатию пневматика;\n";
            preout += "s"+sss.setNum(k)+".ABfree=    " + ss.setNum(st[k]->AB_          *1000,'f',3) + " мм    \t- длина амортстойки между шарнирами в разжатом состоянии;\n";
            preout += "s"+sss.setNum(k)+".ABfull=    " + ss.setNum(st[k]->ABfull       *1000,'f',3) + " мм    \t- длина амортстойки между шарнирами в предельно сжатом состоянии;\n";
            preout += "s"+sss.setNum(k)+".Bfull.x=   " + ss.setNum(sks_Bfull.x         *1000,'f',1) + " мм    \t- координаты центра колеса при предельно сжатом амортизаторе;\n";
            preout += "s"+sss.setNum(k)+".Bfull.y=   " + ss.setNum(sks_Bfull.y         *1000,'f',1) + " мм    \t- координаты центра колеса при предельно сжатом амортизаторе;\n";
            preout += "s"+sss.setNum(k)+".Bfull.z=   " + ss.setNum(sks_Bfull.z         *1000,'f',1) + " мм    \t- координаты центра колеса при предельно сжатом амортизаторе;\n";
            preout += "s"+sss.setNum(k)+".Wfull.x=   " + ss.setNum(sks_Wfull.x         *1000,'f',1) + " мм    \t- координаты центра колеса при предельно сжатом амортизаторе;\n";
            preout += "s"+sss.setNum(k)+".Wfull.y=   " + ss.setNum(sks_Wfull.y         *1000,'f',1) + " мм    \t- координаты центра колеса при предельно сжатом амортизаторе;\n";
            preout += "s"+sss.setNum(k)+".Wfull.z=   " + ss.setNum(sks_Wfull.z         *1000,'f',1) + " мм    \t- координаты центра колеса при предельно сжатом амортизаторе;\n";
            preout += "s"+sss.setNum(k)+".angsz_full=" + ss.setNum(st[k]->ang_sz_full*180/pi,'f',4) + " грд   \t- угол между OW и строительной горизонталью для рычажной схемы;\n";
            preout += "s"+sss.setNum(k)+".angsx_full=" + ss.setNum(st[k]->ang_sx_full*180/pi,'f',4) + " грд   \t- угол между OW и строительной горизонталью для пирамидальной схемы;\n";
            preout += "s"+sss.setNum(k)+".ang_a_full=" + ss.setNum(st[k]->ang_a_full *180/pi,'f',4) + " грд   \t- угол между амортизатором и вертикалью (осью Y СКС);\n";
            preout += "s"+sss.setNum(k)+".angpn_full=" + ss.setNum(st[k]->ang_pn_full*180/pi,'f',4) + " грд   \t- угол между плоскостью колеса и плоскостью симметрии вертолета;\n";
            preout += "s"+sss.setNum(k)+".whd=       " + ss.setNum(st[k]->whd          *1000,'f',1) + " мм    \t- половина расстояния между колесами при nwh=2 (смещение колес относительно плоскости качания рычага стойки);\n";
            preout += "s"+sss.setNum(k)+".lambda=    " + ss.setNum(st[k]->lambda     *180/pi,'f',4) + " грд   \t- отклонение плоскости поворота нижнего рычага шасси от Y СКС\n";//угол наклона плоскости вилки относительно горизонтальной плоскости
            preout += "s"+sss.setNum(k)+".ang_wh=    " + ss.setNum(st[k]->ang_wh     *180/pi,'f',4) + " грд   \t- угол между осью колеса и линией OW\n";//
            preout += "s"+sss.setNum(k)+".BOWxy=     " + ss.setNum(st[k]->BOWxy      *180/pi,'f',4) + " грд   \t- угол между стержнями шасси OB и OW;\n";
            preout += "s"+sss.setNum(k)+".BOWyz=     " + ss.setNum(st[k]->BOWyz      *180/pi,'f',4) + " грд   \t- угол между стержнями шасси OB и OW;\n";
            preout += "s"+sss.setNum(k)+".ang_sx=    " + ss.setNum(st[k]->ang_sx     *180/pi,'f',4) + " грд   \t- угол между OW и строительной горизонталью для пирамидальной схемы;\n";
            preout += "s"+sss.setNum(k)+".ang_sz=    " + ss.setNum(st[k]->ang_sz     *180/pi,'f',4) + " грд   \t- угол между OW и строительной горизонталью для рычажной схемы;\n";
            preout += "s"+sss.setNum(k)+".ang_ax=    " + ss.setNum(st[k]->ang_ax     *180/pi,'f',4) + " грд   \t- угол между YZ-проекцией амортизатора и вертикалью (осью Y СКС);\n";
            preout += "s"+sss.setNum(k)+".ang_az=    " + ss.setNum(st[k]->ang_az     *180/pi,'f',4) + " грд   \t- угол между XY-проекцией амортизатора и вертикалью (осью Y СКС);\n";
            preout += "s"+sss.setNum(k)+".ang_a=     " + ss.setNum(st[k]->ang_a      *180/pi,'f',4) + " грд   \t- угол между амортизатором и вертикалью (осью Y СКС);\n";
            preout += "s"+sss.setNum(k)+".ang_pn=    " + ss.setNum(st[k]->ang_pn     *180/pi,'f',4) + " грд   \t- угол между плоскостью колеса и плоскостью симметрии вертолета;\n";

            preout += "s"+sss.setNum(k)+".p.dpmd=    " + ss.setNum(st[k]->pn->dpmd    *1000 ,'f',1) + " мм    \t- максимально допустимое обжатие;\n";
            preout += "s"+sss.setNum(k)+".p.cpx=     " + ss.setNum(st[k]->pn->cpx     *0.001,'f',1) + " Н/мм  \t- жесткость пневматика;\n";
            preout += "s"+sss.setNum(k)+".p.cpy=     " + ss.setNum(st[k]->pn->cpy     *0.001,'f',1) + " Н/мм  \t- жесткость пневматика;\n";
            preout += "s"+sss.setNum(k)+".p.cpz=     " + ss.setNum(st[k]->pn->cpz     *0.001,'f',1) + " Н/мм  \t- жесткость пневматика;\n";
            preout += "s"+sss.setNum(k)+".p.knp=     " + ss.setNum(st[k]->pn->alfanp        ,'f',3) + "       \t- коэффициент нелинейности диаграммы обжатия пневматика;\n";
            preout += "s"+sss.setNum(k)+".p.kdp=     " + ss.setNum(st[k]->pn->kdp           ,'f',3) + "       \t- коэффициент демпфирования пневматика;\n";
            preout += "s"+sss.setNum(k)+".p.Dp=      " + ss.setNum(st[k]->pn->Dp       *1000,'f',1) + " мм    \t- диаметр пневматика;\n";
            preout += "s"+sss.setNum(k)+".p.Bp=      " + ss.setNum(st[k]->pn->Bp       *1000,'f',1) + " мм    \t- ширина пневматика;\n";
            preout += "s"+sss.setNum(k)+".p.f0=      " + ss.setNum(st[k]->pn->f0            ,'f',3) + "       \t- коэф.трения покоя;\n";
            preout += "s"+sss.setNum(k)+".p.fs=      " + ss.setNum(st[k]->pn->fs            ,'f',3) + "       \t- коэф.трения скольжения;\n";
            preout += "s"+sss.setNum(k)+".p.fk=      " + ss.setNum(st[k]->pn->fk            ,'f',3) + "       \t- коэф.трения качения;\n";

            preout += "s"+sss.setNum(k)+".a.V0=      " + ss.setNum(st[k]->ab->V0       *1000,'f',3) + " л(дм3)\t- объем газовой камеры в разжатом состоянии;\n";
            preout += "s"+sss.setNum(k)+".a.c1.V0=   " + ss.setNum(st[k]->ab->c1.V0    *1000,'f',3) + " л(дм3)\t- объем газовой камеры в разжатом состоянии;\n";
            preout += "s"+sss.setNum(k)+".a.c2.V0=   " + ss.setNum(st[k]->ab->c2.V0    *1000,'f',3) + " л(дм3)\t- объем газовой камеры в разжатом состоянии;\n";
            preout += "s"+sss.setNum(k)+".a.p0=      " + ss.setNum(st[k]->ab->p0    /98066.5,'f',2) + " кгс/cм2\t- давление газа в разжатом состоянии;\n";
            preout += "s"+sss.setNum(k)+".a.c1.p0=   " + ss.setNum(st[k]->ab->c1.p0 /98066.5,'f',2) + " кгс/cм2\t- давление газа в разжатом состоянии;\n";
            preout += "s"+sss.setNum(k)+".a.c2.p0=   " + ss.setNum(st[k]->ab->c2.p0 /98066.5,'f',2) + " кгс/cм2\t- давление газа в разжатом состоянии;\n";
            preout += "s"+sss.setNum(k)+".a.ro=      " + ss.setNum(st[k]->ab->ro            ,'f',1) + " кг/м3\t- плотность жидкости;\n";
            preout += "s"+sss.setNum(k)+".a.mu=      " + ss.setNum(st[k]->ab->mu            ,'f',3) + "       \t- коэффициент истечения;\n";
            preout += "s"+sss.setNum(k)+".a.kpoly=   " + ss.setNum(st[k]->ab->kpoly         ,'f',3) + "       \t- показатель политропы при сжатии;\n";
            preout += "s"+sss.setNum(k)+".a.kpoly=   " + ss.setNum(st[k]->ab->kpolyr        ,'f',3) + "       \t- показатель политропы при разжатии;\n";
            preout += "s"+sss.setNum(k)+".a.S=       " + ss.setNum(st[k]->ab->S        *1000,'f',1) + " мм    \t- полный ход штоков амортизатора;\n";
            preout += "s"+sss.setNum(k)+".a.c1.s=    " + ss.setNum(st[k]->ab->c1.s     *1000,'f',1) + " мм    \t- полный ход штока №1;\n";
            preout += "s"+sss.setNum(k)+".a.c1.Dc=   " + ss.setNum(st[k]->ab->c1.Dc    *1000,'f',1) + " мм    \t- внутренний диаметр цилиндра (наружный диаметр буксы);\n";
            preout += "s"+sss.setNum(k)+".a.c1.Ds=   " + ss.setNum(st[k]->ab->c1.Ds    *1000,'f',1) + " мм    \t- наружный диаметр штока;\n";
            preout += "s"+sss.setNum(k)+".a.c1.Dv=   " + ss.setNum(st[k]->ab->c1.Dv    *1000,'f',1) + " мм    \t- внутренний диаметр штока (диаметр плунжера);\n";
            preout += "s"+sss.setNum(k)+".a.c1.fb=   " + ss.setNum(st[k]->ab->c1.fb *1000000,'f',1) + " мм2   \t- площадь отверстий в буксе;\n";
            preout += "s"+sss.setNum(k)+".a.c1.f=    " + ss.setNum(st[k]->ab->c1.f  *1000000,'f',1) + " мм2   \t- площадь отверстий в штоке(плунжере) на сжатии;\n";
            preout += "s"+sss.setNum(k)+".a.c1.fr=   " + ss.setNum(st[k]->ab->c1.fr *1000000,'f',1) + " мм2   \t- площадь отверстий в штоке(плунжере) на разжатии;\n";
            preout += "s"+sss.setNum(k)+".a.c1.ktr=  " + ss.setNum(st[k]->ab->c1.ktr        ,'f',3) + "       \t- коэффициент трения в буксах и уплотнениях;\n";
            if(st[k]->atype > 1)
            {
            preout += "s"+sss.setNum(k)+".a.c2.s=    " + ss.setNum(st[k]->ab->c2.s     *1000,'f',1) + " мм   \t- полный ход штока №2;\n";
            preout += "s"+sss.setNum(k)+".a.c2.Dc=   " + ss.setNum(st[k]->ab->c2.Dc    *1000,'f',1) + " мм   \t- внутренний диаметр цилиндра (наружный диаметр буксы);\n";
            preout += "s"+sss.setNum(k)+".a.c2.Ds=   " + ss.setNum(st[k]->ab->c2.Ds    *1000,'f',1) + " мм    \t- наружный диаметр штока;\n";
            preout += "s"+sss.setNum(k)+".a.c2.Dv=   " + ss.setNum(st[k]->ab->c2.Dv    *1000,'f',1) + " мм    \t- внутренний диаметр штока (диаметр плунжера);\n";
            preout += "s"+sss.setNum(k)+".a.c2.fb=   " + ss.setNum(st[k]->ab->c2.fb *1000000,'f',1) + " мм2   \t- площадь отверстий в буксе;\n";
            preout += "s"+sss.setNum(k)+".a.c2.f=    " + ss.setNum(st[k]->ab->c2.f  *1000000,'f',1) + " мм2  \t- площадь отверстий в штоке на сжатии;\n";
            preout += "s"+sss.setNum(k)+".a.c2.fr=   " + ss.setNum(st[k]->ab->c2.fr *1000000,'f',1) + " мм2  \t- площадь отверстий в штоке на разжатии;\n";
            preout += "s"+sss.setNum(k)+".a.c2.ktr=  " + ss.setNum(st[k]->ab->c2.ktr        ,'f',3) + "      \t- коэффициент трения в буксах и уплотнениях;\n";
            }
        }

        preout += "\nпосадочная поверхность:\n";
        for (int k=0; k<nst; k++)
        {
            preout += "ground.s"+sss.setNum(k)+"=    " + ss.setNum(st[k]->ground*1000,'f',1) + " мм    \t- отклонение действительной контактной поверхности от номинальной под стойкой 's#';\n";
        }

        preout += "\nгруз на внешней тросовой подвеске:\n";
        preout += "vtp.m=    " + ss.setNum(mcg            ,'f',1) + " кг \t\t- масса груза;\n";
        preout += "vtp.x=    " + ss.setNum(zacep.x   *1000,'f',1) + " мм\t\t- x-координата в СКС точки зацепления;\n";
        preout += "vtp.y=    " + ss.setNum(zacep.y   *1000,'f',1) + " мм\t\t- y-координата в СКС точки зацепления;\n";
        preout += "vtp.z=    " + ss.setNum(zacep.z   *1000,'f',1) + " мм\t\t- z-координата в СКС точки зацепления;\n";
        preout += "vtp.l=    " + ss.setNum(lcg       *1000,'f',1) + " мм\t\t- длина условного троса (до ЦТ груза);\n";

    }
}

int Helicopter::SolveFlying(outstructure &out)//МОДЕЛИРОВАНИЕ ДИНАМИКИ ВЕРТОЛЕТА НА ЗЕМЛЕ
{
    warning = 0;    logstr = "";    gofly = true;

    //ВИНТЫ: управляющие сигналы и стабилизация при необходимости
    if(trsolve)
    {
        fi0_tr = manual.fi0tr;
        if(tr->revcount<nstabrev)
             tr->zakrut = false;
        else tr->zakrut = true;
    }
    if(prsolve)
    {
        fi0_pr = manual.fi0pr;
    }
    if(mrsolve)
    {
        fi0_mr = manual.fi0mr;
        eta_ap = manual.etamr;
        chi_ap = manual.chimr;
        if(mr->revcount<nstabrev)//=3 стабилизация винтов
        {
            gofly = false;
            //параметры набегающего потока
            inFlow();
            //стабилизация НВ
            mr->zakrut = false;
            MainRotorForces();
            if(trsolve)
               TailRotorForces();//стабилизация PВ
            if(prsolve)
               PushRotorForces();//стабилизация TВ
        }
        else mr->zakrut = true;
    }

    //УРАВНЕНИЯ ДВИЖЕНИЯ ПЛАНЕРА
    if(gofly) SolveSystemNL6();
    if(warning) logstr += "Airframe: warning=" + ss.setNum(warning) + "\n";

    //работа стоек шасси
    for (int k=0; k<nst; k++)
    {
        st[k]->Aa  += st[k]->ab->dA;
        st[k]->Ap  += st[k]->pn->dA;
        st[k]->Aptr+= st[k]->pn->dAtr;
        if(st[k]->warning)     logstr +=    "Shassi: warning=" + ss.setNum(st[k]->warning)     + "\n";
        if(st[k]->ab->warning) logstr +=  "Absorber: warning=" + ss.setNum(st[k]->ab->warning) + "\n";
        if(st[k]->pn->warning) logstr += "Pneumatic: warning=" + ss.setNum(st[k]->pn->warning) + "\n";
    }

    //вывод результатов в структуру текущей итерации
    fillOut(out);
    if(mrsolve)
    {
        mr->getAverage();//обработка параметров за предыдущий полный оборот
        mr->fillOut(out);
        if(mr->warning) logstr += "MainRotor: warning=" + ss.setNum(mr->warning) + "\n";
    }
    if(trsolve)
    {
        tr->getAverage();//обработка параметров за предыдущий полный оборот
        tr->fillOut(out);
        if(tr->warning) logstr += "TailRotor: warning=" + ss.setNum(tr->warning) + "\n";
    }
    if(prsolve)
    {
        pr->getAverage();//обработка параметров за предыдущий полный оборот
        pr->fillOut(out);
        if(pr->warning) logstr += "PushRotor: warning=" + ss.setNum(pr->warning) + "\n";
    }

return flyerror;
}

int Helicopter::FixFlying(outstructure &out)//
{
    warning = 0;    logstr = "";    gofly = true;
    nst = 0;

    //ВИНТЫ: управляющие сигналы и стабилизация при необходимости
    if(trsolve)
    {
        fi0_tr = manual.fi0tr;
        if(tr->revcount<nstabrev)
             tr->zakrut = false;
        else tr->zakrut = true;
    }
    if(prsolve)
    {
        fi0_pr = manual.fi0pr;
    }
    if(mrsolve)
    {
        fi0_mr = manual.fi0mr;
        eta_ap = manual.etamr;
        chi_ap = manual.chimr;
        if(mr->revcount<nstabrev)//=3 стабилизация винтов
        {
        //    gofly = false;
            //параметры набегающего потока
            //inFlow();
            mr->zakrut = false;


        //    //стабилизация НВ
        //    MainRotorForces();
        //    if(trsolve)
        //       TailRotorForces();//стабилизация PВ
        //    if(prsolve)
        //       PushRotorForces();//стабилизация TВ
        }
        else mr->zakrut = true;
    }

    //ДВИЖЕНИЕ ПЛАНЕРА
    HelicopterKinematic(v.x, v.y, v.z, w.x, w.y, w.z);
    //пересчет сил и моментов       //ПОРЯДОК ВЫЗОВА НЕ МЕНЯТЬ!
    MainRotorForces();              //СИЛЫ/МОМЕНТЫ ОТ ВТУЛКИ НЕСУЩЕГО ВИНТА
    TailRotorForces();              //СИЛЫ/МОМЕНТЫ ОТ ВТУЛКИ РУЛЕВОГО ВИНТА
    PushRotorForces();              //СИЛЫ/МОМЕНТЫ ОТ толкающего винта
    alfa0f = alfa_mr - eps_fz - chi_hub;//угол атаки фюзеляжа без скоса от НВ
    PlanerAirDynForces();           //СИЛЫ/МОМЕНТЫ ОТ АЭРОДИНАМИЧЕСКИХ СИЛ НА ФЮЗЕЛЯЖЕ
    HelicopterGravity(ocw.t, ocw.k);//СИЛЫ/МОМЕНТЫ ОТ ВЕСА ВЕРТОЛЕТА
    OutboardCargo(ocw.t, ocw.k);    //СИЛЫ/МОМЕНТЫ ОТ ВЕСА ГРУЗА НА ВНЕШНЕЙ ТРОСОВОЙ ПОДВЕСКЕ
    //ShassiForces();                 //СИЛЫ/МОМЕНТЫ ОТ СТОЕК ШАССИ (точек контакта с землей)
    ExternalLoads(t);               //СИЛА/МОМЕНТ  ОТ ВНЕШНЕГО ВОЗДЕЙСТВИЯ
    //суммирование сил и моментов
    Fx = FMhg.Fx + FMcg.Fx + FMsh.Fx + FMmr.Fx + FMtr.Fx + FMpr.Fx + FMadp.Fx + FM1.Fx + FM2.Fx;
    Fy = FMhg.Fy + FMcg.Fy + FMsh.Fy + FMmr.Fy + FMtr.Fy + FMpr.Fy + FMadp.Fy + FM1.Fy + FM2.Fy;
    Fz = FMhg.Fz + FMcg.Fz + FMsh.Fz + FMmr.Fz + FMtr.Fz + FMpr.Fz + FMadp.Fz + FM1.Fz + FM2.Fz;
    Mx = FMhg.Mx + FMcg.Mx + FMsh.Mx + FMmr.Mx + FMtr.Mx + FMpr.Mx + FMadp.Mx + FM1.Mx + FM2.Mx;
    My = FMhg.My + FMcg.My + FMsh.My + FMmr.My + FMtr.My + FMpr.My + FMadp.My + FM1.My + FM2.My;
    Mz = FMhg.Mz + FMcg.Mz + FMsh.Mz + FMmr.Mz + FMtr.Mz + FMpr.Mz + FMadp.Mz + FM1.Mz + FM2.Mz;


    //работа стоек шасси
    //for (int k=0; k<nst; k++)
    //{
    //    st[k]->Aa  = 0;
    //    st[k]->Ap  = 0;
    //    st[k]->Aptr= 0;
    //    if(st[k]->warning)     logstr +=    "Shassi: warning=" + ss.setNum(st[k]->warning)     + "\n";
    //    if(st[k]->ab->warning) logstr +=  "Absorber: warning=" + ss.setNum(st[k]->ab->warning) + "\n";
    //    if(st[k]->pn->warning) logstr += "Pneumatic: warning=" + ss.setNum(st[k]->pn->warning) + "\n";
    //}
    if(warning) logstr += "Airframe: warning=" + ss.setNum(warning) + "\n";
    //вывод результатов в структуру текущей итерации
    fillOut(out);
    if(mrsolve)
    {
        mr->getAverage();//обработка параметров за предыдущий полный оборот
        mr->fillOut(out);
        if(mr->warning) logstr += "MainRotor: warning=" + ss.setNum(mr->warning) + "\n";
    }
    if(trsolve)
    {
        tr->getAverage();//обработка параметров за предыдущий полный оборот
        tr->fillOut(out);
        if(tr->warning) logstr += "TailRotor: warning=" + ss.setNum(tr->warning) + "\n";
    }
    if(prsolve)
    {
        pr->getAverage();//обработка параметров за предыдущий полный оборот
        pr->fillOut(out);
        if(pr->warning) logstr += "PushRotor: warning=" + ss.setNum(pr->warning) + "\n";
    }

    //ограничение перемещений
    //if(fix_org_x) {  org.x = org_.x;   ocw.x = ocw_.x;   v.x = v_.x;  }
    //if(fix_org_y) {  org.y = org_.y;   ocw.y = ocw_.y;   v.y = v_.y;  }
    //if(fix_org_z) {  org.z = org_.z;   ocw.z = ocw_.z;   v.z = v_.z;  }
    //if(fix_org_k) {  org.k = org_.k;   ocw.k = ocw_.k;   w.x = w_.x;  }
    //if(fix_org_r) {  org.r = org_.r;   ocw.r = ocw_.r;   w.y = w_.y;  }
    //if(fix_org_t) {  org.t = org_.t;   ocw.t = ocw_.t;   w.z = w_.z;  }

return flyerror;
}

void Helicopter::HelicopterDynamic(double x0, double x1, double x2, double x3, double x4, double x5)//пересчет усилий для заданных скоростей и заданного положения
{                                 //v.x = x0;  v.y = x1;  v.z = x2;  w.x = x3;  w.y = x4;  w.z = x5;
    Fx=0;  Fy=0;  Fz=0;  Mx=0;  My=0;  Mz=0;
    //пересчет углов Эйлера, ускорения, скорости и положения СКС в ЗемнойСК
    HelicopterKinematic(x0, x1, x2, x3, x4, x5);
    //пересчет сил и моментов       //ПОРЯДОК ВЫЗОВА НЕ МЕНЯТЬ!
    MainRotorForces();              //СИЛЫ/МОМЕНТЫ ОТ ВТУЛКИ НЕСУЩЕГО ВИНТА
    TailRotorForces();              //СИЛЫ/МОМЕНТЫ ОТ ВТУЛКИ РУЛЕВОГО ВИНТА
    PushRotorForces();              //СИЛЫ/МОМЕНТЫ ОТ толкающего винта
    alfa0f = alfa_mr - eps_fz - chi_hub;//угол атаки фюзеляжа без скоса от НВ
    PlanerAirDynForces();           //СИЛЫ/МОМЕНТЫ ОТ АЭРОДИНАМИЧЕСКИХ СИЛ НА ФЮЗЕЛЯЖЕ
    HelicopterGravity(ocw.t, ocw.k);//СИЛЫ/МОМЕНТЫ ОТ ВЕСА ВЕРТОЛЕТА
    OutboardCargo(ocw.t, ocw.k);    //СИЛЫ/МОМЕНТЫ ОТ ВЕСА ГРУЗА НА ВНЕШНЕЙ ТРОСОВОЙ ПОДВЕСКЕ
    ShassiForces();                 //СИЛЫ/МОМЕНТЫ ОТ СТОЕК ШАССИ (точек контакта с землей)
    ExternalLoads(t);               //СИЛА/МОМЕНТ  ОТ ВНЕШНЕГО ВОЗДЕЙСТВИЯ
    //суммирование сил и моментов
    Fx = FMhg.Fx + FMcg.Fx + FMsh.Fx + FMmr.Fx + FMtr.Fx + FMpr.Fx + FMadp.Fx + FM1.Fx + FM2.Fx;
    Fy = FMhg.Fy + FMcg.Fy + FMsh.Fy + FMmr.Fy + FMtr.Fy + FMpr.Fy + FMadp.Fy + FM1.Fy + FM2.Fy;
    Fz = FMhg.Fz + FMcg.Fz + FMsh.Fz + FMmr.Fz + FMtr.Fz + FMpr.Fz + FMadp.Fz + FM1.Fz + FM2.Fz;
    Mx = FMhg.Mx + FMcg.Mx + FMsh.Mx + FMmr.Mx + FMtr.Mx + FMpr.Mx + FMadp.Mx + FM1.Mx + FM2.Mx;
    My = FMhg.My + FMcg.My + FMsh.My + FMmr.My + FMtr.My + FMpr.My + FMadp.My + FM1.My + FM2.My;
    Mz = FMhg.Mz + FMcg.Mz + FMsh.Mz + FMmr.Mz + FMtr.Mz + FMpr.Mz + FMadp.Mz + FM1.Mz + FM2.Mz;
}

void Helicopter::HelicopterKinematic(double x0,double x1,double x2,double x3,double x4,double x5)//пересчет кинематических параметров для заданных скоростей
{//Браверман,с.73-75
    //(квази-)скорости ЦТ вертолета в СКС
    v.x = x0;                   w.x = x3;
    v.y = x1;                   w.y = x4;
    v.z = x2;                   w.z = x5;

    //(квази-)ускорение ЦТ вертолета в СКС
    a.x = (v.x - v_.x) /dt;     e.x = (w.x - w_.x) /dt;
    a.y = (v.y - v_.y) /dt;     e.y = (w.y - w_.y) /dt;
    a.z = (v.z - v_.z) /dt;     e.z = (w.z - w_.z) /dt;

    ///положение осей СКС относительно СКЗ
    duo seq2 = SolveSystemNL2();//решение системы нелинейных уравнений
    //угол крена (ориетация СКС в СКЗ)
    ocw.k = seq2.a;
    //угол тангажа (ориетация СКС в СКЗ)
    ocw.t = seq2.b;
    //скорость изменения угла рыскания в нормальной СК
    vn.r = ( w.y*cos(ocw.k) - w.z*sin(ocw.k) ) / cos(ocw.t);
    //угол рыскания (ориетация СКС в СКЗ)
    ocw.r = ocw_.r + vn.r*dt;

    //ускорение ЦТ вертолета в Земной (нормальной) СК
    an = SKStoSKN(a);
    en = SKStoSKN(e);
    //линейная скорость ЦТ вертолета в Земной (нормальной) СК
    xyz vnorm = SKStoSKN(v);
    vn.x = vnorm.x;
    vn.y = vnorm.y;
    vn.z = vnorm.z;
    //новое положение ЦТ вертолета в Земной (нормальной) СК
    ocw.x = ocw_.x + vn.x*dt;
    ocw.y = ocw_.y + vn.y*dt;
    ocw.z = ocw_.z + vn.z*dt;
    //положение начала СКС
    org = ocw2org(ocw);

    //параметры набегающего потока
    inFlow();// Vi, tetra, beta_
}

void Helicopter::MainRotorForces()//СИЛЫ/МОМЕНТЫ ОТ НЕСУЩЕГО ВИНТА
{
    if(mrsolve)
    {
        //вектор тяги винта, для определения параметров набегающего потока
        Fx_mr = mr->H_sr;
        Fy_mr = mr->T_sr;
        Fz_mr = mr->S_sr;
        if(mrtilt)
            mr->RotorIterate( Fy_mr,Fx_mr,Fz_mr, Vi, alfa_mr, beta_sk, fi0_mr, 0, 0 );
        else
            mr->RotorIterate( Fy_mr,Fx_mr,Fz_mr, Vi, alfa_mr, beta_sk, fi0_mr, eta_ap, chi_ap );
        //усилия на втулке НВ в полусвязанной СК (от всех сил)
        Fx_mr = mr->Fhx_sum;
        Fy_mr = mr->Fhy_sum;
        Fz_mr = mr->Fhz_sum;
        Mx_mr = mr->Mhx_sum;
        My_mr = mr->Mhy_sum;
        Mz_mr = mr->Mhz_sum;
    }
    else
    {
        //приращение продольной силы НВ за счет дополнительного отклонения автомата перекоса
        //dH_mr = T_mr_*(D1_mr*dchi-D2_mr*deta);
        //приращение поперечной силы НВ за счет дополнительного отклонения автомата перекоса
        //dS_mr = T_mr_*(D1_mr*deta+D2_mr*dchi);
        Fx_mr = Fx_mr_;
        Fy_mr = Fy_mr_;
        Fz_mr = Fz_mr_;
        Mx_mr = Mx_mr_;// + zswlmr*(D1_mr*deta+D2_mr*dchi) + 0.5*My_mr_*(D1_mr*dchi-D2_mr*deta);
        My_mr = My_mr_;
        Mz_mr = Mz_mr_;// - zswlmr*(D1_mr*dchi-D2_mr*deta) - 0.5*My_mr_*(D1_mr*deta+D2_mr*dchi);///20200316 изменил знак у My_mr_
    }
    Ttr_need = -My_mr/Lctr;
    //пересчет усилий на втулке из СКHB в СКС (привел к ЦТ)
    SKMRtoSKS();
}
void Helicopter::MainRotorForceCorrection(double dchi, double deta)//СИЛЫ/МОМЕНТЫ ОТ НЕСУЩЕГО ВИНТА
{
    if(mrtilt)
    {
        chi_hub = dchi;
        eta_hub = deta;
        Fx_mr = Fx_mr_;
        Fy_mr = Fy_mr_;
        Fz_mr = Fz_mr_;
        Mx_mr = Mx_mr_;
        My_mr = My_mr_;
        Mz_mr = Mz_mr_;
    }
    else
    {
        //приращение продольной силы НВ за счет дополнительного отклонения автомата перекоса
        dH_mr = Fy_mr_*(D1_mr*dchi-D2_mr*deta);
        //приращение поперечной силы НВ за счет дополнительного отклонения автомата перекоса
        dS_mr = Fy_mr_*(D1_mr*deta+D2_mr*dchi);
        //скорректированные значения усилий на втулке
        Fx_mr = Fx_mr_ + dH_mr;
        Fy_mr = Fy_mr_;
        Fz_mr = Fz_mr_ + dS_mr;
        Mx_mr = Mx_mr_ + zswlmr*(D1_mr*deta+D2_mr*dchi) + 0.5*My_mr_*(D1_mr*dchi-D2_mr*deta);
        My_mr = My_mr_;
        Mz_mr = Mz_mr_ - zswlmr*(D1_mr*dchi-D2_mr*deta) - 0.5*My_mr_*(D1_mr*deta+D2_mr*dchi);
    }
    //пересчет усилий на втулке из СКHB в СКС (привел к ЦТ)
    SKMRtoSKS();
}
void Helicopter::SKMRtoSKS()//из наклоненной втулки НВ  в связанную СК
{
    double _t_ =  chi_hub + eps_fz;
    double _k_ = -eta_hub + eps_fx;//<< положительное направление крена не совпадает с полож.напр.поперечного наклона АП
    double _r_ =  0;//= eps_y

    xyz F, M;
    F.x = -Fx_mr;    M.x = -Mx_mr;
    F.y =  Fy_mr;    M.y =  My_mr;
    F.z = -Fz_mr;    M.z = -Mz_mr;

    double a11 = cos(_r_)*cos(_t_);
    double a21 = sin(_k_)*sin(_r_) - cos(_k_)*cos(_r_)*sin(_t_);
    double a31 = cos(_k_)*sin(_r_) + sin(_k_)*cos(_r_)*sin(_t_);
    double a12 = sin(_t_);
    double a22 = cos(_t_)*cos(_k_);
    double a32 =-cos(_t_)*sin(_k_);
    double a13 =-sin(_r_)*cos(_t_);
    double a23 = sin(_k_)*cos(_r_) + cos(_k_)*sin(_r_)*sin(_t_);
    double a33 = cos(_k_)*cos(_r_) - sin(_k_)*sin(_r_)*sin(_t_);

    FMmr.Fx = a11*F.x + a21*F.y + a31*F.z;
    FMmr.Fy = a12*F.x + a22*F.y + a32*F.z;
    FMmr.Fz = a13*F.x + a23*F.y + a33*F.z;
    FMmr.Mx = a11*M.x + a21*M.y + a31*M.z  - FMmr.Fy*(z_mr-zct) + FMmr.Fz*(y_mr-yct);
    FMmr.My = a12*M.x + a22*M.y + a32*M.z  + FMmr.Fx*(z_mr-zct) - FMmr.Fz*(x_mr-xct);
    FMmr.Mz = a13*M.x + a23*M.y + a33*M.z  - FMmr.Fx*(y_mr-yct) + FMmr.Fy*(x_mr-xct);
}

void Helicopter::TailRotorForces()//СИЛЫ/МОМЕНТЫ ОТ РУЛЕВОГО ВИНТА
{
    if(trsolve)
    {
        //вектор тяги винта, для определения параметров набегающего потока
        Fx_tr = tr->H_sr;
        Fy_tr = tr->T_sr;
        Fz_tr = tr->S_sr;
        if(Vi<10) { Vi=10; Fy_tr = 0.5*(Ttr_need +Fy_tr); }//20210329
        tr->RotorIterate( Fy_tr,Fx_tr,Fz_tr, Vi, alfa_tr, 0, fi0_tr, 0, 0 );
        //усилия на втулке НВ в полусвязанной СК (от всех сил)
        Fx_tr = tr->Fhx_sum;
        Fy_tr = tr->Fhy_sum;
        Fz_tr = tr->Fhz_sum;
        Mx_tr = tr->Mhx_sum;
        My_tr = tr->Mhy_sum;
        Mz_tr = tr->Mhz_sum;
    }
    else
    {
        Fx_tr = Fx_tr_;    Fy_tr = Fy_tr_;    Fz_tr = Fz_tr_;
        Mx_tr = Mx_tr_;    My_tr = My_tr_;    Mz_tr = Mz_tr_;
    }
    //
    FMtr.Fx = -Fx_tr;
    FMtr.Fy =  Fz_tr;
    FMtr.Fz =  Fy_tr;
    FMtr.Mx =  Fy_tr*(y_tr-yct) -Fz_tr*(z_tr-zct) -Mx_tr;//исправлено, было -S_tr*y_tr
    FMtr.My = -Fy_tr*(x_tr-xct) -Fx_tr*(z_tr-zct) +Mz_tr;
    FMtr.Mz =  Fz_tr*(x_tr-xct) +Fx_tr*(y_tr-yct) +My_tr;
}
void Helicopter::TailRotorForceCorrection(double T_tr_)//СИЛЫ/МОМЕНТЫ ОТ РУЛЕВОГО ВИНТА
{
    Fx_tr = Fx_tr_;    Fy_tr =  T_tr_;    Fz_tr = Fz_tr_;
    Mx_tr = Mx_tr_;    My_tr = My_tr_;    Mz_tr = Mz_tr_;
    //
    FMtr.Fx = -Fx_tr;
    FMtr.Fy =  Fz_tr;
    FMtr.Fz =  Fy_tr;
    FMtr.Mx =  Fy_tr*(y_tr-yct) -Fz_tr*(z_tr-zct) -Mx_tr;//исправлено, было -S_tr*y_tr
    FMtr.My = -Fy_tr*(x_tr-xct) -Fx_tr*(z_tr-zct) +Mz_tr;
    FMtr.Mz =  Fz_tr*(x_tr-xct) +Fx_tr*(y_tr-yct) +My_tr;
}

void Helicopter::PushRotorForces()//СИЛЫ/МОМЕНТЫ ОТ толкающего винта
{
    if(prsolve)
    {
        //вектор тяги винта, для определения параметров набегающего потока
        Fx_pr = pr->H_sr;
        Fy_pr = pr->T_sr;
        Fz_pr = pr->S_sr;
        pr->RotorIterate( Fy_pr,Fx_pr,Fz_pr, Vi, alfa_pr, 0, fi0_pr, 0, 0 );
        //усилия на втулке TВ в СКTB
        Fx_pr = pr->Fhx_sum;
        Fy_pr = pr->Fhy_sum;
        Fz_pr = pr->Fhz_sum;
        Mx_pr = pr->Mhx_sum;
        My_pr = pr->Mhy_sum;
        Mz_pr = pr->Mhz_sum;
    }
    else
    {
        Fx_pr = Fx_pr_;    Fy_pr = Fy_pr_;    Fz_pr = Fz_pr_;
        Mx_pr = Mx_pr_;    My_pr = My_pr_;    Mz_pr = Mz_pr_;
    }
    //приведение к ЦТ вертолета
    FMpr.Fx = Fy_pr;
    FMpr.Fy = Fz_pr;
    FMpr.Fz = Fx_pr;
    FMpr.Mx = My_pr -Fz_pr*(z_pr-zct) +Fx_pr*(y_pr-yct);
    FMpr.My = Mz_pr +Fy_pr*(z_pr-zct) -Fx_pr*(x_pr-xct);
    FMpr.Mz = Mx_pr -Fy_pr*(y_pr-yct) +Fz_pr*(x_pr-xct);
}
void Helicopter::PushRotorForceCorrection(double T_pr_)//СИЛЫ/МОМЕНТЫ ОТ толкающего винта
{
    Fx_pr = Fx_pr_;    Fy_pr = T_pr_;     Fz_pr = Fz_pr_;
    Mx_pr = Mx_pr_;    My_pr = My_pr_;    Mz_pr = Mz_pr_;
    //приведение к ЦТ вертолета
    FMpr.Fx = Fy_pr;
    FMpr.Fy = Fz_pr;
    FMpr.Fz = Fx_pr;
    FMpr.Mx = My_pr -Fz_pr*(z_pr-zct) +Fx_pr*(y_pr-yct);
    FMpr.My = Mz_pr +Fy_pr*(z_pr-zct) -Fx_pr*(x_pr-xct);
    FMpr.Mz = Mx_pr -Fy_pr*(y_pr-yct) +Fz_pr*(x_pr-xct);
}

void Helicopter::ShassiForces()//СИЛЫ/МОМЕНТЫ ОТ ШАССИ
{
    //определение кинематических и динамических параметров стоек шасси
    for (int k=0; k<nst; k++)
    {
        st[k]->orient = org;
        st[k]->ShassiSolver();
        //Силы и моменты в СКС
        FMw[k].Fx  = st[k]->Pp_s.x ;
        FMw[k].Fy  = st[k]->Pp_s.y ;
        FMw[k].Fz  = st[k]->Pp_s.z ;
        FMw[k].Mx  = st[k]->Pp_s.z *(st[k]->C_sks.y -yct) - st[k]->Pp_s.y *(st[k]->C_sks.z -zct);
        FMw[k].My  = st[k]->Pp_s.x *(st[k]->C_sks.z -zct) - st[k]->Pp_s.z *(st[k]->C_sks.x -xct);
        FMw[k].Mz  = st[k]->Pp_s.y *(st[k]->C_sks.x -xct) - st[k]->Pp_s.x *(st[k]->C_sks.y -yct);
        //для двух колес на стойке (только для визуализации, в расчете не используется)
        if (st[k]->nwh==2)
        {
            FMw1[k].Fx = st[k]->Pp1s.x;
            FMw1[k].Fy = st[k]->Pp1s.y;
            FMw1[k].Fz = st[k]->Pp1s.z;
            FMw2[k].Fx = st[k]->Pp2s.x;
            FMw2[k].Fy = st[k]->Pp2s.y;
            FMw2[k].Fz = st[k]->Pp2s.z;
        }
        else
        {
            FMw1[k].Fx = 0;
            FMw1[k].Fy = 0;
            FMw1[k].Fz = 0;
            FMw2[k].Fx = 0;
            FMw2[k].Fy = 0;
            FMw2[k].Fz = 0;
        }
    }
    //суммирование сил и моментов от стоек в СКС
    FMsh.Fx = 0;
    FMsh.Fy = 0;
    FMsh.Fz = 0;
    FMsh.Mx = 0;
    FMsh.My = 0;
    FMsh.Mz = 0;
    for(int k=0; k<nst; k++)
    {
        FMsh.Fx += FMw[k].Fx;
        FMsh.Fy += FMw[k].Fy;
        FMsh.Fz += FMw[k].Fz;
        FMsh.Mx += FMw[k].Mx;
        FMsh.My += FMw[k].My;
        FMsh.Mz += FMw[k].Mz;
    }
}

void Helicopter::HelicopterGravity(double tang, double kren)//СИЛА / МОМЕНТ ОТ ВЕСА ВЕРТОЛЕТА
{
    //поправка к моменту веса на вынос центра тяжести от начала СКС
    //float Fix = mh*a.x;
    //float Fiy = mh*a.y;
    //float Fiz = mh*a.z;
    //float Mix = 0;//    Fiz*y_ct -Fiy*z_ct;
    //float Miy = 0;//    Fix*z_ct -Fiz*x_ct;
    //float Miz = 0;//    Fiy*x_ct -Fix*y_ct;

    FMhg.Fx = -Gh*sin(tang);           //(Gh-Gl)
    FMhg.Fy = -Gh*cos(tang)*cos(kren); //(Gh-Gl)
    FMhg.Fz =  Gh*cos(tang)*sin(kren); //(Gh-Gl)
    FMhg.Mx = 0;// Gh*(z_ct -zct)*cos(tang)*cos(kren) +Gh*(y_ct -yct)*cos(tang)*sin(kren) -Mix;//
    FMhg.My = 0;//-Gh*(x_ct -xct)*cos(tang)*sin(kren) -Gh*(z_ct -zct)*sin(tang)           -Miy;//
    FMhg.Mz = 0;//-Gh*(x_ct -xct)*cos(tang)*cos(kren) +Gh*(y_ct -yct)*sin(tang)   +0.01   -Miz;//+0.01 только для сходимости SolveSystemNL6();
}

void Helicopter::OutboardCargo(double tang, double kren)//СИЛЫ/МОМЕНТЫ ОТ ВЕСА ГРУЗА НА ВНЕШНЕЙ ТРОСОВОЙ ПОДВЕСКЕ
{
    //вес груза с учетом перегрузки в цт вертолета
    Gcg = mcg*g;// *(1+ Fy/Gh);

    //угловое отклонение в продольной плоскости от инерции
    double dcgtng = 0;
    //угловое отклонение в поперечной плоскости от инерции
    double dcgkrn = 0;
    //результирующие углы линии троса в СКС
    double cgtng = tang + dcgtng;
    double cgkrn = kren + dcgkrn;

    //проекции троса в земной СК
    xyz normcargoct;
    normcargoct.x =    0; //-lcg*sin(dcgtng);//(положительный тангаж - груз отходит назад "-Х")
    normcargoct.y = -lcg; //-lcg*cos(dcgtng)*cos(dcgkrn);//немного уменьшается проекция троса на ось Y СКС
    normcargoct.z =    0; //+lcg*sin(dcgkrn);//(положительный крен - груз отходит в сторону правого борта: "+Z")
    //проекции троса в СКС
    crgct = SKNtoSKS(normcargoct);
    //координаты ЦТ груза в СКС
    crgct.x = zacep.x + crgct.x;
    crgct.y = zacep.y + crgct.y;
    crgct.z = zacep.z + crgct.z;

    //проекции силы в точке зацепления троса (в земной СК)
    Gcrg.x  = -Gcg*sin(dcgtng);
    Gcrg.y  = -Gcg*cos(dcgtng)*cos(dcgkrn);
    Gcrg.z  =  Gcg*sin(dcgkrn);
    //проекции силы в точке зацепления троса (в СКС)
    Gcrg = SKNtoSKS(Gcrg);
    //СИЛЫ/МОМЕНТЫ ОТ ВЕСА ГРУЗА в СКС
    FMcg.Fx = -Gcg*sin(cgtng);
    FMcg.Fy = -Gcg*cos(cgtng)*cos(cgkrn);
    FMcg.Fz =  Gcg*cos(cgtng)*sin(cgkrn);
    FMcg.Mx =  Gcg*(zacep.z-zct)*cos(cgtng)*cos(cgkrn) +Gcg*(zacep.y-yct)*cos(cgtng)*sin(cgkrn);
    FMcg.My = -Gcg*(zacep.x-xct)*cos(cgtng)*sin(cgkrn) -Gcg*(zacep.z-zct)*sin(cgtng);
    FMcg.Mz = -Gcg*(zacep.x-xct)*cos(cgtng)*cos(cgkrn) +Gcg*(zacep.y-yct)*sin(cgtng);
}

void Helicopter::ExternalLoads(float t)//СИЛА/МОМЕНТ  ОТ ВНЕШНЕГО ВОЗДЕЙСТВИЯ
{
    extloadstruct a,b;
    float  Fx, Fy, Fz;
    FM1.Fx = 0;   FM1.Mx = 0;
    FM1.Fy = 0;   FM1.My = 0;
    FM1.Fz = 0;   FM1.Mz = 0;
    FM2.Fx = 0;   FM2.Mx = 0;
    FM2.Fy = 0;   FM2.My = 0;
    FM2.Fz = 0;   FM2.Mz = 0;

    if(load1.size()>1)//extload1
    {
        Fx=0; Fy=0; Fz=0;
        for(int j=0; j<load1.size()-1; j++)
        {
            a = load1.at(j);
            b = load1.at(j+1);
            if((a.t<=t)&&(t<b.t))
            {//по пропорции вычисляем усилие в нужный момент времени
                Fx = inter(t, a.t, b.t, a.Fx, b.Fx);
                Fy = inter(t, a.t, b.t, a.Fy, b.Fy);
                Fz = inter(t, a.t, b.t, a.Fz, b.Fz);
                //Сила-момент в СКС
                FM1.Fx  = Fx;
                FM1.Fy  = Fy;
                FM1.Fz  = Fz;
                FM1.Mx  = Fz *load1pos.y - Fy *load1pos.z;
                FM1.My  = Fx *load1pos.z - Fz *load1pos.x;
                FM1.Mz  = Fy *load1pos.x - Fx *load1pos.y;
            }
        }
    }
    if(load2.size()>1)//extload2
    {
        Fx=0; Fy=0; Fz=0;
        for(int j=0; j<load2.size()-1; j++)
        {
            a = load2.at(j);
            b = load2.at(j+1);
            if((a.t<=t)&&(t<b.t))
            {//по пропорции вычисляем усилие в нужный момент времени
                Fx = inter(t, a.t, b.t, a.Fx, b.Fx);
                Fy = inter(t, a.t, b.t, a.Fy, b.Fy);
                Fz = inter(t, a.t, b.t, a.Fz, b.Fz);
                //Силы и моменты в СКС
                FM2.Fx = Fx;
                FM2.Fy = Fy;
                FM2.Fz = Fz;
                FM2.Mx = Fz *load2pos.y - Fy *load2pos.z;
                FM2.My = Fx *load2pos.z - Fz *load2pos.x;
                FM2.Mz = Fy *load2pos.x - Fx *load2pos.y;
            }
        }
    }
}

//уравнения кинематики полета
void Helicopter::RecalcAngles(double x0,double x1)
{
    ocw.k = x0;    ocw.t = x1;    //ocw.r = 0;
    //угл.скорость поворота  в нормальной СК
    vn.k = (ocw.k - ocw_.k) /dt;
    vn.t = (ocw.t - ocw_.t) /dt;
    //vn.r = (ocw.r - ocw_.r) /dt;
}
double Helicopter::f_dkren()//
{//Браверман,с.75
    //return vn.k - w.x + tan(orient.t) * ( w.y*cos(orient.k) - w.z*sin(orient.k) );
    return vn.k - w.x + tan(ocw.t) * ( w.y*cos(ocw.k) - w.z*sin(ocw.k) );
}
double Helicopter::f_drysk()// не используется в системе NL2
{//Браверман,с.75
    //return vn.r - ( w.y*cos(orient.k) - w.z*sin(orient.k) ) / cos(orient.t);
    return vn.r - ( w.y*cos(ocw.k) - w.z*sin(ocw.k) ) / cos(ocw.t);
}
double Helicopter::f_dtang()//
{//Браверман,с.75
    //return vn.t - w.y*sin(orient.k) - w.z*cos(orient.k);
    return vn.t - w.y*sin(ocw.k) - w.z*cos(ocw.k);
}
//уравнения динамики полета
double Helicopter::f_Fx()//
{
    return Fx - mh*(a.x + w.y*v.z - w.z*v.y);
}
double Helicopter::f_Fy()//
{
    return Fy - mh*(a.y + w.z*v.x - w.x*v.z);
}
double Helicopter::f_Fz()//
{
    return Fz - mh*(a.z + w.x*v.y - w.y*v.x);
}
double Helicopter::f_Mx()//
{
    return Mx - Ix*e.x + (Iy-Iz)*w.y*w.z - Ixy*(w.x*w.z - e.y);
}
double Helicopter::f_My()//
{
    return My - Iy*e.y + (Iz-Ix)*w.z*w.x + Ixy*(w.y*w.z + e.x);
}
double Helicopter::f_Mz()//
{
    return Mz - Iz*e.z + (Ix-Iy)*w.x*w.y + Ixy*(w.y*w.y - w.x*w.x);
}

QString Helicopter::Post2log()
{
    xyz cts,ctn;
    QString logstring = "";
    logstring += "i=" + sss.setNum(iter) + "\nt=" + ss.setNum(t,'f',6) + "\n";
    logstring += "Ox=       " + ss.setNum(org.x        ,'f',3) + " м   \t\t- продольное положение начала связанной СКС в земной СК;\n";
    logstring += "Oy=       " + ss.setNum(org.y        ,'f',3) + " м   \t\t- положение по высоте  начала связанной СКС в земной СК;\n";
    logstring += "Oz=       " + ss.setNum(org.z        ,'f',3) + " м   \t\t- поперечное положение начала связанной СКС в земной СК;\n";
    logstring += "Ocwx=     " + ss.setNum(ocw.x        ,'f',3) + " м   \t\t- продольное положение ЦТ вертолета в земной СК;\n";
    logstring += "Ocwy=     " + ss.setNum(ocw.y        ,'f',3) + " м   \t\t- положение по высоте  ЦТ вертолета в земной СК;\n";
    logstring += "Ocwz=     " + ss.setNum(ocw.z        ,'f',3) + " м   \t\t- поперечное положение ЦТ вертолета в земной СК;\n";
    logstring += "kren=     " + ss.setNum(ocw.k*180/pi ,'f',3) + " грд \t\t- угол крена   (gama-поворот СКС в нормальной СК);\n";
    logstring += "rysk=     " + ss.setNum(ocw.r*180/pi ,'f',3) + " грд \t\t- угол рыскания (psi-поворот СКС в нормальной СК);\n";
    logstring += "tang=     " + ss.setNum(ocw.t*180/pi ,'f',3) + " грд \t\t- угол тангажа (teta-поворот СКС в нормальной СК);\n";
    logstring += "vn(xyz)=  " + ss.setNum(vn.x,'f',3) + ", ";
                   logstring += ss.setNum(vn.y,'f',3) + ", ";
                   logstring += ss.setNum(vn.z,'f',3) + " м/с  \t- скорость ЦТ вертолета в нормальной СК\n";
    logstring += "wn(krt)=  " + ss.setNum(vn.k,'f',3) + ", ";
                   logstring += ss.setNum(vn.r,'f',3) + ", ";
                   logstring += ss.setNum(vn.t,'f',3) + " рад/с\t- угл.скорость фюзеляжа в нормальной СК\n";
    logstring += "v(xyz)=   " + ss.setNum( v.x,'f',3) + ", ";
                   logstring += ss.setNum( v.y,'f',3) + ", ";
                   logstring += ss.setNum( v.z,'f',3) + " м/с  \t- скорость  ЦТ вертолета в СКС\n";
    logstring += "w(krt)=   " + ss.setNum( w.x,'f',3) + ", ";
                   logstring += ss.setNum( w.y,'f',3) + ", ";
                   logstring += ss.setNum( w.z,'f',3) + " рад/с\t- угл.скорость фюзеляжа в СКС\n";
    logstring += "n(xyz)=   " + ss.setNum(Fx/Gh  ,'f',3) + ", ";
                   logstring += ss.setNum(1+Fy/Gh,'f',3) + ", ";
                   logstring += ss.setNum(Fz/Gh  ,'f',3) + "  \t- перегрузка в ЦТ вертолета в СКС\n";
    logstring += "Fsum=     " + ss.setNum(Fx,'f',1) + ", ";    logstring += ss.setNum(Fy,'f',1) + ", ";    logstring += ss.setNum(Fz,'f',1) + "\t- проекции результирующей силы приведенной к ЦТ в СКС [Н];\n";
    logstring += "Msum=     " + ss.setNum(Mx,'f',1) + ", ";    logstring += ss.setNum(My,'f',1) + ", ";    logstring += ss.setNum(Mz,'f',1) + "\t- проекции результирующего момента приведенного к ЦТ в СКС [Н·м];\n";

    //положение центра тяжести вертолета в СКС
    cts.x = xct;   cts.y = yct;   cts.z = zct;
    //положение центра тяжести вертолета в СКН
    ctn = SKStoSKN(cts);
    logstring += "xctn=     " + ss.setNum(ctn.x  *1000,'f',1) + " мм \t\t- положение ЦТ вертолета в СКН;\n";
    logstring += "yctn=     " + ss.setNum(ctn.y  *1000,'f',1) + " мм \t\t- положение ЦТ вертолета в СКН;\n";
    logstring += "zctn=     " + ss.setNum(ctn.z  *1000,'f',1) + " мм \t\t- положение ЦТ вертолета в СКН;\n";

    float pmrear,pmfront,plrear,plfront;
    float snCx_front=0, snCx_rear=0;
    int nfront=0, nrear=0, nfrontrear=0;
    if (nst>0)  nfrontrear = (st[nst-1]->kostyl)?(nst-1):(nst);
    for (int k=0; k < nfrontrear; k++)
    {
        if (st[k]->snC.x > 0)  {  snCx_front+= st[k]->snC.x;  nfront++; }
        if (st[k]->snC.x < 0)  {  snCx_rear += st[k]->snC.x;  nrear++;  }
    }
    if (nfront>0 && nrear>0)
    {
        pmrear = 100*( snCx_front/nfront - ctn.x ) / ( snCx_front/nfront - snCx_rear/nrear );
        pmfront= 100 - pmrear;
        plrear = ctn.x - snCx_rear/nrear;
        plfront= ctn.x - snCx_front/nfront;
        logstring += "Lfront=   " + ss.setNum(plfront*1000,'f',1) + " мм \t\t- положение ЦТ вертолета относительно колес шасси;\n";
        logstring += "Lrear=    " + ss.setNum(plrear *1000,'f',1) + " мм \t\t- положение ЦТ вертолета относительно колес шасси;\n";
        logstring += "Wfront=   " + ss.setNum(pmfront     ,'f',1) + " % \t\t- часть веса вертолета приходящаяся на колеса передних стоек шасси;\n";
        logstring += "Wrear=    " + ss.setNum(pmrear      ,'f',1) + " % \t\t- часть веса вертолета приходящаяся на колеса задних стоек шасси;\n";
        logstring += "\n";
    }

        /*switch (nst)
        {
        case 1: case 2:

            break;
        case 3:
            pmrear = 100*(st[0]->snC.x - ctn.x) / (st[0]->snC.x - 0.5*(st[1]->snC.x+st[2]->snC.x));//100*(st[0]->saC.x - ctn.x) / (st[0]->saC.x - 0.5*(st[1]->saC.x+st[2]->saC.x));
            pmfront= 100 - pmrear;
            plrear = ctn.x - 0.5*(st[1]->snC.x+st[2]->snC.x);
            plfront= ctn.x - st[0]->snC.x;
            logstring += "Lfront=   " + ss.setNum(plfront*1000,'f',1) + " мм \t\t- положение ЦТ вертолета относительно колес шасси;\n";
            logstring += "Lrear=    " + ss.setNum(plrear *1000,'f',1) + " мм \t\t- положение ЦТ вертолета относительно колес шасси;\n";
            logstring += "Wfront=   " + ss.setNum(pmfront     ,'f',1) + " % \t\t- часть веса вертолета приходящаяся на колеса передних стоек шасси;\n";
            logstring += "Wrear=    " + ss.setNum(pmrear      ,'f',1) + " % \t\t- часть веса вертолета приходящаяся на колеса задних стоек шасси;\n";
            break;
        case 4: case 5:
            pmrear = 100*(0.5*(st[0]->snC.x+st[1]->snC.x) - ctn.x) / (0.5*(st[0]->snC.x+st[1]->snC.x) - 0.5*(st[2]->snC.x+st[3]->snC.x));//100*(st[0]->saC.x - ctn.x) / (st[0]->saC.x - 0.5*(st[1]->saC.x+st[2]->saC.x));
            pmfront= 100 - pmrear;
            plrear = ctn.x - 0.5*(st[2]->snC.x+st[3]->snC.x);
            plfront= ctn.x - 0.5*(st[0]->snC.x+st[1]->snC.x);
            logstring += "Lfront=   " + ss.setNum(plfront*1000,'f',1) + " мм \t\t- положение ЦТ вертолета относительно колес шасси;\n";
            logstring += "Lrear=    " + ss.setNum(plrear *1000,'f',1) + " мм \t\t- положение ЦТ вертолета относительно колес шасси;\n";
            logstring += "Wfront=   " + ss.setNum(pmfront     ,'f',1) + " % \t\t- часть веса вертолета приходящаяся на колеса передних стоек шасси;\n";
            logstring += "Wrear=    " + ss.setNum(pmrear      ,'f',1) + " % \t\t- часть веса вертолета приходящаяся на колеса задних стоек шасси;\n";
            break;
        }*/

    //стойки шасси
    for (int k=0; k<nst; k++)
    {
        double hpsum   = sqrt( st[k]->pn->h.x *st[k]->pn->h.x  + st[k]->pn->h.y *st[k]->pn->h.y  + st[k]->pn->h.x *st[k]->pn->h.x  );
        double hp1summ = sqrt( st[k]->pn->h1.x*st[k]->pn->h1.x + st[k]->pn->h1.y*st[k]->pn->h1.y + st[k]->pn->h1.x*st[k]->pn->h1.x );
        double hp2summ = sqrt( st[k]->pn->h2.x*st[k]->pn->h2.x + st[k]->pn->h2.y*st[k]->pn->h2.y + st[k]->pn->h2.x*st[k]->pn->h2.x );
        double Ppsumm  = sqrt( st[k]->pn->P.x *st[k]->pn->P.x  + st[k]->pn->P.y *st[k]->pn->P.y  + st[k]->pn->P.x *st[k]->pn->P.x  );
        double Pp1summ = sqrt( st[k]->pn->P1.x*st[k]->pn->P1.x + st[k]->pn->P1.y*st[k]->pn->P1.y + st[k]->pn->P1.x*st[k]->pn->P1.x );
        double Pp2summ = sqrt( st[k]->pn->P2.x*st[k]->pn->P2.x + st[k]->pn->P2.y*st[k]->pn->P2.y + st[k]->pn->P2.x*st[k]->pn->P2.x );

        logstring += "стойка №" + ss.setNum(k);
        logstring += " (контакт=" + ss.setNum(st[k]->contact) + ")\n";
        logstring +=     "s"+sss.setNum(k)+".p.h=   " + ss.setNum(hpsum  *1000,'f',1) + " (";   logstring += ss.setNum(st[k]->pn->h.x *1000,'f',1) + ", ";   logstring += ss.setNum(st[k]->pn->h.y *1000,'f',1) + ", ";   logstring += ss.setNum(st[k]->pn->h.z *1000,'f',1) + ")    \t- обжатие пневматика (+ проекции обжатия), мм;\n";
        if(st[k]->nwh==2)
        {
            logstring += "s"+sss.setNum(k)+".p1.h=  " + ss.setNum(hp1summ*1000,'f',1) + " (";   logstring += ss.setNum(st[k]->pn->h1.x*1000,'f',1) + ", ";   logstring += ss.setNum(st[k]->pn->h1.y*1000,'f',1) + ", ";   logstring += ss.setNum(st[k]->pn->h1.z*1000,'f',1) + ")  \t- обжатие пневматика #1 в паре (+ проекции обжатия), мм;\n";
            logstring += "s"+sss.setNum(k)+".p2.h=  " + ss.setNum(hp2summ*1000,'f',1) + " (";   logstring += ss.setNum(st[k]->pn->h2.x*1000,'f',1) + ", ";   logstring += ss.setNum(st[k]->pn->h2.y*1000,'f',1) + ", ";   logstring += ss.setNum(st[k]->pn->h2.z*1000,'f',1) + ")  \t- обжатие пневматика #2 в паре (+ проекции обжатия), мм;\n";
        }
        logstring +=     "s"+sss.setNum(k)+".p.P=   " + ss.setNum(Ppsumm, 'f',1) + " (";    logstring += ss.setNum(st[k]->pn->P.x, 'f',1) + ", ";    logstring += ss.setNum(st[k]->pn->P.y, 'f',1) + ", ";    logstring += ss.setNum(st[k]->pn->P.z, 'f',1) + ")\t- усилие на пневматике (+ проекции усилия), Н;\n";
        if(st[k]->nwh==2)
        {
            logstring += "s"+sss.setNum(k)+".p1.P=  " + ss.setNum(Pp1summ,'f',1) + " (";    logstring += ss.setNum(st[k]->pn->P1.x,'f',1) + ", ";    logstring += ss.setNum(st[k]->pn->P1.y,'f',1) + ", ";    logstring += ss.setNum(st[k]->pn->P1.z,'f',1) + ")\t- усилие на пневматике #1 в паре (+ проекции усилия), Н;\n";
            logstring += "s"+sss.setNum(k)+".p2.P=  " + ss.setNum(Pp2summ,'f',1) + " (";    logstring += ss.setNum(st[k]->pn->P2.x,'f',1) + ", ";    logstring += ss.setNum(st[k]->pn->P2.y,'f',1) + ", ";    logstring += ss.setNum(st[k]->pn->P2.z,'f',1) + ")\t- усилие на пневматике #2 в паре (+ проекции усилия), Н;\n";
        }
        logstring +=     "s"+sss.setNum(k)+".a.h=   " + ss.setNum(st[k]->ab->ha*1000,  'f',1) ;//+ " мм "
        if(st[k]->atype>1)
        {
            logstring += " ("+ss.setNum(st[k]->ab->c1.h*1000,'f',1)+ " + ";    logstring += ss.setNum(st[k]->ab->c2.h*1000,'f',1)+ ")   \t\t- обжатие штоков амортизатора, мм;\n";
        }
        else logstring += "\t\t\t- обжатие штоков амортизатора;\n";
        logstring +=     "s"+sss.setNum(k)+".a.P=   " + ss.setNum(st[k]->ab->Pa,  'f',1)+ "\t\t\t- осевое усилие на штоках амортизатора, Н;\n";
    }
    logstring += "\n\n";
    if(mrsolve)
    {
        logstring += CurrentInfo_MainRotor();///временно
        //"mainrotor:\n";
        //logstring += mr->logstr;
    }
    if(trmount)
    {
        logstring += CurrentInfo_TailRotor();///временно
    }
    //if(prmount)
    //{
    //    logstring += CurrentInfo_PushRotor();///временно
    //}

return logstring;
}
QString Helicopter::CurrentInfo_Airframe()//вывод результатов расчета ПЛАНЕР на текущей итерации
{
    QString info="";
    //ПЛАНЕР
    info += "AIRFRAME:  warning= " + ss.setNum(warning) + "\n";
    info += "O.x="  + ss.setNum(org.x           ,'f',3) + "\t";
    info += "O.y="  + ss.setNum(org.y           ,'f',3) + "\t";
    info += "O.z="  + ss.setNum(org.z           ,'f',3) + "\n";
    info += "kren=" + ss.setNum(org.k    *180/pi,'f',2) + "\t";
    info += "rysk=" + ss.setNum(org.r    *180/pi,'f',2) + "\t";
    info += "tang=" + ss.setNum(org.t    *180/pi,'f',2) + "\n";
    info += "\n";
    info += "Vnx="  + ss.setNum(vn.x            ,'f',2) + "\t";
    info += "Vny="  + ss.setNum(vn.y            ,'f',2) + "\t";
    info += "Vnz="  + ss.setNum(vn.z            ,'f',2) + "\n";
    //info += "wnk="  + ss.setNum(vn.k     *180/pi,'f',2) + "\t";
    //info += "wnr="  + ss.setNum(vn.r     *180/pi,'f',2) + "\t";
    //info += "wnt="  + ss.setNum(vn.t     *180/pi,'f',2) + "\n";
    info += "vx="   + ss.setNum(v.x             ,'f',2) + "\t";
    info += "vy="   + ss.setNum(v.y             ,'f',2) + "\t";
    info += "vz="   + ss.setNum(v.z             ,'f',2) + "\n";
    info += "wk="   + ss.setNum(w.x      *180/pi,'f',2) + "\t";
    info += "wr="   + ss.setNum(w.y      *180/pi,'f',2) + "\t";
    info += "wt="   + ss.setNum(w.z      *180/pi,'f',2) + "\n";
    info += "ax="   + ss.setNum(a.x             ,'f',2) + "\t";
    info += "ay="   + ss.setNum(a.y             ,'f',2) + "\t";
    info += "az="   + ss.setNum(a.z             ,'f',2) + "\n";
    info += "ex="   + ss.setNum(e.x             ,'f',2) + "\t";
    info += "ey="   + ss.setNum(e.y             ,'f',2) + "\t";
    info += "ez="   + ss.setNum(e.z             ,'f',2) + "\n";
    //info += "nx="   + ss.setNum(a.x/g           ,'f',2) + "\t";
    //info += "ny="   + ss.setNum(1+a.y/g         ,'f',2) + "\t";
    //info += "nz="   + ss.setNum(a.z/g           ,'f',2) + "\n";
    info += "nx="   + ss.setNum(Fx/Gh           ,'f',2) + "\t";
    info += "ny="   + ss.setNum(1+Fy/Gh         ,'f',2) + "\t";
    info += "nz="   + ss.setNum(Fz/Gh           ,'f',2) + "\n";
    info += "Fx="   + ss.setNum(Fx              ,'f',0) + "\t";
    info += "Fy="   + ss.setNum(Fy              ,'f',0) + "\t";
    info += "Fz="   + ss.setNum(Fz              ,'f',0) + "\n";
    info += "Mx="   + ss.setNum(Mx              ,'f',0) + "\t";
    info += "My="   + ss.setNum(My              ,'f',0) + "\t";
    info += "Mz="   + ss.setNum(Mz              ,'f',0) + "\n";
    info += "\nвнешнее воздействие:\n";
    info += "Fx="   + ss.setNum(FM1.Fx          ,'f',0) + "\t";
    info += "Fy="   + ss.setNum(FM1.Fy          ,'f',0) + "\t";
    info += "Fz="   + ss.setNum(FM1.Fz          ,'f',0) + "\n";
    info += "\nвес вертолета:\n";
    info += "Fx="   + ss.setNum(FMhg.Fx         ,'f',0) + "\t";
    info += "Fy="   + ss.setNum(FMhg.Fy         ,'f',0) + "\t";
    info += "Fz="   + ss.setNum(FMhg.Fz         ,'f',0) + "\n";
    info += "Mx="   + ss.setNum(FMhg.Mx         ,'f',0) + "\t";
    info += "My="   + ss.setNum(FMhg.My         ,'f',0) + "\t";
    info += "Mz="   + ss.setNum(FMhg.Mz         ,'f',0) + "\n";
    info += "\nшасси (приведено к ЦТ вертолета):\n";
    info += "Fx="   + ss.setNum(FMsh.Fx         ,'f',0) + "\t";
    info += "Fy="   + ss.setNum(FMsh.Fy         ,'f',0) + "\t";
    info += "Fz="   + ss.setNum(FMsh.Fz         ,'f',0) + "\n";
    info += "Mx="   + ss.setNum(FMsh.Mx         ,'f',0) + "\t";
    info += "My="   + ss.setNum(FMsh.My         ,'f',0) + "\t";
    info += "Mz="   + ss.setNum(FMsh.Mz         ,'f',0) + "\n";
    info += "\nнесущий винт:\n";
    info += "alfa=" + ss.setNum(alfa_mr*180/M_PI,'f',1) + " грд\n";
    info += "Fx="   + ss.setNum(FMmr.Fx         ,'f',0) + "\t";
    info += "Fy="   + ss.setNum(FMmr.Fy         ,'f',0) + "\t";
    info += "Fz="   + ss.setNum(FMmr.Fz         ,'f',0) + "\n";
    info += "Mx="   + ss.setNum(FMmr.Mx         ,'f',0) + "\t";
    info += "My="   + ss.setNum(FMmr.My         ,'f',0) + "\t";
    info += "Mz="   + ss.setNum(FMmr.Mz         ,'f',0) + "\n";
    info += "\nрулевой винт:\n";
    info += "alfa=" + ss.setNum(alfa_tr*180/M_PI,'f',1) + " грд\n";
    info += "Fx="   + ss.setNum(FMtr.Fx         ,'f',0) + "\t";
    info += "Fy="   + ss.setNum(FMtr.Fy         ,'f',0) + "\t";
    info += "Fz="   + ss.setNum(FMtr.Fz         ,'f',0) + "\n";
    info += "Mx="   + ss.setNum(FMtr.Mx         ,'f',0) + "\t";
    info += "My="   + ss.setNum(FMtr.My         ,'f',0) + "\t";
    info += "Mz="   + ss.setNum(FMtr.Mz         ,'f',0) + "\n";
    info += "\nтолкающий винт:\n";
    //info += "alfa=" + ss.setNum(alfa_tr*180/M_PI,'f',1) + " грд\n";
    info += "Fx="   + ss.setNum(FMpr.Fx         ,'f',0) + "\t";
    info += "Fy="   + ss.setNum(FMpr.Fy         ,'f',0) + "\t";
    info += "Fz="   + ss.setNum(FMpr.Fz         ,'f',0) + "\n";
    info += "Mx="   + ss.setNum(FMpr.Mx         ,'f',0) + "\t";
    info += "My="   + ss.setNum(FMpr.My         ,'f',0) + "\t";
    info += "Mz="   + ss.setNum(FMpr.Mz         ,'f',0) + "\n";
    info += "\nпланер:\n";
    info += "Fx="   + ss.setNum(FMadp.Fx        ,'f',0) + "\t";
    info += "Fy="   + ss.setNum(FMadp.Fy        ,'f',0) + "\t";
    info += "Fz="   + ss.setNum(FMadp.Fz        ,'f',0) + "\n";
    info += "Mx="   + ss.setNum(FMadp.Mx        ,'f',0) + "\t";
    info += "My="   + ss.setNum(FMadp.My        ,'f',0) + "\t";
    info += "Mz="   + ss.setNum(FMadp.Mz        ,'f',0) + "\n";
    info += "\nфюзеляж (лок.СК):\n";//(без оперения)
    info += "kren=" + ss.setNum(kren   *180/M_PI,'f',1) + "\t";
    info += "tang=" + ss.setNum(tang   *180/M_PI,'f',1) + "\t";
    info += "ttra=" + ss.setNum(tetra  *180/M_PI,'f',1) + "\n";
    info += "alfa=" + ss.setNum(alfa_f *180/M_PI,'f',1) + "\t";
    info += "beta=" + ss.setNum(beta_sk*180/M_PI,'f',1) + "\n";
    info += "Fx="   + ss.setNum(Fx_f            ,'f',0) + "\t";
    info += "Fy="   + ss.setNum(Fy_f            ,'f',0) + "\t";
    info += "Fz="   + ss.setNum(Fz_f            ,'f',0) + "\n";
    info += "Mx="   + ss.setNum(Mx_f            ,'f',0) + "\t";
    info += "My="   + ss.setNum(My_f            ,'f',0) + "\t";
    info += "Mz="   + ss.setNum(Mz_f            ,'f',0) + "\n";
    info += "\nкрыло (лок.СК):\n";
    info += "alfa=" + ss.setNum(alfa_wing*180/M_PI,'f',1) + " грд\n";
    info += "Fx="   + ss.setNum(Fx_wing         ,'f',0) + "\t";
    info += "Fy="   + ss.setNum(Fy_wing         ,'f',0) + "\t";
    info += "Fz="   + ss.setNum(Fz_wing         ,'f',0) + "\n";
    info += "\nстабилизатор (лок.СК):\n";
    info += "alfa=" + ss.setNum(alfa_st*180/M_PI,'f',1) + " грд\n";
    info += "Fx="   + ss.setNum(Fx_st           ,'f',0) + "\t";
    info += "Fy="   + ss.setNum(Fy_st           ,'f',0) + "\t";
    info += "Fz="   + ss.setNum(Fz_st           ,'f',0) + "\n";
    info += "\nкиль (лок.СК):\n";
    info += "alfa=" + ss.setNum(alfa_keel*180/M_PI,'f',1) + " грд\n";
    info += "Fx="   + ss.setNum(Fx_keel         ,'f',0) + "\t";
    info += "Fy="   + ss.setNum(Fy_keel         ,'f',0) + "\t";
    info += "Fz="   + ss.setNum(Fz_keel         ,'f',0) + "\n";

return info;
}
QString Helicopter::CurrentInfo_MainRotor()//вывод результатов расчета MainRotor на текущей итерации
{
    QString info="\nMAINROTOR: ";
    if(mrsolve)
    {
        //if(mr->revcount<2)
        //    info += "stabilization... \nrevcount= " + ss.setNum(mr->revcount) + "\n";
        //else
        //{
            info += "warning= " + ss.setNum(mr->warning) + "\n";
            info += mr->logstr;
        //}
    }
    else
    {
        info += "unsolved\n";
        info += "Fx=" + ss.setNum(Fx_mr  ,'f',0) + "\t";
        info += "Fy=" + ss.setNum(Fy_mr  ,'f',0) + "\t";
        info += "Fz=" + ss.setNum(Fz_mr  ,'f',0) + ";\n";
        info += "Mx=" + ss.setNum(Mx_mr  ,'f',0) + "\t";
        info += "My=" + ss.setNum(My_mr  ,'f',0) + "\t";
        info += "Mz=" + ss.setNum(Mz_mr  ,'f',0) + ";\n";
    }
return info;
}
QString Helicopter::CurrentInfo_TailRotor()//вывод результатов расчета TailRotor на текущей итерации
{
    QString info="\nTAILROTOR: ";
    if(trsolve)
    {
        //if(tr->revcount<2)
        //    info += "stabilization... \nrevcount= " + ss.setNum(tr->revcount) + "\n";
        //else
        //{
            info += "warning= " + ss.setNum(tr->warning) + "\n";
            info += tr->logstr;
        //}
        //info += "\n";
        //info += "T_tr_=      " + ss.setNum(T_tr_,'f',1) + "\n";
        //info += "T_need=      " + ss.setNum(Ttr_need,'f',1) + "\n";
        //info += "tr->T_sr=   " + ss.setNum(tr->T_sr ,'f',1) + "\n";
        //info += "tr->Fay_sum=" + ss.setNum(tr->Fay_sum,'f',1) + "\n";
        //info += "tr->Fhy_sum=" + ss.setNum(tr->Fhy_sum,'f',1) + "\n";
        //info += "T_tr=       " + ss.setNum(T_tr ,'f',1) + "\n";
        //info += "\n";
    }
    else
    {
        info += "unsolved\n";
        info += "Fx=" + ss.setNum(Fx_tr  ,'f',0) + "\t";
        info += "Fy=" + ss.setNum(Fy_tr  ,'f',0) + "\t";
        info += "Fz=" + ss.setNum(Fz_tr  ,'f',0) + "\n";
        info += "Mx=" + ss.setNum(Mx_tr  ,'f',0) + "\t";
        info += "My=" + ss.setNum(My_tr  ,'f',0) + "\t";
        info += "Mz=" + ss.setNum(Mz_tr  ,'f',0) + "\n";
    }
return info;
}
QString Helicopter::CurrentInfo_Shassi()//вывод результатов расчета ШАССИ на текущей итерации
{
    QString info="";
    info += "\nGEAR: nst=" + ss.setNum(nst)+ "\n";
    for (int k = 0; k < nst; k++)
    {
        info += "\nstrut №" + ss.setNum(k);
        info += " (contact="+ ss.setNum(st[k]->contact);
        info += ", status=" + ss.setNum(st[k]->status);
        if(st[k]->warning)     { info +=    ", warning=" + ss.setNum(st[k]->warning    ); }// warnings = true;
        if(st[k]->ab->warning) { info += ", ab.warning=" + ss.setNum(st[k]->ab->warning); }// warnings = true;
        if(st[k]->pn->warning) { info += ", pn.warning=" + ss.setNum(st[k]->pn->warning); }// warnings = true;
        info += ")\n";
        info += "p~a: ";
        info += "Ppy="  + ss.setNum(st[k]->Ppy,  'f',0) + "[";
        info += ss.setNum(st[k]->pn->status);
        if(st[k]->nwh==2)
        {
            info += ":" + ss.setNum(st[k]->pn->status1);
            info += "," + ss.setNum(st[k]->pn->status2);
        }
        info += "] ~ ";
        info += "Pay="  + ss.setNum(st[k]->Pay, 'f',0);
        info +=     "[" + ss.setNum(st[k]->ab->status) + "]\n";
        info += "hp=  " + ss.setNum(st[k]->pn->h.x*1000, 'f',2) + ", ";
        info +=  ss.setNum(st[k]->pn->h.y*1000, 'f',2) + ", ";
        info +=  ss.setNum(st[k]->pn->h.z*1000, 'f',2) + "\n";
        if(st[k]->nwh==2)
        {
            info += "hp1= " + ss.setNum(st[k]->pn->h1.x*1000,'f',2) + ", ";
            info +=           ss.setNum(st[k]->pn->h1.y*1000,'f',2) + ", ";
            info +=           ss.setNum(st[k]->pn->h1.z*1000,'f',2) + "\n";
            info += "hp2= " + ss.setNum(st[k]->pn->h2.x*1000,'f',2) + ", ";
            info +=           ss.setNum(st[k]->pn->h2.y*1000,'f',2) + ", ";
            info +=           ss.setNum(st[k]->pn->h2.z*1000,'f',2) + "\n";
        }
            info += "Pp=  " + ss.setNum(st[k]->pn->P.x, 'f',1) + ", ";
            info +=           ss.setNum(st[k]->pn->P.y, 'f',1) + ", ";
            info +=           ss.setNum(st[k]->pn->P.z, 'f',1) + "\n";
        if(st[k]->nwh==2)
        {
            info += "Pp1= " + ss.setNum(st[k]->pn->P1.x,'f',1) + ", ";
            info +=           ss.setNum(st[k]->pn->P1.y,'f',1) + ", ";
            info +=           ss.setNum(st[k]->pn->P1.z,'f',1) + "\n";
            info += "Pp2= " + ss.setNum(st[k]->pn->P2.x,'f',1) + ", ";
            info +=           ss.setNum(st[k]->pn->P2.y,'f',1) + ", ";
            info +=           ss.setNum(st[k]->pn->P2.z,'f',1) + "\n";
        }
            info += "ha=  " + ss.setNum(st[k]->ab->ha*1000,  'f',2);
        if(st[k]->atype>1)
        {
            info +=    " (" + ss.setNum(st[k]->ab->c1.h*1000,'f',2)+ " + ";
            info +=           ss.setNum(st[k]->ab->c2.h*1000,'f',2)+ ") \n";
        }
        else info += " \n";
            info += "Pa=  " + ss.setNum(st[k]->ab->Pa,  'f',1)+ " \n";
        if(st[k]->atype>1)
        {
            info += "c1~c2: ";
            info += "Pac1=" + ss.setNum(st[k]->ab->c1.P,'f',0) + " ~ ";
            info += "Pac2=" + ss.setNum(st[k]->ab->c2.P,'f',0) + "\n";
        }
        //if(st[k]->atype==0) info += "q=" + ss.setNum(st[k]->ab->q) + "\n";
        //info += "ang_sx= " + ss.setNum(st[k]->ang_sx *r2d,'f',3)+ " грд\n";
        //info += "ang_sz= " + ss.setNum(st[k]->ang_sz *r2d,'f',3)+ " грд\n";
        //info += "ang_ax= " + ss.setNum(st[k]->ang_ax *r2d,'f',3)+ " грд\n";
        //info += "ang_az= " + ss.setNum(st[k]->ang_az *r2d,'f',3)+ " грд\n";
        //info += "Aa=   "   + ss.setNum(st[k]->Aa         ,'f',1)+ " Дж\n";
        //info += "psih= "   + ss.setNum(st[k]->psih        ,'f',3)+ " \t";
        //info += "psif= "   + ss.setNum(st[k]->psif        ,'f',3)+ " \n";
        //info += "c1.f= "   + ss.setNum(st[k]->ab->c1.f *1000000,'f',3)+ " \t";//0.000001
        //info += "c1.fr="   + ss.setNum(st[k]->ab->c1.fr*1000000,'f',3)+ " \n";//0.000001
        //info += "c2.f= "   + ss.setNum(st[k]->ab->c2.f *1000000,'f',3)+ " \t";//0.000001
        //info += "c2.fr="   + ss.setNum(st[k]->ab->c2.fr*1000000,'f',3)+ " \n";//0.000001
        /*if(st[k]->stype==2)
        {
            xyz Wfree ;//= st[k]->Wfree;
            Wfree.x = st[k]->Wfree.x;/// +st[k]->dOx;//
            Wfree.y = st[k]->Wfree.y;/// +st[k]->dOy;//
            Wfree.z = st[k]->Wfree.z;
            xyz sksWfree = st[k]->SKo2SKS(Wfree);

            info += "\n";
            info +=  "dOx= " + ss.setNum(st[k]->dOx *1000,'f',3)+ " мм\n";
            info +=  "dOy= " + ss.setNum(st[k]->dOy *1000,'f',3)+ " мм\n";
            info +=  "sksWfree= {" + ss.setNum(sksWfree.x  *1000,'f',0) + ", ";
            info +=           ss.setNum(sksWfree.y  *1000,'f',0) + ", ";
            info +=           ss.setNum(sksWfree.z  *1000,'f',0) + "}\n";
            //xyz sks_Q  =  st[k]->SKo2SKS(st[k]->Q);
            //info +=  "Q= {" + ss.setNum(sks_Q.x  *1000,'f',0) + ", ";
            //info +=           ss.setNum(sks_Q.y  *1000,'f',0) + ", ";
            //info +=           ss.setNum(sks_Q.z  *1000,'f',0) + "}\n";
        }*/
        /*if(st[k]->stype==8)
        {
            info += "\n";
            info +=  "OB= " + ss.setNum(st[k]->OBxy *1000,'f',1)+ " мм\n";
            info +=  "BV= " + ss.setNum(st[k]->BVxy *1000,'f',1)+ " мм\n";
            info += "BOV= " + ss.setNum(st[k]->BOVxy *r2d,'f',3)+ " грд\n";
            info += "BOW= " + ss.setNum(st[k]->BOWxy *r2d,'f',3)+ " грд\n";
            info += "VOW= " + ss.setNum(st[k]->VOWxy *r2d,'f',3)+ " грд\n";
            info +=  "B= {" + ss.setNum(st[k]->B.x  *1000,'f',0) + ", ";
            info +=           ss.setNum(st[k]->B.y  *1000,'f',0) + ", ";
            info +=           ss.setNum(st[k]->B.z  *1000,'f',0) + "}\n";
            info +=  "V= {" + ss.setNum(st[k]->V.x  *1000,'f',0) + ", ";
            info +=           ss.setNum(st[k]->V.y  *1000,'f',0) + ", ";
            info +=           ss.setNum(st[k]->V.z  *1000,'f',0) + "}\n";
            info +=  "W= {" + ss.setNum(st[k]->W.x  *1000,'f',0) + ", ";
            info +=           ss.setNum(st[k]->W.y  *1000,'f',0) + ", ";
            info +=           ss.setNum(st[k]->W.z  *1000,'f',0) + "}\n";
        }*/
    }
    info += "\n\n";
return info;
}
void Helicopter::fillOut(outstructure &out)//вывод результатов в структуру результатов расчета текущей итерации
{
    out.iter = iter;
    out.t    = t;
    //положение
    out.fus.org.x = org.x ;    //положение начала СКС вертолета в СКЗ
    out.fus.org.y = org.y ;    //положение начала СКС вертолета в СКЗ
    out.fus.org.z = org.z ;    //положение начала СКС вертолета в СКЗ
    out.fus.org.k = org.k *180/M_PI;//угол ориентации осей СКС в нормальной СК
    out.fus.org.r = org.r *180/M_PI;//угол ориентации осей СКС в нормальной СК
    out.fus.org.t = org.t *180/M_PI;//угол ориентации осей СКС в нормальной СК
    out.fus.ocw.x = ocw.x ;    //положение ЦТ вертолета в СКЗ
    out.fus.ocw.y = ocw.y ;    //положение ЦТ вертолета в СКЗ
    out.fus.ocw.z = ocw.z ;    //положение ЦТ вертолета в СКЗ
    //скорость+ускорение
    out.fus.v.x   = v.x; //скорость ЦТ вертолета в СКС
    out.fus.v.y   = v.y; //скорость ЦТ вертолета в СКС
    out.fus.v.z   = v.z; //скорость ЦТ вертолета в СКС
    out.fus.w.x   = w.x; //угл.скорость фюзеляжа в СКС
    out.fus.w.y   = w.y; //угл.скорость фюзеляжа в СКС
    out.fus.w.z   = w.z; //угл.скорость фюзеляжа в СКС
    out.fus.a.x   = a.x; //ускорение ЦТ вертолета в СКС
    out.fus.a.y   = a.y; //ускорение ЦТ вертолета в СКС
    out.fus.a.z   = a.z; //ускорение ЦТ вертолета в СКС
    out.fus.e.x   = e.x; //угл.ускорение фюзеляжа в СКС
    out.fus.e.y   = e.y; //угл.ускорение фюзеляжа в СКС
    out.fus.e.z   = e.z; //угл.ускорение фюзеляжа в СКС
    out.fus.vn.x  = vn.x;//скорость ЦТ вертолета в нормальной СК
    out.fus.vn.y  = vn.y;//скорость ЦТ вертолета в нормальной СК
    out.fus.vn.z  = vn.z;//скорость ЦТ вертолета в нормальной СК
    out.fus.wn.x  = vn.k;//угл.скорость фюзеляжа в нормальной СК
    out.fus.wn.y  = vn.r;//угл.скорость фюзеляжа в нормальной СК
    out.fus.wn.z  = vn.t;//угл.скорость фюзеляжа в нормальной СК
    out.fus.an.x  = an.x;//ускорение ЦТ вертолета в нормальной СК
    out.fus.an.y  = an.y;//ускорение ЦТ вертолета в нормальной СК
    out.fus.an.z  = an.z;//ускорение ЦТ вертолета в нормальной СК
    out.fus.en.x  = en.x;//угл.ускорение фюзеляжа в нормальной СК
    out.fus.en.y  = en.y;//угл.ускорение фюзеляжа в нормальной СК
    out.fus.en.z  = en.z;//угл.ускорение фюзеляжа в нормальной СК
    //Усилия приведенные к ЦТ
    {
    out.fus.Fhg.x    = FMhg.Fx;
    out.fus.Fhg.y    = FMhg.Fy;
    out.fus.Fhg.z    = FMhg.Fz;
    out.fus.Mhg.x    = FMhg.Mx;
    out.fus.Mhg.y    = FMhg.My;
    out.fus.Mhg.z    = FMhg.Mz;
    out.fus.Fadp.x   = FMadp.Fx;
    out.fus.Fadp.y   = FMadp.Fy;
    out.fus.Fadp.z   = FMadp.Fz;
    out.fus.Madp.x   = FMadp.Mx;
    out.fus.Madp.y   = FMadp.My;
    out.fus.Madp.z   = FMadp.Mz;

    out.fus.fus_F.x  = Fx_f;
    out.fus.fus_F.y  = Fy_f;
    out.fus.fus_F.z  = Fz_f;
    out.fus.fus_M.x  = Mx_f;
    out.fus.fus_M.y  = My_f;
    out.fus.fus_M.z  = Mz_f;
    out.fus.wing_F.x = Fx_wing;
    out.fus.wing_F.y = Fy_wing;
    out.fus.wing_F.z = Fz_wing;
    out.fus.wing_M.x = Mx_wing;
    out.fus.wing_M.y = My_wing;
    out.fus.wing_M.z = Mz_wing;
    out.fus.stab_F.x = Fx_st;
    out.fus.stab_F.y = Fy_st;
    out.fus.stab_F.z = Fz_st;
    out.fus.stab_M.x = Mx_st;
    out.fus.stab_M.y = My_st;
    out.fus.stab_M.z = Mz_st;
    out.fus.keel_F.x = Fx_keel;
    out.fus.keel_F.y = Fy_keel;
    out.fus.keel_F.z = Fz_keel;
    out.fus.keel_M.x = Mx_keel;
    out.fus.keel_M.y = My_keel;
    out.fus.keel_M.z = Mz_keel;

    out.fus.fus.x    = x_f *1000;
    out.fus.fus.y    = y_f *1000;
    out.fus.fus.z    = z_f *1000;
    out.fus.stab.x   = x_st*1000;
    out.fus.stab.y   = y_st*1000;
    out.fus.stab.z   = z_st*1000;
    out.fus.keel.x   = x_k *1000;
    out.fus.keel.y   = y_k *1000;
    out.fus.keel.z   = z_k *1000;

    out.fus.Fmr.x    = FMmr.Fx;
    out.fus.Fmr.y    = FMmr.Fy;
    out.fus.Fmr.z    = FMmr.Fz;
    out.fus.Mmr.x    = FMmr.Mx;
    out.fus.Mmr.y    = FMmr.My;
    out.fus.Mmr.z    = FMmr.Mz;
    out.fus.Ftr.x    = FMtr.Fx;
    out.fus.Ftr.y    = FMtr.Fy;
    out.fus.Ftr.z    = FMtr.Fz;
    out.fus.Mtr.x    = FMtr.Mx;
    out.fus.Mtr.y    = FMtr.My;
    out.fus.Mtr.z    = FMtr.Mz;
    out.fus.Fpr.x    = FMpr.Fx;
    out.fus.Fpr.y    = FMpr.Fy;
    out.fus.Fpr.z    = FMpr.Fz;
    out.fus.Mpr.x    = FMpr.Mx;
    out.fus.Mpr.y    = FMpr.My;
    out.fus.Mpr.z    = FMpr.Mz;
    out.fus.Fsh.x    = FMsh.Fx;
    out.fus.Fsh.y    = FMsh.Fy;
    out.fus.Fsh.z    = FMsh.Fz;
    out.fus.Msh.x    = FMsh.Mx;
    out.fus.Msh.y    = FMsh.My;
    out.fus.Msh.z    = FMsh.Mz;
    out.fus.Fsum.x   = Fx;
    out.fus.Fsum.y   = Fy;
    out.fus.Fsum.z   = Fz;
    out.fus.Msum.x   = Mx;
    out.fus.Msum.y   = My;
    out.fus.Msum.z   = Mz;
    out.fus.F1.x = FM1.Fx;    out.fus.F1pos.x = load1pos.x;
    out.fus.F1.y = FM1.Fy;    out.fus.F1pos.y = load1pos.y;
    out.fus.F1.z = FM1.Fz;    out.fus.F1pos.z = load1pos.z;
    //out.fus.F2.x = FM2.Fx;  out.fus.F2pos.x = load2pos.x;
    //out.fus.F2.y = FM2.Fy;  out.fus.F2pos.y = load2pos.y;
    //out.fus.F2.z = FM2.Fz;  out.fus.F2pos.z = load2pos.z;
    }
    out.fus.xct = xct*1000;
    out.fus.yct = yct*1000;
    out.fus.zct = zct*1000;
    out.fus.selpoint.x = 0;
    out.fus.selpoint.y = 0;
    out.fus.selpoint.z = 0;
    out.fus.selpoint_Q.x = out.fus.org.x;
    out.fus.selpoint_Q.y = out.fus.org.y;
    out.fus.selpoint_Q.z = out.fus.org.z;
        out.fus.n.x  = 0 + a.x/g;//Fx/Gh;
        out.fus.n.y  = 1 + a.y/g;//Fy/Gh;
        out.fus.n.z  = 0 + a.z/g;//Fz/Gh;
    //груз на ВТП
    out.cargo.m      = mcg;
    out.cargo.Ox     = zacep.x*1000;
    out.cargo.Oy     = zacep.y*1000;
    out.cargo.Oz     = zacep.z*1000;
    out.cargo.CTx    = crgct.x*1000;
    out.cargo.CTy    = crgct.y*1000;
    out.cargo.CTz    = crgct.z*1000;
    out.cargo.Gx     = Gcrg.x;
    out.cargo.Gy     = Gcrg.y;
    out.cargo.Gz     = Gcrg.z;
    out.fus.Fcg.x    = FMcg.Fx;
    out.fus.Fcg.y    = FMcg.Fy;
    out.fus.Fcg.z    = FMcg.Fz;
    out.fus.Mcg.x    = FMcg.Mx;
    out.fus.Mcg.y    = FMcg.My;
    out.fus.Mcg.z    = FMcg.Mz;
    //MainRotor////////////////
    out.mr.solve     = mrsolve;
    out.mr.eps_fz    = eps_fz*180/M_PI;
    out.mr.eps_fx    = eps_fx*180/M_PI;
    out.mr.chi_hub   = chi_hub *180/M_PI;
    out.mr.eta_hub   = eta_hub *180/M_PI;
    out.mr.O.x       = x_mr *1000;
    out.mr.O.y       = y_mr *1000;
    out.mr.O.z       = z_mr *1000;
    out.mr.D         = Dmr  *1000;
    out.mr.Fhx_sr    = Fx_mr;
    out.mr.Fhy_sr    = Fy_mr;
    out.mr.Fhz_sr    = Fz_mr;
    out.mr.Mhx_sr    = Mx_mr;
    out.mr.Mhy_sr    = My_mr;
    out.mr.Mhz_sr    = Mz_mr;
    if(!mrsolve)
    {
        out.mr.nb      = 0;
        out.mr.l_vsh   = 0;
        //out.mr.sh.x     = 0;
        //out.mr.sh.y     = 0;
        //out.mr.sh.z     = 0;
        out.mr.vh.x     = 0;
        out.mr.vh.y     = 0;
        out.mr.vh.z     = 0;
        out.mr.ah.x     = 0;
        out.mr.ah.y     = 0;
        out.mr.ah.z     = 0;
        out.mr.Faxe    = 0;
        out.mr.angbs   = 0;
        out.mr.dctbs   = 0;
        out.mr.dct.x   = 0;
        out.mr.dct.y   = 0;
        out.mr.dct.z   = 0;
        out.mr.vgsolve = false;
    }
    //TailRotor//////////////
    out.tr.mount     = trmount;
    out.tr.solve     = trsolve;
    out.tr.O.x       = x_tr*1000;
    out.tr.O.y       = y_tr*1000;
    out.tr.O.z       = z_tr*1000;
    out.tr.D         = Dtr*1000;
    out.tr.Fhx_sr    = Fx_tr;
    out.tr.Fhy_sr    = Fy_tr;
    out.tr.Fhz_sr    = Fz_tr;
    out.tr.Mhx_sr    = Mx_tr;
    out.tr.Mhy_sr    = My_tr;
    out.tr.Mhz_sr    = Mz_tr;
    out.tr.az        = 0;
    out.tr.sec07     = 0;
    out.tr.revcount  = 0;
    //PushRotor///////////////
    out.pr.mount     = prmount;
    out.pr.solve     = prsolve;
    out.pr.O.x       = x_pr*1000;
    out.pr.O.y       = y_pr*1000;
    out.pr.O.z       = z_pr*1000;
    out.pr.D         = Dpr*1000;
    out.pr.Fhx_sr    = Fx_pr;
    out.pr.Fhy_sr    = Fy_pr;
    out.pr.Fhz_sr    = Fz_pr;
    out.pr.Mhx_sr    = Mx_pr;
    out.pr.Mhy_sr    = My_pr;
    out.pr.Mhz_sr    = Mz_pr;
    out.pr.az        = 0;
    out.pr.sec07     = 0;
    out.pr.revcount  = 0;
    //Shassi/////////////////
    out.nst = nst;
    if(nst)
    for (int k = 0; k < nst; k++)
    {
        out.st[k].type   = st[k]->stype;
        out.st[k].status = st[k]->status;
        out.st[k].Aa     = st[k]->Aa;
        out.st[k].Ap     = st[k]->Ap;
        out.st[k].Aap    = st[k]->Aa + st[k]->Ap;
        out.st[k].nwh    = st[k]->nwh;
        out.st[k].ground = st[k]->ground;
        out.st[k].AB     = st[k]->AB    *1000;
        out.st[k].Dwh    = st[k]->pn->Dp*1000;
        out.st[k].rwh    = st[k]->Rmd   *1000;
        out.st[k].Bwh    = st[k]->pn->Bp*1000;
        out.st[k].ang_sx = st[k]->ang_sx*180/M_PI;
        out.st[k].ang_sz = st[k]->ang_sz*180/M_PI;
        out.st[k].ang_ax = st[k]->ang_ax*180/M_PI;
        out.st[k].ang_az = st[k]->ang_az*180/M_PI;
        out.st[k].ang_pn = st[k]->ang_pn*180/M_PI;
        out.st[k].psih   = st[k]->psih;
        out.st[k].psif   = st[k]->psif;
        out.st[k].Pay      = st[k]->Pay;
        out.st[k].Ppy      = st[k]->Ppy;
        out.st[k].F.x      = FMw[k].Fx;
        out.st[k].F.y      = FMw[k].Fy;
        out.st[k].F.z      = FMw[k].Fz;
        out.st[k].F1.x     = FMw1[k].Fx;
        out.st[k].F1.y     = FMw1[k].Fy;
        out.st[k].F1.z     = FMw1[k].Fz;
        out.st[k].F2.x     = FMw2[k].Fx;
        out.st[k].F2.y     = FMw2[k].Fy;
        out.st[k].F2.z     = FMw2[k].Fz;

        out.st[k].a.status = st[k]->ab->status;
        out.st[k].a.S      = st[k]->ab->S   *1000;
        out.st[k].a.ha     = st[k]->ab->ha  *1000;
        out.st[k].a.ha1    = st[k]->ab->c1.h*1000;
        out.st[k].a.ha2    = st[k]->ab->c2.h*1000;
        out.st[k].a.Pa     = st[k]->ab->Pa ;
        out.st[k].a.Pa1    = st[k]->ab->c1.P;
        out.st[k].a.Pa2    = st[k]->ab->c2.P;
        out.st[k].a.Pg1    = st[k]->ab->c1.Pg;
        out.st[k].a.Ph1    = st[k]->ab->c1.Ph;
        out.st[k].a.Pf1    = st[k]->ab->c1.Pf;
        out.st[k].a.Pg2    = st[k]->ab->c2.Pg;
        out.st[k].a.Ph2    = st[k]->ab->c2.Ph;
        out.st[k].a.Pf2    = st[k]->ab->c2.Pf;
        out.st[k].a.v1     = st[k]->ab->c1.v;
        out.st[k].a.v2     = st[k]->ab->c2.v;
        out.st[k].a.V1     = st[k]->ab->c1.V*1000;
        out.st[k].a.V2     = st[k]->ab->c2.V*1000;
        out.st[k].a.p1     = st[k]->ab->c1.p/98066.5;
        out.st[k].a.p2     = st[k]->ab->c2.p/98066.5;
        out.st[k].a.dh12   = st[k]->ab->dh12*1000;

        out.st[k].p.status = st[k]->pn->status;
        out.st[k].p.S      = st[k]->pn->dpmd*1000;
        out.st[k].p.h.x    = st[k]->pn->h.x *1000;
        out.st[k].p.h.y    = st[k]->pn->h.y *1000;
        out.st[k].p.h.z    = st[k]->pn->h.z *1000;
        out.st[k].p.h1.x   = st[k]->pn->h1.x*1000;
        out.st[k].p.h1.y   = st[k]->pn->h1.y*1000;
        out.st[k].p.h1.z   = st[k]->pn->h1.z*1000;
        out.st[k].p.h2.x   = st[k]->pn->h2.x*1000;
        out.st[k].p.h2.y   = st[k]->pn->h2.y*1000;
        out.st[k].p.h2.z   = st[k]->pn->h2.z*1000;
        out.st[k].p.P.x    = st[k]->pn->P.x;
        out.st[k].p.P.y    = st[k]->pn->P.y;
        out.st[k].p.P.z    = st[k]->pn->P.z;
        out.st[k].p.P1.x   = st[k]->pn->P1.x;
        out.st[k].p.P1.y   = st[k]->pn->P1.y;
        out.st[k].p.P1.z   = st[k]->pn->P1.z;
        out.st[k].p.P2.x   = st[k]->pn->P2.x;
        out.st[k].p.P2.y   = st[k]->pn->P2.y;
        out.st[k].p.P2.z   = st[k]->pn->P2.z;

        xyz sks_Q  =  st[k]->SKo2SKS(st[k]->Q);
        xyz sks_A  =  st[k]->SKo2SKS(st[k]->A);
        xyz sks_B  =  st[k]->SKo2SKS(st[k]->B);
        xyz sks_V  =  st[k]->SKo2SKS(st[k]->V);
        xyz sks_W  =  st[k]->SKo2SKS(st[k]->W);
        xyz sks_D  =  st[k]->SKo2SKS(st[k]->D);
        xyz sks_C  =  st[k]->SKo2SKS(st[k]->C);
        xyz sks_W1 =  st[k]->SKo2SKS(st[k]->W1);
        xyz sks_D1 =  st[k]->SKo2SKS(st[k]->D1);
        xyz sks_C1 =  st[k]->SKo2SKS(st[k]->C1);
        xyz sks_W2 =  st[k]->SKo2SKS(st[k]->W2);
        xyz sks_D2 =  st[k]->SKo2SKS(st[k]->D2);
        xyz sks_C2 =  st[k]->SKo2SKS(st[k]->C2);
        xyz sks_Wfull=st[k]->SKo2SKS(st[k]->Wfull);
        xyz sks_Wfree=st[k]->SKo2SKS(st[k]->Wfree);
        out.st[k].O.x      = st[k]->O.x*1000;
        out.st[k].O.y      = st[k]->O.y*1000;
        out.st[k].O.z      = st[k]->O.z*1000;
        if(st[k]->stype==0)
        {
        out.st[k].A.x      = out.st[k].O.x;
        out.st[k].A.y      = out.st[k].O.y;
        out.st[k].A.z      = out.st[k].O.z;
        }
        else
        {
        out.st[k].A.x      = sks_A.x   *1000;
        out.st[k].A.y      = sks_A.y   *1000;
        out.st[k].A.z      = sks_A.z   *1000;
        }
        out.st[k].Q.x      = sks_Q.x   *1000;
        out.st[k].Q.y      = sks_Q.y   *1000;
        out.st[k].Q.z      = sks_Q.z   *1000;
        out.st[k].B.x      = sks_B.x   *1000;
        out.st[k].B.y      = sks_B.y   *1000;
        out.st[k].B.z      = sks_B.z   *1000;
        out.st[k].C.x      = sks_C.x   *1000;
        out.st[k].C.y      = sks_C.y   *1000;
        out.st[k].C.z      = sks_C.z   *1000;
        out.st[k].D.x      = sks_D.x   *1000;
        out.st[k].D.y      = sks_D.y   *1000;
        out.st[k].D.z      = sks_D.z   *1000;
        out.st[k].V.x      = sks_V.x   *1000;
        out.st[k].V.y      = sks_V.y   *1000;
        out.st[k].V.z      = sks_V.z   *1000;
        out.st[k].W.x      = sks_W.x   *1000;
        out.st[k].W.y      = sks_W.y   *1000;
        out.st[k].W.z      = sks_W.z   *1000;
        out.st[k].C1.x     = sks_C1.x  *1000;
        out.st[k].C1.y     = sks_C1.y  *1000;
        out.st[k].C1.z     = sks_C1.z  *1000;
        out.st[k].D1.x     = sks_D1.x  *1000;
        out.st[k].D1.y     = sks_D1.y  *1000;
        out.st[k].D1.z     = sks_D1.z  *1000;
        out.st[k].W1.x     = sks_W1.x  *1000;
        out.st[k].W1.y     = sks_W1.y  *1000;
        out.st[k].W1.z     = sks_W1.z  *1000;
        out.st[k].C2.x     = sks_C2.x  *1000;
        out.st[k].C2.y     = sks_C2.y  *1000;
        out.st[k].C2.z     = sks_C2.z  *1000;
        out.st[k].D2.x     = sks_D2.x  *1000;
        out.st[k].D2.y     = sks_D2.y  *1000;
        out.st[k].D2.z     = sks_D2.z  *1000;
        out.st[k].W2.x     = sks_W2.x  *1000;
        out.st[k].W2.y     = sks_W2.y  *1000;
        out.st[k].W2.z     = sks_W2.z  *1000;
        out.st[k].Wfull.x  = sks_Wfull.x *1000;
        out.st[k].Wfull.y  = sks_Wfull.y *1000;
        out.st[k].Wfull.z  = sks_Wfull.z *1000;
        out.st[k].Wfree.x  = sks_Wfree.x *1000;
        out.st[k].Wfree.y  = sks_Wfree.y *1000;
        out.st[k].Wfree.z  = sks_Wfree.z *1000;
        out.st[k].saC.x    = st[k]->saC.x*1000;
        out.st[k].saC.y    = st[k]->saC.y*1000;
        out.st[k].saC.z    = st[k]->saC.z*1000;
        out.st[k].saCfree.x= st[k]->Cfree_abs.x *1000;
        out.st[k].saCfree.y= st[k]->Cfree_abs.y *1000;
        out.st[k].saCfree.z= st[k]->Cfree_abs.z *1000;
    }
}



/// Балансировка вертолета

void Helicopter::PreBalance()//подготовка к расчету балансировки вертолета
{
    preout = "";//Balance.Initial:\n
    Pre();
    /*НВ И АВТОМАТ ПЕРЕКОСА
    //zswlmr = 0.5*zl_mr*Sgsh_mr*w_mr*w_mr*lgsh;//вспомогательная величина для расчета сил-моментов от несущего винта
    //double c0 = atan(dgsh/lvsh);//угол определяемый из геометрии втулки НВ, рад
    //double sigma11 = 0.5*M_PI - (asin((ll+dgsh)/Rap)-c0);//угол определяемый из геометрии втулки НВ, рад
    //D_1 = Rap * cos(sigma11-dpsi_cont-c0) / ll;//передаточные числа управления циклическим шагом, отношение между углом установки лопасти и углом отклонения АП
    //D_2 = Rap * sin(sigma11-dpsi_cont-c0) / ll;//передаточные числа управления циклическим шагом, отношение между углом установки лопасти и углом отклонения АП
    //Передаточное отношение между углом отклонения равнодействующей и углом отклонения автомата перекоса
    //D1_mr = (D_1 + k_* D_2) / (1 + k_*k_);
    //D2_mr = (D_2 - k_* D_1) / (1 + k_*k_);
    //D1_mr = D1_mr_;
    //D2_mr = D2_mr_;*/
    if(logkey)//вывод на печать результатов prebalance
    {
        //preout += "\n";
        preout += "mh=           " + ss.setNum(mh               ,'f',1) + " кг   \tвзлетная масса вертолета\n";
        preout += "Hh=           " + ss.setNum(Hh               ,'f',1) + " м      \tвысота полета\n";
        preout += "Vhx=          " + ss.setNum(vn.x             ,'f',1) + " м/с   \tгоризонтальная скорость полета      (СКН)\n";//Vhx
        preout += "Vhy=          " + ss.setNum(vn.y             ,'f',1) + " м/с    \tвертикальная скорость набора высоты (СКН)\n";//Vhy
        preout += "Vhz=          " + ss.setNum(vn.z             ,'f',1) + " м/с    \tскорость бокового перемещения       (СКН)\n";//Vhz
        preout += "v.x=          " + ss.setNum(v.x              ,'f',3) + " м/с  \tпродольная составляющая линейной скорости, м/с   (в СКС);\n";
        preout += "v.y=          " + ss.setNum(v.y              ,'f',3) + " м/с  \tвертикальная составляющая линейной скорости, м/с (в СКС);\n";
        preout += "v.z=          " + ss.setNum(v.z              ,'f',3) + " м/с  \tбоковая составляющая линейной скорости, м/с      (в СКС);\n";
        preout += "w.x=          " + ss.setNum(w.x       *180/pi,'f',3) + " грд/с\tугловая скорость вертолета отн. оX СКC;\n";
        preout += "w.y=          " + ss.setNum(w.y       *180/pi,'f',3) + " грд/с\tугловая скорость вертолета отн. оY СКC;\n";
        preout += "w.z=          " + ss.setNum(w.z       *180/pi,'f',3) + " грд/с\tугловая скорость вертолета отн. оZ СКC;\n";
        preout += "Vi=           " + ss.setNum(Vi               ,'f',3) + " м/с  \tскорость набегающего потока\n";
        preout += "beta=         " + ss.setNum(beta_sk *180/M_PI,'f',3) + " грд  \tугол скольжения\n";
        preout += "tetra=        " + ss.setNum(tetra   *180/M_PI,'f',3) + " грд  \tугол наклона траектории\n";
        preout += "tang_=        " + ss.setNum(tang    *180/M_PI,'f',3) + " грд  \tугол тангажа вертолета\n";
        preout += "kren_=        " + ss.setNum(kren    *180/M_PI,'f',3) + " грд  \tугол крена вертолета\n";
        preout += "ro=           " + ss.setNum(ro               ,'f',3) + " кг/м3\tплотность воздуха (МСА)\n";
        preout += "roh=          " + ss.setNum(roh              ,'f',3) + " кг/м3\tплотность воздуха на высоте Нh\n";
        preout += "p0=           " + ss.setNum(p0               ,'f',3) + " Па   \tдавление воздуха на уровне моря\n";
        preout += "g=            " + ss.setNum(g                ,'f',5) + " м/с2 \tускорение свободного падения\n";
        preout += "Lpt=          " + ss.setNum(Lpt              ,'f',5) + " К/м  \tскорость падения температуры\n";
        preout += "RjmK=         " + ss.setNum(RjmK             ,'f',4) + " Дж/(мол*К)\tгазовая постоянная\n";
        preout += "RjkgK=        " + ss.setNum(RjkgK            ,'f',1) + " Дж/(кг*К) \tгазовая постоянная\n";
        preout += "Mm=           " + ss.setNum(Mm               ,'f',4) + " кг/мол    \tмолярная масса сухого воздуха\n";
        preout += "\nпланер:     \n";
        preout += "l_mr=         " + ss.setNum(l_mr             ,'f',3) + " м   \tрасстояние по оси НВ от втулки до строительной горизонтали\n";
        preout += "eps_fz=       " + ss.setNum(eps_fz  *180/M_PI,'f',3) + " грд \tконструктивный угол установки оси НВ назад от оY СКС (угол заклинения)\n";
        preout += "eps_fx=       " + ss.setNum(eps_fx  *180/M_PI,'f',3) + " грд \tконструктивный угол установки оси НВ влево от оY СКС\n";
        preout += "xctmr=        " + ss.setNum(xctmr            ,'f',3) + " м   \tx-центровка вертолета от оси НВ\n";
        preout += "yctmr=        " + ss.setNum(yctmr            ,'f',3) + " м   \ty-центровка вертолета от КПНВ паралл.оси НВ\n";
        preout += "zctmr=        " + ss.setNum(zctmr            ,'f',3) + " м   \tz-центровка вертолета от оси НВ\n";
        preout += "x_ct=         " + ss.setNum(xct              ,'f',5) + " м   \tположение ЦТ вертолета в СКС\n";
        preout += "y_ct=         " + ss.setNum(yct              ,'f',5) + " м   \tположение ЦТ вертолета в СКС\n";
        preout += "z_ct=         " + ss.setNum(zct              ,'f',5) + " м   \tположение ЦТ вертолета в СКС\n";
        preout += "x_f=          " + ss.setNum(x_f              ,'f',3) + " м   \tточка приложения аэродинамических сил на фюзеляже\n";
        preout += "y_f=          " + ss.setNum(y_f              ,'f',3) + " м   \tточка приложения аэродинамических сил на фюзеляже\n";
        preout += "z_f=          " + ss.setNum(z_f              ,'f',3) + " м   \tточка приложения аэродинамических сил на фюзеляже\n";
        preout += "x_k=          " + ss.setNum(x_k              ,'f',3) + " м   \tточка приложения аэродинамических сил на киле\n";
        preout += "y_k=          " + ss.setNum(y_k              ,'f',3) + " м   \tточка приложения аэродинамических сил на киле\n";
        preout += "z_k=          " + ss.setNum(z_k              ,'f',3) + " м   \tточка приложения аэродинамических сил на киле\n";
        preout += "x_st=         " + ss.setNum(x_st             ,'f',3) + " м   \tточка приложения аэродинамических сил на стабилизаторе\n";
        preout += "y_st=         " + ss.setNum(y_st             ,'f',3) + " м   \tточка приложения аэродинамических сил на стабилизаторе\n";
        preout += "z_st=         " + ss.setNum(z_st             ,'f',3) + " м   \tточка приложения аэродинамических сил на стабилизаторе\n";
        preout += "est_min=      " + ss.setNum(est_min *180/M_PI,'f',3) + " грд  \tдиапазон углов установки стабилизатора (относительно СГ)\n";
        preout += "est_max=      " + ss.setNum(est_max *180/M_PI,'f',3) + " грд  \tдиапазон углов установки стабилизатора (относительно СГ)\n";
        preout += "fi0_min=      " + ss.setNum(fi0_min *180/M_PI,'f',3) + " грд  \tдиапазон общего шага НВ (для угла установки стабилизатора)\n";
        preout += "fi0_max=      " + ss.setNum(fi0_max *180/M_PI,'f',3) + " грд  \tдиапазон общего шага НВ (для угла установки стабилизатора)\n";
        preout += "lmodel=       " + ss.setNum(lmodel           ,'f',4) + " м    \tлинейный размер аэродинамической модели планера\n";
        preout += "Smodel=       " + ss.setNum(Smodel           ,'f',4) + " м2   \tплощадь миделя аэродинамической модели планера\n";
        preout += "Mmodel=       " + ss.setNum(Mmodel           ,'f',4) + "      \tмасштаб аэродинамической модели планера\n";
        preout += "\nнесущий винт:\n";
        preout += "D_mr=         " + ss.setNum(Dmr              ,'f',3) + " м    \tдиаметр несущего винта\n";
        preout += "x_mr=         " + ss.setNum(x_mr             ,'f',3) + " м   \tположение втулки НВ в СКС\n";
        preout += "y_mr=         " + ss.setNum(y_mr             ,'f',3) + " м   \tположение втулки НВ в СКС\n";
        preout += "z_mr=         " + ss.setNum(z_mr             ,'f',3) + " м   \tположение втулки НВ в СКС\n";
        preout += "w_mr=         " + ss.setNum(w_mr             ,'f',3) + " рад/с\tугловая скорость вращения НВ\n";
        preout += "zl_mr=        " + ss.setNum(zl_mr                  ) + "         \tколичество лопастей НВ\n";
        preout += "Sgsh_mr=      " + ss.setNum(Sgsh_mr          ,'f',2) + " кг*м \tстатический момент лопасти отн.ГШ\n";
        preout += "B_mr=         " + ss.setNum(Bmr              ,'f',3) + "      \tкоэф. концевых потерь\n";
        preout += "k_=           " + ss.setNum(k_               ,'f',3) + "      \tкоэф. компенсатора взмаха\n";
        preout += "D1=           " + ss.setNum(D1_mr            ,'f',6) + "    \tпередат.отношение между наклоном конуса НВ и отклонением АП в продольном канале\n";
        preout += "D2=           " + ss.setNum(D2_mr            ,'f',6) + "    \tпередат.отношение между наклоном конуса НВ и отклонением АП в поперечном канале\n";
        preout += "chi_hub_=     " + ss.setNum(chi_hub_*180/M_PI,'f',4) + " грд \tзаданный наклон оси НВ в продольном канале (отн.линии ОСКС-НВ)\n";/////////////////
        preout += "eta_hub_=     " + ss.setNum(eta_hub_*180/M_PI,'f',4) + " грд \tзаданный наклон оси НВ в поперечном канале (отн.линии ОСКС-НВ)\n";/////////////////
        preout += "chi_ap_=      " + ss.setNum(chi_ap_ *180/M_PI,'f',4) + " грд \tзаданный угол отклонения АП в продольном канале\n";
        preout += "eta_ap_=      " + ss.setNum(eta_ap_ *180/M_PI,'f',4) + " грд \tзаданный угол отклонения АП в поперечном канале\n";
        preout += "Fx_mr_=       " + ss.setNum(Fx_mr_           ,'f',1) + " Н     \tпродольная сила  (по оси X CKНВ)\n";
        preout += "Fy_mr_=       " + ss.setNum(Fy_mr_           ,'f',1) + " Н     \tтяга винта       (по оси Y CKНВ)\n";
        preout += "Fz_mr_=       " + ss.setNum(Fz_mr_           ,'f',1) + " Н     \tпоперечная сила  (по оси Z CKНВ)\n";
        preout += "Mx_mr_=       " + ss.setNum(Mx_mr_           ,'f',1) + " Н·м   \tсуммарный момент отн.оси X CKНВ \n";
        preout += "My_mr_=       " + ss.setNum(My_mr_           ,'f',1) + " Н·м   \tкрутящий момент  отн.оси Y CKНВ \n";
        preout += "Mz_mr_=       " + ss.setNum(Mz_mr_           ,'f',1) + " Н·м   \tсуммарный момент отн.оси Z CKНВ \n";
        preout += "alfa_mr=      " + ss.setNum(alfa_mr *180/M_PI,'f',3) + " грд \tугол атаки КПНВ\n";
        if(trmount)
        {
        preout += "\nрулевой винт:\n";
        preout += "D_tr=         " + ss.setNum(Dtr             ,'f',3) + " м    \tдиаметр рулевого винта\n";
        preout += "x_tr=         " + ss.setNum(x_tr            ,'f',3) + " м   \tположение втулки РВ в СКС\n";
        preout += "y_tr=         " + ss.setNum(y_tr            ,'f',3) + " м   \tположение втулки РВ в СКС\n";
        preout += "z_tr=         " + ss.setNum(z_tr            ,'f',3) + " м   \tположение втулки РВ в СКС\n";
        preout += "Fx_tr_=       " + ss.setNum(Fx_tr_          ,'f',1) + " Н     \tпродольная сила   (по оси X CKРВ)\n";
        preout += "Fy_tr_=       " + ss.setNum(Fy_tr_          ,'f',1) + " Н     \tтяга винта        (по оси Y CKРВ)\n";
        preout += "Fz_tr_=       " + ss.setNum(Fz_tr_          ,'f',1) + " Н     \tвертикальная сила (по оси Z CKРВ)\n";
        preout += "Mx_tr_=       " + ss.setNum(Mx_tr_          ,'f',1) + " Н·м   \tсуммарный момент  отн.оси X CKРВ \n";
        preout += "My_tr_=       " + ss.setNum(My_tr_          ,'f',1) + " Н·м   \tкрутящий момент   отн.оси Y CKРВ \n";
        preout += "Mz_tr_=       " + ss.setNum(Mz_tr_          ,'f',1) + " Н·м   \tсуммарный момент  отн.оси Z CKРВ \n";
        preout += "alfa_tr=      " + ss.setNum(alfa_tr*180/M_PI,'f',3) + " грд \tугол атаки КПРВ\n";
        }
        if(prmount)
        {
        preout += "\nтолкающий винт:\n";
        preout += "D_pr=         " + ss.setNum(Dpr             ,'f',3) + " м    \tдиаметр толкающего винта\n";
        preout += "x_pr=         " + ss.setNum(x_pr            ,'f',3) + " м   \tположение втулки ТВ в СКС\n";
        preout += "y_pr=         " + ss.setNum(y_pr            ,'f',3) + " м   \tположение втулки ТВ в СКС\n";
        preout += "z_pr=         " + ss.setNum(z_pr            ,'f',3) + " м   \tположение втулки ТВ в СКС\n";
        preout += "Fx_pr_=       " + ss.setNum(Fx_pr_          ,'f',1) + " Н     \tбоковая сила      (по оси X CKТВ)\n";
        preout += "Fy_pr_=       " + ss.setNum(Fy_pr_          ,'f',1) + " Н     \tтяга винта        (по оси Y CKТВ)\n";
        preout += "Fz_pr_=       " + ss.setNum(Fz_pr_          ,'f',1) + " Н     \tвертикальная сила (по оси Z CKТВ)\n";
        preout += "Mx_pr_=       " + ss.setNum(Mx_pr_          ,'f',1) + " Н·м   \tсуммарный момент  отн.оси X CKТВ \n";
        preout += "My_pr_=       " + ss.setNum(My_pr_          ,'f',1) + " Н·м   \tкрутящий момент   отн.оси Y CKТВ \n";
        preout += "Mz_pr_=       " + ss.setNum(Mz_pr_          ,'f',1) + " Н·м   \tсуммарный момент  отн.оси Z CKТВ \n";
        preout += "alfa_pr=      " + ss.setNum(alfa_pr*180/M_PI,'f',3) + " грд \tугол атаки КПТВ\n";
        }
        /*/preout +=  "\n   аэродинамические характеристики:\n";
        //if(logkey>1)
        toLogf("__cx_st:");
        QString sss="";
        int zz;
        for (int xx = 0; xx < hc->__cx_st_row; xx++)
        {
            for (int yy = 0; yy < hc->__cx_st_col; yy++)
            {
                zz = yy + xx* hc->__cx_st_col;
                sss += "\t";
                sss += ss.setNum(hc->__cx_st[zz],'f',3);
            }
            sss += "\n";
        }
        preout += sss;*/
    }
}
int  Helicopter::SolveBalance0()//0-я итерация расчетa балансировки
{
    solvout = "";
    int error = 0;
    //ВЕС ВЕРТОЛЕТА и ГРУЗА НА ВНЕШНЕЙ ТРОСОВОЙ ПОДВЕСКЕ
    HelicopterGravity(tang,kren);
    OutboardCargo(tang,kren);
    //СИЛЫ/МОМЕНТЫ ОТ толкающего винта
    if(prmount) PushRotorForceCorrection(Fy_pr_);
    //угол атаки фюзеляжа
    alfa_f = alfa_mr - chi_hub;//- eps_fz  < очень приближенно
    //Силы и моменты от фюзеляжа (СКC)
    if(solvePlaner)
    {
        //скорость потока у фюзеляжа
        V_f = Vi;
        if (V_f == 0 || (alfa_f>1.40 && alfa_f<1.75) || (alfa_f>-1.75 && alfa_f<-1.4))//FLIGHTMODE = AXIAL;
        {
            Fx_f = 0;
            Fy_f = 0;
            Fx_st= 0;
            Fy_st= 0;
        }
        else
        {
            //Скоростной напор в расч.точке у фюзеляжа
            q_f = 0.5*roh*V_f*V_f;

            skos_f_mr = 5 *d2r;
            //аэродинамические коэф.фюзеляжа в скоростной СК
            cxaf = f_cx_f(alfa_f-skos_f_mr);
            cyaf = f_cy_f(alfa_f-skos_f_mr);
            czaf = f_cz_f(alfa_f-skos_f_mr);
            //поворот скоростной СК на угол alfa_f
            cx_f = cxaf*cos(alfa_f-skos_f_mr) - cyaf*sin(alfa_f-skos_f_mr);
            cy_f = cyaf*cos(alfa_f-skos_f_mr) + cxaf*sin(alfa_f-skos_f_mr);
            cz_f = czaf;
            //аэродин.сила+момент от фюзеляжа (без стабилизатора) в локальной СК ||СКC
            Fx_f =-cx_f *q_f *Smod;
            Fy_f = cy_f *q_f *Smod;
            Fz_f = cz_f *q_f *Smod;

            if(autogyro)
                alfa_st = alfa_f - skos_f_mr - f_dalf_stf(alfa_f);
            else
                alfa_st = 2.5* alfa_f;//очень приближенно
            //аэродинамические коэф.стабилизатора в скоростной СК
            cxast = f_cx_st(alfa_st);
            cyast = f_cy_st(alfa_st);
            //аэродинамические коэф. в СК, связанной с профилем стабилизатора
            cx_st = cxast*cos(alfa_st) - cyast*sin(alfa_st);
            cy_st = cyast*cos(alfa_st) + cxast*sin(alfa_st);
            //аэродин.сила на стабилизаторе в СК, связанной с профилем стабилизатора
            X_st = q_f *cx_st *S_st *K_st;
            Y_st = q_f *cy_st *S_st *K_st;
            //аэродин.сила на оси стабилизатора (в СКC)
            Fx_st =-X_st;
            Fy_st = Y_st;
        }
    }
    //аэродинамическая сила от планера
    FMadp.Fx = Fx_f + Fx_st;
    FMadp.Fy = Fy_f + Fy_st;
    //суммы проекций сил без НВ (в связанной СК)
    fx = FMhg.Fx + FMcg.Fx + FMadp.Fx + FMtr.Fx + FMpr.Fx;
    fy = FMhg.Fy + FMcg.Fy + FMadp.Fy + FMtr.Fy + FMpr.Fy;
    //потребная сила на втулке НВ
    Xa_need = Xa_need_ =  fx*cos(tang-tetra) -fy*sin(tang-tetra);
    Ya_need = Ya_need_ = -fy*cos(tang-tetra) -fx*sin(tang-tetra);
    Ra_need = sqrt(Xa_need*Xa_need + Ya_need*Ya_need);
    //лобовое сопротивление вертолета, м2
    CxS = ( -(fx-FMpr.Fx)*cos(tang-tetra) + (fy-FMpr.Fy)*sin(tang-tetra) ) *2/roh/Vxy/Vxy;
    //потребная тяга рулевого винта
    Ttr_need = (Fy_tr_!=0) ? (Fy_tr_) : (My_mr_/x_tr);
    if(autogyro) Ttr_need = 0;
    //solvout
    solvout += "       tetra=   " + ss.setNum(tetra  *180/M_PI,'f',3) + " грд\n";
    solvout += "        tang=   " + ss.setNum(tang   *180/M_PI,'f',3) + " грд\n";
    solvout += "      alfa_f=   " + ss.setNum(alfa_f *180/M_PI,'f',3) + " грд\n";
    solvout += "         CxS=   " + ss.setNum(CxS             ,'f',4) + " м2\n";
    solvout += "       Fx.пл=   " + ss.setNum(FMadp.Fx        ,'f',1) + " Н\n";
    solvout += "       Fy.пл=   " + ss.setNum(FMadp.Fy        ,'f',1) + " Н\n";
    solvout += "        Xa.п=   " + ss.setNum(Xa_need_        ,'f',1) + " Н\n";
    solvout += "        Ya.п=   " + ss.setNum(Ya_need_        ,'f',1) + " Н\n";
    solvout += "       Ttr.п=   " + ss.setNum(Ttr_need        ,'f',1) + " Н\n";

return error;
}
double Helicopter::get_dMkr_autogyro(double Tpr_)//определение разницы между рассчитанным крут.моментом на валу НВ и заданным
{
    alfa_mr = org.t + eps_fz + chi_hub - tetra;//сбрасываем угол атаки до заданного по tang,eps,tetra
    alfa_f = org.t - tetra;
    //приведение усилий на втулке ТВ к ЦТ вертолета
    PushRotorForceCorrection(Tpr_);
    /*//Силы и моменты от фюзеляжа (СКC)
    if(solvePlaner)
    {
        //скорость потока у фюзеляжа
        V_f = Vi;
        if (V_f == 0 || (alfa_f>1.40 && alfa_f<1.75) || (alfa_f>-1.75 && alfa_f<-1.4))//FLIGHTMODE = AXIAL;
        {
            Fx_f = 0;
            Fy_f = 0;
            Fx_st= 0;
            Fy_st= 0;
        }
        else
        {
            //Скоростной напор в расч.точке у фюзеляжа
            q_f = 0.5*roh*V_f*V_f;
            //аэродинамические коэф.фюзеляжа в скоростной СК
            cxaf = f_cx_f(alfa_f);
            cyaf = f_cy_f(alfa_f);
            czaf = f_cz_f(alfa_f);
            //поворот скоростной СК на угол alfa_f
            cx_f = cxaf*cos(alfa_f) - cyaf*sin(alfa_f);
            cy_f = cyaf*cos(alfa_f) + cxaf*sin(alfa_f);
            cz_f = czaf;
            //аэродин.сила+момент от фюзеляжа (без стабилизатора) в локальной СК ||СКC
            Fx_f =-cx_f *q_f *Smod;
            Fy_f = cy_f *q_f *Smod;
            Fz_f = cz_f *q_f *Smod;

            if(autogyro)
                alfa_st = 0;
            else
                alfa_st = 2.5* alfa_f;//очень приближенно
            //аэродинамические коэф.стабилизатора в скоростной СК
            cxast = f_cx_st(alfa_st);
            cyast = f_cy_st(alfa_st);
            //аэродинамические коэф. в СК, связанной с профилем стабилизатора
            cx_st = cxast*cos(alfa_st) - cyast*sin(alfa_st);
            cy_st = cyast*cos(alfa_st) + cxast*sin(alfa_st);
            //аэродин.сила на стабилизаторе в СК, связанной с профилем стабилизатора
            X_st = q_f *cx_st *S_st *K_st;
            Y_st = q_f *cy_st *S_st *K_st;
            //аэродин.сила на оси стабилизатора (в СКC)
            Fx_st =-X_st;
            Fy_st = Y_st;
        }
    }*/
    //аэродинамическая сила от планера
    FMadp.Fx = Fx_f + Fx_st;
    FMadp.Fy = Fy_f + Fy_st;
    //суммы проекций сил без НВ (в связанной СК)
    fx = FMhg.Fx + FMcg.Fx + FMadp.Fx + FMpr.Fx;
    fy = FMhg.Fy + FMcg.Fy + FMadp.Fy + FMpr.Fy;
    //потребная сила на втулке НВ
    Xa_need_ =  fx*cos(alfa_f) -fy*sin(alfa_f);
    Ya_need_ = -fy*cos(alfa_f) -fx*sin(alfa_f);
    //расчет эквивалентного винта
    if(mrtilt) mr->ClassicEquivalent( Xa_need_, Ya_need_, 0, 0 );
    else       mr->ClassicEquivalent( Xa_need_, Ya_need_, eta_ap_, chi_ap_);
    mr->ClassicForceSynth();
    //угол атаки КПНВ
    alfa_mr = mr->alpha;
    //вычисление разницы с заданным крут.моментом на валу НВ
    return  mr->My_sr - Mkr_;
}
double Helicopter::get_dMkr(double vy)//определение разницы между рассчитанным крут.моментом на валу НВ и заданным
{
    //параметры набегающего потока
    vn.y = vy;//новое значение верт.скорости в земной СК
    v_.x = vn.x;  v_.y = vn.y;  v_.z = vn.z;
    v = SKNtoSKS(v_);//скорость потока в СКС
    inFlow();
    mr->V = Vi;
    //основные углы
    tang = alfa_mr - eps_fz - chi_hub + tetra;
    alfa_f = tang - tetra;
    //Силы и моменты от фюзеляжа (СКC)
    if(solvePlaner)
    {
        //скорость потока у фюзеляжа
        V_f = Vi;
        if (V_f == 0 || (alfa_f>1.40 && alfa_f<1.75) || (alfa_f>-1.75 && alfa_f<-1.4))//FLIGHTMODE = AXIAL;
        {
            Fx_f = 0;
            Fy_f = 0;
        }
        else
        {
            //Скоростной напор в расч.точке у фюзеляжа
            q_f = 0.5*roh*V_f*V_f;
            //аэродинамические коэф.фюзеляжа в скоростной СК
            cxaf = f_cx_f(alfa_f);
            cyaf = f_cy_f(alfa_f);
            czaf = f_cz_f(alfa_f);
            //поворот скоростной СК на угол alfa_f
            cx_f = cxaf*cos(alfa_f) - cyaf*sin(alfa_f);
            cy_f = cyaf*cos(alfa_f) + cxaf*sin(alfa_f);
            cz_f = czaf;
            //аэродин.сила+момент от фюзеляжа (без стабилизатора) в локальной СК ||СКC
            Fx_f =-cx_f *q_f *Smod;
            Fy_f = cy_f *q_f *Smod;
            Fz_f = cz_f *q_f *Smod;

            if(autorot)
                alfa_st = 0;
            else
                alfa_st = 2.5* alfa_f;//очень приближенно
            //аэродинамические коэф.стабилизатора в скоростной СК
            cxast = f_cx_st(alfa_st);
            cyast = f_cy_st(alfa_st);
            //аэродинамические коэф. в СК, связанной с профилем стабилизатора
            cx_st = cxast*cos(alfa_st) - cyast*sin(alfa_st);
            cy_st = cyast*cos(alfa_st) + cxast*sin(alfa_st);
            //аэродин.сила на стабилизаторе в СК, связанной с профилем стабилизатора
            X_st = q_f *cx_st *S_st *K_st;
            Y_st = q_f *cy_st *S_st *K_st;
            //аэродин.сила на оси стабилизатора (в СКC)
            Fx_st =-X_st;
            Fy_st = Y_st;
        }
    }
    //аэродинамическая сила от планера
    FMadp.Fx = Fx_f;
    FMadp.Fy = Fy_f;
    //проекции сил гравитации и сопротивления фюзеляжа
    fx = FMhg.Fx + FMcg.Fx + FMadp.Fx + FMpr.Fx;
    fy = FMhg.Fy + FMcg.Fy + FMadp.Fy + FMpr.Fy;
    //потребная сила на втулке НВ
    Xa_need_ =  fx*cos(alfa_f) -fy*sin(alfa_f);
    Ya_need_ = -fy*cos(alfa_f) -fx*sin(alfa_f);
    //расчет эквивалентного винта
    if(mrtilt) mr->ClassicEquivalent( Xa_need_, Ya_need_, 0, 0 );
    else       mr->ClassicEquivalent( Xa_need_, Ya_need_, eta_ap_, chi_ap_);
    mr->ClassicForceSynth();
    //угол атаки КПНВ
    alfa_mr = mr->alpha;
    //вычисление разницы с заданным крут.моментом на валу НВ
    return  mr->My_sr - Mkr_;
}

int  Helicopter::SolveBalance(outstructure &out)//основной расчет балансировки вертолета
{
    int error=0;  solvout="";  solvout2="";  dchi=0;  deta=0;   quat seq4;
    //расчет винтов
    if(mrsolve)
    {
        if(mrtilt) mr->ClassicEquivalent( Xa_need_, Ya_need_, 0, 0 );
        else       mr->ClassicEquivalent( Xa_need_, Ya_need_, eta_ap_, chi_ap_);
        mr->ClassicForceSynth();
        alfa_mr = mr->alpha ;//угол атаки КПНВ
        fi0_mr  = mr->fi0   ;//не используется в расчете
        rpm_mr  = mr->rpm   ;
        Fx_mr_  = mr->Fhx_sr;//mr->H_sr  ;//
        Fy_mr_  = mr->Fhy_sr;//mr->T_sr  ;//
        Fz_mr_  = mr->Fhz_sr;//mr->S_sr  ;//
        Mx_mr_  = mr->Mhx_sr;//mr->Mx_sr ;//
        My_mr_  = mr->Mhy_sr;//mr->My_sr ;//
        Mz_mr_  = mr->Mhz_sr;//mr->Mz_sr ;//
        Ra      = mr->Ra    ;
        Xa      = mr->Xa    ;
        Ya      = mr->Ya    ;
    }
    if(trsolve)
    {
        tr->ClassicEquivalent( alfa_tr, Ttr_need );
        tr->ClassicForceSynth();
        fi0_tr  = tr->fi0   ;
        rpm_tr  = tr->rpm   ;
        Fx_tr_  = tr->Fhx_sr;//tr->H_sr  ;//
        Fy_tr_  = tr->Fhy_sr;//tr->T_sr  ;//
        Fz_tr_  = tr->Fhz_sr;//tr->S_sr  ;//
        Mx_tr_  = tr->Mhx_sr;//tr->Mx_sr ;//
        My_tr_  = tr->Mhy_sr;//tr->My_sr ;//
        Mz_tr_  = tr->Mhz_sr;//tr->Mz_sr ;//
    }
    if(prsolve)
    {
        pr->ClassicEquivalent( alfa_pr, Tpr_need );
        pr->ClassicForceSynth();
        fi0_pr  = pr->fi0   ;
        rpm_pr  = pr->rpm   ;
        Fx_pr_  = pr->Fhx_sr;//pr->H_sr  ;//
        Fy_pr_  = pr->Fhy_sr;//pr->T_sr  ;//
        Fz_pr_  = pr->Fhz_sr;//pr->S_sr  ;//
        Mx_pr_  = pr->Mhx_sr;//pr->Mx_sr ;//
        My_pr_  = pr->Mhy_sr;//pr->My_sr ;//
        Mz_pr_  = pr->Mhz_sr;//pr->Mz_sr ;//
    }
    //решатель балансировки
    if(autogyro) seq4 = SolveSystemNL4ag();
    else         seq4 = SolveSystemNL4();
    //результат решения системы
    dchi  = seq4.a;
    deta  = seq4.b;
    kren  = seq4.c;
    rysk  = 0;
    ocw.k = org.k = kren;
    ocw.r = org.r = rysk;
    //потребная тяга рулевого/толкающего винта
    if(autogyro) Tpr_need = seq4.d;
    else         Ttr_need = seq4.d;
    //скорректированные углы наклона АП
    chi_ap  = chi_ap_ + dchi;
    eta_ap  = eta_ap_ + deta;
    //суммы проекций сил (в связанной СК)
    Fx = b_Fx();    Mx = b_Mx();
    Fy = b_Fy();    My = b_My();
    Fz = b_Fz();    Mz = b_Mz();
    //суммы проекций сил без НВ (в связанной СК)
    fx = FMhg.Fx + FMcg.Fx + FMadp.Fx + FMtr.Fx + FMpr.Fx;
    fy = FMhg.Fy + FMcg.Fy + FMadp.Fy + FMtr.Fy + FMpr.Fy;
    //проекции потребной силы в скоростной СК
    Xa_need =  fx*cos(tang-tetra) -fy*sin(tang-tetra);
    Ya_need = -fy*cos(tang-tetra) -fx*sin(tang-tetra);
    //потребная сила в продольной плоскости вертолета
    Ra_need = sqrt(Xa_need*Xa_need + Ya_need*Ya_need);
    //лобовое сопротивление вертолета, м2
    CxS = ( -(fx-FMpr.Fx)*cos(tang-tetra) + (fy-FMpr.Fy)*sin(tang-tetra) ) *2/roh/Vxy/Vxy;
    //погрешности расчета
    if(autogyro) errTtr = ( Tpr_need )?( 100*fabs((Tpr_need - Fy_pr_)/Tpr_need) ):( 0 );
    else         errTtr = ( Ttr_need )?( 100*fabs((Ttr_need - Fy_tr_)/Ttr_need) ):( 0 );
    Xa = ( fabs(Xa_need)<99 )? ( Xa_need ):( Xa );
    if(mrsolve)
    {
        errFx  = (Ra     )?( 100*fabs( Fx / Ra               ) ):(0);
        errFy  = (Ra     )?( 100*fabs( Fy / Ra               ) ):(0);
        errRan = (Ra_need)?( 100*fabs( (Ra - Ra_need) / Ra_need  ) ):(0);
        errXan = (Xa_need)?( 100*fabs( (Xa - Xa_need) / Xa_need  ) ):(0);
        errYan = (Ya_need)?( 100*fabs( (Ya - Ya_need) / Ya_need  ) ):(0);
        if(mrtilt)
        {
        errChi = ( chi_hub )?( 100*fabs( (chi_hub - chi_hub_) / chi_hub  ) ):(0);
        errEta = ( eta_hub )?( 100*fabs( (eta_hub - eta_hub_) / eta_hub  ) ):(0);
        eta_ap = 0;
        chi_ap = 0;
        }
        else
        {
        errChi = (chi_ap )?( 100*fabs((chi_ap - chi_ap_) / chi_ap) ):(0);
        errEta = (eta_ap )?( 100*fabs((eta_ap - eta_ap_) / eta_ap) ):(0);
        }
    }
    else
    {
        errRan = 0;
        Ra     = sqrt( pow(FMmr.Fx,2) + pow(FMmr.Fy,2) + pow(FMmr.Fz,2) );
        Ya     =  Fy_mr*cos(alfa_mr) -Fx_mr*sin(alfa_mr);
        Xa     = -Fy_mr*sin(alfa_mr) -Fx_mr*cos(alfa_mr);
        errFx  = ( Ra )? ( 100*fabs(Fx/Ra) ): ( 0 );
        errFy  = ( Ra )? ( 100*fabs(Fy/Ra) ): ( 0 );
        errXan = 0;//(Xa_need)?( 100* fabs( (Xa_need - Xa_need_) / Xa_need) ):(0);
        errYan = 0;//(Ya_need)?( 100* fabs( (Ya_need - Ya_need_) / Ya_need) ):(0);
        errChi = 0;//(chi_ap_)?( 100* fabs( (chi_ap - chi_ap_) / chi_ap_) ):(0);
        errEta = 0;//(eta_ap_)?( 100* fabs( (eta_ap - eta_ap_) / eta_ap_) ):(0);
    }
    //критерии сбалансированности вертолета
    if ( errXan <= precXan && errYan <= precYan && errChi <= precChi && errEta <= precEta )  balanced=1;
    //установка значений для следующей итерации
    eta_hub_  = eta_hub;     chi_hub_  = chi_hub;
    chi_ap_ = chi_ap;    eta_ap_ = eta_ap;
    Xa_need_= Xa_need;   Ya_need_= Ya_need;
    //вывод результатов расчета текущей итерации
    {
        solvout +="   [ %chi="+ss.setNum(errChi   ,'f',3);
        solvout += "  %eta=" + ss.setNum(errEta   ,'f',3);
        solvout += "  %Xa="  + ss.setNum(errXan   ,'f',3);
        solvout += "  %Ya="  + ss.setNum(errYan   ,'f',3);
        solvout += "  %Ttr=" + ss.setNum(errTtr   ,'f',3);
        solvout += "  %Fx="  + ss.setNum(errFx    ,'f',3);
        solvout += "  %Fy="  + ss.setNum(errFy    ,'f',3);
        solvout += "  %Ra="  + ss.setNum(errRan   ,'f',3) + " ]\n";
        solvout +=  "        tang=   " + ss.setNum(tang    *180/M_PI,'f',4) + " грд\n";
        solvout +=  "        kren=   " + ss.setNum(kren    *180/M_PI,'f',4) + " грд\n";
        solvout +=  "     alfa.mr=   " + ss.setNum(alfa_mr *180/M_PI,'f',4) + " грд\n";
        solvout +=  "     chi.hub=   " + ss.setNum(chi_hub *180/M_PI,'f',4) + " грд\n";
        solvout +=  "     eta.hub=   " + ss.setNum(eta_hub *180/M_PI,'f',4) + " грд\n";
        solvout +=  "      chi.ap=   " + ss.setNum(chi_ap  *180/M_PI,'f',4) + " грд   \t";  solvout += "(dchi=   " + ss.setNum(dchi *180/M_PI,'f',4) + ")\n";
        solvout +=  "      eta.ap=   " + ss.setNum(eta_ap  *180/M_PI,'f',4) + " грд   \t";  solvout += "(deta=   " + ss.setNum(deta *180/M_PI,'f',4) + ")\n";
        solvout +=  "      fi0.mr=   " + ss.setNum(fi0_mr  *180/M_PI,'f',4) + " грд   \t";  solvout += "rpm_mr=  " + ss.setNum(rpm_mr  ,'f',1) + " об/мин\n";
        solvout +=  "       Xa.mr=   " + ss.setNum(Xa               ,'f',1) + " Н     \t";  solvout += " Xa.п=   " + ss.setNum(Xa_need ,'f',1) + " Н\n";
        solvout +=  "       Ya.mr=   " + ss.setNum(Ya               ,'f',1) + " Н     \t";  solvout += " Ya.п=   " + ss.setNum(Ya_need ,'f',1) + " Н\n";
        if(trmount)
        {
        solvout +=  "        T.tr=   " + ss.setNum(Fy_tr_           ,'f',1) + " Н     \t";  solvout += "Ttr.п=   " + ss.setNum(Ttr_need,'f',1) + " Н\n";
        solvout +=  "      fi0.tr=   " + ss.setNum(fi0_tr  *180/M_PI,'f',4) + " грд   \t";  solvout += "rpm_tr=  " + ss.setNum(rpm_tr  ,'f',1) + " об/мин\n";
        }
        if(prmount)
        {
        solvout +=  "        T.pr=   " + ss.setNum(Fy_pr            ,'f',1) + " Н     \t";  solvout += "Tpr.п=   " + ss.setNum(Tpr_need,'f',1) + " Н\n";
        solvout +=  "      fi0.pr=   " + ss.setNum(fi0_pr  *180/M_PI,'f',4) + " грд   \t";  solvout += "rpm_pr=  " + ss.setNum(rpm_pr  ,'f',1) + " об/мин\n";
        }
        ///solvout2
        solvout += "\t\tFx, Н\t\tFy, Н\t\tFz, Н\t\tMx, Н·м\t\tMy, Н·м\t\tMz, Н·м\n";
        solvout2 += "gravity   \t" + ss.setNum(FMhg.Fx  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMhg.Fy  ,'f',0) + "      \t";
        solvout2 +=                  ss.setNum(FMhg.Fz  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMhg.Mx  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMhg.My  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMhg.Mz  ,'f',1) + "      \n";
        if(mcg)
        {
        solvout2 += "cargo     \t" + ss.setNum(FMcg.Fx  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMcg.Fy  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMcg.Fz  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMcg.Mx  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMcg.My  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMcg.Mz  ,'f',1) + "      \n";
        }
        solvout2 += "mainrotor \t" + ss.setNum(FMmr.Fx  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMmr.Fy  ,'f',0) + "      \t";
        solvout2 +=                  ss.setNum(FMmr.Fz  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMmr.Mx  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMmr.My  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMmr.Mz  ,'f',1) + "      \n";
        if(trmount)
        {
        solvout2 += "tailrotor \t" + ss.setNum(FMtr.Fx  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMtr.Fy  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMtr.Fz  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMtr.Mx  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMtr.My  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMtr.Mz  ,'f',1) + "      \n";
        }
        if(prmount)
        {
        solvout2 += "pushrotor \t" + ss.setNum(FMpr.Fx  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMpr.Fy  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMpr.Fz  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMpr.Mx  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMpr.My  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMpr.Mz  ,'f',1) + "      \n";
        }
        solvout2 += "fuselage  \t" + ss.setNum(FMadf.Fx ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadf.Fy ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadf.Fz ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadf.Mx ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadf.My ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadf.Mz ,'f',1) + "      \n";
        solvout2 += "wing      \t" + ss.setNum(FMadw.Fx ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadw.Fy ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadw.Fz ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadw.Mx ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadw.My ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadw.Mz ,'f',1) + "      \n";
        solvout2 += "stabilizer\t" + ss.setNum(FMads.Fx ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMads.Fy ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMads.Fz ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMads.Mx ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMads.My ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMads.Mz ,'f',1) + "      \n";
        solvout2 += "keel      \t" + ss.setNum(FMadk.Fx ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadk.Fy ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadk.Fz ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadk.Mx ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadk.My ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadk.Mz ,'f',1) + "\n";
        solvout  += solvout2;
        solvout  += "summary   \t" + ss.setNum(Fx,'f',1) + "      \t";
        solvout  +=                  ss.setNum(Fy,'f',1) + "      \t";
        solvout  +=                  ss.setNum(Fz,'f',1) + "      \t";
        solvout  +=                  ss.setNum(Mx,'f',1) + "      \t";
        solvout  +=                  ss.setNum(My,'f',1) + "      \t";
        solvout  +=                  ss.setNum(Mz,'f',1) + "\n";
    }
    fillOut(out);
    if(mrsolve) mr->fillOutAz(out, 0);
    if(trsolve) tr->fillOutAz(out, 0);
    if(prsolve) pr->fillOutAz(out, 0);
return error;
}

int  Helicopter::SolveSpecified(outstructure &out)//расчет суммарных силы-момента в ЦТ вертолета
{
    int error=0;   solvout2="";   dchi=0;   deta=0;    balanced = 0;
    chi_ap = chi_ap_ + dchi;
    eta_ap = eta_ap_ + deta;

    //расчет винтов
    if(mrsolve)
    {
        if(mrtilt) mr->ClassicSpecified( alfa_mr, manual.fi0mr, 0, 0 );
        else       mr->ClassicSpecified( alfa_mr, manual.fi0mr, manual.etamr, manual.chimr );
        mr->ClassicForceSynth();
        alfa_mr = mr->alpha ;//угол атаки КПНВ
        fi0_mr  = mr->fi0   ;//для справки, не используется в расчете.
        rpm_mr  = mr->rpm   ;
        Fx_mr_  = mr->Fhx_sr;//mr->H_sr  ;//
        Fy_mr_  = mr->Fhy_sr;//mr->T_sr  ;//
        Fz_mr_  = mr->Fhz_sr;//mr->S_sr  ;//
        Mx_mr_  = mr->Mhx_sr;//mr->Mx_sr ;//
        My_mr_  = mr->Mhy_sr;//mr->My_sr ;//
        Mz_mr_  = mr->Mhz_sr;//mr->Mz_sr ;//
        Ra      = mr->Ra    ;
        Xa      = mr->Xa    ;
        Ya      = mr->Ya    ;
    }
    if(trsolve)
    {
        tr->ClassicSpecified( alfa_tr, manual.fi0tr );
        tr->ClassicForceSynth();
        fi0_tr  = tr->fi0   ;
        rpm_tr  = tr->rpm   ;
        Fx_tr_  = tr->Fhx_sr;//tr->H_sr  ;//
        Fy_tr_  = tr->Fhy_sr;//tr->T_sr  ;//
        Fz_tr_  = tr->Fhz_sr;//tr->S_sr  ;//
        Mx_tr_  = tr->Mhx_sr;//tr->Mx_sr ;//
        My_tr_  = tr->Mhy_sr;//tr->My_sr ;//
        Mz_tr_  = tr->Mhz_sr;//tr->Mz_sr ;//
    }
    if(prsolve)
    {
        pr->ClassicSpecified( alfa_pr, manual.fi0pr );
        pr->ClassicForceSynth();
        fi0_pr  = pr->fi0   ;
        rpm_pr  = pr->rpm   ;
        Fx_pr_  = pr->Fhx_sr;//pr->H_sr  ;//
        Fy_pr_  = pr->Fhy_sr;//pr->T_sr  ;//
        Fz_pr_  = pr->Fhz_sr;//pr->S_sr  ;//
        Mx_pr_  = pr->Mhx_sr;//pr->Mx_sr ;//
        My_pr_  = pr->Mhy_sr;//pr->My_sr ;//
        Mz_pr_  = pr->Mhz_sr;//pr->Mz_sr ;//
    }

    //расчет усилий от основных элементов вертолета
    if(autogyro)
        HeliBalanceDynamic( chi_hub, eta_hub, kren, Fy_pr_);
    else
        HeliBalanceDynamic( chi_hub, eta_hub, kren, Fy_tr_);

    //суммы проекций сил (в связанной СК)
    Fx = b_Fx();    Mx = b_Mx();
    Fy = b_Fy();    My = b_My();
    Fz = b_Fz();    Mz = b_Mz();
    //суммы проекций сил без НВ
    fx = FMhg.Fx + FMcg.Fx + FMtr.Fx + FMpr.Fx + FMadp.Fx;
    fy = FMhg.Fy + FMcg.Fy + FMtr.Fy + FMpr.Fy + FMadp.Fy;
    //проекции потребной силы в скоростной СК
    Xa_need = Xa_need_ =  fx*cos(tang-tetra) -fy*sin(tang-tetra);
    Ya_need = Ya_need_ = -fy*cos(tang-tetra) -fx*sin(tang-tetra);
    Ra_need = sqrt(Xa_need*Xa_need + Ya_need*Ya_need);
    //лобовое сопротивление вертолета, м2
    CxS = ( -(fx-FMpr.Fx)*cos(tang-tetra) + (fy-FMpr.Fy)*sin(tang-tetra) ) *2/roh/Vxy/Vxy;//CxS = ( -fx*cos(tang-tetra) + fy*sin(tang-tetra) ) *2/roh/Vxy/Vxy;
    //потребная тяга рулевого винта
    Ttr_need = (My_mr_ + FMadp.My) / x_tr;
    //вывод результатов расчета
    {
        solvout2 += "gravity   \t" + ss.setNum(FMhg.Fx  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMhg.Fy  ,'f',0) + "      \t";
        solvout2 +=                  ss.setNum(FMhg.Fz  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMhg.Mx  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMhg.My  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMhg.Mz  ,'f',1) + "      \n";
        if(mcg)
        {
        solvout2 += "cargo     \t" + ss.setNum(FMcg.Fx  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMcg.Fy  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMcg.Fz  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMcg.Mx  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMcg.My  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMcg.Mz  ,'f',1) + "      \n";
        }
        solvout2 += "mainrotor \t" + ss.setNum(FMmr.Fx  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMmr.Fy  ,'f',0) + "      \t";
        solvout2 +=                  ss.setNum(FMmr.Fz  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMmr.Mx  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMmr.My  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMmr.Mz  ,'f',1) + "      \n";
        if(trmount)
        {
        solvout2 += "tailrotor \t" + ss.setNum(FMtr.Fx  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMtr.Fy  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMtr.Fz  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMtr.Mx  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMtr.My  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMtr.Mz  ,'f',1) + "      \n";
        }
        if(prmount)
        {
        solvout2 += "pushrotor \t" + ss.setNum(FMpr.Fx  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMpr.Fy  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMpr.Fz  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMpr.Mx  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMpr.My  ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMpr.Mz  ,'f',1) + "      \n";
        }
        solvout2 += "fuselage  \t" + ss.setNum(FMadf.Fx ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadf.Fy ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadf.Fz ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadf.Mx ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadf.My ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadf.Mz ,'f',1) + "      \n";
        solvout2 += "wing      \t" + ss.setNum(FMadw.Fx ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadw.Fy ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadw.Fz ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadw.Mx ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadw.My ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadw.Mz ,'f',1) + "      \n";
        solvout2 += "stabilizer\t" + ss.setNum(FMads.Fx ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMads.Fy ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMads.Fz ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMads.Mx ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMads.My ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMads.Mz ,'f',1) + "      \n";
        solvout2 += "keel      \t" + ss.setNum(FMadk.Fx ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadk.Fy ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadk.Fz ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadk.Mx ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadk.My ,'f',1) + "      \t";
        solvout2 +=                  ss.setNum(FMadk.Mz ,'f',1) + "\n";
    }
    fillOut(out);
    if(mrsolve) mr->fillOutAz(out,0);
    if(trsolve) tr->fillOutAz(out,0);
    if(prsolve) pr->fillOutAz(out,0);
return error;
}

void Helicopter::HeliBalanceDynamic(double x0, double x1, double x2, double x3)
{                                 //dchi = x0; deta = x1; kren = x2; T_tr = x3;
    //СИЛЫ/МОМЕНТЫ ОТ НЕСУЩЕГО ВИНТА
    MainRotorForceCorrection(x0,x1);
    //СИЛЫ/МОМЕНТЫ ОТ РУЛЕВОГО or push ВИНТА
    if(autogyro)
       PushRotorForceCorrection(x3);//T_pr
    else
       TailRotorForceCorrection(x3);//T_tr
    //угол тангажа фюзеляжа, рад
    ocw.t = org.t = tang = alfa_mr - chi_hub - eps_fz + tetra;
    //угол атаки фюзеляжа без скоса от НВ
    alfa0f = alfa_mr - eps_fz - chi_hub;
    //СИЛЫ/МОМЕНТЫ ОТ ФЮЗЕЛЯЖА
    PlanerAirDynForces();//использует alfa0f
    //СИЛЫ/МОМЕНТЫ ОТ ВЕСА ВЕРТОЛЕТА
    HelicopterGravity(ocw.t,x2);
    //СИЛЫ/МОМЕНТЫ ОТ ВЕСА ГРУЗА НА ВНЕШНЕЙ ТРОСОВОЙ ПОДВЕСКЕ
    OutboardCargo(ocw.t,x2);
}

//уравнения балансировки вертолета
double Helicopter::b_Fx()//f3
{
    return FMmr.Fx + FMtr.Fx + FMpr.Fx + FMcg.Fx + FMhg.Fx + FMadp.Fx;// + FMads.Fx  + FMadk.Fx + FMadw.Fx;
}
double Helicopter::b_Fy()// /
{
    return FMmr.Fy + FMtr.Fy + FMpr.Fy + FMcg.Fy + FMhg.Fy + FMadp.Fy;// + FMads.Fy  + FMadk.Fy + FMadw.Fy;
}
double Helicopter::b_Fz()//f1
{
    return FMmr.Fz + FMtr.Fz + FMpr.Fz + FMcg.Fz + FMhg.Fz + FMadp.Fz;// + FMads.Fz  + FMadk.Fz + FMadw.Fz;
}
double Helicopter::b_Mx()//f2
{
    return FMmr.Mx + FMtr.Mx + FMpr.Mx + FMcg.Mx + FMhg.Mx + FMadp.Mx;// + FMads.Mx  + FMadk.Mx + FMadw.Mx;
}
double Helicopter::b_My()//f3
{
    return FMmr.My + FMtr.My + FMpr.My + FMcg.My + FMhg.My + FMadp.My;// + FMads.My  + FMadk.My + FMadw.My;
}
double Helicopter::b_Mz()//f4
{
    return FMmr.Mz + FMtr.Mz + FMpr.Mz + FMcg.Mz + FMhg.Mz + FMadp.Mz;// + FMads.Mz  + FMadk.Mz + FMadw.Mz;
}
//Вывод результатов балансировки вертолета
void Helicopter::PostBalance()// подготовка выводных данных
{
    balancereport = "";
    if(logkey)
    {
    if(balanced) balancereport += "Balance.";
                 balancereport += "Initial:\n\n";
                 balancereport += preout;
                 balancereport += "\n----------------------------------------------------------------------\n";
    }
    if(balanced) balancereport += "Balance.";
                 balancereport += "Report:\n\n";
    balancereport += "Vnx=          " + ss.setNum(vn.x            ,'f',1) + " м/с (";//Vhx
    balancereport +=                    ss.setNum(vn.x*3.6        ,'f',0) + " км/ч) горизонтальная скорость (СКН)\n";//Vhx
    balancereport += "Vny=          " + ss.setNum(vn.y            ,'f',3) + " м/с  \tвертикальная скорость (СКН)\n";//Vhy
    balancereport += "Vnz=          " + ss.setNum(vn.z            ,'f',3) + " м/с  \tбоковая скорость (СКН)\n";//Vhz
    balancereport += "v.x=          " + ss.setNum(v.x             ,'f',3) + " м/с  \tпродольная составляющая линейной скорости, м/с   (в СКС);\n";
    balancereport += "v.y=          " + ss.setNum(v.y             ,'f',3) + " м/с  \tвертикальная составляющая линейной скорости, м/с (в СКС);\n";
    balancereport += "v.z=          " + ss.setNum(v.z             ,'f',3) + " м/с  \tбоковая составляющая линейной скорости, м/с      (в СКС);\n";
    balancereport += "w.x=          " + ss.setNum(w.x    *180/M_PI,'f',3) + " грд/с\tугловая скорость вертолета отн. оX СКС;\n";
    balancereport += "w.y=          " + ss.setNum(w.y    *180/M_PI,'f',3) + " грд/с\tугловая скорость вертолета отн. оY СКС;\n";
    balancereport += "w.z=          " + ss.setNum(w.z    *180/M_PI,'f',3) + " грд/с\tугловая скорость вертолета отн. оZ СКС;\n";
    balancereport += "tang=         " + ss.setNum(tang   *180/M_PI,'f',6) + " грд  \tугол тангажа (по СГФ)\n";
    balancereport += "kren=         " + ss.setNum(kren   *180/M_PI,'f',6) + " грд  \tугол крена \n";
    balancereport += "rysk=         " + ss.setNum(rysk   *180/M_PI,'f',6) + " грд  \tугол рыскания\n";
    balancereport += "chi.hub=      " + ss.setNum(chi_hub*180/M_PI,'f',6) + " грд  \tугол отклонения втулки НВ назад\n";
    balancereport += "eta.hub=      " + ss.setNum(eta_hub*180/M_PI,'f',6) + " грд  \tугол отклонения втулки НВ влево\n";
    balancereport += "chi.mr=       " + ss.setNum(chi_ap *180/M_PI,'f',6) + " грд  \tугол отклонения АП НВ в продольном канале назад\n";
    balancereport += "eta.mr=       " + ss.setNum(eta_ap *180/M_PI,'f',6) + " грд  \tугол отклонения АП НВ в поперечном канале влево\n";
    if(trmount)
    {
        balancereport += "Ttr.п=        " + ss.setNum(Ttr_need        ,'f',1) + " Н      \tтяга рулевого винта потребная\n";
        balancereport += "T.tr=         " + ss.setNum(Fy_tr_           ,'f',1) + " Н      \tтяга рулевого винта действительная\n";
    }
    if(prmount)
    {
        balancereport += "Tpr.п=        " + ss.setNum(Tpr_need        ,'f',1) + " Н      \tтяга толкающего винта потребная\n";
        balancereport += "T.pr=         " + ss.setNum(Fy_pr_           ,'f',1) + " Н      \tтяга толкающего винта действительная\n";
    }
    balancereport += "Xa.п=         " + ss.setNum(Xa_need         ,'f',1) + " Н      \tпотребная пропульсивная сила в скоростной СК\n";
    balancereport += "Xa.mr=        " + ss.setNum(Xa              ,'f',1) + " Н      \tсила сопротивления/пропульсивная на втулке НВ в скоростной СК\n";
    balancereport += "Ya.п=         " + ss.setNum(Ya_need         ,'f',1) + " Н      \tпотребная подъемная сила в скоростной СК\n";
    balancereport += "Ya.mr=        " + ss.setNum(Ya              ,'f',1) + " Н      \tподъемная сила на втулке НВ в скоростной СК\n";
    //balancereport += "Za.п=         " + ss.setNum(Za_need         ,'f',1) + " Н  \t\tпотребная боковая сила в скоростной СК\n";
    //balancereport += "Za.mr=        " + ss.setNum(Za              ,'f',1) + " Н  \t\tбоковая сила на втулке НВ в скоростной СК\n";
    //balancereport += "Ra.п=         " + ss.setNum(Ra_need         ,'f',1) + " Н      \tпотребная результирующая сила в продольной плоскости\n";
    //balancereport += "Ra.mr=        " + ss.setNum(Ra              ,'f',1) + " Н      \tрезультирующая сила в продольной плоскости вертолета\n";
    balancereport += "err.Xa=       " + ss.setNum(errXan          ,'f',4) + " %    \tпогрешность по пропульсивной силе\n";
    balancereport += "err.Ya=       " + ss.setNum(errYan          ,'f',4) + " %    \tпогрешность по подъемной силе\n";
    balancereport += "err.chi=      " + ss.setNum(errChi          ,'f',4) + " %    \tпогрешность по углу отклонения АП в продольном канале\n";
    balancereport += "err.eta=      " + ss.setNum(errEta          ,'f',4) + " %    \tпогрешность по углу отклонения АП в поперечном канале\n";
    if(autogyro)
    balancereport += "err.Tpr=      " + ss.setNum(errTtr          ,'f',4) + " %    \tпогрешность по тяге толкающего винта\n";
    else
    balancereport += "err.Ttr=      " + ss.setNum(errTtr          ,'f',4) + " %    \tпогрешность по тяге рулевого винта\n";

    balancereport += "err.Fx=       " + ss.setNum(errFx           ,'f',4) + " %    \tпогрешность по результирующей в продольной плоскости\n";
    balancereport += "err.Fy=       " + ss.setNum(errFy           ,'f',4) + " %    \tпогрешность по результирующей в продольной плоскости\n";
    balancereport += "\nсилы и моменты в СКС, приведенные к ЦТ вертолета:\n";
    balancereport += "------------------------------------------------------------------------------------------------------------------------\n";
    balancereport += "\t\tFx, Н\t\tFy, Н\t\tFz, Н\t\tMx, Н·м\t\tMy, Н·м\t\tMz, Н·м\n";
    balancereport += "------------------------------------------------------------------------------------------------------------------------\n";
    balancereport += solvout2;
    balancereport += "------------------------------------------------------------------------------------------------------------------------\n";
    balancereport += "summary   \t" + ss.setNum(Fx,'f',1) + "      \t";
    balancereport +=                  ss.setNum(Fy,'f',1) + "      \t";
    balancereport +=                  ss.setNum(Fz,'f',1) + "      \t";
    balancereport +=                  ss.setNum(Mx,'f',1) + "      \t";
    balancereport +=                  ss.setNum(My,'f',1) + "      \t";
    balancereport +=                  ss.setNum(Mz,'f',1) + "\n";
    balancereport += "------------------------------------------------------------------------------------------------------------------------\n";
    //objects
    {
    balancereport += "\nнесущий винт: " + ss.setNum(mrsolve) + "\n";
    balancereport += "mr.alfa=      " + ss.setNum(alfa_mr*180/M_PI,'f',6) + " грд\tугол атаки конструктивной плоскости НВ\n";
    balancereport += "mr.alfaэ=     " + ss.setNum((alfa_mr+D1_mr*chi_ap-D2_mr*eta_ap)*180/M_PI,'f',6) + " грд\tугол атаки эквивалентного винта\n";//Миль1,с239
    balancereport += "mr.fi0=       " + ss.setNum(fi0_mr *180/M_PI,'f',6) + " грд  \tугол общего шага НВ\n";
    balancereport += "mr.rpm=       " + ss.setNum(rpm_mr          ,'f',1) + " об/мин  \tчастота вращения винта;\n";
    balancereport += "mr.Fx=        " + ss.setNum(Fx_mr           ,'f',1) + " Н      \tпродольная сила   (по оси X CKНВ)\n";
    balancereport += "mr.Fy=        " + ss.setNum(Fy_mr           ,'f',1) + " Н      \tвертикальная сила (по оси Y CKНВ)\n";
    balancereport += "mr.Fz=        " + ss.setNum(Fz_mr           ,'f',1) + " Н      \tпоперечная сила   (по оси Z CKНВ)\n";
    balancereport += "mr.Mx=        " + ss.setNum(Mx_mr           ,'f',1) + " Н·м    \tмомент на втулке  отн.оси X CKНВ\n";
    balancereport += "mr.My=        " + ss.setNum(My_mr           ,'f',1) + " Н·м    \tреактивный момент отн.оси Y CKНВ\n";
    balancereport += "mr.Mz=        " + ss.setNum(Mz_mr           ,'f',1) + " Н·м    \tмомент на втулке  отн.оси Z CKНВ\n";
    if(trmount)
    {
    balancereport += "\nрулевой винт: " + ss.setNum(trsolve) + "\n";
    balancereport += "tr.alfa=      " + ss.setNum(alfa_tr*180/M_PI,'f',6) + " грд   \tугол атаки РВ\n";
    balancereport += "tr.fi0=       " + ss.setNum(fi0_tr *180/M_PI,'f',6) + " грд  \tугол общего шага рулевого винта\n";
    balancereport += "tr.rpm=       " + ss.setNum(rpm_tr          ,'f',1) + " об/мин  \tчастота вращения винта;\n";
    balancereport += "tr.Fx=        " + ss.setNum(Fx_tr           ,'f',1) + " Н      \tпродольная сила   (по оси X CKРВ)\n";
    balancereport += "tr.Fy=        " + ss.setNum(Fy_tr           ,'f',1) + " Н      \tтяга винта        (по оси Y CKРВ)\n";
    balancereport += "tr.Fz=        " + ss.setNum(Fz_tr           ,'f',1) + " Н      \tвертикальная сила (по оси Z CKРВ)\n";
    balancereport += "tr.Mx=        " + ss.setNum(Mx_tr           ,'f',1) + " Н·м    \tмомент на втулке  отн.оси X CKРВ\n";
    balancereport += "tr.My=        " + ss.setNum(My_tr           ,'f',1) + " Н·м    \tреактивный момент отн.оси Y CKРВ\n";
    balancereport += "tr.Mz=        " + ss.setNum(Mz_tr           ,'f',1) + " Н·м    \tмомент на втулке  отн.оси Z CKРВ\n";
    }
    if(prmount)
    {
    balancereport += "\nтолкающий винт: " + ss.setNum(prsolve) + "\n";
    balancereport += "pr.alfa=      " + ss.setNum(alfa_pr*180/M_PI,'f',6) + " грд \tугол атаки ТВ\n";
    balancereport += "pr.fi0=       " + ss.setNum(fi0_pr *180/M_PI,'f',6) + " грд  \tугол общего шага толкающего винта\n";
    balancereport += "pr.rpm=       " + ss.setNum(rpm_pr          ,'f',1) + " об/мин  \tчастота вращения винта;\n";
    balancereport += "pr.Fx=        " + ss.setNum(Fx_pr           ,'f',1) + " Н      \tбоковая сила      (по оси X CKТВ)\n";
    balancereport += "pr.Fy=        " + ss.setNum(Fy_pr           ,'f',1) + " Н      \tтяга винта        (по оси Y CKТВ)\n";
    balancereport += "pr.Fz=        " + ss.setNum(Fz_pr           ,'f',1) + " Н      \tвертикальная сила (по оси Z CKТВ)\n";
    balancereport += "pr.Mx=        " + ss.setNum(Mx_pr           ,'f',1) + " Н·м    \tмомент на втулке  отн.оси X CKТВ\n";
    balancereport += "pr.My=        " + ss.setNum(My_pr           ,'f',1) + " Н·м    \tреактивный момент отн.оси Y CKТВ\n";
    balancereport += "pr.Mz=        " + ss.setNum(Mz_pr           ,'f',1) + " Н·м    \tмомент на втулке  отн.оси Z CKТВ\n";
    }
    balancereport += "\nвес вертолета:\n";
    balancereport += "Gh=           " + ss.setNum(Gh              ,'f',1) + " Н      \tвес вертолета (без лопастей)\n";
    balancereport += "tang=         " + ss.setNum(tang   *180/M_PI,'f',6) + " грд \tугол тангажа (по СГФ)\n";
    balancereport += "kren=         " + ss.setNum(kren   *180/M_PI,'f',6) + " грд \tугол крена \n";
    if(mcg)
    {
        balancereport += "\nгруз на внешней троссовой подвеске:\n";
        balancereport += "mcrg=         " + ss.setNum(mcg         ,'f',1) + " кг\n";
        balancereport += "zacep.x=      " + ss.setNum(zacep.x*1000,'f',3) + " мм\n";
        balancereport += "zacep.y=      " + ss.setNum(zacep.y*1000,'f',3) + " мм\n";
        balancereport += "zacep.z=      " + ss.setNum(zacep.z*1000,'f',3) + " мм\n";
        balancereport += "crgct.x=      " + ss.setNum(crgct.x*1000,'f',3) + " мм\n";
        balancereport += "crgct.y=      " + ss.setNum(crgct.y*1000,'f',3) + " мм\n";
        balancereport += "crgct.z=      " + ss.setNum(crgct.z*1000,'f',3) + " мм\n";
        balancereport += "Gcrg.x=       " + ss.setNum(Gcrg.x      ,'f',3) + " Н\n";
        balancereport += "Gcrg.x=       " + ss.setNum(Gcrg.x      ,'f',3) + " Н\n";
        balancereport += "Gcrg.x=       " + ss.setNum(Gcrg.x      ,'f',3) + " Н\n";
        //balancereport += "FMcg.Fx=      " + ss.setNum(FMcg.Fx     ,'f',3) + " Н\n";
        //balancereport += "FMcg.Fy=      " + ss.setNum(FMcg.Fy     ,'f',3) + " Н\n";
        //balancereport += "FMcg.Fz=      " + ss.setNum(FMcg.Fz     ,'f',3) + " Н\n";
        //balancereport += "FMcg.Mx=      " + ss.setNum(FMcg.Mx     ,'f',3) + " Н·м\n";
        //balancereport += "FMcg.My=      " + ss.setNum(FMcg.My     ,'f',3) + " Н·м\n";
        //balancereport += "FMcg.Mz=      " + ss.setNum(FMcg.Mz     ,'f',3) + " Н·м\n";
    }

    balancereport += "\nпланер:\n";
    balancereport += "CxS=          " + ss.setNum(CxS             ,'f',4) + " м2    \tлобовое сопротивление планера вертолета (скоростная СК)\n";
    balancereport += "FMadp.Fx=     " + ss.setNum(FMadp.Fx        ,'f',1) + " Н      \tпроекция аэродин.силы    планера  по оси X СКС в ЦТ вертолета\n";
    balancereport += "FMadp.Fy=     " + ss.setNum(FMadp.Fy        ,'f',1) + " Н      \tпроекция аэродин.силы    планера  по оси Y СКС в ЦТ вертолета\n";
    balancereport += "FMadp.Fz=     " + ss.setNum(FMadp.Fz        ,'f',1) + " Н      \tпроекция аэродин.силы    планера  по оси Z СКС в ЦТ вертолета\n";
    balancereport += "FMadp.Mx=     " + ss.setNum(FMadp.Mx        ,'f',1) + " Н·м    \tпроекция аэродин.момента планера отн.оси X СКС в ЦТ вертолета\n";
    balancereport += "FMadp.My=     " + ss.setNum(FMadp.My        ,'f',1) + " Н·м    \tпроекция аэродин.момента планера отн.оси Y СКС в ЦТ вертолета\n";
    balancereport += "FMadp.Mz=     " + ss.setNum(FMadp.Mz        ,'f',1) + " Н·м    \tпроекция аэродин.момента планера отн.оси Z СКС в ЦТ вертолета\n";

    balancereport += "\nфюзеляж (без оперения): " + ss.setNum(solvePlaner) + "\n";
    if(solvePlaner)
    {
        balancereport += "V_f=          " + ss.setNum(V_f           ,'f',3) + " м/с  \tскорость набегающего потока в расч.точке у фюзеляжа\n";
        balancereport += "q_f=          " + ss.setNum(q_f           ,'f',1) + " Па    \tскоростной напор в расч.точке у фюзеляжа\n";
        balancereport += "u0_mr=        " + ss.setNum(u0_mr         ,'f',3) + " м/с  \tскорость индуктивного потока на висении\n";
        balancereport += "Vvol_mr=      " + ss.setNum(Vvol_mr       ,'f',4) + "      \tотносительная скорость набегающего потока\n";
        balancereport += "Vx_mr=        " + ss.setNum(Vx_mr         ,'f',3) + " м/с  \tпроекция скорости набегающего потока на ось Ox СКНВ\n";
        balancereport += "Vy_mr=        " + ss.setNum(Vy_mr         ,'f',3) + " м/с  \tпроекция скорости набегающего потока на ось Oу СКНВ\n";
        balancereport += "delta_mr=     " + ss.setNum(delta_mr  *r2d,'f',4) + " грд  \tугол между вектором равнодействующей аэродин.сил винта и нормалью к скорости набегающего потока\n";
        balancereport += "uvol_mr=      " + ss.setNum(uvol_mr       ,'f',4) + "      \tотносительная индуктивная скорость\n";
        balancereport += "u_mr=         " + ss.setNum(u_mr          ,'f',3) + " м/с  \tсредняя индуктивная скорость от НВ\n";
        balancereport += "uh_mr=        " + ss.setNum(uh_mr         ,'f',3) + " м/с  \tпроекция на Ox средней индуктивной скорости от НВ\n";
        balancereport += "ut_mr=        " + ss.setNum(ut_mr         ,'f',3) + " м/с  \tпроекция на Oу средней индуктивной скорости от НВ\n";
        balancereport += "i1_mr=        " + ss.setNum(i1_mr     *r2d,'f',4) + " грд  \tугол между направлением потока за винтом и осью Ox\n";
        balancereport += "ut_f_mr=      " + ss.setNum(ut_f_mr       ,'f',3) + " м/с  \tиндуктивная скорость, спроецированная на ось Oy\n";
        balancereport += "uh_f_mr=      " + ss.setNum(uh_f_mr       ,'f',3) + " м/с  \tиндуктивная скорость, спроецированная на ось Oх\n";
        balancereport += "i1_mr_f=      " + ss.setNum(i1_mr_f   *r2d,'f',4) + " грд  \tугол между направлением потока за винтом и осью Ox у фюзеляжа\n";
        balancereport += "skos_f_mr=    " + ss.setNum(skos_f_mr *r2d,'f',4) + " грд  \tскос потока в расч.точке у фюзеляжа  ";
        balancereport += "(skos_f_mr_coarse= " + ss.setNum(skos_f_mr_coarse *r2d,'f',2) + " грд)\n";
        balancereport += "alfa0f=       " + ss.setNum(alfa0f    *r2d,'f',4) + " грд  \tугол атаки фюзеляжа без скоса потока (alfa0f = alfa_mr - eps_fz - chi_hub)\n";
        balancereport += "alfa_f=       " + ss.setNum(alfa_f    *r2d,'f',4) + " грд  \tугол атаки фюзеляжа (alfa_f = alfa0f - skos_f_mr)\n";
        switch(solvePlaner)
        {
            case 1:
            balancereport += "cxaf(a_f)=    " + ss.setNum(cxaf      ,'f',5) + "    \tаэродинамические коэф. в скоростной СК\n";
            balancereport += "cyaf(a_f)=    " + ss.setNum(cyaf      ,'f',5) + "    \tаэродинамические коэф. в скоростной СК\n";
            balancereport += "czaf(a_f)=    " + ss.setNum(czaf      ,'f',5) + "    \tаэродинамические коэф. в скоростной СК\n";
            balancereport += "mxaf(a_f)=    " + ss.setNum(mxaf      ,'f',5) + "    \tаэродинамические коэф. в скоростной СК\n";
            balancereport += "myaf(a_f)=    " + ss.setNum(myaf      ,'f',5) + "    \tаэродинамические коэф. в скоростной СК\n";
            balancereport += "mzaf(a_f)=    " + ss.setNum(mzaf      ,'f',5) + "    \tаэродинамические коэф. в скоростной СК\n";
            balancereport += "cx_f(a_f)=    " + ss.setNum(cx_f      ,'f',5) + "    \tаэродинамические коэф. в СКС\n";
            balancereport += "cy_f(a_f)=    " + ss.setNum(cy_f      ,'f',5) + "    \tаэродинамические коэф. в СКС\n";
            balancereport += "cz_f(a_f)=    " + ss.setNum(cz_f      ,'f',5) + "    \tаэродинамические коэф. в СКС\n";
            balancereport += "mx_f(a_f)=    " + ss.setNum(mx_f      ,'f',5) + "    \tаэродинамические коэф. в СКС\n";
            balancereport += "my_f(a_f)=    " + ss.setNum(my_f      ,'f',5) + "    \tаэродинамические коэф. в СКС\n";
            balancereport += "mz_f(a_f)=    " + ss.setNum(mz_f      ,'f',5) + "    \tаэродинамические коэф. в СКС\n";
            break;
            case 2:
            balancereport += "cx_f(a_f)=    " + ss.setNum(cx_f      ,'f',5) + "    \tаэродинамические коэф. в СКС\n";
            balancereport += "cy_f(a_f)=    " + ss.setNum(cy_f      ,'f',5) + "    \tаэродинамические коэф. в СКС\n";
            balancereport += "cz_f(a_f)=    " + ss.setNum(cz_f      ,'f',5) + "    \tаэродинамические коэф. в СКС\n";
            balancereport += "mx_f(a_f)=    " + ss.setNum(mx_f      ,'f',5) + "    \tаэродинамические коэф. в СКС\n";
            balancereport += "my_f(a_f)=    " + ss.setNum(my_f      ,'f',5) + "    \tаэродинамические коэф. в СКС\n";
            balancereport += "mz_f(a_f)=    " + ss.setNum(mz_f      ,'f',5) + "    \tаэродинамические коэф. в СКС\n";
            break;
        }
        balancereport += "Fx_f=         " + ss.setNum(Fx_f            ,'f',1) + " Н      \tсила сопротивления фюзеляжа в точке {x_f,y_f,z_f} (в СКС)\n";
        balancereport += "Fy_f=         " + ss.setNum(Fy_f            ,'f',1) + " Н      \tподъемная сила фюзеляжа в точке {x_f,y_f,z_f} (в СКС)\n";
        balancereport += "Fz_f=         " + ss.setNum(Fz_f            ,'f',1) + " Н      \tбоковая сила фюзеляжа в точке {x_f,y_f,z_f} (в СКС)\n";
        balancereport += "Mx_f=         " + ss.setNum(Mx_f            ,'f',1) + " Н·м    \tмомент отн.X от фюзеляжа в начале СКС\n";
        balancereport += "My_f=         " + ss.setNum(My_f            ,'f',1) + " Н·м    \tмомент отн.Y от фюзеляжа в начале СКС\n";
        balancereport += "Mz_f=         " + ss.setNum(Mz_f            ,'f',1) + " Н·м    \tмомент отн.Z от фюзеляжа в начале СКС\n";
    }

    balancereport += "\nкрыло: " + ss.setNum(solveWing) + "\n";
    if(solveWing)
    {
        balancereport += "V_wing=       " + ss.setNum(V_wing          ,'f',3) + " м/с   \tскорость набегающего потока у крыла\n";
        balancereport += "q_wing=       " + ss.setNum(q_wing          ,'f',1) + " Па     \tскоростной напор у крыла\n";
        balancereport += "ut_wing_mr=   " + ss.setNum(ut_wing_mr      ,'f',3) + " м/с   \tиндуктивная скорость НВ, спроецированная на ось Oy крыла\n";
        balancereport += "uh_wing_mr=   " + ss.setNum(uh_wing_mr      ,'f',3) + " м/с   \tиндуктивная скорость НВ, спроецированная на ось Oх крыла\n";
        balancereport += "i1_mr_wing=   " + ss.setNum(i1_mr_wing  *r2d,'f',4) + " грд   \tугол между направлением потока за винтом и осью Ox у крыла\n";
        balancereport += "skos_wing_mr= " + ss.setNum(skos_wing_mr*r2d,'f',4) + " грд   \tскос потока у крыла от НВ  ";
        balancereport += "(skos_wing_mr_coarse= " + ss.setNum(skos_wing_mr_coarse *r2d,'f',2) + " грд)\n";
        balancereport += "skos_wing_f=  " + ss.setNum(skos_wing_f *r2d,'f',4) + " грд   \tскос потока у крыла от фюзеляжа\n";
        balancereport += "eps_wing=     " + ss.setNum(eps_wing    *r2d,'f',4) + " грд   \tугол установки крыла относительно СГ\n";
        balancereport += "alfa_wing=    " + ss.setNum(alfa_wing   *r2d,'f',4) + " грд   \tугол атаки крыла (alfa_wing = alfa0f + eps_wing - skos_wing_mr - skos_wing_f)\n";
        balancereport += "cxaw(a_w)=    " + ss.setNum(cxawing         ,'f',5) + "    \tаэродинамический коэф.крыла в скоростной СК\n";
        balancereport += "cyaw(a_w)=    " + ss.setNum(cyawing         ,'f',5) + "    \tаэродинамический коэф.крыла в скоростной СК\n";
        balancereport += "cx_w(a_w)=    " + ss.setNum(cx_wing         ,'f',5) + "    \tаэродинамический коэф. в СК, связанной с профилем крыла\n";
        balancereport += "cy_w(a_w)=    " + ss.setNum(cy_wing         ,'f',5) + "    \tаэродинамический коэф. в СК, связанной с профилем крыла\n";
        balancereport += "X_wing=       " + ss.setNum(X_wing          ,'f',1) + " Н      \tсила сопротивления крыла в точке {x_st,y_st} в СК, связанной с профилем крыла\n";
        balancereport += "Y_wing=       " + ss.setNum(Y_wing          ,'f',1) + " Н      \tподъемная  сила  крыла  в точке  {x_st,y_st} в СК, связанной с профилем крыла\n";
        balancereport += "Fx_wing=      " + ss.setNum(Fx_wing         ,'f',1) + " Н      \tсила сопротивления крыла в точке {x_st,y_st} в СКС\n";
        balancereport += "Fy_wing=      " + ss.setNum(Fy_wing         ,'f',1) + " Н      \tподъемная  сила  крыла  в точке  {x_st,y_st} в СКС\n";
    }

    balancereport += "\nстабилизатор: " + ss.setNum(solveStab) + "\n";
    if(solveStab)
    {
        balancereport += "V_st=         " + ss.setNum(V_st            ,'f',3) + " м/с   \tскорость у стабилизатора от набегающего потока и НВ\n";
        balancereport += "q_st=         " + ss.setNum(q_st            ,'f',1) + " Па     \tскоростной напор у стабилизатора\n";
        balancereport += "ut_st_mr=     " + ss.setNum(ut_st_mr        ,'f',3) + " м/с   \tиндуктивная скорость НВ, спроецированная на ось Oy\n";
        balancereport += "uh_st_mr=     " + ss.setNum(uh_st_mr        ,'f',3) + " м/с   \tиндуктивная скорость НВ, спроецированная на ось Oх\n";
        balancereport += "i1_mr_st=     " + ss.setNum(i1_mr_st    *r2d,'f',4) + " грд   \tугол между направлением потока за винтом и осью Ox у стабилизатора\n";
        balancereport += "skos_st_mr=   " + ss.setNum(skos_st_mr  *r2d,'f',4) + " грд   \tскос потока у стабилизатора от НВ  ";
        balancereport +="(skos_st_mr_coarse= " + ss.setNum(skos_st_mr_coarse *r2d,'f',3) + " грд)\n";
        balancereport += "skos_st_f=    " + ss.setNum(skos_st_f   *r2d,'f',4) + " грд   \tскос потока у стабилизатора от фюзеляжа\n";
        balancereport += "skos_st_wing= " + ss.setNum(skos_st_wing*r2d,'f',4) + " грд   \tскос потока у стабилизатора от крыла  ";
           balancereport += "(alfa_stw= " + ss.setNum(alfa_stw    *r2d,'f',2) + " грд) ";
                 balancereport += "(_y_=" + ss.setNum(   _y_,'f',2) + ")\n";
        balancereport += "dalf_st_tng=  " + ss.setNum(dalf_st_tng *r2d,'f',4) + " грд   \tизменение угла атаки стабилизатора от изм.тангажа\n";
        balancereport += "eps_st=       " + ss.setNum(eps_st      *r2d,'f',4) + " грд   \tугол установки стабилизатора относительно СГ\n";
        balancereport += "alfa_st=      " + ss.setNum(alfa_st     *r2d,'f',4) + " грд   \tугол атаки стабилизатора (alfa_st = alfa0f + eps_st - skos_st_mr - skos_st_f - skos_st_wing + dalf_st_tng)\n";
        switch(solveStab)
        {
            case 1:
            balancereport += "cxast(a_st)=  " + ss.setNum(cxast ,'f',5) + "    \tаэродинамический коэф.стабилизатора в скоростной СК\n";
            balancereport += "cyast(a_st)=  " + ss.setNum(cyast ,'f',5) + "    \tаэродинамический коэф.стабилизатора в скоростной СК\n";
            balancereport += "cx_st(a_st)=  " + ss.setNum(cx_st ,'f',5) + "    \tаэродинамический коэф. в СК, связанной с профилем стабилизатора\n";
            balancereport += "cy_st(a_st)=  " + ss.setNum(cy_st ,'f',5) + "    \tаэродинамический коэф. в СК, связанной с профилем стабилизатора\n";
            balancereport += "X_st=         " + ss.setNum(X_st  ,'f',1) + " Н      \tсила сопротивления стабилизатора в точке {x_st,y_st} в СК, связанной с профилем стабилизатора\n";
            balancereport += "Y_st=         " + ss.setNum(Y_st  ,'f',1) + " Н      \tподъемная  сила  стабилизатора  в точке  {x_st,y_st} в СК, связанной с профилем стабилизатора\n";
            balancereport += "Fx_st=        " + ss.setNum(Fx_st ,'f',1) + " Н      \tсила сопротивления стабилизатора в точке {x_st,y_st} в СКС\n";
            balancereport += "Fy_st=        " + ss.setNum(Fy_st ,'f',1) + " Н      \tподъемная  сила  стабилизатора  в точке  {x_st,y_st} в СКС\n";
            break;
            case 2:
            balancereport += "eps_st0mr=    " + ss.setNum(eps_st0mr *r2d,'f',4) + " грд  \tугол установки стабилизатора относительно КПНВ при котором mz.st=0\n";
            balancereport += "eps_st_pr=    " + ss.setNum(eps_st_pr *r2d,'f',4) + " грд  \tугол установки стабилизатора в продувке\n";
            balancereport += "cx_st(ep,af)= " + ss.setNum(f_cx_st2(eps_st_pr, alfa_f),'f',5) + "    \tаэродинамический коэф. в СКС\n";
            balancereport += "cy_st(ep,af)= " + ss.setNum(f_cy_st2(eps_st_pr, alfa_f),'f',5) + "    \tаэродинамический коэф. в СКС\n";
            balancereport += "mz_st(ep,af)= " + ss.setNum(f_mz_st2(eps_st_pr, alfa_f),'f',5) + "    \tаэродинамический коэф. в СКС\n";
            balancereport += "Fx_st=        " + ss.setNum(Fx_st ,'f',1) + " Н      \tсила сопротивления стабилизатора в начале СКС\n";
            balancereport += "Fy_st=        " + ss.setNum(Fy_st ,'f',1) + " Н      \tподъемная сила стабилизатора в начале СКС\n";
            balancereport += "Mz_st=        " + ss.setNum(Mz_st ,'f',1) + " Н·м    \tмомент отн.Z от стабилизатора в начале СКС\n";
            break;
        }
    }

    balancereport += "\nкиль: " + ss.setNum(solveKeel) + "\n";
    if(solveKeel)
    {
        balancereport += "V_k=          " + ss.setNum(V_keel           ,'f',3) + " м/с   \tскорость у киля от набегающего потока и НВ\n";
        balancereport += "q_k=          " + ss.setNum(q_keel           ,'f',1) + " Па     \tскоростной напор у киля\n";
        balancereport += "ut_k_mr=      " + ss.setNum(ut_keel_mr       ,'f',3) + " м/с   \tиндуктивная скорость НВ, спроецированная на ось Oy\n";
        balancereport += "uh_k_mr=      " + ss.setNum(uh_keel_mr       ,'f',3) + " м/с   \tиндуктивная скорость НВ, спроецированная на ось Oх\n";
        balancereport += "vs_mr=        " + ss.setNum(vs_mr            ,'f',4) + " грд   \tиндуктивная скорость (боковая) несущего винта\n";
        balancereport += "skos_k_mr=    " + ss.setNum(skos_k_mr    *r2d,'f',4) + " грд   \tскос потока у киля от несущего винта\n";
        if(trmount)
        {
            balancereport += "u_tr=         " + ss.setNum(u_tr        ,'f',4) + " грд   \tиндуктивная скорость (осевая) рулевого винта\n";
            balancereport += "skos_k_tr=    " + ss.setNum(skos_k_tr*r2d,'f',4) + " грд   \tскос потока у киля от рулевого винта\n";
            //balancereport += "xi_co=        " + ss.setNum(xi_co      ,'f',4) + "       \tпоправка, учитывающая положение киля относительно следа винта\n";
        }
        balancereport += "skos_k_f=     " + ss.setNum(skos_k_f     *r2d,'f',4) + " грд   \tскос потока у киля от фюзеляжа\n";
        balancereport += "dalf_k_rsk=   " + ss.setNum(dalf_k_rsk   *r2d,'f',4) + " грд   \tизменение угла атаки киля от рыскания\n";
        balancereport += "eps_k=        " + ss.setNum(eps_keel     *r2d,'f',4) + " грд   \tугол установки киля (>0 носок влево), грд \n";
        balancereport += "alfa_k=       " + ss.setNum(alfa_keel    *r2d,'f',4) + " грд  \tугол атаки киля (alfa_keel = eps_keel + beta_sk + dalf_k_rsk - skos_k_mr - skos_k_tr - skos_k_f)\n";
        balancereport += "cxak(a_k)=    " + ss.setNum(cxakeel          ,'f',5) + "    \tаэродинамический коэф.стабилизатора в скоростной СК\n";
        balancereport += "cyak(a_k)=    " + ss.setNum(cyakeel          ,'f',5) + "    \tаэродинамический коэф.стабилизатора в скоростной СК\n";
        balancereport += "cx_k(a_k)=    " + ss.setNum(cx_keel          ,'f',5) + "    \tаэродинамический коэф. в СК, связанной с профилем стабилизатора\n";
        balancereport += "cy_k(a_k)=    " + ss.setNum(cy_keel          ,'f',5) + "    \tаэродинамический коэф. в СК, связанной с профилем стабилизатора\n";
        balancereport += "X_k=          " + ss.setNum(X_keel           ,'f',1) + " Н      \tаэродин.сила на киле в СК, связанной с профилем киля\n";
        balancereport += "Y_k=          " + ss.setNum(Y_keel           ,'f',1) + " Н      \tаэродин.сила на киле в СК, связанной с профилем киля\n";
        balancereport += "Fx_k=         " + ss.setNum(Fx_keel          ,'f',1) + " Н      \tсила сопротивления киля в точке {x_k,y_k} в СКС\n";
        balancereport += "Fz_k=         " + ss.setNum(Fz_keel          ,'f',1) + " Н      \tбоковая сила на киле (вправо) в {x_k,y_k} в СКС\n";
    }

    balancereport += "\n\n";
    }
    /* ROTORS
    //MainRotor
    if(mrsolve)
    {
        balancereport += "\n------------------------------------------------------------------------------------------------------------------------\n";
        balancereport += mr->Pre2Str(logkey);
        balancereport += mr->Res2Str(logkey);
    }
    //TailRotor
    if(trsolve)
    {
        balancereport += "\n------------------------------------------------------------------------------------------------------------------------\n";
        balancereport += tr->Pre2Str(logkey);
        balancereport += tr->Res2Str(logkey);
    }
    //PushRotor
    if(prsolve)
    {
        balancereport += "\n------------------------------------------------------------------------------------------------------------------------\n";
        balancereport += pr->Pre2Str(logkey);
        balancereport += pr->Res2Str(logkey);
    }*/
}



/// вспомогательные

xyzkrt Helicopter::org2ocw(xyzkrt &org)//
{
    xyzkrt ocw;
    xyz ct, snCW;
    ct.x = xct;
    ct.y = yct;
    ct.z = zct;
    snCW  = SKStoSKN( ct );
    ocw.x = org.x + snCW.x;
    ocw.y = org.y + snCW.y;
    ocw.z = org.z + snCW.z;
    ocw.k = org.k;
    ocw.r = org.r;
    ocw.t = org.t;
return ocw;
}

xyzkrt Helicopter::ocw2org(xyzkrt &ocw)//
{
    xyzkrt org;
    xyz ct, snCW;
    ct.x = xct;
    ct.y = yct;
    ct.z = zct;
    snCW  = SKStoSKN( ct );
    org.x = ocw.x - snCW.x;
    org.y = ocw.y - snCW.y;
    org.z = ocw.z - snCW.z;
    org.k = ocw.k;
    org.r = ocw.r;
    org.t = ocw.t;
return org;
}

xyz Helicopter::SKStoSKN(xyz &A)//перевод из связанной СК в нормальную
{
    xyz Askn;

    double a11 = cos(ocw.r)*cos(ocw.t);
    double a21 = sin(ocw.k)*sin(ocw.r) - cos(ocw.k)*cos(ocw.r)*sin(ocw.t);
    double a31 = cos(ocw.k)*sin(ocw.r) + sin(ocw.k)*cos(ocw.r)*sin(ocw.t);
    double a12 = sin(ocw.t);
    double a22 = cos(ocw.t)*cos(ocw.k);
    double a32 =-cos(ocw.t)*sin(ocw.k);
    double a13 =-sin(ocw.r)*cos(ocw.t);
    double a23 = sin(ocw.k)*cos(ocw.r) + cos(ocw.k)*sin(ocw.r)*sin(ocw.t);
    double a33 = cos(ocw.k)*cos(ocw.r) - sin(ocw.k)*sin(ocw.r)*sin(ocw.t);

    Askn.x = a11*A.x + a21*A.y + a31*A.z;
    Askn.y = a12*A.x + a22*A.y + a32*A.z;
    Askn.z = a13*A.x + a23*A.y + a33*A.z;

return Askn;
}

xyz Helicopter::SKNtoSKS(xyz &A)//перевод из нормальной СК в связанную
{
    xyz Asks;

    double a11 = cos(ocw.r)*cos(ocw.t);
    double a21 = sin(ocw.k)*sin(ocw.r) - cos(ocw.k)*cos(ocw.r)*sin(ocw.t);
    double a31 = cos(ocw.k)*sin(ocw.r) + sin(ocw.k)*cos(ocw.r)*sin(ocw.t);
    double a12 = sin(ocw.t);
    double a22 = cos(ocw.t)*cos(ocw.k);
    double a32 =-cos(ocw.t)*sin(ocw.k);
    double a13 =-sin(ocw.r)*cos(ocw.t);
    double a23 = sin(ocw.k)*cos(ocw.r) + cos(ocw.k)*sin(ocw.r)*sin(ocw.t);
    double a33 = cos(ocw.k)*cos(ocw.r) - sin(ocw.k)*sin(ocw.r)*sin(ocw.t);

    Asks.x = a11*A.x + a12*A.y + a13*A.z;
    Asks.y = a21*A.x + a22*A.y + a23*A.z;
    Asks.z = a31*A.x + a32*A.y + a33*A.z;

return Asks;
}

float Helicopter::inter(float x, float x1, float x2, float y1, float y2)
{
    if(x2==x1) return 0;
    else return  y1 + (x-x1)*(y2-y1)/(x2-x1);
}

void Helicopter::setNull()
{
    balanced=0;
    pi  = M_PI;
    g       =0;
    Lpt     =0;
    RjmK    =0;
    RjkgK   =0;
    Mm      =0;
    t0      =0;
    p0      =0;
    ro      =0;
    roh     =0;
    TK      =0;
    P_      =0;
    t_      =0;

     v.x =0;     v.y =0;     v.z =0;
    v_.x =0;    v_.y =0;    v_.z =0;
     w.x =0;     w.y =0;     w.z =0;
    w_.x =0;    w_.y =0;    w_.z =0;
     a.x =0;     a.y =0;     a.z =0;
     e.x =0;     e.y =0;     e.z =0;
    org.k =0;   org.r =0;   org.t =0;
    org.x =0;   org.y =0;   org.z =0;
    org_.k =0;  org_.r =0;  org_.t =0;
    org_.x =0;  org_.y =0;  org_.z =0;
    vn.k =0;    vn.r =0;    vn.t =0;
    vn.x =0;    vn.y =0;    vn.z =0;
    an.x =0;    an.y =0;    an.z =0;
    en.x =0;    en.y =0;    en.z =0;

    // п а р а м е т р ы  в е р т о л е т а
    mh      =0;
    Gh      =0;
    Ghb     =0;
    Ix      =0;
    Iy      =0;
    Iz      =0;
    Iyz     =0;
    Ixz     =0;
    Ixy     =0;
    eps_fz  =0;
    eps_fx  =0;
    l_mr    =0;
    xctmr   =0;    yctmr   =0;    zctmr   =0;
    xct     =0;    yct     =0;    zct     =0;
    x_mr    =0;    y_mr    =0;    z_mr    =0;
    x_tr    =0;    y_tr    =0;    z_tr    =0;
    x_f     =0;    y_f     =0;    z_f     =0;
    x_st    =0;    y_st    =0;    z_st    =0;
    x_k     =0;
    mcg     =0;
    Gcg     =0;
    lcg     =0;
    zacep.x =0;    zacep.y =0;    zacep.z =0;
    crgct.x =0;    crgct.y =0;    crgct.z =0;
    Gcrg .x =0;    Gcrg .y =0;    Gcrg .z =0;

    Hh      =0;
    Vi      =0;
    tetra   =0;
    fi0_min =0;
    fi0_max =0;
    //rpm_min =0;
    //rpm_max =0;
    est_min =0;
    est_max =0;
    Lst     =0;
    lst_mr  =0;
    dlt_st  =0;
    Lmod    =0;
    Smod    =0;
    lmodel  =0;
    Smodel  =0;
    Mmodel  =0;

    //MainRotor
    Fx_mr_ =0;    Fy_mr_ =0;    Fz_mr_ =0;
    Mx_mr_ =0;    My_mr_ =0;    Mz_mr_ =0;
    Fx_mr  =0;    Fy_mr  =0;    Fz_mr  =0;
    Mx_mr  =0;    My_mr  =0;    Mz_mr  =0;
    dH_mr  =0;
    dS_mr  =0;
    w_mr        =0;
    Dmr         =0;
    Rmr         =0;
    Fmr         =0;
    zl_mr       =0;
    Sgsh_mr     =0;
    Bmr         =0;
    k_          =0;
    Rap         =0;
    ll          =0;
    lgsh        =0;
    lvsh        =0;
    dgsh        =0;
    dpsi_cont   =0;
    D_2         =0;
    D_1         =0;
    D1_mr       =0;
    D2_mr       =0;
    zswlmr      =0;
    vs_mr_      =0;
    alfa_mr     =0;
    beta_sk     =0;
    chi_ap = chi_ap_  =0;
    eta_ap = eta_ap_  =0;
    chi_hub= chi_hub_ =0;
    eta_hub= eta_hub_ =0;
    fi0_mr = fi0_mr_  =0;
    FMmr.Fx =0;    FMmr.Fy =0;    FMmr.Fz =0;
    FMmr.Mx =0;    FMmr.My =0;    FMmr.Mz =0;

    //TailRotor
    Fx_tr   =0;    Fy_tr   =0;    Fz_tr   =0;
    Mx_tr   =0;    My_tr   =0;    Mz_tr   =0;
    Fx_tr_  =0;    Fy_tr_  =0;    Fz_tr_  =0;
    Mx_tr_  =0;    My_tr_  =0;    Mz_tr_  =0;
    FMtr.Fx =0;    FMtr.Fy =0;    FMtr.Fz =0;
    FMtr.Mx =0;    FMtr.My =0;    FMtr.Mz =0;
    Dtr     =0;
    u_tr_   =0;
    u_pr_   =0;

    Fx_pr_ = 0;    Fy_pr_ = 0;    Fz_pr_ = 0;
    Mx_pr_ = 0;    My_pr_ = 0;    Mz_pr_ = 0;
    FMpr.Fx =0;    FMpr.Fy =0;    FMpr.Fz =0;
    FMpr.Mx =0;    FMpr.My =0;    FMpr.Mz =0;

    //HelicopterGravity()
    FMhg.Fx =0;    FMhg.Fy =0;    FMhg.Fz =0;
    FMhg.Mx =0;    FMhg.My =0;    FMhg.Mz =0;

    //OutboardCargo()
    FMcg.Fx =0;    FMcg.Fy =0;    FMcg.Fz =0;
    FMcg.Mx =0;    FMcg.My =0;    FMcg.Mz =0;

    //PlanerAirDynForces
    FMadp.Fx =0;   FMadp.Fy =0;   FMadp.Fz =0;
    FMadp.Mx =0;   FMadp.My =0;   FMadp.Mz =0;

    FMadf.Fx =0;   FMadf.Fy =0;   FMadf.Fz =0;
    FMadf.Mx =0;   FMadf.My =0;   FMadf.Mz =0;
    Vx_mr    =0;
    Vy_mr    =0;
    delta_mr =0;
    i1_mr    =0;
    Vvol_mr  =0;
    uvol_mr  =0;
    u0_mr    =0;
    u_mr     =0;
    uh_mr    =0;
    ut_mr    =0;
    ut_f_mr  =0;
    uh_f_mr  =0;
    Ksi1f    =0;
    Ksi2     =0;
    Ksi3n    =0;
    Ksi3t    =0;
    V_f      =0;
    i1_mr_f  =0;
    skos_f_mr =0;
    alfa_f  =0;
    Fx_f    =0;    Fy_f    =0;    Fz_f    =0;
    Mx_f    =0;    My_f    =0;    Mz_f    =0;

    //StabilizerForces()
    _alfafst  =0;
    Ksi1st    =0;
    ut_st_mr  =0;
    uh_st_mr  =0;
    V_st      =0;
    i1_mr_st  =0;
    skos_st_mr=0;
    skos_st_f =0;
    skos_st   =0;
    eps_st0mr =0;
    eps_st    =0;
    alfa_st   =0;
    eps_st_pr =0;
    eps_st_pr_=0;
    Fx_st    =0;    Fy_st    =0;    Fz_st    =0;
    Mx_st    =0;    My_st    =0;    Mz_st    =0;
    FMads.Fx =0;    FMads.Fy =0;    FMads.Fz =0;
    FMads.Mx =0;    FMads.My =0;    FMads.Mz =0;

    //f_uth()
    Xmr_    = 0;
    Ymr_    = 0;
    tau_mr  = 0;
    nor_mr  = 0;
    x1_mr   = 0;
    R1_mr   = 0;
    hflow_mr= 0;

    //SolveBalance()
    dchi    = 0;
    deta    = 0;
    kren    = 0;
    tang    = 0;
    rysk    = 0;
    alfa_mr = 0;
    chi_ap  = 0;
    eta_ap  = 0;
    fx      = 0;
    fy      = 0;
    CxS     = 0;
    Xa_need = 0;
    Ya_need = 0;
    errFx   = 0;
    errFy   = 0;
    errXan  = 0;
    errYan  = 0;
    errChi  = 0;
    errEta  = 0;
    errTtr  = 0;

    //Shassi
    FMsh.Fx = 0;    FMsh.Fy = 0;    FMsh.Fz = 0;
    FMsh.Mx = 0;    FMsh.My = 0;    FMsh.Mz = 0;
    for(int k=0; k<mst; k++)
    {
        FMw[k].Fx = 0;  FMw[k].Mx = 0;
        FMw[k].Fy = 0;  FMw[k].My = 0;
        FMw[k].Fz = 0;  FMw[k].Mz = 0;
        FMw1[k].Fx= 0;  FMw1[k].Mx= 0;
        FMw1[k].Fy= 0;  FMw1[k].My= 0;
        FMw1[k].Fz= 0;  FMw1[k].Mz= 0;
        FMw2[k].Fx= 0;  FMw2[k].Mx= 0;
        FMw2[k].Fy= 0;  FMw2[k].My= 0;
        FMw2[k].Fz= 0;  FMw2[k].Mz= 0;
    }

    //outerloads
    FM1.Fx = 0;   FM1.Mx = 0;
    FM1.Fy = 0;   FM1.My = 0;
    FM1.Fz = 0;   FM1.Mz = 0;
    FM2.Fx = 0;   FM2.Mx = 0;
    FM2.Fy = 0;   FM2.My = 0;
    FM2.Fz = 0;   FM2.Mz = 0;

}


