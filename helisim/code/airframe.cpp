//HELICOPTER.AIRFRAME.MODEL
#include "helicopter.h"
//-----------------------------------------------------------------------------

void Helicopter::PlanerAirDynForces()//Аэродинамические силы и моменты от фюзеляжа (без стабилизатора, киля, крыла)
{
    //Аэродинамические силы и моменты от фюзеляжа в ц.т.вертолета (без стабилизатора, киля, крыла)
    FMadf.Fx = 0;    FMadf.Mx = 0;
    FMadf.Fy = 0;    FMadf.My = 0;
    FMadf.Fz = 0;    FMadf.Mz = 0;
    //индуктивная скорость TВ
    if(prmount) u_pr = (prsolve)?(pr->vt *1.1):(u_pr_ *1.1);
    else        u_pr = 0;
    //уже определен alfa0f = alfa_mr - eps_fz - chi_hub;//угол атаки фюзеляжа без скоса от НВ

    switch (solvePlaner)
    {
    case 0://по заданным усилиям в характерной точке фюзеляжа
        {
        skos_f_mr = 0;
        alfa_f = 0;
        //Силы и моменты от фюзеляжа приведенные к ЦТ вертолета (в СКС)
        FMadf.Fx = Fx_f;
        FMadf.Fy = Fy_f;
        FMadf.Fz = Fz_f;
        FMadf.Mx = Mx_f + Fz_f*(y_f-yct) - Fy_f*(z_f-zct);
        FMadf.My = My_f - Fz_f*(x_f-xct) + Fx_f*(z_f-zct);
        FMadf.Mz = Mz_f + Fy_f*(x_f-xct) - Fx_f*(y_f-yct);
        }
        break;

    case 1://Упрощенный расчет аэродинамических сил и моментов от фюзеляжа в ц.т.вертолета
        {
        //Скорость индуктивного потока на висении, м/с
        u0_mr = sqrt(1.1*mh*g/(2*roh*Fmr*Bmr*Bmr));
        //Относительная скорость набегающего потока для рис.3.
        Vvol_mr = Vi/u0_mr;
        //Проекция скорости набегающего потока на ось Ox СКНВ, м/с
        Vx_mr =  Vi*cos(alfa_mr);
        //Проекция скорости набегающего потока на ось Oy СКНВ, м/с
        Vy_mr = -Vi*sin(alfa_mr);
        //Угол между вектором равнодействующей аэродин.сил винта и нормалью к скорости набегающего потока, рад
        double rprrbbv = 2*roh*Fmr*Bmr*Bmr*pow(u0_mr,3)*Vvol_mr;
        if (rprrbbv==0) delta_mr = 0;
        else delta_mr = asin( (Fx_mr*Vx_mr - Fy_mr*Vy_mr) / rprrbbv );
        //Относительная индуктивная скорость
        uvol_mr = f_uvol_mr(Vvol_mr, delta_mr);
        //Средняя индуктивная скорость от НВ, м/с
        u_mr  = uvol_mr*u0_mr;
        uh_mr = u_mr *Fx_mr/sqrt(Fx_mr*Fx_mr+Fy_mr*Fy_mr);//проекция на Ox СКНВ ср.инд.скорости
        ut_mr = u_mr *Fy_mr/sqrt(Fx_mr*Fx_mr+Fy_mr*Fy_mr);//проекция на Oу СКНВ ср.инд.скорости
        //Угол между направлением потока за винтом и осью Ox СКНВ, рад
        i1_mr = atan( (Vy_mr+ut_mr) / (Vx_mr-uh_mr) );
        //коэффициент, учитывающий влияние сужения крыла
        Ksi1f = 1.0;
        //...индуктивная скорость
        duo uth_f = f_uth( x_f, y_f, Ksi1f);
        ut_f_mr = uth_f.a;//ut -индуктивная скорость, спроецированная на ось Oy СКНВ, м/с
        uh_f_mr = uth_f.b;//uh -индуктивная скорость, спроецированная на ось Ox СКНВ, м/с
        //скорость набегающего потока в расч.точке у фюзеляжа, м/с
        V_f = sqrt( pow(Vx_mr-uh_f_mr,2) + pow(Vy_mr+ut_f_mr,2) );
        //Скоростной напор в расч.точке у фюзеляжа
        q_f = 0.5*roh*V_f*V_f;
        //Угол между направлением потока за винтом и осью Ox СКНВ у фюзеляжа, рад
        i1_mr_f = atan( (Vy_mr+ut_f_mr) / (Vx_mr-uh_f_mr) );
        //скос потока в расч.точке у фюзеляжа, рад
        skos_f_mr_coarse = (Vi>9)?( 0.5*Fy_mr/(roh*Vi*Vi*Fmr) ):(88.88*d2r);//рад, в горизонтальном полете !!!!   [Володко'84,c163]
        skos_f_mr = (fabs(alfa_mr)>0.4*M_PI)?(0):fabs(alfa_mr + i1_mr_f);
        if(skos_mr_) skos_f_mr = skos_mr_;
        //угол атаки фюзеляжа, рад
        alfa_f = alfa0f - skos_f_mr;

        //аэродинамические коэф.фюзеляжа в скоростной СК
        cxaf = f_cx_f(alfa_f);     mxaf = f_mx_f(alfa_f);
        cyaf = f_cy_f(alfa_f);     myaf = f_my_f(alfa_f);
        czaf = f_cz_f(alfa_f);     mzaf = f_mz_f(alfa_f);
        //поворот скоростной СК на угол alfa_f
        cx_f =  cxaf*cos(alfa_f) - cyaf*sin(alfa_f);
        cy_f =  cyaf*cos(alfa_f) + cxaf*sin(alfa_f);
        cz_f =  czaf;
        mx_f =  mxaf*cos(alfa_f) + myaf*sin(alfa_f);
        my_f =  myaf*cos(alfa_f) - mxaf*sin(alfa_f);
        mz_f =  mzaf;
        //аэродин.сила+момент от фюзеляжа (без стабилизатора) в локальной СК ||СКC
        Fx_f = -cx_f *q_f *Smod;
        Fy_f =  cy_f *q_f *Smod;
        Fz_f =  cz_f *q_f *Smod;
        Mx_f =  mx_f *q_f *Smod*Lmod;
        My_f =  my_f *q_f *Smod*Lmod;
        Mz_f =  mz_f *q_f *Smod*Lmod;
        //Силы и моменты от фюзеляжа (без стабилизатора) приведенные к ЦТ вертолета (в СКС)
        FMadf.Fx = Fx_f;
        FMadf.Fy = Fy_f;
        FMadf.Fz = Fz_f;
        FMadf.Mx = Mx_f + Fz_f*(y_f-yct) - Fy_f*(z_f-zct);
        FMadf.My = My_f - Fz_f*(x_f-xct) + Fx_f*(z_f-zct);
        FMadf.Mz = Mz_f + Fy_f*(x_f-xct) - Fx_f*(y_f-yct);
        }
        break;

    case 2://Расчет аэродин.сил и моментов от фюзеляжа в ц.т.вертолета (по методике балансировки) Ми-2
        {
        //Скорость индуктивного потока на висении, м/с
        u0_mr = sqrt(1.1*mh*g/(2*roh*Fmr*Bmr*Bmr));
        //Относительная скорость набегающего потока для рис.3.
        Vvol_mr = Vi/u0_mr;
        //Проекция скорости набегающего потока на ось Ox СКНВ, м/с
        Vx_mr =  Vi*cos(alfa_mr);
        //Проекция скорости набегающего потока на ось Oy СКНВ, м/с
        Vy_mr = -Vi*sin(alfa_mr);
        //Угол между вектором равнодействующей аэродин.сил винта и нормалью к скорости набегающего потока, рад
        double rprrbbv = 2*roh*Fmr*Bmr*Bmr*pow(u0_mr,3)*Vvol_mr;
        if (rprrbbv==0) delta_mr = 0;
        else delta_mr = asin( (Fx_mr*Vx_mr - Fy_mr*Vy_mr) / rprrbbv );
        //Относительная индуктивная скорость
        uvol_mr = f_uvol_mr(Vvol_mr, delta_mr);
        //Средняя индуктивная скорость от НВ, м/с
        u_mr  = uvol_mr*u0_mr;
        uh_mr = u_mr *Fx_mr/sqrt(Fx_mr*Fx_mr+Fy_mr*Fy_mr);//проекция на Ox ср.инд.скорости
        ut_mr = u_mr *Fy_mr/sqrt(Fx_mr*Fx_mr+Fy_mr*Fy_mr);//проекция на Oу ср.инд.скорости
        //Угол между направлением потока за винтом и осью Ox, рад
        i1_mr = atan( (Vy_mr+ut_mr) / (Vx_mr-uh_mr) );
        //коэффициент, учитывающий влияние сужения крыла
        Ksi1f = 0.5;
        //...индуктивная скорость
        duo uth_f = f_uth( x_f, y_f, Ksi1f);
        ut_f_mr = uth_f.a;//ut -индуктивная скорость, спроецированная на ось Oy, м/с
        uh_f_mr = uth_f.b;//uh -индуктивная скорость, спроецированная на ось Ox, м/с
        //скорость набегающего потока в расч.точке у фюзеляжа, м/с
        V_f = sqrt( pow(Vx_mr-uh_f_mr,2) + pow(Vy_mr+ut_f_mr,2) );
        //Скоростной напор в расч.точке у фюзеляжа
        q_f = 0.5*roh*V_f*V_f;
        //Угол между направлением потока за винтом и осью Ox у фюзеляжа, рад
        i1_mr_f = atan( (Vy_mr+ut_f_mr) / (Vx_mr-uh_f_mr) );
        //скос потока в расч.точке у фюзеляжа, рад
        skos_f_mr = alfa_mr + i1_mr_f;
        //угол атаки фюзеляжа, рад
        alfa_f = alfa0f - skos_f_mr;

        //аэродинамические коэф.фюзеляжа в связанной СК
        cx_f = f_cx_f(alfa_f);
        cy_f = f_cy_f(alfa_f);
        cz_f = f_cz_f(alfa_f);
        mx_f = f_mx_f(alfa_f);
        my_f = f_my_f(alfa_f);
        mz_f = f_mz_f(alfa_f);
        //аэродин.сила+момент от планера (без стабилизатора) в локальной СК ||СКC
        Fx_f = cx_f *q_f *Smod;
        Fy_f = cy_f *q_f *Smod;
        Fz_f = cz_f *q_f *Smod;
        Mx_f = mx_f *q_f *Smod*Lmod;
        My_f = my_f *q_f *Smod*Lmod;
        Mz_f = mz_f *q_f *Smod*Lmod;
        //Силы и моменты от фюзеляжа приведенные к ЦТ вертолета (в СКС)
        FMadf.Fx = Fx_f;
        FMadf.Fy = Fy_f;
        FMadf.Fz = Fz_f;
        FMadf.Mx = Mx_f + Fz_f*(y_f-yct) - Fy_f*(z_f-zct);
        FMadf.My = My_f - Fz_f*(x_f-xct) + Fx_f*(z_f-zct);
        FMadf.Mz = Mz_f + Fy_f*(x_f-xct) - Fx_f*(y_f-yct);
        //Силы и моменты от фюзеляжа (СК на оси НВ)
        //FMadf.Fx =  Fx_f*cos(-eps_f) +Fy_f*sin(-eps_f);
        //FMadf.Fy = -Fx_f*sin(-eps_f) +Fy_f*cos(-eps_f);
        //FMadf.Fz =  Fz_f;
        //FMadf.Mx = z_f*(Fx_f*sin(-eps_f)-Fy_f*cos(-eps_f)) + y_f*Fz_f + Mx_f*cos(-eps_f) + My_f*sin(-eps_f);
        //FMadf.My = z_f*(Fx_f*cos(-eps_f)+Fy_f*sin(-eps_f)) - x_f*Fz_f - Mx_f*sin(-eps_f) + My_f*cos(-eps_f);
        //FMadf.Mz = Mz_f - x_f*(Fx_f*sin(-eps_f)-Fy_f*cos(-eps_f)) - y_f*(Fx_f*cos(-eps_f) +Fy_f*sin(-eps_f)); // */
        }
        break;
    }

    WingAirDynForces();//СИЛЫ/МОМЕНТЫ ОТ КРЫЛА
    StabAirDynForces();//СИЛЫ/МОМЕНТЫ ОТ СТАБИЛИЗАТОРА
    KeelAirDynForces();//СИЛЫ/МОМЕНТЫ ОТ КИЛЯ
    //Аэродинамические силы и моменты от планера в СКP
    FMadp.Fx = FMadf.Fx + FMads.Fx + FMadk.Fx + FMadw.Fx;
    FMadp.Fy = FMadf.Fy + FMads.Fy + FMadk.Fy + FMadw.Fy;
    FMadp.Fz = FMadf.Fz + FMads.Fz + FMadk.Fz + FMadw.Fz;
    FMadp.Mx = FMadf.Mx + FMads.Mx + FMadk.Mx + FMadw.Mx;
    FMadp.My = FMadf.My + FMads.My + FMadk.My + FMadw.My;
    FMadp.Mz = FMadf.Mz + FMads.Mz + FMadk.Mz + FMadw.Mz;
}

void Helicopter::WingAirDynForces()//Аэродинамические силы и моменты от крыла
{
    Fz_wing = 0;
    Mz_wing = 0;
    Mx_wing = 0;
    My_wing = 0;
    alfa_wing = 0;
    skos_wing_mr=0;  skos_wing_f=0;
    cxawing = 0;     cyawing = 0;
    cx_wing = 0;     cy_wing = 0;
    X_wing  = 0;     Y_wing  = 0;
    if(solveWing)
    {
        //коэффициент, учитывающий влияние сужения крыла
        Ksi1wing = 0.8;///0.5;
        //индуктивная скорость
        duo uth_wing = f_uth( x_w, y_w, Ksi1wing);
        ut_wing_mr = uth_wing.a;//индуктивная скорость, спроецированная на ось Oy СКНВ, м/с
        uh_wing_mr = uth_wing.b;//индуктивная скорость, спроецированная на ось Oх СКНВ, м/с
        //Скорость у крыла от набегающего потока и НВ, м/с
        V_wing = sqrt( pow(Vx_mr-uh_wing_mr,2) + pow(Vy_mr+ut_wing_mr,2) );
        //Скоростной напор у крыла, Па
        q_wing = 0.5*roh*V_wing*V_wing;
        //Угол между направлением потока за винтом и осью Ox СКНВ у крыла, рад
        i1_mr_wing = atan( (Vy_mr+ut_wing_mr) / (Vx_mr-uh_wing_mr) );
        //Скос потока у крыла от НВ, рад
        skos_wing_mr_coarse = (Vi>9)?( 0.5*Fy_mr/(roh*Vi*Vi*Fmr) ):(88.88*d2r);
        skos_wing_mr = i1_mr_wing + alfa_mr;//около: +2...+5 градус [Володко'84,c169];
        if(skos_mr_) skos_wing_mr = skos_mr_;
        //Скос потока у крыла от фюзеляжа, рад
        skos_wing_f = 0;///f_dalf_wingf(alfa_f);
        //Угол атаки крыла, рад
        alfa_wing = alfa0f + eps_wing - skos_wing_mr - skos_wing_f;

        //аэродинамические коэф.крыла в скоростной СК
        cxawing = f_cx_wing(alfa_wing);
        cyawing = f_cy_wing(alfa_wing);
        //аэродинамические коэф.крыла в СК, связанной с профилем крыла
        cx_wing = cxawing*cos(alfa_wing) - cyawing*sin(alfa_wing);
        cy_wing = cyawing*cos(alfa_wing) + cxawing*sin(alfa_wing);
        //аэродин.сила на крыле в СК, связанной с профилем крыла
         X_wing = q_wing *cx_wing *S_wing *K_wing;
         Y_wing = q_wing *cy_wing *S_wing *K_wing;
        //аэродинамические силы на крыле (в СК, параллельной связанной СКвертолета)
        Fx_wing =-X_wing*cos(-eps_wing) + Y_wing*sin(-eps_wing);
        Fy_wing = Y_wing*cos(-eps_wing) + X_wing*sin(-eps_wing);
        Fz_wing = 0;
        Mx_wing = 0;
        My_wing = 0;
        Mz_wing = 0;
    }// */
    //Силы и моменты от крыла в ЦТ вертолета
    FMadw.Fx = Fx_wing;
    FMadw.Fy = Fy_wing;
    FMadw.Fz = Fz_wing;
    FMadw.Mx = Mx_wing + Fz_wing*(y_w-yct) - Fy_wing*(z_w-zct);
    FMadw.My = My_wing - Fz_wing*(x_w-xct) + Fx_wing*(z_w-zct);
    FMadw.Mz = Mz_wing + Fy_wing*(x_w-xct) - Fx_wing*(y_w-yct);
}

void Helicopter::StabAirDynForces()//Аэродинамические силы и моменты от стабилизатора
{
    Fz_st = 0;
    Mz_st = 0;
    Mx_st = 0;
    My_st = 0;
    FMads.Fx = 0;    FMads.Mx = 0;
    FMads.Fy = 0;    FMads.My = 0;
    FMads.Fz = 0;    FMads.Mz = 0;
    switch(solveStab)
    {
    case 0://по заданным усилиям на оси стабилизатора
        {
        skos_st_mr= 0;
        skos_st_f = 0;
        dalf_st_tng=0;
         eps_st = 0;
        alfa_st = 0;
        //силы и моменты от стабилизатора приведенные к ЦТ вертолета (в СКС)
        FMads.Fx = Fx_st;
        FMads.Fy = Fy_st;
        FMads.Fz = Fz_st;
        FMads.Mx = Mx_st + (y_st-yct)*Fz_st - (z_st-zct)*Fy_st;
        FMads.My = My_st - (x_st-xct)*Fz_st + (z_st-zct)*Fx_st;
        FMads.Mz = Mz_st + (x_st-xct)*Fy_st - (y_st-yct)*Fx_st;
        }
        break;

    case 1://Упрощенный расчет аэродинамических сил и моментов от стабилизатора в ц.т.вертолета
        {
        //коэффициент, учитывающий влияние сужения крыла
        Ksi1st = 0.5;//0.9;///0.5
        //индуктивная скорость HB
        duo uth_st = f_uth( x_st, y_st, Ksi1st);
        ut_st_mr = uth_st.a;//ut - 5.63396;//индуктивная скорость, спроецированная на ось Oy СКНВ, м/с
        uh_st_mr = uth_st.b;//uh - 0.76518;//индуктивная скорость, спроецированная на ось Oх СКНВ, м/с
        //Скорость потока у стабилизатора, м/с
        V_st = sqrt( pow(Vx_mr-uh_st_mr +u_pr,2) + pow(Vy_mr+ut_st_mr,2) );//
        //Скоростной напор у стабилизатора, Па
        q_st = 0.5*roh*V_st*V_st;
        //Угол между направлением потока за винтом и осью Ox СКНВ у стабилизатора, рад
        i1_mr_st = atan( (Vy_mr+ut_st_mr) / (Vx_mr-uh_st_mr +u_pr) );//
        //Скос потока у стабилизатора от НВ, рад
        skos_st_mr_coarse = (Vi>9)?( Fy_mr / (roh*Vi*Vi*Fmr) ):(88.88*d2r);
        skos_st_mr = (fabs(alfa_mr)>0.4*M_PI)?(0):fabs(alfa_mr + i1_mr_st);
        if(skos_mr_) skos_st_mr = skos_mr_;
        //Скос потока у стабилизатора от фюзеляжа, рад
        skos_st_f = f_dalf_stf(alfa_f);
        //влияние крыла, если оно есть
        if(Y_wing)
        {
            //удлинение крыла
            lmbd_wing = l_wing*l_wing/S_wing;
            //расстояние между фокусами крыла и стабилизатора
            Lstw = fabs(x_st-x_w);
            //угол между линией фокусов крыла и стабилизатора и СГФ
            a_stw = (Lstw)?( asin( (y_st-y_w)/Lstw ) ):(0);
            //угол между скоростью и линией фокусов крыла и стабилизатора
            alfa_stw = alfa_wing - a_stw;
            //отн.расстояния от стабилизатора до оси вихревой пелены крыла
            _y_ = fabs(2*Lstw*sin(alfa_stw)/l_wing);
            //поправка, зависит от отн.расстояния от стабилизатора до оси вихревой пелены крыла.
            xi_co = (_y_<2)?( 1.02 -1.21*_y_ +0.0778*pow(_y_,2) +0.677*pow(_y_,3) -0.422*pow(_y_,4) +0.076*pow(_y_,5) ):(0);
            //поправка, учитывает влияние сужения крыла
            double xi1 = 0.9;
            //Скос потока у стабилизатора от крыла, рад
            skos_st_wing = (lmbd_wing)?(  0.8 *xi1 *xi_co *cyawing /lmbd_wing  ):(0);
        }
        else skos_st_wing = 0;
        //изменение угла атаки стабилизатора от изменения тангажа, рад
        double signwz = (w.z)?(w.z/fabs(w.z)):(0);
        dalf_st_tng = (V_st)?( -w.z*(x_st-xct)/V_st/sqrt(K_st) ): (90*d2r*signwz);// x_st отрицательная
        //Угол установки стабилизатора относительно СГ фюзеляжа, рад
        eps_st = f_eps_st(fi0_mr);//eps_st_mr + eps_fz;

        //Угол атаки стабилизатора, рад
        alfa_st = alfa0f + eps_st - skos_st_mr - skos_st_f - skos_st_wing + dalf_st_tng;//положительный скос уменьшает угол атаки (закручивает поток в соответствии с СКС)

        //аэродинамические коэф.стабилизатора в скоростной СК
        cxast = f_cx_st(alfa_st);
        cyast = f_cy_st(alfa_st);
        //аэродинамические коэф. в СК, связанной с профилем стабилизатора
        cx_st = cxast*cos(alfa_st) - cyast*sin(alfa_st);
        cy_st = cyast*cos(alfa_st) + cxast*sin(alfa_st);
        //аэродин.сила на стабилизаторе в СК, связанной с профилем стабилизатора
         X_st = q_st *cx_st *S_st *K_st;
         Y_st = q_st *cy_st *S_st *K_st;
        //аэродин.сила на оси стабилизатора (в СКC)
        Fx_st =-X_st*cos(-eps_st) + Y_st*sin(-eps_st);
        Fy_st = Y_st*cos(-eps_st) + X_st*sin(-eps_st);
        Fz_st = 0;
        Mz_st = 0;
        Mx_st = 0;
        My_st = 0;
        //Силы и моменты от стабилизатора приведенные к ЦТ вертолета (в СКС)
        FMads.Fx = Fx_st;
        FMads.Fy = Fy_st;
        FMads.Fz = Fz_st;
        FMads.Mx = Mx_st + Fz_st*(y_st-yct) - Fy_st*(z_st-zct);
        FMads.My = My_st - Fz_st*(x_st-xct) + Fx_st*(z_st-zct);
        FMads.Mz = Mz_st + Fy_st*(x_st-xct) - Fx_st*(y_st-yct);
        }
        break;

    case 2://Расчет аэродинамических сил и моментов от стабилизатора в ц.т.вертолета (по методике балансировки) Ми-2
        {
        for(int i=1; i<__alfa_f_col-1; i++)
        {
            _alfafst = __alfa_f[i];     //
            //Угол установки стабилизатора при котором моменты вариантов со стабилизатором и без стабилизатора равны
            __eps_st0[i] = SolvEquation( 3, __eps_st[1], __eps_st[__eps_st_col-2] );//f_e0root3()
            //Угол скоса потока у стабилизатора от фюзеляжа, рад
            _dalf_stf[i] = _alfafst + __eps_st0[i];
        }
        __eps_st0[0] = __eps_st0[1];
        __eps_st0[__alfa_f_col-1] = __eps_st0[__alfa_f_col-2];
        __eps_st0[__alfa_f_col-0] = __eps_st0[__alfa_f_col-2];
        _dalf_stf[0] = _dalf_stf[1];
        _dalf_stf[__alfa_f_col-1] = _dalf_stf[__alfa_f_col-2];
        _dalf_stf[__alfa_f_col-0] = _dalf_stf[__alfa_f_col-2];
        //коэффициент, учитывающий влияние сужения крыла
        Ksi1st = 1;
        //...индуктивная скорость
        duo uth_st = f_uth( x_st, y_st, Ksi1st);
        ut_st_mr = uth_st.a;//ut - 5.63396;//индуктивная скорость, спроецированная на ось Oy, м/с
        uh_st_mr = uth_st.b;//uh - 0.76518;//индуктивная скорость, спроецированная на ось Oх, м/с
        //Скорость у стабилизатора от набегающего потока и НВ, м/с
        V_st = sqrt( pow(Vx_mr-uh_st_mr,2) + pow(Vy_mr+ut_st_mr,2) );
        //Скоростной напор в расч.точке у фюзеляжа
        q_st = 0.5*roh*V_st*V_st;
        //Угол между направлением потока за винтом и осью Ox у стабилизатора, рад
        i1_mr_st = atan( (Vy_mr+ut_st_mr) / (Vx_mr-uh_st_mr) );
        //Скос потока у стабилизатора от НВ, рад //знак перед chi_!!! и смотри определение alfa_st
        skos_st_mr = i1_mr_st + alfa_mr;
        //Угол скоса потока от фюзеляжа у стабилизатора, рад
        skos_st_f = f_dalf_stf2(alfa_f);//_regress
        //Скос потока на стабилизаторе, рад
        skos_st = skos_st_mr + skos_st_f;
        //Угол установки стабилизатора относительно КПНВ при котором mz.st=0, рад
        eps_st0mr = f_eps_st0(alfa_f) - eps_fz;//_regress
        //Угол установки стабилизатора относительно КПНВ, рад
        double eps_st_mr = f_eps_st(fi0_mr);
        //Угол атаки стабилизатора, рад
        alfa_st = alfa_mr + eps_st_mr - skos_st;/// +D1_mr*chi_ap_
        //Угол установки стабилизатора в продувке, рад
        eps_st_pr_ = alfa_st + f_eps_st0(alfa_f);//_regress
        //eps_st_pr
        double lim_estpr = 10*M_PI/180;
        if(eps_st_pr_ <-lim_estpr)   eps_st_pr = -lim_estpr;
        if(eps_st_pr_ > lim_estpr)   eps_st_pr =  lim_estpr;
        if((eps_st_pr_< lim_estpr)&&(eps_st_pr_> -lim_estpr)) eps_st_pr = eps_st_pr_;
        //аэродин.сила+момент от стабилизатора в СКC
        Fx_st = f_cx_st2(eps_st_pr, alfa_f) *0.5*roh*V_st*V_st*Smod;
        Fy_st = f_cy_st2(eps_st_pr, alfa_f) *0.5*roh*V_st*V_st*Smod;
        Fz_st = 0;
        Mz_st = f_mz_st2(eps_st_pr, alfa_f) *0.5*roh*V_st*V_st*Smod*Lmod;
        Mx_st = 0;
        My_st = 0;
        //Силы и моменты от стабилизатора: (СК на оси НВ)
        //FMads.Fx = Fx_st*cos(-eps_f) +Fy_st*sin(-eps_f);
        //FMads.Fy =-Fx_st*sin(-eps_f) +Fy_st*cos(-eps_f);
        //FMads.Fz = Fz_st;
        //FMads.Mx = z_st*(Fx_st*sin(-eps_f)-Fy_st*cos(-eps_f)) +y_st*Fz_st +Mx_st*cos(-eps_f) +My_st*sin(-eps_f);
        //FMads.My = z_st*(Fx_st*cos(-eps_f)+Fy_st*sin(-eps_f)) -x_st*Fz_st -Mx_st*sin(-eps_f) +My_st*cos(-eps_f);
        //FMads.Mz = Mz_st;// -x_st*( Fx_st*sin(-eps_f) + Fy_st*cos(-eps_f) ) -y_st*( Fx_st*cos(-eps_f) + Fy_st*sin(-eps_f) );
        //Силы и моменты от стабилизатора приведенные к ЦТ вертолета (в СКС)
        FMads.Fx = Fx_st;
        FMads.Fy = Fy_st;
        FMads.Fz = Fz_st;
        FMads.Mx = Mx_st;                      /// + (y_st-yct)*Fz_st - (z_st-zct)*Fy_st;//2020.03
        FMads.My = My_st;                      /// - (x_st-xct)*Fz_st + (z_st-zct)*Fx_st;//2020.03
        FMads.Mz = Mz_st + yct*Fx_st - xct*Fy_st;/// + (x_st-xct)*Fy_st - (y_st-yct)*Fx_st;//2020.03
        }
        break;
    }
}

void Helicopter::KeelAirDynForces()//Аэродинамические силы и моменты от киля
{
    Fy_keel = 0;
    Mz_keel = 0;
    Mx_keel = 0;
    My_keel = 0;
    FMadk.Fx = 0;    FMadk.Mx = 0;
    FMadk.Fy = 0;    FMadk.My = 0;
    FMadk.Fz = 0;    FMadk.Mz = 0;
    if(solveKeel)
    {
        //коэффициент, учитывающий влияние сужения крыла
        Ksi1keel = 0.5;
        //индуктивная скорость HB
        duo uth_keel = f_uth( x_k, y_k, Ksi1keel);
        ut_keel_mr = uth_keel.a;//ut - 5.63396;//индуктивная скорость, спроецированная на ось Oy СКНВ, м/с
        uh_keel_mr = uth_keel.b;//uh - 0.76518;//индуктивная скорость, спроецированная на ось Oх СКНВ, м/с
        //Скорость у крыла от набегающего потока и НВ, м/с
        V_keel = sqrt( pow(Vx_mr-uh_keel_mr +u_pr,2) + pow(Vy_mr+ut_keel_mr,2) );//sqrt( pow(Vx_mr,2) + pow(Vy_mr,2) );//
        //Скоростной напор у крыла, Па
        q_keel = 0.5*roh*V_keel*V_keel;
        //скос потока у киля от фюзеляжа
        skos_k_f = 0;//f_dbet_k_f(beta_sk);
        //индуктивная скорость HВ
        vs_mr = (mrsolve)?(mr->vs):( vs_mr_ );
        //Скос потока у киля от НВ, рад
        skos_k_mr = (V_keel>9)?( vs_mr /V_keel/sqrt(K_keel) ):(0);//vs_mr противоположно направлен S_mr
        //Влияние рулевого винта
        if(trmount)
        {
            //индуктивная скорость РВ
            u_tr = (trsolve)?(tr->vt):( u_tr_ );
            //поправка, зависит от отн.расстояния (z_tr-z_k)/Rtr и следом().
            xi_co = f_Xco( fabs(z_tr-z_k) *2/Dtr );
            //Скос потока у киля от РВ, рад
            skos_k_tr = (V_keel)?( -u_tr *xi_co /V_keel/sqrt(K_keel) ):(0);//-90*d2r* vt_tr/fabs(vt_tr)
        }
        else skos_k_tr = 0;
        //изменение угла атаки киля от рыскания, рад
        double signwy = (w.y)?(w.y/fabs(w.y)):(0);
        dalf_k_rsk = (V_keel)?( -w.y*(x_k-xct)/V_keel/sqrt(K_keel) ):(90*d2r*signwy);// x_к отрицательная
        //угол атаки киля, рад
        alfa_keel = eps_keel + beta_sk + dalf_k_rsk - skos_k_mr - skos_k_tr - skos_k_f;//положительный скос уменьшает угол атаки (закручивает поток в соответствии с СКС)

        //аэродинамические коэф.киля в скоростной СК
        cxakeel = f_cx_keel(alfa_keel);
        cyakeel = f_cy_keel(alfa_keel);
        //аэродинамические коэф.киля в СК, связанной с профилем киля
        cx_keel = cxakeel*cos(alfa_keel) - cyakeel*sin(alfa_keel);
        cy_keel = cyakeel*cos(alfa_keel) + cxakeel*sin(alfa_keel);
        //аэродин.сила на киле в СК, связанной с профилем киля
        X_keel = q_keel *cx_keel *S_keel *K_keel;
        Y_keel = q_keel *cy_keel *S_keel *K_keel;
        //аэродин.сила на оси киля (в СК, параллельной связанной СКвертолета)
        Fx_keel =-(X_keel*cos(-eps_keel) - Y_keel*sin(-eps_keel));
        Fz_keel =-(Y_keel*cos(-eps_keel) + X_keel*sin(-eps_keel));
        Fy_keel = 0;
        Mx_keel = 0;
        My_keel = 0;
        Mz_keel = 0;
    }
    //Силы и моменты от киля в ЦТ вертолета
    FMadk.Fx = Fx_keel;
    FMadk.Fy = Fy_keel;
    FMadk.Fz = Fz_keel;
    FMadk.Mx = Mx_keel + Fz_keel*(y_k-yct) - Fy_keel*(z_k-zct);
    FMadk.My = My_keel - Fz_keel*(x_k-xct) + Fx_keel*(z_k-zct);
    FMadk.Mz = Mz_keel + Fy_keel*(x_k-xct) - Fx_keel*(y_k-yct);
}
double Helicopter::f_Xco(double _y_)
{//поправка, учитывающая положение крыла относительно следа винта (Трошин,с.92-93)
    return (_y_<=0.9)?( 1 - 2.4345*_y_ + 4.536*pow(_y_,2) - 3.333*pow(_y_,3) ):(0.5);
}


//коеффициенты
void Helicopter::initial_airdyn_characters()//оцифрованные графики для аэродинамических коеффициентов
{
    //коэффициент взаимоиндукции//коэффициент, характеризующий изменение скоса при удалении от крыла в направлении, перпендикулярном к скорости невозмущенного потока.
    double ini_h[]     = { 0.00,    0.10,           0.20,           0.30,           0.40,           0.50,           0.60,           0.70,           0.80        };
    double ini_Ksi3n[] = { 1.00,	0.873133651, 	0.755095219, 	0.659393138, 	0.574123663, 	0.501743765, 	0.443244938, 	0.399866578, 	0.367642649 };
    _h_col  = sizeof(ini_h)/sizeof(double);
    int i=0;
    while(i<_h_col)
    {
        _Ksi3n_[i] = ini_Ksi3n[i];
        _h_[i]     = ini_h[i];
        i++;
    }

    //ИНДУКТИВНАЯ СКОРОСТЬ ОТ СКОРОСТИ V И УГЛА delta Из Юрьева
   double ini_Vvol_mr[] = { 0.00,	0.05,	0.10,	0.15,	0.20,	0.25,	0.30,	0.35,	0.40,	0.45,	0.50,	0.55,	0.60,	0.65,	0.70,	0.75,	0.80,	0.85,	0.90,	0.95,
                            1.00,	1.05,	1.10,	1.15,	1.20,	1.25,	1.30,	1.35,	1.40,	1.45,	1.50,	1.55,	1.60,	1.65,	1.70,	1.75,	1.80,	1.85,	1.90,	1.95,
                            2.00,	2.05,	2.10,	2.15,	2.20,	2.25,	2.30,	2.35,	2.40,	2.45,	2.50,	2.55,	2.60,	2.65,	2.70,	2.75,	2.80,	2.85,	2.90,	2.95,
                            3.00,	3.05,	3.10,	3.15,	3.20,	3.25,	3.30,	3.35,	3.40,	3.45,	3.50,	3.55,	3.60,	3.65,	3.70,	3.75,	3.80,	3.85,	3.90,	3.95,
                            4.00,	4.05,	4.10,	4.15,	4.20,	4.25,	4.30,	4.35,	4.40,	4.45,	4.50,	4.55,	4.60,	4.65,	4.70,	4.75,	4.80,	4.85,	4.90,	4.95,
                            5.00 };
   double ini_delta_mr[]= { -1.5708, -1.0472, -0.5236, -0.2618, 0.000, 0.2618, 0.5236, 1.0472, 1.5708 };
   double  ini_uvol_mr[]= { 1.000,	1.000,	1.000,	1.000,	1.000,	1.000,	1.000,	1.000,	1.000,
                            0.975,	0.983,	0.991,	0.995,	0.998,	1.005,	1.007,	1.020,	1.030,
                            0.952,	0.966,	0.981,	0.990,	0.996,	1.009,	1.016,	1.041,	1.063,
                            0.928,	0.948,	0.970,	0.983,	0.994,	1.013,	1.025,	1.062,	1.099,
                            0.905,	0.930,	0.958,	0.975,	0.991,	1.016,	1.034,	1.084,	1.137,
                            0.883,	0.912,	0.944,	0.966,	0.987,	1.018,	1.043,	1.103,	1.175,
                            0.862,	0.893,	0.927,	0.956,	0.982,	1.018,	1.051,	1.119,	1.214,
                            0.841,	0.873,	0.909,	0.944,	0.976,	1.016,	1.057,	1.134,	1.251,
                            0.820,	0.854,	0.891,	0.931,	0.968,	1.012,	1.062,	1.150,	1.289,
                            0.800,	0.834,	0.872,	0.916,	0.959,	1.007,	1.065,	1.169,	1.327,
                            0.781,	0.814,	0.854,	0.900,	0.949,	1.000,	1.067,	1.188,	1.365,
                            0.762,	0.794,	0.835,	0.882,	0.938,	0.991,	1.068,	1.206,	1.404,
                            0.744,	0.775,	0.815,	0.863,	0.926,	0.980,	1.067,	1.223,	1.442,
                            0.727,	0.756,	0.797,	0.845,	0.913,	0.968,	1.065,	1.238,	1.478,
                            0.710,	0.738,	0.781,	0.828,	0.899,	0.956,	1.062,	1.251,	1.512,
                            0.694,	0.721,	0.765,	0.810,	0.884,	0.943,	1.057,	1.264,	1.546,
                            0.679,	0.705,	0.749,	0.793,	0.869,	0.928,	1.050,	1.274,	1.580,
                            0.664,	0.690,	0.733,	0.777,	0.852,	0.911,	1.041,	1.284,	1.617,
                            0.650,	0.676,	0.718,	0.760,	0.835,	0.894,	1.030,	1.291,	1.656,
                            0.636,	0.662,	0.702,	0.743,	0.818,	0.878,	1.017,	1.297,	1.695,
                            0.622,	0.648,	0.687,	0.727,	0.800,	0.861,	1.003,	1.300,	1.730,
                            0.609,	0.635,	0.671,	0.712,	0.782,	0.842,	0.988,	1.302,	1.757,
                            0.596,	0.622,	0.656,	0.697,	0.764,	0.822,	0.970,	1.303,	1.777,
                            0.584,	0.610,	0.643,	0.682,	0.745,	0.802,	0.951,	1.302,	1.794,
                            0.572,	0.597,	0.630,	0.667,	0.726,	0.782,	0.928,	1.298,	1.809,
                            0.560,	0.585,	0.617,	0.652,	0.707,	0.763,	0.904,	1.290,	1.817,
                            0.548,	0.573,	0.602,	0.638,	0.690,	0.744,	0.880,	1.278,	1.820,
                            0.536,	0.561,	0.587,	0.624,	0.673,	0.725,	0.854,	1.262,	1.817,
                            0.525,	0.549,	0.573,	0.611,	0.656,	0.707,	0.828,	1.241,	1.813,
                            0.513,	0.538,	0.562,	0.599,	0.640,	0.689,	0.799,	1.216,	1.803,
                            0.502,	0.526,	0.552,	0.586,	0.624,	0.671,	0.771,	1.186,	1.776,
                            0.491,	0.515,	0.541,	0.574,	0.611,	0.654,	0.743,	1.152,	1.725,
                            0.481,	0.504,	0.529,	0.562,	0.598,	0.637,	0.717,	1.114,	1.652,
                            0.470,	0.494,	0.518,	0.549,	0.585,	0.621,	0.693,	1.073,	1.562,
                            0.460,	0.483,	0.507,	0.537,	0.571,	0.605,	0.671,	1.026,	1.466,
                            0.451,	0.473,	0.497,	0.526,	0.557,	0.590,	0.650,	0.970,	1.372,
                            0.442,	0.463,	0.486,	0.515,	0.544,	0.575,	0.630,	0.902,	1.289,
                            0.433,	0.453,	0.476,	0.504,	0.531,	0.560,	0.610,	0.834,	1.222,
                            0.425,	0.444,	0.466,	0.493,	0.519,	0.547,	0.591,	0.780,	1.171,
                            0.417,	0.435,	0.456,	0.482,	0.505,	0.534,	0.573,	0.742,	1.112,
                            0.409,	0.426,	0.447,	0.472,	0.493,	0.522,	0.557,	0.710,	1.000,
                            0.402,	0.418,	0.437,	0.463,	0.481,	0.510,	0.542,	0.676,	0.829,
                            0.395,	0.410,	0.428,	0.454,	0.471,	0.498,	0.528,	0.641,	0.740,
                            0.388,	0.402,	0.419,	0.444,	0.461,	0.486,	0.514,	0.611,	0.706,
                            0.381,	0.394,	0.411,	0.434,	0.450,	0.474,	0.500,	0.587,	0.662,
                            0.374,	0.387,	0.402,	0.424,	0.440,	0.462,	0.487,	0.566,	0.619,
                            0.368,	0.380,	0.394,	0.414,	0.431,	0.452,	0.473,	0.545,	0.586,
                            0.361,	0.373,	0.386,	0.406,	0.422,	0.441,	0.460,	0.524,	0.561,
                            0.355,	0.367,	0.379,	0.398,	0.413,	0.431,	0.449,	0.505,	0.539,
                            0.349,	0.360,	0.372,	0.390,	0.404,	0.421,	0.438,	0.488,	0.519,
                            0.342,	0.354,	0.365,	0.382,	0.395,	0.411,	0.428,	0.473,	0.501,
                            0.337,	0.348,	0.359,	0.374,	0.386,	0.402,	0.418,	0.458,	0.484,
                            0.331,	0.342,	0.353,	0.367,	0.378,	0.393,	0.409,	0.445,	0.469,
                            0.325,	0.336,	0.346,	0.360,	0.371,	0.384,	0.399,	0.432,	0.455,
                            0.319,	0.330,	0.340,	0.353,	0.364,	0.376,	0.390,	0.420,	0.442,
                            0.314,	0.324,	0.334,	0.346,	0.357,	0.368,	0.381,	0.409,	0.430,
                            0.309,	0.319,	0.328,	0.339,	0.350,	0.360,	0.372,	0.399,	0.419,
                            0.304,	0.313,	0.322,	0.332,	0.344,	0.352,	0.363,	0.389,	0.409,
                            0.299,	0.308,	0.317,	0.325,	0.338,	0.345,	0.355,	0.380,	0.399,
                            0.295,	0.303,	0.311,	0.319,	0.331,	0.337,	0.347,	0.372,	0.390,
                            0.290,	0.298,	0.306,	0.313,	0.326,	0.330,	0.339,	0.364,	0.381,
                            0.286,	0.293,	0.300,	0.307,	0.320,	0.323,	0.331,	0.356,	0.372,
                            0.282,	0.288,	0.295,	0.301,	0.314,	0.317,	0.324,	0.348,	0.364,
                            0.278,	0.283,	0.290,	0.295,	0.309,	0.310,	0.316,	0.341,	0.356,
                            0.274,	0.278,	0.285,	0.289,	0.304,	0.304,	0.309,	0.334,	0.349,
                            0.270,	0.274,	0.280,	0.284,	0.298,	0.298,	0.303,	0.327,	0.342,
                            0.266,	0.269,	0.276,	0.279,	0.293,	0.292,	0.296,	0.321,	0.335,
                            0.262,	0.265,	0.271,	0.274,	0.288,	0.286,	0.289,	0.314,	0.328,
                            0.258,	0.261,	0.267,	0.269,	0.284,	0.280,	0.283,	0.308,	0.322,
                            0.254,	0.256,	0.262,	0.264,	0.279,	0.275,	0.277,	0.302,	0.316,
                            0.251,	0.252,	0.258,	0.259,	0.275,	0.269,	0.272,	0.296,	0.310,
                            0.247,	0.249,	0.254,	0.255,	0.270,	0.264,	0.266,	0.291,	0.305,
                            0.244,	0.245,	0.250,	0.250,	0.266,	0.259,	0.261,	0.285,	0.299,
                            0.240,	0.241,	0.246,	0.246,	0.262,	0.255,	0.256,	0.281,	0.295,
                            0.237,	0.238,	0.242,	0.242,	0.259,	0.250,	0.251,	0.276,	0.290,
                            0.234,	0.234,	0.239,	0.238,	0.255,	0.246,	0.246,	0.271,	0.285,
                            0.231,	0.231,	0.235,	0.234,	0.252,	0.242,	0.242,	0.267,	0.281,
                            0.229,	0.228,	0.232,	0.231,	0.249,	0.238,	0.237,	0.263,	0.277,
                            0.226,	0.225,	0.229,	0.227,	0.245,	0.234,	0.233,	0.259,	0.272,
                            0.223,	0.222,	0.226,	0.224,	0.242,	0.230,	0.229,	0.255,	0.268,
                            0.221,	0.219,	0.223,	0.221,	0.239,	0.227,	0.226,	0.252,	0.264,
                            0.218,	0.216,	0.220,	0.218,	0.236,	0.224,	0.222,	0.248,	0.260,
                            0.216,	0.213,	0.217,	0.215,	0.233,	0.221,	0.219,	0.244,	0.256,
                            0.214,	0.211,	0.214,	0.212,	0.230,	0.218,	0.216,	0.241,	0.252,
                            0.211,	0.208,	0.212,	0.210,	0.227,	0.215,	0.214,	0.238,	0.248,
                            0.209,	0.206,	0.209,	0.207,	0.225,	0.212,	0.211,	0.235,	0.245,
                            0.207,	0.204,	0.207,	0.205,	0.222,	0.210,	0.209,	0.231,	0.241,
                            0.205,	0.202,	0.205,	0.203,	0.219,	0.208,	0.207,	0.229,	0.238,
                            0.203,	0.200,	0.203,	0.201,	0.217,	0.206,	0.205,	0.226,	0.235,
                            0.201,	0.198,	0.201,	0.200,	0.214,	0.204,	0.203,	0.223,	0.232,
                            0.199,	0.196,	0.199,	0.198,	0.212,	0.203,	0.202,	0.221,	0.229,
                            0.197,	0.195,	0.198,	0.197,	0.210,	0.201,	0.201,	0.218,	0.227,
                            0.195,	0.193,	0.196,	0.195,	0.208,	0.200,	0.200,	0.216,	0.224,
                            0.193,	0.192,	0.195,	0.194,	0.206,	0.199,	0.199,	0.214,	0.222,
                            0.191,	0.190,	0.193,	0.193,	0.204,	0.198,	0.198,	0.212,	0.219,
                            0.190,	0.189,	0.192,	0.192,	0.202,	0.197,	0.198,	0.210,	0.217,
                            0.188,	0.188,	0.191,	0.192,	0.200,	0.197,	0.198,	0.208,	0.214,
                            0.186,	0.187,	0.190,	0.191,	0.198,	0.197,	0.198,	0.206,	0.212,
                            0.185,	0.186,	0.189,	0.191,	0.197,	0.196,	0.198,	0.205,	0.209,
                            0.183,	0.186,	0.188,	0.191,	0.195,	0.196,	0.199,	0.204,	0.207,
                            0.182,	0.185,	0.188,	0.191,	0.194,	0.197,	0.200,	0.202,	0.205 };
    _Vvol_mr_col    = sizeof( ini_Vvol_mr)/sizeof(double);
    _delta_mr_col   = sizeof(ini_delta_mr)/sizeof(double);
    int _uvol_mr_col= sizeof( ini_uvol_mr)/sizeof(double);
    int v=0, d=0; i=0;
    while(i<_uvol_mr_col)
    {
        _uvol_mr[i] = ini_uvol_mr[i];
        i++;
    }
    while(v<_Vvol_mr_col)
    {
        _Vvol_mr[v] = ini_Vvol_mr[v];
        v++;
    }
    while(d<_delta_mr_col)
    {
        _delta_mr[d] = ini_delta_mr[d];
        d++;
    }
}
double Helicopter::f_cx_f(double alfa)//АЭРОДИН.ХАРАКТЕРИСТИКИ ПЛАНЕРА
{
return SplineXY( alfa, _alfa_f, _cx_f, _alfa_f_col );
}
double Helicopter::f_cy_f(double alfa)//АЭРОДИН.ХАРАКТЕРИСТИКИ ПЛАНЕРА
{
return SplineXY( alfa, _alfa_f, _cy_f, _alfa_f_col );
}
double Helicopter::f_cz_f(double alfa)//АЭРОДИН.ХАРАКТЕРИСТИКИ ПЛАНЕРА
{
return SplineXY( alfa, _alfa_f, _cz_f, _alfa_f_col );
}
double Helicopter::f_mx_f(double alfa)//АЭРОДИН.ХАРАКТЕРИСТИКИ ПЛАНЕРА
{
return SplineXY( alfa, _alfa_f, _mx_f, _alfa_f_col );
}
double Helicopter::f_my_f(double alfa)//АЭРОДИН.ХАРАКТЕРИСТИКИ ПЛАНЕРА
{
return SplineXY( alfa, _alfa_f, _my_f, _alfa_f_col );
}
double Helicopter::f_mz_f(double alfa)//АЭРОДИН.ХАРАКТЕРИСТИКИ ПЛАНЕРА, коэффициент момента фюзеляжа без стабилизатора
{
return SplineXY( alfa, _alfa_f, _mz_f, _alfa_f_col );
}
double Helicopter::f_cx_keel(double ak)//new
{
return SplineXY( ak, _alfa_keel, _cx_keel, _alfa_keel_col );
}
double Helicopter::f_cy_keel(double ak)//new
{
return SplineXY( ak, _alfa_keel, _cy_keel, _alfa_keel_col );
}
double Helicopter::f_cx_wing(double aw)//new
{
return SplineXY( aw, _alfa_wing, _cx_wing, _alfa_wing_col );
}
double Helicopter::f_cy_wing(double aw)//new
{
return SplineXY( aw, _alfa_wing, _cy_wing, _alfa_wing_col );
}
double Helicopter::f_cx_st(double ast) //new
{
return SplineXY( ast, _alfa_st, _cx_st, _alfa_st_col );
}
double Helicopter::f_cy_st(double ast) //new
{
return SplineXY( ast, _alfa_st, _cy_st, _alfa_st_col );
}
double Helicopter::f_dalf_stf(double afus)//new
{
return SplineXY( afus, _alfa_fs, _dalf_stf, _alfa_fs_col );
}

duo Helicopter::f_uth(double x,double y,double Ksi1)//Balancing:
{
    duo uth;
    Xmr_ = x_mr - x;//x_mr - x_f;
    Ymr_ = y - y_mr;//y_f - y_mr;
    tau_mr  = Xmr_*cos(i1_mr)-Ymr_*sin(i1_mr);//расстояние х до НВ  = xkr_mr
    nor_mr  = Ymr_*cos(i1_mr)+Xmr_*sin(i1_mr);//расстояние y до НВ  = ykr_mr
    double nor_mod = fabs(nor_mr);

    if(nor_mr<0) x1_mr = SolvEquation(1, -100, 100);
    else         x1_mr = SolvEquation(2, -100, 100);

    R1_mr  = f_R1(x1_mr);
    hflow_mr = R1_mr*sin(i1_mr);//высота струи в точке tau_mr
    double hflow_mod = fabs(hflow_mr);
    double _h_ = (nor_mod-hflow_mod)*2/Dmr;//относительное расстояние до спутной струи
    if(nor_mod < hflow_mod)
    {
        Ksi3n = 1.0;
        Ksi3t = 1.0;
    }
    else
    {
        Ksi3n = f_Ksi3n(_h_);//0.1296
        Ksi3t = 0.0;
    }
    Ksi2 = f_Ksi2(tau_mr*2/Dmr);
    uth.a = Ksi1*Ksi2*( (ut_mr*cos(i1_mr)+uh_mr*sin(i1_mr))*Ksi3n*cos(i1_mr) + (-uh_mr*cos(i1_mr)+ut_mr*sin(i1_mr))*Ksi3t*sin(i1_mr) );//ut_f_mr
    uth.b = Ksi1*Ksi2*( (uh_mr*cos(i1_mr)-ut_mr*sin(i1_mr))*Ksi3t*cos(i1_mr) + ( ut_mr*cos(i1_mr)+uh_mr*sin(i1_mr))*Ksi3n*sin(i1_mr) );//uh_f_mr

return uth;
}
double Helicopter::f_R1(double x)//for ut_f_mr,uh_f_mr
{    //double R_mr = 0.5*Dmr;
return Rmr*sqrt( (pow(Vx_mr-uh_mr,2) + pow(Vy_mr+ut_mr,2)) / (pow(Vx_mr-f_Ksi2(x/Rmr)*uh_mr,2) + pow( Vy_mr+f_Ksi2(x/Rmr)*ut_mr ,2)) );
}
double Helicopter::f_x1root1(double x, void *params)//for ut_f_mr,uh_f_mr
{
return phc->tau_mr + phc->f_R1(x)*cos(phc->i1_mr) - x;
}
double Helicopter::f_x1root2(double x, void *params)//for ut_f_mr,uh_f_mr
{
return phc->tau_mr - phc->f_R1(x)*cos(phc->i1_mr) - x;
}
double Helicopter::f_uvol_mr(double Vvol, double delta)//ИНДУКТИВНАЯ СКОРОСТЬ ОТ СКОРОСТИ V И УГЛА delta Из Юрьева
{
    double uvol = 0;
    if(Vvol>=5) uvol = 1.0/Vvol;
    else        uvol = SplineXYZ(Vvol,delta,  _Vvol_mr,_delta_mr,_uvol_mr,  _Vvol_mr_col, _delta_mr_col);
return uvol;
}
double Helicopter::f_Ksi3n(double h)//коэффициент взаимоиндукции
{//коэффициент, характеризующий изменение скоса при удалении от крыла в направлении, перпендикулярном к скорости невозмущенного потока.
return SplineXY( h, _h_, _Ksi3n_, _h_col );
}



// Аэродинамика планера по методике балансировки Ми-2
double Helicopter::f_e0root3(double x, void *params)
{
    double b2 = phc->f_mz_f(phc->_alfafst);
    double b1 = phc->f_mz_fst(x,phc->_alfafst);
return b1 - b2;
}
double Helicopter::f_dalf_stf2(double afus)
{
return SplineXY( afus, __alfa_f, _dalf_stf, __alfa_f_col );
}
double Helicopter::f_eps_st0(double afus)
{
return SplineXY( afus, __alfa_f, __eps_st0, __alfa_f_col );
}
double Helicopter::f_mz_fst(double est, double alfa)//АЭРОДИН.ХАРАКТЕРИСТИКИ ПЛАНЕРА, коэффициент момента фюзеляжа со стабилизатором
{
return SplineXYZ(est,alfa,  __eps_st,__alfa_f,_mz_fst,  __eps_st_col, __alfa_f_col);
}
double Helicopter::f_cx_st2(double est, double alfa)//АЭРОДИН.ХАРАКТЕРИСТИКИ СТАБИЛИЗАТОРА
{
return SplineXYZ(est,alfa,  __eps_st,__alfa_f,__cx_st,  __eps_st_col, __alfa_f_col);
}
double Helicopter::f_cy_st2(double est, double alfa)//АЭРОДИН.ХАРАКТЕРИСТИКИ СТАБИЛИЗАТОРА
{
return SplineXYZ(est,alfa,  __eps_st,__alfa_f,__cy_st,  __eps_st_col, __alfa_f_col);
}
double Helicopter::f_mz_st2(double est, double alfa)//АЭРОДИН.ХАРАКТЕРИСТИКИ СТАБИЛИЗАТОРА
{
return SplineXYZ(est,alfa,  __eps_st,__alfa_f,__mz_st,  __eps_st_col, __alfa_f_col);
}
double Helicopter::f_eps_st(double fi0)  //ЗАКОН УПРАВЛЕНИЯ СТАБИЛИЗАТОРОМ
{
    double k1 = 0;
    double xmin = fi0_min,
           xmax = fi0_max,
           ymin = est_min,
           ymax = est_max;
    if(xmax!=xmin) k1 = (ymax-ymin)/(xmax-xmin);

return ymin + k1*(fi0 - xmin);
}
double Helicopter::f_Ksi2(double x)//
{//коэффициент, учитывающий изменение скоса при удалении от крыла по направлению скорости невозмущенного потока;
    double Ksi2 = 0;
    pent  e1, e2, e3, e4;
    quat sr1,sr2,sr3,sr4;

    xc000 = 0.5;    xc015 = 1.2;    yc000 = 1.4;    yc015 = 2.0;    a0yb1 = 68.9;

    //system#1
    e1.a = 0;               e1.b = 0;               e1.c = 0;           e1.d = 1.0;         e1.e = xc000;
    e2.a = 0;               e2.b = 0;               e2.c = 1.0;         e2.d = 0;           e2.e = 0;
    e3.a = pow(0.15,3);     e3.b = pow(0.15,2);     e3.c = 0.15;        e3.d = 1.0;         e3.e = xc015;
    e4.a = 3*pow(0.15,2);   e4.b = 2*0.15;          e4.c = 1.0;         e4.d = 0;           e4.e = 0;
    sr1  = SolveSystem4x4(e1, e2, e3, e4);

    //system#2
    e1.a = 0;               e1.b = 0;               e1.c = 0;           e1.d = 1.0;         e1.e = yc000;
    e2.a = 3*0.0;           e2.b = 2*0.0;           e2.c = 1.0;         e2.d = 0;           e2.e = 0;
    e3.a = pow(0.15,3);     e3.b = pow(0.15,2);     e3.c = 0.15;        e3.d = 1.0;         e3.e = yc015;
    e4.a = 3*pow(0.15,2);   e4.b = 2*0.15;          e4.c = 1.0;         e4.d = 0;           e4.e = 0;
    sr2  = SolveSystem4x4(e1, e2, e3, e4);

    double V_  = 2*Vi/w_mr/Dmr;
    double x_c = f_xc(V_, sr1.a, sr1.b, sr1.c, sr1.d);
    double y_c = f_yc(V_, sr2.a, sr2.b, sr2.c, sr2.d);
    double x_a =-x_c;
    double y_a = 0;
    double x_b = 0;
    double y_b = 1;
    double y_a1= 0;
    double y_c1= 0;
    double y_b1= tan(a0yb1*M_PI/180.0);

    //system#3
    e1.a = pow(x_a,3);      e1.b = pow(x_a,2);      e1.c = x_a;         e1.d = 1;       e1.e = y_a;
    e2.a = pow(x_b,3);      e2.b = pow(x_b,2);      e2.c = x_b;         e2.d = 1;       e2.e = y_b;
    e3.a = 3*pow(x_a,2);    e3.b = 2*x_a;           e3.c = 1;           e3.d = 0;       e3.e = y_a1;
    e4.a = 3*pow(x_b,2);    e4.b = 2*x_b;           e4.c = 1;           e4.d = 0;       e4.e = y_b1;
    sr3  = SolveSystem4x4(e1, e2, e3, e4);

    //system#4
    e1.a = pow(x_c,3);      e1.b = pow(x_c,2);      e1.c = x_c;         e1.d = 1;       e1.e = y_c;
    e2.a = pow(x_b,3);      e2.b = pow(x_b,2);      e2.c = x_b;         e2.d = 1;       e2.e = y_b;
    e3.a = 3*pow(x_c,2);    e3.b = 2*x_c;           e3.c = 1;           e3.d = 0;       e3.e = y_c1;
    e4.a = 3*pow(x_b,2);    e4.b = 2*x_b;           e4.c = 1;           e4.d = 0;       e4.e = y_b1;
    sr4  = SolveSystem4x4(e1, e2, e3, e4);

    //double x_ = ksi2arg;
    if (x<x_a)              Ksi2 = 0;
    if ((x>=x_a)&&(x<x_b))  Ksi2 = sr3.a*pow(x,3) + sr3.b*pow(x,2) + sr3.c*x + sr3.d;
    if ((x>=x_b)&&(x<x_c))  Ksi2 = sr4.a*pow(x,3) + sr4.b*pow(x,2) + sr4.c*x + sr4.d;
    if (x >= x_c)           Ksi2 = y_c;

return Ksi2;
}
double Helicopter::f_xc(double si, double a,double b,double c,double d)//
{
    double xc = 0;
    if(si<0)                xc = xc000;
    if((si>=0)&&(si<0.15))  xc = a*pow(si,3) + b*pow(si,2) + c*si + d;
    if(si>=0.15)            xc = xc015;
return xc;
}
double Helicopter::f_yc(double si, double a,double b,double c,double d)//
{
    double yc = 0;
    if(si<0)                yc = yc000;
    if((si>=0)&&(si<0.15))  yc = a*pow(si,3) + b*pow(si,2) + c*si + d;
    if(si>=0.15)            yc = yc015;
return yc;
}
