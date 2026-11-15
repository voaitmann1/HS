#include <QtMath>
#include <math.h>
#include "shassi.h"
//-----------------------------------------------------------------------------
IShassi::~IShassi()
{
}

PyramidType::PyramidType(int pneumID, int absorbID)
{
    //type = typeID;
    switch (pneumID)
    {
        case 1:
            pn = new Pneumatic();
        break;
        default:
            pn = new Pneumatic();
    }
    switch (absorbID)
    {
       case 1:
            ab = new Absorber1cam();
       break;
       case 2:
            ab = new Absorber2cam();
       break;
       case 3:
            ab = new AbsorberMi2();
       break;
       default:
            ab = new Absorber2cam();
    }
}
PyramidType::~PyramidType()
{
        delete ab;
        delete pn;
}
LeverType::LeverType(int pneumID, int absorbID)
{
    //type = typeID;
    switch (pneumID)
    {
        case 1:
            pn = new Pneumatic();
        break;
        default:
            pn = new Pneumatic();
    }
    switch (absorbID)
    {
       case 1:
            ab = new Absorber1cam();
       break;
       case 2:
            ab = new Absorber2cam();
       break;
       case 3:
            ab = new AbsorberMi2();
       break;
       default:
            ab = new Absorber1cam();
    }
}
LeverType::~LeverType()
{
        delete ab;
        delete pn;
}

void IShassi::commonPre()
{
    status = 0;
    contact = 0;
    ab->status_=-2;  ab->status= 0;
    pn->status_= 0;  pn->status= 0;
    pn->h_.x = 0;    pn->h1_.x = 0;    pn->h2_.x = 0;
    pn->h_.y = 0;    pn->h1_.y = 0;    pn->h2_.y = 0;
    pn->h_.z = 0;    pn->h1_.z = 0;    pn->h2_.z = 0;
    Aa = 0;          Ap = 0;
    dOx = 0;         dOy = 0;//
    //радиус колеса, соответствующий максимально допустимому обжатию пневматика
    Rmd= 0.5*pn->Dp - pn->dpmd;
    WD = Rmd;
    //угол расположения стойки на планере: 0 - на правом борту, M_PI - на левом борту
    yrot = 0.5*M_PI - 0.5*M_PI*direct;
    //начальные координаты  точек
    O.x = O_.x;    O.y = O_.y;    O.z = O_.z;//координаты начала локальной СК
    A.x = A_.x;    A.y = A_.y;    A.z = A_.z;//верхняя точка амортстойки с карданами (неизменна в СКС)
    B.x = B_.x;    B.y = B_.y;    B.z = B_.z;//нижняя точка амортстойки в разжатом положении с карданами
    V.x = V_.x;    V.y = V_.y;    V.z = V_.z;//нижняя точка доп.звена в разжатом положении
    W.x = W_.x;    W.y = W_.y;    W.z = W_.z;//центр колеса в разжатом положении
    Q.x = Q_.x;    Q.y = Q_.y;    Q.z = Q_.z;//нижняя точка доп.звена в разжатом положении
    //длины условных стержней шасси
    OA = sqrt( A.x*A.x + A.y*A.y + A.z*A.z );
    OB = sqrt( B.x*B.x + B.y*B.y + B.z*B.z );
    OW = sqrt( W.x*W.x + W.y*W.y + W.z*W.z );
    BW = sqrt((B.x-W.x)*(B.x-W.x) + (B.y-W.y)*(B.y-W.y) + (B.z-W.z)*(B.z-W.z));
    VW = sqrt((V.x-W.x)*(V.x-W.x) + (V.y-W.y)*(V.y-W.y) + (V.z-W.z)*(V.z-W.z));//MI8
    OV = sqrt( V.x*V.x + V.y*V.y + V.z*V.z );//MI8
    BV = sqrt((B.x-V.x)*(B.x-V.x) + (B.y-V.y)*(B.y-V.y) + (B.z-V.z)*(B.z-V.z));//MI8
    OQ = sqrt( Q.y*Q.y + Q.x*Q.x );//mi2
    //проекции на XY (for LeverType)
    OAxy = sqrt( A.y*A.y + A.x*A.x );//const
    OBxy = sqrt( B.y*B.y + B.x*B.x );
    OWxy = sqrt( W.y*W.y + W.x*W.x );//const
    BWxy = sqrt((B.y-W.y)*(B.y-W.y) + (B.x-W.x)*(B.x-W.x));
    VWxy = sqrt((V.y-W.y)*(V.y-W.y) + (V.x-W.x)*(V.x-W.x));//MI8 const
    OVxy = sqrt( V.y*V.y + V.x*V.x );                      //MI8 const
    BVxy = sqrt((B.y-V.y)*(B.y-V.y) + (B.x-V.x)*(B.x-V.x));//MI8 const
    //проекции на YZ (for PyramidType)
    OAyz = sqrt( A.y*A.y + A.z*A.z );
    OByz = sqrt( B.y*B.y + B.z*B.z );
    OWyz = sqrt( W.y*W.y + W.z*W.z );
    BWyz = sqrt((B.y-W.y)*(B.y-W.y) + (B.z-W.z)*(B.z-W.z));
    //OVyz = sqrt( V.y*V.y + V.z*V.z );//MI8
    //BVyz = sqrt((B.y-V.y)*(B.y-V.y) + (B.z-V.z)*(B.z-V.z));//MI8
    //проекция на плоскость XY постоянного угла между линиями OB и OW
    if(stype!=1)
    {
        if(fabs(W.y/B.y)<fabs(W.x/B.x))
        {
        BOWxy =-acos( 0.5*(OWxy*OWxy + OBxy*OBxy - BWxy*BWxy)/OWxy/OBxy );
        BOVxy =-acos( 0.5*(OVxy*OVxy + OBxy*OBxy - BVxy*BVxy)/OVxy/OBxy );
        //VOWxy =-acos( 0.5*(OWxy*OWxy + OVxy*OVxy - VWxy*VWxy)/OWxy/OVxy );
        }
        else
        {
        BOWxy = acos( 0.5*(OWxy*OWxy + OBxy*OBxy - BWxy*BWxy)/OWxy/OBxy );
        BOVxy = acos( 0.5*(OVxy*OVxy + OBxy*OBxy - BVxy*BVxy)/OVxy/OBxy );
        //VOWxy = acos( 0.5*(OWxy*OWxy + OVxy*OVxy - VWxy*VWxy)/OWxy/OVxy );
        }

        if(fabs(W.y/V.y)<fabs(W.x/V.x))
        {
        VOWxy =-acos( 0.5*(OWxy*OWxy + OVxy*OVxy - VWxy*VWxy)/OWxy/OVxy );
        }
        else
        {
        VOWxy = acos( 0.5*(OWxy*OWxy + OVxy*OVxy - VWxy*VWxy)/OWxy/OVxy );
        }
    }
    else
    {
        BOWxy = -0.5*M_PI;//чтобы ang_sz=0
        BOVxy = 0;
        VOWxy = 0;
    }
    //проекция на плоскость YZ постоянного угла между линиями OB и OW
    if(stype==1)
    {
    if(fabs(W.y/B.y)<fabs(W.z/B.z))
         BOWyz =-acos( 0.5*(OWyz*OWyz + OByz*OByz - BWyz*BWyz)/OWyz/OByz );
    else BOWyz = acos( 0.5*(OWyz*OWyz + OByz*OByz - BWyz*BWyz)/OWyz/OByz );
    }
    else BOWyz = -0.5*M_PI;//чтобы ang_sx=0

    //длина амортстойки в разжатом положении
    AB  = AB_  = sqrt( (A.x-B.x)*(A.x-B.x) + (A.y-B.y)*(A.y-B.y) + (A.z-B.z)*(A.z-B.z));
    ABxy= ABxy_= sqrt( (A.y-B.y)*(A.y-B.y) + (A.x-B.x)*(A.x-B.x) );//проекция на xy
    AByz= AByz_= sqrt( (A.y-B.y)*(A.y-B.y) + (A.z-B.z)*(A.z-B.z) );//проекция на yz
    //длина амортстойки в предельно обжатом положении
    ABfull = AB_- ab->S;
    AByzfull= (AByz)*ABfull/AB;
    ABxyfull= (ABxy)*ABfull/AB;
    // подготовка объекта амортстойка
    ab->Preprocess();
    // подготовка объекта пневматик
    pn->Preprocess();
}

void PyramidType::Preprocess()// ОПРЕДЕЛЕНИЕ ОСНОВНЫХ НЕИЗМЕНЯЕМЫХ ПАРАМЕТРОВ ШАССИ (и изменяемых на шаге 0)
{
    commonPre();
    //проекция на плоскость YZ постоянного угла между OA и строительной горизонталью вертолета
    AOH = asin( A.y / OAyz );
    //проекция на плоскость YZ вертолета угла между OW и строительной горизонталью, в предельных положениях амортстойки
    double angsx_free = -asin(B_.y/OByz) + BOWyz;//то же: double angsx_free1 = acos(0.5*(OAyz*OAyz + OByz*OByz - AByz*AByz)/fabs(OAyz)/fabs(OByz)) - AOH + BOWyz;
    //координаты характерных точек в локальной СК стойки, постоянные в этой СК
    Bloc = SKo2SKsh(B_,angsx_free,yrot);
    Wloc = SKo2SKsh(W_,angsx_free,yrot);
    //нижняя точка "на ободе колеса" (в плоскости симм.колеса) соответствующая максимально допустимому обжатию пневматика, с корректировкой по lambda
    double dDx = WD*sin(lambda);
    double dDy = WD - WD*cos(lambda);
    Dloc.x = Wloc.x - dDx *direct;
    Dloc.y = Wloc.y - WD*cos(ang_wh) + dDy*cos(angsx_free);
    Dloc.z = Wloc.z - WD*sin(ang_wh) - dDy*sin(angsx_free);
    D = D_ = SKsh2SKo(Dloc,angsx_free,yrot);
    D = D_ = corrD(D_);//корректировка по тангажу т.D в разжатом положении амортстойки
    //координаты точек в разжатом положении амортстойки
    Bfree = B_;
    Wfree = W_;
    Dfree = D_;

    //поиск координат т.Bfull,Wfull в положении предельно обжатой амортстойки
    xyz Bfix = B_;//запомним координаты т.В в свободном состоянии //xyz Ofix = O_;  //xyz Afix = A_;
    double ABtest=AB, ABfix=AB, dh=0.001;//
    v_ab.clear();
    v_pk_a.clear();
    v_pk_p.clear();
    while (ABtest > ABfull)
    {
        set_pk(dh);
        ABtest = AB;    AB_ = AB;
        Q_ = Q;  O_ = O;  A_ = A;  B_ = B;  W_ = W;
    }//get_pk(dh);
    pk_initial(ABfull, pk_a, pk_p);//дабавляем предельно обжатое положение
    ang_sx_full = ang_sx;
    ang_sz_full = ang_sz;
    ang_a_full  = ang_a;
    ang_pn_full = ang_pn;
    //координата точки  в предельно обжатом положении амортстойки (нужна для проверки и 3D-визуализации)
    Bfull = B;
    Wfull = W;

    //возврат к разжатому положению амортстойки, расчет конфигурации стойки в разжатом положении //O = O_ = Ofix;  //A = A_ = Afix;
    B = B_ = Bfix;
    AB_= ABfix;
    get_pk(0);
    //проекция усилия на амортизаторе на верт.ось
    Pay = ab->Pa *pkfa/pkfp;
    //для получения координат Cfree_abs
    isContact();
}

void LeverType::Preprocess()// ОПРЕДЕЛЕНИЕ ОСНОВНЫХ НЕИЗМЕНЯЕМЫХ ПАРАМЕТРОВ ШАССИ (и изменяемых на шаге 0)
{
    commonPre();
    //координаты характерных точек в локальной СК
    Dloc.x = Wloc.x = Bloc.x = 0;
    Dloc.y = Wloc.y = Bloc.y = 0;
    Dloc.z = Wloc.z = Bloc.z = 0;
    //проекция на плоскость XY постоянного угла между OA и строительной горизонталью вертолета
    AOH = asin( A.y / OAxy );
    //проекция на плоскость XY вертолета угла между OW и строительной горизонталью, в предельных положениях амортстойки
    double angsz_free = -asin(B.y/OBxy) + BOWxy;    //angsz_free = acos(0.5*(OAxy*OAxy + OBxy*OBxy - ABxy*ABxy)/fabs(OAxy)/fabs(OBxy)) - AOH + BOWxy;
    //координаты нижней точки амортизатора в предельных положениях
    Bfree = B_;
    //центр колеса в предельных положениях
    Wfree = getW(angsz_free);/// нужна ПРОВЕРКА  Mi-8
    //нижняя точка обода колеса
    Dfree.x = Wfree.x;
    Dfree.y = Wfree.y - Rmd;
    Dfree.z = Wfree.z;
    D = D_ = corrD(Dfree);//корректировка т.D по тангажу

    //поиск координат т.Bfull,Wfull в положении предельно обжатой амортстойки
    double ABtest=AB, ABfix=AB,  dOx_full=0,dOy_full=0,  dh=0.001;//0;
    xyz Ofix = O_, Qfix = Q_, Afix = A_, Bfix = B_;//запомним координаты точек в свободном состоянии
    v_ab.clear();
    v_pk_a.clear();
    v_pk_p.clear();
    while (ABtest > ABfull)
    {
        set_pk(dh);
        ABtest = AB;      AB_ = AB;
        Q_ = Q;  O_ = O;  A_ = A;  B_ = B;  W_ = W;
        dOx_full += dOx;  dOy_full += dOy;
    }
    pk_initial(ABfull, pk_a, pk_p);//дабавляем предельно обжатое положение
    ang_sx_full = ang_sx;//0
    ang_sz_full = ang_sz;
    ang_a_full  = ang_a;
    ang_pn_full = ang_pn;
    //координата точки в предельно обжатом положении амортстойки (нужна для проверки и 3D-визуализации)
    Bfull.x = B.x -dOx_full;//т.к.3D-прорисовка будет при состоянии free
    Bfull.y = B.y -dOy_full;//т.к.3D-прорисовка будет при состоянии free
    Bfull.z = B.z;
    Wfull.x = W.x -dOx_full;//т.к.3D-прорисовка будет при состоянии free
    Wfull.y = W.y -dOy_full;//т.к.3D-прорисовка будет при состоянии free
    Wfull.z = W.z;

    //расчет передаточных коэффициентов и конфигурации стойки в разжатом положении амортизатора
    O = O_ = Ofix;
    Q = Q_ = Qfix;
    A = A_ = Afix;
    B = B_ = Bfix;
    dOx = 0;
    dOy = 0;
    AB_= ABfix;
    get_pk(0);
    // проекция усилия на амортизаторе на верт.ось в начале отсчета
    Pay = ab->Pa *pkfa/pkfp;
    //для получения координат Cfree_abs
    isContact();
}



//ДИНАМИКА --------------------------------------------------------------------

void IShassi::ShassiSolver()//поиск сбалансированного состояния при контакте с землей  C-version
{
    status = 0;   contact = 0;   warning = 0;
    //обнуление работ стойки
    ab->dA = 0;   pn->dA = 0;   pn->dAtr = 0;
    //поправки обжатия пневматиков для случая двух колес на стойке
    if (nwh == 2)
    {
        kh2wh.x = 0;
        kh2wh.y = 2*whd*sin(0.5*orient.k);// *cos(orient.t);
        kh2wh.z = 0;//2*whd*sin(0.5*orient.k)*sin(orient.t);
    }
    else
    {
        kh2wh.x = 0;
        kh2wh.y = 0;
        kh2wh.z = 0;
    }

    //РАЗДЕЛЕНИЕ РЕЖИМОВ РАБОТЫ ШАССИ
    if ( isContact() )//КОНТАКТ С ЗЕМЛЕЙ ВОЗМОЖЕН
    {
        //проекция теоретического положения т.контакта на поверхность земли
        snC.x = snCteor.x;
        snC.z = snCteor.z;
        snC.y = ground - orient.y;//реальное Y-положение т.контакта с землей
        //определение delta
        getDelta(snC);
        //баланс шасси
        if ( CG>=0 && ab->status_<0 && pn->status_==0)
            contact = 0;//отрыв
        else
            contact = 1;//контакт
        if ( atype==0 && ab->status_==0 && ab->ha_>0)
            contact = 0;//отрыв
        paBalance();
    }

    else//КОНТАКТ НЕВОЗМОЖЕН
    {
        contact = 0;//отрыв
        //определение delta
        getDelta(snCteor);
        //проекции сумм.обжатия в СКC и в норм.СК
        if (ab->ha_ == 0)
        { delta.x = 0;  delta.y = 0;  delta.z = 0; }
        else          { delta.y = -fabs(delta.y);  }

        //вообщето балансировать нечего, но разжать амортстойки нужно
        paBalance();

        status = 0;
        ab->status=0;     pn->status=0;
        pn->h.x = 0;      pn->h1.x = 0;   pn->h2.x = 0;
        pn->h.y = 0;      pn->h1.y = 0;   pn->h2.y = 0;
        pn->h.z = 0;      pn->h1.z = 0;   pn->h2.z = 0;
        pn->P.x = 0.001;  pn->P1.x = 0;   pn->P2.x = 0;
        pn->P.y = 0.001;  pn->P1.y = 0;   pn->P2.y = 0;
        pn->P.z = 0.001;  pn->P1.z = 0;   pn->P2.z = 0;
        Ppy = 0;          Pay = 0;        ab->Pa = 0;
    }

    //координата т.С
    getC();

    Pp_s = SKN2SKS(pn->P);
    if (nwh==2)
    {
        Pp1s = SKN2SKS(pn->P1);
        Pp2s = SKN2SKS(pn->P2);
    }
}

xyz  IShassi::getDelta(xyz snC)//--------------------------------------------------------------
{
    //в глобальной СК
    saC.x = orient.x + snC.x;
    saC.y = orient.y + snC.y;
    saC.z = orient.z + snC.z;
    //проекции сумм.обжатия в норм.СК
    deltan.x = saC.x - saC_.x;
    deltan.y = saC.y - saC_.y;
    deltan.z = saC.z - saC_.z;
    //проекции сумм.обжатия в СКC
    delta = SKN2SKS( deltan );

return delta;
}

int  IShassi::isContact()//расчет "размерной цепи"
{
    maycont = 0;//по умолчанию: нет контакта
    //проворот т.D по ободу и преобразование из локальной СКшасси в СКС
    xyz Dfree_sks = SKo2SKS( corrD( Dfree ) );
    //координаты т.W колеса в СКН //координаты нижней точки колеса в СКС определены в Pre
    Dfree_skn = SKS2SKN( Dfree_sks );
    //координаты т.C колеса в СКН
    Cfree_skn.x = Dfree_skn.x;
    Cfree_skn.y = Dfree_skn.y - pn->dpmd;
    Cfree_skn.z = Dfree_skn.z;
    //расстояние между нижней точкой колеса и землей по вертикали для крайних положений
    CGfree = orient.y + Cfree_skn.y - ground - fabs(kh2wh.y);///-kh2wh.y
    if (CGfree<0) maycont = 1;//КОНТАКТ ВОЗМОЖЕН: но не обязательно
    //для отрисовки opengl
    Cfree_abs.x = orient.x + Cfree_skn.x;
    Cfree_abs.y = orient.y + Cfree_skn.y;
    Cfree_abs.z = orient.z + Cfree_skn.z;
    //координаты контактной точки пневматика при отсутствии контакта с землей
    snCteor = SKS2SKN( SKo2SKS( C_ ) );
    //расстояние между нижней точкой колеса и землей по вертикали для теоретического положения т.С
    CG = orient.y + snCteor.y - ground - fabs(kh2wh.y);///-kh2wh.y

return maycont;
}

void IShassi::paBalance()// БАЛАНС СИЛ НА ПНЕВМАТИКЕ И НА АМОРТИЗАТОРЕ
{
    int zdf_ = 1; zdf = 1; df = 0;
    int mltplr=1, q=0;
    stopbalance = false;
    //начальные доли смещений для амортизатора и пневматика
    double na = 0.5, np = 0.5;

    //поиск распределения смещений + getForces(np,na)
    if (contact)//есть контакт///////////////////////////////////////////////////////////////////////////////
    {
        if (delta.y==0) { status = 0;  stopbalance = 1; }
        if (delta.y>0)  { status = 1; }
        if (delta.y<0)  { status =-1; }
        //первичный расчет при начальном распределении смещений "100% на пневматик" для определения координаты т.D
        getForces(1,0);
        //фиксация т.D перед изменением угла ang_ax
        snD0 = SKS2SKN(SKo2SKS(D));

        if ( stype==2 && pn->status==0 )
        {
            ab->SolveFree( pn->P.y*pkfp/pkfa );
            Pay = ab->Pa *pkfa/pkfp;//проекция на ось Y связанной СК и перенесенная в пятно контакта
            Ppy = pn->P.y;          //проекция на ось Y связанной СК
            ab->dha = ab->ha - ab->ha_;
            dhay = ab->dha/pk_a;
            get_pk( dhay );
        }
        else
        {
            //пересчет для начального распределения смещений 50%+50%, если нет ограничений - переход к поиску равновесного состояния в цикле
            if (!getForces(np,na) && !stopbalance)
            {
                q = 0;
                zdf_ = zdf;
                while (zdf_==zdf)
                {// перебор до смены знака (Pp-Pa)
                    q++;
                    if (q<100)         mltplr=1;
                    else if (q>=100)   mltplr=10;
                    else if (q>=1000)  mltplr=100;
                    zdf_ = zdf;
                    if (df*status<=0)// Ppy < Pay
                    {
                        np += krp*mltplr;
                        na = 1 - np;
                    }
                    else            // Ppy >= Pay
                    {
                        na += krp*mltplr;
                        np = 1 - na;
                    }
                    if (na<-1000 || na>1000 || (q>1111))////
                    {
                        getForces(np,na);
                        if (q>999) {  warning=200;  break; }
                        break;
                    }
                    else if (getForces(np,na)) break;
                }
            }
            if (ab->Pa < ab->Pmin)
            {
                ab->SolveFree( pn->P.y*pkfp/pkfa );
                Pay = ab->Pa *pkfa/pkfp;///проекция на ось Y связанной СК и перенесенная в пятно контакта
                Ppy = pn->P.y;          ///проекция на ось Y связанной СК
                ab->dha = ab->ha - ab->ha_;
                dhay = ab->dha/pk_a;
                get_pk( dhay );
            }
        }
        delta_correction();

        //реакции в пятне контакта
        if(pn->status_ != pn->status)  pn->dAtr = 0;
        if(stype==0 && pn->dhy!=0 && pn->dhz!=0 && pn->Ptr!=0)
        {
            pn->P.y = (pn->dA + pn->dAtr) / pn->dhy;
        }
    }

    else//отрыв, contact=0 //////////////////////////////////////////////////////////////////////////////////
    {
        status = 0;
        getForces(1,1);
        if ( atype==0 && pn->P.y==0)
        {
            ab->status = 0;
            ab->Pa = 0;
            ab->ha = ab->ha_;
        }
        else
            ab->SolveFree( pn->P.y*pkfp/pkfa );

        if (stype==2 && (pn->h1_.y > 0 || pn->h2_.y > 0))
        {// не уверен что это вообще нужно
            status = -1;
            pn->Solve(nwh);
            Pay = ab->Pa *pkfa/pkfp;//проекция на ось Y связанной СК и перенесенная в пятно контакта:
            Ppy = pn->P.y;          //проекция на ось Y связанной СК
        }
        else
        {
            pn->h.x = 0;    pn->h1.x = 0;    pn->h2.x = 0;
            pn->h.y = 0;    pn->h1.y = 0;    pn->h2.y = 0;
            pn->h.z = 0;    pn->h1.z = 0;    pn->h2.z = 0;
            pn->Solve(nwh);
            ///Pay = 0;//ab->Pa *pkfa/pkfp;//проекция на ось Y связанной СК и перенесенная в пятно контакта:
            ///Ppy = 0;//pn->P.y;          //проекция на ось Y связанной СК         <modify 2020.02
            Pay = pn->P.y;          //проекция на ось Y связанной СК
            ab->Pa = Pay *pkfp/pkfa;//проекция на ось Y связанной СК и перенесенная в пятно контакта:
        }
        ab->dha = ab->ha - ab->ha_;
        dhay = ab->dha/pk_a;
        get_pk( dhay );
        ///обнуление счетчика для полозкового шасси////////////////
        if(atype==0 && ab->ha > ab->hmax)  ab->q = 0;
    }

    //сглаживание на переходных режимах
    if ((ab->ha_ + ab->dha > ab->S) || (ab->ha_ + ab->dha < 0) || (fabs(ab->va) < 0.01) || (pn->status == 2))
    {
        Pay = pn->P.y;          //проекция на ось Y связанной СК
        ab->Pa = Pay *pkfp/pkfa;//проекция на ось Y связанной СК и перенесенная в пятно контакта:
        ab->Pa = (ab->Pa<0)?(0):(ab->Pa);
        ab->ha = (ab->ha<0)?(0):(ab->ha);
    }
    if(Pay==0 && pn->P.y>0)//// <modify 2020.02
    {
        Pay = pn->P.y;          //проекция на ось Y связанной СК
        ab->Pa = Pay *pkfp/pkfa;//проекция на ось Y связанной СК и перенесенная в пятно контакта:
    }
}

void IShassi::delta_correction()
{
    xyz snD;
    //определение т.D при изменении угла ang_ax
    snD = SKS2SKN(SKo2SKS(D));
    //фиксация разницы в положении т.D перед изменением угла ang_ax и после
    dD.x = snD.x - snD0.x;
    dD.y = snD.y - snD0.y;
    dD.z = snD.z - snD0.z;
    //коррекция именно deltan разницей от изменения геометрии стойки
    deltan.x += dD.x;
    deltan.y += dD.y;
    deltan.z += dD.z;
    //тангенциальное и аксиальное смещения пневматика за итерацию
    pn->dhx = deltan.x;
    ////pn->dhy = deltan.y;
    pn->dhz = deltan.z;
    //величина обжатия пневматика
    pn->get_pncompress(nwh,kh2wh);
    pn->Solve(nwh);
}

bool IShassi::getForces(double np, double na)
{
    bool stop = 0;
    //раздача проекций обжатий на вертикаль (у пятна контакта)
    dhpy = np*delta.y;
    dhay = na*delta.y/pk_p;
    //пересчет геометрии стойки шасси
    get_pk( dhay );
    //обжатие амортизатора и проекции обжатия пневматика
    ab->dha = dhay*pk_a;
    pn->dhy = dhpy;
    pn->dhx = delta.x;
    pn->dhz = delta.z;
    //величина обжатия пневматика
    pn->get_pncompress(nwh,kh2wh);
    //проверка по предельным обжатиям и расчет реакций при обычном режиме
    if (CheckLimit(delta.y))
    {
        Pay = ab->Pa *pkfa/pkfp;///проекция на ось Y связанной СК и перенесенная в пятно контакта
        Ppy = pn->P.y;          ///проекция на ось Y связанной СК
        df = Ppy-Pay;
        stop = true;
    }
    else
    {
        pn->Solve(nwh);
        ab->Solve();
        Pay = ab->Pa *pkfa/pkfp;//проекция на ось Y связанной СК и перенесенная в пятно контакта
        Ppy = pn->P.y;          //проекция на ось Y связанной СК
        df  = Ppy-Pay;
        if(df>0) zdf = 1;
        else     zdf =-1;
    }
return stop;
}

bool IShassi::CheckLimit(double dy)// проверка по предельным обжатиям
{
    bool limit = false;
    double dhpy, dhay;
    // проверка по предельным обжатиям амортизатора
    if (ab->ha_+ab->dha > ab->S)
    {
        limit = true;
        dhay = (ab->S - ab->ha_)/pk_a;
        dhpy = dy - dhay*pk_p;
        get_pk(dhay);
        ab->dha = dhay *pk_a/pk_p;
        pn->dhy = dhpy;
        pn->dhx = delta.x;
        pn->dhz = delta.z;
        pn->get_pncompress(nwh,kh2wh);//обжатия пневматика
        pn->Solve(nwh);
        ab->Solve();
    }
    // проверка по предельным обжатиям амортизатора
    if (ab->ha_+ab->dha < 0)
    {
        limit = true;
        dhay = (0 - ab->ha_) /(pk_a);
        dhpy = dy - dhay*pk_p;
        get_pk(dhay);
        ab->dha = dhay *pk_a;
        pn->dhy = dhpy;
        pn->dhx = delta.x;
        pn->dhz = delta.z;
        pn->get_pncompress(nwh,kh2wh);//обжатия пневматика
        pn->Solve(nwh);
        ab->Solve();
    }
    //  проверка по предельным обжатиям пневматика
    if ((nwh==1) && (pn->h.y < 0))
    {
        limit = true;
        dhpy = 0 - pn->h_.y;
        dhay = (dy - dhpy)/pk_p;
        get_pk(dhay);//pk_p
        ab->dha = dhay *pk_a;//pk_p
        pn->dhy = dhpy;
        pn->dhx = 0;
        pn->dhz = 0;
        pn->get_pncompress(nwh,kh2wh);//обжатия пневматика
        pn->Solve(nwh);
        ab->SolveFree(pn->P.y*pkfp/pkfa);///(0) <modify 2020.02
    }
    if ((nwh==2) && (pn->h1.y < 0) && (pn->h2.y < 0))
    {
        limit = true;
        dhpy = 0 - pn->h_.y - fabs(kh2wh.y);
        dhay = (dy - dhpy)/pk_p;
        get_pk(dhay);//pk_p
        ab->dha = dhay *pk_a;//pk_p
        pn->dhy = dhpy;//0;//dhpy;
        pn->dhx = 0;
        pn->dhz = 0;
        pn->get_pncompress(nwh,kh2wh);
        pn->Solve(nwh);
        ab->SolveFree(pn->P.y*pkfp/pkfa);///(0) <modify 2020.02
    }
return limit;
}



//КИНЕМАТИКА ------------------------------------------------------------------

void PyramidType::set_pk(double dhay)// ОПРЕДЕЛЕНИЕ РЕАЛЬНОГО ОБЖАТИЯ (ОБЩЕГО) АМОРТИЗАТОРА ПО ЗАДАННОЙ ПРОЕКЦИИ ОБЖАТИЯ НА ВЕРТИКАЛЬНУЮ ОСЬ (на направление силы действ. на пневматик)
{   //dhay - проекция dha на вертикаль СКС
    //верхняя точка амортизатора
    A.x = A_.x;
    A.y = A_.y;
    A.z = A_.z;
    //нижняя точка амортизатора
    B.y = B_.y + dhay;
    B.x = B_.x - dhay*tan(lambda);
    B.z = sqrt(OByz*OByz-B.y*B.y) *direct;//так вроде должно быть;    //    B.z = sqrt(OB*OB-B.y*B.y) *direct;
    //пересчет прооекций для корректных ang_sx и ang_sz
    OByz = sqrt( B.y*B.y + B.z*B.z );//обычно константа
    OBxy = sqrt( B.y*B.y + B.x*B.x );
    //проекции угла между OW и строительной горизонталью вертолета
    ang_sx = -asin(B.y/OByz) + BOWyz;//проекция на плоскость YZ вертолета угла между OW и строительной горизонталью
    ang_sz = -asin(B.y/OBxy) + BOWxy;//проекция на плоскость XY вертолета угла между OW и строительной горизонталью
    //точка крепления шатуна или амортстойки к рычагу
    //V.x = B.x;
    //V.y = B.y;
    //V.z = B.z;
    //центр колеса
    W = SKsh2SKo(Wloc,ang_sx ,yrot);
    //нижняя точка обода колеса + корректировка по тангажу
    //D = corrD( SKsh2SKo(Dloc,ang_sx,yrot) );
    //центры колес & нижние точки обода для пары колес
    //W1.x = W.x;    W1.y = W.y;    W1.z = W.z;
    //D1.x = D.x;    D1.y = D.y;    D1.z = D.z;
    //W2.x = 0;      W2.y = 0;      W2.z = 0;
    //D2.x = 0;      D2.y = 0;      D2.z = 0;
    //длина амортизатора условная (между A и B)
    AB  = sqrt((A.x-B.x)*(A.x-B.x) + (A.y-B.y)*(A.y-B.y) + (A.z-B.z)*(A.z-B.z));
    //AByz= sqrt((A.y-B.y)*(A.y-B.y) + (A.z-B.z)*(A.z-B.z));
    //ABxy= sqrt((A.y-B.y)*(A.y-B.y) + (A.x-B.x)*(A.x-B.x));
    //угол между осью амортизатора и вертикалью (осью Y)
    ang_a = acos((A.y-B.y)/AB)  *(-direct);
    //ang_ax= acos((A.y-B.y)/AByz)*(-direct);
    //ang_az= acos((A.y-B.y)/ABxy)*(-direct);
    //ang_a_yz=acos(AByz/AB) *(-direct);//нужен если pkfa рассчитывается по "cos(ang_ax)*cos(ang_a_yz)"
    //угол между плоскостью колеса и плоскостью симметрии вертолета
    ang_pn = (ang_sx + ang_wh)*direct;
    //~передаточный коэффициент для усилий на штоке амортизатора
    //pkfa = cos(ang_a);//громоздкий вариант: cos(ang_ax)*cos(ang_a_yz); //fail: cos(ang_ax)*cos(ang_az);//<1
    //~передаточный коэффициент для смещений амортизатора
    if((dhay==0)||(AB_==AB)) pk_a = cos(ang_a);//pkfa;
    else   pk_a = (AB_-AB)/dhay;//<1
    //~передаточный коэффициент для пневматика
    if(B.z==0) pk_p = 1;// = pkfp
    else
    {
        //передаточный коэффициент для усилий на пневматике
        //pkfp = fabs( D.z / B.z );//>1
        //передаточный коэффициент для смещений пневматика //ВНИМАНИЕ! в старой методике pk_p = pkfp = 1;
        pk_p = fabs( D.z / B.z );//pkfp;//sqrt(D.z*D.z+D.x*D.x) / sqrt(B.z*B.z+B.x*B.x);//>1;
    }
    //передаточный коэффициент по перемещению
    psih = pk_p / pk_a;
    //передаточный коэффициент по усилию
    //psif = pkfp / pkfa;
    //добавляем текущие значения в массив
    pk_initial(AB_, pk_a, pk_p);
}
void PyramidType::get_pk(double dhay)// ОПРЕДЕЛЕНИЕ РЕАЛЬНОГО ОБЖАТИЯ (ОБЩЕГО) АМОРТИЗАТОРА ПО ЗАДАННОЙ ПРОЕКЦИИ ОБЖАТИЯ НА ВЕРТИКАЛЬНУЮ ОСЬ (на направление силы действ. на пневматик)
{   //dhay - проекция dha на вертикаль СКС
    //верхняя точка амортизатора
    A.x = A_.x;
    A.y = A_.y;
    A.z = A_.z;
    //нижняя точка амортизатора
    B.y = B_.y + dhay;
    B.x = B_.x - dhay*tan(lambda);
    B.z = sqrt(OByz*OByz-B.y*B.y) *direct;//так вроде должно быть;    //    B.z = sqrt(OB*OB-B.y*B.y) *direct;
    //пересчет прооекций для корректных ang_sx и ang_sz
    OByz = sqrt( B.y*B.y + B.z*B.z );//обычно константа
    OBxy = sqrt( B.y*B.y + B.x*B.x );
    //проекции угла между OW и строительной горизонталью вертолета
    ang_sx = -asin(B.y/OByz) + BOWyz;//проекция на плоскость YZ вертолета угла между OW и строительной горизонталью
    ang_sz = -asin(B.y/OBxy) + BOWxy;//проекция на плоскость XY вертолета угла между OW и строительной горизонталью

    //точка крепления шатуна или амортстойки к рычагу
    V.x = B.x;
    V.y = B.y;
    V.z = B.z;
    //центр колеса
    W = SKsh2SKo(Wloc,ang_sx ,yrot);
    //нижняя точка обода колеса + корректировка по тангажу
    D = corrD( SKsh2SKo(Dloc,ang_sx,yrot) );
    //центры колес & нижние точки обода для пары колес
    W1.x = W.x;    W1.y = W.y;    W1.z = W.z;
    D1.x = D.x;    D1.y = D.y;    D1.z = D.z;
    W2.x = 0;      W2.y = 0;      W2.z = 0;
    D2.x = 0;      D2.y = 0;      D2.z = 0;
    //длина амортизатора условная (между A и B)
    AB  = sqrt((A.x-B.x)*(A.x-B.x) + (A.y-B.y)*(A.y-B.y) + (A.z-B.z)*(A.z-B.z));
    AByz= sqrt((A.y-B.y)*(A.y-B.y) + (A.z-B.z)*(A.z-B.z));
    ABxy= sqrt((A.y-B.y)*(A.y-B.y) + (A.x-B.x)*(A.x-B.x));
    //угол между осью амортизатора и вертикалью (осью Y)
    ang_a = acos((A.y-B.y)/AB)  *(-direct);
    ang_ax= acos((A.y-B.y)/AByz)*(-direct);
    ang_az= acos((A.y-B.y)/ABxy)*(-direct);
    //ang_a_yz=acos(AByz/AB) *(-direct);//нужен если pkfa рассчитывается по "cos(ang_ax)*cos(ang_a_yz)"
    //угол между плоскостью колеса и плоскостью симметрии вертолета
    ang_pn = (ang_sx + ang_wh)*direct;

    //~передаточный коэффициент для усилий на штоке амортизатора
    pkfa = cos(ang_a);//громоздкий вариант: cos(ang_ax)*cos(ang_a_yz); //fail: cos(ang_ax)*cos(ang_az);//<1
    //~передаточный коэффициент для смещений амортизатора
    //if((dhay==0)||(AB_==AB)) pk_a = pkfa;
    //else pk_a = (AB_-AB)/dhay;//<1
    //~передаточный коэффициент для пневматика
    if(B.z==0) pkfp = 1;//pk_p =
    else
    {
        //передаточный коэффициент для усилий на пневматике
        pkfp = fabs( D.z / B.z );//sqrt(D.z*D.z+D.x*D.x) / sqrt(B.z*B.z+B.x*B.x);///неточность: //>1;
        //передаточный коэффициент для смещений пневматика //ВНИМАНИЕ! в старой методике pk_p = pkfp = 1;
        //pk_p = pkfp;///fabs( D.z / B.z );//>1
    }
    //коэффициент по перемещению  pk_p & pk_a;
    pk_return(AB);
    //передаточный коэффициент по перемещению
    psih = pk_p / pk_a;
    //передаточный коэффициент по усилию
    psif = pkfp / pkfa;

    //test///
    //BWtest = sqrt((B.x-W.x)*(B.x-W.x) + (B.y-W.y)*(B.y-W.y) + (B.z-W.z)*(B.z-W.z));
}

void LeverType::set_pk(double dhay)//заполнение массива с передаточными числами
{
    //нижняя точка амортизатора //double beta= -asin(B.y/OBxy);
    B.y = B_.y + dhay;
    B.x = -OBxy*cos(asin(B.y/OBxy));//sqrt(OBxy*OBxy-B.y*B.y);
    B.z = B_.z;
    //dOx,dOy изменение координаты т.O  (не передавать для следующей итерации!)
    double AB_AB=1;
    switch (stype)
    {//0 - полозковое; 1 - пирамидальная; 2 - рычажная Ми2; 3 - рычажная; 8 - рычажная Ми8);
       case 0: case 1: case 3:
            dOx = 0;
            dOy = 0;
       break;
       case 2://Ми-2
            dOx = B.x - B_.x;
            Q.x = Q_.x + dOx;
            Q.y = sqrt( OQ*OQ - Q.x*Q.x );
            dOy = Q.y - Q_.y;
       break;
       case 8://Ми-8 только для вертикального амортизатора!
            dOx = 0;
            dOy = 0;
            AB_AB = (B.y-A_.y)/(B_.y-A_.y);
            B.x = (AB_AB)?((B_.x-A_.x)*AB_AB +A_.x):(B_.x-A_.x);//B.x = B_.x;//на самом деле перемещается по оси амортизатора, а она не всегда вертикальная
            OBxy = sqrt( B.y*B.y + B.x*B.x );
            BOVxy = acos( 0.5*(OVxy*OVxy + OBxy*OBxy - BVxy*BVxy)/OVxy/OBxy ); //BOx = asin(B.y/OBxy);//VOx = BOVxy - BOx;
            BOWxy = BOVxy + VOWxy;
       break;
       default:
            dOx = 0;
            dOy = 0;
    }
    //начало локальной СК амортстойки
    O.x = O_.x - dOx;    O.y = O_.y - dOy;    O.z = O_.z;
    //координаты т.A
    A.x = A_.x + dOx;    A.y = A_.y + dOy;    A.z = A_.z;
    //корректировка координаты т.B
    B.y += dOy;
    //проекции угла между OW и строительной горизонталью вертолета
    ang_sx = 0;//-asin(B.y/OByz) + BOWyz;
    ang_sz = -asin(B.y/OBxy) + BOWxy;//ang_sz = beta + BOW;//проекция на плоскость XY вертолета угла между OW и строительной горизонталью
    //точка крепления шатуна или амортстойки к рычагу
    if(stype==8)
    {//точка крепления шатуна к рычагу
        V.x = -OVxy *cos(ang_sz - VOWxy);
        V.y = -OVxy *sin(ang_sz - VOWxy);
        V.z = B.z;
    }
    else
    {//точка крепления амортстойки к рычагу
        V.x = B.x;
        V.y = B.y;
        V.z = B.z;
    }
    //центр & нижняя точка обода колеса/колес
    W = getW(ang_sz);
    //D.x = W.x;
    //D.y = W.y - Rmd;
    //D.z = W.z;//временно для одного колеса
    //D = corrD(D);//учтен тангаж
    //W1.x = W.x;     W1.y = W.y;     W1.z = W.z + whd;
    //W2.x = W.x;     W2.y = W.y;     W2.z = W.z - whd;
    //D1.x = D.x;     D1.y = D.y;     D1.z = D.z + whd;
    //D2.x = D.x;     D2.y = D.y;     D2.z = D.z - whd;
    //длина амортизатора условная (между A и B)
    AB  = sqrt((A.x-B.x)*(A.x-B.x) + (A.y-B.y)*(A.y-B.y) + (A.z-B.z)*(A.z-B.z));
    //AByz= sqrt((A.y-B.y)*(A.y-B.y) + (A.z-B.z)*(A.z-B.z));
    //ABxy= sqrt((A.y-B.y)*(A.y-B.y) + (A.x-B.x)*(A.x-B.x));
    //угол между осью амортизатора и вертикалью (осью Y)
    ang_a  = acos((A.y-B.y)/AB);// *(-direct);
    //ang_a - угол между осью амортизатора и плоскостью симметрии вертолета (осью Y)
    //ang_ax = acos((A.y-B.y)/AByz);// *(direct);///*(-direct);//////////////////////
    //ang_az = acos((A.y-B.y)/ABxy);// *(direct);///*(-direct);/////////////////////
    //угол между плоскостью колеса и плоскостью симметрии вертолета
    ang_pn = ang_wh *direct;//(ang_sx + ang_wh)*direct;//-(0.5*M_PI - ang_sx - (OWz-ang_wh)) *direct;
    //~передаточный коэффициент от вертикали в нижней точке амортстойки к оси амортстойки
    //pkfa = cos(ang_a);//передаточный коэффициент для усилий//cos(ang_ax)*cos(ang_az);//~1;
    //передаточный коэффициент от вертикали в нижней точке амортстойки к оси амортстойки для смещений
    if((dhay==0)||(AB_==AB)) pk_a = cos(ang_a);//pkfa;
    else pk_a = (AB_-AB)/dhay;//<1  //pk_a = (AB_-AB)/(dhay  );///
    //~передаточный коэффициент от пневматика к вертикали в нижней точке амортстойки
    if(B.x==0) pk_p = 1;
    else
    {
        //передаточный коэффициент от пневматика к вертикали в нижней точке амортстойки для усилий
        //pkfp = fabs( W.x / B.x );///fabs( D.x / B.x );//>1;
        //передаточный коэффициентот от пневматика к вертикали в нижней точке амортстойки для смещений
        if(dhay) pk_p = (W.y-W_.y -dOy) / (B.y-B_.y -dOy);// -dOy так как координаты длжны быть в СКС, а не локальной СК (это для схемы Ми-2)
        else     pk_p = fabs( W.x / B.x );//pkfp;
    }
    //передаточный коэффициент по перемещению
    psih = pk_p / pk_a;
    //передаточный коэффициент по усилию
    //psif = pkfp / pkfa;
    //добавляем текущие значения в массив
    pk_initial(AB_, pk_a, pk_p);
}
void LeverType::get_pk(double dhay)// ОПРЕДЕЛЕНИЕ РЕАЛЬНОГО ОБЖАТИЯ (ОБЩЕГО) АМОРТИЗАТОРА ПО ЗАДАННОЙ ПРОЕКЦИИ ОБЖАТИЯ НА ВЕРТИКАЛЬНУЮ ОСЬ (на направление силы действ. на пневматик)
{
    //нижняя точка амортизатора //double beta= -asin(B.y/OBxy);
    B.y = B_.y + dhay;
    B.x = -OBxy*cos(asin(B.y/OBxy));//sqrt(OBxy*OBxy-B.y*B.y);
    B.z = B_.z;
    double AB_AB=1;
    //dOx,dOy изменение координаты т.O  (не передавать для следующей итерации!)
    switch (stype)
    {//0 - полозковое; 1 - пирамидальная; 2 - рычажная Ми2; 3 - рычажная; 8 - рычажная Ми8);
       case 0: case 1: case 3:
            dOx = 0;
            dOy = 0;
       break;
       case 2://Ми-2
            dOx = B.x - B_.x;
            Q.x = Q_.x + dOx;
            Q.y = sqrt( OQ*OQ - Q.x*Q.x );
            dOy = Q.y - Q_.y;
       break;
       case 8://Ми-8 только для вертикального амортизатора!
            dOx = 0;
            dOy = 0;
            AB_AB = (B.y-A_.y)/(B_.y-A_.y);
            B.x = (AB_AB)?((B_.x-A_.x)*AB_AB +A_.x):(B_.x-A_.x);//B.x = B_.x;//на самом деле перемещается по оси амортизатора, а она не всегда вертикальная
            OBxy = sqrt( B.y*B.y + B.x*B.x );
            BOVxy = acos( 0.5*(OVxy*OVxy + OBxy*OBxy - BVxy*BVxy)/OVxy/OBxy ); //BOx = asin(B.y/OBxy);//VOx = BOVxy - BOx;
            BOWxy = BOVxy + VOWxy;
       break;
       default:
            dOx = 0;
            dOy = 0;
    }
    //начало локальной СК амортстойки
    O.x = O_.x - dOx;    O.y = O_.y - dOy;    O.z = O_.z;
    //координаты т.A
    A.x = A_.x + dOx;    A.y = A_.y + dOy;    A.z = A_.z;
    //корректировка координаты т.B
    B.y += dOy;
    //проекции угла между OW и строительной горизонталью вертолета
    ang_sx = 0;//-asin(B.y/OByz) + BOWyz;
    ang_sz = -asin(B.y/OBxy) + BOWxy;//ang_sz = beta + BOW;//проекция на плоскость XY вертолета угла между OW и строительной горизонталью
    //точка крепления шатуна или амортстойки к рычагу
    if(stype==8)
    {//точка крепления шатуна к рычагу
        V.x = -OVxy *cos(ang_sz - VOWxy);
        V.y = -OVxy *sin(ang_sz - VOWxy);
        V.z = B.z;
    }
    else
    {//точка крепления амортстойки к рычагу
        V.x = B.x;
        V.y = B.y;
        V.z = B.z;
    }
    //центр & нижняя точка обода колеса/колес
    W = getW(ang_sz);
    D.x = W.x;
    D.y = W.y - Rmd;
    D.z = W.z;//временно для одного колеса
    D = corrD(D);//учтен тангаж
    W1.x = W.x;     W1.y = W.y;     W1.z = W.z + whd;
    W2.x = W.x;     W2.y = W.y;     W2.z = W.z - whd;
    D1.x = D.x;     D1.y = D.y;     D1.z = D.z + whd;
    D2.x = D.x;     D2.y = D.y;     D2.z = D.z - whd;
    //длина амортизатора условная (между A и B)
    AB  = sqrt((A.x-B.x)*(A.x-B.x) + (A.y-B.y)*(A.y-B.y) + (A.z-B.z)*(A.z-B.z));
    AByz= sqrt((A.y-B.y)*(A.y-B.y) + (A.z-B.z)*(A.z-B.z));
    ABxy= sqrt((A.y-B.y)*(A.y-B.y) + (A.x-B.x)*(A.x-B.x));
    //угол между осью амортизатора и вертикалью (осью Y)
    if(fabs(A.y-B.y)/AB>0.99999) ang_a = 0;  else ang_a = acos((A.y-B.y)/AB);// *(-direct);
    //ang_a - угол между осью амортизатора и плоскостью симметрии вертолета (осью Y)
    if(fabs(A.y-B.y)/AByz>0.99999) ang_ax = 0;  else ang_ax = acos((A.y-B.y)/AByz);// *(direct);///*(-direct);//////////////////////
    if(fabs(A.y-B.y)/ABxy>0.99999) ang_az = 0;  else ang_az = acos((A.y-B.y)/ABxy);// *(direct);///*(-direct);/////////////////////
    //угол между плоскостью колеса и плоскостью симметрии вертолета
    ang_pn = ang_wh *direct;//(ang_sx + ang_wh)*direct;//-(0.5*M_PI - ang_sx - (OWz-ang_wh)) *direct;

    //~передаточный коэффициент от вертикали в нижней точке амортстойки к оси амортстойки
    pkfa = cos(ang_a);//передаточный коэффициент для усилий//cos(ang_ax)*cos(ang_az);//~1;
    //передаточный коэффициент от вертикали в нижней точке амортстойки к оси амортстойки для смещений
    //if((dhay==0)||(AB_==AB)) pk_a = pkfa;
    //else //pk_a = (AB_-AB   -dOy)/(dhay  );///
    //pk_a = (AB_-AB)/(dhay);//<1
    //~передаточный коэффициент от пневматика к вертикали в нижней точке амортстойки
    if(B.x==0) pkfp = 1;//pk_p =
    else
    {
        //передаточный коэффициент от пневматика к вертикали в нижней точке амортстойки для усилий
        pkfp = fabs( W.x / B.x );//fabs( D.x / B.x );//>1;
        //передаточный коэффициентот от пневматика к вертикали в нижней точке амортстойки для смещений
        //if(dhay) pk_p = (W.y-W_.y  -dOy)  / (B.y-B_.y  -dOy);/// /(AB_-AB);//// -dOy так как координаты длжны быть в СКС, а не локальной СК (это для схемы Ми-2)
        //else  pk_p = pkfp;
    }
    //коэффициент по перемещению  pk_p & pk_a;
    pk_return(AB);
    //передаточный коэффициент по перемещению
    psih = pk_p / pk_a;
    //передаточный коэффициент по усилию
    psif = pkfp / pkfa;
}
xyz LeverType::getW(double ang_s_z)
{
    xyz _W_;
    //центр колеса
    _W_.x = -OWxy*cos(ang_s_z);
    _W_.y = -OWxy*sin(ang_s_z);
    _W_.z = W.z;

return _W_;
}



void IShassi::pk_initial(double AB, double pk_a, double pk_p)
{
    v_ab.push_back(AB);
    v_pk_a.push_back(pk_a);
    v_pk_p.push_back(pk_p);
}
void IShassi::pk_return(double AB)
{//pk_a = (AB_-AB)/(dhay);  //pk_p = (W.y-W_.y  -dOy)  / (B.y-B_.y  -dOy);
    pk_a = 1;
    pk_p = 1;
    double x1,y1, x2,y2, a,b;

    if(AB>=v_ab[0])
    {
        pk_a = v_pk_a[0];
        pk_p = v_pk_p[0];
    }
    else if(AB<=v_ab[v_ab.size()-1])
    {
        pk_a = v_pk_a[v_ab.size()-1];
        pk_p = v_pk_p[v_ab.size()-1];
    }
    else
    for(int i=1; i<v_ab.size(); i++)
    {
        x1 = v_ab[i-1];
        x2 = v_ab[i];
        if (AB<=x1 && AB>x2)
        {
            //pk_a
            y1 = v_pk_a[i-1];
            y2 = v_pk_a[i];
            a = (y2-y1)/(x2-x1);
            b = y1-a*x1;
            pk_a = a*AB+b;
            //pk_p
            y1 = v_pk_p[i-1];
            y2 = v_pk_p[i];
            a = (y2-y1)/(x2-x1);
            b = y1-a*x1;
            pk_p = a*AB+b;
            break;
        }
    }
}

xyz IShassi::corrD(xyz D_)//корректировка т.D по тангажу
{
    xyz D;
    //проворот в плоскости колеса
    double dDx = WD*sin(orient.t);
    double dDy = WD - WD*cos(orient.t);
    //коррекция от угла ang_p
    D.x = D_.x - dDx;
    D.y = D_.y + dDy*cos(ang_pn);
    D.z = D_.z - dDy*sin(ang_pn)*direct;

return D;
}

xyz IShassi::getC()
{
    xyz snD = SKS2SKN(SKo2SKS(D));
    //т.C в нормальной СК
    snC.x = snD.x            + pn->h.x;
    snC.y = snD.y - pn->dpmd + pn->h.y;
    snC.z = snD.z            + pn->h.z;
    //т.C в глобальной СК
    saC.x = orient.x + snC.x;
    saC.y = orient.y + snC.y;
    saC.z = orient.z + snC.z;
    //т.C в связанной СК
    C_sks = SKN2SKS(snC);
    C = SKS2SKo( C_sks );

    if (nwh==2)
    {
        xyz snD1,snD2,snC1,snC2;

        snD1 = SKS2SKN(SKo2SKS(D1));
        snD2 = SKS2SKN(SKo2SKS(D2));
        //т.C в нормальной СК
        snC1.x = snD1.x            + pn->h1.x;
        snC1.y = snD1.y - pn->dpmd + pn->h1.y;
        snC1.z = snD1.z            + pn->h1.z;
        snC2.x = snD2.x            + pn->h2.x;
        snC2.y = snD2.y - pn->dpmd + pn->h2.y;
        snC2.z = snD2.z            + pn->h2.z;
        //т.C в связанной СК
        C1sks = SKN2SKS(snC1);
        C1   = SKS2SKo(C1sks);
        C2sks = SKN2SKS(snC2);
        C2   = SKS2SKo(C2sks);
    }

return C;
}

xyz IShassi::SKsh2SKo(xyz A, double ang_s_x, double localerotate)//перевод из локальной СК в недосвязанную
{
    xyz Askn;
    double t = lambda *direct;//-
    double k = ang_s_x;// /cos(lambda)
    double r = localerotate;

    double a11 = cos(r)*cos(t);
    double a21 = sin(k)*sin(r) - cos(k)*cos(r)*sin(t);
    double a31 = cos(k)*sin(r) + sin(k)*cos(r)*sin(t);
    double a12 = sin(t);
    double a22 = cos(t)*cos(k);
    double a32 =-cos(t)*sin(k);
    double a13 =-sin(r)*cos(t);
    double a23 = sin(k)*cos(r) + cos(k)*sin(r)*sin(t);
    double a33 = cos(k)*cos(r) - sin(k)*sin(r)*sin(t);

    Askn.x = a11*A.x + a21*A.y + a31*A.z;
    Askn.y = a12*A.x + a22*A.y + a32*A.z;
    Askn.z = a13*A.x + a23*A.y + a33*A.z;

return Askn;
}
xyz IShassi::SKo2SKsh(xyz A, double ang_s_x, double localerotate)//перевод из недосвязанной СК в локальную
{
    xyz Asks;
    double t = lambda *direct;//-
    double k = ang_s_x;// /cos(lambda)
    double r = localerotate;

    double a11 = cos(r)*cos(t);
    double a21 = sin(k)*sin(r) - cos(k)*cos(r)*sin(t);
    double a31 = cos(k)*sin(r) + sin(k)*cos(r)*sin(t);
    double a12 = sin(t);
    double a22 = cos(t)*cos(k);
    double a32 =-cos(t)*sin(k);
    double a13 =-sin(r)*cos(t);
    double a23 = sin(k)*cos(r) + cos(k)*sin(r)*sin(t);
    double a33 = cos(k)*cos(r) - sin(k)*sin(r)*sin(t);

    Asks.x = a11*A.x + a12*A.y + a13*A.z;
    Asks.y = a21*A.x + a22*A.y + a23*A.z;
    Asks.z = a31*A.x + a32*A.y + a33*A.z;

return Asks;
}

xyz IShassi::SKo2SKS(xyz A)//перевод из недосвязанной СКшасси в связанную
{
    xyz Asks;
    Asks.x = O.x + A.x;
    Asks.y = O.y + A.y;
    Asks.z = O.z + A.z;

return Asks;
}
xyz IShassi::SKS2SKo(xyz Asks)//перевод из связанной СКшасси в недосвязанную
{
    xyz Aps;
    Aps.x = Asks.x - O.x;
    Aps.y = Asks.y - O.y;
    Aps.z = Asks.z - O.z;

return Aps;
}

xyz IShassi::SKS2SKN(xyz A)//перевод из связанной СК в нормальную
{
    xyz Askn;

    double a11 = cos(orient.r)*cos(orient.t);
    double a21 = sin(orient.k)*sin(orient.r) - cos(orient.k)*cos(orient.r)*sin(orient.t);
    double a31 = cos(orient.k)*sin(orient.r) + sin(orient.k)*cos(orient.r)*sin(orient.t);
    double a12 = sin(orient.t);
    double a22 = cos(orient.t)*cos(orient.k);
    double a32 =-cos(orient.t)*sin(orient.k);
    double a13 =-sin(orient.r)*cos(orient.t);
    double a23 = sin(orient.k)*cos(orient.r) + cos(orient.k)*sin(orient.r)*sin(orient.t);
    double a33 = cos(orient.k)*cos(orient.r) - sin(orient.k)*sin(orient.r)*sin(orient.t);

    Askn.x = a11*A.x + a21*A.y + a31*A.z;
    Askn.y = a12*A.x + a22*A.y + a32*A.z;
    Askn.z = a13*A.x + a23*A.y + a33*A.z;

return Askn;
}
xyz IShassi::SKN2SKS(xyz A)//перевод из нормальной СК в связанную
{
    xyz Asks;

    double a11 = cos(orient.r)*cos(orient.t);
    double a21 = sin(orient.k)*sin(orient.r) - cos(orient.k)*cos(orient.r)*sin(orient.t);
    double a31 = cos(orient.k)*sin(orient.r) + sin(orient.k)*cos(orient.r)*sin(orient.t);
    double a12 = sin(orient.t);
    double a22 = cos(orient.t)*cos(orient.k);
    double a32 =-cos(orient.t)*sin(orient.k);
    double a13 =-sin(orient.r)*cos(orient.t);
    double a23 = sin(orient.k)*cos(orient.r) + cos(orient.k)*sin(orient.r)*sin(orient.t);
    double a33 = cos(orient.k)*cos(orient.r) - sin(orient.k)*sin(orient.r)*sin(orient.t);

    Asks.x = a11*A.x + a12*A.y + a13*A.z;
    Asks.y = a21*A.x + a22*A.y + a23*A.z;
    Asks.z = a31*A.x + a32*A.y + a33*A.z;

return Asks;
}



//old&trash ------------------------------------------------------------------
SkidType::SkidType(int pneumID, int absorbID)
{
    //type = typeID;
    pn = new Pneumatic();
    ab = new AbsorberEq();
}
SkidType::~SkidType()
{
    delete ab;
    delete pn;
}
void SkidType::Preprocess()// ОПРЕДЕЛЕНИЕ ОСНОВНЫХ НЕИЗМЕНЯЕМЫХ ПАРАМЕТРОВ ШАССИ (и изменяемых на шаге 0)
{
    commonPre();
    //проекция на плоскость YZ постоянного угла между линиями OB и OW
    //if(fabs(W.y/B.y)<fabs(W.z/B.z))
         BOWyz = 0;//-acos( 0.5*(OWyz*OWyz + OByz*OByz - BWyz*BWyz)/OWyz/OByz );
    //else BOWyz =  acos( 0.5*(OWyz*OWyz + OByz*OByz - BWyz*BWyz)/OWyz/OByz );

         BOWxy = 0;
    //проекция на плоскость YZ постоянного угла между OA и строительной горизонталью вертолета
    AOH = asin( A.y / OAyz );//OA  нужна проекция на YOZ
    //проекция на плоскость YZ вертолета угла между OW и строительной горизонталью, в предельных положениях амортстойки
    double cosss_free = 0.5*(OAyz*OAyz + OByz*OByz - AByz*AByz)/fabs(OAyz)/fabs(OByz);
    cosss_free = (cosss_free>1)?(1):(cosss_free);///
    double angsx_free = acos(cosss_free) - AOH + BOWyz;
    //координаты характерных точек в локальной СК стойки, постоянные в этой СК
    Bloc = SKo2SKsh(B_,angsx_free,yrot);
    Wloc = SKo2SKsh(W_,angsx_free,yrot);
    //нижняя точка "на ободе колеса" (в плоскости симм.колеса) соответствующая максимально допустимому обжатию пневматика, с корректировкой по lambda
    double dDx = WD*sin(lambda);
    double dDy = WD - WD*cos(lambda);
    Dloc.x = Wloc.x - dDx *direct;
    Dloc.y = Wloc.y - WD*cos(ang_wh) + dDy*cos(0);/// + dDy*cos(angsx_free);
    Dloc.z = Wloc.z - WD*sin(ang_wh) - dDy*sin(0);/// - dDy*sin(angsx_free);
    D = D_ = SKsh2SKo(Dloc,angsx_free,yrot);
    //координаты точек в разжатом положении амортстойки
    Bfree = B_;
    Wfree = W_;
    Dfree = D_;
    D = D_ = corrD(D_);//корректировка по тангажу т.D в разжатом положении амортстойки

    //поиск т.В и angsx_full в положении пред.обжатой амортстойки
    double cosss_full = 0.5*(OAyz*OAyz + OByz*OByz - AByzfull*AByzfull)/fabs(OAyz)/fabs(OByz);
    cosss_full = (cosss_full>1)?(1):(cosss_full);///
    double angsx_full = acos(cosss_full) - AOH + BOWyz;
    //double ABtest=AB;
    //double dh=0;
    //while(ABtest>ABfull)
    //{
    //    dh+=0.0001;
    //    B.x = B_.x - dh*sin(lambda)/cos(lambda);
    //    B.y = B_.y + dh;
    //    B.z = sqrt(fabs(OB*OB-B.y*B.y)) *direct;
    //    //Bfull = B;
    //    angsx_full = -asin(B.y/OByz) + BOWyz;
    //    ABtest = sqrt( (A.x-B.x)*(A.x-B.x) + (A.y-B.y)*(A.y-B.y) + (A.z-B.z)*(A.z-B.z));
    //}
    //координата точки W в предельно обжатом положении амортстойки, нужна для 3D-визуализации
    Wfull = SKsh2SKo(Wloc,angsx_full,yrot);///
    //пересчет кинематики шасси при dha=0 в разжатом положении амортизатора
    get_pk(0);
    // проекция усилия на амортизаторе на верт.ось в начале отсчета
    Pay = ab->Pa *pkfa/pkfp;
}
void SkidType::get_pk(double dhay)// пересчет конфигурации стойки шасси /////////////
{   //dhay - проекция dha на вертикаль СКС
    //нижняя точка амортизатора
    B.x = B_.x - dhay*sin(lambda)/cos(lambda);
    B.y = B_.y + dhay;
    B.z = sqrt(OB*OB-B.y*B.y) *direct;
    //
    V.x = B.x;
    V.y = B.y;
    V.z = B.z;
    //верхняя точка амортизатора
    A.x = A_.x;
    A.y = A_.y;
    A.z = B.z;///A_.z;
    //ang_sx - угол между OW и строительной горизонталью вертолета
    ang_sx = -asin(B.y/OByz) + BOWyz;//проекция на плоскость YZ вертолета угла между OW и строительной горизонталью
    ang_sz = -asin(B.y/OBxy) + BOWxy;
    //центр колеса
    W = SKsh2SKo(Wloc,ang_sx,yrot);
    //нижняя точка обода колеса + корректировка по тангажу
    D = corrD( SKsh2SKo(Dloc,ang_sx,yrot) );
    //центры колес & нижние точки обода для пары колес
    W1.x = W.x;    W1.y = W.y;    W1.z = W.z;
    D1.x = D.x;    D1.y = D.y;    D1.z = D.z;
    W2.x = 0;      W2.y = 0;      W2.z = 0;
    D2.x = 0;      D2.y = 0;      D2.z = 0;
    //длина амортизатора условная (между A и B)
    AB  = sqrt((A.x-B.x)*(A.x-B.x) + (A.y-B.y)*(A.y-B.y) + (A.z-B.z)*(A.z-B.z));
    AByz= sqrt((A.y-B.y)*(A.y-B.y) + (A.z-B.z)*(A.z-B.z));
    ABxy= sqrt((A.y-B.y)*(A.y-B.y) + (A.x-B.x)*(A.x-B.x));
    //ang_a - угол между осью амортизатора и вертикалью (осью Y)
    ang_ax= acos((A.y-B.y)/AByz) *(-direct);
    ang_az= acos((A.y-B.y)/ABxy) *(-direct);
    //угол между плоскостью колеса и плоскостью симметрии вертолета
    ang_pn = 0;///(ang_sx + ang_wh)*direct;
    //передаточный коэффициент для усилий на штоке амортизатора
    pkfa = 1;///cos(ang_ax)*cos(ang_az);//<1
    //передаточный коэффициент для смещений амортизатора
    pk_a = pkfa;///if((dhay==0)||(AB_==AB)) pk_a = pkfa;  else pk_a = (AB_-AB)/dhay;//<1
    //передаточный коэффициент для пневматика
    pk_p = pkfp = 1;
    //if(B.z==0) pk_p = pkfp = 1;
    //else
    //{
    //    //передаточный коэффициент для смещений пневматика //ВНИМАНИЕ! в старой методике pk_p = pkfp = 1;
    //    pk_p = fabs( D.z / B.z );//>1
    //    //передаточный коэффициент для усилий на пневматике
    //    pkfp = pk_p;//fabs( D.z / B.z );//>1;
    //}
    pn->ang_sx = ang_sx;

    psih = pk_p / pk_a;
    psif = pkfp / pkfa;
}
/*xyz SkidType::getW(double ang_s_z)
{
    xyz _W_;

    _W_.x = -OWxy*cos(ang_s_z) + dOx;
    _W_.y = -OWxy*sin(ang_s_z);
    _W_.z = W.z;

return _W_;
}*/
/*Stojka_LeverType::Stojka_LeverType(int pneumID, int absorbID)
{
    //type = typeID;
    switch (pneumID)
    {
        case 1:
            pn = new Pneumatic();
        break;
        default:
            pn = new Pneumatic();
    }
    switch (absorbID)
    {
       case 1:
            ab = new Absorber1cam();
       break;
       case 2:
            ab = new Absorber2cam();
       break;
       case 3:
            ab = new AbsorberMi2();
       break;
       default:
            ab = new Absorber1cam();
    }
}
Stojka_LeverType::~Stojka_LeverType()
{
        delete ab;
        delete pn;
}
void Stojka_LeverType::Preprocess()// ОПРЕДЕЛЕНИЕ ОСНОВНЫХ НЕИЗМЕНЯЕМЫХ ПАРАМЕТРОВ ШАССИ (и изменяемых на шаге 0)
{
    commonPre();
    //проекция на плоскость XY постоянного угла между линиями OB и OW
    if(fabs(W.y/B.y)<fabs(W.x/B.x))
         BOWxy =-acos( 0.5*(OWxy*OWxy + OBxy*OBxy - BWxy*BWxy)/OWxy/OBxy );
    else BOWxy = acos( 0.5*(OWxy*OWxy + OBxy*OBxy - BWxy*BWxy)/OWxy/OBxy );
    //координаты характерных точек в локальной СК
    Dloc.x = Wloc.x = Bloc.x = 0;
    Dloc.y = Wloc.y = Bloc.y = 0;
    Dloc.z = Wloc.z = Bloc.z = 0;
    //координаты нижней точки амортизатора в предельных положениях
    xyz Bfull;
    Bfree = B_;
    Bfull.x = Bfree.x;
    Bfull.y = Bfree.y + ab->S;
    Bfull.z = Bfree.z;
    //угол между линией OB и горизонтальной плоскостью в предельных положениях
    double betafull= -asin( Bfull.y / OBxy );
    double betafree= -asin( Bfree.y / OBxy );
    //центр колеса в предельных положениях
    dOx = 0;//
    Wfree = getW(betafree);
    Wfull = getW(betafull);
    //нижняя точка обода колеса
    Dfree.x = Wfree.x;
    Dfree.y = Wfree.y - Rmd;
    Dfree.z = Wfree.z;
    D = D_ = corrD(Dfree);//корректировка т.D по тангажу
    //пересчет кинематики шасси при dha=0 в разжатом положении амортизатора
    get_pk(0);
    // проекция усилия на амортизаторе на верт.ось в начале отсчета
    Pay = ab->Pa *pkfa/pkfp;
}
void Stojka_LeverType::get_pk(double dhay)// ОПРЕДЕЛЕНИЕ РЕАЛЬНОГО ОБЖАТИЯ (ОБЩЕГО) АМОРТИЗАТОРА ПО ЗАДАННОЙ ПРОЕКЦИИ ОБЖАТИЯ НА ВЕРТИКАЛЬНУЮ ОСЬ (на направление силы действ. на пневматик)
{
    //нижняя точка амортизатора //double beta= -asin(B.y/OBxy);
    B.y = B_.y + dhay;
    B.x = -OBxy*cos(asin(B.y/OBxy));
    B.z = B_.z;//-OByz*cos(asin(B.y/OByz));//
    //точка крепления шатуна или амортстойки к рычагу
    V.x = B.x;
    V.y = B.y;
    V.z = B.z;
    //изменились координаты т.O  не передавать для следующей итерации !!!!!!!
    dOx = (B.x - B_.x);
    //начало локальной СК амортстойки
    O.x = O_.x - dOx;
    O.y = O_.y + 0;//меняется незначительно
    O.z = O_.z;
    //изменились координаты т.A
    A.x = A_.x + dOx;
    A.y = A_.y + 0;//меняется незначительно
    A.z = A_.z;
    //ang_s - угол между OW и строительной горизонталью вертолета
    ang_sx = -asin(B.y/OByz) - yrot;// + BOW;//ang_sx = 0;//тоже нужно вычислять
    ang_sz = -asin(B.y/OBxy) + BOWxy;//ang_sz = beta + BOWxy;//проекция на плоскость XY вертолета угла между OW и строительной горизонталью
    //центр & нижняя точка обода колеса
    W = getW(ang_sz);
    D.x = W.x ;
    D.y = W.y - Rmd;
    D.z = W.z ;//временно для одного колеса
    D = corrD(D);// учтен тангаж
    W1.x = W.x;     W1.y = W.y;     W1.z = W.z + whd;
    W2.x = W.x;     W2.y = W.y;     W2.z = W.z - whd;
    D1.x = D.x;     D1.y = D.y;     D1.z = D.z + whd;
    D2.x = D.x;     D2.y = D.y;     D2.z = D.z - whd;
    //длина амортизатора условная (между A и B)
    AB  = sqrt((A.x-B.x)*(A.x-B.x) + (A.y-B.y)*(A.y-B.y) + (A.z-B.z)*(A.z-B.z));
    AByz= sqrt((A.y-B.y)*(A.y-B.y) + (A.z-B.z)*(A.z-B.z));
    ABxy= sqrt((A.y-B.y)*(A.y-B.y) + (A.x-B.x)*(A.x-B.x));
    //угол между осью амортизатора и вертикалью (осью Y)
    ang_a = acos((A.y-B.y)/AB);// *(-direct);
    //ang_a - угол между осью амортизатора и плоскостью симметрии вертолета (осью Y)
    ang_ax = 0;//acos((A.y-B.y)/AByz) *(-direct);//
    ang_az = 0;//acos((A.y-B.y)/ABxy) *(-direct);//
    //угол между плоскостью колеса и плоскостью симметрии вертолета
    ang_pn = 0;//(ang_sx + ang_wh)*direct;//-(0.5*M_PI - ang_sx - (OWz-ang_wh)) *direct;
    //передаточный коэффициент для смещений амортизатора
    pk_a = cos(ang_a);///cos(ang_ax)*cos(ang_az);//1;
    //передаточный коэффициент для усилий на штоке амортизатора
    pkfa = pk_a;//1;
    if(B.x==0) pk_p = pkfp = 1;
    else
    {
        //передаточный коэффициент для смещений пневматика
        pk_p = fabs( D.x / B.x );//>1
        //передаточный коэффициент для усилий на пневматике
        pkfp = pk_p;//>1;
    }
    //передаточный коэффициент по перемещению
    psih = pk_p / pk_a;
    //передаточный коэффициент по усилию
    psif = pkfp / pkfa;
}
xyz Stojka_LeverType::getW(double ang_s_z)
{
    xyz _W_;
    //центр колеса
    _W_.x = -OWxy*cos(ang_s_z) + dOx;
    _W_.y = -OWxy*sin(ang_s_z);
    _W_.z = W.z;

return _W_;
}*/

