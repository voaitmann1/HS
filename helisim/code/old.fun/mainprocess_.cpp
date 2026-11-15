#include "main_.h"
QTextStream qout(stdout);


// П О Д Г О Т О В К А  К  Р А С Ч Е Т А М
bool Pre()//подготовка к сценарию
{
    readerror = false;
    //setStopped(false);
    ok = true;
    steps = -1;
    ini.start = 0;
    ini.calcmode = 0;
    ini.applyairdyn = 0;

    /* infilename = ui->pathEdit->text();
    QFile infile_(infilename);
    if(!infile_.exists())
    {
        OpenInputData();
        infilename = ui->pathEdit->text();
    }*/
    QFile infile(infilename);
    if(infile.exists())
    {
        titlefilename = getFileNameOnly(infilename);
        //setWindowTitle(titlefilename + " - " + titleapplname);//изменение заголовка окна
        logfilename = infilename+".log";
        //первая запись в log-файл
        QFile logfile(logfilename);
        logfile.open(QIODevice::WriteOnly | QIODevice::Text);
        QTextStream logstream(&logfile);//logstream.setCodec("CP1251");
        logstream << "ExeFile : " << QCoreApplication::applicationFilePath();
        logfile.close();
        //создание папки файлов состояния
        ini.datafolder = getStateDataFolderName(infilename);
        QDir datafolder(ini.datafolder);
        if(!datafolder.exists()) datafolder.mkdir(ini.datafolder);
        if(!datafolder.exists())
        {
            toMemo("ошибка создания папки ini.datafolder");
            toLogf("\nошибка создания папки ini.datafolder");
        }
        //фиксация текущих даты-времени как начала расчета
        startdate = QDate::currentDate();
        starttime = QTime::currentTime();
        tm.start();
        toLogf("Version : " + titleapplname);
        toLogf("Initial : " + infilename);
        toLogf("Started : " + startdate.toString("yyyy.MM.dd") + " " + starttime.toString());
        toMemo("Started: "  + startdate.toString("yyyy.MM.dd") + " " + starttime.toString());
        toLogf(singleline);
    }
    else { ok=0; toMemo("некорректное имя файла"); }
return ok;
}

// У П Р А В Л Е Н И Е  Р А С Ч Е Т А М И
bool MainProcess()//запуск подготовки и выполнения сценария
{
    switch (read_ini(infilename))
    {

    case 10: case 11://расчет НВ по имульсной теории, по классической теории
        //if(read_mr(  fname_mr  )) { ok=0;  toMemo("read errors in " + fname_mr ); }
        //запуск основного решателя винтов
        //MainRotorCalc();
        toMemo("MainRotorCalc() в процессе разработки");
    break;

    case 2://расчет РВ
        //if(read_tr(  fname_tr  )) { ok=0;  toMemo("read errors in " + fname_tr ); }
        //запуск основного решателя винтов
        //TailRotorCalc();
        toMemo("TailRotorCalc() в процессе разработки");
    break;

    case 3://балансировка
        if(read_frm( fname_frm )) { ok=0;  toMemo("read errors in " + fname_frm); }
        if(read_fax( fname_fax )) { ok=0;  toMemo("read errors in " + fname_fax); }
        if(read_mr(  fname_mr  )) { ok=0;  toMemo("read errors in " + fname_mr ); }
      //if(read_tr(  fname_tr  )) { ok=0;  toMemo("read errors in " + fname_tr ); }
        if(read_mrr( fname_mrr )) { ok=0;  toMemo("read errors in " + fname_mrr); }
        if(read_trr( fname_trr )) { ok=0;  toMemo("read errors in " + fname_trr); }
        //запуск основного решателя балансировки вертолета
        if(ok) MainBalancing();
    break;

    case 4://динамика полета/посадки
        if(read_frm( fname_frm )) { ok=0;  toMemo("read errors in " + fname_frm); }
      //if(read_fax( fname_fax )) { ok=0;  toMemo("read errors in " + fname_fax); }
      //if(read_mr(  fname_mr  )) { ok=0;  toMemo("read errors in " + fname_mr ); }
      //if(read_tr(  fname_tr  )) { ok=0;  toMemo("read errors in " + fname_tr ); }
        //после расчета винтов!:
        if(read_mrr( fname_mrr )) { ok=0;  toMemo("read errors in " + fname_mrr); }
        if(read_trr( fname_trr )) { ok=0;  toMemo("read errors in " + fname_trr); }
        if(ini.extloads)
        {
        if(read_load(fname_load1, load1, ini.load1pos)) { toMemo("read errors in " + fname_load1); }
        if(read_load(fname_load2, load2, ini.load2pos)) { toMemo("read errors in " + fname_load2); }
        }
        //if(read_cntr(fname_cntr)) {        toMemo("read errors in " + fname_cntr); }
        //не учитывать аэродинамические нагрузки
        ini.applyairdyn = false;
        //запуск основного решателя динамики посадки вертолета
        if(ok) MainLanding();       //MainFlight()
    break;

    //ошибки
    //case 0:  toMemo("reading error");           break;//ошибки считывания
    case 1:  toMemo("уточните модель расчета"); break;//расчет винта не указана методика расчета
    default: toMemo("invalid calcmode-index");  break;//некорректное значение
    }
    //загрузка .log файла
    //if(ok) loadLog();
return !ok;
}



// Д И Н А М И К А  П О Л Е Т А  И  П О С А Д К И
void MainLanding()//организатор моделирования посадки
{
    //удаление ранее рожденного объекта Helicopter (нужно при прерываниях расчета)
    hc = 0; delete hc;
    //создание объекта Landing
    hc = new Helicopter();
    //инициализация указателя класса на себя (нужно для статических функций)
    hc->SetClassPointer();
    //обнуление переменных
    hc->setNull();
    //передача в объект Landing исходных данных, считанных из файла
    SetLandingInitial();
    //подготовка к расчету, положение вертолета на шаге "0"
    i = 0;
    hc->iter = i;
    setExtLoads(0);
    hc->PreLanding(out);
    //запись результатов presolve
    toLogf(hc->IniPre2Log(ini.calcmode));//запись результатов presolve (шаг "0") //Landing.presolve
    toLogf(singleline);
    toLogf("Landing.Solve:\n");
    //запись шага "0" (начального положения)
    write_helicopter_state(ini.datafolder+"0");
    //результат presolve на форму
    //loadLog();//загрузить initial&presolve
    //ui->tabWidget->setCurrentIndex(1);//активная вкладка
    //setShassiButton(hc->nst);//установка активности кнопок выбора стоек
    //write_to_opengl();//opengl-прорисовка
    //установка итерации для продолжения расчета
    i = ini.start;
    if(i)
    {//если расчет нужно продолжить с итерации №ini.start
        readerror = read_helicopter_state( ini.datafolder + ss.setNum(i) );
        if(!readerror)
        {
            //ui->EditCurrent->setText(ss.setNum(i));
            SetDataFromFile();
        }
        else
        {
            toMemo("ошибка считывания файла состояния на шаге ini.start=" + ss.setNum(i));
            ok=false;
        }
    }
    if(ok)
    {
        hc->isHelicopterSolve = true;
        hc->isMRtest = false;
        LandingCycles();
    }
    else toMemo("ошибка запуска основного цикла LandingCycles");
}
void LandingCycles()//итерационные циклы посадки
{
    while( ok && i<ini.steps && !isStop )//
    {
        steps = i++;
        hc->iter = i;
        hc->t = hc->t_ + hc->dt;
        //ВНЕШНЯЯ НАРУЗКА
        outerloads();
        setExtLoads(hc->t);
        //расчет текущей итерации
        if(hc->SolveLanding(out)) ok=false;
        //вывод предупреждений
        if(hc->warnings)
        {
            toLogf(hc->log_ogl);
            //loadLog();
        }
        //вывод результатов расчета
        if(ok)
        {
            //результат расчета текущей итерации в общую кучу (в папку)
            if(i%ini.logstep==0)
            {
                if (write_helicopter_state( ini.datafolder + ss.setNum(i) ))//запись в файл состояния
                {
                    toLogf("ошибка записи в файл (write_helicopter_state)");
                    toMemo("ошибка записи в файл (write_helicopter_state)");
                }
            }
            //передача значений текущей итерации для расчета следующей итерации
            SetDataForNextI();
            //запись в log-файл
            //ui->showiter->setText(ss.setNum(i));
            //ui->showtime->setText(ss.setNum(hc->t,'f',6));
            //ui->progress->setValue( 100*i/float(ini.steps) );
            //setWindowTitle(ss.setNum(hc->t,'f',6) + " - " + titlefilename + " - " + titleapplname);
            //вывод результатов расчета текущей итерации на форму
            //if( ui->btn_control->isChecked() )  write_to_oglmemo( &(hc->logstring) );
            //opengl-прорисовка текущего положения вертолета
            //write_to_opengl();
            //прерывание расчета
            //QCoreApplication::processEvents();
        }
        else
        {
            toMemo("error at " + ss.setNum(i) + " iteration");
            toLogf("error at " + ss.setNum(i) + " iteration");
            //setStopped(1);
        }
    }//end of перебор итераций
    toLogf(hc->Post2log(ini.calcmode));//запись результатов
    QDate finishdate = QDate::currentDate();//фиксация текущего времени
    QTime finishtime = QTime::currentTime();//фиксация текуще date
    QString elapsed = getElapsedTime(tm);
    if(ok && !isStop) toLogf("Done!");  else toLogf("Aborted!");
    //else toLogf("Errors  : 0");// + hc->errors);
    //toLogf("Warnings: " + ss.setNum(hc->warning));
    toLogf("Finished: " + finishdate.toString("yyyy.MM.dd") + " " + finishtime.toString());
    toLogf("Elapsed : " + elapsed);
    toLogf("\n" + singleline);
    toMemo("Finishd: " + finishdate.toString("yyyy.MM.dd") + " " + finishtime.toString());
    toMemo("Elapsed: " + elapsed);
}
void SetLandingInitial()//LANDING передача исходных данных в объект hc
{
    hc->logkey       = ini.logkey;
    hc->applyairdyn  = ini.applyairdyn;//учитывать аэродинамические нагрузки
    hc->makebalance  = 0;//не выполнять балансировку вертолета
    hc->t_           = 0;
    hc->g            = ini.g;
    hc->dt           = ini.dt;
    hc->snlf6prc     = ini.snlf6prc;
    hc->snlf6lim     = ini.snlf6lim;
    hc->snlf2prc     = ini.snlf2prc;
    hc->snlf2lim     = ini.snlf2lim;
    hc->fix_orient_x = ini.fix_orient_x;
    hc->fix_orient_y = ini.fix_orient_y;
    hc->fix_orient_z = ini.fix_orient_z;
    hc->fix_orient_k = ini.fix_orient_k;
    hc->fix_orient_r = ini.fix_orient_r;
    hc->fix_orient_t = ini.fix_orient_t;

    //ПЛАНЕР
    hc->orient_.x    = ini.Hx;
    hc->orient_.y    = ini.Hy;
    hc->orient_.z    = ini.Hz;
    hc->orient_.k    = ini.kren_*M_PI/180.;
    hc->orient_.r    = ini.rysk_*M_PI/180.;
    hc->orient_.t    = ini.tang_*M_PI/180.;
    hc->v_.x         = ini.vHx;
    hc->v_.y         = ini.vHy;
    hc->v_.z         = ini.vHz;
    hc->w_.x         = ini.wHx;
    hc->w_.y         = ini.wHy;
    hc->w_.z         = ini.wHz;
    hc->mh           = ini.mH;
    hc->xctmr        = ini.xctmr*0.001;
    hc->yctmr        = ini.yctmr*0.001;
    hc->zctmr        = ini.zctmr*0.001;
    hc->Ix           = ini.Ix ;
    hc->Iy           = ini.Iy ;
    hc->Iz           = ini.Iz ;
    hc->Iyz          = ini.Iyz;
    hc->Ixz          = ini.Ixz;
    hc->Ixy          = ini.Ixy;

    //НВ
    hc->mrsolve      = ini.mrsolve;
    hc->eps_f        = ini.eps_f*M_PI/180.;
    hc->Dmr          = ini.mr.D *0.001;
    hc->l_mr         = ini.mr.l *0.001;
    hc->T_mr_        = ini.mr.T_;
    hc->H_mr_        = ini.mr.H_;
    hc->S_mr_        = ini.mr.S_;
    hc->Mx_mr_       = ini.mr.Mx_;
    hc->My_mr_       = ini.mr.My_;
    hc->Mz_mr_       = ini.mr.Mz_;
    if(ini.mrsolve)
    {
        hc->mr = new MainRotor();//new MainRotor( ini.a, ini.b );//тип --- 1; тип --- 1;
        hc->mr->SetClassPointer();
        hc->mr->g       = ini.g;
        hc->mr->dt      = ini.dt;
        hc->mr->Dmr     = ini.mr.D *0.001     ;//диаметр несущего винта, мм;
        hc->mr->nmr     = ini.mr.nmr                ;//частота вращения НВ, об/мин
        hc->mr->nb      = ini.mr.nb                 ;//количество лопастей НВ
        hc->mr->mb      = ini.mr.mb                 ;//масса лопасти НВ
        hc->mr->rvs     = ini.mr.rvs     *0.001     ;//разнос ВШ
        hc->mr->svs     = ini.mr.svs     *0.001     ;//смещение ВШ
        hc->mr->lct     = ini.mr.lct     *0.001     ;//расст. от ВШ до ЦТ лопасти
        hc->mr->d.h     = ini.mr.d.h     *0.001     ;//плечо силы на демпфере
        hc->mr->d.x2    = ini.mr.d.x2    *0.001     ;//коорд.шарнира демпфера на втулке отн.оси НВ
        hc->mr->d.z2    = ini.mr.d.z2    *0.001     ;//коорд.шарнира демпфера на втулке отн.оси НВ
        hc->mr->d.x3    = ini.mr.d.x3    *0.001     ;//коорд.шарнира демпфера на лопасти отн.оси НВ
        hc->mr->d.z3    = ini.mr.d.z3    *0.001     ;//коорд.шарнира демпфера на лопасти отн.оси НВ
        hc->mr->d.ro    = ini.mr.d.ro               ;//плотность жидкости в демпфере, кг/м^3:
        hc->mr->d.mu    = ini.mr.d.mu               ;//коэф. истечения жидкости в демпфере
        hc->mr->d.f     = ini.mr.d.f     *0.000001  ;//суммарная площадь отверстий в демпфере, мм^2:
        hc->mr->d.dv    = ini.mr.d.dv    *0.001     ;//диаметр штока демпфера, мм:
        hc->mr->d.dn    = ini.mr.d.dn    *0.001     ;//диаметр цилиндра демпфера внутренний, мм:
        hc->mr->d.pmax  = ini.mr.d.pmax             ;//давление срабатывания клапана демпфера, кгс/см2:
        hc->mr->d.fimax = ini.mr.d.fimax *M_PI/180.0;//предельный угол поворота лопасти отн.ВШ вперед, рад;
        hc->mr->d.fimin = ini.mr.d.fimin *M_PI/180.0;//предельный угол поворота лопасти отн.ВШ назад, рад;
        hc->mr->s.m     = ini.mr.s.m                ;//масса
        hc->mr->s.ro    = ini.mr.s.ro               ;//плотность жидкости в демпфере, кг/м^3:
        hc->mr->s.mu    = ini.mr.s.mu               ;//коэф. истечения жидкости в демпфере
        hc->mr->s.dvx   = ini.mr.s.dvx   *0.001     ;//диаметр штока демпфера стенда, мм
        hc->mr->s.dnx   = ini.mr.s.dnx   *0.001     ;//диаметр цилиндра демпфера стенда внутренний, мм;
        hc->mr->s.fx    = ini.mr.s.fx    *0.000001  ;//суммарная площадь отверстий в демпфере стенда, мм^2;
        hc->mr->s.kx    = ini.mr.s.kx               ;//жесткость пружины стенда, Н/м;
        hc->mr->s.dvz   = ini.mr.s.dvz   *0.001     ;//диаметр штока демпфера стенда, мм
        hc->mr->s.dnz   = ini.mr.s.dnz   *0.001     ;//диаметр цилиндра демпфера стенда внутренний, мм;
        hc->mr->s.fz    = ini.mr.s.fz    *0.000001  ;//суммарная площадь отверстий в демпфере стенда, мм^2;
        hc->mr->s.kz    = ini.mr.s.kz               ;//жесткость пружины стенда, Н/м;
        hc->mr->Jy      = ini.mr.Jy                 ;//
        hc->mr->cy      = ini.mr.cy      *1000000   ;//кН*м/мрад -> Н*м/рад
        hc->mr->ax_     = 0;//ini.mr.ax             ;//ускорение внешнего воздействия (на одной итерации ihit), м/с2
        hc->mr->az_     = 0;//ini.mr.az             ;//ускорение внешнего воздействия (на одной итерации ihit), м/с2
        hc->mr->vgsolve = ini.vgsolve               ;//
        if(ini.vgsolve)
        {
            hc->mr->vg.n = ini.mr.vg.n           ;//
            hc->mr->vg.m = ini.mr.vg.m           ;//
            hc->mr->vg.h = ini.mr.vg.h*0.001     ;//
            hc->mr->vg.d = ini.mr.vg.d*0.001     ;//
            hc->mr->vg.D = ini.mr.vg.D*0.001     ;//
            //hc->mr->M0vnesh = ini.mr.vg.M0;
        }
    }

    //РВ
    hc->trsolve      = ini.trsolve;
    hc->Dtr          = ini.tr.D *0.001;
    hc->x_tr         = ini.tr.Ox*0.001;
    hc->y_tr         = ini.tr.Oy*0.001;
    hc->z_tr         = ini.tr.Oz*0.001;
    hc->T_tr_        = ini.tr.T_;
    hc->H_tr_        = ini.tr.H_;
    hc->S_tr_        = ini.tr.S_;
    hc->Mx_tr_       = ini.tr.Mx_;
    hc->My_tr_       = ini.tr.My_;
    hc->Mz_tr_       = ini.tr.Mz_;

    //ГРУЗ НА ВНЕШНЕЙ ТРОСОВОЙ ПОДВЕСКЕ
    hc->zacep.x      = ini.xcg*0.001;
    hc->zacep.y      = ini.ycg*0.001;
    hc->zacep.z      = ini.zcg*0.001;
    hc->mcg          = ini.mcg;
    hc->lcg          = ini.lcg*0.001;

    //ШАССИ
    hc->nst          = ini.st[0].nst + ini.st[1].nst;
    for ( k = 0; k < hc->nst; k++)    //рождение и заполнение объектов класса IShassi
    {
        if ( k < ini.st[0].nst ) ik=0; else ik=1;
        switch (ini.st[ik].stype)
        {
            case 0://рычажная
                hc->st[k] = new Stojka_LeverType( ini.st[ik].ptype, ini.st[ik].atype );//тип пневматика 1; тип амортизатора 2;  //тип стойки 1;
            break;
            case 1://пирамидальная
                hc->st[k] = new Stojka_PyramidType( ini.st[ik].ptype, ini.st[ik].atype );//тип пневматика 1; тип амортизатора 2; //тип стойки 1;
            break;
        }
        hc->st[k]->bpa     =   ini.bpa;
        hc->st[k]->ab->kra =   ini.kra;
        hc->st[k]->krp     =   ini.krp;
        hc->st[k]->stype =     ini.st[ik].stype;
        hc->st[k]->ptype =     ini.st[ik].ptype;
        hc->st[k]->atype =     ini.st[ik].atype;
        hc->st[k]->nwh   =     ini.st[ik].nwh;
        hc->st[k]->whd   =     ini.st[ik].whd    *0.001;
        hc->st[k]->O_.x=       ini.st[ik].O_.x   *0.001;
        hc->st[k]->O_.y=       ini.st[ik].O_.y   *0.001;
        hc->st[k]->O_.z=       ini.st[ik].O_.z   *0.001;
        hc->st[k]->A_.x=      (ini.st[ik].A_.x-ini.st[ik].O_.x)*0.001;
        hc->st[k]->A_.y=      (ini.st[ik].A_.y-ini.st[ik].O_.y)*0.001;
        hc->st[k]->A_.z=      (ini.st[ik].A_.z-ini.st[ik].O_.z)*0.001;
        hc->st[k]->B_.x=      (ini.st[ik].B_.x-ini.st[ik].O_.x)*0.001;
        hc->st[k]->B_.y=      (ini.st[ik].B_.y-ini.st[ik].O_.y)*0.001;
        hc->st[k]->B_.z=      (ini.st[ik].B_.z-ini.st[ik].O_.z)*0.001;
        hc->st[k]->W_.x=      (ini.st[ik].W_.x-ini.st[ik].O_.x)*0.001;
        hc->st[k]->W_.y=      (ini.st[ik].W_.y-ini.st[ik].O_.y)*0.001;
        hc->st[k]->W_.z=      (ini.st[ik].W_.z-ini.st[ik].O_.z)*0.001;
        hc->st[k]->lambda=     ini.st[ik].lambda*M_PI/180.;
        hc->st[k]->ang_wh=     ini.st[ik].ang_wh*M_PI/180.;
        hc->st[k]->pn->dpmd=   ini.st[ik].pn.dpmd   *0.001;
        hc->st[k]->pn->cpx=    ini.st[ik].pn.cpx    *1000;
        hc->st[k]->pn->cpy=    ini.st[ik].pn.cpy    *1000;
        hc->st[k]->pn->cpz=    ini.st[ik].pn.cpz    *1000;
        hc->st[k]->pn->alfanp= ini.st[ik].pn.alfanp    ;
        hc->st[k]->pn->kdp=    ini.st[ik].pn.kdp       ;
        hc->st[k]->pn->Dp=     ini.st[ik].pn.D   *0.001;
        hc->st[k]->pn->Bp=     ini.st[ik].pn.B   *0.001;
        hc->st[k]->pn->isbrake=ini.st[ik].pn.isbrake;
        hc->st[k]->pn->f0=     ini.st[ik].pn.f0;
        hc->st[k]->pn->fs=     ini.st[ik].pn.fs;
        hc->st[k]->pn->fk=     ini.st[ik].pn.fk;
        hc->st[k]->ab->Vsum=   ini.st[ik].ab.Vsum *0.001;//
        hc->st[k]->ab->Voil=   ini.st[ik].ab.Voil *0.001;//
        hc->st[k]->ab->V0= hc->st[k]->ab->Vsum - hc->st[k]->ab->Voil;
        hc->st[k]->ab->ro=     ini.st[ik].ab.ro        ;
        hc->st[k]->ab->mu=     ini.st[ik].ab.mu        ;
        hc->st[k]->ab->p0=     ini.st[ik].ab.p0*98066.5;
        hc->st[k]->ab->kpoly=  ini.st[ik].ab.kpoly     ;
        hc->st[k]->ab->c1.s=   ini.st[ik].ab.c1.s   *0.001;
        hc->st[k]->ab->c1.D=   ini.st[ik].ab.c1.D   *0.001;
        hc->st[k]->ab->c1.Dv=  ini.st[ik].ab.c1.Dv  *0.001;
        hc->st[k]->ab->c1.Dn=  ini.st[ik].ab.c1.Dn  *0.001;
        hc->st[k]->ab->c1.fb=  ini.st[ik].ab.c1.fb  *0.000001;
        hc->st[k]->ab->c1.f=   ini.st[ik].ab.c1.f   *0.000001;
        hc->st[k]->ab->c1.fr=  ini.st[ik].ab.c1.fr  *0.000001;
        hc->st[k]->ab->c1.ktr= ini.st[ik].ab.c1.ktr       ;
        hc->st[k]->ab->c2.s = 0;
        if(ini.st[ik].atype > 1)
        {
            hc->st[k]->ab->c2.s=    ini.st[ik].ab.c2.s  *0.001;
            hc->st[k]->ab->c2.D=    ini.st[ik].ab.c2.D  *0.001;
            hc->st[k]->ab->c2.f=    ini.st[ik].ab.c2.f  *0.000001;
            hc->st[k]->ab->c2.fr=   ini.st[ik].ab.c2.fr *0.000001;
            hc->st[k]->ab->c2.ktr=  ini.st[ik].ab.c2.ktr    ;
        }
        hc->st[k]->ab->S = hc->st[k]->ab->c2.s + hc->st[k]->ab->c1.s;
        hc->st[k]->ground = ini.ground[k]*0.001;
        hc->st[k]->direct = 0;
    }
    //заполнение "симметричных" параметров (по disposition)
    for ( k = 0; k < ini.st[0].nst; k++)//передние/носовые опоры
    {
        switch (ini.st[0].nst)
        {
            case 0:
                hc->st[k]->direct = 0;//k=0
            break;
            case 1:
                hc->st[k]->direct = 0;//k=0
            break;
            case 2:
                if ( k == ini.st[0].nst -2 )  hc->st[k]->direct = 1;//k=1
                if ( k == ini.st[0].nst -1 )  hc->st[k]->direct =-1;//k=2
                hc->st[k]->O_.z =                  ini.st[ik].O_.z  *hc->st[k]->direct *0.001;
                hc->st[k]->B_.z = (ini.st[ik].B_.z-ini.st[ik].O_.z) *hc->st[k]->direct *0.001;
                hc->st[k]->A_.z = (ini.st[ik].A_.z-ini.st[ik].O_.z) *hc->st[k]->direct *0.001;
                hc->st[k]->W_.z = (ini.st[ik].W_.z-ini.st[ik].O_.z) *hc->st[k]->direct *0.001;
            break;
        }
    }
    for ( k = ini.st[0].nst; k < ini.st[0].nst + ini.st[1].nst; k++)//основные опоры
    {
        switch (ini.st[1].nst)
        {
            case 0:
                hc->st[k]->direct = 0;
            break;
            case 1:
                hc->st[k]->direct = 0;
            break;
            case 2:
                if ( k == ini.st[0].nst + ini.st[1].nst -2 )  hc->st[k]->direct = 1;//k=1 /2
                if ( k == ini.st[0].nst + ini.st[1].nst -1 )  hc->st[k]->direct =-1;//k=2 /3
                hc->st[k]->O_.z =                  ini.st[ik].O_.z  *hc->st[k]->direct *0.001;
                hc->st[k]->B_.z = (ini.st[ik].B_.z-ini.st[ik].O_.z) *hc->st[k]->direct *0.001;
                hc->st[k]->A_.z = (ini.st[ik].A_.z-ini.st[ik].O_.z) *hc->st[k]->direct *0.001;
                hc->st[k]->W_.z = (ini.st[ik].W_.z-ini.st[ik].O_.z) *hc->st[k]->direct *0.001;
            break;
        }
    }
    //хвостовая опора
    //

}
void SetDataForNextI()//установление "предыдущей итерации"
{
    hc->t_        = hc->t;
    //положение планера
    hc->orient_.x = hc->orient.x;
    hc->orient_.y = hc->orient.y;
    hc->orient_.z = hc->orient.z;
    hc->orient_.k = hc->orient.k;
    hc->orient_.r = hc->orient.r;
    hc->orient_.t = hc->orient.t;
    //скорость планера
    hc->v_.x      = hc->v.x;
    hc->v_.y      = hc->v.y;
    hc->v_.z      = hc->v.z;
    hc->w_.x      = hc->w.x;
    hc->w_.y      = hc->w.y;
    hc->w_.z      = hc->w.z;
    //положение стоек шасси
    for (k=0; k<hc->nst; k++)
    {
        hc->st[k]->O_         = hc->st[k]->O;
        hc->st[k]->A_         = hc->st[k]->A;
        hc->st[k]->B_         = hc->st[k]->B;
        hc->st[k]->D_         = hc->st[k]->D;
        hc->st[k]->C_         = hc->st[k]->C;
        hc->st[k]->saC_       = hc->st[k]->saC;
        hc->st[k]->pn->h_     = hc->st[k]->pn->h;
        hc->st[k]->ab->ha_    = hc->st[k]->ab->ha;
        hc->st[k]->ab->c1.h_  = hc->st[k]->ab->c1.h;
        hc->st[k]->ab->c2.h_  = hc->st[k]->ab->c2.h;
        hc->st[k]->ab->V_     = hc->st[k]->ab->V;
        hc->st[k]->AB_        = hc->st[k]->AB;
    }
}
void SetDataFromFile()//установление "предыдущей итерации"
{
    hc->t_        = hc->t        = out.t;
    hc->v_.x      = hc->v.x      = out.fus.v.x;
    hc->v_.y      = hc->v.y      = out.fus.v.y;
    hc->v_.z      = hc->v.z      = out.fus.v.z;
    hc->w_.x      = hc->w.x      = out.fus.w.x;
    hc->w_.y      = hc->w.y      = out.fus.w.y;
    hc->w_.z      = hc->w.z      = out.fus.w.z;
    hc->orient_.x = hc->orient.x = out.fus.orient.x *0.001;
    hc->orient_.y = hc->orient.y = out.fus.orient.y *0.001;
    hc->orient_.z = hc->orient.z = out.fus.orient.z *0.001;
    hc->orient_.k = hc->orient.k = out.fus.orient.k *M_PI/180.0;
    hc->orient_.r = hc->orient.r = out.fus.orient.r *M_PI/180.0;
    hc->orient_.t = hc->orient.t = out.fus.orient.t *M_PI/180.0;
    //hc->zacep.x   = out.cargo.Ox ;
    //hc->zacep.y   = out.cargo.Oy ;
    //hc->zacep.z   = out.cargo.Oz ;
    hc->crgct.x   = out.cargo.CTx;
    hc->crgct.y   = out.cargo.CTy;
    hc->crgct.z   = out.cargo.CTz;
    hc->Gcrg.x    = out.cargo.Gx ;
    hc->Gcrg.y    = out.cargo.Gy ;
    hc->Gcrg.z    = out.cargo.Gz ;

    //шасси
    for (k=0; k<hc->nst; k++)
    {
        hc->st[k]->O_.x       = hc->st[k]->O.x       = out.st[k].O.x    *0.001;
        hc->st[k]->O_.y       = hc->st[k]->O.y       = out.st[k].O.y    *0.001;
        hc->st[k]->O_.z       = hc->st[k]->O.z       = out.st[k].O.z    *0.001;
        hc->st[k]->A_.x       = hc->st[k]->A.x       = (out.st[k].A.x-out.st[k].O.x) *0.001;
        hc->st[k]->A_.y       = hc->st[k]->A.y       = (out.st[k].A.y-out.st[k].O.y) *0.001;
        hc->st[k]->A_.z       = hc->st[k]->A.z       = (out.st[k].A.z-out.st[k].O.z) *0.001;
        hc->st[k]->B_.x       = hc->st[k]->B.x       = (out.st[k].B.x-out.st[k].O.x) *0.001;
        hc->st[k]->B_.y       = hc->st[k]->B.y       = (out.st[k].B.y-out.st[k].O.y) *0.001;
        hc->st[k]->B_.z       = hc->st[k]->B.z       = (out.st[k].B.z-out.st[k].O.z) *0.001;
        hc->st[k]->D_.x       = hc->st[k]->D.x       = (out.st[k].D.x-out.st[k].O.x) *0.001;
        hc->st[k]->D_.y       = hc->st[k]->D.y       = (out.st[k].D.y-out.st[k].O.y) *0.001;
        hc->st[k]->D_.z       = hc->st[k]->D.z       = (out.st[k].D.z-out.st[k].O.z) *0.001;
        hc->st[k]->C_.x       = hc->st[k]->C.x       = (out.st[k].C.x-out.st[k].O.x) *0.001;
        hc->st[k]->C_.y       = hc->st[k]->C.y       = (out.st[k].C.y-out.st[k].O.y) *0.001;
        hc->st[k]->C_.z       = hc->st[k]->C.z       = (out.st[k].C.z-out.st[k].O.z) *0.001;
        hc->st[k]->saC_.x     = hc->st[k]->saC.x     = out.st[k].saC.x  *0.001;
        hc->st[k]->saC_.y     = hc->st[k]->saC.y     = out.st[k].saC.y  *0.001;
        hc->st[k]->saC_.z     = hc->st[k]->saC.z     = out.st[k].saC.z  *0.001;
        hc->st[k]->AB_        = hc->st[k]->AB        = out.st[k].AB     *0.001;
        hc->st[k]->ab->ha_    = hc->st[k]->ab->ha    = out.st[k].a.ha   *0.001;
        hc->st[k]->ab->c1.h_  = hc->st[k]->ab->c1.h  = out.st[k].a.ha1  *0.001;
        hc->st[k]->ab->c2.h_  = hc->st[k]->ab->c2.h  = out.st[k].a.ha2  *0.001;
        hc->st[k]->ab->V_     = hc->st[k]->ab->V     = out.st[k].a.V1   *0.001;
        //hc->st[k]->ab->V_     = hc->st[k]->ab->V     = out.st[k].a.V2   *0.001;
        hc->st[k]->pn->h_.x   = hc->st[k]->pn->h.x   = out.st[k].p.h.x  *0.001;
        hc->st[k]->pn->h_.y   = hc->st[k]->pn->h.y   = out.st[k].p.h.y  *0.001;
        hc->st[k]->pn->h_.z   = hc->st[k]->pn->h.z   = out.st[k].p.h.z  *0.001;

        hc->st[k]->pn->P.x   = out.st[k].p.P.x ;
        hc->st[k]->pn->P.y   = out.st[k].p.P.y ;
        hc->st[k]->pn->P.z   = out.st[k].p.P.z ;
        hc->st[k]->pn->P1.x  = out.st[k].p.P1.x;
        hc->st[k]->pn->P1.y  = out.st[k].p.P1.y;
        hc->st[k]->pn->P1.z  = out.st[k].p.P1.z;
        hc->st[k]->pn->P2.x  = out.st[k].p.P2.x;
        hc->st[k]->pn->P2.y  = out.st[k].p.P2.y;
        hc->st[k]->pn->P2.z  = out.st[k].p.P2.z;

        hc->st[k]->Aa        = out.st[k].Aa;
        hc->st[k]->Ap        = out.st[k].Ap;
    }
}
void outerloads()//итерационные циклы посадки
{
    hc->S_mr_add = 0;
    if(hc->mrsolve) hc->mr->Mvnesh = 0;//My_mr_add

    if((ini.F0)||(ini.M0))
    {
        if((hc->t_>=ini.t1)&&(hc->t_<ini.t2))
        {
            //1//постоянным воздействием
            //hc->S_mr = ini.F0;
            //hc->mr->az_ = ini.F0 / ini.mr.mvmr;//ini.mr.az;//20*g;
            //hc->mr->ax_ = 0;

            //гармонические колебания
            float whit = 2*M_PI*ini.niu;//круговая частота, rad/s
            float thit = hc->t_-ini.t1;
            //гармонические колебания поперечной силы на втулке НВ
            if(ini.F0)  hc->S_mr_add = ini.F0*cos(whit*thit);            //else        hc->S_mr = 0;
            //гармонические колебания крутящего момента
            if(hc->mrsolve)// if(ini.M0)
               hc->mr->Mvnesh = ini.M0*sin(whit*thit);// My_mr_add            //else        hc->M_mr = 0;
        }
    }
}
void setExtLoads(float t)
{
    extloadstruct a,b;
    float Fx, Fy, Fz;
    hc->FM1.Fx = 0;   hc->FM1.Mx = 0;
    hc->FM1.Fy = 0;   hc->FM1.My = 0;
    hc->FM1.Fz = 0;   hc->FM1.Mz = 0;
    hc->FM2.Fx = 0;   hc->FM2.Mx = 0;
    hc->FM2.Fy = 0;   hc->FM2.My = 0;
    hc->FM2.Fz = 0;   hc->FM2.Mz = 0;
    out.fus.F1.x = 0; out.fus.F1pos.x = 0;
    out.fus.F1.y = 0; out.fus.F1pos.y = 0;
    out.fus.F1.z = 0; out.fus.F1pos.z = 0;
    out.fus.F2.x = 0; out.fus.F2pos.x = 0;
    out.fus.F2.y = 0; out.fus.F2pos.y = 0;
    out.fus.F2.z = 0; out.fus.F2pos.z = 0;

    QString s, ss;//для вывода при отладке
    s = "t= ";
    s+=ss.setNum(t,'f',5);

    if(load1.size()>1)//ini.extload1
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
                out.fus.F1.x = Fx;  out.fus.F1pos.x = ini.load1pos.x;
                out.fus.F1.y = Fy;  out.fus.F1pos.y = ini.load1pos.y;
                out.fus.F1.z = Fz;  out.fus.F1pos.z = ini.load1pos.z;
                //Сила-момент в СКС
                hc->FM1.Fx  = Fx;
                hc->FM1.Fy  = Fy;
                hc->FM1.Fz  = Fz;
                hc->FM1.Mx  = Fz *ini.load1pos.y - Fy *ini.load1pos.z;
                hc->FM1.My  = Fx *ini.load1pos.z - Fz *ini.load1pos.x;
                hc->FM1.Mz  = Fy *ini.load1pos.x - Fx *ini.load1pos.y;
                s+="\tFx="; s+=ss.setNum(Fx,'f',1);
                s+="\tFy="; s+=ss.setNum(Fy,'f',1);
                s+="\tFz="; s+=ss.setNum(Fz,'f',1);
            }
        }
    }
    if(load2.size()>1)//ini.extload2
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
                out.fus.F2.x = Fx;  out.fus.F2pos.x = ini.load2pos.x;
                out.fus.F2.y = Fy;  out.fus.F2pos.y = ini.load2pos.y;
                out.fus.F2.z = Fz;  out.fus.F2pos.z = ini.load2pos.z;
                //Силы и моменты в СКС
                hc->FM2.Fx = Fx;
                hc->FM2.Fy = Fy;
                hc->FM2.Fz = Fz;
                hc->FM2.Mx = Fz *ini.load2pos.y - Fy *ini.load2pos.z;
                hc->FM2.My = Fx *ini.load2pos.z - Fz *ini.load2pos.x;
                hc->FM2.Mz = Fy *ini.load2pos.x - Fx *ini.load2pos.y;
                s+="\tFx="; s+=ss.setNum(Fx,'f',1);
                s+="\tFy="; s+=ss.setNum(Fy,'f',1);
                s+="\tFz="; s+=ss.setNum(Fz,'f',1);
            }
        }
    }

}
float inter(float x, float x1, float x2, float y1, float y2)
{
    if(x2==x1) return 0;
    else return  y1 + (x-x1)*(y2-y1)/(x2-x1);
}



// Б А Л А Н С И Р О В К А  В Е Р Т О Л Е Т А
void MainBalancing()//организатор расчета балансировки вертолета
{
    /*запуск внешних модулей
    //рождение процессов для сценария "Балансировка вертолета"
    process_balance = new QProcess(this);
    process_glrotor = new QProcess(this);
    QString ss, balanceout;
    i = 0;
    setStopped(false);//isStop = false;
    ui->progress->setValue(0);
    ui->memout->appendPlainText("i=" + ss.setNum(i) + ":\n");
    //задание списка аргументов для модуля balance на 0-й итерации
    arg_hc = arg;    arg_hc << ss.setNum(0);
    //----------------------------------------------------------------------------------------------------------------------
    if(!runBalance(process_balance,arg_hc))
    {
        readBalance(res_hc);//считывание из файла res_hc
        ui->memout->appendPlainText("");
    }
    else { ui->memout->appendPlainText( "runBalance error" ); setStopped(true); }
    //изменение списка аргументов для модуля balance
    arg_hc = arg;    arg_hc << ss.setNum(1);
    //do//----------------------------------------------------------------------------------------------------------------------
    for(i=1; i<=10; i++)
    {
        balanceout = "";
        ui->memout->appendPlainText("\n----------------------------------------------------------------------------------------------------------------------");
        ui->memout->appendPlainText("i=" + ss.setNum(i) + ":\n");
        //run main rotor
        if(!runRotors(process_glrotor,arg_mr))
        {
            readMRotor(res_glmr);//считывание из файла res_glmr
        }
        else { ui->memout->appendPlainText( "runRotors(mr) error" ); setStopped(true); }
        //run tail rotor
        if(!runRotors(process_glrotor,arg_tr))
        {
            readTRotor(res_gltr);//считывание из файла res_gltr
        }
        else { ui->memout->appendPlainText( "runRotors(tr) error" ); setStopped(true); }
        //run balance
        if(!runBalance(process_balance,arg_hc))
        {
            readBalance(res_hc);//считывание из файла res_hc
        }
        else { ui->memout->appendPlainText( "runBalance error" ); setStopped(true); }
        //расчет
        //currTtr = mRotor->M_kr / L;
        //delta_Xa_need = 100.0 * fabs(MRotor->Xa_need - curr_Xa_need) / fabs(curr_Xa);
        //delta_Ya_need = 100.0 * fabs(MRotor->Ya_need - curr_Ya_need) / fabs(curr_Ya);
        float delta_chi = 100.0 * fabs(mrotorfile.chi - balancefile.chi) / fabs(balancefile.chi);
        float delta_eta = 100.0 * fabs(mrotorfile.eta - balancefile.eta) / fabs(balancefile.eta);
        if ( (balancefile.delta_Xa < 1.0) && (balancefile.delta_Ya < 1.0) && (delta_chi < 1.0) && (delta_eta < 1.0) ) isBalanced = true; //|| (iterAmount > 15)
        //вывод на форму
        balanceout += "    %Xa=      " + ss.setNum(balancefile.delta_Xa,'f',3) + " %\tпогрешность по Xa_need\n";
        balanceout += "    %Ya=      " + ss.setNum(balancefile.delta_Ya,'f',3) + " %\tпогрешность по Ya_need\n";
        balanceout += "    %chi=     " + ss.setNum(delta_chi           ,'f',3) + " %\tпогрешность по chi\n";
        balanceout += "    %eta=     " + ss.setNum(delta_eta           ,'f',3) + " %\tпогрешность по eta\n";
        ui->memout->appendPlainText( balanceout );
        //прерывание расчета
        //QCoreApplication::processEvents();
        //ui->progress->setValue( 100*i/float(ini.steps) );
        ui->showiter->setText(ss.setNum(i+1));//ui->lcdnumber->display(i);
        ui->showtime->setText(ss.setNum(hc->t,'f',6));
        if(isStop) break;
    }//while((i<steps)&&(!isBalanced)&&(!isStop))// !stop

    //удаление процессов из памяти
    //delete process_glrotor;
    //delete process_balance;*/
    //удаление ранее рожденного объекта Helicopter (нужно при прерываниях расчета)
    hc = 0; delete hc;
    //создаем объект Balancing
    hc = new Helicopter();
    //инициализируем классу указатель на себя (нужно для статических функций)
    hc->SetClassPointer();
    //обнуление переменных
    hc->setNull();
    //передаем исходные данные
    SetBalanceInitial();
    //расчет вспомогательных параметров
    hc->PreBalance();
    //вывод подготовленных данных
    toLogf(hc->preout);
    //основной расчет
        hc->SolveBalance( 2 );//1,2    {
        //получение результатов расчета
        //toLogf(hc->solvout);//hc->PostBalance();
        //GetBalanceData();
        //hc->SolveBalance( 2 );
        //получили после балансировки:
            //balancefile.Xa_need;//Н\tпотребная пропульсивная сила НВ\n";
            //balancefile.Ya_need;//Н\tпотребная подъемная сила НВ\n";
            //balancefile.chi    ;//градус\tугол отклонения тарелки АП НВ по продольному каналу\n";
            //balancefile.eta    ;//градус\tугол отклонения тарелки АП НВ по поперечному каналу\n";
            //balancefile.T_tr   ;//Н\tпотребная тяга РВ, Н\n";

        //обработка результатов расчета
        hc->PostBalance();

        //запись результатов расчета в log-файл
        toLogf(hc->solvout);
        //запись искомых величин в файл с результатами
        write_outfile(fname_bal);//  */
        toLogf("SolveBalance() ok!");
        toMemo("SolveBalance() ok!");
    //}
    //else
    //{
    //    toLogf("SolveBalance() error!");
    //    toMemo("SolveBalance() error!");
    //}
    //расчет закончен, удаляем объект Balancing
    //delete hc;//  */
    //ui->tabWidget->setCurrentIndex(1);
}
void SetBalanceInitial()// передача исходных данных
{
    //toLogf(singleline);
    //toLogf("Balancing.initial:\n");
    hc->makebalance = ini.makebalance;
    hc->applyairdyn = ini.applyairdyn;//учитывать аэродинамические нагрузки
    hc->logkey   = ini.logkey;//выводить подробное состояние расчета
    //const
    hc->g        = ini.g    ;//9.80665;//ускорение свободного падения, м/с2
    hc->Lpt      = ini.Lpt  ;//0.0065; //скорость падения температуры К/м
    hc->RjmK     = ini.RjmK ;//8.31447;//газовая постоянная Дж/(мол*К)
    hc->RjkgK    = ini.RjkgK;//287.0; //газовая постоянная Дж/(кг*К)
    hc->Mm       = ini.Mm   ;//0.0289644; //молярная масса сухого воздуха кг/мол
    //ПАРАМЕТРЫ ПОЛЕТА
    hc->ro       = ini.ro   ;//
    hc->p0       = ini.p0   ;//
    hc->t0       = ini.t0   ;//
    hc->Hh       = ini.Hy   ;//0;//Высота полета, м
    hc->Vhx      = ini.vHx  ;//120 /3.6;//Скорость полета, км/час
    hc->loc      = ini.loc      ;//4;//закон управления стабилизатором, [1,2,3,4,5]
    hc->tetra    = ini.tetra*d2r;//угол наклона траектории, град =Teta_tr
    //hc->kren_  = ini.kren_    ;//0;//угол крена вертолета, рад       = gama
    hc->beta_    = ini.beta_*d2r;//Угол скольжения, град
    if(hc->makebalance==0)
    {
        hc->dkapa= ini.dkapa;
        hc->deta = ini.deta ;
        hc->kren = ini.kren ;
        hc->T_tr = ini.tr.T_;
    }
    //ГРУЗ НА ВНЕШНЕЙ ТРОСОВОЙ ПОДВЕСКЕ
    hc->zacep.x =  ini.xcg*0.001;
    hc->zacep.y =  ini.ycg*0.001;
    hc->zacep.z =  ini.zcg*0.001;
    hc->mcg     =  ini.mcg;
    hc->lcg     =  ini.lcg*0.001;

    //ПАРАМЕТРЫ ВЕРТОЛЕТА
    hc->mh       = ini.mH         ;//3550;//расчетная масса, кг
    hc->eps_f    = ini.eps_f  *d2r;//Угол заклинения фюзеляжа, рад
    hc->l_mr     = ini.mr.l *0.001;//расстояние по оси НВ от КПНВ до строительной горизонтали, мм;
    hc->xctmr    = ini.xctmr*0.001;//0.165;//Горизонтальная центровка вертолета в СК НВ, м  xTH = xt_in;
    hc->yctmr    = ini.yctmr*0.001;//1.700;//Вертикальная центровка вертолета в СК НВ, м    yTH = yt_in;
    hc->zctmr    = ini.zctmr*0.001;//0.000;//центровка вертолета вдоль Oz, м                zTH = zt_in;
    hc->x_f      = ini.x_f  *0.001;//0;//Характерная точка для определения скоса на фюзеляже от несущего винта по оси Ох, м
    hc->y_f      = ini.y_f  *0.001;//0;//Характерная точка для определения скоса на фюзеляже от несущего винта по оси Оу, м
    hc->z_f      = ini.z_f  *0.001;//0;//Характерная точка для определения скоса на фюзеляже от несущего винта по оси Оz, м
    hc->x_st     = ini.x_st *0.001;//-6.425572483;//Координаты стабилизатора в по Ox CK Oxyz, м
    hc->y_st     = ini.y_st *0.001;//-0.018215118;//Координаты стабилизатора в по Оу CK Oxyz, м
    hc->z_st     = ini.z_st *0.001;//0;//Координаты стабилизатора в по Оz CK Oxyz, м
    hc->x_tr     = ini.tr.Ox*0.001;//-8.767456812;//Расстояние от оси несущего винта до оси рулевого винта, м
    hc->y_tr     = ini.tr.Oy*0.001;//0.796711039;//Расстояние от оси рулевого винта до Oxyz. м
    hc->z_tr     = ini.tr.Oz*0.001;//0.445;//Расстояние от начала координат до плоскости вращения рулевого винта по оси Oz. м
    //hc->x_k    = ini.x_k  *0.001;//-8.5;//Растояние от оси вращения несущего винта до центра давления киля (25% хорды bсах), м
    hc->fi0_min  = ini.fi0_min*d2r;//
    hc->fi0_max  = ini.fi0_max*d2r;//
    hc->est_min  = ini.est_min*d2r;//
    hc->est_max  = ini.est_max*d2r;//
    //ПАРАМЕТРЫ модели
    hc->lmodel   = ini.lmodel     ;//0.76;//Линейный размер на модели, м
    hc->Smodel   = ini.Smodel     ;//0.012;//Площадь миделя модели, м2
    hc->Mmodel   = ini.Mmodel     ;//16.45;//Масштаб модели
    //НЕСУЩИЙ ВИНТ
    hc->w_mr     = ini.mr.w_mr    ;//187/7.25;//угловая скорость вращения НВ при заданом режиме, рад/с. 1/с
    hc->R_mr     = ini.mr.R_mr    ;//7.25;//Радиус несущего винта, м
    hc->zl_mr    = ini.mr.zl      ;//3;//Количество лопастей несущего винта
    hc->Sgsh_mr  = ini.mr.Sgsh_mr ;//197;//Стат. момент инерции лопасти относительно г.ш, N*m
    hc->B_mr     = ini.mr.B_mr    ;//0.97;//Коэф. концевых потерь
    hc->D1_mr_   = ini.mr.D1_mr_  ;//1.5646828;
    hc->D2_mr_   = ini.mr.D2_mr_  ;//-0.0883295;
    //АВТОМАТ ПЕРЕКОСА
    hc->k_       = ini.mr.k_         ;//0.4;//Компенсатор взмаха
    hc->Rap      = ini.mr.Rap        ;//0.265;//радиус крепления тяг, м
    hc->ll       = ini.mr.ll         ;//0.157;//
    hc->lgsh     = ini.mr.lgsh       ;//0.102;//Расстояние от оси несущего винта до оси ГШ, м
    hc->lvsh     = ini.mr.lvsh       ;//0.262;//разнос вертикальных шарниров, м
    hc->dgsh     = ini.mr.dgsh       ;//0.035;//вынос горизонтального шарнира вперед, м
    hc->dpsi_cont= ini.mr.dpsi_cont*d2r;//угол опережения управления, град
    //РУЛЕВОЙ ВИНТ
    //hc->i_tr   = ini.i_tr ;//0.24496975831;//передаточное число главного редуктора от выходного вала двигателя к рулевому винту (расчетное)
    //hc->R_tr   = ini.R_tr;//1.35;//Радиус рулевого винта, м
    //hc->w_tr   = ini.w_tr;//151.356;//Угловая скорость вращения рулевого винта при заданом режиме, рад/с
    //ИЗ КЛАСИЧЕСКОЙ ТЕОРИИ
    hc->beta_    = ini.beta_    ;//0;//Угол скольжения, рад
    hc->alfa_mr__= ini.mr.alfa__*d2r;//Угол между набегающим потоком и КПВ, рад
    hc->kapa_mr_ = ini.mr.kapa_ *d2r;//Угол отклонения кольца автомата перекоса в продольном направлении, рад
    hc->eta_mr_  = ini.mr.eta_  *d2r;//Угол отклонения кольца автомата перекоса в поперечном направлении, рад
    hc->fi0_mr_  = ini.mr.fi0_  *d2r;//Угол общего шага, рад
    hc->T_mr_    = ini.mr.T_    ;//35527.516;//Тяга несущего винта, Н           [положительное направление - вверх]
    hc->H_mr_    = ini.mr.H_    ;//1674.038;//Продольная сила несущего винта, Н [положительное направление - назад]
    hc->S_mr_    = ini.mr.S_    ;//-262.093;//Боковая сила несущего винта, Н    [положительное направление - влево]
    hc->Mx_mr_   = ini.mr.Mx_   ;//256.192;//Момент крена несущего винта, Нм
    hc->My_mr_   = ini.mr.My_   ;//9350.742;//Момент кручения, Нм
    hc->Mz_mr_   = ini.mr.Mz_   ;//-905.401;//Момент тангажа несущего винта, Нм
    hc->T_tr_    = ini.mr.T_    ;//-944.835;//Тяга рулевого винта, Н
    hc->H_tr_    = ini.mr.H_    ;//0; //Продольная сила рулевого винта, Н
    hc->S_tr_    = ini.mr.S_    ;//-0;//Поперечная сила рулевого винта, Н
    hc->Mx_tr_   = ini.mr.Mx_   ;//0;//Продольный момент рулевого винта, Н*м
    hc->My_tr_   = ini.mr.My_   ;//60.7;//Вертикальный момент рулевого винта, Н*м
    hc->Mz_tr_   = ini.mr.Mz_   ;//0;//Поперечный момент, рулевого винта, Н*м
    //указатели на массивы:
    hc->_alfa_f  = ini._alfa_f;    hc->_alfa_f_col  = ini._alfa_f_col  ;
    hc->_cx_f    = ini._cx_f;    //hc->_cx_f_col    = ini._cx_f_col    ;
    hc->_cy_f    = ini._cy_f;    //hc->_cy_f_col    = ini._cy_f_col    ;
    hc->_cz_f    = ini._cz_f;    //hc->_cz_f_col    = ini._cz_f_col    ;
    hc->_mx_f    = ini._mx_f;    //hc->_mx_f_col    = ini._mx_f_col    ;
    hc->_my_f    = ini._my_f;    //hc->_my_f_col    = ini._my_f_col    ;
    hc->_mz_f    = ini._mz_f;    //hc->_mz_f_col    = ini._mz_f_col    ;
    hc->_mz_fst  = ini._mz_fst;    hc->_mz_fst_row  = ini._mz_fst_row  ; hc->_mz_fst_col = ini._mz_fst_col;
    hc->__cx_st  = ini.__cx_st;    hc->__cx_st_row  = ini.__cx_st_row  ; hc->__cx_st_col = ini.__cx_st_col;
    hc->__cy_st  = ini.__cy_st;    hc->__cy_st_row  = ini.__cy_st_row  ; hc->__cy_st_col = ini.__cy_st_col;
    hc->__mz_st  = ini.__mz_st;    hc->__mz_st_row  = ini.__mz_st_row  ; hc->__mz_st_col = ini.__mz_st_col;
    hc->__alfa_f = ini.__alfa_f;   hc->__alfa_f_col = ini.__alfa_f_col ;//нужен для StabilizerForces()
    hc->__eps_st = ini.__eps_st;   hc->__eps_st_col = ini.__eps_st_col ;
    hc->_Vvol_mr = ini._Vvol_mr;   hc->_Vvol_mr_col = ini._Vvol_mr_col ;
    hc->_delta_mr= ini._delta_mr;  hc->_delta_mr_col= ini._delta_mr_col;
    hc->_uvol_mr = ini._uvol_mr; //hc->_uvol_mr_row = ini._uvol_mr_row ; hc->_uvol_mr_col=_uvol_mr_col;
    hc->_h_      = ini._h_;        hc->_h_col       = ini._h_col       ;
    hc->_Ksi3n_  = ini._Ksi3n_;  //hc->_Ksi3n_col   = ini._Ksi3n_col   ;
    //временные массивы используемые в расчете StabilizerForces()
    hc->__eps_st0= ini.__eps_st0;
    hc->_dalf_stf= ini._dalf_stf;

    /*///передача1 исх. данных в решатель балансировки
    ////Helicopter
    //balancingSolver->SetValue("V_in", (V * 3.6));
    //balancingSolver->SetValue("ro_in", ro);
    //balancingSolver->SetValue("Theta_in", Theta);
    //balancingSolver->SetValue("m_in", m);
    //balancingSolver->SetValue("xt_in", x_t);
    //balancingSolver->SetValue("yt_in", y_t);
    //balancingSolver->SetValue("zt_in", z_t);
    //balancingSolver->SetValue("H_in", H);
    ////MainRotor
    //balancingSolver->SetValue("R_mr_in", MRotor->R);
    //balancingSolver->SetValue("w_mr_in", MRotor->w);
    //balancingSolver->SetValue("z_mr_in", MRotor->z);
    //balancingSolver->SetValue("Sgsh_mr_in", MRotor->S_g_sh);
    //balancingSolver->SetValue("B_mr_in", MRotor->B);
    //balancingSolver->SetValue("D1_mr_in", MRotor->D1);
    //balancingSolver->SetValue("D2_mr_in", MRotor->D2);
    ////TailRotor
    //balancingSolver->SetValue("R_tr_in", TRotor->R);
    //balancingSolver->SetValue("w_tr_in", TRotor->w);
    //
    ////передача2 исх. данных в решатель балансировки
    ////MainRotor
    //balancingSolver->SetValue("alpha_mr_in", MRotor->alpha);
    //balancingSolver->SetValue("eta_mr_in", MRotor->eta);
    //balancingSolver->SetValue("chi_mr_in", MRotor->chi);
    //balancingSolver->SetValue("T_mr_in", MRotor->T);
    //balancingSolver->SetValue("H_mr_in", MRotor->H);
    //balancingSolver->SetValue("S_mr_in", MRotor->S);
    //balancingSolver->SetValue("Mx_mr_in", MRotor->Mx);
    //balancingSolver->SetValue("My_mr_in", MRotor->M_kr);
    //balancingSolver->SetValue("Mz_mr_in", MRotor->Mz);
    //balancingSolver->SetValue("fi0_mr_in", MRotor->fi0);
    ////TailRotor
    //balancingSolver->SetValue("T_tr_in", TRotor->T);
    //balancingSolver->SetValue("H_tr_in", 0);
    //balancingSolver->SetValue("S_tr_in", 0);
    //balancingSolver->SetValue("Mx_tr_in", TRotor->Mx);
    //balancingSolver->SetValue("My_tr_in", TRotor->M_kr);
    //balancingSolver->SetValue("Mz_tr_in", TRotor->Mz);
*/
    QString ss, readout="Initialdata:\n";
    if(hc->logkey>0)
    {
    readout += "      ro=  \t"      + ss.setNum( hc->ro       ,'f',3) + " \n";
    readout += "      p0=  \t"      + ss.setNum( hc->p0       ,'f',3) + " \n";
    readout += "      g=   \t"      + ss.setNum( hc->g        ,'f',5) + " \n";
    readout += "      Lpt= \t"      + ss.setNum( hc->Lpt      ,'f',5) + " \n";
    readout += "      RjmK=\t"      + ss.setNum( hc->RjmK     ,'f',5) + " \n";
    readout += "      RjkgK=\t"     + ss.setNum( hc->RjkgK    ,'f',3) + " \n";
    readout += "      Mm=   \t"     + ss.setNum( hc->Mm       ,'f',6) + " \n";
    readout += "\n   параметры полета:\n";
    readout += "      Hh=  \t"      + ss.setNum( hc->Hh       ,'f',1) + " \n";
    readout += "      Vhx=\t"       + ss.setNum( hc->Vhx      ,'f',1) + " \n";
    readout += "      loc=\t"       + ss.setNum( hc->loc            ) + " \n";
    readout += "      tetra=\t"     + ss.setNum( hc->tetra    ,'f',3) + " \n";
    //readout += "      kren_=\t"     + ss.setNum( hc->kren_    ,'f',3) + " \n";
    readout += "      beta_=\t"     + ss.setNum( hc->beta_    ,'f',3) + " \n";
    readout += "\n   планер:\n";
    readout += "      mh=  \t"      + ss.setNum( hc->mh       ,'f',1) + " \n";
    readout += "      xctmr=\t"     + ss.setNum( hc->xctmr    ,'f',3) + " \n";
    readout += "      yctmr=\t"     + ss.setNum( hc->yctmr    ,'f',3) + " \n";
    readout += "      zctmr=\t"     + ss.setNum( hc->zctmr    ,'f',3) + " \n";
    readout += "      x_mr=\t"      + ss.setNum( hc->x_mr     ,'f',3) + " \n";
    readout += "      y_mr=\t"      + ss.setNum( hc->y_mr     ,'f',3) + " \n";
    readout += "      z_mr=\t"      + ss.setNum( hc->z_mr     ,'f',3) + " \n";
    readout += "      x_f=\t"       + ss.setNum( hc->x_f      ,'f',3) + " \n";
    readout += "      y_f=\t"       + ss.setNum( hc->y_f      ,'f',3) + " \n";
    readout += "      z_f=\t"       + ss.setNum( hc->z_f      ,'f',3) + " \n";
    readout += "      x_st=\t"      + ss.setNum( hc->x_st     ,'f',3) + " \n";
    readout += "      y_st=\t"      + ss.setNum( hc->y_st     ,'f',3) + " \n";
    readout += "      z_st=\t"      + ss.setNum( hc->z_st     ,'f',3) + " \n";
    readout += "      x_tr=\t"      + ss.setNum( hc->x_tr     ,'f',3) + " \n";
    readout += "      y_tr=\t"      + ss.setNum( hc->y_tr     ,'f',3) + " \n";
    readout += "      z_tr=\t"      + ss.setNum( hc->z_tr     ,'f',3) + " \n";
    //readout += "      x_k=  \t"     + ss.setNum( hc->x_k      ,'f',3) + " \n";
    readout += "      eps_f=\t"     + ss.setNum( hc->eps_f    ,'f',6) + " \n";
    readout += "      fi0_min=\t"   + ss.setNum( hc->fi0_min  ,'f',6) + " \n";
    readout += "      fi0_max=\t"   + ss.setNum( hc->fi0_max  ,'f',6) + " \n";
    readout += "      est_min=\t"   + ss.setNum( hc->est_min  ,'f',6) + " \n";
    readout += "      est_max=\t"   + ss.setNum( hc->est_max  ,'f',6) + " \n";
    readout += "      lmodel=\t"    + ss.setNum( hc->lmodel   ,'f',3) + " \n";
    readout += "      Smodel=\t"    + ss.setNum( hc->Smodel   ,'f',3) + " \n";
    readout += "      Mmodel=\t"    + ss.setNum( hc->Mmodel   ,'f',3) + " \n";
    readout += "\n   несущий винт:\n";
    readout += "      w_mr=\t"      + ss.setNum( hc->w_mr     ,'f',3) + " \n";
    readout += "      R_mr=\t"      + ss.setNum( hc->R_mr     ,'f',3) + " \n";
    readout += "      zl_mr=\t"     + ss.setNum( hc->zl_mr          ) + " \n";
    readout += "      Sgsh_mr=\t"   + ss.setNum( hc->Sgsh_mr  ,'f',2) + " \n";
    readout += "      B_mr=\t"      + ss.setNum( hc->B_mr     ,'f',3) + " \n";
    readout += "      D1_mr_=\t"    + ss.setNum( hc->D1_mr_   ,'f',6) + " \n";
    readout += "      D2_mr_=\t"    + ss.setNum( hc->D2_mr_   ,'f',6) + " \n";
    readout += "      k_=  \t"      + ss.setNum( hc->k_       ,'f',3) + " \n";
    readout += "      Rap=\t"       + ss.setNum( hc->Rap      ,'f',3) + " \n";
    readout += "      ll=  \t"      + ss.setNum( hc->ll       ,'f',3) + " \n";
    readout += "      lgsh=\t"      + ss.setNum( hc->lgsh     ,'f',3) + " \n";
    readout += "      lvsh=\t"      + ss.setNum( hc->lvsh     ,'f',3) + " \n";
    readout += "      dgsh=\t"      + ss.setNum( hc->dgsh     ,'f',3) + " \n";
    readout += "      dpsi_cont=\t" + ss.setNum( hc->dpsi_cont,'f',6) + " \n";

    readout += "      T_mr_=\t"     + ss.setNum( hc->T_mr_    ,'f',3) + " \n";
    readout += "      H_mr_=\t"     + ss.setNum( hc->H_mr_    ,'f',3) + " \n";
    readout += "      S_mr_=\t"     + ss.setNum( hc->S_mr_    ,'f',3) + " \n";
    readout += "      Mx_mr_=\t"    + ss.setNum( hc->Mx_mr_   ,'f',3) + " \n";
    readout += "      My_mr_=\t"    + ss.setNum( hc->My_mr_   ,'f',3) + " \n";
    readout += "      Mz_mr_=\t"    + ss.setNum( hc->Mz_mr_   ,'f',3) + " \n";
    readout += "      alfa_mr__=\t" + ss.setNum( hc->alfa_mr__,'f',6) + " \n";
    readout += "      kapa_mr_=\t"  + ss.setNum( hc->kapa_mr_ ,'f',6) + " \n";
    readout += "      eta_mr_=\t"   + ss.setNum( hc->eta_mr_  ,'f',6) + " \n";
    readout += "      fi0_mr_=\t"   + ss.setNum( hc->fi0_mr_  ,'f',6) + " \n";
    readout += "\n   рулевой винт:\n";
    readout += "      T_tr_=\t"     + ss.setNum( hc->T_tr_    ,'f',3) + " \n";
    readout += "      H_tr_=\t"     + ss.setNum( hc->H_tr_    ,'f',3) + " \n";
    readout += "      S_tr_=\t"     + ss.setNum( hc->S_tr_    ,'f',3) + " \n";
    readout += "      Mx_tr_=\t"    + ss.setNum( hc->Mx_tr_   ,'f',3) + " \n";
    readout += "      My_tr_=\t"    + ss.setNum( hc->My_tr_   ,'f',3) + " \n";
    readout += "      Mz_tr_=\t"    + ss.setNum( hc->Mz_tr_   ,'f',3) + " \n";
    //readout +=  "\n   аэродинамические характеристики:\n";

    /*/вывести массив
    //if(rdata.log>2)
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
        toLogf(sss);*/


    //readout += readoutmass;
    }
    toLogf(readout);
}
void write_outfile(QString outfileName)//ResToMemoOut
{
    QString ss;
    QString ob_out = "балансировка вертолета: \n";//" + ss.setNum( calcmode ) + "
    ob_out += "--------------------------------------------------------------------------------\n";
    ob_out += "сходимость расчета:\n";
    //ob_out += "    isBalanced= "+ ss.setNum(hc->isBalanced)       + "\t(0-несбалансировано 1-сбалансировано)\n";
    ob_out += "    %Xa_need=  " + ss.setNum(hc->proX         ,'f',3) + " %  \tпогрешность по Xa_need\n";
    ob_out += "    %Ya_need=  " + ss.setNum(hc->proY         ,'f',3) + " %  \tпогрешность по Ya_need\n";
    ob_out += "    %T_tr=     " + ss.setNum(hc->dT_tr        ,'f',3) + " %  \tпогрешность по Ttr\n";
    //ob_out += "    %kapa=     " + ss.setNum(hc->pro_kapa   ,'f',3) + " %  \tпогрешность по kapa\n";
    //ob_out += "    %eta=      " + ss.setNum(hc->pro_eta 	  ,'f',3) + " %  \tпогрешность по eta\n";
    ob_out += "параметры винтов:\n";
    ob_out += "    Xa_need=   " + ss.setNum(hc->Xneed        ,'f',3) + " Н  \tпотребная пропульсивная сила НВ\n";
    ob_out += "    Ya_need=   " + ss.setNum(hc->Yneed        ,'f',3) + " Н  \tпотребная подъемная сила НВ\n";
    ob_out += "    d_chi=     " + ss.setNum(hc->dkapa*r2d    ,'f',4) + " градус\tпоправка к углу отклонения тарелки АП НВ по продольному каналу (=delta_chi)\n";
    ob_out += "    d_eta=     " + ss.setNum(hc->deta*r2d     ,'f',4) + " градус\tпоправка к углу отклонения тарелки АП НВ по поперечному каналу\n";
    ob_out += "    chi=       " + ss.setNum(hc->kapa_mr*r2d  ,'f',4) + " градус\tугол отклонения тарелки АП НВ по продольному каналу (=chi)\n";
    ob_out += "    eta=       " + ss.setNum(hc->eta_mr*r2d   ,'f',4) + " градус\tугол отклонения тарелки АП НВ по поперечному каналу\n";
    ob_out += "    alfa_mr=   " + ss.setNum(hc->alfa_mr*r2d  ,'f',4) + " градус\tугол атаки НВ\n";
    ob_out += "    T_tr=      " + ss.setNum(hc->T_tr         ,'f',3) + " Н  \tпотребная тяга РВ, Н\n";

    ob_out += "параметры фюзеляжа:\n";
    ob_out += "    alfa_f=    " + ss.setNum(hc->alfa_f*r2d   ,'f',4) + " градус\tугол атаки фюзеляжа\n";
    //ob_out += "    dalf_f_mr= " + ss.setNum(hc->dalf_f_mr* ,'f',4) + " градус\tскос потока на фюзеляже от НВ\n";
    ob_out += "    kren=      " + ss.setNum(hc->orient.k*r2d ,'f',4) + " градус\tугол крена вертолета (gama)\n";
    ob_out += "    tang=      " + ss.setNum(hc->orient.t*r2d ,'f',4) + " градус\tугол тангажа вертолета\n";
    //ob_out += "    tangf=     " + ss.setNum(hc->ufn*r2d    ,'f',4) + " градус\tугол тангажа по СГФ\n";
    //ob_out += "    tau_obd=   " + ss.setNum(hc->tau_obd    ,'f',4) + " \n";
    ob_out += "    CxS=       " + ss.setNum(hc->CxS          ,'f',4) + " \t\n";
    ob_out += "    X_f=       " + ss.setNum(hc->X_f          ,'f',3) + " Н  \tсила сопротивления фюзеляжа (СК фюзеляжа)\n";
    ob_out += "    Y_f=       " + ss.setNum(hc->Y_f          ,'f',3) + " Н  \tподъемная сила фюзеляжа (СК фюзеляжа)\n";
    ob_out += "    Z_f=       " + ss.setNum(hc->Z_f          ,'f',3) + " Н  \tбоковая сила фюзеляжа (СК фюзеляжа)\n";
    ob_out += "    Mx_f=      " + ss.setNum(hc->Mx_f         ,'f',3) + " Н*м\tмомент фюзеляжа отн. оси X (СК фюзеляжа)\n";
    ob_out += "    My_f=      " + ss.setNum(hc->My_f         ,'f',3) + " Н*м\tмомент фюзеляжа отн. оси Y (СК фюзеляжа)\n";
    ob_out += "    Mz_f=      " + ss.setNum(hc->Mz_f         ,'f',3) + " Н*м\tмомент фюзеляжа отн. оси Z (СК фюзеляжа)\n";

    ob_out += "параметры стабилизатора:\n";
    ob_out += "    eps_st_mr= " + ss.setNum(hc->eps_st_mr*r2d,'f',4) + " градус\tугол установки стабилизатора относительно КПНВ\n";
    ob_out += "    alfa_st=   " + ss.setNum(hc->alfa_st*r2d  ,'f',4) + " градус\tугол атаки стабилизатора\n";
    //ob_out += "    dalf_st_mr= "+ ss.setNum(hc->dalf_st_mr*,'f',4) + " градус\tскос потока у стабилизатора от НВ\n";
    //ob_out += "    dalf_st_f= " + ss.setNum(hc->dalf_st_f* ,'f',4) + " градус\tскос потока у стабилизатора от фюзеляжа\n";
    //ob_out += "    dalf_st=   " + ss.setNum(hc->dalf_st*r2d,'f',4) + " градус\tскос потока на стабилизаторе\n";
    ob_out += "    X_st=      " + ss.setNum(hc->X_st         ,'f',3) + " Н  \tсила сопротивления стабилизатора (СК фюзеляжа)\n";
    ob_out += "    Y_st=      " + ss.setNum(hc->Y_st         ,'f',3) + " Н  \tподъемная сила стабилизатора (СК фюзеляжа)\n";

    if(!outfileName.isEmpty())
    {
        QFile outfile(outfileName);
        outfile.open( QIODevice::WriteOnly | QIODevice::Text );
        if(outfile.isWritable())
        {
            QTextStream ob(&outfile);
            //ob.setCodec("UTF-8");//"ISO 8859-1"
            //ob.setCodec("ISO 8859-1");
            ob << ob_out;
        }
        outfile.close();
    }
}
/*bool readBalance(QString filename)//res_hc
{
    bool readerror = 0;
    /*QFile infile(filename);
    infile.open( QIODevice::ReadOnly | QIODevice::Text );
    inb = new QTextStream(&infile);
    QString ss, readout = "balancing:\n";
    //
    if( find_float(inb,"%T_tr="    , balancefile.delta_Ttr)) { readerror=true; ui->memout->appendPlainText("%T_tr   not found"); }//" Н\tпотребная тяга РВ, Н\n";
    if( find_float(inb,"%Xa_need=" , balancefile.delta_Xa )) { readerror=true; ui->memout->appendPlainText("%Xa_need not found");}//" погрешность по Xa_need НВ\n";
    if( find_float(inb,"%Ya_need=" , balancefile.delta_Ya )) { readerror=true; ui->memout->appendPlainText("%Ya_need not found");}//" погрешность по Ya_need НВ\n";
    if( find_float(inb,"Xa_need="  , balancefile.Xa_need  )) { readerror=true; ui->memout->appendPlainText("Xa_need not found"); }//" Н\tпотребная пропульсивная сила НВ\n";
    if( find_float(inb,"Ya_need="  , balancefile.Ya_need  )) { readerror=true; ui->memout->appendPlainText("Ya_need not found"); }//" Н\tпотребная подъемная сила НВ\n";
    if( find_float(inb,"chi="      , balancefile.chi      )) { readerror=true; ui->memout->appendPlainText("chi     not found"); }//" градус\tугол отклонения тарелки АП НВ по продольному каналу (=chi)\n";
    if( find_float(inb,"eta="      , balancefile.eta      )) { readerror=true; ui->memout->appendPlainText("eta     not found"); }//" градус\tугол отклонения тарелки АП НВ по поперечному каналу\n";
    if( find_float(inb,"T_tr="     , balancefile.T_tr     )) { readerror=true; ui->memout->appendPlainText("T_tr    not found"); }//" Н\tпотребная тяга РВ, Н\n";

    //readout += "    %Xa_need= " + ss.setNum(balancefile.delta_Xa ,'f',3) + " %\tпогрешность по Xa_need\n";
    //readout += "    %Ya_need= " + ss.setNum(balancefile.delta_Ya ,'f',3) + " %\tпогрешность по Ya_need\n";
    readout += "    Xa=       " + ss.setNum(balancefile.Xa_need  ,'f',1) + " Н\tпотребная пропульсивная сила НВ\n";
    readout += "    Ya=       " + ss.setNum(balancefile.Ya_need  ,'f',1) + " Н\tпотребная подъемная сила НВ\n";
    readout += "    chi=      " + ss.setNum(balancefile.chi      ,'f',4) + " градус\tугол отклонения тарелки АП НВ по продольному каналу\n";
    readout += "    eta=      " + ss.setNum(balancefile.eta      ,'f',4) + " градус\tугол отклонения тарелки АП НВ по поперечному каналу\n";
    readout += "    T_tr=     " + ss.setNum(balancefile.T_tr     ,'f',1) + " Н\tпотребная тяга РВ, Н\n";
    readout += "    %T_tr=    " + ss.setNum(balancefile.delta_Ttr,'f',3) + " %\tпогрешность по T_tr";//ВНИМАНИЕ! без \n

    ui->memout->appendPlainText( readout );
    delete inb;
    infile.close();
return readerror;
}*/
/*void GetBalanceData()// получение результатов расчета
{
    /*
    //получение1 нулевых приближений потребных сил и межосевого расстояния
    L =  balancingSolver->GetValue("L_out");
    Xa_need.push_back( balancingSolver->GetValue("X0_need_out") );
    Ya_need.push_back( balancingSolver->GetValue("Y0_need_out") );
    eta.push_back(0);
    chi.push_back(0);

    //получение2 приближений потребных сил
    Xa_need.push_back( balancingSolver->GetValue("Xa_need_out") );
    Ya_need.push_back( balancingSolver->GetValue("Ya_need_out") );
    chi.push_back( balancingSolver->GetValue("chi_out") );
    eta.push_back( balancingSolver->GetValue("eta_out") );
    Ttr_need.push_back( balancingSolver->GetValue("Ttr_need_out") );
    alpha_f = balancingSolver->GetValue("alfa_f_out");
    delta_alpha_f_mr = balancingSolver->GetValue("delta_alfa_f_mr_out");
    X_f = balancingSolver->GetValue("X_f_out");
    Y_f = balancingSolver->GetValue("Y_f_out");
    Z_f = balancingSolver->GetValue("Z_f_out");
    Mx_f = balancingSolver->GetValue("Mx_f_out");
    My_f = balancingSolver->GetValue("My_f_out");
    Mz_f = balancingSolver->GetValue("Mz_f_out");
    k_q_f = balancingSolver->GetValue("k_q_f_out");
    alpha_st = balancingSolver->GetValue("alfa_st_out");
    delta_alpha_st_mr = balancingSolver->GetValue("delta_alfa_st_mr_out");
    fi_st = balancingSolver->GetValue("fi_st_out");
    Y_st = balancingSolver->GetValue("Y_st_out");
    k_q_st = balancingSolver->GetValue("k_q_st_out");

    theta_h = balancingSolver->GetValue("teta_h_out");
    gamma_h = balancingSolver->GetValue("gamma_h_out");


}*/



// Р А С Ч Е Т  В И Н Т О В
void MainRotorCalc()//организатор расчета аэродинамики винта
{
    /*запуск внешних модулей
    QTime start = QTime::currentTime();
    tm.start();
    process_glrotor = new QProcess(this);//рождение процессов для сценария "Аэродинамика винта"
    switch ( obj_calc )
    {
        case 0://MR
            if(!runRotors(process_glrotor,arg_mr)) readMRotor(res_glmr);//считывание из файла res_glmr
            else ui->memout->appendPlainText( "runRotors(mr) error" );
        break;
        case 1://TR
            if(!runRotors(process_glrotor,arg_tr)) readTRotor(res_gltr);//считывание из файла res_gltr
            else ui->memout->appendPlainText( "runRotors(tr) error" );
        break;
    }
    QTime finish = QTime::currentTime();//время расчета
    ui->progress->setValue( 0 );
    ui->memout->appendPlainText("\n----------------------------------------------------------------------------------------------------------------------");
    ui->memout->appendPlainText("steps: "    + ss.setNum(i)      );
    ui->memout->appendPlainText("started:  " + start.toString()  );
    ui->memout->appendPlainText("finished: " + finish.toString() );
    ui->memout->appendPlainText("elapsed: "  + getElapsedTime(tm));
    //удаление процессов из памяти
    delete process_glrotor;*/

    toMemo("MainRotors() в процессе разработки");

}
/*bool readMRotor(QString filename)//res_glmr res_gltr
{
    bool readerror = 0;
    QFile infile(filename);
    infile.open( QIODevice::ReadOnly | QIODevice::Text );
    inmr = new QTextStream(&infile);
    QString ss, readout = "mainrotor:\n";
    //MR
    if( find_float(inmr,"chi_mr="   , mrotorfile.chi    )) { readerror=true; ui->memout->appendPlainText("chi_mr  not found"); }//" градус\tугол отклонения тарелки АП НВ по продольному каналу (=chi)\n";
    if( find_float(inmr,"eta_mr="   , mrotorfile.eta    )) { readerror=true; ui->memout->appendPlainText("eta_mr  not found"); }//" градус\tугол отклонения тарелки АП НВ по поперечному каналу\n";
    if( find_float(inmr,"alfa_mr="  , mrotorfile.alfa   )) { readerror=true; ui->memout->appendPlainText("alfa_mr not found"); }//" градус\tугол атаки НВ\n";
    if( find_float(inmr,"fi0_mr="   , mrotorfile.fi0    )) { readerror=true; ui->memout->appendPlainText("fi0_mr  not found"); }//" градус\tОбщий шаг НВ\n";
    if( find_float(inmr,"T_mr_="    , mrotorfile.T      )) { readerror=true; ui->memout->appendPlainText("T_mr    not found"); }//" Н\tпотребная тяга РВ, Н\n";
    if( find_float(inmr,"H_mr_="    , mrotorfile.H      )) { readerror=true; ui->memout->appendPlainText("H_mr    not found"); }//
    if( find_float(inmr,"S_mr_="    , mrotorfile.S      )) { readerror=true; ui->memout->appendPlainText("S_mr    not found"); }//
    if( find_float(inmr,"Mx_mr_="   , mrotorfile.Mx     )) { readerror=true; ui->memout->appendPlainText("Mx_mr   not found"); }//
    if( find_float(inmr,"My_mr_="   , mrotorfile.My     )) { readerror=true; ui->memout->appendPlainText("My_mr   not found"); }//
    if( find_float(inmr,"Mz_mr_="   , mrotorfile.Mz     )) { readerror=true; ui->memout->appendPlainText("Mz_mr   not found"); }//

    //M_kr
    //L
    readout += "    chi_mr=   " + ss.setNum(mrotorfile.chi       ,'f',4) + " градус\tугол отклонения тарелки АП НВ по продольному каналу\n";
    readout += "    eta_mr=   " + ss.setNum(mrotorfile.eta       ,'f',4) + " градус\tугол отклонения тарелки АП НВ по поперечному каналу\n";
    readout += "    alfa_mr=  " + ss.setNum(mrotorfile.alfa      ,'f',4) + " градус\tугол атаки НВ\n";
    readout += "    fi0_mr=   " + ss.setNum(mrotorfile.fi0       ,'f',4) + " градус\tобщий шаг НВ\n";
    readout += "    T_mr_=    " + ss.setNum(mrotorfile.T         ,'f',1) + " Н  \t\n";
    readout += "    H_mr_=    " + ss.setNum(mrotorfile.H         ,'f',1) + " Н  \t\n";
    readout += "    S_mr_=    " + ss.setNum(mrotorfile.S         ,'f',1) + " Н  \t\n";
    readout += "    Mx_mr_=   " + ss.setNum(mrotorfile.Mx        ,'f',1) + " Н*м\t\n";
    readout += "    My_mr_=   " + ss.setNum(mrotorfile.My        ,'f',1) + " Н*м\t\n";
    readout += "    Mz_mr_=   " + ss.setNum(mrotorfile.Mz        ,'f',1) + " Н*м\t\n";

    ui->memout->appendPlainText( readout );
    delete inmr;
    infile.close();
return readerror;
}
bool readTRotor(QString filename)//res_glmr res_gltr
{
    bool readerror = 0;
    QFile infile(filename);
    infile.open( QIODevice::ReadOnly | QIODevice::Text );
    intr = new QTextStream(&infile);
    QString ss, readout = "tailrotor:\n";
    //TR
    if( find_float(intr,"fi0_tr="   , trotorfile.fi0   )) { readerror=true; ui->memout->appendPlainText("fi0_tr  not found"); }//" градус\tОбщий шаг РВ\n";
    if( find_float(intr,"T_tr_="    , trotorfile.T     )) { readerror=true; ui->memout->appendPlainText("T_tr    not found"); }//" Н\tпотребная тяга РВ, Н\n";
    if( find_float(intr,"H_tr_="    , trotorfile.H     )) { readerror=true; ui->memout->appendPlainText("H_tr    not found"); }//
    if( find_float(intr,"S_tr_="    , trotorfile.S     )) { readerror=true; ui->memout->appendPlainText("S_tr    not found"); }//
    if( find_float(intr,"Mx_tr_="   , trotorfile.Mx    )) { readerror=true; ui->memout->appendPlainText("Mx_tr   not found"); }//
    if( find_float(intr,"My_tr_="   , trotorfile.My    )) { readerror=true; ui->memout->appendPlainText("My_tr   not found"); }//
    if( find_float(intr,"Mz_tr_="   , trotorfile.Mz    )) { readerror=true; ui->memout->appendPlainText("Mz_tr   not found"); }//

    readout += "    fi0_tr=    " + ss.setNum(trotorfile.fi0       ,'f',4) + " градус\tобщий шаг РВ\n";
    readout += "    T_tr_=     " + ss.setNum(trotorfile.T         ,'f',1) + " Н  \tтяга РВ\n";
    readout += "    H_tr_=     " + ss.setNum(trotorfile.H         ,'f',1) + " Н  \t\n";
    readout += "    S_tr_=     " + ss.setNum(trotorfile.S         ,'f',1) + " Н  \t\n";
    readout += "    Mx_tr_=    " + ss.setNum(trotorfile.Mx        ,'f',1) + " Н*м\t\n";
    readout += "    My_tr_=    " + ss.setNum(trotorfile.My        ,'f',1) + " Н*м\t\n";
    readout += "    Mz_tr_=    " + ss.setNum(trotorfile.Mz        ,'f',1) + " Н*м\t\n";
    ui->memout->appendPlainText( readout );
    delete intr;
    infile.close();
return readerror;
}*/



//считывание исх.данных
/*int read_ini_(QString infileName)//сокращенное считывание из файла
{
    bool readerror_in = 0;
    QFile inifile(infileName);
    if(inifile.open( QIODevice::ReadOnly | QIODevice::Text ))
    {
        in = new QTextStream(&inifile);
        if(find_str(  in,"frm.cntr:" ,fname_cntr      )) {                    toMemo("frm.cntr not found"); }
        if(find_int(  in,"start="    ,ini.start       )) { readerror_in=true; toMemo("start    not found"); }
        if(find_int(  in,"steps="    ,ini.steps       )) { readerror_in=true; toMemo("steps    not found"); }
                          ini.steps +=ini.start;
        if(find_int(  in,"logstep="  ,ini.logstep     )) { readerror_in=true; toMemo("logstep  not found"); }
        //корректировка контура планера frm.cntr
        if(find_int(  in,"frmdir.x=" ,frmpos.xdir     )) { readerror_in=true; toMemo("frmdir.x not found"); }
        if(find_int(  in,"frmdir.y=" ,frmpos.ydir     )) { readerror_in=true; toMemo("frmdir.y not found"); }
        if(find_int(  in,"frmdir.z=" ,frmpos.zdir     )) { readerror_in=true; toMemo("frmdir.z not found"); }
        if(find_float(in,"frmpos.x=" ,frmpos.xbias    )) { readerror_in=true; toMemo("frmpos.x not found"); }
        if(find_float(in,"frmpos.y=" ,frmpos.ybias    )) { readerror_in=true; toMemo("frmpos.y not found"); }
        if(find_float(in,"frmpos.z=" ,frmpos.zbias    )) { readerror_in=true; toMemo("frmpos.z not found"); }
        if(find_float(in,"g="        ,ini.g           )) { readerror_in=true; toMemo("g        not found"); }

        delete in;
        inifile.close();
    }
    else
    {
        toMemo("cannot open " + infileName);
        toLogf("cannot open " + infileName);
    }
    if(readerror_in)
        return 0;
    else
        return 1;//ini.calcmode;//тип расчета
}*/
int  read_ini(QString infileName)//считывание данных из файла
{
    bool readerror_in = 0;
    QFile inifile(infileName);
    if(inifile.open( QIODevice::ReadOnly | QIODevice::Text ))
    {
        in = new QTextStream(&inifile);
        //организация файлов
        if(find_str(  in,"file.frm:" ,fname_frm       )) { readerror_in=true; toMemo("file.frm: not found"); }//(конструктивные параметры фюзеляжа и шасси)
        if(find_str(  in,"file.fax:" ,fname_fax       )) { readerror_in=true; toMemo("file.fax: not found"); }//(аэродинамические характеристики фюзеляжа)
        if(find_str(  in,"file.mr:"  ,fname_mr        )) { readerror_in=true; toMemo("file.mr:  not found"); }//(конструктивные и аэродинамические характеристики винта несущего)
        if(find_str(  in,"file.tr:"  ,fname_tr        )) { readerror_in=true; toMemo("file.tr:  not found"); }//(конструктивные и аэродинамические характеристики винта рулевого)
        if(find_str(  in,"file.mrr:" ,fname_mrr       )) { readerror_in=true; toMemo("file.mrr: not found"); }//(результаты расчета винта несущего)
        if(find_str(  in,"file.trr:" ,fname_trr       )) { readerror_in=true; toMemo("file.trr: not found"); }//(результаты расчета винта рулевого)
        if(find_str(  in,"file.bal:" ,fname_bal       )) { readerror_in=true; toMemo("file.bal: not found"); }//(результаты расчета балансировки вертолета)
        if(find_str(  in,"extload1:" ,fname_load1     )) {                    toMemo("extload1: not found"); }//
        if(find_str(  in,"extload2:" ,fname_load2     )) {                    toMemo("extload2: not found"); }//
        //if(find_str(  in,"frm.cntr:" ,fname_cntr      )) {                    toMemo("frm.cntr: not found"); }//(контур фюзеляжа)readerror_in=true;
        //управляющие ключи
        if(find_int(  in,"calcmode=" ,ini.calcmode    )) { readerror_in=true; toMemo("calcmode not found"); }//tip_calc
        if(find_int(  in,"trsolve="  ,ini.trsolve     )) { readerror_in=true; toMemo("trsolve  not found"); }
        if(find_int(  in,"mrsolve="  ,ini.mrsolve     )) { readerror_in=true; toMemo("mrsolve  not found"); }
        if(find_int(  in,"vgsolve="  ,ini.vgsolve     )) { readerror_in=true; toMemo("vgsolve  not found"); }
        if(find_int(  in,"airdyn="   ,ini.applyairdyn )) { readerror_in=true; toMemo("airdyn   not found"); }
        if(find_int(  in,"extloads=" ,ini.extloads    )) { readerror_in=true; toMemo("extloads not found"); }
        if(find_int(  in,"balance="  ,ini.makebalance )) { readerror_in=true; toMemo("balance  not found"); }
        if(ini.makebalance==0)
        {
            if(find_float(in,"dchi=" ,ini.dkapa       )) { readerror_in=true; toMemo("dchi     not found"); }
            if(find_float(in,"deta=" ,ini.deta        )) { readerror_in=true; toMemo("deta     not found"); }
            if(find_float(in,"kren=" ,ini.kren        )) { readerror_in=true; toMemo("kren     not found"); }
            if(find_float(in,"T_tr=" ,ini.tr.T_       )) { readerror_in=true; toMemo("T_tr     not found"); }
        }
        //НАСТРОЙКИ РАСЧЕТА
        if(find_int(  in,"start="    ,ini.start       )) { readerror_in=true; toMemo("start    not found"); }
        if(find_int(  in,"steps="    ,ini.steps       )) { readerror_in=true; toMemo("steps    not found"); }
                          ini.steps +=ini.start;
        if(find_float(in,"dt="       ,ini.dt          )) { readerror_in=true; toMemo("dt       not found"); }
        if(find_int(  in,"logstep="  ,ini.logstep     )) { readerror_in=true; toMemo("logstep  not found"); }
        if(find_int(  in,"logkey="   ,ini.logkey      )) { readerror_in=true; toMemo("logkey   not found"); }
        if(find_float(in,"kra="      ,ini.kra         )) { readerror_in=true; toMemo("kra      not found"); }
        if(find_float(in,"krp="      ,ini.krp         )) { readerror_in=true; toMemo("krp      not found"); }
        //if(find_float(in,"bpa="      ,ini.bpa         )) { readerror_in=true; toMemo("bpa      not found"); }
        if(find_float(in,"snlf6prc=" ,ini.snlf6prc    )) { readerror_in=true; toMemo("snlf6prc not found"); }
        if(find_float(in,"snlf2prc=" ,ini.snlf2prc    )) { readerror_in=true; toMemo("snlf2prc not found"); }
        ini.snlf6lim = 100000;//if(find_float(in,"snlf6lim=" ,ini.snlf6lim    )) { readerror_in=true; toMemo("snlf6lim not found"); }
        ini.snlf2lim = 100000;//if(find_float(in,"snlf2lim=" ,ini.snlf2lim    )) { readerror_in=true; toMemo("snlf2lim not found"); }
        //ПАРАМЕТРЫ ПОЛЕТА начальные условия
        if(find_float(in,"Hx="       ,ini.Hx          )) { readerror_in=true; toMemo("Hx       not found"); }
        if(find_float(in,"Hy="       ,ini.Hy          )) { readerror_in=true; toMemo("Hy       not found"); }
        if(find_float(in,"Hz="       ,ini.Hz          )) { readerror_in=true; toMemo("Hz       not found"); }
        if(find_float(in,"kren_="    ,ini.kren_       )) { readerror_in=true; toMemo("kren_    not found"); }
        if(find_float(in,"rysk_="    ,ini.rysk_       )) { readerror_in=true; toMemo("rysk_    not found"); }
        if(find_float(in,"tang_="    ,ini.tang_       )) { readerror_in=true; toMemo("tang_    not found"); }
        if(find_float(in,"vHx="      ,ini.vHx         )) { readerror_in=true; toMemo("vHx      not found"); }
        if(find_float(in,"vHy="      ,ini.vHy         )) { readerror_in=true; toMemo("vHy      not found"); }
        if(find_float(in,"vHz="      ,ini.vHz         )) { readerror_in=true; toMemo("vHz      not found"); }
        if(find_float(in,"wHx="      ,ini.wHx         )) { readerror_in=true; toMemo("wHx      not found"); }
        if(find_float(in,"wHy="      ,ini.wHy         )) { readerror_in=true; toMemo("wHy      not found"); }
        if(find_float(in,"wHz="      ,ini.wHz         )) { readerror_in=true; toMemo("wHz      not found"); }
        if(find_float(in,"tetra="    ,ini.tetra       )) { readerror_in=true; toMemo("tetra    not found"); }//угол наклона траектории, рад
        if(find_float(in,"beta_="    ,ini.beta_       )) { readerror_in=true; toMemo("beta_    not found"); }//Угол скольжения, рад
        if(find_float(in,"g="        ,ini.g           )) { readerror_in=true; toMemo("g        not found"); }
        if(find_float(in,"t="        ,ini.t0          )) { readerror_in=true; toMemo("t        not found"); }
        if(find_float(in,"P/p="      ,ini.P_p         )) { readerror_in=true; toMemo("P/p      not found"); }
        //ВНЕШНЕЕ ВОЗМУЩЕНИЕ ВАЛА НВ
        if(find_float(in,"hit.My0="  ,ini.M0          )) { readerror_in=true; toMemo("hit.My0  not found"); }
        if(find_float(in,"hit.Fz0="  ,ini.F0          )) { readerror_in=true; toMemo("hit.Fz0  not found"); }
        if(find_float(in,"hit.niu="  ,ini.niu         )) { readerror_in=true; toMemo("hit.niu  not found"); }
        if(find_float(in,"hit.t0="   ,ini.t1          )) { readerror_in=true; toMemo("hit.t0   not found"); }
        if(find_float(in,"hit.t1="   ,ini.t2          )) { readerror_in=true; toMemo("hit.t1   not found"); }
        //ограничение перемещений
        if(find_int(  in,"fix_x="    ,ini.fix_orient_x)) { readerror_in=true; toMemo("fix_x    not found"); }
        if(find_int(  in,"fix_y="    ,ini.fix_orient_y)) { readerror_in=true; toMemo("fix_y    not found"); }
        if(find_int(  in,"fix_z="    ,ini.fix_orient_z)) { readerror_in=true; toMemo("fix_z    not found"); }
        if(find_int(  in,"fix_k="    ,ini.fix_orient_k)) { readerror_in=true; toMemo("fix_k    not found"); }
        if(find_int(  in,"fix_r="    ,ini.fix_orient_r)) { readerror_in=true; toMemo("fix_r    not found"); }
        if(find_int(  in,"fix_t="    ,ini.fix_orient_t)) { readerror_in=true; toMemo("fix_t    not found"); }
        //корректировка контура планера frm.cntr
        //if(find_int(  in,"frmdir.x=" ,frmpos.xdir     )) { readerror_in=true; toMemo("frmdir.x not found"); }
        //if(find_int(  in,"frmdir.y=" ,frmpos.ydir     )) { readerror_in=true; toMemo("frmdir.y not found"); }
        //if(find_int(  in,"frmdir.z=" ,frmpos.zdir     )) { readerror_in=true; toMemo("frmdir.z not found"); }
        //if(find_float(in,"frmpos.x=" ,frmpos.xbias    )) { readerror_in=true; toMemo("frmpos.x not found"); }
        //if(find_float(in,"frmpos.y=" ,frmpos.ybias    )) { readerror_in=true; toMemo("frmpos.y not found"); }
        //if(find_float(in,"frmpos.z=" ,frmpos.zbias    )) { readerror_in=true; toMemo("frmpos.z not found"); }

        if(ini.P_p>10)
             { ini.p0 = ini.P_p;  ini.ro = 0; }
        else { ini.ro = ini.P_p;  ini.p0 = 0; }
        ini.Lpt   = 0.0065;//К/м        //if(find_float(in,"Lpt="   ,rdata.Lpt  )) { readerror=true; foundout+="Lpt   not found\n"; }//rd.Lpt  = 0.0065; //скорость падения температуры К/м
        ini.RjmK  = 8.31447;//Дж/(мол*К)//if(find_float(in,"RjmK="  ,rdata.RjmK )) { readerror=true; foundout+="RjmK  not found\n"; }//rd.RjmK = 8.31447;//газовая постоянная Дж/(мол*К)
        ini.RjkgK = 287.0;//Дж/(кг*К)   //if(find_float(in,"RjkgK=" ,rdata.RjkgK)) { readerror=true; foundout+="RjkgK not found\n"; }//rd.RjkgK= 287.0;  //газовая постоянная Дж/(кг*К)
        ini.Mm    = 0.0289644;//кг/моль //if(find_float(in,"Mm="    ,rdata.Mm   )) { readerror=true; foundout+="Mm    not found\n"; }//rd.Mm   = 0.0289644; //молярная масса сухого воздуха кг/мол
        pi    = M_PI;
        d2r   = M_PI/180.0;
        r2d   = 180.0/M_PI;

        delete in;
        inifile.close();
    }
    else
    {
        toMemo("cannot open " + infileName);
        toLogf("cannot open " + infileName);
    }
    if(readerror_in)
        return 0;
    else
        return ini.calcmode;//тип расчета
}
bool read_frm(QString infileName)//считывание данных из файла
{
    bool readerror_frm = 0;
    QFile infile(infileName);
    if(infile.open( QIODevice::ReadOnly | QIODevice::Text )) //if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
    {
        in_frm = new QTextStream(&infile);
        //ПАРАМЕТРЫ ВЕРТОЛЕТА
        if(find_float(in_frm,"mH="       ,ini.mH       )) { readerror_frm=true; toMemo("mH      not found"); }
        if(find_float(in_frm,"xctmr="    ,ini.xctmr    )) { readerror_frm=true; toMemo("xc      not found"); }//xTmr Горизонтальная центровка вертолета в СК НВ, м     xTH = xt_in_geo;
        if(find_float(in_frm,"yctmr="    ,ini.yctmr    )) { readerror_frm=true; toMemo("yc      not found"); }//yTmr Вертикальная центровка вертолета в СК НВ, м       yTH = yt_in_geo;
        if(find_float(in_frm,"zctmr="    ,ini.zctmr    )) { readerror_frm=true; toMemo("zc      not found"); }//zTmr центровка вертолета вдоль Oz, м                   zT = zt_in_geo;
        if(find_float(in_frm,"eps_f="    ,ini.eps_f    )) { readerror_frm=true; toMemo("eps_f   not found"); }//Угол заклинения фюзеляжа, рад
        if(find_float(in_frm,"l_mr="     ,ini.mr.l     )) { readerror_frm=true; toMemo("l_mr    not found"); }//расстояние по оси НВ от КПНВ до строительной горизонтали, мм;
        if(find_float(in_frm,"x_tr="     ,ini.tr.Ox    )) { readerror_frm=true; toMemo("x_tr    not found"); }//=67456812;//Расстояние от оси несущего винта до оси рулевого винта, м
        if(find_float(in_frm,"y_tr="     ,ini.tr.Oy    )) { readerror_frm=true; toMemo("y_tr    not found"); }//=6711039;//Расстояние от оси рулевого винта до Oxyz. м
        if(find_float(in_frm,"z_tr="     ,ini.tr.Oz    )) { readerror_frm=true; toMemo("z_tr    not found"); }//=5;//Расстояние от начала координат до плоскости вращения рулевого винта по оси Oz. м
        if(find_float(in_frm,"x_f="      ,ini.x_f      )) { readerror_frm=true; toMemo("x_f     not found"); }//=Характерная точка для определения скоса на фюзеляже от несущего винта по оси Ох, м
        if(find_float(in_frm,"y_f="      ,ini.y_f      )) { readerror_frm=true; toMemo("y_f     not found"); }//=Характерная точка для определения скоса на фюзеляже от несущего винта по оси Оу, м
        if(find_float(in_frm,"z_f="      ,ini.z_f      )) { readerror_frm=true; toMemo("z_f     not found"); }//=Характерная точка для определения скоса на фюзеляже от несущего винта по оси Оz, м
        if(find_float(in_frm,"Ix="       ,ini.Ix       )) { readerror_frm=true; toMemo("Ix      not found"); }
        if(find_float(in_frm,"Iy="       ,ini.Iy       )) { readerror_frm=true; toMemo("Iy      not found"); }
        if(find_float(in_frm,"Iz="       ,ini.Iz       )) { readerror_frm=true; toMemo("Iz      not found"); }
        if(find_float(in_frm,"Iyz="      ,ini.Iyz      )) { readerror_frm=true; toMemo("Iyz     not found"); }
        if(find_float(in_frm,"Ixz="      ,ini.Ixz      )) { readerror_frm=true; toMemo("Ixz     not found"); }
        if(find_float(in_frm,"Ixy="      ,ini.Ixy      )) { readerror_frm=true; toMemo("Ixy     not found"); }
         //СТАБИЛИЗАТОР
        if(  find_int(in_frm,"loc="      ,ini.loc      )) { readerror_frm=true; toMemo("loc     not found"); }//закон управления стабилизатором, [1,2,3,4,5]
        if(find_float(in_frm,"x_st="     ,ini.x_st     )) { readerror_frm=true; toMemo("x_st    not found"); }//=25572483;//Координаты стабилизатора в по Ox CK Oxyz, м
        if(find_float(in_frm,"y_st="     ,ini.y_st     )) { readerror_frm=true; toMemo("y_st    not found"); }//=18215118;//Координаты стабилизатора в по Оу CK Oxyz, м
        if(find_float(in_frm,"z_st="     ,ini.z_st     )) { readerror_frm=true; toMemo("z_st    not found"); }//=Координаты стабилизатора в по Оz CK Oxyz, м
        if(find_float(in_frm,"fi0_min="  ,ini.fi0_min  )) { readerror_frm=true; toMemo("fi0_min not found"); }//УПРАВЛЕНИЯ СТАБИЛИЗАТОРОМ
        if(find_float(in_frm,"fi0_max="  ,ini.fi0_max  )) { readerror_frm=true; toMemo("fi0_max not found"); }//УПРАВЛЕНИЯ СТАБИЛИЗАТОРОМ
        if(find_float(in_frm,"est_min="  ,ini.est_min  )) { readerror_frm=true; toMemo("est_min not found"); }//УПРАВЛЕНИЯ СТАБИЛИЗАТОРОМ
        if(find_float(in_frm,"est_max="  ,ini.est_max  )) { readerror_frm=true; toMemo("est_max not found"); }//УПРАВЛЕНИЯ СТАБИЛИЗАТОРОМ
        //ГРУЗ НА ВНЕШНЕЙ ТРОСОВОЙ ПОДВЕСКЕ
        if(find_float(in_frm,"mcg="      ,ini.mcg      )) { readerror_frm=true; toMemo("mcg     not found"); }
        if(find_float(in_frm,"lcg="      ,ini.lcg      )) { readerror_frm=true; toMemo("lcg     not found"); }
        if(find_float(in_frm,"xcg="      ,ini.xcg      )) { readerror_frm=true; toMemo("xcg     not found"); }
        if(find_float(in_frm,"ycg="      ,ini.ycg      )) { readerror_frm=true; toMemo("ycg     not found"); }
        if(find_float(in_frm,"zcg="      ,ini.zcg      )) { readerror_frm=true; toMemo("zcg     not found"); }
        //ШАССИ
        if(read_sts(0,".")) readerror_frm=true; //fore - передние стойки
        if(read_sts(1,":")) readerror_frm=true; //main - основные стойки
        ini.st[2].nst = 0;//if( read_st(2,"|") ) readerror=true; //tail - хвостовая стойка
        //ПОСАДОЧНАЯ ПОВЕРХНОСТЬ
        int n_st = ini.st[0].nst + ini.st[1].nst + ini.st[2].nst;
        for ( int k = 0; k < n_st; k++)
        {
            grndstr = "ground." + ss.setNum(k) + "=";
            if( find_float(in_frm, grndstr, ini.ground[k] )) { readerror_frm=true; toMemo(grndstr + " not found"); }
        }

        //in_frm->seek(0);
        //toLogf(singleline);
        //toLogf("file.frm: " + infileName);
        //if(ini.logkey) toLogf(in_frm->readAll());
        //delete &in_frm;
        infile.close();
    }
    else
    {
        readerror_frm=true;
        //toMemo("cannot open " + infileName);
        toLogf("cannot open " + infileName);
    }
return readerror_frm;
}
bool read_sts(int k, QString prefix)//считывание данных шасси
{
    bool readerror_frm = 0;
    if( find_int(  in_frm, strSumm(prefix,"shtype="  ), ini.st[k].stype     )) { readerror_frm=true; toMemo(prefix + "shtype  not found"); }//констр.исполнение амортизатора ( 1 - однокамерный, 2 - две раздельные газ.камеры, 3 - общая газ.камера на два штока )
    if( find_int(  in_frm, strSumm(prefix,"p.type="  ), ini.st[k].ptype     )) { readerror_frm=true; toMemo(prefix + "p.type  not found"); }//констр.исполнение амортизатора ( 1 - однокамерный, 2 - две раздельные газ.камеры, 3 - общая газ.камера на два штока )
    if( find_int(  in_frm, strSumm(prefix,"a.type="  ), ini.st[k].atype     )) { readerror_frm=true; toMemo(prefix + "a.type  not found"); }//констр.исполнение амортизатора ( 1 - однокамерный, 2 - две раздельные газ.камеры, 3 - общая газ.камера на два штока )
    if( find_int(  in_frm, strSumm(prefix,"nst="     ), ini.st[k].nst       )) { readerror_frm=true; toMemo(prefix + "nst     not found"); }//nfront = FindInt( isFile,".nfront=");//
    if( find_int(  in_frm, strSumm(prefix,"nwh="     ), ini.st[k].nwh       )) { readerror_frm=true; toMemo(prefix + "nwh     not found"); }//fwheel = FindInt( isFile,".nwheel=");//
    if(ini.st[k].nwh==2)
    if( find_float(in_frm, strSumm(prefix,"whd="     ), ini.st[k].whd       )) { readerror_frm=true; toMemo(prefix + "whd     not found"); }//fwheel = FindInt( isFile,".nwheel=");//
    if( find_float(in_frm, strSumm(prefix,"O.x="     ), ini.st[k].O_.x      )) { readerror_frm=true; toMemo(prefix + "O.x     not found"); }
    if( find_float(in_frm, strSumm(prefix,"O.y="     ), ini.st[k].O_.y      )) { readerror_frm=true; toMemo(prefix + "O.y     not found"); }
    if( find_float(in_frm, strSumm(prefix,"O.z="     ), ini.st[k].O_.z      )) { readerror_frm=true; toMemo(prefix + "O.z     not found"); }
    if( find_float(in_frm, strSumm(prefix,"A.x="     ), ini.st[k].A_.x      )) { readerror_frm=true; toMemo(prefix + "A.x     not found"); }
    if( find_float(in_frm, strSumm(prefix,"A.y="     ), ini.st[k].A_.y      )) { readerror_frm=true; toMemo(prefix + "A.y     not found"); }
    if( find_float(in_frm, strSumm(prefix,"A.z="     ), ini.st[k].A_.z      )) { readerror_frm=true; toMemo(prefix + "A.z     not found"); }
    if( find_float(in_frm, strSumm(prefix,"B.x="     ), ini.st[k].B_.x      )) { readerror_frm=true; toMemo(prefix + "B.x     not found"); }
    if( find_float(in_frm, strSumm(prefix,"B.y="     ), ini.st[k].B_.y      )) { readerror_frm=true; toMemo(prefix + "B.y     not found"); }
    if( find_float(in_frm, strSumm(prefix,"B.z="     ), ini.st[k].B_.z      )) { readerror_frm=true; toMemo(prefix + "B.z     not found"); }
    if( find_float(in_frm, strSumm(prefix,"W.x="     ), ini.st[k].W_.x      )) { readerror_frm=true; toMemo(prefix + "W.x     not found"); }
    if( find_float(in_frm, strSumm(prefix,"W.y="     ), ini.st[k].W_.y      )) { readerror_frm=true; toMemo(prefix + "W.y     not found"); }
    if( find_float(in_frm, strSumm(prefix,"W.z="     ), ini.st[k].W_.z      )) { readerror_frm=true; toMemo(prefix + "W.z     not found"); }
    if( find_float(in_frm, strSumm(prefix,"lambda="  ), ini.st[k].lambda    )) { readerror_frm=true; toMemo(prefix + "lambda  not found"); }
    if( find_float(in_frm, strSumm(prefix,"ang_wh="  ), ini.st[k].ang_wh    )) { readerror_frm=true; toMemo(prefix + "ang_wh  not found"); }
  //if( find_float(in_frm, strSumm(prefix,"ground="  ), ini.st[k].ground    )) { readerror_frm=true; toMemo(prefix + "ground  not found"); }
    if( find_float(in_frm, strSumm(prefix,"p.dpmd="  ), ini.st[k].pn.dpmd   )) { readerror_frm=true; toMemo(prefix + "p.dpmd  not found"); }
    if( find_float(in_frm, strSumm(prefix,"p.cpx="   ), ini.st[k].pn.cpx    )) { readerror_frm=true; toMemo(prefix + "p.cpx   not found"); }
    if( find_float(in_frm, strSumm(prefix,"p.cpy="   ), ini.st[k].pn.cpy    )) { readerror_frm=true; toMemo(prefix + "p.cpy   not found"); }
    if( find_float(in_frm, strSumm(prefix,"p.cpz="   ), ini.st[k].pn.cpz    )) { readerror_frm=true; toMemo(prefix + "p.cpz   not found"); }
    if( find_float(in_frm, strSumm(prefix,"p.knp="   ), ini.st[k].pn.alfanp )) { readerror_frm=true; toMemo(prefix + "p.knp   not found"); }
    if( find_float(in_frm, strSumm(prefix,"p.kdp="   ), ini.st[k].pn.kdp    )) { readerror_frm=true; toMemo(prefix + "p.kdp   not found"); }
    if( find_float(in_frm, strSumm(prefix,"p.D="     ), ini.st[k].pn.D      )) { readerror_frm=true; toMemo(prefix + "p.D     not found"); }
    if( find_float(in_frm, strSumm(prefix,"p.B="     ), ini.st[k].pn.B      )) { readerror_frm=true; toMemo(prefix + "p.B     not found"); }
    if( find_int(  in_frm, strSumm(prefix,"p.brake=" ), ini.st[k].pn.isbrake)) { readerror_frm=true; toMemo(prefix + "p.brake not found"); }
    if( find_float(in_frm, strSumm(prefix,"p.f0="    ), ini.st[k].pn.f0     )) { readerror_frm=true; toMemo(prefix + "p.f0    not found"); }
    if( find_float(in_frm, strSumm(prefix,"p.fs="    ), ini.st[k].pn.fs     )) { readerror_frm=true; toMemo(prefix + "p.fs    not found"); }
    if( find_float(in_frm, strSumm(prefix,"p.fk="    ), ini.st[k].pn.fk     )) { readerror_frm=true; toMemo(prefix + "p.fk    not found"); }
    if( find_float(in_frm, strSumm(prefix,"a.Vsum="  ), ini.st[k].ab.Vsum   )) { readerror_frm=true; toMemo(prefix + "a.Vsum  not found"); }
    if( find_float(in_frm, strSumm(prefix,"a.Voil="  ), ini.st[k].ab.Voil   )) { readerror_frm=true; toMemo(prefix + "a.Voil  not found"); }
    if( find_float(in_frm, strSumm(prefix,"a.ro="    ), ini.st[k].ab.ro     )) { readerror_frm=true; toMemo(prefix + "a.ro    not found"); }
    if( find_float(in_frm, strSumm(prefix,"a.mu="    ), ini.st[k].ab.mu     )) { readerror_frm=true; toMemo(prefix + "a.mu    not found"); }
    if( find_float(in_frm, strSumm(prefix,"a.po="    ), ini.st[k].ab.p0     )) { readerror_frm=true; toMemo(prefix + "a.po    not found"); }
    if( find_float(in_frm, strSumm(prefix,"a.kp="    ), ini.st[k].ab.kpoly  )) { readerror_frm=true; toMemo(prefix + "a.k     not found"); }
    if( find_float(in_frm, strSumm(prefix,"a1.S="    ), ini.st[k].ab.c1.s   )) { readerror_frm=true; toMemo(prefix + "a1.S    not found"); }
    if( find_float(in_frm, strSumm(prefix,"a1.D="    ), ini.st[k].ab.c1.D   )) { readerror_frm=true; toMemo(prefix + "a1.D    not found"); }
    if( find_float(in_frm, strSumm(prefix,"a1.Dv="   ), ini.st[k].ab.c1.Dv  )) { readerror_frm=true; toMemo(prefix + "a1.Dv   not found"); }
    if( find_float(in_frm, strSumm(prefix,"a1.Dn="   ), ini.st[k].ab.c1.Dn  )) { readerror_frm=true; toMemo(prefix + "a1.Dn   not found"); }
    if( find_float(in_frm, strSumm(prefix,"a1.fb="   ), ini.st[k].ab.c1.fb  )) { readerror_frm=true; toMemo(prefix + "a1.fb   not found"); }
    if( find_float(in_frm, strSumm(prefix,"a1.f="    ), ini.st[k].ab.c1.f   )) { readerror_frm=true; toMemo(prefix + "a1.f    not found"); }
    if( find_float(in_frm, strSumm(prefix,"a1.fr="   ), ini.st[k].ab.c1.fr  )) { readerror_frm=true; toMemo(prefix + "a1.fr   not found"); }
    if( find_float(in_frm, strSumm(prefix,"a1.ktr="  ), ini.st[k].ab.c1.ktr )) { readerror_frm=true; toMemo(prefix + "a1.ktr  not found"); }
    ini.st[k].ab.c2.s = 0;// !!!
    if(ini.st[k].atype > 1)
    {
    if( find_float(in_frm, strSumm(prefix,"a2.S="    ), ini.st[k].ab.c2.s   )) { readerror_frm=true; toMemo(prefix + "a2.S=   not found"); }
    if( find_float(in_frm, strSumm(prefix,"a2.D="    ), ini.st[k].ab.c2.D   )) { readerror_frm=true; toMemo(prefix + "a2.D=   not found"); }
    if( find_float(in_frm, strSumm(prefix,"a2.f="    ), ini.st[k].ab.c2.f   )) { readerror_frm=true; toMemo(prefix + "a2.f=   not found"); }
    if( find_float(in_frm, strSumm(prefix,"a2.fr="   ), ini.st[k].ab.c2.fr  )) { readerror_frm=true; toMemo(prefix + "a2.fr=  not found"); }
    if( find_float(in_frm, strSumm(prefix,"a2.ktr="  ), ini.st[k].ab.c2.ktr )) { readerror_frm=true; toMemo(prefix + "a2.ktr  not found"); }
    }
return readerror_frm;
}
bool read_fax(QString infileName)//считывание данных из файла
{
    bool readerror_fax = 0;
    QFile infile(infileName);
    if(infile.open( QIODevice::ReadOnly | QIODevice::Text )) //if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
    {
        in_fax = new QTextStream(&infile);

        //ПАРАМЕТРЫ модели                                                   foundout+=                    \n
        if( find_float(in_fax,"lmodel=" ,ini.lmodel )) { readerror_fax=true; toMemo("lmodel not found"); }//rd.lmodel   = 0.76;//Линейный размер на модели, м
        if( find_float(in_fax,"Smodel=" ,ini.Smodel )) { readerror_fax=true; toMemo("Smodel not found"); }//rd.Smodel   = 0.012;//Площадь миделя модели, м2
        if( find_float(in_fax,"Mmodel=" ,ini.Mmodel )) { readerror_fax=true; toMemo("Mmodel not found"); }//rd.Mmodel   = 16.45;//Масштаб модели
        //аэродинамические характеристики стабилизатора
        if( find_Xmass(in_fax,"__alfa_f:",                      ini.__alfa_f, ini.__alfa_f_col                 )) { readerror_fax=true; toMemo("__alfa_f: not found"); }
        if( find_Xmass(in_fax,"__eps_st:",                      ini.__eps_st, ini.__eps_st_col                 )) { readerror_fax=true; toMemo("__eps_st: not found"); }
        if(find_XYmass(in_fax,"__cx_st=f(_eps_st,_alfa_f):",    ini.__cx_st,  ini.__cx_st_row, ini.__cx_st_col )) { readerror_fax=true; toMemo("__cx_st=f(_eps_st,_alfa_f): not found"); }
        if(find_XYmass(in_fax,"__cy_st=f(_eps_st,_alfa_f):",    ini.__cy_st,  ini.__cy_st_row, ini.__cy_st_col )) { readerror_fax=true; toMemo("__cy_st=f(_eps_st,_alfa_f): not found"); }
        if(find_XYmass(in_fax,"__mz_st=f(_eps_st,_alfa_f):",    ini.__mz_st,  ini.__mz_st_row, ini.__mz_st_col )) { readerror_fax=true; toMemo("__mz_st=f(_eps_st,_alfa_f): not found"); }
        //аэродинамические характеристики планера
        if( find_Xmass(in_fax,"_alfa_f:",                       ini._alfa_f,  ini._alfa_f_col                  )) { readerror_fax=true; toMemo("_alfa_f: not found"); }
        if( find_Xmass(in_fax,"_cx_f=f(_alfa_f):",              ini._cx_f,    ini._cx_f_col                    )) { readerror_fax=true; toMemo("_cx_f=f(_alfa_f): not found"); }
        if( find_Xmass(in_fax,"_cy_f=f(_alfa_f):",              ini._cy_f,    ini._cy_f_col                    )) { readerror_fax=true; toMemo("_cy_f=f(_alfa_f): not found"); }
        if( find_Xmass(in_fax,"_cz_f=f(_alfa_f):",              ini._cz_f,    ini._cz_f_col                    )) { readerror_fax=true; toMemo("_cz_f=f(_alfa_f): not found"); }
        if( find_Xmass(in_fax,"_mx_f=f(_alfa_f):",              ini._mx_f,    ini._mx_f_col                    )) { readerror_fax=true; toMemo("_mx_f=f(_alfa_f): not found"); }
        if( find_Xmass(in_fax,"_my_f=f(_alfa_f):",              ini._my_f,    ini._my_f_col                    )) { readerror_fax=true; toMemo("_my_f=f(_alfa_f): not found"); }
        if( find_Xmass(in_fax,"_mz_f=f(_alfa_f):",              ini._mz_f,    ini._mz_f_col                    )) { readerror_fax=true; toMemo("_mz_f=f(_alfa_f): not found"); }
        if(find_XYmass(in_fax,"_mz_fst=f(_eps_st,_alfa_f):",    ini._mz_fst,  ini._mz_fst_row, ini._mz_fst_col )) { readerror_fax=true; toMemo("_mz_fst=f(_eps_st,_alfa_f): not found"); }
        //индуктивная скорость в зависимости от скорости V_vol и угла delta
        if( find_Xmass(in_fax,"_Vvol_mr:" ,                     ini._Vvol_mr, ini._Vvol_mr_col                 )) { readerror_fax=true; toMemo("_Vvol_mr: not found "); }
        if( find_Xmass(in_fax,"_delta_mr:",                     ini._delta_mr,ini._delta_mr_col                )) { readerror_fax=true; toMemo("_delta_mr: not found"); }
        if(find_XYmass(in_fax,"_uvol_mr=f(_Vvol_mr,_delta_mr):",ini._uvol_mr, ini._uvol_mr_row,ini._uvol_mr_col)) { readerror_fax=true; toMemo("_uvol_mr=f(_Vvol_mr,_delta_mr): not found"); }
        //коэффициент взаимоиндукции
        if( find_Xmass(in_fax,"_h_:",        ini._h_,     ini._h_col    )) { readerror_fax=true; toMemo("_h_: not found"); }
        if( find_Xmass(in_fax,"_Ksi3n_:",    ini._Ksi3n_, ini._Ksi3n_col)) { readerror_fax=true; toMemo("_Ksi3n_: not found"); }

        //in_fax->seek(0);
        //toLogf(singleline);
        //toLogf("file.fax: " + infileName);
        //if(ini.logkey) toLogf(in_fax->readAll());
        //delete &in_ax;
        infile.close();
    }
    else
    {
        readerror_fax=true;
        //toMemo("cannot open " + infileName);
        toLogf("cannot open " + infileName);
    }
return readerror_fax;
}
bool read_mr(QString infileName) //считывание данных из файла
{//НЕСУЩИЙ ВИНТ
    bool readerror_mr = 0;
    QFile infile(infileName);
    if(infile.open( QIODevice::ReadOnly | QIODevice::Text )) //if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
    {
        in_mr = new QTextStream(&infile);

        if(  find_int(in_mr,"z_mr="     , ini.mr.zl       )) { readerror_mr=true; toMemo("z_mr      not found"); }//rd.zl_mr    = 3;//Количество лопастей несущего винта
        if(find_float(in_mr,"w_mr="     , ini.mr.w_mr     )) { readerror_mr=true; toMemo("w_mr      not found"); }//rd.w_mr     = 187/7.25;//угловая скорость вращения НВ при заданом режиме, рад/с. 1/с
        if(find_float(in_mr,"D_mr="     , ini.mr.D        )) { readerror_mr=true; toMemo("D_mr      not found"); }//rd.R_mr     = 7.25;//Радиус несущего винта, м
        if(find_float(in_mr,"Sgsh_mr="  , ini.mr.Sgsh_mr  )) { readerror_mr=true; toMemo("S_g_sh_mr not found"); }//rd.Sgsh_mr  = 197;//Стат. момент инерции лопасти относительно г.ш, N*m
        if(find_float(in_mr,"B_mr="     , ini.mr.B_mr     )) { readerror_mr=true; toMemo("B_mr      not found"); }//rd.B_mr     = 0.97;//Коэф. концевых потерь
        if(find_float(in_mr,"D1="       , ini.mr.D1_mr_   )) { readerror_mr=true; toMemo("D1_mr     not found"); }//rd.D1_mr_   = 1.5646828;
        if(find_float(in_mr,"D2="       , ini.mr.D2_mr_   )) { readerror_mr=true; toMemo("D2_mr     not found"); }//rd.D2_mr_   = -0.0883295;
        if(find_float(in_mr,"kvz="      , ini.mr.k_       )) { readerror_mr=true; toMemo("_k_mr     not found"); }//rd.k_       = 0.4;//Компенсатор взмаха
        if(find_float(in_mr,"Rap="      , ini.mr.Rap      )) { readerror_mr=true; toMemo("Rap       not found"); }//rd.Rap      = 0.265;//радиус крепления тяг, м
        if(find_float(in_mr,"ll="       , ini.mr.ll       )) { readerror_mr=true; toMemo("ll        not found"); }//rd.ll       = 0.157;//
        if(find_float(in_mr,"lgsh="     , ini.mr.lgsh     )) { readerror_mr=true; toMemo("l_g_sh_mr not found"); }//rd.lgsh     = 0.102;//Расстояние от оси несущего винта до оси ГШ, м
        if(find_float(in_mr,"lvsh="     , ini.mr.lvsh     )) { readerror_mr=true; toMemo("lvsh      not found"); }//rd.lvsh     = 0.262;//разнос вертикальных шарниров, м
        if(find_float(in_mr,"dgsh="     , ini.mr.dgsh     )) { readerror_mr=true; toMemo("dgsh      not found"); }//rd.dgsh     = 0.035;//вынос горизонтального шарнира вперед, м
        if(find_float(in_mr,"dpsi_cont=", ini.mr.dpsi_cont)) { readerror_mr=true; toMemo("dpsi_cont not found"); }//rd.dpsi_cont= 25 *pi/180;//угол опережения управления, град
      //if(find_float(in_mr,"alpha_mr=" , ini.mr.alfa_mr__)) { readerror_mr=true; toMemo("alpha_mr  not found"); }//rd.alfa_mr__= -4.214 *pi/180;//Угол между набегающим потоком и КПВ, рад
      //if(find_float(in_mr,"chi="      , ini.mr.kapa_mr_ )) { readerror_mr=true; toMemo("chi       not found"); }//rd.kapa_mr_ = -0.956 *pi/180;//Угол отклонения кольца автомата перекоса в продольном направлении, рад
      //if(find_float(in_mr,"eta="      , ini.mr.eta_mr_  )) { readerror_mr=true; toMemo("eta       not found"); }//rd.eta_mr_  =  0.222 *pi/180;//Угол отклонения кольца автомата перекоса в поперечном направлении, рад
      //if(find_float(in_mr,"fi0_mr="   , ini.mr.fi0_mr_  )) { readerror_mr=true; toMemo("fi0_mr    not found"); }//rd.fi0_mr_  = 10.374 *pi/180;//Угол общего шага, рад
      /*if(find_float(in_mr, "mr.n="    , ini.mr.nMR      )) { readerror_mr=true; toMemo("mr.n    not found"); }
        if(find_float(in_mr, "mr.D="    , ini.mr.DMR      )) { readerror_mr=true; toMemo("mr.D    not found"); }
        if(find_float(in_mr, "mr.Jy="   , ini.mr.Jy       )) { readerror_mr=true; toMemo("mr.Jy   not found"); }
        if(find_float(in_mr, "mr.cy="   , ini.mr.cy       )) { readerror_mr=true; toMemo("mr.cy   not found"); }
      //if(find_float(in_mr, "mvmr="    , ini.mr.mvmr     )) { readerror_mr=true; toMemo("mvmr    not found"); }
        //лопасти                                                                 toMemo
        if( find_int(  in_mr, "mr.nb="   , ini.mr.nb      )) { readerror_mr=true; toMemo("mr.nb   not found"); }
        if( find_float(in_mr, "mr.mb="   , ini.mr.mb      )) { readerror_mr=true; toMemo("mr.mb   not found"); }
        if( find_float(in_mr, "mr.rvs="  , ini.mr.rvs     )) { readerror_mr=true; toMemo("mr.rvs  not found"); }
        if( find_float(in_mr, "mr.svs="  , ini.mr.svs     )) { readerror_mr=true; toMemo("mr.svs  not found"); }
        if( find_float(in_mr, "mr.lct="  , ini.mr.lct     )) { readerror_mr=true; toMemo("mr.lct  not found"); }
        //демпфер                                                                 toMemo
        if( find_float(in_mr, "d.h="     , ini.mr.d.h     )) { readerror_mr=true; toMemo("d.h     not found"); }
        if( find_float(in_mr, "d.x2="    , ini.mr.d.x2    )) { readerror_mr=true; toMemo("d.x2    not found"); }
        if( find_float(in_mr, "d.z2="    , ini.mr.d.z2    )) { readerror_mr=true; toMemo("d.z2    not found"); }
        if( find_float(in_mr, "d.x3="    , ini.mr.d.x3    )) { readerror_mr=true; toMemo("d.x3    not found"); }
        if( find_float(in_mr, "d.z3="    , ini.mr.d.z3    )) { readerror_mr=true; toMemo("d.z3    not found"); }
        if( find_float(in_mr, "d.ro="    , ini.mr.d.ro    )) { readerror_mr=true; toMemo("d.ro    not found"); }
        if( find_float(in_mr, "d.mu="    , ini.mr.d.mu    )) { readerror_mr=true; toMemo("d.mu    not found"); }
        if( find_float(in_mr, "d.f="     , ini.mr.d.f     )) { readerror_mr=true; toMemo("d.f     not found"); }
        if( find_float(in_mr, "d.dv="    , ini.mr.d.dv    )) { readerror_mr=true; toMemo("d.dv    not found"); }
        if( find_float(in_mr, "d.dn="    , ini.mr.d.dn    )) { readerror_mr=true; toMemo("d.dn    not found"); }
        if( find_float(in_mr, "d.pmax="  , ini.mr.d.pmax  )) { readerror_mr=true; toMemo("d.pmax  not found"); }
        if( find_float(in_mr, "d.fimax=" , ini.mr.d.fimax )) { readerror_mr=true; toMemo("d.fimax not found"); }
        if( find_float(in_mr, "d.fimin=" , ini.mr.d.fimin )) { readerror_mr=true; toMemo("d.fimin not found"); }
        //стенд                                                                   toMemo
        if( find_float(in_mr, "s.m="     , ini.mr.s.m     )) { readerror_mr=true; toMemo("s.m     not found"); }
        if( find_float(in_mr, "s.ro="    , ini.mr.s.ro    )) { readerror_mr=true; toMemo("s.ro    not found"); }
        if( find_float(in_mr, "s.mu="    , ini.mr.s.mu    )) { readerror_mr=true; toMemo("s.mu    not found"); }
        if( find_float(in_mr, "s.dvx="   , ini.mr.s.dvx   )) { readerror_mr=true; toMemo("s.dvx   not found"); }
        if( find_float(in_mr, "s.dnx="   , ini.mr.s.dnx   )) { readerror_mr=true; toMemo("s.dnx   not found"); }
        if( find_float(in_mr, "s.fx="    , ini.mr.s.fx    )) { readerror_mr=true; toMemo("s.fx    not found"); }
        if( find_float(in_mr, "s.kx="    , ini.mr.s.kx    )) { readerror_mr=true; toMemo("s.kx    not found"); }
        if( find_float(in_mr, "s.dvz="   , ini.mr.s.dvz   )) { readerror_mr=true; toMemo("s.dvz   not found"); }
        if( find_float(in_mr, "s.dnz="   , ini.mr.s.dnz   )) { readerror_mr=true; toMemo("s.dnz   not found"); }
        if( find_float(in_mr, "s.fz="    , ini.mr.s.fz    )) { readerror_mr=true; toMemo("s.fz    not found"); }
        if( find_float(in_mr, "s.kz="    , ini.mr.s.kz    )) { readerror_mr=true; toMemo("s.kz    not found"); }
        //виброгаситель НВ
        if(ini.vgsolve)
        {
        if( find_int(  in_mr, "vg.n="    , ini.mr.vg.n    )) { readerror_mr=true; toMemo("vg.n   not found"); }
        if( find_float(in_mr, "vg.m="    , ini.mr.vg.m    )) { readerror_mr=true; toMemo("vg.m   not found"); }
        if( find_float(in_mr, "vg.h="    , ini.mr.vg.h    )) { readerror_mr=true; toMemo("vg.h   not found"); }
        if( find_float(in_mr, "vg.d="    , ini.mr.vg.d    )) { readerror_mr=true; toMemo("vg.d   not found"); }
        if( find_float(in_mr, "vg.D="    , ini.mr.vg.D    )) { readerror_mr=true; toMemo("vg.D   not found"); }
        }*/

        //delete &in_mr;
        infile.close();
    }
    else
    {
        readerror_mr=true;
        //toMemo("cannot open " + infileName);
        toLogf("cannot open " + infileName);
    }
return readerror_mr;
}
bool read_tr(QString infileName) //считывание данных из файла
{//РУЛЕВОЙ ВИНТ
    bool readerror_tr = 0;
    QFile infile(infileName);
    if(infile.open( QIODevice::ReadOnly | QIODevice::Text )) //if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
    {
        in_tr = new QTextStream(&infile);

        if(find_float(in_tr,"D_tr="     , ini.tr.D     )) { readerror_tr=true; toMemo("D_tr      not found"); }//7.25 *2   ;//-- несущего винта, м
        if(find_float(in_tr,"T_tr_="    , ini.tr.T_    )) { readerror_tr=true; toMemo("T_tr_     not found"); }//rd.T_tr_    = -944.835;//Тяга рулевого винта, Н

        //delete &in_tr;
        infile.close();
    }
    else
    {
        readerror_tr=true;
        //toMemo("cannot open " + infileName);
        toLogf("cannot open " + infileName);
    }
return readerror_tr;
}
bool read_mrr(QString infileName)//считывание данных из файла
{//НЕСУЩИЙ ВИНТ
    bool readerror_mrr = 0;
    QFile infile(infileName);
    if(infile.open( QIODevice::ReadOnly | QIODevice::Text )) //if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
    {
        in_mrr = new QTextStream(&infile);

        //ИЗ КЛАСИЧЕСКОЙ ТЕОРИИ
        if( find_float(in_mrr,"alfa_mr="  , ini.mr.alfa__) ) { readerror_mrr=true; toMemo("alfa_mr not found"); }//rd.alfa_mr__= -4.214 *pi/180;//Угол между набегающим потоком и КПВ, рад
        if( find_float(in_mrr,"fi0_mr="   , ini.mr.fi0_  ) ) { readerror_mrr=true; toMemo("fi0_mr  not found"); }//rd.fi0_mr_  = 10.374 *pi/180;//Угол общего шага, рад
        if( find_float(in_mrr,"chi_mr="   , ini.mr.kapa_ ) ) { readerror_mrr=true; toMemo("chi_mr  not found"); }//rd.kapa_mr_ = -0.956 *pi/180;//Угол отклонения кольца автомата перекоса в продольном направлении, рад
        if( find_float(in_mrr,"eta_mr="   , ini.mr.eta_  ) ) { readerror_mrr=true; toMemo("eta_mr  not found"); }//rd.eta_mr_  =  0.222 *pi/180;//Угол отклонения кольца автомата перекоса в поперечном направлении, рад
        if( find_float(in_mrr,"mr.D="     , ini.mr.D     ) ) { readerror_mrr=true; toMemo("mr.D    not found"); }//
        if( find_float(in_mrr,"mr.T="     , ini.mr.T_    ) ) { readerror_mrr=true; toMemo("mr.T    not found"); }//rd.T_mr_    = 35527.516;//Тяга несущего винта, Н           [положительное направление - вверх]
        if( find_float(in_mrr,"mr.H="     , ini.mr.H_    ) ) { readerror_mrr=true; toMemo("mr.H    not found"); }//rd.H_mr_    = 1674.038;//Продольная сила несущего винта, Н [положительное направление - назад]
        if( find_float(in_mrr,"mr.S="     , ini.mr.S_    ) ) { readerror_mrr=true; toMemo("mr.S    not found"); }//rd.S_mr_    = -262.093;//Боковая сила несущего винта, Н    [положительное направление - влево]
        if( find_float(in_mrr,"mr.Mx="    , ini.mr.Mx_   ) ) { readerror_mrr=true; toMemo("mr.Mx   not found"); }//rd.Mx_mr_   = 256.192;//Момент крена несущего винта, Нм
        if( find_float(in_mrr,"mr.My="    , ini.mr.My_   ) ) { readerror_mrr=true; toMemo("mr.My   not found"); }//rd.My_mr_   = 9350.742;//Момент кручения, Нм
        if( find_float(in_mrr,"mr.Mz="    , ini.mr.Mz_   ) ) { readerror_mrr=true; toMemo("mr.Mz   not found"); }//rd.Mz_mr_   = -905.401;//Момент тангажа несущего винта, Нм

        //delete &in_glmr;
        infile.close();
    }
    else
    {
        readerror_mrr=true;
        //toMemo("cannot open " + infileName);
        toLogf("cannot open " + infileName);
    }
return readerror_mrr;
}
bool read_trr(QString infileName)//считывание данных из файла
{//РУЛЕВОЙ ВИНТ
    bool readerror_trr = 0;
    QFile infile(infileName);
    if(infile.open( QIODevice::ReadOnly | QIODevice::Text )) //if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
    {
        in_trr = new QTextStream(&infile);

        if( find_float(in_trr,"tr.D="     , ini.tr.D     )) { readerror_trr=true; toMemo("tr.D   not found"); }//
        if( find_float(in_trr,"tr.T="     , ini.tr.T_    )) { readerror_trr=true; toMemo("tr.T   not found"); }
        if( find_float(in_trr,"tr.H="     , ini.tr.H_    )) { readerror_trr=true; toMemo("tr.H   not found"); }
        if( find_float(in_trr,"tr.S="     , ini.tr.S_    )) { readerror_trr=true; toMemo("tr.S   not found"); }
        if( find_float(in_trr,"tr.Mx="    , ini.tr.Mx_   )) { readerror_trr=true; toMemo("tr.Mx  not found"); }
        if( find_float(in_trr,"tr.My="    , ini.tr.My_   )) { readerror_trr=true; toMemo("tr.My  not found"); }
        if( find_float(in_trr,"tr.Mz="    , ini.tr.Mz_   )) { readerror_trr=true; toMemo("tr.Mz  not found"); }

        //delete &in_gltr;
        infile.close();
    }
    else
    {
        readerror_trr=true;
        //toMemo("cannot open " + infileName);
        toLogf("cannot open " + infileName);
    }
return readerror_trr;
}
bool read_load(QString infileName, QVector<extloadstruct> &load, xyz &loadpos)//считывание данных из файла
{//
    bool readerror = 0;
    float posx,posy,posz;
    extloadstruct dload;
    QStringList fields;
    QString line,s;
    QRegExp rx("\\s");
    QFile infileload(infileName);
    load.clear();
    loadpos.x = 0;
    loadpos.y = 0;
    loadpos.z = 0;

    if(ini.extloads)
    {
        if(infileload.open( QIODevice::ReadOnly | QIODevice::Text ))
            in_load = new QTextStream(&infileload);
        else
        {
            readerror=1;
            in_load = 0;
        }
        if(!readerror)
        {
            //считывание точки приложения
            if( find_float(in_load,"extloadnode.x=", posx )) { readerror=true; toMemo("extloadpos.x not found"); }
            if( find_float(in_load,"extloadnode.y=", posy )) { readerror=true; toMemo("extloadpos.y not found"); }
            if( find_float(in_load,"extloadnode.z=", posz )) { readerror=true; toMemo("extloadpos.z not found"); }

            loadpos.x = posx;
            loadpos.y = posy;
            loadpos.z = posz;

            //считывание шапки таблицы
            line = in_load->readLine();
            fields = line.split(rx, QString::SkipEmptyParts);
            if(fields.at(0)=="i_______t,s_____Fx,N____Fy,N____Fz,N")
            {
               while(!in_load->atEnd())
                {
                    s="";
                    line = in_load->readLine();
                    fields = line.split(rx, QString::SkipEmptyParts);
                    if(fields.size()>0)
                    {
                        dload.i = fields.at(0).toInt();     s+="i= "; s+=fields.at(0);
                        dload.t = fields.at(1).toFloat();   s+="t= "; s+=fields.at(1);
                        dload.Fx= fields.at(2).toFloat();   s+="Fx="; s+=fields.at(2);
                        dload.Fy= fields.at(3).toFloat();   s+="Fy="; s+=fields.at(3);
                        dload.Fz= fields.at(4).toFloat();   s+="Fz="; s+=fields.at(4);
                        //dload.Mx= fields.at(5).toFloat(); s+="Mx="; s+=fields.at(5);
                        //dload.My= fields.at(6).toFloat(); s+="My="; s+=fields.at(6);
                        //dload.Mz= fields.at(7).toFloat(); s+="Mz="; s+=fields.at(7);
                        load.push_back(dload);
                    }
                    //toMemo(s);
                }
            }
            else
            {
                readerror=1;
                toMemo("not found string: i_______t,s_____Fx,N____Fy,N____Fz,N");
            }
        }

        //in_load->seek(0);
        //toLogf(singleline);
        //toLogf("file.load: " + infileName);
        //if(ini.logkey) toLogf(in_load->readAll());
        //delete &in_load;
        infileload.close();
    }
return readerror;
}
/*bool read_cntr(QString infileName)//считывание данных из файла
{//
    bool readerror=0;
    xyz node;
    QString line;
    QStringList fields;
    QRegExp rx("\\s");
    QFile infilecntr(infileName);
    infilecntr.open( QIODevice::ReadOnly | QIODevice::Text );
    in_cntr = new QTextStream(&infilecntr);
    frmcntr.clear();
    //считывание данных из файла
    while(!in_cntr->atEnd())
    {
        line = in_cntr->readLine();
        fields = line.split(rx, QString::SkipEmptyParts);
        if((fields.size()>0)&&(fields.at(0)=="vertex"))
        {
            node.x= fields.at(1).toFloat() *frmpos.xdir +frmpos.xbias*0.001;
            node.y= fields.at(2).toFloat() *frmpos.ydir +frmpos.ybias*0.001;
            node.z= fields.at(3).toFloat() *frmpos.zdir +frmpos.zbias*0.001;
            frmcntr.push_back(node);
        }
    }
    infilecntr.close();
    if(frmcntr.size()>1) ui->oglwidget->frmcntr = &frmcntr;
return readerror;
}*/

bool read_helicopter_state(QString filename)
{
    bool readerror=0;
    QString line, eofidentity;
    QStringList fields;
    QRegExp rx("\\s");//пробелы и табуляции
    QFile infile(filename);
    if( infile.open( QIODevice::ReadOnly | QIODevice::Text ) )
    {
        in = new QTextStream(&infile);
        line = in->readLine();//HeliSim.2019
        line = in->readLine();//-------------------------
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.iter=          fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.t=             fields.at(1).toFloat();
        line = in->readLine();//-------------------------
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.orient.x=  fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.orient.y=  fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.orient.z=  fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.orient.k=  fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.orient.r=  fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.orient.t=  fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.speed.x=   fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.speed.y=   fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.speed.z=   fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.speed.k=   fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.speed.r=   fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.speed.t=   fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.v.x=       fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.v.y=       fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.v.z=       fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.w.x=       fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.w.y=       fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.w.z=       fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.a.x=       fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.a.y=       fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.a.z=       fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.e.x=       fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.e.y=       fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.e.z=       fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.n.x=       fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.n.y=       fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.n.z=       fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Fsum.x=    fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Fsum.y=    fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Fsum.z=    fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Msum.x=    fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Msum.y=    fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Msum.z=    fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Fhg.x=     fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Fhg.y=     fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Fhg.z=     fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Mhg.x=     fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Mhg.y=     fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Mhg.z=     fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Fad.x=     fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Fad.y=     fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Fad.z=     fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Mad.x=     fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Mad.y=     fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Mad.z=     fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Fmr.x=     fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Fmr.y=     fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Fmr.z=     fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Mmr.x=     fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Mmr.y=     fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Mmr.z=     fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Ftr.x=     fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Ftr.y=     fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Ftr.z=     fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Mtr.x=     fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Mtr.y=     fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Mtr.z=     fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Fsh.x=     fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Fsh.y=     fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Fsh.z=     fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Msh.x=     fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Msh.y=     fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Msh.z=     fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.xct=       fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.yct=       fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.zct=       fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.cargo.Ox =     fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.cargo.Oy =     fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.cargo.Oz =     fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.cargo.CTx=     fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.cargo.CTy=     fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.cargo.CTz=     fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.cargo.Gx =     fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.cargo.Gy =     fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.cargo.Gz =     fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.F1pos.x=   fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.F1pos.y=   fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.F1pos.z=   fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.F1.x=      fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.F1.y=      fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.F1.z=      fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.F2pos.x=   fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.F2pos.y=   fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.F2pos.z=   fields.at(3).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.F2.x=      fields.at(1).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.F2.y=      fields.at(2).toFloat();
        /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.F2.z=      fields.at(3).toFloat();
        line = in->readLine();//------------------------
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.nst=           fields.at(1).toInt();//
        for (int k = 0; k < out.nst; k++)
        {
            line = in->readLine();//#st=         #
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].nwh         = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].status      = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.status    = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].p.status    = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].Bwh         = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].Dwh         = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].rwh         = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].Aap         = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].Aa          = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].Ap          = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].Pay         = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].Ppy         = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].ang_sx      = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].ang_sz      = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].ang_ax      = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].ang_az      = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].ang_pn      = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].AB          = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.S         = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.ha        = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.ha1       = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.ha2       = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.Pa        = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.Pa1       = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.Pa2       = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.Pg1       = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.Pg2       = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.Ph1       = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.Ph2       = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.Pf1       = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.Pf2       = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.V1        = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.V2        = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.p1        = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.p2        = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].p.S         = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].p.h.x       = fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].p.h.y       = fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].p.h.z       = fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].p.P.x       = fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].p.P.y       = fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].p.P.z       = fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].O.x         = fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].O.y         = fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].O.z         = fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].A.x         = fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].A.y         = fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].A.z         = fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].B.x         = fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].B.y         = fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].B.z         = fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].W.x         = fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].W.y         = fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].W.z         = fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].Wfull.x     = fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].Wfull.y     = fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].Wfull.z     = fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].Wfree.x     = fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].Wfree.y     = fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].Wfree.z     = fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].D.x         = fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].D.y         = fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].D.z         = fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].C.x         = fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].C.y         = fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].C.z         = fields.at(3).toFloat();
           // line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].ssC.x       = fields.at(1).toFloat();
           // /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].ssC.y       = fields.at(2).toFloat();
           // /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].ssC.z       = fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].saC.x       = fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].saC.y       = fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].saC.z       = fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].saCfree.x   = fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].saCfree.y   = fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].saCfree.z   = fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].F.x         = fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].F.y         = fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].F.z         = fields.at(3).toFloat();
            if(out.st[k].nwh==2)
            {
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].p.h1.x  = fields.at(1).toFloat();//,'f',3);
                /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].p.h1.y  = fields.at(2).toFloat();//,'f',3);
                /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].p.h1.z  = fields.at(3).toFloat();//,'f',3);
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].p.h2.x  = fields.at(1).toFloat();//,'f',3);
                /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].p.h2.y  = fields.at(2).toFloat();//,'f',3);
                /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].p.h2.z  = fields.at(3).toFloat();//,'f',3);
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].p.P1.x  = fields.at(1).toFloat();//,'f',3);
                /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].p.P1.y  = fields.at(2).toFloat();//,'f',3);
                /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].p.P1.z  = fields.at(3).toFloat();//,'f',3);
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].p.P2.x  = fields.at(1).toFloat();//,'f',3);
                /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].p.P2.y  = fields.at(2).toFloat();//,'f',3);
                /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].p.P2.z  = fields.at(3).toFloat();//,'f',3);
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].W1.x    = fields.at(1).toFloat();//,'f',3);
                /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].W1.y    = fields.at(2).toFloat();//,'f',3);
                /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].W1.z    = fields.at(3).toFloat();//,'f',3);
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].W2.x    = fields.at(1).toFloat();//,'f',3);
                /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].W2.y    = fields.at(2).toFloat();//,'f',3);
                /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].W2.z    = fields.at(3).toFloat();//,'f',3);
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].D1.x    = fields.at(1).toFloat();//,'f',3);
                /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].D1.y    = fields.at(2).toFloat();//,'f',3);
                /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].D1.z    = fields.at(3).toFloat();//,'f',3);
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].D2.x    = fields.at(1).toFloat();//,'f',3);
                /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].D2.y    = fields.at(2).toFloat();//,'f',3);
                /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].D2.z    = fields.at(3).toFloat();//,'f',3);
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].C1.x    = fields.at(1).toFloat();//,'f',3);
                /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].C1.y    = fields.at(2).toFloat();//,'f',3);
                /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].C1.z    = fields.at(3).toFloat();//,'f',3);
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].C2.x    = fields.at(1).toFloat();//,'f',3);
                /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].C2.y    = fields.at(2).toFloat();//,'f',3);
                /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].C2.z    = fields.at(3).toFloat();//,'f',3);
              //  line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].ssC1.x  = fields.at(1).toFloat();//,'f',3);
              //  /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].ssC1.y  = fields.at(2).toFloat();//,'f',3);
              //  /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].ssC1.z  = fields.at(3).toFloat();//,'f',3);
              //  line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].ssC2.x  = fields.at(1).toFloat();//,'f',3);
              //  /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].ssC2.y  = fields.at(2).toFloat();//,'f',3);
              //  /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].ssC2.z  = fields.at(3).toFloat();//,'f',3);
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].F1.x    = fields.at(1).toFloat();//,'f',3);
                /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].F1.y    = fields.at(2).toFloat();//,'f',3);
                /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].F1.z    = fields.at(3).toFloat();//,'f',3);
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].F2.x    = fields.at(1).toFloat();//,'f',3);
                /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].F2.y    = fields.at(2).toFloat();//,'f',3);
                /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].F2.z    = fields.at(3).toFloat();//,'f',3);
            }
            else
            {
                out.st[k].p.h1.x  = 0;
                out.st[k].p.h1.y  = 0;
                out.st[k].p.h1.z  = 0;
                out.st[k].p.h2.x  = 0;
                out.st[k].p.h2.y  = 0;
                out.st[k].p.h2.z  = 0;
                out.st[k].p.P1.x  = 0;
                out.st[k].p.P1.y  = 0;
                out.st[k].p.P1.z  = 0;
                out.st[k].p.P2.x  = 0;
                out.st[k].p.P2.y  = 0;
                out.st[k].p.P2.z  = 0;
                out.st[k].W1.x    = 0;
                out.st[k].W1.y    = 0;
                out.st[k].W1.z    = 0;
                out.st[k].W2.x    = 0;
                out.st[k].W2.y    = 0;
                out.st[k].W2.z    = 0;
                out.st[k].D1.x    = 0;
                out.st[k].D1.y    = 0;
                out.st[k].D1.z    = 0;
                out.st[k].D2.x    = 0;
                out.st[k].D2.y    = 0;
                out.st[k].D2.z    = 0;
                out.st[k].C1.x    = 0;
                out.st[k].C1.y    = 0;
                out.st[k].C1.z    = 0;
                out.st[k].C2.x    = 0;
                out.st[k].C2.y    = 0;
                out.st[k].C2.z    = 0;
                out.st[k].ssC1.x  = 0;
                out.st[k].ssC1.y  = 0;
                out.st[k].ssC1.z  = 0;
                out.st[k].ssC2.x  = 0;
                out.st[k].ssC2.y  = 0;
                out.st[k].ssC2.z  = 0;
                out.st[k].F1.x    = 0;
                out.st[k].F1.y    = 0;
                out.st[k].F1.z    = 0;
                out.st[k].F2.x    = 0;
                out.st[k].F2.y    = 0;
                out.st[k].F2.z    = 0;
            }
            out.st[k].ground = ini.ground[k]*0.001;// ?
        }
        line = in->readLine();//---------------
        line = in->readLine();//tailrotor
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.tr.O.x=        fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.tr.O.y=        fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.tr.O.z=        fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.tr.D  =        fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.tr.H  =        fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.tr.T  =        fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.tr.S  =        fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.tr.Mx =        fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.tr.My =        fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.tr.Mz =        fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.tr.solve     = fields.at(1).toFloat();
        line = in->readLine();//---------------
        line = in->readLine();//mainrotor
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.O.x=        fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.O.y=        fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.O.z=        fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.D  =        fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.eps=        fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.nb=         fields.at(1).toInt();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.rvs=        fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.Lb=         fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.H         = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.T         = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.S         = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.Mx        = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.My        = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.Mz        = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.solve     = fields.at(1).toFloat();
        if(out.mr.solve)
        {
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.a.x       = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.a.y       = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.a.z       = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.v.x       = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.v.y       = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.v.z       = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.s.x       = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.s.y       = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.s.z       = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.dct.x     = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.dct.y     = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.dct.z     = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.Faxe      = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.deltabs   = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.dctbs     = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.Mv        = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.dfi       = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.Mvg       = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.dfig      = fields.at(1).toFloat();
            for (int l = 0; l < out.mr.nb; l++)
            {
                line = in->readLine();//mr.blade=   #
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.b[l].psi = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.b[l].psi0= fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.b[l].fi  = fields.at(1).toFloat();
            }
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.vgsolve   = fields.at(1).toFloat();
            if(out.mr.vgsolve)
            {
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.vg.psi    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.vg.eps    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.vg.Mn     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.vg.Mt     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.vg.Mk     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.vg.Mi     = fields.at(1).toFloat();
            }
        }
        line = in->readLine();//------------------------
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  eofidentity = fields.at(0);
        if(eofidentity!="[eof]") readerror=true;
        delete in;
        infile.close();
    }
    else readerror=true;
return readerror;
}
bool write_helicopter_state(QString filename)
{//var:
    QString s,ss;
    QString outstr = titleapplname;
    outstr += "\n----------------------------------------------------------------------";
    outstr += "\ni= " + ss.setNum(out.iter);
    outstr += "\nt= " + ss.setNum(out.t,'f',6);
    outstr += "\n----------------------------------------------------------------------";
    outstr += "\norient(xyz)=     \t" + ss.setNum(out.fus.orient.x,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.orient.y,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.orient.z,'f',3);
    outstr += "\norient(krt)=     \t" + ss.setNum(out.fus.orient.k,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.orient.r,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.orient.t,'f',6);
    outstr += "\nspeed(xyz)=      \t" + ss.setNum(out.fus.speed.x ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.speed.y ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.speed.z ,'f',6);
    outstr += "\nspeed(krt)=      \t" + ss.setNum(out.fus.speed.k ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.speed.r ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.speed.t ,'f',6);
    outstr += "\nv(xyz)=          \t" + ss.setNum(out.fus.v.x     ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.v.y     ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.v.z     ,'f',6);
    outstr += "\nw(xyz)=          \t" + ss.setNum(out.fus.w.x     ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.w.y     ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.w.z     ,'f',6);
    outstr += "\na(xyz)=          \t" + ss.setNum(out.fus.a.x     ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.a.y     ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.a.z     ,'f',6);
    outstr += "\ne(xyz)=          \t" + ss.setNum(out.fus.e.x     ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.e.y     ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.e.z     ,'f',6);
    outstr += "\nn(xyz)=          \t" + ss.setNum(out.fus.n.x     ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.n.y     ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.n.z     ,'f',6);
    outstr += "\nFsum(xyz)=       \t" + ss.setNum(out.fus.Fsum.x  ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Fsum.y  ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Fsum.z  ,'f',3);
    outstr += "\nMsum(xyz)=       \t" + ss.setNum(out.fus.Msum.x  ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Msum.y  ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Msum.z  ,'f',3);
    outstr += "\nFhg(xyz)=        \t" + ss.setNum(out.fus.Fhg.x   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Fhg.y   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Fhg.z   ,'f',3);
    outstr += "\nMhg(xyz)=        \t" + ss.setNum(out.fus.Mhg.x   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Mhg.y   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Mhg.z   ,'f',3);
    outstr += "\nFad(xyz)=        \t" + ss.setNum(out.fus.Fad.x   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Fad.y   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Fad.z   ,'f',3);
    outstr += "\nMad(xyz)=        \t" + ss.setNum(out.fus.Mad.x   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Mad.y   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Mad.z   ,'f',3);
    outstr += "\nFmr(xyz)=        \t" + ss.setNum(out.fus.Fmr.x   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Fmr.y   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Fmr.z   ,'f',3);
    outstr += "\nMmr(xyz)=        \t" + ss.setNum(out.fus.Mmr.x   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Mmr.y   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Mmr.z   ,'f',3);
    outstr += "\nFtr(xyz)=        \t" + ss.setNum(out.fus.Ftr.x   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Ftr.y   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Ftr.z   ,'f',3);
    outstr += "\nMtr(xyz)=        \t" + ss.setNum(out.fus.Mtr.x   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Mtr.y   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Mtr.z   ,'f',3);
    outstr += "\nFsh(xyz)=        \t" + ss.setNum(out.fus.Fsh.x   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Fsh.y   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Fsh.z   ,'f',3);
    outstr += "\nMsh(xyz)=        \t" + ss.setNum(out.fus.Msh.x   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Msh.y   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Msh.z   ,'f',3);
    outstr += "\nct(xyz)=         \t" + ss.setNum(out.fus.xct     ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.yct     ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.zct     ,'f',3);
    outstr += "\nvtp.Oxyz=        \t" + ss.setNum(out.cargo.Ox    ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.cargo.Oy    ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.cargo.Oz    ,'f',3);
    outstr += "\nvtp.Qxyz=        \t" + ss.setNum(out.cargo.CTx   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.cargo.CTy   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.cargo.CTz   ,'f',3);
    outstr += "\nvtp.Gxyz=        \t" + ss.setNum(out.cargo.Gx    ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.cargo.Gy    ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.cargo.Gz    ,'f',3);
    outstr += "\nextload1pos=     \t" + ss.setNum(out.fus.F1pos.x ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.F1pos.y ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.F1pos.z ,'f',3);
    outstr += "\nextload1val=     \t" + ss.setNum(out.fus.F1.x    ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.F1.y    ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.F1.z    ,'f',3);
    outstr += "\nextload2pos=     \t" + ss.setNum(out.fus.F2pos.x ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.F2pos.y ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.F2pos.z ,'f',3);
    outstr += "\nextload2val=     \t" + ss.setNum(out.fus.F2.x    ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.F2.y    ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.F2.z    ,'f',3);
    outstr += "\n----------------------------------------------------------------------";
    outstr += "\nshassi.nst= " + ss.setNum(out.nst);
    for (int k = 0; k < out.nst; k++)
    {
        outstr += "\n#st= " + ss.setNum(k);
        outstr += "\ns"+s.setNum(k)+".nwh=          \t" + ss.setNum(out.st[k].nwh     );
        outstr += "\ns"+s.setNum(k)+".status=       \t" + ss.setNum(out.st[k].status  );
        outstr += "\ns"+s.setNum(k)+".a.status=     \t" + ss.setNum(out.st[k].a.status);
        outstr += "\ns"+s.setNum(k)+".p.status=     \t" + ss.setNum(out.st[k].p.status);
        outstr += "\ns"+s.setNum(k)+".Bwh=          \t" + ss.setNum(out.st[k].Bwh      ,'f',1);
        outstr += "\ns"+s.setNum(k)+".Dwh=          \t" + ss.setNum(out.st[k].Dwh      ,'f',1);
        outstr += "\ns"+s.setNum(k)+".rwh=          \t" + ss.setNum(out.st[k].rwh      ,'f',1);
        outstr += "\ns"+s.setNum(k)+".Aap=          \t" + ss.setNum(out.st[k].Aap      ,'f',3);
        outstr += "\ns"+s.setNum(k)+".Aa=           \t" + ss.setNum(out.st[k].Aa       ,'f',3);
        outstr += "\ns"+s.setNum(k)+".Ap=           \t" + ss.setNum(out.st[k].Ap       ,'f',3);
        outstr += "\ns"+s.setNum(k)+".Pay=          \t" + ss.setNum(out.st[k].Pay      ,'f',3);
        outstr += "\ns"+s.setNum(k)+".Ppy=          \t" + ss.setNum(out.st[k].Ppy      ,'f',3);
        outstr += "\ns"+s.setNum(k)+".ang_sx=       \t" + ss.setNum(out.st[k].ang_sx   ,'f',6);
        outstr += "\ns"+s.setNum(k)+".ang_sz=       \t" + ss.setNum(out.st[k].ang_sz   ,'f',6);
        outstr += "\ns"+s.setNum(k)+".ang_ax=       \t" + ss.setNum(out.st[k].ang_ax   ,'f',6);
        outstr += "\ns"+s.setNum(k)+".ang_az=       \t" + ss.setNum(out.st[k].ang_az   ,'f',6);
        outstr += "\ns"+s.setNum(k)+".ang_pn=       \t" + ss.setNum(out.st[k].ang_pn   ,'f',6);
        outstr += "\ns"+s.setNum(k)+".AB=           \t" + ss.setNum(out.st[k].AB       ,'f',3);
        outstr += "\ns"+s.setNum(k)+".a.S=          \t" + ss.setNum(out.st[k].a.S      ,'f',3);
        outstr += "\ns"+s.setNum(k)+".a.ha=         \t" + ss.setNum(out.st[k].a.ha     ,'f',4);
        outstr += "\ns"+s.setNum(k)+".a.hac1=       \t" + ss.setNum(out.st[k].a.ha1    ,'f',4);
        outstr += "\ns"+s.setNum(k)+".a.hac2=       \t" + ss.setNum(out.st[k].a.ha2    ,'f',4);
        outstr += "\ns"+s.setNum(k)+".a.Pa=         \t" + ss.setNum(out.st[k].a.Pa     ,'f',3);
        outstr += "\ns"+s.setNum(k)+".a.P1=         \t" + ss.setNum(out.st[k].a.Pa1    ,'f',3);
        outstr += "\ns"+s.setNum(k)+".a.P2=         \t" + ss.setNum(out.st[k].a.Pa2    ,'f',3);
        outstr += "\ns"+s.setNum(k)+".a.Pg1=        \t" + ss.setNum(out.st[k].a.Pg1    ,'f',3);
        outstr += "\ns"+s.setNum(k)+".a.Pg2=        \t" + ss.setNum(out.st[k].a.Pg2    ,'f',3);
        outstr += "\ns"+s.setNum(k)+".a.Ph1=        \t" + ss.setNum(out.st[k].a.Ph1    ,'f',3);
        outstr += "\ns"+s.setNum(k)+".a.Ph2=        \t" + ss.setNum(out.st[k].a.Ph2    ,'f',3);
        outstr += "\ns"+s.setNum(k)+".a.Pf1=        \t" + ss.setNum(out.st[k].a.Pf1    ,'f',3);
        outstr += "\ns"+s.setNum(k)+".a.Pf2=        \t" + ss.setNum(out.st[k].a.Pf2    ,'f',3);
        outstr += "\ns"+s.setNum(k)+".a.V1=         \t" + ss.setNum(out.st[k].a.V1     ,'f',6);
        outstr += "\ns"+s.setNum(k)+".a.V2=         \t" + ss.setNum(out.st[k].a.V2     ,'f',6);
        outstr += "\ns"+s.setNum(k)+".a.p1=         \t" + ss.setNum(out.st[k].a.p1     ,'f',3);
        outstr += "\ns"+s.setNum(k)+".a.p2=         \t" + ss.setNum(out.st[k].a.p2     ,'f',3);
        outstr += "\ns"+s.setNum(k)+".p.S=          \t" + ss.setNum(out.st[k].p.S      ,'f',3);
        outstr += "\ns"+s.setNum(k)+".p.h(xyz)=     \t" + ss.setNum(out.st[k].p.h.x    ,'f',4);
        outstr +=                               "   \t" + ss.setNum(out.st[k].p.h.y    ,'f',4);
        outstr +=                               "   \t" + ss.setNum(out.st[k].p.h.z    ,'f',4);
        outstr += "\ns"+s.setNum(k)+".p.P(xyz)=     \t" + ss.setNum(out.st[k].p.P.x    ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].p.P.y    ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].p.P.z    ,'f',3);
        outstr += "\ns"+s.setNum(k)+".O(xyz)=       \t" + ss.setNum(out.st[k].O.x      ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].O.y      ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].O.z      ,'f',3);
        outstr += "\ns"+s.setNum(k)+".A(xyz)=       \t" + ss.setNum(out.st[k].A.x      ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].A.y      ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].A.z      ,'f',3);
        outstr += "\ns"+s.setNum(k)+".B(xyz)=       \t" + ss.setNum(out.st[k].B.x      ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].B.y      ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].B.z      ,'f',3);
        outstr += "\ns"+s.setNum(k)+".W(xyz)=       \t" + ss.setNum(out.st[k].W.x      ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].W.y      ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].W.z      ,'f',3);
        outstr += "\ns"+s.setNum(k)+".Wfull(xyz)=   \t" + ss.setNum(out.st[k].Wfull.x  ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].Wfull.y  ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].Wfull.z  ,'f',3);
        outstr += "\ns"+s.setNum(k)+".Wfree(xyz)=   \t" + ss.setNum(out.st[k].Wfree.x  ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].Wfree.y  ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].Wfree.z  ,'f',3);
        outstr += "\ns"+s.setNum(k)+".D(xyz)=       \t" + ss.setNum(out.st[k].D.x      ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].D.y      ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].D.z      ,'f',3);
        outstr += "\ns"+s.setNum(k)+".C(xyz)=       \t" + ss.setNum(out.st[k].C.x      ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].C.y      ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].C.z      ,'f',3);
        //outstr += "\ns"+s.setNum(k)+".ssC(xyz)=     \t" + ss.setNum(out.st[k].ssC.x    ,'f',3);
        //outstr +=                               "   \t" + ss.setNum(out.st[k].ssC.y    ,'f',3);
        //outstr +=                               "   \t" + ss.setNum(out.st[k].ssC.z    ,'f',3);
        outstr += "\ns"+s.setNum(k)+".saC(xyz)=     \t" + ss.setNum(out.st[k].saC.x    ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].saC.y    ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].saC.z    ,'f',3);
        outstr += "\ns"+s.setNum(k)+".saCfree(xyz)= \t" + ss.setNum(out.st[k].saCfree.x,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].saCfree.y,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].saCfree.z,'f',3);
        outstr += "\ns"+s.setNum(k)+".F(xyz)=       \t" + ss.setNum(out.st[k].F.x      ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].F.y      ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].F.z      ,'f',3);

        if(out.st[k].nwh==2)
        {
            outstr += "\ns"+s.setNum(k)+".p.h1(xyz)=    \t" + ss.setNum(out.st[k].p.h1.x  ,'f',4);
            outstr +=                               "   \t" + ss.setNum(out.st[k].p.h1.y  ,'f',4);
            outstr +=                               "   \t" + ss.setNum(out.st[k].p.h1.z  ,'f',4);
            outstr += "\ns"+s.setNum(k)+".p.h2(xyz)=    \t" + ss.setNum(out.st[k].p.h2.x  ,'f',4);
            outstr +=                               "   \t" + ss.setNum(out.st[k].p.h2.y  ,'f',4);
            outstr +=                               "   \t" + ss.setNum(out.st[k].p.h2.z  ,'f',4);
            outstr += "\ns"+s.setNum(k)+".p.P1(xyz)=    \t" + ss.setNum(out.st[k].p.P1.x  ,'f',3);
            outstr +=                               "   \t" + ss.setNum(out.st[k].p.P1.y  ,'f',3);
            outstr +=                               "   \t" + ss.setNum(out.st[k].p.P1.z  ,'f',3);
            outstr += "\ns"+s.setNum(k)+".p.P2(xyz)=    \t" + ss.setNum(out.st[k].p.P2.x  ,'f',3);
            outstr +=                               "   \t" + ss.setNum(out.st[k].p.P2.y  ,'f',3);
            outstr +=                               "   \t" + ss.setNum(out.st[k].p.P2.z  ,'f',3);
            outstr += "\ns"+s.setNum(k)+".W1(xyz)=      \t" + ss.setNum(out.st[k].W1.x    ,'f',3);
            outstr +=                               "   \t" + ss.setNum(out.st[k].W1.y    ,'f',3);
            outstr +=                               "   \t" + ss.setNum(out.st[k].W1.z    ,'f',3);
            outstr += "\ns"+s.setNum(k)+".W2(xyz)=      \t" + ss.setNum(out.st[k].W2.x    ,'f',3);
            outstr +=                               "   \t" + ss.setNum(out.st[k].W2.y    ,'f',3);
            outstr +=                               "   \t" + ss.setNum(out.st[k].W2.z    ,'f',3);
            outstr += "\ns"+s.setNum(k)+".D1(xyz)=      \t" + ss.setNum(out.st[k].D1.x    ,'f',3);
            outstr +=                               "   \t" + ss.setNum(out.st[k].D1.y    ,'f',3);
            outstr +=                               "   \t" + ss.setNum(out.st[k].D1.z    ,'f',3);
            outstr += "\ns"+s.setNum(k)+".D2(xyz)=      \t" + ss.setNum(out.st[k].D2.x    ,'f',3);
            outstr +=                               "   \t" + ss.setNum(out.st[k].D2.y    ,'f',3);
            outstr +=                               "   \t" + ss.setNum(out.st[k].D2.z    ,'f',3);
            outstr += "\ns"+s.setNum(k)+".C1(xyz)=      \t" + ss.setNum(out.st[k].C1.x    ,'f',3);
            outstr +=                               "   \t" + ss.setNum(out.st[k].C1.y    ,'f',3);
            outstr +=                               "   \t" + ss.setNum(out.st[k].C1.z    ,'f',3);
            outstr += "\ns"+s.setNum(k)+".C2(xyz)=      \t" + ss.setNum(out.st[k].C2.x    ,'f',3);
            outstr +=                               "   \t" + ss.setNum(out.st[k].C2.y    ,'f',3);
            outstr +=                               "   \t" + ss.setNum(out.st[k].C2.z    ,'f',3);
            //outstr += "\ns"+s.setNum(k)+".C1sks(xyz)=   \t" + ss.setNum(out.st[k].ssC1.x  ,'f',3);
            //outstr +=                               "   \t" + ss.setNum(out.st[k].ssC1.y  ,'f',3);
            //outstr +=                               "   \t" + ss.setNum(out.st[k].ssC1.z  ,'f',3);
            //outstr += "\ns"+s.setNum(k)+".C2sks(xyz)=   \t" + ss.setNum(out.st[k].ssC2.x  ,'f',3);
            //outstr +=                               "   \t" + ss.setNum(out.st[k].ssC2.y  ,'f',3);
            //outstr +=                               "   \t" + ss.setNum(out.st[k].ssC2.z  ,'f',3);
            outstr += "\ns"+s.setNum(k)+".F1(xyz)=      \t" + ss.setNum(out.st[k].F1.x    ,'f',3);
            outstr +=                               "   \t" + ss.setNum(out.st[k].F1.y    ,'f',3);
            outstr +=                               "   \t" + ss.setNum(out.st[k].F1.z    ,'f',3);
            outstr += "\ns"+s.setNum(k)+".F2(xyz)=      \t" + ss.setNum(out.st[k].F2.x    ,'f',3);
            outstr +=                               "   \t" + ss.setNum(out.st[k].F2.y    ,'f',3);
            outstr +=                               "   \t" + ss.setNum(out.st[k].F2.z    ,'f',3);
        }
    }
    outstr += "\n----------------------------------------------------------------------";
    outstr += "\ntailrotor";
    outstr += "\ntr.O.x=          \t" + ss.setNum(out.tr.O.x      ,'f',1);
    outstr += "\ntr.O.y=          \t" + ss.setNum(out.tr.O.y      ,'f',1);
    outstr += "\ntr.O.z=          \t" + ss.setNum(out.tr.O.z      ,'f',1);
    outstr += "\ntr.D=            \t" + ss.setNum(out.tr.D        ,'f',1);
    outstr += "\ntr.H=            \t" + ss.setNum(out.tr.H        ,'f',3);
    outstr += "\ntr.T=            \t" + ss.setNum(out.tr.T        ,'f',3);
    outstr += "\ntr.S=            \t" + ss.setNum(out.tr.S        ,'f',3);
    outstr += "\ntr.Mx=           \t" + ss.setNum(out.tr.Mx       ,'f',3);
    outstr += "\ntr.My=           \t" + ss.setNum(out.tr.My       ,'f',3);
    outstr += "\ntr.Mz=           \t" + ss.setNum(out.tr.Mz       ,'f',3);
    outstr += "\ntr.solve=        \t" + ss.setNum(out.tr.solve);
    outstr += "\n----------------------------------------------------------------------";
    outstr += "\nmainrotor";
    outstr += "\nmr.O.x=          \t" + ss.setNum(out.mr.O.x      ,'f',1);
    outstr += "\nmr.O.y=          \t" + ss.setNum(out.mr.O.y      ,'f',1);
    outstr += "\nmr.O.z=          \t" + ss.setNum(out.mr.O.z      ,'f',1);
    outstr += "\nmr.D=            \t" + ss.setNum(out.mr.D        ,'f',1);
    outstr += "\nmr.eps=          \t" + ss.setNum(out.mr.eps      ,'f',4);
    outstr += "\nmr.nb=           \t" + ss.setNum(out.mr.nb);
    outstr += "\nmr.rvs=          \t" + ss.setNum(out.mr.rvs      ,'f',3);
    outstr += "\nmr.Lb=           \t" + ss.setNum(out.mr.Lb       ,'f',3);
    outstr += "\nmr.H=            \t" + ss.setNum(out.mr.H        ,'f',3);
    outstr += "\nmr.T=            \t" + ss.setNum(out.mr.T        ,'f',3);
    outstr += "\nmr.S=            \t" + ss.setNum(out.mr.S        ,'f',3);
    outstr += "\nmr.Mx=           \t" + ss.setNum(out.mr.Mx       ,'f',3);
    outstr += "\nmr.My=           \t" + ss.setNum(out.mr.My       ,'f',3);
    outstr += "\nmr.Mz=           \t" + ss.setNum(out.mr.Mz       ,'f',3);
    outstr += "\nmr.solve=        \t" + ss.setNum(out.mr.solve);
    if(out.mr.solve)
    {
        outstr += "\nmr.a.x=          \t" + ss.setNum(out.mr.a.x      ,'f',6);
        outstr += "\nmr.a.y=          \t" + ss.setNum(out.mr.a.y      ,'f',6);
        outstr += "\nmr.a.z=          \t" + ss.setNum(out.mr.a.z      ,'f',6);
        outstr += "\nmr.v.x=          \t" + ss.setNum(out.mr.v.x      ,'f',6);
        outstr += "\nmr.v.y=          \t" + ss.setNum(out.mr.v.y      ,'f',6);
        outstr += "\nmr.v.z=          \t" + ss.setNum(out.mr.v.z      ,'f',6);
        outstr += "\nmr.s.x=          \t" + ss.setNum(out.mr.s.x      ,'f',6);
        outstr += "\nmr.s.y=          \t" + ss.setNum(out.mr.s.y      ,'f',6);
        outstr += "\nmr.s.z=          \t" + ss.setNum(out.mr.s.z      ,'f',6);
        outstr += "\nmr.dct.x=        \t" + ss.setNum(out.mr.dct.x    ,'f',6);
        outstr += "\nmr.dct.y=        \t" + ss.setNum(out.mr.dct.y    ,'f',6);
        outstr += "\nmr.dct.z=        \t" + ss.setNum(out.mr.dct.z    ,'f',6);
        outstr += "\nmr.Faxe =        \t" + ss.setNum(out.mr.Faxe     ,'f',3);
        outstr += "\nmr.deltabs=      \t" + ss.setNum(out.mr.deltabs  ,'f',6);
        outstr += "\nmr.dctbs=        \t" + ss.setNum(out.mr.dctbs    ,'f',6);
        outstr += "\nmr.Mvt=          \t" + ss.setNum(out.mr.Mv       ,'f',3);
        outstr += "\nmr.dfi=          \t" + ss.setNum(out.mr.dfi      ,'f',6);
        outstr += "\nmr.Mvtg=         \t" + ss.setNum(out.mr.Mvg      ,'f',3);
        outstr += "\nmr.dfig=         \t" + ss.setNum(out.mr.dfig     ,'f',6);
        for (int l = 0; l < out.mr.nb; l++)
        {
            outstr += "\nmr.blade=           \t" + ss.setNum(l);
            outstr += "\nmr.b.psi=           \t" + ss.setNum(out.mr.b[l].psi ,'f',6);
            outstr += "\nmr.b.psi0=          \t" + ss.setNum(out.mr.b[l].psi0,'f',6);
            outstr += "\nmr.b.fi=            \t" + ss.setNum(out.mr.b[l].fi  ,'f',6);
        }
        outstr += "\nmr.vgsolve=      \t" + ss.setNum(out.mr.vgsolve);
        if(out.mr.vgsolve)
        {
            outstr += "\nvg.psi=          \t" + ss.setNum(out.mr.vg.psi   ,'f',6);
            outstr += "\nvg.eps=          \t" + ss.setNum(out.mr.vg.eps   ,'f',6);
            outstr += "\nvg.Mn=           \t" + ss.setNum(out.mr.vg.Mn    ,'f',3);
            outstr += "\nvg.Mt=           \t" + ss.setNum(out.mr.vg.Mt    ,'f',3);
            outstr += "\nvg.Mk=           \t" + ss.setNum(out.mr.vg.Mk    ,'f',3);
            outstr += "\nvg.Mi=           \t" + ss.setNum(out.mr.vg.Mi    ,'f',3);
        }
    }
    outstr += "\n----------------------------------------------------------------------";
    outstr += "\n[eof]";

    if(!filename.isEmpty())
    {
        QFile outfile(filename);
        if(outfile.open( QIODevice::WriteOnly | QIODevice::Text )) //outfile.isWritable())
        {
            QTextStream outstream(&outfile);
            outstream << outstr;
            outfile.close();
            return 0;
        }
        else
        {
            toMemo("не удалось открыть для записи файл " + filename);
            return 1;
        }
    }
    else return 1;
}



//вспомогательные
QString getStateDataFolderName(QString fullname)
{
    QString fname="";
    QRegExp rx("\\.");
    QStringList fields = fullname.split(rx);
    int fldsz = fields.size();
    for(int _=0; _<fldsz-1; _++)
    {
        fname += fields.at(_);
        if(_==(fldsz-2)) fname += "\/"; else fname += ".";
    }
    return fname;
}
QString getFileNameOnly(QString fullname)
{
    QRegExp rx("\\/");
    QStringList fields = fullname.split(rx);
    return fields.at(fields.size()-1);
}
QString strSumm(QString str1, QString str2)
{
return  str1 + str2;
}
QString getElapsedTime(QTime started)//
{
    QString elapsedtimeformat = "";
    QString ss1,ss2;

    int elaps = started.elapsed();
    int elaps_ms= elaps%1000;
    int elaps_s = (elaps-elaps_ms)*0.001;
    int elaps_m = elaps_s/float(60);
    if(elaps_s >= elaps_m*60) elaps_s -= elaps_m*60;
    elapsedtimeformat = ss.setNum(elaps_m)+"m " + ss1.setNum(elaps_s)+"s " + ss2.setNum(elaps_ms)+"ms ";

return elapsedtimeformat;
}
void toMemo(QString string)//currentstep to memoout
{
    qout << string << "\n";  qout.flush();
}
void toLogf(QString string)//currentstep to logfile
{
    QFile logfile(logfilename);
    logfile.open(QIODevice::Append | QIODevice::Text);
    QTextStream logstream(&logfile);//logstream.setCodec("CP1251"); //log = &logstream;//logstream << "\nlogstream: ok"; *log << "\n*log: ok";    //*log << "\n";    //*log << string;
    logstream << "\n";
    logstream << string;
    logfile.close();
}
void loadLog()
{
//    QFile logfile(logfilename);
//    if(logfile.exists())
//    {
//        ui->memout->clear();
//        logfile.open( QIODevice::ReadOnly | QIODevice::Text );
//        QTextStream loadlog(&logfile);
//        toMemo(loadlog.readAll());
//        logfile.close();
//    }
//    else toMemo("warning: " + logfilename + " not found");
}
bool find_str(QTextStream *in, QString searchString, QString &findStr )
{//------------------------------------------------------------------------------------------------
    in->seek(0);
    bool finded = false, isbreak = false;
    QString line, findS;
    QStringList fields;
    QRegExp rx("\\s");

    while((!in->atEnd())&&(!isbreak))
    {
        line = in->readLine();
        fields = line.split(rx, QString::SkipEmptyParts);
        if(fields.size()>1)
        {
            for(int i=0; i<fields.size(); i++)
            {
                findS = fields.at(i);
                if(findS==searchString)
                {
                      findStr = fields.at(i+1);
                      finded = true; isbreak=true;  break;
                }
            }
        }
    }
return !finded;
}
bool find_int( QTextStream *in, QString searchString, int &findI)
{//------------------------------------------------------------------------------------------------
    in->seek(0);
    findI = 0;
    bool finded = false, isbreak = false;
    QString line, findS;
    QStringList fields;
    QRegExp rx("\\s");

    while((!in->atEnd())&&(!isbreak))
    {
        line = in->readLine();
        fields = line.split(rx, QString::SkipEmptyParts);
        if(fields.size()>1)
        {
            for(int i=0; i<fields.size(); i++)
            {
                findS = fields.at(i);
                if(findS==searchString)
                {
                      findI = fields.at(i+1).toInt();
                      finded = true; isbreak=true;  break;
                }
                else  findI = 0;
            }
        }
    }
return !finded;
}
bool find_float(QTextStream *in, QString searchString, float &findF)
{//------------------------------------------------------------------------------------------------
    in->seek(0);
    findF = 0;
    bool finded = false, isbreak = false;
    QString line, findS;
    QStringList fields;
    QRegExp rx("\\s");

    while((!in->atEnd())&&(!isbreak))
    {
        line = in->readLine();
        fields = line.split(rx, QString::SkipEmptyParts);
        if(fields.size()>1)
        {
            for(int i=0; i<fields.size(); i++)
            {
                findS = fields.at(i);
                if(findS==searchString)
                {
                      findF = fields.at(i+1).toFloat();
                      finded = true; isbreak=true;  break;
                }
                else  findF = 0;
            }
        }
    }
return !finded;
}
bool find_2float(QTextStream *in, QString searchString, float &findF1, float &findF2)
{//------------------------------------------------------------------------------------------------
    in->seek(0);
    findF1 = 0;
    findF2 = 0;
    bool finded = false, isbreak = false;
    QString line, findS;
    QStringList fields;
    QRegExp rx("\\s");

    while((!in->atEnd())&&(!isbreak))
    {
        line = in->readLine();
        fields = line.split(rx, QString::SkipEmptyParts);
        if(fields.size()>1)
        {
            for(int i=0; i<fields.size(); i++)
            {
                findS = fields.at(i);
                if(findS==searchString)
                {
                    findF1 = fields.at(i+1).toFloat();
                    findF2 = fields.at(i+2).toFloat();
                    finded = true; isbreak=true;  break;
                }
                else
                {
                    findF1 = 0; findF2 = 0;
                }
            }
        }
    }
return !finded;
}
bool find_3float( QTextStream *in, QString searchString, float &findF1, float &findF2, float &findF3)
{//------------------------------------------------------------------------------------------------
    in->seek(0);
    findF1 = 0;
    findF2 = 0;
    findF3 = 0;
    bool finded = false, isbreak = false;
    QString line, findS;
    QStringList fields;
    QRegExp rx("\\s");

    while((!in->atEnd())&&(!isbreak))
    {
        line = in->readLine();
        fields = line.split(rx, QString::SkipEmptyParts);
        if(fields.size()>1)
        {
            for(int i=0; i<fields.size(); i++)
            {
                findS = fields.at(i);
                if(findS==searchString)
                {
                    findF1 = fields.at(i+1).toFloat();
                    findF2 = fields.at(i+2).toFloat();
                    findF3 = fields.at(i+3).toFloat();
                    finded = true; isbreak=true; break;
                }
                else
                {
                    findF1 = 0; findF2 = 0; findF3 = 0;
                }
            }
        }
    }
return !finded;
}
bool find_Xmass(QTextStream *in, QString searchString, double mass[M1x], int &ncol)
{//считывание одномерного массива
    in->seek(0);
    bool finded = false, isbreak = false;
    QString line, findS;
    QStringList fields;
    QRegExp rx("\\s");
    QString ss;

    while((!in->atEnd())&&(!isbreak))
    {
        line = in->readLine();
        fields = line.split(rx, QString::SkipEmptyParts);
        if(fields.size()>=1)
        {
            for(int i=0; i<fields.size(); i++)
            {
                findS = fields.at(i);
                if(findS==searchString)
                {
                      finded = true; isbreak=true;  break;
                }
            }
        }
    }
    line = in->readLine();
    fields = line.split(rx, QString::SkipEmptyParts);
    ncol = fields.size();
    for(int i=0; i<ncol; i++)
    {
        mass[i] = fields.at(i).toDouble();
    }

    outstr += "      " + searchString + " " + ss.setNum(ncol) + "cols;\n";
    //if(log) ui->memout->appendPlainTextoutmass;
return !finded;
}
bool find_XYmass(QTextStream *in, QString searchString, double mass[M2x], int &nrow, int &ncol)
{//считывание двумерного массива
    in->seek(0);
    bool finded = false, isbreak = false;
    QString line, findS;
    QStringList fields;
    QRegExp rx("\\s");
    QString ss;

    while((!in->atEnd())&&(!isbreak))
    {
        line = in->readLine();
        fields = line.split(rx, QString::SkipEmptyParts);
        if(fields.size()>=1)
        {
            for(int i=0; i<fields.size(); i++)
            {
                findS = fields.at(i);
                if(findS==searchString)
                {
                      //nrow = fields.at(i+1).toInt();
                      finded = true; isbreak=true;  break;
                }
               // else  nrow = 0;
            }
        }
    }
    line = in->readLine();//считывание нулевой'заголовочной' строки
    fields = line.split(rx, QString::SkipEmptyParts);
    const int ncol_ = fields.size()-1;

    int j=0, k=0;//, kk=0;
    isbreak = false;
    while((!in->atEnd())&&(!isbreak))
    {
        //nrow = j--;
        line = in->readLine();
        fields = line.split(rx, QString::SkipEmptyParts);
        if(fields.size()-1==ncol_)
        {
            for(int i=0; i<ncol_; i++)//игнорируем нулевой элемент
            {
                k = i + j*ncol_;
                mass[k] = fields.at(i+1).toDouble();
            }
        }
        else
        {
            isbreak=true;
            nrow = j--;
            break;
        }
        j++;
    }
    ncol = ncol_;

    outstr += "      " + searchString + " " ;
    outstr += ss.setNum(nrow) + "rows, ";
    outstr += ss.setNum(ncol) + "cols;\n";
    //if(log) ui->memout->appendPlainText(out);
return !finded;
}


