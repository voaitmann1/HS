//HELICOPTER.SIMULATION.CONTROL
#include "mainwindow.h"
//-----------------------------------------------------------------------------
#define statefold "state\/"
#define  repfold  "report\/"
//-----------------------------------------------------------------------------

// управление расчетами
bool MainWindow::PreManagement()//ПОДГОТОВКА К РАСЧЕТАМ
{
    setNullInterface();//обнуления интерфейса
    //загрузить исходные данные, если на форме пусто
    QString meministr = ui->memo_hs->toPlainText();
    if( meministr.size() < 88)
    {
        OpenInputData();
        infilename = ui->pathEdit->text();
    }
    //теперь работаем с данными на форме
    QFile infile(infilename);
    if(infile.exists())
    {
        //сохраняем измененные данные на форме (memini)
        infile.open(QIODevice::WriteOnly | QIODevice::Text);
        QTextStream insave(&infile);         //out.setCodec("UTF-8");
        insave << ui->memo_hs->toPlainText();//out << tr("Значение x = ") << 12 << "\n";//поэлементно
        infile.close();
        //формируем-изменяем заголовок окна
        titlefilename = getFileNameOnly(infilename);
        setWindowTitle(titlefilename + " - " + titleapplname);
        //state.folder
        ini.datafolder = getFolderName(infilename) + statefold;
        QDir datafolder( ini.datafolder );
        if(!datafolder.exists()) datafolder.mkdir( ini.datafolder );
        //report.folder
        ini.reportfolder = getFolderName(infilename) + repfold;
        QDir reportfolder( ini.reportfolder );
        if(!reportfolder.exists()) reportfolder.mkdir( ini.reportfolder );
        //формируем имя .log файла
        logfilename = ini.reportfolder + titlefilename +".log";
        //первая запись в log-файл
        QFile logfile(logfilename);
        logfile.open(QIODevice::WriteOnly | QIODevice::Text);
        QTextStream logstream(&logfile);//logstream.setCodec("CP1251");
        logstream << "ExeFile : " << QCoreApplication::applicationFilePath();
        logfile.close();
        //фиксация текущих даты-времени как начала расчета
        startdate = QDate::currentDate();
        starttime = QTime::currentTime();
        tm.start();
        toLogf("Version : " + titleapplname);
        toLogf("Initial : " + infilename);
        toLogf("Started : " + startdate.toString("yyyy.MM.dd") + " " + starttime.toString());
        //toLogf(singleline);
    }
    else { ok=0; toMemo("некорректное имя файла"); }
return ok;
}
void MainWindow::ProcessManagement()//считывание исходных данных и запуск выбранного решателя
{
    //вывести в отчет выбранный режим расчета
    switch (ini.calcmode)
    {
    case 0:  toLogf("CalcMode: расчет эквивалентных винтов по классической теории");                                    break;
    case 1:  toLogf("CalcMode: расчет винтов с заданным управлением по классической теории");                           break;
    case 2:  toLogf("CalcMode: расчет сил и моментов приведенных к ЦТ вертолета при заданных параметрах управления");   break;
    case 30: toLogf("CalcMode: авторотация - расчет скорости снижения Vnу по заданным скорости Vnx и моменту mr.My на валу НВ");  break;
    case 33: toLogf("CalcMode: автожир - расчет тяги толкающего винта Tpr по заданной горизонтальной скорости Vnx");    break;
    case 3:  toLogf("CalcMode: балансировка вертолета для заданной скорости");                                          break;
    case 4:  toLogf("CalcMode: балансировка вертолета (V=0…Vmax)");                                                     break;
    case 5:  toLogf("CalcMode: динамика полета и посадки вертолета");                                                   break;
    }
    toLogf(singleline);
    if(read_ini(infilename))//считывание основных исходных данных
    {//считывание дополнительных исходных данных и запуск мат.моделей соотв.выбранному режиму расчета
        switch (ini.calcmode)
        {
        case 0: //расчет эквивалентного винта
        case 1: //расчет НВ/РВ по классической теории
            ini.solvePlaner = 0;
            if(ini.mrsolve)
            {
                if(read_mr(  fname_mr  )) { ok=0;  toMemo("read errors in " + fname_mr ); }
            }
            if(ini.trsolve)
            {
                if(read_tr(  fname_tr  )) { ok=0;  toMemo("read errors in " + fname_tr ); }
            }
            if(ini.prsolve)
            {
                if(read_pr(  fname_pr  )) { ok=0;  toMemo("read errors in " + fname_pr ); }
            }
            if(ok) computeRotors();//запуск основного решателя винтов
        break;
        case 2: //расчет суммарных силы-момента в ЦТ вертолета
            if(ini.solvePlaner)
            {
                if(read_fax( fname_fax )) { ok=0;  toMemo("read errors in " + fname_fax); }
            }
            if(ini.mrsolve)
            {
                if(read_mr(  fname_mr  )) { ok=0;  toMemo("read errors in " + fname_mr ); }
            }
            if(ini.trsolve)//if(ini.TRmount)//
            {
                if(read_tr(  fname_tr  )) { ok=0;  toMemo("read errors in " + fname_tr ); }
            }
            if(ini.prsolve)//if(ini.PRmount)//
            {
                if(read_pr(  fname_pr  )) { ok=0;  toMemo("read errors in " + fname_pr ); }
            }
            read_cntr(fname_cntr);
            if(ok) MainBalancing();//запуск основного решателя балансировки вертолета
        break;
        case 30://авторотация, определение Vу по заданным Vx и крут.моменту на валу
        case 33://расчет Tpr автожира по заданной горизонтальной скорости Vnx
        case 3: ///балансировка вертолета Vx
        case 4: //балансировка вертолета V=0...Vmax
            if(ini.solvePlaner)
            {
                if(read_fax( fname_fax )) { ok=0;  toMemo("read errors in " + fname_fax); }
            }
            if(ini.mrsolve)
            {
                if(read_mr(  fname_mr  )) { ok=0;  toMemo("read errors in " + fname_mr ); }
            }
            else { ok=0;  toMemo("solveMainRotor=0 >> расчет балансировки вертолета невозможен"); }

            if(ini.trsolve)//if(ini.TRmount)//
            {
                if(read_tr(  fname_tr  )) { ok=0;  toMemo("read errors in " + fname_tr ); }
            }
            if(ini.prsolve)//if(ini.PRmount)//
            {
                if(read_pr(  fname_pr  )) { ok=0;  toMemo("read errors in " + fname_pr ); }
            }
            read_cntr(fname_cntr);
            if(ok) MainBalancing();//запуск основного решателя балансировки вертолета
        break;
        case 5: ///динамика полета
            if(ini.solvePlaner)
            {
                if(read_fax( fname_fax )) { ok=0;  toMemo("read errors in " + fname_fax); }
            }
            if(ini.mrsolve)
            {
                if(read_mr(  fname_mr  )) { ok=0;  toMemo("read errors in " + fname_mr ); }
            }
            if(ini.trsolve)//if(ini.TRmount)//
            {
                if(read_tr(  fname_tr  )) { ok=0;  toMemo("read errors in " + fname_tr ); }
            }
            if(ini.prsolve)//if(ini.PRmount)//
            {
                if(read_pr(  fname_pr  )) { ok=0;  toMemo("read errors in " + fname_pr ); }
            }
            if(!ini.mrsolve && !ini.trsolve && !ini.prsolve)
            {
                MainWindow::on_check_MR_2_toggled(false);
                ui->check_MR_2->setChecked(0);
            }
            if(ini.extloads)//после расчета винтов!:
            {
                if(read_load(fname_load1, ini.load1, ini.load1pos)) { toMemo("read errors in " + fname_load1); }
                if(read_load(fname_load2, ini.load2, ini.load2pos)) { toMemo("read errors in " + fname_load2); }
            }
            read_cntr(fname_cntr);
            if(ok) MainFlying();//запуск основного решателя динамики посадки вертолета
        break;
        default: toMemo("invalid calcmode-index");  break;//некорректное значение
        }
        //загрузка .log файла
        if(ok) loadLog();
    }
    else ok=0;
    if(!ok) ui->tabWidget->setCurrentIndex(1);
}



//// Р А С Ч Е Т  В И Н Т О В
void MainWindow::computeRotors()//организатор расчета аэродинамики винта
{
    //удаление ранее рожденного объекта Helicopter (нужно при прерываниях расчета)
    hc = 0; delete hc;
    //создание объекта Landing
    hc = new Helicopter();
    //инициализация указателя класса на себя (нужно для статических функций)
    hc->SetClassPointer();
    //обнуление переменных
    hc->setNull();
    ui->tabWidget->setCurrentIndex(1);//активная вкладка
    //передача в объект Landing исходных данных, считанных из файла
    setPlanerInitial();
    setRotorsInitial();
    //подготовка к расчету, положение вертолета на шаге "0"
    i = 0;
    hc->iter = i;
    hc->nst  = 0;
    if(ini.mrsolve)
    {
        hc->mr->model = fname_mr;
        hc->mr->dt  = 0;//важно! для расчета полного оборота
        //hc->mr->ehx = hc->mr->ehy = hc->mr->ehz = 0;
        //hc->mr->whx = hc->mr->why = hc->mr->whz = 0;
        //hc->mr->ahx = hc->mr->ahy = hc->mr->ahz = 0;
        //hc->mr->vhx = hc->mr->vhy = hc->mr->vhz = 0;
        //hc->mr->shz = hc->mr->shy = hc->mr->shz = 0;
    }
    if(ini.trsolve)
    {
        hc->tr->model = fname_tr;
        hc->tr->dt  = 0;//важно! для расчета полного оборота
        //hc->tr->ehx = hc->tr->ehy = hc->tr->ehz = 0;
        //hc->tr->whx = hc->tr->why = hc->tr->whz = 0;
        //hc->tr->ahx = hc->tr->ahy = hc->tr->ahz = 0;
        //hc->tr->vhx = hc->tr->vhy = hc->tr->vhz = 0;
        //hc->tr->shz = hc->tr->shy = hc->tr->shz = 0;
    }
    if(ini.prsolve)
    {
        hc->pr->model = fname_pr;
        hc->pr->dt  = 0;//важно! для расчета полного оборота
        //hc->pr->ehx = hc->pr->ehy = hc->pr->ehz = 0;
        //hc->pr->whx = hc->pr->why = hc->pr->whz = 0;
        //hc->pr->ahx = hc->pr->ahy = hc->pr->ahz = 0;
        //hc->pr->vhx = hc->pr->vhy = hc->pr->vhz = 0;
        //hc->pr->shz = hc->pr->shy = hc->pr->shz = 0;
    }
    //подготовка модели планера
    hc->Pre();//
    //заполнение структуры результатов данными по планеру
    hc->fillOut(out);
    //MainRotor.Solve///
    if(ini.mrsolve)
    {
        toLogf("MainRotor.Prepare:");
        report = hc->mr->Pre2Str(ini.logkey);
        toLogf("ok");   //toLogf(report);
        toLogf("MainRotor.Solve:");
        loadLog();//загрузить log на форму
        QCoreApplication::processEvents();//прерывание интерфейса
        switch (ini.calcmode)
        {
        case 0://EQ_SCREW
            toLogf("ClassicEquivalent()...");  loadLog();  QCoreApplication::processEvents();
            if(hc->mrtilt)
            hc->mr->ClassicEquivalent( ini.mr.Xa, ini.mr.Ya, 0, 0 );//_Xa, _Ya, _eta, _chi
            else
            hc->mr->ClassicEquivalent( ini.mr.Xa, ini.mr.Ya, ini.mr.eta_*M_PI/180., ini.mr.chi_*M_PI/180. );//_Xa, _Ya, _eta, _chi
            if (hc->mr->warning) toLogf("warning=" + ss.setNum(hc->mr->warning));
            else toLogf("ok");
            break;
        case 1://FORCES
            toLogf("ClassicSpecified()...");  loadLog();  QCoreApplication::processEvents();
            if(hc->mrtilt)
            hc->mr->ClassicSpecified( hc->alfa_mr, ini.mr.fi0_*M_PI/180., 0, 0 );//_alpha, _fi0, _eta, _chi
            else
            hc->mr->ClassicSpecified( hc->alfa_mr, ini.mr.fi0_*M_PI/180., ini.mr.eta_*M_PI/180., ini.mr.chi_*M_PI/180. );//_alpha, _fi0, _eta, _chi
            if (hc->mr->warning) toLogf("warning=" + ss.setNum(hc->mr->warning));
            else toLogf("ok");
            break;
        default://
            toLogf("вид расчета винта несущего не выбран.\n");
            ok = false;
            break;
        }
        if(ok)
        {
            toLogf("ClassicForceSynth()...");  loadLog();  QCoreApplication::processEvents();
            hc->mr->ClassicForceSynth();
            //вывод сообщений
            toLogf("ok");
            toLogf(hc->mr->logstr);
            //вывод результатов
            report += hc->mr->Res2Str(ini.logkey);
            writeReport( fname_mrr, report );
            toLogf("MainRotor.Results: \nsaved to " + fname_mrr);
            toLogf(singleline);
        }
    }
    //TailRotor.Solve///
    if(ini.trsolve)
    {
        toLogf("TailRotor.Prepare:");
        report = hc->tr->Pre2Str(ini.logkey);
        toLogf("ok");   //toLogf(report);
        toLogf("TailRotor.Solve:");
        loadLog();//загрузить log на форму
        QCoreApplication::processEvents();//прерывание интерфейса
        switch (ini.calcmode)
        {
        case 0://EQ_SCREW
            toLogf("ClassicEquivalent()...");  loadLog();  QCoreApplication::processEvents();
            hc->tr->ClassicEquivalent( hc->alfa_tr, ini.tr.Fy_ );//ini.tr.alfa_*M_PI/180.
            if (hc->tr->warning) toLogf("warning=" + ss.setNum(hc->tr->warning));
            else toLogf("ok");
            break;
        case 1://FORCES
            toLogf("ClassicSpecified()...");  loadLog();  QCoreApplication::processEvents();
            hc->tr->ClassicSpecified( hc->alfa_tr, ini.tr.fi0_*M_PI/180. );//ini.tr.alfa_*M_PI/180.
            if (hc->tr->warning) toLogf("warning=" + ss.setNum(hc->tr->warning));
            else toLogf("ok");//
            break;
        default://
            toLogf("вид расчета винта рулевого не выбран.\n");
            ok = false;
            break;
        }
        if(ok)
        {
            toLogf("ClassicForceSynth()...");  loadLog();  QCoreApplication::processEvents();
            hc->tr->ClassicForceSynth();
            //вывод сообщений
            toLogf("ok");
            toLogf(hc->tr->logstr);
            //вывод результатов
            report += hc->tr->Res2Str(ini.logkey);
            writeReport( fname_trr, report );
            toLogf("TailRotor.Results: \nsaved to " + fname_trr );
            toLogf(singleline);
        }
    }
    //PushRotor.Solve///
    if(ini.prsolve)
    {
        toLogf("PushRotor.Prepare:");
        report = hc->pr->Pre2Str(ini.logkey);
        toLogf("ok");   //toLogf(report);
        toLogf("PushRotor.Solve:");
        loadLog();//загрузить log на форму
        QCoreApplication::processEvents();//прерывание интерфейса
        switch (ini.calcmode)
        {
        case 0://EQ_SCREW
            toLogf("ClassicEquivalent()...");  loadLog();  QCoreApplication::processEvents();
            hc->pr->ClassicEquivalent( hc->alfa_pr, ini.pr.Fy_ );
            if (hc->pr->warning) toLogf("warning=" + ss.setNum(hc->pr->warning));
            else toLogf("ok");
            break;
        case 1://FORCES
            toLogf("ClassicSpecified()...");  loadLog();  QCoreApplication::processEvents();
            hc->pr->ClassicSpecified( hc->alfa_pr, ini.pr.fi0_*M_PI/180. );
            if (hc->pr->warning) toLogf("warning=" + ss.setNum(hc->pr->warning));
            else toLogf("ok");//
            break;
        default://
            toLogf("вид расчета винта толкающего не выбран.\n");
            ok = false;
            break;
        }
        if(ok)
        {
            toLogf("ClassicForceSynth()...");  loadLog();  QCoreApplication::processEvents();
            hc->pr->ClassicForceSynth();
            //вывод сообщений
            toLogf("ok");
            toLogf(hc->pr->logstr);
            //вывод результатов
            report += hc->pr->Res2Str(ini.logkey);
            writeReport( fname_prr, report );
            toLogf("PushRotor.Results: \nsaved to " + fname_prr );
            toLogf(singleline);
        }
    }
    //вывод файлов состояния
    if(ok)
    {
        //заполнение структуры результатов по азимуту
        out.mr.solve = ini.mrsolve;
        out.tr.solve = ini.trsolve;
        out.pr.solve = ini.prsolve;
        for (int az = 0; az <= ini.nAz; az++)
        {
            if(ini.mrsolve) hc->mr->fillOutAz(out,az);
            if(ini.trsolve) hc->tr->fillOutAz(out,az);
            if(ini.prsolve) hc->pr->fillOutAz(out,az);
            write_helicopter_state( ini.datafolder + ss.setNum(az) );//запись в файл состояния
        }
        if (!ini.nAz)  toLogf("Warning: Файлы состояния не записаны!");
    }
    //фиксация текущих даты-времени и общего времени расчета
    QDate finishdate = QDate::currentDate();//фиксация текущего времени
    QTime finishtime = QTime::currentTime();//фиксация текуще date
    QString elapsed = getElapsedTime(tm);
    toLogf("Finished: " + finishdate.toString("yyyy.MM.dd") + " " + finishtime.toString());
    toLogf("Elapsed : " + elapsed);
}
void MainWindow::setPlanerInitial()//передача исходных данных в объект airframe
{
    hc->logkey    = ini.logkey;//выводить подробное состояние расчета
    hc->autogyro  = ini.autogyro;
    //ПАРАМЕТРЫ ПОЛЕТА
    hc->g         = ini.g;
    hc->Lpt       = ini.Lpt;
    hc->RjmK      = ini.RjmK;
    hc->RjkgK     = ini.RjkgK;
    hc->Mm        = ini.Mm;
    hc->ro        = ini.ro;
    hc->p0        = ini.p0;
    hc->t0        = ini.t0;
    hc->Hh        = ini.Hy;
    hc->mh        = ini.mH;
    hc->org.x = hc->org_.x = ini.Hx;
    hc->org.y = hc->org_.y = ini.Hy;
    hc->org.z = hc->org_.z = ini.Hz;
    hc->org.k = hc->org_.k = ini.kren_*M_PI/180.;
    hc->org.r = hc->org_.r = ini.rysk_*M_PI/180.;
    hc->org.t = hc->org_.t = ini.tang_*M_PI/180.;
    hc->v_.x      = ini.vHx;//= hc->Vhx
    hc->v_.y      = ini.vHy;//= hc->Vhy
    hc->v_.z      = ini.vHz;//= hc->Vhz
    hc->w_.x      = ini.wHx;
    hc->w_.y      = ini.wHy;
    hc->w_.z      = ini.wHz;
    hc->xctmr     = ini.xctmr*0.001;
    hc->yctmr     = ini.yctmr*0.001;
    hc->zctmr     = ini.zctmr*0.001;
    //ФЮЗЕЛЯЖ
    hc->solvePlaner= ini.solvePlaner;//учитывать аэродинамические нагрузки
    hc->x_f       = ini.x_f  *0.001;
    hc->y_f       = ini.y_f  *0.001;
    hc->z_f       = ini.z_f  *0.001;
    hc->Fx_f      = ini.Fx_f;
    hc->Fy_f      = ini.Fy_f;
    hc->Fz_f      = ini.Fz_f;
    hc->Mx_f      = ini.Mx_f;
    hc->My_f      = ini.My_f;
    hc->Mz_f      = ini.Mz_f;
    //СТАБИЛИЗАТОР
    hc->solveStab = ini.solveStab;//учитывать аэродинамические нагрузки
    hc->x_st      = ini.x_st *0.001;
    hc->y_st      = ini.y_st *0.001;
    hc->z_st      = ini.z_st *0.001;
    hc->Fx_st     = ini.Fx_st;
    hc->Fy_st     = ini.Fy_st;
    //КИЛЬ
    hc->solveKeel = ini.solveKeel;//учитывать аэродинамические нагрузки
    hc->x_k       = ini.x_k  *0.001;//Растояние от оси вращения несущего винта до центра давления киля (25% хорды bсах), м
    hc->y_k       = ini.y_k  *0.001;
    hc->z_k       = ini.z_k  *0.001;
    hc->Fx_keel   = ini.Fx_keel;
    hc->Fz_keel   = ini.Fz_keel;
    //КРЫЛО
    hc->solveWing = ini.solveWing;//учитывать аэродинамические нагрузки
    hc->x_w       = ini.x_w  *0.001;
    hc->y_w       = ini.y_w  *0.001;
    hc->z_w       = ini.z_w  *0.001;
    hc->Fx_wing   = ini.Fx_wing;
    hc->Fy_wing   = ini.Fy_wing;
    if(ini.solvePlaner)
    {
        int i;
        //ПАРАМЕТРЫ модели
        hc->lmodel = ini.lmodel;//0.76;//Линейный размер на модели, м
        hc->Smodel = ini.Smodel;//0.012;//Площадь миделя модели, м2
        hc->Mmodel = ini.Mmodel;//16.45;//Масштаб модели
        ///ФЮЗЕЛЯЖ
        hc->_alfa_f_col  = ini._alfa_f_col;
        i=0;  while(i < ini._alfa_f_col) {  hc->_alfa_f[i]= ini._alfa_f[i] *M_PI/180.;  i++;  }
        i=0;  while(i < ini._cx_f_col )  {  hc->_cx_f[i]  = ini._cx_f[i];  i++;  }
        i=0;  while(i < ini._cy_f_col )  {  hc->_cy_f[i]  = ini._cy_f[i];  i++;  }
        i=0;  while(i < ini._cz_f_col )  {  hc->_cz_f[i]  = ini._cz_f[i];  i++;  }
        i=0;  while(i < ini._mx_f_col )  {  hc->_mx_f[i]  = ini._mx_f[i];  i++;  }
        i=0;  while(i < ini._my_f_col )  {  hc->_my_f[i]  = ini._my_f[i];  i++;  }
        i=0;  while(i < ini._mz_f_col )  {  hc->_mz_f[i]  = ini._mz_f[i];  i++;  }

        ///КРЫЛО
        hc->S_wing        = ini.S_wing;
        hc->l_wing        = ini.l_wing;
        hc->K_wing        = ini.K_wing;
        hc->eps_wing      = ini.eps_wing*M_PI/180.;
        hc->_alfa_wing_col= ini._alfa_wing_col;
        i=0;  while(i < ini._cx_wing_col )  {  hc->_cx_wing[i]   = ini._cx_wing[i];  i++;  }
        i=0;  while(i < ini._cy_wing_col )  {  hc->_cy_wing[i]   = ini._cy_wing[i];  i++;  }
        i=0;  while(i < ini._alfa_wing_col) {  hc->_alfa_wing[i] = ini._alfa_wing[i]*M_PI/180.;  i++;  }

        ///КИЛЬ
        if(ini.solveKeel)
        {
            hc->S_keel        = ini.S_keel;
            hc->K_keel        = ini.K_keel;
            hc->eps_keel      = ini.eps_keel*M_PI/180.;
            hc->_alfa_keel_col= ini._alfa_keel_col;
            i=0;  while(i < ini._cx_keel_col )  {  hc->_cx_keel[i]   = ini._cx_keel[i];  i++;  }
            i=0;  while(i < ini._cy_keel_col )  {  hc->_cy_keel[i]   = ini._cy_keel[i];  i++;  }
            i=0;  while(i < ini._alfa_keel_col) {  hc->_alfa_keel[i] = ini._alfa_keel[i]*M_PI/180.;  i++;  }
        }

        ///СТАБИЛИЗАТОР
        hc->est_min      = ini.est_min*M_PI/180.;//
        hc->est_max      = ini.est_max*M_PI/180.;//
        hc->S_st         = ini.S_st;
        hc->l_st         = ini.l_st;
        hc->K_st         = ini.K_st;
        hc->__alfa_f_col = ini.__alfa_f_col;
        hc->__eps_st_col = ini.__eps_st_col;
        i=0;  while(i < ini._mz_fst_col*ini._mz_fst_row ) {  hc->_mz_fst[i]  = ini._mz_fst[i];  i++;  }
        i=0;  while(i < ini.__cx_st_col*ini.__cx_st_row ) {  hc->__cx_st[i]  = ini.__cx_st[i];  i++;  }
        i=0;  while(i < ini.__cy_st_col*ini.__cy_st_row ) {  hc->__cy_st[i]  = ini.__cy_st[i];  i++;  }
        i=0;  while(i < ini.__mz_st_col*ini.__mz_st_row ) {  hc->__mz_st[i]  = ini.__mz_st[i];  i++;  }
        i=0;  while(i < ini.__alfa_f_col) {  hc->__alfa_f[i] = ini.__alfa_f[i]*M_PI/180.;  hc->__eps_st[i] = ini.__eps_st[i] *M_PI/180.;   i++;  }
        switch (ini.solveStab)
        {
        case 1:
            hc->_alfa_st_col = ini._alfa_st_col;
            hc->_alfa_fs_col = ini._alfa_fs_col;
            i=0;  while(i < ini._cx_st_col ) {  hc->_cx_st[i]  = ini._cx_st[i];  i++;  }
            i=0;  while(i < ini._cy_st_col ) {  hc->_cy_st[i]  = ini._cy_st[i];  i++;  }
            i=0;  while(i < ini._alfa_st_col) {  hc->_alfa_st[i] = ini._alfa_st[i]*M_PI/180.;  i++;  }
            i=0;  while(i < ini._alfa_fs_col) {  hc->_alfa_fs[i] = ini._alfa_fs[i]*M_PI/180.;  hc->_dalf_stf[i] = ini._dalf_stf[i] *M_PI/180.;  i++;  }
            break;
        //case 2:
        //    hc->__eps_st0 = ini.__eps_st0;//временные массивы используемые в расчете StabilizerForces()
        //    break;
        }
    }
}
void MainWindow::setRotorsInitial()//передача исходных данных в объект rotor
{
    ///НВ////////////////////////////////////////////////////////////////////////////////
    hc->mrsolve  = ini.mrsolve;
    hc->mrtilt   = ini.mrtilt;
    hc->Dmr      = ini.mr.D *0.001;//используется только для отображения
    hc->x_mr     = ini.mr.Ox*0.001;
    hc->y_mr     = ini.mr.Oy*0.001;
    hc->z_mr     = ini.mr.Oz*0.001;
    hc->Fx_mr_   = hc->Fx_mr = ini.mr.Fx_;
    hc->Fy_mr_   = hc->Fy_mr = ini.mr.Fy_;
    hc->Fz_mr_   = hc->Fz_mr = ini.mr.Fz_;
    hc->Mx_mr_   = hc->Mx_mr = ini.mr.Mx_;
    hc->My_mr_   = hc->My_mr = ini.mr.My_;
    hc->Mz_mr_   = hc->Mz_mr = ini.mr.Mz_;
    hc->vs_mr_   = ini.mr.vs_;
    hc->skos_mr_ = ini.mr.skos*M_PI/180.;
    //MainRotor для балансировки
    hc->w_mr     = ini.mr.rpm*M_PI/30.0;//187/7.25;//угловая скорость вращения НВ при заданом режиме, рад/с. 1/с
    hc->zl_mr    = ini.mr.zl          ;//3;//Количество лопастей несущего винта
    hc->Bmr      = ini.mr.Bkp         ;//0.97;//Коэф. концевых потерь
    hc->k_       = ini.mr.kvz         ;//0.4;//Компенсатор взмаха
    hc->Rap      = ini.mr.Rap         ;//0.265;//радиус крепления тяг, м
    hc->ll       = ini.mr.ll          ;//0.157;////////////
    hc->lgsh     = ini.mr.l_gsh *0.001;//0.102;//Расстояние от оси несущего винта до оси ГШ, м
    hc->lvsh     = ini.mr.l_vsh *0.001;//0.262;//разнос вертикальных шарниров, м
    hc->dgsh     =-ini.mr.s_vsh *0.001;//вынос горизонтального шарнира вперед, м
    //hc->losh     = ini.mr.l_osh *0.001;//разнос осевых шарниров, м
    hc->dpsi_cont= ini.mr.dpsi_cont*M_PI/180.;//угол опережения управления, град
    hc->Sgsh_mr  = ini.mr.Sb0 - ini.mr.mlp*(0.5*ini.mr.D-ini.mr.l_osh)*0.001*hc->lgsh;//приближенно Стат. момент инерции лопасти относительно г.ш, N*m
    hc->D1_mr_   = ini.mr.D1_;
    hc->D2_mr_   = ini.mr.D2_;
    hc->fi0_min  = ini.fi0_min*M_PI/180.;//
    hc->fi0_max  = ini.fi0_max*M_PI/180.;//
    hc->nstabrev = ini.nstabrev;
    if(ini.mrsolve)
    {
        hc->mr = new MainRotor();
        hc->mr->hubtype     = ini.mr.hubtype;
        hc->mr->rdirect     = ini.mr.rdirect;
        hc->mr->warning     = 0;
        hc->mr->logstr      = "";
        hc->mr->logkey      = ini.logkey;
        hc->mr->prec        = ini.rotprec;
        hc->mr->dt          = ini.dt;
        hc->mr->nAz         = ini.nAz;//Кол-во шагов лопасти по азимуту
        hc->mr->g           = ini.g;
        hc->mr->ro          = ini.ro;
        hc->mr->p0          = ini.p0;//
        hc->mr->t0          = ini.t0;//
        hc->mr->Hy          = ini.Hy; //Высота полета, м
        hc->mr->zl          = ini.mr.zl;//reader->readInt("z" + _suff);
        hc->mr->rpmORfi0  = ini.mr.rpmORfi0;
        hc->mr->rpm = hc->rpm_mr = ini.mr.rpm;
        hc->mr->fi0         = ini.mr.fi0_ *M_PI/180.;//только для Pre2Str()
        hc->mr->mlp         = ini.mr.mlp;//reader->readFloat("ml" + _suff);
        hc->mr->l_gsh       = ini.mr.l_gsh*0.001;//Расстояние от оси несущего винта до оси ГШ, м
        hc->mr->l_vsh       = ini.mr.l_vsh*0.001;
        hc->mr->s_vsh       = ini.mr.s_vsh*0.001;
        hc->mr->Iop         = ini.mr.Iop;//
        hc->mr->Ib0         = ini.mr.Ib0;//Момент инерции лопасти отн.оси винта, кг*м^2
        hc->mr->Sb0         = ini.mr.Sb0;//Стат. момент инерции лопасти отн.оси винта, N*m
        hc->mr->dCxp        = ini.mr.dCxp;   //Поправка к коэф. профильного сопротивления
        hc->mr->Bkp         = ini.mr.Bkp;    //0.97;//Коэф. концевых потерь reader->readFloat("B" + _suff);
        hc->mr->_k          = ini.mr.kvz;    //0.4;//Коэф. компенсатора взмаха reader->readFloat("_k" + _suff);
        hc->mr->D1          = ini.mr.D1_;
        hc->mr->D2          = ini.mr.D2_;
        hc->mr->alpha_min   = ini.mr.alfa_min*M_PI/180.;//Диапазон угла атаки
        hc->mr->alpha_max   = ini.mr.alfa_max*M_PI/180.;//Диапазон угла атаки
        hc->mr->fi0_min     = ini.mr.fi0_min *M_PI/180.;//Диапазон общего шага
        hc->mr->fi0_max     = ini.mr.fi0_max *M_PI/180.;//Диапазон общего шага
        hc->mr->rpm_min     = ini.mr.rpm_min;
        hc->mr->rpm_max     = ini.mr.rpm_max;
        hc->mr->bet_max     = ini.mr.bet_max *M_PI/180.;//предельный угол взмаха лопасти отн.ГШ вверх, рад;
        hc->mr->bet_min     = ini.mr.bet_min *M_PI/180.;//предельный угол взмаха лопасти отн.ГШ вниз, рад;
        hc->mr->ksi_max     = ini.mr.ksi_max *M_PI/180.;//предельный угол поворота лопасти отн.ВШ вперед, рад;
        hc->mr->ksi_min     = ini.mr.ksi_min *M_PI/180.;//предельный угол поворота лопасти отн.ВШ назад, рад;
        hc->mr->gam_max     =  20*M_PI/180.;//ini.mr.gam_max *M_PI/180.;//предельный угол     лопасти отн.OШ, рад;
        hc->mr->gam_min     = -20*M_PI/180.;//ini.mr.gam_min *M_PI/180.;//предельный угол     лопасти отн.OШ, рад;

        hc->mr->xo          = ini.mr.xo       ;//
        hc->mr->xt          = ini.mr.xt       ;//
        //hc->mr->xf          = ini.mr.xf       ;//
        //hc->mr->stiff1      = ini.mr.stiff1 ;//
        //hc->mr->stiff2      = ini.mr.stiff2 ;//
        hc->mr->r_pov       = ini.mr.r_pov *0.001;//
        hc->mr->k_pov       = ini.mr.k_pov *1000;//
        hc->mr->f_pod       = ini.mr.f_pod      ;//
        hc->mr->r_pod       = ini.mr.r_pod *0.001;//
        hc->mr->sigma3      = ini.mr.sigma3   ;//

            if(hc->mrtilt)
            {
                hc->mr->chi = 0;
                hc->mr->eta = 0;
            }
            else
            {
                hc->mr->chi = ini.mr.chi_    *M_PI/180.;///только для Pre2Str()
                hc->mr->eta = ini.mr.eta_    *M_PI/180.;///только для Pre2Str()
            }
        hc->mr->Ya_need     = ini.mr.Ya ;    // Подъемная сила в скоростной с-ме координат, Н:
        hc->mr->Xa_need     = ini.mr.Xa ;    // Пропульсивная сила в скоростной с-ме координат, Н:
        ///демпфер лопасти НВ
        hc->mr->damper      = ini.mr.damper;//
        if(ini.mr.damper)
        {
        hc->mr->d.h         = ini.mr.d.h  *0.001   ;//плечо силы на демпфере
        hc->mr->d.x2        = ini.mr.d.x2 *0.001   ;//коорд.шарнира демпфера на втулке отн.оси НВ
        hc->mr->d.z2        = ini.mr.d.z2 *0.001   ;//коорд.шарнира демпфера на втулке отн.оси НВ
        hc->mr->d.x3        = ini.mr.d.x3 *0.001   ;//коорд.шарнира демпфера на лопасти отн.оси НВ
        hc->mr->d.z3        = ini.mr.d.z3 *0.001   ;//коорд.шарнира демпфера на лопасти отн.оси НВ
        hc->mr->d.ro        = ini.mr.d.ro          ;//плотность жидкости в демпфере, кг/м^3:
        hc->mr->d.mu        = ini.mr.d.mu          ;//коэф. истечения жидкости в демпфере
        hc->mr->d.f         = ini.mr.d.f  *0.000001;//суммарная площадь отверстий в демпфере, мм^2:
        hc->mr->d.f2        = ini.mr.d.f2 *0.000001;//суммарная площадь отверстий в демпфере, мм^2:
        hc->mr->d.dv        = ini.mr.d.dv *0.001   ;//диаметр штока демпфера, мм:
        hc->mr->d.dn        = ini.mr.d.dn *0.001   ;//диаметр цилиндра демпфера внутренний, мм:
        hc->mr->d.pmax      = ini.mr.d.pmax        ;//давление срабатывания клапана демпфера, кгс/см2:
        }
        ///ПАРАМЕТРЫ ЛОПАСТИ НВ
        hc->mr->r_is        = ini.r_mr;         //hc->mr->r_mr_col   = ini.r_mr_col  ;//Радиус, м:reader->readFloatArr ("r" + _suff, &r_is_[0]);
        hc->mr->bh_is       = ini.bh_mr;         //hc->mr->b_mr_col   = ini.b_mr_col  ;//Хорда, м:reader->readFloatArr ("b" + _suff, &b_is_[0]);
        hc->mr->dfi_is      = ini.dfi_mr;       //hc->mr->dfi_mr_col = ini.dfi_mr_col;//Крутка, градусы:reader->readFloatArr ("dfi" + _suff, &dfi_is_[0]);
        hc->mr->prof_is     = ini.prof_mr;      //hc->mr->jSeq_isx = ini.prof_mr_col;///reader->readIntArr_ ("prof" + _suff, &prof_is[0], &jSeq_isx);
        //hc->mr->kh_bl     = ini.kh_mr;
        ///ХАРАКТЕРИСТИКИ СЕЧЕНИЙ ЛОПАСТИ НВ
        for (int ii = 0; ii < ini.ProfilesAmount; ii++)
        {
            hc->mr->profiles[ii] = ini.profiles[ii];
            hc->mr->nA[ii][0] = ini.nA[ii][0]; //для Су
            hc->mr->nA[ii][1] = ini.nA[ii][1]; //для Сх
            hc->mr->nA[ii][2] = ini.nA[ii][2]; //для Mz
            hc->mr->nM[ii][0] = ini.nM[ii][0]; //для Су
            hc->mr->nM[ii][1] = ini.nM[ii][1]; //для Сх
            hc->mr->nM[ii][2] = ini.nM[ii][2]; //для Mz
            for (int ca = 0; ca < Ca; ca++)
            {
                hc->mr->ALPHA[ii][ca][0] = ini.ALPHA[ii][ca][0] *M_PI/180.;
                hc->mr->ALPHA[ii][ca][1] = ini.ALPHA[ii][ca][1] *M_PI/180.;
                hc->mr->ALPHA[ii][ca][3] = ini.ALPHA[ii][ca][3] *M_PI/180.;
            }
            for (int cm = 0; cm < CM; cm++)
            {
                hc->mr->MACH[ii][cm][0] = ini.MACH[ii][cm][0];
                hc->mr->MACH[ii][cm][1] = ini.MACH[ii][cm][1];
                hc->mr->MACH[ii][cm][2] = ini.MACH[ii][cm][2];
                for (int ca = 0; ca < Ca; ca++)
                {
                    hc->mr->_Cy[ii][cm][ca] = ini._Cy[ii][cm][ca];
                    hc->mr->_Cx[ii][cm][ca] = ini._Cx[ii][cm][ca];
                    hc->mr->_Mz[ii][cm][ca] = ini._Mz[ii][cm][ca];
                }
            }

        }//   */
        //hc->mr->vgsolve = ini.mr.vgsolve ;//
        //if(ini.mr.vgsolve)
        //{
        //    hc->mr->vg.n = ini.mr.vg.n        ;//
        //    hc->mr->vg.m = ini.mr.vg.m        ;//
        //    hc->mr->vg.h = ini.mr.vg.h  *0.001;//
        //    hc->mr->vg.d = ini.mr.vg.d  *0.001;//
        //    hc->mr->vg.D = ini.mr.vg.D  *0.001;//
        //    //hc->mr->M0vnesh =   ini.mr.vg.M0;
        //}
    }

    ///РВ////////////////////////////////////////////////////////////////////////////////
    out.tr.mount = ini.TRmount;
    hc->trmount  = ini.TRmount;
    if(ini.TRmount)
    {
    hc->trsolve = ini.trsolve;
    hc->Dtr     = ini.tr.D *0.001;
    hc->x_tr    = ini.tr.Ox*0.001;
    hc->y_tr    = ini.tr.Oy*0.001;
    hc->z_tr    = ini.tr.Oz*0.001;
    hc->Fx_tr_  = hc->Fx_tr = ini.tr.Fx_;
    hc->Fy_tr_  = hc->Fy_tr = ini.tr.Fy_;
    hc->Fz_tr_  = hc->Fz_tr = ini.tr.Fz_;
    hc->Mx_tr_  = hc->Mx_tr = ini.tr.Mx_;
    hc->My_tr_  = hc->My_tr = ini.tr.My_;
    hc->Mz_tr_  = hc->Mz_tr = ini.tr.Mz_;
    hc->u_tr_   = ini.tr.vt_;
    hc->skos_tr_= ini.tr.skos*M_PI/180.;
    if(ini.trsolve)
    {
        hc->tr = new TailRotor();
        hc->tr->hubtype     = ini.tr.hubtype;
        hc->tr->rdirect     = ini.tr.rdirect;
        hc->tr->warning     = 0;
        hc->tr->logstr      = "";
        hc->tr->logkey      = ini.logkey;
        hc->tr->prec        = ini.rotprec;
        hc->tr->dt          = ini.dt;
        hc->tr->nAz         = ini.nAz;//Кол-во шагов лопасти по азимуту
        hc->tr->g           = ini.g ;
        hc->tr->ro          = ini.ro;
        hc->tr->p0          = ini.p0;//
        hc->tr->t0          = ini.t0;//
        hc->tr->Hy          = ini.Hy; //Высота полета, м
        hc->tr->zl          = ini.tr.zl;//reader->readInt("z" + _suff);
        hc->tr->R           = 0.0005*ini.tr.D;//Радиус несущего винта, м
        hc->tr->rpmORfi0  = ini.tr.rpmORfi0;
        hc->tr->rpm = hc->rpm_tr = ini.tr.rpm;
        hc->tr->fi0         = ini.tr.fi0_ *M_PI/180.;
        hc->tr->mlp         = ini.tr.mlp;//reader->readFloat("ml" + _suff);
        hc->tr->l_gsh       = ini.tr.l_gsh*0.001;//Расстояние от оси несущего винта до оси ГШ, м
        hc->tr->l_vsh       = ini.tr.l_vsh*0.001;//ini.mr.l_vsh*0.001;
        hc->tr->s_vsh       = ini.tr.s_vsh*0.001;
        hc->tr->l_osh       = ini.tr.l_osh*0.001;//разнос осевых шарниров, м
        hc->tr->Iop         = ini.tr.Iop;//
        hc->tr->Ib0         = ini.tr.Ib0;//Момент инерции лопасти отн.оси винта, кг*м^2
        hc->tr->Sb0         = ini.tr.Sb0;//197;//Стат. момент инерции лопасти отн.оси винта, N*m
        hc->tr->dCxp        = ini.tr.dCxp;//Поправка к коэф. профильного сопротивления
        hc->tr->Bkp         = ini.tr.Bkp;//0.97;//Коэф. концевых потерь reader->readFloat("B" + _suff);
        hc->tr->_k          = ini.tr.kvz ;//0.4;//Коэф. компенсатора взмаха reader->readFloat("_k" + _suff);
        hc->Ttr_need        = ini.tr.Fy_;//reader->readFloat("T_" + suffix + "=");
        hc->tr->bet_max     = ini.tr.bet_max *M_PI/180.;//предельный угол взмаха лопасти отн.ГШ вверх, рад;
        hc->tr->bet_min     = ini.tr.bet_min *M_PI/180.;//предельный угол взмаха лопасти отн.ГШ вниз, рад;
        hc->tr->ksi_max     = ini.tr.ksi_max *M_PI/180.;//предельный угол поворота лопасти отн.ВШ вперед, рад;
        hc->tr->ksi_min     = ini.tr.ksi_min *M_PI/180.;//предельный угол поворота лопасти отн.ВШ назад, рад;
        hc->tr->fi0_min     = ini.tr.fi0_min *M_PI/180.;//Диапазон общего шага
        hc->tr->fi0_max     = ini.tr.fi0_max *M_PI/180.;//Диапазон общего шага
        hc->tr->rpm_min     = ini.tr.rpm_min;
        hc->tr->rpm_max     = ini.tr.rpm_max;
        hc->tr->gam_max     =  20*M_PI/180.;//ini.mr.gam_max *M_PI/180.;//предельный угол     лопасти отн.OШ, рад;
        hc->tr->gam_min     = -20*M_PI/180.;//ini.mr.gam_min *M_PI/180.;//предельный угол     лопасти отн.OШ, рад;
        hc->tr->xo          = ini.tr.xo       ;//
        hc->tr->xt          = ini.tr.xt       ;//
        //hc->tr->xf          = ini.tr.xf       ;//
        //hc->tr->stiff1      = ini.tr.stiff1 ;//
        //hc->tr->stiff2      = ini.tr.stiff2 ;//
        hc->tr->r_pov       = ini.tr.r_pov *0.001;//
        hc->tr->k_pov       = ini.tr.k_pov *1000;//
        hc->tr->f_pod       = ini.tr.f_pod       ;//
        hc->tr->r_pod       = ini.tr.r_pod *0.001;//
        hc->tr->sigma3      = ini.tr.sigma3   ;//

        ///ПАРАМЕТРЫ ЛОПАСТИ РВ
        hc->tr->r_is        = ini.r_tr;         //hc->mr->r_mr_col   = ini.r_mr_col  ;//Радиус, м:reader->readFloatArr ("r" + _suff, &r_is_[0]);
        hc->tr->bh_is       = ini.bh_tr;        //hc->mr->b_mr_col   = ini.b_mr_col  ;//Хорда, м:reader->readFloatArr ("b" + _suff, &b_is_[0]);
        hc->tr->dfi_is      = ini.dfi_tr;       //hc->mr->dfi_mr_col = ini.dfi_mr_col;//Крутка, градусы:reader->readFloatArr ("dfi" + _suff, &dfi_is_[0]);
        hc->tr->prof_is     = ini.prof_tr;      //hc->mr->jSeq_isx = ini.prof_mr_col;///reader->readIntArr_ ("prof" + _suff, &prof_is[0], &jSeq_isx);
        //hc->tr->kh_bl     = ini.kh_tr;
        ///ХАРАКТЕРИСТИКИ СЕЧЕНИЙ ЛОПАСТИ РВ
        for (int ii = 0; ii < ini.ProfilesAmount; ii++)
        {
            hc->tr->profiles[ii] = ini.profiles[ii];
            hc->tr->nA[ii][0] = ini.nA[ii][0]; //для Су
            hc->tr->nA[ii][1] = ini.nA[ii][1]; //для Сх
            hc->tr->nA[ii][2] = ini.nA[ii][2]; //для Mz
            hc->tr->nM[ii][0] = ini.nM[ii][0]; //для Су
            hc->tr->nM[ii][1] = ini.nM[ii][1]; //для Сх
            hc->tr->nM[ii][2] = ini.nM[ii][2]; //для Mz
            for (int ca = 0; ca < Ca; ca++)
            {
                hc->tr->ALPHA[ii][ca][0] = ini.ALPHA[ii][ca][0] *M_PI/180.;
                hc->tr->ALPHA[ii][ca][1] = ini.ALPHA[ii][ca][1] *M_PI/180.;
                hc->tr->ALPHA[ii][ca][3] = ini.ALPHA[ii][ca][3] *M_PI/180.;
            }
            for (int cm = 0; cm < CM; cm++)
            {
                hc->tr->MACH[ii][cm][0] = ini.MACH[ii][cm][0];
                hc->tr->MACH[ii][cm][1] = ini.MACH[ii][cm][1];
                hc->tr->MACH[ii][cm][2] = ini.MACH[ii][cm][2];
                for (int ca = 0; ca < Ca; ca++)
                {
                    hc->tr->_Cy[ii][cm][ca] = ini._Cy[ii][cm][ca];
                    hc->tr->_Cx[ii][cm][ca] = ini._Cx[ii][cm][ca];
                    hc->tr->_Mz[ii][cm][ca] = ini._Mz[ii][cm][ca];
                }
            }
        }//
        hc->tr->damper = ini.tr.damper;//
    }
    }
    else
    {
        hc->trsolve= 0;
        hc->Dtr    = 0;
        hc->x_tr   = 0;
        hc->y_tr   = 0;
        hc->z_tr   = 0;
        hc->Fx_tr_ = 0;
        hc->Fy_tr_ = 0;
        hc->Fz_tr_ = 0;
        hc->Mx_tr_ = 0;
        hc->My_tr_ = 0;
        hc->Mz_tr_ = 0;
        hc->u_tr_  = 0;
    }

    ///TВ////////////////////////////////////////////////////////////////////////////////
    out.pr.mount = ini.PRmount;
    hc->prmount  = ini.PRmount;
    if(ini.PRmount)
    {
        hc->prsolve  = ini.prsolve;
        hc->Dpr      = ini.pr.D *0.001;
        hc->x_pr     = ini.pr.Ox*0.001;
        hc->y_pr     = ini.pr.Oy*0.001;
        hc->z_pr     = ini.pr.Oz*0.001;
        hc->Fx_pr_   = hc->Fx_pr = ini.pr.Fx_;
        hc->Fy_pr_   = hc->Fy_pr = ini.pr.Fy_;
        hc->Fz_pr_   = hc->Fz_pr = ini.pr.Fz_;
        hc->Mx_pr_   = hc->Mx_pr = ini.pr.Mx_;
        hc->My_pr_   = hc->My_pr = ini.pr.My_;
        hc->Mz_pr_   = hc->Mz_pr = ini.pr.Mz_;
        hc->Kt_pr    = ini.pr.Kt;
        hc->u_pr_    = ini.pr.vt_;
        hc->Tpr_need        = ini.pr.Fy_;//reader->readFloat("T_" + suffix + "=");
        if(ini.prsolve)
        {
            hc->pr = new PushRotor();
            hc->pr->hubtype     = ini.pr.hubtype;
            hc->pr->rdirect     = ini.pr.rdirect;
            hc->pr->warning     = 0;
            hc->pr->logstr      = "";
            hc->pr->logkey      = ini.logkey;
            hc->pr->prec        = ini.rotprec;
            hc->pr->dt          = ini.dt;
            hc->pr->g           = ini.g ;
            hc->pr->ro          = ini.ro;
            hc->pr->p0          = ini.p0;//
            hc->pr->t0          = ini.t0;//
            hc->pr->nAz         = ini.nAz;//Кол-во шагов лопасти по азимуту
            hc->pr->zl          = ini.pr.zl;//reader->readInt("z" + _suff);
            hc->pr->R           = 0.0005*ini.pr.D;//Радиус несущего винта, м
            hc->pr->rpmORfi0  = ini.pr.rpmORfi0;
            hc->pr->rpm = hc->rpm_pr = ini.pr.rpm;
            hc->pr->fi0         = ini.pr.fi0_ *M_PI/180.;
            hc->pr->mlp         = ini.pr.mlp;//reader->readFloat("ml" + _suff);
            hc->pr->l_gsh       = ini.pr.l_gsh*0.001;//Расстояние от оси несущего винта до оси ГШ, м
            hc->pr->l_vsh       = ini.pr.l_vsh*0.001;//ini.mr.l_vsh*0.001;
            hc->pr->s_vsh       = 0;//ini.mr.s_vsh*0.001;
            hc->pr->Iop         = ini.pr.Iop  ;//
            hc->pr->Ib0         = ini.pr.Ib0;//Момент инерции лопасти отн.оси винта, кг*м^2
            hc->pr->Sb0         = ini.pr.Sb0;//197;//Стат. момент инерции лопасти отн.оси винта, N*m
            hc->pr->dCxp        = ini.pr.dCxp;//Поправка к коэф. профильного сопротивления
            hc->pr->Bkp         = ini.pr.Bkp;//0.97;//Коэф. концевых потерь reader->readFloat("B" + _suff);
            hc->pr->_k          = ini.pr.kvz ;//0.4;//Коэф. компенсатора взмаха reader->readFloat("_k" + _suff);
            hc->pr->Hy          = ini.Hy; //Высота полета, м
            hc->pr->bet_max     = ini.pr.bet_max *M_PI/180.;//предельный угол взмаха лопасти отн.ГШ вверх, рад;
            hc->pr->bet_min     = ini.pr.bet_min *M_PI/180.;//предельный угол взмаха лопасти отн.ГШ вниз, рад;
            hc->pr->ksi_max     = ini.pr.ksi_max *M_PI/180.;//предельный угол поворота лопасти отн.ВШ вперед, рад;
            hc->pr->ksi_min     = ini.pr.ksi_min *M_PI/180.;//предельный угол поворота лопасти отн.ВШ назад, рад;
            hc->pr->fi0_min     = ini.pr.fi0_min *M_PI/180.;//Диапазон общего шага
            hc->pr->fi0_max     = ini.pr.fi0_max *M_PI/180.;//Диапазон общего шага
            hc->pr->rpm_min     = ini.pr.rpm_min;
            hc->pr->rpm_max     = ini.pr.rpm_max;
            hc->pr->gam_max     = 20*M_PI/180.;//ini.mr.gam_max *M_PI/180.;//предельный угол     лопасти отн.OШ, рад;
            hc->pr->gam_min     =-20*M_PI/180.;//ini.mr.gam_min *M_PI/180.;//предельный угол     лопасти отн.OШ, рад;
            hc->pr->xo          = ini.pr.xo       ;//
            hc->pr->xt          = ini.pr.xt       ;//
            //hc->pr->xf          = ini.pr.xf       ;//
            hc->pr->r_pov       = ini.pr.r_pov *0.001;//
            hc->pr->k_pov       = ini.pr.k_pov *1000;//
            hc->pr->f_pod       = ini.pr.f_pod      ;//
            hc->pr->r_pod       = ini.pr.r_pod *0.001;//
            hc->pr->sigma3      = ini.pr.sigma3   ;//

            ///ПАРАМЕТРЫ ЛОПАСТИ TВ
            hc->pr->r_is        = ini.r_pr;         //hc->mr->r_mr_col   = ini.r_mr_col  ;//Радиус, м:reader->readFloatArr ("r" + _suff, &r_is_[0]);
            hc->pr->bh_is       = ini.bh_pr;        //hc->mr->b_mr_col   = ini.b_mr_col  ;//Хорда, м:reader->readFloatArr ("b" + _suff, &b_is_[0]);
            hc->pr->dfi_is      = ini.dfi_pr;       //hc->mr->dfi_mr_col = ini.dfi_mr_col;//Крутка, градусы:reader->readFloatArr ("dfi" + _suff, &dfi_is_[0]);
            hc->pr->prof_is     = ini.prof_pr;      //hc->mr->jSeq_isx = ini.prof_mr_col;///reader->readIntArr_ ("prof" + _suff, &prof_is[0], &jSeq_isx);
            //hc->pr->kh_bl     = ini.kh_pr;
            ///ХАРАКТЕРИСТИКИ СЕЧЕНИЙ ЛОПАСТИ TВ
            for (int ii = 0; ii < ini.ProfilesAmount; ii++)
            {
                hc->pr->profiles[ii] = ini.profiles[ii];
                hc->pr->nA[ii][0] = ini.nA[ii][0]; //для Су
                hc->pr->nA[ii][1] = ini.nA[ii][1]; //для Сх
                hc->pr->nA[ii][2] = ini.nA[ii][2]; //для Mz
                hc->pr->nM[ii][0] = ini.nM[ii][0]; //для Су
                hc->pr->nM[ii][1] = ini.nM[ii][1]; //для Сх
                hc->pr->nM[ii][2] = ini.nM[ii][2]; //для Mz
                for (int ca = 0; ca < Ca; ca++)
                {
                    hc->pr->ALPHA[ii][ca][0] = ini.ALPHA[ii][ca][0] *M_PI/180.;
                    hc->pr->ALPHA[ii][ca][1] = ini.ALPHA[ii][ca][1] *M_PI/180.;
                    hc->pr->ALPHA[ii][ca][3] = ini.ALPHA[ii][ca][3] *M_PI/180.;
                }
                for (int cm = 0; cm < CM; cm++)
                {
                    hc->pr->MACH[ii][cm][0] = ini.MACH[ii][cm][0];
                    hc->pr->MACH[ii][cm][1] = ini.MACH[ii][cm][1];
                    hc->pr->MACH[ii][cm][2] = ini.MACH[ii][cm][2];
                    for (int ca = 0; ca < Ca; ca++)
                    {
                        hc->pr->_Cy[ii][cm][ca] = ini._Cy[ii][cm][ca];
                        hc->pr->_Cx[ii][cm][ca] = ini._Cx[ii][cm][ca];
                        hc->pr->_Mz[ii][cm][ca] = ini._Mz[ii][cm][ca];
                    }
                }
            }//
            hc->pr->damper = ini.pr.damper;//
        }
    }
    else
    {
        hc->prsolve  = 0;
        hc->Dpr      = 0;
        hc->x_pr     = 0;
        hc->y_pr     = 0;
        hc->z_pr     = 0;
        hc->Fx_pr_   = 0;
        hc->Fy_pr_   = 0;
        hc->Fz_pr_   = 0;
        hc->Mx_pr_   = 0;
        hc->My_pr_   = 0;
        hc->Mz_pr_   = 0;
        hc->u_pr_    = 0;
    }

    hc->manual.fi0mr = ini.mr.fi0_*M_PI/180.;
    hc->manual.fi0tr = ini.tr.fi0_*M_PI/180.;
    hc->manual.fi0pr = ini.pr.fi0_*M_PI/180.;
    hc->manual.chimr = ini.mr.chi_*M_PI/180.;
    hc->manual.etamr = ini.mr.eta_*M_PI/180.;
    //установка заданных управляющих сигналов
    setManualControl(ini.tr.fi0_, ini.mr.fi0_, ini.mr.eta_, ini.mr.chi_);
    //установка номинальных целеуказателей
    setTargetControl(ini.tr.fi0_, ini.mr.fi0_, ini.mr.eta_, ini.mr.chi_);
}
void MainWindow::setManualControl(float fi0tr, float fi0mr, float etamr, float chimr)
{
    if(ini.trsolve)
    {
        ui->SpinBox_fi0tr->setValue( fi0tr );//*M_PI/180.
        ui->Slider_fi0tr->setValue( float2int(fi0tr*1000) );
    }
    if(ini.mrsolve)
    {
        ui->SpinBox_fi0mr->setValue( fi0mr );//*M_PI/180.
        ui->SpinBox_etamr->setValue( etamr );//*M_PI/180.
        ui->SpinBox_chimr->setValue( chimr );//*M_PI/180.
        ui->Slider_fi0mr->setValue( float2int(fi0mr*1000) );     //ui->Slider_etamr_->setRange( -10*1000, 10*1000 );
        ui->Slider_etamr->setValue( float2int(etamr*1000) ); //ui->Slider_etamr_->setValue( ceil(ini.mr.eta_*1000) );
        ui->Slider_chimr->setValue( float2int(chimr*1000) );
    }
}
void MainWindow::setTargetControl(float fi0tr, float fi0mr, float etamr, float chimr)
{
    int X,Y;
    if(ini.trsolve)
    {
    X = float2int( (250-11.5)*(fi0tr*1000 - ui->Slider_fi0tr->minimum())/(ui->Slider_fi0tr->maximum() - ui->Slider_fi0tr->minimum()));//ui->Slider_fi0tr->width()
    ui->nom_fitr->setGeometry(X-1,0,13,20);
    }
    if(ini.mrsolve)
    {
    Y = float2int( (250-11.5)*(ui->Slider_fi0mr->maximum() - fi0mr*1000)/(ui->Slider_fi0mr->maximum() - ui->Slider_fi0mr->minimum()));//ui->Slider_fi0mr-> height()
    ui->nom_fimr->setGeometry(0,Y-1,20,13);
    X = float2int( (250-11.5)*(ui->Slider_etamr->maximum() - etamr*1000)/(ui->Slider_etamr->maximum() - ui->Slider_etamr->minimum()));//ui->Slider_etamr->width()
    ui->nom_eta->setGeometry(X-1,0,13,20);
    Y = float2int( (250-11.5)*(chimr*1000 - ui->Slider_chimr->minimum())/(ui->Slider_chimr->maximum() - ui->Slider_chimr->minimum()));//ui->Slider_chimr->height()
    //Y = float2int( (250-11.5) *(ui->Slider_chimr->maximum()-chimr*1000)/(ui->Slider_chimr->maximum() - ui->Slider_chimr->minimum()));//ui->Slider_chimr->height()
    ui->nom_chi->setGeometry(0,Y-1,20,13);
    }
}



//// Б А Л А Н С И Р О В К А  В Е Р Т О Л Е Т А
void MainWindow::MainBalancing()//организатор расчета балансировки вертолета
{
    QDate currentdate;   QTime currenttime;   QString elapsed;
    ok = true;
    hc = 0;  delete hc;//удаление ранее рожденного объекта Helicopter (нужно при прерываниях расчета)
    hc = new Helicopter();//создаем объект Balancing
    hc->SetClassPointer();//инициализируем классу указатель на себя (нужно для статических функций)
    ui->tabWidget->setCurrentIndex(1);
    hc->setNull();
    SetBalanceInitial();//передаем исходные данные
    hc->autorot = false;//указатель режима авторотации
    hc->Mkr_ = ini.mr.My_;
    hc->nst = 0;//шасси выключено из расчета
    int q = 0;//счетчик итераций

    //выбор расчета
    switch (ini.calcmode)
    {
      case 2:///расчет сил и моментов в ЦТ вертолета по заданному управлению
        {
        //расчет вспомогательных параметров
        hc->PreBalance();
        //расчет и передача скоростей втулки НВ
        SetDataForRotors();///добавил 20211004
        loadLog();
        QCoreApplication::processEvents();
        hc->SolveSpecified(out);
        //показать состояние на 3Д модели
        if(ui->btn_infopanel->isChecked()) showCurrentInfo();
        write_to_opengl();
        //обработка результатов расчета
        hc->PostBalance();
        //запись результатов расчета в log-файл
        toLogf(hc->balancereport);
        //toLogf("\n");
        toLogf(singleline);
        }
        break;//case 2

      case 30:///определение Vу на авторотации по заданным Vx и крут.моменту на валу
        {
        hc->autorot  = true;
        hc->autogyro = false;
        ini.trsolve  = false;
        ini.prsolve  = false;
        QString solvout = "";
        //подготовка к расчету
        hc->PreBalance();
        toLogf("Autorotation.Velocity.Solve:");
        toLogf("расчет скорости снижения Vny... ");//режим самовращения   ...по заданным горизонтальной скорости и реактивному моменту НВ
        loadLog();
        //нулевое приближение балансировки
        ok = (hc->SolveBalance0())?(0):(1);

        if (hc->mrsolve)
        {
            double  x = -4;//vny_min,
            double dm = 111;
            int q = 0;
            while ( dm>0  && !isStop )
            {
                q++;
                x -= 1;
                dm = hc->get_dMkr(x) * hc->mr->krd;
                if (q>=99 || x>22)  break;
                solvout +=   "      Vy= " + ss.setNum(hc->vn.y              ,'f',1) + " м/с,\t";
                solvout +=  " Nпотр_mr= " + ss.setNum(hc->mr->N_need        ,'f',1) + " кВт,\t";
                solvout +=   "  Mkr_mr= " + ss.setNum(hc->mr->My_sr         ,'f',1) + " H·м,\t";
                solvout +=  "  alfa_mr= " + ss.setNum(hc->alfa_mr  *180/M_PI,'f',1) + " грд;";
                //solvout +=   "  fi0_mr= " + ss.setNum(hc->mr->fi0  *180/M_PI,'f',3) + " грд,\t";
                //solvout +=    " rpm_mr= " + ss.setNum(hc->mr->rpm           ,'f',1) + " об/мин,\t";
                toLogf(solvout);
                loadLog();
                QCoreApplication::processEvents();
                solvout = "";
            }
            q = 0;
            while ( dm<-50  && !isStop )
            {
                q++;
                x += 0.1;
                dm = hc->get_dMkr(x) * hc->mr->krd;
                if (q>=99)  break;
                solvout +=   "      Vy= " + ss.setNum(hc->vn.y              ,'f',1)+ " м/с,\t";
                solvout +=  " Nпотр_mr= " + ss.setNum(hc->mr->N_need        ,'f',1) + " кВт,\t";
                solvout +=   "  Mkr_mr= " + ss.setNum(hc->mr->My_sr         ,'f',1) + " H·м,\t";
                solvout +=  "  alfa_mr= " + ss.setNum(hc->alfa_mr  *180/M_PI,'f',1) + " грд;";
                //solvout +=   "  fi0_mr= " + ss.setNum(hc->mr->fi0  *180/M_PI,'f',3) + " грд,\t";
                //solvout +=    " rpm_mr= " + ss.setNum(hc->mr->rpm           ,'f',1) + " об/мин,\t";
                toLogf(solvout);
                loadLog();
                QCoreApplication::processEvents();
                solvout = "";
            }
            solvout += "\n";
            solvout += "          Vy=   " + ss.setNum(hc->vn.y  ,'f',2) + " м/с\n";
            solvout += "    Nпотр_mr=   " + ss.setNum(hc->mr->N_need         ,'f',1) + " кВт\n";
            solvout += "      Mkr_mr=   " + ss.setNum(hc->mr->My_sr ,'f',1) + " Н·м\n";
            solvout += "        T_mr=   " + ss.setNum(hc->mr->T_sr  ,'f',1) + " H\n";
            solvout += "     alfa_mr=   " + ss.setNum(hc->mr->alpha *180/M_PI,'f',3) + " грд\n";
            solvout += "      fi0_mr=   " + ss.setNum(hc->mr->fi0   *180/M_PI,'f',3) + " грд\n";
            solvout += "      rpm_mr=   " + ss.setNum(hc->mr->rpm   ,'f',1) + " об/мин\n";
        }
        else ok = false;

        toLogf(solvout);
        toLogf(hc->solvout);
        toLogf(singleline);
        if(ok) toLogf("Done!"); else toLogf("Error...");
        loadLog();
        }
        break;//case 30

      case 33:///определение Tpr по заданным Vx и крут.моменту на валу
        {
        hc->autogyro = true;
        hc->autorot  = false;
        ini.trsolve  = false;
        ini.prsolve  = false;
        hc->Mkr_ = 0;
        QString solvout = "";
        //подготовка к расчету
        hc->PreBalance();
        toLogf("Autogyro.Tpr.Solve:");
        toLogf("определение тяги толкающего винта автожира... ");
        loadLog();
        //нулевое приближение балансировки
        ok = (hc->SolveBalance0())?(0):(1);

        if (hc->mrsolve)
        {
            double  x = 500;//Tpr_min,
            double dm = 111;
            int q = 0;
            while ( dm>0  && !isStop )
            {
                q++;
                x += 500;
                dm = hc->get_dMkr_autogyro(x) * hc->mr->krd;
                if (q>=90)  break;
                solvout += "      T_pr= " + ss.setNum(hc->Fy_pr             ,'f',0)+ " H,\t";
                solvout +=  " Nпотр_mr= " + ss.setNum(hc->mr->N_need        ,'f',1) + " кВт,\t";
                solvout +=   "  Mkr_mr= " + ss.setNum(hc->mr->My_sr         ,'f',1) + " H·м,\t";
                solvout +=  "  alfa_mr= " + ss.setNum(hc->alfa_mr  *180/M_PI,'f',1) + " грд;";
                //solvout +=   "  fi0_mr= " + ss.setNum(hc->mr->fi0  *180/M_PI,'f',3) + " грд,\t";
                //solvout +=    " rpm_mr= " + ss.setNum(hc->mr->rpm           ,'f',1) + " об/мин,\t";
                toLogf(solvout);
                loadLog();
                QCoreApplication::processEvents();
                solvout = "";
            }
            q = 0;
            while ( dm<-50 && !isStop )
            {
                q++;
                x -= 50;
                dm = hc->get_dMkr_autogyro(x) * hc->mr->krd;

                if (q>=90)  break;
                solvout += "      T_pr= " + ss.setNum(hc->Fy_pr             ,'f',0)+ " H,\t";
                solvout +=  " Nпотр_mr= " + ss.setNum(hc->mr->N_need        ,'f',1) + " кВт,\t";
                solvout +=   "  Mkr_mr= " + ss.setNum(hc->mr->My_sr         ,'f',1) + " H·м,\t";
                solvout +=  "  alfa_mr= " + ss.setNum(hc->alfa_mr  *180/M_PI,'f',1) + " грд;";
                toLogf(solvout);
                loadLog();
                QCoreApplication::processEvents();
                solvout = "";
            }
            solvout += "\n";
            solvout += "        T_pr=   " + ss.setNum(hc->Fy_pr              ,'f',1) + " H\n";
            solvout += "    Nпотр_mr=   " + ss.setNum(hc->mr->N_need         ,'f',1) + " кВт\n";
            solvout += "      Mkr_mr=   " + ss.setNum(hc->mr->My_sr          ,'f',1) + " Н·м\n";
            solvout += "        T_mr=   " + ss.setNum(hc->mr->T_sr           ,'f',1) + " H\n";
            solvout += "     alfa_mr=   " + ss.setNum(hc->mr->alpha *180/M_PI,'f',3) + " грд\n";
            solvout += "      fi0_mr=   " + ss.setNum(hc->mr->fi0   *180/M_PI,'f',3) + " грд\n";
            solvout += "      rpm_mr=   " + ss.setNum(hc->mr->rpm            ,'f',1) + " об/мин\n";
        }
        else ok = false;

        toLogf(solvout);
        toLogf(hc->solvout);
        toLogf(singleline);
        if(ok) toLogf("Done!"); else toLogf("Error...");
        }
        break;//case 33

      case 3:///балансировка вертолета Vx ///////////////////////////////////////////////////////////////////////////////////////////////
        {
        //расчет вспомогательных параметров
        hc->PreBalance();
        //расчет и передача скоростей втулки НВ
        SetDataForRotors();//добавил 20211004
        //изменение диапазона угла ОШ НВ
        //if( hc->Mkr_ <= 0  && hc->mrsolve && hc->vn.x>28) hc->mr->fi0_max = 1.3* ini.mr.fi0_max *M_PI/180.;
        toLogf("Balance.Solve:");
        toLogf("\n#iteration: 0");
        loadLog();
        //расчет балансировки вертолета в 0-м приближении
        hc->SolveBalance0();
        toLogf(hc->solvout);
        loadLog();
        QCoreApplication::processEvents();
        //основной расчет балансировки вертолета
        while (!hc->balanced && !isStop && q<20)
        {
            q++;
            if(!hc->SolveBalance(out))
            {
                toLogf("#iteration: " + ss.setNum(q) + hc->solvout);
                loadLog();
                QCoreApplication::processEvents();
            }
            else break;
        }
        //запись полученного состояния вертолета в файл
        out.iter = out.t = float2int(ini.vHx*3.6);
        //запись полученного состояния вертолета в файл
        symbal = "B-" + ss.setNum(out.iter);
        write_helicopter_state( ini.datafolder + symbal );
        //показать состояние на 3Д модели
        if(ui->btn_infopanel->isChecked()) showCurrentInfo();
        write_to_opengl();
        //обработка результатов расчета
        hc->PostBalance();
        //запись результатов расчета в log-файл
        toLogf(singleline);
        toLogf(hc->balancereport);
        toLogf(singleline);
        //запись результатов в файл .bal
        if(hc->balanced)
        {
            writeReport(fname_bal, hc->balancereport);//+ss.setNum(ini.vHx*3.6,'f',0)
            toLogf("Done!");
        }
        else toLogf("Aborted on iteration #" + ss.setNum(q));
        }
        break;//case 3

      case 4:// удалил из расчета // балансировка вертолета V=0...Vmax /////////////////////////////////////////////////////////////////////////////////////
        {
        hc->autorot = ( hc->Mkr_ <= 0 )?(true):(false);//указатель режима авторотации
        //диапазон скорости
        double vi = 0;       // начальная скорость - начало расчетного диапазона
        double stepv = ini.logstep;   // шаг скорости(км/ч) расчетного диапазона
        double vmax = ceil(3.6*ini.vHx *1.05);// максимальная скорость диапазона
        //передаем исходные данные
        SetBalanceInitial();
        //расчет вспомогательных параметров
        hc->PreBalance();
        toLogf("Balance.Solve:");
        if(hc->autorot) toLogf("режим авторотации");
        toLogf("\n#iteration: 0");
        loadLog();
        //расчет балансировки вертолета в 0-м приближении на скорости vi=0
        if(!hc->autorot)
        {
            hc->SolveBalance0();
            toLogf(hc->solvout);
            loadLog();
        }
        QCoreApplication::processEvents();
        //перебор скорости от 0 до vmax и балансировка вертолета
        while (vi <= vmax && !isStop)
        {
            hc->balanced = 0;
            isStop = 0;  q = 0;
            ui->showiter->setText(ss.setNum(vi));
            ui->showtime->setText(ss.setNum(vi,'f',6));
            setWindowTitle(ss.setNum(vi,'f',0) +" км/ч - "+ titlefilename +" - "+ titleapplname);
            symbal = ss.setNum(vi);
            //передаем планеру и винтам новое значение скорости
            hc->vn.x = hc->v.x= vi/3.6;
            hc->inFlow();
            //mr.prep
            if(hc->mrsolve)
            {
                hc->mr->V  = hc->Vi;
                hc->mr->alpha = hc->alfa_mr;
                //hc->mr->beta_sk = beta_sk;
                hc->mr->getIniFlow();//Preparation();
                hc->mr->logkey = 0;
            }
            //tr.prep
            if(hc->trsolve)
            {
                hc->tr->V  = hc->Vi;
                hc->tr->alpha = hc->alfa_tr;
                hc->tr->beta_sk = -hc->tetra;
                hc->tr->getIniFlow();//Preparation();
                hc->tr->logkey = 0;
            }
            //pr.prep
            if(hc->prsolve)
            {
                hc->pr->V  = hc->Vi;
                hc->pr->alpha = (hc->Vxy)?(-asin( hc->v.x / hc->Vxy )):(-0.5*M_PI);//hc->alfa_pr;
                //hc->pr->beta_sk = hc->beta_sk;
                hc->pr->getIniFlow();//Preparation();
                hc->pr->logkey = 0;
            }
            //изменение диапазона угла ОШ НВ
            ///if(hc->autorot && hc->mrsolve && hc->vn.x>28) hc->mr->fi0_max = 1.3* ini.mr.fi0_max *M_PI/180.;
            //на авторотации: определение Vу по заданному крут.моменту
            if(hc->autorot && hc->mrsolve)
            {
                toLogf("балансировка на режиме авторотации при скорости полета Vnx= "+ss.setNum(vi,'f',0)+ " км/ч");
                toLogf("расчет скорости снижения Vnу ...");
                loadLog();
                hc->SolveBalance0();
                toLogf(hc->solvout);
                loadLog();
            }
            else toLogf("балансировка вертолета на горизонтальной скорости Vnx= "+ss.setNum(vi,'f',0)+" км/ч");
            QCoreApplication::processEvents();
            //балансировка вертолета на заданной скорости vi
            while (!hc->balanced && !isStop && q<20)
            {
                q++;
                //балансируем на скорости горизонт.полета vi
                if(!hc->SolveBalance(out))
                {
                    toLogf("Vnx= "+ss.setNum(vi,'f',0)+ " km/h");
                    toLogf("#iteration: " + ss.setNum(q) + hc->solvout);
                    loadLog();
                    //показать состояние на 3Д модели
                    if(ui->btn_infopanel->isChecked()) showCurrentInfo();
                    write_to_opengl();
                    QCoreApplication::processEvents();
                }
                else break;
            }
            //обработка результатов расчета
            hc->PostBalance();
            //запись результатов расчета в log-файл
            toLogf(singleline);
            if(hc->balanced)
            {
                //фиксация текущих даты-времени и общего времени расчета
                currenttime = QTime::currentTime();
                toLogf("balancing for Vnx= "+ss.setNum(vi,'f',0)+ " km/h was finished at " + currenttime.toString());
                toLogf(singleline +"\n");
            }
            else
            {
                ok = false;
                isStop = 1;
            }
            loadLog();
            ui->progress->setValue( 100*vi/vmax );
            QCoreApplication::processEvents();
            //запись полученного состояния вертолета в файл
            out.iter = out.t = vi;
            //запись результатов в файл .bal##
            writeReport(fname_bal+ss.setNum(vi,'f',0), hc->balancereport);
            write_helicopter_state( ini.datafolder + symbal );
            write_to_opengl();
            //прирост итерации
            vi += stepv;
        }
        if(ok) toLogf("Done!");
          else toLogf("Aborted with error...");
        }
        break;//case 4
    }

    //вывод в файл результатов расчета винтов
    if(ok)
    {
        if(ini.mrsolve)
        {
            //вывод исходных данных
            report = hc->mr->Pre2Str(ini.logkey);
            //вывод результатов
            report += hc->mr->Res2Str(ini.logkey);
            writeReport( fname_mrr, report );
            toLogf("MainRotor.Results: saved to " + fname_mrr);
        }
        if(ini.trsolve)
        {
            //вывод исходных данных
            report = hc->tr->Pre2Str(ini.logkey);
            //вывод результатов
            report += hc->tr->Res2Str(ini.logkey);
            writeReport( fname_trr, report );
            toLogf("TailRotor.Results: saved to " + fname_trr);
        }
        if(ini.prsolve)
        {
            //вывод исходных данных
            report = hc->pr->Pre2Str(ini.logkey);
            //вывод результатов
            report += hc->pr->Res2Str(ini.logkey);
            writeReport( fname_prr, report );
            toLogf("PushRotor.Results: saved to " + fname_prr);
        }
    }

    //вывод файлов состояния
    if(ok)
    {
        //заполнение структуры результатов по азимуту
        out.mr.solve = ini.mrsolve;
        out.tr.solve = ini.trsolve;
        out.pr.solve = ini.prsolve;
        for (int az = 0; az <= ini.nAz; az++)
        {
            if(ini.mrsolve) hc->mr->fillOutAz(out,az);
            if(ini.trsolve) hc->tr->fillOutAz(out,az);
            if(ini.prsolve) hc->pr->fillOutAz(out,az);
            write_helicopter_state( ini.datafolder + ss.setNum(az) );//запись в файл состояния
        }
        if (!ini.nAz)  toLogf("Warning: Файлы состояния не записаны!");
    }

    //фиксация текущих даты-времени и общего времени расчета
    currentdate = QDate::currentDate();
    currenttime = QTime::currentTime();
    elapsed = getElapsedTime(tm);
    toLogf("Finished: " + currentdate.toString("yyyy.MM.dd") + " " + currenttime.toString());
    toLogf("Elapsed : " + elapsed);
}
void MainWindow::SetBalanceInitial()// передача исходных данных
{
    hc->precXan = ini.precXan;
    hc->precYan = ini.precYan;
    hc->precChi = ini.precChi;
    hc->precEta = ini.precEta;

    //ПЛАНЕР
    setPlanerInitial();

    //ВИНТЫ
    setRotorsInitial();
    if (ini.mrsolve)  hc->mr->dt = 0;
    if (ini.trsolve)  hc->tr->dt = 0;
    if (ini.prsolve)  hc->pr->dt = 0;

    //внешнее возмущение НВ
    out.fus.a.x = 0;
    out.fus.a.y = 0;
    out.fus.a.z = 0;
    out.fus.e.x = 0;
    out.fus.e.y = 0;
    out.fus.e.z = 0;
    out.fus.v.x = hc->v_.x;//ini.vHx;
    out.fus.v.y = hc->v_.y;//ini.vHy;
    out.fus.v.z = hc->v_.z;//ini.vHz;
    out.fus.w.x = hc->w_.x;//ini.wHx;
    out.fus.w.y = hc->w_.y;//ini.wHy;
    out.fus.w.z = hc->w_.z;//ini.wHz;

    //ГРУЗ НА ВНЕШНЕЙ ТРОСОВОЙ ПОДВЕСКЕ
    hc->zacep.x = ini.xcg*0.001;
    hc->zacep.y = ini.ycg*0.001;
    hc->zacep.z = ini.zcg*0.001;
    hc->mcg     = ini.mcg;
    hc->lcg     = ini.lcg*0.001;
}



//// Д И Н А М И К А  П О Л Е Т А
void MainWindow::MainFlying()//организатор моделирования посадки
{
    //удаление ранее рожденного объекта Helicopter (нужно при прерываниях расчета)
    hc = 0; delete hc;
    //создание объекта Landing
    hc = new Helicopter();
    //инициализация указателя класса на себя (нужно для статических функций)
    hc->SetClassPointer();
    //обнуление переменных
    hc->setNull();
    i = 0;
    hc->iter = i;
    //передача в объект Landing исходных данных, считанных из файла
    SetFlyInitial();
    //подготовка данных для основного расчета
    hc->PreFlying(out);
    //вывод подготовленных данных
    toLogf(hc->preout);
    //передаем данные для следующей итерации
    SetDataForNextI();
    //запись результатов presolve.rotors
    if(ini.mrsolve)
    {
        toLogf("\nMainRotor.Prepare:");
        hc->mr->model = fname_mr;
        report = hc->mr->Pre2Str(ini.logkey);
        toLogf(report);
    }
    if(ini.trsolve)
    {
        toLogf("\nTailRotor.Prepare:");
        hc->tr->model = fname_tr;
        report = hc->tr->Pre2Str(ini.logkey);
        toLogf(report);
    }
    if(ini.prsolve)
    {
        toLogf("\nPushRotor.Prepare:");
        hc->pr->model = fname_pr;
        report = hc->pr->Pre2Str(ini.logkey);
        toLogf(report);
    }

    //прерывание интерфейса
    QCoreApplication::processEvents();
    //метка запуска основного расчета в лог-файле
    toLogf(singleline);
    toLogf("Flying.Solve:\n");
    //загрузить результаты presolve на форму
    loadLog();
    //установка активности кнопок выбора стоек
    setShassiButton(hc->nst);
    //opengl-прорисовка
    write_to_opengl();
    //установка итерации для продолжения расчета
    i = ini.start;
    //загрузка данных, если расчет продолжается с итерации ini.start
    if(i)
    {
        readerror = read_helicopter_state( ini.datafolder + ss.setNum(i) );
        if(!readerror)
        {
            ui->EditCurrent->setText(ss.setNum(i));
            SetDataFromFile();
        }
        else
        {
            toMemo("ошибка считывания файла состояния на шаге ini.start=" + ss.setNum(i));
            ok=false;
        }
    }
    else write_helicopter_state(ini.datafolder+"0");//запись шага "0" (начального положения)
    //запуск цикла расчетов состояния вертолета
    if(ok) FlyingCycles();
    else toMemo("ошибка запуска основного цикла FlyingCycles");
}
void MainWindow::FlyingCycles()//основной цикл расчета состояния вертолета
{
    while( ok && i<ini.steps && !isStop )
    {
        steps  = i++;
        hc->iter = i;
        hc->t = hc->t_ + hc->dt;
        //пересчет кинематических параметров в центре втулки НВ
        SetDataForRotors();

        /*TEST-button///
        if(ui->TestButton->isChecked())
             hc->mr->ahy=0;
        else hc->mr->ahy=0;*/

        //ручное управление в процессе расчета
        hc->manual.fi0mr = ui->SpinBox_fi0mr->value()*M_PI/180.;
        hc->manual.etamr = ui->SpinBox_etamr->value()*M_PI/180.;
        hc->manual.chimr = ui->SpinBox_chimr->value()*M_PI/180.;
        hc->manual.fi0tr = ui->SpinBox_fi0tr->value()*M_PI/180.;

        if( hc->fix_org_x && hc->fix_org_y && hc->fix_org_z && hc->fix_org_k && hc->fix_org_r && hc->fix_org_t )
        {//расчет элементов вертолета без решения динамики полета
            if(hc->FixFlying(out)) ok=false;
        }
        else
        {//вызов основного решателя динамики полета
            if(hc->SolveFlying(out)) ok=false;
        }

        //обнуление при раскрутке-стабилизации винтов (несущего в основном)
        if(!hc->gofly)
        {
            out.iter = i = 0;
            out.t = hc->t_ = hc->t = 0;
            if( hc->mrsolve)
            {
                hc->mr->psih_= hc->mr->normAng(hc->mr->psih);
                out.mr.psih  = hc->mr->psih_;
            }
            if( hc->trsolve)
            {
                hc->tr->psih_= hc->tr->normAng(hc->tr->psih);
                out.tr.psih  = hc->tr->psih_;
            }
            if( hc->prsolve)
            {
                hc->pr->psih_= hc->pr->normAng(hc->pr->psih);
                out.pr.psih  = hc->pr->psih_;
            }
        }
        //вывод результатов расчета текущей итерации
        if(ok)
        {
            //файл состояния в папку
            if(i%ini.logstep==0)
            {
                if (write_helicopter_state( ini.datafolder + ss.setNum(i) ))//запись в файл состояния
                {
                    toLogf("ошибка записи в файл (write_helicopter_state)");
                    toMemo("ошибка записи в файл (write_helicopter_state)");
                }
            }
            //вспомогательные данные на форму
            ui->showiter->setText(ss.setNum(i));
            ui->showtime->setText(ss.setNum(hc->t,'f',6));
            ui->progress->setValue( 100*i/float(ini.steps) );
            setWindowTitle(ss.setNum(hc->t,'f',6) + " - " + titlefilename + " - " + titleapplname);
            //вывод текущих результатов расчета
            if(ui->btn_infopanel->isChecked()) showCurrentInfo();
            //opengl-прорисовка вертолета
            write_to_opengl();
            //прерывание интерфейса
            QCoreApplication::processEvents();
        }
        else
        {
            toMemo("error at " + ss.setNum(i) + " iteration");
            toLogf("error at " + ss.setNum(i) + " iteration");
            setStopped(1);
            ui->tabWidget->setCurrentIndex(1);//активная вкладка
        }
        //запись результатов после очередного полного оборота НВ
        if(hc->mrsolve && hc->logkey && hc->mr->nextrev && hc->gofly)
        {
            if(ui->checkBox_break1rev->isChecked())
            {
                MainWindow::on_StopButton_clicked();//прерывание расчета при warnings
            }
            else
            {
                toLogf(hc->Post2log());
                toLogf(singleline);
                loadLog();
            }
        }

        //передача результатов для расчета следующей итерации
        SetDataForNextI();
    }
    //запись результатов при прерывании или окончании расчета
    toLogf(hc->Post2log());
    //доп.информация
    {
    if (ok && !isStop) toLogf("Done!");
    else if (isStop) toLogf("Aborted!"); else toLogf("Fail!");// + hc->errors);
    toLogf("Warnings: " + ss.setNum(hc->warning));
    //фиксация текущих даты-времени и общего времени расчета
    QDate finishdate = QDate::currentDate();
    QTime finishtime = QTime::currentTime();
    QString elapsed = getElapsedTime(tm);
    toLogf("Finished: " + finishdate.toString("yyyy.MM.dd") + " " + finishtime.toString());
    toLogf("Elapsed : " + elapsed);
    //toLogf(singleline);//"\n" +
    }
}
void MainWindow::SetFlyInitial()//LANDING передача исходных данных в объект hc
{
    //НАСТРОЙКИ
    hc->dt        = ini.dt;
    hc->t_        = 0;
    hc->snlf6prc  = ini.snlf6prc;
    hc->snlf6lim  = ini.snlf6lim;
    hc->snlf2prc  = ini.snlf2prc;
    hc->snlf2lim  = ini.snlf2lim;
    hc->fix_org_x = ini.fix_org_x;
    hc->fix_org_y = ini.fix_org_y;
    hc->fix_org_z = ini.fix_org_z;
    hc->fix_org_k = ini.fix_org_k;
    hc->fix_org_r = ini.fix_org_r;
    hc->fix_org_t = ini.fix_org_t;

    //ПЛАНЕР
    hc->Ix        = ini.Ix ;
    hc->Iy        = ini.Iy ;
    hc->Iz        = ini.Iz ;
    hc->Iyz       = ini.Iyz;
    hc->Ixz       = ini.Ixz;
    hc->Ixy       = ini.Ixy;
    setPlanerInitial();

    //ВИНТЫ
    setRotorsInitial();//передача исходных данных MainRotor и TailRotor

    //ШАССИ
    if(ini.st[0].nst + ini.st[1].nst + ini.st[2].nst > 0)
    {
        hc->nst = ini.st[0].nst + ini.st[1].nst + ini.st[2].nst;
        //рождение и заполнение объектов класса IShassi
        for ( k = 0; k < hc->nst; k++)
        {
            if ( k < ini.st[0].nst )  ik=0;     else ik=1;
            if ( k>= ini.st[0].nst + ini.st[1].nst ) ik=2;
            switch (ini.st[ik].stype)
            {
                case 0://полозковое
                    hc->st[k] = new SkidType(0,0);//тип пневматика 0; тип амортизатора 0;
                break;
                case 1://пирамидальная
                    hc->st[k] = new PyramidType( ini.st[ik].ptype, ini.st[ik].atype );//тип пневматика 1; тип амортизатора 2;
                break;
                case 2: case 3: case 8://рычажная
                    hc->st[k] = new LeverType( ini.st[ik].ptype, ini.st[ik].atype );//тип пневматика 1; тип амортизатора 2;
                break;
                default:
                    hc->st[k] = new LeverType( ini.st[ik].ptype, ini.st[ik].atype );
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
            hc->st[k]->V_.x=      (ini.st[ik].V_.x-ini.st[ik].O_.x)*0.001;
            hc->st[k]->V_.y=      (ini.st[ik].V_.y-ini.st[ik].O_.y)*0.001;
            hc->st[k]->V_.z=      (ini.st[ik].V_.z-ini.st[ik].O_.z)*0.001;
            hc->st[k]->W_.x=      (ini.st[ik].W_.x-ini.st[ik].O_.x)*0.001;
            hc->st[k]->W_.y=      (ini.st[ik].W_.y-ini.st[ik].O_.y)*0.001;
            hc->st[k]->W_.z=      (ini.st[ik].W_.z-ini.st[ik].O_.z)*0.001;
            hc->st[k]->Q_.x=      (ini.st[ik].Q_.x-ini.st[ik].O_.x)*0.001;
            hc->st[k]->Q_.y=      (ini.st[ik].Q_.y-ini.st[ik].O_.y)*0.001;
            hc->st[k]->Q_.z=      (ini.st[ik].Q_.z-ini.st[ik].O_.z)*0.001;
            hc->st[k]->lambda=     ini.st[ik].lambda*M_PI/180.;
            hc->st[k]->ang_wh=     ini.st[ik].ang_wh*M_PI/180.;
            hc->st[k]->pn->dpmd=   ini.st[ik].pn.dpmd*0.001;
            hc->st[k]->pn->cpx=    ini.st[ik].pn.cpx  *1000;
            hc->st[k]->pn->cpy=    ini.st[ik].pn.cpy  *1000;
            hc->st[k]->pn->cpz=    ini.st[ik].pn.cpz  *1000;
            hc->st[k]->pn->alfanp= ini.st[ik].pn.alfanp  ;
            hc->st[k]->pn->kdp=    ini.st[ik].pn.kdp     ;
            hc->st[k]->pn->Dp=     ini.st[ik].pn.D *0.001;
            hc->st[k]->pn->Bp=     ini.st[ik].pn.B *0.001;
            hc->st[k]->pn->isbrake=ini.st[ik].pn.isbrake;
            hc->st[k]->pn->f0=     ini.st[ik].pn.f0;
            hc->st[k]->pn->fs=     ini.st[ik].pn.fs;
            hc->st[k]->pn->fk=     ini.st[ik].pn.fk;

            hc->st[k]->Aa = 0;
            hc->st[k]->Ap = 0;
            hc->st[k]->ab->c1.v_h.clear();
            hc->st[k]->ab->c1.v_f.clear();
            hc->st[k]->ab->c1.v_fr.clear();
            hc->st[k]->ab->c2.v_h.clear();
            hc->st[k]->ab->c2.v_f.clear();
            hc->st[k]->ab->c2.v_fr.clear();
            hc->st[k]->kostyl = false;

            if(ini.st[ik].atype == 0)/// НЕЛИНЕЙНАЯ ПРУЖИНА
            {
                hc->st[k]->ab->hu=  ini.st[ik].ab.hu*0.001;
                hc->st[k]->ab->Ke=  ini.st[ik].ab.Ke*1000;
                hc->st[k]->ab->Ki=  ini.st[ik].ab.Ki*1000;
                hc->st[k]->ab->Ci=  ini.st[ik].ab.Ci;
            }
            ///else                  /// АМОРТИЗАТОР
            {
                hc->st[k]->ab->ro=      ini.st[ik].ab.ro;
                hc->st[k]->ab->mu=      ini.st[ik].ab.mu;
                hc->st[k]->ab->kpoly=   ini.st[ik].ab.kpoly;
                hc->st[k]->ab->kpolyr=  ini.st[ik].ab.kpolyr;

                hc->st[k]->ab->c1.p0=   ini.st[ik].ab.c1.p0*98066.5;
                hc->st[k]->ab->c1.V0=  (ini.st[ik].ab.c1.Vsum - ini.st[ik].ab.c1.Voil) *0.001;
                hc->st[k]->ab->c1.s=    ini.st[ik].ab.c1.s  *0.001;
                hc->st[k]->ab->c1.Dc=   ini.st[ik].ab.c1.Dc *0.001;
                hc->st[k]->ab->c1.Dv=   ini.st[ik].ab.c1.Dv *0.001;
                hc->st[k]->ab->c1.Ds=   ini.st[ik].ab.c1.Ds *0.001;
                hc->st[k]->ab->c1.ktr=  ini.st[ik].ab.c1.ktr;
                hc->st[k]->ab->c1.fb=   ini.st[ik].ab.c1.fb *0.000001;
                hc->st[k]->ab->c1.fbr=  ini.st[ik].ab.c1.fbr*0.000001;
                hc->st[k]->ab->c1.f=    ini.st[ik].ab.c1.f  *0.000001;
                hc->st[k]->ab->c1.fr=   ini.st[ik].ab.c1.fr *0.000001;

                switch (ini.st[ik].atype)//тип амортизатора
                {
                case 1: //однокамерный
                    hc->st[k]->ab->p0 =   hc->st[k]->ab->c1.p0;
                    hc->st[k]->ab->V0 =   hc->st[k]->ab->c1.V0;
                    hc->st[k]->ab->c2.p0= 0;
                    hc->st[k]->ab->c2.V0= 0;
                    hc->st[k]->ab->c2.s = 0;
                    break;

                case 2: //двухкамерный  (Ми-8)
                    hc->st[k]->ab->c2.p0=  ini.st[ik].ab.c2.p0*98066.5;
                    hc->st[k]->ab->c2.V0= (ini.st[ik].ab.c2.Vsum - ini.st[ik].ab.c2.Voil) *0.001;
                    hc->st[k]->ab->c2.s=   ini.st[ik].ab.c2.s  *0.001;
                    hc->st[k]->ab->c2.Dc=  ini.st[ik].ab.c2.Dc *0.001;
                    hc->st[k]->ab->c2.Dv=  ini.st[ik].ab.c2.Dv *0.001;
                    hc->st[k]->ab->c2.Ds=  ini.st[ik].ab.c2.Ds *0.001;
                    hc->st[k]->ab->c2.ktr= ini.st[ik].ab.c2.ktr;
                    hc->st[k]->ab->c2.fb=  ini.st[ik].ab.c2.fb *0.000001;
                    hc->st[k]->ab->c2.fbr= ini.st[ik].ab.c2.fbr*0.000001;
                    hc->st[k]->ab->c2.f=   ini.st[ik].ab.c2.f  *0.000001;
                    hc->st[k]->ab->c2.fr=  ini.st[ik].ab.c2.fr *0.000001;
                    hc->st[k]->ab->p0 = 0;
                    hc->st[k]->ab->V0 = 0;
                    break;

                case 3: //два штока с общей газ.камерой (Ми-2)
                    hc->st[k]->ab->p0 =    ini.st[ik].ab.p0*98066.5;
                    hc->st[k]->ab->c1.p0=  hc->st[k]->ab->p0;
                    hc->st[k]->ab->c2.p0=  hc->st[k]->ab->p0;
                    hc->st[k]->ab->V0 =   (ini.st[ik].ab.Vsum - ini.st[ik].ab.Voil) *0.001;
                    hc->st[k]->ab->c1.V0=  hc->st[k]->ab->V0;
                    hc->st[k]->ab->c2.V0=  hc->st[k]->ab->V0;
                    hc->st[k]->ab->c2.s=   ini.st[ik].ab.c2.s  *0.001;
                    hc->st[k]->ab->c2.Dc=  ini.st[ik].ab.c2.Dc *0.001;
                    hc->st[k]->ab->c2.Dv=  ini.st[ik].ab.c2.Dv *0.001;
                    hc->st[k]->ab->c2.Ds=  ini.st[ik].ab.c2.Ds *0.001;
                    hc->st[k]->ab->c2.fb=  ini.st[ik].ab.c2.fb *0.000001;
                    hc->st[k]->ab->c2.fbr= ini.st[ik].ab.c2.fbr*0.000001;
                    hc->st[k]->ab->c2.f=   ini.st[ik].ab.c2.f  *0.000001;
                    hc->st[k]->ab->c2.fr=  ini.st[ik].ab.c2.fr *0.000001;
                    hc->st[k]->ab->c2.ktr= ini.st[ik].ab.c2.ktr;
                    break;
                }
                hc->st[k]->ab->S = hc->st[k]->ab->c2.s + hc->st[k]->ab->c1.s;
                //ab.c#.f как функция положения штока
                int v_h_size1 = ini.st[ik].ab.c1.v_h.size();
                if (v_h_size1>1)//c1.f задана функцией
                {
                    hc->st[k]->ab->c1.f  = 0;
                    hc->st[k]->ab->c1.fr = 0;
                    i=0;
                    while(i<v_h_size1)
                    {
                        hc->st[k]->ab->c1.v_h .push_back(ini.st[ik].ab.c1.v_h[i] *0.001);
                        hc->st[k]->ab->c1.v_f .push_back(ini.st[ik].ab.c1.v_f[i] *0.000001);
                        hc->st[k]->ab->c1.v_fr.push_back(ini.st[ik].ab.c1.v_fr[i]*0.000001);
                        i++;
                    }
                }
                int v_h_size2 = ini.st[ik].ab.c2.v_h.size();
                if (v_h_size2>1)//c2.f задана функцией
                {
                    hc->st[k]->ab->c2.f  = 0;
                    hc->st[k]->ab->c2.fr = 0;
                    i=0;
                    while(i<v_h_size2)
                    {
                        hc->st[k]->ab->c2.v_h .push_back(ini.st[ik].ab.c2.v_h[i] *0.001);
                        hc->st[k]->ab->c2.v_f .push_back(ini.st[ik].ab.c2.v_f[i] *0.000001);
                        hc->st[k]->ab->c2.v_fr.push_back(ini.st[ik].ab.c2.v_fr[i]*0.000001);
                        i++;
                    }
                }

            }
            hc->st[k]->ground = ini.ground[k]*0.001;
            hc->st[k]->direct = 0;
        }
        //заполнение "симметричных" параметров (по direct)
        // передние/носовые опоры
        for ( k = 0; k < ini.st[0].nst; k++)
        {
            ik = 0;
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
                    hc->st[k]->A_.z = (ini.st[ik].A_.z-ini.st[ik].O_.z) *hc->st[k]->direct *0.001;
                    hc->st[k]->B_.z = (ini.st[ik].B_.z-ini.st[ik].O_.z) *hc->st[k]->direct *0.001;
                    hc->st[k]->V_.z = (ini.st[ik].V_.z-ini.st[ik].O_.z) *hc->st[k]->direct *0.001;
                    hc->st[k]->W_.z = (ini.st[ik].W_.z-ini.st[ik].O_.z) *hc->st[k]->direct *0.001;
                    hc->st[k]->Q_.z = (ini.st[ik].Q_.z-ini.st[ik].O_.z) *hc->st[k]->direct *0.001;
                break;
            }
        }
        // основные опоры
        for ( k = ini.st[0].nst; k < ini.st[0].nst + ini.st[1].nst; k++)
        {
            ik = 1;
            switch (ini.st[1].nst)
            {
                case 0:
                    hc->st[k]->direct = 0;
                break;
                case 1:
                    hc->st[k]->direct = 0;//0
                break;
                case 2:
                    if ( k == ini.st[0].nst + ini.st[1].nst -2 )  hc->st[k]->direct = 1;//k=1 /2
                    if ( k == ini.st[0].nst + ini.st[1].nst -1 )  hc->st[k]->direct =-1;//k=2 /3
                    hc->st[k]->O_.z =                  ini.st[ik].O_.z  *hc->st[k]->direct *0.001;
                    hc->st[k]->A_.z = (ini.st[ik].A_.z-ini.st[ik].O_.z) *hc->st[k]->direct *0.001;
                    hc->st[k]->B_.z = (ini.st[ik].B_.z-ini.st[ik].O_.z) *hc->st[k]->direct *0.001;
                    hc->st[k]->V_.z = (ini.st[ik].V_.z-ini.st[ik].O_.z) *hc->st[k]->direct *0.001;
                    hc->st[k]->W_.z = (ini.st[ik].W_.z-ini.st[ik].O_.z) *hc->st[k]->direct *0.001;
                    hc->st[k]->Q_.z = (ini.st[ik].Q_.z-ini.st[ik].O_.z) *hc->st[k]->direct *0.001;
                break;
            }
        }
        // хвостовая опора
        for ( k = ini.st[0].nst + ini.st[1].nst; k < ini.st[0].nst + ini.st[1].nst + ini.st[2].nst; k++)
        {
            hc->st[k]->direct = 0;
            hc->st[k]->kostyl = true;
        }
    }
    else hc->nst = 0;

    //ГРУЗ НА ВНЕШНЕЙ ТРОСОВОЙ ПОДВЕСКЕ
    hc->zacep.x = ini.xcg*0.001;
    hc->zacep.y = ini.ycg*0.001;
    hc->zacep.z = ini.zcg*0.001;
    hc->mcg     = ini.mcg;
    hc->lcg     = ini.lcg*0.001;

    //ВНЕШНЯЯ НАГРУЗКА
    hc->load1      = ini.load1;
    hc->load1pos.x = ini.load1pos.x*0.001;
    hc->load1pos.y = ini.load1pos.y*0.001;
    hc->load1pos.z = ini.load1pos.z*0.001;
    hc->load2      = ini.load2;
    hc->load2pos.x = ini.load2pos.x*0.001;
    hc->load2pos.y = ini.load2pos.y*0.001;
    hc->load2pos.z = ini.load2pos.z*0.001;
}
void MainWindow::SetDataForRotors()//
{
    //присвоение значений временным переменным
    float  x = hc->x_mr - hc->xct;/////////////////
    float  y = hc->y_mr - hc->yct;/////////////////
    float  z = hc->z_mr - hc->zct;/////////////////
    float sx = 0;//     *0.001;
    float sy = 0;//     *0.001;
    float sz = 0;//     *0.001;
    float ux = out.fus.org.k*M_PI/180;
    float uy = out.fus.org.r*M_PI/180;
    float uz = out.fus.org.t*M_PI/180;
    float vx = out.fus.v.x;
    float vy = out.fus.v.y;
    float vz = out.fus.v.z;
    float wx = out.fus.w.x;
    float wy = out.fus.w.y;
    float wz = out.fus.w.z;
    float ax = out.fus.a.x;
    float ay = out.fus.a.y;
    float az = out.fus.a.z;
    float ex = out.fus.e.x;
    float ey = out.fus.e.y;
    float ez = out.fus.e.z;
    //расчет смещения заданной точки
    sx += (uy*z - uz*y);
    sy += (uz*x - ux*z);
    sz += (ux*y - uy*x);
    //расчет скорости в заданной точке
    vx += (wy*z - wz*y);
    vy += (wz*x - wx*z);
    vz += (wx*y - wy*x);
    //расчет ускорения в заданной точке
    ax += (ey*z - ez*y);
    ay += (ez*x - ex*z);
    az += (ex*y - ey*x);

    if (hc->mrsolve)
    {
        hc->mr->nextrev = 0;
        hc->mr->a_x = -ax;     hc->mr->a_y = ay;     hc->mr->a_z = -az;
        hc->mr->e_x = -ex;     hc->mr->e_y = ey;     hc->mr->e_z = -ez;
        hc->mr->v_x = -vx;     hc->mr->v_y = vy;     hc->mr->v_z = -vz;
        hc->mr->w_x = -wx;     hc->mr->w_y = wy;     hc->mr->w_z = -wz;
        //hc->mr->shx = -sx;     hc->mr->shy = sy;     hc->mr->shz = -sz;

        out.mr.ah.x  = hc->mr->a_x;
        out.mr.ah.y  = hc->mr->a_y;
        out.mr.ah.z  = hc->mr->a_z;
        out.mr.eh.x  = hc->mr->e_x;
        out.mr.eh.y  = hc->mr->e_y;
        out.mr.eh.z  = hc->mr->e_z;
        out.mr.vh.x  = hc->mr->v_x;
        out.mr.vh.y  = hc->mr->v_y;
        out.mr.vh.z  = hc->mr->v_z;
        out.mr.wh.x  = hc->mr->w_x;
        out.mr.wh.y  = hc->mr->w_y;
        out.mr.wh.z  = hc->mr->w_z;
        //out.mr.sh.x  = hc->mr->shx*1000;
        //out.mr.sh.y  = hc->mr->shy*1000;
        //out.mr.sh.z  = hc->mr->shz*1000;
    }

    if (hc->prsolve)
    {
        hc->pr->w_y = wx;     hc->pr->w_z = wy;     hc->pr->w_x = wz;
    }
}
void MainWindow::SetDataForNextI()//установление параметров "предыдущей итерации"
{
    hc->t_ = hc->t;
    //ограничение перемещений
    if(!hc->fix_org_x) {  hc->org_.x = hc->org.x;  hc->ocw_.x = hc->ocw.x;  hc->v_.x = hc->v.x;  hc->w_.x = hc->w.x;  } // hc->org.x = hc->org_.x;   hc->ocw.x = hc->ocw_.x;   hc->v.x = hc->v_.x;
    if(!hc->fix_org_y) {  hc->org_.y = hc->org.y;  hc->ocw_.y = hc->ocw.y;  hc->v_.y = hc->v.y;  hc->w_.y = hc->w.y;  } // hc->org.y = hc->org_.y;   hc->ocw.y = hc->ocw_.y;   hc->v.y = hc->v_.y;
    if(!hc->fix_org_z) {  hc->org_.z = hc->org.z;  hc->ocw_.z = hc->ocw.z;  hc->v_.z = hc->v.z;  hc->w_.z = hc->w.z;  } // hc->org.z = hc->org_.z;   hc->ocw.z = hc->ocw_.z;   hc->v.z = hc->v_.z;
    if(!hc->fix_org_k) {  hc->org_.k = hc->org.k;  hc->ocw_.k = hc->ocw.k;    } // hc->org.k = hc->org_.k;   hc->ocw.k = hc->ocw_.k;   hc->w.x = hc->w_.x;
    if(!hc->fix_org_r) {  hc->org_.r = hc->org.r;  hc->ocw_.r = hc->ocw.r;    } // hc->org.r = hc->org_.r;   hc->ocw.r = hc->ocw_.r;   hc->w.y = hc->w_.y;
    if(!hc->fix_org_t) {  hc->org_.t = hc->org.t;  hc->ocw_.t = hc->ocw.t;    } // hc->org.t = hc->org_.t;   hc->ocw.t = hc->ocw_.t;   hc->w.z = hc->w_.z;

    ////положение планера
    //hc->org_.x = hc->org.x;     hc->org_.k = hc->org.k;
    //hc->org_.y = hc->org.y;     hc->org_.r = hc->org.r;
    //hc->org_.z = hc->org.z;     hc->org_.t = hc->org.t;
    //hc->ocw_.x = hc->ocw.x;     hc->ocw_.k = hc->ocw.k;
    //hc->ocw_.y = hc->ocw.y;     hc->ocw_.r = hc->ocw.r;
    //hc->ocw_.z = hc->ocw.z;     hc->ocw_.t = hc->ocw.t;
    ////скорость планера в СКС
    //hc->v_.x = hc->v.x;         hc->w_.x = hc->w.x;
    //hc->v_.y = hc->v.y;         hc->w_.y = hc->w.y;
    //hc->v_.z = hc->v.z;         hc->w_.z = hc->w.z;
    //Gear
    for (k=0; k<hc->nst; k++)
    {
        hc->st[k]->O_        = hc->st[k]->O;
        hc->st[k]->Q_        = hc->st[k]->Q;
        hc->st[k]->A_        = hc->st[k]->A;
        hc->st[k]->B_        = hc->st[k]->B;
        hc->st[k]->D_        = hc->st[k]->D;
        hc->st[k]->C_        = hc->st[k]->C;
        hc->st[k]->saC_      = hc->st[k]->saC;
        hc->st[k]->pn->h_    = hc->st[k]->pn->h;
        hc->st[k]->pn->h1_   = hc->st[k]->pn->h1;
        hc->st[k]->pn->h2_   = hc->st[k]->pn->h2;
        hc->st[k]->ab->ha_   = hc->st[k]->ab->ha;
        hc->st[k]->ab->c1.h_ = hc->st[k]->ab->c1.h;
        hc->st[k]->ab->c2.h_ = hc->st[k]->ab->c2.h;
        hc->st[k]->ab->V_    = hc->st[k]->ab->V;
        hc->st[k]->ab->c1.V_ = hc->st[k]->ab->c1.V;
        hc->st[k]->ab->c2.V_ = hc->st[k]->ab->c2.V;
        hc->st[k]->ab->c1.p_ = hc->st[k]->ab->c1.p;
        hc->st[k]->ab->c2.p_ = hc->st[k]->ab->c2.p;
        hc->st[k]->AB_       = hc->st[k]->AB;
        //hc->st[k]->status_   = hc->st[k]->status;
        hc->st[k]->ab->status_=hc->st[k]->ab->status;
        hc->st[k]->pn->status_=hc->st[k]->pn->status;
        //hc->FMw_[k].Fx = hc->FMw_[k].Fx;         hc->FMw_[k].Mx = hc->FMw[k].Mx;
        //hc->FMw_[k].Fy = hc->FMw_[k].Fy;         hc->FMw_[k].My = hc->FMw[k].My;
        //hc->FMw_[k].Fz = hc->FMw_[k].Fz;         hc->FMw_[k].Mz = hc->FMw[k].Mz;
    }
    //MR
    if(hc->mrsolve)
    {
        hc->mr->psih_ = hc->mr->psih;
        for (int l = 0; l < hc->mr->zl; l++)
        {
            hc->mr->B[l].psi_    = hc->mr->B[l].psi   ;
            hc->mr->B[l].fi7_    = hc->mr->B[l].fi7   ;
            hc->mr->B[l].dfi7_   = hc->mr->B[l].dfi7  ;
            hc->mr->B[l].d2fi7_  = hc->mr->B[l].d2fi7 ;
            hc->mr->B[l].beta_   = hc->mr->B[l].beta  ;
            hc->mr->B[l].dbeta_  = hc->mr->B[l].dbeta ;
            hc->mr->B[l].d2beta_ = hc->mr->B[l].d2beta;
            hc->mr->B[l].ksi_    = hc->mr->B[l].ksi   ;
            hc->mr->B[l].dksi_   = hc->mr->B[l].dksi  ;
            hc->mr->B[l].d2ksi_  = hc->mr->B[l].d2ksi ;
            if(hc->mr->zakrut)
            {
                hc->mr->B[l].gama_   = hc->mr->B[l].gama  ;
                hc->mr->B[l].dgama_  = hc->mr->B[l].dgama ;
                hc->mr->B[l].d2gama_ = hc->mr->B[l].d2gama;
            }
        }
    }
    //TR
    if(hc->trsolve)
    {
        hc->tr->psih_ = hc->tr->psih;
        for (int l = 0; l < hc->tr->zl; l++)
        {
            hc->tr->B[l].psi_    = hc->tr->B[l].psi   ;
            hc->tr->B[l].fi7_    = hc->tr->B[l].fi7   ;
            hc->tr->B[l].dfi7_   = hc->tr->B[l].dfi7  ;
            hc->tr->B[l].d2fi7_  = hc->tr->B[l].d2fi7 ;
            hc->tr->B[l].beta_   = hc->tr->B[l].beta  ;
            hc->tr->B[l].dbeta_  = hc->tr->B[l].dbeta ;
            hc->tr->B[l].d2beta_ = hc->tr->B[l].d2beta;
            hc->tr->B[l].ksi_    = hc->tr->B[l].ksi   ;
            hc->tr->B[l].dksi_   = hc->tr->B[l].dksi  ;
            hc->tr->B[l].d2ksi_  = hc->tr->B[l].d2ksi ;
            if(hc->tr->zakrut)
            {
                hc->tr->B[l].gama_   = hc->tr->B[l].gama  ;
                hc->tr->B[l].dgama_  = hc->tr->B[l].dgama ;
                hc->tr->B[l].d2gama_ = hc->tr->B[l].d2gama;
            }
        }
    }
    //PR
    if(hc->prsolve)
    {
        hc->pr->psih_ = hc->pr->psih;
        for (int l = 0; l < hc->pr->zl; l++)
        {
            hc->pr->B[l].psi_    = hc->pr->B[l].psi   ;
            hc->pr->B[l].fi7_    = hc->pr->B[l].fi7   ;
            hc->pr->B[l].dfi7_   = hc->pr->B[l].dfi7  ;
            hc->pr->B[l].d2fi7_  = hc->pr->B[l].d2fi7 ;
            hc->pr->B[l].gama_   = hc->pr->B[l].gama  ;
            hc->pr->B[l].dgama_  = hc->pr->B[l].dgama ;
            hc->pr->B[l].d2gama_ = hc->pr->B[l].d2gama;
            hc->pr->B[l].beta_   = hc->pr->B[l].beta  ;
            hc->pr->B[l].dbeta_  = hc->pr->B[l].dbeta ;
            hc->pr->B[l].d2beta_ = hc->pr->B[l].d2beta;
            hc->pr->B[l].ksi_    = hc->pr->B[l].ksi   ;
            hc->pr->B[l].dksi_   = hc->pr->B[l].dksi  ;
            hc->pr->B[l].d2ksi_  = hc->pr->B[l].d2ksi ;
        }
    }
}
void MainWindow::SetDataFromFile()//установление "предыдущей итерации"   добавить !!!
{
    hc->t_      = hc->t      = out.t;
    //положение планера
    hc->org_.x  = hc->org.x  = out.fus.org.x ;
    hc->org_.y  = hc->org.y  = out.fus.org.y ;
    hc->org_.z  = hc->org.z  = out.fus.org.z ;
    hc->org_.k  = hc->org.k  = out.fus.org.k *M_PI/180.0;
    hc->org_.r  = hc->org.r  = out.fus.org.r *M_PI/180.0;
    hc->org_.t  = hc->org.t  = out.fus.org.t *M_PI/180.0;
    hc->ocw_.x  = hc->ocw.x  = out.fus.ocw.x ;
    hc->ocw_.y  = hc->ocw.y  = out.fus.ocw.y ;
    hc->ocw_.z  = hc->ocw.z  = out.fus.ocw.z ;
    hc->ocw_.k  = hc->ocw.k  = hc->org_.k;
    hc->ocw_.r  = hc->ocw.r  = hc->org_.r;
    hc->ocw_.t  = hc->ocw.t  = hc->org_.t;
    //скорость планера в СКС
    hc->v_.x    = hc->v.x    = out.fus.v.x;
    hc->v_.y    = hc->v.y    = out.fus.v.y;
    hc->v_.z    = hc->v.z    = out.fus.v.z;
    hc->w_.x    = hc->w.x    = out.fus.w.x;
    hc->w_.y    = hc->w.y    = out.fus.w.y;
    hc->w_.z    = hc->w.z    = out.fus.w.z;

                  hc->a.x    = out.fus.a.x;
                  hc->a.y    = out.fus.a.y;
                  hc->a.z    = out.fus.a.z;
                  hc->e.x    = out.fus.e.x;
                  hc->e.y    = out.fus.e.y;
                  hc->e.z    = out.fus.e.z;

    hc->crgct.x = out.cargo.CTx;
    hc->crgct.y = out.cargo.CTy;
    hc->crgct.z = out.cargo.CTz;
    hc->Gcrg.x  = out.cargo.Gx ;
    hc->Gcrg.y  = out.cargo.Gy ;
    hc->Gcrg.z  = out.cargo.Gz ;
    //manual установка управляющих сигналов
    setManualControl(out.tr.fi0, out.mr.fi0, out.mr.eta, out.mr.chi);
    ui->check_manualcontrol->setChecked(true);
    //MR
    hc->Fx_mr_ = hc->Fx_mr = out.mr.Fhx_sr;
    hc->Fy_mr_ = hc->Fy_mr = out.mr.Fhy_sr;
    hc->Fz_mr_ = hc->Fz_mr = out.mr.Fhz_sr;
    hc->Mx_mr_ = hc->Mx_mr = out.mr.Mhx_sr;
    hc->My_mr_ = hc->My_mr = out.mr.Mhy_sr;
    hc->Mz_mr_ = hc->Mz_mr = out.mr.Mhz_sr;
    if(hc->mrsolve)
    {
        hc->mr->H_sr    = out.mr.Fhx_sr;
        hc->mr->T_sr    = out.mr.Fhy_sr;
        hc->mr->S_sr    = out.mr.Fhz_sr;
        hc->mr->Mx_sr   = out.mr.Mhx_sr;
        hc->mr->My_sr   = out.mr.Mhy_sr;
        hc->mr->Mz_sr   = out.mr.Mhz_sr;
        hc->mr->Fhx_sum = out.mr.Fhx_sum;
        hc->mr->Fhy_sum = out.mr.Fhy_sum;
        hc->mr->Fhz_sum = out.mr.Fhz_sum;
        hc->mr->Mhx_sum = out.mr.Mhx_sum;
        hc->mr->Mhy_sum = out.mr.Mhy_sum;
        hc->mr->Mhz_sum = out.mr.Mhz_sum;
        hc->mr->revcount= out.mr.revcount;
        hc->mr->psih_   = out.mr.psih*M_PI/180;
        for (int l = 0; l < hc->mr->zl; l++)
        {
            hc->mr->B[l].psi_    = out.mr.b[l].psi *M_PI/180;
            hc->mr->B[l].fi7_    = out.mr.b[l].fi7 *M_PI/180;
            hc->mr->B[l].dfi7_   = out.mr.b[l].dfi7;
            hc->mr->B[l].d2fi7_  = out.mr.b[l].d2fi7;
            hc->mr->B[l].gama_   = out.mr.b[l].gama *M_PI/180;
            hc->mr->B[l].dgama_  = out.mr.b[l].dgama ;
            hc->mr->B[l].d2gama_ = out.mr.b[l].d2gama;
            hc->mr->B[l].beta_   = out.mr.b[l].beta*M_PI/180;
            hc->mr->B[l].dbeta_  = out.mr.b[l].dbeta ;
            hc->mr->B[l].d2beta_ = out.mr.b[l].d2beta;
            hc->mr->B[l].ksi_    = out.mr.b[l].ksi *M_PI/180;
            hc->mr->B[l].dksi_   = out.mr.b[l].dksi  ;
            hc->mr->B[l].d2ksi_  = out.mr.b[l].d2ksi ;
        }
    }
    //TR
    hc->Fx_tr_ = hc->Fx_tr = out.tr.Fhx_sr;
    hc->Fy_tr_ = hc->Fy_tr = out.tr.Fhy_sr;
    hc->Fz_tr_ = hc->Fz_tr = out.tr.Fhz_sr;
    hc->Mx_tr_ = hc->Mx_tr = out.tr.Mhx_sr;
    hc->My_tr_ = hc->My_tr = out.tr.Mhy_sr;
    hc->Mz_tr_ = hc->Mz_tr = out.tr.Mhz_sr;
    if(hc->trsolve)
    {
        hc->tr->H_sr    = out.tr.Fhx_sr;
        hc->tr->T_sr    = out.tr.Fhy_sr;
        hc->tr->S_sr    = out.tr.Fhz_sr;
        hc->tr->Mx_sr   = out.tr.Mhx_sr;
        hc->tr->My_sr   = out.tr.Mhy_sr;
        hc->tr->Mz_sr   = out.tr.Mhz_sr;
        hc->tr->Fhx_sum = out.tr.Fhx_sum;
        hc->tr->Fhy_sum = out.tr.Fhy_sum;
        hc->tr->Fhz_sum = out.tr.Fhz_sum;
        hc->tr->Mhx_sum = out.tr.Mhx_sum;
        hc->tr->Mhy_sum = out.tr.Mhy_sum;
        hc->tr->Mhz_sum = out.tr.Mhz_sum;
        hc->tr->revcount= out.tr.revcount;
        hc->tr->psih_   = out.tr.psih*M_PI/180;
        for (int l = 0; l < hc->tr->zl; l++)
        {
            hc->tr->B[l].psi_    = out.tr.b[l].psi *M_PI/180;
            hc->tr->B[l].fi7_    = out.tr.b[l].fi7 *M_PI/180;
            hc->tr->B[l].dfi7_   = out.tr.b[l].dfi7 ;
            hc->tr->B[l].d2fi7_  = out.tr.b[l].d2fi7;
            hc->tr->B[l].gama_   = out.tr.b[l].gama *M_PI/180;
            hc->tr->B[l].dgama_  = out.tr.b[l].dgama ;
            hc->tr->B[l].d2gama_ = out.tr.b[l].d2gama;
            hc->tr->B[l].beta_   = out.tr.b[l].beta*M_PI/180;
            hc->tr->B[l].dbeta_  = out.tr.b[l].dbeta ;
            hc->tr->B[l].d2beta_ = out.tr.b[l].d2beta;
            hc->tr->B[l].ksi_    = out.tr.b[l].ksi *M_PI/180;
            hc->tr->B[l].dksi_   = out.tr.b[l].dksi  ;
            hc->tr->B[l].d2ksi_  = out.tr.b[l].d2ksi ;
        }
    }
    //PR
    hc->Fx_pr_ = hc->Fx_pr = out.pr.Fhx_sr;
    hc->Fy_pr_ = hc->Fy_pr = out.pr.Fhy_sr;
    hc->Fz_pr_ = hc->Fz_pr = out.pr.Fhz_sr;
    hc->Mx_pr_ = hc->Mx_pr = out.pr.Mhx_sr;
    hc->My_pr_ = hc->My_pr = out.pr.Mhy_sr;
    hc->Mz_pr_ = hc->Mz_pr = out.pr.Mhz_sr;
    if(hc->prsolve)
    {
        hc->pr->H_sr    = out.pr.Fhx_sr;
        hc->pr->T_sr    = out.pr.Fhy_sr;
        hc->pr->S_sr    = out.pr.Fhz_sr;
        hc->pr->Mx_sr   = out.pr.Mhx_sr;
        hc->pr->My_sr   = out.pr.Mhy_sr;
        hc->pr->Mz_sr   = out.pr.Mhz_sr;
        hc->pr->Fhx_sum = out.pr.Fhx_sum;
        hc->pr->Fhy_sum = out.pr.Fhy_sum;
        hc->pr->Fhz_sum = out.pr.Fhz_sum;
        hc->pr->Mhx_sum = out.pr.Mhx_sum;
        hc->pr->Mhy_sum = out.pr.Mhy_sum;
        hc->pr->Mhz_sum = out.pr.Mhz_sum;
        hc->pr->revcount= out.pr.revcount;
        hc->pr->psih_   = out.pr.psih*M_PI/180;
        for (int l = 0; l < hc->pr->zl; l++)
        {
            hc->pr->B[l].psi_    = out.pr.b[l].psi *M_PI/180;
            hc->pr->B[l].fi7_    = out.pr.b[l].fi7 *M_PI/180;
            hc->pr->B[l].dfi7_   = out.pr.b[l].dfi7  ;
            hc->pr->B[l].d2fi7_  = out.pr.b[l].d2fi7;
            hc->pr->B[l].gama_   = out.pr.b[l].gama *M_PI/180;
            hc->pr->B[l].dgama_  = out.pr.b[l].dgama ;
            hc->pr->B[l].d2gama_ = out.pr.b[l].d2gama;
            hc->pr->B[l].beta_   = out.pr.b[l].beta*M_PI/180;
            hc->pr->B[l].dbeta_  = out.pr.b[l].dbeta ;
            hc->pr->B[l].d2beta_ = out.pr.b[l].d2beta;
            hc->pr->B[l].ksi_    = out.pr.b[l].ksi *M_PI/180;
            hc->pr->B[l].dksi_   = out.pr.b[l].dksi  ;
            hc->pr->B[l].d2ksi_  = out.pr.b[l].d2ksi ;
        }
    }
    //Gear
    for (k=0; k<hc->nst; k++)
    {
        //hc->st[k]->stype = out.st[k].type;
        hc->st[k]->status     = out.st[k].status;
        hc->st[k]->ab->status = out.st[k].a.status;
        hc->st[k]->pn->status = out.st[k].p.status;
        hc->st[k]->O_.x       = hc->st[k]->O.x       = out.st[k].O.x    *0.001;
        hc->st[k]->O_.y       = hc->st[k]->O.y       = out.st[k].O.y    *0.001;
        hc->st[k]->O_.z       = hc->st[k]->O.z       = out.st[k].O.z    *0.001;
        hc->st[k]->Q_.x       = hc->st[k]->Q.x       = (out.st[k].Q.x-out.st[k].O.x) *0.001;
        hc->st[k]->Q_.y       = hc->st[k]->Q.y       = (out.st[k].Q.y-out.st[k].O.y) *0.001;
        hc->st[k]->Q_.z       = hc->st[k]->Q.z       = (out.st[k].Q.z-out.st[k].O.z) *0.001;
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
        hc->st[k]->ab->c1.V_  = hc->st[k]->ab->c1.V  = out.st[k].a.V1   *0.001;
        hc->st[k]->ab->c2.V_  = hc->st[k]->ab->c2.V  = out.st[k].a.V2   *0.001;
        hc->st[k]->ab->V_     = hc->st[k]->ab->V     = out.st[k].a.V1   *0.001;/// warning!
        hc->st[k]->ab->c1.p_  = hc->st[k]->ab->c1.V  = out.st[k].a.p1   *0.001;
        hc->st[k]->ab->c2.p_  = hc->st[k]->ab->c2.V  = out.st[k].a.p2   *0.001;
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
        hc->st[k]->ab->dh12  = out.st[k].a.dh12*0.001;  if(hc->st[k]->ab->dh12) hc->st[k]->ab->q=1;
        //hc->FMw_[k].Fx = out.st[k].F.x;         ///hc->FMw_[k].Mx = out.st[k].M.x;
        //hc->FMw_[k].Fy = out.st[k].F.y;         ///hc->FMw_[k].My = out.st[k].M.y;
        //hc->FMw_[k].Fz = out.st[k].F.z;         ///hc->FMw_[k].Mz = out.st[k].M.z;
    }
}
void MainWindow::showCurrentInfo()//CurrentInfo на opengl-widget
{
    QString log_ogl = "";
    //Airframe
    if(ui->pushPlanerInfo->isChecked())    log_ogl += hc->CurrentInfo_Airframe();
    //Shassi
    if(ui->pushShassiInfo->isChecked())    log_ogl += hc->CurrentInfo_Shassi();
    //MainRotor
    if(ui->pushMainRotorInfo->isChecked()) log_ogl += hc->CurrentInfo_MainRotor();
    //TailRotor
    if(ui->pushTailRotorInfo->isChecked()) log_ogl += hc->CurrentInfo_TailRotor();

    ui->oglmemo->setPlainText( log_ogl );
}
void MainWindow::showReadedInfo()//ReadedInfo на opengl-widget
{
    QString logl = "";
    //Airframe
    if(ui->pushPlanerInfo->isChecked())    logl += "";
    /*{
    logl += "AIRFRAME:  warning= " + ss.setNum(warning) + "\n";
    logl += "o.x=" + ss.setNum(org.x       ,'f',3) + "\t";
    logl += "o.y=" + ss.setNum(org.y       ,'f',3) + "\t";
    logl += "o.z=" + ss.setNum(org.z       ,'f',3) + "\n";
    //logl += "kren=" + ss.setNum(org.k*180/pi,'f',2) + "\t";
    //logl += "rysk=" + ss.setNum(org.r*180/pi,'f',2) + "\t";
    //logl += "tang=" + ss.setNum(org.t*180/pi,'f',2) + "\n";
    //logl += "\n";
    //logl += "cw.x=" + ss.setNum(ocw.x          ,'f',3) + "\t";
    //logl += "cw.y=" + ss.setNum(ocw.y          ,'f',3) + "\t";
    //logl += "cw.z=" + ss.setNum(ocw.z          ,'f',3) + "\n";
    logl += "cw.k=" + ss.setNum(ocw.k   *180/pi,'f',2) + "\t";
    logl += "cw.r=" + ss.setNum(ocw.r   *180/pi,'f',2) + "\t";
    logl += "cw.t=" + ss.setNum(ocw.t   *180/pi,'f',2) + "\n";
    logl += "\n";
    logl += "vx="   + ss.setNum(vn.x        ,'f',2) + "\t";
    logl += "vy="   + ss.setNum(vn.y        ,'f',2) + "\t";
    logl += "vz="   + ss.setNum(vn.z        ,'f',2) + "\n";
    logl += "wk="   + ss.setNum(vn.k *180/pi,'f',2) + "\t";
    logl += "wr="   + ss.setNum(vn.r *180/pi,'f',2) + "\t";
    logl += "wt="   + ss.setNum(vn.t *180/pi,'f',2) + "\n";
    logl += "ax="   + ss.setNum(a.x            ,'f',2) + "\t";
    logl += "ay="   + ss.setNum(a.y            ,'f',2) + "\t";
    logl += "az="   + ss.setNum(a.z            ,'f',2) + "\n";
    logl += "ex="   + ss.setNum(e.x            ,'f',2) + "\t";
    logl += "ey="   + ss.setNum(e.y            ,'f',2) + "\t";
    logl += "ez="   + ss.setNum(e.z            ,'f',2) + "\n";
    logl += "nx="   + ss.setNum(Fx/Gh          ,'f',2) + "\t";
    logl += "ny="   + ss.setNum(1+Fy/Gh        ,'f',2) + "\t";
    logl += "nz="   + ss.setNum(Fz/Gh          ,'f',2) + "\n";
    logl += "Fx="   + ss.setNum(Fx             ,'f',0) + "\t";
    logl += "Fy="   + ss.setNum(Fy             ,'f',0) + "\t";
    logl += "Fz="   + ss.setNum(Fz             ,'f',0) + "\n";
    logl += "Mx="   + ss.setNum(Mx             ,'f',0) + "\t";
    logl += "My="   + ss.setNum(My             ,'f',0) + "\t";
    logl += "Mz="   + ss.setNum(Mz             ,'f',0) + "\n";
    logl += "\nвес вертолета:\n";
    logl += "Fx="   + ss.setNum(FMhg.Fx        ,'f',0) + "\t";
    logl += "Fy="   + ss.setNum(FMhg.Fy        ,'f',0) + "\t";
    logl += "Fz="   + ss.setNum(FMhg.Fz        ,'f',0) + "\n";
    logl += "Mx="   + ss.setNum(FMhg.Mx        ,'f',0) + "\t";
    logl += "My="   + ss.setNum(FMhg.My        ,'f',0) + "\t";
    logl += "Mz="   + ss.setNum(FMhg.Mz        ,'f',0) + "\n";
    logl += "\nшасси (приведено к ЦТ вертолета):\n";
    logl += "Fx="   + ss.setNum(FMsh.Fx        ,'f',0) + "\t";
    logl += "Fy="   + ss.setNum(FMsh.Fy        ,'f',0) + "\t";
    logl += "Fz="   + ss.setNum(FMsh.Fz        ,'f',0) + "\n";
    logl += "Mx="   + ss.setNum(FMsh.Mx        ,'f',0) + "\t";
    logl += "My="   + ss.setNum(FMsh.My        ,'f',0) + "\t";
    logl += "Mz="   + ss.setNum(FMsh.Mz        ,'f',0) + "\n";
    logl += "\nнесущий винт:\n";
    logl += "alfa=" + ss.setNum(alfa_mr*180/M_PI,'f',1) + " грд\n";
    logl += "Fx="   + ss.setNum(FMmr.Fx        ,'f',0) + "\t";
    logl += "Fy="   + ss.setNum(FMmr.Fy        ,'f',0) + "\t";
    logl += "Fz="   + ss.setNum(FMmr.Fz        ,'f',0) + "\n";
    logl += "Mx="   + ss.setNum(FMmr.Mx        ,'f',0) + "\t";
    logl += "My="   + ss.setNum(FMmr.My        ,'f',0) + "\t";
    logl += "Mz="   + ss.setNum(FMmr.Mz        ,'f',0) + "\n";
    logl += "Tdmp=" + ss.setNum(Tdmp           ,'f',1) + " Н\n";
    logl += "\nрулевой винт:\n";
    logl += "alfa=" + ss.setNum(alfa_tr*180/M_PI,'f',1) + " грд\n";
    logl += "Fx="   + ss.setNum(FMtr.Fx        ,'f',0) + "\t";
    logl += "Fy="   + ss.setNum(FMtr.Fy        ,'f',0) + "\t";
    logl += "Fz="   + ss.setNum(FMtr.Fz        ,'f',0) + "\n";
    logl += "Mx="   + ss.setNum(FMtr.Mx        ,'f',0) + "\t";
    logl += "My="   + ss.setNum(FMtr.My        ,'f',0) + "\t";
    logl += "Mz="   + ss.setNum(FMtr.Mz        ,'f',0) + "\n";
    logl += "\nпланер:\n";
    logl += "Fx="   + ss.setNum(FMadp.Fx       ,'f',0) + "\t";
    logl += "Fy="   + ss.setNum(FMadp.Fy       ,'f',0) + "\t";
    logl += "Fz="   + ss.setNum(FMadp.Fz       ,'f',0) + "\n";
    logl += "Mx="   + ss.setNum(FMadp.Mx       ,'f',0) + "\t";
    logl += "My="   + ss.setNum(FMadp.My       ,'f',0) + "\t";
    logl += "Mz="   + ss.setNum(FMadp.Mz       ,'f',0) + "\n";
    logl += "\nфюзеляж (лок.СК):\n";//(без оперения)
    logl += "kren=" + ss.setNum(kren  *180/M_PI,'f',1) + "\t";
    logl += "tang=" + ss.setNum(tang  *180/M_PI,'f',1) + "\t";
    logl += "ttra=" + ss.setNum(tetra *180/M_PI,'f',1) + "\n";
    logl += "alfa=" + ss.setNum(alfa_f*180/M_PI,'f',1) + "\t";
    logl += "beta=" + ss.setNum(beta_sk*180/M_PI,'f',1) + "\n";
    logl += "fx="   + ss.setNum( Fx_f           ,'f',0) + "\t";
    logl += "fy="   + ss.setNum( Fy_f           ,'f',0) + "\t";
    logl += "fz="   + ss.setNum( Fz_f           ,'f',0) + "\n";
    logl += "mx="   + ss.setNum(Mx_f           ,'f',0) + "\t";
    logl += "my="   + ss.setNum(My_f           ,'f',0) + "\t";
    logl += "mz="   + ss.setNum(Mz_f           ,'f',0) + "\n";
    logl += "\nстабилизатор (лок.СК):\n";
    logl += "alfa=" + ss.setNum(alfa_st*180/M_PI,'f',1) + " грд\n";
    logl += "fx="   + ss.setNum( Fx_st          ,'f',0) + "\t";
    logl += "fy="   + ss.setNum( Fy_st          ,'f',0) + "\t";
    logl += "fz="   + ss.setNum( Fz_st          ,'f',0) + "\n";
    logl += "mx="   + ss.setNum(Mx_st          ,'f',0) + "\t";
    logl += "my="   + ss.setNum(My_st          ,'f',0) + "\t";
    logl += "mz="   + ss.setNum(Mz_st          ,'f',0) + "\n";
    logl += "\nкиль (лок.СК):\n";
    logl += "alfa=" + ss.setNum(alfa_keel*180/M_PI,'f',1) + " грд\n";
    logl += "fx="   + ss.setNum( Fx_keel        ,'f',0) + "\t";
    logl += "fy="   + ss.setNum( Fy_keel        ,'f',0) + "\t";
    logl += "fz="   + ss.setNum( Fz_keel        ,'f',0) + "\n";
    logl += "mx="   + ss.setNum(Mx_keel        ,'f',0) + "\t";
    logl += "my="   + ss.setNum(My_keel        ,'f',0) + "\t";
    logl += "mz="   + ss.setNum(Mz_keel        ,'f',0) + "\n";
    logl += "\nкрыло (лок.СК):\n";
    logl += "alfa=" + ss.setNum(alfa_wing*180/M_PI,'f',1) + " грд\n";
    logl += "fx="   + ss.setNum( X_wing         ,'f',0) + "\t";
    logl += "fy="   + ss.setNum( Fy_wing         ,'f',0) + "\t";
    logl += "fz="   + ss.setNum( Fz_wing         ,'f',0) + "\n";
    logl += "mx="   + ss.setNum(Mx_wing         ,'f',0) + "\t";
    logl += "my="   + ss.setNum(My_wing         ,'f',0) + "\t";
    logl += "mz="   + ss.setNum(Mz_wing         ,'f',0) + "\n";
    }*/

    //Shassi
    if(ui->pushShassiInfo->isChecked())    logl += "";
    /*
    logl += "\nGEAR: nst=" + ss.setNum(nst)+ "\n";
    for (int k = 0; k < nst; k++)
    {
        logl += "\nstrut №" + ss.setNum(k);
        logl += " (contact="+ ss.setNum(st[k]->contact);
        logl += ", status=" + ss.setNum(st[k]->status);
        if(st[k]->warning)     { logl +=    ", warning=" + ss.setNum(st[k]->warning    ); }// warnings = true;
        if(st[k]->ab->warning) { logl += ", ab.warning=" + ss.setNum(st[k]->ab->warning); }// warnings = true;
        if(st[k]->pn->warning) { logl += ", pn.warning=" + ss.setNum(st[k]->pn->warning); }// warnings = true;
        logl += ")\n";
        logl += "p~a: ";
        logl += "Ppy="  + ss.setNum(st[k]->Ppy,  'f',0) + "[";
        logl += ss.setNum(st[k]->pn->status);
        if(st[k]->nwh==2)
        {
            logl += ":" + ss.setNum(st[k]->pn->status1);
            logl += "," + ss.setNum(st[k]->pn->status2);
        }
        logl += "] ~ ";
        logl += "Pay="  + ss.setNum(st[k]->Pay, 'f',0);
        logl +=     "[" + ss.setNum(st[k]->ab->status) + "]\n";
        logl += "hp=  " + ss.setNum(st[k]->pn->h.x*1000, 'f',2) + ", ";
        logl +=  ss.setNum(st[k]->pn->h.y*1000, 'f',2) + ", ";
        logl +=  ss.setNum(st[k]->pn->h.z*1000, 'f',2) + "\n";
        if(st[k]->nwh==2)
        {
            logl += "hp1= " + ss.setNum(st[k]->pn->h1.x*1000,'f',2) + ", ";
            logl +=           ss.setNum(st[k]->pn->h1.y*1000,'f',2) + ", ";
            logl +=           ss.setNum(st[k]->pn->h1.z*1000,'f',2) + "\n";
            logl += "hp2= " + ss.setNum(st[k]->pn->h2.x*1000,'f',2) + ", ";
            logl +=           ss.setNum(st[k]->pn->h2.y*1000,'f',2) + ", ";
            logl +=           ss.setNum(st[k]->pn->h2.z*1000,'f',2) + "\n";
        }
            logl += "Pp=  " + ss.setNum(st[k]->pn->P.x, 'f',1) + ", ";
            logl +=           ss.setNum(st[k]->pn->P.y, 'f',1) + ", ";
            logl +=           ss.setNum(st[k]->pn->P.z, 'f',1) + "\n";
        if(st[k]->nwh==2)
        {
            logl += "Pp1= " + ss.setNum(st[k]->pn->P1.x,'f',1) + ", ";
            logl +=           ss.setNum(st[k]->pn->P1.y,'f',1) + ", ";
            logl +=           ss.setNum(st[k]->pn->P1.z,'f',1) + "\n";
            logl += "Pp2= " + ss.setNum(st[k]->pn->P2.x,'f',1) + ", ";
            logl +=           ss.setNum(st[k]->pn->P2.y,'f',1) + ", ";
            logl +=           ss.setNum(st[k]->pn->P2.z,'f',1) + "\n";
        }
            logl += "ha=  " + ss.setNum(st[k]->ab->ha*1000,  'f',2);
        if(st[k]->atype>1)
        {
            logl +=    " (" + ss.setNum(st[k]->ab->c1.h*1000,'f',2)+ " + ";
            logl +=           ss.setNum(st[k]->ab->c2.h*1000,'f',2)+ ") \n";
        }
        else logl += " \n";
            logl += "Pa=  " + ss.setNum(st[k]->ab->Pa,  'f',1)+ " \n";
        if(st[k]->atype>1)
        {
            logl += "c1~c2: ";
            logl += "Pac1=" + ss.setNum(st[k]->ab->c1.P,'f',0) + " ~ ";
            logl += "Pac2=" + ss.setNum(st[k]->ab->c2.P,'f',0) + "\n";
        }
        if(st[k]->atype==0) logl += "q=" + ss.setNum(st[k]->ab->q) + "\n";
        logl += "ang_sx= " + ss.setNum(st[k]->ang_sx*r2d ,'f',3)+ " грд\n";
        logl += "p.v.y= " + ss.setNum(st[k]->pn->v.y     ,'f',9)+ " м/c\n";
        logl += "p.dhy= " + ss.setNum(st[k]->pn->dhy*1000,'f',9)+ " мм\n";
        logl += "Aa=   " + ss.setNum(st[k]->Aa     ,'f',1)+ " Дж\n";
        logl += "Ap=   " + ss.setNum(st[k]->Ap     ,'f',1)+ " Дж\n";
        logl += "Aptr= " + ss.setNum(st[k]->Aptr   ,'f',1)+ " Дж\n";
        logl += "Ptr=  " + ss.setNum(st[k]->pn->Ptr,'f',1)+ " Н\n";
        logl += "P.y=  " + ss.setNum(st[k]->pn->P.y,'f',1)+ " Н\n";
    }*/

    //MainRotor
    if(ui->pushMainRotorInfo->isChecked()) logl += "";
    /*{
    QString logl="\nMAINROTOR: ";
    if(mrsolve)
    {
        if(mr->revcount<2)
            logl += "stabilization... \nrevcount= " + ss.setNum(mr->revcount) + "\n";
        else
        {
            logl += "warning= " + ss.setNum(mr->warning) + "\n";
            logl += mr->logstr;
        }
    }
    else
    {
        logl += "unsolved\n";
        logl += "H="  + ss.setNum(H_mr   ,'f',0) + "\t";
        logl += "T="  + ss.setNum(T_mr   ,'f',0) + "\t";
        logl += "S="  + ss.setNum(S_mr   ,'f',0) + ";\n";
        logl += "Mx=" + ss.setNum(Mx_mr  ,'f',0) + "\t";
        logl += "My=" + ss.setNum(My_mr  ,'f',0) + "\t";
        logl += "Mz=" + ss.setNum(Mz_mr  ,'f',0) + ";\n";
    }
    }*/

    //TailRotor
    if(ui->pushTailRotorInfo->isChecked()) logl += "";
    /*{
    QString logl="\nTAILROTOR: ";
    if(trsolve)
    {
        if(tr->revcount<2)
            logl += "stabilization... \nrevcount= " + ss.setNum(tr->revcount) + "\n";
        else
        {
            logl += "warning= " + ss.setNum(tr->warning) + "\n";
            logl += tr->logstr;
        }
    }
    else
    {
        logl += "unsolved\n";
        logl += "H="  + ss.setNum(H_tr   ,'f',0) + "\t";
        logl += "T="  + ss.setNum(T_tr   ,'f',0) + "\t";
        logl += "S="  + ss.setNum(S_tr   ,'f',0) + "\n";
        logl += "Mx=" + ss.setNum(Mx_tr  ,'f',0) + "\t";
        logl += "My=" + ss.setNum(My_tr  ,'f',0) + "\t";
        logl += "Mz=" + ss.setNum(Mz_tr  ,'f',0) + "\n";
    }
    }*/

    ui->oglmemo->setPlainText( logl );
}



// исходные данные
int  MainWindow::read_ini_(QString infileName)//сокращенное считывание из основного файла для POST.процесса
{
    bool readerror_in = 0;
    QFile inifile(infileName);
    if(inifile.open( QIODevice::ReadOnly | QIODevice::Text ))
    {
        in = new QTextStream(&inifile);

        if(find_int(  in,"start="    ,ini.start       )) { readerror_in=true; toMemo("start    not found"); }
        if(find_int(  in,"steps="    ,ini.steps       )) { readerror_in=true; toMemo("steps    not found"); }
                          ini.steps +=ini.start;
        if(find_int(  in,"lstep="    ,ini.logstep     )) { readerror_in=true; toMemo("lstep  not found"); }

        if(find_float(in,"x_f="      ,ini.x_f       )) { toMemo("x_f  not found"); }//=Характерная точка для определения скоса на фюзеляже от несущего винта по оси Ох, м
        if(find_float(in,"y_f="      ,ini.y_f       )) { toMemo("y_f  not found"); }//=Характерная точка для определения скоса на фюзеляже от несущего винта по оси Оу, м
        if(find_float(in,"z_f="      ,ini.z_f       )) { toMemo("z_f  not found"); }//=Характерная точка для определения скоса на фюзеляже от несущего винта по оси Оz, м
        if(find_float(in,"x_w="      ,ini.x_w       )) { toMemo("x_w  not found"); }//=25572483;//Координаты стабилизатора в по Ox CK Oxyz, м
        if(find_float(in,"y_w="      ,ini.y_w       )) { toMemo("y_w  not found"); }//=18215118;//Координаты стабилизатора в по Оу CK Oxyz, м
        if(find_float(in,"z_w="      ,ini.z_w       )) { toMemo("z_w  not found"); }//=Координаты стабилизатора в по Оz CK Oxyz, м
        if(find_float(in,"x_st="     ,ini.x_st      )) { toMemo("x_st not found"); }//=25572483;//Координаты стабилизатора в по Ox CK Oxyz, м
        if(find_float(in,"y_st="     ,ini.y_st      )) { toMemo("y_st not found"); }//=18215118;//Координаты стабилизатора в по Оу CK Oxyz, м
        if(find_float(in,"z_st="     ,ini.z_st      )) { toMemo("z_st not found"); }//=Координаты стабилизатора в по Оz CK Oxyz, м
        if(find_float(in,"x_k="      ,ini.x_k       )) { toMemo("x_k  not found"); }
        if(find_float(in,"y_k="      ,ini.y_k       )) { toMemo("y_k  not found"); }
        if(find_float(in,"z_k="      ,ini.z_k       )) { toMemo("z_k  not found"); }

        if(find_float(in,"selpoint.x=", ini.selpoint.x)) { ini.selpoint.x = 0; }
        if(find_float(in,"selpoint.y=", ini.selpoint.y)) { ini.selpoint.y = 0; }
        if(find_float(in,"selpoint.z=", ini.selpoint.z)) { ini.selpoint.z = 0; }

        //корректировка контура планера frm.cntr
        if(find_int(  in,"frmdir.x=" ,frmpos.xdir     )) { readerror_in=true; toMemo("frmdir.x not found"); }
        if(find_int(  in,"frmdir.y=" ,frmpos.ydir     )) { readerror_in=true; toMemo("frmdir.y not found"); }
        if(find_int(  in,"frmdir.z=" ,frmpos.zdir     )) { readerror_in=true; toMemo("frmdir.z not found"); }
        if(find_float(in,"frmpos.x=" ,frmpos.xbias    )) { readerror_in=true; toMemo("frmpos.x not found"); }
        if(find_float(in,"frmpos.y=" ,frmpos.ybias    )) { readerror_in=true; toMemo("frmpos.y not found"); }
        if(find_float(in,"frmpos.z=" ,frmpos.zbias    )) { readerror_in=true; toMemo("frmpos.z not found"); }
        if(find_float(in,"g="        ,ini.g           )) { readerror_in=true; toMemo("g        not found"); }

        //Винт рулевой
        if(find_bool(in,"solveTailRotor=", ini.trsolve)) { ini.TRmount=false; ini.trsolve=0; }  else
        {
            ini.TRmount = true;
            out.tr.mount = ini.TRmount;
        }
        //Винт толкающий
        if(find_bool(in,"solvePushRotor=", ini.prsolve)) { ini.PRmount=false; ini.prsolve=0; }  else
        {
            ini.PRmount = true;
            out.pr.mount = ini.PRmount;
        }
        //Винт несущий
        if(find_bool( in,"solveMainRotor=",ini.mrsolve)) { ini.mrsolve=0;  readerror_in=true;  toMemo("solveMainRotor  not found"); }

        //связь с файлами
        if(ini.mrsolve)
        {  if(find_str( in,"data.mr:", fname_mr   )) { readerror_in=true; toMemo("data.mr:  not found"); } }//(конструктивные и аэродинамические характеристики винта несущего)
        if(ini.trsolve)
        {  if(find_str( in,"data.tr:", fname_tr   )) { readerror_in=true; toMemo("data.tr:  not found"); } }//(конструктивные и аэродинамические характеристики винта рулевого)
        if(ini.prsolve)
        {  if(find_str( in,"data.pr:", fname_pr   )) { readerror_in=true; toMemo("data.pr:  not found"); } }//(конструктивные и аэродинамические характеристики винта )
           find_str( in,"frame.stl:", fname_cntr   );//(контур фюзеляжа)
        fname_fax   = getFolderName(infilename) + fname_fax   ;
        fname_mr    = getFolderName(infilename) + fname_mr    ;
        fname_tr    = getFolderName(infilename) + fname_tr    ;
        fname_pr    = getFolderName(infilename) + fname_pr    ;
        fname_load1 = getFolderName(infilename) + fname_load1 ;
        fname_cntr  = getFolderName(infilename) + fname_cntr  ;

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
        return 1;
}
int  MainWindow::read_ini(QString infileName)//считывание из основного файла
{
    bool readerror_in = 0;
    QFile inifile(infileName);
    if(inifile.open( QIODevice::ReadOnly | QIODevice::Text ))
    {
        in = new QTextStream(&inifile);

        //НАСТРОЙКИ РАСЧЕТА
        if(find_float(in,"dt="       ,ini.dt       )) { readerror_in=true;   toMemo("dt       not found"); }
        if(find_int(  in,"start="    ,ini.start    )) { ini.start=0;         toMemo("start    not found"); }
        if(find_int(  in,"steps="    ,ini.steps    )) { readerror_in=true;   toMemo("steps    not found"); }
                                      ini.steps+=ini.start;
        if(find_int(  in,"lstep="    ,ini.logstep  )) { ini.logstep=1;       toMemo("lstep    not found"); }
        if(find_int(  in,"logkey="   ,ini.logkey   )) { ini.logkey=1;        toMemo("logkey   not found"); }
        if(find_float(in,"rotprec="  ,ini.rotprec  )) { ini.rotprec=0.01;    toMemo("rotprec  not found"); }//
        if( ini.rotprec<0.0000001 || ini.rotprec>1.0)   ini.rotprec=0.01;
        if(find_float(in,"kra="      ,ini.kra      )) { ini.kra=0.01;        toMemo("kra      not found"); }
        if(find_float(in,"krp="      ,ini.krp      )) { ini.krp=0.01;        toMemo("krp      not found"); }
        if(find_float(in,"snlf6prc=" ,ini.snlf6prc )) { ini.snlf6prc=100;    toMemo("snlf6prc not found"); }
        if(find_float(in,"snlf2prc=" ,ini.snlf2prc )) { ini.snlf2prc=0.0001; toMemo("snlf2prc not found"); }
        ini.snlf6lim = 10000;
        ini.snlf2lim = 10000;
        if(find_float(in,"precXan="  ,ini.precXan  )) { ini.precXan=1;       toMemo("precXan  not found"); }
        if(find_float(in,"precYan="  ,ini.precYan  )) { ini.precYan=1;       toMemo("precYan  not found"); }
        if(find_float(in,"precChi="  ,ini.precChi  )) { ini.precChi=1;       toMemo("precChi  not found"); }
        if(find_float(in,"precEta="  ,ini.precEta  )) { ini.precEta=1;       toMemo("precEta  not found"); }
        //ПАРАМЕТРЫ ПОЛЕТА (начальные условия)
        if(find_bool( in,"autogyro=" ,ini.autogyro )) { ini.autogyro=0; }
        if(find_float(in,"Ox="       ,ini.Hx       )) { ini.Hx=0;            toMemo("Ox       not found"); }
        if(find_float(in,"Oy="       ,ini.Hy       ))
        {
            if(find_float(in,"H="    ,ini.Hy       ))
            { readerror_in=true; toMemo("H or Oy  not found"); }
        }
        if(find_float(in,"Oz="       ,ini.Hz       )) { ini.Hz=0;          toMemo("Oz       not found"); }
        if(find_float(in,"kren="     ,ini.kren_    )) { ini.kren_=0;       toMemo("kren     not found"); }
        if(find_float(in,"rysk="     ,ini.rysk_    )) { ini.rysk_=0;       toMemo("rysk     not found"); }
        if(find_float(in,"tang="     ,ini.tang_    )) { ini.tang_=0;       toMemo("tang     not found"); }
        if(find_float(in,"Vnx="      ,ini.vHx      )) { ini.vHx  =0;       toMemo("Vnx      not found"); }
        if(find_float(in,"Vny="      ,ini.vHy      )) { ini.vHy  =0;       toMemo("Vny      not found"); }
        if(find_float(in,"Vnz="      ,ini.vHz      )) { ini.vHz  =0;       toMemo("Vnz      not found"); }
        if(find_float(in,"w.x="      ,ini.wHx      )) { ini.wHx=0;         toMemo("w.x      not found"); }
        if(find_float(in,"w.y="      ,ini.wHy      )) { ini.wHy=0;         toMemo("w.y      not found"); }
        if(find_float(in,"w.z="      ,ini.wHz      )) { ini.wHz=0;         toMemo("w.z      not found"); }
        if(find_float(in,"g="        ,ini.g        )) { ini.g=9.80665;     toMemo("g        not found"); }
        if(find_float(in,"t="        ,ini.t0       )) { ini.t0=15.0;       toMemo("t        not found"); }
        if(find_float(in,"P/p="      ,ini.P_p      )) { ini.P_p=101325.0;  toMemo("P/p      not found"); }
        if(ini.P_p>10)
             { ini.p0 = ini.P_p;  ini.ro = 0; }
        else { ini.ro = ini.P_p;  ini.p0 = 0; }
        ini.Lpt       = 0.0065;   //К/м       //скорость падения температуры К/м
        ini.RjmK      = 8.31447;  //Дж/(мол*К)//газовая постоянная Дж/(мол*К)
        ini.RjkgK     = 287.0;    //Дж/(кг*К) //газовая постоянная Дж/(кг*К)
        ini.Mm        = 0.0289644;//кг/моль   //молярная масса сухого воздуха кг/мол

        //ПАРАМЕТРЫ ВЕРТОЛЕТА
        if(find_float(in,"mh="       ,ini.mH       )) { readerror_in=true; toMemo("mH    not found"); }
        if(find_float(in,"xctmr="    ,ini.xctmr    )) { readerror_in=true; toMemo("xc    not found"); }//xTmr Горизонтальная центровка вертолета в СК НВ, м     xTH = xt_in_geo;
        if(find_float(in,"yctmr="    ,ini.yctmr    )) { readerror_in=true; toMemo("yc    not found"); }//yTmr Вертикальная центровка вертолета в СК НВ, м       yTH = yt_in_geo;
        if(find_float(in,"zctmr="    ,ini.zctmr    )) { ini.zctmr=0;       toMemo("zc    not found"); }//zTmr центровка вертолета вдоль Oz, м                   zT = zt_in_geo;
        if(find_float(in,"Ix="       ,ini.Ix       )) { readerror_in=true; toMemo("Ix    not found"); }
        if(find_float(in,"Iy="       ,ini.Iy       )) { readerror_in=true; toMemo("Iy    not found"); }
        if(find_float(in,"Iz="       ,ini.Iz       )) { readerror_in=true; toMemo("Iz    not found"); }
        if(find_float(in,"Iyz="      ,ini.Iyz      )) { ini.Iyz=0;         toMemo("Iyz   not found"); }//readerror_in=true;
        if(find_float(in,"Ixz="      ,ini.Ixz      )) { ini.Ixz=0;         toMemo("Ixz   not found"); }//readerror_in=true;
        if(find_float(in,"Ixy="      ,ini.Ixy      )) { readerror_in=true; toMemo("Ixy   not found"); }
        //ФЮЗЕЛЯЖ (как ародинамическая часть планера)
        if(find_int(in,"solvePlaner=",ini.solvePlaner)) { ini.solvePlaner=0; toMemo("solvePlaner  not found"); }        //}
        if(find_float(in,"x_f="      ,ini.x_f      )) { ini.x_f  =0;         toMemo("x_f   not found"); }//=Характерная точка для определения скоса на фюзеляже от несущего винта по оси Ох, м
        if(find_float(in,"y_f="      ,ini.y_f      )) { ini.y_f  =0;         toMemo("y_f   not found"); }//=Характерная точка для определения скоса на фюзеляже от несущего винта по оси Оу, м
        if(find_float(in,"z_f="      ,ini.z_f      )) { ini.z_f  =0;         toMemo("z_f   not found"); }//=Характерная точка для определения скоса на фюзеляже от несущего винта по оси Оz, м
        if(find_float(in,"Fx_f="     ,ini.Fx_f     )) { ini.Fx_f =0;         toMemo("Fx_f  not found"); }
        if(find_float(in,"Fy_f="     ,ini.Fy_f     )) { ini.Fy_f =0;         toMemo("Fy_f  not found"); }
        if(find_float(in,"Fz_f="     ,ini.Fz_f     )) { ini.Fz_f =0;         toMemo("Fz_f  not found"); }
        if(find_float(in,"Mx_f="     ,ini.Mx_f     )) { ini.Mx_f =0;         toMemo("Mx_f  not found"); }
        if(find_float(in,"My_f="     ,ini.My_f     )) { ini.My_f =0;         toMemo("My_f  not found"); }
        if(find_float(in,"Mz_f="     ,ini.Mz_f     )) { ini.Mz_f =0;         toMemo("Mz_f  not found"); }
        //КРЫЛО
        if(find_int( in,"solveWing=",ini.solveWing )) { ini.solveWing=0;   toMemo("solveWing not found"); }
        if(find_float(in,"x_w="     ,ini.x_w       )) { ini.x_w      =0;   toMemo("x_w   not found"); }
        if(find_float(in,"y_w="     ,ini.y_w       )) { ini.y_w      =0;   toMemo("y_w   not found"); }
        if(find_float(in,"z_w="     ,ini.z_w       )) { ini.z_w      =0;   toMemo("z_w   not found"); }
        if(find_float(in,"Fx_w="    ,ini.Fx_wing   )) { ini.Fx_wing  =0;   toMemo("Fx_w  not found"); }
        if(find_float(in,"Fy_w="    ,ini.Fy_wing   )) { ini.Fy_wing  =0;   toMemo("Fy_w  not found"); }
        //СТАБИЛИЗАТОР
        if(find_int( in,"solveStab=",ini.solveStab )) { ini.solveStab=0;   toMemo("solveStab not found"); }
        if(find_float(in,"x_st="    ,ini.x_st      )) { ini.x_st     =0;   toMemo("x_st  not found"); }//=25572483;//Координаты стабилизатора в по Ox CK Oxyz, м
        if(find_float(in,"y_st="    ,ini.y_st      )) { ini.y_st     =0;   toMemo("y_st  not found"); }//=18215118;//Координаты стабилизатора в по Оу CK Oxyz, м
        if(find_float(in,"z_st="    ,ini.z_st      )) { ini.z_st     =0;   toMemo("z_st  not found"); }//=Координаты стабилизатора в по Оz CK Oxyz, м
        if(find_float(in,"Fx_st="   ,ini.Fx_st     )) { ini.Fx_st    =0;   toMemo("Fx_st not found"); }
        if(find_float(in,"Fy_st="   ,ini.Fy_st     )) { ini.Fy_st    =0;   toMemo("Fy_st not found"); }
        //КИЛЬ
        if(find_int( in,"solveKeel=",ini.solveKeel )) { ini.solveKeel=0;   toMemo("solveKeel not found"); }
        if(find_float(in,"x_k="     ,ini.x_k       )) { ini.x_k      =0;   toMemo("x_k   not found"); }
        if(find_float(in,"y_k="     ,ini.y_k       )) { ini.y_k      =0;   toMemo("y_k   not found"); }
        if(find_float(in,"z_k="     ,ini.z_k       )) { ini.z_k      =0;   toMemo("z_k   not found"); }
        if(find_float(in,"Fx_k="    ,ini.Fx_keel   )) { ini.Fx_keel  =0;   toMemo("Fx_k  not found"); }
        if(find_float(in,"Fz_k="    ,ini.Fz_keel   )) { ini.Fz_keel  =0;   toMemo("Fz_k  not found"); }
        //if(ini.solvePlaner==false) то и все остальные элементы планера не считать
        if(ini.solvePlaner==0) {  ini.solveStab = 0;   ini.solveWing = 0;   ini.solveKeel = 0;  }
        //Внешняя нарузка
        if(find_bool( in,"setExtLoads=",ini.extloads)) { ini.extloads=0;   toMemo("setExtLoads  not found"); }
        //Винт несущий
        if(find_bool(in,"solveMainRotor=",ini.mrsolve     )) { ini.mrsolve=0;     toMemo("solveMainRotor not found"); }
        if(find_float(in,"mr.Fx="        ,ini.mr.Fx_      )) { ini.mr.Fx_=0;      toMemo("mr.Fx   not found"); }//rd.H_mr_    = 1674.038;//Продольная сила несущего винта, Н [положительное направление - назад]
        if(find_float(in,"mr.Fy="        ,ini.mr.Fy_      )) { ini.mr.Fy_=0;      toMemo("mr.Fy   not found"); }//rd.T_mr_    = 35527.516;//Тяга несущего винта, Н           [положительное направление - вверх]
        if(find_float(in,"mr.Fz="        ,ini.mr.Fz_      )) { ini.mr.Fz_=0;      toMemo("mr.Fz   not found"); }//rd.S_mr_    = -262.093;//Боковая сила несущего винта, Н    [положительное направление - влево]
        if(find_float(in,"mr.Mx="        ,ini.mr.Mx_      )) { ini.mr.Mx_=0;      toMemo("mr.Mx   not found"); }//rd.Mx_mr_   = 256.192;//Момент крена несущего винта, Нм
        if(find_float(in,"mr.My="        ,ini.mr.My_      )) { ini.mr.My_=0;      toMemo("mr.My   not found"); }//rd.My_mr_   = 9350.742;//Момент кручения, Нм
        if(find_float(in,"mr.Mz="        ,ini.mr.Mz_      )) { ini.mr.Mz_=0;      toMemo("mr.Mz   not found"); }//rd.Mz_mr_   = -905.401;//Момент тангажа несущего винта, Нм
        if(find_float(in,"mr.D="         ,ini.mr.D        )) { ini.mr.D=1000;     toMemo("mr.D    not found"); }
        if(find_float(in,"x_mr="         ,ini.mr.Ox       )) { ini.mr.Ox=0;       toMemo("x_mr    not found"); }//
        if(find_float(in,"y_mr="         ,ini.mr.Oy       )) { ini.mr.Oy=0;       toMemo("y_mr    not found"); }//
        if(find_float(in,"z_mr="         ,ini.mr.Oz       )) { ini.mr.Oz=0;       toMemo("z_mr    not found"); }//
        if(find_bool( in,"mrtilt="       ,ini.mrtilt      )) { ini.mrtilt =0;     }
        if(find_float(in,"mr.chi="       ,ini.mr.chi_     )) { ini.mr.chi_=0;     toMemo("mr.chi  not found"); }// Угол отклонения тарелки АП или всей втулки(при ismrtilt=1) в продольном направлении, рад
        if(find_float(in,"mr.eta="       ,ini.mr.eta_     )) { ini.mr.eta_=0;     toMemo("mr.eta  not found"); }// Угол отклонения тарелки АП или всей втулки(при ismrtilt=1) в поперечном направлении, рад
        if(find_float(in,"mr.fi0="       ,ini.mr.fi0_     )) { ini.mr.fi0_=0;     toMemo("mr.fi0  not found"); }
        if(find_float(in,"mr.rpm="       ,ini.mr.rpm      )) { ini.mr.rpm =0;     toMemo("mr.rpm  not found"); }
        if(find_int(  in,"mr.rpm|fi0="   ,ini.mr.rpmORfi0 )) { ini.mr.rpmORfi0=1; toMemo("mr.rpm|fi0 not found"); }//определять угол ОШ (1) для ЭВ
        if(find_float(in,"mr.vs="        ,ini.mr.vs_      )) { ini.mr.vs_= 0;     toMemo("mr.vs   not found"); }
        if(find_float(in,"mr.skos="      ,ini.mr.skos     )) { ini.mr.skos=0;     toMemo("mr.skos not found"); }
        if(find_float(in,"mr.Ya="        ,ini.mr.Ya       )) { ini.mr.Ya = 0;     toMemo("mr.Ya   not found"); }//
        if(find_float(in,"mr.Xa="        ,ini.mr.Xa       )) { ini.mr.Xa = 0;     toMemo("mr.Xa   not found"); }//
        if(find_int(  in,"nAzimut="      ,ini.nAz         )) { ini.nAz=360;       toMemo("nAzimut  not found"); }
        if(find_int(  in,"nstabrev="     ,ini.nstabrev    )) { ini.nstabrev=1;    toMemo("nstabrev  not found"); }

        //Винт рулевой
        if(find_bool(in,"solveTailRotor=", ini.trsolve    )) { ini.TRmount=false; ini.trsolve=0; }  else
        {
            ini.TRmount = true;
            if(find_float(in,"tr.Fx="     ,ini.tr.Fx_     )) { ini.tr.Fx_=0;      toMemo("tr.Fx   not found"); }
            if(find_float(in,"tr.Fy="     ,ini.tr.Fy_     )) { ini.tr.Fy_=0;      toMemo("tr.Fy   not found"); }
            if(find_float(in,"tr.Fz="     ,ini.tr.Fz_     )) { ini.tr.Fz_=0;      toMemo("tr.Fz   not found"); }
            if(find_float(in,"tr.Mx="     ,ini.tr.Mx_     )) { ini.tr.Mx_=0;      toMemo("tr.Mx   not found"); }
            if(find_float(in,"tr.My="     ,ini.tr.My_     )) { ini.tr.My_=0;      toMemo("tr.My   not found"); }
            if(find_float(in,"tr.Mz="     ,ini.tr.Mz_     )) { ini.tr.Mz_=0;      toMemo("tr.Mz   not found"); }
            if(find_float(in,"tr.D="      ,ini.tr.D       )) { ini.tr.D=1000;     toMemo("tr.D    not found"); }
            if(find_float(in,"x_tr="      ,ini.tr.Ox      )) { ini.tr.Ox=0;       toMemo("x_tr    not found"); }//=67456812;//Расстояние от оси несущего винта до оси рулевого винта, м
            if(find_float(in,"y_tr="      ,ini.tr.Oy      )) { ini.tr.Oy=0;       toMemo("y_tr    not found"); }//=6711039;//Расстояние от оси рулевого винта до Oxyz. м
            if(find_float(in,"z_tr="      ,ini.tr.Oz      )) { ini.tr.Oz=0;       toMemo("z_tr    not found"); }//=5;//Расстояние от начала координат до плоскости вращения рулевого винта по оси Oz. м
            if(find_float(in,"tr.fi0="    ,ini.tr.fi0_    )) { ini.tr.fi0_=0;     toMemo("tr.fi0  not found"); }
            if(find_float(in,"tr.rpm="    ,ini.tr.rpm     )) { ini.tr.rpm =0;     toMemo("tr.rpm  not found"); }
            if(find_int(  in,"tr.rpm|fi0=",ini.tr.rpmORfi0)) { ini.tr.rpmORfi0=1; toMemo("tr.rpm|fi0 not found"); }//определять угол ОШ (1) для ЭВ
            if(find_float(in,"tr.vt="     ,ini.tr.vt_     )) { ini.tr.vt_= 0;     toMemo("tr.vt   not found"); }
            if(find_float(in,"tr.skos="   ,ini.tr.skos    )) { ini.tr.skos=0;     toMemo("tr.skos not found"); }
        }

        //Винт толкающий
        if(find_bool(in,"solvePushRotor=", ini.prsolve    )) { ini.PRmount=false; ini.prsolve=0; }  else
        {
            ini.PRmount = true;
            if(find_float(in,"pr.Fx="     ,ini.pr.Fx_     )) { ini.pr.Fx_=0;      toMemo("pr.Fx   not found"); }
            if(find_float(in,"pr.Fy="     ,ini.pr.Fy_     )) { ini.pr.Fy_=0;      toMemo("pr.Fy   not found"); }
            if(find_float(in,"pr.Fz="     ,ini.pr.Fz_     )) { ini.pr.Fz_=0;      toMemo("pr.Fz   not found"); }
            if(find_float(in,"pr.Mx="     ,ini.pr.Mx_     )) { ini.pr.Mx_=0;      toMemo("pr.Mx   not found"); }
            if(find_float(in,"pr.My="     ,ini.pr.My_     )) { ini.pr.My_=0;      toMemo("pr.My   not found"); }
            if(find_float(in,"pr.Mz="     ,ini.pr.Mz_     )) { ini.pr.Mz_=0;      toMemo("pr.Mz   not found"); }
            if(find_float(in,"pr.D="      ,ini.pr.D       )) { ini.pr.D=1000;     toMemo("pr.D    not found"); }
            if(find_float(in,"x_pr="      ,ini.pr.Ox      )) { ini.pr.Ox=0;       toMemo("x_pr    not found"); }
            if(find_float(in,"y_pr="      ,ini.pr.Oy      )) { ini.pr.Oy=0;       toMemo("y_pr    not found"); }
            if(find_float(in,"z_pr="      ,ini.pr.Oz      )) { ini.pr.Oz=0;       toMemo("z_pr    not found"); }
            if(find_float(in,"pr.fi0="    ,ini.pr.fi0_    )) { ini.pr.fi0_=0;     toMemo("pr.fi0  not found"); }
            if(find_float(in,"pr.rpm="    ,ini.pr.rpm     )) { ini.pr.rpm =0;     toMemo("pr.rpm  not found"); }
            if(find_int(  in,"pr.rpm|fi0=",ini.pr.rpmORfi0)) { ini.pr.rpmORfi0=1; toMemo("pr.rpm|fi0 not found"); }//определять угол ОШ (1) для ЭВ
            if(find_float(in,"pr.vt="     ,ini.pr.vt_     )) { ini.pr.vt_= 0;     toMemo("pr.vt   not found"); }
            if(find_float(in,"pr.Kt="     ,ini.pr.Kt      )) { ini.pr.Kt=0;       toMemo("pr.Kt   not found"); }
        }

        //Шасси
        if(read_sts(0,".")) readerror_in=true; //fore - передние стойки
        if(read_sts(1,":")) readerror_in=true; //main - основные стойки
        if(read_sts(2,"|")) readerror_in=true; //tail - хвостовая опора
        //Поверхность посадки
        int n_st = ini.st[0].nst + ini.st[1].nst + ini.st[2].nst;
        for ( int k = 0; k < n_st; k++)
        {
            grndstr = "ground." + ss.setNum(k) + "=";
            if( find_float(in, grndstr, ini.ground[k] )) { ini.ground[k]=0; toMemo(grndstr + " not found"); }
        }
        //ГРУЗ НА ВНЕШНЕЙ ТРОСОВОЙ ПОДВЕСКЕ
        if(find_float(in,"mcg="      ,ini.mcg       )) { ini.mcg = 0; }
        if(find_float(in,"lcg="      ,ini.lcg       )) { ini.lcg = 0; }
        if(find_float(in,"xcg="      ,ini.xcg       )) { ini.xcg = 0; }
        if(find_float(in,"ycg="      ,ini.ycg       )) { ini.ycg = 0; }
        if(find_float(in,"zcg="      ,ini.zcg       )) { ini.zcg = 0; }
        //ограничение перемещений
        if(find_int(  in,"fix_x="    ,ini.fix_org_x )) { ini.fix_org_x = 0; }
        if(find_int(  in,"fix_y="    ,ini.fix_org_y )) { ini.fix_org_y = 0; }
        if(find_int(  in,"fix_z="    ,ini.fix_org_z )) { ini.fix_org_z = 0; }
        if(find_int(  in,"fix_k="    ,ini.fix_org_k )) { ini.fix_org_k = 0; }
        if(find_int(  in,"fix_r="    ,ini.fix_org_r )) { ini.fix_org_r = 0; }
        if(find_int(  in,"fix_t="    ,ini.fix_org_t )) { ini.fix_org_t = 0; }
        //корректировка контура планера frm.cntr
        if(find_int(  in,"frmdir.x=" ,frmpos.xdir   )) { frmpos.xdir =1; }
        if(find_int(  in,"frmdir.y=" ,frmpos.ydir   )) { frmpos.ydir =1; }
        if(find_int(  in,"frmdir.z=" ,frmpos.zdir   )) { frmpos.zdir =1; }
        if(find_float(in,"frmpos.x=" ,frmpos.xbias  )) { frmpos.xbias=0; }
        if(find_float(in,"frmpos.y=" ,frmpos.ybias  )) { frmpos.ybias=0; }
        if(find_float(in,"frmpos.z=" ,frmpos.zbias  )) { frmpos.zbias=0; }

        //связь с дополнительными файлами
        if(ini.mrsolve)
        {  if(find_str( in,"data.mr:", fname_mr    )) { readerror_in=true; toMemo("data.mr:  not found"); } }
        if(ini.trsolve)
        {  if(find_str( in,"data.tr:", fname_tr    )) { readerror_in=true; toMemo("data.tr:  not found"); } }
        if(ini.prsolve)
        {  if(find_str( in,"data.pr:", fname_pr    )) { readerror_in=true; toMemo("data.pr:  not found"); } }
        if(ini.solvePlaner)
        {  if(find_str( in,"data.fax:", fname_fax  )) { readerror_in=true; toMemo("data.fax: not found"); } }

        find_str( in,"ext.load:" , fname_load1  );
        find_str( in,"frame.stl:", fname_cntr   );//(контур фюзеляжа)
        find_str( in,"abs.tst:"  , fname_abstst );
        fname_fax   = getFolderName(infilename) + fname_fax   ;
        fname_mr    = getFolderName(infilename) + fname_mr    ;
        fname_tr    = getFolderName(infilename) + fname_tr    ;
        fname_pr    = getFolderName(infilename) + fname_pr    ;
        fname_load1 = getFolderName(infilename) + fname_load1 ;
        fname_cntr  = getFolderName(infilename) + fname_cntr  ;
        fname_abstst= getFolderName(infilename) + fname_abstst;
        fname_mrr  = ini.reportfolder + titlefilename + ".mrr";
        fname_trr  = ini.reportfolder + titlefilename + ".trr";
        fname_prr  = ini.reportfolder + titlefilename + ".prr";
        fname_bal  = ini.reportfolder + titlefilename + ".bal";

        //характеристики аэродинамических сечений
        if(ini.mrsolve || ini.trsolve || ini.prsolve)
        {
            QString profiledir = inifiledir;//getFolderName(infilename);
            int prn = 1;
            while (!find_str(in, "profile#" + ss.setNum(prn) + ":", fname_profile[prn] ))
            {
                ini.profiles[prn-1] = fname_profile[prn];
                fname_profile[prn]  = profiledir + fname_profile[prn];
                prn++;
            };
            ini.ProfilesAmount = prn-1;
            if ( ini.ProfilesAmount < 1 || profiledir.size() < 1 )
            {
                readerror_in = true;
                toMemo("не считаны характеристики сечений лопасти НВ, fname_profile[1]:" + fname_profile[1]);
            }
            else
                readProfile();
        }
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
        return 1;
}
bool MainWindow::read_sts(int k, QString prefix)//считывание данных шасси
{
    bool readerror_sts = 0;
    ini.st[k].ab.c1.v_h.clear();
    ini.st[k].ab.c1.v_f.clear();
    ini.st[k].ab.c1.v_fr.clear();
    ini.st[k].ab.c2.v_h.clear();
    ini.st[k].ab.c2.v_f.clear();
    ini.st[k].ab.c2.v_fr.clear();

    if( find_int(  in, strSumm(prefix,"nst="), ini.st[k].nst)) { ini.st[k].nst = 0; }
    if(ini.st[k].nst>0)
    {
        if( find_int(  in, strSumm(prefix,"shtype=" ), ini.st[k].stype     )) { readerror_sts=true; toMemo(prefix + "shtype  not found"); }//констр.исполнение стойки ( 0 - полозковое; 1 - пирамидальное; 2 - рычажное )
        if(ini.st[k].nst==1 && ini.st[k].stype==1)
        { readerror_sts=true; toMemo("количество стоек шасси пирамидального типа не может быть меньше двух"); }
        if( find_int(  in, strSumm(prefix,"p.type=" ), ini.st[k].ptype     )) { readerror_sts=true; toMemo(prefix + "p.type  not found"); }//констр.исполнение пневматика
        if( find_int(  in, strSumm(prefix,"a.type=" ), ini.st[k].atype     )) { readerror_sts=true; toMemo(prefix + "a.type  not found"); }//констр.исполнение амортизатора ( 1 - однокамерный, 2 - две раздельные газ.камеры, 3 - общая газ.камера на два штока )
        if( find_int(  in, strSumm(prefix,"nwh="    ), ini.st[k].nwh       )) { readerror_sts=true; toMemo(prefix + "nwh     not found"); }//fwheel = FindInt( isFile,".nwheel=");//
        if(ini.st[k].nwh==2)
        if( find_float(in, strSumm(prefix,"whd="    ), ini.st[k].whd       )) { readerror_sts=true; toMemo(prefix + "whd     not found"); }//fwheel = FindInt( isFile,".nwheel=");//
        if( find_float(in, strSumm(prefix,"O.x="    ), ini.st[k].O_.x      )) { readerror_sts=true; toMemo(prefix + "O.x     not found"); }
        if( find_float(in, strSumm(prefix,"O.y="    ), ini.st[k].O_.y      )) { readerror_sts=true; toMemo(prefix + "O.y     not found"); }
        if( find_float(in, strSumm(prefix,"O.z="    ), ini.st[k].O_.z      )) { readerror_sts=true; toMemo(prefix + "O.z     not found"); }
        if( find_float(in, strSumm(prefix,"A.x="    ), ini.st[k].A_.x      )) { readerror_sts=true; toMemo(prefix + "A.x     not found"); }
        if( find_float(in, strSumm(prefix,"A.y="    ), ini.st[k].A_.y      )) { readerror_sts=true; toMemo(prefix + "A.y     not found"); }
        if( find_float(in, strSumm(prefix,"A.z="    ), ini.st[k].A_.z      )) { readerror_sts=true; toMemo(prefix + "A.z     not found"); }
        if( find_float(in, strSumm(prefix,"B.x="    ), ini.st[k].B_.x      )) { readerror_sts=true; toMemo(prefix + "B.x     not found"); }
        if( find_float(in, strSumm(prefix,"B.y="    ), ini.st[k].B_.y      )) { readerror_sts=true; toMemo(prefix + "B.y     not found"); }
        if( find_float(in, strSumm(prefix,"B.z="    ), ini.st[k].B_.z      )) { readerror_sts=true; toMemo(prefix + "B.z     not found"); }
        if( find_float(in, strSumm(prefix,"W.x="    ), ini.st[k].W_.x      )) { readerror_sts=true; toMemo(prefix + "W.x     not found"); }
        if( find_float(in, strSumm(prefix,"W.y="    ), ini.st[k].W_.y      )) { readerror_sts=true; toMemo(prefix + "W.y     not found"); }
        if( find_float(in, strSumm(prefix,"W.z="    ), ini.st[k].W_.z      )) { readerror_sts=true; toMemo(prefix + "W.z     not found"); }
        if((ini.st[k].nst>1 && ini.st[k].B_.z==0)||(ini.st[k].nst>1 && ini.st[k].W_.z==0))
        { readerror_sts=true; toMemo("у стоек шасси пирамидального типа z-координаты должны быть больше нуля"); }

        if(ini.st[k].stype==2)
        {
            if( find_float(in, strSumm(prefix,"Q.x="), ini.st[k].Q_.x      )) { readerror_sts=true; toMemo(prefix + "Q.x     not found"); }
            if( find_float(in, strSumm(prefix,"Q.y="), ini.st[k].Q_.y      )) { readerror_sts=true; toMemo(prefix + "Q.y     not found"); }
            if( find_float(in, strSumm(prefix,"Q.z="), ini.st[k].Q_.z      )) { readerror_sts=true; toMemo(prefix + "Q.z     not found"); }
        }
        else
        {
            ini.st[k].Q_.x = ini.st[k].O_.x;
            ini.st[k].Q_.y = ini.st[k].O_.y;
            ini.st[k].Q_.z = ini.st[k].O_.z;
        }
        if(ini.st[k].stype==8)
        {
            if( find_float(in, strSumm(prefix,"V.x="), ini.st[k].V_.x      )) { readerror_sts=true; toMemo(prefix + "V.x     not found"); }
            if( find_float(in, strSumm(prefix,"V.y="), ini.st[k].V_.y      )) { readerror_sts=true; toMemo(prefix + "V.y     not found"); }
            if( find_float(in, strSumm(prefix,"V.z="), ini.st[k].V_.z      )) { readerror_sts=true; toMemo(prefix + "V.z     not found"); }
        }
        else
        {
            ini.st[k].V_.x = ini.st[k].B_.x;
            ini.st[k].V_.y = ini.st[k].B_.y;
            ini.st[k].V_.z = ini.st[k].B_.z;
        }

        if( find_float(in, strSumm(prefix,"lambda=" ), ini.st[k].lambda    )) { ini.st[k].lambda=0; toMemo(prefix + "lambda  not found"); }
        if( find_float(in, strSumm(prefix,"ang_wh=" ), ini.st[k].ang_wh    )) { ini.st[k].ang_wh=0; toMemo(prefix + "ang_wh  not found"); }
        if( find_float(in, strSumm(prefix,"p.dpmd=" ), ini.st[k].pn.dpmd   )) { readerror_sts=true; toMemo(prefix + "p.dpmd  not found"); }
        if( find_float(in, strSumm(prefix,"p.cpx="  ), ini.st[k].pn.cpx    )) { readerror_sts=true; toMemo(prefix + "p.cpx   not found"); }
        if( find_float(in, strSumm(prefix,"p.cpy="  ), ini.st[k].pn.cpy    )) { readerror_sts=true; toMemo(prefix + "p.cpy   not found"); }
        if( find_float(in, strSumm(prefix,"p.cpz="  ), ini.st[k].pn.cpz    )) { readerror_sts=true; toMemo(prefix + "p.cpz   not found"); }
        if( find_float(in, strSumm(prefix,"p.knp="  ), ini.st[k].pn.alfanp )) { readerror_sts=true; toMemo(prefix + "p.knp   not found"); }
        if( find_float(in, strSumm(prefix,"p.kdp="  ), ini.st[k].pn.kdp    )) { readerror_sts=true; toMemo(prefix + "p.kdp   not found"); }
        if( find_float(in, strSumm(prefix,"p.D="    ), ini.st[k].pn.D      )) { readerror_sts=true; toMemo(prefix + "p.D     not found"); }
        if( find_float(in, strSumm(prefix,"p.B="    ), ini.st[k].pn.B      )) { readerror_sts=true; toMemo(prefix + "p.B     not found"); }
        if( find_int(  in, strSumm(prefix,"p.brake="), ini.st[k].pn.isbrake)) { readerror_sts=true; toMemo(prefix + "p.brake not found"); }
        if( find_float(in, strSumm(prefix,"p.f0="   ), ini.st[k].pn.f0     )) { readerror_sts=true; toMemo(prefix + "p.f0    not found"); }
        if( find_float(in, strSumm(prefix,"p.fs="   ), ini.st[k].pn.fs     )) { readerror_sts=true; toMemo(prefix + "p.fs    not found"); }
        if( find_float(in, strSumm(prefix,"p.fk="   ), ini.st[k].pn.fk     )) { readerror_sts=true; toMemo(prefix + "p.fk    not found"); }
        if(ini.st[k].stype == 0)/// ПОЛОЗКОВОЕ ШАССИ - НЕЛИНЕЙНАЯ ПРУЖИНА
        {//if(ini.st[k].atype == 0)
        if( find_float(in, strSumm(prefix,"a.hu="   ), ini.st[k].ab.hu     )) { readerror_sts=true; toMemo(prefix + "a.hu    not found"); }
        if( find_float(in, strSumm(prefix,"a.Ke="   ), ini.st[k].ab.Ke     )) { readerror_sts=true; toMemo(prefix + "a.Ke    not found"); }
        if( find_float(in, strSumm(prefix,"a.Ki="   ), ini.st[k].ab.Ki     )) { readerror_sts=true; toMemo(prefix + "a.Ki    not found"); }
        if( find_float(in, strSumm(prefix,"a.Ci="   ), ini.st[k].ab.Ci     )) { readerror_sts=true; toMemo(prefix + "a.Ci    not found"); }
        ini.st[k].ab.Vsum   = 0;
        ini.st[k].ab.Voil   = 0;
        ini.st[k].ab.ro     = 0;
        ini.st[k].ab.mu     = 0;
        ini.st[k].ab.p0     = 0;
        ini.st[k].ab.kpoly  = 0;
        ini.st[k].ab.kpolyr = 0;
        if( find_float(in, strSumm(prefix,"a.hmd="  ), ini.st[k].ab.c1.s   )) { readerror_sts=true; toMemo(prefix + "a.hmd  not found"); }
        ini.st[k].ab.c1.Dc  = 0;
        ini.st[k].ab.c1.Dv  = 0;
        ini.st[k].ab.c1.Ds  = 0;
        ini.st[k].ab.c1.fb  = 0;
        ini.st[k].ab.c1.fbr = 0;
        ini.st[k].ab.c1.f   = 0;
        ini.st[k].ab.c1.fr  = 0;
        if( find_float(in, strSumm(prefix,"a.kdmp=" ), ini.st[k].ab.c1.ktr )) { readerror_sts=true; toMemo(prefix + "a.kdmp  not found"); }
        }
        else /// АМОРТИЗАТОР    /// КОЛЕСНОЕ ШАССИ
        {
            ini.st[k].ab.hu = 0;
            ini.st[k].ab.Ke = 0;
            ini.st[k].ab.Ki = 0;
            ini.st[k].ab.Ci = 0;
            if( find_float(in, strSumm(prefix,"a.ro="    ), ini.st[k].ab.ro     )) { readerror_sts=true; toMemo(prefix + "a.ro    not found"); }
            if( find_float(in, strSumm(prefix,"a.mu="    ), ini.st[k].ab.mu     )) { readerror_sts=true; toMemo(prefix + "a.mu    not found"); }
            if( find_float(in, strSumm(prefix,"a.kp="    ), ini.st[k].ab.kpoly  )) { readerror_sts=true; toMemo(prefix + "a.k     not found"); }
            if( find_float(in, strSumm(prefix,"a.kpr="   ), ini.st[k].ab.kpolyr )) { ini.st[k].ab.kpolyr = ini.st[k].ab.kpoly; }

            switch (ini.st[k].atype)//тип амортизатора
            {
            case 1: ///однокамерный
                if( find_float(in, strSumm(prefix,"a1.po="   ), ini.st[k].ab.c1.p0  )) { readerror_sts=true; toMemo(prefix + "a1.po   not found"); }
                if( find_float(in, strSumm(prefix,"a1.Vsum=" ), ini.st[k].ab.c1.Vsum)) { readerror_sts=true; toMemo(prefix + "a1.Vsum not found"); }
                if( find_float(in, strSumm(prefix,"a1.Voil=" ), ini.st[k].ab.c1.Voil)) { readerror_sts=true; toMemo(prefix + "a1.Voil not found"); }
                if( find_float(in, strSumm(prefix,"a1.S="    ), ini.st[k].ab.c1.s   )) { readerror_sts=true; toMemo(prefix + "a1.S    not found"); }
                if( find_float(in, strSumm(prefix,"a1.Dc="   ), ini.st[k].ab.c1.Dc  )) { readerror_sts=true; toMemo(prefix + "a1.Dc   not found"); }
                if( find_float(in, strSumm(prefix,"a1.Ds="   ), ini.st[k].ab.c1.Ds  )) { readerror_sts=true; toMemo(prefix + "a1.Ds   not found"); }
                if( find_float(in, strSumm(prefix,"a1.Dv="   ), ini.st[k].ab.c1.Dv  )) { readerror_sts=true; toMemo(prefix + "a1.Dv   not found"); }
                if( find_float(in, strSumm(prefix,"a1.ktr="  ), ini.st[k].ab.c1.ktr )) { readerror_sts=true; toMemo(prefix + "a1.ktr  not found"); }
                if( find_float(in, strSumm(prefix,"a1.fb="   ), ini.st[k].ab.c1.fb  )) { ini.st[k].ab.c1.fb = 0; }
                if( find_float(in, strSumm(prefix,"a1.fbr="  ), ini.st[k].ab.c1.fbr )) { ini.st[k].ab.c1.fbr = ini.st[k].ab.c1.fb; }
                //переменные "a1.f=" по "a1.h="
                if(find_vectorFloat(in,strSumm(prefix,"a1.h="), ini.st[k].ab.c1.v_h ))
                {//не найден "a1.h="
                    if( find_float(in,strSumm(prefix,"a1.f=" ), ini.st[k].ab.c1.f   )) { ini.st[k].ab.c1.f = 0; }
                    if( find_float(in,strSumm(prefix,"a1.fr="), ini.st[k].ab.c1.fr  )) { ini.st[k].ab.c1.fr = ini.st[k].ab.c1.f; }
                }
                else
                {//найден "a1.h="
                    if(find_vectorFloat(in,strSumm(prefix,"a1.f=" ), ini.st[k].ab.c1.v_f )) { readerror_sts=true; toMemo(prefix + "a1.f  not found in " ); }
                    if(find_vectorFloat(in,strSumm(prefix,"a1.fr="), ini.st[k].ab.c1.v_fr)) { ini.st[k].ab.c1.v_fr = ini.st[k].ab.c1.v_f; }
                }

                ini.st[k].ab.c2.s = 0;
                ini.st[k].ab.c2.p0 = 0;
                ini.st[k].ab.c2.Vsum = 0;
                ini.st[k].ab.c2.Voil = 0;
                ini.st[k].ab.Vsum = ini.st[k].ab.c1.Vsum;
                ini.st[k].ab.Voil = ini.st[k].ab.c1.Voil;
                ini.st[k].ab.p0   = ini.st[k].ab.c1.p0;
                break;

            case 2: ///двухкамерный  (Ми-8)
                if( find_float(in, strSumm(prefix,"a1.po="   ), ini.st[k].ab.c1.p0  )) { readerror_sts=true; toMemo(prefix + "a1.po   not found"); }
                if( find_float(in, strSumm(prefix,"a1.Vsum=" ), ini.st[k].ab.c1.Vsum)) { readerror_sts=true; toMemo(prefix + "a1.Vsum not found"); }
                if( find_float(in, strSumm(prefix,"a1.Voil=" ), ini.st[k].ab.c1.Voil)) { readerror_sts=true; toMemo(prefix + "a1.Voil not found"); }
                if( find_float(in, strSumm(prefix,"a1.S="    ), ini.st[k].ab.c1.s   )) { readerror_sts=true; toMemo(prefix + "a1.S    not found"); }
                if( find_float(in, strSumm(prefix,"a1.Dc="   ), ini.st[k].ab.c1.Dc  )) { readerror_sts=true; toMemo(prefix + "a1.Dc   not found"); }
                if( find_float(in, strSumm(prefix,"a1.Ds="   ), ini.st[k].ab.c1.Ds  )) { readerror_sts=true; toMemo(prefix + "a1.Ds   not found"); }
                if( find_float(in, strSumm(prefix,"a1.Dv="   ), ini.st[k].ab.c1.Dv  )) { readerror_sts=true; toMemo(prefix + "a1.Dv   not found"); }
                if( find_float(in, strSumm(prefix,"a1.ktr="  ), ini.st[k].ab.c1.ktr )) { readerror_sts=true; toMemo(prefix + "a1.ktr  not found"); }
                if( find_float(in, strSumm(prefix,"a1.fb="   ), ini.st[k].ab.c1.fb  )) { ini.st[k].ab.c1.fb = 0; }
                if( find_float(in, strSumm(prefix,"a1.fbr="  ), ini.st[k].ab.c1.fbr )) { ini.st[k].ab.c1.fbr = ini.st[k].ab.c1.fb; }
                //переменные "a1.f=" по "a1.h="
                if(find_vectorFloat(in,strSumm(prefix,"a1.h="), ini.st[k].ab.c1.v_h ))
                {//не найден "a1.h="
                    if( find_float(in,strSumm(prefix,"a1.f=" ), ini.st[k].ab.c1.f   )) { ini.st[k].ab.c1.f = 0; }
                    if( find_float(in,strSumm(prefix,"a1.fr="), ini.st[k].ab.c1.fr  )) { ini.st[k].ab.c1.fr = ini.st[k].ab.c1.f; }
                }
                else
                {//найден "a1.h="
                    if(find_vectorFloat(in,strSumm(prefix,"a1.f=" ), ini.st[k].ab.c1.v_f )) { readerror_sts=true; toMemo(prefix + "a1.f  not found in " ); }
                    if(find_vectorFloat(in,strSumm(prefix,"a1.fr="), ini.st[k].ab.c1.v_fr)) { ini.st[k].ab.c1.v_fr = ini.st[k].ab.c1.v_f; }
                }

                if( find_float(in, strSumm(prefix,"a2.po="   ), ini.st[k].ab.c2.p0  )) { readerror_sts=true; toMemo(prefix + "a2.po   not found"); }
                if( find_float(in, strSumm(prefix,"a2.Vsum=" ), ini.st[k].ab.c2.Vsum)) { readerror_sts=true; toMemo(prefix + "a2.Vsum not found"); }
                if( find_float(in, strSumm(prefix,"a2.Voil=" ), ini.st[k].ab.c2.Voil)) { readerror_sts=true; toMemo(prefix + "a2.Voil not found"); }
                if( find_float(in, strSumm(prefix,"a2.S="    ), ini.st[k].ab.c2.s   )) { readerror_sts=true; toMemo(prefix + "a2.S    not found"); }
                if( find_float(in, strSumm(prefix,"a2.Dc="   ), ini.st[k].ab.c2.Dc  )) { readerror_sts=true; toMemo(prefix + "a2.Dc   not found"); }
                if( find_float(in, strSumm(prefix,"a2.Ds="   ), ini.st[k].ab.c2.Ds  )) { readerror_sts=true; toMemo(prefix + "a2.Ds   not found"); }
                if( find_float(in, strSumm(prefix,"a2.Dv="   ), ini.st[k].ab.c2.Dv  )) { readerror_sts=true; toMemo(prefix + "a2.Dv   not found"); }
                if( find_float(in, strSumm(prefix,"a2.ktr="  ), ini.st[k].ab.c2.ktr )) { readerror_sts=true; toMemo(prefix + "a2.ktr  not found"); }
                if( find_float(in, strSumm(prefix,"a2.fb="   ), ini.st[k].ab.c2.fb  )) { ini.st[k].ab.c2.fb = 0; }
                if( find_float(in, strSumm(prefix,"a2.fbr="  ), ini.st[k].ab.c2.fbr )) { ini.st[k].ab.c2.fbr = ini.st[k].ab.c2.fb; }
                //переменные "a2.f=" по "a2.h="
                if(find_vectorFloat(in,strSumm(prefix,"a2.h="), ini.st[k].ab.c2.v_h ))
                {//не найден "a2.h="
                    if( find_float(in,strSumm(prefix,"a2.f=" ), ini.st[k].ab.c2.f   )) { ini.st[k].ab.c2.f = 0; }
                    if( find_float(in,strSumm(prefix,"a2.fr="), ini.st[k].ab.c2.fr  )) { ini.st[k].ab.c2.fr = ini.st[k].ab.c2.f; }
                }
                else
                {//найден "a2.h="
                    if(find_vectorFloat(in,strSumm(prefix,"a2.f=" ), ini.st[k].ab.c2.v_f )) { readerror_sts=true; toMemo(prefix + "a2.f  not found in " ); }
                    if(find_vectorFloat(in,strSumm(prefix,"a2.fr="), ini.st[k].ab.c2.v_fr)) { ini.st[k].ab.c2.v_fr = ini.st[k].ab.c2.v_f; }
                }

                ini.st[k].ab.Vsum = 0;
                ini.st[k].ab.Voil = 0;
                ini.st[k].ab.p0   = 0;
                break;

            case 3: ///два штока с общей газ.камерой (Ми-2)
                if( find_float(in, strSumm(prefix,"a.po="    ), ini.st[k].ab.p0     )) { readerror_sts=true; toMemo(prefix + "a.po    not found"); }
                if( find_float(in, strSumm(prefix,"a.Vsum="  ), ini.st[k].ab.Vsum   )) { readerror_sts=true; toMemo(prefix + "a.Vsum  not found"); }
                if( find_float(in, strSumm(prefix,"a.Voil="  ), ini.st[k].ab.Voil   )) { readerror_sts=true; toMemo(prefix + "a.Voil  not found"); }
                if( find_float(in, strSumm(prefix,"a1.S="    ), ini.st[k].ab.c1.s   )) { readerror_sts=true; toMemo(prefix + "a1.S    not found"); }
                if( find_float(in, strSumm(prefix,"a1.Dc="   ), ini.st[k].ab.c1.Dc  )) { readerror_sts=true; toMemo(prefix + "a1.Dc   not found"); }
                if( find_float(in, strSumm(prefix,"a1.Ds="   ), ini.st[k].ab.c1.Ds  )) { readerror_sts=true; toMemo(prefix + "a1.Ds   not found"); }
                if( find_float(in, strSumm(prefix,"a1.Dv="   ), ini.st[k].ab.c1.Dv  )) { readerror_sts=true; toMemo(prefix + "a1.Dv   not found"); }
                if( find_float(in, strSumm(prefix,"a1.f="    ), ini.st[k].ab.c1.f   )) { ini.st[k].ab.c1.f = 0; }
                if( find_float(in, strSumm(prefix,"a1.fr="   ), ini.st[k].ab.c1.fr  )) { ini.st[k].ab.c1.fr = ini.st[k].ab.c1.f; }
                if( find_float(in, strSumm(prefix,"a1.fb="   ), ini.st[k].ab.c1.fb  )) { ini.st[k].ab.c1.fb = 0; }
                if( find_float(in, strSumm(prefix,"a1.fbr="  ), ini.st[k].ab.c1.fbr )) { ini.st[k].ab.c1.fbr = ini.st[k].ab.c1.fb; }
                if( find_float(in, strSumm(prefix,"a1.ktr="  ), ini.st[k].ab.c1.ktr )) { readerror_sts=true; toMemo(prefix + "a1.ktr  not found"); }
                if( find_float(in, strSumm(prefix,"a2.S="    ), ini.st[k].ab.c2.s   )) { readerror_sts=true; toMemo(prefix + "a2.S    not found"); }
                if( find_float(in, strSumm(prefix,"a2.Dc="   ), ini.st[k].ab.c2.Dc  )) { readerror_sts=true; toMemo(prefix + "a2.Dc   not found"); }
                if( find_float(in, strSumm(prefix,"a2.Ds="   ), ini.st[k].ab.c2.Ds  )) { readerror_sts=true; toMemo(prefix + "a2.Ds   not found"); }
                if( find_float(in, strSumm(prefix,"a2.Dv="   ), ini.st[k].ab.c2.Dv  )) { readerror_sts=true; toMemo(prefix + "a2.Dv   not found"); }
                if( find_float(in, strSumm(prefix,"a2.f="    ), ini.st[k].ab.c2.f   )) { ini.st[k].ab.c2.f = 0; }
                if( find_float(in, strSumm(prefix,"a2.fr="   ), ini.st[k].ab.c2.fr  )) { ini.st[k].ab.c2.fr = ini.st[k].ab.c2.f; }
                if( find_float(in, strSumm(prefix,"a2.fb="   ), ini.st[k].ab.c2.fb  )) { ini.st[k].ab.c2.fb = 0; }
                if( find_float(in, strSumm(prefix,"a2.fbr="  ), ini.st[k].ab.c2.fbr )) { ini.st[k].ab.c2.fbr = ini.st[k].ab.c2.fb; }
                if( find_float(in, strSumm(prefix,"a2.ktr="  ), ini.st[k].ab.c2.ktr )) { readerror_sts=true; toMemo(prefix + "a2.ktr  not found"); }
                ini.st[k].ab.c1.Vsum = 0;
                ini.st[k].ab.c1.Voil = 0;
                ini.st[k].ab.c1.p0   = 0;
                ini.st[k].ab.c2.Vsum = 0;
                ini.st[k].ab.c2.Voil = 0;
                ini.st[k].ab.c2.p0   = 0;
                break;
            }
        }
    }
return readerror_sts;
}
bool MainWindow::read_fax(QString infileName)//считывание аэродинамических параметров планера
{
    bool readerror_fax = 0;
    int  fax_method;
    QFile infile(infileName);
    if(infile.open( QIODevice::ReadOnly | QIODevice::Text )) //if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
    {
        in_fax = new QTextStream(&infile);
        if(find_int(in_fax,"method:", fax_method)) { toMemo("method not found"); }//формат и алгоритм расчета: 1- новый,универсальный; 2- старый по балансировке Ми-2
        if(ini.solvePlaner) ini.solvePlaner=fax_method;
        if(ini.solveStab)   ini.solveStab = fax_method;
        if(ini.solveWing)   ini.solveWing = fax_method;
        if(ini.solveKeel)   ini.solveKeel = fax_method;
        //модель продувки
        if(find_float(in_fax,"lmodel=" , ini.lmodel )) { readerror_fax=true; toMemo("lmodel  not found in " + infileName); }//rd.lmodel   = 0.76;//Линейный размер на модели, м
        if(find_float(in_fax,"Smodel=" , ini.Smodel )) { readerror_fax=true; toMemo("Smodel  not found in " + infileName); }//rd.Smodel   = 0.012;//Площадь миделя модели, м2
        if(find_float(in_fax,"Mmodel=" , ini.Mmodel )) { readerror_fax=true; toMemo("Mmodel  not found in " + infileName); }//rd.Mmodel   = 16.45;//Масштаб модели
        //фюзеляж
        if( find_Xmass(in_fax,"_alfa_f:",              ini._alfa_f,  ini._alfa_f_col  )) { readerror_fax=true; toMemo("_alfa_f: not found in " + infileName); }
        if( find_Xmass(in_fax,"_cxa_f=f(_alfa_f):",    ini._cx_f,    ini._cx_f_col    )) { readerror_fax=true; toMemo("_cx_f=f(_alfa_f): not found in " + infileName); }
        if( find_Xmass(in_fax,"_cya_f=f(_alfa_f):",    ini._cy_f,    ini._cy_f_col    )) { readerror_fax=true; toMemo("_cy_f=f(_alfa_f): not found in " + infileName); }
        if( find_Xmass(in_fax,"_cza_f=f(_alfa_f):",    ini._cz_f,    ini._cz_f_col    )) { readerror_fax=true; toMemo("_cz_f=f(_alfa_f): not found in " + infileName); }
        if( find_Xmass(in_fax,"_mxa_f=f(_alfa_f):",    ini._mx_f,    ini._mx_f_col    )) { readerror_fax=true; toMemo("_mx_f=f(_alfa_f): not found in " + infileName); }
        if( find_Xmass(in_fax,"_mya_f=f(_alfa_f):",    ini._my_f,    ini._my_f_col    )) { readerror_fax=true; toMemo("_my_f=f(_alfa_f): not found in " + infileName); }
        if( find_Xmass(in_fax,"_mza_f=f(_alfa_f):",    ini._mz_f,    ini._mz_f_col    )) { readerror_fax=true; toMemo("_mz_f=f(_alfa_f): not found in " + infileName); }
        //стабилизатор
        if(find_float(in_fax,"est_min=", ini.est_min)) { readerror_fax=true; toMemo("est_min not found in " + infileName); }//УПРАВЛЕНИЯ СТАБИЛИЗАТОРОМ
        if(find_float(in_fax,"est_max=", ini.est_max)) { readerror_fax=true; toMemo("est_max not found in " + infileName); }//УПРАВЛЕНИЯ СТАБИЛИЗАТОРОМ
        if(find_float(in_fax,"fi0_min=", ini.fi0_min)) { readerror_fax=true; toMemo("fi0_min not found in " + infileName); }
        if(find_float(in_fax,"fi0_max=", ini.fi0_max)) { readerror_fax=true; toMemo("fi0_max not found in " + infileName); }
        switch (ini.solveStab)
        {
        case 1:
            //стабилизатор
            if( find_float(in_fax,"S_st="    ,             ini.S_st                       )) { readerror_fax=true; toMemo("S_st    not found in " + infileName); }
            if( find_float(in_fax,"l_st="    ,             ini.l_st                       )) { readerror_fax=true; toMemo("l_st    not found in " + infileName); }
            if( find_float(in_fax,"k_st="    ,             ini.K_st                       )) { readerror_fax=true; toMemo("K_st    not found in " + infileName); }
            if( find_Xmass(in_fax,"_alfa_st:",             ini._alfa_st, ini._alfa_st_col )) { readerror_fax=true; toMemo("_alfa_st: not found in " + infileName); }
            if( find_Xmass(in_fax,"_cxa_st=f(_alfa_st):",  ini._cx_st,   ini._cx_st_col   )) { readerror_fax=true; toMemo("_cx_st=f(_alfa_st): not found in " + infileName); }
            if( find_Xmass(in_fax,"_cya_st=f(_alfa_st):",  ini._cy_st,   ini._cy_st_col   )) { readerror_fax=true; toMemo("_cy_st=f(_alfa_st): not found in " + infileName); }
            if( find_Xmass(in_fax,"_alfa_fs:",             ini._alfa_fs, ini._alfa_fs_col )) { readerror_fax=true; toMemo("_alfa_fs: not found in " + infileName); }
            if( find_Xmass(in_fax,"_dalf_stf=f(_alfa_fs):",ini._dalf_stf,ini._dalf_stf_col)) { readerror_fax=true; toMemo("_dalf_stf=f(_alfa_fs): not found in " + infileName); }
            break;
        case 2://old method as mi2
            //фюзеляж
            //if( find_Xmass(in_fax,"_alfa_f:",              ini._alfa_f,  ini._alfa_f_col  )) { readerror_fax=true; toMemo("_alfa_f: not found"); }
            //if( find_Xmass(in_fax,"_cx_f=f(_alfa_f):",     ini._cx_f,    ini._cx_f_col    )) { readerror_fax=true; toMemo("_cx_f=f(_alfa_f): not found"); }
            //if( find_Xmass(in_fax,"_cy_f=f(_alfa_f):",     ini._cy_f,    ini._cy_f_col    )) { readerror_fax=true; toMemo("_cy_f=f(_alfa_f): not found"); }
            //if( find_Xmass(in_fax,"_cz_f=f(_alfa_f):",     ini._cz_f,    ini._cz_f_col    )) { readerror_fax=true; toMemo("_cz_f=f(_alfa_f): not found"); }
            //if( find_Xmass(in_fax,"_mx_f=f(_alfa_f):",     ini._mx_f,    ini._mx_f_col    )) { readerror_fax=true; toMemo("_mx_f=f(_alfa_f): not found"); }
            //if( find_Xmass(in_fax,"_my_f=f(_alfa_f):",     ini._my_f,    ini._my_f_col    )) { readerror_fax=true; toMemo("_my_f=f(_alfa_f): not found"); }
            //if( find_Xmass(in_fax,"_mz_f=f(_alfa_f):",     ini._mz_f,    ini._mz_f_col    )) { readerror_fax=true; toMemo("_mz_f=f(_alfa_f): not found"); }
            //стабилизатор
            if( find_Xmass(in_fax,"__alfa_f:",                   ini.__alfa_f, ini.__alfa_f_col                 )) { readerror_fax=true; toMemo("__alfa_f: not found in " + infileName); }
            if( find_Xmass(in_fax,"__eps_st:",                   ini.__eps_st, ini.__eps_st_col                 )) { readerror_fax=true; toMemo("__eps_st: not found in " + infileName); }
            if(find_XYmass(in_fax,"__cx_st=f(_eps_st,_alfa_f):", ini.__cx_st,  ini.__cx_st_row, ini.__cx_st_col )) { readerror_fax=true; toMemo("__cx_st=f(_eps_st,_alfa_f): not found in " + infileName); }
            if(find_XYmass(in_fax,"__cy_st=f(_eps_st,_alfa_f):", ini.__cy_st,  ini.__cy_st_row, ini.__cy_st_col )) { readerror_fax=true; toMemo("__cy_st=f(_eps_st,_alfa_f): not found in " + infileName); }
            if(find_XYmass(in_fax,"__mz_st=f(_eps_st,_alfa_f):", ini.__mz_st,  ini.__mz_st_row, ini.__mz_st_col )) { readerror_fax=true; toMemo("__mz_st=f(_eps_st,_alfa_f): not found in " + infileName); }
            if(find_XYmass(in_fax,"_mz_fst=f(_eps_st,_alfa_f):", ini._mz_fst,  ini._mz_fst_row, ini._mz_fst_col )) { readerror_fax=true; toMemo("_mz_fst=f(_eps_st,_alfa_f): not found in " + infileName); }
            break;
        }
        if(ini.solveWing)
        {
            if( find_float(in_fax,"S_wing="    ,             ini.S_wing                         )) { readerror_fax=true; toMemo("S_wing  not found in " + infileName); }
            if( find_float(in_fax,"l_wing="    ,             ini.l_wing                         )) { readerror_fax=true; toMemo("l_wing  not found in " + infileName); }
            if( find_float(in_fax,"k_wing="    ,             ini.K_wing                         )) { readerror_fax=true; toMemo("K_wing  not found in " + infileName); }
            if( find_float(in_fax,"eps_wing="  ,             ini.eps_wing                       )) { readerror_fax=true; toMemo("eps_wing  not found in " + infileName); }
            if( find_Xmass(in_fax,"_alfa_wing:",             ini._alfa_wing, ini._alfa_wing_col )) { readerror_fax=true; toMemo("_alfa_wing: not found in " + infileName); }
            if( find_Xmass(in_fax,"_cxa_wing=f(_alfa_wing):",ini._cx_wing,   ini._cx_wing_col   )) { readerror_fax=true; toMemo("_cx_wing=f(_alfa_wing): not found in " + infileName); }
            if( find_Xmass(in_fax,"_cya_wing=f(_alfa_wing):",ini._cy_wing,   ini._cy_wing_col   )) { readerror_fax=true; toMemo("_cy_wing=f(_alfa_wing): not found in " + infileName); }
        }
        if(ini.solveKeel)
        {
            if( find_float(in_fax,"S_keel="    ,             ini.S_keel                         )) { readerror_fax=true; toMemo("S_keel    not found in " + infileName); }
            if( find_float(in_fax,"k_keel="    ,             ini.K_keel                         )) { readerror_fax=true; toMemo("K_keel    not found in " + infileName); }
            if( find_float(in_fax,"eps_keel="  ,             ini.eps_keel                       )) { readerror_fax=true; toMemo("eps_keel  not found in " + infileName); }
            if( find_Xmass(in_fax,"_alfa_keel:",             ini._alfa_keel, ini._alfa_keel_col )) { readerror_fax=true; toMemo("_alfa_keel: not found in " + infileName); }
            if( find_Xmass(in_fax,"_cxa_keel=f(_alfa_keel):",ini._cx_keel,   ini._cx_keel_col   )) { readerror_fax=true; toMemo("_cx_keel=f(_alfa_keel): not found in " + infileName); }
            if( find_Xmass(in_fax,"_cya_keel=f(_alfa_keel):",ini._cy_keel,   ini._cy_keel_col   )) { readerror_fax=true; toMemo("_cy_keel=f(_alfa_keel): not found in " + infileName); }
        }
        infile.close();
    }
    else
    {
        readerror_fax=true;
        toLogf("cannot open " + infileName);
    }
return readerror_fax;
}
bool MainWindow::read_mr(QString infileName) //считывание исходных данных винта несущего
{//НЕСУЩИЙ ВИНТ
    bool readerror_mr = 0;
    QFile infile(infileName);
    if(infile.open( QIODevice::ReadOnly | QIODevice::Text )) //if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
    {
        in_mr = new QTextStream(&infile);

        if(find_int(  in_mr, "mr.hubtype="  , ini.mr.hubtype  )) { readerror_mr=true; toMemo("mr.hubtype not found in " + infileName); }
        if(find_int(  in_mr, "mr.rdirect="  , ini.mr.rdirect  )) { readerror_mr=true; toMemo("mr.rdirect not found in " + infileName); }
        if(find_int(  in_mr, "mr.z="        , ini.mr.zl       )) { readerror_mr=true; toMemo("mr.z    not found in " + infileName); }
        if(find_float(in_mr, "mr.mlp="      , ini.mr.mlp      )) { readerror_mr=true; toMemo("mr.mlp  not found in " + infileName); }
        if(find_float(in_mr, "mr.Ib0="      , ini.mr.Ib0      )) { readerror_mr=true; toMemo("mr.Ib0  not found in " + infileName); }//Момент инерции лопасти отн. ГШ, кг*м^2
        if(find_float(in_mr, "mr.Sb0="      , ini.mr.Sb0      )) { readerror_mr=true; toMemo("mr.Sb0  not found in " + infileName); }//rd.Sgsh_mr  = 197;//Стат. момент инерции лопасти относительно г.ш, N*m
        if(find_float(in_mr, "mr.lgsh="     , ini.mr.l_gsh    )) { readerror_mr=true; toMemo("mr.lgsh not found in " + infileName); }//rd.lgsh     = 0.102;//Расстояние от оси несущего винта до оси ГШ, м
        if(find_float(in_mr, "mr.lvsh="     , ini.mr.l_vsh    )) { readerror_mr=true; toMemo("mr.lvsh not found in " + infileName); }
        if(find_float(in_mr, "mr.svsh="     , ini.mr.s_vsh    )) { readerror_mr=true; toMemo("mr.svsh not found in " + infileName); }
        if(find_float(in_mr, "mr.kvz="      , ini.mr.kvz      )) { readerror_mr=true; toMemo("mr.kvz  not found in " + infileName); }//rd.k_       = 0.4;//Компенсатор взмаха
        if(find_float(in_mr, "mr.dCxp="     , ini.mr.dCxp     )) { readerror_mr=true; toMemo("mr.dCxp not found in " + infileName); }//
        if(find_float(in_mr, "mr.Bkp="      , ini.mr.Bkp      )) { readerror_mr=true; toMemo("mr.Bkp  not found in " + infileName); }//rd.B_mr     = 0.97;//Коэф. концевых потерь

        if(find_float(in_mr, "mr.D1="       , ini.mr.D1_      )) { readerror_mr=true; toMemo("mr.D1   not found in " + infileName); }//rd.D1_mr_   = 1.5646828;
        if(find_float(in_mr, "mr.D2="       , ini.mr.D2_      )) { readerror_mr=true; toMemo("mr.D2   not found in " + infileName); }//rd.D2_mr_   = -0.0883295;
        if(find_float(in_mr, "mr.Rap="      , ini.mr.Rap      )) { readerror_mr=true; toMemo("mr.Rap  not found in " + infileName); }//rd.Rap      = 0.265;//радиус крепления тяг, м
        if(find_float(in_mr, "mr.ll="       , ini.mr.ll       )) { readerror_mr=true; toMemo("mr.ll   not found in " + infileName); }//rd.ll       = 0.157;//
        if(find_float(in_mr, "mr.dpsi_cont=", ini.mr.dpsi_cont)) { readerror_mr=true; toMemo("mr.dpsi_cont not found in " + infileName); }//rd.dpsi_cont= 25 *pi/180;//угол опережения управления, град

        if(find_float(in_mr, "mr.xo="       , ini.mr.xo       )) { readerror_mr=true; toMemo("mr.xo       not found in " + infileName); }
        if(find_float(in_mr, "mr.xt="       , ini.mr.xt       )) { readerror_mr=true; toMemo("mr.xt       not found in " + infileName); }
        if(find_float(in_mr, "mr.Iop="      , ini.mr.Iop      )) { readerror_mr=true; toMemo("mr.Iop    not found in " + infileName); }//Момент инерции лопасти отн. ГШ, кг*м^2
        if(find_float(in_mr, "mr.r_pov="    , ini.mr.r_pov    )) { readerror_mr=true; toMemo("mr.r_pov  not found in " + infileName); }
        if(find_float(in_mr, "mr.k_pov="    , ini.mr.k_pov    )) { readerror_mr=true; toMemo("mr.k_pov  not found in " + infileName); }
        if(find_float(in_mr, "mr.f_pod="    , ini.mr.f_pod    )) { readerror_mr=true; toMemo("mr.f_pod    not found in " + infileName); }
        if(find_float(in_mr, "mr.r_pod="    , ini.mr.r_pod    )) { readerror_mr=true; toMemo("mr.r_pod    not found in " + infileName); }
        if(find_float(in_mr, "mr.sigma3="   , ini.mr.sigma3   )) { readerror_mr=true; toMemo("mr.sigma3 not found in " + infileName); }

        //параметры лопасти
        if(find_vectorFloat(in_mr,"mr.r="     , ini.r_mr   )) { readerror_mr=true; toMemo("mr.r    not found in " + infileName); }//readFloatArr ("r" + _suff, &r_is_[0]);
        if(find_vectorFloat(in_mr,"mr.b="     , ini.bh_mr  )) { readerror_mr=true; toMemo("mr.b    not found in " + infileName); }
        if(find_vectorFloat(in_mr,"mr.dfi="   , ini.dfi_mr )) { readerror_mr=true; toMemo("mr.dfi  not found in " + infileName); }
        if(find_vectorInt(  in_mr,"mr.prof="  , ini.prof_mr)) { readerror_mr=true; toMemo("mr.prof not found in " + infileName); }

        /*//характеристики сечений
        int prn = 1;
        QString profiledir = inifiledir;//getFolderName(infilename);
        while (!find_str(in_mr, "profile" + ss.setNum(prn) + ":", fname_profile[prn] ))
        {
            fname_profile[prn] = profiledir + fname_profile[prn];
            prn++;
        };
        ini.ProfilesAmount = prn-1;
        if ( ini.ProfilesAmount < 1 || profiledir.size() < 1 )
        {   readerror_mr = true;  toMemo("не считаны характеристики сечений лопасти НВ, fname_profile[1]:" + fname_profile[1]);  }
        else
            readProfile();
*/
        //демпфер лопасти
        if( find_bool( in_mr, "damper="  , ini.mr.damper  )) { readerror_mr=true; toMemo("damper  not found"); }
        if(ini.mr.damper)
        {
        if( find_float(in_mr, "d.h="     , ini.mr.d.h     )) { readerror_mr=true; toMemo("d.h     not found"); }
        if( find_float(in_mr, "d.x2="    , ini.mr.d.x2    )) { readerror_mr=true; toMemo("d.x2    not found"); }
        if( find_float(in_mr, "d.z2="    , ini.mr.d.z2    )) { readerror_mr=true; toMemo("d.z2    not found"); }
        if( find_float(in_mr, "d.x3="    , ini.mr.d.x3    )) { readerror_mr=true; toMemo("d.x3    not found"); }
        if( find_float(in_mr, "d.z3="    , ini.mr.d.z3    )) { readerror_mr=true; toMemo("d.z3    not found"); }
        if( find_float(in_mr, "d.ro="    , ini.mr.d.ro    )) { readerror_mr=true; toMemo("d.ro    not found"); }
        if( find_float(in_mr, "d.mu="    , ini.mr.d.mu    )) { readerror_mr=true; toMemo("d.mu    not found"); }
        if( find_float(in_mr, "d.f="     , ini.mr.d.f     )) { readerror_mr=true; toMemo("d.f     not found"); }
        if( find_float(in_mr, "d.f2="    , ini.mr.d.f2    )) { readerror_mr=true; toMemo("d.f2    not found"); }
        if( find_float(in_mr, "d.dv="    , ini.mr.d.dv    )) { readerror_mr=true; toMemo("d.dv    not found"); }
        if( find_float(in_mr, "d.dn="    , ini.mr.d.dn    )) { readerror_mr=true; toMemo("d.dn    not found"); }
        if( find_float(in_mr, "d.pmax="  , ini.mr.d.pmax  )) { readerror_mr=true; toMemo("d.pmax  not found"); }
        }

        //виброгаситель
        ini.mr.vgsolve = 0;//if( find_bool( in_mr,"vabsorber=", ini.mr.vgsolve  )) { readerror_mr=true; toMemo("vabsorber  not found"); }
        if(ini.mr.vgsolve)
        {
        if( find_int(  in_mr, "vg.n="    , ini.mr.vg.n    )) { readerror_mr=true; toMemo("vg.n   not found"); }
        if( find_float(in_mr, "vg.m="    , ini.mr.vg.m    )) { readerror_mr=true; toMemo("vg.m   not found"); }
        if( find_float(in_mr, "vg.h="    , ini.mr.vg.h    )) { readerror_mr=true; toMemo("vg.h   not found"); }
        if( find_float(in_mr, "vg.d="    , ini.mr.vg.d    )) { readerror_mr=true; toMemo("vg.d   not found"); }
        if( find_float(in_mr, "vg.D="    , ini.mr.vg.D    )) { readerror_mr=true; toMemo("vg.D   not found"); }
        }


        if(find_float(in_mr, "mr.alfa_min=" , ini.mr.alfa_min )) { readerror_mr=true; toMemo("mr.alfa_min not found in " + infileName); }
        if(find_float(in_mr, "mr.alfa_max=" , ini.mr.alfa_max )) { readerror_mr=true; toMemo("mr.alfa_max not found in " + infileName); }
        if(find_float(in_mr, "mr.rpm_min="  , ini.mr.rpm_min  )) { readerror_mr=true; toMemo("mr.rpm_min  not found in " + infileName); }
        if(find_float(in_mr, "mr.rpm_max="  , ini.mr.rpm_max  )) { readerror_mr=true; toMemo("mr.rpm_max  not found in " + infileName); }
        if(find_float(in_mr, "mr.fi0_min="  , ini.mr.fi0_min  )) { readerror_mr=true; toMemo("mr.fi0_min  not found in " + infileName); }
        if(find_float(in_mr, "mr.fi0_max="  , ini.mr.fi0_max  )) { readerror_mr=true; toMemo("mr.fi0_max  not found in " + infileName); }
        if(find_float(in_mr, "mr.chi_min="  , ini.mr.chi_min  )) { readerror_mr=true; toMemo("mr.chi_min  not found in " + infileName); }
        if(find_float(in_mr, "mr.chi_max="  , ini.mr.chi_max  )) { readerror_mr=true; toMemo("mr.chi_max  not found in " + infileName); }
        if(find_float(in_mr, "mr.eta_min="  , ini.mr.eta_min  )) { readerror_mr=true; toMemo("mr.eta_min  not found in " + infileName); }
        if(find_float(in_mr, "mr.eta_max="  , ini.mr.eta_max  )) { readerror_mr=true; toMemo("mr.eta_max  not found in " + infileName); }
        if(find_float(in_mr, "mr.bet_min="  , ini.mr.bet_min  )) { readerror_mr=true; toMemo("mr.bet_min  not found in " + infileName); }
        if(find_float(in_mr, "mr.bet_max="  , ini.mr.bet_max  )) { readerror_mr=true; toMemo("mr.bet_max  not found in " + infileName); }
        if(find_float(in_mr, "mr.ksi_min="  , ini.mr.ksi_min  )) { readerror_mr=true; toMemo("mr.ksi_min  not found in " + infileName); }
        if(find_float(in_mr, "mr.ksi_max="  , ini.mr.ksi_max  )) { readerror_mr=true; toMemo("mr.ksi_max  not found in " + infileName); }

        //построение 3D модели лопасти
        xyz frnt,neut,rear;//точки сечения
        mrbldfrnt.clear();
        mrbldneut.clear();
        mrbldrear.clear();
        neut.x= 0;//ini.mr.rvs*0.001;
        neut.y= 0;
        neut.z= 0;
        mrbldfrnt.push_back(neut);
        mrbldneut.push_back(neut);
        mrbldrear.push_back(neut);
        //считать вынос носа сечения отн.ном.линии лопасти
        ini.kh_mr = 0.33;
        for(int bs=0; bs<ini.r_mr.size(); bs++)
        {
            frnt.x = ini.r_mr[bs] - ini.mr.l_vsh*0.001;
            frnt.y = ini.kh_mr*ini.bh_mr[bs]*sin(ini.dfi_mr[bs]*M_PI/180);
            frnt.z = ini.kh_mr*ini.bh_mr[bs]*cos(ini.dfi_mr[bs]*M_PI/180);

            neut.x = ini.r_mr[bs] - ini.mr.l_vsh*0.001;
            neut.y = 0;
            neut.z = 0;

            rear.x = ini.r_mr[bs] - ini.mr.l_vsh*0.001;
            rear.y =-(1-ini.kh_mr)*ini.bh_mr[bs]*sin(ini.dfi_mr[bs]*M_PI/180);
            rear.z =-(1-ini.kh_mr)*ini.bh_mr[bs]*cos(ini.dfi_mr[bs]*M_PI/180);

            mrbldfrnt.push_back(frnt);
            mrbldneut.push_back(neut);
            mrbldrear.push_back(rear);
        }

        ui->oglwidget->mrbldfrnt = &mrbldfrnt;
        ui->oglwidget->mrbldneut = &mrbldneut;
        ui->oglwidget->mrbldrear = &mrbldrear;

        ui->SpinBox_fi0mr->setRange( ini.mr.fi0_min, ini.mr.fi0_max );
        ui->SpinBox_etamr->setRange( ini.mr.eta_min, ini.mr.eta_max );
        ui->SpinBox_chimr->setRange( ini.mr.chi_min, ini.mr.chi_max );

        ui->Slider_fi0mr->setRange( float2int(ini.mr.fi0_min*1000), float2int(ini.mr.fi0_max*1000) );
        ui->Slider_etamr->setRange( float2int(ini.mr.eta_min*1000), float2int(ini.mr.eta_max*1000) );
        ui->Slider_chimr->setRange( float2int(ini.mr.chi_min*1000), float2int(ini.mr.chi_max*1000) );

        /*
        ///демпфер лопасти
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
        ///стенд
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

        //out.mr.Lb  = (0.5*ini.mr.D - ini.mr.l_vsh);
        // */

        //if(find_float(in_mr, "mr.T_="       , ini.mr.T_       )) { readerror_mr=true; toMemo("mr.T_  not found in " + infileName); }//rd.T_mr_    = 35527.516;//Тяга несущего винта, Н           [положительное направление - вверх]
        //if(find_float(in_mr, "mr.H_="       , ini.mr.H_       )) { readerror_mr=true; toMemo("mr.H_  not found in " + infileName); }//rd.H_mr_    = 1674.038;//Продольная сила несущего винта, Н [положительное направление - назад]
        //if(find_float(in_mr, "mr.S_="       , ini.mr.S_       )) { readerror_mr=true; toMemo("mr.S_  not found in " + infileName); }//rd.S_mr_    = -262.093;//Боковая сила несущего винта, Н    [положительное направление - влево]
        //if(find_float(in_mr, "mr.Mx_="      , ini.mr.Mx_      )) { readerror_mr=true; toMemo("mr.Mx_ not found in " + infileName); }//rd.Mx_mr_   = 256.192;//Момент крена несущего винта, Нм
        //if(find_float(in_mr, "mr.My_="      , ini.mr.My_      )) { readerror_mr=true; toMemo("mr.My_ not found in " + infileName); }//rd.My_mr_   = 9350.742;//Момент кручения, Нм
        //if(find_float(in_mr, "mr.Mz_="      , ini.mr.Mz_      )) { readerror_mr=true; toMemo("mr.Mz_ not found in " + infileName); }//rd.Mz_mr_   = -905.401;//Момент тангажа несущего винта, Нм

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
bool MainWindow::read_tr(QString infileName) //считывание исходных данных винта рулевого
{//РУЛЕВОЙ ВИНТ
    bool readerror_tr = 0;
    QFile infile(infileName);
    if(infile.open( QIODevice::ReadOnly | QIODevice::Text )) //if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
    {
        in_tr = new QTextStream(&infile);
        if(find_int(  in_tr, "tr.hubtype="  , ini.tr.hubtype  )) { readerror_tr=true; toMemo("tr.hubtype not found in " + infileName); }
        if(find_int(  in_tr, "tr.rdirect="  , ini.tr.rdirect  )) { readerror_tr=true; toMemo("tr.rdirect not found in " + infileName); }
        if(find_int(  in_tr, "tr.z="        , ini.tr.zl       )) { readerror_tr=true; toMemo("tr.z       not found in " + infileName); }
        if(find_float(in_tr, "tr.mlp="      , ini.tr.mlp      )) { readerror_tr=true; toMemo("tr.mlp     not found in " + infileName); }
        if(find_float(in_tr, "tr.Ib0="      , ini.tr.Ib0      )) { readerror_tr=true; toMemo("tr.Ib0     not found in " + infileName); }//Момент инерции лопасти отн. ГШ, кг*м^2
        if(find_float(in_tr, "tr.Sb0="      , ini.tr.Sb0      )) { readerror_tr=true; toMemo("tr.Sb0     not found in " + infileName); }//rd.Sgsh_mr  = 197;//Стат. момент инерции лопасти относительно г.ш, N*m
        if(find_float(in_tr, "tr.lgsh="     , ini.tr.l_gsh    )) { readerror_tr=true; toMemo("tr.lgsh    not found in " + infileName); }//rd.lgsh     = 0.102;//Расстояние от оси несущего винта до оси ГШ, м
        if(find_float(in_tr, "tr.lvsh="     , ini.tr.l_vsh    )) { ini.tr.l_vsh=ini.tr.l_gsh;  toMemo("tr.lvsh not found in " + infileName); }
        if(find_float(in_tr, "tr.svsh="     , ini.tr.s_vsh    )) { ini.tr.s_vsh=0;             toMemo("tr.svsh not found in " + infileName); }
        if(find_float(in_tr, "tr.kvz="      , ini.tr.kvz      )) { readerror_tr=true; toMemo("tr.kvz     not found in " + infileName); }//rd.k_       = 0.4;//Компенсатор взмаха
        if(find_float(in_tr, "tr.dCxp="     , ini.tr.dCxp     )) { readerror_tr=true; toMemo("tr.dCxp    not found in " + infileName); }//
        if(find_float(in_tr, "tr.Bkp="      , ini.tr.Bkp      )) { readerror_tr=true; toMemo("tr.Bkp     not found in " + infileName); }//rd.B_mr     = 0.97;//Коэф. концевых потерь
        if(find_float(in_tr, "tr.rpm_min="  , ini.tr.rpm_min  )) { readerror_tr=true; toMemo("tr.rpm_min not found in " + infileName); }
        if(find_float(in_tr, "tr.rpm_max="  , ini.tr.rpm_max  )) { readerror_tr=true; toMemo("tr.rpm_max not found in " + infileName); }
        if(find_float(in_tr, "tr.fi0_min="  , ini.tr.fi0_min  )) { readerror_tr=true; toMemo("tr.fi0_min not found in " + infileName); }
        if(find_float(in_tr, "tr.fi0_max="  , ini.tr.fi0_max  )) { readerror_tr=true; toMemo("tr.fi0_max not found in " + infileName); }
        if(find_float(in_tr, "tr.bet_min="  , ini.tr.bet_min  )) { readerror_tr=true; toMemo("tr.bet_min not found in " + infileName); }
        if(find_float(in_tr, "tr.bet_max="  , ini.tr.bet_max  )) { readerror_tr=true; toMemo("tr.bet_max not found in " + infileName); }
        if(find_float(in_tr, "tr.ksi_min="  , ini.tr.ksi_min  )) { readerror_tr=true; toMemo("tr.ksi_min not found in " + infileName); }
        if(find_float(in_tr, "tr.ksi_max="  , ini.tr.ksi_max  )) { readerror_tr=true; toMemo("tr.ksi_max not found in " + infileName); }
        if(find_float(in_tr, "tr.xo="       , ini.tr.xo       )) { readerror_tr=true; toMemo("tr.xo     not found in " + infileName); }
        if(find_float(in_tr, "tr.xt="       , ini.tr.xt       )) { readerror_tr=true; toMemo("tr.xt     not found in " + infileName); }
        if(find_float(in_tr, "tr.Iop="      , ini.tr.Iop      )) { readerror_tr=true; toMemo("tr.Iop    not found in " + infileName); }//Момент инерции лопасти отн. ГШ, кг*м^2
        if(find_float(in_tr, "tr.r_pov="    , ini.tr.r_pov    )) { readerror_tr=true; toMemo("tr.r_pov  not found in " + infileName); }
        if(find_float(in_tr, "tr.k_pov="    , ini.tr.k_pov    )) { readerror_tr=true; toMemo("tr.k_pov  not found in " + infileName); }
        if(find_float(in_tr, "tr.f_pod="    , ini.tr.f_pod    )) { readerror_tr=true; toMemo("tr.f_pod  not found in " + infileName); }
        if(find_float(in_tr, "tr.r_pod="    , ini.tr.r_pod    )) { readerror_tr=true; toMemo("tr.r_pod  not found in " + infileName); }
        if(find_float(in_tr, "tr.sigma3="   , ini.tr.sigma3   )) { readerror_tr=true; toMemo("tr.sigma3 not found in " + infileName); }

        //параметры лопасти
        if(find_vectorFloat(in_tr,"tr.r="   , ini.r_tr        )) { readerror_tr=true; toMemo("tr.r    not found in " + infileName); }//readFloatArr ("r" + _suff, &r_is_[0]);
        if(find_vectorFloat(in_tr,"tr.b="   , ini.bh_tr       )) { readerror_tr=true; toMemo("tr.b    not found in " + infileName); }
        if(find_vectorFloat(in_tr,"tr.dfi=" , ini.dfi_tr      )) { readerror_tr=true; toMemo("tr.dfi  not found in " + infileName); }
        if(find_vectorInt(  in_tr,"tr.prof=", ini.prof_tr     )) { readerror_tr=true; toMemo("tr.prof not found in " + infileName); }

        /*//характеристики сечений
        int prn = 1;
        QString profiledir = getFolderName(infilename);
        while (!find_str(in_tr, "profile" + ss.setNum(prn) + ":", fname_profile[prn] ))
        {
            fname_profile[prn] = profiledir + fname_profile[prn];
            prn++;
        };
        ini.ProfilesAmount = prn-1;
        if (ini.ProfilesAmount < 1)
        {   readerror_tr = true;  toMemo("не считаны характеристики сечений лопасти НВ");   }
        else
            readProfile();*/

        //демпфер лопасти
        ini.tr.damper = 0;//if( find_bool( in_tr, "damper=" , ini.tr.damper  )) { readerror_tr=true; toMemo("damper= not found"); }

        //построение 3D модели лопасти
        xyz frnt,neut,rear;//точки сечения
        trbldfrnt.clear();
        trbldneut.clear();
        trbldrear.clear();
        neut.x= 0;//ini.tr.rvs*0.001;
        neut.y= 0;
        neut.z= 0;
        trbldfrnt.push_back(neut);
        trbldneut.push_back(neut);
        trbldrear.push_back(neut);
        //считать вынос носа сечения отн.ном.линии лопасти
        ini.kh_tr = 0.33;
        for(int bs=0; bs<ini.r_tr.size(); bs++)
        {
            frnt.x = ini.r_tr[bs] - ini.tr.l_vsh*0.001;
            frnt.y = ini.kh_tr*ini.bh_tr[bs]*sin(ini.dfi_tr[bs]*M_PI/180);
            frnt.z = ini.kh_tr*ini.bh_tr[bs]*cos(ini.dfi_tr[bs]*M_PI/180);

            neut.x = ini.r_tr[bs] - ini.tr.l_vsh*0.001;
            neut.y = 0;
            neut.z = 0;

            rear.x = ini.r_tr[bs] - ini.tr.l_vsh*0.001;
            rear.y =-(1-ini.kh_tr)*ini.bh_tr[bs]*sin(ini.dfi_tr[bs]*M_PI/180);
            rear.z =-(1-ini.kh_tr)*ini.bh_tr[bs]*cos(ini.dfi_tr[bs]*M_PI/180);

            trbldfrnt.push_back(frnt);
            trbldneut.push_back(neut);
            trbldrear.push_back(rear);
        }

        ui->oglwidget->trbldfrnt = &trbldfrnt;
        ui->oglwidget->trbldneut = &trbldneut;
        ui->oglwidget->trbldrear = &trbldrear;

        ui->SpinBox_fi0tr->setRange( ini.tr.fi0_min, ini.tr.fi0_max );
        ui->Slider_fi0tr->setRange( ceil(ini.tr.fi0_min*1000), ceil(ini.tr.fi0_max*1000) );

        /*//old характеристики сечений
        int pr = 1;
        while (!find_str(in_tr, "profile" + ss.setNum(pr) + ":", fname_profile[pr] ))  pr++;
        ini.ProfilesAmount = pr-1;
        if (ini.ProfilesAmount < 1)
        {   readerror_tr = true;  toMemo("не считаны характеристики сечений лопасти НВ");   }
        else
            readProfile();
*/
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
bool MainWindow::read_pr(QString infileName) //считывание исходных данных винта
{//РУЛЕВОЙ ВИНТ
    bool readerror_pr = 0;
    QFile infile(infileName);
    if(infile.open( QIODevice::ReadOnly | QIODevice::Text )) //if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
    {
        in_pr = new QTextStream(&infile);

        if(find_int(  in_pr, "pr.hubtype="  , ini.pr.hubtype  )) { readerror_pr=true; toMemo("pr.hubtype not found in " + infileName); }
        if(find_int(  in_pr, "pr.rdirect="  , ini.pr.rdirect  )) { readerror_pr=true; toMemo("pr.rdirect not found in " + infileName); }
        if(find_int(  in_pr, "pr.z="        , ini.pr.zl       )) { readerror_pr=true; toMemo("pr.z   not found in " + infileName); }
        if(find_float(in_pr, "pr.mlp="      , ini.pr.mlp      )) { readerror_pr=true; toMemo("pr.mlp not found in " + infileName); }
        if(find_float(in_pr, "pr.Ib0="      , ini.pr.Ib0      )) { readerror_pr=true; toMemo("pr.Ib0 not found in " + infileName); }//Момент инерции лопасти отн. ГШ, кг*м^2
        if(find_float(in_pr, "pr.Sb0="      , ini.pr.Sb0      )) { readerror_pr=true; toMemo("pr.Sb0 not found in " + infileName); }//rd.Sgsh_mr  = 197;//Стат. момент инерции лопасти относительно г.ш, N*m
        if(find_float(in_pr, "pr.lgsh="     , ini.pr.l_gsh    )) { readerror_pr=true; toMemo("pr.lgsh not found in " + infileName); }//rd.lgsh     = 0.102;//Расстояние от оси несущего винта до оси ГШ, м
        ini.pr.l_vsh = ini.pr.l_gsh;
        if(find_float(in_pr, "pr.kvz="      , ini.pr.kvz      )) { readerror_pr=true; toMemo("pr.kvz  not found in " + infileName); }//rd.k_       = 0.4;//Компенсатор взмаха
        if(find_float(in_pr, "pr.dCxp="     , ini.pr.dCxp     )) { readerror_pr=true; toMemo("pr.dCxp not found in " + infileName); }//
        if(find_float(in_pr, "pr.Bkp="      , ini.pr.Bkp      )) { readerror_pr=true; toMemo("pr.Bkp  not found in " + infileName); }//rd.B_mr     = 0.97;//Коэф. концевых потерь
        if(find_float(in_pr, "pr.rpm_min="  , ini.pr.rpm_min  )) { readerror_pr=true; toMemo("pr.rpm_min not found in " + infileName); }
        if(find_float(in_pr, "pr.rpm_max="  , ini.pr.rpm_max  )) { readerror_pr=true; toMemo("pr.rpm_max not found in " + infileName); }
        if(find_float(in_pr, "pr.fi0_min="  , ini.pr.fi0_min  )) { readerror_pr=true; toMemo("pr.fi0_min not found in " + infileName); }
        if(find_float(in_pr, "pr.fi0_max="  , ini.pr.fi0_max  )) { readerror_pr=true; toMemo("pr.fi0_max not found in " + infileName); }
        if(find_float(in_pr, "pr.bet_min="  , ini.pr.bet_min  )) { readerror_pr=true; toMemo("pr.bet_min not found in " + infileName); }
        if(find_float(in_pr, "pr.bet_max="  , ini.pr.bet_max  )) { readerror_pr=true; toMemo("pr.bet_max not found in " + infileName); }
        if(find_float(in_pr, "pr.ksi_min="  , ini.pr.ksi_min  )) { readerror_pr=true; toMemo("pr.ksi_min not found in " + infileName); }
        if(find_float(in_pr, "pr.ksi_max="  , ini.pr.ksi_max  )) { readerror_pr=true; toMemo("pr.ksi_max not found in " + infileName); }

        if(find_float(in_pr, "pr.xo="       , ini.pr.xo       )) { readerror_pr=true; toMemo("pr.xo       not found in " + infileName); }
        if(find_float(in_pr, "pr.xt="       , ini.pr.xt       )) { readerror_pr=true; toMemo("pr.xt       not found in " + infileName); }
        //if(find_float(in_pr, "pr.xf="       , ini.pr.xf       )) { readerror_pr=true; toMemo("pr.xf       not found in " + infileName); }
        if(find_float(in_pr, "pr.Iop="      , ini.pr.Iop      )) { readerror_pr=true; toMemo("pr.Iop      not found in " + infileName); }//Момент инерции лопасти отн. ГШ, кг*м^2
        if(find_float(in_pr, "pr.r_pov="    , ini.pr.r_pov    )) { readerror_pr=true; toMemo("pr.r_pov not found in " + infileName); }
        if(find_float(in_pr, "pr.k_pov="    , ini.pr.k_pov    )) { readerror_pr=true; toMemo("pr.k_pov not found in " + infileName); }
        if(find_float(in_pr, "pr.f_pod="    , ini.pr.f_pod    )) { readerror_pr=true; toMemo("pr.f_pod    not found in " + infileName); }
        if(find_float(in_pr, "pr.r_pod="    , ini.pr.r_pod    )) { readerror_pr=true; toMemo("pr.r_pod    not found in " + infileName); }
        if(find_float(in_pr, "pr.sigma3="   , ini.pr.sigma3   )) { readerror_pr=true; toMemo("pr.sigma3 not found in " + infileName); }

        ///параметры лопасти
        if(find_vectorFloat(in_pr,"pr.r="   , ini.r_pr        )) { readerror_pr=true; toMemo("pr.r    not found in " + infileName); }//readFloatArr ("r" + _suff, &r_is_[0]);
        if(find_vectorFloat(in_pr,"pr.b="   , ini.bh_pr       )) { readerror_pr=true; toMemo("pr.b    not found in " + infileName); }
        if(find_vectorFloat(in_pr,"pr.dfi=" , ini.dfi_pr      )) { readerror_pr=true; toMemo("pr.dfi  not found in " + infileName); }
        if(find_vectorInt(  in_pr,"pr.prof=", ini.prof_pr     )) { readerror_pr=true; toMemo("pr.prof not found in " + infileName); }

        /*//характеристики сечений
        int prn = 1;
        QString profiledir = getFolderName(infilename);
        while (!find_str(in_pr, "profile" + ss.setNum(prn) + ":", fname_profile[prn] ))
        {
            fname_profile[prn] = profiledir + fname_profile[prn];
            prn++;
        };
        ini.ProfilesAmount = prn-1;
        if (ini.ProfilesAmount < 1)
        {   readerror_pr = true;  toMemo("не считаны характеристики сечений лопасти НВ");   }
        else
            readProfile();*/

        //демпфер лопасти
        ini.pr.damper = 0;

        ///построение 3D модели лопасти
        xyz frnt,neut,rear;//точки сечения
        prbldfrnt.clear();
        prbldneut.clear();
        prbldrear.clear();
        neut.x= 0;//ini.pr.rvs*0.001;
        neut.y= 0;
        neut.z= 0;
        prbldfrnt.push_back(neut);
        prbldneut.push_back(neut);
        prbldrear.push_back(neut);
        //считать вынос носа сечения отн.ном.линии лопасти
        ini.kh_pr = 0.33;
        for(int bs=0; bs<ini.r_pr.size(); bs++)
        {
            frnt.x = ini.r_pr[bs] - ini.pr.l_vsh*0.001;
            frnt.y = ini.kh_pr*ini.bh_pr[bs]*sin(ini.dfi_pr[bs]*M_PI/180);
            frnt.z = ini.kh_pr*ini.bh_pr[bs]*cos(ini.dfi_pr[bs]*M_PI/180);

            neut.x = ini.r_pr[bs] - ini.pr.l_vsh*0.001;
            neut.y = 0;
            neut.z = 0;

            rear.x = ini.r_pr[bs] - ini.pr.l_vsh*0.001;
            rear.y =-(1-ini.kh_pr)*ini.bh_pr[bs]*sin(ini.dfi_pr[bs]*M_PI/180);
            rear.z =-(1-ini.kh_pr)*ini.bh_pr[bs]*cos(ini.dfi_pr[bs]*M_PI/180);

            prbldfrnt.push_back(frnt);
            prbldneut.push_back(neut);
            prbldrear.push_back(rear);
        }

        ui->oglwidget->prbldfrnt = &prbldfrnt;
        ui->oglwidget->prbldneut = &prbldneut;
        ui->oglwidget->prbldrear = &prbldrear;

        ui->SpinBox_fi0tr->setRange( ini.pr.fi0_min, ini.pr.fi0_max );
        ui->Slider_fi0tr->setRange( ceil(ini.pr.fi0_min*1000), ceil(ini.pr.fi0_max*1000) );

        infile.close();
    }
    else
    {
        readerror_pr=true;
        toLogf("cannot open " + infileName);
    }
return readerror_pr;
}
bool MainWindow::readProfile()
{
        int j, ii=0; //номер профиля
        FILE *ish;
        string fileread;
        char s[200]="", ss[51];
        char *sc[] = {
               "M\\a"    //0  для Су
              ,"_M\\a"   //1  для Сх
              ,"__M\\a"  //2  для Mz
              };
        float ttt;

        for (ii = 0; ii < ini.ProfilesAmount; ii++)
        {
            fileread = fname_profile[ii+1].toStdString(); //*ExtractFilePath(Application->ExeName)*/ + f_name /*IntToStr(ii + 1)*/ + ".prof";
            ish = fopen(fileread.c_str(), "r");
            if (ish != NULL)
            {
                ini.nM[ii][0] = 0; //для Су
                ini.nM[ii][1] = 0; //для Сх
                ini.nM[ii][2] = 0; //для Mz
                while (fscanf(ish,"%s",s) != EOF)
                {
                    //Cy
                    if(strcmp(s,sc[0]) == 0) //если = "M\\a"
                    {
                        ini.nA[ii][0] = 0;
                        while (fscanf(ish,"%f",&ttt) == 1)
                        {
                            ini.ALPHA[ii][ini.nA[ii][0]][0] = ttt;
                            ini.nA[ii][0]++;
                        }
                    }
                    for (int i = 0; i <= CM; i++)
                    {
                        strcpy(ss, "M");//                        char cbuf;
                        string ind = to_string(i+1);
                        strcat(ss, ind.c_str());
                        if(strcmp(s, ss) == 0)
                        {
                            if (fscanf(ish,"%f",&ttt) == 1)
                                    ini.MACH[ii][i][0] = ttt;
                            j = 0;
                            while (fscanf(ish,"%f",&ttt) == 1)
                            {
                                ini._Cy[ii][ini.nM[ii][0]][j] = ttt;
                                j++;
                            }
                            ini.nM[ii][0]++;
                         }
                    }

                    //Cx
                    if(strcmp(s,sc[1]) == 0) //если = "_M\\a"
                    {
                        ini.nA[ii][1] = 0;
                        while (fscanf(ish,"%f",&ttt) == 1)
                        {
                            ini.ALPHA[ii][ini.nA[ii][1]][1] = ttt;
                            ini.nA[ii][1]++;
                        }
                    }
                    for (int i = 0; i <= CM; i++)
                    {
                        strcpy(ss, "_M");//"_M"                        char cbuf;
                        string ind = to_string(i+1);
                        strcat(ss, ind.c_str());
                        if(strcmp(s,ss) == 0)
                        {
                            if (fscanf(ish,"%f",&ttt) == 1)
                                    ini.MACH[ii][i][1] = ttt;
                            j = 0;
                            while (fscanf(ish,"%f",&ttt) == 1)
                            {
                                ini._Cx[ii][ini.nM[ii][1]][j] = ttt;
                                j++;
                            }
                            ini.nM[ii][1]++;
                        }
                    }

                    //Mz
                    if(strcmp(s,sc[2]) == 0) //если = "__M\\a"
                    {
                        ini.nA[ii][2] = 0;
                        while (fscanf(ish,"%f",&ttt) == 1)
                        {
                            ini.ALPHA[ii][ini.nA[ii][2]][2] = ttt;
                            ini.nA[ii][2]++;
                        }
                    }
                    for (int i = 0; i <= CM; i++)
                    {
                        strcpy(ss, "__M");//"__M"                        char cbuf;
                        string ind = to_string(i+1);
                        strcat(ss, ind.c_str());
                        if(strcmp(s,ss) == 0)
                        {
                            if (fscanf(ish,"%f",&ttt) == 1)
                                    ini.MACH[ii][i][2] = ttt;
                            j = 0;
                            while (fscanf(ish,"%f",&ttt) == 1)
                            {
                                ini._Mz[ii][ini.nM[ii][2]][j] = ttt;
                                j++;
                            }
                            ini.nM[ii][2]++;
                        }
                    }

                }
            }
            else QMessageBox::about(0,"info","fileread: " + fname_profile[ii+1]);
        }
        return true;
}
bool MainWindow::read_load(QString infileName, QVector<extloadstruct> &load, xyz &loadpos)//считывание данных по внешней нагрузке
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
bool MainWindow::read_cntr(QString infileName)//считывание точек 3D модели планера
{//
    bool readerror=0;
    xyz node;
    double ku=1;
    QString line, unit;
    QStringList fields;
    QRegExp rx("\\s");
    QFile infilecntr(infileName);
    infilecntr.open( QIODevice::ReadOnly | QIODevice::Text );
    in_cntr = new QTextStream(&infilecntr);
    frmcntr.clear();
    //считывание данных из файла
    find_str( in_cntr, "unit=", unit );
    if(unit=="MM" || unit=="mm") ku=0.001; else ku=1;
    while(!in_cntr->atEnd())
    {
        line = in_cntr->readLine();
        fields = line.split(rx, QString::SkipEmptyParts);
        if((fields.size()>0)&&(fields.at(0)=="vertex"))
        {
            node.x= fields.at(1).toFloat()*ku *frmpos.xdir +frmpos.xbias*0.001;
            node.y= fields.at(2).toFloat()*ku *frmpos.ydir +frmpos.ybias*0.001;
            node.z= fields.at(3).toFloat()*ku *frmpos.zdir +frmpos.zbias*0.001;
            frmcntr.push_back(node);
        }
    }
    infilecntr.close();
    if(frmcntr.size()>1) ui->oglwidget->frmcntr = &frmcntr;
return readerror;
}

// файлы состояния вертолета
bool MainWindow::read_helicopter_state(QString filename)
{
    bool readerror=0;
    int  version = 0;
    QString line, eofidentity;
    QStringList fields;
    QRegExp rx("\\s");//пробелы и табуляции
    QFile infile(filename);
    if( infile.open( QIODevice::ReadOnly | QIODevice::Text ) )
    {
        in = new QTextStream(&infile);
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);
        if(fields.size()>1)     version= fields.at(1).toInt();
        if(version>20000000)    version*=0.01;
        line = in->readLine();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.iter= fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.t=    fields.at(1).toFloat();
        line = in->readLine();
        ///airframe
        {
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.org.x=    fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.org.y=    fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.org.z=    fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.org.k=    fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.org.r=    fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.org.t=    fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.ocw.x=    fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.ocw.y=    fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.ocw.z=    fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.vn.x=     fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.vn.y=     fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.vn.z=     fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.wn.x=     fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.wn.y=     fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.wn.z=     fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.an.x=     fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.an.y=     fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.an.z=     fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.en.x=     fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.en.y=     fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.en.z=     fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.v.x=      fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.v.y=      fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.v.z=      fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.w.x=      fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.w.y=      fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.w.z=      fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.a.x=      fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.a.y=      fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.a.z=      fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.e.x=      fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.e.y=      fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.e.z=      fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.n.x=      fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.n.y=      fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.n.z=      fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Fsum.x=   fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Fsum.y=   fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Fsum.z=   fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Msum.x=   fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Msum.y=   fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Msum.z=   fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Fhg.x=    fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Fhg.y=    fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Fhg.z=    fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Mhg.x=    fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Mhg.y=    fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Mhg.z=    fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Fadp.x=   fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Fadp.y=   fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Fadp.z=   fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Madp.x=   fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Madp.y=   fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Madp.z=   fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.fus_F.x = fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.fus_F.y = fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.fus_F.z = fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.fus_M.x = fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.fus_M.y = fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.fus_M.z = fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.stab_F.x= fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.stab_F.y= fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.stab_F.z= fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.keel_F.x= fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.keel_F.y= fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.keel_F.z= fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.wing_F.x= fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.wing_F.y= fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.wing_F.z= fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Fmr.x=    fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Fmr.y=    fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Fmr.z=    fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Mmr.x=    fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Mmr.y=    fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Mmr.z=    fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Ftr.x=    fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Ftr.y=    fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Ftr.z=    fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Mtr.x=    fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Mtr.y=    fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Mtr.z=    fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Fpr.x=    fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Fpr.y=    fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Fpr.z=    fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Mpr.x=    fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Mpr.y=    fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Mpr.z=    fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Fsh.x=    fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Fsh.y=    fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Fsh.z=    fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.Msh.x=    fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Msh.y=    fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.Msh.z=    fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.xct=      fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.yct=      fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.zct=      fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.cargo.Ox =    fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.cargo.Oy =    fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.cargo.Oz =    fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.cargo.CTx=    fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.cargo.CTy=    fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.cargo.CTz=    fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.cargo.Gx =    fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.cargo.Gy =    fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.cargo.Gz =    fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.F1.x=     fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.F1.y=     fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.F1.z=     fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.fus.F1pos.x=  fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.F1pos.y=  fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.fus.F1pos.z=  fields.at(3).toFloat();
            line = in->readLine();
        }
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.nst=           fields.at(1).toInt();//
        for (int k = 0; k < out.nst; k++)
        {
            line = in->readLine();
            //if(version>202010)   {
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].type    = fields.at(1).toInt();
              //  }//if(version>20201000)
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].nwh         = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].status      = fields.at(1).toInt();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.status    = fields.at(1).toInt();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].p.status    = fields.at(1).toInt();
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
            if(version>=202208)
            {
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].psih    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].psif    = fields.at(1).toFloat();
                //line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].pk_a    = fields.at(1).toFloat();
                //line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].pkfa    = fields.at(1).toFloat();
                //line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].pk_p    = fields.at(1).toFloat();
                //line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].pkfp    = fields.at(1).toFloat();
            }
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].AB          = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.S         = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.ha        = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.ha1       = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.ha2       = fields.at(1).toFloat();
            if(version>=202208)
            {
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.v1    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.v2    = fields.at(1).toFloat();
            }
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
            //if(version>202010) {
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].a.dh12 = fields.at(1).toFloat();
              //  }//20201019
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
            if(version>=202208)
            {
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].Q.x         = fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].Q.y         = fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].Q.z         = fields.at(3).toFloat();
            }
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].A.x         = fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].A.y         = fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].A.z         = fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].B.x         = fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].B.y         = fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].B.z         = fields.at(3).toFloat();
            if(version>=202208)
            {
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.st[k].V.x         = fields.at(1).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].V.y         = fields.at(2).toFloat();
            /*line = in->readLine();fields = line.split(rx, QString::SkipEmptyParts);*/  out.st[k].V.z         = fields.at(3).toFloat();
            }
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
                //out.st[k].ssC1.x  = 0;
                //out.st[k].ssC1.y  = 0;
                //out.st[k].ssC1.z  = 0;
                //out.st[k].ssC2.x  = 0;
                //out.st[k].ssC2.y  = 0;
                //out.st[k].ssC2.z  = 0;
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
        line = in->readLine();///mainrotor
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.mr.O.x       = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.mr.O.y       = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.mr.O.z       = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.mr.Fhx_sr    = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.mr.Fhy_sr    = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.mr.Fhz_sr    = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.mr.Mhx_sr    = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.mr.Mhy_sr    = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.mr.Mhz_sr    = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.mr.D         = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.mr.eps_fz    = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.mr.eps_fx    = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.mr.chi_hub   = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.mr.eta_hub   = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.mr.solve     = fields.at(1).toFloat();
        if(out.mr.solve)
        {
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.eh.x    = fields.at(1).toFloat();
            /*line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);*/ out.mr.eh.y    = fields.at(2).toFloat();
            /*line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);*/ out.mr.eh.z    = fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.wh.x    = fields.at(1).toFloat();
            /*line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);*/ out.mr.wh.y    = fields.at(2).toFloat();
            /*line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);*/ out.mr.wh.z    = fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.ah.x    = fields.at(1).toFloat();
            /*line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);*/ out.mr.ah.y    = fields.at(2).toFloat();
            /*line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);*/ out.mr.ah.z    = fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.vh.x    = fields.at(1).toFloat();
            /*line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);*/ out.mr.vh.y    = fields.at(2).toFloat();
            /*line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);*/ out.mr.vh.z    = fields.at(3).toFloat();
            // line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.sh.x    = fields.at(1).toFloat();
            // /*line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);*/ out.mr.sh.y    = fields.at(2).toFloat();
            // /*line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);*/ out.mr.sh.z    = fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.dct.x   = fields.at(1).toFloat();
            /*line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);*/ out.mr.dct.y   = fields.at(2).toFloat();
            /*line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);*/ out.mr.dct.z   = fields.at(3).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.angbs   = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.dctbs   = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.nb      = fields.at(1).toInt();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.nSec    = fields.at(1).toInt();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.sec07   = fields.at(1).toInt();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.revcount= fields.at(1).toInt();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.psih    = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.fi0     = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.chi     = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.eta     = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.alfa    = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.beta    = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.Fhx_sum = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.Fhy_sum = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.Fhz_sum = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.Mhx_sum = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.Mhy_sum = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.Mhz_sum = fields.at(1).toFloat();

            for (int l = 0; l < out.mr.nb; l++)
            {
                line = in->readLine();
                line = in->readLine();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].psi0   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].psi    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].fi7    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].dfi7   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].d2fi7  = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].gama   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].dgama  = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].d2gama = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].beta   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].dbeta  = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].d2beta = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].ksi    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].dksi   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].d2ksi  = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].T      = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].Q      = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].Ma     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].H      = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].S      = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].Nr     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].Nf     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].K1     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].P1     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].P2     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].Fdmp   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].Fpov   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].Paxz   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].Payy   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].Fdmp   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].Fy     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].Fx     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].Fz     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M1T    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M1Q    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M1Gx   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M1Gy   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M1K    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M1Cx   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M1Cy   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M1Px   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M1Py   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M1x    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M1y    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M1z    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M2T    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M2Q    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M2Gx   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M2Gy   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M2K    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M2Cx   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M2Cy   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M2Px   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M2Py   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M2D    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M2x    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M2y    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M2z    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M3Ma   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M3Cz   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M3su   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M3tr   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M3iz   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M3x    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M3y    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].M3z    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].Fhx    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].Fhy    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].Fhz    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].Mhx    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].Mhy    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.mr.b[l].Mhz    = fields.at(1).toFloat();

                out.mr.b[l]._r   .clear();
                out.mr.b[l].r    .clear();
                out.mr.b[l].bh   .clear();
                out.mr.b[l].dfi  .clear();
                out.mr.b[l].prof .clear();
                out.mr.b[l].alfar.clear();
                out.mr.b[l].U    .clear();
                out.mr.b[l].Ux   .clear();
                out.mr.b[l].Uy   .clear();
                out.mr.b[l].Uywx .clear();
                out.mr.b[l].Uywz .clear();
                out.mr.b[l].fi   .clear();
                out.mr.b[l].fl   .clear();
                out.mr.b[l].Fi   .clear();
                out.mr.b[l].lambda.clear();
                out.mr.b[l].M    .clear();
                out.mr.b[l].cy   .clear();
                out.mr.b[l].cx   .clear();
                out.mr.b[l].mz   .clear();
                out.mr.b[l].dT_dr.clear();
                out.mr.b[l].dQ_dr.clear();
                out.mr.b[l].dMa_dr.clear();
                line = in->readLine();
                for (int sec = 0; sec < out.mr.nSec; sec++)
                {
                    line = in->readLine();
                    fields = line.split(rx, QString::SkipEmptyParts);
                    out.mr.b[l]._r    .push_back( fields.at( 0).toFloat() );
                    out.mr.b[l].r     .push_back( fields.at( 1).toFloat() );
                    out.mr.b[l].alfar .push_back( fields.at( 2).toFloat() );
                    out.mr.b[l].fi    .push_back( fields.at( 3).toFloat() );
                    out.mr.b[l].fl    .push_back( fields.at( 4).toFloat() );
                    out.mr.b[l].U     .push_back( fields.at( 5).toFloat() );
                    out.mr.b[l].Ux    .push_back( fields.at( 6).toFloat() );
                    out.mr.b[l].Uy    .push_back( fields.at( 7).toFloat() );
                    out.mr.b[l].Fi    .push_back( fields.at( 8).toFloat() );
                    out.mr.b[l].lambda.push_back( fields.at( 9).toFloat() );
                    out.mr.b[l].M     .push_back( fields.at(10).toFloat() );
                    out.mr.b[l].cy    .push_back( fields.at(11).toFloat() );
                    out.mr.b[l].cx    .push_back( fields.at(12).toFloat() );
                    out.mr.b[l].mz    .push_back( fields.at(13).toFloat() );
                    out.mr.b[l].dT_dr .push_back( fields.at(14).toFloat() );
                    out.mr.b[l].dQ_dr .push_back( fields.at(15).toFloat() );
                    out.mr.b[l].dMa_dr.push_back( fields.at(16).toFloat() );
                    //out.mr.b[l].Uywx  .push_back( fields.at().toFloat() );
                    //out.mr.b[l].Uywz  .push_back( fields.at().toFloat() );
                }
            }
            /** vgsolve
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.vgsolve   = fields.at(1).toFloat();
            if(out.mr.vgsolve)
            {
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.vg.psi    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.vg.eps    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.vg.Mn     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.vg.Mt     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.vg.Mk     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);  out.mr.vg.Mi     = fields.at(1).toFloat();
            }*/
        }
        line = in->readLine();//---------------
        line = in->readLine();///tailrotor
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.tr.O.x       = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.tr.O.y       = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.tr.O.z       = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.tr.Fhx_sr    = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.tr.Fhy_sr    = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.tr.Fhz_sr    = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.tr.Mhx_sr    = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.tr.Mhy_sr    = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.tr.Mhz_sr    = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.tr.D         = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.tr.solve     = fields.at(1).toFloat();
        if (out.tr.solve)
        {
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.nb      = fields.at(1).toInt();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.nSec    = fields.at(1).toInt();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.sec07   = fields.at(1).toInt();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.revcount= fields.at(1).toInt();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.psih    = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.fi0     = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.alfa    = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.beta    = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.Fhx_sum = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.Fhy_sum = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.Fhz_sum = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.Mhx_sum = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.Mhy_sum = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.Mhz_sum = fields.at(1).toFloat();

            for (int l = 0; l < out.tr.nb; l++)
            {
                line = in->readLine();
                line = in->readLine();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].psi0   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].psi    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].fi7    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].dfi7   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].d2fi7  = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].gama   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].dgama  = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].d2gama = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].beta   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].dbeta  = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].d2beta = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].ksi    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].dksi   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].d2ksi  = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].T      = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].Q      = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].Ma     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].H      = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].S      = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].Nr     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].Nf     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].K1     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].P1     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].P2     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].Fdmp   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].Fpov   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].Fy     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].Fx     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].Fz     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M1T    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M1Q    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M1Gx   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M1Gy   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M1K    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M1Cx   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M1Cy   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M1Px   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M1Py   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M1x    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M1y    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M1z    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M2T    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M2Q    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M2Gx   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M2Gy   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M2K    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M2Cx   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M2Cy   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M2Px   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M2Py   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M2D    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M2x    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M2y    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M2z    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M3Ma   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M3Cz   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M3su   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M3tr   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M3iz   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M3x    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M3y    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].M3z    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].Fhx    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].Fhy    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].Fhz    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].Mhx    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].Mhy    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.tr.b[l].Mhz    = fields.at(1).toFloat();

                out.tr.b[l]._r   .clear();
                out.tr.b[l].r    .clear();
                out.tr.b[l].bh   .clear();
                out.tr.b[l].dfi  .clear();
                out.tr.b[l].prof .clear();
                out.tr.b[l].alfar.clear();
                out.tr.b[l].U    .clear();
                out.tr.b[l].Ux   .clear();
                out.tr.b[l].Uy   .clear();
                out.tr.b[l].Uywx .clear();
                out.tr.b[l].Uywz .clear();
                out.tr.b[l].fi   .clear();
                out.tr.b[l].fl   .clear();
                out.tr.b[l].Fi   .clear();
                out.tr.b[l].lambda.clear();
                out.tr.b[l].M    .clear();
                out.tr.b[l].cy   .clear();
                out.tr.b[l].cx   .clear();
                out.tr.b[l].mz   .clear();
                out.tr.b[l].dT_dr.clear();
                out.tr.b[l].dQ_dr.clear();
                out.tr.b[l].dMa_dr.clear();
                line = in->readLine();
                for (int sec = 0; sec < out.tr.nSec; sec++)
                {
                    line = in->readLine();
                    fields = line.split(rx, QString::SkipEmptyParts);
                    out.tr.b[l]._r    .push_back( fields.at( 0).toFloat() );
                    out.tr.b[l].r     .push_back( fields.at( 1).toFloat() );
                    out.tr.b[l].alfar .push_back( fields.at( 2).toFloat() );
                    out.tr.b[l].fi    .push_back( fields.at( 3).toFloat() );
                    out.tr.b[l].fl    .push_back( fields.at( 4).toFloat() );
                    out.tr.b[l].U     .push_back( fields.at( 5).toFloat() );
                    out.tr.b[l].Ux    .push_back( fields.at( 6).toFloat() );
                    out.tr.b[l].Uy    .push_back( fields.at( 7).toFloat() );
                    out.tr.b[l].Fi    .push_back( fields.at( 8).toFloat() );
                    out.tr.b[l].lambda.push_back( fields.at( 9).toFloat() );
                    out.tr.b[l].M     .push_back( fields.at(10).toFloat() );
                    out.tr.b[l].cy    .push_back( fields.at(11).toFloat() );
                    out.tr.b[l].cx    .push_back( fields.at(12).toFloat() );
                    out.tr.b[l].mz    .push_back( fields.at(13).toFloat() );
                    out.tr.b[l].dT_dr .push_back( fields.at(14).toFloat() );
                    out.tr.b[l].dQ_dr .push_back( fields.at(15).toFloat() );
                    out.tr.b[l].dMa_dr.push_back( fields.at(16).toFloat() );
                }
            }
        }
        line = in->readLine();//---------------
        line = in->readLine();///pushrotor
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.pr.O.x       = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.pr.O.y       = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.pr.O.z       = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.pr.Fhx_sr    = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.pr.Fhy_sr    = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.pr.Fhz_sr    = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.pr.Mhx_sr    = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.pr.Mhy_sr    = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.pr.Mhz_sr    = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.pr.D         = fields.at(1).toFloat();
        line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);    out.pr.solve     = fields.at(1).toFloat();
        if (out.pr.solve)
        {
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.nb      = fields.at(1).toInt();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.nSec    = fields.at(1).toInt();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.sec07   = fields.at(1).toInt();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.revcount= fields.at(1).toInt();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.psih    = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.fi0     = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.alfa    = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.beta    = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.Fhx_sum = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.Fhy_sum = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.Fhz_sum = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.Mhx_sum = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.Mhy_sum = fields.at(1).toFloat();
            line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.Mhz_sum = fields.at(1).toFloat();

            for (int l = 0; l < out.pr.nb; l++)
            {
                line = in->readLine();
                line = in->readLine();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].psi0   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].psi    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].fi7    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].dfi7   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].d2fi7  = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].gama   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].dgama  = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].d2gama = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].beta   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].dbeta  = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].d2beta = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].ksi    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].dksi   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].d2ksi  = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].T      = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].Q      = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].Ma     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].H      = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].S      = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].Nr     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].Nf     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].K1     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].P1     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].P2     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].Fdmp   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].Fpov   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].Fy     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].Fx     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].Fz     = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M1T    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M1Q    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M1Gx   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M1Gy   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M1K    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M1Cx   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M1Cy   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M1Px   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M1Py   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M1x    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M1y    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M1z    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M2T    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M2Q    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M2Gx   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M2Gy   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M2K    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M2Cx   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M2Cy   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M2Px   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M2Py   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M2D    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M2x    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M2y    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M2z    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M3Ma   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M3Cz   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M3su   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M3tr   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M3iz   = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M3x    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M3y    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].M3z    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].Fhx    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].Fhy    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].Fhz    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].Mhx    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].Mhy    = fields.at(1).toFloat();
                line = in->readLine();  fields = line.split(rx, QString::SkipEmptyParts);     out.pr.b[l].Mhz    = fields.at(1).toFloat();

                out.pr.b[l]._r   .clear();
                out.pr.b[l].r    .clear();
                out.pr.b[l].bh   .clear();
                out.pr.b[l].dfi  .clear();
                out.pr.b[l].prof .clear();
                out.pr.b[l].alfar.clear();
                out.pr.b[l].U    .clear();
                out.pr.b[l].Ux   .clear();
                out.pr.b[l].Uy   .clear();
                out.pr.b[l].Uywx .clear();
                out.pr.b[l].Uywz .clear();
                out.pr.b[l].fi   .clear();
                out.pr.b[l].fl   .clear();
                out.pr.b[l].Fi   .clear();
                out.pr.b[l].lambda.clear();
                out.pr.b[l].M    .clear();
                out.pr.b[l].cy   .clear();
                out.pr.b[l].cx   .clear();
                out.pr.b[l].mz   .clear();
                out.pr.b[l].dT_dr.clear();
                out.pr.b[l].dQ_dr.clear();
                out.pr.b[l].dMa_dr.clear();
                line = in->readLine();
                for (int sec = 0; sec < out.pr.nSec; sec++)
                {
                    line = in->readLine();
                    fields = line.split(rx, QString::SkipEmptyParts);
                    out.pr.b[l]._r    .push_back( fields.at( 0).toFloat() );
                    out.pr.b[l].r     .push_back( fields.at( 1).toFloat() );
                    out.pr.b[l].alfar .push_back( fields.at( 2).toFloat() );
                    out.pr.b[l].fi    .push_back( fields.at( 3).toFloat() );
                    out.pr.b[l].fl    .push_back( fields.at( 4).toFloat() );
                    out.pr.b[l].U     .push_back( fields.at( 5).toFloat() );
                    out.pr.b[l].Ux    .push_back( fields.at( 6).toFloat() );
                    out.pr.b[l].Uy    .push_back( fields.at( 7).toFloat() );
                    out.pr.b[l].Fi    .push_back( fields.at( 8).toFloat() );
                    out.pr.b[l].lambda.push_back( fields.at( 9).toFloat() );
                    out.pr.b[l].M     .push_back( fields.at(10).toFloat() );
                    out.pr.b[l].cy    .push_back( fields.at(11).toFloat() );
                    out.pr.b[l].cx    .push_back( fields.at(12).toFloat() );
                    out.pr.b[l].mz    .push_back( fields.at(13).toFloat() );
                    out.pr.b[l].dT_dr .push_back( fields.at(14).toFloat() );
                    out.pr.b[l].dQ_dr .push_back( fields.at(15).toFloat() );
                    out.pr.b[l].dMa_dr.push_back( fields.at(16).toFloat() );
                }
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
bool MainWindow::write_helicopter_state(QString filename)
{//var:
    QString s,ss;
    QString outstr = titleapplname;
    outstr += "\n----------------------------------------------------------------------";
    outstr += "\ni= " + ss.setNum(out.iter);
    outstr += "\nt= " + ss.setNum(out.t,'f',6);
    outstr += "\n----------------------------------------------------------------------";
    //airframe
    {
    outstr += "\norg(xyz)=        \t" + ss.setNum(out.fus.org.x   ,'f',5);
    outstr +=                 "   \t" + ss.setNum(out.fus.org.y   ,'f',5);
    outstr +=                 "   \t" + ss.setNum(out.fus.org.z   ,'f',5);
    outstr += "\norg(krt)=        \t" + ss.setNum(out.fus.org.k   ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.org.r   ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.org.t   ,'f',6);
    outstr += "\nocw(xyz)=        \t" + ss.setNum(out.fus.ocw.x   ,'f',5);
    outstr +=                 "   \t" + ss.setNum(out.fus.ocw.y   ,'f',5);
    outstr +=                 "   \t" + ss.setNum(out.fus.ocw.z   ,'f',5);
    outstr += "\nvn(xyz)=         \t" + ss.setNum(out.fus.vn.x    ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.vn.y    ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.vn.z    ,'f',6);
    outstr += "\nwn(krt)=         \t" + ss.setNum(out.fus.wn.x    ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.wn.y    ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.wn.z    ,'f',6);
    outstr += "\nan(xyz)=         \t" + ss.setNum(out.fus.an.x    ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.an.y    ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.an.z    ,'f',6);
    outstr += "\nen(krt)=         \t" + ss.setNum(out.fus.en.x    ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.en.y    ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.en.z    ,'f',6);
    outstr += "\nv(xyz)=          \t" + ss.setNum(out.fus.v.x     ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.v.y     ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.v.z     ,'f',6);
    outstr += "\nw(krt)=          \t" + ss.setNum(out.fus.w.x     ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.w.y     ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.w.z     ,'f',6);
    outstr += "\na(xyz)=          \t" + ss.setNum(out.fus.a.x     ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.a.y     ,'f',6);
    outstr +=                 "   \t" + ss.setNum(out.fus.a.z     ,'f',6);
    outstr += "\ne(krt)=          \t" + ss.setNum(out.fus.e.x     ,'f',6);
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
    outstr += "\nFadp(xyz)=       \t" + ss.setNum(out.fus.Fadp.x  ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Fadp.y  ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Fadp.z  ,'f',3);
    outstr += "\nMadp(xyz)=       \t" + ss.setNum(out.fus.Madp.x  ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Madp.y  ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Madp.z  ,'f',3);
    outstr += "\nFfus(xyz)=       \t" + ss.setNum(out.fus.fus_F.x ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.fus_F.y ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.fus_F.z ,'f',3);
    outstr += "\nMfus(xyz)=       \t" + ss.setNum(out.fus.fus_M.x ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.fus_M.y ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.fus_M.z ,'f',3);
    outstr += "\nFstab(xyz)=      \t" + ss.setNum(out.fus.stab_F.x,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.stab_F.y,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.stab_F.z,'f',3);
    outstr += "\nFkeel(xyz)=      \t" + ss.setNum(out.fus.keel_F.x,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.keel_F.y,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.keel_F.z,'f',3);
    outstr += "\nFwing(xyz)=      \t" + ss.setNum(out.fus.wing_F.x,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.wing_F.y,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.wing_F.z,'f',3);
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
    outstr += "\nFpr(xyz)=        \t" + ss.setNum(out.fus.Fpr.x   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Fpr.y   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Fpr.z   ,'f',3);
    outstr += "\nMpr(xyz)=        \t" + ss.setNum(out.fus.Mpr.x   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Mpr.y   ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.Mpr.z   ,'f',3);
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
    outstr += "\nextload.F=       \t" + ss.setNum(out.fus.F1.x    ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.F1.y    ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.F1.z    ,'f',3);
    outstr += "\nextload.pos=     \t" + ss.setNum(out.fus.F1pos.x ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.F1pos.y ,'f',3);
    outstr +=                 "   \t" + ss.setNum(out.fus.F1pos.z ,'f',3);
    outstr += "\n----------------------------------------------------------------------";
    }
    outstr += "\nshassi.nst=      \t" + ss.setNum(out.nst);
    for (int k = 0; k < out.nst; k++)
    {
        outstr += "\nst#              \t" + ss.setNum(k);
        outstr += "\ns"+s.setNum(k)+".type=         \t" + ss.setNum(out.st[k].type    );
        outstr += "\ns"+s.setNum(k)+".nwh=          \t" + ss.setNum(out.st[k].nwh     );
        outstr += "\ns"+s.setNum(k)+".status=       \t" + ss.setNum(out.st[k].status  );
        outstr += "\ns"+s.setNum(k)+".a.status=     \t" + ss.setNum(out.st[k].a.status);
        outstr += "\ns"+s.setNum(k)+".p.status=     \t" + ss.setNum(out.st[k].p.status);
        outstr += "\ns"+s.setNum(k)+".Bwh=          \t" + ss.setNum(out.st[k].Bwh      ,'f',1);
        outstr += "\ns"+s.setNum(k)+".Dwh=          \t" + ss.setNum(out.st[k].Dwh      ,'f',1);
        outstr += "\ns"+s.setNum(k)+".rwh=          \t" + ss.setNum(out.st[k].rwh      ,'f',1);
        outstr += "\ns"+s.setNum(k)+".Aap=          \t" + ss.setNum(out.st[k].Aap      ,'f',1);
        outstr += "\ns"+s.setNum(k)+".Aa=           \t" + ss.setNum(out.st[k].Aa       ,'f',1);
        outstr += "\ns"+s.setNum(k)+".Ap=           \t" + ss.setNum(out.st[k].Ap       ,'f',1);
        outstr += "\ns"+s.setNum(k)+".Pay=          \t" + ss.setNum(out.st[k].Pay      ,'f',1);
        outstr += "\ns"+s.setNum(k)+".Ppy=          \t" + ss.setNum(out.st[k].Ppy      ,'f',1);
        outstr += "\ns"+s.setNum(k)+".ang_sx=       \t" + ss.setNum(out.st[k].ang_sx   ,'f',4);
        outstr += "\ns"+s.setNum(k)+".ang_sz=       \t" + ss.setNum(out.st[k].ang_sz   ,'f',4);
        outstr += "\ns"+s.setNum(k)+".ang_ax=       \t" + ss.setNum(out.st[k].ang_ax   ,'f',4);
        outstr += "\ns"+s.setNum(k)+".ang_az=       \t" + ss.setNum(out.st[k].ang_az   ,'f',4);
        outstr += "\ns"+s.setNum(k)+".ang_pn=       \t" + ss.setNum(out.st[k].ang_pn   ,'f',4);
        outstr += "\ns"+s.setNum(k)+".psih=         \t" + ss.setNum(out.st[k].psih     ,'f',3);
        outstr += "\ns"+s.setNum(k)+".psif=         \t" + ss.setNum(out.st[k].psif     ,'f',3);
        //outstr += "\ns"+s.setNum(k)+".pk_a=         \t" + ss.setNum(out.st[k].pk_a     ,'f',3);
        //outstr += "\ns"+s.setNum(k)+".pkfa=         \t" + ss.setNum(out.st[k].pkfa     ,'f',3);
        //outstr += "\ns"+s.setNum(k)+".pk_p=         \t" + ss.setNum(out.st[k].pk_p     ,'f',3);
        //outstr += "\ns"+s.setNum(k)+".pkfp=         \t" + ss.setNum(out.st[k].pkfp     ,'f',3);
        outstr += "\ns"+s.setNum(k)+".AB=           \t" + ss.setNum(out.st[k].AB       ,'f',3);
        outstr += "\ns"+s.setNum(k)+".a.S=          \t" + ss.setNum(out.st[k].a.S      ,'f',3);
        outstr += "\ns"+s.setNum(k)+".a.ha=         \t" + ss.setNum(out.st[k].a.ha     ,'f',3);
        outstr += "\ns"+s.setNum(k)+".a.hac1=       \t" + ss.setNum(out.st[k].a.ha1    ,'f',3);
        outstr += "\ns"+s.setNum(k)+".a.hac2=       \t" + ss.setNum(out.st[k].a.ha2    ,'f',3);
        outstr += "\ns"+s.setNum(k)+".a.v1=         \t" + ss.setNum(out.st[k].a.v1     ,'f',4);
        outstr += "\ns"+s.setNum(k)+".a.v2=         \t" + ss.setNum(out.st[k].a.v2     ,'f',4);
        outstr += "\ns"+s.setNum(k)+".a.Pa=         \t" + ss.setNum(out.st[k].a.Pa     ,'f',1);
        outstr += "\ns"+s.setNum(k)+".a.P1=         \t" + ss.setNum(out.st[k].a.Pa1    ,'f',1);
        outstr += "\ns"+s.setNum(k)+".a.P2=         \t" + ss.setNum(out.st[k].a.Pa2    ,'f',1);
        outstr += "\ns"+s.setNum(k)+".a.Pg1=        \t" + ss.setNum(out.st[k].a.Pg1    ,'f',1);
        outstr += "\ns"+s.setNum(k)+".a.Pg2=        \t" + ss.setNum(out.st[k].a.Pg2    ,'f',1);
        outstr += "\ns"+s.setNum(k)+".a.Ph1=        \t" + ss.setNum(out.st[k].a.Ph1    ,'f',1);
        outstr += "\ns"+s.setNum(k)+".a.Ph2=        \t" + ss.setNum(out.st[k].a.Ph2    ,'f',1);
        outstr += "\ns"+s.setNum(k)+".a.Pf1=        \t" + ss.setNum(out.st[k].a.Pf1    ,'f',1);
        outstr += "\ns"+s.setNum(k)+".a.Pf2=        \t" + ss.setNum(out.st[k].a.Pf2    ,'f',1);
        outstr += "\ns"+s.setNum(k)+".a.V1=         \t" + ss.setNum(out.st[k].a.V1     ,'f',6);
        outstr += "\ns"+s.setNum(k)+".a.V2=         \t" + ss.setNum(out.st[k].a.V2     ,'f',6);
        outstr += "\ns"+s.setNum(k)+".a.p1=         \t" + ss.setNum(out.st[k].a.p1     ,'f',3);
        outstr += "\ns"+s.setNum(k)+".a.p2=         \t" + ss.setNum(out.st[k].a.p2     ,'f',3);
        outstr += "\ns"+s.setNum(k)+".a.dh12=       \t" + ss.setNum(out.st[k].a.dh12   ,'f',3);
        outstr += "\ns"+s.setNum(k)+".p.S=          \t" + ss.setNum(out.st[k].p.S      ,'f',3);
        outstr += "\ns"+s.setNum(k)+".p.h(xyz)=     \t" + ss.setNum(out.st[k].p.h.x    ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].p.h.y    ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].p.h.z    ,'f',3);
        outstr += "\ns"+s.setNum(k)+".p.P(xyz)=     \t" + ss.setNum(out.st[k].p.P.x    ,'f',1);
        outstr +=                               "   \t" + ss.setNum(out.st[k].p.P.y    ,'f',1);
        outstr +=                               "   \t" + ss.setNum(out.st[k].p.P.z    ,'f',1);
        outstr += "\ns"+s.setNum(k)+".O(xyz)=       \t" + ss.setNum(out.st[k].O.x      ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].O.y      ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].O.z      ,'f',3);
        outstr += "\ns"+s.setNum(k)+".Q(xyz)=       \t" + ss.setNum(out.st[k].Q.x      ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].Q.y      ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].Q.z      ,'f',3);
        outstr += "\ns"+s.setNum(k)+".A(xyz)=       \t" + ss.setNum(out.st[k].A.x      ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].A.y      ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].A.z      ,'f',3);
        outstr += "\ns"+s.setNum(k)+".B(xyz)=       \t" + ss.setNum(out.st[k].B.x      ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].B.y      ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].B.z      ,'f',3);
        outstr += "\ns"+s.setNum(k)+".V(xyz)=       \t" + ss.setNum(out.st[k].V.x      ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].V.y      ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].V.z      ,'f',3);
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
        outstr += "\ns"+s.setNum(k)+".saC(xyz)=     \t" + ss.setNum(out.st[k].saC.x    ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].saC.y    ,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].saC.z    ,'f',3);
        outstr += "\ns"+s.setNum(k)+".saCfree(xyz)= \t" + ss.setNum(out.st[k].saCfree.x,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].saCfree.y,'f',3);
        outstr +=                               "   \t" + ss.setNum(out.st[k].saCfree.z,'f',3);
        outstr += "\ns"+s.setNum(k)+".F(xyz)=       \t" + ss.setNum(out.st[k].F.x      ,'f',1);
        outstr +=                               "   \t" + ss.setNum(out.st[k].F.y      ,'f',1);
        outstr +=                               "   \t" + ss.setNum(out.st[k].F.z      ,'f',1);

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
            outstr += "\ns"+s.setNum(k)+".F1(xyz)=      \t" + ss.setNum(out.st[k].F1.x    ,'f',3);
            outstr +=                               "   \t" + ss.setNum(out.st[k].F1.y    ,'f',3);
            outstr +=                               "   \t" + ss.setNum(out.st[k].F1.z    ,'f',3);
            outstr += "\ns"+s.setNum(k)+".F2(xyz)=      \t" + ss.setNum(out.st[k].F2.x    ,'f',3);
            outstr +=                               "   \t" + ss.setNum(out.st[k].F2.y    ,'f',3);
            outstr +=                               "   \t" + ss.setNum(out.st[k].F2.z    ,'f',3);
        }
    }
    outstr += "\n----------------------------------------------------------------------";
    outstr += "\nmainrotor";
    outstr += "\nmr.O.x=          \t" + ss.setNum(out.mr.O.x      ,'f',1);
    outstr += "\nmr.O.y=          \t" + ss.setNum(out.mr.O.y      ,'f',1);
    outstr += "\nmr.O.z=          \t" + ss.setNum(out.mr.O.z      ,'f',1);
    outstr += "\nmr.H_sr=         \t" + ss.setNum(out.mr.Fhx_sr   ,'f',1);
    outstr += "\nmr.T_sr=         \t" + ss.setNum(out.mr.Fhy_sr   ,'f',1);
    outstr += "\nmr.S_sr=         \t" + ss.setNum(out.mr.Fhz_sr   ,'f',1);
    outstr += "\nmr.Mx_sr=        \t" + ss.setNum(out.mr.Mhx_sr   ,'f',1);
    outstr += "\nmr.My_sr=        \t" + ss.setNum(out.mr.Mhy_sr   ,'f',1);
    outstr += "\nmr.Mz_sr=        \t" + ss.setNum(out.mr.Mhz_sr   ,'f',1);
    outstr += "\nmr.D=            \t" + ss.setNum(out.mr.D        ,'f',1);
    outstr += "\nmr.eps_fz=       \t" + ss.setNum(out.mr.eps_fz   ,'f',4);
    outstr += "\nmr.eps_fx=       \t" + ss.setNum(out.mr.eps_fx   ,'f',4);
    outstr += "\nmr.chi_hub=      \t" + ss.setNum(out.mr.chi_hub    ,'f',4);
    outstr += "\nmr.eta_hub=      \t" + ss.setNum(out.mr.eta_hub    ,'f',4);
    outstr += "\nmr.solve=        \t" + ss.setNum(out.mr.solve);
    if (out.mr.solve)
    {
        outstr += "\nmr.eh=           \t" + ss.setNum(out.mr.eh.x     ,'f',6);
        outstr +=                 "   \t" + ss.setNum(out.mr.eh.y     ,'f',6);
        outstr +=                 "   \t" + ss.setNum(out.mr.eh.z     ,'f',6);
        outstr += "\nmr.wh=           \t" + ss.setNum(out.mr.wh.x     ,'f',6);
        outstr +=                 "   \t" + ss.setNum(out.mr.wh.y     ,'f',6);
        outstr +=                 "   \t" + ss.setNum(out.mr.wh.z     ,'f',6);
        outstr += "\nmr.ah=           \t" + ss.setNum(out.mr.ah.x     ,'f',6);
        outstr +=                 "   \t" + ss.setNum(out.mr.ah.y     ,'f',6);
        outstr +=                 "   \t" + ss.setNum(out.mr.ah.z     ,'f',6);
        outstr += "\nmr.vh=           \t" + ss.setNum(out.mr.vh.x     ,'f',6);
        outstr +=                 "   \t" + ss.setNum(out.mr.vh.y     ,'f',6);
        outstr +=                 "   \t" + ss.setNum(out.mr.vh.z     ,'f',6);
        outstr += "\nmr.dct=          \t" + ss.setNum(out.mr.dct.x    ,'f',3);
        outstr +=                 "   \t" + ss.setNum(out.mr.dct.y    ,'f',3);
        outstr +=                 "   \t" + ss.setNum(out.mr.dct.z    ,'f',3);
        outstr += "\nmr.angbs=        \t" + ss.setNum(out.mr.angbs    ,'f',3);
        outstr += "\nmr.dctbs=        \t" + ss.setNum(out.mr.dctbs    ,'f',6);
        outstr += "\nmr.nb=           \t" + ss.setNum(out.mr.nb);
        outstr += "\nmr.nSec=         \t" + ss.setNum(out.mr.nSec);
        outstr += "\nmr.sec07=        \t" + ss.setNum(out.mr.sec07);
        outstr += "\nmr.revcount=     \t" + ss.setNum(out.mr.revcount);
        outstr += "\nmr.psihub=       \t" + ss.setNum(out.mr.psih     ,'f',1);
        outstr += "\nmr.fi0=          \t" + ss.setNum(out.mr.fi0      ,'f',3);
        outstr += "\nmr.chi=          \t" + ss.setNum(out.mr.chi      ,'f',3);
        outstr += "\nmr.eta=          \t" + ss.setNum(out.mr.eta      ,'f',3);
        outstr += "\nmr.alfa=         \t" + ss.setNum(out.mr.alfa     ,'f',3);
        outstr += "\nmr.beta=         \t" + ss.setNum(out.mr.beta     ,'f',3);
        outstr += "\nmr.Fhx_sum=      \t" + ss.setNum(out.mr.Fhx_sum  ,'f',1);
        outstr += "\nmr.Fhy_sum=      \t" + ss.setNum(out.mr.Fhy_sum  ,'f',1);
        outstr += "\nmr.Fhz_sum=      \t" + ss.setNum(out.mr.Fhz_sum  ,'f',1);
        outstr += "\nmr.Mhx_sum=      \t" + ss.setNum(out.mr.Mhx_sum  ,'f',1);
        outstr += "\nmr.Mhy_sum=      \t" + ss.setNum(out.mr.Mhy_sum  ,'f',1);
        outstr += "\nmr.Mhz_sum=      \t" + ss.setNum(out.mr.Mhz_sum  ,'f',1);
        for (int l = 0; l < out.mr.nb; l++)
        {
            outstr += "\n----------------------------------------------------------------------";
            outstr += "\nmr.blade#        \t" + ss.setNum(l+1);
            outstr += "\nmr.b"+s.setNum(l)+".psi0=         \t" + ss.setNum(out.mr.b[l].psi0  ,'f',4);
            outstr += "\nmr.b"+s.setNum(l)+".psi=          \t" + ss.setNum(out.mr.b[l].psi   ,'f',4);
            outstr += "\nmr.b"+s.setNum(l)+".fi07=         \t" + ss.setNum(out.mr.b[l].fi7   ,'f',6);
            outstr += "\nmr.b"+s.setNum(l)+".dfi07=        \t" + ss.setNum(out.mr.b[l].dfi7  ,'f',6);
            outstr += "\nmr.b"+s.setNum(l)+".d2fi07=       \t" + ss.setNum(out.mr.b[l].d2fi7 ,'f',6);
            outstr += "\nmr.b"+s.setNum(l)+".gama=         \t" + ss.setNum(out.mr.b[l].gama  ,'f',6);
            outstr += "\nmr.b"+s.setNum(l)+".dgama=        \t" + ss.setNum(out.mr.b[l].dgama ,'f',6);
            outstr += "\nmr.b"+s.setNum(l)+".d2gama=       \t" + ss.setNum(out.mr.b[l].d2gama,'f',6);
            outstr += "\nmr.b"+s.setNum(l)+".beta=         \t" + ss.setNum(out.mr.b[l].beta  ,'f',6);
            outstr += "\nmr.b"+s.setNum(l)+".dbeta=        \t" + ss.setNum(out.mr.b[l].dbeta ,'f',6);
            outstr += "\nmr.b"+s.setNum(l)+".d2beta=       \t" + ss.setNum(out.mr.b[l].d2beta,'f',6);
            outstr += "\nmr.b"+s.setNum(l)+".ksi=          \t" + ss.setNum(out.mr.b[l].ksi   ,'f',6);
            outstr += "\nmr.b"+s.setNum(l)+".dksi=         \t" + ss.setNum(out.mr.b[l].dksi  ,'f',6);
            outstr += "\nmr.b"+s.setNum(l)+".d2ksi=        \t" + ss.setNum(out.mr.b[l].d2ksi ,'f',6);
            outstr += "\nmr.b"+s.setNum(l)+".T=            \t" + ss.setNum(out.mr.b[l].T     ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".Q=            \t" + ss.setNum(out.mr.b[l].Q     ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".Ma=           \t" + ss.setNum(out.mr.b[l].Ma    ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".H=            \t" + ss.setNum(out.mr.b[l].H     ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".S=            \t" + ss.setNum(out.mr.b[l].S     ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".Nr=           \t" + ss.setNum(out.mr.b[l].Nr    ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".Nf=           \t" + ss.setNum(out.mr.b[l].Nf    ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".K=            \t" + ss.setNum(out.mr.b[l].K1    ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".P1=           \t" + ss.setNum(out.mr.b[l].P1    ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".P2=           \t" + ss.setNum(out.mr.b[l].P2    ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".Fdmp=         \t" + ss.setNum(out.mr.b[l].Fdmp  ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".Fpov=         \t" + ss.setNum(out.mr.b[l].Fpov  ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".Paxz=         \t" + ss.setNum(out.mr.b[l].Paxz  ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".Payy=         \t" + ss.setNum(out.mr.b[l].Payy  ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".Fdmp=         \t" + ss.setNum(out.mr.b[l].Fdmp  ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".Fy=           \t" + ss.setNum(out.mr.b[l].Fy    ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".Fx=           \t" + ss.setNum(out.mr.b[l].Fx    ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".Fz=           \t" + ss.setNum(out.mr.b[l].Fz    ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M1T=          \t" + ss.setNum(out.mr.b[l].M1T   ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M1Q=          \t" + ss.setNum(out.mr.b[l].M1Q   ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M1Gx=         \t" + ss.setNum(out.mr.b[l].M1Gx  ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M1Gy=         \t" + ss.setNum(out.mr.b[l].M1Gy  ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M1K=          \t" + ss.setNum(out.mr.b[l].M1K   ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M1Cx=         \t" + ss.setNum(out.mr.b[l].M1Cx  ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M1Cy=         \t" + ss.setNum(out.mr.b[l].M1Cy  ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M1Px=         \t" + ss.setNum(out.mr.b[l].M1Px  ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M1Py=         \t" + ss.setNum(out.mr.b[l].M1Py  ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M1x=          \t" + ss.setNum(out.mr.b[l].M1x   ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M1y=          \t" + ss.setNum(out.mr.b[l].M1y   ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M1z=          \t" + ss.setNum(out.mr.b[l].M1z   ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M2T=          \t" + ss.setNum(out.mr.b[l].M2T   ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M2Q=          \t" + ss.setNum(out.mr.b[l].M2Q   ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M2Gx=         \t" + ss.setNum(out.mr.b[l].M2Gx  ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M2Gy=         \t" + ss.setNum(out.mr.b[l].M2Gy  ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M2K=          \t" + ss.setNum(out.mr.b[l].M2K   ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M2Cx=         \t" + ss.setNum(out.mr.b[l].M2Cx  ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M2Cy=         \t" + ss.setNum(out.mr.b[l].M2Cy  ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M2Px=         \t" + ss.setNum(out.mr.b[l].M2Px  ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M2Py=         \t" + ss.setNum(out.mr.b[l].M2Py  ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M2D=          \t" + ss.setNum(out.mr.b[l].M2D   ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M2x=          \t" + ss.setNum(out.mr.b[l].M2x   ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M2y=          \t" + ss.setNum(out.mr.b[l].M2y   ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M2z=          \t" + ss.setNum(out.mr.b[l].M2z   ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M3Ma=         \t" + ss.setNum(out.mr.b[l].M3Ma  ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M3cb=         \t" + ss.setNum(out.mr.b[l].M3Cz  ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M3su=         \t" + ss.setNum(out.mr.b[l].M3su  ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M3tr=         \t" + ss.setNum(out.mr.b[l].M3tr  ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M3i=          \t" + ss.setNum(out.mr.b[l].M3iz  ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M3x=          \t" + ss.setNum(out.mr.b[l].M3x   ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M3y=          \t" + ss.setNum(out.mr.b[l].M3y   ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".M3z=          \t" + ss.setNum(out.mr.b[l].M3z   ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".Fxh=          \t" + ss.setNum(out.mr.b[l].Fhx   ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".Fyh=          \t" + ss.setNum(out.mr.b[l].Fhy   ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".Fzh=          \t" + ss.setNum(out.mr.b[l].Fhz   ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".Mxh=          \t" + ss.setNum(out.mr.b[l].Mhx   ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".Myh=          \t" + ss.setNum(out.mr.b[l].Mhy   ,'f',1);
            outstr += "\nmr.b"+s.setNum(l)+".Mzh=          \t" + ss.setNum(out.mr.b[l].Mhz   ,'f',1);
            //Распределенные параметры лопасти по сечению и по азимуту)     \t\tUywx,м/с\tUywz,м/с
            outstr += "\n_r,   \tr,м   \t alfr,грд  \tfi,грд\t\tgama,грд  \tU,м/с \t\tUx,м/с\t\tUy,м/с\t\tFi,грд\t\tlambda,\t\tM,    \t\tcy,   \t\tcx,  \t\tmz,  \t\tdT,Н  \t\tdQ,Н  \t\tdMa,Нм ";
            for (int sec = 0; sec < out.mr.nSec; sec++)
            {
                outstr += "\n" + ss.setNum(out.mr.b[l]._r    .at(sec),'f',3);//
                outstr += "\t" + ss.setNum(out.mr.b[l].r     .at(sec),'f',3);//
                outstr += "\t "+ ss.setNum(out.mr.b[l].alfar .at(sec),'f',6);//Угол атаки alpha_r, градусы
                outstr += "\t" + ss.setNum(out.mr.b[l].fi    .at(sec),'f',6);//Угол установки сечения fi, градусы
                outstr += "\t" + ss.setNum(out.mr.b[l].fl    .at(sec),'f',6);//Угол закрутки сечения , градусы
                outstr += "\t" + ss.setNum(out.mr.b[l].U     .at(sec),'f',6);//Результирующая скорость обтекания U, м/с
                outstr += "\t" + ss.setNum(out.mr.b[l].Ux    .at(sec),'f',6);//скорость обтекания Ux, м/с
                outstr += "\t" + ss.setNum(out.mr.b[l].Uy    .at(sec),'f',6);//скорость обтекания Uy, м/с
                outstr += "\t" + ss.setNum(out.mr.b[l].Fi    .at(sec),'f',6);//Угол притекания Fi, градусы
                outstr += "\t" + ss.setNum(out.mr.b[l].lambda.at(sec),'f',6);//коэф. протекания в сечении
                outstr += "\t" + ss.setNum(out.mr.b[l].M     .at(sec),'f',6);//Число Маха M
                outstr += "\t" + ss.setNum(out.mr.b[l].cy    .at(sec),'f',6);//Коэффициент подъемной силы Cy
                outstr += "\t" + ss.setNum(out.mr.b[l].cx    .at(sec),'f',6);//Коэффициент профильного сопротивления Cxp
                outstr += "\t" + ss.setNum(out.mr.b[l].mz    .at(sec),'f',6);//Коэффициент момента аэродин.сил
                outstr += "\t" + ss.setNum(out.mr.b[l].dT_dr .at(sec),'f',6);//погонная сила тяги dT/dr, [Н/м]
                outstr += "\t" + ss.setNum(out.mr.b[l].dQ_dr .at(sec),'f',6);//погонная сила сопротивления профиля dQ/dr, [Н/м]
                outstr += "\t" + ss.setNum(out.mr.b[l].dMa_dr.at(sec),'f',6);//погонный момент dMa/dr, [Нм/м]
                //outstr += "\t" + ss.setNum(out.mr.b[l].Uywx  .at(sec),'f',6);
                //outstr += "\t" + ss.setNum(out.mr.b[l].Uywz  .at(sec),'f',6);
            }
        }
        /*outstr += "\nmr.vgsolve=      \t" + ss.setNum(out.mr.vgsolve);
        if(out.mr.vgsolve)
        {
            outstr += "\nvg.psi=          \t" + ss.setNum(out.mr.vg.psi   ,'f',6);
            outstr += "\nvg.eps=          \t" + ss.setNum(out.mr.vg.eps   ,'f',6);
            outstr += "\nvg.Mn=           \t" + ss.setNum(out.mr.vg.Mn    ,'f',3);
            outstr += "\nvg.Mt=           \t" + ss.setNum(out.mr.vg.Mt    ,'f',3);
            outstr += "\nvg.Mk=           \t" + ss.setNum(out.mr.vg.Mk    ,'f',3);
            outstr += "\nvg.Mi=           \t" + ss.setNum(out.mr.vg.Mi    ,'f',3);
        }*/
    }
    outstr += "\n----------------------------------------------------------------------";
    outstr += "\ntailrotor";
    outstr += "\ntr.O.x=          \t" + ss.setNum(out.tr.O.x      ,'f',1);
    outstr += "\ntr.O.y=          \t" + ss.setNum(out.tr.O.y      ,'f',1);
    outstr += "\ntr.O.z=          \t" + ss.setNum(out.tr.O.z      ,'f',1);
    outstr += "\ntr.H=            \t" + ss.setNum(out.tr.Fhx_sr   ,'f',1);
    outstr += "\ntr.T=            \t" + ss.setNum(out.tr.Fhy_sr   ,'f',1);
    outstr += "\ntr.S=            \t" + ss.setNum(out.tr.Fhz_sr   ,'f',1);
    outstr += "\ntr.Mx=           \t" + ss.setNum(out.tr.Mhx_sr   ,'f',1);
    outstr += "\ntr.My=           \t" + ss.setNum(out.tr.Mhy_sr   ,'f',1);
    outstr += "\ntr.Mz=           \t" + ss.setNum(out.tr.Mhz_sr   ,'f',1);
    outstr += "\ntr.D=            \t" + ss.setNum(out.tr.D        ,'f',1);
    outstr += "\ntr.solve=        \t" + ss.setNum(out.tr.solve);
    if (out.tr.solve)
    {
        outstr += "\ntr.nb=           \t" + ss.setNum(out.tr.nb);
        outstr += "\ntr.nSec=         \t" + ss.setNum(out.tr.nSec);
        outstr += "\ntr.sec07=        \t" + ss.setNum(out.tr.sec07);
        outstr += "\ntr.revcount=     \t" + ss.setNum(out.tr.revcount);
        outstr += "\ntr.psihub=       \t" + ss.setNum(out.tr.psih     ,'f',1);
        outstr += "\ntr.fi0=          \t" + ss.setNum(out.tr.fi0      ,'f',3);
        outstr += "\ntr.alfa=         \t" + ss.setNum(out.tr.alfa     ,'f',3);
        outstr += "\ntr.beta=         \t" + ss.setNum(out.tr.beta     ,'f',3);
        outstr += "\ntr.Fhx_sum=      \t" + ss.setNum(out.tr.Fhx_sum  ,'f',1);
        outstr += "\ntr.Fhy_sum=      \t" + ss.setNum(out.tr.Fhy_sum  ,'f',1);
        outstr += "\ntr.Fhz_sum=      \t" + ss.setNum(out.tr.Fhz_sum  ,'f',1);
        outstr += "\ntr.Mhx_sum=      \t" + ss.setNum(out.tr.Mhx_sum  ,'f',1);
        outstr += "\ntr.Mhy_sum=      \t" + ss.setNum(out.tr.Mhy_sum  ,'f',1);
        outstr += "\ntr.Mhz_sum=      \t" + ss.setNum(out.tr.Mhz_sum  ,'f',1);
        for (int l = 0; l < out.tr.nb; l++)
        {
            outstr += "\n----------------------------------------------------------------------";
            outstr += "\ntr.blade#        \t" + ss.setNum(l+1);
            outstr += "\ntr.b"+s.setNum(l)+".psi0=         \t" + ss.setNum(out.tr.b[l].psi0  ,'f',4);
            outstr += "\ntr.b"+s.setNum(l)+".psi=          \t" + ss.setNum(out.tr.b[l].psi   ,'f',4);
            outstr += "\ntr.b"+s.setNum(l)+".fi07=         \t" + ss.setNum(out.tr.b[l].fi7   ,'f',6);
            outstr += "\ntr.b"+s.setNum(l)+".dfi07=        \t" + ss.setNum(out.tr.b[l].dfi7  ,'f',6);
            outstr += "\ntr.b"+s.setNum(l)+".d2fi07=       \t" + ss.setNum(out.tr.b[l].d2fi7 ,'f',6);
            outstr += "\ntr.b"+s.setNum(l)+".gama=         \t" + ss.setNum(out.tr.b[l].gama  ,'f',6);
            outstr += "\ntr.b"+s.setNum(l)+".dgama=        \t" + ss.setNum(out.tr.b[l].dgama ,'f',6);
            outstr += "\ntr.b"+s.setNum(l)+".d2gama=       \t" + ss.setNum(out.tr.b[l].d2gama,'f',6);
            outstr += "\ntr.b"+s.setNum(l)+".beta=         \t" + ss.setNum(out.tr.b[l].beta  ,'f',6);
            outstr += "\ntr.b"+s.setNum(l)+".dbeta=        \t" + ss.setNum(out.tr.b[l].dbeta ,'f',6);
            outstr += "\ntr.b"+s.setNum(l)+".d2beta=       \t" + ss.setNum(out.tr.b[l].d2beta,'f',6);
            outstr += "\ntr.b"+s.setNum(l)+".ksi=          \t" + ss.setNum(out.tr.b[l].ksi   ,'f',6);
            outstr += "\ntr.b"+s.setNum(l)+".dksi=         \t" + ss.setNum(out.tr.b[l].dksi  ,'f',6);
            outstr += "\ntr.b"+s.setNum(l)+".d2ksi=        \t" + ss.setNum(out.tr.b[l].d2ksi ,'f',6);
            outstr += "\ntr.b"+s.setNum(l)+".T=            \t" + ss.setNum(out.tr.b[l].T     ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".Q=            \t" + ss.setNum(out.tr.b[l].Q     ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".Ma=           \t" + ss.setNum(out.tr.b[l].Ma    ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".H=            \t" + ss.setNum(out.tr.b[l].H     ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".S=            \t" + ss.setNum(out.tr.b[l].S     ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".Nr=           \t" + ss.setNum(out.tr.b[l].Nr    ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".Nf=           \t" + ss.setNum(out.tr.b[l].Nf    ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".K=            \t" + ss.setNum(out.tr.b[l].K1    ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".P1=           \t" + ss.setNum(out.tr.b[l].P1    ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".P2=           \t" + ss.setNum(out.tr.b[l].P2    ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".Fdmp=         \t" + ss.setNum(out.tr.b[l].Fdmp  ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".Fpov=         \t" + ss.setNum(out.tr.b[l].Fpov  ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".Fy=           \t" + ss.setNum(out.tr.b[l].Fy    ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".Fx=           \t" + ss.setNum(out.tr.b[l].Fx    ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".Fz=           \t" + ss.setNum(out.tr.b[l].Fz    ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M1T=          \t" + ss.setNum(out.tr.b[l].M1T   ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M1Q=          \t" + ss.setNum(out.tr.b[l].M1Q   ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M1Gx=         \t" + ss.setNum(out.tr.b[l].M1Gx  ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M1Gy=         \t" + ss.setNum(out.tr.b[l].M1Gy  ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M1K=          \t" + ss.setNum(out.tr.b[l].M1K   ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M1Cx=         \t" + ss.setNum(out.tr.b[l].M1Cx  ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M1Cy=         \t" + ss.setNum(out.tr.b[l].M1Cy  ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M1Px=         \t" + ss.setNum(out.tr.b[l].M1Px  ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M1Py=         \t" + ss.setNum(out.tr.b[l].M1Py  ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M1x=          \t" + ss.setNum(out.tr.b[l].M1x   ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M1y=          \t" + ss.setNum(out.tr.b[l].M1y   ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M1z=          \t" + ss.setNum(out.tr.b[l].M1z   ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M2T=          \t" + ss.setNum(out.tr.b[l].M2T   ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M2Q=          \t" + ss.setNum(out.tr.b[l].M2Q   ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M2Gx=         \t" + ss.setNum(out.tr.b[l].M2Gx  ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M2Gy=         \t" + ss.setNum(out.tr.b[l].M2Gy  ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M2K=          \t" + ss.setNum(out.tr.b[l].M2K   ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M2Cx=         \t" + ss.setNum(out.tr.b[l].M2Cx  ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M2Cy=         \t" + ss.setNum(out.tr.b[l].M2Cy  ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M2Px=         \t" + ss.setNum(out.tr.b[l].M2Px  ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M2Py=         \t" + ss.setNum(out.tr.b[l].M2Py  ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M2D=          \t" + ss.setNum(out.tr.b[l].M2D   ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M2x=          \t" + ss.setNum(out.tr.b[l].M2x   ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M2y=          \t" + ss.setNum(out.tr.b[l].M2y   ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M2z=          \t" + ss.setNum(out.tr.b[l].M2z   ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M3Ma=         \t" + ss.setNum(out.tr.b[l].M3Ma  ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M3cb=         \t" + ss.setNum(out.tr.b[l].M3Cz  ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M3su=         \t" + ss.setNum(out.tr.b[l].M3su  ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M3tr=         \t" + ss.setNum(out.tr.b[l].M3tr  ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M3i=          \t" + ss.setNum(out.tr.b[l].M3iz  ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M3x=          \t" + ss.setNum(out.tr.b[l].M3x   ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M3y=          \t" + ss.setNum(out.tr.b[l].M3y   ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".M3z=          \t" + ss.setNum(out.tr.b[l].M3z   ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".Fxh=          \t" + ss.setNum(out.tr.b[l].Fhx   ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".Fyh=          \t" + ss.setNum(out.tr.b[l].Fhy   ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".Fzh=          \t" + ss.setNum(out.tr.b[l].Fhz   ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".Mxh=          \t" + ss.setNum(out.tr.b[l].Mhx   ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".Myh=          \t" + ss.setNum(out.tr.b[l].Mhy   ,'f',1);
            outstr += "\ntr.b"+s.setNum(l)+".Mzh=          \t" + ss.setNum(out.tr.b[l].Mhz   ,'f',1);
            //Распределенные параметры лопасти по сечению и по азимуту)    \t\tUywx,м/с\tUywz,м/с
            outstr += "\n_r,   \tr,м   \t alfr,грд  \tfi,грд\t\tgama,грд  \tU,м/с \t\tUx,м/с\t\tUy,м/с\t\tFi,грд\t\tlambda,\t\tM,    \t\tcy,   \t\tcx,  \t\tmz,  \t\tdT,Н  \t\tdQ,Н  \t\tdMa,Нм ";
            for (int sec = 0; sec < out.tr.nSec; sec++)
            {
                outstr += "\n" + ss.setNum(out.tr.b[l]._r    .at(sec),'f',3);//
                outstr += "\t" + ss.setNum(out.tr.b[l].r     .at(sec),'f',3);//
                outstr += "\t "+ ss.setNum(out.tr.b[l].alfar .at(sec),'f',6);//Угол атаки alpha_r, градусы
                outstr += "\t" + ss.setNum(out.tr.b[l].fi    .at(sec),'f',6);//Угол установки сечения fi, градусы
                outstr += "\t" + ss.setNum(out.tr.b[l].fl    .at(sec),'f',6);//Угол закрутки сечения , градусы
                outstr += "\t" + ss.setNum(out.tr.b[l].U     .at(sec),'f',6);//Результирующая скорость обтекания U, м/с
                outstr += "\t" + ss.setNum(out.tr.b[l].Ux    .at(sec),'f',6);//скорость обтекания Ux, м/с
                outstr += "\t" + ss.setNum(out.tr.b[l].Uy    .at(sec),'f',6);//скорость обтекания Uy, м/с
                outstr += "\t" + ss.setNum(out.tr.b[l].Fi    .at(sec),'f',6);//Угол притекания Fi, градусы
                outstr += "\t" + ss.setNum(out.tr.b[l].lambda.at(sec),'f',6);//коэф. протекания в сечении
                outstr += "\t" + ss.setNum(out.tr.b[l].M     .at(sec),'f',6);//Число Маха M
                outstr += "\t" + ss.setNum(out.tr.b[l].cy    .at(sec),'f',6);//Коэффициент подъемной силы Cy
                outstr += "\t" + ss.setNum(out.tr.b[l].cx    .at(sec),'f',6);//Коэффициент профильного сопротивления Cxp
                outstr += "\t" + ss.setNum(out.tr.b[l].mz    .at(sec),'f',6);//Коэффициент момента аэродин.сил
                outstr += "\t" + ss.setNum(out.tr.b[l].dT_dr .at(sec),'f',6);//погонная сила тяги dT/dr, [Н/м]
                outstr += "\t" + ss.setNum(out.tr.b[l].dQ_dr .at(sec),'f',6);//погонная сила сопротивления профиля dQ/dr, [Н/м]
                outstr += "\t" + ss.setNum(out.tr.b[l].dMa_dr.at(sec),'f',6);//погонный момент dMa/dr, [Нм/м]
                //outstr += "\t" + ss.setNum(out..b[l].Uywx  .at(sec),'f',6);
                //outstr += "\t" + ss.setNum(out..b[l].Uywz  .at(sec),'f',6);
            }
        }
    }
    outstr += "\n----------------------------------------------------------------------";
    outstr += "\npushrotor";
    outstr += "\npr.O.x=          \t" + ss.setNum(out.pr.O.x      ,'f',1);
    outstr += "\npr.O.y=          \t" + ss.setNum(out.pr.O.y      ,'f',1);
    outstr += "\npr.O.z=          \t" + ss.setNum(out.pr.O.z      ,'f',1);
    outstr += "\npr.H=            \t" + ss.setNum(out.pr.Fhx_sr   ,'f',1);
    outstr += "\npr.T=            \t" + ss.setNum(out.pr.Fhy_sr   ,'f',1);
    outstr += "\npr.S=            \t" + ss.setNum(out.pr.Fhz_sr   ,'f',1);
    outstr += "\npr.Mx=           \t" + ss.setNum(out.pr.Mhx_sr   ,'f',1);
    outstr += "\npr.My=           \t" + ss.setNum(out.pr.Mhy_sr   ,'f',1);
    outstr += "\npr.Mz=           \t" + ss.setNum(out.pr.Mhz_sr   ,'f',1);
    outstr += "\npr.D=            \t" + ss.setNum(out.pr.D        ,'f',1);
    outstr += "\npr.solve=        \t" + ss.setNum(out.pr.solve);
    if (out.pr.solve)
    {
        outstr += "\npr.nb=           \t" + ss.setNum(out.pr.nb);
        outstr += "\npr.nSec=         \t" + ss.setNum(out.pr.nSec);
        outstr += "\npr.sec07=        \t" + ss.setNum(out.pr.sec07);
        outstr += "\npr.revcount=     \t" + ss.setNum(out.pr.revcount);
        outstr += "\npr.psihub=       \t" + ss.setNum(out.pr.psih     ,'f',1);
        outstr += "\npr.fi0=          \t" + ss.setNum(out.pr.fi0      ,'f',3);
        outstr += "\npr.alfa=         \t" + ss.setNum(out.pr.alfa     ,'f',3);
        outstr += "\npr.beta=         \t" + ss.setNum(out.pr.beta     ,'f',3);
        outstr += "\npr.Fhx_sum=      \t" + ss.setNum(out.pr.Fhx_sum  ,'f',1);
        outstr += "\npr.Fhy_sum=      \t" + ss.setNum(out.pr.Fhy_sum  ,'f',1);
        outstr += "\npr.Fhz_sum=      \t" + ss.setNum(out.pr.Fhz_sum  ,'f',1);
        outstr += "\npr.Mhx_sum=      \t" + ss.setNum(out.pr.Mhx_sum  ,'f',1);
        outstr += "\npr.Mhy_sum=      \t" + ss.setNum(out.pr.Mhy_sum  ,'f',1);
        outstr += "\npr.Mhz_sum=      \t" + ss.setNum(out.pr.Mhz_sum  ,'f',1);
        for (int l = 0; l < out.pr.nb; l++)
        {
            outstr += "\n----------------------------------------------------------------------";
            outstr += "\npr.blade#        \t" + ss.setNum(l+1);
            outstr += "\npr.b"+s.setNum(l)+".psi0=         \t" + ss.setNum(out.pr.b[l].psi0  ,'f',4);
            outstr += "\npr.b"+s.setNum(l)+".psi=          \t" + ss.setNum(out.pr.b[l].psi   ,'f',4);
            outstr += "\npr.b"+s.setNum(l)+".fi07=         \t" + ss.setNum(out.pr.b[l].fi7   ,'f',6);
            outstr += "\npr.b"+s.setNum(l)+".dfi07=        \t" + ss.setNum(out.pr.b[l].dfi7  ,'f',6);
            outstr += "\npr.b"+s.setNum(l)+".d2fi07=       \t" + ss.setNum(out.pr.b[l].d2fi7 ,'f',6);
            outstr += "\npr.b"+s.setNum(l)+".gama=         \t" + ss.setNum(out.pr.b[l].gama  ,'f',6);
            outstr += "\npr.b"+s.setNum(l)+".dgama=        \t" + ss.setNum(out.pr.b[l].dgama ,'f',6);
            outstr += "\npr.b"+s.setNum(l)+".d2gama=       \t" + ss.setNum(out.pr.b[l].d2gama,'f',6);
            outstr += "\npr.b"+s.setNum(l)+".beta=         \t" + ss.setNum(out.pr.b[l].beta  ,'f',6);
            outstr += "\npr.b"+s.setNum(l)+".dbeta=        \t" + ss.setNum(out.pr.b[l].dbeta ,'f',6);
            outstr += "\npr.b"+s.setNum(l)+".d2beta=       \t" + ss.setNum(out.pr.b[l].d2beta,'f',6);
            outstr += "\npr.b"+s.setNum(l)+".ksi=          \t" + ss.setNum(out.pr.b[l].ksi   ,'f',6);
            outstr += "\npr.b"+s.setNum(l)+".dksi=         \t" + ss.setNum(out.pr.b[l].dksi  ,'f',6);
            outstr += "\npr.b"+s.setNum(l)+".d2ksi=        \t" + ss.setNum(out.pr.b[l].d2ksi ,'f',6);
            outstr += "\npr.b"+s.setNum(l)+".T=            \t" + ss.setNum(out.pr.b[l].T     ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".Q=            \t" + ss.setNum(out.pr.b[l].Q     ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".Ma=           \t" + ss.setNum(out.pr.b[l].Ma    ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".H=            \t" + ss.setNum(out.pr.b[l].H     ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".S=            \t" + ss.setNum(out.pr.b[l].S     ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".Nr=           \t" + ss.setNum(out.pr.b[l].Nr    ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".Nf=           \t" + ss.setNum(out.pr.b[l].Nf    ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".K=            \t" + ss.setNum(out.pr.b[l].K1    ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".P1=           \t" + ss.setNum(out.pr.b[l].P1    ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".P2=           \t" + ss.setNum(out.pr.b[l].P2    ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".Fdmp=         \t" + ss.setNum(out.pr.b[l].Fdmp  ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".Fpov=         \t" + ss.setNum(out.pr.b[l].Fpov  ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".Fy=           \t" + ss.setNum(out.pr.b[l].Fy    ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".Fx=           \t" + ss.setNum(out.pr.b[l].Fx    ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".Fz=           \t" + ss.setNum(out.pr.b[l].Fz    ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M1T=          \t" + ss.setNum(out.pr.b[l].M1T   ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M1Q=          \t" + ss.setNum(out.pr.b[l].M1Q   ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M1Gx=         \t" + ss.setNum(out.pr.b[l].M1Gx  ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M1Gy=         \t" + ss.setNum(out.pr.b[l].M1Gy  ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M1K=          \t" + ss.setNum(out.pr.b[l].M1K   ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M1Cx=         \t" + ss.setNum(out.pr.b[l].M1Cx  ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M1Cy=         \t" + ss.setNum(out.pr.b[l].M1Cy  ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M1Px=         \t" + ss.setNum(out.pr.b[l].M1Px  ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M1Py=         \t" + ss.setNum(out.pr.b[l].M1Py  ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M1x=          \t" + ss.setNum(out.pr.b[l].M1x   ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M1y=          \t" + ss.setNum(out.pr.b[l].M1y   ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M1z=          \t" + ss.setNum(out.pr.b[l].M1z   ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M2T=          \t" + ss.setNum(out.pr.b[l].M2T   ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M2Q=          \t" + ss.setNum(out.pr.b[l].M2Q   ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M2Gx=         \t" + ss.setNum(out.pr.b[l].M2Gx  ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M2Gy=         \t" + ss.setNum(out.pr.b[l].M2Gy  ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M2K=          \t" + ss.setNum(out.pr.b[l].M2K   ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M2Cx=         \t" + ss.setNum(out.pr.b[l].M2Cx  ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M2Cy=         \t" + ss.setNum(out.pr.b[l].M2Cy  ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M2Px=         \t" + ss.setNum(out.pr.b[l].M2Px  ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M2Py=         \t" + ss.setNum(out.pr.b[l].M2Py  ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M2D=          \t" + ss.setNum(out.pr.b[l].M2D   ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M2x=          \t" + ss.setNum(out.pr.b[l].M2x   ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M2y=          \t" + ss.setNum(out.pr.b[l].M2y   ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M2z=          \t" + ss.setNum(out.pr.b[l].M2z   ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M3Ma=         \t" + ss.setNum(out.pr.b[l].M3Ma  ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M3cb=         \t" + ss.setNum(out.pr.b[l].M3Cz  ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M3su=         \t" + ss.setNum(out.pr.b[l].M3su  ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M3tr=         \t" + ss.setNum(out.pr.b[l].M3tr  ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M3i=          \t" + ss.setNum(out.pr.b[l].M3iz  ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M3x=          \t" + ss.setNum(out.pr.b[l].M3x   ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M3y=          \t" + ss.setNum(out.pr.b[l].M3y   ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".M3z=          \t" + ss.setNum(out.pr.b[l].M3z   ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".Fxh=          \t" + ss.setNum(out.pr.b[l].Fhx   ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".Fyh=          \t" + ss.setNum(out.pr.b[l].Fhy   ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".Fzh=          \t" + ss.setNum(out.pr.b[l].Fhz   ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".Mxh=          \t" + ss.setNum(out.pr.b[l].Mhx   ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".Myh=          \t" + ss.setNum(out.pr.b[l].Mhy   ,'f',1);
            outstr += "\npr.b"+s.setNum(l)+".Mzh=          \t" + ss.setNum(out.pr.b[l].Mhz   ,'f',1);
            //Распределенные параметры лопасти по сечению и по азимуту)    \t\tUywx,м/с\tUywz,м/с
            outstr += "\n_r,   \tr,м   \t alfr,грд  \tfi,грд\t\tgama,грд  \tU,м/с \t\tUx,м/с\t\tUy,м/с\t\tFi,грд\t\tlambda,\t\tM,    \t\tcy,   \t\tcx,  \t\tmz,  \t\tdT,Н  \t\tdQ,Н  \t\tdMa,Нм ";
            for (int sec = 0; sec < out.pr.nSec; sec++)
            {
                outstr += "\n" + ss.setNum(out.pr.b[l]._r    .at(sec),'f',3);//
                outstr += "\t" + ss.setNum(out.pr.b[l].r     .at(sec),'f',3);//
                outstr += "\t "+ ss.setNum(out.pr.b[l].alfar .at(sec),'f',6);//Угол атаки alpha_r, градусы
                outstr += "\t" + ss.setNum(out.pr.b[l].fi    .at(sec),'f',6);//Угол установки сечения fi, градусы
                outstr += "\t" + ss.setNum(out.pr.b[l].fl  .at(sec),'f',6);//Угол закрутки сечения , градусы
                outstr += "\t" + ss.setNum(out.pr.b[l].U     .at(sec),'f',6);//Результирующая скорость обтекания U, м/с
                outstr += "\t" + ss.setNum(out.pr.b[l].Ux    .at(sec),'f',6);//скорость обтекания Ux, м/с
                outstr += "\t" + ss.setNum(out.pr.b[l].Uy    .at(sec),'f',6);//скорость обтекания Uy, м/с
                outstr += "\t" + ss.setNum(out.pr.b[l].Fi    .at(sec),'f',6);//Угол притекания Fi, градусы
                outstr += "\t" + ss.setNum(out.pr.b[l].lambda.at(sec),'f',6);//коэф. протекания в сечении
                outstr += "\t" + ss.setNum(out.pr.b[l].M     .at(sec),'f',6);//Число Маха M
                outstr += "\t" + ss.setNum(out.pr.b[l].cy    .at(sec),'f',6);//Коэффициент подъемной силы Cy
                outstr += "\t" + ss.setNum(out.pr.b[l].cx    .at(sec),'f',6);//Коэффициент профильного сопротивления Cxp
                outstr += "\t" + ss.setNum(out.pr.b[l].mz    .at(sec),'f',6);//Коэффициент момента аэродин.сил
                outstr += "\t" + ss.setNum(out.pr.b[l].dT_dr .at(sec),'f',6);//погонная сила тяги dT/dr, [Н/м]
                outstr += "\t" + ss.setNum(out.pr.b[l].dQ_dr .at(sec),'f',6);//погонная сила сопротивления профиля dQ/dr, [Н/м]
                outstr += "\t" + ss.setNum(out.pr.b[l].dMa_dr.at(sec),'f',6);//погонный момент dMa/dr, [Нм/м]
                //outstr += "\t" + ss.setNum(out..b[l].Uywx  .at(sec),'f',6);
                //outstr += "\t" + ss.setNum(out..b[l].Uywz  .at(sec),'f',6);
            }
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

// вспомогательные функции
void MainWindow::writeReport(QString outfileName, QString outstring)//сброс в файл
{
    if(!outfileName.isEmpty())
    {
        QFile outfile(outfileName);
        outfile.open( QIODevice::WriteOnly | QIODevice::Text );
        if(outfile.isWritable())
        {
            QTextStream outstream(&outfile);//outstream.setCodec("UTF-8");//"ISO 8859-1"
            outstream << outstring;
        }
        outfile.close();
    }
}
QString MainWindow::getFolderName(QString fullname)
{
    QString fname="";
    QRegExp rx("\\/");//"\\.""\\\\"
    QStringList fields = fullname.split(rx);
    int fldsz = fields.size();
    for(int _=0; _<fldsz-1; _++)
    {
        fname += fields.at(_);
        fname += "\/";//"\\"
    }
    return fname;
}
QString MainWindow::getStateDataFolderName(QString fullname)
{
    QString fname="";
    QRegExp rx("\\.");
    QStringList fields = fullname.split(rx);
    int fldsz = fields.size();
    for(int _=0; _<fldsz-1; _++)
    {
        fname += fields.at(_);
        if(_==(fldsz-2)) fname += "\\"; else fname += ".";
    }
    return fname;
}
QString MainWindow::getFileNameOnly(QString fullname)
{
    QRegExp rx("\\/");
    QStringList fields = fullname.split(rx);
    int fldsz = fields.size();
    inifiledir="";
    for(int _=0; _<fldsz-1; _++)
    {
        inifiledir += fields.at(_);
        inifiledir += "\/";//"\\"
    }
    return fields.at(fields.size()-1);
}
QString MainWindow::strSumm(QString str1, QString str2)
{
return  str1 + str2;
}
QString MainWindow::getElapsedTime(QTime started)//
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
int MainWindow::float2int(float in)
{
    int out;
    int down = floor(in);
    out = ((in-down)>=0.5)?(down+1):(down);
return out;
}
void MainWindow::toMemo(QString string)//currentstep to memoout
{
    ui->memout->appendPlainText( string );
}
void MainWindow::toLogf(QString string)//currentstep to logfile
{
    QFile logfile(logfilename);
    logfile.open(QIODevice::Append | QIODevice::Text);
    QTextStream logstream(&logfile);//logstream.setCodec("CP1251");
    logstream << "\n";
    logstream << string;
    logfile.close();
}
void MainWindow::loadLog()
{
    QFile logfile(logfilename);
    if(logfile.exists())
    {
        ui->memout->clear();
        logfile.open( QIODevice::ReadOnly | QIODevice::Text );
        QTextStream loadlog(&logfile);
        toMemo(loadlog.readAll());
        logfile.close();
    }
    else toMemo("warning: " + logfilename + " not found");
}
bool MainWindow::find_str(QTextStream *in, QString searchString, QString &findStr )
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
bool MainWindow::find_int(QTextStream *in, QString searchString, int &findI)
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
bool MainWindow::find_bool(QTextStream *in, QString searchString, bool &findB)
{//------------------------------------------------------------------------------------------------
    in->seek(0);
    findB = 0;
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
                      findB = fields.at(i+1).toInt();
                      finded = true; isbreak=true;  break;
                }
                else  findB = 0;
            }
        }
    }
return !finded;
}
bool MainWindow::find_float(QTextStream *in, QString searchString, float &findF)
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
bool MainWindow::find_2float(QTextStream *in, QString searchString, float &findF1, float &findF2)
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
bool MainWindow::find_3float( QTextStream *in, QString searchString, float &findF1, float &findF2, float &findF3)
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
bool MainWindow::find_vectorInt(QTextStream *in, QString searchString, vector<int> &vi)
{//считывание одномерного массива
    in->seek(0);
    vi.clear();
    bool finded = false, isbreak = false;
    QString line, findS;
    QStringList fields;
    QRegExp rx("\\s");
    //QString ss;

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
                    //ncol = 0;
                    for(int j=i+1; j<fields.size(); j++)
                    {
                        vi.push_back(fields.at(j).toInt());
                      //  mass[ncol] = fields.at(j).toFloat();
                      //  ncol++;
                    }
                    finded = true; isbreak=true;  break;
                }
            }
        }
    }
return !finded;
}
bool MainWindow::find_vectorFloat(QTextStream *in, QString searchString, vector<float> &vf)
{//считывание одномерного массива
    in->seek(0);
    vf.clear();
    bool finded = false, isbreak = false;
    QString line, findS;
    QStringList fields;
    QRegExp rx("\\s");
    //QString ss;

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
                    //ncol = 0;
                    for(int j=i+1; j<fields.size(); j++)
                    {
                        vf.push_back(fields.at(j).toFloat());
                      //  mass[ncol] = fields.at(j).toFloat();
                      //  ncol++;
                    }
                    finded = true; isbreak=true;  break;
                }
            }
        }
    }
return !finded;
}
bool MainWindow::find_Xmass(QTextStream *in, QString searchString, double mass[mass1], int &ncol)
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
bool MainWindow::find_XYmass(QTextStream *in, QString searchString, double mass[mass2], int &nrow, int &ncol)
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
/*float MainWindow::inter(float x, float x1, float x2, float y1, float y2)
{
    if(x2==x1) return 0;
    else return  y1 + (x-x1)*(y2-y1)/(x2-x1);
}
*/
