//HELICOPTER.SIMULATION.INTERFACE
#include <QString>
#include <QTime>
#include <qcolor.h>
#include "mainwindow.h"
#include "mainprocess.cpp"
//-----------------------------------------------------------------------
#define applicationname "HeliSim 202307"
#define defaultinifname "D:\/helisim\/code\/test.lnk"
//-----------------------------------------------------------------------

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    titleapplname = applicationname;
    titlefilename = "";
    setWindowTitle(titleapplname);
    ui->pathEdit->setText(defaultinifname);
    singleline="------------------------------------------------------------------------------------------------------------------------";
    ui->grid_planer->clear();
    ui->progress->setValue(0);
    ui->showiter->setText("0");
    ui->showtime->setText(ss.setNum(0.0,'f',6));
    ui->EditCurrent->setText("0");
    ui->solveread->setText(" SOLVE:");
    setStopped(false);
    ui->memout->clear();
    frmcntr.clear();
    MainWindow::on_btn_controlpanel_clicked(1);
    MainWindow::on_btn_infopanel_clicked(0);
    ui->oglmemo->setText("info is available when solve is running");
    //const
    hc = 0;
    pi = M_PI;
    d2r = M_PI/180.0;
    r2d = 180.0/M_PI;

    airframeplotprep();
    shassiplotprep();
    abtestplotprep();
    rotorsplotprep();
    //
    this->logger.ConsoleInterface=false;
    this->logger.debugConsole=true;
    this->logger.ShowMessage=false;
    this->logger.ShowPlainText=false;
    this->logger.ShowTextBrowser=false;
    //this->logger.qListView=ui->listView;
    this->logger.qPlainTextEdit=ui->memo_hs;
    //this->logger.qTextBrowser=ui->textBrowser;
}
MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::on_action_quit_triggered()//exit
{
    exit(0);
}
void MainWindow::setNullInterface()//обнуление-очистка интерфейса
{
    //common
    readerror = false;
    ok = true;
    steps = -1;
    ini.start = 0;
    ini.autorot  = 0;
    ini.autogyro  = 0;
    ini.solvePlaner = 0;

    //interface
    ui->grid_planer->clear();
    ui->progress->setValue(0);
    ui->showiter->setText("0");
    ui->showtime->setText(ss.setNum(0.0,'f',6));
    ui->EditCurrent->setText("0");
    ui->solveread->setText(" SOLVE:");
    setStopped(false);
    ui->memout->clear();
    frmcntr.clear();

    out.mr.solve = 0;    out.mr.D     = 0;
    out.mr.O.x   = 0;    out.mr.O.y   = 0;    out.mr.O.z   = 0;
    out.mr.Fhx_sr= 0;    out.mr.Fhy_sr= 0;    out.mr.Fhz_sr= 0;
    out.mr.Mhx_sr= 0;    out.mr.Mhy_sr= 0;    out.mr.Mhz_sr= 0;
    out.tr.solve = 0;    out.tr.D     = 0;
    out.tr.O.x   = 0;    out.tr.O.y   = 0;    out.tr.O.z   = 0;
    out.tr.Fhx_sr= 0;    out.tr.Fhy_sr= 0;    out.tr.Fhz_sr= 0;
    out.tr.Mhx_sr= 0;    out.tr.Mhy_sr= 0;    out.tr.Mhz_sr= 0;
    out.pr.solve = 0;    out.pr.D     = 0;
    out.pr.O.x   = 0;    out.pr.O.y   = 0;    out.pr.O.z   = 0;
    out.pr.Fhx_sr= 0;    out.pr.Fhy_sr= 0;    out.pr.Fhz_sr= 0;
    out.pr.Mhx_sr= 0;    out.pr.Mhy_sr= 0;    out.pr.Mhz_sr= 0;
}

//Process/////////////////////////////////////////
void MainWindow::on_action0_triggered()
{
    startProcess(0);//расчет эквивалентных винтов
}
void MainWindow::on_action1_triggered()
{
    startProcess(1);//расчет винтов с заданным управлением по классической теории
}
void MainWindow::on_action2_triggered()
{
    startProcess(2);//расчет сил и моментов в ЦТ вертолета по заданному управлению
}
void MainWindow::on_action3_triggered()
{
    startProcess(3);//балансировка вертолета для заданной скорости
}
void MainWindow::on_action4_triggered()
{
//startProcess(4);//балансировка вертолета для диапазона скоростей
}
void MainWindow::on_action5_triggered()
{
    startProcess(5);//динамика полета и посадки вертолета
}
void MainWindow::on_actionHELICOPTER_autorotation_Vy_define_triggered()
{
    startProcess(30);//определение Vу по заданным Vx и крут.моменту на валу
}
void MainWindow::on_actionAUTOGYRO_Tpr_define_triggered()
{
    startProcess(33);//расчет Tpr (тяги пропульсивного винта) автожира по заданной горизонтальной скорости Vnx
}
void MainWindow::startProcess(int mode)
{
    ini.calcmode = mode;
    if(PreManagement()) ProcessManagement();
    /*old запуск внешних модулей
    //строка аргументов для вызова расчетных модулей
    arg.clear();
    arg << QString(yafilename);
    //аргументы для вызываемых модулей
    arg_mr = arg;    arg_mr << ss.setNum(0) ;//QString(
    arg_tr = arg;    arg_tr << ss.setNum(1) ;//QString(
    //сброс измененных исх.данных (memoin) в файл (tmpfilename)
    MemoFlush(yafilename);*/
}
void MainWindow::on_StopButton_clicked()
{
    isStop =  !isStop;
    setStopped(isStop);
    if(!isStop && (ui->solveread->text()==" SOLVE:")&&(hc!=0))
    {
        FlyingCycles();
        if(ok) loadLog();
    }
}
void MainWindow::setStopped(bool isstop)
{
    isStop = isstop;
    if(isStop) ui->StopButton->setText("CONTINUE");
    else       ui->StopButton->setText("STOP");
    QCoreApplication::processEvents();
}

//PostProcess
void MainWindow::on_actionPost_triggered()
{
    ini.calcmode = 5;//динамика полета
    if(PreReading_new()) ReadingPlotGridCycles();
    else ui->memout->appendPlainText("ошибка выполнения PreReading()");
}
bool MainWindow::PreReading_new()//ПОДГОТОВКА К СЧИТЫВАНИЮ И АНАЛИЗУ
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
        QTextStream insave(&infile);        //out.setCodec("UTF-8");
        insave << ui->memo_hs->toPlainText();//весь memini сразу  //out << tr("Значение x = ") << 12 << "\n";//поэлементно
        infile.close();
        //формируем-изменяем заголовок окна
        titlefilename = getFileNameOnly(infilename);
        setWindowTitle(titlefilename + " - " + titleapplname);
        //report.folder
        ini.reportfolder = getFolderName(infilename) + repfold;
        //формируем имя .log файла
        logfilename = ini.reportfolder + titlefilename +".log";
        //state.folder
        ini.datafolder = getFolderName(infilename) + statefold;
        if(read_ini_(infilename))//считывание сокращенного списка
        {
            //считывание параметров винтов
            if (ini.mrsolve)
            {
                read_mr(fname_mr);
                out.mr.solve = ini.mrsolve;
                out.mr.l_vsh = ini.mr.l_vsh;
            }
            if (ini.trsolve)
            {
                read_tr(fname_tr);
                out.tr.solve = ini.trsolve;
                out.tr.l_vsh = ini.tr.l_vsh;
            }
            if (ini.prsolve)
            {
                read_pr(fname_pr);
                out.pr.solve = ini.prsolve;
                out.pr.l_vsh = ini.pr.l_vsh;
            }
            if(!ini.mrsolve && !ini.trsolve && !ini.prsolve)
            {
                MainWindow::on_check_MR_2_toggled(false);
                ui->check_MR_2->setChecked(0);
            }
            //установка координат элементов планера
            out.fus.fus.x      = ini.x_f ;
            out.fus.fus.y      = ini.y_f ;
            out.fus.fus.z      = ini.z_f ;
            out.fus.stab.x     = ini.x_st;
            out.fus.stab.y     = ini.y_st;
            out.fus.stab.z     = ini.z_st;
            out.fus.keel.x     = ini.x_k ;
            out.fus.keel.y     = ini.y_k ;
            out.fus.keel.z     = ini.z_k ;
            //установка заданных управляющих сигналов
            //setManualControl(ini.tr.fi0_, ini.mr.fi0_, ini.mr.eta_, ini.mr.chi_);
            //установка номинальных целеуказателей
            setTargetControl(ini.tr.fi0_, ini.mr.fi0_, ini.mr.eta_, ini.mr.chi_);

            read_cntr(fname_cntr);
            ui->EditCurrent->setText(ss.setNum(ini.start));
            ui->solveread->setText(" READ:");
            j = ini.start;
            //
            ui->box_start->setRange(ini.start,ini.steps);
            ui->box_end  ->setRange(ini.start,ini.steps);
            ui->box_start->setValue(ini.start);
            ui->box_end  ->setValue(ini.steps);
            //передача в объект currentstep данных, считанных из файла
            if(read_helicopter_state( ini.datafolder+ss.setNum(j))) readerror=1;
            //установка активности кнопок выбора стоек
            setShassiButton(out.nst);
            //opengl-прорисовка
            if(!readerror) write_to_opengl();
            else toMemo("ошибка считывания данных из " + ini.datafolder);
        }
        else { readerror=1; toMemo("ошибка считывания исходных данных"); }
    }
    else toMemo("warning: " + infilename + " not found");
    if (readerror) return 0;  else return 1;
}
void MainWindow::ReadingPlotGridCycles()//READ & FILL PLOT DATA
{
    bool readerror=0;
    QString repoststring;
    ui->EditCurrent->setText(ss.setNum(j));
    write_to_grid_const();
    write_to_grid_gear_const();
    write_to_grid_mr_const();
    write_to_grid_tr_const();
    write_to_grid_pr_const();
    //пересчет скорости, ускорения, перегрузки в заданной точке планера вертолета (в СКС) //пересчет проводится после выполнения основного расчета!
    {
        repoststring = "\n\nPostProcess:\n";
        repoststring += "Пересчет скорости, ускорения и перегрузки в заданной точке ";//
        repoststring += "{ " + ss.setNum(ini.selpoint.x*0.001,'f',3);
        repoststring += ", " + ss.setNum(ini.selpoint.y*0.001,'f',3);
        repoststring += ", " + ss.setNum(ini.selpoint.z*0.001,'f',3);
        repoststring += " }(СКС) и положение этой точки в земной СК 'Q'\niter";
        repoststring += "\tt, c";
        repoststring += "\tQx,м ";
        repoststring += "\tQy,м ";
        repoststring += "\tQz,м ";
        repoststring += "\tvx,м/с";
        repoststring += "\tvy,м/с";
        repoststring += "\tvz,м/с";
        repoststring += "\tax,м/с2";
        repoststring += "\tay,м/с2";
        repoststring += "\taz,м/с2";
        repoststring += "\tnx, - ";
        repoststring += "\tny, - ";
        repoststring += "\tnz, - ";
        repoststring += "\n";
    }

    //CLEARING
    {
        it.clear();
        //helicopter.clear
        h_org_x   .clear();    h_org_y   .clear();    h_org_z   .clear();
        h_org_k   .clear();    h_org_r   .clear();    h_org_t   .clear();
        h_vn_x    .clear();    h_vn_y    .clear();    h_vn_z    .clear();
        h_v_x     .clear();    h_v_y     .clear();    h_v_z     .clear();
        h_w_x     .clear();    h_w_y     .clear();    h_w_z     .clear();
        h_a_x     .clear();    h_a_y     .clear();    h_a_z     .clear();
        h_Msum_x  .clear();    h_Msum_y  .clear();    h_Msum_z  .clear();
        h_Fsum_x  .clear();    h_Fsum_y  .clear();    h_Fsum_z  .clear();
        h_n_x     .clear();    h_n_y     .clear();    h_n_z     .clear();
        h_Fadp_x  .clear();    h_Fadp_y  .clear();    h_Fadp_z  .clear();
        h_Madp_x  .clear();    h_Madp_y  .clear();    h_Madp_z  .clear();
        h_Ffus_x  .clear();    h_Ffus_y  .clear();    h_Ffus_z  .clear();
        h_Mfus_x  .clear();    h_Mfus_y  .clear();    h_Mfus_z  .clear();
        h_Fstab_x .clear();    h_Fstab_y .clear();    h_Fstab_z .clear();
        h_Fkeel_x .clear();    h_Fkeel_y .clear();    h_Fkeel_z .clear();
        //shassi.clear
        for (int k = 0; k < mst; k++)
        {
               s_ha[k].clear();      s_ha1[k].clear();      s_ha2[k].clear();
               s_Pa[k].clear();      s_Pa1[k].clear();      s_Pa2[k].clear();
              s_Pap[k].clear();
             s_hp_x[k].clear();     s_hp_y[k].clear();     s_hp_z[k].clear();
            s_hp1_x[k].clear();    s_hp1_y[k].clear();    s_hp1_z[k].clear();
            s_hp2_x[k].clear();    s_hp2_y[k].clear();    s_hp2_z[k].clear();
             s_Pp_x[k].clear();     s_Pp_y[k].clear();     s_Pp_z[k].clear();
            s_Pp1_x[k].clear();    s_Pp1_y[k].clear();    s_Pp1_z[k].clear();
            s_Pp2_x[k].clear();    s_Pp2_y[k].clear();    s_Pp2_z[k].clear();
        }
        //mainrotor.clear
        mr_psi0   .clear();    mr_alfa   .clear();
        mr_fi0    .clear();    mr_chi    .clear();    mr_eta    .clear();
        mr_a_x    .clear();    mr_a_y    .clear();    mr_a_z    .clear();
        mr_v_x    .clear();    mr_v_y    .clear();    mr_v_z    .clear();
        mr_dct    .clear();    mr_dct_x  .clear();    mr_dct_z  .clear();
        mr_H_sr   .clear();    mr_T_sr   .clear();    mr_S_sr   .clear();
        mr_Mx_sr  .clear();    mr_My_sr  .clear();    mr_Mz_sr  .clear();
        mr_Fhx_sum.clear();    mr_Fhy_sum.clear();    mr_Fhz_sum.clear();
        mr_Mhx_sum.clear();    mr_Mhy_sum.clear();    mr_Mhz_sum.clear();
        for (int l = 0; l < mbl; l++)
        {

            mr_psi[l] .clear();        mr_fi7[l]  .clear();        mr_dfi7[l] .clear();
            mr_gama[l].clear();        mr_dgama[l].clear();        mr_d2gama[l].clear();
            mr_beta[l].clear();        mr_dbeta[l].clear();        mr_d2beta[l].clear();
            mr_ksi[l] .clear();        mr_dksi[l] .clear();        mr_d2ksi[l] .clear();
            mr_T1[l]  .clear();        mr_Q1[l]   .clear();        mr_Ma1[l] .clear();
            mr_H1[l]  .clear();        mr_S1[l]   .clear();
            mr_Nr[l]  .clear();        mr_Nf[l]   .clear();
            mr_P1[l]  .clear();        mr_P2[l]   .clear();
            mr_K1[l]  .clear();        mr_Fdmp[l] .clear();
            mr_Fx[l]  .clear();        mr_Fy[l]   .clear();        mr_Fz[l] .clear();
            mr_M1T[l] .clear();        mr_M1Q[l]  .clear();
            mr_M1Gx[l].clear();        mr_M1Gy[l] .clear();
            mr_M1K[l] .clear();
            mr_M1Cx[l].clear();        mr_M1Cy[l] .clear();
            mr_M1Px[l].clear();        mr_M1Py[l] .clear();
            mr_M1x[l] .clear();        mr_M1y[l]  .clear();        mr_M1z[l].clear();
            mr_M2T[l] .clear();        mr_M2Q[l]  .clear();
            mr_M2Gx[l].clear();        mr_M2Gy[l] .clear();
            mr_M2K[l] .clear();
            mr_M2Cx[l].clear();        mr_M2Cy[l] .clear();
            mr_M2Px[l].clear();        mr_M2Py[l] .clear();
            mr_M2D[l] .clear();
            mr_M2x[l] .clear();        mr_M2y[l]  .clear();        mr_M2z[l].clear();
            mr_M3x[l] .clear();        mr_M3y[l]  .clear();        mr_M3z[l].clear();
            mr_M3Ma[l].clear();/// = mr_Ma1
            mr_M3su[l].clear();        mr_Fpov[l] .clear();
            mr_M3cb[l].clear();
            mr_M3tr[l].clear();
            mr_M3i[l] .clear();
            mr_Fxh[l] .clear();        mr_Fyh[l]  .clear();        mr_Fzh[l].clear();
            mr_Mxh[l] .clear();        mr_Myh[l]  .clear();        mr_Mzh[l].clear();
        }
        //tailrotor.clear
        tr_fi0.clear();
        tr_H_sr   .clear();    tr_T_sr   .clear();    tr_S_sr   .clear();
        tr_Mx_sr  .clear();    tr_My_sr  .clear();    tr_Mz_sr  .clear();
        tr_Fhx_sum.clear();    tr_Fhy_sum.clear();    tr_Fhz_sum.clear();
        tr_Mhx_sum.clear();    tr_Mhy_sum.clear();    tr_Mhz_sum.clear();
        tr_psi0   .clear();
        for (int l = 0; l < mbl; l++)
        {
            tr_psi[l] .clear();        tr_fi7[l]  .clear();        tr_dfi7[l] .clear();
            tr_gama[l].clear();        tr_dgama[l].clear();        tr_d2gama[l].clear();
            tr_beta[l].clear();        tr_dbeta[l].clear();        tr_d2beta[l].clear();
            tr_ksi[l] .clear();        tr_dksi[l] .clear();        tr_d2ksi[l] .clear();
            tr_T1[l]  .clear();        tr_Q1[l]   .clear();        tr_Ma1[l] .clear();
            tr_H1[l]  .clear();        tr_S1[l]   .clear();
            tr_Nr[l]  .clear();        tr_Nf[l]   .clear();
            tr_P1[l]  .clear();        tr_P2[l]   .clear();
            tr_K1[l]  .clear();        tr_Fdmp[l] .clear();
            tr_Fx[l]  .clear();        tr_Fy[l]   .clear();        tr_Fz[l] .clear();
            tr_M1T[l] .clear();        tr_M1Q[l]  .clear();
            tr_M1Gx[l].clear();        tr_M1Gy[l] .clear();
            tr_M1K[l] .clear();
            tr_M1Cx[l].clear();        tr_M1Cy[l] .clear();
            tr_M1Px[l].clear();        tr_M1Py[l] .clear();
            tr_M1x[l] .clear();        tr_M1y[l]  .clear();        tr_M1z[l].clear();
            tr_M2T[l] .clear();        tr_M2Q[l]  .clear();
            tr_M2Gx[l].clear();        tr_M2Gy[l] .clear();
            tr_M2K[l] .clear();
            tr_M2Cx[l].clear();        tr_M2Cy[l] .clear();
            tr_M2Px[l].clear();        tr_M2Py[l] .clear();
            tr_M2D[l] .clear();
            tr_M2x[l] .clear();        tr_M2y[l]  .clear();        tr_M2z[l].clear();
            tr_M3x[l] .clear();        tr_M3y[l]  .clear();        tr_M3z[l].clear();
            tr_M3Ma[l].clear();
            tr_M3su[l].clear();        tr_Fpov[l] .clear();
            tr_M3cb[l].clear();
            tr_M3tr[l].clear();
            tr_M3i[l] .clear();
            tr_Fxh[l] .clear();        tr_Fyh[l]  .clear();        tr_Fzh[l].clear();
            tr_Mxh[l] .clear();        tr_Myh[l]  .clear();        tr_Mzh[l].clear();
        }
        //pushrotor.clear
        pr_fi0.clear();
        pr_H_sr   .clear();    pr_T_sr   .clear();    pr_S_sr   .clear();
        pr_Mx_sr  .clear();    pr_My_sr  .clear();    pr_Mz_sr  .clear();
        pr_Fhx_sum.clear();    pr_Fhy_sum.clear();    pr_Fhz_sum.clear();
        pr_Mhx_sum.clear();    pr_Mhy_sum.clear();    pr_Mhz_sum.clear();
        pr_psi0   .clear();
        for (int l = 0; l < mbl; l++)
        {
            pr_psi[l] .clear();        pr_fi7[l]  .clear();        pr_dfi7[l] .clear();
            pr_gama[l].clear();        pr_dgama[l].clear();        pr_d2gama[l].clear();
            pr_beta[l].clear();        pr_dbeta[l].clear();        pr_d2beta[l].clear();
            pr_ksi[l] .clear();        pr_dksi[l] .clear();        pr_d2ksi[l] .clear();
            pr_T1[l]  .clear();        pr_Q1[l]   .clear();        pr_Ma1[l] .clear();
            pr_H1[l]  .clear();        pr_S1[l]   .clear();
            pr_Nr[l]  .clear();        pr_Nf[l]   .clear();
            pr_P1[l]  .clear();        pr_P2[l]   .clear();
            pr_K1[l]  .clear();        pr_Fdmp[l] .clear();
            pr_Fx[l]  .clear();        pr_Fy[l]   .clear();        pr_Fz[l] .clear();
            pr_M1T[l] .clear();        pr_M1Q[l]  .clear();
            pr_M1Gx[l].clear();        pr_M1Gy[l] .clear();
            pr_M1K[l] .clear();
            pr_M1Cx[l].clear();        pr_M1Cy[l] .clear();
            pr_M1Px[l].clear();        pr_M1Py[l] .clear();
            pr_M1x[l] .clear();        pr_M1y[l]  .clear();        pr_M1z[l].clear();
            pr_M2T[l] .clear();        pr_M2Q[l]  .clear();
            pr_M2Gx[l].clear();        pr_M2Gy[l] .clear();
            pr_M2K[l] .clear();
            pr_M2Cx[l].clear();        pr_M2Cy[l] .clear();
            pr_M2Px[l].clear();        pr_M2Py[l] .clear();
            pr_M2D[l] .clear();
            pr_M2x[l] .clear();        pr_M2y[l]  .clear();        pr_M2z[l].clear();
            pr_M3x[l] .clear();        pr_M3y[l]  .clear();        pr_M3z[l].clear();
            pr_M3Ma[l].clear();
            pr_M3su[l].clear();        pr_Fpov[l] .clear();
            pr_M3cb[l].clear();
            pr_M3tr[l].clear();
            pr_M3i[l] .clear();
            pr_Fxh[l] .clear();        pr_Fyh[l]  .clear();        pr_Fzh[l].clear();
            pr_Mxh[l] .clear();        pr_Myh[l]  .clear();        pr_Mzh[l].clear();
        }
    }

    //READ & FILL
    while( !readerror && j<=ini.steps && !isStop )
    {
        readerror = read_helicopter_state( ini.datafolder + ss.setNum(j) );
        if(!readerror)
        {
            it.push_back(out.t);
            /// пересчет параметров для заданной точки планера
            {
                RecalcSelected(ini.selpoint.x*0.001, ini.selpoint.y*0.001, ini.selpoint.z*0.001);

                repoststring += ss.setNum(out.iter         ) + "\t";
                repoststring += ss.setNum(out.t      ,'f',4) + "\t";
                repoststring += ss.setNum(out.fus.selpoint_Q.x,'f',3) + "\t";
                repoststring += ss.setNum(out.fus.selpoint_Q.y,'f',3) + "\t";
                repoststring += ss.setNum(out.fus.selpoint_Q.z,'f',3) + "\t";
                repoststring += ss.setNum(out.fus.selpoint_v.x,'f',3) + "\t";
                repoststring += ss.setNum(out.fus.selpoint_v.y,'f',3) + "\t";
                repoststring += ss.setNum(out.fus.selpoint_v.z,'f',3) + "\t";
                repoststring += ss.setNum(out.fus.selpoint_a.x,'f',3) + "\t";
                repoststring += ss.setNum(out.fus.selpoint_a.y,'f',3) + "\t";
                repoststring += ss.setNum(out.fus.selpoint_a.z,'f',3) + "\t";
                repoststring += ss.setNum(out.fus.selpoint_n.x,'f',3) + "\t";
                repoststring += ss.setNum(out.fus.selpoint_n.y,'f',3) + "\t";
                repoststring += ss.setNum(out.fus.selpoint_n.z,'f',3) + "\n";
            }

            /// Helicopter /////////////////////////////////////////////////////
            {
            h_org_x     .push_back( out.fus.org.x    );
            h_org_y     .push_back( out.fus.org.y    );
            h_org_z     .push_back( out.fus.org.z    );
            h_org_k     .push_back( out.fus.org.k    );
            h_org_r     .push_back( out.fus.org.r    );
            h_org_t     .push_back( out.fus.org.t    );
            h_vn_x      .push_back( out.fus.vn.x     );
            h_vn_y      .push_back( out.fus.vn.y     );
            h_vn_z      .push_back( out.fus.vn.z     );
            h_v_x       .push_back( out.fus.v.x      );
            h_v_y       .push_back( out.fus.v.y      );
            h_v_z       .push_back( out.fus.v.z      );
            h_w_x       .push_back( out.fus.w.x      );
            h_w_y       .push_back( out.fus.w.y      );
            h_w_z       .push_back( out.fus.w.z      );
            h_a_x       .push_back( out.fus.a.x      );
            h_a_y       .push_back( out.fus.a.y      );
            h_a_z       .push_back( out.fus.a.z      );
            h_n_x       .push_back( out.fus.n.x      );
            h_n_y       .push_back( out.fus.n.y      );
            h_n_z       .push_back( out.fus.n.z      );
            h_Fsum_x    .push_back( out.fus.Fsum.x   );
            h_Fsum_y    .push_back( out.fus.Fsum.y   );
            h_Fsum_z    .push_back( out.fus.Fsum.z   );
            h_Msum_x    .push_back( out.fus.Msum.x   );
            h_Msum_y    .push_back( out.fus.Msum.y   );
            h_Msum_z    .push_back( out.fus.Msum.z   );
            h_Fadp_x    .push_back( out.fus.Fadp.x   );
            h_Fadp_y    .push_back( out.fus.Fadp.y   );
            h_Fadp_z    .push_back( out.fus.Fadp.z   );
            h_Madp_x    .push_back( out.fus.Madp.x   );
            h_Madp_y    .push_back( out.fus.Madp.y   );
            h_Madp_z    .push_back( out.fus.Madp.z   );
            h_Ffus_x    .push_back( out.fus.fus_F.x  );
            h_Ffus_y    .push_back( out.fus.fus_F.y  );
            h_Ffus_z    .push_back( out.fus.fus_F.z  );
            h_Mfus_x    .push_back( out.fus.fus_M.x  );
            h_Mfus_y    .push_back( out.fus.fus_M.y  );
            h_Mfus_z    .push_back( out.fus.fus_M.z  );
            h_Fstab_x   .push_back( out.fus.stab_F.x );
            h_Fstab_y   .push_back( out.fus.stab_F.y );
            h_Fstab_z   .push_back( out.fus.stab_F.z );
            h_Fkeel_x   .push_back( out.fus.keel_F.x );
            h_Fkeel_y   .push_back( out.fus.keel_F.y );
            h_Fkeel_z   .push_back( out.fus.keel_F.z );
            }

            /// MainRotor //////////////////////////////////////////////////////
            {
            mr_fi0      .push_back( out.mr.fi0     );
            mr_chi      .push_back( out.mr.chi     );
            mr_eta      .push_back( out.mr.eta     );
            mr_a_x      .push_back( out.mr.ah.x    );
            mr_a_y      .push_back( out.mr.ah.y    );
            mr_a_z      .push_back( out.mr.ah.z    );
            mr_v_x      .push_back( out.mr.vh.x    );
            mr_v_y      .push_back( out.mr.vh.y    );
            mr_v_z      .push_back( out.mr.vh.z    );
            mr_dct      .push_back( out.mr.dctbs   );
            mr_dct_x    .push_back( out.mr.dct.x   );
            mr_dct_z    .push_back( out.mr.dct.z   );
            mr_psi0     .push_back( out.mr.psih    );
            mr_alfa     .push_back( out.mr.alfa    );
            mr_H_sr     .push_back( out.mr.Fhx_sr  );
            mr_T_sr     .push_back( out.mr.Fhy_sr  );
            mr_S_sr     .push_back( out.mr.Fhz_sr  );
            mr_Mx_sr    .push_back( out.mr.Mhx_sr  );
            mr_My_sr    .push_back( out.mr.Mhy_sr  );
            mr_Mz_sr    .push_back( out.mr.Mhz_sr  );
            mr_Fhx_sum  .push_back( out.mr.Fhx_sum );
            mr_Fhy_sum  .push_back( out.mr.Fhy_sum );
            mr_Fhz_sum  .push_back( out.mr.Fhz_sum );
            mr_Mhx_sum  .push_back( out.mr.Mhx_sum );
            mr_Mhy_sum  .push_back( out.mr.Mhy_sum );
            mr_Mhz_sum  .push_back( out.mr.Mhz_sum );
            // MainRotor's Blades //////////////////
            for (int l = 0; l < out.mr.nb; l++)
            {

                mr_psi[l]       .push_back( out.mr.b[l].psi   );
                mr_fi7[l]       .push_back( out.mr.b[l].fi7   );
                mr_dfi7[l]      .push_back( out.mr.b[l].dfi7  );
                mr_gama[l]      .push_back( out.mr.b[l].gama  );
                mr_dgama[l]     .push_back( out.mr.b[l].dgama );
                mr_d2gama[l]    .push_back( out.mr.b[l].d2gama);
                mr_beta[l]      .push_back( out.mr.b[l].beta  );
                mr_dbeta[l]     .push_back( out.mr.b[l].dbeta );
                mr_d2beta[l]    .push_back( out.mr.b[l].d2beta);
                mr_ksi[l]       .push_back( out.mr.b[l].ksi   );
                mr_dksi[l]      .push_back( out.mr.b[l].dksi  );
                mr_d2ksi[l]     .push_back( out.mr.b[l].d2ksi );
                mr_Ma1[l]       .push_back( out.mr.b[l].Ma    );
                mr_T1[l]        .push_back( out.mr.b[l].T     );
                mr_Q1[l]        .push_back( out.mr.b[l].Q     );
                mr_H1[l]        .push_back( out.mr.b[l].H     );
                mr_S1[l]        .push_back( out.mr.b[l].S     );
                mr_Nr[l]        .push_back( out.mr.b[l].Nr    );
                mr_Nf[l]        .push_back( out.mr.b[l].Nf    );
                mr_P1[l]        .push_back( out.mr.b[l].P1    );
                mr_P2[l]        .push_back( out.mr.b[l].P2    );
                mr_K1[l]        .push_back( out.mr.b[l].K1    );
                mr_Fdmp[l]      .push_back( out.mr.b[l].Fdmp  );
                mr_Fpov[l]      .push_back( out.mr.b[l].Fpov  );
                mr_Fx[l]        .push_back( out.mr.b[l].Fx    );//Fx
                mr_Fy[l]        .push_back( out.mr.b[l].Fy    );//Fy
                mr_Fz[l]        .push_back( out.mr.b[l].Fz    );//F12z
                mr_M1T[l]       .push_back( out.mr.b[l].M1T   );
                mr_M1Q[l]       .push_back( out.mr.b[l].M1Q   );
                mr_M1Gx[l]      .push_back( out.mr.b[l].M1Gx  );
                mr_M1Gy[l]      .push_back( out.mr.b[l].M1Gy  );
                mr_M1K[l]       .push_back( out.mr.b[l].M1K   );
                mr_M1Cx[l]      .push_back( out.mr.b[l].M1Cx  );
                mr_M1Cy[l]      .push_back( out.mr.b[l].M1Cy  );
                mr_M1Px[l]      .push_back( out.mr.b[l].M1Px  );
                mr_M1Py[l]      .push_back( out.mr.b[l].M1Py  );
                mr_M1x[l]       .push_back( out.mr.b[l].M1x   );
                mr_M1y[l]       .push_back( out.mr.b[l].M1y   );
                mr_M1z[l]       .push_back( out.mr.b[l].M1z   );
                mr_M2T[l]       .push_back( out.mr.b[l].M2T   );
                mr_M2Q[l]       .push_back( out.mr.b[l].M2Q   );
                mr_M2Gx[l]      .push_back( out.mr.b[l].M2Gx  );
                mr_M2Gy[l]      .push_back( out.mr.b[l].M2Gy  );
                mr_M2K[l]       .push_back( out.mr.b[l].M2K   );
                mr_M2Cx[l]      .push_back( out.mr.b[l].M2Cx  );
                mr_M2Cy[l]      .push_back( out.mr.b[l].M2Cy  );
                mr_M2Px[l]      .push_back( out.mr.b[l].M2Px  );
                mr_M2Py[l]      .push_back( out.mr.b[l].M2Py  );
                mr_M2D[l]       .push_back( out.mr.b[l].M2D   );
                mr_M2x[l]       .push_back( out.mr.b[l].M2x   );
                mr_M2y[l]       .push_back( out.mr.b[l].M2y   );
                mr_M2z[l]       .push_back( out.mr.b[l].M2z   );
                mr_M3Ma[l]      .push_back( out.mr.b[l].M3Ma  );
                mr_M3su[l]      .push_back( out.mr.b[l].M3su  );
                mr_M3cb[l]      .push_back( out.mr.b[l].M3Cz  );
                mr_M3tr[l]      .push_back( out.mr.b[l].M3tr  );
                mr_M3i[l]       .push_back( out.mr.b[l].M3iz  );
                mr_M3x[l]       .push_back( out.mr.b[l].M3x   );
                mr_M3y[l]       .push_back( out.mr.b[l].M3y   );
                mr_M3z[l]       .push_back( out.mr.b[l].M3z   );
                mr_Fxh[l]       .push_back( out.mr.b[l].Fhx   );
                mr_Fyh[l]       .push_back( out.mr.b[l].Fhy   );
                mr_Fzh[l]       .push_back( out.mr.b[l].Fhz   );
                mr_Mxh[l]       .push_back( out.mr.b[l].Mhx   );
                mr_Myh[l]       .push_back( out.mr.b[l].Mhy   );
                mr_Mzh[l]       .push_back( out.mr.b[l].Mhz   );
            }
            /* виброгаситель
                mr_dfi.push_back(out.mr.dfi   );//закручивание вала без ВГ
               mr_dfig.push_back(out.mr.dfig  );//закручивание вала с ВГ
                 mr_Mv.push_back(out.mr.Mv    );//момент на валу без ВГ
                mr_Mvg.push_back(out.mr.Mvg   );//момент на валу с ВГ
                vg_psi.push_back(out.mr.vg.psi);
                vg_eps.push_back(out.mr.vg.eps);
                 vg_Mn.push_back(out.mr.vg.Mn );
                 vg_Mt.push_back(out.mr.vg.Mt );
                 vg_Mk.push_back(out.mr.vg.Mk );
                 vg_Mi.push_back(out.mr.vg.Mi );*/
            }

            /// TailRotor //////////////////////////////////////////////////////
            {
            tr_fi0      .push_back( out.tr.fi0      );
            tr_psi0     .push_back( out.tr.psih     );
            tr_H_sr     .push_back( out.tr.Fhx_sr   );
            tr_T_sr     .push_back( out.tr.Fhy_sr   );
            tr_S_sr     .push_back( out.tr.Fhz_sr   );
            tr_Mx_sr    .push_back( out.tr.Mhx_sr   );
            tr_My_sr    .push_back( out.tr.Mhy_sr   );
            tr_Mz_sr    .push_back( out.tr.Mhz_sr   );
            tr_Fhx_sum  .push_back( out.tr.Fhx_sum  );
            tr_Fhy_sum  .push_back( out.tr.Fhy_sum  );
            tr_Fhz_sum  .push_back( out.tr.Fhz_sum  );
            tr_Mhx_sum  .push_back( out.tr.Mhx_sum  );
            tr_Mhy_sum  .push_back( out.tr.Mhy_sum  );
            tr_Mhz_sum  .push_back( out.tr.Mhz_sum  );
            // TailRotor's Blades //////////////////////////////////////////////
            for (int l = 0; l < out.tr.nb; l++)
            {
                tr_psi[l]       .push_back( out.tr.b[l].psi   );
                tr_fi7[l]       .push_back( out.tr.b[l].fi7   );
                tr_dfi7[l]      .push_back( out.tr.b[l].dfi7  );
                tr_gama[l]      .push_back( out.tr.b[l].gama  );
                tr_dgama[l]     .push_back( out.tr.b[l].dgama );
                tr_d2gama[l]    .push_back( out.tr.b[l].d2gama);
                tr_beta[l]      .push_back( out.tr.b[l].beta  );
                tr_dbeta[l]     .push_back( out.tr.b[l].dbeta );
                tr_d2beta[l]    .push_back( out.tr.b[l].d2beta);
                tr_ksi[l]       .push_back( out.tr.b[l].ksi   );
                tr_dksi[l]      .push_back( out.tr.b[l].dksi  );
                tr_d2ksi[l]     .push_back( out.tr.b[l].d2ksi );
                tr_Ma1[l]       .push_back( out.tr.b[l].Ma    );
                tr_T1[l]        .push_back( out.tr.b[l].T     );
                tr_Q1[l]        .push_back( out.tr.b[l].Q     );
                tr_H1[l]        .push_back( out.tr.b[l].H     );
                tr_S1[l]        .push_back( out.tr.b[l].S     );
                tr_Nr[l]        .push_back( out.tr.b[l].Nr    );
                tr_Nf[l]        .push_back( out.tr.b[l].Nf    );
                tr_P1[l]        .push_back( out.tr.b[l].P1    );
                tr_P2[l]        .push_back( out.tr.b[l].P2    );
                tr_K1[l]        .push_back( out.tr.b[l].K1    );
                tr_Fdmp[l]      .push_back( out.tr.b[l].Fdmp  );
                tr_Fpov[l]      .push_back( out.tr.b[l].Fpov  );
                tr_Fx[l]        .push_back( out.tr.b[l].Fx    );//Fx
                tr_Fy[l]        .push_back( out.tr.b[l].Fy    );//Fy
                tr_Fz[l]        .push_back( out.tr.b[l].Fz    );//F12z
                tr_M1T[l]       .push_back( out.tr.b[l].M1T   );
                tr_M1Q[l]       .push_back( out.tr.b[l].M1Q   );
                tr_M1Gx[l]      .push_back( out.tr.b[l].M1Gx  );
                tr_M1Gy[l]      .push_back( out.tr.b[l].M1Gy  );
                tr_M1K[l]       .push_back( out.tr.b[l].M1K   );
                tr_M1Cx[l]      .push_back( out.tr.b[l].M1Cx  );
                tr_M1Cy[l]      .push_back( out.tr.b[l].M1Cy  );
                tr_M1Px[l]      .push_back( out.tr.b[l].M1Px  );
                tr_M1Py[l]      .push_back( out.tr.b[l].M1Py  );
                tr_M1x[l]       .push_back( out.tr.b[l].M1x   );
                tr_M1y[l]       .push_back( out.tr.b[l].M1y   );
                tr_M1z[l]       .push_back( out.tr.b[l].M1z   );
                tr_M2T[l]       .push_back( out.tr.b[l].M2T   );
                tr_M2Q[l]       .push_back( out.tr.b[l].M2Q   );
                tr_M2Gx[l]      .push_back( out.tr.b[l].M2Gx  );
                tr_M2Gy[l]      .push_back( out.tr.b[l].M2Gy  );
                tr_M2K[l]       .push_back( out.tr.b[l].M2K   );
                tr_M2Cx[l]      .push_back( out.tr.b[l].M2Cx  );
                tr_M2Cy[l]      .push_back( out.tr.b[l].M2Cy  );
                tr_M2Px[l]      .push_back( out.tr.b[l].M2Px  );
                tr_M2Py[l]      .push_back( out.tr.b[l].M2Py  );
                tr_M2D[l]       .push_back( out.tr.b[l].M2D   );
                tr_M2x[l]       .push_back( out.tr.b[l].M2x   );
                tr_M2y[l]       .push_back( out.tr.b[l].M2y   );
                tr_M2z[l]       .push_back( out.tr.b[l].M2z   );
                tr_M3Ma[l]      .push_back( out.tr.b[l].M3Ma  );
                tr_M3su[l]      .push_back( out.tr.b[l].M3su  );
                tr_M3cb[l]      .push_back( out.tr.b[l].M3Cz  );
                tr_M3tr[l]      .push_back( out.tr.b[l].M3tr  );
                tr_M3i[l]       .push_back( out.tr.b[l].M3iz  );
                tr_M3x[l]       .push_back( out.tr.b[l].M3x   );
                tr_M3y[l]       .push_back( out.tr.b[l].M3y   );
                tr_M3z[l]       .push_back( out.tr.b[l].M3z   );
                tr_Fxh[l]       .push_back( out.tr.b[l].Fhx   );
                tr_Fyh[l]       .push_back( out.tr.b[l].Fhy   );
                tr_Fzh[l]       .push_back( out.tr.b[l].Fhz   );
                tr_Mxh[l]       .push_back( out.tr.b[l].Mhx   );
                tr_Myh[l]       .push_back( out.tr.b[l].Mhy   );
                tr_Mzh[l]       .push_back( out.tr.b[l].Mhz   );
            }
            }

            /// PushRotor //////////////////////////////////////////////////////
            {
            pr_fi0      .push_back( out.pr.fi0      );
            pr_psi0     .push_back( out.pr.psih     );
            pr_H_sr     .push_back( out.pr.Fhx_sr   );
            pr_T_sr     .push_back( out.pr.Fhy_sr   );
            pr_S_sr     .push_back( out.pr.Fhz_sr   );
            pr_Mx_sr    .push_back( out.pr.Mhx_sr   );
            pr_My_sr    .push_back( out.pr.Mhy_sr   );
            pr_Mz_sr    .push_back( out.pr.Mhz_sr   );
            pr_Fhx_sum  .push_back( out.pr.Fhx_sum  );
            pr_Fhy_sum  .push_back( out.pr.Fhy_sum  );
            pr_Fhz_sum  .push_back( out.pr.Fhz_sum  );
            pr_Mhx_sum  .push_back( out.pr.Mhx_sum  );
            pr_Mhy_sum  .push_back( out.pr.Mhy_sum  );
            pr_Mhz_sum  .push_back( out.pr.Mhz_sum  );
            // PushRotor's Blades //////////////////////////////////////////////
            for (int l = 0; l < out.pr.nb; l++)
            {
                pr_psi[l]       .push_back( out.pr.b[l].psi   );
                pr_fi7[l]       .push_back( out.pr.b[l].fi7   );
                pr_dfi7[l]      .push_back( out.pr.b[l].dfi7  );
                pr_gama[l]      .push_back( out.pr.b[l].gama  );
                pr_dgama[l]     .push_back( out.pr.b[l].dgama );
                pr_d2gama[l]    .push_back( out.pr.b[l].d2gama);
                pr_beta[l]      .push_back( out.pr.b[l].beta  );
                pr_dbeta[l]     .push_back( out.pr.b[l].dbeta );
                pr_d2beta[l]    .push_back( out.pr.b[l].d2beta);
                pr_ksi[l]       .push_back( out.pr.b[l].ksi   );
                pr_dksi[l]      .push_back( out.pr.b[l].dksi  );
                pr_d2ksi[l]     .push_back( out.pr.b[l].d2ksi );
                pr_Ma1[l]       .push_back( out.pr.b[l].Ma    );
                pr_T1[l]        .push_back( out.pr.b[l].T     );
                pr_Q1[l]        .push_back( out.pr.b[l].Q     );
                pr_H1[l]        .push_back( out.pr.b[l].H     );
                pr_S1[l]        .push_back( out.pr.b[l].S     );
                pr_Nr[l]        .push_back( out.pr.b[l].Nr    );
                pr_Nf[l]        .push_back( out.pr.b[l].Nf    );
                pr_P1[l]        .push_back( out.pr.b[l].P1    );
                pr_P2[l]        .push_back( out.pr.b[l].P2    );
                pr_K1[l]        .push_back( out.pr.b[l].K1    );
                pr_Fdmp[l]      .push_back( out.pr.b[l].Fdmp  );
                pr_Fpov[l]      .push_back( out.pr.b[l].Fpov  );
                pr_Fx[l]        .push_back( out.pr.b[l].Fx    );
                pr_Fy[l]        .push_back( out.pr.b[l].Fy    );
                pr_Fz[l]        .push_back( out.pr.b[l].Fz    );
                pr_M1T[l]       .push_back( out.pr.b[l].M1T   );
                pr_M1Q[l]       .push_back( out.pr.b[l].M1Q   );
                pr_M1Gx[l]      .push_back( out.pr.b[l].M1Gx  );
                pr_M1Gy[l]      .push_back( out.pr.b[l].M1Gy  );
                pr_M1K[l]       .push_back( out.pr.b[l].M1K   );
                pr_M1Cx[l]      .push_back( out.pr.b[l].M1Cx  );
                pr_M1Cy[l]      .push_back( out.pr.b[l].M1Cy  );
                pr_M1Px[l]      .push_back( out.pr.b[l].M1Px  );
                pr_M1Py[l]      .push_back( out.pr.b[l].M1Py  );
                pr_M1x[l]       .push_back( out.pr.b[l].M1x   );
                pr_M1y[l]       .push_back( out.pr.b[l].M1y   );
                pr_M1z[l]       .push_back( out.pr.b[l].M1z   );
                pr_M2T[l]       .push_back( out.pr.b[l].M2T   );
                pr_M2Q[l]       .push_back( out.pr.b[l].M2Q   );
                pr_M2Gx[l]      .push_back( out.pr.b[l].M2Gx  );
                pr_M2Gy[l]      .push_back( out.pr.b[l].M2Gy  );
                pr_M2K[l]       .push_back( out.pr.b[l].M2K   );
                pr_M2Cx[l]      .push_back( out.pr.b[l].M2Cx  );
                pr_M2Cy[l]      .push_back( out.pr.b[l].M2Cy  );
                pr_M2Px[l]      .push_back( out.pr.b[l].M2Px  );
                pr_M2Py[l]      .push_back( out.pr.b[l].M2Py  );
                pr_M2D[l]       .push_back( out.pr.b[l].M2D   );
                pr_M2x[l]       .push_back( out.pr.b[l].M2x   );
                pr_M2y[l]       .push_back( out.pr.b[l].M2y   );
                pr_M2z[l]       .push_back( out.pr.b[l].M2z   );
                pr_M3Ma[l]      .push_back( out.pr.b[l].M3Ma  );
                pr_M3su[l]      .push_back( out.pr.b[l].M3su  );
                pr_M3cb[l]      .push_back( out.pr.b[l].M3Cz  );
                pr_M3tr[l]      .push_back( out.pr.b[l].M3tr  );
                pr_M3i[l]       .push_back( out.pr.b[l].M3iz  );
                pr_M3x[l]       .push_back( out.pr.b[l].M3x   );
                pr_M3y[l]       .push_back( out.pr.b[l].M3y   );
                pr_M3z[l]       .push_back( out.pr.b[l].M3z   );
                pr_Fxh[l]       .push_back( out.pr.b[l].Fhx   );
                pr_Fyh[l]       .push_back( out.pr.b[l].Fhy   );
                pr_Fzh[l]       .push_back( out.pr.b[l].Fhz   );
                pr_Mxh[l]       .push_back( out.pr.b[l].Mhx   );
                pr_Myh[l]       .push_back( out.pr.b[l].Mhy   );
                pr_Mzh[l]       .push_back( out.pr.b[l].Mhz   );
            }
            }

            /// Shassi  ////////////////////////////////////////////////////////
            if(out.nst) for (int k = 0; k < out.nst; k++)
            {
                s_ha[k]     .push_back(out.st[k].a.ha        );//обжатие суммарное
                s_ha1[k]    .push_back(out.st[k].a.ha1       );//обжатие камеры 1
                s_ha2[k]    .push_back(out.st[k].a.ha2       );//обжатие камеры 2
                s_Pa[k]     .push_back(out.st[k].a.Pa  *0.001);//усилие общее
                s_Pa1[k]    .push_back(out.st[k].a.Pa1 *0.001);//усилие на штоке 1
                s_Pa2[k]    .push_back(out.st[k].a.Pa2 *0.001);//усилие на штоке 2
                s_Pap[k]    .push_back(out.st[k].Pay   *0.001);//усилие общее, приведенное к пневматику
                s_hp_x[k]   .push_back(out.st[k].p.h.x       );//обжатие эквивалентное
                s_hp_y[k]   .push_back(out.st[k].p.h.y       );//обжатие эквивалентное
                s_hp_z[k]   .push_back(out.st[k].p.h.z       );//обжатие эквивалентное
                s_hp1_x[k]  .push_back(out.st[k].p.h1.x      );//обжатие пневматика 1
                s_hp1_y[k]  .push_back(out.st[k].p.h1.y      );//обжатие пневматика 1
                s_hp1_z[k]  .push_back(out.st[k].p.h1.z      );//обжатие пневматика 1
                s_hp2_x[k]  .push_back(out.st[k].p.h2.x      );//обжатие пневматика 2
                s_hp2_y[k]  .push_back(out.st[k].p.h2.y      );//обжатие пневматика 2
                s_hp2_z[k]  .push_back(out.st[k].p.h2.z      );//обжатие пневматика 2
                s_Pp_x[k]   .push_back(out.st[k].p.P.x *0.001);//усилие эквивалентное
                s_Pp_y[k]   .push_back(out.st[k].p.P.y *0.001);//усилие эквивалентное
                s_Pp_z[k]   .push_back(out.st[k].p.P.z *0.001);//усилие эквивалентное
                s_Pp1_x[k]  .push_back(out.st[k].p.P1.x*0.001);//усилие на пневматике 1
                s_Pp1_y[k]  .push_back(out.st[k].p.P1.y*0.001);//усилие на пневматике 1
                s_Pp1_z[k]  .push_back(out.st[k].p.P1.z*0.001);//усилие на пневматике 1
                s_Pp2_x[k]  .push_back(out.st[k].p.P2.x*0.001);//усилие на пневматике 2
                s_Pp2_y[k]  .push_back(out.st[k].p.P2.y*0.001);//усилие на пневматике 2
                s_Pp2_z[k]  .push_back(out.st[k].p.P2.z*0.001);//усилие на пневматике 2
            }

            //заполнение таблицы значениями текущей итерации
            write_to_grid();
            write_to_grid_gear();
            write_to_grid_mr();
            write_to_grid_tr();
            write_to_grid_pr();
            write_to_opengl();
            //установка управляющих сигналов
            setManualControl(out.tr.fi0, out.mr.fi0, out.mr.eta, out.mr.chi);
            //данные на форме
            ui->showiter->setText(ss.setNum(out.iter));
            ui->showtime->setText(ss.setNum(out.t,'f',6));
            ui->progress->setValue( 100*out.iter/float(ini.steps) );
            QCoreApplication::processEvents();
            j += ini.logstep;
        }
        else break;
    }

    //запись пересчитанных параметров
    {
        repoststring += "\n";
        repoststring += singleline;
        toLogf(repoststring);
    }

    write_to_plot();//построение графиков
    loadLog();//загрузка .log файла
    ui->tabWidget->setCurrentIndex(3);//активная вкладка
}
void MainWindow::RecalcSelected(float x_, float y_, float z_)
{
    //передача в список координат заданной точки
    out.fus.selpoint.x = x_;
    out.fus.selpoint.y = y_;
    out.fus.selpoint.z = z_;
    //заданная точка в земной СК
    out.fus.selpoint_Q = SKStoSKZ(out.fus.selpoint);
    //присвоение значений временным переменным
    float  g = ini.g;
    float  x = x_;
    float  y = y_;
    float  z = z_;
    float vx = out.fus.v.x;
    float vy = out.fus.v.y;
    float vz = out.fus.v.z;
    float ax = out.fus.a.x;
    float ay = out.fus.a.y;
    float az = out.fus.a.z;
    float wx = out.fus.w.x;
    float wy = out.fus.w.y;
    float wz = out.fus.w.z;
    float ex = out.fus.e.x;
    float ey = out.fus.e.y;
    float ez = out.fus.e.z;
    //расчет скорости в заданной точке
    vx += (wy*z - wz*y);
    vy += (wz*x - wx*z);
    vz += (wx*y - wy*x);
    //расчет ускорения в заданной точке
    ax += (ey*z - ez*y);
    ay += (ez*x - ex*z);
    az += (ex*y - ey*x);
    //расчет перегрузки в заданной точке
    out.fus.selpoint_n.x =    ax /g;
    out.fus.selpoint_n.y = (g+ay)/g;
    out.fus.selpoint_n.z =    az /g;
    //
    out.fus.selpoint_v.x = vx;
    out.fus.selpoint_v.y = vy;
    out.fus.selpoint_v.z = vz;
    out.fus.selpoint_a.x = ax;
    out.fus.selpoint_a.y = ay;
    out.fus.selpoint_a.z = az;
}
void MainWindow::ReadingCycle(int updwn)//
{
    setStopped(true);
    int  j = ui->EditCurrent->text().toInt();
    j += ini.logstep*updwn;
    read1state(ss.setNum(j));
}
void MainWindow::ReadingCycles(int updwn)//
{
    setStopped(false);
    bool readerror=0;
    int  j = ui->EditCurrent->text().toInt();
    do
    {
        readerror = read1state(ss.setNum(j));
        if(!readerror)  j+= ini.logstep *updwn* brakeSlider();
        QCoreApplication::processEvents();
    }
    while( !readerror && !isStop && j>=ui->box_start->value() && j<=ui->box_end->value() );
    //сброс на зафиксированную итерацию
    if((ui->btnRepeat->isChecked())&&(!isStop))
    {
        ui->EditCurrent->setText(ss.setNum(ui->box_start->value()));
        read1state(ui->EditCurrent->text());
    }
}
int  MainWindow::brakeSlider()
{
    int br = ui->brakeSlider->value();
    if (br <= 0)
    {
        Sleep( pow(double(ui->brakeSlider->value()*(-0.5)),1.75) );
        QCoreApplication::processEvents();
        return 1;
    }
   else return br;
}
bool MainWindow::read1state(QString fname)//
{
    ui->solveread->setText(" READ:");
    readerror = read_helicopter_state( ini.datafolder + fname );
    if(!readerror)
        ui->EditCurrent->setText(fname);
    else
    {//файл не найден
        if(ui->btnRepeat->isChecked())
        {
            ui->EditCurrent->setText(ss.setNum(ui->box_start->value()));//ui->toolButton->text()
            readerror = read_helicopter_state( ini.datafolder + ui->EditCurrent->text() );//ui->toolButton->text()
        }
    }
    //показать состояние на 3Д модели
    write_to_opengl();
    if(ui->btn_infopanel->isChecked()) showReadedInfo();
    //установка управляющих сигналов
    setManualControl(out.tr.fi0, out.mr.fi0, out.mr.eta, out.mr.chi);
    ui->showiter->setText(ss.setNum(out.iter));
    ui->showtime->setText(ss.setNum(out.t,'f',6));
    ui->progress->setValue( 100*out.iter/float(ini.steps) );
return readerror;
}
void MainWindow::on_pushButton_2_clicked()//ReadingCycle(+1)
{
    ReadingCycle(+1);
}
void MainWindow::on_pushButton_3_clicked()//ReadingCycle(-1)
{
    ReadingCycle(-1);
}
void MainWindow::on_pushButton_5_clicked()//ReadingCycles(+1)
{
    ReadingCycles(+1);
}
void MainWindow::on_pushButton_4_clicked()//ReadingCycles(-1)
{
    ReadingCycles(-1);
}
void MainWindow::on_EditCurrent_returnPressed()//показать заданную итерацию
{
    read1state(ui->EditCurrent->text());
}

//Grid
void MainWindow::write_to_grid_const()//currentstep to Grid
{
    int j;
    ui->grid_planer->setRowCount(1);//(igrid+1);
    ui->grid_planer->setColumnCount(333);
    j=0;

    headcellp = new QTableWidgetItem("i");              ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("t, c");           ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("O.x, м");         ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("O.y, м");         ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("O.z, м");         ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Ocw.x, м");       ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Ocw.y, м");       ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Ocw.z, м");       ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("kren, грд");      ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("rysk, грд");      ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("tang, грд");      ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("vn.x, м/с");      ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("vn.y, м/с");      ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("vn.z, м/с");      ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("wn.x, рад/с");    ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("wn.y, рад/с");    ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("wn.z, рад/с");    ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("v.x, м/с");       ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("v.y, м/с");       ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("v.z, м/с");       ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("w.x, рад/с");     ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("w.y, рад/с");     ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("w.z, рад/с");     ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("a.x, м/с2");      ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("a.y, м/с2");      ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("a.z, м/с2");      ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("e.x, рад/с2");    ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("e.y, рад/с2");    ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("e.z, рад/с2");    ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("n.x, -");         ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("n.y, -");         ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("n.z, -");         ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);

    headcellp = new QTableWidgetItem("spv.x, м/с");     ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("spv.y, м/с");     ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("spv.z, м/с");     ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("spa.x, м/с2");    ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("spa.y, м/с2");    ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("spa.z, м/с2");    ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("spn.x, -");       ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("spn.y, -");       ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("spn.z, -");       ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("spQ.x, м");       ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("spQ.y, м");       ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("spQ.z, м");       ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);

    headcellp = new QTableWidgetItem("Fx, Н");          ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Fy, Н");          ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Fz, Н");          ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Mx, Н·m");        ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("My, Н·m");        ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Mz, Н·m");        ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Fadp.x, Н"   );   ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Fadp.y, Н"   );   ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Fadp.z, Н"   );   ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Madp.x, Н·m" );   ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Madp.y, Н·m" );   ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Madp.z, Н·m" );   ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("fus_F.x, Н"  );   ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("fus_F.y, Н"  );   ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("fus_F.z, Н"  );   ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("fus_M.x, Н·m");   ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("fus_M.y, Н·m");   ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("fus_M.z, Н·m");   ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("stab_F.x, Н" );   ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("stab_F.y, Н" );   ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("stab_F.z, Н" );   ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("keel_F.x, Н" );   ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("keel_F.y, Н" );   ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("keel_F.z, Н" );   ui->grid_planer->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);

    headcellp = new QTableWidgetItem("");
    ui->grid_planer->setColumnCount(j);
}
void MainWindow::write_to_grid()//currentstep to Grid
{
    int j;
    QString cellstr;
    int igrid=(out.iter-ini.start)/ini.logstep;
    ui->grid_planer->setRowCount(igrid+1);
    j=0;
    cellp = new QTableWidgetItem(cellstr.setNum(out.iter                ));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.t,             'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.org.x,     'f',3));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.org.y,     'f',3));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.org.z,     'f',3));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.ocw.x,     'f',3));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.ocw.y,     'f',3));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.ocw.z,     'f',3));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.org.k,     'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.org.r,     'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.org.t,     'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.vn.x,      'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.vn.y,      'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.vn.z,      'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.wn.x,      'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.wn.y,      'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.wn.z,      'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.v.x,       'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.v.y,       'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.v.z,       'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.w.x,       'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.w.y,       'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.w.z,       'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.a.x,       'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.a.y,       'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.a.z,       'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.e.x,       'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.e.y,       'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.e.z,       'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.n.x,       'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.n.y,       'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.n.z,       'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);

    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.selpoint_v.x, 'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.selpoint_v.y, 'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.selpoint_v.z, 'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.selpoint_a.x, 'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.selpoint_a.y, 'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.selpoint_a.z, 'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.selpoint_n.x, 'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.selpoint_n.y, 'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.selpoint_n.z, 'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.selpoint_Q.x, 'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.selpoint_Q.y, 'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.selpoint_Q.z, 'f',4));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);

    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.Fsum.x,    'f',1));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.Fsum.y,    'f',1));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.Fsum.z,    'f',1));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.Msum.x,    'f',1));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.Msum.y,    'f',1));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.Msum.z,    'f',1));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.Fadp.x,    'f',1));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.Fadp.y,    'f',1));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.Fadp.z,    'f',1));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.Madp.x,    'f',1));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.Madp.y,    'f',1));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.Madp.z,    'f',1));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.fus_F.x,   'f',1));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.fus_F.y,   'f',1));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.fus_F.z,   'f',1));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.fus_M.x,   'f',1));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.fus_M.y,   'f',1));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.fus_M.z,   'f',1));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.stab_F.x,  'f',1));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.stab_F.y,  'f',1));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.stab_F.z,  'f',1));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.keel_F.x,  'f',1));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.keel_F.y,  'f',1));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.fus.keel_F.z,  'f',1));    ui->grid_planer->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);

    cellp = new QTableWidgetItem("");  ui->grid_planer->setItem(igrid,j,cellp); cellp->setBackgroundColor(QColor(212,208,200)); j++;
    cellp = new QTableWidgetItem("");
}
void MainWindow::write_to_grid_gear_const()//currentstep to Grid
{
    int j;
    QString ss;
    ui->grid_gear->setRowCount(1);//(igrid+1);
    ui->grid_gear->setColumnCount(333);
    j=0;
    headcellp = new QTableWidgetItem("i");           ui->grid_gear->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("t, c");        ui->grid_gear->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("h|s");         ui->grid_gear->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    for(int k=0; k<out.nst; k++)
    {
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".status"    );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".psih"      );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".psif"      );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".Aa, Дж"    );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".Ap, Дж"    );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".Aa+p, Дж"  );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);

    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".a.status"  );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".a.h, мм"   );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".a.h1, мм"  );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".a.h2, мм"  );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".a.P , кН"  );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".a.P1, кН"  );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".a.P2, кН"  );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    //headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".a.Pпр, кН" );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".a.Pg1, кН" );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".a.Pg2, кН" );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".a.Ph1, кН" );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".a.Ph2, кН" );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".a.Pf1, кН" );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".a.Pf2, кН" );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".a.v1, мм/с");   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".a.v2, мм/с");   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".a.V1, дм3" );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".a.V2, дм3" );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".a.p1, ат"  );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".a.p2, ат"  );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);

    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".p.status"  );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".p.h.x, мм" );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".p.h.y, мм" );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".p.h.z, мм" );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".p.h1.x, мм");   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".p.h1.y, мм");   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".p.h1.z, мм");   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".p.h2.x, мм");   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".p.h2.y, мм");   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".p.h2.z, мм");   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".p.P.x, Н"  );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".p.P.y, Н"  );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".p.P.z, Н"  );   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".p.P1.x, кН");   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".p.P1.y, кН");   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".p.P1.z, кН");   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".p.P2.x, кН");   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".p.P2.y, кН");   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s"+ss.setNum(k)+".p.P2.z, кН");   ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("s|s");                            ui->grid_gear->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    }
    headcellp = new QTableWidgetItem("");
    ui->grid_gear->setColumnCount(j);
}
void MainWindow::write_to_grid_gear()//currentstep to Grid
{
    int j;
    QString cellstr;
    int igrid=(out.iter-ini.start)/ini.logstep;
    ui->grid_gear->setRowCount(igrid+1);
    j=0;
    cellp = new QTableWidgetItem(cellstr.setNum(out.iter     ));    ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.t,  'f',4));    ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    for(int k=0; k<out.nst; k++)
    {
    cellp = new QTableWidgetItem("");                                            ui->grid_gear->setItem(igrid,j,cellp); cellp->setBackgroundColor(QColor(212,208,200)); j++;
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].status        ));      ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].psih    ,'f',3));      ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //передаточный коэффициент по перемещению
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].psif    ,'f',3));      ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //передаточный коэффициент по усилию
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].Aa      ,'f',1));      ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //работа амортизатора
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].Ap      ,'f',1));      ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //работа пневматика
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].Aap     ,'f',1));      ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //работа стойки шасси

    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].a.status      ));      ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //состояние амортизатора
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].a.ha    ,'f',3));      ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //обжатие суммарное
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].a.ha1   ,'f',3));      ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //обжатие камеры 1
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].a.ha2   ,'f',3));      ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //обжатие камеры 2
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].a.Pa *0.001,'f',3));   ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //усилие общее
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].a.Pa1*0.001,'f',3));   ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //усилие на штоке 1
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].a.Pa2*0.001,'f',3));   ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //усилие на штоке 2
    //cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].Pay  *0.001,'f',3));   ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //усилие общее, приведенное к пневматику
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].a.Pg1*0.001,'f',3));   ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //усилие на штоке 1
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].a.Pg2*0.001,'f',3));   ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //усилие на штоке 1
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].a.Ph1*0.001,'f',3));   ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //усилие на штоке 1
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].a.Ph2*0.001,'f',3));   ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //усилие на штоке 1
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].a.Pf1*0.001,'f',3));   ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //усилие на штоке 1
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].a.Pf2*0.001,'f',3));   ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //усилие на штоке 1
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].a.v1*1000  ,'f',1));   ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //скорость штока
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].a.v2*1000  ,'f',1));   ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //скорость штока
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].a.V1    ,'f',4));      ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //объем газ.камеры
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].a.V2    ,'f',4));      ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //объем газ.камеры
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].a.p1    ,'f',2));      ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //давление в газ.камере
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].a.p2    ,'f',2));      ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //давление в газ.камере

    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].p.status      ));      ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //состояние пневматика
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].p.h.x   ,'f',3));      ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //обжатие эквивалентное
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].p.h.y   ,'f',3));      ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //обжатие эквивалентное
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].p.h.z   ,'f',3));      ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //обжатие эквивалентное
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].p.h1.x  ,'f',3));      ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //обжатие пневматика 1
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].p.h1.y  ,'f',3));      ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //обжатие пневматика 1
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].p.h1.z  ,'f',3));      ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //обжатие пневматика 1
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].p.h2.x  ,'f',3));      ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //обжатие пневматика 2
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].p.h2.y  ,'f',3));      ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //обжатие пневматика 2
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].p.h2.z  ,'f',3));      ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //обжатие пневматика 2
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].p.P.x *0.001,'f',3));  ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //усилие эквивалентное
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].p.P.y *0.001,'f',3));  ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //усилие эквивалентное
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].p.P.z *0.001,'f',3));  ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //усилие эквивалентное
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].p.P1.x*0.001,'f',3));  ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //усилие на пневматике 1
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].p.P1.y*0.001,'f',3));  ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //усилие на пневматике 1
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].p.P1.z*0.001,'f',3));  ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //усилие на пневматике 1
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].p.P2.x*0.001,'f',3));  ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //усилие на пневматике 2
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].p.P2.y*0.001,'f',3));  ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //усилие на пневматике 2
    cellp = new QTableWidgetItem(cellstr.setNum(out.st[k].p.P2.z*0.001,'f',3));  ui->grid_gear->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);    //усилие на пневматике 2
    }
    cellp = new QTableWidgetItem("");  ui->grid_gear->setItem(igrid,j,cellp); cellp->setBackgroundColor(QColor(212,208,200)); j++;
    cellp = new QTableWidgetItem("");
}
void MainWindow::write_to_grid_mr_const()//currentstep to Grid
{
    int j;
    QString ss;
    ui->grid_mr->setRowCount(1);
    ui->grid_mr->setColumnCount(333);
    j=0;
    headcellp = new QTableWidgetItem("i");            ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("t, c");         ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("psih, грд");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("alfa, грд");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("fi0, грд");     ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("chi, грд");     ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("eta, грд");     ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Fhx+, Н");      ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Fhy+, Н");      ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Fhz+, Н");      ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Mhx+, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Mhy+, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Mhz+, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("v.x, м/с");     ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("v.y, м/с");     ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("v.z, м/с");     ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("a.x, м/с2");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("a.y, м/с2");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("a.z, м/с2");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("w.x, рад/с");   ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("w.y, рад/с");   ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("w.z, рад/с");   ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("e.x, рад/с2");  ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("e.y, рад/с2");  ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("e.z, рад/с2");  ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
  //headcellp = new QTableWidgetItem("dct.x, мм");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
  //headcellp = new QTableWidgetItem("dct.y, мм");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
  //headcellp = new QTableWidgetItem("dct.z, мм");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
  //headcellp = new QTableWidgetItem("Faxe, Н");      ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("v|b");          ui->grid_mr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    for(int l=0; l<out.mr.nb; l++)
    {
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".psi,   грд");   ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".fi07,  грд");   ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".beta,  грд");   ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".dbeta, грд");   ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".d2beta,грд");   ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".ksi,   грд");   ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".dksi,  грд");   ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".d2ksi, грд");   ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".T,  Н");        ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Q,  Н");        ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".H,  Н");        ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".S,  Н");        ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Nr, Н");        ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Nf, Н");        ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".K1, Н");        ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".P1, Н");        ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".P2, Н");        ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Fdmp, Н");      ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Fpov, Н");      ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Fy, Н");        ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Fx, Н");        ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Fz, Н");        ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1T,  Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1Q,  Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1Gx, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1Gy, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1K,  Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1Cx, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1Cy, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1Px, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1Py, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1x,  Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1y,  Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1z,  Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2T,  Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2Q,  Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2Gx, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2Gy, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2K,  Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2Cx, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2Cy, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2Px, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2Py, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2D,  Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2x,  Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2y,  Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2z,  Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);

    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M3Ma, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M3su, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M3cb, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M3tr, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M3i,  Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);

    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M3x,  Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M3y,  Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M3z,  Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Fhx, Н"  );     ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Fhy, Н"  );     ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Fhz, Н"  );     ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Mhx, Н·м");     ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Mhy, Н·м");     ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Mhz, Н·м");     ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b|b");                            ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    }
    headcellp = new QTableWidgetItem("");
    ui->grid_mr->setColumnCount(j);
}
void MainWindow::write_to_grid_mr()//int igrid
{
    int j;
    QString cellstr;
    int igrid=(out.iter-ini.start)/ini.logstep;
    ui->grid_mr->setRowCount(igrid+1);
    j=0;
    cellp = new QTableWidgetItem(cellstr.setNum(out.iter                   ));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.t                ,'f',4));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.psih          ,'f',1));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.alfa          ,'f',3));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.fi0           ,'f',3));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.chi           ,'f',3));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.eta           ,'f',3));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.Fhx_sum       ,'f',1));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.Fhy_sum       ,'f',1));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.Fhz_sum       ,'f',1));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.Mhx_sum       ,'f',1));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.Mhy_sum       ,'f',1));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.Mhz_sum       ,'f',1));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.vh.x          ,'f',3));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.vh.y          ,'f',3));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.vh.z          ,'f',3));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.ah.x          ,'f',3));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.ah.y          ,'f',3));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.ah.z          ,'f',3));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.wh.x          ,'f',3));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.wh.y          ,'f',3));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.wh.z          ,'f',3));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.eh.x          ,'f',3));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.eh.y          ,'f',3));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.eh.z          ,'f',3));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    //cellp = new QTableWidgetItem(cellstr.setNum(out.mr.dct.x         ,'f',3));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    //cellp = new QTableWidgetItem(cellstr.setNum(out.mr.dct.y         ,'f',3));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    //cellp = new QTableWidgetItem(cellstr.setNum(out.mr.dct.z         ,'f',3));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    //cellp = new QTableWidgetItem(cellstr.setNum(out.mr.Faxe          ,'f',1));    ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    for(int l=0; l<out.mr.nb; l++)
    {
    cellp = new QTableWidgetItem("");                                            ui->grid_mr->setItem(igrid,j,cellp); cellp->setBackgroundColor(QColor(212,208,200)); j++;
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].psi      ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].fi7      ,'f',3));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].beta     ,'f',4));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].dbeta    ,'f',4));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].d2beta   ,'f',4));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].ksi      ,'f',4));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].dksi     ,'f',4));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].d2ksi    ,'f',4));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].T        ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].Q        ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].H        ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].S        ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].Nr       ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].Nf       ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].K1       ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].P1       ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].P2       ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].Fdmp     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].Fpov     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].Fy       ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].Fx       ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].Fz       ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M1T      ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M1Q      ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M1Gx     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M1Gy     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M1K      ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M1Cx     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M1Cy     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M1Px     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M1Py     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M1x      ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M1y      ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M1z      ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M2T      ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M2Q      ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M2Gx     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M2Gy     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M2K      ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M2Cx     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M2Cy     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M2Px     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M2Py     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M2D      ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M2x      ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M2y      ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M2z      ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);

    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M3Ma     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M3su     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M3Cz     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M3tr     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M3iz     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);

    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M3x      ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M3y      ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].M3z      ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].Fhx      ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].Fhy      ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].Fhz      ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].Mhx      ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].Mhy      ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.mr.b[l].Mhz      ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    }
    cellp = new QTableWidgetItem("");  ui->grid_mr->setItem(igrid,j,cellp); cellp->setBackgroundColor(QColor(212,208,200)); j++;
    cellp = new QTableWidgetItem("");
}
void MainWindow::write_to_grid_tr_const()//currentstep to Grid
{
    int j;
    QString ss;
    ui->grid_tr->setRowCount(1);
    ui->grid_tr->setColumnCount(333);
    j=0;
    headcellp = new QTableWidgetItem("i");            ui->grid_tr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("t, c");         ui->grid_tr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("psih, грд");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("alfa, грд");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("fi0, грд");     ui->grid_tr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Fhx+, Н");      ui->grid_tr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Fhy+, Н");      ui->grid_tr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Fhz+, Н");      ui->grid_tr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Mhx+, Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Mhy+, Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Mhz+, Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("v|b");          ui->grid_tr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    for(int l=0; l<out.tr.nb; l++)
    {
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".psi,   грд");   ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".fi07,  грд");   ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".beta,  грд");   ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".dbeta, грд");   ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".d2beta,грд");   ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".ksi,   грд");   ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".dksi,  грд");   ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".d2ksi, грд");   ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".T,  Н");        ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Q,  Н");        ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".H,  Н");        ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".S,  Н");        ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Nr, Н");        ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Nf, Н");        ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".K1, Н");        ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".P1, Н");        ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".P2, Н");        ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Fdmp, Н");      ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Fpov, Н");      ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Fy, Н");        ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Fx, Н");        ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Fz, Н");        ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1T,  Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1Q,  Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1Gx, Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1Gy, Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1K,  Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1Cx, Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1Cy, Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1Px, Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1Py, Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1x,  Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1y,  Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1z,  Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2T,  Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2Q,  Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2Gx, Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2Gy, Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2K,  Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2Cx, Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2Cy, Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2Px, Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2Py, Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2D,  Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2x,  Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2y,  Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2z,  Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M3Ma, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M3su, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M3cb, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M3tr, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M3i,  Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M3x,  Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M3y,  Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M3z,  Н·м");    ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Fhx, Н"  );     ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Fhy, Н"  );     ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Fhz, Н"  );     ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Mhx, Н·м");     ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Mhy, Н·м");     ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Mhz, Н·м");     ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b|b");                            ui->grid_tr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    }
    headcellp = new QTableWidgetItem("");
    ui->grid_tr->setColumnCount(j);
}
void MainWindow::write_to_grid_tr()//int igrid
{
    int j;
    QString cellstr;
    int igrid=(out.iter-ini.start)/ini.logstep;
    ui->grid_tr->setRowCount(igrid+1);
    j=0;
    cellp = new QTableWidgetItem(cellstr.setNum(out.iter                   ));    ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.t                ,'f',4));    ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.psih          ,'f',1));    ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.alfa          ,'f',3));    ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.fi0           ,'f',3));    ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.Fhx_sum       ,'f',1));    ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.Fhy_sum       ,'f',1));    ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.Fhz_sum       ,'f',1));    ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.Mhx_sum       ,'f',1));    ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.Mhy_sum       ,'f',1));    ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.Mhz_sum       ,'f',1));    ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    for(int l=0; l<out.tr.nb; l++)
    {
    cellp = new QTableWidgetItem("");                                            ui->grid_tr->setItem(igrid,j,cellp); cellp->setBackgroundColor(QColor(212,208,200)); j++;
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].psi      ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].fi7      ,'f',3));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].beta     ,'f',4));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].dbeta    ,'f',4));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].d2beta   ,'f',4));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].ksi      ,'f',4));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].dksi     ,'f',4));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].d2ksi    ,'f',4));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].T        ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].Q        ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].H        ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].S        ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].Nr       ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].Nf       ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].K1       ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].P1       ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].P2       ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].Fdmp     ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].Fpov     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].Fy       ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].Fx       ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].Fz       ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M1T      ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M1Q      ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M1Gx     ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M1Gy     ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M1K      ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M1Cx     ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M1Cy     ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M1Px     ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M1Py     ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M1x      ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M1y      ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M1z      ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M2T      ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M2Q      ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M2Gx     ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M2Gy     ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M2K      ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M2Cx     ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M2Cy     ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M2Px     ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M2Py     ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M2D      ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M2x      ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M2y      ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M2z      ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M3Ma     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M3su     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M3Cz     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M3tr     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M3iz     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M3x      ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M3y      ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].M3z      ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].Fhx      ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].Fhy      ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].Fhz      ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].Mhx      ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].Mhy      ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.tr.b[l].Mhz      ,'f',1));   ui->grid_tr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    }
    cellp = new QTableWidgetItem("");  ui->grid_tr->setItem(igrid,j,cellp); cellp->setBackgroundColor(QColor(212,208,200)); j++;
    cellp = new QTableWidgetItem("");
}
void MainWindow::write_to_grid_pr_const()//currentstep to Grid
{
    int j;
    QString ss;
    ui->grid_pr->setRowCount(1);
    ui->grid_pr->setColumnCount(333);
    j=0;
    headcellp = new QTableWidgetItem("i");            ui->grid_pr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("t, c");         ui->grid_pr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("psih, грд");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("alfa, грд");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("fi0, грд");     ui->grid_pr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Fhx+, Н");      ui->grid_pr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Fhy+, Н");      ui->grid_pr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Fhz+, Н");      ui->grid_pr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Mhx+, Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Mhy+, Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("Mhz+, Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("v|b");          ui->grid_pr->setHorizontalHeaderItem(j,headcellp);   j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    for(int l=0; l<out.pr.nb; l++)
    {
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".psi,   грд");   ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".fi07,  грд");   ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".beta,  грд");   ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".dbeta, грд");   ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".d2beta,грд");   ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".ksi,   грд");   ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".dksi,  грд");   ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".d2ksi, грд");   ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".T,  Н");        ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Q,  Н");        ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".H,  Н");        ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".S,  Н");        ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Nr, Н");        ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Nf, Н");        ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".K1, Н");        ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".P1, Н");        ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".P2, Н");        ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Fdmp, Н");      ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Fpov, Н");      ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Fy, Н");        ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Fx, Н");        ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Fz, Н");        ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1T,  Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1Q,  Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1Gx, Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1Gy, Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1K,  Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1Cx, Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1Cy, Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1Px, Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1Py, Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1x,  Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1y,  Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M1z,  Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2T,  Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2Q,  Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2Gx, Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2Gy, Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2K,  Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2Cx, Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2Cy, Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2Px, Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2Py, Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2D,  Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2x,  Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2y,  Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M2z,  Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M3Ma, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M3su, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M3cb, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M3tr, Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M3i,  Н·м");    ui->grid_mr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M3x,  Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M3y,  Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".M3z,  Н·м");    ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Fhx, Н"  );     ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Fhy, Н"  );     ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Fhz, Н"  );     ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Mhx, Н·м");     ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Mhy, Н·м");     ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b"+ss.setNum(l)+".Mhz, Н·м");     ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    headcellp = new QTableWidgetItem("b|b");                            ui->grid_pr->setHorizontalHeaderItem(j,headcellp); j++;    headcellp->setTextAlignment(Qt::AlignLeft);
    }
    headcellp = new QTableWidgetItem("");
    ui->grid_pr->setColumnCount(j);
}
void MainWindow::write_to_grid_pr()//int igrid
{
    int j;
    QString cellstr;
    int igrid=(out.iter-ini.start)/ini.logstep;
    ui->grid_pr->setRowCount(igrid+1);
    j=0;
    cellp = new QTableWidgetItem(cellstr.setNum(out.iter                   ));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.t                ,'f',4));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.psih          ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.alfa          ,'f',3));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.fi0           ,'f',3));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.Fhx_sum       ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.Fhy_sum       ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.Fhz_sum       ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.Mhx_sum       ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.Mhy_sum       ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.Mhz_sum       ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    for(int l=0; l<out.pr.nb; l++)
    {
    cellp = new QTableWidgetItem("");                                            ui->grid_pr->setItem(igrid,j,cellp); cellp->setBackgroundColor(QColor(212,208,200)); j++;
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].psi      ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].fi7      ,'f',3));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].beta     ,'f',4));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].dbeta    ,'f',4));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].d2beta   ,'f',4));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].ksi      ,'f',4));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].dksi     ,'f',4));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].d2ksi    ,'f',4));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].T        ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].Q        ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].H        ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].S        ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].Nr       ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].Nf       ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].K1       ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].P1       ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].P2       ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].Fdmp     ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].Fpov     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].Fy       ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].Fx       ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].Fz       ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M1T      ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M1Q      ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M1Gx     ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M1Gy     ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M1K      ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M1Cx     ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M1Cy     ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M1Px     ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M1Py     ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M1x      ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M1y      ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M1z      ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M2T      ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M2Q      ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M2Gx     ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M2Gy     ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M2K      ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M2Cx     ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M2Cy     ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M2Px     ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M2Py     ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M2D      ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M2x      ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M2y      ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M2z      ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M3Ma     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M3su     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M3Cz     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M3tr     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M3iz     ,'f',1));   ui->grid_mr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M3x      ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M3y      ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].M3z      ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].Fhx      ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].Fhy      ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].Fhz      ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].Mhx      ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].Mhy      ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    cellp = new QTableWidgetItem(cellstr.setNum(out.pr.b[l].Mhz      ,'f',1));   ui->grid_pr->setItem(igrid,j,cellp); j++;    cellp->setTextAlignment(Qt::AlignVCenter);
    }
    cellp = new QTableWidgetItem("");  ui->grid_pr->setItem(igrid,j,cellp); cellp->setBackgroundColor(QColor(212,208,200)); j++;
    cellp = new QTableWidgetItem("");
}

//PLOT.widget
void MainWindow::write_to_plot()//ResToPlot
{
    QString s;
    int k = ui->buttonGroupSt->checkedId();//номер выбранной стойки из buttonGroupSt
    int l = ui->buttonGroupBl->checkedId();//номер выбранной лопасти из buttonGroupBl
    graph_h_black->clearData();    graph_h_red   ->clearData();    graph_h_green->clearData();    graph_h_blue  ->clearData();
    graph_sh_red ->clearData();    graph_sh_green->clearData();    graph_sh_blue->clearData();
    graph_sP_red ->clearData();    graph_sP_green->clearData();    graph_sP_blue->clearData();

    ///graph.setData HELICOPTER
    switch (ui->combo_h_red->  currentIndex())
    {
        case  0:    graph_h_red->setVisible(false);        graph_h_red->setName("");              graph_h_red->setVisible(false);  ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(false);  break;
        case  1:    graph_h_red->setData(it,h_org_x   );   graph_h_red->setName("O.x, м"     );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case  2:    graph_h_red->setData(it,h_org_y   );   graph_h_red->setName("O.y, м"     );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case  3:    graph_h_red->setData(it,h_org_z   );   graph_h_red->setName("O.z, м"     );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case  4:    graph_h_red->setData(it,h_org_k   );   graph_h_red->setName("kren, грд"  );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case  5:    graph_h_red->setData(it,h_org_r   );   graph_h_red->setName("rysk, грд"  );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case  6:    graph_h_red->setData(it,h_org_t   );   graph_h_red->setName("tang, грд"  );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case  7:    graph_h_red->setData(it,h_vn_x    );   graph_h_red->setName("vn.x, м/с"  );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case  8:    graph_h_red->setData(it,h_vn_y    );   graph_h_red->setName("vn.y, м/с"  );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case  9:    graph_h_red->setData(it,h_vn_z    );   graph_h_red->setName("vn.z, м/с"  );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 10:    graph_h_red->setData(it,h_v_x     );   graph_h_red->setName("v.x, м/с"   );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 11:    graph_h_red->setData(it,h_v_y     );   graph_h_red->setName("v.y, м/с"   );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 12:    graph_h_red->setData(it,h_v_z     );   graph_h_red->setName("v.z, м/с"   );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 13:    graph_h_red->setData(it,h_w_x     );   graph_h_red->setName("w.k, рад/с" );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 14:    graph_h_red->setData(it,h_w_y     );   graph_h_red->setName("w.r, рад/с" );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 15:    graph_h_red->setData(it,h_w_z     );   graph_h_red->setName("w.t, рад/с" );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 16:    graph_h_red->setData(it,h_a_x     );   graph_h_red->setName("a.x, м/с2"  );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 17:    graph_h_red->setData(it,h_a_y     );   graph_h_red->setName("a.y, м/с2"  );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 18:    graph_h_red->setData(it,h_a_z     );   graph_h_red->setName("a.z, м/с2"  );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 19:    graph_h_red->setData(it,h_n_x     );   graph_h_red->setName("n.x, -"     );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 20:    graph_h_red->setData(it,h_n_y     );   graph_h_red->setName("n.y, -"     );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 21:    graph_h_red->setData(it,h_n_z     );   graph_h_red->setName("n.z, -"     );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 22:    graph_h_red->setData(it,h_Fsum_x  );   graph_h_red->setName("F.x, Н"     );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 23:    graph_h_red->setData(it,h_Fsum_y  );   graph_h_red->setName("F.y, Н"     );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 24:    graph_h_red->setData(it,h_Fsum_z  );   graph_h_red->setName("F.z, Н"     );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 25:    graph_h_red->setData(it,h_Msum_x  );   graph_h_red->setName("M.x, Н·м"   );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 26:    graph_h_red->setData(it,h_Msum_y  );   graph_h_red->setName("M.y, Н·м"   );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 27:    graph_h_red->setData(it,h_Msum_z  );   graph_h_red->setName("M.z, Н·м"   );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 28:    graph_h_red->setData(it,h_Fadp_x  );   graph_h_red->setName("Fadp_x, Н"  );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//22
        case 29:    graph_h_red->setData(it,h_Fadp_y  );   graph_h_red->setName("Fadp_y, Н"  );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//23
        case 30:    graph_h_red->setData(it,h_Fadp_z  );   graph_h_red->setName("Fadp_z, Н"  );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//24
        case 31:    graph_h_red->setData(it,h_Madp_x  );   graph_h_red->setName("Madp_x, Н·м");   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//25
        case 32:    graph_h_red->setData(it,h_Madp_y  );   graph_h_red->setName("Madp_y, Н·м");   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//26
        case 33:    graph_h_red->setData(it,h_Madp_z  );   graph_h_red->setName("Madp_z, Н·м");   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//27
        case 34:    graph_h_red->setData(it,h_Ffus_x  );   graph_h_red->setName("Ffus_x, Н"  );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//28
        case 35:    graph_h_red->setData(it,h_Ffus_y  );   graph_h_red->setName("Ffus_y, Н"  );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//29
        case 36:    graph_h_red->setData(it,h_Ffus_z  );   graph_h_red->setName("Ffus_z, Н"  );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//30
        case 37:    graph_h_red->setData(it,h_Mfus_x  );   graph_h_red->setName("Mfus_x, Н·м");   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//31
        case 38:    graph_h_red->setData(it,h_Mfus_y  );   graph_h_red->setName("Mfus_y, Н·м");   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//32
        case 39:    graph_h_red->setData(it,h_Mfus_z  );   graph_h_red->setName("Mfus_z, Н·м");   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//33
        case 40:    graph_h_red->setData(it,h_Fstab_x );   graph_h_red->setName("Fstab_x, Н" );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//34
        case 41:    graph_h_red->setData(it,h_Fstab_y );   graph_h_red->setName("Fstab_y, Н" );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//35
        case 42:    graph_h_red->setData(it,h_Fstab_z );   graph_h_red->setName("Fstab_z, Н" );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//36
        case 43:    graph_h_red->setData(it,h_Fkeel_x );   graph_h_red->setName("Fkeel_x, Н" );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//37
        case 44:    graph_h_red->setData(it,h_Fkeel_y );   graph_h_red->setName("Fkeel_y, Н" );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//38
        case 45:    graph_h_red->setData(it,h_Fkeel_z );   graph_h_red->setName("Fkeel_z, Н" );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//39
        case 46:    graph_h_red->setData(it,mr_fi0    );   graph_h_red->setName("fi0, грд"   );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 47:    graph_h_red->setData(it,mr_chi    );   graph_h_red->setName("chi, грд"   );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 48:    graph_h_red->setData(it,mr_eta    );   graph_h_red->setName("eta, грд"   );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 49:    graph_h_red->setData(it,tr_fi0    );   graph_h_red->setName("fi0tr,грд"  );   graph_h_red->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
    }
    switch (ui->combo_h_green->currentIndex())
    {
        case  0:    graph_h_green->setVisible(false);        graph_h_green->setName("");              graph_h_green->setVisible(false);  ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,1)->setVisible(false);  break;
        case  1:    graph_h_green->setData(it,h_org_x   );   graph_h_green->setName("O.x, м"     );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,1)->setVisible(true);   break;
        case  2:    graph_h_green->setData(it,h_org_y   );   graph_h_green->setName("O.y, м"     );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,1)->setVisible(true);   break;
        case  3:    graph_h_green->setData(it,h_org_z   );   graph_h_green->setName("O.z, м"     );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,1)->setVisible(true);   break;
        case  4:    graph_h_green->setData(it,h_org_k   );   graph_h_green->setName("kren, грд"  );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,1)->setVisible(true);   break;
        case  5:    graph_h_green->setData(it,h_org_r   );   graph_h_green->setName("rysk, грд"  );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,1)->setVisible(true);   break;
        case  6:    graph_h_green->setData(it,h_org_t   );   graph_h_green->setName("tang, грд"  );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,1)->setVisible(true);   break;
        case  7:    graph_h_green->setData(it,h_vn_x    );   graph_h_green->setName("vn.x, м/с"  );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case  8:    graph_h_green->setData(it,h_vn_y    );   graph_h_green->setName("vn.y, м/с"  );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case  9:    graph_h_green->setData(it,h_vn_z    );   graph_h_green->setName("vn.z, м/с"  );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 10:    graph_h_green->setData(it,h_v_x     );   graph_h_green->setName("v.x, м/с"   );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,1)->setVisible(true);   break;
        case 11:    graph_h_green->setData(it,h_v_y     );   graph_h_green->setName("v.y, м/с"   );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,1)->setVisible(true);   break;
        case 12:    graph_h_green->setData(it,h_v_z     );   graph_h_green->setName("v.z, м/с"   );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,1)->setVisible(true);   break;
        case 13:    graph_h_green->setData(it,h_w_x     );   graph_h_green->setName("w.k, рад/с" );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 14:    graph_h_green->setData(it,h_w_y     );   graph_h_green->setName("w.r, рад/с" );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 15:    graph_h_green->setData(it,h_w_z     );   graph_h_green->setName("w.t, рад/с" );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 16:    graph_h_green->setData(it,h_a_x     );   graph_h_green->setName("a.x, м/с2"  );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,1)->setVisible(true);   break;
        case 17:    graph_h_green->setData(it,h_a_y     );   graph_h_green->setName("a.y, м/с2"  );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,1)->setVisible(true);   break;
        case 18:    graph_h_green->setData(it,h_a_z     );   graph_h_green->setName("a.z, м/с2"  );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,1)->setVisible(true);   break;
        case 19:    graph_h_green->setData(it,h_n_x     );   graph_h_green->setName("n.x, -"     );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,1)->setVisible(true);   break;
        case 20:    graph_h_green->setData(it,h_n_y     );   graph_h_green->setName("n.y, -"     );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,1)->setVisible(true);   break;
        case 21:    graph_h_green->setData(it,h_n_z     );   graph_h_green->setName("n.z, -"     );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,1)->setVisible(true);   break;
        case 22:    graph_h_green->setData(it,h_Fsum_x  );   graph_h_green->setName("F.x, Н"     );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,1)->setVisible(true);   break;
        case 23:    graph_h_green->setData(it,h_Fsum_y  );   graph_h_green->setName("F.y, Н"     );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,1)->setVisible(true);   break;
        case 24:    graph_h_green->setData(it,h_Fsum_z  );   graph_h_green->setName("F.z, Н"     );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,1)->setVisible(true);   break;
        case 25:    graph_h_green->setData(it,h_Msum_x  );   graph_h_green->setName("M.x, Н·м"   );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,1)->setVisible(true);   break;
        case 26:    graph_h_green->setData(it,h_Msum_y  );   graph_h_green->setName("M.y, Н·м"   );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,1)->setVisible(true);   break;
        case 27:    graph_h_green->setData(it,h_Msum_z  );   graph_h_green->setName("M.z, Н·м"   );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,1)->setVisible(true);   break;
        case 28:    graph_h_green->setData(it,h_Fadp_x  );   graph_h_green->setName("Fadp_x, Н"  );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//22
        case 29:    graph_h_green->setData(it,h_Fadp_y  );   graph_h_green->setName("Fadp_y, Н"  );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//23
        case 30:    graph_h_green->setData(it,h_Fadp_z  );   graph_h_green->setName("Fadp_z, Н"  );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//24
        case 31:    graph_h_green->setData(it,h_Madp_x  );   graph_h_green->setName("Madp_x, Н·м");   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//25
        case 32:    graph_h_green->setData(it,h_Madp_y  );   graph_h_green->setName("Madp_y, Н·м");   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//26
        case 33:    graph_h_green->setData(it,h_Madp_z  );   graph_h_green->setName("Madp_z, Н·м");   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//27
        case 34:    graph_h_green->setData(it,h_Ffus_x  );   graph_h_green->setName("Ffus_x, Н"  );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//28
        case 35:    graph_h_green->setData(it,h_Ffus_y  );   graph_h_green->setName("Ffus_y, Н"  );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//29
        case 36:    graph_h_green->setData(it,h_Ffus_z  );   graph_h_green->setName("Ffus_z, Н"  );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//30
        case 37:    graph_h_green->setData(it,h_Mfus_x  );   graph_h_green->setName("Mfus_x, Н·м");   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//31
        case 38:    graph_h_green->setData(it,h_Mfus_y  );   graph_h_green->setName("Mfus_y, Н·м");   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//32
        case 39:    graph_h_green->setData(it,h_Mfus_z  );   graph_h_green->setName("Mfus_z, Н·м");   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//33
        case 40:    graph_h_green->setData(it,h_Fstab_x );   graph_h_green->setName("Fstab_x, Н" );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//34
        case 41:    graph_h_green->setData(it,h_Fstab_y );   graph_h_green->setName("Fstab_y, Н" );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//35
        case 42:    graph_h_green->setData(it,h_Fstab_z );   graph_h_green->setName("Fstab_z, Н" );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//36
        case 43:    graph_h_green->setData(it,h_Fkeel_x );   graph_h_green->setName("Fkeel_x, Н" );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//37
        case 44:    graph_h_green->setData(it,h_Fkeel_y );   graph_h_green->setName("Fkeel_y, Н" );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//38
        case 45:    graph_h_green->setData(it,h_Fkeel_z );   graph_h_green->setName("Fkeel_z, Н" );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//39
        case 46:    graph_h_green->setData(it,mr_fi0    );   graph_h_green->setName("fi0, грд"   );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 47:    graph_h_green->setData(it,mr_chi    );   graph_h_green->setName("chi, грд"   );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 48:    graph_h_green->setData(it,mr_eta    );   graph_h_green->setName("eta, грд"   );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 49:    graph_h_green->setData(it,tr_fi0    );   graph_h_green->setName("fi0tr,грд"  );   graph_h_green->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
    }
    switch (ui->combo_h_blue-> currentIndex())
    {
        case  0:    graph_h_blue->setVisible(false);        graph_h_blue->setName("");              graph_h_blue->setVisible(false);  ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,2)->setVisible(false);  break;
        case  1:    graph_h_blue->setData(it,h_org_x   );   graph_h_blue->setName("O.x, м"     );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,2)->setVisible(true);   break;
        case  2:    graph_h_blue->setData(it,h_org_y   );   graph_h_blue->setName("O.y, м"     );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,2)->setVisible(true);   break;
        case  3:    graph_h_blue->setData(it,h_org_z   );   graph_h_blue->setName("O.z, м"     );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,2)->setVisible(true);   break;
        case  4:    graph_h_blue->setData(it,h_org_k   );   graph_h_blue->setName("kren, грд"  );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,2)->setVisible(true);   break;
        case  5:    graph_h_blue->setData(it,h_org_r   );   graph_h_blue->setName("rysk, грд"  );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,2)->setVisible(true);   break;
        case  6:    graph_h_blue->setData(it,h_org_t   );   graph_h_blue->setName("tang, грд"  );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,2)->setVisible(true);   break;
        case  7:    graph_h_blue->setData(it,h_vn_x    );   graph_h_blue->setName("vn.x, м/с"  );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case  8:    graph_h_blue->setData(it,h_vn_y    );   graph_h_blue->setName("vn.y, м/с"  );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case  9:    graph_h_blue->setData(it,h_vn_z    );   graph_h_blue->setName("vn.z, м/с"  );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 10:    graph_h_blue->setData(it,h_v_x     );   graph_h_blue->setName("v.x, м/с"   );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,2)->setVisible(true);   break;
        case 11:    graph_h_blue->setData(it,h_v_y     );   graph_h_blue->setName("v.y, м/с"   );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,2)->setVisible(true);   break;
        case 12:    graph_h_blue->setData(it,h_v_z     );   graph_h_blue->setName("v.z, м/с"   );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,2)->setVisible(true);   break;
        case 13:    graph_h_blue->setData(it,h_w_x     );   graph_h_blue->setName("w.k, рад/с" );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 14:    graph_h_blue->setData(it,h_w_y     );   graph_h_blue->setName("w.r, рад/с" );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 15:    graph_h_blue->setData(it,h_w_z     );   graph_h_blue->setName("w.t, рад/с" );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 16:    graph_h_blue->setData(it,h_a_x     );   graph_h_blue->setName("a.x, м/с2"  );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,2)->setVisible(true);   break;
        case 17:    graph_h_blue->setData(it,h_a_y     );   graph_h_blue->setName("a.y, м/с2"  );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,2)->setVisible(true);   break;
        case 18:    graph_h_blue->setData(it,h_a_z     );   graph_h_blue->setName("a.z, м/с2"  );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,2)->setVisible(true);   break;
        case 19:    graph_h_blue->setData(it,h_n_x     );   graph_h_blue->setName("n.x, -"     );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,2)->setVisible(true);   break;
        case 20:    graph_h_blue->setData(it,h_n_y     );   graph_h_blue->setName("n.y, -"     );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,2)->setVisible(true);   break;
        case 21:    graph_h_blue->setData(it,h_n_z     );   graph_h_blue->setName("n.z, -"     );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,2)->setVisible(true);   break;
        case 22:    graph_h_blue->setData(it,h_Fsum_x  );   graph_h_blue->setName("F.x, Н"     );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,2)->setVisible(true);   break;
        case 23:    graph_h_blue->setData(it,h_Fsum_y  );   graph_h_blue->setName("F.y, Н"     );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,2)->setVisible(true);   break;
        case 24:    graph_h_blue->setData(it,h_Fsum_z  );   graph_h_blue->setName("F.z, Н"     );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,2)->setVisible(true);   break;
        case 25:    graph_h_blue->setData(it,h_Msum_x  );   graph_h_blue->setName("M.x, Н·м"   );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,2)->setVisible(true);   break;
        case 26:    graph_h_blue->setData(it,h_Msum_y  );   graph_h_blue->setName("M.y, Н·м"   );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,2)->setVisible(true);   break;
        case 27:    graph_h_blue->setData(it,h_Msum_z  );   graph_h_blue->setName("M.z, Н·м"   );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,2)->setVisible(true);   break;
        case 28:    graph_h_blue->setData(it,h_Fadp_x  );   graph_h_blue->setName("Fadp_x, Н"  );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//22
        case 29:    graph_h_blue->setData(it,h_Fadp_y  );   graph_h_blue->setName("Fadp_y, Н"  );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//23
        case 30:    graph_h_blue->setData(it,h_Fadp_z  );   graph_h_blue->setName("Fadp_z, Н"  );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//24
        case 31:    graph_h_blue->setData(it,h_Madp_x  );   graph_h_blue->setName("Madp_x, Н·м");   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//25
        case 32:    graph_h_blue->setData(it,h_Madp_y  );   graph_h_blue->setName("Madp_y, Н·м");   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//26
        case 33:    graph_h_blue->setData(it,h_Madp_z  );   graph_h_blue->setName("Madp_z, Н·м");   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//27
        case 34:    graph_h_blue->setData(it,h_Ffus_x  );   graph_h_blue->setName("Ffus_x, Н"  );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//28
        case 35:    graph_h_blue->setData(it,h_Ffus_y  );   graph_h_blue->setName("Ffus_y, Н"  );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//29
        case 36:    graph_h_blue->setData(it,h_Ffus_z  );   graph_h_blue->setName("Ffus_z, Н"  );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//30
        case 37:    graph_h_blue->setData(it,h_Mfus_x  );   graph_h_blue->setName("Mfus_x, Н·м");   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//31
        case 38:    graph_h_blue->setData(it,h_Mfus_y  );   graph_h_blue->setName("Mfus_y, Н·м");   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//32
        case 39:    graph_h_blue->setData(it,h_Mfus_z  );   graph_h_blue->setName("Mfus_z, Н·м");   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//33
        case 40:    graph_h_blue->setData(it,h_Fstab_x );   graph_h_blue->setName("Fstab_x, Н" );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//34
        case 41:    graph_h_blue->setData(it,h_Fstab_y );   graph_h_blue->setName("Fstab_y, Н" );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//35
        case 42:    graph_h_blue->setData(it,h_Fstab_z );   graph_h_blue->setName("Fstab_z, Н" );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//36
        case 43:    graph_h_blue->setData(it,h_Fkeel_x );   graph_h_blue->setName("Fkeel_x, Н" );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//37
        case 44:    graph_h_blue->setData(it,h_Fkeel_y );   graph_h_blue->setName("Fkeel_y, Н" );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//38
        case 45:    graph_h_blue->setData(it,h_Fkeel_z );   graph_h_blue->setName("Fkeel_z, Н" );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//39
        case 46:    graph_h_blue->setData(it,mr_fi0    );   graph_h_blue->setName("fi0, грд"   );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 47:    graph_h_blue->setData(it,mr_chi    );   graph_h_blue->setName("chi, грд"   );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 48:    graph_h_blue->setData(it,mr_eta    );   graph_h_blue->setName("eta, грд"   );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 49:    graph_h_blue->setData(it,tr_fi0    );   graph_h_blue->setName("fi0tr,грд"  );   graph_h_blue->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
    }
    switch (ui->combo_h_black->currentIndex())
    {
        case  0:    graph_h_black->setVisible(false);        graph_h_black->setName("");              graph_h_black->setVisible(false);  ui->plot_helicopter->axisRect()->axis(QCPAxis::atRight,1)->setVisible(false);  break;
        case  1:    graph_h_black->setData(it,h_org_x   );   graph_h_black->setName("O.x, м"     );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);   break;
        case  2:    graph_h_black->setData(it,h_org_y   );   graph_h_black->setName("O.y, м"     );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);   break;
        case  3:    graph_h_black->setData(it,h_org_z   );   graph_h_black->setName("O.z, м"     );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);   break;
        case  4:    graph_h_black->setData(it,h_org_k   );   graph_h_black->setName("kren, грд"  );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);   break;
        case  5:    graph_h_black->setData(it,h_org_r   );   graph_h_black->setName("rysk, грд"  );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);   break;
        case  6:    graph_h_black->setData(it,h_org_t   );   graph_h_black->setName("tang, грд"  );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);   break;
        case  7:    graph_h_black->setData(it,h_vn_x    );   graph_h_black->setName("vn.x, м/с"  );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case  8:    graph_h_black->setData(it,h_vn_y    );   graph_h_black->setName("vn.y, м/с"  );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case  9:    graph_h_black->setData(it,h_vn_z    );   graph_h_black->setName("vn.z, м/с"  );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 10:    graph_h_black->setData(it,h_v_x     );   graph_h_black->setName("v.x, м/с"   );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);   break;
        case 11:    graph_h_black->setData(it,h_v_y     );   graph_h_black->setName("v.y, м/с"   );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);   break;
        case 12:    graph_h_black->setData(it,h_v_z     );   graph_h_black->setName("v.z, м/с"   );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);   break;
        case 13:    graph_h_black->setData(it,h_w_x     );   graph_h_black->setName("w.k, рад/с" );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 14:    graph_h_black->setData(it,h_w_y     );   graph_h_black->setName("w.r, рад/с" );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 15:    graph_h_black->setData(it,h_w_z     );   graph_h_black->setName("w.t, рад/с" );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 16:    graph_h_black->setData(it,h_a_x     );   graph_h_black->setName("a.x, м/с2"  );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);   break;
        case 17:    graph_h_black->setData(it,h_a_y     );   graph_h_black->setName("a.y, м/с2"  );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);   break;
        case 18:    graph_h_black->setData(it,h_a_z     );   graph_h_black->setName("a.z, м/с2"  );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);   break;
        case 19:    graph_h_black->setData(it,h_n_x     );   graph_h_black->setName("n.x, -"     );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);   break;
        case 20:    graph_h_black->setData(it,h_n_y     );   graph_h_black->setName("n.y, -"     );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);   break;
        case 21:    graph_h_black->setData(it,h_n_z     );   graph_h_black->setName("n.z, -"     );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);   break;
        case 22:    graph_h_black->setData(it,h_Fsum_x  );   graph_h_black->setName("F.x, Н"     );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);   break;
        case 23:    graph_h_black->setData(it,h_Fsum_y  );   graph_h_black->setName("F.y, Н"     );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);   break;
        case 24:    graph_h_black->setData(it,h_Fsum_z  );   graph_h_black->setName("F.z, Н"     );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);   break;
        case 25:    graph_h_black->setData(it,h_Msum_x  );   graph_h_black->setName("M.x, Н·м"   );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);   break;
        case 26:    graph_h_black->setData(it,h_Msum_y  );   graph_h_black->setName("M.y, Н·м"   );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);   break;
        case 27:    graph_h_black->setData(it,h_Msum_z  );   graph_h_black->setName("M.z, Н·м"   );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);   break;
        case 28:    graph_h_black->setData(it,h_Fadp_x  );   graph_h_black->setName("Fadp_x, Н"  );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//22
        case 29:    graph_h_black->setData(it,h_Fadp_y  );   graph_h_black->setName("Fadp_y, Н"  );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//23
        case 30:    graph_h_black->setData(it,h_Fadp_z  );   graph_h_black->setName("Fadp_z, Н"  );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//24
        case 31:    graph_h_black->setData(it,h_Madp_x  );   graph_h_black->setName("Madp_x, Н·м");   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//25
        case 32:    graph_h_black->setData(it,h_Madp_y  );   graph_h_black->setName("Madp_y, Н·м");   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//26
        case 33:    graph_h_black->setData(it,h_Madp_z  );   graph_h_black->setName("Madp_z, Н·м");   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//27
        case 34:    graph_h_black->setData(it,h_Ffus_x  );   graph_h_black->setName("Ffus_x, Н"  );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//28
        case 35:    graph_h_black->setData(it,h_Ffus_y  );   graph_h_black->setName("Ffus_y, Н"  );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//29
        case 36:    graph_h_black->setData(it,h_Ffus_z  );   graph_h_black->setName("Ffus_z, Н"  );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//30
        case 37:    graph_h_black->setData(it,h_Mfus_x  );   graph_h_black->setName("Mfus_x, Н·м");   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//31
        case 38:    graph_h_black->setData(it,h_Mfus_y  );   graph_h_black->setName("Mfus_y, Н·м");   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//32
        case 39:    graph_h_black->setData(it,h_Mfus_z  );   graph_h_black->setName("Mfus_z, Н·м");   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//33
        case 40:    graph_h_black->setData(it,h_Fstab_x );   graph_h_black->setName("Fstab_x, Н" );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//34
        case 41:    graph_h_black->setData(it,h_Fstab_y );   graph_h_black->setName("Fstab_y, Н" );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//35
        case 42:    graph_h_black->setData(it,h_Fstab_z );   graph_h_black->setName("Fstab_z, Н" );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//36
        case 43:    graph_h_black->setData(it,h_Fkeel_x );   graph_h_black->setName("Fkeel_x, Н" );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//37
        case 44:    graph_h_black->setData(it,h_Fkeel_y );   graph_h_black->setName("Fkeel_y, Н" );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//38
        case 45:    graph_h_black->setData(it,h_Fkeel_z );   graph_h_black->setName("Fkeel_z, Н" );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;//39
        case 46:    graph_h_black->setData(it,mr_fi0    );   graph_h_black->setName("fi0, грд"   );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 47:    graph_h_black->setData(it,mr_chi    );   graph_h_black->setName("chi, грд"   );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 48:    graph_h_black->setData(it,mr_eta    );   graph_h_black->setName("eta, грд"   );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
        case 49:    graph_h_black->setData(it,tr_fi0    );   graph_h_black->setName("fi0tr,грд"  );   graph_h_black->setVisible(true);   ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(true);   break;
    }

    ///graph.setData SHASSI
    switch (ui->combo_sh_red->  currentIndex())
    {
        case  0:    graph_sh_red->setVisible(false);        graph_sh_red->setName("");            graph_sh_red->setVisible(false);    break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(false);
        case  1:    graph_sh_red->setData(it,s_ha[k]   );   graph_sh_red->setName("s"+s.setNum(k)+".ha   , мм");   graph_sh_red->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case  2:    graph_sh_red->setData(it,s_ha1[k]  );   graph_sh_red->setName("s"+s.setNum(k)+".ha1  , мм");   graph_sh_red->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case  3:    graph_sh_red->setData(it,s_ha2[k]  );   graph_sh_red->setName("s"+s.setNum(k)+".ha2  , мм");   graph_sh_red->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case  4:    graph_sh_red->setData(it,s_hp_x[k] );   graph_sh_red->setName("s"+s.setNum(k)+".hp.x , мм");   graph_sh_red->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case  5:    graph_sh_red->setData(it,s_hp_y[k] );   graph_sh_red->setName("s"+s.setNum(k)+".hp.y , мм");   graph_sh_red->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case  6:    graph_sh_red->setData(it,s_hp_z[k] );   graph_sh_red->setName("s"+s.setNum(k)+".hp.z , мм");   graph_sh_red->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case  7:    graph_sh_red->setData(it,s_hp1_x[k]);   graph_sh_red->setName("s"+s.setNum(k)+".hp1.x, мм");   graph_sh_red->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case  8:    graph_sh_red->setData(it,s_hp1_y[k]);   graph_sh_red->setName("s"+s.setNum(k)+".hp1.y, мм");   graph_sh_red->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case  9:    graph_sh_red->setData(it,s_hp1_z[k]);   graph_sh_red->setName("s"+s.setNum(k)+".hp1.z, мм");   graph_sh_red->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case 10:    graph_sh_red->setData(it,s_hp2_x[k]);   graph_sh_red->setName("s"+s.setNum(k)+".hp2.x, мм");   graph_sh_red->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case 11:    graph_sh_red->setData(it,s_hp2_y[k]);   graph_sh_red->setName("s"+s.setNum(k)+".hp2.y, мм");   graph_sh_red->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case 12:    graph_sh_red->setData(it,s_hp2_z[k]);   graph_sh_red->setName("s"+s.setNum(k)+".hp2.z, мм");   graph_sh_red->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
    }
    switch (ui->combo_sh_green->currentIndex())
    {
        case  0:    graph_sh_green->setVisible(false);        graph_sh_green->setName("");            graph_sh_green->setVisible(false);    break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft,1)->setVisible(false);
        case  1:    graph_sh_green->setData(it,s_ha[k]   );   graph_sh_green->setName("s"+s.setNum(k)+".ha   , мм");   graph_sh_green->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case  2:    graph_sh_green->setData(it,s_ha1[k]  );   graph_sh_green->setName("s"+s.setNum(k)+".ha1  , мм");   graph_sh_green->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case  3:    graph_sh_green->setData(it,s_ha2[k]  );   graph_sh_green->setName("s"+s.setNum(k)+".ha2  , мм");   graph_sh_green->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case  4:    graph_sh_green->setData(it,s_hp_x[k] );   graph_sh_green->setName("s"+s.setNum(k)+".hp.x , мм");   graph_sh_green->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case  5:    graph_sh_green->setData(it,s_hp_y[k] );   graph_sh_green->setName("s"+s.setNum(k)+".hp.y , мм");   graph_sh_green->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case  6:    graph_sh_green->setData(it,s_hp_z[k] );   graph_sh_green->setName("s"+s.setNum(k)+".hp.z , мм");   graph_sh_green->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case  7:    graph_sh_green->setData(it,s_hp1_x[k]);   graph_sh_green->setName("s"+s.setNum(k)+".hp1.x, мм");   graph_sh_green->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case  8:    graph_sh_green->setData(it,s_hp1_y[k]);   graph_sh_green->setName("s"+s.setNum(k)+".hp1.y, мм");   graph_sh_green->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case  9:    graph_sh_green->setData(it,s_hp1_z[k]);   graph_sh_green->setName("s"+s.setNum(k)+".hp1.z, мм");   graph_sh_green->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case 10:    graph_sh_green->setData(it,s_hp2_x[k]);   graph_sh_green->setName("s"+s.setNum(k)+".hp2.x, мм");   graph_sh_green->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case 11:    graph_sh_green->setData(it,s_hp2_y[k]);   graph_sh_green->setName("s"+s.setNum(k)+".hp2.y, мм");   graph_sh_green->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case 12:    graph_sh_green->setData(it,s_hp2_z[k]);   graph_sh_green->setName("s"+s.setNum(k)+".hp2.z, мм");   graph_sh_green->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
    }
    switch (ui->combo_sh_blue-> currentIndex())
    {
        case  0:    graph_sh_blue->setVisible(false);        graph_sh_blue->setName("");            graph_sh_blue->setVisible(false);    break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft,2)->setVisible(false);
        case  1:    graph_sh_blue->setData(it,s_ha[k]   );   graph_sh_blue->setName("s"+s.setNum(k)+".ha   , мм");   graph_sh_blue->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case  2:    graph_sh_blue->setData(it,s_ha1[k]  );   graph_sh_blue->setName("s"+s.setNum(k)+".ha1  , мм");   graph_sh_blue->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case  3:    graph_sh_blue->setData(it,s_ha2[k]  );   graph_sh_blue->setName("s"+s.setNum(k)+".ha2  , мм");   graph_sh_blue->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case  4:    graph_sh_blue->setData(it,s_hp_x[k] );   graph_sh_blue->setName("s"+s.setNum(k)+".hp.x , мм");   graph_sh_blue->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case  5:    graph_sh_blue->setData(it,s_hp_y[k] );   graph_sh_blue->setName("s"+s.setNum(k)+".hp.y , мм");   graph_sh_blue->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case  6:    graph_sh_blue->setData(it,s_hp_z[k] );   graph_sh_blue->setName("s"+s.setNum(k)+".hp.z , мм");   graph_sh_blue->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case  7:    graph_sh_blue->setData(it,s_hp1_x[k]);   graph_sh_blue->setName("s"+s.setNum(k)+".hp1.x, мм");   graph_sh_blue->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case  8:    graph_sh_blue->setData(it,s_hp1_y[k]);   graph_sh_blue->setName("s"+s.setNum(k)+".hp1.y, мм");   graph_sh_blue->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case  9:    graph_sh_blue->setData(it,s_hp1_z[k]);   graph_sh_blue->setName("s"+s.setNum(k)+".hp1.z, мм");   graph_sh_blue->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case 10:    graph_sh_blue->setData(it,s_hp2_x[k]);   graph_sh_blue->setName("s"+s.setNum(k)+".hp2.x, мм");   graph_sh_blue->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case 11:    graph_sh_blue->setData(it,s_hp2_y[k]);   graph_sh_blue->setName("s"+s.setNum(k)+".hp2.y, мм");   graph_sh_blue->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
        case 12:    graph_sh_blue->setData(it,s_hp2_z[k]);   graph_sh_blue->setName("s"+s.setNum(k)+".hp2.z, мм");   graph_sh_blue->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1)->setVisible(true);
    }
    switch (ui->combo_sP_red->  currentIndex())
    {
        case  0:    graph_sP_red->setVisible(false);        graph_sP_red->setName("");            graph_sP_red->setVisible(false);    break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(false);
        case  1:    graph_sP_red->setData(it,s_Pa[k]   );   graph_sP_red->setName("s"+s.setNum(k)+".Pa   , кН");   graph_sP_red->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
        case  2:    graph_sP_red->setData(it,s_Pa1[k]  );   graph_sP_red->setName("s"+s.setNum(k)+".Pa1  , кН");   graph_sP_red->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
        case  3:    graph_sP_red->setData(it,s_Pa2[k]  );   graph_sP_red->setName("s"+s.setNum(k)+".Pa2  , кН");   graph_sP_red->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
        case  4:    graph_sP_red->setData(it,s_Pap[k]  );   graph_sP_red->setName("s"+s.setNum(k)+".P_пр , кН");   graph_sP_red->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
        case  5:    graph_sP_red->setData(it,s_Pp_x[k] );   graph_sP_red->setName("s"+s.setNum(k)+".Pp.x , кН");   graph_sP_red->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
        case  6:    graph_sP_red->setData(it,s_Pp_y[k] );   graph_sP_red->setName("s"+s.setNum(k)+".Pp.y , кН");   graph_sP_red->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
        case  7:    graph_sP_red->setData(it,s_Pp_z[k] );   graph_sP_red->setName("s"+s.setNum(k)+".Pp.z , кН");   graph_sP_red->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
        case  8:    graph_sP_red->setData(it,s_Pp1_x[k]);   graph_sP_red->setName("s"+s.setNum(k)+".Pp1.x, кН");   graph_sP_red->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
        case  9:    graph_sP_red->setData(it,s_Pp1_y[k]);   graph_sP_red->setName("s"+s.setNum(k)+".Pp1.y, кН");   graph_sP_red->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
        case 10:    graph_sP_red->setData(it,s_Pp1_z[k]);   graph_sP_red->setName("s"+s.setNum(k)+".Pp1.z, кН");   graph_sP_red->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
        case 11:    graph_sP_red->setData(it,s_Pp2_x[k]);   graph_sP_red->setName("s"+s.setNum(k)+".Pp2.x, кН");   graph_sP_red->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
        case 12:    graph_sP_red->setData(it,s_Pp2_y[k]);   graph_sP_red->setName("s"+s.setNum(k)+".Pp2.y, кН");   graph_sP_red->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
        case 13:    graph_sP_red->setData(it,s_Pp2_z[k]);   graph_sP_red->setName("s"+s.setNum(k)+".Pp2.z, кН");   graph_sP_red->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    }
    switch (ui->combo_sP_green->currentIndex())
    {
    case  0:    graph_sP_green->setVisible(false);        graph_sP_green->setName("");            graph_sP_green->setVisible(false);    break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(false);
    case  1:    graph_sP_green->setData(it,s_Pa[k]   );   graph_sP_green->setName("s"+s.setNum(k)+".Pa   , кН");   graph_sP_green->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    case  2:    graph_sP_green->setData(it,s_Pa1[k]  );   graph_sP_green->setName("s"+s.setNum(k)+".Pa1  , кН");   graph_sP_green->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    case  3:    graph_sP_green->setData(it,s_Pa2[k]  );   graph_sP_green->setName("s"+s.setNum(k)+".Pa2  , кН");   graph_sP_green->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    case  4:    graph_sP_green->setData(it,s_Pap[k]  );   graph_sP_green->setName("s"+s.setNum(k)+".P_пр , кН");   graph_sP_green->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    case  5:    graph_sP_green->setData(it,s_Pp_x[k] );   graph_sP_green->setName("s"+s.setNum(k)+".Pp.x , кН");   graph_sP_green->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    case  6:    graph_sP_green->setData(it,s_Pp_y[k] );   graph_sP_green->setName("s"+s.setNum(k)+".Pp.y , кН");   graph_sP_green->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    case  7:    graph_sP_green->setData(it,s_Pp_z[k] );   graph_sP_green->setName("s"+s.setNum(k)+".Pp.z , кН");   graph_sP_green->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    case  8:    graph_sP_green->setData(it,s_Pp1_x[k]);   graph_sP_green->setName("s"+s.setNum(k)+".Pp1.x, кН");   graph_sP_green->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    case  9:    graph_sP_green->setData(it,s_Pp1_y[k]);   graph_sP_green->setName("s"+s.setNum(k)+".Pp1.y, кН");   graph_sP_green->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    case 10:    graph_sP_green->setData(it,s_Pp1_z[k]);   graph_sP_green->setName("s"+s.setNum(k)+".Pp1.z, кН");   graph_sP_green->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    case 11:    graph_sP_green->setData(it,s_Pp2_x[k]);   graph_sP_green->setName("s"+s.setNum(k)+".Pp2.x, кН");   graph_sP_green->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    case 12:    graph_sP_green->setData(it,s_Pp2_y[k]);   graph_sP_green->setName("s"+s.setNum(k)+".Pp2.y, кН");   graph_sP_green->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    case 13:    graph_sP_green->setData(it,s_Pp2_z[k]);   graph_sP_green->setName("s"+s.setNum(k)+".Pp2.z, кН");   graph_sP_green->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    }
    switch (ui->combo_sP_blue-> currentIndex())
    {
    case  0:    graph_sP_blue->setVisible(false);        graph_sP_blue->setName("");            graph_sP_blue->setVisible(false);    break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft,0)->setVisible(false);
    case  1:    graph_sP_blue->setData(it,s_Pa[k]   );   graph_sP_blue->setName("s"+s.setNum(k)+".Pa   , кН");   graph_sP_blue->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    case  2:    graph_sP_blue->setData(it,s_Pa1[k]  );   graph_sP_blue->setName("s"+s.setNum(k)+".Pa1  , кН");   graph_sP_blue->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    case  3:    graph_sP_blue->setData(it,s_Pa2[k]  );   graph_sP_blue->setName("s"+s.setNum(k)+".Pa2  , кН");   graph_sP_blue->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    case  4:    graph_sP_blue->setData(it,s_Pap[k]  );   graph_sP_blue->setName("s"+s.setNum(k)+".P_пр , кН");   graph_sP_blue->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    case  5:    graph_sP_blue->setData(it,s_Pp_x[k] );   graph_sP_blue->setName("s"+s.setNum(k)+".Pp.x , кН");   graph_sP_blue->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    case  6:    graph_sP_blue->setData(it,s_Pp_y[k] );   graph_sP_blue->setName("s"+s.setNum(k)+".Pp.y , кН");   graph_sP_blue->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    case  7:    graph_sP_blue->setData(it,s_Pp_z[k] );   graph_sP_blue->setName("s"+s.setNum(k)+".Pp.z , кН");   graph_sP_blue->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    case  8:    graph_sP_blue->setData(it,s_Pp1_x[k]);   graph_sP_blue->setName("s"+s.setNum(k)+".Pp1.x, кН");   graph_sP_blue->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    case  9:    graph_sP_blue->setData(it,s_Pp1_y[k]);   graph_sP_blue->setName("s"+s.setNum(k)+".Pp1.y, кН");   graph_sP_blue->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    case 10:    graph_sP_blue->setData(it,s_Pp1_z[k]);   graph_sP_blue->setName("s"+s.setNum(k)+".Pp1.z, кН");   graph_sP_blue->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    case 11:    graph_sP_blue->setData(it,s_Pp2_x[k]);   graph_sP_blue->setName("s"+s.setNum(k)+".Pp2.x, кН");   graph_sP_blue->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    case 12:    graph_sP_blue->setData(it,s_Pp2_y[k]);   graph_sP_blue->setName("s"+s.setNum(k)+".Pp2.y, кН");   graph_sP_blue->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    case 13:    graph_sP_blue->setData(it,s_Pp2_z[k]);   graph_sP_blue->setName("s"+s.setNum(k)+".Pp2.z, кН");   graph_sP_blue->setVisible(true);     break;//ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft ,0)->setVisible(true);
    }

    ///graph.setData MAINROTOR
    if(ui->pushButton_mainrotor->isChecked())
    {
        setdata_to_plot_mr( ui->combo_mr_red,   graph_mr_red,   l);
        setdata_to_plot_mr( ui->combo_mr_green, graph_mr_green, l);
        setdata_to_plot_mr( ui->combo_mr_blue,  graph_mr_blue,  l);
        setdata_to_plot_mr( ui->combo_mr_black, graph_mr_black, l);
    }

    ///graph.setData TAILROTOR
    if(ui->pushButton_tailrotor->isChecked() && out.tr.mount)
    {
        setdata_to_plot_tr( ui->combo_mr_red,   graph_mr_red,   l);
        setdata_to_plot_tr( ui->combo_mr_green, graph_mr_green, l);
        setdata_to_plot_tr( ui->combo_mr_blue,  graph_mr_blue,  l);
        setdata_to_plot_tr( ui->combo_mr_black, graph_mr_black, l);
    }

    ///graph.setData PUSHROTOR
    if(ui->pushButton_pushrotor->isChecked() && out.pr.mount)
    {
        setdata_to_plot_pr( ui->combo_mr_red,   graph_mr_red,   l);
        setdata_to_plot_pr( ui->combo_mr_green, graph_mr_green, l);
        setdata_to_plot_pr( ui->combo_mr_blue,  graph_mr_blue,  l);
        setdata_to_plot_pr( ui->combo_mr_black, graph_mr_black, l);
    }

    ///rescale & replot
    {
    //plot_helicopter/////////////////////////////////////////////////////////
        //ui->plot_helicopter->xAxis->setTickVector(xlabels);
        //ui->plot_helicopter->axisRect()->axis(QCPAxis::atBottom,0)->setAutoTickCount(xlabels.size()-2);
        //ui->plot_helicopter->axisRect()->axis(QCPAxis::atBottom,0)->setSubTickLength(0,0);
    ui->plot_helicopter->rescaleAxes();//обновили шкалы
    ui->plot_helicopter->replot();     //обновили всё

    //plot_shassi/////////////////////////////////////////////////////////////
    ui->plot_shassi-> rescaleAxes(); //обновили шкалы
    ui->plot_shassi-> axisRect()->axis(QCPAxis::atLeft ,0)->setRange(ui->Edit_sP_min->text().toInt(), ui->Edit_sP_max->text().toInt());
    int ntick = (ui->Edit_sP_max->text().toInt() - ui->Edit_sP_min->text().toInt());
    ntick = (ntick==0)?(10):(ntick);
    if(ntick>110)
        ui->plot_shassi-> axisRect()->axis(QCPAxis::atLeft ,0)->setAutoTickCount(0.2*(ntick-1));
    else if(ntick>60)
        ui->plot_shassi-> axisRect()->axis(QCPAxis::atLeft ,0)->setAutoTickCount(0.5*(ntick-1));
    else if(ntick>10)
        ui->plot_shassi-> axisRect()->axis(QCPAxis::atLeft ,0)->setAutoTickCount(1*(ntick-1));
    else
        ui->plot_shassi-> axisRect()->axis(QCPAxis::atLeft ,0)->setAutoTickCount(1.9*(ntick-1));
    ui->plot_shassi-> axisRect()->axis(QCPAxis::atLeft ,0)->setSubTickLength(0,0);
    ui->plot_shassi-> axisRect()->axis(QCPAxis::atLeft ,0)->setTickLength(2,2);
    ui->plot_shassi-> axisRect()->axis(QCPAxis::atRight,1)->setRange(ui->Edit_sh_min->text().toInt(), ui->Edit_sh_max->text().toInt());
    ntick = 0.1*(ui->Edit_sh_max->text().toInt() - ui->Edit_sh_min->text().toInt());
    ntick = (ntick==0)?(10):(ntick);
    ui->plot_shassi-> axisRect()->axis(QCPAxis::atRight,1)->setAutoTickCount(ntick-2);
    ui->plot_shassi-> xAxis->setLabel("время, с");
    ui->plot_shassi-> replot();      //обновили всё

    //plot_mainrotor//////////////////////////////////////////////////////////
    ui->plot_mainrotor->rescaleAxes();//обновили шкалы
    ui->plot_mainrotor->replot();     //обновили всё
    }
}
void MainWindow::setdata_to_plot_mr(QComboBox *cb, QCPGraph *graph, int l)
{
    QString s;  QVector <double> x;
    graph->clearData();
    if(ui->radioButton_mrXtime->isChecked()) {  x=it;      ui->plot_mainrotor->xAxis->setLabel("время, с");       }
    if(ui->radioButton_mrXazim->isChecked()) {  x=mr_psi0; ui->plot_mainrotor->xAxis->setLabel("азимут, градус"); }
    if(ini.calcmode==4)  ui->plot_mainrotor->xAxis->setLabel("скорость, км/ч");
    switch (cb->currentIndex())
    {
    case  0:    graph->setVisible(false);          graph->setName("");                              graph->setVisible(false);   break;
    case  1:    graph->setData(x,mr_psi0      );   graph->setName("psi0  ");                        graph->setVisible(true);    break;
    case  2:    graph->setData(x,mr_fi0       );   graph->setName("fi0, грд");                      graph->setVisible(true);    break;
    case  3:    graph->setData(x,mr_chi       );   graph->setName("chi, грд");                      graph->setVisible(true);    break;
    case  4:    graph->setData(x,mr_eta       );   graph->setName("eta, грд");                      graph->setVisible(true);    break;
    case  5:    graph->setVisible(false);          graph->setName("");                              graph->setVisible(false);   break;
    case  6:    graph->setData(x,mr_H_sr      );   graph->setName("H_cp, Н");                       graph->setVisible(true);    break;
    case  7:    graph->setData(x,mr_T_sr      );   graph->setName("T_cp, Н");                       graph->setVisible(true);    break;
    case  8:    graph->setData(x,mr_S_sr      );   graph->setName("S_cp, Н");                       graph->setVisible(true);    break;
    case  9:    graph->setData(x,mr_Mx_sr     );   graph->setName("Mhx_cp, Н·м");                   graph->setVisible(true);    break;
    case 10:    graph->setData(x,mr_My_sr     );   graph->setName("Mhy_cp, Н·м");                   graph->setVisible(true);    break;
    case 11:    graph->setData(x,mr_Mz_sr     );   graph->setName("Mhz_cp, Н·м");                   graph->setVisible(true);    break;
    case 12:    graph->setData(x,mr_Fhx_sum   );   graph->setName("Fhx_s, Н");                      graph->setVisible(true);    break;
    case 13:    graph->setData(x,mr_Fhy_sum   );   graph->setName("Fhy_s, Н");                      graph->setVisible(true);    break;
    case 14:    graph->setData(x,mr_Fhz_sum   );   graph->setName("Fhz_s, Н");                      graph->setVisible(true);    break;
    case 15:    graph->setData(x,mr_Mhx_sum   );   graph->setName("Mhx_s, Н·м");                    graph->setVisible(true);    break;
    case 16:    graph->setData(x,mr_Mhy_sum   );   graph->setName("Mhy_s, Н·м");                    graph->setVisible(true);    break;
    case 17:    graph->setData(x,mr_Mhz_sum   );   graph->setName("Mhz_s, Н·м");                    graph->setVisible(true);    break;
    case 18:    graph->setVisible(false);          graph->setName("");                              graph->setVisible(false);   break;
    case 19:    graph->setData(x,mr_psi[l]    );   graph->setName("["+s.setNum(l)+"] psi");         graph->setVisible(true);    break;
    case 20:    graph->setData(x,mr_fi7[l]    );   graph->setName("["+s.setNum(l)+"] fi07");        graph->setVisible(true);    break;
    case 21:    graph->setData(x,mr_dfi7[l]   );   graph->setName("["+s.setNum(l)+"] dfi07");       graph->setVisible(true);    break;
    case 22:    graph->setData(x,mr_gama[l]   );   graph->setName("["+s.setNum(l)+"] gama");        graph->setVisible(true);    break;
    case 23:    graph->setData(x,mr_dgama[l]  );   graph->setName("["+s.setNum(l)+"] dgama");       graph->setVisible(true);    break;
    case 24:    graph->setData(x,mr_d2gama[l] );   graph->setName("["+s.setNum(l)+"] d2gama");      graph->setVisible(true);    break;
    case 25:    graph->setData(x,mr_beta[l]   );   graph->setName("["+s.setNum(l)+"] beta");        graph->setVisible(true);    break;
    case 26:    graph->setData(x,mr_dbeta[l]  );   graph->setName("["+s.setNum(l)+"] dbeta");       graph->setVisible(true);    break;
    case 27:    graph->setData(x,mr_d2beta[l] );   graph->setName("["+s.setNum(l)+"] d2beta");      graph->setVisible(true);    break;
    case 28:    graph->setData(x,mr_ksi[l]    );   graph->setName("["+s.setNum(l)+"] ksi");         graph->setVisible(true);    break;
    case 29:    graph->setData(x,mr_dksi[l]   );   graph->setName("["+s.setNum(l)+"] dksi");        graph->setVisible(true);    break;
    case 30:    graph->setData(x,mr_d2ksi[l]  );   graph->setName("["+s.setNum(l)+"] d2ksi");       graph->setVisible(true);    break;
    case 31:    graph->setVisible(false);          graph->setName("");                              graph->setVisible(false);   break;
    case 32:    graph->setData(x, mr_Fxh[l]   );   graph->setName("["+s.setNum(l)+"] Fhx_1, Н·м");  graph->setVisible(true);    break;
    case 33:    graph->setData(x, mr_Fyh[l]   );   graph->setName("["+s.setNum(l)+"] Fhy_1, Н·м");  graph->setVisible(true);    break;
    case 34:    graph->setData(x, mr_Fzh[l]   );   graph->setName("["+s.setNum(l)+"] Fhz_1, Н·м");  graph->setVisible(true);    break;
    case 35:    graph->setData(x, mr_Mxh[l]   );   graph->setName("["+s.setNum(l)+"] Mhx_1, Н·м");  graph->setVisible(true);    break;
    case 36:    graph->setData(x, mr_Myh[l]   );   graph->setName("["+s.setNum(l)+"] Mhy_1, Н·м");  graph->setVisible(true);    break;
    case 37:    graph->setData(x, mr_Mzh[l]   );   graph->setName("["+s.setNum(l)+"] Mhz_1, Н·м");  graph->setVisible(true);    break;
    case 38:    graph->setData(x, mr_H1[l]    );   graph->setName("["+s.setNum(l)+"] H1, Н");       graph->setVisible(true);    break;
    case 39:    graph->setData(x, mr_S1[l]    );   graph->setName("["+s.setNum(l)+"] S1, Н");       graph->setVisible(true);    break;
    case 40:    graph->setData(x, mr_T1[l]    );   graph->setName("["+s.setNum(l)+"] T1, Н");       graph->setVisible(true);    break;
    case 41:    graph->setData(x, mr_Q1[l]    );   graph->setName("["+s.setNum(l)+"] Q1, Н");       graph->setVisible(true);    break;
    case 42:    graph->setData(x, mr_Ma1[l]   );   graph->setName("["+s.setNum(l)+"] Ma1, Н");      graph->setVisible(true);    break;
    case 43:    graph->setData(x, mr_Nr[l]    );   graph->setName("["+s.setNum(l)+"] Nr, Н");       graph->setVisible(true);    break;
    case 44:    graph->setData(x, mr_Nf[l]    );   graph->setName("["+s.setNum(l)+"] Nf, Н");       graph->setVisible(true);    break;
    case 45:    graph->setData(x, mr_P1[l]    );   graph->setName("["+s.setNum(l)+"] P1, Н");       graph->setVisible(true);    break;
    case 46:    graph->setData(x, mr_P2[l]    );   graph->setName("["+s.setNum(l)+"] P2, Н");       graph->setVisible(true);    break;
    case 47:    graph->setData(x, mr_K1[l]    );   graph->setName("["+s.setNum(l)+"] K1, Н");       graph->setVisible(true);    break;
    case 48:    graph->setData(x, mr_Fdmp[l]  );   graph->setName("["+s.setNum(l)+"] Fdmp, Н");     graph->setVisible(true);    break;
    case 49:    graph->setData(x, mr_Fy[l]    );   graph->setName("["+s.setNum(l)+"] Fy, Н");       graph->setVisible(true);    break;
    case 50:    graph->setData(x, mr_Fx[l]    );   graph->setName("["+s.setNum(l)+"] Fx, Н");       graph->setVisible(true);    break;
    case 51:    graph->setData(x, mr_Fz[l]    );   graph->setName("["+s.setNum(l)+"] Fz, Н");       graph->setVisible(true);    break;
    case 52:    graph->setData(x, mr_M1T[l]   );   graph->setName("["+s.setNum(l)+"] M1T,  Н·м");   graph->setVisible(true);    break;
    case 53:    graph->setData(x, mr_M1Q[l]   );   graph->setName("["+s.setNum(l)+"] M1Q,  Н·м");   graph->setVisible(true);    break;
    case 54:    graph->setData(x, mr_M1Gx[l]  );   graph->setName("["+s.setNum(l)+"] M1Gx, Н·м");   graph->setVisible(true);    break;
    case 55:    graph->setData(x, mr_M1Gy[l]  );   graph->setName("["+s.setNum(l)+"] M1Gy, Н·м");   graph->setVisible(true);    break;
    case 56:    graph->setData(x, mr_M1K[l]   );   graph->setName("["+s.setNum(l)+"] M1K,  Н·м");   graph->setVisible(true);    break;
    case 57:    graph->setData(x, mr_M1Cx[l]  );   graph->setName("["+s.setNum(l)+"] M1Cx, Н·м");   graph->setVisible(true);    break;
    case 58:    graph->setData(x, mr_M1Cy[l]  );   graph->setName("["+s.setNum(l)+"] M1Cy, Н·м");   graph->setVisible(true);    break;
    case 59:    graph->setData(x, mr_M1Px[l]  );   graph->setName("["+s.setNum(l)+"] M1Px, Н·м");   graph->setVisible(true);    break;
    case 60:    graph->setData(x, mr_M1Py[l]  );   graph->setName("["+s.setNum(l)+"] M1Py, Н·м");   graph->setVisible(true);    break;
    case 61:    graph->setData(x, mr_M1x[l]   );   graph->setName("["+s.setNum(l)+"] M1x,  Н·м");   graph->setVisible(true);    break;
    case 62:    graph->setData(x, mr_M1y[l]   );   graph->setName("["+s.setNum(l)+"] M1y,  Н·м");   graph->setVisible(true);    break;
    case 63:    graph->setData(x, mr_M1z[l]   );   graph->setName("["+s.setNum(l)+"] M1z,  Н·м");   graph->setVisible(true);    break;
    case 64:    graph->setData(x, mr_M2T[l]   );   graph->setName("["+s.setNum(l)+"] M2T,  Н·м");   graph->setVisible(true);    break;
    case 65:    graph->setData(x, mr_M2Q[l]   );   graph->setName("["+s.setNum(l)+"] M2Q,  Н·м");   graph->setVisible(true);    break;
    case 66:    graph->setData(x, mr_M2Gx[l]  );   graph->setName("["+s.setNum(l)+"] M2Gx, Н·м");   graph->setVisible(true);    break;
    case 67:    graph->setData(x, mr_M2Gy[l]  );   graph->setName("["+s.setNum(l)+"] M2Gy, Н·м");   graph->setVisible(true);    break;
    case 68:    graph->setData(x, mr_M2K[l]   );   graph->setName("["+s.setNum(l)+"] M2K,  Н·м");   graph->setVisible(true);    break;
    case 69:    graph->setData(x, mr_M2Cx[l]  );   graph->setName("["+s.setNum(l)+"] M2Cx, Н·м");   graph->setVisible(true);    break;
    case 70:    graph->setData(x, mr_M2Cy[l]  );   graph->setName("["+s.setNum(l)+"] M2Cy, Н·м");   graph->setVisible(true);    break;
    case 71:    graph->setData(x, mr_M2Px[l]  );   graph->setName("["+s.setNum(l)+"] M2Px, Н·м");   graph->setVisible(true);    break;
    case 72:    graph->setData(x, mr_M2Py[l]  );   graph->setName("["+s.setNum(l)+"] M2Py, Н·м");   graph->setVisible(true);    break;
    case 73:    graph->setData(x, mr_M2D[l]   );   graph->setName("["+s.setNum(l)+"] M2D,  Н·м");   graph->setVisible(true);    break;
    case 74:    graph->setData(x, mr_M2x[l]   );   graph->setName("["+s.setNum(l)+"] M2x,  Н·м");   graph->setVisible(true);    break;
    case 75:    graph->setData(x, mr_M2y[l]   );   graph->setName("["+s.setNum(l)+"] M2y,  Н·м");   graph->setVisible(true);    break;
    case 76:    graph->setData(x, mr_M2z[l]   );   graph->setName("["+s.setNum(l)+"] M2z,  Н·м");   graph->setVisible(true);    break;
    case 77:    graph->setData(x, mr_M3x[l]   );   graph->setName("["+s.setNum(l)+"] M3x,  Н·м");   graph->setVisible(true);    break;
    case 78:    graph->setData(x, mr_M3y[l]   );   graph->setName("["+s.setNum(l)+"] M3y,  Н·м");   graph->setVisible(true);    break;
    case 79:    graph->setData(x, mr_M3z[l]   );   graph->setName("["+s.setNum(l)+"] M3z,  Н·м");   graph->setVisible(true);    break;
    case 80:    graph->setData(x, mr_M3Ma[l]  );   graph->setName("["+s.setNum(l)+"] M3Ma, Н·м");   graph->setVisible(true);    break;
    case 81:    graph->setData(x, mr_M3su[l]  );   graph->setName("["+s.setNum(l)+"] M3su, Н·м");   graph->setVisible(true);    break;
    case 82:    graph->setData(x, mr_M3cb[l]  );   graph->setName("["+s.setNum(l)+"] M3cb, Н·м");   graph->setVisible(true);    break;
    case 83:    graph->setData(x, mr_M3tr[l]  );   graph->setName("["+s.setNum(l)+"] M3tr, Н·м");   graph->setVisible(true);    break;
    case 84:    graph->setData(x, mr_M3i[l]   );   graph->setName("["+s.setNum(l)+"] M3i,  Н·м");   graph->setVisible(true);    break;
    case 85:    graph->setData(x, mr_Fpov[l]  );   graph->setName("["+s.setNum(l)+"] Fpov, Н");     graph->setVisible(true);    break;
    }
}
void MainWindow::setdata_to_plot_tr(QComboBox *cb, QCPGraph *graph, int l)
{
    QString s;
    graph->clearData();
    QVector<double> x;
    if(ui->radioButton_mrXtime->isChecked()) {  x=it;      ui->plot_mainrotor->xAxis->setLabel("время, с");       }
    if(ui->radioButton_mrXazim->isChecked()) {  x=tr_psi0; ui->plot_mainrotor->xAxis->setLabel("азимут, градус"); }

    switch (cb->currentIndex())
    {
    case  0:    graph->setVisible(false);          graph->setName("");                              graph->setVisible(false);   break;
    case  1:    graph->setData(x, tr_psi0     );   graph->setName("psi0  ");                        graph->setVisible(true);    break;
    case  2:    graph->setData(x, tr_fi0      );   graph->setName("fi0, грд");                      graph->setVisible(true);    break;
    case  3:    graph->setVisible(false);          graph->setName("");                              graph->setVisible(false);   break;
    case  4:    graph->setVisible(false);          graph->setName("");                              graph->setVisible(false);   break;
    case  5:    graph->setVisible(false);          graph->setName("");                              graph->setVisible(false);   break;
    case  6:    graph->setData(x, tr_H_sr     );   graph->setName("H_cp, Н");                       graph->setVisible(true);    break;
    case  7:    graph->setData(x, tr_T_sr     );   graph->setName("T_cp, Н");                       graph->setVisible(true);    break;
    case  8:    graph->setData(x, tr_S_sr     );   graph->setName("S_cp, Н");                       graph->setVisible(true);    break;
    case  9:    graph->setData(x, tr_Mx_sr    );   graph->setName("Mhx_cp, Н·м");                   graph->setVisible(true);    break;
    case 10:    graph->setData(x, tr_My_sr    );   graph->setName("Mhy_cp, Н·м");                   graph->setVisible(true);    break;
    case 11:    graph->setData(x, tr_Mz_sr    );   graph->setName("Mhz_cp, Н·м");                   graph->setVisible(true);    break;
    case 12:    graph->setData(x, tr_Fhx_sum  );   graph->setName("Fhx_s, Н");                      graph->setVisible(true);    break;
    case 13:    graph->setData(x, tr_Fhy_sum  );   graph->setName("Fhy_s, Н");                      graph->setVisible(true);    break;
    case 14:    graph->setData(x, tr_Fhz_sum  );   graph->setName("Fhz_s, Н");                      graph->setVisible(true);    break;
    case 15:    graph->setData(x, tr_Mhx_sum  );   graph->setName("Mhx_s, Н·м");                    graph->setVisible(true);    break;
    case 16:    graph->setData(x, tr_Mhy_sum  );   graph->setName("Mhy_s, Н·м");                    graph->setVisible(true);    break;
    case 17:    graph->setData(x, tr_Mhz_sum  );   graph->setName("Mhz_s, Н·м");                    graph->setVisible(true);    break;
    case 18:    graph->setVisible(false);          graph->setName("");                              graph->setVisible(false);   break;
    case 19:    graph->setData(x, tr_psi[l]   );   graph->setName("["+s.setNum(l)+"] psi");         graph->setVisible(true);    break;
    case 20:    graph->setData(x, tr_fi7[l]   );   graph->setName("["+s.setNum(l)+"] fi07");        graph->setVisible(true);    break;
    case 21:    graph->setData(x, tr_dfi7[l]  );   graph->setName("["+s.setNum(l)+"] dfi07");       graph->setVisible(true);    break;
    case 22:    graph->setData(x, tr_gama[l]  );   graph->setName("["+s.setNum(l)+"] gama");        graph->setVisible(true);    break;
    case 23:    graph->setData(x, tr_dgama[l] );   graph->setName("["+s.setNum(l)+"] dgama");       graph->setVisible(true);    break;
    case 24:    graph->setData(x, tr_d2gama[l]);   graph->setName("["+s.setNum(l)+"] d2gama");      graph->setVisible(true);    break;
    case 25:    graph->setData(x, tr_beta[l]  );   graph->setName("["+s.setNum(l)+"] beta");        graph->setVisible(true);    break;
    case 26:    graph->setData(x, tr_dbeta[l] );   graph->setName("["+s.setNum(l)+"] dbeta");       graph->setVisible(true);    break;
    case 27:    graph->setData(x, tr_d2beta[l]);   graph->setName("["+s.setNum(l)+"] d2beta");      graph->setVisible(true);    break;
    case 28:    graph->setData(x, tr_ksi[l]   );   graph->setName("["+s.setNum(l)+"] ksi");         graph->setVisible(true);    break;
    case 29:    graph->setData(x, tr_dksi[l]  );   graph->setName("["+s.setNum(l)+"] dksi");        graph->setVisible(true);    break;
    case 30:    graph->setData(x, tr_d2ksi[l] );   graph->setName("["+s.setNum(l)+"] d2ksi");       graph->setVisible(true);    break;
    case 31:    graph->setVisible(false);          graph->setName("");                              graph->setVisible(false);   break;
    case 32:    graph->setData(x, tr_Fxh[l]   );   graph->setName("["+s.setNum(l)+"] Fhx_1, Н·м");  graph->setVisible(true);    break;
    case 33:    graph->setData(x, tr_Fyh[l]   );   graph->setName("["+s.setNum(l)+"] Fhy_1, Н·м");  graph->setVisible(true);    break;
    case 34:    graph->setData(x, tr_Fzh[l]   );   graph->setName("["+s.setNum(l)+"] Fhz_1, Н·м");  graph->setVisible(true);    break;
    case 35:    graph->setData(x, tr_Mxh[l]   );   graph->setName("["+s.setNum(l)+"] Mhx_1, Н·м");  graph->setVisible(true);    break;
    case 36:    graph->setData(x, tr_Myh[l]   );   graph->setName("["+s.setNum(l)+"] Mhy_1, Н·м");  graph->setVisible(true);    break;
    case 37:    graph->setData(x, tr_Mzh[l]   );   graph->setName("["+s.setNum(l)+"] Mhz_1, Н·м");  graph->setVisible(true);    break;
    case 38:    graph->setData(x, tr_H1[l]    );   graph->setName("["+s.setNum(l)+"] H1, Н");       graph->setVisible(true);    break;
    case 39:    graph->setData(x, tr_S1[l]    );   graph->setName("["+s.setNum(l)+"] S1, Н");       graph->setVisible(true);    break;
    case 40:    graph->setData(x, tr_T1[l]    );   graph->setName("["+s.setNum(l)+"] T1, Н");       graph->setVisible(true);    break;
    case 41:    graph->setData(x, tr_Q1[l]    );   graph->setName("["+s.setNum(l)+"] Q1, Н");       graph->setVisible(true);    break;
    case 42:    graph->setData(x, tr_Ma1[l]   );   graph->setName("["+s.setNum(l)+"] Ma1, Н");      graph->setVisible(true);    break;
    case 43:    graph->setData(x, tr_Nr[l]    );   graph->setName("["+s.setNum(l)+"] Nr, Н");       graph->setVisible(true);    break;
    case 44:    graph->setData(x, tr_Nf[l]    );   graph->setName("["+s.setNum(l)+"] Nf, Н");       graph->setVisible(true);    break;
    case 45:    graph->setData(x, tr_P1[l]    );   graph->setName("["+s.setNum(l)+"] P1, Н");       graph->setVisible(true);    break;
    case 46:    graph->setData(x, tr_P2[l]    );   graph->setName("["+s.setNum(l)+"] P2, Н");       graph->setVisible(true);    break;
    case 47:    graph->setData(x, tr_K1[l]    );   graph->setName("["+s.setNum(l)+"] K1, Н");       graph->setVisible(true);    break;
    case 48:    graph->setData(x, tr_Fdmp[l]  );   graph->setName("["+s.setNum(l)+"] Fdmp, Н");     graph->setVisible(true);    break;
    case 49:    graph->setData(x, tr_Fy[l]    );   graph->setName("["+s.setNum(l)+"] Fy, Н");       graph->setVisible(true);    break;
    case 50:    graph->setData(x, tr_Fx[l]    );   graph->setName("["+s.setNum(l)+"] Fx, Н");       graph->setVisible(true);    break;
    case 51:    graph->setData(x, tr_Fz[l]    );   graph->setName("["+s.setNum(l)+"] Fz, Н");       graph->setVisible(true);    break;
    case 52:    graph->setData(x, tr_M1T[l]   );   graph->setName("["+s.setNum(l)+"] M1T,  Н·м");   graph->setVisible(true);    break;
    case 53:    graph->setData(x, tr_M1Q[l]   );   graph->setName("["+s.setNum(l)+"] M1Q,  Н·м");   graph->setVisible(true);    break;
    case 54:    graph->setData(x, tr_M1Gx[l]  );   graph->setName("["+s.setNum(l)+"] M1Gx, Н·м");   graph->setVisible(true);    break;
    case 55:    graph->setData(x, tr_M1Gy[l]  );   graph->setName("["+s.setNum(l)+"] M1Gy, Н·м");   graph->setVisible(true);    break;
    case 56:    graph->setData(x, tr_M1K[l]   );   graph->setName("["+s.setNum(l)+"] M1K,  Н·м");   graph->setVisible(true);    break;
    case 57:    graph->setData(x, tr_M1Cx[l]  );   graph->setName("["+s.setNum(l)+"] M1Cx, Н·м");   graph->setVisible(true);    break;
    case 58:    graph->setData(x, tr_M1Cy[l]  );   graph->setName("["+s.setNum(l)+"] M1Cy, Н·м");   graph->setVisible(true);    break;
    case 59:    graph->setData(x, tr_M1Px[l]  );   graph->setName("["+s.setNum(l)+"] M1Px, Н·м");   graph->setVisible(true);    break;
    case 60:    graph->setData(x, tr_M1Py[l]  );   graph->setName("["+s.setNum(l)+"] M1Py, Н·м");   graph->setVisible(true);    break;
    case 61:    graph->setData(x, tr_M1x[l]   );   graph->setName("["+s.setNum(l)+"] M1x,  Н·м");   graph->setVisible(true);    break;
    case 62:    graph->setData(x, tr_M1y[l]   );   graph->setName("["+s.setNum(l)+"] M1y,  Н·м");   graph->setVisible(true);    break;
    case 63:    graph->setData(x, tr_M1z[l]   );   graph->setName("["+s.setNum(l)+"] M1z,  Н·м");   graph->setVisible(true);    break;
    case 64:    graph->setData(x, tr_M2T[l]   );   graph->setName("["+s.setNum(l)+"] M2T,  Н·м");   graph->setVisible(true);    break;
    case 65:    graph->setData(x, tr_M2Q[l]   );   graph->setName("["+s.setNum(l)+"] M2Q,  Н·м");   graph->setVisible(true);    break;
    case 66:    graph->setData(x, tr_M2Gx[l]  );   graph->setName("["+s.setNum(l)+"] M2Gx, Н·м");   graph->setVisible(true);    break;
    case 67:    graph->setData(x, tr_M2Gy[l]  );   graph->setName("["+s.setNum(l)+"] M2Gy, Н·м");   graph->setVisible(true);    break;
    case 68:    graph->setData(x, tr_M2K[l]   );   graph->setName("["+s.setNum(l)+"] M2K,  Н·м");   graph->setVisible(true);    break;
    case 69:    graph->setData(x, tr_M2Cx[l]  );   graph->setName("["+s.setNum(l)+"] M2Cx, Н·м");   graph->setVisible(true);    break;
    case 70:    graph->setData(x, tr_M2Cy[l]  );   graph->setName("["+s.setNum(l)+"] M2Cy, Н·м");   graph->setVisible(true);    break;
    case 71:    graph->setData(x, tr_M2Px[l]  );   graph->setName("["+s.setNum(l)+"] M2Px, Н·м");   graph->setVisible(true);    break;
    case 72:    graph->setData(x, tr_M2Py[l]  );   graph->setName("["+s.setNum(l)+"] M2Py, Н·м");   graph->setVisible(true);    break;
    case 73:    graph->setData(x, tr_M2D[l]   );   graph->setName("["+s.setNum(l)+"] M2D,  Н·м");   graph->setVisible(true);    break;
    case 74:    graph->setData(x, tr_M2x[l]   );   graph->setName("["+s.setNum(l)+"] M2x,  Н·м");   graph->setVisible(true);    break;
    case 75:    graph->setData(x, tr_M2y[l]   );   graph->setName("["+s.setNum(l)+"] M2y,  Н·м");   graph->setVisible(true);    break;
    case 76:    graph->setData(x, tr_M2z[l]   );   graph->setName("["+s.setNum(l)+"] M2z,  Н·м");   graph->setVisible(true);    break;
    case 77:    graph->setData(x, tr_M3x[l]   );   graph->setName("["+s.setNum(l)+"] M3x,  Н·м");   graph->setVisible(true);    break;
    case 78:    graph->setData(x, tr_M3y[l]   );   graph->setName("["+s.setNum(l)+"] M3y,  Н·м");   graph->setVisible(true);    break;
    case 79:    graph->setData(x, tr_M3z[l]   );   graph->setName("["+s.setNum(l)+"] M3z,  Н·м");   graph->setVisible(true);    break;
    case 80:    graph->setData(x, tr_M3Ma[l]  );   graph->setName("["+s.setNum(l)+"] M3Ma, Н·м");   graph->setVisible(true);    break;
    case 81:    graph->setData(x, tr_M3su[l]  );   graph->setName("["+s.setNum(l)+"] M3su, Н·м");   graph->setVisible(true);    break;
    case 82:    graph->setData(x, tr_M3cb[l]  );   graph->setName("["+s.setNum(l)+"] M3cb, Н·м");   graph->setVisible(true);    break;
    case 83:    graph->setData(x, tr_M3tr[l]  );   graph->setName("["+s.setNum(l)+"] M3tr, Н·м");   graph->setVisible(true);    break;
    case 84:    graph->setData(x, tr_M3i[l]   );   graph->setName("["+s.setNum(l)+"] M3i,  Н·м");   graph->setVisible(true);    break;
    case 85:    graph->setData(x, tr_Fpov[l]  );   graph->setName("["+s.setNum(l)+"] Fpov, Н");     graph->setVisible(true);    break;
    }
}
void MainWindow::setdata_to_plot_pr(QComboBox *cb, QCPGraph *graph, int l)
{
    QString s;
    graph->clearData();
    QVector<double> x;
    if(ui->radioButton_mrXtime->isChecked()) {  x=it;      ui->plot_mainrotor->xAxis->setLabel("время, с");       }
    if(ui->radioButton_mrXazim->isChecked()) {  x=pr_psi0; ui->plot_mainrotor->xAxis->setLabel("азимут, градус"); }

    switch (cb->currentIndex())
    {
    case  0:    graph->setVisible(false);          graph->setName("");                              graph->setVisible(false);   break;
    case  1:    graph->setData(x, pr_psi0     );   graph->setName("psi0  ");                        graph->setVisible(true);    break;
    case  2:    graph->setData(x, pr_fi0      );   graph->setName("fi0, грд");                      graph->setVisible(true);    break;
    case  3:    graph->setVisible(false);          graph->setName("");                              graph->setVisible(false);   break;
    case  4:    graph->setVisible(false);          graph->setName("");                              graph->setVisible(false);   break;
    case  5:    graph->setVisible(false);          graph->setName("");                              graph->setVisible(false);   break;
    case  6:    graph->setData(x, pr_H_sr     );   graph->setName("H_cp, Н");                       graph->setVisible(true);    break;
    case  7:    graph->setData(x, pr_T_sr     );   graph->setName("T_cp, Н");                       graph->setVisible(true);    break;
    case  8:    graph->setData(x, pr_S_sr     );   graph->setName("S_cp, Н");                       graph->setVisible(true);    break;
    case  9:    graph->setData(x, pr_Mx_sr    );   graph->setName("Mhx_cp, Н·м");                   graph->setVisible(true);    break;
    case 10:    graph->setData(x, pr_My_sr    );   graph->setName("Mhy_cp, Н·м");                   graph->setVisible(true);    break;
    case 11:    graph->setData(x, pr_Mz_sr    );   graph->setName("Mhz_cp, Н·м");                   graph->setVisible(true);    break;
    case 12:    graph->setData(x, pr_Fhx_sum  );   graph->setName("Fhx_s, Н");                      graph->setVisible(true);    break;
    case 13:    graph->setData(x, pr_Fhy_sum  );   graph->setName("Fhy_s, Н");                      graph->setVisible(true);    break;
    case 14:    graph->setData(x, pr_Fhz_sum  );   graph->setName("Fhz_s, Н");                      graph->setVisible(true);    break;
    case 15:    graph->setData(x, pr_Mhx_sum  );   graph->setName("Mhx_s, Н·м");                    graph->setVisible(true);    break;
    case 16:    graph->setData(x, pr_Mhy_sum  );   graph->setName("Mhy_s, Н·м");                    graph->setVisible(true);    break;
    case 17:    graph->setData(x, pr_Mhz_sum  );   graph->setName("Mhz_s, Н·м");                    graph->setVisible(true);    break;
    case 18:    graph->setVisible(false);          graph->setName("");                              graph->setVisible(false);   break;
    case 19:    graph->setData(x, pr_psi[l]   );   graph->setName("["+s.setNum(l)+"] psi");         graph->setVisible(true);    break;
    case 20:    graph->setData(x, pr_fi7[l]   );   graph->setName("["+s.setNum(l)+"] fi07");        graph->setVisible(true);    break;
    case 21:    graph->setData(x, pr_dfi7[l]  );   graph->setName("["+s.setNum(l)+"] dfi07");       graph->setVisible(true);    break;
    case 22:    graph->setData(x, pr_gama[l]  );   graph->setName("["+s.setNum(l)+"] gama");        graph->setVisible(true);    break;
    case 23:    graph->setData(x, pr_dgama[l] );   graph->setName("["+s.setNum(l)+"] dgama");       graph->setVisible(true);    break;
    case 24:    graph->setData(x, pr_d2gama[l]);   graph->setName("["+s.setNum(l)+"] d2gama");      graph->setVisible(true);    break;
    case 25:    graph->setData(x, pr_beta[l]  );   graph->setName("["+s.setNum(l)+"] beta");        graph->setVisible(true);    break;
    case 26:    graph->setData(x, pr_dbeta[l] );   graph->setName("["+s.setNum(l)+"] dbeta");       graph->setVisible(true);    break;
    case 27:    graph->setData(x, pr_d2beta[l]);   graph->setName("["+s.setNum(l)+"] d2beta");      graph->setVisible(true);    break;
    case 28:    graph->setData(x, pr_ksi[l]   );   graph->setName("["+s.setNum(l)+"] ksi");         graph->setVisible(true);    break;
    case 29:    graph->setData(x, pr_dksi[l]  );   graph->setName("["+s.setNum(l)+"] dksi");        graph->setVisible(true);    break;
    case 30:    graph->setData(x, pr_d2ksi[l] );   graph->setName("["+s.setNum(l)+"] d2ksi");       graph->setVisible(true);    break;
    case 31:    graph->setVisible(false);          graph->setName("");                              graph->setVisible(false);   break;
    case 32:    graph->setData(x, pr_Fxh[l]   );   graph->setName("["+s.setNum(l)+"] Fhx_1, Н·м");  graph->setVisible(true);    break;
    case 33:    graph->setData(x, pr_Fyh[l]   );   graph->setName("["+s.setNum(l)+"] Fhy_1, Н·м");  graph->setVisible(true);    break;
    case 34:    graph->setData(x, pr_Fzh[l]   );   graph->setName("["+s.setNum(l)+"] Fhz_1, Н·м");  graph->setVisible(true);    break;
    case 35:    graph->setData(x, pr_Mxh[l]   );   graph->setName("["+s.setNum(l)+"] Mhx_1, Н·м");  graph->setVisible(true);    break;
    case 36:    graph->setData(x, pr_Myh[l]   );   graph->setName("["+s.setNum(l)+"] Mhy_1, Н·м");  graph->setVisible(true);    break;
    case 37:    graph->setData(x, pr_Mzh[l]   );   graph->setName("["+s.setNum(l)+"] Mhz_1, Н·м");  graph->setVisible(true);    break;
    case 38:    graph->setData(x, pr_H1[l]    );   graph->setName("["+s.setNum(l)+"] H1, Н");       graph->setVisible(true);    break;
    case 39:    graph->setData(x, pr_S1[l]    );   graph->setName("["+s.setNum(l)+"] S1, Н");       graph->setVisible(true);    break;
    case 40:    graph->setData(x, pr_T1[l]    );   graph->setName("["+s.setNum(l)+"] T1, Н");       graph->setVisible(true);    break;
    case 41:    graph->setData(x, pr_Q1[l]    );   graph->setName("["+s.setNum(l)+"] Q1, Н");       graph->setVisible(true);    break;
    case 42:    graph->setData(x, pr_Ma1[l]   );   graph->setName("["+s.setNum(l)+"] Ma1, Н");      graph->setVisible(true);    break;
    case 43:    graph->setData(x, pr_Nr[l]    );   graph->setName("["+s.setNum(l)+"] Nr, Н");       graph->setVisible(true);    break;
    case 44:    graph->setData(x, pr_Nf[l]    );   graph->setName("["+s.setNum(l)+"] Nf, Н");       graph->setVisible(true);    break;
    case 45:    graph->setData(x, pr_P1[l]    );   graph->setName("["+s.setNum(l)+"] P1, Н");       graph->setVisible(true);    break;
    case 46:    graph->setData(x, pr_P2[l]    );   graph->setName("["+s.setNum(l)+"] P2, Н");       graph->setVisible(true);    break;
    case 47:    graph->setData(x, pr_K1[l]    );   graph->setName("["+s.setNum(l)+"] K1, Н");       graph->setVisible(true);    break;
    case 48:    graph->setData(x, pr_Fdmp[l]  );   graph->setName("["+s.setNum(l)+"] Fdmp, Н");     graph->setVisible(true);    break;
    case 49:    graph->setData(x, pr_Fy[l]    );   graph->setName("["+s.setNum(l)+"] Fy, Н");       graph->setVisible(true);    break;
    case 50:    graph->setData(x, pr_Fx[l]    );   graph->setName("["+s.setNum(l)+"] Fx, Н");       graph->setVisible(true);    break;
    case 51:    graph->setData(x, pr_Fz[l]    );   graph->setName("["+s.setNum(l)+"] Fz, Н");       graph->setVisible(true);    break;
    case 52:    graph->setData(x, pr_M1T[l]   );   graph->setName("["+s.setNum(l)+"] M1T,  Н·м");   graph->setVisible(true);    break;
    case 53:    graph->setData(x, pr_M1Q[l]   );   graph->setName("["+s.setNum(l)+"] M1Q,  Н·м");   graph->setVisible(true);    break;
    case 54:    graph->setData(x, pr_M1Gx[l]  );   graph->setName("["+s.setNum(l)+"] M1Gx, Н·м");   graph->setVisible(true);    break;
    case 55:    graph->setData(x, pr_M1Gy[l]  );   graph->setName("["+s.setNum(l)+"] M1Gy, Н·м");   graph->setVisible(true);    break;
    case 56:    graph->setData(x, pr_M1K[l]   );   graph->setName("["+s.setNum(l)+"] M1K,  Н·м");   graph->setVisible(true);    break;
    case 57:    graph->setData(x, pr_M1Cx[l]  );   graph->setName("["+s.setNum(l)+"] M1Cx, Н·м");   graph->setVisible(true);    break;
    case 58:    graph->setData(x, pr_M1Cy[l]  );   graph->setName("["+s.setNum(l)+"] M1Cy, Н·м");   graph->setVisible(true);    break;
    case 59:    graph->setData(x, pr_M1Px[l]  );   graph->setName("["+s.setNum(l)+"] M1Px, Н·м");   graph->setVisible(true);    break;
    case 60:    graph->setData(x, pr_M1Py[l]  );   graph->setName("["+s.setNum(l)+"] M1Py, Н·м");   graph->setVisible(true);    break;
    case 61:    graph->setData(x, pr_M1x[l]   );   graph->setName("["+s.setNum(l)+"] M1x,  Н·м");   graph->setVisible(true);    break;
    case 62:    graph->setData(x, pr_M1y[l]   );   graph->setName("["+s.setNum(l)+"] M1y,  Н·м");   graph->setVisible(true);    break;
    case 63:    graph->setData(x, pr_M1z[l]   );   graph->setName("["+s.setNum(l)+"] M1z,  Н·м");   graph->setVisible(true);    break;
    case 64:    graph->setData(x, pr_M2T[l]   );   graph->setName("["+s.setNum(l)+"] M2T,  Н·м");   graph->setVisible(true);    break;
    case 65:    graph->setData(x, pr_M2Q[l]   );   graph->setName("["+s.setNum(l)+"] M2Q,  Н·м");   graph->setVisible(true);    break;
    case 66:    graph->setData(x, pr_M2Gx[l]  );   graph->setName("["+s.setNum(l)+"] M2Gx, Н·м");   graph->setVisible(true);    break;
    case 67:    graph->setData(x, pr_M2Gy[l]  );   graph->setName("["+s.setNum(l)+"] M2Gy, Н·м");   graph->setVisible(true);    break;
    case 68:    graph->setData(x, pr_M2K[l]   );   graph->setName("["+s.setNum(l)+"] M2K,  Н·м");   graph->setVisible(true);    break;
    case 69:    graph->setData(x, pr_M2Cx[l]  );   graph->setName("["+s.setNum(l)+"] M2Cx, Н·м");   graph->setVisible(true);    break;
    case 70:    graph->setData(x, pr_M2Cy[l]  );   graph->setName("["+s.setNum(l)+"] M2Cy, Н·м");   graph->setVisible(true);    break;
    case 71:    graph->setData(x, pr_M2Px[l]  );   graph->setName("["+s.setNum(l)+"] M2Px, Н·м");   graph->setVisible(true);    break;
    case 72:    graph->setData(x, pr_M2Py[l]  );   graph->setName("["+s.setNum(l)+"] M2Py, Н·м");   graph->setVisible(true);    break;
    case 73:    graph->setData(x, pr_M2D[l]   );   graph->setName("["+s.setNum(l)+"] M2D,  Н·м");   graph->setVisible(true);    break;
    case 74:    graph->setData(x, pr_M2x[l]   );   graph->setName("["+s.setNum(l)+"] M2x,  Н·м");   graph->setVisible(true);    break;
    case 75:    graph->setData(x, pr_M2y[l]   );   graph->setName("["+s.setNum(l)+"] M2y,  Н·м");   graph->setVisible(true);    break;
    case 76:    graph->setData(x, pr_M2z[l]   );   graph->setName("["+s.setNum(l)+"] M2z,  Н·м");   graph->setVisible(true);    break;
    case 77:    graph->setData(x, pr_M3x[l]   );   graph->setName("["+s.setNum(l)+"] M3x,  Н·м");   graph->setVisible(true);    break;
    case 78:    graph->setData(x, pr_M3y[l]   );   graph->setName("["+s.setNum(l)+"] M3y,  Н·м");   graph->setVisible(true);    break;
    case 79:    graph->setData(x, pr_M3z[l]   );   graph->setName("["+s.setNum(l)+"] M3z,  Н·м");   graph->setVisible(true);    break;
    case 80:    graph->setData(x, pr_M3Ma[l]  );   graph->setName("["+s.setNum(l)+"] M3Ma, Н·м");   graph->setVisible(true);    break;
    case 81:    graph->setData(x, pr_M3su[l]  );   graph->setName("["+s.setNum(l)+"] M3su, Н·м");   graph->setVisible(true);    break;
    case 82:    graph->setData(x, pr_M3cb[l]  );   graph->setName("["+s.setNum(l)+"] M3cb, Н·м");   graph->setVisible(true);    break;
    case 83:    graph->setData(x, pr_M3tr[l]  );   graph->setName("["+s.setNum(l)+"] M3tr, Н·м");   graph->setVisible(true);    break;
    case 84:    graph->setData(x, pr_M3i[l]   );   graph->setName("["+s.setNum(l)+"] M3i,  Н·м");   graph->setVisible(true);    break;
    case 85:    graph->setData(x, pr_Fpov[l]  );   graph->setName("["+s.setNum(l)+"] Fpov, Н");     graph->setVisible(true);    break;
    }
}
void MainWindow::airframeplotprep()
{
    //HELICOPTER PLOT===========================================================================
    QStringList combolist_h;
    {
    combolist_h << ""                                     // 0
    << "O.x,  положение начала СКС, мм"                   // 1
    << "O.y,  положение начала СКС, мм"                   // 2
    << "O.z,  положение начала СКС, мм"                   // 3
    << "kren, угол крена, грд"                            // 4
    << "rysk, угол рыскания, грд"                         // 5
    << "tang, угол тангажа, грд"                          // 6
    << "vn.x, скорость ЦТ вертолета в СКН, м/с"           // 7
    << "vn.y, скорость ЦТ вертолета в СКН, м/с"           // 8
    << "vn.z, скорость ЦТ вертолета в СКН, м/с"           // 9
    << "v.x,  скорость ЦТ вертолета в СКС, м/с"           //
    << "v.y,  скорость ЦТ вертолета в СКС, м/с"           //
    << "v.z,  скорость ЦТ вертолета в СКС, м/с"           //
    << "w.k,  угл.скорость фюзеляжа в СКС, рад/с"         //10
    << "w.r,  угл.скорость фюзеляжа в СКС, рад/с"         //11
    << "w.t,  угл.скорость фюзеляжа в СКС, рад/с"         //12
    << "a.x,  ускорение ЦТ вертолета в СКС, м/с2"         //13
    << "a.y,  ускорение ЦТ вертолета в СКС, м/с2"         //14
    << "a.z,  ускорение ЦТ вертолета в СКС, м/с2"         //15
    << "n.x,  перегрузка в заданной точке планера, -"     //16
    << "n.y,  перегрузка в заданной точке планера, -"     //17
    << "n.z,  перегрузка в заданной точке планера, -"     //18
    << "F.x,  результирующая сила в ЦТ (в СКС), Н"        //19
    << "F.y,  результирующая сила в ЦТ (в СКС), Н"        //20
    << "F.z,  результирующая сила в ЦТ (в СКС), Н"        //21
    << "M.x,  результирующая момент в ЦТ (в СКС), Н·м"    //22
    << "M.y,  результирующая момент в ЦТ (в СКС), Н·м"    //23
    << "M.z,  результирующая момент в ЦТ (в СКС), Н·м"    //24
    << "Fadp_x,  сумм.аэродин.сила от планера приведенная к ЦТ (в СКС), Н"     //25
    << "Fadp_y,  сумм.аэродин.сила от планера приведенная к ЦТ (в СКС), Н"     //26
    << "Fadp_z,  сумм.аэродин.сила от планера приведенная к ЦТ (в СКС), Н"     //27
    << "Madp_x,  сумм.аэродин.момент от планера приведенный к ЦТ (в СКС), Н·м" //28
    << "Madp_y,  сумм.аэродин.момент от планера приведенный к ЦТ (в СКС), Н·м" //29
    << "Madp_z,  сумм.аэродин.момент от планера приведенный к ЦТ (в СКС), Н·м" //30
    << "Ffus_x,  аэродин.сила на фюзеляже приведенная к ЦТ (в СКС), Н"         //31
    << "Ffus_y,  аэродин.сила на фюзеляже приведенная к ЦТ (в СКС), Н"         //32
    << "Ffus_z,  аэродин.сила на фюзеляже приведенная к ЦТ (в СКС), Н"         //33
    << "Mfus_x,  аэродин.момент на фюзеляже приведенный к ЦТ (в СКС), Н·м"     //34
    << "Mfus_y,  аэродин.момент на фюзеляже приведенный к ЦТ (в СКС), Н·м"     //35
    << "Mfus_z,  аэродин.момент на фюзеляже приведенный к ЦТ (в СКС), Н·м"     //36
    << "Fstab_x, аэродин.сила на стабилизаторе приведенная к ЦТ (в СКС), Н"    //37
    << "Fstab_y, аэродин.сила на стабилизаторе приведенная к ЦТ (в СКС), Н"    //38
    << "Fstab_z, аэродин.сила на стабилизаторе приведенная к ЦТ (в СКС), Н"    //39
    << "Fkeel_x, аэродин.сила на киле приведенная к ЦТ (в СКС), Н"             //40
    << "Fkeel_y, аэродин.сила на киле приведенная к ЦТ (в СКС), Н"             //41
    << "Fkeel_z, аэродин.сила на киле приведенная к ЦТ (в СКС), Н"             //42
    << "fi0MR угол общего шага несущего винта, грд"         //43
    << "chi,  угол отклонения АП в продольном канале, грд"  //44
    << "eta,  угол отклонения АП в поперечном канале, грд"  //45
    << "fi0TR угол общего шага рулевого винта, грд"         //46
    ;
    ui->combo_h_red  ->addItems(combolist_h);   ui->combo_h_red  ->setCurrentIndex( 2);
    ui->combo_h_green->addItems(combolist_h);   ui->combo_h_green->setCurrentIndex( 4);
    ui->combo_h_blue ->addItems(combolist_h);   ui->combo_h_blue ->setCurrentIndex( 6);
    ui->combo_h_black->addItems(combolist_h);   ui->combo_h_black->setCurrentIndex(20);
    }
    //добавляем недостающие оси Y
    ui->plot_helicopter->axisRect()->addAxis(QCPAxis::atLeft);
    ui->plot_helicopter->axisRect()->addAxis(QCPAxis::atLeft);
    ui->plot_helicopter->axisRect()->addAxis(QCPAxis::atRight);

    ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft ,0)->setTickLabelColor(QColor("#cc0000"));//red
    graph_h_red  = ui->plot_helicopter->addGraph(ui->plot_helicopter->axisRect()->axis(QCPAxis::atBottom), ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,0));
    graph_h_red  ->setPen(QPen(QColor("#cc0000"), 1.5, Qt::SolidLine));

    ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft ,1)->setTickLabelColor(QColor("#008800"));//green
    graph_h_green= ui->plot_helicopter->addGraph(ui->plot_helicopter->axisRect()->axis(QCPAxis::atBottom), ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,1));
    graph_h_green->setPen(QPen(QColor("#008800"), 1.5, Qt::DotLine));

    ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft ,2)->setTickLabelColor(QColor("#0000ff"));//blue
    graph_h_blue = ui->plot_helicopter->addGraph(ui->plot_helicopter->axisRect()->axis(QCPAxis::atBottom), ui->plot_helicopter->axisRect()->axis(QCPAxis::atLeft,2));
    graph_h_blue ->setPen(QPen(QColor("#0000ff"), 1.5, Qt::DashLine));

    ui->plot_helicopter->axisRect()->axis(QCPAxis::atRight ,1)->setTickLabelColor(QColor("#000000"));//black
    graph_h_black= ui->plot_helicopter->addGraph(ui->plot_helicopter->axisRect()->axis(QCPAxis::atBottom), ui->plot_helicopter->axisRect()->axis(QCPAxis::atRight,1));
    graph_h_black->setPen(QPen(QColor("#000000"), 1.5));

    ui->plot_helicopter-> xAxis ->grid()->        setPen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->plot_helicopter-> yAxis ->grid()->        setPen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->plot_helicopter-> yAxis2->grid()->        setPen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->plot_helicopter-> xAxis ->grid()-> setSubGridPen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->plot_helicopter-> yAxis ->grid()-> setSubGridPen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->plot_helicopter-> yAxis2->grid()-> setSubGridPen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->plot_helicopter-> yAxis ->grid()->setZeroLinePen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->plot_helicopter-> xAxis ->grid()->setSubGridVisible(1);
    ui->plot_helicopter-> yAxis ->grid()->setSubGridVisible(1);
    ui->plot_helicopter-> yAxis2->grid()->setSubGridVisible(1);

    ui->plot_helicopter->legend->setVisible( true );
    ui->plot_helicopter->xAxis->setLabel("время, с  /  скорость, км/ч");
}
void MainWindow::shassiplotprep()
{
    //SHASSI PLOT==============================================================================
    {
    ui->buttonGroupSt->setId( ui->ButtonSt0, 0);    ui->ButtonSt0->setEnabled(false);
    ui->buttonGroupSt->setId( ui->ButtonSt1, 1);    ui->ButtonSt1->setEnabled(false);
    ui->buttonGroupSt->setId( ui->ButtonSt2, 2);    ui->ButtonSt2->setEnabled(false);
    ui->buttonGroupSt->setId( ui->ButtonSt3, 3);    ui->ButtonSt3->setEnabled(false);
    ui->buttonGroupSt->setId( ui->ButtonSt4, 4);    ui->ButtonSt4->setEnabled(false);

    QStringList combolist_sh,combolist_sP;
    combolist_sh << ""                                            // 0
    << "ha,    сумм.обжатие амортстойки, мм"                      // 1
    << "ha1,   обжатие штока камеры №1, мм"                       // 2
    << "ha2,   обжатие штока камеры №2, мм"                       // 3
    << "hp.x,  обжатие пневматика, мм"                            // 4
    << "hp.y,  обжатие пневматика, мм"                            // 5
    << "hp.z,  обжатие пневматика, мм"                            // 6
    << "hp1.x, обжатие пневматика правого в паре, мм"             // 7
    << "hp1.y, обжатие пневматика правого в паре, мм"             // 8
    << "hp1.z, обжатие пневматика правого в паре, мм"             // 9
    << "hp2.x, обжатие пневматика левого в паре, мм"              //10
    << "hp2.y, обжатие пневматика левого в паре, мм"              //11
    << "hp2.z, обжатие пневматика левого в паре, мм"              //12
    ;
    combolist_sP << ""                                            // 0
    << "Pa,    результ.усилие на штоках амортстойки, кН"          // 1
    << "Pa1,   усилие на штоке камеры №1, кН"                     // 2
    << "Pa2,   усилие на штоке камеры №2, кН"                     // 3
    << "P_пр,  приведенное к пневматику усилие на амортстойке, кН"// 4
    << "Pp.x,  усилие на пневматике, кН"                          // 5
    << "Pp.y,  усилие на пневматике, кН"                          // 6
    << "Pp.z,  усилие на пневматике, кН"                          // 7
    << "Pp1.x, усилие на пневматике правом в паре, кН"            // 8
    << "Pp1.y, усилие на пневматике правом в паре, кН"            // 9
    << "Pp1.z, усилие на пневматике правом в паре, кН"            //10
    << "Pp2.x, усилие на пневматике левом в паре, кН"             //11
    << "Pp2.y, усилие на пневматике левом в паре, кН"             //12
    << "Pp2.z, усилие на пневматике левом в паре, кН"             //13
    ;
    ui->combo_sh_red  ->addItems(combolist_sh);   ui->combo_sh_red  ->setCurrentIndex(1);
    ui->combo_sh_green->addItems(combolist_sh);   ui->combo_sh_green->setCurrentIndex(0);
    ui->combo_sh_blue ->addItems(combolist_sh);   ui->combo_sh_blue ->setCurrentIndex(5);
    ui->combo_sP_red  ->addItems(combolist_sP);   ui->combo_sP_red  ->setCurrentIndex(1);
    ui->combo_sP_green->addItems(combolist_sP);   ui->combo_sP_green->setCurrentIndex(0);
    ui->combo_sP_blue ->addItems(combolist_sP);   ui->combo_sP_blue ->setCurrentIndex(6);
    }
    ui->plot_shassi->axisRect()->addAxis(QCPAxis::atRight);

    graph_sP_red  = ui->plot_shassi->addGraph(ui->plot_shassi->axisRect()->axis(QCPAxis::atBottom), ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft,0));
    graph_sP_red  ->setPen(QPen(QColor("#cc0000"), 2.0, Qt::DotLine));
    graph_sP_green= ui->plot_shassi->addGraph(ui->plot_shassi->axisRect()->axis(QCPAxis::atBottom), ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft,0));
    graph_sP_green->setPen(QPen(QColor("#008800"), 2.0, Qt::DotLine));
    graph_sP_blue = ui->plot_shassi->addGraph(ui->plot_shassi->axisRect()->axis(QCPAxis::atBottom), ui->plot_shassi->axisRect()->axis(QCPAxis::atLeft,0));
    graph_sP_blue ->setPen(QPen(QColor("#0000ff"), 2.0, Qt::DotLine));

    graph_sh_red  = ui->plot_shassi->addGraph(ui->plot_shassi->axisRect()->axis(QCPAxis::atBottom), ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1));
    graph_sh_red  ->setPen(QPen(QColor("#cc0000"), 2.0));
    graph_sh_green= ui->plot_shassi->addGraph(ui->plot_shassi->axisRect()->axis(QCPAxis::atBottom), ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1));
    graph_sh_green->setPen(QPen(QColor("#008800"), 2.0));
    graph_sh_blue = ui->plot_shassi->addGraph(ui->plot_shassi->axisRect()->axis(QCPAxis::atBottom), ui->plot_shassi->axisRect()->axis(QCPAxis::atRight,1));
    graph_sh_blue ->setPen(QPen(QColor("#0000ff"), 2.0));

    ui->plot_shassi-> xAxis ->grid()->        setPen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->plot_shassi-> yAxis ->grid()->        setPen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->plot_shassi-> yAxis2->grid()->        setPen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->plot_shassi-> xAxis ->grid()-> setSubGridPen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->plot_shassi-> yAxis ->grid()-> setSubGridPen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->plot_shassi-> yAxis2->grid()-> setSubGridPen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->plot_shassi-> yAxis ->grid()->setZeroLinePen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->plot_shassi-> xAxis ->grid()->setSubGridVisible(1);
    ui->plot_shassi-> yAxis ->grid()->setSubGridVisible(0);
    ui->plot_shassi-> yAxis2->grid()->setSubGridVisible(0);

    ui->plot_shassi->legend->setVisible( true );
    ui->plot_shassi-> xAxis->setLabel("время, с");
}
void MainWindow::abtestplotprep()
{
    //ABTEST PLOT===========================================================================
    ui->abtestplot->setVisible(true);
    ui->abtestplot->axisRect()->addAxis(QCPAxis::atRight);
    ui->abtestplot->axisRect()->axis(QCPAxis::atLeft ,0)->setTickLabelColor(QColor("#cc0000"));

    graph_ab_red1 = ui->abtestplot->addGraph(ui->abtestplot->axisRect()->axis(QCPAxis::atBottom), ui->abtestplot->axisRect()->axis(QCPAxis::atLeft,0));
    graph_ab_red1 ->setPen(QPen(QColor("#cc0000"), 3.0, Qt::DashLine));///DashLine
    graph_ab_red2 = ui->abtestplot->addGraph(ui->abtestplot->axisRect()->axis(QCPAxis::atBottom), ui->abtestplot->axisRect()->axis(QCPAxis::atLeft,0));
    graph_ab_red2 ->setPen(QPen(QColor("#cc0000"), 3.0, Qt::DotLine));///DotLine

    graph_ab_green1= ui->abtestplot->addGraph(ui->abtestplot->axisRect()->axis(QCPAxis::atBottom), ui->abtestplot->axisRect()->axis(QCPAxis::atLeft,0));
    graph_ab_green1->setPen(QPen(QColor("#008800"), 2.0, Qt::DashLine));
    graph_ab_green2= ui->abtestplot->addGraph(ui->abtestplot->axisRect()->axis(QCPAxis::atBottom), ui->abtestplot->axisRect()->axis(QCPAxis::atLeft,0));
    graph_ab_green2->setPen(QPen(QColor("#008800"), 2.0, Qt::DotLine));

    graph_ab_blue1 = ui->abtestplot->addGraph(ui->abtestplot->axisRect()->axis(QCPAxis::atBottom), ui->abtestplot->axisRect()->axis(QCPAxis::atLeft,0));
    graph_ab_blue1 ->setPen(QPen(QColor("#0000ff"), 2.0, Qt::DashLine));
    graph_ab_blue2 = ui->abtestplot->addGraph(ui->abtestplot->axisRect()->axis(QCPAxis::atBottom), ui->abtestplot->axisRect()->axis(QCPAxis::atLeft,0));
    graph_ab_blue2 ->setPen(QPen(QColor("#0000ff"), 2.0, Qt::DotLine));

    graph_ab_Pg1_1 = ui->abtestplot->addGraph(ui->abtestplot->axisRect()->axis(QCPAxis::atBottom), ui->abtestplot->axisRect()->axis(QCPAxis::atLeft,0));
    graph_ab_Pg1_1 ->setPen(QPen(QColor("#006600"), 1.0, Qt::DashLine));
    graph_ab_Pg1_2 = ui->abtestplot->addGraph(ui->abtestplot->axisRect()->axis(QCPAxis::atBottom), ui->abtestplot->axisRect()->axis(QCPAxis::atLeft,0));
    graph_ab_Pg1_2 ->setPen(QPen(QColor("#006600"), 1.0, Qt::DotLine));
    //
    graph_ab_Pg2_1 = ui->abtestplot->addGraph(ui->abtestplot->axisRect()->axis(QCPAxis::atBottom), ui->abtestplot->axisRect()->axis(QCPAxis::atLeft,0));
    graph_ab_Pg2_1 ->setPen(QPen(QColor("#0000bb"), 1.0, Qt::DashLine));
    graph_ab_Pg2_2 = ui->abtestplot->addGraph(ui->abtestplot->axisRect()->axis(QCPAxis::atBottom), ui->abtestplot->axisRect()->axis(QCPAxis::atLeft,0));
    graph_ab_Pg2_2 ->setPen(QPen(QColor("#0000bb"), 1.0, Qt::DotLine));
    //
    graph_ab_Ph1_1 = ui->abtestplot->addGraph(ui->abtestplot->axisRect()->axis(QCPAxis::atBottom), ui->abtestplot->axisRect()->axis(QCPAxis::atLeft,0));
    graph_ab_Ph1_1 ->setPen(QPen(QColor("#006600"), 1.0, Qt::DashLine));
    graph_ab_Ph1_2 = ui->abtestplot->addGraph(ui->abtestplot->axisRect()->axis(QCPAxis::atBottom), ui->abtestplot->axisRect()->axis(QCPAxis::atLeft,0));
    graph_ab_Ph1_2 ->setPen(QPen(QColor("#006600"), 1.0, Qt::DotLine));
    //
    graph_ab_Ph2_1 = ui->abtestplot->addGraph(ui->abtestplot->axisRect()->axis(QCPAxis::atBottom), ui->abtestplot->axisRect()->axis(QCPAxis::atLeft,0));
    graph_ab_Ph2_1 ->setPen(QPen(QColor("#0000bb"), 1.0, Qt::DashLine));
    graph_ab_Ph2_2 = ui->abtestplot->addGraph(ui->abtestplot->axisRect()->axis(QCPAxis::atBottom), ui->abtestplot->axisRect()->axis(QCPAxis::atLeft,0));
    graph_ab_Ph2_2 ->setPen(QPen(QColor("#0000bb"), 1.0, Qt::DotLine));
    //
    graph_ab_Pf1_1 = ui->abtestplot->addGraph(ui->abtestplot->axisRect()->axis(QCPAxis::atBottom), ui->abtestplot->axisRect()->axis(QCPAxis::atLeft,0));
    graph_ab_Pf1_1 ->setPen(QPen(QColor("#006600"), 1.0, Qt::DashLine));
    graph_ab_Pf1_2 = ui->abtestplot->addGraph(ui->abtestplot->axisRect()->axis(QCPAxis::atBottom), ui->abtestplot->axisRect()->axis(QCPAxis::atLeft,0));
    graph_ab_Pf1_2 ->setPen(QPen(QColor("#006600"), 1.0, Qt::DotLine));
    //
    graph_ab_Pf2_1 = ui->abtestplot->addGraph(ui->abtestplot->axisRect()->axis(QCPAxis::atBottom), ui->abtestplot->axisRect()->axis(QCPAxis::atLeft,0));
    graph_ab_Pf2_1 ->setPen(QPen(QColor("#0000bb"), 1.0, Qt::DashLine));
    graph_ab_Pf2_2 = ui->abtestplot->addGraph(ui->abtestplot->axisRect()->axis(QCPAxis::atBottom), ui->abtestplot->axisRect()->axis(QCPAxis::atLeft,0));
    graph_ab_Pf2_2 ->setPen(QPen(QColor("#0000bb"), 1.0, Qt::DotLine));

    ui->abtestplot->axisRect()->axis(QCPAxis::atRight ,1)->setTickLabelColor(QColor("#000000"));//black
    graph_ab_blk11= ui->abtestplot->addGraph(ui->abtestplot->axisRect()->axis(QCPAxis::atBottom), ui->abtestplot->axisRect()->axis(QCPAxis::atRight,1));
    graph_ab_blk11->setPen(QPen(QColor("#000000"), 2.0, Qt::DashLine));
    graph_ab_blk12= ui->abtestplot->addGraph(ui->abtestplot->axisRect()->axis(QCPAxis::atBottom), ui->abtestplot->axisRect()->axis(QCPAxis::atRight,1));
    graph_ab_blk12->setPen(QPen(QColor("#000000"), 2.0, Qt::DotLine));
    graph_ab_blk21= ui->abtestplot->addGraph(ui->abtestplot->axisRect()->axis(QCPAxis::atBottom), ui->abtestplot->axisRect()->axis(QCPAxis::atRight,1));
    graph_ab_blk21->setPen(QPen(QColor("#000000"), 2.0, Qt::DashLine));
    graph_ab_blk22= ui->abtestplot->addGraph(ui->abtestplot->axisRect()->axis(QCPAxis::atBottom), ui->abtestplot->axisRect()->axis(QCPAxis::atRight,1));
    graph_ab_blk22->setPen(QPen(QColor("#000000"), 2.0, Qt::DotLine));

    ui->abtestplot-> xAxis ->grid()->        setPen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->abtestplot-> yAxis ->grid()->        setPen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->abtestplot-> yAxis2->grid()->        setPen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->abtestplot-> xAxis ->grid()-> setSubGridPen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->abtestplot-> yAxis ->grid()-> setSubGridPen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->abtestplot-> yAxis2->grid()-> setSubGridPen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->abtestplot-> yAxis ->grid()->setZeroLinePen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->abtestplot-> xAxis ->grid()->setSubGridVisible(1);
    ui->abtestplot-> yAxis ->grid()->setSubGridVisible(0);
    ui->abtestplot-> yAxis2->grid()->setSubGridVisible(0);

    ui->abtestplot->legend->setVisible( true );
    ui->abtestplot->xAxis->setLabel("суммарное обжатие штоков, мм");
    ui->abtestplot->replot();

    ui->checkBox_at_F  ->setPalette(QPalette(QColor("#cc0000")));
    ui->checkBox_at_F  ->setPalette(QPalette(QColor("#cc0000")));
    ui->checkBox_at_F1 ->setPalette(QPalette(QColor("#008800")));
    ui->checkBox_at_F1 ->setPalette(QPalette(QColor("#008800")));
    ui->checkBox_at_Fg1->setPalette(QPalette(QColor("#008800")));
    ui->checkBox_at_Fg1->setPalette(QPalette(QColor("#008800")));
    ui->checkBox_at_Fh1->setPalette(QPalette(QColor("#008800")));
    ui->checkBox_at_Fh1->setPalette(QPalette(QColor("#008800")));
    ui->checkBox_at_Ff1->setPalette(QPalette(QColor("#008800")));
    ui->checkBox_at_Ff1->setPalette(QPalette(QColor("#008800")));
    ui->checkBox_at_F2 ->setPalette(QPalette(QColor("#0000bb")));
    ui->checkBox_at_F2 ->setPalette(QPalette(QColor("#0000bb")));
    ui->checkBox_at_Fg2->setPalette(QPalette(QColor("#0000bb")));
    ui->checkBox_at_Fg2->setPalette(QPalette(QColor("#0000bb")));
    ui->checkBox_at_Fh2->setPalette(QPalette(QColor("#0000bb")));
    ui->checkBox_at_Fh2->setPalette(QPalette(QColor("#0000bb")));
    ui->checkBox_at_Ff2->setPalette(QPalette(QColor("#0000bb")));
    ui->checkBox_at_Ff2->setPalette(QPalette(QColor("#0000bb")));
}
void MainWindow::rotorsplotprep()
{
    //MAINROTOR PLOT===========================================================================
    ui->buttonGroupBl->setId( ui->ButtonBl0, 0);    //ui->ButtonBl0->setEnabled(true);
    ui->buttonGroupBl->setId( ui->ButtonBl1, 1);    //ui->ButtonBl1->setEnabled(false);
    ui->buttonGroupBl->setId( ui->ButtonBl2, 2);    //ui->ButtonBl2->setEnabled(false);
    ui->buttonGroupBl->setId( ui->ButtonBl3, 3);    //ui->ButtonBl3->setEnabled(false);
    ui->buttonGroupBl->setId( ui->ButtonBl4, 4);    //ui->ButtonBl4->setEnabled(false);
    ui->buttonGroupBl->setId( ui->ButtonBl5, 5);    //ui->ButtonBl5->setEnabled(false);

    QStringList combolist_mr;
    {
    combolist_mr << ""                                                              // 0
    << "psi0   \t- положение втулки (направление оси Х СКВ) в СКВ, грд"             // 1
    << "fiОШ   \t- угол общего шага, грд"                                           // 2
    << "chi    \t- угол отклонения АП в продольном канале, грд"                     // 3
    << "eta    \t- угол отклонения АП в поперечном канале, грд"                     // 4
    << ""                                                                           // 5
    << "H_cp   \t- средняя за оборот продольная сила винта (СКВ), Н"                // 6
    << "T_cp   \t- средняя за оборот тяга винта (СКВ), Н"                           // 7
    << "S_cp   \t- средняя за оборот боковая сила винта (СКВ), Н"                   // 8
    << "Mx_cp  \t- средний за оборот суммарный момент отн.оX винта (СКВ), Н·м"      // 9
    << "My_cp  \t- средний за оборот крутящий момент на валу винта (СКВ), Н·м"      //10
    << "Mz_cp  \t- средний за оборот суммарный момент отн.оZ винта (СКВ), Н·м"      //11
    << "Fhx_s  \t- X-проекция сумм.силы (СКВ) в центре втулки от всех лопастей, Н"  //12
    << "Fhy_s  \t- Y-проекция сумм.силы (СКВ) в центре втулки от всех лопастей, Н"  //13
    << "Fhz_s  \t- Z-проекция сумм.силы (СКВ) в центре втулки от всех лопастей, Н"  //14
    << "Mhx_s  \t- сумм.момент отн.оX (СКВ) в центре втулки от всех лопастей, Н·м"  //15
    << "Mhy_s  \t- сумм.момент отн.оY (СКВ) в центре втулки от всех лопастей, Н·м"  //16
    << "Mhz_s  \t- сумм.момент отн.оZ (СКВ) в центре втулки от всех лопастей, Н·м"  //17
    << ""                                                                           //18
    << "psi    \t- направление оси лопасти в СКВ, грд"                              //19
    << "fi07   \t- угол установки сечения 07R, грд"                                 //20
    << "dfi07  \t- угл.скорость установки сечения 07R, грд"                         //21
    << "gama   \t- угол закрутки сечения 07R отн.ОШ, грд"                           //22
    << "dgama  \t- угл.скорость закрутки сечения 07R отн.ОШ"                        //23
    << "d2gama \t- угл.ускорение закрутки сечения 07R отн.ОШ,*/c"                   //24
    << "beta   \t- угол взмаха лопасти, грд"                                        //25
    << "dbeta  \t- угл.скорость взмаха, рад[взм]/рад[об]"                           //26
    << "d2beta \t- угл.ускорение взмаха, рад[взм]/рад2[об]"                         //27
    << "ksi    \t- угол качания лопасти, грд"                                       //28
    << "dksi   \t- угл.скорость качания, рад[взм]/рад[об]"                          //29
    << "d2ksi  \t- угл.ускорение качания, рад[взм]/рад2[об]"                        //30
    << ""                                                                           //31
    << "Fhx_1  \t- X-проекция силы (СКВ) в центре втулки от одной лопасти, Н"       //32
    << "Fhy_1  \t- Y-проекция силы (СКВ) в центре втулки от одной лопасти, Н"       //33
    << "Fhz_1  \t- Z-проекция силы (СКВ) в центре втулки от одной лопасти, Н"       //34
    << "Mhx_1  \t- момент отн.оX (СКВ) в центре втулки от одной лопасти, Н·м"       //35
    << "Mhy_1  \t- момент отн.оY (СКВ) в центре втулки от одной лопасти, Н·м"       //36
    << "Mhz_1  \t- момент отн.оZ (СКВ) в центре втулки от одной лопасти, Н·м"       //37
    << "H1     \t- продольная сила лопасти на втулке (СКВ), Н"                      //38
    << "S1     \t- боковая сила лопасти на втулке (СКВ), Н"                         //39
    << "T1     \t- сила тяги лопасти (СКЛ), Н"                                      //40
    << "Q1     \t- сила сопротивления лопасти (СКЛ), Н"                             //41
    << "Ma1    \t- аэродинамический момент лопасти отн.ОШ, Н·м"                     //42
    << "N1     \t- центробежная сила от вращения винта (СКЛ), Н"                    //43
    << "Nf     \t- центробежная сила от маховых движений (СКЛ), Н"                  //44
    << "P1     \t- сила инерции взмаха (СКЛ), Н"                                    //45
    << "P2     \t- сила инерции качания (СКЛ), Н"                                   //46
    << "Fк     \t- сила Кориолиса в плоскости вращения (СКЛ), Н"                    //47
    << "Fdmp   \t- сила на штоке демпфера лопасти, Н"                               //48
    << "Fy     \t- проекция на oY(СКЛ) суммарной силы в шарнирах, Н"                //49
    << "Fx     \t- проекция на oX(СКЛ) суммарной силы в шарнирах, Н"                //50
    << "Fz     \t- проекция на oZ(СКЛ) суммарной силы в шарнирах, Н"                //51
    << "M1T    \t- момент отн.оX в ГШ силы тяги (СКЛ), Н·м"                         //52
    << "M1Q    \t- момент отн.оY в ГШ силы сопротивления лопасти dQ (СКЛ), Н·м"     //53
    << "M1Gx   \t- момент отн.оX в ГШ веса лопасти (СКЛ), Н·м"                      //54
    << "M1Gy   \t- момент отн.оY в ГШ веса лопасти (СКЛ), Н·м"                      //55
    << "M1K    \t- момент отн.оY в ГШ силы Кориолиса (СКЛ), Н·м"                    //56
    << "M1Cx   \t- момент отн.оX в ГШ центробежной силы (СКЛ), Н·м"                 //57
    << "M1Cy   \t- момент отн.оY в ГШ центробежной силы (СКЛ), Н·м"                 //58
    << "M1Px   \t- момент отн.оX отн.ГШ силы инерции взмаха (СКЛ), Н·м"             //59
    << "M1Py   \t- момент отн.оY отн.ГШ силы инерции взмаха (СКЛ), Н·м"             //60
    << "M1x    \t- сумм.момент отн.оX сил в ГШ отн.X СКЛ, Н·м"                      //61
    << "M1y    \t- сумм.момент отн.оY сил в ГШ отн.Y СКЛ, Н·м"                      //62
    << "M1z    \t- сумм.момент отн.оZ сил в ГШ отн.Z СКЛ, Н·м"                      //63
    << "M2T    \t- момент отн.оX в ВШ силы тяги (СКЛ), Н·м"                         //64
    << "M2Q    \t- момент отн.оY в ВШ силы сопротивления лопасти dQ (СКЛ), Н·м"     //65
    << "M2Gx   \t- момент отн.оX в ВШ веса лопасти (СКЛ), Н·м"                      //66
    << "M2Gy   \t- момент отн.оY в ВШ веса лопасти (СКЛ), Н·м"                      //67
    << "M2K    \t- момент отн.оY в ВШ силы Кориолиса (СКЛ), Н·м"                    //68
    << "M2Cx   \t- момент отн.оX в ВШ центробежной силы (СКЛ), Н·м"                 //69
    << "M2Cy   \t- момент отн.оY в ВШ центробежной силы (СКЛ), Н·м"                 //70
    << "M2Px   \t- момент отн.оX в ВШ силы инерции (СКЛ), Н·м"                      //71
    << "M2Py   \t- момент отн.оY в ВШ силы инерции (СКЛ), Н·м"                      //72
    << "M2D    \t- момент отн.оY в ВШ силы демпфера, Н·м"                           //73
    << "M2x    \t- сумм.момент отн.оX в ВШ отн.X СКЛ, Н·м"                          //74
    << "M2y    \t- сумм.момент отн.оY в ВШ отн.Y СКЛ, Н·м"                          //75
    << "M2z    \t- сумм.момент отн.оZ в ВШ отн.Z СКЛ, Н·м"                          //76
    << "M3x    \t- сумм.момент отн.оX в OШ отн.X СКЛ, Н·м"                          //77
    << "M3y    \t- сумм.момент отн.оY в OШ отн.Y СКЛ, Н·м"                          //78
    << "M3z    \t- сумм.момент отн.оZ в OШ отн.Z СКЛ, Н·м"                          //79
    << "M3Ma   \t- момент  в OШ отн.oZ СКЛ, Н·м"                                    //80
    << "M3su   \t- момент силы в поводке(тяге) сист.управления отн.OШ (СКЛ), Н·м"   //81
    << "M3cb   \t- момент центробежных сил отн.OШ (СКЛ), Н·м"                       //82
    << "M3tr   \t- момент сил трения в OШ (СКЛ), Н·м"                               //83
    << "M3i    \t- момент сил инерции закрутки отн.OШ (СКЛ), Н·м"                   //84
    << "Fpov   \t- усилие растяжения в поводке(тяге) системы управления, Н"         //85
    ;
    ui->combo_mr_red  ->addItems(combolist_mr);   ui->combo_mr_red  ->setCurrentIndex(20);
    ui->combo_mr_green->addItems(combolist_mr);   ui->combo_mr_green->setCurrentIndex(22);
    ui->combo_mr_blue ->addItems(combolist_mr);   ui->combo_mr_blue ->setCurrentIndex(25);
    ui->combo_mr_black->addItems(combolist_mr);   ui->combo_mr_black->setCurrentIndex(0 );
    }
    //добавляем недостающие оси Y
    ui->plot_mainrotor->axisRect()->addAxis(QCPAxis::atLeft);
    ui->plot_mainrotor->axisRect()->addAxis(QCPAxis::atLeft);
    ui->plot_mainrotor->axisRect()->addAxis(QCPAxis::atRight);

    ui->plot_mainrotor->axisRect()->axis(QCPAxis::atLeft ,0)->setTickLabelColor(QColor("#cc0000"));//red
    graph_mr_red  = ui->plot_mainrotor->addGraph(ui->plot_mainrotor->axisRect()->axis(QCPAxis::atBottom), ui->plot_mainrotor->axisRect()->axis(QCPAxis::atLeft,0));
    graph_mr_red  ->setPen(QPen(QColor("#cc0000"), 1.5));
    //ui->plot_mainrotor->axisRect()->axis(QCPAxis::atLeft ,0)->setTickLabelColor(QColor("#cc0000"));//red
    //graph_mr_red2  = ui->plot_mainrotor->addGraph(ui->plot_mainrotor->axisRect()->axis(QCPAxis::atBottom), ui->plot_mainrotor->axisRect()->axis(QCPAxis::atLeft,0));
    //graph_mr_red2  ->setPen(QPen(QColor("#cc0000"), 2.0));

    ui->plot_mainrotor->axisRect()->axis(QCPAxis::atLeft ,1)->setTickLabelColor(QColor("#008800"));//green
    graph_mr_green= ui->plot_mainrotor->addGraph(ui->plot_mainrotor->axisRect()->axis(QCPAxis::atBottom), ui->plot_mainrotor->axisRect()->axis(QCPAxis::atLeft,1));
    graph_mr_green->setPen(QPen(QColor("#008800"), 1.5));
    //ui->plot_mainrotor->axisRect()->axis(QCPAxis::atLeft ,1)->setTickLabelColor(QColor("#008800"));//green
    //graph_mr_green2= ui->plot_mainrotor->addGraph(ui->plot_mainrotor->axisRect()->axis(QCPAxis::atBottom), ui->plot_mainrotor->axisRect()->axis(QCPAxis::atLeft,1));
    //graph_mr_green2->setPen(QPen(QColor("#008800"), 2.0));

    ui->plot_mainrotor->axisRect()->axis(QCPAxis::atLeft ,2)->setTickLabelColor(QColor("#0000ff"));//blue
    graph_mr_blue = ui->plot_mainrotor->addGraph(ui->plot_mainrotor->axisRect()->axis(QCPAxis::atBottom), ui->plot_mainrotor->axisRect()->axis(QCPAxis::atLeft ,2));
    graph_mr_blue ->setPen(QPen(QColor("#0000ff"), 1.5));
    //ui->plot_mainrotor->axisRect()->axis(QCPAxis::atLeft ,2)->setTickLabelColor(QColor("#0000ff"));//blue
    //graph_mr_blue2 = ui->plot_mainrotor->addGraph(ui->plot_mainrotor->axisRect()->axis(QCPAxis::atBottom), ui->plot_mainrotor->axisRect()->axis(QCPAxis::atLeft ,2));
    //graph_mr_blue2 ->setPen(QPen(QColor("#0000ff"), 2.0));

    ui->plot_mainrotor->axisRect()->axis(QCPAxis::atRight ,1)->setTickLabelColor(QColor("#000000"));//black
    graph_mr_black= ui->plot_mainrotor->addGraph(ui->plot_mainrotor->axisRect()->axis(QCPAxis::atBottom), ui->plot_mainrotor->axisRect()->axis(QCPAxis::atRight,1));
    graph_mr_black->setPen(QPen(QColor("#000000"), 1.5, Qt::DotLine));//
    //ui->plot_mainrotor->axisRect()->axis(QCPAxis::atRight ,2)->setTickLabelColor(QColor("#000000"));//black
    //graph_mr_black2= ui->plot_mainrotor->addGraph(ui->plot_mainrotor->axisRect()->axis(QCPAxis::atBottom), ui->plot_mainrotor->axisRect()->axis(QCPAxis::atRight,1));
    //graph_mr_black2->setPen(QPen(QColor("#000000"), 2.0));

    ui->plot_mainrotor-> xAxis ->grid()->        setPen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->plot_mainrotor-> yAxis ->grid()->        setPen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->plot_mainrotor-> yAxis2->grid()->        setPen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->plot_mainrotor-> xAxis ->grid()-> setSubGridPen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->plot_mainrotor-> yAxis ->grid()-> setSubGridPen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->plot_mainrotor-> yAxis2->grid()-> setSubGridPen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->plot_mainrotor-> yAxis ->grid()->setZeroLinePen(QPen(QColor(0,0,0), 0.0, Qt::DotLine));
    ui->plot_mainrotor-> xAxis ->grid()->setSubGridVisible(1);
    ui->plot_mainrotor-> yAxis ->grid()->setSubGridVisible(1);
    ui->plot_mainrotor-> yAxis2->grid()->setSubGridVisible(1);

    ui->plot_mainrotor->legend->setVisible( true );
    ui->plot_mainrotor->xAxis->setLabel("время, с");//"азимут, градус"
}
void MainWindow::on_checkBox_h_toggled(bool checked)
{
    ui->plotbox_h->setVisible(checked);
}
void MainWindow::on_checkBox_s_toggled(bool checked)
{
    ui->plotbox_s->setVisible(checked);
}
void MainWindow::on_checklabel1_clicked(bool checked)
{
    ui->plot_shassi->legend->setVisible( checked );
    ui->plot_shassi->replot();
}
void MainWindow::on_checkBox_2_clicked(bool checked)
{
    ui->plot_helicopter->legend->setVisible( checked );
    ui->plot_helicopter->replot();
}
void MainWindow::setShassiButton(int nst)//установление активности кнопок выбора стоек
{
    switch (nst)
    {
    case 3:
        ui->ButtonSt0->setEnabled(true);
        ui->ButtonSt1->setEnabled(true);
        ui->ButtonSt2->setEnabled(true);
        ui->ButtonSt3->setEnabled(false);
        ui->ButtonSt4->setEnabled(false);
    break;
    case 4:
        ui->ButtonSt0->setEnabled(true);
        ui->ButtonSt1->setEnabled(true);
        ui->ButtonSt2->setEnabled(true);
        ui->ButtonSt3->setEnabled(true);
        ui->ButtonSt4->setEnabled(false);
    break;
    case 5:
        ui->ButtonSt0->setEnabled(true);
        ui->ButtonSt1->setEnabled(true);
        ui->ButtonSt2->setEnabled(true);
        ui->ButtonSt3->setEnabled(true);
        ui->ButtonSt4->setEnabled(true);
    break;
    default:
        ui->ButtonSt0->setEnabled(false);
        ui->ButtonSt1->setEnabled(false);
        ui->ButtonSt2->setEnabled(false);
        ui->ButtonSt3->setEnabled(false);
        ui->ButtonSt4->setEnabled(false);
    break;
    }
}
void MainWindow::on_checkBox_mr_toggled(bool checked)
{
    ui->plotbox_mr->setVisible(checked);
}
void MainWindow::on_Button_replot_helicopter_clicked()
{
    write_to_plot();
}
void MainWindow::on_Button_replot_shassi_clicked()
{
    write_to_plot();
}
void MainWindow::on_ButtonSt0_clicked()
{
    MainWindow::on_Button_replot_shassi_clicked();
}
void MainWindow::on_ButtonSt1_clicked()
{
    MainWindow::on_Button_replot_shassi_clicked();
}
void MainWindow::on_ButtonSt2_clicked()
{
    MainWindow::on_Button_replot_shassi_clicked();
}
void MainWindow::on_ButtonSt3_clicked()
{
    MainWindow::on_Button_replot_shassi_clicked();
}
void MainWindow::on_ButtonSt4_clicked()
{
    MainWindow::on_Button_replot_shassi_clicked();
}
void MainWindow::on_Button_replot_rotors_clicked()
{
    write_to_plot();
}
void MainWindow::on_pushButton_mainrotor_clicked()
{
    if(out.mr.nb>0)
    {
    ui->ButtonBl0->setEnabled(out.mr.nb>0);
    ui->ButtonBl1->setEnabled(out.mr.nb>1);
    ui->ButtonBl2->setEnabled(out.mr.nb>2);
    ui->ButtonBl3->setEnabled(out.mr.nb>3);
    ui->ButtonBl4->setEnabled(out.mr.nb>4);
    ui->ButtonBl5->setEnabled(out.mr.nb>5);
    }
    else
    {
    ui->ButtonBl0->setEnabled(true);
    ui->ButtonBl1->setEnabled(false);
    ui->ButtonBl2->setEnabled(false);
    ui->ButtonBl3->setEnabled(false);
    ui->ButtonBl4->setEnabled(false);
    ui->ButtonBl5->setEnabled(false);
    }
    ui->buttonGroupBl->setId(ui->ButtonBl0, 0);
    MainWindow::on_Button_replot_rotors_clicked();
}
void MainWindow::on_pushButton_tailrotor_clicked()
{
    if(out.tr.nb>0)
    {
    ui->ButtonBl0->setEnabled(out.tr.nb>0);
    ui->ButtonBl1->setEnabled(out.tr.nb>1);
    ui->ButtonBl2->setEnabled(out.tr.nb>2);
    ui->ButtonBl3->setEnabled(out.tr.nb>3);
    ui->ButtonBl4->setEnabled(out.tr.nb>4);
    ui->ButtonBl5->setEnabled(out.tr.nb>5);
    }
    else
    {
    ui->ButtonBl0->setEnabled(true);
    ui->ButtonBl1->setEnabled(false);
    ui->ButtonBl2->setEnabled(false);
    ui->ButtonBl3->setEnabled(false);
    ui->ButtonBl4->setEnabled(false);
    ui->ButtonBl5->setEnabled(false);
    }
    ui->buttonGroupBl->setId(ui->ButtonBl0, 0);
    MainWindow::on_Button_replot_rotors_clicked();
}
void MainWindow::on_pushButton_pushrotor_clicked()
{
    if(out.pr.nb>0)
    {
    ui->ButtonBl0->setEnabled(out.pr.nb>0);
    ui->ButtonBl1->setEnabled(out.pr.nb>1);
    ui->ButtonBl2->setEnabled(out.pr.nb>2);
    ui->ButtonBl3->setEnabled(out.pr.nb>3);
    ui->ButtonBl4->setEnabled(out.pr.nb>4);
    ui->ButtonBl5->setEnabled(out.pr.nb>5);
    }
    else
    {
    ui->ButtonBl0->setEnabled(true);
    ui->ButtonBl1->setEnabled(false);
    ui->ButtonBl2->setEnabled(false);
    ui->ButtonBl3->setEnabled(false);
    ui->ButtonBl4->setEnabled(false);
    ui->ButtonBl5->setEnabled(false);
    }
    ui->buttonGroupBl->setId(ui->ButtonBl0, 0);
    MainWindow::on_Button_replot_rotors_clicked();
}
void MainWindow::on_radioButton_mrXtime_clicked()
{
    MainWindow::on_Button_replot_rotors_clicked();
}
void MainWindow::on_radioButton_mrXazim_clicked()
{
    MainWindow::on_Button_replot_rotors_clicked();
}
void MainWindow::on_ButtonBl0_clicked()
{
    MainWindow::on_Button_replot_rotors_clicked();
}
void MainWindow::on_ButtonBl1_clicked()
{
    MainWindow::on_Button_replot_rotors_clicked();
}
void MainWindow::on_ButtonBl2_clicked()
{
    MainWindow::on_Button_replot_rotors_clicked();
}
void MainWindow::on_ButtonBl3_clicked()
{
    MainWindow::on_Button_replot_rotors_clicked();
}
void MainWindow::on_ButtonBl4_clicked()
{
    MainWindow::on_Button_replot_rotors_clicked();
}
void MainWindow::on_ButtonBl5_clicked()
{
    MainWindow::on_Button_replot_rotors_clicked();
}

//OpenGL
void MainWindow::write_to_opengl()//currentstep to oglwidget
{
    ui->oglwidget->model = out;
    ui->oglwidget->update();
}
void MainWindow::on_btn_controlpanel_clicked(bool checked)
{
    if(checked)
    {
        ui->btn_controlpanel->setText("<");
        ui->scrollArea_control->setFixedWidth(250);
        ui->ogl_controlpanel->setFixedWidth(267);
    }
    else
    {
        ui->btn_controlpanel->setText(">");
        ui->scrollArea_control->setFixedWidth(0);
        ui->ogl_controlpanel->setFixedWidth(17);
    }
}
void MainWindow::on_btn_infopanel_clicked(bool checked)
{
    if(checked)
    {
        ui->scrollAreaInfo->setFixedWidth(283);//260
        ui->ogl_infopanel->setFixedWidth(300);//277
        ui->btn_infopanel->setText(">");
    }
    else
    {
        ui->btn_infopanel->setText("<");
        ui->ogl_infopanel->setFixedWidth(17);
        ui->scrollAreaInfo->setFixedWidth(0);
    }
}
void MainWindow::on_actionPause_continue_triggered()
{
    MainWindow::on_StopButton_clicked();
}
void MainWindow::on_Slider_fi0mr_valueChanged(int value)
{
    ui->SpinBox_fi0mr->setValue( value*0.001 );
}
void MainWindow::on_SpinBox_fi0mr_valueChanged(double arg1)
{
    ui->Slider_fi0mr->setValue( ceil(arg1*1000) );
}
void MainWindow::on_Slider_etamr_valueChanged(int value)
{
    ui->SpinBox_etamr->setValue( value*0.001 );
}
void MainWindow::on_SpinBox_etamr_valueChanged(double arg1)
{
    ui->Slider_etamr->setValue( ceil(arg1*1000) );
}
void MainWindow::on_Slider_chimr_valueChanged(int value)
{
    ui->SpinBox_chimr->setValue( value*0.001 );
}
void MainWindow::on_SpinBox_chimr_valueChanged(double arg1)
{
    ui->Slider_chimr->setValue( ceil(arg1*1000) );
}
void MainWindow::on_Slider_fi0tr_valueChanged(int value)
{
    ui->SpinBox_fi0tr->setValue( value*0.001 );
}
void MainWindow::on_SpinBox_fi0tr_valueChanged(double arg1)
{
    ui->Slider_fi0tr->setValue( ceil(arg1*1000) );
}
void MainWindow::on_check_manualcontrol_toggled(bool checked)
{
    if(checked) ui->check_manualcontrol->setText("CONTROLLED");//On
    else        ui->check_manualcontrol->setText("OUT OF CONTROL");

    //ui->check_manualcontrol->setFont();
}
void MainWindow::on_btn_infopanel_toggled(bool checked)
{
    if(checked && hc!=0) showCurrentInfo();
}
void MainWindow::on_pushPlanerInfo_toggled(bool checked)
{
    if(hc) showCurrentInfo();
}
void MainWindow::on_pushMainRotorInfo_toggled(bool checked)
{
    if(hc) showCurrentInfo();
}
void MainWindow::on_pushTailRotorInfo_toggled(bool checked)
{
    if(hc) showCurrentInfo();
}
void MainWindow::on_pushShassiInfo_toggled(bool checked)
{
    if(hc) showCurrentInfo();
}
void MainWindow::on_check_fixlinemoving_2_toggled(bool checked)
{
    ui->oglwidget->fixLineMoving = checked;
    ui->oglwidget->update();
}
void MainWindow::on_pushButton_7_clicked()
{
    ui->rcX_2->setText("0");
    ui->oglwidget->setRotateCenter(ui->rcX_2->text().toFloat(), ui->rcY_2->text().toFloat(), ui->rcZ_2->text().toFloat());
}
void MainWindow::on_rcX_2_returnPressed()
{
    ui->oglwidget->setRotateCenter(ui->rcX_2->text().toFloat(), ui->rcY_2->text().toFloat(), ui->rcZ_2->text().toFloat());
}
void MainWindow::on_rcY_2_returnPressed()
{
    ui->oglwidget->setRotateCenter(ui->rcX_2->text().toFloat(), ui->rcY_2->text().toFloat(), ui->rcZ_2->text().toFloat());
}
void MainWindow::on_rcZ_2_returnPressed()
{
    ui->oglwidget->setRotateCenter(ui->rcX_2->text().toFloat(), ui->rcY_2->text().toFloat(), ui->rcZ_2->text().toFloat());
}
void MainWindow::on_check_axes_2_toggled(bool checked)
{
    ui->oglwidget->visAx = checked;
    ui->oglwidget->update();
}
void MainWindow::on_check_axesSKS_2_toggled(bool checked)
{
    ui->oglwidget->visAxSKS = checked;
    ui->oglwidget->update();
}
void MainWindow::on_check_mraxes_toggled(bool checked)
{
    ui->oglwidget->visAxRotors = checked;
    ui->oglwidget->update();
}
void MainWindow::on_check_ground_2_toggled(bool checked)
{
    ui->oglwidget->visGround = checked;
    ui->oglwidget->update();
}
void MainWindow::on_check_F_2_toggled(bool checked)
{
    ui->oglwidget->visF = checked;

    ui->oglwidget->visFhg = checked;   ui->check_F_hg_2       ->setChecked(checked);
    ui->oglwidget->visFad = checked;   ui->check_F_airdyn_2   ->setChecked(checked);
    ui->oglwidget->visFmr = checked;   ui->check_F_mainrotor_2->setChecked(checked);
    ui->oglwidget->visFtr = checked;   ui->check_F_tailrotor_2->setChecked(checked);
    ui->oglwidget->visFpr = checked;   ui->check_F_pushrotor_2->setChecked(checked);
    ui->oglwidget->visFwh = checked;   ui->check_F_shassi_2   ->setChecked(checked);
    ui->oglwidget->update();
}
void MainWindow::on_check_M_2_toggled(bool checked)
{
    ui->oglwidget->visM = checked;

    ui->oglwidget->visMhg = checked;   //ui->check_M_hg_2       ->setChecked(checked);
    ui->oglwidget->visMad = checked;   //ui->check_M_airdyn_2   ->setChecked(checked);
    ui->oglwidget->visMmr = checked;   ui->check_M_mainrotor_2->setChecked(checked);
    ui->oglwidget->visMtr = checked;   ui->check_M_tailrotor_2->setChecked(checked);
    ui->oglwidget->visMpr = checked;   ui->check_M_pushrotor_2->setChecked(checked);
    ui->oglwidget->visMwh = checked;   //ui->check_M_shassi_2   ->setChecked(checked);
    ui->oglwidget->update();
}
void MainWindow::on_check_F_hg_2_toggled(bool checked)
{
    ui->oglwidget->visFhg = checked;
    ui->oglwidget->update();
}
void MainWindow::on_check_M_hg_2_toggled(bool checked)
{
    ui->oglwidget->visMhg = checked;
    ui->oglwidget->update();
}
void MainWindow::on_check_F_airdyn_2_toggled(bool checked)
{
    ui->oglwidget->visFad = checked;
    ui->oglwidget->update();
}
void MainWindow::on_check_M_airdyn_2_toggled(bool checked)
{
    ui->oglwidget->visMad = checked;
    ui->oglwidget->update();
}
void MainWindow::on_check_F_mainrotor_2_toggled(bool checked)
{
    ui->oglwidget->visFmr = checked;
    if(hc) hc->mrshow = checked;
    ui->oglwidget->update();
}
void MainWindow::on_check_M_mainrotor_2_toggled(bool checked)
{
    ui->oglwidget->visMmr = checked;
    ui->oglwidget->update();
}
void MainWindow::on_check_F_tailrotor_2_toggled(bool checked)
{
    ui->oglwidget->visFtr = checked;
    ui->oglwidget->update();
}
void MainWindow::on_check_M_tailrotor_2_toggled(bool checked)
{
    ui->oglwidget->visMtr = checked;
    ui->oglwidget->update();
}
void MainWindow::on_check_F_pushrotor_2_toggled(bool checked)
{
    ui->oglwidget->visFpr = checked;
    ui->oglwidget->update();
}
void MainWindow::on_check_M_pushrotor_2_toggled(bool checked)
{
    ui->oglwidget->visMpr = checked;
    ui->oglwidget->update();
}
void MainWindow::on_check_F_shassi_2_toggled(bool checked)
{
    ui->oglwidget->visFwh = checked;
    if(hc) hc->shassishow = checked;
    ui->oglwidget->update();
}
void MainWindow::on_check_M_shassi_2_toggled(bool checked)
{
    ui->oglwidget->visMwh = checked;
    ui->oglwidget->update();
}

void MainWindow::on_check_planer_toggled(bool checked)
{
    ui->oglwidget->visPlaner = checked;
    ui->oglwidget->update();
}
void MainWindow::on_check_MR_2_toggled(bool checked)
{
    ui->oglwidget->visMR = checked;
    ui->oglwidget->visTR = checked;
    ui->oglwidget->visPR = checked;
    ui->oglwidget->update();
}
void MainWindow::on_radioButton_earth_toggled(bool checked)
{
    ui->oglwidget->SKSbinding = !checked;
    ui->oglwidget->update();
}
void MainWindow::on_radioButton_sks_toggled(bool checked)
{
    ui->oglwidget->SKSbinding = checked;
    ui->oglwidget->update();
}
void MainWindow::on_radioButton_mr_toggled(bool checked)
{
    ui->oglwidget->onMRcamera = checked;
    if(checked)
    {
        ui->oglwidget->visMR = checked;
        ui->check_MR_2->setChecked(true);
    }
    ui->oglwidget->update();
}
void MainWindow::on_radioButton_tr_toggled(bool checked)
{
    ui->oglwidget->onTRcamera = checked;
    if(checked)
    {
        ui->oglwidget->visTR = checked;
        ui->check_MR_2->setChecked(true);
    }
    ui->oglwidget->update();
}
void MainWindow::on_radioButton_pr_toggled(bool checked)
{
    ui->oglwidget->onPRcamera = checked;
    if(checked)
    {
        ui->oglwidget->visPR = checked;
        ui->check_MR_2->setChecked(true);
    }
    ui->oglwidget->update();
}
void MainWindow::on_checkBox_1blade_2_toggled(bool checked)
{
    ui->oglwidget->oneBlade = checked;
    //ui->oglwidget->onRotorCamera = checked;
    //ui->oglwidget->visMR = checked;
    ui->oglwidget->update();
}
void MainWindow::on_pushButton_clicked()
{
    MainWindow::on_Slider_etamr_valueChanged( ui->Slider_etamr->value()-10 );
}
void MainWindow::on_pushButton_6_clicked()
{
    MainWindow::on_Slider_etamr_valueChanged( ui->Slider_etamr->value()+10 );
}
void MainWindow::on_pushButton_8_clicked()
{
    MainWindow::on_Slider_chimr_valueChanged( ui->Slider_chimr->value()-10 );
}
void MainWindow::on_pushButton_9_clicked()
{
    MainWindow::on_Slider_chimr_valueChanged( ui->Slider_chimr->value()+10 );
}
void MainWindow::on_pushButton_11_clicked()
{
    MainWindow::on_Slider_fi0mr_valueChanged( ui->Slider_fi0mr->value()+10 );
}
void MainWindow::on_pushButton_10_clicked()
{
    MainWindow::on_Slider_fi0mr_valueChanged( ui->Slider_fi0mr->value()-10 );
}
void MainWindow::on_pushButton_13_clicked()
{
    MainWindow::on_Slider_fi0tr_valueChanged( ui->Slider_fi0tr->value()-10 );
}
void MainWindow::on_pushButton_12_clicked()
{
    MainWindow::on_Slider_fi0tr_valueChanged( ui->Slider_fi0tr->value()+10 );
}
void MainWindow::on_spinBoxVm_valueChanged(int arg1)//масштаб векторов
{
    ui->oglwidget->mv = arg1*0.00005;
    ui->oglwidget->update();
}

//menu OPEN
void MainWindow::on_action_open_inputdata_triggered()//open infile
{
    infilename = QFileDialog::getOpenFileName(this,"OpenFile",".\/","helisim ini files (*.hs);;landing ini files (*.dat);;all files (*.*)");
    ui->pathEdit->setText(infilename);
    OpenInputData();
}
void MainWindow::on_pathEdit_returnPressed()
{
    OpenInputData();
}
bool MainWindow::OpenInputData()//open infile
{
    infilename = ui->pathEdit->text();
    QFile infile_(infilename);
    if(!infile_.exists())
    {
        infilename = QFileDialog::getOpenFileName(this,"OpenFile",".\/","helisim ini files (*.hs);;landing ini files (*.dat);;all files (*.*)");
        ui->pathEdit->setText(infilename);
    }
    else
    {
        QRegExp rx("\\.");
        QStringList fields = infilename.split(rx);
        if(fields.at(fields.size()-1)=="lnk")
        {
            infilename = infile_.readLink();
            ui->pathEdit->setText(infilename);
        }
    }
    if(infilename.isEmpty())  return 0;
    else
    {
        //загрузка данных из файла на форму
        QFile infile(infilename);
        if(infile.exists())
        {
            ui->memo_hs->clear();
            infile.open( QIODevice::ReadOnly | QIODevice::Text );
            QTextStream load_in(&infile);
            ui->memo_hs->appendPlainText( load_in.readAll() );
            infile.close();
            //небольшая прокрутка загруженного ini.файла
            ui->memo_hs->moveCursor(QTextCursor::Start);
        }
        else toMemo("warning: " + infilename + " not found");
        //запись данных в файл-копию
        QString copyfilename = infilename+".copy";
        QFile copyfile(copyfilename);
        copyfile.open(QIODevice::WriteOnly | QIODevice::Text);
        QTextStream copy(&copyfile);        //out.setCodec("UTF-8");
        copy << ui->memo_hs->toPlainText(); //out << tr("Значение x = ") << 12 << "\n";//поэлементно
        copyfile.close();
        return 1;
    }
}

//menu SAVE
void MainWindow::on_actionSave_report_as_TXT_triggered()
{
    QString outfileName = QFileDialog::getSaveFileName(this,"SaveMemoOut","","text (*.txt)");
    if(!outfileName.isEmpty())
    {
        QFile file(outfileName);
        file.open(QIODevice::WriteOnly | QIODevice::Text);
        QTextStream out(&file);        //out.setCodec("UTF-8");
        out << ui->memout->toPlainText();  //весь memout сразу        //out << tr("Значение x = ") << 12 << "\n";//поэлементно
        file.close();
    }
}
void MainWindow::on_action_save_text_as_pdf_triggered()//SaveMemoOutAsPDF
{
    QString outfileName = QFileDialog::getSaveFileName(this,"SaveMemoOut","","pdf (*.pdf)");
    if(!outfileName.isEmpty())
    {
        QPrinter printer(QPrinter::HighResolution);
        printer.setOutputFormat(QPrinter::PdfFormat);
        printer.setPageOrientation(QPageLayout::Landscape);
        printer.setOutputFileName(outfileName);
        ui->memout->document()->print(&printer);
    }
}
void MainWindow::on_action_save_plot1_as_pdf_triggered()//SavePlotAirframeAsPdf
{//------------------------------------------------------------------------------------------------
    QString outfileName = QFileDialog::getSaveFileName(this,"SavePlot:AirFrame","","pdf (*.pdf)");
    if(!outfileName.isEmpty())
    {
        ui->plot_helicopter->savePdf(outfileName);
    }
}
void MainWindow::on_actionMainrotor_plot_pdf_triggered()//SavePlotRotorAsPdf
{
    QString outfileName = QFileDialog::getSaveFileName(this,"SavePlot:Rotor","","pdf (*.pdf)");
    if(!outfileName.isEmpty())
    {
        ui->plot_mainrotor->savePdf(outfileName);
    }
}
void MainWindow::on_action_save_plot2_as_pdf_triggered()//SavePlotShassiAsPdf
{//------------------------------------------------------------------------------------------------
    QString outfileName = QFileDialog::getSaveFileName(this,"SavePlot:Shassi","","pdf (*.pdf)");
    if(!outfileName.isEmpty())
    {
        ui->plot_shassi->savePdf(outfileName);
    }
}
void MainWindow::on_actionSave_absorber_as_pdf_triggered()//SavePlotAbsorberAsPdf
{
    QString outfileName = QFileDialog::getSaveFileName(this,"SavePlot","","pdf (*.pdf)");
    if(!outfileName.isEmpty())
    {
        ui->abtestplot->savePdf(outfileName);
    }
}
void MainWindow::on_action_save_plot1_as_png_triggered()//SavePlotAirframeAsPng
{//------------------------------------------------------------------------------------------------
    QString outfileName = QFileDialog::getSaveFileName(this,"SavePlot:Helicopter","","images (*.png)");
    if(!outfileName.isEmpty())
    {
        ui->plot_helicopter->savePng(outfileName);
    }
}
void MainWindow::on_actionMainrotor_plot_png_triggered()//SavePlotRotorAsPng
{
    QString outfileName = QFileDialog::getSaveFileName(this,"SavePlot:Rotor","","images (*.png)");
    if(!outfileName.isEmpty())
    {
        ui->plot_mainrotor->savePng(outfileName);
    }
}
void MainWindow::on_action_save_plot2_as_png_triggered()//SavePlotShassiAsPng
{//------------------------------------------------------------------------------------------------
    QString outfileName = QFileDialog::getSaveFileName(this,"SavePlot:Shassi","","images (*.png)");
    if(!outfileName.isEmpty())
    {
        ui->plot_shassi->savePng(outfileName);
    }
}
void MainWindow::on_actionSave_absorber_as_png_triggered()//SavePlotAbsorberAsPng
{
    QString outfileName = QFileDialog::getSaveFileName(this,"SavePlot:Absorber","","images (*.png)");
    if(!outfileName.isEmpty())
    {
        ui->abtestplot->savePng(outfileName);
    }
}
void MainWindow::on_actionPlaner_grid_tab_triggered()//SaveGridAirframeAsTab
{
    QString savefilename = QFileDialog::getSaveFileName(this,"SaveTable:AirFrame","","grid data (*.tab)");
    if(!savefilename.isEmpty()) savegrid(savefilename, ui->grid_planer);
}
void MainWindow::on_actionMainrotor_grid_tab_triggered()//SaveGridMainRotorAsTab
{
    QString savefilename = QFileDialog::getSaveFileName(this,"SaveTable:MainRotor","","grid data (*.tab)");
    if(!savefilename.isEmpty()) savegrid(savefilename, ui->grid_mr);
}
void MainWindow::on_actionTailrotor_grid_tab_triggered()
{
    QString savefilename = QFileDialog::getSaveFileName(this,"SaveTable:TailRotor","","grid data (*.tab)");
    if(!savefilename.isEmpty()) savegrid(savefilename, ui->grid_tr);
}
void MainWindow::on_actionGear_grid_tab_triggered()
{
    QString savefilename = QFileDialog::getSaveFileName(this,"SaveTable:Gear","","grid data (*.tab)");
    if(!savefilename.isEmpty()) savegrid(savefilename, ui->grid_gear);
}
void MainWindow::savegrid(QString savefilename, QTableWidget *grid)//запись таблицы результатов в txt.файл
{
    QFile file(savefilename);
    if(file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QTextStream outtab(&file);//out.setCodec("UTF-8");
        for(int col=0; col < grid->columnCount(); col++)
        {
            outtab << grid->horizontalHeaderItem(col)->text();
            if(col < grid->columnCount()-1) outtab << "\t";
        }
        outtab << "\n";
        for(int row=0; row < grid->rowCount(); row++)
        {
            for(int col=0; col < grid->columnCount(); col++)
            {
                outtab << grid->item(row,col)->text();
                if(col < grid->columnCount()-1) outtab << "\t";
            }
            if(row < grid->rowCount()) outtab << "\n";
        }
        file.flush();
        file.close();
    }
    else ui->memout->appendPlainText("не удалось открыть для записи файл " + savefilename);
}
void MainWindow::on_grid_planer_cellDoubleClicked(int row, int column)
{
    QClipboard* pcbuf = QApplication::clipboard();
    QString buf = "";
    for(int r=0; r < ui->grid_planer->rowCount(); r++)
    {
        buf += ui->grid_planer->item(r,column)->text();
        buf += "\n";
    }
    pcbuf->setText(buf);
}
void MainWindow::on_grid_mr_cellDoubleClicked(int row, int column)
{
    QClipboard* pcbuf = QApplication::clipboard();
    QString buf = "";
    for(int r=0; r < ui->grid_mr->rowCount(); r++)
    {
        buf += ui->grid_mr->item(r,column)->text();
        buf += "\n";
    }
    pcbuf->setText(buf);
}
void MainWindow::on_grid_gear_cellDoubleClicked(int row, int column)
{
    QClipboard* pcbuf = QApplication::clipboard();
    QString buf = "";
    for(int r=0; r < ui->grid_gear->rowCount(); r++)
    {
        buf += ui->grid_gear->item(r,column)->text();
        buf += "\n";
    }
    pcbuf->setText(buf);
}


//TEST.absorber
void MainWindow::on_action_absorber_test_triggered()
{
    ui->memout->clear();
    readyforplotA = false;
    readyforplotP = false;
     ab_P_1.clear();     ab_P_2.clear();
     ab_h_1.clear();     ab_h_2.clear();
    ab_p1_1.clear();    ab_p1_2.clear();
    ab_p2_1.clear();    ab_p2_2.clear();
    ab_h1_1.clear();    ab_h1_2.clear();    ab_h2_1.clear();    ab_h2_2.clear();
    ab_P1_1.clear();    ab_P1_2.clear();    ab_P2_1.clear();    ab_P2_2.clear();
   ab_Pg1_1.clear();   ab_Pg1_2.clear();   ab_Pg2_1.clear();   ab_Pg2_2.clear();
   ab_Ph1_1.clear();   ab_Ph1_2.clear();   ab_Ph2_1.clear();   ab_Ph2_2.clear();
   ab_Pf1_1.clear();   ab_Pf1_2.clear();   ab_Pf2_1.clear();   ab_Pf2_2.clear();

    graph_ab_red1->clearData();    graph_ab_green1->clearData();    graph_ab_blue1->clearData();    graph_ab_blk11->clearData();    graph_ab_blk21->clearData();
    graph_ab_red2->clearData();    graph_ab_green2->clearData();    graph_ab_blue2->clearData();    graph_ab_blk12->clearData();    graph_ab_blk22->clearData();
    graph_ab_Pg1_1->clearData();   graph_ab_Pg1_2->clearData();     graph_ab_Pg2_1->clearData();    graph_ab_Pg2_2->clearData();
    graph_ab_Ph1_1->clearData();   graph_ab_Ph1_2->clearData();     graph_ab_Ph2_1->clearData();    graph_ab_Ph2_2->clearData();
    graph_ab_Pf1_1->clearData();   graph_ab_Pf1_2->clearData();     graph_ab_Pf2_1->clearData();    graph_ab_Pf2_2->clearData();

    ini.calcmode = 5;
    PreManagement();
    ok = read_ini(infilename);
    if(ok)//если подготовка выполнена без ошибок...
    {
        if(ini.solvePlaner)
        {
            if(read_fax( fname_fax )) { ok=0;  toMemo("read errors in " + fname_fax); }
        }
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
        //считывание таблицы нагружения
        TestFileRead();

        t=0;
        ui->grid_planer->clear();

        //удаление ранее рожденного объекта Helicopter (нужно при прерываниях расчета)
        hc = 0; delete hc;
        //создание объекта Landing
        hc = new Helicopter();
        //инициализация указателя класса на себя (нужно для статических функций)
        hc->SetClassPointer();
        //обнуление переменных
        hc->setNull();
        //передача в объект Landing исходных данных, считанных из файла
        SetFlyInitial();
        //подготовка к расчету, положение вертолета на шаге "0"
        i = 0;
        hc->iter = i;
        hc->PreFlying(out);
        //установка активности кнопок выбора стоек
        setShassiButton(hc->nst);
        ui->tabWidget->setCurrentIndex(3);

        hc->dt=(abtest_v)?(0.005/abtest_v/30.0):(1);
        hc->st[abtest_s]->Aa = 0;
        out.iter = i = 0;
        out.nst  = hc->nst;
        out.st[abtest_s].a.status = hc->st[abtest_s]->ab->status;
        out.st[abtest_s].a.S      = hc->st[abtest_s]->ab->S   *1000;
        out.st[abtest_s].a.ha     = hc->st[abtest_s]->ab->ha  *1000;
        out.st[abtest_s].a.ha1    = hc->st[abtest_s]->ab->c1.h*1000;
        out.st[abtest_s].a.ha2    = hc->st[abtest_s]->ab->c2.h*1000;
        out.st[abtest_s].a.Pa     = hc->st[abtest_s]->ab->Pa  ;
        out.st[abtest_s].a.Pa1    = hc->st[abtest_s]->ab->c1.P;
        out.st[abtest_s].a.Pa2    = hc->st[abtest_s]->ab->c2.P;
        out.st[abtest_s].a.V1     = hc->st[abtest_s]->ab->c1.V*1000;
        out.st[abtest_s].a.V2     = hc->st[abtest_s]->ab->c2.V*1000;
        out.st[abtest_s].a.p1     = hc->st[abtest_s]->ab->c1.p/98066.5;
        out.st[abtest_s].a.p2     = hc->st[abtest_s]->ab->c2.p/98066.5;

        it.clear();
        s_ha[abtest_s].clear(); s_ha1[abtest_s].clear(); s_ha2[abtest_s].clear();
        s_Pa[abtest_s].clear(); s_Pa1[abtest_s].clear(); s_Pa2[abtest_s].clear();

        AbsorbTestCycles();////

        ui->combo_sh_red  ->setCurrentIndex(1);
        ui->combo_sh_green->setCurrentIndex(2);
        ui->combo_sh_blue ->setCurrentIndex(3);
        ui->combo_sP_red  ->setCurrentIndex(1);
        ui->combo_sP_green->setCurrentIndex(2);
        ui->combo_sP_blue ->setCurrentIndex(3);

        //ui->tabWidget->setCurrentIndex(3);
        ui->allplots->setCurrentIndex(3);
    }
    else ui->memout->appendPlainText("ошибки при выполнении prelanding()");
}
void MainWindow::TestFileRead()
{
    bool finded = false, isbreak = false;
    QString line, findS;
    QStringList fields;
    QRegExp rx("\\s");
    float _,t,h,abtest_v_;

    abtest_t.clear();
    abtest_h.clear();
    abtest_v = 0;

    QFile infile( fname_abstst );//infilename
    infile.open( QIODevice::ReadOnly | QIODevice::Text );
    in = new QTextStream(&infile);
    if( find_int(in,"abtest_#", abtest_s )) { readerror=true; ui->memout->appendPlainText("abtest_# not found"); }
    //ищем строку "#ts-table:"
    while((!in->atEnd())&&(!isbreak))
    {
        line = in->readLine();
        fields = line.split(rx, QString::SkipEmptyParts);
        if(fields.size()>=1)
        {
            for(int i=0; i<fields.size(); i++)
            {
                findS = fields.at(i);
                if(findS=="#ts-table:")
                {
                    finded = true;  isbreak=true;  break;
                }
            }
        }
    }
    //если нашли:
    if(finded)
    {
        abtestistabled = true;
        line = in->readLine();//"заголовок" таблицы
        while(!in->atEnd())
        {
            line = in->readLine();
            fields = line.split(rx, QString::SkipEmptyParts);
            if(fields.size()>1)
            {
                //_ = fields.at(0).toFloat();
                t = fields.at(0).toFloat();
                h = fields.at(1).toFloat()*0.001;
                abtest_t.push_back(t);
                abtest_h.push_back(h);
            }
            else break;
        }
    }
    //если не нашли:
    else
    {
        abtestistabled = false;
        if( find_float(in,"abtest_v="  , abtest_v_ )) { readerror=true; ui->memout->appendPlainText("abtest_v= not found"); }
        abtest_v = abtest_v_/60.0/1000.0;//from [mm/min] to [m/c]
    }

    delete in;
    infile.close();
}
void MainWindow::AbsorbTestCycles()
{/////// обжатие амортизатора на стенде /////////////////////////////////
    write_to_grid_gear_const();
    write_to_grid_gear();
    hc->st[abtest_s]->ab->t_ = 0;
    hc->st[abtest_s]->ab->va = 1;
    AbsorbTestOut();

    if(abtestistabled)
    {// ПО СЧИТАННЫМ ДАННЫМ : ВРЕМЯ-ПЕРЕМЕЩЕНИЕ
        abteststeps = abtest_h.size();
        for (i=1; i<abteststeps; i++)// ПРЯМОЙ+ОБРАТНЫЙ ХОД
        {
            //hc->st[abtest_s]->ab->vlvnotclosed = false;
            hc->st[abtest_s]->ab->dt = abtest_t.at(i) - abtest_t.at(i-1);
            hc->st[abtest_s]->ab->va = (abtest_h.at(i) - abtest_h.at(i-1)) / hc->st[abtest_s]->ab->dt;
            if(hc->st[abtest_s]->ab->va<0)  hc->st[abtest_s]->ab->t_+= hc->st[abtest_s]->ab->dt;
            // проверка по предельным обжатиям амортизатора
            if ( abtest_h.at(i)*0.9999 > hc->st[abtest_s]->ab->S )  break;
            if ( abtest_h.at(i) < 0 )                               break;
            hc->st[abtest_s]->ab->SolveSingleTest();
            hc->st[abtest_s]->Aa += hc->st[abtest_s]->ab->dA;
            AbsorbTestOut();
        }
    }
    else
    {
        do// ПРЯМОЙ ХОД
        {
            i++;
            hc->st[abtest_s]->ab->dt = hc->dt;
            hc->st[abtest_s]->ab->va = abtest_v;
            hc->st[abtest_s]->ab->SolveSingleTest();
            hc->st[abtest_s]->Aa += hc->st[abtest_s]->ab->dA;
            AbsorbTestOut();
        }
        while(hc->st[abtest_s]->ab->ha + hc->st[abtest_s]->ab->dha < hc->st[abtest_s]->ab->S);

        do// ОБРАТНЫЙ ХОД
        {
            i++;
            hc->st[abtest_s]->ab->dt = hc->dt;
            hc->st[abtest_s]->ab->t_+= hc->dt;
            hc->st[abtest_s]->ab->va =-abtest_v;
            hc->st[abtest_s]->ab->SolveSingleTest();
            hc->st[abtest_s]->Aa += hc->st[abtest_s]->ab->dA;
            AbsorbTestOut();
        }
        while(hc->st[abtest_s]->ab->ha > 0 && hc->st[abtest_s]->ab->status!=-2);
    }
    AbsorbTestPlot();
    write_to_plot();
}
void MainWindow::AbsorbTestOut()
{
    hc->st[abtest_s]->ab->ha_   = hc->st[abtest_s]->ab->ha;
    hc->st[abtest_s]->ab->c1.h_ = hc->st[abtest_s]->ab->c1.h;
    hc->st[abtest_s]->ab->c2.h_ = hc->st[abtest_s]->ab->c2.h;
    hc->st[abtest_s]->ab->V_    = hc->st[abtest_s]->ab->V;
    hc->st[abtest_s]->ab->c1.V_ = hc->st[abtest_s]->ab->c1.V;
    hc->st[abtest_s]->ab->c2.V_ = hc->st[abtest_s]->ab->c2.V;
    hc->st[abtest_s]->ab->c1.p_ = hc->st[abtest_s]->ab->c1.p;
    hc->st[abtest_s]->ab->c2.p_ = hc->st[abtest_s]->ab->c2.p;

    t += hc->st[abtest_s]->ab->dt;

    if(hc->st[abtest_s]->ab->va > 0)
    {
         ab_P_1.push_back( hc->st[abtest_s]->ab->Pa   *0.001 );
         ab_h_1.push_back( hc->st[abtest_s]->ab->ha   *1000  );
        ab_h1_1.push_back( hc->st[abtest_s]->ab->c1.h *1000  );
        ab_P1_1.push_back( hc->st[abtest_s]->ab->c1.P *0.001 );
        ab_p1_1.push_back( hc->st[abtest_s]->ab->c1.p/98066.5);
        ab_h2_1.push_back( hc->st[abtest_s]->ab->c2.h *1000  );
        ab_P2_1.push_back( hc->st[abtest_s]->ab->c2.P *0.001 );
        ab_p2_1.push_back( hc->st[abtest_s]->ab->c2.p/98066.5);

        ab_Pg1_1.push_back( hc->st[abtest_s]->ab->c1.Pg*0.001 );
        ab_Ph1_1.push_back( hc->st[abtest_s]->ab->c1.Ph*0.001 );
        ab_Pf1_1.push_back( hc->st[abtest_s]->ab->c1.Pf*0.001 );
        ab_Pg2_1.push_back( hc->st[abtest_s]->ab->c2.Pg*0.001 );
        ab_Ph2_1.push_back( hc->st[abtest_s]->ab->c2.Ph*0.001 );
        ab_Pf2_1.push_back( hc->st[abtest_s]->ab->c2.Pf*0.001 );
    }
    if(hc->st[abtest_s]->ab->va < 0)
    {
         ab_P_2.push_back( hc->st[abtest_s]->ab->Pa   *0.001 );
         ab_h_2.push_back( hc->st[abtest_s]->ab->ha   *1000  );
        ab_h1_2.push_back( hc->st[abtest_s]->ab->c1.h *1000  );
        ab_P1_2.push_back( hc->st[abtest_s]->ab->c1.P *0.001 );
        ab_p1_2.push_back( hc->st[abtest_s]->ab->c1.p/98066.5);
        ab_h2_2.push_back( hc->st[abtest_s]->ab->c2.h *1000  );
        ab_P2_2.push_back( hc->st[abtest_s]->ab->c2.P *0.001 );
        ab_p2_2.push_back( hc->st[abtest_s]->ab->c2.p/98066.5);

        ab_Pg1_2.push_back( hc->st[abtest_s]->ab->c1.Pg*0.001 );
        ab_Ph1_2.push_back(-hc->st[abtest_s]->ab->c1.Ph*0.001 );
        ab_Pf1_2.push_back(-hc->st[abtest_s]->ab->c1.Pf*0.001 );
        ab_Pg2_2.push_back( hc->st[abtest_s]->ab->c2.Pg*0.001 );
        ab_Ph2_2.push_back(-hc->st[abtest_s]->ab->c2.Ph*0.001 );
        ab_Pf2_2.push_back(-hc->st[abtest_s]->ab->c2.Pf*0.001 );
    }
    readyforplotA = true;

    out.iter= i;
    out.t   = t;
    out.st[abtest_s].a.status = hc->st[abtest_s]->ab->status;
    out.st[abtest_s].a.S      = hc->st[abtest_s]->ab->S   *1000;
    out.st[abtest_s].a.ha     = hc->st[abtest_s]->ab->ha  *1000;
    out.st[abtest_s].a.ha1    = hc->st[abtest_s]->ab->c1.h*1000;
    out.st[abtest_s].a.ha2    = hc->st[abtest_s]->ab->c2.h*1000;
    out.st[abtest_s].a.Pa     = hc->st[abtest_s]->ab->Pa ;
    out.st[abtest_s].a.Pa1    = hc->st[abtest_s]->ab->c1.P;
    out.st[abtest_s].a.Pa2    = hc->st[abtest_s]->ab->c2.P;
    out.st[abtest_s].a.Pg1    = hc->st[abtest_s]->ab->c1.Pg;
    out.st[abtest_s].a.Ph1    = hc->st[abtest_s]->ab->c1.Ph;
    out.st[abtest_s].a.Pf1    = hc->st[abtest_s]->ab->c1.Pf;
    out.st[abtest_s].a.Pg2    = hc->st[abtest_s]->ab->c2.Pg;
    out.st[abtest_s].a.Ph2    = hc->st[abtest_s]->ab->c2.Ph;
    out.st[abtest_s].a.Pf2    = hc->st[abtest_s]->ab->c2.Pf;
    out.st[abtest_s].a.V1     = hc->st[abtest_s]->ab->c1.V*1000;
    out.st[abtest_s].a.V2     = hc->st[abtest_s]->ab->c2.V*1000;
    out.st[abtest_s].a.p1     = hc->st[abtest_s]->ab->c1.p/98066.5;
    out.st[abtest_s].a.p2     = hc->st[abtest_s]->ab->c2.p/98066.5;
    out.st[abtest_s].a.v1     = hc->st[abtest_s]->ab->c1.v;
    out.st[abtest_s].a.v2     = hc->st[abtest_s]->ab->c2.v;
    out.st[abtest_s].Aa       = hc->st[abtest_s]->Aa;

                it.push_back(out.t);
    s_ha[abtest_s].push_back(out.st[abtest_s].a.ha        );//обжатие суммарное
   s_ha1[abtest_s].push_back(out.st[abtest_s].a.ha1       );//обжатие камеры 1
   s_ha2[abtest_s].push_back(out.st[abtest_s].a.ha2       );//обжатие камеры 2
    s_Pa[abtest_s].push_back(out.st[abtest_s].a.Pa  *0.001);//усилие общее
   s_Pa1[abtest_s].push_back(out.st[abtest_s].a.Pa1 *0.001);//усилие на штоке 1
   s_Pa2[abtest_s].push_back(out.st[abtest_s].a.Pa2 *0.001);//усилие на штоке 2

    if((i%ini.logstep)==0)
    {
        write_to_grid_gear();

        QString logstring="";
        logstring += " h= "  + ss.setNum(hc->st[abtest_s]->ab->c1.h*1000 ,'f',1) + " / ";
        logstring +=           ss.setNum(hc->st[abtest_s]->ab->c2.h*1000 ,'f',1) + " мм\t";
        //logstring += "t_= "  + ss.setNum(hc->st[abtest_s]->ab->t_,'f',3) + " c;\t";
        //logstring += "vd= "  + ss.setNum(hc->st[abtest_s]->ab->valve_damp(hc->st[abtest_s]->ab->c2.f,hc->st[abtest_s]->ab->c2.fr),'f',3) + " c;\t";
        //logstring +=  "wr= " + ss.setNum(hc->st[abtest_s]->ab->c1.wr ,'f',1) + " / ";
        //logstring +=           ss.setNum(hc->st[abtest_s]->ab->c2.wr ,'f',1) + " м/c\t";
        //logstring += "w12= " + ss.setNum(hc->st[abtest_s]->ab->w12 ,'f',1) + " м/c\t";
        //logstring += "w12r= "+ ss.setNum(hc->st[abtest_s]->ab->w12r ,'f',1) + " м/c;";
        //logstring += "dpa1= "  + ss.setNum(hc->st[abtest_s]->ab->c1.dpa /98066.5,'f',3) + ",\t";
        //logstring += "dpa2= "  + ss.setNum(hc->st[abtest_s]->ab->c2.dpa /98066.5,'f',3) + " кг/см2;\t";
        logstring += "dpar1= " + ss.setNum(hc->st[abtest_s]->ab->c1.dpar/98066.5,'f',3) + ",\t";
        logstring += "dpar2= " + ss.setNum(hc->st[abtest_s]->ab->c2.dpar/98066.5,'f',3) + " кг/см2;\t";
        logstring += "dpbr1= " + ss.setNum(hc->st[abtest_s]->ab->c1.dpbr/98066.5,'f',3) + ",\t";
        logstring += "dpbr2= " + ss.setNum(hc->st[abtest_s]->ab->c2.dpbr/98066.5,'f',3) + " кг/см2;\t";
        //logstring += "p= "+ ss.setNum(hc->st[abtest_s]->ab->c1.p/98066.5,'f',3) + " кг/см2;\t";
        //double VV = hc->st[abtest_s]->ab->V * pow( (hc->st[abtest_s]->ab->c1.dpar+hc->st[abtest_s]->ab->c2.dpar) / hc->st[abtest_s]->ab->c1.p , 1/hc->st[abtest_s]->ab->kpoly);
        toLogf(logstring);
        loadLog();
    }
}
void MainWindow::AbsorbTestPlot()
{
    graph_ab_red1  ->setData(ab_h_1, ab_P_1);    graph_ab_red1  ->setName("P\', кН");    graph_ab_red1  ->setVisible(ui->checkBox_at_F  ->isChecked());
    graph_ab_red2  ->setData(ab_h_2, ab_P_2);    graph_ab_red2  ->setName("P\", кН");    graph_ab_red2  ->setVisible(ui->checkBox_at_F  ->isChecked());
    graph_ab_green1->setData(ab_h_1, ab_P1_1);   graph_ab_green1->setName("P1\', кН");   graph_ab_green1->setVisible(ui->checkBox_at_F1 ->isChecked());
    graph_ab_green2->setData(ab_h_2, ab_P1_2);   graph_ab_green2->setName("P1\", кН");   graph_ab_green2->setVisible(ui->checkBox_at_F1 ->isChecked());
    graph_ab_blue1 ->setData(ab_h_1, ab_P2_1);   graph_ab_blue1 ->setName("P2\', кН");   graph_ab_blue1 ->setVisible(ui->checkBox_at_F2 ->isChecked());
    graph_ab_blue2 ->setData(ab_h_2, ab_P2_2);   graph_ab_blue2 ->setName("P2\", кН");   graph_ab_blue2 ->setVisible(ui->checkBox_at_F2 ->isChecked());
    graph_ab_Pg1_1 ->setData(ab_h_1, ab_Pg1_1);  graph_ab_Pg1_1 ->setName("Pg1\', кН");  graph_ab_Pg1_1 ->setVisible(ui->checkBox_at_Fg1->isChecked());
    graph_ab_Pg1_2 ->setData(ab_h_2, ab_Pg1_2);  graph_ab_Pg1_2 ->setName("Pg1\", кН");  graph_ab_Pg1_2 ->setVisible(ui->checkBox_at_Fg1->isChecked());
    graph_ab_Pg2_1 ->setData(ab_h_1, ab_Pg2_1);  graph_ab_Pg2_1 ->setName("Pg2\', кН");  graph_ab_Pg2_1 ->setVisible(ui->checkBox_at_Fg2->isChecked());
    graph_ab_Pg2_2 ->setData(ab_h_2, ab_Pg2_2);  graph_ab_Pg2_2 ->setName("Pg2\", кН");  graph_ab_Pg2_2 ->setVisible(ui->checkBox_at_Fg2->isChecked());
    graph_ab_Ph1_1 ->setData(ab_h_1, ab_Ph1_1);  graph_ab_Ph1_1 ->setName("Ph1\', кН");  graph_ab_Ph1_1 ->setVisible(ui->checkBox_at_Fh1->isChecked());
    graph_ab_Ph1_2 ->setData(ab_h_2, ab_Ph1_2);  graph_ab_Ph1_2 ->setName("Ph1\", кН");  graph_ab_Ph1_2 ->setVisible(ui->checkBox_at_Fh1->isChecked());
    graph_ab_Ph2_1 ->setData(ab_h_1, ab_Ph2_1);  graph_ab_Ph2_1 ->setName("Ph2\', кН");  graph_ab_Ph2_1 ->setVisible(ui->checkBox_at_Fh2->isChecked());
    graph_ab_Ph2_2 ->setData(ab_h_2, ab_Ph2_2);  graph_ab_Ph2_2 ->setName("Ph2\", кН");  graph_ab_Ph2_2 ->setVisible(ui->checkBox_at_Fh2->isChecked());
    graph_ab_Pf1_1 ->setData(ab_h_1, ab_Pf1_1);  graph_ab_Pf1_1 ->setName("Pf1\', кН");  graph_ab_Pf1_1 ->setVisible(ui->checkBox_at_Ff1->isChecked());
    graph_ab_Pf1_2 ->setData(ab_h_2, ab_Pf1_2);  graph_ab_Pf1_2 ->setName("Pf1\", кН");  graph_ab_Pf1_2 ->setVisible(ui->checkBox_at_Ff1->isChecked());
    graph_ab_Pf2_1 ->setData(ab_h_1, ab_Pf2_1);  graph_ab_Pf2_1 ->setName("Pf2\', кН");  graph_ab_Pf2_1 ->setVisible(ui->checkBox_at_Ff2->isChecked());
    graph_ab_Pf2_2 ->setData(ab_h_2, ab_Pf2_2);  graph_ab_Pf2_2 ->setName("Pf2\", кН");  graph_ab_Pf2_2 ->setVisible(ui->checkBox_at_Ff2->isChecked());
    graph_ab_blk11 ->setData(ab_h_1, ab_p1_1);   graph_ab_blk11 ->setName("p1\', ат");   graph_ab_blk11 ->setVisible(ui->checkBox_at_p1 ->isChecked());
    graph_ab_blk12 ->setData(ab_h_2, ab_p1_2);   graph_ab_blk12 ->setName("p1\", ат");   graph_ab_blk12 ->setVisible(ui->checkBox_at_p1 ->isChecked());
    graph_ab_blk21 ->setData(ab_h_1, ab_p2_1);   graph_ab_blk21 ->setName("p2\', ат");   graph_ab_blk21 ->setVisible(ui->checkBox_at_p2 ->isChecked());
    graph_ab_blk22 ->setData(ab_h_2, ab_p2_2);   graph_ab_blk22 ->setName("p2\", ат");   graph_ab_blk22 ->setVisible(ui->checkBox_at_p2 ->isChecked());

    ui->abtestplot->rescaleAxes(); //обновили шкалы

    ui->abtestplot-> axisRect()->axis(QCPAxis::atLeft ,0)->setRange(ui->Edit_at_F_min->text().toInt(), ui->Edit_at_F_max->text().toInt());
    int ntick = 0.75*(ui->Edit_at_F_max->text().toInt() - ui->Edit_at_F_min->text().toInt());
    ntick = (ntick==0)?(10):(ntick);
    if(ntick>70)
        ui->abtestplot-> axisRect()->axis(QCPAxis::atLeft ,0)->setAutoTickCount(0.25*(ntick));
    else if(ntick>50)
        ui->abtestplot-> axisRect()->axis(QCPAxis::atLeft ,0)->setAutoTickCount(0.5*(ntick));
    else
        ui->abtestplot-> axisRect()->axis(QCPAxis::atLeft ,0)->setAutoTickCount(ntick);//1.9*
    ui->abtestplot-> axisRect()->axis(QCPAxis::atLeft ,0)->setSubTickLength(0,0);
    ui->abtestplot-> axisRect()->axis(QCPAxis::atRight,1)->setRange(ui->Edit_at_p_min->text().toInt(), ui->Edit_at_p_max->text().toInt());
    ntick = 0.1*(ui->Edit_at_p_max->text().toInt() - ui->Edit_at_p_min->text().toInt());
    ntick = (ntick==0)?(10):(ntick);
    ui->abtestplot-> axisRect()->axis(QCPAxis::atRight,1)->setAutoTickCount(ntick-2);
    ntick = 0.1*(ab_h_1.at( ab_h_1.size()-1 ) );
    ntick = (ntick==0)?(10):(ntick);
    ui->abtestplot-> axisRect()->axis(QCPAxis::atBottom,0)->setAutoTickCount(ntick);//-1
    ui->abtestplot-> axisRect()->axis(QCPAxis::atBottom ,0)->setRange(ab_h_1.at(0), ab_h_1.at(ab_h_1.size()-1));

    ui->abtestplot->replot();      //обновили всё
    ui->abtestplot->setVisible(true);
    ui->abtestplot->setEnabled(true);
}

void MainWindow::on_checkBox_at_toggled(bool checked)
{
    ui->plotbox_at->setVisible(checked);
}
void MainWindow::on_Button_replot_shassi_3_clicked()
{
    if(readyforplotA) AbsorbTestPlot();
    if(readyforplotP) PneumaTestPlot();
}
void MainWindow::on_checklegend_at_toggled(bool checked)
{
    ui->abtestplot->legend->setVisible( checked );
    ui->abtestplot->replot();
}
void MainWindow::on_checkBox_at_F_clicked()
{
    MainWindow::on_Button_replot_shassi_3_clicked();
}
void MainWindow::on_checkBox_at_F1_clicked()
{
    MainWindow::on_Button_replot_shassi_3_clicked();
}
void MainWindow::on_checkBox_at_F2_clicked()
{
    MainWindow::on_Button_replot_shassi_3_clicked();
}
void MainWindow::on_checkBox_at_Fg1_clicked()
{
    MainWindow::on_Button_replot_shassi_3_clicked();
}
void MainWindow::on_checkBox_at_Fh1_clicked()
{
    MainWindow::on_Button_replot_shassi_3_clicked();
}
void MainWindow::on_checkBox_at_Ff1_clicked()
{
    MainWindow::on_Button_replot_shassi_3_clicked();
}
void MainWindow::on_checkBox_at_Fg2_clicked()
{
    MainWindow::on_Button_replot_shassi_3_clicked();
}
void MainWindow::on_checkBox_at_Fh2_clicked()
{
    MainWindow::on_Button_replot_shassi_3_clicked();
}
void MainWindow::on_checkBox_at_Ff2_clicked()
{
    MainWindow::on_Button_replot_shassi_3_clicked();
}
void MainWindow::on_checkBox_at_p1_clicked()
{
    MainWindow::on_Button_replot_shassi_3_clicked();
}
void MainWindow::on_checkBox_at_p2_clicked()
{
    MainWindow::on_Button_replot_shassi_3_clicked();
}

//TEST.pneumatic
void MainWindow::on_action_pneumatic_test_triggered()
{
    ui->memout->clear();
    readyforplotA = false;
    readyforplotP = false;
    pn_Ppy_1.clear();   pn_Ppy_2.clear();
    pn_Pdf_1.clear();   pn_Pdf_2.clear();
    pn_hpy_1.clear();   pn_hpy_2.clear();
      ab_P_1.clear();     ab_P_2.clear();
      ab_h_1.clear();     ab_h_2.clear();
     ab_p1_1.clear();    ab_p1_2.clear();
     ab_p2_1.clear();    ab_p2_2.clear();
     ab_h1_1.clear();    ab_h1_2.clear();    ab_h2_1.clear();    ab_h2_2.clear();
     ab_P1_1.clear();    ab_P1_2.clear();    ab_P2_1.clear();    ab_P2_2.clear();
    ab_Pg1_1.clear();   ab_Pg1_2.clear();   ab_Pg2_1.clear();   ab_Pg2_2.clear();
    ab_Ph1_1.clear();   ab_Ph1_2.clear();   ab_Ph2_1.clear();   ab_Ph2_2.clear();
    ab_Pf1_1.clear();   ab_Pf1_2.clear();   ab_Pf2_1.clear();   ab_Pf2_2.clear();

    graph_ab_red1->clearData();    graph_ab_green1->clearData();    graph_ab_blue1->clearData();    graph_ab_blk11->clearData();    graph_ab_blk21->clearData();
    graph_ab_red2->clearData();    graph_ab_green2->clearData();    graph_ab_blue2->clearData();    graph_ab_blk12->clearData();    graph_ab_blk22->clearData();
    graph_ab_Pg1_1->clearData();   graph_ab_Pg1_2->clearData();     graph_ab_Pg2_1->clearData();    graph_ab_Pg2_2->clearData();
    graph_ab_Ph1_1->clearData();   graph_ab_Ph1_2->clearData();     graph_ab_Ph2_1->clearData();    graph_ab_Ph2_2->clearData();
    graph_ab_Pf1_1->clearData();   graph_ab_Pf1_2->clearData();     graph_ab_Pf2_1->clearData();    graph_ab_Pf2_2->clearData();

    ini.calcmode = 5;
    PreManagement();
    ok = read_ini(infilename);
    if(ok)//если подготовка выполнена без ошибок...
    {
        if(ini.solvePlaner)
        {
            if(read_fax( fname_fax )) { ok=0;  toMemo("read errors in " + fname_fax); }
        }
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
        //считывание таблицы нагружения
        TestFileRead();

        t=0;
        ui->grid_planer->clear();

        //удаление ранее рожденного объекта Helicopter (нужно при прерываниях расчета)
        hc = 0; delete hc;
        //создание объекта Landing
        hc = new Helicopter();
        //инициализация указателя класса на себя (нужно для статических функций)
        hc->SetClassPointer();
        //обнуление переменных
        hc->setNull();
        //передача в объект Landing исходных данных, считанных из файла
        SetFlyInitial();
        //подготовка к расчету, положение вертолета на шаге "0"
        i = 0;
        hc->iter = i;
        hc->PreFlying(out);
        //установка активности кнопок выбора стоек
        setShassiButton(hc->nst);
        ui->tabWidget->setCurrentIndex(3);

        hc->dt=(abtest_v)?(0.001/abtest_v/3.0):(1);//1;
        hc->st[abtest_s]->Ap = 0;
        out.iter = i = 0;
        out.nst  = hc->nst;
        out.st[abtest_s].nwh = hc->st[abtest_s]->nwh;
        out.st[abtest_s].p.status = hc->st[abtest_s]->pn->status;
        out.st[abtest_s].p.h.y    = hc->st[abtest_s]->pn->h_.y  = 0;
        out.st[abtest_s].p.h.x    = hc->st[abtest_s]->pn->h_.x  = 0;
        out.st[abtest_s].p.h.z    = hc->st[abtest_s]->pn->h_.z  = 0;
        out.st[abtest_s].p.P.y    = hc->st[abtest_s]->pn->P.y  ;

        it.clear();
        s_hp_y[abtest_s].clear();
        s_Pp_y[abtest_s].clear();

        s_ha[abtest_s].clear(); s_ha1[abtest_s].clear(); s_ha2[abtest_s].clear();
        s_Pa[abtest_s].clear(); s_Pa1[abtest_s].clear(); s_Pa2[abtest_s].clear();

        PneumaTestCycles();////

        ui->combo_sh_red  ->setCurrentIndex(0);
        ui->combo_sh_green->setCurrentIndex(0);
        ui->combo_sh_blue ->setCurrentIndex(5);
        ui->combo_sP_red  ->setCurrentIndex(6);
        ui->combo_sP_green->setCurrentIndex(0);
        ui->combo_sP_blue ->setCurrentIndex(0);

        //ui->tabWidget->setCurrentIndex(3);
        ui->allplots ->setCurrentIndex(3);
    }
    else ui->memout->appendPlainText("ошибки при выполнении prelanding()");
}
void MainWindow::PneumaTestCycles()
{/////// обжатие пневматика на стенде /////////////////////////////////
    write_to_grid_gear_const();
    write_to_grid_gear();
    hc->st[abtest_s]->pn->v.y = 1;
    PneumaTestOut();
    if(abtestistabled)
    {// ПО СЧИТАННЫМ ДАННЫМ : ВРЕМЯ-ПЕРЕМЕЩЕНИЕ
        abteststeps = abtest_h.size();
        for (i=1; i<abteststeps; i++)// ПРЯМОЙ+ОБРАТНЫЙ ХОД
        {
            hc->st[abtest_s]->pn->dt  =  abtest_t.at(i) - abtest_t.at(i-1);
            hc->st[abtest_s]->pn->v.y = (abtest_h.at(i) - abtest_h.at(i-1)) / hc->st[abtest_s]->pn->dt;
            hc->st[abtest_s]->pn->dhx = 0;
            hc->st[abtest_s]->pn->dhz = 0;
            hc->st[abtest_s]->pn->dhy = hc->st[abtest_s]->pn->v.y * hc->st[abtest_s]->pn->dt;
            hc->st[abtest_s]->pn->h.y = hc->st[abtest_s]->pn->h_.y + hc->st[abtest_s]->pn->dhy;
            hc->st[abtest_s]->pn->h.x = hc->st[abtest_s]->pn->h_.x + hc->st[abtest_s]->pn->dhx;
            hc->st[abtest_s]->pn->h.z = hc->st[abtest_s]->pn->h_.z + hc->st[abtest_s]->pn->dhz;
            hc->st[abtest_s]->pn->h1  = hc->st[abtest_s]->pn->h;
            hc->st[abtest_s]->pn->h2  = hc->st[abtest_s]->pn->h;

            // проверка по предельным обжатиям амортизатора
            if ( abtest_h.at(i) > hc->st[abtest_s]->pn-> dpmd *1.2 )  break;
            if ( abtest_h.at(i) < 0 )                                 break;
            hc->st[abtest_s]->pn->Solve(hc->st[abtest_s]->nwh);
            PneumaTestOut();
        }
    }
    else
    {
        do// ПРЯМОЙ ХОД
        {
            i++;
            hc->st[abtest_s]->pn->dt  = hc->dt;
            hc->st[abtest_s]->pn->v.y = abtest_v;
            hc->st[abtest_s]->pn->dhx = 0;
            hc->st[abtest_s]->pn->dhz = 0;
            hc->st[abtest_s]->pn->dhy = hc->st[abtest_s]->pn->v.y  * hc->st[abtest_s]->pn->dt;
            hc->st[abtest_s]->pn->h.y = hc->st[abtest_s]->pn->h_.y + hc->st[abtest_s]->pn->dhy;
            hc->st[abtest_s]->pn->h.x = hc->st[abtest_s]->pn->h_.x + hc->st[abtest_s]->pn->dhx;
            hc->st[abtest_s]->pn->h.z = hc->st[abtest_s]->pn->h_.z + hc->st[abtest_s]->pn->dhz;
            hc->st[abtest_s]->pn->h1  = hc->st[abtest_s]->pn->h;
            hc->st[abtest_s]->pn->h2  = hc->st[abtest_s]->pn->h;
            hc->st[abtest_s]->pn->Solve(hc->st[abtest_s]->nwh);
            hc->st[abtest_s]->Ap += hc->st[abtest_s]->pn->dA;
            PneumaTestOut();
        }
        while(hc->st[abtest_s]->pn->h.y < hc->st[abtest_s]->pn->dpmd *1.2);//

        do// ОБРАТНЫЙ ХОД
        {
            i++;
            hc->st[abtest_s]->pn->dt  = hc->dt;
            hc->st[abtest_s]->pn->v.y =-abtest_v;
            hc->st[abtest_s]->pn->dhx = 0;
            hc->st[abtest_s]->pn->dhz = 0;
            hc->st[abtest_s]->pn->dhy = hc->st[abtest_s]->pn->v.y  * hc->st[abtest_s]->pn->dt;
            hc->st[abtest_s]->pn->h.y = hc->st[abtest_s]->pn->h_.y + hc->st[abtest_s]->pn->dhy;
            hc->st[abtest_s]->pn->h.x = hc->st[abtest_s]->pn->h_.x + hc->st[abtest_s]->pn->dhx;
            hc->st[abtest_s]->pn->h.z = hc->st[abtest_s]->pn->h_.z + hc->st[abtest_s]->pn->dhz;
            hc->st[abtest_s]->pn->h1  = hc->st[abtest_s]->pn->h;
            hc->st[abtest_s]->pn->h2  = hc->st[abtest_s]->pn->h;
            hc->st[abtest_s]->pn->Solve(hc->st[abtest_s]->nwh);
            hc->st[abtest_s]->Ap += hc->st[abtest_s]->pn->dA;
            PneumaTestOut();
        }
        while(hc->st[abtest_s]->pn->h.y > 0);
    }
    PneumaTestPlot();
    write_to_plot();
}
void MainWindow::PneumaTestOut()
{
    hc->st[abtest_s]->pn->h_     = hc->st[abtest_s]->pn->h;

    t += hc->st[abtest_s]->pn->dt;

    if(hc->st[abtest_s]->pn->v.y > 0)
    {
        pn_Ppy_1.push_back( hc->st[abtest_s]->pn->P.y  *0.001);
        pn_Pdf_1.push_back( hc->st[abtest_s]->pn->Pdf.y*0.001);
        pn_hpy_1.push_back( hc->st[abtest_s]->pn->h.y  *1000 );
    }
    if(hc->st[abtest_s]->pn->v.y < 0)
    {
        pn_Ppy_2.push_back( hc->st[abtest_s]->pn->P.y  *0.001);
        pn_Pdf_2.push_back( hc->st[abtest_s]->pn->Pdf.y*0.001);
        pn_hpy_2.push_back( hc->st[abtest_s]->pn->h.y  *1000 );
    }
    readyforplotP = true;

    out.iter= i;
    out.t   = t;
    out.st[abtest_s].p.status = hc->st[abtest_s]->pn->status;
    out.st[abtest_s].p.h.y    = hc->st[abtest_s]->pn->h.y *1000;
    out.st[abtest_s].p.h1.y   = hc->st[abtest_s]->pn->h1.y*1000;
    out.st[abtest_s].p.h2.y   = hc->st[abtest_s]->pn->h2.y*1000;
    out.st[abtest_s].p.P.y    = hc->st[abtest_s]->pn->P.y;
    out.st[abtest_s].p.P1.y   = hc->st[abtest_s]->pn->P1.y;
    out.st[abtest_s].p.P2.y   = hc->st[abtest_s]->pn->P2.y;
    out.st[abtest_s].Ap       = hc->st[abtest_s]->Ap;

                  it.push_back(out.t);
    s_hp_y[abtest_s].push_back(out.st[abtest_s].p.h.y       );//обжатие
    s_Pp_y[abtest_s].push_back(out.st[abtest_s].p.P.y *0.001);//усилие

    if((i%ini.logstep)==0)
        write_to_grid_gear();
}
void MainWindow::PneumaTestPlot()
{
    graph_ab_red1  ->setData(pn_hpy_1, pn_Ppy_1);   graph_ab_red1  ->setName("P\', кН");   graph_ab_red1  ->setVisible(ui->checkBox_at_F  ->isChecked());
    graph_ab_red2  ->setData(pn_hpy_2, pn_Ppy_2);   graph_ab_red2  ->setName("P\", кН");   graph_ab_red2  ->setVisible(ui->checkBox_at_F  ->isChecked());
    graph_ab_green1->setData(pn_hpy_1, pn_Pdf_1);  graph_ab_green1->setName("Pdf\', кН");  graph_ab_green1->setVisible(ui->checkBox_at_F1 ->isChecked());
    graph_ab_green2->setData(pn_hpy_2, pn_Pdf_2);  graph_ab_green2->setName("Pdf\", кН");  graph_ab_green2->setVisible(ui->checkBox_at_F1 ->isChecked());
    //graph_ab_blue1 ->setData(pn_hpy_1, pn_Ppy_1);  graph_ab_blue1 ->setName("P\', кН");  graph_ab_blue1 ->setVisible(ui->checkBox_at_F2 ->isChecked());
    //graph_ab_blue2 ->setData(pn_hpy_2, pn_Ppy_2);  graph_ab_blue2 ->setName("P\", кН");  graph_ab_blue2 ->setVisible(ui->checkBox_at_F2 ->isChecked());

    ui->abtestplot->rescaleAxes(); //обновили шкалы
    ui->abtestplot-> axisRect()->axis(QCPAxis::atLeft ,0)->setRange(ui->Edit_at_F_min->text().toInt(), ui->Edit_at_F_max->text().toInt());
    int ntick = 0.75*(ui->Edit_at_F_max->text().toInt() - ui->Edit_at_F_min->text().toInt());
    ntick = (ntick==0)?(10):(ntick);
    if(ntick>70)
        ui->abtestplot-> axisRect()->axis(QCPAxis::atLeft ,0)->setAutoTickCount(0.25*(ntick));
    else if(ntick>50)
        ui->abtestplot-> axisRect()->axis(QCPAxis::atLeft ,0)->setAutoTickCount(0.5*(ntick));
    else
        ui->abtestplot-> axisRect()->axis(QCPAxis::atLeft ,0)->setAutoTickCount(ntick);//1.9*
    ui->abtestplot-> axisRect()->axis(QCPAxis::atLeft ,0)->setSubTickLength(0,0);
    ui->abtestplot-> axisRect()->axis(QCPAxis::atRight,1)->setRange(ui->Edit_at_p_min->text().toInt(), ui->Edit_at_p_max->text().toInt());
    ntick = 0.1*(ui->Edit_at_p_max->text().toInt() - ui->Edit_at_p_min->text().toInt());
    //ntick = (ntick==0)?(10)(ntick);
    if(ntick>2)
    ui->abtestplot-> axisRect()->axis(QCPAxis::atRight,1)->setAutoTickCount(ntick-2);
    ntick = 0.1*(pn_hpy_1.at( pn_hpy_1.size()-1 ) );
    ntick = (ntick==0)?(10):(ntick);
    ui->abtestplot-> axisRect()->axis(QCPAxis::atBottom,0)->setAutoTickCount(ntick);//-1
    ui->abtestplot->replot();      //обновили всё
    ui->abtestplot->setVisible(true);
    ui->abtestplot->setEnabled(true);
}

//TEST.mainrotor
void MainWindow::on_actionMainrotor_test_triggered()
{
/*    Pre();
    ok = read_ini(infilename);
    if(ok)//если подготовка выполнена без ошибок...
    {
        if(read_frm( fname_frm )) { ok=0;  toMemo("read errors in " + fname_frm); }//для положения втулки НВ
        if(read_mr( fname_mr )) { ok=0;  toMemo("read errors in " + fname_mr); }//параметры НВ
        //считывание таблицы нагружения
        if(ini.extloads)
        {
        if(read_load(fname_load1, load1, ini.load1pos)) { toMemo("read errors in " + fname_load1); }
        if(read_load(fname_load2, load2, ini.load2pos)) { toMemo("read errors in " + fname_load2); }
        }
        if(read_cntr(fname_cntr)) { toMemo("read errors in " + fname_cntr); }
        //удаление ранее рожденного объекта Helicopter (нужно при прерываниях расчета)
        hc = 0; delete hc;
        //создание объекта Helicopter
        hc = new Helicopter();
        //инициализация указателя класса на себя (нужно для статических функций)
        hc->SetClassPointer();
        //обнуление переменных
        hc->setNull();
        //передача в объект Helicopter исходных данных, считанных из файла
        SetFlyInitial();
        //подготовка к расчету, положение вертолета на шаге "0"
        i = 0;
        hc->iter = i;
        hc->t_ = 0;
        setExtLoads(0);
        hc->PreLanding(out);

        //установка ключей
        hc->isHelicopterSolve = false;
        hc->isMRtest = true;

        //запись initial&presolve
        toLogf(hc->IniPre2Log(ini.calcmode));
        toLogf(singleline);
        toLogf("MainRotor.Test:\n");
        //запись шага "0" (начального положения)
        write_helicopter_state(ini.datafolder+"0");
        //загрузка initial&presolve на форму
        loadLog();
        //opengl-прорисовка
        write_to_opengl();
        //основной цикл

            hc->mr->b[0].fi_ = -5*M_PI/180;   hc->mr->BladeForcesV(hc->mr->b[0].fi_);
            hc->mr->b[1].fi_ = 1*M_PI/180;    hc->mr->BladeForcesV(hc->mr->b[1].fi_);
            hc->mr->b[2].fi_ = 10*M_PI/180;   hc->mr->BladeForcesV(hc->mr->b[2].fi_);

        if(hc->mrsolve) MainRotorTest();
        //загрузка .log файла
        loadLog();
    }
    else ui->memout->appendPlainText("ошибки при выполнении prelanding()");
*/
}
void MainWindow::MainRotorTest()
{
 /*   ok=true;
    while( ok && i<ini.steps && !isStop )
    {
        steps = i++;
        hc->iter = i;
        hc->t = hc->t_ + hc->dt;
        //внешнее воздействие на НВ
        //setExtLoads(hc->t);

        ///проекции неуравновешенной силы НВ на ось Z
        hc->mr->UnbalancedForce();
        ///реакция вертолета на изменение сил -> новое положение оси НВ и ее ускорение
        hc->mr->StandBalance();

        //вывод результатов в структуру текущей итерации
        hc->OutFilling(out);
        hc->Post2ogl();
            out.mr.H = -hc->mr->Faxex;
            out.mr.S = -hc->mr->Faxez;

            //вывод предупреждений
            if(hc->mr->warning)
            {
                toLogf(hc->log_ogl);
                loadLog();
            }

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
        //инфо на форму
        ui->showiter->setText(ss.setNum(i));
        ui->showtime->setText(ss.setNum(hc->t,'f',6));
        ui->progress->setValue( 100*i/float(ini.steps) );
        setWindowTitle(ss.setNum(hc->t,'f',6) + " - " + titlefilename + " - " + titleapplname);
        //вывод результатов расчета текущей итерации на форму
        if(ui->btn_control->isChecked()) write_to_oglmemo(&(hc->log_ogl));
        //opengl-прорисовка текущего положения вертолета
        write_to_opengl();
        //прерывание расчета
        QCoreApplication::processEvents();
    }
    //toLogf(hc->Post2log(ini.calcmode));//запись результатов
    if(ok && !isStop) toLogf("Done!");  else toLogf("Aborted!");
    toLogf("\n" + singleline);
*/
}

//HELP
void MainWindow::on_action_About_triggered()
{
    QMessageBox::about(0, "About", titleapplname + "\n-------------------------------------------------------------------------------------------------------------\n'Helicopter Simulation' - программный комплекс для моделирования динамики полета и посадки вертолетов; \nРазработка ОКБ 'Мотор Сич', группа математического моделирования и разработки прикладного программного обеспечения.");
}
void MainWindow::run_helpfile(QString name)
{
    process = new QProcess(this);
    arg.clear();
    arg << name;
    process->startDetached("explorer",arg);
    delete process;
}
void MainWindow::on_action_ug_triggered()
{
    run_helpfile("help\\helisim.userguide.pdf");
    //run_helpfile("help\\index.html");
}
void MainWindow::on_action_help_planer_triggered()
{
    run_helpfile("help\\helisim.airframe.pdf");
}
void MainWindow::on_action_help_rotors_triggered()
{
    run_helpfile("help\\helisim.rotors.pdf");
}
void MainWindow::on_action_help_gear_triggered()
{
    run_helpfile("help\\helisim.gear.pdf");
}



floatxyz MainWindow::SKStoSKZ(floatxyz A)//перевод из связанной СК в земную
{// ВНИМАНИЕ! единица измерения: мм
    floatxyz Askn;
    xyzkrt o;
    o.x = out.fus.org.x;
    o.y = out.fus.org.y;
    o.z = out.fus.org.z;
    o.k = out.fus.org.k*M_PI/180;
    o.r = out.fus.org.r*M_PI/180;
    o.t = out.fus.org.t*M_PI/180;

    float a11 = cos(o.r)*cos(o.t);
    float a21 = sin(o.k)*sin(o.r) - cos(o.k)*cos(o.r)*sin(o.t);
    float a31 = cos(o.k)*sin(o.r) + sin(o.k)*cos(o.r)*sin(o.t);
    float a12 = sin(o.t);
    float a22 = cos(o.t)*cos(o.k);
    float a32 =-cos(o.t)*sin(o.k);
    float a13 =-sin(o.r)*cos(o.t);
    float a23 = sin(o.k)*cos(o.r) + cos(o.k)*sin(o.r)*sin(o.t);
    float a33 = cos(o.k)*cos(o.r) - sin(o.k)*sin(o.r)*sin(o.t);

    Askn.x = o.x +  a11*A.x + a21*A.y + a31*A.z;
    Askn.y = o.y +  a12*A.x + a22*A.y + a32*A.z;
    Askn.z = o.z +  a13*A.x + a23*A.y + a33*A.z;

return Askn;
}











