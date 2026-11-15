#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include <QVector>
#include <QTextStream>
#include <QString>
#include "ui_mainwindow.h"
#include "helicopter.h"
#include "openglwdg.h"
//---------------------------------------------------------------------------
#define   mst     5 //макс.кол.стоек
#define   mbl     6 //макс.кол.лопастей
#define   mpr     5 //макс.кол.профилей
#define mass1   111 //101
#define mass2 10001 //10001
//---------------------------------------------------------------------------
using namespace std;
namespace Ui { class MainWindow; }

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    TValsShowHide logger;

private slots:
    void on_action_quit_triggered();
    void on_action_open_inputdata_triggered();
    void on_action_save_text_as_pdf_triggered();
    void on_action_save_plot1_as_pdf_triggered();
    void on_action_save_plot2_as_pdf_triggered();
    void on_action_save_plot1_as_png_triggered();
    void on_action_save_plot2_as_png_triggered();
    void on_checklabel1_clicked(bool checked);
    void on_pushButton_2_clicked();
    void on_pushButton_3_clicked();
    void on_EditCurrent_returnPressed();
    void on_StopButton_clicked();
    void on_action_absorber_test_triggered();
    void on_Button_replot_helicopter_clicked();
    void on_Button_replot_shassi_clicked();
    void on_checkBox_h_toggled(bool checked);
    void on_checkBox_s_toggled(bool checked);
    void on_checkBox_2_clicked(bool checked);
    void on_actionMainrotor_test_triggered();
    void on_Button_replot_shassi_3_clicked();
    void on_actionSave_absorber_as_png_triggered();
    void on_actionSave_absorber_as_pdf_triggered();
    void on_pushButton_4_clicked();
    void on_pushButton_5_clicked();
    void on_checkBox_mr_toggled(bool checked);
    void on_checkBox_at_toggled(bool checked);
    void on_checklegend_at_toggled(bool checked);
    void on_action_pneumatic_test_triggered();
    void on_grid_planer_cellDoubleClicked(int row, int column);
    void on_actionPost_triggered();
    void on_action_About_triggered();
    void on_actionMainrotor_plot_pdf_triggered();
    void on_actionPlaner_grid_tab_triggered();
    void on_actionMainrotor_grid_tab_triggered();
    void on_actionTailrotor_grid_tab_triggered();
    void on_actionGear_grid_tab_triggered();
    void on_grid_mr_cellDoubleClicked(int row, int column);
    void on_actionMainrotor_plot_png_triggered();
    void on_btn_controlpanel_clicked(bool checked);
    void on_btn_infopanel_clicked(bool checked);
    void on_actionPause_continue_triggered();
    void on_Slider_fi0mr_valueChanged(int value);
    void on_Slider_etamr_valueChanged(int value);
    void on_Slider_chimr_valueChanged(int value);
    void on_SpinBox_fi0mr_valueChanged(double arg1);
    void on_SpinBox_etamr_valueChanged(double arg1);
    void on_SpinBox_chimr_valueChanged(double arg1);
    void on_Slider_fi0tr_valueChanged(int value);
    void on_SpinBox_fi0tr_valueChanged(double arg1);
    void on_check_manualcontrol_toggled(bool checked);
    void on_btn_infopanel_toggled(bool checked);
    void on_pushPlanerInfo_toggled(bool checked);
    void on_pushMainRotorInfo_toggled(bool checked);
    void on_pushTailRotorInfo_toggled(bool checked);
    void on_pushShassiInfo_toggled(bool checked);
    void on_pushButton_7_clicked();
    void on_check_ground_2_toggled(bool checked);
    void on_check_axes_2_toggled(bool checked);
    void on_check_axesSKS_2_toggled(bool checked);
    void on_check_MR_2_toggled(bool checked);
    void on_check_fixlinemoving_2_toggled(bool checked);
    void on_check_F_2_toggled(bool checked);
    void on_check_F_hg_2_toggled(bool checked);
    void on_check_F_airdyn_2_toggled(bool checked);
    void on_check_F_mainrotor_2_toggled(bool checked);
    void on_check_F_tailrotor_2_toggled(bool checked);
    void on_check_F_shassi_2_toggled(bool checked);
    void on_check_M_2_toggled(bool checked);
    void on_check_M_hg_2_toggled(bool checked);
    void on_check_M_airdyn_2_toggled(bool checked);
    void on_check_M_mainrotor_2_toggled(bool checked);
    void on_check_M_tailrotor_2_toggled(bool checked);
    void on_check_M_shassi_2_toggled(bool checked);
    void on_rcX_2_returnPressed();
    void on_rcY_2_returnPressed();
    void on_rcZ_2_returnPressed();
    void on_radioButton_earth_toggled(bool checked);
    void on_radioButton_sks_toggled(bool checked);
    void on_radioButton_mr_toggled(bool checked);
    void on_checkBox_1blade_2_toggled(bool checked);
    void on_pushButton_clicked();
    void on_pushButton_6_clicked();
    void on_pushButton_8_clicked();
    void on_pushButton_9_clicked();
    void on_pushButton_11_clicked();
    void on_pushButton_10_clicked();
    void on_pushButton_13_clicked();
    void on_pushButton_12_clicked();
    void on_check_mraxes_toggled(bool checked);
    void on_action0_triggered();
    void on_action1_triggered();
    void on_action2_triggered();
    void on_action3_triggered();
    void on_pathEdit_returnPressed();
    void on_radioButton_tr_toggled(bool checked);
    void on_pushButton_mainrotor_clicked();
    void on_pushButton_tailrotor_clicked();
    void on_Button_replot_rotors_clicked();
    void on_radioButton_mrXtime_clicked();
    void on_radioButton_mrXazim_clicked();
    void on_ButtonBl0_clicked();
    void on_ButtonBl1_clicked();
    void on_ButtonBl2_clicked();
    void on_ButtonBl3_clicked();
    void on_ButtonBl4_clicked();
    void on_ButtonBl5_clicked();
    void on_ButtonSt0_clicked();
    void on_ButtonSt1_clicked();
    void on_ButtonSt2_clicked();
    void on_ButtonSt3_clicked();
    void on_ButtonSt4_clicked();
    void on_action4_triggered();
    void on_action5_triggered();
    void on_spinBoxVm_valueChanged(int arg1);
    void on_actionHELICOPTER_autorotation_Vy_define_triggered();
    void on_action_help_planer_triggered();
    void on_action_help_rotors_triggered();
    void on_action_help_gear_triggered();
    void on_action_ug_triggered();
    void on_actionSave_report_as_TXT_triggered();
    void on_actionAUTOGYRO_Tpr_define_triggered();
    void on_pushButton_pushrotor_clicked();
    void on_radioButton_pr_toggled(bool checked);
    void on_check_F_pushrotor_2_toggled(bool checked);
    void on_check_M_pushrotor_2_toggled(bool checked);

    void on_check_planer_toggled(bool checked);

    void on_grid_gear_cellDoubleClicked(int row, int column);

    void on_checkBox_at_F_clicked();

    void on_checkBox_at_F1_clicked();

    void on_checkBox_at_F2_clicked();

    void on_checkBox_at_Fg1_clicked();

    void on_checkBox_at_Fh1_clicked();

    void on_checkBox_at_Ff1_clicked();

    void on_checkBox_at_Fg2_clicked();

    void on_checkBox_at_Fh2_clicked();

    void on_checkBox_at_Ff2_clicked();

    void on_checkBox_at_p1_clicked();

    void on_checkBox_at_p2_clicked();

private:
    void startProcess(int mode);
    bool PreManagement();
    void ProcessManagement();
    void setPlanerInitial();
    void setRotorsInitial();
    //посадка
    void MainFlying();
    void FlyingCycles();
    void SetFlyInitial();
    void SetDataForRotors();
    void SetDataForNextI();
    void SetDataFromFile();
    void setExtLoads(float t);
    //балансировка
    void MainBalancing();
    void SetBalanceInitial();
    //винты
    void computeRotors();
    void writeReport(QString outfileName, QString outstring);
    bool readProfile();
    void MainRotorTest();
    //test
    void TestFileRead();
    void AbsorbTestCycles();
    void AbsorbTestOut();
    void AbsorbTestPlot();
    void PneumaTestCycles();
    void PneumaTestOut();
    void PneumaTestPlot();
    //read
    int  read_ini(QString infileName);
    int  read_ini_(QString infileName);
    bool read_sts(int k, QString prefix);
    bool read_mr(QString infileName);
    bool read_tr(QString infileName);
    bool read_pr(QString infileName);
    bool read_fax(QString infileName);
    bool read_load(QString infileName, QVector<extloadstruct> &load, xyz &loadpos);
    bool read_cntr(QString infileName);
    bool PreReading_new();
    bool PreReading();
    void ReadingCycle(int updwn);
    void ReadingCycles(int updwn);
    void ReadingPlotGridCycles();
    bool read1state(QString fname);
    bool read_helicopter_state(QString filename);
    bool find_float(QTextStream *in, QString searchString, float &findF);
    bool find_2float(QTextStream *in, QString searchString, float &findF1, float &findF2);
    bool find_3float(QTextStream *in, QString searchString, float &findF1, float &findF2, float &findF3);
    bool find_int(QTextStream *in, QString searchString, int &findI);
    bool find_str(QTextStream *in, QString searchString, QString &findStr );
    bool find_bool(QTextStream *in, QString searchString, bool &findB);
    bool find_vectorFloat(QTextStream *in, QString searchString, vector<float> &vf);
    bool find_vectorInt(QTextStream *in, QString searchString, vector<int> &vi);
    bool find_XmassI(QTextStream *in, QString searchString, int mass[mass1], int &ncol);//[M2x]
    bool find_Xmass_(QTextStream *in, QString searchString, float mass[mass1], int &ncol);//[M2x]
    bool find_Xmass(QTextStream *in, QString searchString, double mass[mass1], int &ncol);//[M2x]
    bool find_XYmass(QTextStream *in, QString searchString, double mass[mass2], int &nrow, int &ncol);//[M2x]
    void loadLog();
    //write
    void showCurrentInfo();
    void showReadedInfo();
    void toLogf(QString string);
    void toMemo(QString string);
    bool write_helicopter_state(QString filename);
    void write_to_grid_const();
    void write_to_grid();
    void write_to_grid_gear_const();
    void write_to_grid_gear();
    void write_to_grid_mr_const();
    void write_to_grid_mr();
    void write_to_grid_tr_const();
    void write_to_grid_tr();
    void write_to_grid_pr_const();
    void write_to_grid_pr();
    void savegrid(QString savefilename, QTableWidget *grid);
    void write_to_opengl();
    void write_to_plot();
    void setdata_to_plot_mr(QComboBox *cb, QCPGraph *graph, int l);
    void setdata_to_plot_tr(QComboBox *cb, QCPGraph *graph, int l);
    void setdata_to_plot_pr(QComboBox *cb, QCPGraph *graph, int l);
    void airframeplotprep();
    void shassiplotprep();
    void abtestplotprep();
    void rotorsplotprep();
    void run_helpfile(QString name);

    void setShassiButton(int nst);
    float inter(float x, float x1, float x2, float y1, float y2);
    void RecalcSelected(float x_, float y_, float z_);
    floatxyz SKStoSKZ(floatxyz A);
    void SetManualControl();
    void setNullInterface();
    void setStopped(bool isstop);
    int  brakeSlider();
    void setManualControl(float fi0tr, float fi0mr, float etamr, float chimr);
    void setTargetControl(float fi0tr, float fi0mr, float etamr, float chimr);
    void MemoFlush(QString filename);
    bool OpenInputData();
    QString getElapsedTime(QTime started);
    QString strSumm(QString str1, QString str2);
    QString getFolderName(QString fullname);
    QString getStateDataFolderName(QString fullname);
    QString getFileNameOnly(QString fullname);
    int float2int(float in);

    //-----------------------------------------------------------------------
         Ui::MainWindow *ui;
             Helicopter *hc = 0;
            instructure ini;//все исходные (считанные)
           outstructure out;//результат расчета текущей итерации динамики вертолета, состояние-положение
    //-----------------------------------------------------------------------
    QProcess *process;
    QStringList arg;
    //-----------------------------------------------------------------------
    QTextStream *in, *in_frm, *in_cntr, *in_fax, *in_mr, *in_tr, *in_mrr, *in_trr, *in_pr, *in_prr, *in_load;
    QTextStream *log = 0;

    QString report;
    QString inifiledir,
            logfilename,
            infilename,//<<OpenInputData()
            titlefilename,
            titleapplname,
            fname_abstst,
            fname_fax,
            fname_mr, fname_mrr,
            fname_tr, fname_trr,
            fname_pr, fname_prr,
            fname_bal,
            fname_load1, fname_load2,
            fname_cntr,
            fname_profile[mpr];
    QString outstr, ss,ss2, singleline;
    QString readout,readoutmass;
    QString symbal;
    QDate   startdate;
    QTime   starttime,tm;
    //test.absorber-----------------------------------------------------------
    double  dt, t, g;
    double  kra, krp;//коэф.разжатия
    bool    ok,//pre_ok,//ключ продолжения выполнения
            isStop, isBalanced,
            readerror,//ошибка считывания исх.данных
            landerror;//можно удалить
    int     steps, rsteps, nstep, stepfix,
            i, j, k, ik,
            nst,nFront,nRear;
    float   pi,d2r,r2d;
    QString grndstr;

    //framecontour-----------------------------------------------------------
    //QVector <extloadstruct> load1, load2;
    QVector <xyz>  frmcntr, mrbldfrnt, mrbldneut, mrbldrear, trbldfrnt, trbldneut, trbldrear, prbldfrnt, prbldneut, prbldrear;
    struct frmcntrposition
    {
        int   xdir, ydir, zdir ;//направление контура
        float xbias,ybias,zbias;//смещение контура
    } frmpos;

    //gridtable--------------------------------------------------------------
    QTableWidgetItem *headcellp, *cellp;
    //plot pointers----------------------------------------------------------
    QCPGraph
             *graph_mr_red ,    *graph_mr_green ,   *graph_mr_blue ,    *graph_mr_black ,
           //*graph_mr_red2,    *graph_mr_green2,   *graph_mr_blue2,    *graph_mr_black2,
             *graph_h_red  ,    *graph_h_green  ,   *graph_h_blue  ,    *graph_h_black  ,
             *graph_sh_red ,    *graph_sh_green ,   *graph_sh_blue ,
             *graph_sP_red ,    *graph_sP_green ,   *graph_sP_blue ,
             *graph_ab_red1,    *graph_ab_green1,   *graph_ab_blue1,    *graph_ab_blk11,  *graph_ab_blk21,
             *graph_ab_red2,    *graph_ab_green2,   *graph_ab_blue2,    *graph_ab_blk12,  *graph_ab_blk22,
             *graph_ab_Pg1_1,   *graph_ab_Pg1_2,    *graph_ab_Pg2_1,    *graph_ab_Pg2_2,
             *graph_ab_Ph1_1,   *graph_ab_Ph1_2,    *graph_ab_Ph2_1,    *graph_ab_Ph2_2,
             *graph_ab_Pf1_1,   *graph_ab_Pf1_2,    *graph_ab_Pf2_1,    *graph_ab_Pf2_2;
    //plot data----------------------------------------------------------------
    QVector<double> it,  //xlabels,
            h_org_x,    h_org_y,    h_org_z,
            h_org_k,    h_org_r,    h_org_t,
                 h_vn_x,        h_vn_y,        h_vn_z,
                 h_v_x,         h_v_y,         h_v_z,
                 h_w_x,         h_w_y,         h_w_z,
                 h_a_x,         h_a_y,         h_a_z,
                 h_n_x,         h_n_y,         h_n_z,
              h_Fsum_x,      h_Fsum_y,      h_Fsum_z,
              h_Msum_x,      h_Msum_y,      h_Msum_z,
              h_Fadp_x ,     h_Fadp_y ,     h_Fadp_z ,
              h_Madp_x ,     h_Madp_y ,     h_Madp_z ,
              h_Ffus_x ,     h_Ffus_y ,     h_Ffus_z ,
              h_Mfus_x ,     h_Mfus_y ,     h_Mfus_z ,
              h_Fstab_x,     h_Fstab_y,     h_Fstab_z,
              h_Fkeel_x,     h_Fkeel_y,     h_Fkeel_z,
                s_hp_x[mst],   s_hp_y[mst],   s_hp_z[mst],
               s_hp1_x[mst],  s_hp1_y[mst],  s_hp1_z[mst],
               s_hp2_x[mst],  s_hp2_y[mst],  s_hp2_z[mst],
                s_Pp_x[mst],   s_Pp_y[mst],   s_Pp_z[mst],
               s_Pp1_x[mst],  s_Pp1_y[mst],  s_Pp1_z[mst],
               s_Pp2_x[mst],  s_Pp2_y[mst],  s_Pp2_z[mst],
                  s_ha[mst],    s_ha1[mst],    s_ha2[mst],
                  s_Pa[mst],    s_Pa1[mst],    s_Pa2[mst],
                 s_Pap[mst],
        //массивы для несущего винта
            mr_a_x,     mr_a_y,     mr_a_z,
            mr_v_x,     mr_v_y,     mr_v_z,
            mr_s_x,     mr_s_y,     mr_s_z,
            mr_dct,     mr_dct_x,   mr_dct_z,

            mr_gama[mbl],
            mr_dgama[mbl],
            mr_d2gama[mbl],
            mr_Ma1[mbl],
            mr_H_sr,    mr_T_sr,    mr_S_sr,    mr_Mx_sr,   mr_My_sr,  mr_Mz_sr,
            mr_Fhx_sum, mr_Fhy_sum, mr_Fhz_sum, mr_Mhx_sum, mr_Mhy_sum, mr_Mhz_sum,
            mr_psi0,        mr_fi0,    mr_chi,  mr_eta,     mr_alfa,
            mr_psi[mbl],    mr_fi7[mbl],    mr_dfi7[mbl],
            mr_beta[mbl],   mr_dbeta[mbl],  mr_d2beta[mbl],
            mr_ksi[mbl],    mr_dksi[mbl],   mr_d2ksi[mbl],
            mr_T1[mbl],     mr_Q1[mbl],     mr_H1[mbl],     mr_S1[mbl],    mr_Fdmp[mbl],
            mr_Nr[mbl],     mr_Nf[mbl],     mr_P1[mbl],     mr_P2[mbl],    mr_K1[mbl],
            mr_Fx[mbl],     mr_Fy[mbl],     mr_Fz[mbl],
            mr_M1x[mbl],    mr_M1y[mbl],    mr_M1z[mbl],
            mr_M2x[mbl],    mr_M2y[mbl],    mr_M2z[mbl],
            mr_M3x[mbl],    mr_M3y[mbl],    mr_M3z[mbl],
            mr_M1T[mbl],    mr_M1Q[mbl],    mr_M1Gx[mbl],  mr_M1Gy[mbl],   mr_M1K[mbl],   mr_M1Px[mbl], mr_M1Py[mbl],   mr_M1Cx[mbl], mr_M1Cy[mbl],
            mr_M2T[mbl],    mr_M2Q[mbl],    mr_M2Gx[mbl],  mr_M2Gy[mbl],   mr_M2K[mbl],   mr_M2Px[mbl], mr_M2Py[mbl],   mr_M2Cx[mbl], mr_M2Cy[mbl],
            mr_M2D[mbl],
            mr_M3Ma[mbl],   mr_M3su[mbl],   mr_M3cb[mbl],  mr_M3tr[mbl],   mr_M3i[mbl],   mr_Fpov[mbl],
            mr_Fxh[mbl],    mr_Fyh[mbl],    mr_Fzh[mbl],
            mr_Mxh[mbl],    mr_Myh[mbl],    mr_Mzh[mbl],
        //массивы для рулевого винта
            tr_gama[mbl],
            tr_dgama[mbl],
            tr_d2gama[mbl],
            tr_Ma1[mbl],
            tr_H_sr,        tr_T_sr,        tr_S_sr,        tr_Mx_sr,   tr_My_sr,  tr_Mz_sr,
            tr_Fhx_sum,     tr_Fhy_sum,     tr_Fhz_sum,     tr_Mhx_sum, tr_Mhy_sum, tr_Mhz_sum,
            tr_psi0,        tr_fi0,
            tr_psi[mbl],    tr_fi7[mbl],    tr_dfi7[mbl],
            tr_beta[mbl],   tr_dbeta[mbl],  tr_d2beta[mbl],
            tr_ksi[mbl],    tr_dksi[mbl],   tr_d2ksi[mbl],
            tr_T1[mbl],     tr_Q1[mbl],     tr_H1[mbl],     tr_S1[mbl],     tr_Fdmp[mbl],
            tr_Nr[mbl],     tr_Nf[mbl],     tr_P1[mbl],     tr_P2[mbl],     tr_K1[mbl],
            tr_Fx[mbl],     tr_Fy[mbl],     tr_Fz[mbl],
            tr_M1x[mbl],    tr_M1y[mbl],    tr_M1z[mbl],
            tr_M2x[mbl],    tr_M2y[mbl],    tr_M2z[mbl],
            tr_M3x[mbl],    tr_M3y[mbl],    tr_M3z[mbl],
            tr_M3Ma[mbl],   tr_M3su[mbl],   tr_M3cb[mbl],   tr_M3tr[mbl],   tr_M3i[mbl],   tr_Fpov[mbl],
            tr_M1T[mbl],    tr_M1Q[mbl],    tr_M1Gx[mbl],   tr_M1Gy[mbl],   tr_M1K[mbl],   tr_M1Px[mbl], tr_M1Py[mbl],   tr_M1Cx[mbl], tr_M1Cy[mbl],
            tr_M2T[mbl],    tr_M2Q[mbl],    tr_M2Gx[mbl],   tr_M2Gy[mbl],   tr_M2K[mbl],   tr_M2Px[mbl], tr_M2Py[mbl],   tr_M2Cx[mbl], tr_M2Cy[mbl],
            tr_M2D[mbl],
            tr_Fxh[mbl],    tr_Fyh[mbl],    tr_Fzh[mbl],
            tr_Mxh[mbl],    tr_Myh[mbl],    tr_Mzh[mbl],
        //массивы для толкающего винта
            pr_gama[mbl],
            pr_dgama[mbl],
            pr_d2gama[mbl],
            pr_Ma1[mbl],
            pr_H_sr,        pr_T_sr,        pr_S_sr,        pr_Mx_sr,   pr_My_sr,   pr_Mz_sr,
            pr_Fhx_sum,     pr_Fhy_sum,     pr_Fhz_sum,     pr_Mhx_sum, pr_Mhy_sum, pr_Mhz_sum,
            pr_psi0,        pr_fi0,
            pr_psi[mbl],    pr_fi7[mbl],    pr_dfi7[mbl],
            pr_beta[mbl],   pr_dbeta[mbl],  pr_d2beta[mbl],
            pr_ksi[mbl],    pr_dksi[mbl],   pr_d2ksi[mbl],
            pr_T1[mbl],     pr_Q1[mbl],     pr_H1[mbl],     pr_S1[mbl],     pr_Fdmp[mbl],
            pr_Nr[mbl],     pr_Nf[mbl],     pr_P1[mbl],     pr_P2[mbl],     pr_K1[mbl],
            pr_Fx[mbl],     pr_Fy[mbl],     pr_Fz[mbl],
            pr_M1x[mbl],    pr_M1y[mbl],    pr_M1z[mbl],
            pr_M2x[mbl],    pr_M2y[mbl],    pr_M2z[mbl],
            pr_M3x[mbl],    pr_M3y[mbl],    pr_M3z[mbl],
            pr_M3Ma[mbl],   pr_M3su[mbl],   pr_M3cb[mbl],   pr_M3tr[mbl],   pr_M3i[mbl],  pr_Fpov[mbl],
            pr_M1T[mbl],    pr_M1Q[mbl],    pr_M1Gx[mbl],   pr_M1Gy[mbl],   pr_M1K[mbl],  pr_M1Px[mbl], pr_M1Py[mbl],  pr_M1Cx[mbl], pr_M1Cy[mbl],
            pr_M2T[mbl],    pr_M2Q[mbl],    pr_M2Gx[mbl],   pr_M2Gy[mbl],   pr_M2K[mbl],  pr_M2Px[mbl], pr_M2Py[mbl],  pr_M2Cx[mbl], pr_M2Cy[mbl],
            pr_M2D[mbl],
            pr_Fxh[mbl],    pr_Fyh[mbl],    pr_Fzh[mbl],
            pr_Mxh[mbl],    pr_Myh[mbl],    pr_Mzh[mbl];

    //absorbertest-----------------------------------------------------------
    bool    abtestistabled,//задано ли обжатие таблицей
            readyforplotA, //получены ли результаты расчета
            readyforplotP; //получены ли результаты расчета
    int     abtest_s,      //тестируемая стойка
            abteststeps;   //шагов всего
    double  abtest_v;      //скорость обжатия штоков
    vector <float>  abtest_t;//время
    vector <float>  abtest_h;//сумм.ход штоков
    QVector<double> ab_t, ab_p1_1, ab_p1_2, ab_p2_1, ab_p2_2,
                    ab_h_1, ab_h_2, ab_h1_1, ab_h2_1, ab_h1_2, ab_h2_2,
                    ab_P_1, ab_P_2, ab_P1_1, ab_P2_1, ab_P1_2, ab_P2_2,
                    ab_Pg1_1, ab_Pg1_2, ab_Pg2_1, ab_Pg2_2,
                    ab_Ph1_1, ab_Ph1_2, ab_Ph2_1, ab_Ph2_2,
                    ab_Pf1_1, ab_Pf1_2, ab_Pf2_1, ab_Pf2_2,
                    pn_hpy_1, pn_hpy_2, pn_Ppy_1, pn_Ppy_2, pn_Pdf_1, pn_Pdf_2;
};//-------------------------------------------------------------------------

#endif // MAINWINDOW_H
