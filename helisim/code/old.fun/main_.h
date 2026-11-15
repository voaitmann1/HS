#ifndef MAIN_CMD_H
#define MAIN_CMD_H
#include <QVector>
#include <QDir>
#include <QFile>
#include <QTime>
#include <QTextStream>
#include <QString>
#include <iostream>
#include <windows.h>
#include "helicopter.h"
//---------------------------------------------------------------------------
//#define  mst  5  //макс.кол.стоек
//#define  mbl  6  //макс.кол.лопастей
#define  M1x  101
#define  M2x  10001
//---------------------------------------------------------------------------
//using namespace std;
//namespace Ui { class MainWindow; }



Helicopter *hc=0;
instructure ini;//исходные (считанные)
outstructure out;//результат расчета текущей итерации динамики вертолета, состояние-положение



bool Pre();
bool MainProcess();
//void setShassiButton(int nst);
//посадка
void MainLanding();
void LandingCycles();
void outerloads();
void setExtLoads(float t);
float inter(float x, float x1, float x2, float y1, float y2);
void SetLandingInitial();
void SetDataForNextI();
void SetDataFromFile();
//балансировка
void MainBalancing();
void SetBalanceInitial();
//винты
void MainRotorCalc();
void MainRotorTest();
//test
//void abTestFileRead();
//void AbsorbSingleTest();
//void AbsorbSingleOut();
//void AbsorbSinglePlot();
//void PneumaticTest();
//void PneumaticTestOut();
//void PneumaticTestPlot();
//read
int  read_ini(QString infileName);
bool read_frm(QString infileName);
bool read_sts(int k, QString prefix);
bool read_mr(QString infileName);
bool read_tr(QString infileName);
bool read_mrr(QString infileName);
bool read_trr(QString infileName);
bool read_fax(QString infileName);
bool read_load(QString infileName, QVector<extloadstruct> &load, xyz &loadpos);
//bool read_cntr(QString infileName);
//bool PreReading();
//void ReadingCycle(int updwn);
//void ReadingCycles(int updwn);
//void ReadingPlotGridCycles();
//bool read1state(QString fname);
bool read_helicopter_state(QString filename);

bool find_float(QTextStream *in, QString searchString, float &findF);
bool find_2float(QTextStream *in, QString searchString, float &findF1, float &findF2);
bool find_3float(QTextStream *in, QString searchString, float &findF1, float &findF2, float &findF3);
bool find_int(QTextStream *in, QString searchString, int &findI);
bool find_str(QTextStream *in, QString searchString, QString &findStr );
bool find_Xmass(QTextStream *in, QString searchString, double mass[M1x], int &ncol);//[M2x]
bool find_XYmass(QTextStream *in, QString searchString, double mass[M2x], int &nrow, int &ncol);//[M2x]
//write
bool write_helicopter_state(QString filename);
void toLogf(QString string_);
void toMemo(QString string_);
void write_outfile(QString outfileName);//balancing
//void write_to_oglmemo(QString *string);
//void write_to_grid_const();
//void write_to_grid();
//void savegrid(QString savefilename);
//void plotpreprocess();
//void write_to_plot();
//void write_to_opengl();

//void SetManualControl();
//void setNullInterface();
//void setStopped(bool isstop);
//void MemoFlush(QString filename);
//void OpenInputData();
QString getElapsedTime(QTime started);
QString strSumm(QString str1, QString str2);
QString getStateDataFolderName(QString fullname);
QString getFileNameOnly(QString fullname);

//-----------------------------------------------------------------------
QTextStream *in, *in_frm, *in_fax, *in_mr, *in_tr, *in_mrr, *in_trr, *in_load;
QTextStream *logs = 0;
QString logfilename,
        infilename,
        titlefilename,
        titleapplname,
        fname_frm,
        fname_fax,
        fname_mr, fname_mrr,
        fname_tr, fname_trr,
        fname_bal,
        fname_load1, fname_load2;
        //fname_cntr;
QString outstr, ss,ss2, singleline;
QString readout,readoutmass,foundout;
QDate   startdate;
QTime   starttime,tm;
//-----------------------------------------------------------------------
double  dt, t, g;
double  kra, krp;//коэф.разжатия
bool    ok, //pre_ok,//ключ продолжения
        isStop, isBalanced,
        readerror,//ошибка считывания исх.данных
        landerror;//ошибка расчета
int     steps, rsteps, nstep, stepfix,
        i, j, k, ik,
        nst,nFront,nRear;
float   pi,d2r,r2d;
QString grndstr;

//framecontour-----------------------------------------------------------
QVector <extloadstruct> load1,load2;



#endif // MAINWINDOW_H
