#include <QtWidgets>
#include <QtOpenGL>
#include <GL/glu.h>
#include <math.h>
#include "openglwdg.h"
//---------------------------------------------------------------------------
#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif
#define hrzn 10.0
//---------------------------------------------------------------------------
QColor spacecolor(255, 255, 255, 255);
QColor earthcolor(230, 230, 230, 255);//234, 234, 234, 0
QColor airframecolor(160, 160, 160, 255);
QColor bladecolor(130, 130, 130, 100);//100

QColor Black(0, 0, 0, 255);
QColor Brown(190, 90, 90, 255);
QColor Gray(110, 110, 110, 255);
QColor Orange(250, 80, 50, 255);
QColor Red(240, 0, 0, 255);
QColor Green(0, 180, 0, 255);
QColor Blue(0, 0, 255, 255);
QColor lightBlue(0, 100, 250, 255);
QColor halfRed(100, 0, 0, 195);
QColor halfGreen(10, 70, 10, 195);
QColor halfBlue(5, 5, 100, 195);
//---------------------------------------------------------------------------
GLWidget::GLWidget(QWidget *parent) : QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{
    model.nst = 0;
    RotCenterX = 0;    RotCenterY = 0;    RotCenterZ = 0;
    visAx        = false;
    visAxSKS     = true;
    visAxRotors  = false;
    visGround    = false;
    SKSbinding   = false;
    oneBlade     = false;
    fixLineMoving= false;
    visPlaner    = true;
    visMR        = true;      onMRcamera   = false;
    visTR        = true;      onTRcamera   = false;
    visPR        = true;      onPRcamera   = false;
    visF         = false;     visM         = false;
    visFhg       = false;     visMhg       = false;
    visFad       = false;     visMad       = false;
    visFmr       = false;     visMmr       = false;
    visFtr       = false;     visMtr       = false;
    visFpr       = false;     visMpr       = false;
    visFwh       = false;     visMwh       = false;
}
GLWidget::~GLWidget()
{
}
void GLWidget::initializeGL()
{
    float lightcolor[]= {1,1,1};
    float difcolor[]=   {0.8,0.8,0.8};
    float matambient[]= {0.2,0.2,0.2};
    float matspecular[]={0.6,0.6,0.6};
    float shininess=0.7*128;
    //--Определение свойств материала-------------------------------------//
    glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,matambient);
    glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,matspecular);
    glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,shininess);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightcolor);
    glEnable(GL_LIGHT0);
    //--Проводить удаление невидимых линий и поверхностей-----------------//
        glEnable(GL_DEPTH_TEST);
    //--Проводить нормирование нормалей-----------------------------------//
        glEnable(GL_NORMALIZE);
    //--Материалы объектов отличаются только цветом диффузного отражения--//
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK,GL_DIFFUSE);
    //--Задаем цвет диффузного отражения для колец------------------------//
    glColor3fv(difcolor);
    glEnable(GL_MULTISAMPLE);
    qglClearColor(Qt::white);// заполняем экран белым цветом    qglClearColor(spacecolor);// заполняем экран белым цветом
    glEnable(GL_DEPTH_TEST); // задаем глубину проверки пикселей
    glShadeModel(GL_FLAT); // убираем режим сглаживания цветов
    glShadeModel(GL_SMOOTH);
    glPolygonMode(GL_FRONT_AND_BACK,GL_FILL); // фигуры будут закрашены с обеих сторон
    //    glPolygonMode(GL_FRONT,GL_LINE);//GL_FILL
    //    glPolygonMode(GL_BACK, GL_LINE);

    xPos =  0.1;
    yPos = -5;//-1.66;
    zPos = -50;//-27
    xRot = -3.5;
    yRot = -130;//-90
    zRot =  0;
    m = 50;//1;
    mv= 0.0005;//0.0002;
    model.fus.org.x =0;
    model.fus.org.y =0;
    model.fus.org.z =0;
    model.fus.org.k =0;
    model.fus.org.r =0;
    model.fus.org.t =0;
}
void GLWidget::resizeGL(int w, int h)
{
    float aspect = (float)w/(float)h;    //float x = qMax(w,h);
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective( 10.0, aspect, 0.1f, 500*hrzn ); //gluPerspective( 20.0, aspect, 0.5f, 500*hrzn );
    glMatrixMode(GL_MODELVIEW);
}

void GLWidget::paintGL()///main
{
    glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT);
    glEnable(GL_ALPHA_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(xPos, yPos, zPos);
    glRotatef(xRot, 1.0, 0.0, 0.0);
    glRotatef(yRot, 0.0, 1.0, 0.0);
    glRotatef(zRot, 0.0, 0.0, 1.0);

    ///привязка к земле или к СКС ==============================================================
    if( !onMRcamera && !onTRcamera && !onPRcamera && !oneBlade )
    {

        ///земля///
        glPushMatrix();
        if(SKSbinding)//привязка к началу СКС
        {
            //поворот СКС на углы Эйлера
            glRotatef(-model.fus.org.k, 1.0f, 0.0f, 0.0f);//x
            glRotatef(-model.fus.org.t, 0.0f, 0.0f, 1.0f);//z
            glRotatef(-model.fus.org.r, 0.0f, 1.0f, 0.0f);//y
            //смещение начала СКС
            glTranslatef(-model.fus.org.x, -model.fus.org.y, -model.fus.org.z);
        }
        else//        - привязка к земле
        {
            //смещение начала СКС
            glTranslatef(-RotCenterX, -RotCenterY, -RotCenterZ);//glTranslatef(0, 0, 0);
            //поворот СКС на углы Эйлера
            glRotatef(0, 0.0f, 1.0f, 0.0f);//y
            glRotatef(0, 0.0f, 0.0f, 1.0f);//z
            glRotatef(0, 1.0f, 0.0f, 0.0f);//x
        }
            //оси координат
            if((visAx)) axlearth( 3*m, 2*m, 2*m, 2*m );//(12000*m,12000*m,10000*m,10000*m);
            //земля front rear left right
            earth( 2*m, 1*m, 1*m, 1*m );//( 12*m, 12*m, 10*m, 10*m )
        glPopMatrix();

        ///вертолет///
        glPushMatrix();
            if(!SKSbinding) // привязка к земле
            {
                /* большая зеленая точка в ЦТ вертолета
                qglColor(Green);
                glPointSize(10);
                glBegin(GL_POINTS);
                if(fixLineMoving)
                    glVertex3f(model.fus.ocw.x-model.fus.org.x, model.fus.ocw.y, model.fus.ocw.z);
                else
                    glVertex3f(model.fus.ocw.x, model.fus.ocw.y, model.fus.ocw.z);
                glEnd();*/
                //смещение начала СКС
                if(fixLineMoving)
                    glTranslatef( 0.0, model.fus.org.y, model.fus.org.z );
                else
                    glTranslatef( model.fus.org.x, model.fus.org.y, model.fus.org.z );
                glTranslatef(-RotCenterX, -RotCenterY, -RotCenterZ);
                //поворот СКС на углы Эйлера
                glRotatef(model.fus.org.r, 0.0f, 1.0f, 0.0f);//y
                glRotatef(model.fus.org.t, 0.0f, 0.0f, 1.0f);//z
                glRotatef(model.fus.org.k, 1.0f, 0.0f, 0.0f);//x
            }
            else//SKSbinding - привязка к началу СКС
            {
                //смещение начала СКС
                glTranslatef(0, 0, 0);
                //поворот СКС на углы Эйлера
                glRotatef(0, 0.0f, 1.0f, 0.0f);//y
                glRotatef(0, 0.0f, 0.0f, 1.0f);//z
                glRotatef(0, 1.0f, 0.0f, 0.0f);//x
            }
            //оси СКС
            if(visAxSKS) axlesks();
            //планер
            fuselage();
            //вектора сил-моментов
            fmvectors();
            //стойки шасси
            for (int k=0; k<model.nst; k++)
            {
                shassi(k);
                //glPointSize(3);
                //qglColor(Green);
                //glBegin(GL_POINTS);
                //    glVertex3f( 0.001*model.st[k].saCfree.x, 0.001*model.st[k].saCfree.y, 0.001*model.st[k].saCfree.z );
                //glEnd();
            }
            if(model.mr.solve)///HB
            {
                //neutral point
                glPointSize(5);
                qglColor(Black);
                glBegin(GL_POINTS);
                    glVertex3f(0.001*model.mr.O.x, 0.001*model.mr.O.y, 0.001*model.mr.O.z);
                glEnd();

                //винт несущий
                glPushMatrix();
                    ///glTranslatef( 0.001*model.mr.O.x, 0.001*model.mr.O.y, 0.001*model.mr.O.z );
                    ///glRotatef(-model.mr.eps, 0.0f, 0.0f, 1.0f);
                    glTranslatef( 0.001*model.mr.O.x, 0.001*model.mr.O.y, 0.001*model.mr.O.z );
                    glRotatef(model.mr.eps_fz + model.mr.chi_hub, 0.0f, 0.0f, 1.0f);///glRotatef(-model.mr.eps, 0.0f, 0.0f, 1.0f);
                    glRotatef(model.mr.eps_fx + model.mr.eta_hub, 1.0f, 0.0f, 0.0f);//////////////// не проверил
                    glRotatef( 180.0,  0.0f, 1.0f, 0.0f);
                    //оси СКHВ
                    if(visAxRotors) mr_axle();
                    //втулка с лопастями
                    if(visMR) mainrotor();
                    //кольцо - кромка концов лопастей
                    mr_ring();
                glPopMatrix();

            }
            if(model.tr.mount)///PB
            {
                //neutral point
                glPointSize(5);
                qglColor(Black);
                glBegin(GL_POINTS);
                    glVertex3f(0.001*model.tr.O.x, 0.001*model.tr.O.y, 0.001*model.tr.O.z);
                glEnd();

                //винт рулевой
                glPushMatrix();
                    glTranslatef( 0.001*model.tr.O.x, 0.001*model.tr.O.y, 0.001*model.tr.O.z );//сдвиг к втулке РВ
                    glRotatef( 180.0,  0.0f, 1.0f, 0.0f);//разворот СК отн.У
                    glRotatef( -90.0,  1.0f, 0.0f, 0.0f);//поворот СК отн.Х на правый борт
                    //оси СКРВ
                    if(visAxRotors) tr_axle();
                    //втулка с лопастями
                    if(visTR) tailrotor();
                    //кольцо - кромка концов лопастей
                    tr_ring();
                glPopMatrix();

            }
            if(model.pr.mount)///TB
            {
                //neutral point
                glPointSize(5);
                qglColor(Black);
                glBegin(GL_POINTS);
                    glVertex3f(0.001*model.pr.O.x, 0.001*model.pr.O.y, 0.001*model.pr.O.z);
                glEnd();

                //винт рулевой
                glPushMatrix();
                    glTranslatef( 0.001*model.pr.O.x, 0.001*model.pr.O.y, 0.001*model.pr.O.z );//сдвиг к втулке
                    glRotatef( -90.0,  0.0f, 1.0f, 0.0f);//разворот СК отн.У
                    glRotatef( -90.0,  1.0f, 0.0f, 0.0f);//поворот СК отн.Х на правый борт
                    //оси СКРВ
                    if(visAxRotors) pr_axle();
                    //втулка с лопастями
                    if(visPR) pushrotor();
                    //кольцо - кромка концов лопастей
                    pr_ring();
                glPopMatrix();
            }
        glPopMatrix();

        /* //saCfree - нижние точки колес при разжатых стойках шасси
        if(model.isLanding)
        {
        //характерные точки saCfree//////////////////////////////////////////////////////////////////////////////////////
        glPushMatrix();
            if(SKSbinding)//
            {
                //смещение начала СКС
                if(fixLineMoving)///добавил
                    glTranslatef( 0.0, -0.001*model.fus.org.y, -0.001*model.fus.org.z);
                else
                    glTranslatef(-0.001*model.fus.org.x, -0.001*model.fus.org.y, -0.001*model.fus.org.z);
                glTranslatef(RotCenterX, RotCenterY, RotCenterZ);
                //поворот СКС на углы Эйлера
                glRotatef(-model.fus.org.r, 0.0f, 1.0f, 0.0f);//y
                glRotatef(-model.fus.org.t, 0.0f, 0.0f, 1.0f);//z
                glRotatef(-model.fus.org.k, 1.0f, 0.0f, 0.0f);//x
            }
            else///добавил /// привязка к земле
            {
                if(fixLineMoving)
                    glTranslatef(-0.001*model.fus.org.x, 0, 0 );
                    //glTranslatef( 0.0, -0.001*model.fus.org.y, -0.001*model.fus.org.z );
                else
                    glTranslatef(0.001*model.fus.org.x, 0.001*model.fus.org.y, 0.001*model.fus.org.z );
            }
            glPointSize(3);
            qglColor(Green);
            glBegin(GL_POINTS);
            for (int k=0; k<model.nst; k++)
                glVertex3f( 0.001*model.st[k].saCfree.x, 0.001*model.st[k].saCfree.y, 0.001*model.st[k].saCfree.z );
            glEnd();
        glPopMatrix();
        }*/
    }

    ///привязка к втулке НВ ====================================================================
    if( visMR && onMRcamera )
    {
        glPushMatrix();
            glTranslatef(-RotCenterX, -RotCenterY, -RotCenterZ);
            glRotatef( 180.0,  0.0f, 1.0f, 0.0f);//разворот СК отн.У
            mainrotor();
            mr_ring();
        glPopMatrix();
    }

    ///привязка к втулке PВ ====================================================================
    if( visTR && onTRcamera )
    {
        glPushMatrix();
            glTranslatef(-RotCenterX, -RotCenterY, -RotCenterZ);
            glRotatef( 180.0,  0.0f, 1.0f, 0.0f);//разворот СК отн.У
            tailrotor();
            tr_ring();
        glPopMatrix();
    }

    ///привязка к втулке TВ ====================================================================
    if( visPR && onPRcamera )
    {
        glPushMatrix();
            glTranslatef(-RotCenterX, -RotCenterY, -RotCenterZ);
            glRotatef( 180.0,  0.0f, 1.0f, 0.0f);//разворот СК отн.У
            pushrotor();
            pr_ring();
        glPopMatrix();
    }

}

void GLWidget::earth(int front,int rear,int left,int right)
{
    float I,II;
    glLineWidth(1);
    /*/сетка
    QColor netcolor(180, 180, 180, 255);
    glBegin(GL_LINES);//
        qglColor(netcolor);
        for(int I=-rear/100; I<front/100; I++)
        {
            for(int II=-left/100; II<right/100; II++)
            {
                glVertex3f(I*100,   0,  II*100);
                glVertex3f((I)*100, 0,(II+1)*100);
            }
        }
        for(int I=-rear/100; I<front/100; I++)
        {
            for(int II=-left/100; II<right/100; II++)
            {
                glVertex3f(I*100,    0,  II*100);
                glVertex3f((I+1)*100,0,(II)*100);
            }
        }
    glEnd();// */

    //плитка
    //glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
    glBegin(GL_LINES);//
    qglColor(Orange);
    if(model.nst)
        for (int k=0; k<model.nst; k++)
        {
            if(fixLineMoving &&      !SKSbinding)
                I  = 0.001*model.st[k].saC.x - model.fus.org.x;
            else
                I  = 0.001*model.st[k].saC.x;
            II = 0.001*model.st[k].saC.z;
            glVertex3f( I -0.2, model.st[k].ground, II      );
            glVertex3f( I +0.2, model.st[k].ground, II      );
            glVertex3f( I     , model.st[k].ground, II +0.2 );
            glVertex3f( I     , model.st[k].ground, II -0.2 );
        }
    glEnd();
    if(visGround)
    {
        glBegin(GL_QUADS);//GL_LINES
        qglColor(earthcolor);
        for(int I=-rear; I<front; I++)
        {
            for(int II=-left; II<right; II++)
            {
                glVertex3f( (I)   +0.005, 0, (II)   +0.005 );
                glVertex3f( (I)   +0.005, 0, (II+1) -0.005 );
                glVertex3f( (I+1) -0.005, 0, (II+1) -0.005 );
                glVertex3f( (I+1) -0.005, 0, (II)   +0.005 );
            }
        }
        glEnd();
    }

}
void GLWidget::axlearth(int front,int rear,int left,int right)
{
    //основные линии
    glLineWidth(2);
    glBegin( GL_LINES );

        qglColor(Red);
        glVertex3f(0,0,0);
        glVertex3f(1*front, 0, 0);

        qglColor(Green);
        glVertex3f(0,0,0);
        glVertex3f(0, 0.5*right, 0);

        qglColor(Blue);
        glVertex3f(0,0,0);
        glVertex3f(0, 0, 1*right);

    glEnd();

    //добавленные
    glLineWidth(2);
    glBegin( GL_LINES );

        qglColor(halfRed);
        glVertex3f(0,0,0);
        glVertex3f(-1*rear, 0, 0);

        qglColor(halfGreen);
        glVertex3f(0,0,0);
        glVertex3f(0, -0.2*right, 0);

        qglColor(halfBlue);
        glVertex3f(0,0,0);
        glVertex3f(0, 0, -1*left);

    glEnd();
}

void GLWidget::fuselage()
{
    //центр тяжести вертолета
    glPushMatrix();
        qglColor(Red);
        glPointSize(8);
        glTranslatef( 0.001*model.fus.xct, 0.001*model.fus.yct, 0.001*model.fus.zct );
        glBegin(GL_POINTS);
            glVertex3f(0,0,0);
        glEnd();
    glPopMatrix();

    //вектор скорости из ЦТ вертолета
    ///vector_(1, Blue,  0.001*model.fus.xct, 0.001*model.fus.yct, 0.001*model.fus.zct,  10*mv*model.fus.v.x, 10*mv*model.fus.v.y, 10*mv*model.fus.v.z);

    //линии фюзеляжа
    glLineWidth(1);
    qglColor(airframecolor);//Black
    glBegin(GL_LINES);
        for (int k=0; k<model.nst; k++)
        {

            glVertex3f( 0, 0, 0 );//glVertex3f( 0.001*model.fus.xct,    0.001*model.fus.yct,    0.001*model.fus.zct   );
            ///glVertex3f( 0.001*model.st[k].A.x,  0.001*model.st[k].A.y,  0.001*model.st[k].A.z );
            ///glVertex3f( 0.001*model.st[k].A.x,  0.001*model.st[k].A.y,  0.001*model.st[k].A.z );
            glVertex3f( 0.001*model.st[k].O.x,  0.001*model.st[k].O.y,  0.001*model.st[k].O.z );
        }

        //for (int k=1; k<model.nst; k++)
        //{
        //    glVertex3f( 0.001*model.st[k-1].O.x, 0.001*model.st[k-1].O.y, 0.001*model.st[k-1].O.z );
        //    glVertex3f( 0.001*model.st[k].O.x,   0.001*model.st[k].O.y,   0.001*model.st[k].O.z  );
        //}
        if(model.nst==3)
        {
            glVertex3f( 0.001*model.st[0].O.x,  0.001*model.st[0].O.y,  0.001*model.st[0].O.z );
            glVertex3f( 0.001*model.st[1].O.x,  0.001*model.st[1].O.y,  0.001*model.st[1].O.z );
            glVertex3f( 0.001*model.st[0].O.x,  0.001*model.st[0].O.y,  0.001*model.st[0].O.z );
            glVertex3f( 0.001*model.st[2].O.x,  0.001*model.st[2].O.y,  0.001*model.st[2].O.z );
            glVertex3f( 0.001*model.st[1].O.x,  0.001*model.st[1].O.y,  0.001*model.st[1].O.z );
            glVertex3f( 0.001*model.st[2].O.x,  0.001*model.st[2].O.y,  0.001*model.st[2].O.z );
        }
        if(model.nst==4)
        {
            glVertex3f( 0.001*model.st[0].O.x,  0.001*model.st[0].O.y,  0.001*model.st[0].O.z );
            glVertex3f( 0.001*model.st[1].O.x,  0.001*model.st[1].O.y,  0.001*model.st[1].O.z );
            glVertex3f( 0.001*model.st[1].O.x,  0.001*model.st[1].O.y,  0.001*model.st[1].O.z );
            glVertex3f( 0.001*model.st[3].O.x,  0.001*model.st[3].O.y,  0.001*model.st[3].O.z );
            glVertex3f( 0.001*model.st[3].O.x,  0.001*model.st[3].O.y,  0.001*model.st[3].O.z );
            glVertex3f( 0.001*model.st[2].O.x,  0.001*model.st[2].O.y,  0.001*model.st[2].O.z );
            glVertex3f( 0.001*model.st[2].O.x,  0.001*model.st[2].O.y,  0.001*model.st[2].O.z );
            glVertex3f( 0.001*model.st[0].O.x,  0.001*model.st[0].O.y,  0.001*model.st[0].O.z );
            if(model.st[0].type==0)
            {
                glVertex3f( 0.001*model.st[0].B.x,  0.001*model.st[0].B.y,  0.001*model.st[0].B.z );
                glVertex3f( 0.001*model.st[2].B.x,  0.001*model.st[2].B.y,  0.001*model.st[2].B.z );
                glVertex3f( 0.001*model.st[1].B.x,  0.001*model.st[1].B.y,  0.001*model.st[1].B.z );
                glVertex3f( 0.001*model.st[3].B.x,  0.001*model.st[3].B.y,  0.001*model.st[3].B.z );
            }
        }
        //glVertex3f( 0.001*model.st[0].A.x,  0.001*model.st[0].A.y,  0.001*model.st[0].A.z );
        //glVertex3f( 0.001*model.st[1].A.x,  0.001*model.st[1].A.y,  0.001*model.st[1].A.z );
        //glVertex3f( 0.001*model.st[0].A.x,  0.001*model.st[0].A.y,  0.001*model.st[0].A.z );
        //glVertex3f( 0.001*model.st[2].A.x,  0.001*model.st[2].A.y,  0.001*model.st[2].A.z );

        glVertex3f( 0, 0, 0 );//glVertex3f( 0.001*model.fus.xct,    0.001*model.fus.yct,    0.001*model.fus.zct  );
        glVertex3f( 0.001*model.tr.O.x,     0.001*model.tr.O.y,     0.0 );
        //на несущий винт
        glVertex3f( 0,                       0,                       0 );
        glVertex3f( 0.001*model.mr.O.x,      0.001*model.mr.O.y,      0.001*model.mr.O.z );
        //на рулевой винт
        glVertex3f( 0.001*model.tr.O.x,      0.001*model.tr.O.y,      0.0 );//0.001*model.mr.O.z*0.9 );
        glVertex3f( 0.001*model.tr.O.x,      0.001*model.tr.O.y,      0.001*model.tr.O.z );
    glEnd();

    //сетка фюзеляжа framecontour
    if(visPlaner && frmcntr)
    {
        glLineWidth(1);
        qglColor(airframecolor);
        glBegin(GL_LINES);
            for (int j=0; j<frmcntr->size()-3; j+=3)
            {
                node1 = frmcntr->at(j);
                node2 = frmcntr->at(j+1);
                node3 = frmcntr->at(j+2);
                glVertex3f(node1.x,node1.y,node1.z);
                glVertex3f(node2.x,node2.y,node2.z);
                glVertex3f(node2.x,node2.y,node2.z);
                glVertex3f(node3.x,node3.y,node3.z);
                glVertex3f(node3.x,node3.y,node3.z);
                glVertex3f(node1.x,node1.y,node1.z);
            }
        glEnd();
    }
}
void GLWidget::axlesks()
{
    float ml=0.1*m;//2500*m;
    float sl=0.1*m;//2500*m;
    //основные линии
    glLineWidth(1);
    glBegin( GL_LINES );

        qglColor(Red);
        glVertex3f(0,0,0);
        glVertex3f(1.5*ml, 0, 0);

        qglColor(Green);
        glVertex3f(0,0,0);
        glVertex3f(0, ml, 0);

        qglColor(Blue);
        glVertex3f(0,0,0);
        glVertex3f(0, 0, ml);

    glEnd();

    //добавленные
    glLineWidth(1);
    glBegin( GL_LINES );

        qglColor(halfRed);
        glVertex3f(0,0,0);
        glVertex3f(-2.5*sl, 0, 0);

        qglColor(halfGreen);
        glVertex3f(0,0,0);
        glVertex3f(0, -sl, 0);

        qglColor(halfBlue);
        glVertex3f(0,0,0);
        glVertex3f(0, 0, -sl);

    glEnd();
}

void GLWidget::shassi(int k)
{
    //жесткая часть стойки шасси
    glLineWidth(1);
    qglColor(Gray);
    glBegin(GL_LINES);
        glVertex3f( 0.001*model.st[k].B.x,  0.001*model.st[k].B.y,  0.001*model.st[k].B.z );
        glVertex3f( 0.001*model.st[k].V.x,  0.001*model.st[k].V.y,  0.001*model.st[k].V.z );
        glVertex3f( 0.001*model.st[k].V.x,  0.001*model.st[k].V.y,  0.001*model.st[k].V.z );
        glVertex3f( 0.001*model.st[k].O.x,  0.001*model.st[k].O.y,  0.001*model.st[k].O.z );
        glVertex3f( 0.001*model.st[k].V.x,  0.001*model.st[k].V.y,  0.001*model.st[k].V.z );
        glVertex3f( 0.001*model.st[k].W.x,  0.001*model.st[k].W.y,  0.001*model.st[k].W.z );
        ///glVertex3f( 0.001*model.st[k].W.x,  0.001*model.st[k].W.y,  0.001*model.st[k].W.z );
        ///glVertex3f( 0.001*model.st[k].O.x,  0.001*model.st[k].O.y,  0.001*model.st[k].O.z );
        switch (model.st[k].nwh)
        {
          case 1:
            glVertex3f( 0.001*model.st[k].W.x,  0.001*model.st[k].W.y,  0.001*model.st[k].W.z );
            glVertex3f( 0.001*model.st[k].D.x,  0.001*model.st[k].D.y,  0.001*model.st[k].D.z );
            break;
          case 2:
            glVertex3f( 0.001*model.st[k].W.x,  0.001*model.st[k].W.y,  0.001*model.st[k].W.z );
            glVertex3f( 0.001*model.st[k].W1.x, 0.001*model.st[k].W1.y, 0.001*model.st[k].W1.z );
            glVertex3f( 0.001*model.st[k].W1.x, 0.001*model.st[k].W1.y, 0.001*model.st[k].W1.z );
            glVertex3f( 0.001*model.st[k].D1.x, 0.001*model.st[k].D1.y, 0.001*model.st[k].D1.z );
            glVertex3f( 0.001*model.st[k].W.x,  0.001*model.st[k].W.y,  0.001*model.st[k].W.z );
            glVertex3f( 0.001*model.st[k].W2.x, 0.001*model.st[k].W2.y, 0.001*model.st[k].W2.z );
            glVertex3f( 0.001*model.st[k].W2.x, 0.001*model.st[k].W2.y, 0.001*model.st[k].W2.z );
            glVertex3f( 0.001*model.st[k].D2.x, 0.001*model.st[k].D2.y, 0.001*model.st[k].D2.z );
            break;
        }
    glEnd();

    //колесо стойки шасси
    switch (model.st[k].nwh)
     {
       case 1:
        glPushMatrix();
        glTranslatef(0.001*model.st[k].W.x,  0.001*model.st[k].W.y,  0.001*model.st[k].W.z);
        glRotatef(model.st[k].ang_pn, 1.0f, 0.0f, 0.0f);
            wheel(k);
        glPopMatrix();
       break;
       case 2:
        glPushMatrix();
        glTranslatef(0.001*model.st[k].W1.x,  0.001*model.st[k].W1.y,  0.001*model.st[k].W1.z);
        glRotatef(model.st[k].ang_pn, 1.0f, 0.0f, 0.0f);
            wheel(k);
        glPopMatrix();
        glPushMatrix();
        glTranslatef(0.001*model.st[k].W2.x,  0.001*model.st[k].W2.y,  0.001*model.st[k].W2.z);
        glRotatef(model.st[k].ang_pn, 1.0f, 0.0f, 0.0f);
            wheel(k);
        glPopMatrix();
       break;
     }

    if(model.nst)//isLanding
    {
        //характерные точки для оценки кинематики амортстойки
        glPointSize(3);
        glBegin(GL_POINTS);
            qglColor(Red);
            glVertex3f( 0.001*model.st[k].Wfull.x,  0.001*model.st[k].Wfull.y,  0.001*model.st[k].Wfull.z );
        glEnd();
        glBegin(GL_POINTS);
            qglColor(Blue);
            glVertex3f( 0.001*model.st[k].Wfree.x,  0.001*model.st[k].Wfree.y,  0.001*model.st[k].Wfree.z );
        glEnd();

        glLineWidth(5);
        //амортизатор как гибкий стержень
        qglColor( ColorFromLevel( LevelFromValue( model.st[k].a.ha,  model.st[k].a.S,  0) ));
        glBegin(GL_LINES);
            glVertex3f( 0.001*model.st[k].B.x,  0.001*model.st[k].B.y,  0.001*model.st[k].B.z );
            glVertex3f( 0.001*model.st[k].A.x,  0.001*model.st[k].A.y,  0.001*model.st[k].A.z );
        glEnd();
        //пневматик как гибкий стержень
        switch (model.st[k].nwh)
        {
          case 1:
            qglColor( ColorFromLevel( LevelFromValue( model.st[k].p.h.y,  model.st[k].p.S,  0) ));
            glBegin(GL_LINES);
            glVertex3f( 0.001*model.st[k].D.x,  0.001*model.st[k].D.y,  0.001*model.st[k].D.z );
            glVertex3f( 0.001*model.st[k].C.x,  0.001*model.st[k].C.y,  0.001*model.st[k].C.z );
            glEnd();
            break;
          case 2:
            qglColor( ColorFromLevel( LevelFromValue( model.st[k].p.h1.y,  model.st[k].p.S,  0) ));
            glBegin(GL_LINES);
            glVertex3f( 0.001*model.st[k].D1.x,  0.001*model.st[k].D1.y,  0.001*model.st[k].D1.z );
            glVertex3f( 0.001*model.st[k].C1.x,  0.001*model.st[k].C1.y,  0.001*model.st[k].C1.z );
            glEnd();
            qglColor( ColorFromLevel( LevelFromValue( model.st[k].p.h2.y,  model.st[k].p.S,  0) ));
            glBegin(GL_LINES);
            glVertex3f( 0.001*model.st[k].D2.x,  0.001*model.st[k].D2.y,  0.001*model.st[k].D2.z );
            glVertex3f( 0.001*model.st[k].C2.x,  0.001*model.st[k].C2.y,  0.001*model.st[k].C2.z );
            glEnd();
            break;
        }

    /*
    //характерные точки для оценки кинематики амортстойки
    glPointSize(3);
    glBegin(GL_POINTS);
        qglColor(Red);
        glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].Wfull.x,  0.001*model.st[k].O.y + 0.001*model.st[k].Wfull.y,  0.001*model.st[k].O.z + 0.001*model.st[k].Wfull.z );
        //glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].Wfree.x,  0.001*model.st[k].O.y + 0.001*model.st[k].Wfree.y,  0.001*model.st[k].O.z + 0.001*model.st[k].Wfree.z );
    glEnd();
    glBegin(GL_POINTS);
        qglColor(Blue);
        //glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].Wfull.x,  0.001*model.st[k].O.y + 0.001*model.st[k].Wfull.y,  0.001*model.st[k].O.z + 0.001*model.st[k].Wfull.z );
        glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].Wfree.x,  0.001*model.st[k].O.y + 0.001*model.st[k].Wfree.y,  0.001*model.st[k].O.z + 0.001*model.st[k].Wfree.z );
    glEnd();

    //рычаг амортстойки
    glLineWidth(1);
    qglColor(Gray);
    glBegin(GL_LINES);
        glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].B.x,  0.001*model.st[k].O.y + 0.001*model.st[k].B.y,  0.001*model.st[k].O.z + 0.001*model.st[k].B.z );
        glVertex3f( 0.001*model.st[k].O.x,  0.001*model.st[k].O.y,  0.001*model.st[k].O.z );
        glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].B.x,  0.001*model.st[k].O.y + 0.001*model.st[k].B.y,  0.001*model.st[k].O.z + 0.001*model.st[k].B.z );
        glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].W.x,  0.001*model.st[k].O.y + 0.001*model.st[k].W.y,  0.001*model.st[k].O.z + 0.001*model.st[k].W.z );

        glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].W.x,  0.001*model.st[k].O.y + 0.001*model.st[k].W.y,  0.001*model.st[k].O.z + 0.001*model.st[k].W.z );
        glVertex3f( 0.001*model.st[k].O.x,  0.001*model.st[k].O.y,  0.001*model.st[k].O.z );

        switch (model.st[k].nwh)
        {
          case 1:
            glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].W.x,  0.001*model.st[k].O.y + 0.001*model.st[k].W.y,  0.001*model.st[k].O.z + 0.001*model.st[k].W.z );
            glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].D.x,  0.001*model.st[k].O.y + 0.001*model.st[k].D.y,  0.001*model.st[k].O.z + 0.001*model.st[k].D.z );
            break;
          case 2:
            glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].W.x,  0.001*model.st[k].O.y + 0.001*model.st[k].W.y,  0.001*model.st[k].O.z + 0.001*model.st[k].W.z );
            glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].W1.x, 0.001*model.st[k].O.y + 0.001*model.st[k].W1.y, 0.001*model.st[k].O.z + 0.001*model.st[k].W1.z );
            glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].W1.x, 0.001*model.st[k].O.y + 0.001*model.st[k].W1.y, 0.001*model.st[k].O.z + 0.001*model.st[k].W1.z );
            glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].D1.x, 0.001*model.st[k].O.y + 0.001*model.st[k].D1.y, 0.001*model.st[k].O.z + 0.001*model.st[k].D1.z );
            glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].W.x,  0.001*model.st[k].O.y + 0.001*model.st[k].W.y,  0.001*model.st[k].O.z + 0.001*model.st[k].W.z );
            glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].W2.x, 0.001*model.st[k].O.y + 0.001*model.st[k].W2.y, 0.001*model.st[k].O.z + 0.001*model.st[k].W2.z );
            glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].W2.x, 0.001*model.st[k].O.y + 0.001*model.st[k].W2.y, 0.001*model.st[k].O.z + 0.001*model.st[k].W2.z );
            glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].D2.x, 0.001*model.st[k].O.y + 0.001*model.st[k].D2.y, 0.001*model.st[k].O.z + 0.001*model.st[k].D2.z );
            break;
        }
     glEnd();

     //колесо
     switch (model.st[k].nwh)
     {
       case 1:
        glPushMatrix();
        glTranslatef(0.001*model.st[k].O.x + 0.001*model.st[k].W.x,  0.001*model.st[k].O.y + 0.001*model.st[k].W.y,  0.001*model.st[k].O.z + 0.001*model.st[k].W.z);
        glRotatef(model.st[k].ang_pn, 1.0f, 0.0f, 0.0f);
            wheel(k);
        glPopMatrix();
       break;
       case 2:
        glPushMatrix();
        glTranslatef(0.001*model.st[k].O.x + 0.001*model.st[k].W1.x,  0.001*model.st[k].O.y + 0.001*model.st[k].W1.y,  0.001*model.st[k].O.z + 0.001*model.st[k].W1.z);
        glRotatef(model.st[k].ang_pn, 1.0f, 0.0f, 0.0f);
            wheel(k);
        glPopMatrix();
        glPushMatrix();
        glTranslatef(0.001*model.st[k].O.x + 0.001*model.st[k].W2.x,  0.001*model.st[k].O.y + 0.001*model.st[k].W2.y,  0.001*model.st[k].O.z + 0.001*model.st[k].W2.z);
        glRotatef(model.st[k].ang_pn, 1.0f, 0.0f, 0.0f);
            wheel(k);
        glPopMatrix();
       break;
     }

    glLineWidth(5);
    //амортизатор как гибкий стержень
    qglColor( ColorFromLevel( LevelFromValue( model.st[k].a.ha,  model.st[k].a.S,  0) ));
    glBegin(GL_LINES);
        glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].B.x,  0.001*model.st[k].O.y + 0.001*model.st[k].B.y,  0.001*model.st[k].O.z + 0.001*model.st[k].B.z );
        glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].A.x,  0.001*model.st[k].O.y + 0.001*model.st[k].A.y,  0.001*model.st[k].O.z + 0.001*model.st[k].A.z );
    glEnd();
    //пневматик как гибкий стержень
    switch (model.st[k].nwh)
    {
      case 1:
        qglColor( ColorFromLevel( LevelFromValue( model.st[k].p.h.y,  model.st[k].p.S,  0) ));
        glBegin(GL_LINES);
        glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].D.x,  0.001*model.st[k].O.y + 0.001*model.st[k].D.y,  0.001*model.st[k].O.z + 0.001*model.st[k].D.z );
        glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].C.x,  0.001*model.st[k].O.y + 0.001*model.st[k].C.y,  0.001*model.st[k].O.z + 0.001*model.st[k].C.z );
        glEnd();
        break;
      case 2:
        qglColor( ColorFromLevel( LevelFromValue( model.st[k].p.h1.y,  model.st[k].p.S,  0) ));
        glBegin(GL_LINES);
        glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].D1.x,  0.001*model.st[k].O.y + 0.001*model.st[k].D1.y,  0.001*model.st[k].O.z + 0.001*model.st[k].D1.z );
        glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].C1.x,  0.001*model.st[k].O.y + 0.001*model.st[k].C1.y,  0.001*model.st[k].O.z + 0.001*model.st[k].C1.z );
        glEnd();
        qglColor( ColorFromLevel( LevelFromValue( model.st[k].p.h2.y,  model.st[k].p.S,  0) ));
        glBegin(GL_LINES);
        glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].D2.x,  0.001*model.st[k].O.y + 0.001*model.st[k].D2.y,  0.001*model.st[k].O.z + 0.001*model.st[k].D2.z );
        glVertex3f( 0.001*model.st[k].O.x + 0.001*model.st[k].C2.x,  0.001*model.st[k].O.y + 0.001*model.st[k].C2.y,  0.001*model.st[k].O.z + 0.001*model.st[k].C2.z );
        glEnd();
        break;
    }
       */
    }
    else
    {
        glLineWidth(1);
        //амортизатор как гибкий стержень
        glBegin(GL_LINES);
            glVertex3f( 0.001*model.st[k].B.x,  0.001*model.st[k].B.y,  0.001*model.st[k].B.z );
            glVertex3f( 0.001*model.st[k].A.x,  0.001*model.st[k].A.y,  0.001*model.st[k].A.z );
        glEnd();
    }
}
void GLWidget::wheel(int k)
{
    //внешний контур
    glPushMatrix();
        glTranslatef(0, 0, -0.006);//
        cylindr(0.0005*model.st[k].Dwh-0.005, 0.0005*model.st[k].Dwh, 0.006);//внешний контур
        cylindr(0.001 *model.st[k].rwh-0.005, 0.001 *model.st[k].rwh, 0.006);//обод
    glPopMatrix();
    glPushMatrix();
        glTranslatef(0, 0, 0);//
        cylindr(0.0005*model.st[k].Dwh, 0.0005*model.st[k].Dwh-0.005, 0.006);//внешний контур
        cylindr(0.001 *model.st[k].rwh, 0.001 *model.st[k].rwh-0.005, 0.006);//обод
    glPopMatrix();

    /*/обод -1
    glPushMatrix();
        glTranslatef(0, 0, -0.5*0.001*model.st[k].Bwh-0.006);
        cylindr(0.001*model.st[k].rwh-0.005, 0.001*model.st[k].rwh, 0.006);
    glPopMatrix();
    glPushMatrix();
        glTranslatef(0, 0, -0.5*0.001*model.st[k].Bwh);//
        cylindr(0.001*model.st[k].rwh, 0.001*model.st[k].rwh-0.005, 0.006);
    glPopMatrix();
    //обод +1
    glPushMatrix();
        glTranslatef(0, 0, 0.5*0.001*model.st[k].Bwh-0.006);//
        cylindr(0.001*model.st[k].rwh-0.005, 0.001*model.st[k].rwh, 0.006);
    glPopMatrix();
    glPushMatrix();
        glTranslatef(0, 0, 0.5*0.001*model.st[k].Bwh);//
        cylindr(0.001*model.st[k].rwh, 0.001*model.st[k].rwh-0.005, 0.006);
    glPopMatrix();*/

}

void GLWidget::mainrotor()
{
    qglColor(Gray);
    glLineWidth(1);

    //центр втулки НВ
    glPointSize(7);
    qglColor(Blue);
    glBegin(GL_POINTS);
        glVertex3f(0,0,0);
    glEnd();

    //втулка НВ
    if(!onMRcamera) mr_vtulka();

    //лопасти НВ
    if(model.mr.solve)
    {
    if(oneBlade)
    {
        glPushMatrix();
            glRotatef(-model.mr.b[0].psi0, 0.0f, 1.0f, 0.0f);

            xyz beg,prj;
            //double Lb = 0.5*model.mr.D - model.mr.l_vsh;
            //втулка
            glLineWidth(1);
            qglColor(Blue);
            glBegin(GL_LINES);
                glVertex3f(0, 0, 0);
                glVertex3f(0.001*model.mr.l_vsh, 0, 0);
            glEnd();
            //нейтраль
            glLineWidth(1);
            qglColor(Gray);
            glBegin(GL_LINES);
                glVertex3f(0.001*model.mr.l_vsh, 0, 0);
                glVertex3f(0.001*0.5*model.mr.D, 0, 0);
            glEnd();
            //3D-blade
            if(model.mr.solve)
            {
                 if(visFmr)
                 {
                     //вектора на лопастях
                     glLineWidth(1);
                     glPushMatrix();
                         glTranslatef(0.001*model.mr.l_vsh, 0, 0);
                         //погонные аэродин силы
                         for (int j=0; j<mrbldneut->size()-2; j++)//
                         {
                             neut0 = mrbldneut->at(j+1);//mrbldneut
                             ///U - скорости притекания
                             beg.x = neut0.x ;
                             beg.y = neut0.y - 1*mv*model.mr.b[0].Uy.at(j);
                             beg.z = neut0.z + 1*mv*model.mr.b[0].Ux.at(j);
                             prj.x = 0;
                             prj.y = 1*mv*model.mr.b[0].Uy.at(j);//0.005
                             prj.z = 1*mv*model.mr.b[0].Ux.at(j);//0.005
                             vector_(1, lightBlue,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                             ///dT
                             beg.x = neut0.x;
                             beg.y = neut0.y;
                             beg.z = neut0.z;
                             prj.x = 0;
                             prj.y = mv*model.mr.b[0].dT_dr.at(j);//0.0005
                             prj.z = 0;
                             vector_(1, Green,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                             ///dQ
                             beg.x = neut0.x;
                             beg.y = neut0.y;
                             beg.z = neut0.z;
                             prj.x = 0;
                             prj.y = 0;//0.0005
                             prj.z = mv*model.mr.b[0].dQ_dr.at(j);
                             vector_(1, Brown,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                         }
                         //подмена данных на крайнем внешнем сечении
                         {
                             neut0 = mrbldneut->at(mrbldneut->size()-1);
                             int j = model.mr.nSec-1;
                             ///U - скорости притекания
                             beg.x = neut0.x ;
                             beg.y = neut0.y - 1*mv*model.mr.b[0].Uy.at(j);
                             beg.z = neut0.z + 1*mv*model.mr.b[0].Ux.at(j);
                             prj.x = 0;
                             prj.y = 1*mv*model.mr.b[0].Uy.at(j);//0.005
                             prj.z = 1*mv*model.mr.b[0].Ux.at(j);//0.005
                             vector_(1, lightBlue,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                             ///dT
                             beg.x = neut0.x;
                             beg.y = neut0.y;
                             beg.z = neut0.z;
                             prj.x = 0;
                             prj.y = mv*model.mr.b[0].dT_dr.at(j);//0.0005
                             prj.z = 0;
                             vector_(1, Green,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                             ///dQ
                             beg.x = neut0.x;
                             beg.y = neut0.y;
                             beg.z = neut0.z;
                             prj.x = 0;
                             prj.y = 0;//0.0005
                             prj.z = mv*model.mr.b[0].dQ_dr.at(j);
                             vector_(1, Brown,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                         }
                         //инерционные силы
                         {
                         neut0 = mrbldneut->at(model.mr.sec07);//mrbldneut
                         beg.x = neut0.x;
                         beg.y = neut0.y;
                         beg.z = neut0.z;
                         //инерция от ускорений вала НВ
                         prj.x = 0;
                         prj.y = mv*model.mr.b[0].Payy;//в плоскости тяги
                         prj.z = 0;
                         vector_(1, Black,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                         prj.x = 0;
                         prj.y = 0;
                         prj.z = mv*model.mr.b[0].Paxz;//в плоскости винта
                         vector_(1, Black,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                         //P1 инерция взмаха
                         prj.x = 0;
                         prj.y = mv*model.mr.b[0].P1;
                         prj.z = 0;
                         vector_(1, Red,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                         //P2 инерция качания
                         prj.x = 0;
                         prj.y = 0;
                         prj.z = mv*model.mr.b[0].P2;
                         vector_(1, Orange,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                         //K1 сила Кориолиса от взмаха
                         prj.x = 0;
                         prj.y = 0;
                         prj.z = mv*model.mr.b[0].K1;
                         vector_(1, Brown,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                         }
                     glPopMatrix();
                 }

                 //контур лопасти
                 glLineWidth(1);
                 glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
                 glPushMatrix();
                    glTranslatef(0.001*model.mr.l_vsh, 0, 0);
                    //glRotatef( model.mr.b[k].ksi, 0.0f, 1.0f, 0.0f);
                    //glRotatef( model.mr.b[k].beta, 0.0f, 0.0f, 1.0f);
                    glRotatef(-(model.mr.b[0].fi7 + model.mr.b[0].gama), 1.0f, 0.0f, 0.0f);

                    qglColor(Gray);
                    glBegin(GL_LINES);
                    for (int j=0; j<mrbldneut->size()-1; j++)//
                    {
                        frnt0 = mrbldfrnt->at(j);
                        neut0 = mrbldneut->at(j);
                        rear0 = mrbldrear->at(j);
                        frnt1 = mrbldfrnt->at(j+1);
                        neut1 = mrbldneut->at(j+1);
                        rear1 = mrbldrear->at(j+1);

                        glVertex3f(neut0.x,neut0.y,neut0.z);
                        glVertex3f(frnt0.x,frnt0.y,frnt0.z);
                        glVertex3f(frnt0.x,frnt0.y,frnt0.z);
                        glVertex3f(frnt1.x,frnt1.y,frnt1.z);
                        glVertex3f(frnt1.x,frnt1.y,frnt1.z);
                        glVertex3f(neut1.x,neut1.y,neut1.z);
                        glVertex3f(neut1.x,neut1.y,neut1.z);
                        glVertex3f(neut0.x,neut0.y,neut0.z);

                        glVertex3f(neut0.x,neut0.y,neut0.z);
                        glVertex3f(rear0.x,rear0.y,rear0.z);
                        glVertex3f(rear0.x,rear0.y,rear0.z);
                        glVertex3f(rear1.x,rear1.y,rear1.z);
                        glVertex3f(rear1.x,rear1.y,rear1.z);
                        glVertex3f(neut1.x,neut1.y,neut1.z);
                        glVertex3f(neut1.x,neut1.y,neut1.z);
                        glVertex3f(neut0.x,neut0.y,neut0.z);

                    }
                    glEnd();

                    qglColor(bladecolor);
                    glBegin(GL_QUADS);
                    for (int j=0; j<mrbldneut->size()-1; j++)//
                    {
                        frnt0 = mrbldfrnt->at(j);
                        neut0 = mrbldneut->at(j);
                        rear0 = mrbldrear->at(j);
                        frnt1 = mrbldfrnt->at(j+1);
                        neut1 = mrbldneut->at(j+1);
                        rear1 = mrbldrear->at(j+1);

                        glVertex3f(neut0.x,neut0.y,neut0.z);
                        glVertex3f(frnt0.x,frnt0.y,frnt0.z);
                        glVertex3f(frnt1.x,frnt1.y,frnt1.z);
                        glVertex3f(neut1.x,neut1.y,neut1.z);
                        glVertex3f(neut0.x,neut0.y,neut0.z);
                        glVertex3f(rear0.x,rear0.y,rear0.z);
                        glVertex3f(rear1.x,rear1.y,rear1.z);
                        glVertex3f(neut1.x,neut1.y,neut1.z);
                    }
                    glEnd();

                glPopMatrix();
            }
        glPopMatrix();
    }
    else
    {
        for (int bl=0; bl<model.mr.nb; bl++)
        {
            glPushMatrix();
            if(onMRcamera)
                glRotatef(-model.mr.b[bl].psi0, 0.0f, 1.0f, 0.0f);
            else
                glRotatef(-model.mr.b[bl].psi, 0.0f, 1.0f, 0.0f);
            mr_blades(bl);
            glPopMatrix();
        }
    }
    }

    //вектор набегающего потока
    //float mrFlow=3;   xyz beg,prj;   float d2r = M_PI/180.0;
    //beg.x = 0  - mrFlow*cos(model.mr.alfa*d2r)*cos(model.mr.beta*d2r);
    //beg.y = 0  - mrFlow*sin(model.mr.alfa*d2r);
    //beg.z = 0  + mrFlow*sin(model.mr.beta*d2r);
    //prj.x = 0.95*mrFlow*cos(model.mr.alfa*d2r)*cos(model.mr.beta*d2r);
    //prj.y = 0.95*mrFlow*sin(model.mr.alfa*d2r);
    //prj.z = 0.95*mrFlow*sin(model.mr.beta*d2r);
    //vector_(1, Blue,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
}
void GLWidget::mr_axle()
{
    float ml=0.15*m;
    float sl=0.15*m;
    //основные линии
    glLineWidth(2);
    glBegin( GL_LINES );
        qglColor(Red);          glVertex3f(0,0,0);    glVertex3f(ml, 0, 0);
        qglColor(Green);        glVertex3f(0,0,0);    glVertex3f(0, 0.5*ml, 0);
        qglColor(Blue);         glVertex3f(0,0,0);    glVertex3f(0, 0, ml);
    glEnd();
    //добавленные
    glLineWidth(2);
    glBegin( GL_LINES );
        qglColor(halfRed);      glVertex3f(0,0,0);    glVertex3f(-sl, 0, 0);
        qglColor(halfGreen);    glVertex3f(0,0,0);    glVertex3f(0, -0.5*sl, 0);
        qglColor(halfBlue);     glVertex3f(0,0,0);    glVertex3f(0, 0, -sl);
    glEnd();
}
void GLWidget::mr_ring()
{//кольцо - кромка концов лопастей
    qglColor(Gray);
    glPushMatrix();
        glRotatef( 90, 1.0f, 0.0f, 0.0f);
        glTranslatef(0, 0, -0.005);//0.010
        cylindr(0.0005*model.mr.D-0.003, 0.0005*model.mr.D, 0.005);
    glPopMatrix();
    glPushMatrix();
        glRotatef( 90, 1.0f, 0.0f, 0.0f);
        glTranslatef(0, 0, 0);
        //glBegin(GL_QUADS);
        cylindr(0.0005*model.mr.D, 0.0005*model.mr.D-0.003, 0.005);
    glPopMatrix();
}
void GLWidget::mr_vtulka()
{
    /* old /радиус-вектор центра тяжести системы лопастей
    glLineWidth(1);
    qglColor(Black);
    glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(-0.001*model.mr.dct.x, -0.001*model.mr.dct.y, -0.001*model.mr.dct.z);
    glEnd();
    //"наконечник" радиус-вектора
    qglColor(Red);
    glPointSize(9);
    glBegin(GL_POINTS);
        glVertex3f(-0.001*model.mr.dct.x, -0.001*model.mr.dct.y, -0.001*model.mr.dct.z);
    glEnd();*/

    glPushMatrix();
    glRotatef(-model.mr.psih, 0.0f, 1.0f, 0.0f);// b[0].psi0

        ///test
        qglColor(Gray);
        glPointSize(5);

        glBegin(GL_POINTS);
            glVertex3f( 0.001*100,  0,  0);
            glVertex3f( 0.001* 50,  0,  -0.001* 50);
        glEnd();

    //радиус-вектор центра тяжести системы лопастей
    glLineWidth(1);
    qglColor(Black);
    glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f( 0.001*model.mr.dct.x, 0.001*model.mr.dct.y, -0.001*model.mr.dct.z );
    glEnd();
    //"наконечник" радиус-вектора
    qglColor(Green);
    glPointSize(9);
    glBegin(GL_POINTS);
        glVertex3f( 0.001*model.mr.dct.x, 0.001*model.mr.dct.y, -0.001*model.mr.dct.z );
    glEnd();

    glPopMatrix();


    if(visFmr)
    {
    //вектора в центре втулки
        vector_(3, Red,   0,0,0,  mv*model.mr.Fhx_sum, 0, 0);//H
        vector_(3, Green, 0,0,0,  0, mv*model.mr.Fhy_sum, 0);//T
        vector_(3, Blue,  0,0,0,  0, 0, mv*model.mr.Fhz_sum);//S
    }
}
void GLWidget::mr_blades(int k)
{
    xyz beg,prj;
    double Lb = 0.5*model.mr.D - model.mr.l_vsh;
    //вертикальный шарнир
    glPointSize(5);
    qglColor(Blue);
    glBegin(GL_POINTS);
        glVertex3f(0.001*model.mr.l_vsh, 0, 0);
    glEnd();

    //втулка
    glLineWidth(1);
    qglColor(Blue);
    glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(0.001*model.mr.l_vsh, 0, 0);
    glEnd();

    //нейтраль
    glLineWidth(1);
    qglColor(Gray);
    glBegin(GL_LINES);
        glVertex3f(0.001*model.mr.l_vsh, 0, 0);
        glVertex3f(0.001*0.5*model.mr.D, 0, 0);
    glEnd();

    if(model.mr.solve)///3D-blade
    {
        if(visFmr)
        {
            //вектора в шарнирах
            glLineWidth(1);
            glPushMatrix();
                glTranslatef(0.001*model.mr.l_vsh, 0, 0);
                vector_(2, Gray, 0,0,0,  0, 0, -mv*model.mr.b[k].Fx);///x
                vector_(2, Gray, 0,0,0,  0, mv*model.mr.b[k].Fy, -0);///y
                //vector_(3, Gray, 0,0,0,  mv*model.mr.b[k].Fz, 0, -0);///z
            glPopMatrix();

            //вектора на лопастях
            glLineWidth(1);
            glPushMatrix();
                glTranslatef(0.001*model.mr.l_vsh, 0, 0);
                glRotatef( model.mr.b[k].ksi,  0.0f, 1.0f, 0.0f);
                glRotatef( model.mr.b[k].beta, 0.0f, 0.0f, 1.0f);
                //погонные аэродин силы
                for (int j=0; j<mrbldneut->size()-2; j++)//for (int j=1; j<mrbldneut->size(); j++)
                {
                    neut0 = mrbldneut->at(j+1);//mrbldneut
                    ///U - скорости притекания
                    beg.x = neut0.x ;
                    beg.y = neut0.y - 1*mv*model.mr.b[k].Uy.at(j);
                    beg.z = neut0.z + 1*mv*model.mr.b[k].Ux.at(j);
                    prj.x = 0;
                    prj.y = 1*mv*model.mr.b[k].Uy.at(j);
                    prj.z = 1*mv*model.mr.b[k].Ux.at(j);
                    vector_(1, lightBlue,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                    ///dT
                    beg.x = neut0.x;
                    beg.y = neut0.y;
                    beg.z = neut0.z;
                    prj.x = 0;
                    prj.y = mv*model.mr.b[k].dT_dr.at(j);
                    prj.z = 0;
                    vector_(1, Green,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                    ///dQ
                    beg.x = neut0.x;
                    beg.y = neut0.y;
                    beg.z = neut0.z;
                    prj.x = 0;
                    prj.y = 0;//0.0005
                    prj.z = mv*model.mr.b[k].dQ_dr.at(j);
                    vector_(1, Brown,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                }
                //подмена данных на крайнем внешнем отображаемом сечении на реальное крайнее сечение
                {
                    neut0 = mrbldneut->at(mrbldneut->size()-1);
                    int j = model.mr.nSec-1;
                    ///U - скорости притекания
                    beg.x = neut0.x ;
                    beg.y = neut0.y - 1*mv*model.mr.b[k].Uy.at(j);
                    beg.z = neut0.z + 1*mv*model.mr.b[k].Ux.at(j);
                    prj.x = 0;
                    prj.y = 1*mv*model.mr.b[k].Uy.at(j);
                    prj.z = 1*mv*model.mr.b[k].Ux.at(j);
                    vector_(1, lightBlue,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                    ///dT
                    beg.x = neut0.x;
                    beg.y = neut0.y;
                    beg.z = neut0.z;
                    prj.x = 0;
                    prj.y = mv*model.mr.b[k].dT_dr.at(j);
                    prj.z = 0;
                    vector_(1, Green,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                    ///dQ
                    beg.x = neut0.x;
                    beg.y = neut0.y;
                    beg.z = neut0.z;
                    prj.x = 0;
                    prj.y = 0;//0.0005
                    prj.z = mv*model.mr.b[k].dQ_dr.at(j);
                    vector_(1, Brown,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                }
                //инерционные силы
                {
                neut0 = mrbldneut->at(model.mr.sec07);//mrbldneut
                beg.x = neut0.x;
                beg.y = neut0.y;
                beg.z = neut0.z;
                //инерция от ускорений вала НВ
                prj.x = 0;
                prj.y = mv*model.mr.b[k].Payy;//в плоскости тяги
                prj.z = 0;
                vector_(1, Black,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                prj.x = 0;
                prj.y = 0;
                prj.z = mv*model.mr.b[k].Paxz;//в плоскости винта
                vector_(1, Black,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                //P1 инерция взмаха
                prj.x = 0;
                prj.y = mv*model.mr.b[k].P1;
                prj.z = 0;
                vector_(1, Red,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                //P2 инерция качания
                prj.x = 0;
                prj.y = 0;
                prj.z = mv*model.mr.b[k].P2;
                vector_(1, Orange,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                //K1 сила Кориолиса от взмаха
                prj.x = 0;
                prj.y = 0;
                prj.z = mv*model.mr.b[k].K1;
                vector_(1, Brown,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                }
            glPopMatrix();
        }

        //контур лопасти
        glLineWidth(1);
        glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
        glPushMatrix();
            glTranslatef(0.001*model.mr.l_vsh, 0, 0);
            glRotatef( model.mr.b[k].ksi,  0.0f, 1.0f, 0.0f);
            glRotatef( model.mr.b[k].beta, 0.0f, 0.0f, 1.0f);
            glRotatef(-(model.mr.b[k].fi7 + model.mr.b[k].gama), 1.0f, 0.0f, 0.0f);
            //контур
            qglColor(Gray);
            glBegin(GL_LINES);
            for (int j=0; j<mrbldneut->size()-1; j++)//
            {
                frnt0 = mrbldfrnt->at(j);
                neut0 = mrbldneut->at(j);
                rear0 = mrbldrear->at(j);
                frnt1 = mrbldfrnt->at(j+1);
                neut1 = mrbldneut->at(j+1);
                rear1 = mrbldrear->at(j+1);

                glVertex3f(neut0.x,neut0.y,neut0.z);
                glVertex3f(frnt0.x,frnt0.y,frnt0.z);
                glVertex3f(frnt0.x,frnt0.y,frnt0.z);
                glVertex3f(frnt1.x,frnt1.y,frnt1.z);
                glVertex3f(frnt1.x,frnt1.y,frnt1.z);
                glVertex3f(neut1.x,neut1.y,neut1.z);
                glVertex3f(neut1.x,neut1.y,neut1.z);
                glVertex3f(neut0.x,neut0.y,neut0.z);

                glVertex3f(neut0.x,neut0.y,neut0.z);
                glVertex3f(rear0.x,rear0.y,rear0.z);
                glVertex3f(rear0.x,rear0.y,rear0.z);
                glVertex3f(rear1.x,rear1.y,rear1.z);
                glVertex3f(rear1.x,rear1.y,rear1.z);
                glVertex3f(neut1.x,neut1.y,neut1.z);
                glVertex3f(neut1.x,neut1.y,neut1.z);
                glVertex3f(neut0.x,neut0.y,neut0.z);
            }
            glEnd();
            //заполнение
            qglColor(bladecolor);
            glBegin(GL_QUADS);
            for (int j=0; j<mrbldneut->size()-1; j++)//
            {
                frnt0 = mrbldfrnt->at(j);
                neut0 = mrbldneut->at(j);
                rear0 = mrbldrear->at(j);
                frnt1 = mrbldfrnt->at(j+1);
                neut1 = mrbldneut->at(j+1);
                rear1 = mrbldrear->at(j+1);

                glVertex3f(neut0.x,neut0.y,neut0.z);
                glVertex3f(frnt0.x,frnt0.y,frnt0.z);
                glVertex3f(frnt1.x,frnt1.y,frnt1.z);
                glVertex3f(neut1.x,neut1.y,neut1.z);
                glVertex3f(neut0.x,neut0.y,neut0.z);
                glVertex3f(rear0.x,rear0.y,rear0.z);
                glVertex3f(rear1.x,rear1.y,rear1.z);
                glVertex3f(neut1.x,neut1.y,neut1.z);
            }
            glEnd();
            //метки на концах лопасти
            glPointSize(3);
            qglColor(Blue);
            glBegin(GL_POINTS);
            switch (k)
            {
            case 0://1 лопасть
                    glVertex3f(0.001*(Lb+50), 0, 0);
            break;
            case 1://2 лопасти
                    glVertex3f(0.001*(Lb+50), 0,-0.025);
                    glVertex3f(0.001*(Lb+50), 0, 0.025);
            break;
            case 2://3 лопасти
                    glVertex3f(0.001*(Lb+50), 0,-0.050);
                    glVertex3f(0.001*(Lb+50), 0, 0);
                    glVertex3f(0.001*(Lb+50), 0, 0.050);
            break;
            case 3://4 лопасти
                    glVertex3f(0.001*(Lb+50), 0,-0.075);
                    glVertex3f(0.001*(Lb+50), 0,-0.025);
                    glVertex3f(0.001*(Lb+50), 0, 0.025);
                    glVertex3f(0.001*(Lb+50), 0, 0.075);
            break;
            case 4://5 лопасти
                    glVertex3f(0.001*(Lb+50), 0,-0.100);
                    glVertex3f(0.001*(Lb+50), 0,-0.050);
                    glVertex3f(0.001*(Lb+50), 0, 0);
                    glVertex3f(0.001*(Lb+50), 0, 0.050);
                    glVertex3f(0.001*(Lb+50), 0, 0.100);
            break;
            case 5://6 лопастей
                    glVertex3f(0.001*(Lb+50), 0,-0.125);
                    glVertex3f(0.001*(Lb+50), 0,-0.075);
                    glVertex3f(0.001*(Lb+50), 0,-0.025);
                    glVertex3f(0.001*(Lb+50), 0, 0.025);
                    glVertex3f(0.001*(Lb+50), 0, 0.075);
                    glVertex3f(0.001*(Lb+50), 0, 0.125);
            break;
            }
            glEnd();
        glPopMatrix();
    }
    else///1D-blade
    {
        glLineWidth(3);
        qglColor(Green);
        glBegin(GL_LINES);
            glVertex3f(0.001*model.mr.l_vsh, 0, 0);
            glVertex3f(-0.001*Lb*cos(model.mr.b[k].ksi*M_PI/180.0), 0, -0.001*Lb*sin(model.mr.b[k].ksi*M_PI/180.0));
        glEnd();
    }
}

void GLWidget::tailrotor()
{
    qglColor(Gray);
    glLineWidth(1);

    //центр втулки PВ
    glPointSize(7);
    qglColor(Blue);
    glBegin(GL_POINTS);
        glVertex3f(0,0,0);
    glEnd();

    if(visFtr && !onTRcamera)//RGB-вектора в центре втулки
    {
        vector_(3, Red,   0,0,0,  mv*model.tr.Fhx_sum, 0, 0);//H
        vector_(3, Green, 0,0,0,  0, mv*model.tr.Fhy_sum, 0);//T
        vector_(3, Blue,  0,0,0,  0, 0, mv*model.tr.Fhz_sum);//S
    }

    //лопасти РВ
    for (int bl=0; bl<model.tr.nb; bl++)
    {
        glPushMatrix();
        if(onTRcamera)
            glRotatef( -model.tr.b[bl].psi0, 0.0f, 1.0f, 0.0f);
        else
            glRotatef( -model.tr.b[bl].psi,  0.0f, 1.0f, 0.0f);
        tr_blades(bl);
        glPopMatrix();
    }

    //вектор набегающего потока
    //float trFlow=2;   xyz beg,prj;   float d2r = M_PI/180.0;
    //beg.x = 0  - trFlow*cos(model.tr.alfa*d2r)*cos(model.tr.beta*d2r);
    //beg.y = 0  - trFlow*sin(model.tr.alfa*d2r);
    //beg.z = 0  + trFlow*sin(model.tr.beta*d2r);
    //prj.x = 0.95*trFlow*cos(model.tr.alfa*d2r)*cos(model.tr.beta*d2r);
    //prj.y = 0.95*trFlow*sin(model.tr.alfa*d2r);
    //prj.z = 0.95*trFlow*sin(model.tr.beta*d2r);
    //vector_(1, Blue,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);

}
void GLWidget::tr_axle()
{
    float ml=0.1*m;///0.15
    float sl=0.1*m;///0.15
    //основные линии
    glLineWidth(2);
    glBegin( GL_LINES );
        qglColor(Red);          glVertex3f(0,0,0);    glVertex3f(0.5*ml, 0, 0);
        qglColor(Green);        glVertex3f(0,0,0);    glVertex3f(0, 0.5*ml, 0);
        qglColor(Blue);         glVertex3f(0,0,0);    glVertex3f(0, 0, 0.4*ml);
    glEnd();
    //добавленные
    glLineWidth(2);
    glBegin( GL_LINES );
        qglColor(halfRed);      glVertex3f(0,0,0);    glVertex3f(-0.5*sl, 0, 0);
        qglColor(halfGreen);    glVertex3f(0,0,0);    glVertex3f(0, -0.5*sl, 0);
        qglColor(halfBlue);     glVertex3f(0,0,0);    glVertex3f(0, 0, -0.4*sl);
    glEnd();
}
void GLWidget::tr_ring()
{//кольцо - кромка концов лопастей
    qglColor(Gray);
    glPushMatrix();
        glRotatef( 90.0,  1.0f, 0.0f, 0.0f);
        glTranslatef(0, 0, -0.005);//
        cylindr(0.0005*model.tr.D-0.003, 0.0005*model.tr.D, 0.005);
    glPopMatrix();
    glPushMatrix();
        glRotatef( 90.0,  1.0f, 0.0f, 0.0f);
        glTranslatef(0, 0, 0);//
        cylindr(0.0005*model.tr.D, 0.0005*model.tr.D-0.003, 0.005);
    glPopMatrix();
}
void GLWidget::tr_blades(int k)
{
    xyz beg,prj;
    double Lb = 0.5*model.tr.D - model.tr.l_vsh;
    //вертикальный шарнир
    glPointSize(5);
    qglColor(Blue);
    glBegin(GL_POINTS);
        glVertex3f(0.001*model.tr.l_vsh, 0, 0);
    glEnd();

   //втулка
   glLineWidth(1);
   qglColor(Blue);
   glBegin(GL_LINES);
       glVertex3f(0, 0, 0);
       glVertex3f(0.001*model.tr.l_vsh, 0, 0);
   glEnd();

   //нейтраль
   glLineWidth(1);
   qglColor(Gray);
   glBegin(GL_LINES);
       glVertex3f(0.001*model.tr.l_vsh, 0, 0);
       glVertex3f(0.001*0.5*model.tr.D, 0, 0);
   glEnd();

   if(model.tr.solve)///3D-blade
   {
        if(visFtr)
        {
            //if(visF)//вектора в шарнирах
            //{
            glLineWidth(1);
            glPushMatrix();
                glTranslatef(0.001*model.tr.l_vsh, 0, 0);
                vector_(2, Gray, 0,0,0,  0, 0, -mv*model.tr.b[k].Fx);///x
                vector_(2, Gray, 0,0,0,  0, mv*model.tr.b[k].Fy, -0);///y
            glPopMatrix();
            //}

            //вектора на лопастях
            glLineWidth(1);
            glPushMatrix();
                glTranslatef(0.001*model.tr.l_vsh, 0, 0);
                glRotatef( model.tr.b[k].ksi,  0.0f, 1.0f, 0.0f);
                glRotatef( model.tr.b[k].beta, 0.0f, 0.0f, 1.0f);
                //погонные аэродин силы
                for (int j=0; j<trbldneut->size()-2; j++)//
                {
                    neut0 = trbldneut->at(j+1);
                    ///U - скорости притекания
                    beg.x = neut0.x ;
                    beg.y = neut0.y - 1*mv*model.tr.b[k].Uy.at(j);
                    beg.z = neut0.z + 1*mv*model.tr.b[k].Ux.at(j);
                    prj.x = 0;
                    prj.y = 1*mv*model.tr.b[k].Uy.at(j);
                    prj.z = 1*mv*model.tr.b[k].Ux.at(j);
                    vector_(1, lightBlue,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                    ///dT
                    beg.x = neut0.x;
                    beg.y = neut0.y;
                    beg.z = neut0.z;
                    prj.x = 0;
                    prj.y = mv*model.tr.b[k].dT_dr.at(j);
                    prj.z = 0;
                    vector_(1, Green,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                    ///dQ
                    beg.x = neut0.x;
                    beg.y = neut0.y;
                    beg.z = neut0.z;
                    prj.x = 0;
                    prj.y = 0;//0.0005
                    prj.z = mv*model.tr.b[k].dQ_dr.at(j);
                    vector_(1, Brown,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                }
                //подмена данных на крайнем внешнем отображаемом сечении на реальное крайнее сечение
                {
                    neut0 = trbldneut->at(trbldneut->size()-1);
                    int j = model.tr.nSec-1;
                    ///U - скорости притекания
                    beg.x = neut0.x ;
                    beg.y = neut0.y - 1*mv*model.tr.b[k].Uy.at(j);
                    beg.z = neut0.z + 1*mv*model.tr.b[k].Ux.at(j);
                    prj.x = 0;
                    prj.y = 1*mv*model.tr.b[k].Uy.at(j);
                    prj.z = 1*mv*model.tr.b[k].Ux.at(j);
                    vector_(1, lightBlue,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                    ///dT
                    beg.x = neut0.x;
                    beg.y = neut0.y;
                    beg.z = neut0.z;
                    prj.x = 0;
                    prj.y = mv*model.tr.b[k].dT_dr.at(j);
                    prj.z = 0;
                    vector_(1, Green,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                    ///dQ
                    beg.x = neut0.x;
                    beg.y = neut0.y;
                    beg.z = neut0.z;
                    prj.x = 0;
                    prj.y = 0;//0.0005
                    prj.z = mv*model.tr.b[k].dQ_dr.at(j);
                    vector_(1, Brown,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                }
                //инерционные силы
                {
                neut0 = trbldneut->at(model.tr.sec07);//trbldneut
                beg.x = neut0.x;
                beg.y = neut0.y;
                beg.z = neut0.z;
                //инерция от ускорений вала НВ
                prj.x = 0;
                prj.y = mv*model.tr.b[k].Payy;//в плоскости тяги
                prj.z = 0;
                vector_(1, Black,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                prj.x = 0;
                prj.y = 0;
                prj.z = mv*model.tr.b[k].Paxz;//в плоскости винта
                vector_(1, Black,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                //P1 инерция взмаха
                prj.x = 0;
                prj.y = mv*model.tr.b[k].P1;
                prj.z = 0;
                vector_(1, Red,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                //P2 инерция качания
                prj.x = 0;
                prj.y = 0;
                prj.z = mv*model.tr.b[k].P2;
                vector_(1, Orange,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                //K1 сила Кориолиса от взмаха
                prj.x = 0;
                prj.y = 0;
                prj.z = mv*model.tr.b[k].K1;
                vector_(1, Brown,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                }
            glPopMatrix();
        }

        //контур лопасти
        glLineWidth(1);
        glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
        glPushMatrix();
           glTranslatef(0.001*model.tr.l_vsh, 0, 0);
           glRotatef( model.tr.b[k].ksi, 0.0f, 1.0f, 0.0f);
           glRotatef( model.tr.b[k].beta, 0.0f, 0.0f, 1.0f);
           glRotatef(-(model.tr.b[k].fi7 + model.tr.b[k].gama), 1.0f, 0.0f, 0.0f);

           qglColor(Gray);
           glBegin(GL_LINES);
           for (int j=0; j<trbldneut->size()-1; j++)//
           {
               frnt0 = trbldfrnt->at(j);
               neut0 = trbldneut->at(j);
               rear0 = trbldrear->at(j);
               frnt1 = trbldfrnt->at(j+1);
               neut1 = trbldneut->at(j+1);
               rear1 = trbldrear->at(j+1);

               glVertex3f(neut0.x,neut0.y,neut0.z);
               glVertex3f(frnt0.x,frnt0.y,frnt0.z);
               glVertex3f(frnt0.x,frnt0.y,frnt0.z);
               glVertex3f(frnt1.x,frnt1.y,frnt1.z);
               glVertex3f(frnt1.x,frnt1.y,frnt1.z);
               glVertex3f(neut1.x,neut1.y,neut1.z);
               glVertex3f(neut1.x,neut1.y,neut1.z);
               glVertex3f(neut0.x,neut0.y,neut0.z);

               glVertex3f(neut0.x,neut0.y,neut0.z);
               glVertex3f(rear0.x,rear0.y,rear0.z);
               glVertex3f(rear0.x,rear0.y,rear0.z);
               glVertex3f(rear1.x,rear1.y,rear1.z);
               glVertex3f(rear1.x,rear1.y,rear1.z);
               glVertex3f(neut1.x,neut1.y,neut1.z);
               glVertex3f(neut1.x,neut1.y,neut1.z);
               glVertex3f(neut0.x,neut0.y,neut0.z);

           }
           glEnd();

           qglColor(bladecolor);
           glBegin(GL_QUADS);
           for (int j=0; j<trbldneut->size()-1; j++)//
           {
               frnt0 = trbldfrnt->at(j);
               neut0 = trbldneut->at(j);
               rear0 = trbldrear->at(j);
               frnt1 = trbldfrnt->at(j+1);
               neut1 = trbldneut->at(j+1);
               rear1 = trbldrear->at(j+1);

               glVertex3f(neut0.x,neut0.y,neut0.z);
               glVertex3f(frnt0.x,frnt0.y,frnt0.z);
               glVertex3f(frnt1.x,frnt1.y,frnt1.z);
               glVertex3f(neut1.x,neut1.y,neut1.z);
               glVertex3f(neut0.x,neut0.y,neut0.z);
               glVertex3f(rear0.x,rear0.y,rear0.z);
               glVertex3f(rear1.x,rear1.y,rear1.z);
               glVertex3f(neut1.x,neut1.y,neut1.z);
           }
           glEnd();

           glPointSize(3);
           qglColor(Blue);
           glBegin(GL_POINTS);
           switch (k)
           {
           case 0://1 лопасть
                   glVertex3f(0.001*(Lb+50), 0, 0);
           break;
           case 1://2 лопасти
                   glVertex3f(0.001*(Lb+50), 0,-0.025);
                   glVertex3f(0.001*(Lb+50), 0, 0.025);
           break;
           case 2://3 лопасти
                   glVertex3f(0.001*(Lb+50), 0,-0.050);
                   glVertex3f(0.001*(Lb+50), 0, 0);
                   glVertex3f(0.001*(Lb+50), 0, 0.050);
           break;
           case 3://4 лопасти
                   glVertex3f(0.001*(Lb+50), 0,-0.075);
                   glVertex3f(0.001*(Lb+50), 0,-0.025);
                   glVertex3f(0.001*(Lb+50), 0, 0.025);
                   glVertex3f(0.001*(Lb+50), 0, 0.075);
           break;
           case 4://5 лопасти
                   glVertex3f(0.001*(Lb+50), 0,-0.100);
                   glVertex3f(0.001*(Lb+50), 0,-0.050);
                   glVertex3f(0.001*(Lb+50), 0, 0);
                   glVertex3f(0.001*(Lb+50), 0, 0.050);
                   glVertex3f(0.001*(Lb+50), 0, 0.100);
           break;
           case 5://6 лопастей
                   glVertex3f(0.001*(Lb+50), 0,-0.125);
                   glVertex3f(0.001*(Lb+50), 0,-0.075);
                   glVertex3f(0.001*(Lb+50), 0,-0.025);
                   glVertex3f(0.001*(Lb+50), 0, 0.025);
                   glVertex3f(0.001*(Lb+50), 0, 0.075);
                   glVertex3f(0.001*(Lb+50), 0, 0.125);
           break;
           }
           glEnd();
       glPopMatrix();
   }
   else///1D-blade
   {
       glLineWidth(3);
       qglColor(Green);
       glBegin(GL_LINES);
           glVertex3f(0.001*model.tr.l_vsh, 0, 0);
           glVertex3f(-0.001*Lb*cos(model.tr.b[k].ksi*M_PI/180.0), 0, -0.001*Lb*sin(model.tr.b[k].ksi*M_PI/180.0));
       glEnd();
   }
}


void GLWidget::pushrotor()
{
    qglColor(Gray);
    glLineWidth(1);

    //центр втулки PВ
    glPointSize(7);
    qglColor(Blue);
    glBegin(GL_POINTS);
        glVertex3f(0,0,0);
    glEnd();

    if(visFpr && !onPRcamera)//RGB-вектора в центре втулки
    {
        vector_(3, Red,   0,0,0,  mv*model.pr.Fhx_sum, 0, 0);//H
        vector_(3, Green, 0,0,0,  0, mv*model.pr.Fhy_sum, 0);//T
        vector_(3, Blue,  0,0,0,  0, 0, mv*model.pr.Fhz_sum);//S
    }

    //лопасти TВ
    for (int bl=0; bl<model.pr.nb; bl++)
    {
        glPushMatrix();
        if(onPRcamera)
            glRotatef(-model.pr.b[bl].psi0, 0.0f, 1.0f, 0.0f);
        else
            glRotatef(-model.pr.b[bl].psi, 0.0f, 1.0f, 0.0f);
        pr_blades(bl);
        glPopMatrix();
    }

    //вектор набегающего потока
    //float trFlow=2;   xyz beg,prj;   float d2r = M_PI/180.0;
    //beg.x = 0  - trFlow*cos(model.pr.alfa*d2r)*cos(model.pr.beta*d2r);
    //beg.y = 0  - trFlow*sin(model.pr.alfa*d2r);
    //beg.z = 0  + trFlow*sin(model.pr.beta*d2r);
    //prj.x = 0.95*trFlow*cos(model.pr.alfa*d2r)*cos(model.pr.beta*d2r);
    //prj.y = 0.95*trFlow*sin(model.pr.alfa*d2r);
    //prj.z = 0.95*trFlow*sin(model.pr.beta*d2r);
    //vector_(1, Blue,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
}
void GLWidget::pr_axle()
{
    float ml=0.1*m;///0.15
    float sl=0.1*m;///0.15
    //основные линии
    glLineWidth(2);
    glBegin( GL_LINES );
        qglColor(Red);          glVertex3f(0,0,0);   glVertex3f(0.5*ml, 0, 0);
        qglColor(Green);        glVertex3f(0,0,0);   glVertex3f(0, 0.5*ml, 0);
        qglColor(Blue);         glVertex3f(0,0,0);   glVertex3f(0, 0, 0.4*ml);
    glEnd();
    //добавленные
    glLineWidth(2);
    glBegin( GL_LINES );
        qglColor(halfRed);      glVertex3f(0,0,0);   glVertex3f(-0.5*sl, 0, 0);
        qglColor(halfGreen);    glVertex3f(0,0,0);   glVertex3f(0, -0.5*sl, 0);
        qglColor(halfBlue);     glVertex3f(0,0,0);   glVertex3f(0, 0, -0.4*sl);
    glEnd();
}
void GLWidget::pr_ring()
{//кольцо - кромка концов лопастей
    qglColor(Gray);
    glPushMatrix();
        glRotatef( 90.0,  1.0f, 0.0f, 0.0f);
        glTranslatef(0, 0, -0.005);//
        cylindr(0.0005*model.pr.D-0.003, 0.0005*model.pr.D, 0.005);
    glPopMatrix();
    glPushMatrix();
        glRotatef( 90.0,  1.0f, 0.0f, 0.0f);
        glTranslatef(0, 0, 0);//
        cylindr(0.0005*model.pr.D, 0.0005*model.pr.D-0.003, 0.005);
    glPopMatrix();
}
void GLWidget::pr_blades(int k)
{
    xyz beg,prj;
    double Lb = 0.5*model.pr.D - model.pr.l_vsh;
    //вертикальный шарнир
    glPointSize(5);
    qglColor(Blue);
    glBegin(GL_POINTS);
        glVertex3f(0.001*model.pr.l_vsh, 0, 0);
    glEnd();

   //втулка
   glLineWidth(1);
   qglColor(Blue);
   glBegin(GL_LINES);
       glVertex3f(0, 0, 0);
       glVertex3f(0.001*model.pr.l_vsh, 0, 0);
   glEnd();

   //нейтраль
   glLineWidth(1);
   qglColor(Gray);
   glBegin(GL_LINES);
       glVertex3f(0.001*model.pr.l_vsh, 0, 0);
       glVertex3f(0.001*0.5*model.pr.D, 0, 0);
   glEnd();

   if(model.pr.solve)///3D-blade
   {
        if(visFpr)
        {
            if(visF)//вектора в шарнирах
            {
            glLineWidth(1);
            glPushMatrix();
                glTranslatef(0.001*model.pr.l_vsh, 0, 0);
                vector_(2, Gray, 0,0,0,  0, 0, -mv*model.pr.b[k].Fx);///x
                vector_(2, Gray, 0,0,0,  0, mv*model.pr.b[k].Fy, -0);///y
            glPopMatrix();
            }

            //вектора на лопастях
            glLineWidth(1);
            glPushMatrix();
                glTranslatef(0.001*model.pr.l_vsh, 0, 0);
                glRotatef( model.pr.b[k].ksi, 0.0f, 1.0f, 0.0f);
                glRotatef( model.pr.b[k].beta, 0.0f, 0.0f, 1.0f);
                //погонные аэродин силы
                for (int j=0; j<prbldneut->size()-2; j++)//
                {
                    neut0 = prbldneut->at(j+1);
                    ///U - скорости притекания
                    beg.x = neut0.x ;
                    beg.y = neut0.y - 1*mv*model.pr.b[k].Uy.at(j);
                    beg.z = neut0.z + 1*mv*model.pr.b[k].Ux.at(j);
                    prj.x = 0;
                    prj.y = 1*mv*model.pr.b[k].Uy.at(j);
                    prj.z = 1*mv*model.pr.b[k].Ux.at(j);
                    vector_(1, lightBlue,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                    ///dT
                    beg.x = neut0.x;
                    beg.y = neut0.y;
                    beg.z = neut0.z;
                    prj.x = 0;
                    prj.y = mv*model.pr.b[k].dT_dr.at(j);
                    prj.z = 0;
                    vector_(1, Green,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                    ///dQ
                    beg.x = neut0.x;
                    beg.y = neut0.y;
                    beg.z = neut0.z;
                    prj.x = 0;
                    prj.y = 0;//0.0005
                    prj.z = mv*model.pr.b[k].dQ_dr.at(j);
                    vector_(1, Brown,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                }
                //
                {
                    neut0 = prbldneut->at(prbldneut->size()-1);
                    int j = model.pr.nSec-1;
                    ///U - скорости притекания
                    beg.x = neut0.x ;
                    beg.y = neut0.y - 1*mv*model.pr.b[k].Uy.at(j);
                    beg.z = neut0.z + 1*mv*model.pr.b[k].Ux.at(j);
                    prj.x = 0;
                    prj.y = 1*mv*model.pr.b[k].Uy.at(j);
                    prj.z = 1*mv*model.pr.b[k].Ux.at(j);
                    vector_(1, lightBlue,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                    ///dT
                    beg.x = neut0.x;
                    beg.y = neut0.y;
                    beg.z = neut0.z;
                    prj.x = 0;
                    prj.y = mv*model.pr.b[k].dT_dr.at(j);
                    prj.z = 0;
                    vector_(1, Green,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                    ///dQ
                    beg.x = neut0.x;
                    beg.y = neut0.y;
                    beg.z = neut0.z;
                    prj.x = 0;
                    prj.y = 0;//0.0005
                    prj.z = mv*model.pr.b[k].dQ_dr.at(j);
                    vector_(1, Brown,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                }
                //инерционные силы
                {
                neut0 = prbldneut->at(model.pr.sec07);
                beg.x = neut0.x;
                beg.y = neut0.y;
                beg.z = neut0.z;
                //инерция от ускорений вала НВ
                prj.x = 0;
                prj.y = mv*model.pr.b[k].Payy;//в плоскости тяги
                prj.z = 0;
                vector_(1, Black,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                prj.x = 0;
                prj.y = 0;
                prj.z = mv*model.pr.b[k].Paxz;//в плоскости винта
                vector_(1, Black,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                //P1 инерция взмаха
                prj.x = 0;
                prj.y = mv*model.pr.b[k].P1;
                prj.z = 0;
                vector_(1, Red,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                //P2 инерция качания
                prj.x = 0;
                prj.y = 0;
                prj.z = mv*model.pr.b[k].P2;
                vector_(1, Orange,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                //K1 сила Кориолиса от взмаха
                prj.x = 0;
                prj.y = 0;
                prj.z = mv*model.pr.b[k].K1;
                vector_(1, Brown,  beg.x, beg.y, beg.z,  prj.x, prj.y, -prj.z);
                }
            glPopMatrix();
        }

        //контур лопасти
        glLineWidth(1);
        glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
        glPushMatrix();
           glTranslatef(0.001*model.pr.l_vsh, 0, 0);
           glRotatef( model.pr.b[k].ksi, 0.0f, 1.0f, 0.0f);
           glRotatef( model.pr.b[k].beta, 0.0f, 0.0f, 1.0f);
           glRotatef(-(model.pr.b[k].fi7 + model.pr.b[k].gama), 1.0f, 0.0f, 0.0f);

           qglColor(Gray);
           glBegin(GL_LINES);
           for (int j=0; j<prbldneut->size()-1; j++)//
           {
               frnt0 = prbldfrnt->at(j);
               neut0 = prbldneut->at(j);
               rear0 = prbldrear->at(j);
               frnt1 = prbldfrnt->at(j+1);
               neut1 = prbldneut->at(j+1);
               rear1 = prbldrear->at(j+1);

               glVertex3f(neut0.x,neut0.y,neut0.z);
               glVertex3f(frnt0.x,frnt0.y,frnt0.z);
               glVertex3f(frnt0.x,frnt0.y,frnt0.z);
               glVertex3f(frnt1.x,frnt1.y,frnt1.z);
               glVertex3f(frnt1.x,frnt1.y,frnt1.z);
               glVertex3f(neut1.x,neut1.y,neut1.z);
               glVertex3f(neut1.x,neut1.y,neut1.z);
               glVertex3f(neut0.x,neut0.y,neut0.z);

               glVertex3f(neut0.x,neut0.y,neut0.z);
               glVertex3f(rear0.x,rear0.y,rear0.z);
               glVertex3f(rear0.x,rear0.y,rear0.z);
               glVertex3f(rear1.x,rear1.y,rear1.z);
               glVertex3f(rear1.x,rear1.y,rear1.z);
               glVertex3f(neut1.x,neut1.y,neut1.z);
               glVertex3f(neut1.x,neut1.y,neut1.z);
               glVertex3f(neut0.x,neut0.y,neut0.z);

           }
           glEnd();

           qglColor(bladecolor);
           glBegin(GL_QUADS);
           for (int j=0; j<prbldneut->size()-1; j++)//
           {
               frnt0 = prbldfrnt->at(j);
               neut0 = prbldneut->at(j);
               rear0 = prbldrear->at(j);
               frnt1 = prbldfrnt->at(j+1);
               neut1 = prbldneut->at(j+1);
               rear1 = prbldrear->at(j+1);

               glVertex3f(neut0.x,neut0.y,neut0.z);
               glVertex3f(frnt0.x,frnt0.y,frnt0.z);
               glVertex3f(frnt1.x,frnt1.y,frnt1.z);
               glVertex3f(neut1.x,neut1.y,neut1.z);
               glVertex3f(neut0.x,neut0.y,neut0.z);
               glVertex3f(rear0.x,rear0.y,rear0.z);
               glVertex3f(rear1.x,rear1.y,rear1.z);
               glVertex3f(neut1.x,neut1.y,neut1.z);
           }
           glEnd();

           glPointSize(3);
           qglColor(Blue);
           glBegin(GL_POINTS);
           switch (k)
           {
           case 0://1 лопасть
                   glVertex3f(0.001*(Lb+50), 0, 0);
           break;
           case 1://2 лопасти
                   glVertex3f(0.001*(Lb+50), 0,-0.025);
                   glVertex3f(0.001*(Lb+50), 0, 0.025);
           break;
           case 2://3 лопасти
                   glVertex3f(0.001*(Lb+50), 0,-0.050);
                   glVertex3f(0.001*(Lb+50), 0, 0);
                   glVertex3f(0.001*(Lb+50), 0, 0.050);
           break;
           case 3://4 лопасти
                   glVertex3f(0.001*(Lb+50), 0,-0.075);
                   glVertex3f(0.001*(Lb+50), 0,-0.025);
                   glVertex3f(0.001*(Lb+50), 0, 0.025);
                   glVertex3f(0.001*(Lb+50), 0, 0.075);
           break;
           case 4://5 лопасти
                   glVertex3f(0.001*(Lb+50), 0,-0.100);
                   glVertex3f(0.001*(Lb+50), 0,-0.050);
                   glVertex3f(0.001*(Lb+50), 0, 0);
                   glVertex3f(0.001*(Lb+50), 0, 0.050);
                   glVertex3f(0.001*(Lb+50), 0, 0.100);
           break;
           case 5://6 лопастей
                   glVertex3f(0.001*(Lb+50), 0,-0.125);
                   glVertex3f(0.001*(Lb+50), 0,-0.075);
                   glVertex3f(0.001*(Lb+50), 0,-0.025);
                   glVertex3f(0.001*(Lb+50), 0, 0.025);
                   glVertex3f(0.001*(Lb+50), 0, 0.075);
                   glVertex3f(0.001*(Lb+50), 0, 0.125);
           break;
           }
           glEnd();
       glPopMatrix();
   }
   else///1D-blade
   {
       glLineWidth(3);
       qglColor(Green);
       glBegin(GL_LINES);
           glVertex3f(0.001*model.pr.l_vsh, 0, 0);
           glVertex3f(-0.001*Lb*cos(model.pr.b[k].ksi*M_PI/180.0), 0, -0.001*Lb*sin(model.pr.b[k].ksi*M_PI/180.0));
       glEnd();
   }
}

void GLWidget::fmvectors()
{
    if(visM)
    {
        vector(Brown,    0.001*model.fus.xct, 0.001*model.fus.yct, 0.001*model.fus.zct,   model.fus.Msum.x*mv, model.fus.Msum.y*mv, model.fus.Msum.z*mv);
    }
    if(visF)
    {
        vector(halfRed,  0.001*model.fus.xct,     0.001*model.fus.yct,     0.001*model.fus.zct,      model.fus.Fsum.x*mv, model.fus.Fsum.y*mv, model.fus.Fsum.z*mv);
        //vector(Red,    0.001*model.fus.xct,     0.001*model.fus.yct,     0.001*model.fus.zct,      model.fus.n.x*0.5,   model.fus.n.y*0.5,   model.fus.n.z*0.5);
        vector_(2,Black, 0.001*model.fus.F1pos.x, 0.001*model.fus.F1pos.y, 0.001*model.fus.F1pos.z,  model.fus.F1.x*mv,   model.fus.F1.y*mv,   model.fus.F1.z*mv);
        vector_(2,Black, 0.001*model.fus.F2pos.x, 0.001*model.fus.F2pos.y, 0.001*model.fus.F2pos.z,  model.fus.F2.x*mv,   model.fus.F2.y*mv,   model.fus.F2.z*mv);
        //vector(Red,    0.001*model.fus.selpoint.x, 0.001*model.fus.selpoint.y, 0.001*model.fus.selpoint.z, model.fus.n.x*0.5,   model.fus.n.y*0.5,   model.fus.n.z*0.5);
        vector_(2,Black, 0.001*model.mr.O.x, 0.001*model.mr.O.y, 0.001*model.mr.O.z,  mv*model.fus.Fmr.x,  mv*model.fus.Fmr.y, mv*model.fus.Fmr.z);
        vector_(2,Black, 0.001*model.tr.O.x, 0.001*model.tr.O.y, 0.001*model.tr.O.z,  mv*model.fus.Ftr.x,  mv*model.fus.Ftr.y, mv*model.fus.Ftr.z);
        vector_(2,Black, 0.001*model.pr.O.x, 0.001*model.pr.O.y, 0.001*model.pr.O.z,  mv*model.fus.Fpr.x,  mv*model.fus.Fpr.y, mv*model.fus.Fpr.z);
    }

    if(visFhg)
    {
        vector_(2,Gray,  0.001*model.fus.xct, 0.001*model.fus.yct, 0.001*model.fus.zct,  model.fus.Fhg.x*mv, model.fus.Fhg.y*mv, model.fus.Fhg.z*mv);
        if(model.cargo.m>0)//cargo
        {
            //точка крепления троса
            glPointSize(5);
                glBegin(GL_POINTS);
                    glVertex3f(0.001*model.cargo.Ox,  0.001*model.cargo.Oy,  0.001*model.cargo.Oz);
                glEnd();
            //трос
            glLineWidth(1);
            qglColor(Gray);
            glBegin(GL_LINES);
            glVertex3f( 0.001*model.cargo.Ox,  0.001*model.cargo.Oy,  0.001*model.cargo.Oz  );
            glVertex3f( 0.001*model.cargo.CTx, 0.001*model.cargo.CTy, 0.001*model.cargo.CTz );
            glEnd();
            //вектор веса груза
            vector_(3, Gray,  0.001*model.cargo.CTx, 0.001*model.cargo.CTy, 0.001*model.cargo.CTz, model.cargo.Gx*0.00005, model.cargo.Gy*0.00005, model.cargo.Gz*0.00005);
        }
    }

    if(visFad)
    {
        vector_(2, halfBlue,  0.001*model.fus.fus.x,  0.001*model.fus.fus.y,  0.001*model.fus.fus.z,   mv*model.fus.fus_F.x,  mv*model.fus.fus_F.y,  mv*model.fus.fus_F.z);
        vector_(2, halfBlue,  0.001*model.fus.stab.x, 0.001*model.fus.stab.y, 0.001*model.fus.stab.z,  mv*model.fus.stab_F.x, mv*model.fus.stab_F.y, mv*model.fus.stab_F.z);
        vector_(2, halfBlue,  0.001*model.fus.keel.x, 0.001*model.fus.keel.y, 0.001*model.fus.keel.z,  mv*model.fus.keel_F.x, mv*model.fus.keel_F.y, mv*model.fus.keel_F.z);
    }

    if(visFwh)
    {
        if(model.nst)//isLanding
        {
            for (int k=0; k<model.nst; k++)//
            {
                if(model.st[k].nwh==2)
                {
                    vector_(2,Gray, 0.001*model.st[k].C1.x, 0.001*model.st[k].C1.y, 0.001*model.st[k].C1.z,  mv*model.st[k].F1.x, mv*model.st[k].F1.y, mv*model.st[k].F1.z);
                    vector_(2,Gray, 0.001*model.st[k].C2.x, 0.001*model.st[k].C2.y, 0.001*model.st[k].C2.z,  mv*model.st[k].F2.x, mv*model.st[k].F2.y, mv*model.st[k].F2.z);
                }
                else
                {
                    vector_(2,Gray, 0.001*model.st[k].C.x, 0.001*model.st[k].C.y, 0.001*model.st[k].C.z,  mv*model.st[k].F.x, mv*model.st[k].F.y, mv*model.st[k].F.z);
                }
            }
        }
    }
}

void GLWidget::vector(QColor F_M, float Ox, float Oy, float Oz, float Vx, float Vy, float Vz)
{
    //центр
    glPointSize(6);
    glBegin(GL_POINTS);
        qglColor(Black);
        glVertex3f( Ox,    Oy,    Oz );
    glEnd();
    //axe
    glLineWidth(1);
    glBegin( GL_LINES );
        qglColor(F_M);
        glVertex3f( Ox,    Oy,    Oz );
        glVertex3f( Ox+Vx, Oy+Vy, Oz+Vz );
    glEnd();
    glLineWidth(2);
    glBegin( GL_LINES );
        qglColor(F_M);
        glVertex3f( Ox,    Oy,    Oz );
        glVertex3f( Ox+0.8*Vx, Oy+0.8*Vy, Oz+0.8*Vz );
    glEnd();
    glLineWidth(3);
    glBegin( GL_LINES );
        qglColor(F_M);
        glVertex3f( Ox,    Oy,    Oz );
        glVertex3f( Ox+0.6*Vx, Oy+0.6*Vy, Oz+0.6*Vz );
    glEnd();
    glLineWidth(4);
    glBegin( GL_LINES );
        qglColor(F_M);
        glVertex3f( Ox,    Oy,    Oz );
        glVertex3f( Ox+0.4*Vx, Oy+0.4*Vy, Oz+0.4*Vz );
    glEnd();
    glLineWidth(5);
    glBegin( GL_LINES );
        qglColor(F_M);
        glVertex3f( Ox,    Oy,    Oz );
        glVertex3f( Ox+0.2*Vx, Oy+0.2*Vy, Oz+0.2*Vz );
    glEnd();


    /* //pointer
    glPushMatrix();
        glTranslatef(0, 0, -4);// 20 - расст. между B и плоскостью колеса
        Cylindr(s[k].rwheel-2, s[k].rwheel,    4, ColorCyl);
    glPopMatrix();
    glPushMatrix();
        glTranslatef(0, 0, 0);// 20 - расст. между B и плоскостью колеса
        Cylindr(s[k].rwheel, s[k].rwheel-2,    4, ColorCyl);
    glPopMatrix();
    */
}

void GLWidget::vector_(float th, QColor F_M, float Ox, float Oy, float Oz, float Vx, float Vy, float Vz)
{
    float kp = 0.4;
    if ( fabs(kp*Vx/0.006)>40 )  kp=40/fabs(Vx/0.006);
    if ( fabs(kp*Vy/0.006)>40 )  kp=40/fabs(Vy/0.006);
    if ( fabs(kp*Vz/0.006)>40 )  kp=40/fabs(Vz/0.006);
    float Cx = Ox+(1-kp)*Vx;
    float Cy = Oy+(1-kp)*Vy;
    float Cz = Oz+(1-kp)*Vz;

    glLineWidth(th);
    glBegin( GL_LINES );
        qglColor(F_M);
        glVertex3f( Ox,    Oy,    Oz );
        glVertex3f( Ox+Vx, Oy+Vy, Oz+Vz );
    glEnd();
    glLineWidth(th+1);
    glBegin( GL_LINES );
        qglColor(F_M);
        glVertex3f( Cx, Cy, Cz );
        glVertex3f( Cx+0.8*(kp)*Vx,  Cy+0.8*(kp)*Vy,  Cz+0.8*(kp)*Vz );
    glEnd();
    glLineWidth(th+2);
    glBegin( GL_LINES );
        qglColor(F_M);
        glVertex3f( Cx, Cy, Cz );
        glVertex3f( Cx+0.6*(kp)*Vx,  Cy+0.6*(kp)*Vy,  Cz+0.6*(kp)*Vz );
    glEnd();
    glLineWidth(th+3);
    glBegin( GL_LINES );
        qglColor(F_M);
        glVertex3f( Cx, Cy, Cz );
        glVertex3f( Cx+0.4*(kp)*Vx,  Cy+0.4*(kp)*Vy,  Cz+0.4*(kp)*Vz );
    glEnd();
    glLineWidth(th+4);
    glBegin( GL_LINES );
        qglColor(F_M);
        glVertex3f( Cx, Cy, Cz );
        glVertex3f( Cx+0.2*(kp)*Vx,  Cy+0.2*(kp)*Vy,  Cz+0.2*(kp)*Vz );
    glEnd();
}

void GLWidget::cylindr(float bRadius, float tRadius, float Height)
{
     //glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); //GL_LINE
     glBegin(GL_QUAD_STRIP);
       gluCylinder(gluNewQuadric(), bRadius,tRadius,Height, 200,100);
     glEnd();
}

static void qNormalizeAngle(float &angle)
{
    while (angle < -180)
        angle += 360;
    while (angle > 180)
        angle -= 360;
}

float GLWidget::LevelFromValue(float value, float max, float min)
{
    float level=0;
    if(max-min!=0) level = (value-min)/(max-min);
    if(level<0)    level = 0;
    if(level>max)  level = max;
return level;
}
QColor GLWidget::ColorFromLevel(float level)
{
    float R=0, G=0, B=0;
    if( level<=0 )//Blue
    {
        R = 0;
        G = 0;
        B = 255;
    }
    if((level>0  )&&(level<=0.25))//Blue->Green
    {
        R = 0;
        G = level*255/0.25;
        B = 255;
    }
    if((level>0.25)&&(level<=0.5))//Blue->Green
    {
        R = 0;
        G = 255;
        B = 255*(2.0-level/0.25);
    }
    if((level>0.5)&&(level<=0.75))//Green->Red
    {
        R = (level-0.5)*255/0.25;
        G = 255;
        B = 0;
    }
    if((level>0.75)&&(level<=1.0))//Green->Red
    {
        R = 255;
        G = 255*(1.0-level)/0.25;
        B = 0;
    }
    if((level>1.0))//Red
    {
        R = 255;
        G = 0;
        B = 0;
    }
    QColor col( R, G, B, 255 );
return col;
}

void GLWidget::setRotateCenter(float xpos,float ypos,float zpos)
{
        RotCenterX = xpos;
        RotCenterY = ypos;
        RotCenterZ = zpos;
        updateGL();
}
void GLWidget::mousePressEvent(QMouseEvent *mevnt)
{
    mousePos = mevnt->pos(); // запоминаем координату нажатия мыши
}
void GLWidget::mouseMoveEvent(QMouseEvent *mevnt)
{
    float dx = mevnt->x() - mousePos.x();
    float dy = mevnt->y() - mousePos.y();
    if (mevnt->buttons() & Qt::LeftButton)//Position
    {
        xPos += (0.0000078-0.000186*zPos)*dx;//xPos += (-0.000373*zPos)*dx;
        yPos -= (0.0000078-0.000186*zPos)*dy;//yPos -= (-0.000373*zPos)*dy;
    }
    else if (mevnt->buttons() & Qt::RightButton)//Rotation
    {
        qNormalizeAngle( xRot += 0.1*dy );
        qNormalizeAngle( yRot += 0.1*dx );
    }
    else if (mevnt->buttons() & Qt::MiddleButton)//Rotation
    {
        qNormalizeAngle( xRot += 0.1*dy );
        qNormalizeAngle( yRot += 0.1*dx );
        //qNormalizeAngle( zRot -= 0.1*dy );
    }
    mousePos = mevnt->pos();
    updateGL();
}
void GLWidget::wheelEvent(QWheelEvent* mevnt)
{
    zPos += (-0.001*zPos)*mevnt->delta();
    if(zPos>-0.1)      zPos=-0.1;
    if(zPos<-300*hrzn) zPos=-300*hrzn;
    updateGL();
}

/*
void GLWidget::vector_build(Cell dot)
{//-----------------------------------------------------------------------------
    QColor vcolor = ColorFromLevel( LevelFromValue(dot.size, surfmax, surfmin) );
    vcolor.setAlphaF(1.0);
    glColor4f( vcolor.redF(), vcolor.greenF(), vcolor.blueF(), vcolor.alphaF() );
    //glColor4f( 0.5, 0.5, 0.5, 0.8 );
    //хвост
    //glLineWidth(1*width);
        glVertex3f( (dot.x)*m,  (dot.y)*m,  (dot.z)*m );
        glVertex3f( (dot.x+dot.vx*ms)*m, (dot.y+dot.vy*ms)*m, (dot.z+dot.vz*ms)*m );
        //glVertex3f( (dot.x+dot.vx*ms/3.0)*m, (dot.y+dot.vy*ms/3.0)*m, (dot.z+dot.vz*ms/3.0)*m );

}//-----------------------------------------------------------------------------



void GLWidget::show_cloud()
{//-----------------------------------------------------------------------------
    Cell dot;
    vector < Cell > line;
    glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);

    float wdt = 1.0 - 250/zPos;

    //вектора
    glLineWidth(0.5*wdt);
    glBegin(GL_LINES);      // для линий
    for(int l=1; l<surface.size()-1; l++)//пройти по линиям поверхности
    {
        line.clear();
        line = surface.at(l);
        for(int i=1; i<line.size()-1; i++)//пройти по полоске
        {
            dot = line.at(i);
            if( (i>xmin)&&(i<xmax) && (l>zmin)&&(l<zmax) )
                vector_build(dot);
        }
    }
    glEnd();

    glPointSize( 1.3*wdt );
    glBegin(GL_POINTS);//GL_LINE_STRIP
    for(int l=1; l<surface.size()-1; l++)//пройти по линиям поверхности
    {
        line.clear();
        line = surface.at(l);
        for(int i=1; i<line.size()-1; i++)//пройти по полоске
        {
            dot = line.at(i);
            QColor vcolor = ColorFromLevel( LevelFromValue(dot.size, surfmax, surfmin) );
            vcolor.setAlphaF(1.0);
            glColor4f( vcolor.redF(), vcolor.greenF(), vcolor.blueF(), vcolor.alphaF() );
            if( (i>xmin)&&(i<xmax) && (l>zmin)&&(l<zmax) )
                glVertex3f( dot.x*m, dot.y*m, dot.z*m );
        }
    }
    glEnd();

}//-----------------------------------------------------------------------------





void GLWidget::surface_fill()
{//-----------------------------------------------------------------------------
    Cell dot[4];
    vector < Cell > line[2];

    glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
    glBegin(GL_QUADS);// GL_LINE_STRIP

    for(int l=1; l<surface.size()-2; l++)//пройти по линиям поверхности
    {
        line[0].clear();
        line[1].clear();
        // получаем полосу выделяя две линии из поверхности
        line[0] = surface.at(l);
        line[1] = surface.at(l+1);

        for(int i=1; i<line[0].size()-2; i++)//пройти по полоске
        {
            //выделить элемент
            dot[0] = line[0].at(i);
            dot[1] = line[0].at(i+1);
            dot[2] = line[1].at(i+1);
            dot[3] = line[1].at(i);
            //прорисовать элемент
            if( (i>xmin)&&(i<xmax) && (l>zmin)&&(l<zmax) )
            quad_fill( dot[0],dot[1],dot[2],dot[3]);
        }
    }
    glEnd();
}//-----------------------------------------------------------------------------



void GLWidget::quad_fill(Cell dot1, Cell dot2, Cell dot3, Cell dot4)
{//-----------------------------------------------------------------------------
    QColor dot1color,dot2color,dot3color,dot4color;
    //quadcolor = ColorFromLevel(quadlevel);
    dot1color = ColorFromLevel( LevelFromValue(dot1.size, surfmax, surfmin) );    dot1color.setAlphaF(1.0);
    dot2color = ColorFromLevel( LevelFromValue(dot2.size, surfmax, surfmin) );    dot2color.setAlphaF(1.0);
    dot3color = ColorFromLevel( LevelFromValue(dot3.size, surfmax, surfmin) );    dot3color.setAlphaF(1.0);
    dot4color = ColorFromLevel( LevelFromValue(dot4.size, surfmax, surfmin) );    dot4color.setAlphaF(1.0);

//заполнение
    glColor4f( dot1color.redF(), dot1color.greenF(), dot1color.blueF(), dot1color.alphaF() );
        glVertex3f( dot1.x*m, (dot1.y+dot1.size*ms)*m, dot1.z*m );  //glVertex3f(dot2.x*m,dot2.y*m,dot2.z*m);
    glColor4f( dot2color.redF(), dot2color.greenF(), dot2color.blueF(), dot2color.alphaF() );
        glVertex3f( dot2.x*m, (dot2.y+dot2.size*ms)*m, dot2.z*m );  //glVertex3f(dot3.x*m,dot3.y*m,dot3.z*m);
    glColor4f( dot3color.redF(), dot3color.greenF(), dot3color.blueF(), dot3color.alphaF() );
        glVertex3f( dot3.x*m, (dot3.y+dot3.size*ms)*m, dot3.z*m );  //glVertex3f(dot4.x*m,dot4.y*m,dot4.z*m);
    glColor4f( dot4color.redF(), dot4color.greenF(), dot4color.blueF(), dot4color.alphaF() );
        glVertex3f( dot4.x*m, (dot4.y+dot4.size*ms)*m, dot4.z*m );  //glVertex3f(dot1.x*m,dot1.y*m,dot1.z*m);
}//-----------------------------------------------------------------------------



void GLWidget::surface_line()
{//-----------------------------------------------------------------------------
    Cell dot[4];
    vector < Cell > line[2];
    float dot0level = 0, dot1level = 0, dot2level = 0, dot3level = 0;
    float quadlevel = 0;
    QColor quadcolor;

    qglColor( Black );
    glLineWidth(0.01);//2*m/(-0.05*zPos*zPos));
    glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
    glBegin(GL_QUADS);//GL_LINE_STRIP

    for(int l=1; l<surface.size()-2; l++)//пройти по линиям поверхности
    {
        line[0].clear();
        line[1].clear();
        // получаем полосу выделяя две линии из поверхности
        line[0] = surface.at(l);
        line[1] = surface.at(l+1);

        for(int i=1; i<line[0].size()-2; i++)//пройти по полоске
        {
            //выделить элемент
            dot[0] = line[0].at(i);
            dot[1] = line[0].at(i+1);
            dot[2] = line[1].at(i+1);
            dot[3] = line[1].at(i);
            //вычислить уровень вершин
            //прорисовать элемент
            if( (i>xmin)&&(i<xmax) && (l>zmin)&&(l<zmax) )
            quad_line( dot[0],dot[1],dot[2],dot[3]);
        }
    }
    glEnd();
}//-----------------------------------------------------------------------------



void GLWidget::quad_line(Cell dot1, Cell dot2, Cell dot3, Cell dot4)
{//-----------------------------------------------------------------------------
        glVertex3f( dot1.x*m, (dot1.y+dot1.size*ms)*m, dot1.z*m );  //glVertex3f(dot2.x*m,dot2.y*m,dot2.z*m);
        glVertex3f( dot2.x*m, (dot2.y+dot2.size*ms)*m, dot2.z*m );  //glVertex3f(dot3.x*m,dot3.y*m,dot3.z*m);
        glVertex3f( dot3.x*m, (dot3.y+dot3.size*ms)*m, dot3.z*m );  //glVertex3f(dot4.x*m,dot4.y*m,dot4.z*m);
        glVertex3f( dot4.x*m, (dot4.y+dot4.size*ms)*m, dot4.z*m );  //glVertex3f(dot1.x*m,dot1.y*m,dot1.z*m);
}//-----------------------------------------------------------------------------



void GLWidget::geo_wirebox(Cell dot1, Cell dot2)// прозрачный ящик
{//-----------------------------------------------------------------------------
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glColor4f(0.0,0.0,0.0,1.0);
    glLineWidth(1);
    glBegin(GL_QUADS);// GL_LINE_STRIP
        glVertex3f( dot1.x*m, dot1.y*m, dot1.z*m );
        glVertex3f( dot2.x*m, dot1.y*m, dot1.z*m );
        glVertex3f( dot2.x*m, dot2.y*m, dot1.z*m );
        glVertex3f( dot1.x*m, dot2.y*m, dot1.z*m );
        glVertex3f( dot1.x*m, dot1.y*m, dot2.z*m );
        glVertex3f( dot2.x*m, dot1.y*m, dot2.z*m );
        glVertex3f( dot2.x*m, dot2.y*m, dot2.z*m );
        glVertex3f( dot1.x*m, dot2.y*m, dot2.z*m );
        glVertex3f( dot1.x*m, dot1.y*m, dot1.z*m );
        glVertex3f( dot1.x*m, dot2.y*m, dot1.z*m );
        glVertex3f( dot1.x*m, dot2.y*m, dot2.z*m );
        glVertex3f( dot1.x*m, dot1.y*m, dot2.z*m );
        glVertex3f( dot2.x*m, dot1.y*m, dot1.z*m );
        glVertex3f( dot2.x*m, dot2.y*m, dot1.z*m );
        glVertex3f( dot2.x*m, dot2.y*m, dot2.z*m );
        glVertex3f( dot2.x*m, dot1.y*m, dot2.z*m );
    glEnd();
}//-----------------------------------------------------------------------------



void GLWidget::Point_(float x, float z, float y, float S, QColor clr)
{//-----------------------------------------------------------------------------
    qglColor( clr );
        //GLUquadricObj* gluNewQuadric(void);
     //glDisable(GL_LIGHTING);
     glPointSize(S);
     glBegin(GL_POINTS);// );

      glVertex3f(x,y,z);
     glEnd();
     //glEnable(GL_LIGHTING);
}//-----------------------------------------------------------------------------


void GLWidget::Line(float pX[20],float pY[20],float pZ[20], QColor clr,float Width)
{//-----------------------------------------------------------------------------
    glDisable(GL_LIGHTING);
     glLineWidth(Width);
    qglColor( clr );
       glBegin(GL_LINE_STRIP);
         for(i=1;i<20;i++) glVertex3f(pX[i],pY[i],pZ[i]);
       glEnd();
     glEnable(GL_LIGHTING);
}//-----------------------------------------------------------------------------

void GLWidget::Vector(float x, float y, float z, float X, float Y, float Z, float S, float W, float Color[4])
{//Dot_
     //glDisable(GL_LIGHTING);
     glPointSize(S);
     glColor4f(0.0,0.0,0.0,1.0);
     glBegin(GL_POINTS);
       glVertex3f(x,y,z*mpelena);
     glEnd();
     //glEnable(GL_LIGHTING);
 //Line_
     //glDisable(GL_LIGHTING);
     glLineWidth(W);
     glColor4f(Color[0],Color[1],Color[2],Color[3]);
     glBegin(GL_LINES);
       glVertex3f( x,  y,  z*mpelena );
       glVertex3f(x+X,y+Y,z*mpelena+Z*scale);//+
     glEnd();
    // glEnable(GL_LIGHTING);
}//---------------------------------------------------------------------------

void GLWidget::Cylindr(float bRadius, float tRadius, float Height, QColor clr)
{//-----------------------------------------------------------------------------
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); //GL_LINE
    qglColor( clr );
     glBegin(GL_QUAD_STRIP);
       gluCylinder(gluNewQuadric(), bRadius,tRadius,Height, 50,20);
     glEnd();
}//-----------------------------------------------------------------------------

*/
