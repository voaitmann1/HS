#ifndef OPENGLWDG_H
#define OPENGLWDG_H
#include <QGLWidget>
#include <QKeyEvent>
#include "helicopter.h"



class GLWidget : public QGLWidget
{
    Q_OBJECT

public:
    GLWidget(QWidget *parent = 0);
    ~GLWidget();
    outstructure model;
    QVector<xyz> *frmcntr=0;
    QVector<xyz> *mrbldfrnt=0;
    QVector<xyz> *mrbldneut=0;
    QVector<xyz> *mrbldrear=0;
    QVector<xyz> *trbldfrnt=0;
    QVector<xyz> *trbldneut=0;
    QVector<xyz> *trbldrear=0;
    QVector<xyz> *prbldfrnt=0;
    QVector<xyz> *prbldneut=0;
    QVector<xyz> *prbldrear=0;
    xyz   frnt0, neut0, rear0;
    xyz   frnt1, neut1, rear1;
    xyz   node1, node2, node3;
    bool  visAx,visGround,visAxSKS,visAxRotors,SKSbinding,
          visMR,visTR,visPR,visPlaner,
          onMRcamera,onTRcamera,onPRcamera,oneBlade, fixLineMoving;
    bool  visF,visM, visFhg,visMhg, visFad,visMad, visFmr,visMmr, visFtr,visMtr, visFpr,visMpr, visFwh,visMwh;
    float ms, m, mv, x0sks, y0sks, z0sks;
    int   mode;
    int   xmin,xmax, ymin,ymax, zmin,zmax;
    QColor ColorFromLevel(float level);

public slots:
    //void setPositionRotation(float xpos, float ypos, float zpos);//, float xrot, float yrot, float zrot
    void setRotateCenter(float xpos,float ypos,float zpos);

signals:

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int w, int h);

    void mousePressEvent(QMouseEvent* mevnt);   // нажатие на клавишу мыши
    void mouseMoveEvent(QMouseEvent* mevnt);    // перемещение мыши
    void wheelEvent(QWheelEvent *mevnt);        // вращение колесика
    float LevelFromValue(float value, float max, float min);

    void earth(int front, int rear, int left, int right);
    void fuselage();
    void mainrotor();
    void mr_blades(int k);
    void mr_vtulka();
    void mr_ring();
    void mr_axle();
    void tailrotor();
    void tr_blades(int k);
    void tr_ring();
    void tr_axle();
    void pushrotor();
    void pr_blades(int k);
    void pr_ring();
    void pr_axle();
    void shassi(int k);
    void wheel(int k);
    void fmvectors();

    void axlesks();
    void axlearth(int front, int rear, int left, int right);
    void cylindr(float bRadius, float tRadius, float Height);
    void vector(QColor F_M, float Ox, float Oy, float Oz, float Vx, float Vy, float Vz);
    void vector_(float th, QColor F_M, float Ox, float Oy, float Oz, float Vx, float Vy, float Vz);

private:
    QPoint  mousePos;//переменная для запоминания позиции нажатия мышки
    QPoint  lastPos;
    float   xRot, yRot, zRot;
    float   xPos, yPos, zPos;
    float   RotCenterX,RotCenterY,RotCenterZ;
};
#endif
