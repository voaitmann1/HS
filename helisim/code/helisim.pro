#-------------------------------------------------
# Project created by QtCreator 2015-03-27T15:47:24
#-------------------------------------------------

QT       += core gui \
        widgets \
        opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = helisim
TEMPLATE = app

SOURCES  += main.cpp\
        mainwindow.cpp \
        qcustomplot.cpp \
        openglwdg.cpp \
    absorber.cpp \
    pneumatic.cpp \
    shassi.cpp \
    helicopter.cpp \
    solver_hc.cpp \
    mainprocess.cpp \
    rotors.cpp \ 
    airframe.cpp \
    impulse.cpp \
    mymathlib.cpp \
    fluttermodels.cpp \
    mylib.cpp

HEADERS  += mainwindow.h \
        ui_mainwindow.h \
        qcustomplot.h \
        openglwdg.h \
    absorber.h \
    pneumatic.h \
    shassi.h \
    helicopter.h \
    types.h \
    rotors.h \  
    mymathlib.h \
    fluttermodels.h \
    mylib.h

FORMS    += mainwindow.ui

#static release:
QMAKE_LFLAGS_RELEASE += -static -static-libgcc -s

#gsl:
INCLUDEPATH += C:/Qt/gsl_2_1/include
LIBS += -static -LC:/Qt/gsl_2_1/lib -lgsl -lgslcblas -lm \
        -lopengl32 -lglu32

OTHER_FILES +=
