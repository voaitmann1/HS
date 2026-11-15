QT += core
QT -= gui

CONFIG += c++11
TARGET = helisolve
CONFIG += console
CONFIG -= app_bundle
TEMPLATE = app

SOURCES +=\
        absorber.cpp \
        pneumatic.cpp \
        shassi.cpp \
        mrotor.cpp \
        helicopter.cpp \
        solver_hc.cpp \
        airdyn_hc.cpp \
    main_.cpp \
    mainprocess_.cpp

HEADERS +=\
        absorber.h \
        pneumatic.h \
        shassi.h \
        mrotor.h \
        helicopter.h \
    main_.h

#gsl:
INCLUDEPATH += C:/Qt/gsl_2_1/include
LIBS += -static -LC:/Qt/gsl_2_1/lib -lgsl -lgslcblas -lm
