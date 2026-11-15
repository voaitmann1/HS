#ifndef pneumaticH
#define pneumaticH
#include "types.h"

class Pneumatic
{
public:
        void    setNull();
        void    Preprocess();
        void    Solve(int nwh);
        //bool    SolveTest(int nwh);
        void    get_pncompress(int nwh, xyz kh);

        int     warning;
        //bool    trouble;
        int     status,status_,status1,status2; // -2:отрыв  -1:разжимается  0:в_покое  1:сжимается  2:обжат  3:разрушается
        bool    isbrake;
        xyz     v, h,h1,h2,h_,h1_,h2_,Pdmp,Pdf,P,P1,P2  ,Pdf_,Pd_;
        double  dt, dpmd, dppo, cpx,cpy,cpz, kdp,alfanp, kpy,
                Pst, Ppmax, hpmax, ovrld, dA,
                Ptr, dhtr, dAtr, ang_sx,//параметры определяемые проскальзыванием пневматика
                dhx,dhy,dhz,
                Dp, Bp;//vp,
        double  Pf0, f0,//сила и коэф. трения покоя
                Pfs, fs,//сила и коэф. трения скольжения
                Pfk, fk;//сила и коэф. трения качения
private:
        int     PneumaForce(xyz &h, xyz &P);
        void    FrictionalCheck(xyz &h, xyz &P);
        void    AddDamping();

};

#endif
