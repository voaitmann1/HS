//HELICOPTER.MAINROTOR.IMPULSEMODEL
#include "helicopter.h"
//#include "airframe.cpp"
#define mas  801
#define mas_ 301
#define np   8
//-----------------------------------------------------------------------------

Impulse::Impulse()
{
}
Impulse::~Impulse()
{
}

/*bool Impulse::DataRead()
{
        //
        //filewrite=ExtractFilePath(Application->ExeName)+"~tmp.txt";
        //Memo->Lines->SaveToFile(filewrite);
        int x[51],xx=19;
        char s[200]="";
        char ss[51];
        HNnom.clear();
        VNnom.clear();
        Nn.clear();
        HNvzlet.clear();
        VNvzlet.clear();
        Nv.clear();
        HNk.clear();
        VNk.clear();
        Nk.clear();
        V_Ksi.clear();
        Ksi.clear();
        V_Ksi.reserve(10);
        Ksi.reserve(10);
        HNnom.reserve(10);
        VNnom.reserve(10);
        HNvzlet.reserve(10);
        VNvzlet.reserve(10);
        HNk.reserve(10);
        VNk.reserve(10);
        char *sc[] = {"End_source_data"
              ,"D_nv="           //1
              ,"G="              //2
              ,"z="              //3
              ,"b_07="         //4
              ,"wR="             //5
              ,"ksi="            //6
              ,"Cx_vr="          //7
              ,"e1="             //8
              ,"t_obd="          //9
              ,"H1="             //10
              ,"H2="             //11
              ,"H3="             //12
              ,"V1="             //13
              ,"V2="             //14
              ,"V3="             //15
              ,"V1"              //16
              ,"V2"              //17
              ,"V3"              //18
              ,"`V1"             //19
              ,"`V2"             //20
              ,"`V3"             //21
              ,"`V1`"            //22
              ,"`V2`"            //23
              ,"`V3`"            //24
              ,"n="              //25
              ,"t="              //26
              ,"p="              //27
              ,"alfa1="          //28
              ,"alfa2="          //29
              ,"alfa3="          //30
              ,"alfa4="          //31
              ,"alfa5="          //32
              ,"alfa6="          //33
              ,"alfa7="          //34
              ,"alfa8="          //35
              ,"D_rv="          //36
              ,"b_rv="          //37
              ,"z_rv="          //38
              ,"k="             //39
              ,"note"             //40
              ,"end_note\n"         //41
              ,"V\\H"               //42
              ,"`V\\H"               //43
              ,"`V\\H`"               //44
              ,"Cy_max_pr="               //45

              ,"a="               //46
              ,"_k="               //47
              ,"Igs="               //48
              ,"B="               //49
              ,"dfi="               //50
              ,"V_Ksi"              //51
              ,"Ksi"                //52
              };                        //

        //Form1->OpenDialog1->FileName = filewrite;
        //is=fopen(Form1->OpenDialog1->FileName.c_str(),"r");
        if (is!=NULL)
        {
                 for (int i=0;i<52;i++)
                 {
                        x[i]=0;
                 }
                 while (fscanf(is,"%s",s)!=EOF)
                 {
                        if(strcmp(s,sc[0])==0) break;
                        if(strcmp(s,sc[1])==0) {if(fscanf(is,"%f",&D_nv)==1) x[1]++;}     //1
                        if(strcmp(s,sc[2])==0) {if(fscanf(is,"%f",&G)==1) x[2]++;}
                        if(strcmp(s,sc[3])==0) {if(fscanf(is,"%f",&z)==1) x[3]++;}
                        if(strcmp(s,sc[4])==0) {if(fscanf(is,"%f",&b_07)==1) x[4]++;}
                        if(strcmp(s,sc[5])==0) {if(fscanf(is,"%f",&wR)==1) x[5]++;}
                        //if(strcmp(s,sc[6])==0) {if(fscanf(is,"%f",&ksi)==1) x[6]++;}
                        if(strcmp(s,sc[7])==0) {if(fscanf(is,"%f",&Cx_vr)==1) x[7]++;}
                        //if(strcmp(s,sc[8])==0) {if(fscanf(is,"%f",&e1)==1) x[8]++;}
                        if(strcmp(s,sc[9])==0) {if(fscanf(is,"%f",&t_obd)==1) x[9]++;}

                        float ttt = 0;
                        //Коэффициент использования мощности
                        if(strcmp(s,sc[51])==0)
                        {
                                while (fscanf(is,"%f",&ttt)==1)
                                {
                                        V_Ksi.push_back(ttt); //добавление элемента в конец массива
                                }
                        }
                        if(strcmp(s,sc[52])==0)
                        {
                                while (fscanf(is,"%f",&ttt)==1)
                                {
                                        Ksi.push_back(ttt); //добавление элемента в конец массива
                                }
                        }

                        //взлетная мощность
                        if(strcmp(s,sc[42])==0)
                        {
                                while (fscanf(is,"%f",&ttt)==1)
                                {
                                        HNvzlet.push_back(ttt); //добавление элемента в конец массива
                                }
                        }
                        for (int i=1;i<=10;i++)
                        {
                                strcpy(ss,"V");
                                strcat(ss,IntToStr(i).c_str());
                                strcat(ss,"=");
                                if(strcmp(s,ss)==0)
                                {
                                        if (fscanf(is,"%f",&ttt)==1)
                                                VNvzlet.push_back(ttt);

                                        t_vec.clear();
                                        while (fscanf(is,"%f",&ttt)==1)
                                        {
                                                t_vec.push_back(ttt); //
                                        }

                                        Nv.push_back(t_vec);
                                 }
                        }
                        //номинальная мощность
                        if(strcmp(s,sc[43])==0)
                        {
                                while (fscanf(is,"%f",&ttt)==1)
                                {
                                        HNnom.push_back(ttt); //добавление элемента в конец массива
                                }
                        }
                        for (int i=1;i<=10;i++)
                        {
                                strcpy(ss,"`V");
                                strcat(ss,IntToStr(i).c_str());
                                strcat(ss,"=");
                                if(strcmp(s,ss)==0)
                                {
                                        if (fscanf(is,"%f",&ttt)==1)
                                                VNnom.push_back(ttt);
                                        t_vec.clear();
                                        while (fscanf(is,"%f",&ttt)==1)
                                        {
                                                t_vec.push_back(ttt); //
                                        }

                                        Nn.push_back(t_vec);
                                }
                        }

                        //крейсерская мощность
                        if(strcmp(s,sc[44])==0)
                        {
                                while (fscanf(is,"%f",&ttt)==1)
                                {
                                        HNk.push_back(ttt);
                                }
                        }
                        for (int i=1;i<=10;i++)
                        {
                                strcpy(ss,"`V");
                                strcat(ss,IntToStr(i).c_str());
                                strcat(ss,"`=");
                                if(strcmp(s,ss)==0)
                                {
                                        if (fscanf(is,"%f",&ttt)==1)
                                                VNk.push_back(ttt);
                                        t_vec.clear();
                                        while (fscanf(is,"%f",&ttt)==1)
                                        {
                                                t_vec.push_back(ttt); //
                                        }

                                        Nk.push_back(t_vec);
                                }
                        }

                        if(strcmp(s,sc[25])==0) {if(fscanf(is,"%f",&n)==1) x[25]++;}//43
                        if(strcmp(s,sc[26])==0) {if(fscanf(is,"%f",&t)==1) x[26]++;}//44
                        if(strcmp(s,sc[27])==0) {if(fscanf(is,"%f",&p_)==1) x[27]++;}//45

                        if(strcmp(s,sc[36])==0) {if(fscanf(is,"%f",&D_rv)==1) x[36]++;}//46
                        if(strcmp(s,sc[37])==0) {if(fscanf(is,"%f",&b_rv)==1) x[36]++;}//47
                        if(strcmp(s,sc[38])==0) {if(fscanf(is,"%f",&z_rv)==1) x[36]++;}//48
                        //if(strcmp(s,sc[39])==0) {if(fscanf(is,"%f",&k)==1) x[37]++;}//49
                        if(strcmp(s,sc[46])==0) {if(fscanf(is,"%f",&a)==1) x[46]++;}//46
                        if(strcmp(s,sc[47])==0) {if(fscanf(is,"%f",&_k)==1) x[47]++;}//47
                        if(strcmp(s,sc[48])==0) {if(fscanf(is,"%f",&Igs)==1) x[48]++;}//48
                        if(strcmp(s,sc[49])==0) {if(fscanf(is,"%f",&B)==1) x[49]++;}//49
                        if(strcmp(s,sc[50])==0) {if(fscanf(is,"%f",&dfi)==1) x[49]++;}//50

                        //очистка коммент.
                        for (int i=0; i<=9;i++)
                        {
                                strcpy(prim[i],"");
                        }
                        if(strcmp(s,sc[40])==0)
                        {
                                int kk=0;
                                fgets(prim[kk],10000,is);
                                while ((strcmp(prim[kk],sc[0])!=0) && (strcmp(prim[kk],sc[41])!=0) &&  (kk<10) && (strcmp(prim[kk],"End_source_data -------------------------------------\n")!=0))
                                {
                                        kk++;
                                        fgets(prim[kk],10000,is);

                                }
                                strcpy(prim[kk],"");
                        }
                        if(strcmp(s,sc[45])==0) {
                        if(fscanf(is,"%f",&Cy_MAX)==1) x[45]++;}//49

                 }
        }

        //fclose(is);
        check_Vector(HNvzlet,VNvzlet,&Nv);
        check_Vector(HNnom,VNnom,&Nn);
        check_Vector(HNk,VNk,&Nk);
        check_Vektor_Ksi(V_Ksi, &Ksi);
        bool pr = ProfilDataRead();
        //int sum=0;
        //
        //         for (int i=1;i<(50);i++)
        //         {
        //                sum = sum + x[i];
        //         }
        //         if ((sum != (xx)) || (pr!=true) || (HNvzlet.size()<2) || (HNnom.size()<2) || (HNk.size()<2))
        //         {
        //                Application->MessageBoxA(" Ошибка считывания исходных данных!",MB_OK);
        //                return false;
        //         }
        //         else
        //
        //                return true;

}*/

/*bool Impulse::ProfilDataRead()
{
        int x[150];
        char s[200]="";
        char ss[50];
        char *sc[] = {"End_source_data"

              ,"a1="          //1
              ,"a2="          //2
              ,"a3="          //3
              ,"a4="          //4
              ,"a5="          //5
              ,"a6="          //6
              ,"a7="          //7
              ,"a8="          //8
              ,"a9="          //9

              ,"M1"          //10
              ,"M2"          //11
              ,"M3"          //12
              ,"M4"          //13
              ,"M5"          //14
              ,"M6"          //15
              };
        alfaP[0]=-7;
        alfaP[10]=72;
        for (int i=1;i<7;i++)
        {
                CYp[i-1][0]=-0.62;
                CYp[i-1][10]=0.35;
                CXp[i-1][0]=0.04;
                CXp[i-1][10]=1.1;
        }
        is=fopen(Form1->OpenDialog1->FileName.c_str(),"r");
        float tt;
        if (is!=NULL)
        {
                 for (int i=0;i<=69;i++)
                 {
                        x[i]=0;
                 }
                 int n=9,m=5;
                 while (fscanf(is,"%s",s)!=EOF)
                 {

                        if(strcmp(s,sc[0])==0) break;
                        for (int i=1;i<=n;i++)
                        {
                                strcpy(ss,"a");
                                strcat(ss,IntToStr(i).c_str());
                                strcat(ss,"=");
                                if(strcmp(s,ss)==0) {if(fscanf(is,"%f",&alfaP[i])==1) x[i]++;}     //9
                        }
                        for (int i=0;i<=m;i++)
                        {
                                strcpy(ss,"M");
                                strcat(ss,IntToStr(i+1).c_str());
                                strcat(ss,"=");
                                if(strcmp(s,ss)==0) {if(fscanf(is,"%f",&MP[i])==1) x[i+10]++;}     //15
                        }
                        //read Cy
                        for (int i=0;i<=m;i++)
                        {
                                strcpy(ss,"M");
                                strcat(ss,IntToStr(i+1).c_str());
                                if(strcmp(s,ss)==0)
                                {
                                        for (int j=1;j<=n;j++)
                                        {
                                                if(fscanf(is,"%f",&CYp[i][j])==1) x[i+16]++;  //54
                                        }
                                }
                        }
                        //read Cx
                        for (int i=0;i<=m;i++)
                        {
                                strcpy(ss,"`M");
                                strcat(ss,IntToStr(i+1).c_str());
                                if(strcmp(s,ss)==0)
                                {
                                        for (int j=1;j<=n;j++)
                                        {
                                                if(fscanf(is,"%f",&CXp[i][j])==1) x[i+23]++;  //64
                                        }
                                }
                        }
                        for (int i=0;i<=m;i++)
                        {
                                strcpy(ss,"a_sr");
                                strcat(ss,IntToStr(i+1).c_str());
                                strcat(ss,"=");
                                if(strcmp(s,ss)==0) {if(fscanf(is,"%f",&a_sr[i])==1) x[i+30]++;}     //70
                        }
                 }
        }
        //tt = CYp[5][9];
        fclose(is);
        int xx=129;
        int sum=0;
                 for (int i=1;i<=69;i++)
                 {
                        sum = sum + x[i];
                 }
                 if (sum != (xx))
                 {
                        Application->MessageBoxA(" Ошибка считывания исходных данных профиля!",MB_OK);
                        return false;
                 }
                 else
                        return true;


}*/

void Impulse::check_Vector(vector<float> H, vector<float> V,vector< vector<float> >* N)
{
        //
        float hl = H.size();
        float vl = V.size();
        t_vec.clear();
        t_vec.resize(hl,0.0);
        if (N->size()!=vl)
                N->resize(vl,t_vec);
        //t_vec.clear();

        //t_vec.resize(hl,0);
        for (int i=0; i<vl;i++)
        {
                if (N->at(i).size()!= hl)
                        N->at(i).resize(hl,0.0);
        }
}
void Impulse::check_Vektor_Ksi(vector<float> V_ksi, vector<float>* ksi)
{
        if (ksi->size()!= V_ksi.size())
                ksi->resize(V_ksi.size(),0.81);
}



void Impulse::SolveImpulse()
{
    //if (DataRead()==true)
    //{
        //определение коснтант------------------------------------------
        for (int i=0;i<mas;i++)
            for (int j=0;j<mas_;j++)
                __ind[j][i] = 0;

        //ComboBox1->Items->Clear();
        i_ = 1000;
        j_ = 1000;
        in = 0;
        I0 = getI0(dfi);
        //--------------------------------------------------------------

        //блок расчета**************************************************
        Calc_max_step();
        StaticCeiling();
        DynCeiling();
        DynCeiling_nom();
        //**************************************************************
        dfi = dfi*180/M_PI;
        //запись результатов
        //RezWrite();
        //DrawChart();
    //}
}



void Impulse::Calc()//в расчете не используется
{
    g = 9.80665;
    db = 0.5/57.2957;//балансировочный угол //Юрьев, с. 404
    dfi = dfi*M_PI/180;
    F = (M_PI*pow(D_nv,2))/4;//площадь винта,м^2
    p = G*g/F;//  Удельная нагрузка, Н/ м2
    sigma = z*b_07/(M_PI*(D_nv/2)); //коэф. заполнения НВ
    sigma_rv = z_rv*b_rv/(M_PI*(D_rv/2)); //коэф. заполнения РВ
    F_rv = (M_PI*pow(D_rv,2))/4;//площадь винта,м^2
    popr_RV = 0;//popr_RV = sigma_rv*F_rv/(sigma*F);

        for (int i=0; i<mas_;i++)
                V[i]=i;

        for (int i=0; i<mas; i++)
        {

                H[i] = i*10;
                //ComboBox1->Items->Add(FloatToStr(H[i]));

                for (int j=0; j<mas_;j++)
                {
                       //N_gor2(H[i],V[j],i,j);
                       N_gor_Vildgrube(H[i],V[j],i,j);
                }
                Vmax_sr[i] = stallingSpeed_Ct(i);
                Vsr_a[i] = stallingSpeed_a_kr(i);
                Vsr_Cy[i] = stallingSpeed_Cy(i);
                Vmax[i] = MaxSpeed(i);
                Vmin[i] = MinSpeed(i);
                VmaxNom[i] = MaxSpeedNom(i);
                VminNom[i] = MinSpeedNom(i);
                Vnaiv[i] = NaivSpeed(i);
                Vy_g[i] = Vy_gor(i);
                Vy_g_nom[i] = Vy_gor_nom(i);
                Vy_v[i] = Vy_vert2(i);
                Vek[i] = EkSpeed(i);
                Vnaiv[i] = NaivSpeed(i);

        }
        setV500();
}

void Impulse::Calc_max_step()
{
    g = 9.80665;
    db = 0.5/57.2957;//балансировочный угол //Юрьев, с. 404
    dfi = dfi*M_PI/180;
    F = (M_PI*pow(D_nv,2))/4;//площадь винта,м^2
    p = G*g/F;//  Удельная нагрузка, Н/ м2
    sigma = z*b_07/(M_PI*(D_nv/2)); //коэф. заполнения НВ
    sigma_rv = z_rv*b_rv/(M_PI*(D_rv/2)); //коэф. заполнения РВ
    F_rv = (M_PI*pow(D_rv,2))/4;//площадь винта,м^2
    popr_RV = 0;//popr_RV = sigma_rv*F_rv/(sigma*F);

    for (int i=0; i<mas_;i++)//0-300
        V[i]=i;

    for (int i=0; i<mas; i++)//0-800
    {
        H[i] = i*10;
        //ComboBox1->Items->Add(FloatToStr(H[i]));
        V_sryv_Ct[i]=mas_-1;
        for (int j=0; j<mas_;j++)
        {
            N_max_step(H[i],V[j],i,j);
        }
        Vmax[i]     = MaxSpeed(i);
        Vmin[i]     = MinSpeed(i);
        VmaxNom[i]  = MaxSpeedNom(i);
        VminNom[i]  = MinSpeedNom(i);
        Vnaiv[i]    = NaivSpeed(i);
        NNaiv[i]    = NaivN(i);
        Vy_g[i]     = Vy_gor(i);
        Vy_g_nom[i] = Vy_gor_nom(i);
        Vy_v[i]     = Vy_vert2(i);
        Vek[i]      = EkSpeed(i);
        Vnaiv[i]    = NaivSpeed(i);
    }
    setV500();
}



void Impulse::N_max_step(float H,float V, int i, int j)///мощность для гор. полета
{
        float ro,CxS=0;
        Xvr[j][i] = tau[j][i] = alfa_k[j][i] = mu[j][i]= B_g[j][i] = 0;
        //if (V==100) V=V;
        V = V/3.6; //в м/с
        ro = getRo(H);
        CxS = 1.28*F*Cx_vr; //+
        gama = ro*a*b_07*pow((0.5*D_nv),4)*0.5/Igs/g;//массовая х-ка лопасти // не подтверждено
        Xvr[j][i] = 0.5*CxS*ro*pow(V,2); //сопротивление фюзеляжа,Н //+
        tau[j][i] = (-1)*atan(Xvr[j][i]/(g*G)); //угол качества     //+
        alfa_k[j][i] = (tau[j][i] - db);//конструктивный угол атаки //+ средний угол //Юрьев,с.410
        mu[j][i] = V*cos(alfa_k[j][i])/wR; //коэф. режима работы     //+
            //alfa_k[j][i] =(-1)*(0.5*mu[j][i]*mu[j][i]*CxS/(sigma*F)+0.25*mu[j][i]*0.013)*ro*sigma*F*wR*wR/(G*g);//Брамвелл, с.154
        T_g[j][i] = G*g/(cos(tau[j][i])*(1-t_obd));//Тяга НВ, Н     //+
        Ct[j][i] = T_g[j][i]/(0.5*ro*F*pow(wR,2)); //коэф. тяги     //+
        float v1v = sqrt(T_g[j][i]/(2*ro*F));//индуктивная скорость изолированного НВ на висении //+
            ///откуда alfa_k[j][i] ???
            alfa_k[j][i] = (-1)*(1/(1+0.6*mu[j][i]*mu[j][i]))*((1.6*mu[j][i]*(1+1.25*mu[j][i]*mu[j][i])-0.5*sigma)*Ct[j][i]/sigma+0.008*mu[j][i]*sigma/Ct[j][i]+CxS*mu[j][i]*mu[j][i]/(Ct[j][i]*F));
            mu[j][i] = V*cos(alfa_k[j][i])/wR; ///повтор !!! коэф. режима работы //
        B_g[j][i] = B;//Коэффициент концевых потерь (xi)
        Cy[j][i] = 3*Ct[j][i]/(B_g[j][i]*sigma*(1+1.2*mu[j][i]*mu[j][i]));//коэф. подъемной силы, Мартынов, с.65 //+

        //коэф. профильного сопротивления***************************************
        float M07,al,Np0;
        if (V==0)
        {
            M07 = 0.7*wR/getAzv(H);
            al =  GetAlfa(Cy[j][i],M07);
            Cx[j][i] = GetCx(M07,al);
            m_pr[j][i] = sigma*Cx[j][i]*(1+n*mu[j][i]*mu[j][i])/4.0;
            N_p[j][i] = 0.5*m_pr[j][i]*ro*F*pow(wR,3)/735.5;
            Np0 = N_p[j][i];
        }
        else
        {
            ///Np0 не определен
            N_p[j][i] = Np0 + G*3.774E-9*(V*3.6)*(V*3.6)*(V*3.6);
            m_pr[j][i] = 2*N_p[j][i]*735.5/(ro*F*pow(wR,3));
            Cx[j][i] = m_pr[j][i]*4.0/(sigma*(1+n*mu[j][i]*mu[j][i]));
        }
        /*
        float M[8], Cy_az[8],a_zv, a_az[8],Cx_az[8],azimut[8];
        float A1,A2,A3;
        for (int m=0; m<8;m++)
        {
                azimut[m] = m*45/57.2957;
                //Cy_az[m] = Cy[j][i]*(1+mu[j][i]*mu[j][i])/(1+(8/3)*mu[j][i]*sin(azimut[m])+ 2*mu[j][i]*mu[j][i]*sin(azimut[m])*sin(azimut[m]));
                A1 = 1+mu[j][i]*mu[j][i];
                A2 = 8.0*mu[j][i]*sin(azimut[m])/3.0;
                //A2 = (8/3)*mu[j][i]*sin(azimut[m]);
                //A2 = 8/3;         `
                A3 = 2*mu[j][i]*mu[j][i]*sin(azimut[m])*sin(azimut[m]);

                Cy_az[m] = Cy[j][i] * A1 /(1+ A2 + A3);
                a_zv = getAzv(H);//скорость звука, м/с
                M[m] = (wR*0.7+V*cos(alfa_k[j][i])*sin(azimut[m]))/a_zv;
                a_az[m] = GetAlfa(Cy_az[m],M[m]); //угол атаки на азимуте
                if (m==6)
                        M270[j][i] = (wR*0.7+V*cos(alfa_k[j][i])*sin(azimut[m]))/a_zv;
                Cx_az[m] = GetCx(M[m],a_az[m]);
        }
        alfa7[j][i] = 0 ;
        Cx[j][i] = 0;
        for (int m=0; m<8;m++)
        {
                alfa7[j][i] = alfa7[j][i] +a_az[m] ;
                Cx[j][i] = Cx[j][i] + Cx_az[m];
        }
        alfa7[j][i] = alfa7[j][i]/8;
        Cx[j][i] = Cx[j][i]/8;
        */

        //индуктивная скорость++++++++++++++++++++++++++++++++++++++++++++++++++
        float ct = Ct[j][i]*0.5;
        float d=1;
        float lambda_0, lambda_1, lambda_i;
        alfa_k[j][i] = alfa_k[j][i]*(-1);
        //mu[j][i]= V*cos(alfa_k[j][i])/wR;   //коэф. режима работы
        lambda_0 = mu[j][i]*tan(alfa_k[j][i])+ct/(2*sqrt(mu[j][i]*mu[j][i]+ct/2*B));//+
        while (d>0.01)
        {
            lambda_1 = (mu[j][i]*tan(alfa_k[j][i]) + ct*(mu[j][i]*mu[j][i] + 2*lambda_0*lambda_0)/(2*B*pow((mu[j][i]*mu[j][i] + lambda_0*lambda_0),1.5)))
                      /(1+ct*lambda_0/(2*B*pow((mu[j][i]*mu[j][i] + lambda_0*lambda_0),1.5)));//+
            d = fabs(lambda_1 - lambda_0)/lambda_1;
            if (d>0.01)
                lambda_0 = lambda_1;
        }
        lambda_i = ct/(2*B*sqrt(mu[j][i]*mu[j][i] + lambda_1*lambda_1));
        Vi_g[j][i] = lambda_i*wR;
        alfa_k[j][i] = alfa_k[j][i]*(-1);
        lambda[j][i] = (-1)*lambda_1;
        //lambda[j][i] = (V*sin(alfa_k[j][i]) - Vi_g[j][i])/wR;
        //alfa_k[j][i]=(-1)*(1/(1+0.6*mu[j][i]*mu[j][i]))*((1.6*mu[j][i]*(1+1.25*mu[j][i]*mu[j][i])-0.5*sigma)*Ct[j][i]/sigma+0.008*mu[j][i]*sigma/Ct[j][i]+CxS*mu[j][i]*mu[j][i]/(Ct[j][i]*F));
        //угол атаки  Тищенко, Урбанович  с. 275
        /*
        if (V==0)
                alfa_k[j][i]= -90/57.2957;
        else
                alfa_k[j][i]= lambda[j][i]/mu[j][i] + 0.25*Ct[j][i]/(mu[j][i]*sqrt(mu[j][i]*mu[j][i]+lambda[j][i]*lambda[j][i]));
        */
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        I[j] = getI(V*3.6);
        float n = get_n(mu[j][i]);
        k = I[j]/B;
        m_i[j][i] = k * (Vi_g[j][i]/wR) * Ct[j][i]; //Джонсон, с.271    //не подтверждено
        //N_p[j][i] = 2.4218E-4*wR*(1+7.0833E-8*(V*3.6)*(V*3.6)*(V*3.6));
        //m_pr[j][i] = sigma*Cx[j][i]*(1+n*mu[j][i]*mu[j][i])/4; //Джонсон, с.215  //не подтверждено
        m_vr[j][i] = CxS*pow((V/wR),3)/F;  //Джонсон, с. 272            //не подтверждено
        mk[j][i] = m_i[j][i] + m_pr[j][i] + m_vr[j][i];

        N_i[j][i] = 0.5*m_i[j][i]*ro*F*pow(wR,3)/735.5;
        //N_p[j][i] = 0.5*m_pr[j][i]*ro*F*pow(wR,3)/735.5;
        N_prop[j][i] = 0.5*m_vr[j][i]*ro*F*pow(wR,3)/735.5;
        //N_prop[j][i] = Xvr[j][i]*V/735.5;
        N_nv[j][i] = N_i[j][i] + N_p[j][i] + N_prop[j][i];  //мощность винта
        //срыв
               //Вильдгрубе
        /*
        float Ct_kr = get_Ct_kr(mu[j][i])*sigma*Cy_MAX;
        if (Ct[j][i]>=Ct_kr)
        {
                if (V_sryv_Ct[i]==(mas_-1))
                        V_sryv_Ct[i] = V*3.6;
        }
        */
        /*            //ХАИ (лекции Урбанович)
        float Ct_kr = get_Ct_kr_KhAI(mu[j][i])*sigma;
        if (Ct[j][i]>=Ct_kr)
        {
                if (V_sryv_Ct[i]==(mas_-1))
                        V_sryv_Ct[i] = V*3.6;
        }
        */
                   //Гребенников, Тимченко с. 213
        float Ct_kr = get_Ct_kr_GT(mu[j][i])*sigma*Cy_MAX;
        if (Ct[j][i]>=Ct_kr)
        {
                if (V_sryv_Ct[i]==(mas_-1))
                        V_sryv_Ct[i] = V*3.6;
        }
        /*             //Вильдгрубе (Макаров)//Гребенников, Тимченко с. 211
        if (Cy[j][i]>=(get_Cy_kr(mu[j][i])*Cy_MAX))
        {
                if (V_sryv_Ct[i]==(mas_-1))
                        V_sryv_Ct[i] = V*3.6;
        }
        */

        B_g[j][i] = sqrt(B_g[j][i]);
        a0[j][i] = (0.125*gama*Ct[j][i]/sigma);//-0.01*D_nv; //угол конусности, Вильдгрубе с.73
        fi0_e[j][i] = (3/(B_g[j][i]*B_g[j][i]+1.5*mu[j][i]*mu[j][i]))*(Ct[j][i]/(sigma*a*B_g[j][i]) - 0.5*lambda[j][i]*B_g[j][i]);
        B_g[j][i] = B_g[j][i]*B_g[j][i];
        //fi0_e_j[j][i] = (3/(1+1.5*mu[j][i]*mu[j][i]))*(Ct[j][i]/(sigma*a) - 0.5*lambda[j][i]-0.125*dfi*(mu[j][i]*mu[j][i]));
        fi0_e_j[j][i] = (3/(1+1.5*mu[j][i]*mu[j][i]))*(Ct[j][i]/(sigma*a*1) - 0.5*lambda[j][i]*1);
        fi0[j][i] = fi0_e[j][i] + _k*a0[j][i];
        fi0_j[j][i] = fi0_e_j[j][i] + _k*a0[j][i];
        if ((V/wR)<1.15*sqrt(Ct[j][i]))
                fi0_v[j][i] = 0;
        else
                fi0_v[j][i] = ((21.8*Ct[j][i]/sigma*(1.43+sigma/mu[j][i])-1.35*mu[j][i]*alfa_k[j][i]*57.2957+0.05*dfi)/57.2957) + _k*a0[j][i];  //Вильдгрубе с.71

        ///РВ

        m_i_rv [j][i] = m_i[j][i]*popr_RV;
        m_pr_rv[j][i] = m_pr[j][i]*popr_RV;
        N_i_rv[j][i] = N_i[j][i]*popr_RV; //индуктивная мошность РВ
        N_p_rv[j][i] = N_p[j][i]*popr_RV; //профильная мощность РВ
        N_rv[j][i] = N_i_rv[j][i] + N_p_rv[j][i];  //суммарная мощность РВ

        //сумарная потребляемая мощность
        N[j][i] = (N_nv[j][i])/getKsi(V*3.6);
        float __ki = getKsi(V*3.6);
        ksi_rv[j][i] = N_rv[j][i]*100/N[j][i];

        /*
        //расход
        Ce[j][i] = getCe(N[j][i])/1000;
        Q[j][i] = Ce[j][i] * N[j][i]; //часовой расход
        if (V!=0)
                q[j][i] = Q[j][i] /(V*3.6); //километровый расход
        else    q[j][i] = 0;
        */

        //float I= getI(325);

        //статический потолок
        float Vsp = 0.5; //скороподъемность на статическом потолке
        float deN = 0; //изменение мощности для подъема на 0,5 м/с
        float v1 = 0;
        if (j==0)
        {
                v1 = (-0.5)*Vsp + sqrt(Vsp*Vsp/4 + v1v*v1v);
                //deN = P_g[j][i]*(Vsp - v1v + v1)/(75*B_g[j][i]*ksi);
                deN = G*(Vsp - v1v + v1)/(75*B_g[j][i]*getKsi(0));
                Nsp[i] = N[j][i] + deN;
        }
        if (j==0)
                eta0[i] = pow(ct,1.5)/(k*pow(ct,1.5) + (sqrt(2)/8)*sigma*Cx[j][i]);
}



float Impulse::getKsi(float V)
{
        float KSI, k =0;
        if (V<=V_Ksi.at(0))
        {
                KSI = Ksi.at(0);
        }
        else
        {
                if (V>V_Ksi.at(V_Ksi.size()-1))
                {
                        KSI = Ksi.at(V_Ksi.size()-1);
                }
                else
                {
                        for (int i=0;i<V_Ksi.size();i++)
                        {
                                if ((V_Ksi.at(i)<V) && (V<=V_Ksi.at(i+1)))
                                {
                                        k = (Ksi.at(i+1) - Ksi.at(i))/(V_Ksi[i+1] - V_Ksi[i]);
                                        KSI = k*(V - V_Ksi[i]) + Ksi.at(i);
                                }
                        }
                }

        }

        return  KSI;
}



float Impulse::getI(float V)
{
        float I, k=0;
        float _V[] =     {0, 150,  200, 250,  300,    350};
        float _I[] =     {0, 1.09,  1.1, 1.12,  1.18,  1.28};
        int n =6;
        float DFI = dfi*(180/M_PI);

        if (V==0) { I=I0;}
        else
        {
                _I[0] = I0;
                if (V<=_V[0])
                {
                        I = _I[0];
                }
                for (int i=0;i<n;i++)
                {
                        if ((_V[i]<V) && (V<=_V[i+1]))
                        {
                                k = (_I[i+1] - _I[i])/(_V[i+1] - _V[i]);
                                I = k*(V - _V[i]) + _I[i];
                        }
                }
                if (V>_V[n-1])
                {
                        I = _I[n-1];
                }

        }

        return  I;
}



float Impulse::getI0(float dfi)
{
        float I0, k=0;
        const float _df[] =    {0, 2,  4, 6,  8,    10};
        const float _I0[] =    {1.098,1.072 , 1.053, 1.038, 1.03, 1.02};
        int n =6;
        float DFI = dfi*(180/M_PI);

        if (DFI<=_df[0])
        {
                I0 = _I0[0];
        }
        for (int i=0;i<n;i++)
        {
                if ((_df[i]<DFI) && (DFI<=_df[i+1]))
                {
                        k = (_I0[i+1] - _I0[i])/(_df[i+1] - _df[i]);
                        I0 = k*(DFI - _df[i]) + _I0[i];
                }
        }
        if (DFI>_df[n-1])
        {
                I0 = _I0[n-1];
        }

        return  I0;
}



float Impulse::NaivN(int i)
{
        int k=0;
        float Nnaiv,tg, dN[mas_];

        Nnaiv = N[0][i];
        tg = N[0][i];
        for (int j=1;j<mas_;j++)
        {
                if ((N[j][i]/V[j])<= tg)
                {
                        tg = N[j][i]/V[j];
                        Nnaiv = N[j][i];
                }
        }

        return Nnaiv;

        /*
        Nnaiv = Q[0][i];
        tg = Q[0][i];
        for (int j=1;j<mas_;j++)
        {
                if ((Q[j][i]/V[j])<= tg)
                {
                        tg = Q[j][i]/V[j];
                        Nnaiv = N[j][i];
                }
        }

        return Nnaiv;
        */
}



float Impulse::NaivSpeed(int i)
{
        int k=0;
        float Vnaiv,tg, dN[mas_];

        Vnaiv = V[0];

        tg = N[0][i];
        for (int j=1;j<mas_;j++)
        {
                if ((N[j][i]/V[j])<= tg)
                {
                        tg = N[j][i]/V[j];
                        Vnaiv = V[j];
                }
        }

        /*
        tg = Q[0][i];
        for (int j=1;j<mas_;j++)
        {
                if ((Q[j][i]/V[j])<= tg)
                {
                        tg = Q[j][i]/V[j];
                        Vnaiv = V[j];
                }
        }
        */
        for (int j=0;j<mas_;j++)
        {
                //dN[j] = n*getNvzl(H[i],V[j]) - N[j][i];
                dN[j] = n*getN(HNvzlet, VNvzlet, Nv, H[i],V[j]) - N[j][i];
                if (dN[j]>0)
                        k++;
        }
        if (Vnaiv==1)
                Vnaiv = 0;
        return Vnaiv;
}



float Impulse::get_Np(float V, float c_) //профильная мощность по Вильдгрубе
{
        return  0;

}



float Impulse::getCe(float N)
{
       float Ce=0;
       float N_ = 0.5*N;
       return Ce = 2E-06*N_*N_*N_ + 0.0002*N_*N_ - 1.1216*N_ + 661.37;
       //return Ce =0.0018*N_*N_ - 1.6145*N_ + 710.73;
}



float Impulse::get_Cy_kr(float mu)//определение _Cy0 kr с.71 Вильдгрубе
{
        float _Cy_kr, k=0;
        const float mu_1[] =    {0, 0.1,  0.2, 0.3,  0.4,    0.5};
        const float _Cy0_kr[] = {1, 0.78, 0.6, 0.45, 0.3375, 0.2625};
        int n =6;


        if (mu<=mu_1[0])
        {
                _Cy_kr = _Cy0_kr[0];
        }
        for (int i=0;i<n;i++)
        {
                if ((mu_1[i]<mu) && (mu<=mu_1[i+1]))
                {
                        k = (_Cy0_kr[i+1] - _Cy0_kr[i])/(mu_1[i+1] - mu_1[i]);
                        _Cy_kr = k*(mu - mu_1[i]) + _Cy0_kr[i];
                }
        }
        if (mu>mu_1[n-1])
        {
                _Cy_kr = _Cy0_kr[n-1];
        }

        return  _Cy_kr;
}



float Impulse::get_Ct_kr(float mu)//определение _Ct_ kr с.71 Вильдгрубе
{
        float __Ct_kr, k=0;
        const float mu_1[] =    {0,  0.1,  0.2,  0.3,  0.4,    0.5};
        const float _Ct_kr[] = {0.313, 0.245, 0.19, 0.155, 0.128,   0.107};
        int n =6;


        if (mu<=mu_1[0])
        {
                __Ct_kr = _Ct_kr[0];
        }
        for (int i=0;i<n;i++)
        {
                if ((mu_1[i]<mu) && (mu<=mu_1[i+1]))
                {
                        k = (_Ct_kr[i+1] - _Ct_kr[i])/(mu_1[i+1] - mu_1[i]);
                        __Ct_kr = k*(mu - mu_1[i]) + _Ct_kr[i];
                }
        }
        if (mu>mu_1[n-1])
        {
                __Ct_kr = _Ct_kr[n-1];
        }

        return  __Ct_kr;
}



float Impulse::get_Ct_kr_KhAI(float mu)//определение _Ct_ kr KhAI
{
        float __Ct_kr, k=0;
        const float mu_1[] =    {0,  0.1,  0.2,  0.3,  0.4,    0.5};
        const float _Ct_kr[] = {0.5, 0.38, 0.29, 0.215, 0.16,   0.125};
        int n =6;


        if (mu<=mu_1[0])
        {
                __Ct_kr = _Ct_kr[0];
        }
        for (int i=0;i<n;i++)
        {
                if ((mu_1[i]<mu) && (mu<=mu_1[i+1]))
                {
                        k = (_Ct_kr[i+1] - _Ct_kr[i])/(mu_1[i+1] - mu_1[i]);
                        __Ct_kr = k*(mu - mu_1[i]) + _Ct_kr[i];
                }
        }
        if (mu>mu_1[n-1])
        {
                __Ct_kr = _Ct_kr[n-1];
        }

        return  __Ct_kr;
}



float Impulse::get_Ct_kr_GT(float mu)//определение _Ct_ kr KhAI
{
        float __Ct_kr, k=0;
        const float mu_1[] =    {0,  0.1,  0.2,  0.3,  0.4,    0.5};
        const float _Ct_kr[] = {0.313, 0.245, 0.19, 0.155, 0.128,   0.107};
        int n =6;


        if (mu<=mu_1[0])
        {
                __Ct_kr = _Ct_kr[0];
        }
        for (int i=0;i<n;i++)
        {
                if ((mu_1[i]<mu) && (mu<=mu_1[i+1]))
                {
                        k = (_Ct_kr[i+1] - _Ct_kr[i])/(mu_1[i+1] - mu_1[i]);
                        __Ct_kr = k*(mu - mu_1[i]) + _Ct_kr[i];
                }
        }
        if (mu>mu_1[n-1])
        {
                __Ct_kr = _Ct_kr[n-1];
        }

        return  __Ct_kr;
}



float Impulse::getCy_pr_max(float M)//поиск максимального Су профиля
{
        float k1=0,maxCy;
        int m=6; //кол-во значений М
        int n=10; //кол-во a+1

        float _Cy[11];

        if (M<=MP[0])
        {
                for (int i=0;i<=n;i++)
                        _Cy[i] = CYp[0][i];
        }
        for (int i=0;i<m;i++)
        {
                if ((MP[i]<M) && (M<=MP[i+1]))
                {
                        for (int j=0;j<=n;j++)
                        {
                                k1 = (CYp[i+1][j] - CYp[i][j])/(MP[i+1] - MP[i]);
                                _Cy[j] = k1*(M - MP[i]) + CYp[i][j];
                        }
                }
        }
        if (M>MP[m-1])
        {
                for (int j=0;j<=n;j++)
                {
                        k1 = (CYp[m-1][j] - CYp[m-2][j])/(MP[m-1] - MP[m-2]);
                        _Cy[j] = k1*(M - MP[m-2]) + CYp[m-2][j];
                }
        }
        maxCy=_Cy[0];
        for (int j=0;j<=n;j++)
                if (_Cy[j]>maxCy)
                        maxCy = _Cy[j];
        return maxCy;
}



void Impulse::N_gor_Vildgrube(float H,float V, int i, int j)//мощность для гор. полета
{
   float _S=0,ro_otn=0;
   float Ie=0;
   if (V<=275)
        Ie = 1.02+0.0004*V;
   else
        Ie = 0.58 + 0.002*V;
   ro_otn = getRo(H)/getRo(0);
   _S = (0.018*pow(G,0.5646))/(G*g);
   if (V>=40)
        N__[j][i] = 1.36*G*g*(16.4e-3*wR*(1+7.08e-8*V*V*V)+1.64*p*Ie/(V*ro_otn)+13.2e-3*_S*V*V*V*ro_otn)/(ksi*1000);
   else
        N__[j][i] = -1;
}



float Impulse::getRo(float H)//плотность
{
        float ro_H,T,P_;
        float L=0.0065; //скорость падения температуры К/м
        float R=8.31447; //газовая постоянная Дж/(мол*К)
        float M=0.0289644; //молярная масса сухого воздуха кг/мол
        float g=9.80665;

        T = t+273 - L*H;
        P_ = p_*pow((1-L*H/(t+273)),(g*M/(R*L)));
        ro_H = P_*M/(R*T);

        return ro_H;
}



float Impulse::getAzv(float H)//скорость звука
{
        float a_zv,T;
        float L=0.0065; //скорость падения температуры К/м
        float R=287 ;  //газовая постоянная Дж/(кг*К)

        T = t+273 - L*H;
        a_zv = sqrt(1.4*T*R);

        return a_zv;
}



float Impulse::GetAlfaForPull(float ro, float G)//определение угла атаки для указанной высоты и тяги
{
    float ct = G*9.8/(0.5*ro*F*pow(wR,2));
    float cy = 3.2*ct/sigma;
    float alfa = GetAlfOfCy(cy);
    return alfa;
}



float Impulse::GetCxOfAlfa(float alfa)//определение Cx по альфа
{
   float Cx,k,b;
   int index1, index2;
   if (alfa < ALfa_p[0])
   {
        Cx = Cx_p[0];
   }
   if (alfa > ALfa_p[np-1])
   {
        Cx = Cx_p[np-1];
   }
   if ((alfa <= ALfa_p[np-1]) && (alfa >= ALfa_p[0]))
   {
        for (int i=1; i<np;i++)
        {
                if ((alfa>=ALfa_p[i-1]) && (alfa<=ALfa_p[i]))
                {
                        index1 = i-1;
                        index2 = i;
                        break;
                }
        }
        //определение угла атаки-----------------------------------------------
         k = (Cx_p[index2]-Cx_p[index1])/(ALfa_p[index2]-ALfa_p[index1]);
         b = Cx_p[index1] - k*ALfa_p[index1];
         Cx = k*alfa+b;
   }
   return Cx;
}



float Impulse::GetAlfOfCy(float cy)//определение альфа по Су
{
   float Alf,k,b;
   int index1, index2;
   if (cy < Cy_p[0])
   {
        Alf = ALfa_p[0];

   }
   if (cy > Cy_p[np-1])
   {
        Alf = ALfa_p[np-1];

   }
   if ((cy <= Cy_p[np-1]) && (cy >= Cy_p[0]))
   {
        for (int i=1; i<np;i++)
        {
                if ((cy>=Cy_p[i-1]) && (cy<=Cy_p[i]))
                {
                        index1 = i-1;
                        index2 = i;
                        break;
                }
        }
        //определение угла атаки-----------------------------------------------
         k = (ALfa_p[index2]-ALfa_p[index1])/(Cy_p[index2]-Cy_p[index1]);
         b = ALfa_p[index1] - k*Cy_p[index1];
         Alf = k*cy+b;
   }
   return Alf;
}



float Impulse::getNvzl(float h,float v)  //не исп.!!!
{
        float _N[3],k=0,k1=0,N=0;
        int ib, ie;


        //h = 9000; v = 400;

        for (int i=0; i<HNvzlet.size(); i++)
        {
                if  ((h>=HNvzlet[i]) && (h<HNvzlet[i+1]))
                {
                        ib = i;
                        ie = i+1;
                }
        }
        if (h>=HNvzlet[HNvzlet.size()-1])
        {
                ib = HNvzlet.size()-2;
                ie = ib+1;
        }

        /*
        k = (Nvv1[ie] - Nvv1[ib])/(HNvzlet[ie] - HNvzlet[ib]);
        _N[0] = k*(h - HNvzlet.at(ib)) + Nvv1[ib];

        k = (Nvv2[ie] - Nvv2[ib])/(HNvzlet[ie] - HNvzlet[ib]);
        _N[1] = k*(h - HNvzlet[ib]) + Nvv2[ib];

        k = (Nvv3[ie] - Nvv3[ib])/(HNvzlet[ie] - HNvzlet[ib]);
        _N[2] = k*(h - HNvzlet[ib]) + Nvv3[ib];


        if (v<=V_[0])
        {
                N = _N[0];
        }

        if ((V_[0]<v) && (v<=V_[1]))
        {
                k1 = (_N[1] - _N[0])/(V_[1] - V_[0]);
                N = k1*(v - V_[0]) + _N[0];
        }
        if ((V_[1]<v) && (v<=V_[2]))
        {
                k1 = (_N[2] - _N[1])/(V_[2] - V_[1]);
                N = k1*(v - V_[1]) + _N[1];
        }
        if (v>V_[2])
        {
                N = _N[2];
        }
         */
        return N = N*(1.1-0.0066*t);
}



float Impulse::getN(vector<float> H, vector<float> V, vector< vector<float> > N,float h,float v)
{
        float k=0,k1=0,N_=0,nt=0;
        int ib, ie;
        vector<float> _N;
        _N.clear();

        //h=500; v = 300;

        for (int i=0; i<H.size(); i++)
        {
                if  ((h>=H[i]) && (h<H[i+1]))
                {
                        ib = i;
                        ie = i+1;
                }
        }
        if (h>=H[H.size()-1])
        {
                ib = H.size()-2;
                ie = ib+1;
        }

        for (int i=0; i<V.size(); i++)
        {
                k = (N.at(i).at(ie) - N.at(i).at(ib))/(H[ie] - H[ib]);
                nt = k*(h - H[ib]) + N.at(i).at(ib);
                _N.push_back(nt);
        }


        if (v<=V.at(0))
        {
                N_ = _N.at(0);
        }
        for (int i=0;i<(V.size()-1);i++)
        {
                if ((V.at(i)<v) && (v<=V.at(i+1)))
                {
                        k1 = (_N.at(i+1) - _N.at(i))/(V.at(i+1) - V.at(i));
                        N_ = k1*(v - V.at(i)) + _N.at(i);
                }
        }

        if (v>V.at(V.size()-1))
        {
                N_ = _N.at(_N.size()-1);
        }

        return N_ = N_*(1.1-0.0066*t);
}



float Impulse::GetCx(float M,float a)
{
        float _Cx[6],k=0,k1=0,Cx=0;
        int m=6; //кол-во значений М
        int n=10; //кол-во a-1


        for (int i=0; i<m; i++)
        {
                k=0;
                if (a<=alfaP[0])
                {
                        _Cx[i] = CXp[i][0];
                }
                for (int j=0;j<n;j++)
                {
                        if ((alfaP[j]<a) && (a<=alfaP[j+1]))
                        {
                                k = (CXp[i][j+1] - CXp[i][j])/(alfaP[j+1] - alfaP[j]);
                                _Cx[i] = k*(a - alfaP[j]) + CXp[i][j];
                        }
                }
                if (a>alfaP[n])
                {
                        _Cx[i] = CXp[i][n];
                }
        }

        if (M<=MP[0])
        {
                Cx = _Cx[0];
        }
        for (int i=0;i<(m-1);i++)
        {
                if ((MP[i]<M) && (M<=MP[i+1]))
                {
                        k1 = (_Cx[i+1] - _Cx[i])/(MP[i+1] - MP[i]);
                        Cx = k1*(M - MP[i]) + _Cx[i];
                }
        }
        if (M>MP[m-1])
        {
                k1 = (_Cx[m-1] - _Cx[m-2])/(MP[m-1] - MP[m-2]);
                Cx = k1*(M - MP[m-2]) + _Cx[m-2];
        }

        return Cx;
}



float Impulse::get_n(float mu)
{
        float k1=0,n=4.5;
        int m=7; //кол-во значений М
        float mu0[] = {0,0.3,0.4,0.5,0.6,0.75,1};
        float n0[] = {4.5,4.69,4.83,4.99,5.18,5.49,6.11};

        //mu = 1.1;
        if (mu<=mu0[0])
        {
                n = n0[0];
        }
        for (int i=0;i<(m-1);i++)
        {
                if ((mu0[i]<mu) && (mu<=mu0[i+1]))
                {
                        k1 = (n0[i+1] - n0[i])/(mu0[i+1] - mu0[i]);
                        n = k1*(mu - mu0[i]) + n0[i];
                }
        }
        if (mu>mu0[m-1])
        {
                k1 = (n0[m-1] - n0[m-2])/(mu0[m-1] - mu0[m-2]);
                n = k1*(mu - mu0[m-2]) + n0[m-2];
        }

        return n;
}



float Impulse::GetA_sr(float M)
{
        float k1=0,_a_sr=0;
        int m=6; //кол-во значений М

        //M = 0.65;
        if (M<=MP[0])
        {
                _a_sr = a_sr[0];
        }
        for (int i=0;i<(m-1);i++)
        {
                if ((MP[i]<M) && (M<=MP[i+1]))
                {
                        k1 = (a_sr[i+1] - a_sr[i])/(MP[i+1] - MP[i]);
                        _a_sr = k1*(M - MP[i]) + a_sr[i];
                }
        }
        if (M>MP[m-1])
        {
                k1 = (a_sr[m-1] - a_sr[m-2])/(MP[m-1] - MP[m-2]);
                _a_sr = k1*(M - MP[m-2]) + a_sr[m-2];
        }

        return _a_sr;
}



float Impulse::GetAlfa(float cy,float M)
{
        float k1=0,k=0,a=0,maxCy,minCy;
        int max,min;
        int m=6; //кол-во значений М
        int n=10; //кол-во a+1

        float _Cy[11];

        if (M<=MP[0])
        {
                for (int i=0;i<=n;i++)
                        _Cy[i] = CYp[0][i];
        }
        for (int i=0;i<m;i++)
        {
                if ((MP[i]<M) && (M<=MP[i+1]))
                {
                        for (int j=0;j<=n;j++)
                        {
                                k1 = (CYp[i+1][j] - CYp[i][j])/(MP[i+1] - MP[i]);
                                _Cy[j] = k1*(M - MP[i]) + CYp[i][j];
                        }
                }
        }
        if (M>MP[m-1])
        {
                for (int j=0;j<=n;j++)
                {
                        k1 = (CYp[m-1][j] - CYp[m-2][j])/(MP[m-1] - MP[m-2]);
                        _Cy[j] = k1*(M - MP[m-2]) + CYp[m-2][j];
                }
        }
        //????
        maxCy=_Cy[0];
        max=0;
        minCy=_Cy[0];
        min=0;
        for (int j=0;j<=n;j++)
        {
                if (_Cy[j]>maxCy)
                {
                        maxCy = _Cy[j];
                        max = j;
                }
                if (_Cy[j]<minCy)
                {
                        minCy = _Cy[j];
                        min = j;
                }
        }
        if (cy>maxCy)
        {
                a=alfaP[max];
        }
        else
        {
                if (cy<minCy)
                {
                        a=alfaP[min];
                }
                else
                {
                        for (int j=0;j<n;j++)
                        {
                                if ((_Cy[j]<cy) && (cy<=_Cy[j+1]))
                                {
                                        k = (alfaP[j+1] - alfaP[j])/(_Cy[j+1] - _Cy[j]);
                                        a = k*(cy - _Cy[j]) + alfaP[j];
                                }
                        }
                }

        }
        return a;
}



void Impulse::StaticCeiling()//статический потолок
{
        Hst = H[0];
        Hst_sryv = (mas-1)*10;
        for (int i=0; i<mas; i++)
        {
                if (Vy_v[i]>=0.5)
                        Hst = H[i];
        }
        for (int i=0; i<mas; i++)
        {
                if (V_sryv_Ct[i]==0)
                        Hst_sryv = H[i];
        }
        if (Hst_sryv>Hst)
                Hst_sryv = Hst;
}



void Impulse::DynCeiling()
{
        Hdyn = H[0];
        for (int i=0; i<mas; i++)
        {
                if (Vy_g[i]>=0.5)
                        Hdyn = H[i];
        }
        for (int i=0; i<mas; i++)
        {
                if (V_sryv_Ct[i]>Vmin[i])
                        Hdyn_sryv = H[i];
        }
        if (Hdyn_sryv>Hdyn)
                Hdyn_sryv = Hdyn;
}



void Impulse::DynCeiling_nom()
{
        Hdyn_nom = H[0];
        for (int i=0; i<mas; i++)
        {
                if (Vy_g_nom[i]>=0.5)
                        Hdyn_nom = H[i];
                else
                        break;
        }
        for (int i=0; i<mas; i++)
        {
                if (V_sryv_Ct[i]>VminNom[i])
                        Hdyn_sryv_nom = H[i];
                else
                        break;
        }
        if (Hdyn_sryv_nom>Hdyn_nom)
                Hdyn_sryv_nom = Hdyn_nom;
}



void Impulse::setV500()
{
        float v=0;
        for (int j=0;j<(mas_);j++)
        {
                v = V[j];
                if ((v>=Vmax[50]) || (v>=V_sryv_Ct[50]))
                        break;
        }
        V500v = v;
        for (int j=0;j<(mas_);j++)
        {
                v = V[j];
                if ((v>=VmaxNom[50]) || (v>=V_sryv_Ct[50]) )
                        break;
        }
        V500n = v;
}



float Impulse::MaxSpeed(int i)
{
        float Vmax=0;
        float V_r;
        int j_r = (mas_-1),j_l = 0;

        while (__ind[j_r][i]!=0)
        {
                if (j_r>0)
                        j_r--;
                else
                {
                        j_r = 0;
                        break;
                }
        }
        while (__ind[j_l][i]!=0)
        {
                if (j_l<(mas_-1))
                        j_l++;
                else
                {
                        j_l= (mas_-1);
                        break;
                }
        }

        if (j_l<=j_r)
        {
                if (N[j_r][i]<(n*getN(HNvzlet, VNvzlet, Nv, H[i],V[j_r])))
                        Vmax =  V[j_r];
                else
                {
                        for (int j=(j_r);j>=j_l;j--)
                        {
                                Vmax = V[j];
                                if (N[j][i]<=(n*getN(HNvzlet, VNvzlet, Nv, H[i],V[j])))
                                        break;
                        }

                }
                if (Vmax == V[j_l])
                        Vmax = 0;
        }
        else
                Vmax= 0;

        return Vmax;
}



float Impulse::stallingSpeed_Ct(int i)//Grebennikov-Timqenko_Vertoletw-2010_210.jpg
{
        float v=0,_Ct_kr,_Ct,Cy_pr_max;
        for (int j=0;j<(mas_);j++)
        {
                v = V[j];
                //Cy_pr_max = getCy_pr_max(M270[j][i]);
                Cy_pr_max = Cy_MAX;
                _Ct = Ct[j][i]/(sigma*Cy_pr_max);
                _Ct_kr = 0.313*(1+1.2*mu[j][i]*mu[j][i])*(1-(8/3)*mu[j][i] + 2*mu[j][i]*mu[j][i])/
                        (1+mu[j][i]*mu[j][i]);
                if (_Ct>_Ct_kr)
                {
                        _Ct=_Ct_kr;
                        break;
                }
        }

        return v;
}



float Impulse::stallingSpeed_a_kr(int i)
{
        float v=0;
        for (int j=0;j<(mas_);j++)
        {
                v = V[j];
                if (sryv[j][i]>0)
                {
                        v=v;
                        break;
                }
        }
        return v;
}



float Impulse::stallingSpeed_Cy(int i)//Вильдгрубе, стр. 71
{
        float v=0;
        for (int j=0;j<(mas_);j++)
        {
                v = V[j];
                if (sryv_Cy[j][i]>0)
                {
                        v=v;
                        break;
                }
        }
        return v;
}



float Impulse::MinSpeed(int i)
{
        float Vmin=0;
        int j_r = (mas_-1),j_l = 0;

        while (__ind[j_r][i]!=0)
        {
                if (j_r>0)
                        j_r--;
                else
                {
                        j_r = 0;
                        break;
                }
        }
        while (__ind[j_l][i]!=0)
        {
                if (j_l<(mas_-1))
                        j_l++;
                else
                {
                        j_l= (mas_-1);
                        break;
                }
        }

        if (j_l<=j_r)
        {
                if (N[j_l][i]<(n*getN(HNvzlet, VNvzlet, Nv, H[i],V[j_l])))
                        Vmin =  V[j_l];
                else
                {
                        for (int j=(j_l);j<=j_r;j++)
                        {
                                Vmin = V[j];
                                if (N[j][i]<=(n*getN(HNvzlet, VNvzlet, Nv, H[i],V[j])))
                                        break;
                        }

                }
                if (Vmin == V[j_r])
                        Vmin = 349;
        }
        else
                Vmin = 349;
        return Vmin;

}



float Impulse::MaxSpeedNom(int i)
{
        float Vmax=0;
        float V_r;
        int j_r = (mas_-1),j_l = 0;

        while (__ind[j_r][i]!=0)
        {
                if (j_r>0)
                        j_r--;
                else
                {
                        j_r = 0;
                        break;
                }
        }
        while (__ind[j_l][i]!=0)
        {
                if (j_l<(mas_-1))
                        j_l++;
                else
                {
                        j_l= (mas_-1);
                        break;
                }
        }

        if (j_l<=j_r)
        {
                //if (N[j_r][i]<(n*getNnom(H[i],V[j_r])))
                if (N[j_r][i]<(n*getN(HNnom, VNnom, Nn, H[i],V[j_r])))
                        Vmax =  V[j_r];
                else
                {
                        for (int j=(j_r);j>=j_l;j--)
                        {
                                Vmax = V[j];
                                //if (N[j][i]<=(n*getNnom(H[i],V[j])))
                                if (N[j][i]<=(n*getN(HNnom, VNnom, Nn, H[i],V[j])))
                                        break;
                        }

                }
                if (Vmax == V[j_l])
                        Vmax = 0;
        }
        else
                Vmax= 0;

        return Vmax;
}



float Impulse::MinSpeedNom(int i)
{
        float Vmin=0;
        int j_r = (mas_-1),j_l = 0;

        while (__ind[j_r][i]!=0)
        {
                if (j_r>0)
                        j_r--;
                else
                {
                        j_r = 0;
                        break;
                }
        }
        while (__ind[j_l][i]!=0)
        {
                if (j_l<(mas_-1))
                        j_l++;
                else
                {
                        j_l= (mas_-1);
                        break;
                }
        }

        if (j_l<=j_r)
        {
                if (N[j_l][i]<(n*getN(HNnom, VNnom, Nn, H[i],V[j_l])))
                        Vmin =  V[j_l];
                else
                {
                        for (int j=(j_l);j<=j_r;j++)
                        {
                                Vmin = V[j];
                                if (N[j][i]<=(n*getN(HNnom, VNnom, Nn, H[i],V[j])))
                                        break;
                        }

                }
                if (Vmin == V[j_r])
                        Vmin = 349;
        }
        else
                Vmin = 349;
        return Vmin;
}



float Impulse::Vy_gor(int i)
{
        float Vy,dN[mas_],vy[mas_];
        int j_r = (mas_-1),j_l = 0;
        while (__ind[j_r][i]!=0)
        {
                if (j_r>0)
                        j_r--;
                else
                {
                        j_r = 0;
                        break;
                }
        }
        while (__ind[j_l][i]!=0)
        {
                if (j_l<(mas_-1))
                        j_l++;
                else
                {
                        j_l= (mas_-1);
                        break;
                }
        }

        if (j_l<=j_r)
        {
                for (int j=j_l;j<=j_r;j++)
                {
                        dN[j] = n*getN(HNvzlet, VNvzlet, Nv, H[i],V[j]) - N[j][i];
                        vy[j] = 75*dN[j]*B_g[j][i]/G;
                }
                Vy = vy[j_l];
                for (int j=j_l;j<=j_r;j++)
                {
                        if (vy[j]>Vy)
                                Vy = vy[j];
                }

                if (Vy<0)
                        Vy = 0;
        }
        else
                Vy= 0;

        return Vy;
}



float Impulse::Vy_gor_nom(int i)
{
        float Vy,dN[mas_],vy[mas_];

        int j_r = (mas_-1),j_l = 0;

        while (__ind[j_r][i]!=0)
        {
                if (j_r>0)
                        j_r--;
                else
                {
                        j_r = 0;
                        break;
                }
        }
        while (__ind[j_l][i]!=0)
        {
                if (j_l<(mas_-1))
                        j_l++;
                else
                {
                        j_l= (mas_-1);
                        break;
                }
        }

        if (j_l<=j_r)
        {
                for (int j=j_l;j<=j_r;j++)
                {
                        //dN[j] = n*getNnom(H[i],V[j]) - N[j][i];   getN(HNnom, VNnom, Nn, H[i],V[j])
                        dN[j] = n*getN(HNnom, VNnom, Nn, H[i],V[j]) - N[j][i];
                        vy[j] = 75*dN[j]*B_g[j][i]/G;
                }
                Vy = vy[j_l];
                for (int j=j_l;j<=j_r;j++)
                {
                        if (vy[j]>Vy)
                                Vy = vy[j];
                }

                if (Vy<0)
                        Vy = 0;
        }
        else
                Vy= 0;

        return Vy;
}



float Impulse::Vy_vert(int i)
{
        float Vy,Npotr,Nrasp,v;
        Nrasp = n*getN(HNvzlet, VNvzlet, Nv, H[i],0);
        Vy = -0.01;
        Npotr = 0;
        while (Npotr <= Nrasp)
        {
                Vy = Vy + 0.01;
                v  =0.5*(-1*Vy+pow((Vy*Vy+Ct[0][i]*wR*wR),0.5));
                Npotr = G*v/(75*B_g[0][i]) + G*wR*(Cx[0][i]/Cy[0][i])/(100*B_g[0][i]) + G*Vy/(75*B_g[0][i]);
        }
        if (Vy<0)
                Vy = 0;
        return Vy;
}



float Impulse::Vy_vert2(int i)
{
        float Vy,dN,Nrasp,v,ro;
        ro = getRo(H[i]);
        Nrasp = n*getN(HNvzlet, VNvzlet, Nv, H[i],0);
        if (N[0][i]==-1)
        {
                dN = 0;
                Vy = 0;
        }
        else
        {
                dN = Nrasp - N[0][i];
                v = sqrt(G*9.81/(B_g[0][i]*ro*2*F));
                if (dN<0)
                        Vy = 0;
                else
                        Vy = B_g[0][i]*getKsi(0)*75*dN/G + v - (v*v/(B_g[0][i]*getKsi(0)*75*dN/G + v));
                        //по Шайдакову
                        /*
                        dN = dN/Nrasp;
                        ro = getRo(H[i])/getRo(H[0]);
                        Vy = 2*(dN + 2*eta0[i])*dN*sqrt(p/ro/B_g[0][i])/((eta0[i]+dN)*eta0[i]);
                        */
        }
        if (Vy<0)
                Vy = 0;

        return Vy;
}



float Impulse::EkSpeed(int i)
{
        float Ve, dN,Nmin;
        int k=0;
        //с учетом мощности двигателей
        int j_r = (mas_-1),j_l = 0;
        while (__ind[j_r][i]!=0)
        {
                if (j_r>0)
                        j_r--;
                else
                {
                        j_r = 0;
                        break;
                }
        }
        while (__ind[j_l][i]!=0)
        {
                if (j_l<(mas_-1))
                        j_l++;
                else
                {
                        j_l= (mas_-1);
                        break;
                }
        }
        if (j_l<=j_r)
        {
                Nmin = N[j_l][i];
                k = j_l;
                for (int j=j_l;j<=j_r;j++)
                {
                        if ((N[j][i]<Nmin))
                        {
                                Nmin = N[j][i];
                                k = j;
                        }
                }
                Ve = V[k];
                Nmin_[i] = N[k][i];
        }
        else
        {
                Ve= 0;
                Nmin_[i] = 0;
        }
        return Ve;
}



/*void __fastcall Impulse::Button1Click(TObject *Sender)
{
        int i=0,h;
        h = StrToFloat(ComboBox1->Text);
        i = h/10;

        Series1->Clear();
        Series2->Clear();
        Series3->Clear();
        Series4->Clear();
        Series12->Clear();
        Series14->Clear();
        Memo1->Clear();
        //Label3->Caption = " H= "+FloatToStr(h)+" м ";
        Chart1->Title->Text->Clear();
        Chart1->Title->Text->Add("Диаграмма потребных и располагаемых мощностей") ;
        Chart1->Title->Text->Add(" H= "+FloatToStr(h)+" м ") ;
        //Memo->Clear();
        int j_r = (mas_-1),j_l = 0;
        while (__ind[j_r][i]!=0)
        {
                if (j_r>0)
                        j_r--;
                else
                {
                        j_r = 0;
                        break;
                }
        }
        while (__ind[j_l][i]!=0)
        {
                if (j_l<(mas_-1))
                        j_l++;
                else
                {
                        j_l= (mas_-1);
                        break;
                }
        }
        if (j_l<=j_r)
        {

                Chart1->LeftAxis->Maximum = max(N_potr[j_r][i],N_potr[j_l][i])+0.2*max(N_potr[j_r][i],N_potr[j_l][i]);
                for (int j=j_l; j<=j_r;j++)
                {
                        Series1->AddXY(V[j],N[j][i],"");
                        if (N__[j][i]!=-1)
                                Series12->AddXY(V[j],N__[j][i],"");
                }
                //Chart1->LeftAxis->Maximum = AutoSize;
                Series14->AddXY(0,Nmin_[i],"");
                Series14->AddXY(Vek[i],Nmin_[i],"");
                Series14->AddXY(Vek[i],0,"");
        }
        for (int j=0; j<mas_;j++)  //параметры двигателей
        {
                Series2->AddXY(V[j],n*getN(HNvzlet, VNvzlet, Nv, H[i],V[j]),"");
                Series3->AddXY(V[j],n*getN(HNnom, VNnom, Nn, H[i],V[j]),"");
                Series4->AddXY(V[j],n*getN(HNk, VNk, Nk, H[i],V[j]),"");
        }
        if ((Vmax[i]!=0) && (Vmin[i]!=349))
        {
                Memo1->Lines->Add("Vmax= " + FloatToStr(Vmax[i])+" км/час");
                Memo1->Lines->Add("Vmin= " + FloatToStr(Vmin[i])+" км/час");
                Memo1->Lines->Add("Vek = " + FloatToStr(Vek[i])+" км/час");
                Memo1->Lines->Add("Vnaiv= " + FloatToStr(Vnaiv[i])+" км/час");
                Memo1->Lines->Add("Nmin= " + FloatToStrF(Nmin_[i] ,ffFixed,6,3)+" л.с.");
                Memo1->Lines->Add("Nnaiv= " + FloatToStrF(NNaiv[i] ,ffFixed,6,3)+" л.с.");
        }
        else
        {
                Memo1->Lines->Add("Выполнение полета невозможно");
                Memo1->Lines->Add("Vek = " + FloatToStr(Vek[i])+" км/час");
                Memo1->Lines->Add("Nmin= " + FloatToStrF(Nmin_[i] ,ffFixed,6,3)+" л.с.");
        }
        if (CBVildg->Checked == true)
                Series12->LinePen->Visible = true;
        else
                Series12->LinePen->Visible = false;
        ComboBox1 ->SetFocus();
}*/



/*void Impulse::DrawChart()
{
        Series5->Clear();
        Series6->Clear();
        Series7->Clear();
        Series8->Clear();
        Series9->Clear();
        Series10->Clear();
        Series11->Clear();
        Series13->Clear();
        Series15->Clear();
        Series16->Clear();
        Series17->Clear();
        //Series18->Clear();
        Chart2->LeftAxis->Maximum = Hdyn;

        for (int i=0; i<mas; i++)
        {
                for (int j=0; j<mas_;j++)
                {
                        if (sryv[j][i]!=0)
                        {
                                Series16->AddXY(V[j],H[i],"");
                                break;
                        }
                }
                for (int j=0; j<mas_;j++)
                {
                        if (sryv_Cy[j][i]!=0)
                        {
                                Series17->AddXY(V[j],H[i],"");
                                break;
                        }
                }
                if ((Vmin[i] != V[mas_-1]) )
                {
                        Series5->AddXY(Vmin[i],H[i],"");
                        Series13->AddXY(Vek[i],H[i],"");
                }
                if ((Vmax[i] != 0) )
                        Series6->AddXY(Vmax[i],H[i],"");
                if ((VminNom[i] != V[mas_-1]) )
                        Series9->AddXY(VminNom[i],H[i],"");
                if ((VmaxNom[i] != 0) )
                        Series10->AddXY(VmaxNom[i],H[i],"");
                if ((Vy_g[i] >= 0) && (Vy_g[i] != Vy_g[i-1]) && (i<i_))
                        Series7->AddXY(Vy_g[i],H[i],"");
                if ((Vy_g_nom[i] >= 0) && (Vy_g_nom[i] != Vy_g_nom[i-1]) && (i<i_))
                        Series15->AddXY(Vy_g_nom[i],H[i],"");
                if ((Vy_v[i] >= 0) && (Vy_v[i] != Vy_v[i-1]) && (i<i_))
                        Series8->AddXY(Vy_v[i],H[i],"");
                if ((Vy_g[i] >= 0) && (Vy_g[i] != Vy_g[i-1]))
                {
                        Chart2->LeftAxis->Maximum = H[i]+200;
                        Chart3->LeftAxis->Maximum = H[i]+200;
                        Chart3->BottomAxis->Maximum = Vy_g[0]+2;
                        Chart2->BottomAxis->Maximum = Vmax[0]+50;
                }
                if ((Vmax_sr[i] != (mas_-1)) )
                        Series11->AddXY(V_sryv_Ct[i],H[i],"");
        }
}*/



/*void Impulse::RezWrite()
{
        filewrite=ExtractFilePath(Application->ExeName)+"tmp.txt";
        Form1->SaveDialog1->FileName=filewrite;
        res=fopen(Form1->SaveDialog1->FileName.c_str(),"w");
        if(res!=NULL)
        {
                //исходные данные
                fprintf(res,"%s\n","Source_data -----------------------------------------");
                fprintf(res,"%s\n","Исходные данные");
                fprintf(res,"\n%s\t\t\t\t\t\t%s\t%0.3f\n","Диаметр НВ, м:","D_nv= ",D_nv);
                fprintf(res,"%s\t\t\t\t%s\t%0.1f\n","Взлетная масса вертолета, кг:","G= ",G);
                fprintf(res,"%s\t\t\t\t%s\t%0.0f\n","Количество лопастей НВ, шт.:","z= ",z);
                fprintf(res,"%s\t\t%s\t%0.3f\n","Хорда лопасти на относительном радиусе 0.7, м:","b_07= ",b_07);
                fprintf(res,"%s\t\t\t%s\t%0.1f\n","Окружная скорость концов лопастей, м/с:","wR= ",wR);

                fprintf(res,"%s\t%s\t%0.5f\n","Вредная относительная плоская пластинка (0.005...0.01):",  "Cx_vr= ",Cx_vr);

                //fprintf(res,"%s\n","Эмпирический коэффициент, учитывающий дополнительные потери мощности (в основном концевые потери и");
                //fprintf(res,"%s\t\t%s\t%0.3f\n","потери, обусловленные неравномерностью протекания)",  "k= ",k);
                //fprintf(res,"%s\t\t\t\t\t%s\t%0.3f\n","Общий шаг НВ, градусы:",  "fi0= ",fi_is);
                fprintf(res,"%s\t\t\t\t%s\t%0.3f\n","Наклон кривой подъемной силы:",  "a= ",a);
                fprintf(res,"%s\t\t\t\t%s\t%0.3f\n","Коэффициент компенсатора взмаха:",  "_k= ",_k);
                fprintf(res,"%s\t\t%s\t%0.3f\n","Момент инерции лопасти относительно ГШ, кг*м*с^2:",  "Igs= ",Igs);
                fprintf(res,"%s\t\t\t\t%s\t%0.3f\n","Коэффициент концевых потерь (xi):",  "B= ",B);
                fprintf(res,"%s\t\t\t%s\t%0.3f\n","Геометрическая крутка лопасти, градусы:",  "dfi= ",dfi);


                fprintf(res,"\n%s\n","ПАРАМЕТРЫ ОКРУЖАЮЩЕЙ СРЕДЫ:");
                fprintf(res,"%s\t%s%0.3f\n","Температура воздуха на  уровне моря,°С:","t= ",t);
                fprintf(res,"%s\t%s%0.3f\n","Атмосферное давление на уровне моря, Па:","p= ",p_);




                fprintf(res,"\n\n%s\n","ПАРАМЕТРЫ ПРОФИЛЯ НА ОТНОСИТЕЛЬНОМ РАДИУСЕ 0.7");
                fprintf(res,"\n%s\n","a - угол атаки, градусы;   М - число Маха");
                fprintf(res,"%s\n","a_sr - угол атаки, при котором начинается увеличение профильного сопротивления. (для каждого М)");
                //fprintf(res,"%s\t","M\\A");

                for (int i=0; i<=5;i++)
                        fprintf(res,"%s%0.1f\t\t\t","M"+IntToStr(i+1)+"= ",MP[i]);
                fprintf(res,"\n");
                for (int i=0; i<=5;i++)
                        fprintf(res,"%s%0.1f\t\t","a_sr"+IntToStr(i+1)+"= ",a_sr[i]);
                fprintf(res,"\n");
                for (int i=1; i<=9;i++)
                        fprintf(res,"%s%0.1f\t\t","a"+IntToStr(i)+"= ",alfaP[i]);
                fprintf(res,"\n");
                fprintf(res,"\n%s\n","Cy:");
                fprintf(res,"%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t\n","M\\a","a1","a2","a3","a4","a5","a6","a7","a8","a9");
                for (int i=0; i<=5;i++)
                {
                        fprintf(res,"%s\t","M"+IntToStr(i+1));
                        for (int j=1; j<=9;j++)
                                fprintf(res,"%0.4f\t",CYp[i][j]);
                        fprintf(res,"\n");
                }
                fprintf(res,"\n%s\n","Cx:");
                fprintf(res,"%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t\n","M\\a","a1","a2","a3","a4","a5","a6","a7","a8","a9");
                for (int i=0; i<=5;i++)
                {
                        fprintf(res,"%s\t","`M"+IntToStr(i+1));
                        for (int j=1; j<=9;j++)
                                fprintf(res,"%0.4f\t",CXp[i][j]);
                        fprintf(res,"\n");
                }
                fprintf(res,"\n%s\t%s\t%0.4f\n","Максимальный коэффициент подъемной силы профиля с учетом нестационарности обтекания:","Cy_max_pr=",Cy_MAX);
                //fprintf(res,"\n%s\t\t%s\t%0.3f\n","Коэффициент использования мощности (без учета РВ):","ksi= ",ksi);
                //fprintf(res,"%s\t\t\t\t%s\t%0.3f\n","Относительный радиус втулки :",  "e1= ",e1);
                fprintf(res,"%s\t\t%s\t%0.3f\n","Коэффициент обдувки частей вертолета (0.01...0.02):",  "t_obd= ",t_obd);
                fprintf(res,"\n%s\t\t\t\t\t\t%s\t%0.3f\n","Диаметр РВ, м:","D_rv= ",D_rv);
                fprintf(res,"%s\t\t%s\t%0.3f\n","Хорда лопасти РВ на относительном радиусе 0.7, м:","b_rv= ",b_rv);
                fprintf(res,"%s\t\t\t\t%s\t%0.0f\n","Количество лопастей РВ, шт.:","z_rv= ",z_rv);
                fprintf(res,"\n%s\n",                 "ПАРАМЕТРЫ ДВИГАТЕЛЕЙ:");
                fprintf(res,"%s%0.0f\n","Количество двигателей, шт.:  n= ",n);
                fprintf(res,"%s\n",                   "Параметры одного двигателя:");
                fprintf(res,"\n%s\n","Взлетная мощность, л.с.:");
                fprintf(res,"%s","V\\H\t\t");
                for (int i=0; i < HNvzlet.size();i++)
                        fprintf(res,"\t%0.1f",HNvzlet[i]);
                for (int i=0; i < VNvzlet.size();i++)
                {
                        fprintf(res,"\n%s%d%s","V",i+1,"=  ");
                        fprintf(res,"\t%0.1f\t\t",VNvzlet.at(i));
                        for (int j=0; j<Nv.at(i).size();j++)
                                fprintf(res,"%0.1f\t",Nv.at(i).at(j));
                }
                fprintf(res,"\n\n%s\n","Номинальная мощность, л.с.:");
                fprintf(res,"%s","`V\\H\t\t");
                for (int i=0; i < HNnom.size();i++)
                        fprintf(res,"\t%0.1f",HNnom[i]);
                for (int i=0; i < VNnom.size();i++)
                {
                        fprintf(res,"\n%s%d%s","`V",i+1,"=  ");
                        fprintf(res,"\t%0.1f\t\t",VNnom.at(i));
                        for (int j=0; j<Nn.at(i).size();j++)
                                fprintf(res,"%0.1f\t",Nn.at(i).at(j));
                }

                fprintf(res,"\n\n%s\n","Крейсерская мощность, л.с.:");
                fprintf(res,"%s","`V\\H`\t\t");
                for (int i=0; i < HNk.size();i++)
                        fprintf(res,"\t%0.1f",HNk[i]);
                for (int i=0; i < VNk.size();i++)
                {
                        fprintf(res,"\n%s%d%s","`V",i+1,"`=  ");
                        fprintf(res,"\t%0.1f\t\t",VNk.at(i));
                        for (int j=0; j<Nk.at(i).size();j++)
                                fprintf(res,"%0.1f\t",Nk.at(i).at(j));
                }
                fprintf(res,"\n\n%s\n","Коэффициент использования мощности:");
                fprintf(res,"%s","V_Ksi\t");
                for (int i=0; i < V_Ksi.size();i++)
                        fprintf(res,"\t%0.0f",V_Ksi[i]);
                fprintf(res,"\n%s","Ksi\t");
                for (int i=0; i < Ksi.size();i++)
                        fprintf(res,"\t%0.3f",Ksi[i]);
                fprintf(res,"\n\n%s\n","note");

                for (int i=0; i<=9;i++)
                {
                        if (strcmp(prim[i],"\n")!=0)
                                fprintf(res,"%s",prim[i]);
                }
                fprintf(res,"\n%s\n","end_note");

                //fprintf(res,"%s\n",prim);

                fprintf(res,"\n%s\n","End_source_data -------------------------------------");

                fprintf(res,"\n\n%s\n","Результат -------------------------------------------");
                fprintf(res,"\n%s\t%s\n","Время расчета:",DateTimeToStr(Now()));
                fprintf(res,"\n\n%s"," ЛТХ");
                fprintf(res,"\n%s","Характеристики вертолета с использованием ВЗЛЕТНОЙ мощности:");
                fprintf(res,"\n%s\t%0.0f\n","Практический статический потолок, м:",Hst);
                fprintf(res,"%s\t%0.0f\n","Практический статический потолок (с учетом срыва потока на лопастях), м:",Hst_sryv);
                fprintf(res,"%s\t%0.0f\n",  "Практический динамический потолок, м:",Hdyn);
                fprintf(res,"%s\t%0.0f\n","Практический динамический потолок (с учетом срыва потока на лопастях), м:",Hdyn_sryv);
                fprintf(res,"%s\t%0.0f\n","Максимальная скорость на высоте 500м , км/час:",Vmax[50]);
                fprintf(res,"%s\t%0.0f\n","Максимальная скорость на высоте 500м (с учетом срыва потока на лопастях), км/час:",V500v);
                fprintf(res,"\n%s","Характеристики вертолета с использованием НОМИНАЛЬНОЙ (максимальной продолжительной) мощности:");
                fprintf(res,"\n%s\t%0.0f\n",  "Практический динамический потолок, м:",Hdyn_nom);
                fprintf(res,"%s\t%0.0f\n","Практический динамический потолок (с учетом срыва потока на лопастях), м:",Hdyn_sryv_nom);
                fprintf(res,"%s\t%0.0f\n","Максимальная скорость на высоте 500м , км/час:",VmaxNom[50]);
                fprintf(res,"%s\t%0.0f\n","Максимальная скорость на высоте 500м (с учетом срыва потока на лопастях), км/час:",V500n);
                fprintf(res,"\n%s\t%0.3f\n","Удельная нагрузка р, Н/м^2:",p);
                fprintf(res,"%s\t%0.3f\n","Коэффициент заполнения sigma:",sigma);
                fprintf(res,"\n\n%s","Аэродинамический паспорт");
                fprintf(res,"\n%s\n","Н, м          - высота полета");
                fprintf(res,"%s\n","Vmin, км/час  - минимальная скорость полета на заданной высоте на взлетной мощности");
                fprintf(res,"%s\n","Vmax, км/час  - максимальная скорость полета на заданной высоте на взлетной мощности");
                fprintf(res,"%s\n","Vmin_n, км/час  - минимальная скорость полета на заданной высоте на номинальной мощности");
                fprintf(res,"%s\n","Vmax_n, км/час  - максимальная скорость полета на заданной высоте на номинальной мощности");
                fprintf(res,"%s\n","Vek, км/час   - экономическая скорость полета на заданной высоте");
                fprintf(res,"%s\n","Vnaiv, км/час - наивыгоднейшая скорость полета на заданной высоте");
                //fprintf(res,"%s\n","    *Примечание: -если минимальная скорость равна 349, то выполнение полета на заданнной высоте на соответствующей мощности невозможно.");
                fprintf(res,"%s\n","Vy_верт, м/с - скороподъемность при вертикальном полете");
                fprintf(res,"%s\n","Vу_гор, м/с - скороподъемность при горизонтальном полете (c использованием взлетной мощности)");
                fprintf(res,"%s\n","Vу_гор_n, м/с - скороподъемность при горизонтальном полете (c использованием номинальной мощности)");
                fprintf(res,"%s\n","Vср_Ct, км/час - минимальная скорость, при которой происходит срыв потока по Ct");
                fprintf(res,"%s\n","Vср_a, км/час - минимальная скорость, при которой происходит срыв потока по критическому углу атаки ");
                fprintf(res,"%s\n","Vср_Cy, км/час - минимальная скорость, при которой происходит срыв потока по Cy");
                fprintf(res,"%s\n","Nmin, л.с. - минимальное значение потребной мощности на заданной высоте");
                fprintf(res,"%s\n","eta0 - относительный КПД несущего винта на режиме висения на заданной высоте");
                fprintf(res,"%s\n","Ns.p., л.с - мощность, потребная для висения на статическом потолке (скороподъемность 0.5 м/с)");
                fprintf(res,"%s\n\n","Nvzl, л.с - располагаемая взлетная мощность силовой установки");
                fprintf(res,"%s\n\n","Nnaiv, л.с - наивыгоднейшая мощность");
                fprintf(res,"%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t\t%s\t%s\n","H","Vmin","Vmax","Vmin_n","Vmax_n","Vek","Vnaiv","Vy_верт","Vу_гор","Vу_гор_n","Vср_Ct","Nmin","eta0","Ns.p.","Nvzl","Nnaiv");


                String _H;
                String _Vmin;
                String _Vmax;
                String _VminNom;
                String _VmaxNom;
                String _Vek;
                String _Vnaiv;
                String _Vy_v;
                String _Vy_g;
                String _Vy_g_nom;
                String _Vmax_sr;
                String _Vsr_a;
                String _Vsr_Cy;
                String _Nmin_;
                String _eta0;
                String _Nsp;
                String _Nvzl;
                String _Nnaiv;
                for (int i=0; i<mas; i++)
                {
                        if (Vy_g[i]>0)
                        {

                                _H    =  FloatToStrF(H[i]    ,ffFixed,7,0);
                                _Vmin =  FloatToStrF(Vmin[i] ,ffFixed,7,0);
                                _Vmax =  FloatToStrF(Vmax[i] ,ffFixed,7,0);
                                _VminNom =  FloatToStrF(VminNom[i] ,ffFixed,7,0);
                                if (VminNom[i]==(mas_-1))
                                        _VminNom = "-";
                                _VmaxNom =  FloatToStrF(VmaxNom[i] ,ffFixed,7,0);
                                if (VmaxNom[i]==0)
                                        _VmaxNom = "-";
                                _Vek =  FloatToStrF(Vek[i] ,ffFixed,7,0);
                                _Vnaiv =  FloatToStrF(Vnaiv[i] ,ffFixed,7,0);
                                _Vy_v =  FloatToStrF(Vy_v[i] ,ffFixed,7,3);
                                _Vy_g =  FloatToStrF(Vy_g[i] ,ffFixed,7,3);
                                _Vy_g_nom =  FloatToStrF(Vy_g_nom[i] ,ffFixed,7,3);
                                _Vmax_sr =  FloatToStrF(V_sryv_Ct[i] ,ffFixed,7,0);
                                _Vsr_a =  FloatToStrF(Vsr_a[i] ,ffFixed,7,0);
                                _Vsr_Cy =  FloatToStrF(Vsr_Cy[i] ,ffFixed,7,0);
                                if (Vmax_sr[i]==(mas_-1))
                                        _Vmax_sr = "-";
                                if (Vsr_a[i]==(mas_-1))
                                        _Vsr_a = "-";
                                if (Vsr_Cy[i]==(mas_-1))
                                        _Vsr_Cy = "-";
                                _Nmin_ =  FloatToStrF(Nmin_[i] ,ffFixed,7,3);
                                if (Vy_v[i]>0)
                                        _eta0 =  FloatToStrF(eta0[i] ,ffFixed,7,4);
                                else
                                        _eta0 = "-";
                                _Nsp =  FloatToStrF(Nsp[i] ,ffFixed,7,3);
                                _Nvzl =  FloatToStrF(n*getN(HNvzlet, VNvzlet, Nv, H[i],0) ,ffFixed,7,2);
                                _Nnaiv =  FloatToStrF(NNaiv[i] ,ffFixed,7,2);

                                fprintf(res,"%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t\t%s\t%s\n",
                                        _H,
                                        _Vmin,
                                        _Vmax,
                                        _VminNom,
                                        _VmaxNom,
                                        _Vek,
                                        _Vnaiv,
                                        _Vy_v,
                                        _Vy_g,
                                        _Vy_g_nom,
                                        _Vmax_sr,


                                        _Nmin_,
                                        _eta0,
                                        _Nsp,
                                        _Nvzl,
                                        _Nnaiv
                                        );



                                 if ((i==50) || (i==100) || (i==150) || (i==200) || (i==250) || (i==300) || (i==350) || (i==400) || (i==450) ||
                                     (i==500) || (i==550) || (i==600) || (i==650) || (i==700) || (i==750)  )
                                        fprintf(res,"%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t\t%s\t%s\n","H","Vmin","Vmax","Vmin_n","Vmax_n","Vek","Vnaiv","Vy_верт","Vу_гор","Vу_гор_n","Vср_Ct","Nmin","eta0","Ns.p.","Nvzl","Nnaiv");

                        }
                }
                fprintf(res,"\n\n%s\n","Коэффициент тяги НВ Ct  ");
                fprintf(res,"%s","Н\\V");
                for (int j=0; j<mas_; j=j+10)
                        fprintf(res,"\t%0.0f",V[j]);
                fprintf(res,"%s\n","");
                for (int i=0; i<mas; i=i+5)
                {
                        fprintf(res,"%0.0f\t",H[i]);
                        for (int j=0; j<mas_; j=j+10)
                        {
                                fprintf(res,"%0.5f\t",Ct[j][i]);

                        }
                        fprintf(res,"%s\n","");
                }
                fprintf(res,"\n\n%s\n","Тяга несущего винта, кг");
                fprintf(res,"%s","Н\\V");
                for (int j=0; j<mas_; j=j+10)
                        fprintf(res,"\t%0.0f",V[j]);
                fprintf(res,"%s\n","");
                for (int i=0; i<mas; i=i+5)
                {
                        fprintf(res,"%0.0f\t",H[i]);
                        for (int j=0; j<mas_; j=j+10)
                        {
                                fprintf(res,"%0.2f\t",T_g[j][i]/g);

                        }
                        fprintf(res,"%s\n","");
                }
                fprintf(res,"\n\n%s\n","Вредное лобовое сопротивление, Н");
                fprintf(res,"%s","Н\\V");
                for (int j=0; j<mas_; j=j+10)
                        fprintf(res,"\t%0.0f",V[j]);
                fprintf(res,"%s\n","");
                for (int i=0; i<mas; i=i+5)
                {
                        fprintf(res,"%0.0f\t",H[i]);
                        for (int j=0; j<mas_; j=j+10)
                        {
                                fprintf(res,"%0.2f\t",Xvr[j][i]);

                        }
                        fprintf(res,"%s\n","");
                }
                fprintf(res,"\n\n%s\n","Индуктивная скорость, м/с");
                fprintf(res,"%s","Н\\V");
                for (int j=0; j<mas_; j=j+10)
                        fprintf(res,"\t%0.0f",V[j]);
                fprintf(res,"%s\n","");
                for (int i=0; i<mas; i=i+5)
                {
                        fprintf(res,"%0.0f\t",H[i]);
                        for (int j=0; j<mas_; j=j+10)
                        {
                                fprintf(res,"%0.4f\t",Vi_g[j][i]);

                        }
                        fprintf(res,"%s\n","");
                }
                fprintf(res,"\n\n%s\n","Потребный коэффициент подъемной силы");
                fprintf(res,"%s","Н\\V");
                for (int j=0; j<mas_; j=j+10)
                        fprintf(res,"\t%0.0f",V[j]);
                fprintf(res,"%s\n","");
                for (int i=0; i<mas; i=i+5)
                {
                        fprintf(res,"%0.0f\t",H[i]);
                        for (int j=0; j<mas_; j=j+10)
                        {
                                fprintf(res,"%0.4f\t",Cy[j][i]);

                        }
                        fprintf(res,"%s\n","");
                }
                fprintf(res,"\n\n%s\n","Коэффициент профильного сопротивления");
                fprintf(res,"%s","Н\\V");
                for (int j=0; j<mas_; j=j+10)
                        fprintf(res,"\t%0.0f",V[j]);
                fprintf(res,"%s\n","");
                for (int i=0; i<mas; i=i+5)
                {
                        fprintf(res,"%0.0f\t",H[i]);
                        for (int j=0; j<mas_; j=j+10)
                        {
                                fprintf(res,"%0.4f\t",Cx[j][i]);

                        }
                        fprintf(res,"%s\n","");
                }
                fprintf(res,"\n\n%s\n","Коэффициент мощности НВ, х10^(-3)");
                fprintf(res,"%s","Н\\V");
                for (int j=0; j<mas_; j=j+10)
                        fprintf(res,"\t%0.0f",V[j]);
                fprintf(res,"%s\n","");
                for (int i=0; i<mas; i=i+5)
                {
                        fprintf(res,"%0.0f\t",H[i]);
                        for (int j=0; j<mas_; j=j+10)
                        {
                                fprintf(res,"%0.5f\t",mk[j][i]*1000);

                        }
                        fprintf(res,"%s\n","");
                }
                fprintf(res,"\n\n%s\n","Индуктивная мощность НВ, л.с");
                //fprintf(res,"%s\n","    *Примечание: если потребная мощность равна -1, то в заданных условиях потребная тяга не может быть получена несущим винтом с указанными параметрами.");
                fprintf(res,"%s","Н\\V");
                for (int j=0; j<mas_; j=j+10)
                        fprintf(res,"\t%0.0f",V[j]);
                fprintf(res,"%s\n","");
                for (int i=0; i<mas; i=i+5)
                {
                        fprintf(res,"%0.0f\t",H[i]);

                        for (int j=0; j<mas_; j=j+10)
                        {
                                fprintf(res,"%0.2f\t",N_i[j][i]);

                        }
                        fprintf(res,"%s\n","");
                }
                fprintf(res,"\n\n%s\n","Профильная мощность НВ, л.с");
                //fprintf(res,"%s\n","    *Примечание: если потребная мощность равна -1, то в заданных условиях потребная тяга не может быть получена несущим винтом с указанными параметрами.");
                fprintf(res,"%s","Н\\V");
                for (int j=0; j<mas_; j=j+10)
                        fprintf(res,"\t%0.0f",V[j]);
                fprintf(res,"%s\n","");
                for (int i=0; i<mas; i=i+5)
                {
                        fprintf(res,"%0.0f\t",H[i]);

                        for (int j=0; j<mas_; j=j+10)
                        {
                                fprintf(res,"%0.2f\t",N_p[j][i]);

                        }
                        fprintf(res,"%s\n","");
                }

                fprintf(res,"\n\n%s\n","Пропульсивная мощность, л.с");
                //fprintf(res,"%s\n","    *Примечание: если потребная мощность равна -1, то в заданных условиях потребная тяга не может быть получена несущим винтом с указанными параметрами.");
                fprintf(res,"%s","Н\\V");
                for (int j=0; j<mas_; j=j+10)
                        fprintf(res,"\t%0.0f",V[j]);
                fprintf(res,"%s\n","");
                for (int i=0; i<mas; i=i+5)
                {
                        fprintf(res,"%0.0f\t",H[i]);

                        for (int j=0; j<mas_; j=j+10)
                        {
                                fprintf(res,"%0.2f\t",N_prop[j][i]);

                        }
                        fprintf(res,"%s\n","");
                }
                fprintf(res,"\n\n%s\n","Суммарная потребная мощность, л.с");
                //fprintf(res,"%s\n","    *Примечание: если потребная мощность равна -1, то в заданных условиях потребная тяга не может быть получена несущим винтом с указанными параметрами.");
                fprintf(res,"%s","Н\\V");
                for (int j=0; j<mas_; j=j+10)
                        fprintf(res,"\t%0.0f",V[j]);
                fprintf(res,"%s\n","");
                for (int i=0; i<mas; i=i+5)
                {
                        fprintf(res,"%0.0f\t",H[i]);

                        for (int j=0; j<mas_; j=j+10)
                        {
                                fprintf(res,"%0.2f\t",N[j][i]);

                        }
                        fprintf(res,"%s\n","");
                }
                fprintf(res,"\n\n%s\n","Потребная мощность НВ, л.с");
                //fprintf(res,"%s\n","    *Примечание: если потребная мощность равна -1, то в заданных условиях потребная тяга не может быть получена несущим винтом с указанными параметрами.");
                fprintf(res,"%s","Н\\V");
                for (int j=0; j<mas_; j=j+10)
                        fprintf(res,"\t%0.0f",V[j]);
                fprintf(res,"%s\n","");
                for (int i=0; i<mas; i=i+5)
                {
                        fprintf(res,"%0.0f\t",H[i]);

                        for (int j=0; j<mas_; j=j+10)
                        {
                                fprintf(res,"%0.2f\t",N_nv[j][i]);

                        }
                        fprintf(res,"%s\n","");
                }
                fprintf(res,"\n\n%s\n","Коэффициент режима работы мю");
                fprintf(res,"%s","Н\\V");
                for (int j=0; j<mas_; j=j+10)
                        fprintf(res,"\t%0.0f",V[j]);
                fprintf(res,"%s\n","");
                for (int i=0; i<mas; i=i+5)
                {
                        fprintf(res,"%0.0f\t",H[i]);
                        for (int j=0; j<mas_; j=j+10)
                        {
                                fprintf(res,"%0.4f\t",mu[j][i]);

                        }
                        fprintf(res,"%s\n","");
                }
                fprintf(res,"\n\n%s\n","Потери мощности на привод РВ , %");
                fprintf(res,"%s","Н\\V");
                for (int j=0; j<mas_; j=j+10)
                        fprintf(res,"\t%0.0f",V[j]);
                fprintf(res,"%s\n","");
                for (int i=0; i<mas; i=i+5)
                {
                        fprintf(res,"%0.0f\t",H[i]);
                        for (int j=0; j<mas_; j=j+10)
                        {
                                fprintf(res,"%0.4f\t",ksi_rv[j][i]);

                        }
                        fprintf(res,"%s\n","");
                }
                fprintf(res,"\n\n%s\n","Угол ататки НВ, градусы ");
                fprintf(res,"%s","Н\\V");
                for (int j=0; j<mas_; j=j+10)
                        fprintf(res,"\t%0.0f",V[j]);
                fprintf(res,"%s\n","");
                for (int i=0; i<mas; i=i+5)
                {
                        fprintf(res,"%0.0f\t",H[i]);
                        for (int j=0; j<mas_; j=j+10)
                        {
                                fprintf(res,"%0.3f\t",(alfa_k[j][i]*57.2957));

                        }
                        fprintf(res,"%s\n","");
                }
                fprintf(res,"\n\n%s\n","Общий шаг НВ, градусы ");
                fprintf(res,"%s","Н\\V");
                for (int j=0; j<mas_; j=j+10)
                        fprintf(res,"\t%0.0f",V[j]);
                fprintf(res,"%s\n","");
                for (int i=0; i<mas; i=i+5)
                {
                        fprintf(res,"%0.0f\t",H[i]);
                        for (int j=0; j<mas_; j=j+10)
                        {
                                fprintf(res,"%0.3f\t",(fi0[j][i]*57.2957));

                        }
                        fprintf(res,"%s\n","");
                }
                fprintf(res,"\n\n%s\n","Эффективный общий шаг НВ, градусы ");
                fprintf(res,"%s","Н\\V");
                for (int j=0; j<mas_; j=j+10)
                        fprintf(res,"\t%0.0f",V[j]);
                fprintf(res,"%s\n","");
                for (int i=0; i<mas; i=i+5)
                {
                        fprintf(res,"%0.0f\t",H[i]);
                        for (int j=0; j<mas_; j=j+10)
                        {
                                fprintf(res,"%0.3f\t",(fi0_e[j][i]*57.2957));

                        }
                        fprintf(res,"%s\n","");
                }
                fprintf(res,"\n\n%s\n","Общий шаг НВ по Вильдгрубе, градусы ");
                fprintf(res,"%s","Н\\V");
                for (int j=0; j<mas_; j=j+10)
                        fprintf(res,"\t%0.0f",V[j]);
                fprintf(res,"%s\n","");
                for (int i=0; i<mas; i=i+5)
                {
                        fprintf(res,"%0.0f\t",H[i]);
                        for (int j=0; j<mas_; j=j+10)
                        {
                                fprintf(res,"%0.3f\t",(fi0_v[j][i]*57.2957));

                        }
                        fprintf(res,"%s\n","");
                }
                fprintf(res,"\n\n%s\n","Коэффициент притекания");
                fprintf(res,"%s","Н\\V");
                for (int j=0; j<mas_; j=j+10)
                        fprintf(res,"\t%0.0f",V[j]);
                fprintf(res,"%s\n","");
                for (int i=0; i<mas; i=i+5)
                {
                        fprintf(res,"%0.0f\t",H[i]);
                        for (int j=0; j<mas_; j=j+10)
                        {
                                fprintf(res,"%0.3f\t",lambda[j][i]);

                        }
                        fprintf(res,"%s\n","");
                }
                fprintf(res,"\n\n%s\n","Коэффициент индукции НВ");
                fprintf(res,"%s","Н\\V");
                for (int j=0; j<mas_; j=j+10)
                        fprintf(res,"\t%0.0f",V[j]);
                fprintf(res,"%s\n","");
                for (int j=0; j<mas_; j=j+10)
                        fprintf(res,"\t%0.3f",I[j]);

                fprintf(res,"\n\n%s\n","Суммарная потребная мощность Nпотр (л.с.) и общий шаг НВ (градусы) (сокращенно)");
                //fprintf(res,"%s\n","    *Примечание: если потребная мощность равна -1, то в заданных условиях потребная тяга не может быть получена несущим винтом с указанными параметрами.");
                fprintf(res,"%s","Н\\V");
                for (int j=0; j<mas_; j=j+5)
                        fprintf(res,"\t%0.0f",V[j]);
                fprintf(res,"%s\n","");
                for (int i=0; i<mas; i=i+5)
                {
                        if ((H[i]==0) || (H[i]==4000) || (H[i]==6000))
                        {
                                fprintf(res,"%0.0f\t",H[i]);

                                for (int j=0; j<mas_; j=j+5)
                                {
                                        fprintf(res,"%0.2f\t",N[j][i]);

                                }
                                fprintf(res,"\n%0.0f\t",H[i]);

                                for (int j=0; j<mas_; j=j+5)
                                {
                                        fprintf(res,"%0.2f\t",(fi0[j][i]*57.2957));

                                }
                                fprintf(res,"\n%0.0f\t",H[i]);
                                for (int j=0; j<mas_; j=j+5)
                                {
                                        fprintf(res,"%0.2f\t",(fi0_j[j][i]*57.2957));

                                }
                                fprintf(res,"%s\n\n","");
                        }
                }
                fprintf(res,"\n\n%s\n","Общий шаг НВ, градусы (сокращенно)");
                //fprintf(res,"%s\n","    *Примечание: если потребная мощность равна -1, то в заданных условиях потребная тяга не может быть получена несущим винтом с указанными параметрами.");
                fprintf(res,"%s","Н\\V");
                for (int j=0; j<mas_; j=j+5)
                        fprintf(res,"\t%0.0f",V[j]);
                fprintf(res,"%s\n","");
                for (int i=0; i<mas; i=i+5)
                {
                        if ((H[i]==0) || (H[i]==4000) || (H[i]==6000))
                        {
                                fprintf(res,"%0.0f\t",H[i]);

                                for (int j=0; j<mas_; j=j+5)
                                {
                                        fprintf(res,"%0.2f\t",(fi0[j][i]*57.2957));

                                }
                                fprintf(res,"%s\n","");
                        }
                }
                fprintf(res,"\n\n%s\n","Потребная мощность (спец.), л.с");
                //fprintf(res,"%s\n","    *Примечание: если потребная мощность равна -1, то в заданных условиях потребная тяга не может быть получена несущим винтом с указанными параметрами.");
                fprintf(res,"%s","Н\\V");
                for (int j=70; j<230; j=j+10)
                        fprintf(res,"\t%0.2f",V[j]);
                fprintf(res,"%s\n","");
                for (int i=0; i<mas; i=i+5)
                {
                        //if ((H[i]==0) || (H[i]==1000) || (H[i]==2000))
                        if ((H[i]==500) || (H[i]==1000) || (H[i]==2000))
                        {
                                fprintf(res,"%0.0f\t",H[i]);

                                for (int j=70; j<230; j=j+10)
                                {
                                        fprintf(res,"%0.1f\t",N[j][i]);

                                }
                                fprintf(res,"%s\n","");
                                fprintf(res,"%s\t","q");

                                for (int j=70; j<230; j=j+10)
                                {
                                        fprintf(res,"%0.3f\t",q[j][i]);

                                }
                                fprintf(res,"%s\n","");
                                 fprintf(res,"%s\t","Ce");

                                for (int j=70; j<230; j=j+10)
                                {
                                        fprintf(res,"%0.3f\t",Ce[j][i]);

                                }
                                fprintf(res,"%s\n","");
                        }
                }
        }
        fclose(res);
        Memo->Lines->Clear();
        Memo->Lines->LoadFromFile(filewrite);
}
*/


