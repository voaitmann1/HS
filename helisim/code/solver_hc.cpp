#include <QCoreApplication>
#include "helicopter.h"


//Интерполяция.сплайн y=f(x)
double Helicopter::SplineXY(double xi, double x_arr[], double y_arr[], int n )
{
    double yi=0;
    if ( xi<x_arr[0] || xi>x_arr[n-1] )
    {
        if( xi<x_arr[0] )
        {
            solvout += "SplineXY() overlimit:  x=" + ss.setNum(xi,'f',3);   solvout += ", xmin=" + ss.setNum(x_arr[0],'f',3) + "\n";
            xi = x_arr[0];
        }
        if( xi>x_arr[n-1] )
        {
            solvout += "SplineXY() overlimit:  x=" + ss.setNum(xi,'f',3);   solvout += ", xmax=" + ss.setNum(x_arr[n-1],'f',3) + "\n";
            xi = x_arr[n-1];
        }
    }
    gsl_set_error_handler_off();//выключает крах программы при ошибках
    gsl_interp_accel *acc = gsl_interp_accel_alloc (); //Эта функция возвращает указатель на объект ускорителя, который является своего рода итератора для интерполяции выборок.
    const gsl_interp_type *t = gsl_interp_cspline;//polynomial; //указатель на структуру с определяющую тип интерполяции
    gsl_spline *spline = gsl_spline_alloc (t, n);     //создается непосредственно обьект представляющий сплайн (пока что пустой и не инициализированный)
    gsl_spline_init (spline, x_arr, y_arr, n);               //инициализируем пустой обьект сплайна нашими известными данными
    yi = gsl_spline_eval (spline, xi, acc);
    gsl_spline_free (spline);    //удаляем обьект сплайна
    gsl_interp_accel_free (acc); //удаляем акселератор
return yi;
}



//Аппроксимация y=f(x) !!! совпадает с SplineXY !!!s
double Helicopter::RegressXY(int sp, double xi, double x_arr[], double y_arr[], int n )
{
    gsl_set_error_handler_off();//выключает крах программы при ошибках
    if ( xi<x_arr[0] || xi>x_arr[n-1] )
    {
        if( xi<x_arr[0] )
        {
            solvout += "RegressXY() overlimit:  x=" + ss.setNum(xi,'f',3);   solvout += ", xmin=" + ss.setNum(x_arr[0],'f',3) + "\n";
            xi = x_arr[0];
        }
        if( xi>x_arr[n-1] )
        {
            solvout += "RegressXY() overlimit:  x=" + ss.setNum(xi,'f',3);   solvout += ", xmax=" + ss.setNum(x_arr[n-1],'f',3) + "\n";
            xi = x_arr[n-1];
        }
    }
    gsl_interp_accel *acc = gsl_interp_accel_alloc (); //Эта функция возвращает указатель на объект ускорителя, который является своего рода итератора для интерполяции выборок.
    const gsl_interp_type *t = gsl_interp_cspline;//polynomial; //указатель на структуру с определяющую тип интерполяции
    gsl_spline *spline = gsl_spline_alloc (t, n);     //создается непосредственно обьект представляющий сплайн (пока что пустой и не инициализированный)

    gsl_spline_init (spline, x_arr, y_arr, n);               //инициализируем пустой обьект сплайна нашими известными данными
    double yi = gsl_spline_eval (spline, xi, acc);

    gsl_spline_free (spline);    //удаляем обьект сплайна
    gsl_interp_accel_free (acc); //удаляем акселератор
return yi;
}



//Интерполяция.сплайн z=f(x,y)
double Helicopter::SplineXYZ(double xi, double yi,  double x_ar[],double y_ar[],double z_ar[],  int nx, int ny )//
{
    gsl_set_error_handler_off();//выключает крах программы при ошибках
    double zi=0;
    if ( xi<x_ar[0] || xi>x_ar[nx-1] || yi<y_ar[0] || yi>y_ar[ny-1] )
    {
        if( xi<x_ar[0] )
        {
            solvout += "SplineXYZ() overlimit:  x=" + ss.setNum(xi,'f',3);   solvout += ", xmin=" + ss.setNum(x_ar[0],'f',3) + "\n";
            xi = x_ar[0];
        }
        if( xi>x_ar[nx-1] )
        {
            solvout += "SplineXYZ() overlimit:  x=" + ss.setNum(xi,'f',3);   solvout += ", xmax=" + ss.setNum(x_ar[nx-1],'f',3) + "\n";
            xi = x_ar[nx-1];
        }
        if( yi<y_ar[0] )
        {
            solvout += "SplineXYZ() overlimit:  y=" + ss.setNum(yi,'f',3);   solvout += ", ymin=" + ss.setNum(y_ar[0],'f',3) + "\n";
            yi = y_ar[0];
        }
        if( yi>y_ar[ny-1] )
        {
            solvout += "SplineXYZ() overlimit:  y=" + ss.setNum(yi,'f',3);   solvout += ", ymax=" + ss.setNum(y_ar[ny-1],'f',3) + "\n";
            yi = y_ar[ny-1];
        }
    }
    const gsl_interp2d_type *T = gsl_interp2d_bilinear;//bicubic;// //тип интерполяции. ВНИМАНИЕ: gsl_interp2d_bicubic требует сетку минимум 4х4
    gsl_spline2d *spline   = gsl_spline2d_alloc(T, nx, ny);//обьявление указателя на обьект сплафна и его инициализация
    gsl_interp_accel *xacc = gsl_interp_accel_alloc();   //акселераторы для осей Х и У
    gsl_interp_accel *yacc = gsl_interp_accel_alloc();
    double *za = (double*)malloc(nx * ny * sizeof(double)); //выделение памяти под массив z, размером nx*ny для элементов double
    int i, j, k;
    for (i = 0; i < nx; i++)
        for (j = 0; j < ny; j++)
        {
            k = j + i*ny;
            gsl_spline2d_set(spline, za, i, j, z_ar[k]);//заполняем значения za по заданной сетке х и у
        }
    gsl_spline2d_init( spline, x_ar, y_ar, za, nx, ny);//инициализируем сплайн тремя массивами и размерами сетки
    zi = gsl_spline2d_eval(spline, xi, yi, xacc, yacc);//получаем значение z
    gsl_spline2d_free(spline); //очищаем память во избежание утечек
    gsl_interp_accel_free(xacc);
    gsl_interp_accel_free(yacc);
    free(za);//обязательно - использован malloc
return zi;
}



//Решение уравнения
double Helicopter::SolvEquation(int fkey, double x_lo, double x_hi)
{//
    gsl_set_error_handler_off();//выключает крах программы при ошибках
    QString ss, lines="\tstatus of SolvEquation("+ss.setNum(fkey)+"): ";
    double r = 0, r_expected = 0;
    int status,statuss; //переменная хранящая статус выполнения GSL функций
    int iter = 0, max_iter = 100; //переменные счетчик итераций и максимальное значение итераций
    const gsl_root_fsolver_type *T; //обьявляется указатель на тип решателя
    gsl_root_fsolver *s; //обьявляется указатель на решатель
    gsl_function F; //обьвляется структура связывающая фун-ю квадратного уравнения и его параметры
    struct rparams params = {0.0}; //инициализирцуется структура c коефициентами уравнения
    F.params = &params;//в структуру записывается ссылка на структуру с параметрами
    switch(fkey)
    {//в структуру записывается ссылка на функцию квадратного уравнения
        case 1:  F.function = &f_x1root1;  break;
        case 2:  F.function = &f_x1root2;  break;
        case 3:  F.function = &f_e0root3;  break;
    }
    T = gsl_root_fsolver_brent; //инициализируется тип решателя
    s = gsl_root_fsolver_alloc(T);//создается обьект решателя ранее определенного типа
    statuss = gsl_root_fsolver_set(s, &F, x_lo, x_hi); //решателю s передается ссылка на структуру F и пределы поиста решения
    if(statuss) lines += "GSL_WARNING: "+ ss.setNum(statuss)+", status: ";

    do //начало итерационного цикла
    {
        iter++; //инкрементируется счетчик итераций
        status = gsl_root_fsolver_iterate(s); //заставляем решатель выполнить одну итерацию и записываем результат выполнения в статус
        r =      gsl_root_fsolver_root(s);    //в переменную считываем текущее значение корня
        x_lo =   gsl_root_fsolver_x_lower(s); //и границы
        x_hi =   gsl_root_fsolver_x_upper(s); //интервала
        status = gsl_root_test_interval(x_lo, x_hi, 0, 0.001);//и сравниваем на этом интервале сходится ли решение с заданной точностью
        /*if(log>1)
        {
            lines += ss.setNum(iter);//
            lines += "("+ ss.setNum(status) +")";//
            lines += "  \tr=" + ss.setNum(r,'f',6);
            lines += "  \tx_lo=" + ss.setNum(x_lo,'f',6);
            lines += "  \tx_hi=" + ss.setNum(x_hi,'f',6);
            lines += "\n";
            memout->appendPlainText( lines );
        }*/
    }
    while (status == GSL_CONTINUE && iter < max_iter);//условие выполнения цикла
    if(logkey>1)
    {
        lines += ss.setNum(status);//
        lines += "\tr=" + ss.setNum(r,'f',6);
        lines += "  \tx_lo=" + ss.setNum(x_lo,'f',6);
        lines += "  \tx_hi=" + ss.setNum(x_hi,'f',6);
    }
    gsl_root_fsolver_free (s);//удаляем решатель
return r;
}



//решение системы 4x линейных уравнений
quat Helicopter::SolveSystem4x4(pent e1, pent e2, pent e3, pent e4)
{//
    gsl_set_error_handler_off();//выключает крах программы при ошибках
    quat Out;
    QString ss, lines="\tstatus of SolveSystem4x4(): ";
    double a_data[] = { e1.a, e1.b, e1.c, e1.d,
                        e2.a, e2.b, e2.c, e2.d,
                        e3.a, e3.b, e3.c, e3.d,
                        e4.a, e4.b, e4.c, e4.d };//матрица коеффициентов A
    double b_data[] = { e1.e, e2.e, e3.e, e4.e };//вектор свободных членов B
    gsl_vector* res = slae_solve(a_data,b_data, 4, 4);
    Out.a = gsl_vector_get(res,0);
    Out.b = gsl_vector_get(res,1);
    Out.c = gsl_vector_get(res,2);
    Out.d = gsl_vector_get(res,3);
    if(logkey>1)
    {
        lines += "\tx1=" + ss.setNum( Out.a ,'f',6);
        lines += "\tx2=" + ss.setNum( Out.b ,'f',6);
        lines += "\tx3=" + ss.setNum( Out.c ,'f',6);
        lines += "\tx4=" + ss.setNum( Out.d ,'f',6);
    }
    gsl_vector_free (res);
return Out;
}
gsl_vector* Helicopter::slae_solve(double A_data[],double B_data[], int ne, int nx)
{
    int s;
    gsl_matrix_view m = gsl_matrix_view_array (A_data, ne, nx);//на основании матрицы коеффициентов создается структура матрици GSL такая структура умеет возвращать обьекты типа gsl_matrix и т.д.
    gsl_vector_view b = gsl_vector_view_array (B_data, ne);    //на вектор свободных членов создается структура вектора GSL
    gsl_vector *x = gsl_vector_alloc (nx);                     //создается вектор искомых переменных и возвращается указатель на него
    gsl_permutation * p = gsl_permutation_alloc (ne);          //создается структура определяющая количество перестановок для числа позиций порядка решаемой СЛАУ - требуется для декомпозиции и решателя
    gsl_linalg_LU_decomp (&m.matrix, p, &s);                   //выполняется LU разложение матрицы, функция принимает ссылку на матрицу, структуру с кол-вом перестановок, и какуюто переменную типа int
    gsl_linalg_LU_solve (&m.matrix, p, &b.vector, x);          //решается СЛАУ - принимает матрицу на которой выполнено LU разложение, стр-ру с перестановками, ссылку на вектор свободных членов, и вектор куда запишутся искомые неизвестные
    gsl_permutation_free (p);                                  //ОСВОБОЖДЕНИЕ ПАМЯТИ
    return x;
}



//решение системы 4x нелинейных уравнений
quat Helicopter::SolveSystemNL4()//QPlainTextEdit *memo double x1ini, double x2ini, double x3ini, double x4ini
{
    QString ss, lines="\tstatus of SolveSystemNL4(): ";
    double x_init[4] = {0, 0, 0, 0};//kren, T_tr_    //массив содержащий начальные приближения x0, x1,... которые будут переданы в функцию rosenbrock_f
    gsl_set_error_handler_off();//выключает крах программы при ошибках
    quat returnstr;
    const size_t n = 4;                 //это порядок системы уравнений
    const gsl_multiroot_fsolver_type *T;//обьявляется указатель на обьект определяющий тип рещателя который мы будем использовать
    gsl_multiroot_fsolver *s;           //обьявляется указатель на обьект самого решателя
    int status;                         //здесь будет храниться статус решателя, переменная может быть int т.к. статус это enum
    size_t iter = 0;                    //счетчик итераций
    struct rparams p = {0};             //обьявляется переменная p структуры rparams, но она будет нулевым массивом потому что в уравнениях константы забиты вручную
    //gsl_multiroot_function fstr = {&rosenbrock_f,n,&p};//обьявляется структура fstr которая хранит в себе ссылку на функцию для расчета системы, порядок системы и ссылку на структуру с константами, но структура с константами не будет использована потому что в уравнениях они забиты вручную
    gsl_multiroot_function fstr;
    fstr.f = &system4nlf;//
    fstr.n = 4;
    fstr.params = &p;
    gsl_vector* X = gsl_vector_alloc(n);//здесь выделяется память под вектор размером n - порядок системы. как раз он и будет содержать в себе текущие для решателя значения x0,x1... сначала в него будут записаны значения с массива x_init, а потом решатель сам будет их итерационно подбирать и перезаписывать
    gsl_vector_set (X, 0, x_init[0]);   //как раз здесь в вектор x и записываются начальные приближения из x_init
    gsl_vector_set (X, 1, x_init[1]);
    gsl_vector_set (X, 2, x_init[2]);
    gsl_vector_set (X, 3, x_init[3]);
    T = gsl_multiroot_fsolver_hybrids;  //ранее обьявленному указателю на тип решателя присваевается адрес константы определяющей что это будет гибридный решатель, то есть gsl_multiroot_fsolver_hybrids
    s = gsl_multiroot_fsolver_alloc(T,n);//создается в памяти сам решатель принимает переменную T говорящую какой тип решения он должен выполнять и порядок системмы уравнений которую он будет решать
    gsl_multiroot_fsolver_set(s, &fstr, X);//эта функция глобальна для GSL и она привязывает определенному решателю (у нас s) ссылку на структуру fstr которая связывает функцию системы уравнений, порядок системы и ссылку на структуру с константами {&rosenbrock_f,n,&p} и также привязывает вектор с значениями аргументов системы, которые в процессе итераций будут подбираться и перезаписываться в этом же векторе

    lines += "\n";//
    do//цикл итераций
    {
        iter++; //прибавляется счетчик итераций
        status = gsl_multiroot_fsolver_iterate(s);//вызывается глобальная функция которая заставляет определенный решатель (у нас он зовется s) выполнить одну итерацию //результат итерации записывается в переменную status
        //if(logkey>1)
        {
            lines += "\t"+ss.setNum(iter);//
            lines += "  \tx1=" + ss.setNum(gsl_vector_get(s->x,0),'f',9);
            lines += "  \tx2=" + ss.setNum(gsl_vector_get(s->x,1),'f',9);
            lines += "  \tx3=" + ss.setNum(gsl_vector_get(s->x,2),'f',9);
            lines += "  \tx4=" + ss.setNum(gsl_vector_get(s->x,3),'f',9);
            lines += "  \ty1=" + ss.setNum(gsl_vector_get(s->f,0),'f',6);
            lines += "  \ty2=" + ss.setNum(gsl_vector_get(s->f,1),'f',6);
            lines += "  \ty3=" + ss.setNum(gsl_vector_get(s->f,2),'f',6);
            lines += "  \ty4=" + ss.setNum(gsl_vector_get(s->f,3),'f',6);
            lines += "\n";//
        }
        status = gsl_multiroot_test_residual(s->f, 0.01);//1e-3 //если цикл не прерван и все уравнения посчитаны то вызовем глобальную функцию которая проверит меньше ли значения уравнений системы чем заданная точность (здесь точность 1е-9),
        //доступ к значениям уравнений функция gsl_multiroot_test_residual получает через вектор f который находится в решателе s. результат проверки на сходимость записывается в переменную status
        //memo->appendPlainText( lines );
        QCoreApplication::processEvents();
    }
    while(status==GSL_CONTINUE && iter<50);//если проверка на сходимость не выполнена то в статусе будет храниться код GSL_CONTINUE и итерации продолжаются если их не больше 1000
    if(status)//проверка успешно ли выполнена итерация, может системма не считается при текущих подобранных аргументах, что-то типа деления на ноль, тогда прерываем цикл итераций
    {
        switch(status)
        {
            case GSL_CONTINUE :
            warning = 4;
            lines += "\tWarning: iteration has not converged; status=";
            break;
            case GSL_EDOM     : lines += "ERROR: input domain error, e.g sqrt(-1)                     "; break;
            case GSL_ERANGE   : lines += "ERROR: output range error, e.g. exp(1e100)                  "; break;
            case GSL_EFAULT   : lines += "ERROR: invalid pointer                                      "; break;
            case GSL_EINVAL   : lines += "ERROR: invalid argument supplied by user                    "; break;
            case GSL_EFAILED  : lines += "ERROR: generic failure                                      "; break;
            case GSL_EFACTOR  : lines += "ERROR: factorization failed                                 "; break;
            case GSL_ESANITY  : lines += "ERROR: sanity check failed - shouldn't happen               "; break;
            case GSL_ENOMEM   : lines += "ERROR: malloc failed                                        "; break;
            case GSL_EBADFUNC : lines += "ERROR: problem with user-supplied function                  "; break;
            case GSL_ERUNAWAY : lines += "ERROR: iterative process is out of control                  "; break;
            case GSL_EMAXITER : lines += "ERROR: exceeded max number of iterations                    "; break;
            case GSL_EZERODIV : lines += "ERROR: tried to divide by zero                              "; break;
            case GSL_EBADTOL  : lines += "ERROR: user specified an invalid tolerance                  "; break;
            case GSL_ETOL     : lines += "ERROR: failed to reach the specified tolerance              "; break;
            case GSL_EUNDRFLW : lines += "ERROR: underflow                                            "; break;
            case GSL_EOVRFLW  : lines += "ERROR: overflow                                             "; break;
            case GSL_ELOSS    : lines += "ERROR: loss of accuracy                                     "; break;
            case GSL_EROUND   : lines += "ERROR: failed because of roundoff error                     "; break;
            case GSL_EBADLEN  : lines += "ERROR: matrix, vector lengths are not conformant            "; break;
            case GSL_ENOTSQR  : lines += "ERROR: matrix not square                                    "; break;
            case GSL_ESING    : lines += "ERROR: apparent singularity detected                        "; break;
            case GSL_EDIVERGE : lines += "ERROR: integral or series is divergent                      "; break;
            case GSL_EUNSUP   : lines += "ERROR: requested feature is not supported by the hardware   "; break;
            case GSL_EUNIMPL  : lines += "ERROR: requested feature not (yet) implemented              "; break;
            case GSL_ECACHE   : lines += "ERROR: cache limit exceeded                                 "; break;
            case GSL_ETABLE   : lines += "ERROR: table limit exceeded                                 "; break;
            case GSL_ENOPROG  : lines += "ERROR: iteration is not making progress towards solution    "; break;
            case GSL_ENOPROGJ : lines += "ERROR: jacobian evaluations are not improving the solution  "; break;
            case GSL_ETOLF    : lines += "ERROR: cannot reach the specified tolerance in F            "; break;
            case GSL_ETOLX    : lines += "ERROR: cannot reach the specified tolerance in X            "; break;
            case GSL_ETOLG    : lines += "ERROR: cannot reach the specified tolerance in gradient     "; break;
            case GSL_EOF      : lines += "ERROR: end of file                                          "; break;
            default : lines += "Error!";
        }
    }
    //RETURN:
    returnstr.a = gsl_vector_get(s->x, 0);
    returnstr.b = gsl_vector_get(s->x, 1);
    returnstr.c = gsl_vector_get(s->x, 2);
    returnstr.d = gsl_vector_get(s->x, 3);
    //окончательный пересчет уравнений
    phc->HeliBalanceDynamic(returnstr.a,returnstr.b,returnstr.c,returnstr.d);
    phc->b_Fz();
    phc->b_Mx();
    phc->b_My();
    phc->b_Mz();

    gsl_multiroot_fsolver_free(s);//удаляем обьект решателя
    //gsl_vector_free (x);        //удаляем вектор с подобранными аргументами x, а можно его и не удалять а использовать решение где-то еще )))
    //lines += "\tstatus="+ss.setNum(status);
return returnstr;
}
int  Helicopter::system4nlf (const gsl_vector *x, void *params, gsl_vector *f) //описание системы нелинейных уравнений
{
  //вытаскиваются константы уравнения из структуры rparams и присваиваются локальным переменным
  //double a = ((struct rparams *) params)->a;
  //double b = ((struct rparams *) params)->b;
  //локальным переменным x0 и x1 присваиваются значения из вектора аргументов
  //первое присвоение вытаскивает из вектора значение начального приближения, а при последующих в векторе хранятся значения которые "подбирает" итерационный решатель и "подобранные" решателем
  //значение искомых переменных присваиваются локальным переменным y0 и y1 присваиваются значения функций соответствующие аргументам текущих x0 и x1
  //считаются y0 и y1 с помощью функций определенных выше главное чтобы они могли вернуть значение каждого уравнения системмы в заданных аргументах x0 и x1
    const double x0 = gsl_vector_get (x, 0);
    const double x1 = gsl_vector_get (x, 1);
    const double x2 = gsl_vector_get (x, 2);
    const double x3 = gsl_vector_get (x, 3);
    phc->HeliBalanceDynamic(x0,x1,x2,x3);
    const double y0 = phc->b_Fz();//x0,x1,x2,x3
    const double y1 = phc->b_Mx();//x0,x1,x2,x3
    const double y2 = phc->b_My();//x0,x1,x2,x3
    const double y3 = phc->b_Mz();//x0,x1,x2,x3
  //в вектор f который принимает функция rosenbrock_f записываются расчитанные значения уравнения системмы
  //которые в последующем использует итерационный решатель для проверки на сходимость (когда сходится решение значения должны быть нулевыми)
    gsl_vector_set(f, 0, y0);
    gsl_vector_set(f, 1, y1);
    gsl_vector_set(f, 2, y2);
    gsl_vector_set(f, 3, y3);

return GSL_SUCCESS;//возврат итерационному решателю сообщения о том что функция rosenbrock_f выполнилась успешно, тоесть она "считабельна" при данных аргументах x0 и x1
}


//решение системы 4x нелинейных уравнений
quat Helicopter::SolveSystemNL4ag()//QPlainTextEdit *memo double x1ini, double x2ini, double x3ini, double x4ini
{
    QString ss, lines="\tstatus of SolveSystemNL4(): ";
    double x_init[4] = {0, 0, 0, 0};//kren, T_tr_    //массив содержащий начальные приближения x0, x1,... которые будут переданы в функцию rosenbrock_f
    gsl_set_error_handler_off();//выключает крах программы при ошибках
    quat returnstr;
    const size_t n = 4;                 //это порядок системы уравнений
    const gsl_multiroot_fsolver_type *T;//обьявляется указатель на обьект определяющий тип рещателя который мы будем использовать
    gsl_multiroot_fsolver *s;           //обьявляется указатель на обьект самого решателя
    int status;                         //здесь будет храниться статус решателя, переменная может быть int т.к. статус это enum
    size_t iter = 0;                    //счетчик итераций
    struct rparams p = {0};             //обьявляется переменная p структуры rparams, но она будет нулевым массивом потому что в уравнениях константы забиты вручную
    //gsl_multiroot_function fstr = {&rosenbrock_f,n,&p};//обьявляется структура fstr которая хранит в себе ссылку на функцию для расчета системы, порядок системы и ссылку на структуру с константами, но структура с константами не будет использована потому что в уравнениях они забиты вручную
    gsl_multiroot_function fstr;
    fstr.f = &system4ag;//
    fstr.n = 4;
    fstr.params = &p;
    gsl_vector* X = gsl_vector_alloc(n);//здесь выделяется память под вектор размером n - порядок системы. как раз он и будет содержать в себе текущие для решателя значения x0,x1... сначала в него будут записаны значения с массива x_init, а потом решатель сам будет их итерационно подбирать и перезаписывать
    gsl_vector_set (X, 0, x_init[0]);   //как раз здесь в вектор x и записываются начальные приближения из x_init
    gsl_vector_set (X, 1, x_init[1]);
    gsl_vector_set (X, 2, x_init[2]);
    gsl_vector_set (X, 3, x_init[3]);
    T = gsl_multiroot_fsolver_hybrids;  //ранее обьявленному указателю на тип решателя присваевается адрес константы определяющей что это будет гибридный решатель, то есть gsl_multiroot_fsolver_hybrids
    s = gsl_multiroot_fsolver_alloc(T,n);//создается в памяти сам решатель принимает переменную T говорящую какой тип решения он должен выполнять и порядок системмы уравнений которую он будет решать
    gsl_multiroot_fsolver_set(s, &fstr, X);//эта функция глобальна для GSL и она привязывает определенному решателю (у нас s) ссылку на структуру fstr которая связывает функцию системы уравнений, порядок системы и ссылку на структуру с константами {&rosenbrock_f,n,&p} и также привязывает вектор с значениями аргументов системы, которые в процессе итераций будут подбираться и перезаписываться в этом же векторе

    lines += "\n";//
    do//цикл итераций
    {
        iter++; //прибавляется счетчик итераций
        status = gsl_multiroot_fsolver_iterate(s);//вызывается глобальная функция которая заставляет определенный решатель (у нас он зовется s) выполнить одну итерацию //результат итерации записывается в переменную status
        //if(logkey>1)
        {
            lines += "\t"+ss.setNum(iter);//
            lines += "  \tx1=" + ss.setNum(gsl_vector_get(s->x,0),'f',9);
            lines += "  \tx2=" + ss.setNum(gsl_vector_get(s->x,1),'f',9);
            lines += "  \tx3=" + ss.setNum(gsl_vector_get(s->x,2),'f',9);
            lines += "  \tx4=" + ss.setNum(gsl_vector_get(s->x,3),'f',9);
            lines += "  \ty1=" + ss.setNum(gsl_vector_get(s->f,0),'f',6);
            lines += "  \ty2=" + ss.setNum(gsl_vector_get(s->f,1),'f',6);
            lines += "  \ty3=" + ss.setNum(gsl_vector_get(s->f,2),'f',6);
            lines += "  \ty4=" + ss.setNum(gsl_vector_get(s->f,3),'f',6);
            lines += "\n";//
        }
        status = gsl_multiroot_test_residual(s->f, 0.01);//1e-3 //если цикл не прерван и все уравнения посчитаны то вызовем глобальную функцию которая проверит меньше ли значения уравнений системы чем заданная точность (здесь точность 1е-9),
        //доступ к значениям уравнений функция gsl_multiroot_test_residual получает через вектор f который находится в решателе s. результат проверки на сходимость записывается в переменную status
        //memo->appendPlainText( lines );
        QCoreApplication::processEvents();
    }
    while(status==GSL_CONTINUE && iter<50);//если проверка на сходимость не выполнена то в статусе будет храниться код GSL_CONTINUE и итерации продолжаются если их не больше 1000
    if(status)//проверка успешно ли выполнена итерация, может системма не считается при текущих подобранных аргументах, что-то типа деления на ноль, тогда прерываем цикл итераций
    {
        switch(status)
        {
            case GSL_CONTINUE :
            warning = 4;
            lines += "\tWarning: iteration has not converged; status=";
            break;
            case GSL_EDOM     : lines += "ERROR: input domain error, e.g sqrt(-1)                     "; break;
            case GSL_ERANGE   : lines += "ERROR: output range error, e.g. exp(1e100)                  "; break;
            case GSL_EFAULT   : lines += "ERROR: invalid pointer                                      "; break;
            case GSL_EINVAL   : lines += "ERROR: invalid argument supplied by user                    "; break;
            case GSL_EFAILED  : lines += "ERROR: generic failure                                      "; break;
            case GSL_EFACTOR  : lines += "ERROR: factorization failed                                 "; break;
            case GSL_ESANITY  : lines += "ERROR: sanity check failed - shouldn't happen               "; break;
            case GSL_ENOMEM   : lines += "ERROR: malloc failed                                        "; break;
            case GSL_EBADFUNC : lines += "ERROR: problem with user-supplied function                  "; break;
            case GSL_ERUNAWAY : lines += "ERROR: iterative process is out of control                  "; break;
            case GSL_EMAXITER : lines += "ERROR: exceeded max number of iterations                    "; break;
            case GSL_EZERODIV : lines += "ERROR: tried to divide by zero                              "; break;
            case GSL_EBADTOL  : lines += "ERROR: user specified an invalid tolerance                  "; break;
            case GSL_ETOL     : lines += "ERROR: failed to reach the specified tolerance              "; break;
            case GSL_EUNDRFLW : lines += "ERROR: underflow                                            "; break;
            case GSL_EOVRFLW  : lines += "ERROR: overflow                                             "; break;
            case GSL_ELOSS    : lines += "ERROR: loss of accuracy                                     "; break;
            case GSL_EROUND   : lines += "ERROR: failed because of roundoff error                     "; break;
            case GSL_EBADLEN  : lines += "ERROR: matrix, vector lengths are not conformant            "; break;
            case GSL_ENOTSQR  : lines += "ERROR: matrix not square                                    "; break;
            case GSL_ESING    : lines += "ERROR: apparent singularity detected                        "; break;
            case GSL_EDIVERGE : lines += "ERROR: integral or series is divergent                      "; break;
            case GSL_EUNSUP   : lines += "ERROR: requested feature is not supported by the hardware   "; break;
            case GSL_EUNIMPL  : lines += "ERROR: requested feature not (yet) implemented              "; break;
            case GSL_ECACHE   : lines += "ERROR: cache limit exceeded                                 "; break;
            case GSL_ETABLE   : lines += "ERROR: table limit exceeded                                 "; break;
            case GSL_ENOPROG  : lines += "ERROR: iteration is not making progress towards solution    "; break;
            case GSL_ENOPROGJ : lines += "ERROR: jacobian evaluations are not improving the solution  "; break;
            case GSL_ETOLF    : lines += "ERROR: cannot reach the specified tolerance in F            "; break;
            case GSL_ETOLX    : lines += "ERROR: cannot reach the specified tolerance in X            "; break;
            case GSL_ETOLG    : lines += "ERROR: cannot reach the specified tolerance in gradient     "; break;
            case GSL_EOF      : lines += "ERROR: end of file                                          "; break;
            default : lines += "Error!";
        }
    }
    //RETURN:
    returnstr.a = gsl_vector_get(s->x, 0);
    returnstr.b = gsl_vector_get(s->x, 1);
    returnstr.c = gsl_vector_get(s->x, 2);
    returnstr.d = gsl_vector_get(s->x, 3);
    //окончательный пересчет уравнений
    phc->HeliBalanceDynamic(returnstr.a,returnstr.b,returnstr.c,returnstr.d);
    phc->b_Fx();
    phc->b_Fz();
    phc->b_Mx();
    phc->b_Mz();

    gsl_multiroot_fsolver_free(s);//удаляем обьект решателя
    //gsl_vector_free (x);        //удаляем вектор с подобранными аргументами x, а можно его и не удалять а использовать решение где-то еще )))
    //lines += "\tstatus="+ss.setNum(status);
return returnstr;
}
int  Helicopter::system4ag (const gsl_vector *x, void *params, gsl_vector *f) //описание системы нелинейных уравнений
{
  //вытаскиваются константы уравнения из структуры rparams и присваиваются локальным переменным
  //double a = ((struct rparams *) params)->a;
  //double b = ((struct rparams *) params)->b;
  //локальным переменным x0 и x1 присваиваются значения из вектора аргументов
  //первое присвоение вытаскивает из вектора значение начального приближения, а при последующих в векторе хранятся значения которые "подбирает" итерационный решатель и "подобранные" решателем
  //значение искомых переменных присваиваются локальным переменным y0 и y1 присваиваются значения функций соответствующие аргументам текущих x0 и x1
  //считаются y0 и y1 с помощью функций определенных выше главное чтобы они могли вернуть значение каждого уравнения системмы в заданных аргументах x0 и x1
    const double x0 = gsl_vector_get(x, 0);
    const double x1 = gsl_vector_get(x, 1);
    const double x2 = gsl_vector_get(x, 2);
    const double x3 = gsl_vector_get(x, 3);
    phc->HeliBalanceDynamic(x0,x1,x2,x3);
    const double y0 = phc->b_Fx();//x0,x1,x2,x3
    const double y1 = phc->b_Fz();//x0,x1,x2,x3
    const double y2 = phc->b_Mx();//x0,x1,x2,x3
    const double y3 = phc->b_Mz();//x0,x1,x2,x3
  //в вектор f который принимает функция rosenbrock_f записываются расчитанные значения уравнения системмы
  //которые в последующем использует итерационный решатель для проверки на сходимость (когда сходится решение значения должны быть нулевыми)
    gsl_vector_set(f, 0, y0);
    gsl_vector_set(f, 1, y1);
    gsl_vector_set(f, 2, y2);
    gsl_vector_set(f, 3, y3);

return GSL_SUCCESS;//возврат итерационному решателю сообщения о том что функция rosenbrock_f выполнилась успешно, тоесть она "считабельна" при данных аргументах x0 и x1
}



//решение системы 5 нелинейных уравнений
pent Helicopter::SolveSystemNL5()//QPlainTextEdit *memo double x1ini, double x2ini, double x3ini, double x4ini
{
    QString ss, lines="\tstatus of SolveSystemNL5(): ";
    double x_init[5] = {0, 0, 0, 0, 0};//kren, T_tr_    //массив содержащий начальные приближения x0, x1,... которые будут переданы в функцию rosenbrock_f
    gsl_set_error_handler_off();//выключает крах программы при ошибках
    pent returnstr;
    const size_t n = 5;                 //это порядок системы уравнений
    const gsl_multiroot_fsolver_type *T;//обьявляется указатель на обьект определяющий тип рещателя который мы будем использовать
    gsl_multiroot_fsolver *s;           //обьявляется указатель на обьект самого решателя
    int status;                         //здесь будет храниться статус решателя, переменная может быть int т.к. статус это enum
    size_t iter = 0;                    //счетчик итераций
    struct rparams p = {0};             //обьявляется переменная p структуры rparams, но она будет нулевым массивом потому что в уравнениях константы забиты вручную
    //gsl_multiroot_function fstr = {&rosenbrock_f,n,&p};//обьявляется структура fstr которая хранит в себе ссылку на функцию для расчета системы, порядок системы и ссылку на структуру с константами, но структура с константами не будет использована потому что в уравнениях они забиты вручную
    gsl_multiroot_function fstr;
    fstr.f = &system5nlf;//
    fstr.n = 5;
    fstr.params = &p;
    gsl_vector* X = gsl_vector_alloc(n);//здесь выделяется память под вектор размером n - порядок системы. как раз он и будет содержать в себе текущие для решателя значения x0,x1... сначала в него будут записаны значения с массива x_init, а потом решатель сам будет их итерационно подбирать и перезаписывать
    gsl_vector_set (X, 0, x_init[0]);   //как раз здесь в вектор x и записываются начальные приближения из x_init
    gsl_vector_set (X, 1, x_init[1]);
    gsl_vector_set (X, 2, x_init[2]);
    gsl_vector_set (X, 3, x_init[3]);
    gsl_vector_set (X, 4, x_init[4]);
    T = gsl_multiroot_fsolver_hybrids;  //ранее обьявленному указателю на тип решателя присваевается адрес константы определяющей что это будет гибридный решатель, то есть gsl_multiroot_fsolver_hybrids
    s = gsl_multiroot_fsolver_alloc(T,n);//создается в памяти сам решатель принимает переменную T говорящую какой тип решения он должен выполнять и порядок системмы уравнений которую он будет решать
    gsl_multiroot_fsolver_set(s, &fstr, X);//эта функция глобальна для GSL и она привязывает определенному решателю (у нас s) ссылку на структуру fstr которая связывает функцию системы уравнений, порядок системы и ссылку на структуру с константами {&rosenbrock_f,n,&p} и также привязывает вектор с значениями аргументов системы, которые в процессе итераций будут подбираться и перезаписываться в этом же векторе

    lines += "\n";//
    do//цикл итераций
    {
        iter++; //прибавляется счетчик итераций
        status = gsl_multiroot_fsolver_iterate(s);//вызывается глобальная функция которая заставляет определенный решатель (у нас он зовется s) выполнить одну итерацию //результат итерации записывается в переменную status
        //if(logkey>1)
        {
            lines += "\t"+ss.setNum(iter);//
            lines += "  \tx1=" + ss.setNum(gsl_vector_get(s->x,0),'f',9);
            lines += "  \tx2=" + ss.setNum(gsl_vector_get(s->x,1),'f',9);
            lines += "  \tx3=" + ss.setNum(gsl_vector_get(s->x,2),'f',9);
            lines += "  \tx4=" + ss.setNum(gsl_vector_get(s->x,3),'f',9);
            lines += "  \tx5=" + ss.setNum(gsl_vector_get(s->x,4),'f',9);
            lines += "  \ty1=" + ss.setNum(gsl_vector_get(s->f,0),'f',6);
            lines += "  \ty2=" + ss.setNum(gsl_vector_get(s->f,1),'f',6);
            lines += "  \ty3=" + ss.setNum(gsl_vector_get(s->f,2),'f',6);
            lines += "  \ty4=" + ss.setNum(gsl_vector_get(s->f,3),'f',6);
            lines += "  \ty5=" + ss.setNum(gsl_vector_get(s->f,4),'f',6);
            lines += "\n";//
        }
        status = gsl_multiroot_test_residual(s->f, 0.01);//1e-3 //если цикл не прерван и все уравнения посчитаны то вызовем глобальную функцию которая проверит меньше ли значения уравнений системы чем заданная точность (здесь точность 1е-9),
        //доступ к значениям уравнений функция gsl_multiroot_test_residual получает через вектор f который находится в решателе s. результат проверки на сходимость записывается в переменную status
        //memo->appendPlainText( lines );
        QCoreApplication::processEvents();
    }
    while(status==GSL_CONTINUE && iter<50);//если проверка на сходимость не выполнена то в статусе будет храниться код GSL_CONTINUE и итерации продолжаются если их не больше 1000
    if(status)//проверка успешно ли выполнена итерация, может системма не считается при текущих подобранных аргументах, что-то типа деления на ноль, тогда прерываем цикл итераций
    {
        switch(status)
        {
            case GSL_CONTINUE :
            warning = 4;
            lines += "\tWarning: iteration has not converged; status=";
            break;
            case GSL_EDOM     : lines += "ERROR: input domain error, e.g sqrt(-1)                     "; break;
            case GSL_ERANGE   : lines += "ERROR: output range error, e.g. exp(1e100)                  "; break;
            case GSL_EFAULT   : lines += "ERROR: invalid pointer                                      "; break;
            case GSL_EINVAL   : lines += "ERROR: invalid argument supplied by user                    "; break;
            case GSL_EFAILED  : lines += "ERROR: generic failure                                      "; break;
            case GSL_EFACTOR  : lines += "ERROR: factorization failed                                 "; break;
            case GSL_ESANITY  : lines += "ERROR: sanity check failed - shouldn't happen               "; break;
            case GSL_ENOMEM   : lines += "ERROR: malloc failed                                        "; break;
            case GSL_EBADFUNC : lines += "ERROR: problem with user-supplied function                  "; break;
            case GSL_ERUNAWAY : lines += "ERROR: iterative process is out of control                  "; break;
            case GSL_EMAXITER : lines += "ERROR: exceeded max number of iterations                    "; break;
            case GSL_EZERODIV : lines += "ERROR: tried to divide by zero                              "; break;
            case GSL_EBADTOL  : lines += "ERROR: user specified an invalid tolerance                  "; break;
            case GSL_ETOL     : lines += "ERROR: failed to reach the specified tolerance              "; break;
            case GSL_EUNDRFLW : lines += "ERROR: underflow                                            "; break;
            case GSL_EOVRFLW  : lines += "ERROR: overflow                                             "; break;
            case GSL_ELOSS    : lines += "ERROR: loss of accuracy                                     "; break;
            case GSL_EROUND   : lines += "ERROR: failed because of roundoff error                     "; break;
            case GSL_EBADLEN  : lines += "ERROR: matrix, vector lengths are not conformant            "; break;
            case GSL_ENOTSQR  : lines += "ERROR: matrix not square                                    "; break;
            case GSL_ESING    : lines += "ERROR: apparent singularity detected                        "; break;
            case GSL_EDIVERGE : lines += "ERROR: integral or series is divergent                      "; break;
            case GSL_EUNSUP   : lines += "ERROR: requested feature is not supported by the hardware   "; break;
            case GSL_EUNIMPL  : lines += "ERROR: requested feature not (yet) implemented              "; break;
            case GSL_ECACHE   : lines += "ERROR: cache limit exceeded                                 "; break;
            case GSL_ETABLE   : lines += "ERROR: table limit exceeded                                 "; break;
            case GSL_ENOPROG  : lines += "ERROR: iteration is not making progress towards solution    "; break;
            case GSL_ENOPROGJ : lines += "ERROR: jacobian evaluations are not improving the solution  "; break;
            case GSL_ETOLF    : lines += "ERROR: cannot reach the specified tolerance in F            "; break;
            case GSL_ETOLX    : lines += "ERROR: cannot reach the specified tolerance in X            "; break;
            case GSL_ETOLG    : lines += "ERROR: cannot reach the specified tolerance in gradient     "; break;
            case GSL_EOF      : lines += "ERROR: end of file                                          "; break;
            default : lines += "Error!";
        }
    }
    //RETURN:
    returnstr.a = gsl_vector_get(s->x, 0);
    returnstr.b = gsl_vector_get(s->x, 1);
    returnstr.c = gsl_vector_get(s->x, 2);
    returnstr.d = gsl_vector_get(s->x, 3);
    returnstr.e = gsl_vector_get(s->x, 4);
    //окончательный пересчет уравнений
    ///phc->HeliBalanceDynamic5(returnstr.a,returnstr.b,returnstr.c,returnstr.d,returnstr.e);
    ///phc->b_Fx();
    ///phc->b_Fz();
    ///phc->b_Mx();
    ///phc->b_My();
    ///phc->b_Mz();

    gsl_multiroot_fsolver_free(s);//удаляем обьект решателя
    //gsl_vector_free (x);        //удаляем вектор с подобранными аргументами x, а можно его и не удалять а использовать решение где-то еще )))
    //lines += "\tstatus="+ss.setNum(status);
return returnstr;
}
int  Helicopter::system5nlf (const gsl_vector *x, void *params, gsl_vector *f) //описание системы нелинейных уравнений
{
  //вытаскиваются константы уравнения из структуры rparams и присваиваются локальным переменным
  //double a = ((struct rparams *) params)->a;
  //double b = ((struct rparams *) params)->b;
  //локальным переменным x0 и x1 присваиваются значения из вектора аргументов
  //первое присвоение вытаскивает из вектора значение начального приближения, а при последующих в векторе хранятся значения которые "подбирает" итерационный решатель и "подобранные" решателем
  //значение искомых переменных присваиваются локальным переменным y0 и y1 присваиваются значения функций соответствующие аргументам текущих x0 и x1
  //считаются y0 и y1 с помощью функций определенных выше главное чтобы они могли вернуть значение каждого уравнения системмы в заданных аргументах x0 и x1
    const double x0 = gsl_vector_get (x, 0);
    const double x1 = gsl_vector_get (x, 1);
    const double x2 = gsl_vector_get (x, 2);
    const double x3 = gsl_vector_get (x, 3);
    const double x4 = gsl_vector_get (x, 4);
    ///phc->HeliBalanceDynamic5(x0,x1,x2,x3,x4);
    const double y4 = phc->b_Fx();//x0,x1,x2,x3
    const double y0 = phc->b_Fz();//x0,x1,x2,x3
    const double y1 = phc->b_Mx();//x0,x1,x2,x3
    const double y2 = phc->b_My();//x0,x1,x2,x3
    const double y3 = phc->b_Mz();//x0,x1,x2,x3
  //в вектор f который принимает функция rosenbrock_f записываются расчитанные значения уравнения системмы
  //которые в последующем использует итерационный решатель для проверки на сходимость (когда сходится решение значения должны быть нулевыми)
    gsl_vector_set(f, 0, y0);
    gsl_vector_set(f, 1, y1);
    gsl_vector_set(f, 2, y2);
    gsl_vector_set(f, 3, y3);
    gsl_vector_set(f, 4, y4);

return GSL_SUCCESS;//возврат итерационному решателю сообщения о том что функция rosenbrock_f выполнилась успешно, тоесть она "считабельна" при данных аргументах x0 и x1
}



//решение системы 6 нелинейных уравнений
six  Helicopter::SolveSystemNL6()
{
    QString ss;
    snl6log = "";
    double x_init[6] = { v_.x, v_.y, v_.z, w_.x, w_.y, w_.z };//массив содержащий начальные приближения x0, x1,... которые будут переданы в функцию rosenbrock_f
    gsl_set_error_handler_off();//выключает крах программы при ошибках
    six returnstr;
    snl6iter = 0;                       //size_t счетчик итераций
    int status;                         //здесь будет храниться статус решателя, переменная может быть int т.к. статус это enum
    const size_t n = 6;                 //это порядок системы уравнений
    const gsl_multiroot_fsolver_type *T;//обьявляется указатель на обьект определяющий тип рещателя который мы будем использовать
    gsl_multiroot_fsolver *sol6;        //обьявляется указатель на обьект самого решателя
    struct six p = {0};                 //обьявляется переменная p структуры rparams, но она будет нулевым массивом потому что в уравнениях константы забиты вручную
    gsl_multiroot_function fstr;
    fstr.f = &system6nlf;//
    fstr.n = 6;
    fstr.params = &p;
    gsl_vector* X = gsl_vector_alloc(n);//здесь выделяется память под вектор размером n - порядок системы. как раз он и будет содержать в себе текущие для решателя значения x0,x1... сначала в него будут записаны значения с массива x_init, а потом решатель сам будет их итерационно подбирать и перезаписывать
    gsl_vector_set (X, 0, x_init[0]);   //как раз здесь в вектор x и записываются начальные приближения из x_init
    gsl_vector_set (X, 1, x_init[1]);
    gsl_vector_set (X, 2, x_init[2]);
    gsl_vector_set (X, 3, x_init[3]);
    gsl_vector_set (X, 4, x_init[4]);
    gsl_vector_set (X, 5, x_init[5]);
    T = gsl_multiroot_fsolver_broyden;//gsl_multiroot_fsolver_hybrids  //ранее обьявленному указателю на тип решателя присваевается адрес константы определяющей что это будет гибридный решатель, то есть gsl_multiroot_fsolver_hybrids
    sol6 = gsl_multiroot_fsolver_alloc(T,n);//создается в памяти сам решатель принимает переменную T говорящую какой тип решения он должен выполнять и порядок системмы уравнений которую он будет решать
    gsl_multiroot_fsolver_set(sol6, &fstr, X);//эта функция глобальна для GSL и она привязывает определенному решателю (у нас sol6) ссылку на структуру fstr которая связывает функцию системы уравнений, порядок системы и ссылку на структуру с константами {&rosenbrock_f,n,&p} и также привязывает вектор с значениями аргументов системы, которые в процессе итераций будут подбираться и перезаписываться в этом же векторе

    do//цикл итераций
    {
        snl6iter++; //прибавляется счетчик итераций
        if(snl6iter%100==0) QCoreApplication::processEvents();
        status = gsl_multiroot_fsolver_iterate(sol6);//вызывается глобальная функция которая заставляет определенный решатель (у нас он зовется sol6) выполнить одну итерацию //результат итерации записывается в переменную status
        status = gsl_multiroot_test_residual(sol6->f, snlf6prc);//1e-4 //если цикл не прерван и все уравнения посчитаны то вызовем глобальную функцию которая проверит меньше ли значения уравнений системы чем заданная точность (здесь точность 1е-9),
        //доступ к значениям уравнений функция gsl_multiroot_test_residual получает через вектор f который находится в решателе sol6. результат проверки на сходимость записывается в переменную status
    }
    while(status==GSL_CONTINUE && snl6iter<snlf6lim);//<100 //если проверка на сходимость не выполнена то в статусе будет храниться код GSL_CONTINUE и итерации продолжаются если их не больше 1000

    if(status)//проверка успешно ли выполнена итерация, может системма не считается при текущих подобранных аргументах, что-то типа деления на ноль, тогда прерываем цикл итераций
    {
        snl6log = snl2log;
        snl6log += "snl6solver: ";
        switch(status)
        {
            case GSL_CONTINUE :
                warning = 6;
                snl6log += "iteration has not converged, ";
                snl6log += "x6={";
                snl6log += ss.setNum(gsl_vector_get(sol6->x,0),'f',6) + ",";
                snl6log += ss.setNum(gsl_vector_get(sol6->x,1),'f',6) + ",";
                snl6log += ss.setNum(gsl_vector_get(sol6->x,2),'f',6) + ",";
                snl6log += ss.setNum(gsl_vector_get(sol6->x,3),'f',6) + ",";
                snl6log += ss.setNum(gsl_vector_get(sol6->x,4),'f',6) + ",";
                snl6log += ss.setNum(gsl_vector_get(sol6->x,5),'f',6) ;
                snl6log += "}, y6={";
                snl6log += ss.setNum(gsl_vector_get(sol6->f,0),'f',6) + ",";
                snl6log += ss.setNum(gsl_vector_get(sol6->f,1),'f',6) + ",";
                snl6log += ss.setNum(gsl_vector_get(sol6->f,2),'f',6) + ",";
                snl6log += ss.setNum(gsl_vector_get(sol6->f,3),'f',6) + ",";
                snl6log += ss.setNum(gsl_vector_get(sol6->f,4),'f',6) + ",";
                snl6log += ss.setNum(gsl_vector_get(sol6->f,5),'f',6) ;
                snl6log += "}";
            break;
            case GSL_EDOM     : snl6log += "Error: input domain error, e.g. sqrt(-1)                    "; break;
            case GSL_ERANGE   : snl6log += "Error: output range error, e.g. exp(1e100)                  "; break;
            case GSL_EFAULT   : snl6log += "Error: invalid pointer                                      "; break;
            case GSL_EINVAL   : snl6log += "Error: invalid argument supplied by user                    "; break;
            case GSL_EFAILED  : snl6log += "Error: generic failure                                      "; break;
            case GSL_EFACTOR  : snl6log += "Error: factorization failed                                 "; break;
            case GSL_ESANITY  : snl6log += "Error: sanity check failed - shouldn't happen               "; break;
            case GSL_ENOMEM   : snl6log += "Error: malloc failed                                        "; break;
            case GSL_EBADFUNC : snl6log += "Error: problem with user-supplied function                  "; break;
            case GSL_ERUNAWAY : snl6log += "Error: iterative process is out of control                  "; break;
            case GSL_EMAXITER : snl6log += "Error: exceeded max number of iterations                    "; break;
            case GSL_EZERODIV : snl6log += "Error: tried to divide by zero                              "; break;
            case GSL_EBADTOL  : snl6log += "Error: user specified an invalid tolerance                  "; break;
            case GSL_ETOL     : snl6log += "Error: failed to reach the specified tolerance              "; break;
            case GSL_EUNDRFLW : snl6log += "Error: underflow                                            "; break;
            case GSL_EOVRFLW  : snl6log += "Error: overflow                                             "; break;
            case GSL_ELOSS    : snl6log += "Error: loss of accuracy                                     "; break;
            case GSL_EROUND   : snl6log += "Error: failed because of roundoff error                     "; break;
            case GSL_EBADLEN  : snl6log += "Error: matrix, vector lengths are not conformant            "; break;
            case GSL_ENOTSQR  : snl6log += "Error: matrix not square                                    "; break;
            case GSL_ESING    : snl6log += "Error: apparent singularity detected                        "; break;
            case GSL_EDIVERGE : snl6log += "Error: integral or series is divergent                      "; break;
            case GSL_EUNSUP   : snl6log += "Error: requested feature is not supported by the hardware   "; break;
            case GSL_EUNIMPL  : snl6log += "Error: requested feature not (yet) implemented              "; break;
            case GSL_ECACHE   : snl6log += "Error: cache limit exceeded                                 "; break;
            case GSL_ETABLE   : snl6log += "Error: table limit exceeded                                 "; break;
            case GSL_ENOPROG  : snl6log += "Error: iteration is not making progress towards solution    "; break;
            case GSL_ENOPROGJ : snl6log += "Error: jacobian evaluations are not improving the solution  "; break;
            case GSL_ETOLF    : snl6log += "Error: cannot reach the specified tolerance in F            "; break;
            case GSL_ETOLX    : snl6log += "Error: cannot reach the specified tolerance in X            "; break;
            case GSL_ETOLG    : snl6log += "Error: cannot reach the specified tolerance in gradient     "; break;
            case GSL_EOF      : snl6log += "Error: end of file                                          "; break;
            default : snl6log += "Error!";
        }
        if(status!=GSL_CONTINUE) log_ogl += snl6log;
    }
    /*else
      if(logkey)
        {
            snl6log = "\nsnl6solver: ";//   + ss.setNum(snl6iter)
            snl6log += "x6={";
            snl6log += ss.setNum(gsl_vector_get(sol6->x,0),'f',6) + ",";
            snl6log += ss.setNum(gsl_vector_get(sol6->x,1),'f',6) + ",";
            snl6log += ss.setNum(gsl_vector_get(sol6->x,2),'f',6) + ",";
            snl6log += ss.setNum(gsl_vector_get(sol6->x,3),'f',6) + ",";
            snl6log += ss.setNum(gsl_vector_get(sol6->x,4),'f',6) + ",";
            snl6log += ss.setNum(gsl_vector_get(sol6->x,5),'f',6) ;
            snl6log += "}, y6={";
            snl6log += ss.setNum(gsl_vector_get(sol6->f,0),'f',6) + ",";
            snl6log += ss.setNum(gsl_vector_get(sol6->f,1),'f',6) + ",";
            snl6log += ss.setNum(gsl_vector_get(sol6->f,2),'f',6) + ",";
            snl6log += ss.setNum(gsl_vector_get(sol6->f,3),'f',6) + ",";
            snl6log += ss.setNum(gsl_vector_get(sol6->f,4),'f',6) + ",";
            snl6log += ss.setNum(gsl_vector_get(sol6->f,5),'f',6) ;
            snl6log += "}";
            //*logstream << snl6log;
        }*/
    //RETURN:
    returnstr.a = gsl_vector_get(sol6->x, 0);
    returnstr.b = gsl_vector_get(sol6->x, 1);
    returnstr.c = gsl_vector_get(sol6->x, 2);
    returnstr.d = gsl_vector_get(sol6->x, 3);
    returnstr.e = gsl_vector_get(sol6->x, 4);
    returnstr.f = gsl_vector_get(sol6->x, 5);
    //удаляем обьект решателя
    gsl_multiroot_fsolver_free(sol6);
    //gsl_vector_free (x);        //удаляем вектор с подобранными аргументами x, а можно его и не удалять а использовать решение где-то еще )))
return returnstr;
}
int  Helicopter::system6nlf (const gsl_vector *x, void *params, gsl_vector *f) //описание системы нелинейных уравнений
{
  //вытаскиваются константы уравнения из структуры rparams и присваиваются локальным переменным
  //double a = ((struct rparams *) params)->a;
  //double b = ((struct rparams *) params)->b;
  //локальным переменным x0 и x1 присваиваются значения из вектора аргументов
  //первое присвоение вытаскивает из вектора значение начального приближения, а при последующих в векторе хранятся значения которые "подбирает" итерационный решатель и "подобранные" решателем
  //значение искомых переменных присваиваются локальным переменным y0 и y1 присваиваются значения функций соответствующие аргументам текущих x0 и x1
  //считаются y0 и y1 с помощью функций определенных выше главное чтобы они могли вернуть значение каждого уравнения системмы в заданных аргументах x0 и x1
    const double x0 = gsl_vector_get (x, 0);
    const double x1 = gsl_vector_get (x, 1);
    const double x2 = gsl_vector_get (x, 2);
    const double x3 = gsl_vector_get (x, 3);
    const double x4 = gsl_vector_get (x, 4);
    const double x5 = gsl_vector_get (x, 5);
    phc->HelicopterDynamic(x0,x1,x2,x3,x4,x5);
    const double y0 = phc->f_Fx();//x0,x1,x2,x3
    const double y1 = phc->f_Fy();//x0,x1,x2,x3
    const double y2 = phc->f_Fz();//x0,x1,x2,x3
    const double y3 = phc->f_Mx();//x0,x1,x2,x3
    const double y4 = phc->f_My();//x0,x1,x2,x3
    const double y5 = phc->f_Mz();//x0,x1,x2,x3
  //в вектор f который принимает функция rosenbrock_f записываются расчитанные значения уравнения системы
  //которые в последующем использует итерационный решатель для проверки на сходимость (когда сходится решение значения должны быть нулевыми)
    gsl_vector_set(f, 0, y0);
    gsl_vector_set(f, 1, y1);
    gsl_vector_set(f, 2, y2);
    gsl_vector_set(f, 3, y3);
    gsl_vector_set(f, 4, y4);
    gsl_vector_set(f, 5, y5);
return GSL_SUCCESS;//возврат итерационному решателю сообщения о том что функция rosenbrock_f выполнилась успешно, тоесть она "считабельна" при данных аргументах x0 и x1
}



//решение системы 2 нелинейных уравнений
duo  Helicopter::SolveSystemNL2()
{
    snl2log = "";
    double x_init[2] = { org_.k, org_.t };  //массив содержащий начальные приближения x0, x1,... которые будут переданы в функцию rosenbrock_f
    gsl_set_error_handler_off();//выключает крах программы при ошибках
    duo returnstr;
    const size_t n = 2;         //это порядок системы уравнений
    const gsl_multiroot_fsolver_type *T;//обьявляется указатель на обьект определяющий тип рещателя который мы будем использовать
    gsl_multiroot_fsolver *sol2;//обьявляется указатель на обьект самого решателя
    int status;                 //здесь будет храниться статус решателя, переменная может быть int т.к. статус это enum
    snl2iter = 0;            //size_t счетчик итераций
    struct duo p = {0};//обьявляется переменная p структуры rparams, но она будет нулевым массивом потому что в уравнениях константы забиты вручную
    gsl_multiroot_function fstr2;
    fstr2.f = &system2nlf;
    fstr2.n = 2;
    fstr2.params = &p;
    gsl_vector* X = gsl_vector_alloc(n);//здесь выделяется память под вектор размером n - порядок системы. как раз он и будет содержать в себе текущие для решателя значения x0,x1... сначала в него будут записаны значения с массива x_init, а потом решатель сам будет их итерационно подбирать и перезаписывать
    gsl_vector_set (X, 0, x_init[0]);   //как раз здесь в вектор x и записываются начальные приближения из x_init
    gsl_vector_set (X, 1, x_init[1]);
    T = gsl_multiroot_fsolver_broyden;//gsl_multiroot_fsolver_hybrids  //ранее обьявленному указателю на тип решателя присваевается адрес константы определяющей что это будет гибридный решатель, то есть gsl_multiroot_fsolver_hybrids
    sol2 = gsl_multiroot_fsolver_alloc(T,n);//создается в памяти сам решатель принимает переменную T говорящую какой тип решения он должен выполнять и порядок системмы уравнений которую он будет решать
    gsl_multiroot_fsolver_set(sol2, &fstr2, X);//эта функция глобальна для GSL и она привязывает определенному решателю (у нас sol2) ссылку на структуру fstr которая связывает функцию системы уравнений, порядок системы и ссылку на структуру с константами {&rosenbrock_f,n,&p} и также привязывает вектор с значениями аргументов системы, которые в процессе итераций будут подбираться и перезаписываться в этом же векторе

    do//цикл итераций
    {
        snl2iter++; //прибавляется счетчик итераций        //if(snl2iter%1000==0) QCoreApplication::processEvents();
        status = gsl_multiroot_fsolver_iterate(sol2);//вызывается глобальная функция которая заставляет определенный решатель (у нас он зовется s) выполнить одну итерацию //результат итерации записывается в переменную status
        status = gsl_multiroot_test_residual(sol2->f, snlf2prc);//1e-4 //если цикл не прерван и все уравнения посчитаны то вызовем глобальную функцию которая проверит меньше ли значения уравнений системы чем заданная точность (здесь точность 1е-9),
        //доступ к значениям уравнений функция gsl_multiroot_test_residual получает через вектор f который находится в решателе s. результат проверки на сходимость записывается в переменную status
    }
    while(status==GSL_CONTINUE && snl2iter<snlf2lim);//<10//если проверка на сходимость не выполнена то в статусе будет храниться код GSL_CONTINUE и итерации продолжаются если их не больше 1000
    //RETURN:
    returnstr.a = gsl_vector_get(sol2->x, 0);
    returnstr.b = gsl_vector_get(sol2->x, 1);
    if(status)//проверка успешно ли выполнена итерация, может системма не считается при текущих подобранных аргументах, что-то типа деления на ноль, тогда прерываем цикл итераций
    {
        snl2log = "\nsnl2solver: ";
        switch(status)
        {
            case GSL_CONTINUE :
                warning = 2;
                snl2log += "iteration has not converged, ";
                snl2log += "x2={";//
                snl2log += ss.setNum(gsl_vector_get(sol2->x,0),'f',6) + ",";
                snl2log += ss.setNum(gsl_vector_get(sol2->x,1),'f',6) ;
                snl2log += "}, y2={";//
                snl2log += ss.setNum(gsl_vector_get(sol2->f,0),'f',6) + ",";
                snl2log += ss.setNum(gsl_vector_get(sol2->f,1),'f',6) ;
                snl2log += "}";
            break;
            case GSL_EDOM     : snl2log += "Error: input domain error, e.g sqrt(-1)                     "; break;
            case GSL_ERANGE   : snl2log += "Error: output range error, e.g. exp(1e100)                  "; break;
            case GSL_EFAULT   : snl2log += "Error: invalid pointer                                      "; break;
            case GSL_EINVAL   : snl2log += "Error: invalid argument supplied by user                    "; break;
            case GSL_EFAILED  : snl2log += "Error: generic failure                                      "; break;
            case GSL_EFACTOR  : snl2log += "Error: factorization failed                                 "; break;
            case GSL_ESANITY  : snl2log += "Error: sanity check failed - shouldn't happen               "; break;
            case GSL_ENOMEM   : snl2log += "Error: malloc failed                                        "; break;
            case GSL_EBADFUNC : snl2log += "Error: problem with user-supplied function                  "; break;
            case GSL_ERUNAWAY : snl2log += "Error: iterative process is out of control                  "; break;
            case GSL_EMAXITER : snl2log += "Error: exceeded max number of iterations                    "; break;
            case GSL_EZERODIV : snl2log += "Error: tried to divide by zero                              "; break;
            case GSL_EBADTOL  : snl2log += "Error: user specified an invalid tolerance                  "; break;
            case GSL_ETOL     : snl2log += "Error: failed to reach the specified tolerance              "; break;
            case GSL_EUNDRFLW : snl2log += "Error: underflow                                            "; break;
            case GSL_EOVRFLW  : snl2log += "Error: overflow                                             "; break;
            case GSL_ELOSS    : snl2log += "Error: loss of accuracy                                     "; break;
            case GSL_EROUND   : snl2log += "Error: failed because of roundoff error                     "; break;
            case GSL_EBADLEN  : snl2log += "Error: matrix, vector lengths are not conformant            "; break;
            case GSL_ENOTSQR  : snl2log += "Error: matrix not square                                    "; break;
            case GSL_ESING    : snl2log += "Error: apparent singularity detected                        "; break;
            case GSL_EDIVERGE : snl2log += "Error: integral or series is divergent                      "; break;
            case GSL_EUNSUP   : snl2log += "Error: requested feature is not supported by the hardware   "; break;
            case GSL_EUNIMPL  : snl2log += "Error: requested feature not (yet) implemented              "; break;
            case GSL_ECACHE   : snl2log += "Error: cache limit exceeded                                 "; break;
            case GSL_ETABLE   : snl2log += "Error: table limit exceeded                                 "; break;
            case GSL_ENOPROG  : snl2log += "Error: iteration is not making progress towards solution    "; break;
            case GSL_ENOPROGJ : snl2log += "Error: jacobian evaluations are not improving the solution  "; break;
            case GSL_ETOLF    : snl2log += "Error: cannot reach the specified tolerance in F            "; break;
            case GSL_ETOLX    : snl2log += "Error: cannot reach the specified tolerance in X            "; break;
            case GSL_ETOLG    : snl2log += "Error: cannot reach the specified tolerance in gradient     "; break;
            case GSL_EOF      : snl2log += "Error: end of file                                          "; break;
            default : snl2log += "Error!";
        }
        if(status!=GSL_CONTINUE) log_ogl += snl2log;
        //solvefail = true;
    }
    //удаляем обьект решателя
    gsl_multiroot_fsolver_free(sol2);
    //gsl_vector_free (x);        //удаляем вектор с подобранными аргументами x, а можно его и не удалять а использовать решение где-то еще )))
return returnstr;
}
int  Helicopter::system2nlf(const gsl_vector *x, void *params, gsl_vector *f) //описание системы нелинейных уравнений
{
    const double x0 = gsl_vector_get(x, 0);
    const double x1 = gsl_vector_get(x, 1);

    phc->RecalcAngles(x0,x1);

    const double y0 = phc->f_dkren();
    const double y1 = phc->f_dtang();

    gsl_vector_set(f, 0, y0);
    gsl_vector_set(f, 1, y1);
return GSL_SUCCESS;
}



