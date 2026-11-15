#include <QCoreApplication>
#include "mainprocess_.cpp"

int main(int argc, char *argv[])
{
    titleapplname = "HeliSolve.2019.03";
    singleline="-------------------------------------------------------------------------------";
    setlocale(0,"");//setlocale(LC_ALL,"ru_RU.utf-8");
    SetConsoleOutputCP(CP_WINUNICODE);//1251
    SetConsoleCP(CP_WINUNICODE);//866
    QCoreApplication a(argc, argv);
    if (qApp->arguments().count() == 2)
    {//запуск внешней программой
        infilename  = qApp->arguments().at(1);
    }
    else
    {//запуск из консоли
        std::string ar1;
        std::cout << "Version: ";  std::cout << titleapplname.toStdString();  std::cout << "\n";
        while(1)
        {
            std::cout << "-------------------------------------------------------------------------------\n";
            std::cout << "Initial: ";
            std::cin >> ar1;
            infilename = QString::fromStdString(ar1);
            if(Pre()) MainProcess();
        }
    }
    //if (MainProcess()) return a.exec();  else return 0;
return a.exec();
}


