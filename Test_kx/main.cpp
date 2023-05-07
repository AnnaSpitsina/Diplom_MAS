#include <QCoreApplication>
#include <QApplication>
#include "kx_pult/kx_protocol.h"
#include "kx_pult/qkx_coeffs.h"
#include "high_level.h"
#include "cs_rov.h"
#include "trajectory.h"

double X[2000][2];

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    QApplication b(argc, argv);
    //передача K
    Qkx_coeffs* kProtocol = new Qkx_coeffs(ConfigFile, KI);
    //передача X
    x_protocol* xProtocol = new x_protocol(ConfigFile, XI, X);
    HIGH_LEVEL high_level(2);
    return a.exec();
    return b.exec();

}