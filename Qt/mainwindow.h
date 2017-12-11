#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "masterthread.h"
#include "qcustomplot.h"
#include "aboutme.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:
    void  closeEvent(QCloseEvent*);

private slots:
    void connectGetConfigButtonCB();
    void uploadConfigButtonCB();
    void getDataButtonCB();
    void saveDataButtonCB();
    void aboutActionCB();
    void updatePortCB();
    void modifySampleRateCB();
    void wThreadStr(QString);
    void wThreadBin(QByteArray);

private slots:
    /* used to communicate with masterthread */
    void answerHandler(const QByteArray &s);
    void errorHandler(const QString &s);
    void timeoutHandler(const QString &s);
    void downloadHandler(int d);

private:
    void setupPlot(QCustomPlot *customPlot,
                   QVector<double> &t,
                   QVector<double> &x,
                   QVector<double> &y,
                   QVector<double> &z);
    void fillConfigurationUI(bool enable);
    void goToDisconnectedState();
    double totalTimeCalculate(unsigned int sampleSpeed);
    QString arrayPrint(int16_t *vector, char sensor);
    void writeToConsole(QString type, QString msg);

private:
    Ui::MainWindow *ui;
    enum STATES {DISCONNECTED,
                 CONNECTED_IDLE,
                 GET_HELLO_CONFIG,
                 GET_DATA,
                 SEND_UPLOAD_COMMAND,
                 UPLOADING_CONFIG} state;
    const unsigned int totalSize = 128 * 1024;
    const double gravity = 9.81;
    const double MPUscaleFactor = 1/16384. * gravity;
    const double ADXLscaleFactor =  1/256. * gravity;
    MasterThread thread;
    QVector<double> MPUt, MPUx, MPUy, MPUz;
    QVector<double> ADXLt, ADXLx, ADXLy, ADXLz;
    struct configStructure {
        int16_t calibrationMPU[3];
        int16_t calibrationADXL[3];
        uint16_t magicNumber;
        uint16_t samplingSpeed;
        uint8_t accelerometerRange;
        uint8_t calibrationDelay;
        uint8_t acquisitionDelay;
        bool gyroActivatedMPU;
    } configVariables;
};

#endif // MAINWINDOW_H
