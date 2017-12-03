#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "masterthread.h"
#include "qcustomplot.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void connectGetConfigButtonCB();
    void getDataButtonCB();
    void saveDataButtonCB();
    void wThread(QString);
    void showError(QString error, QString s);

private slots:
    /* used to communicate with masterthread */
    void answerHandler(const QByteArray &s);
    void errorHandler(const QString &s);
    void timeoutHandler(const QString &s);
    void downloadHandler(int d);

private:
    Ui::MainWindow *ui;
    enum STATES {IDLE, GET_HELLO_CONFIG, GET_DATA, SAVE_CONFIG} state;
    void setupPlot(QCustomPlot *customPlot,
                   QVector<double> &t,
                   QVector<double> &x,
                   QVector<double> &y,
                   QVector<double> &z);
    void fillConfigurationUI(bool enable);
    double totalTimeCalculate(unsigned int sampleSpeed);
    QString arrayPrint(int16_t *vector);

private:
    const unsigned int totalSize = 128 * 1024;
    bool isConnected;
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
