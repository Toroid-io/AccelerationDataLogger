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
    void connectGetConfig();
    void getData();
    void wThread(QString);
    void showError(QString error, QString s);

private slots:
    void answerHandler(const QByteArray &s);
    void errorHandler(const QString &s);
    void timeoutHandler(const QString &s);


private:
    Ui::MainWindow *ui;
    enum STATES {IDLE, GET_HELLO, GET_DATA, GET_CONFIG, SAVE_CONFIG} state;
    void setupPlot(QCustomPlot *customPlot,
                   QVector<double> &t,
                   QVector<double> &x,
                   QVector<double> &y,
                   QVector<double> &z);
    void fillConfigurationUI(bool enable);
    double totalTimeCalculate(unsigned int sampleSpeed);

private:
    /* totalSamples = 128 * 1024 / 14
     * 14 is the number of bytes required for two sensors
     */
    const unsigned int totalSamples = 9362;
    bool isConnected;
    MasterThread thread;
    QVector<double> MPUt, MPUx, MPUy, MPUz;
    QVector<double> ADXLt, ADXLx, ADXLy, ADXLz;
    struct ConfigVariables {
        int calibrationMPU[3];
        int calibrationADXL[3];
        unsigned int magicNumber;
        unsigned int samplingSpeed;
        unsigned int accelerometerRange;
        unsigned int calibrationDelay;
        unsigned int acquisitionDelay;
    } configVariables;
};

#endif // MAINWINDOW_H
