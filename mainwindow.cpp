/* TODO
 * Variable acceleration range (currently fixed)
 * Variable sample time (currently fixed)
 * Variable calibration and acquisition delay
 */

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QLabel>
#include <QLineEdit>
#include <QComboBox>
#include <QSpinBox>
#include <QPushButton>
#include <QGridLayout>
#include <QShortcut>
#include <QDebug>
#include <QTime>
#include <QFile>
#include <QStringList>

#include <QtSerialPort/QSerialPortInfo>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle(tr("Acceleration Data Logger"));

    /* Close window shortcut */
    new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_Q), this, SLOT(close()));

    /* Set available ports in list */
    const auto infos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : infos)
        ui->serialPortComboBox->addItem(info.portName());

    /* Serial thread callbacks */
    connect(&thread,
            &MasterThread::response, this,
            &MainWindow::answerHandler);
    connect(&thread,
            &MasterThread::error, this,
            &MainWindow::errorHandler);
    connect(&thread,
            &MasterThread::timeout, this,
            &MainWindow::timeoutHandler);

    /* UI actions */
    connect(ui->connectGetConfPushButton,
            &QPushButton::clicked, this,
            &MainWindow::connectGetConfig);
    connect(ui->getDataPushButton,
            &QPushButton::clicked, this,
            &MainWindow::getData);

   ui->serialPortComboBox->setFocus();

   fillConfigurationUI(false);

   isConnected = false;
   state = IDLE;
}

void MainWindow::connectGetConfig()
{
   if (state != IDLE) {
       showError("NO IDLE", nullptr);
       return;
   }
   if (isConnected) {
       /* Disconnect*/
       isConnected = false;
       fillConfigurationUI(false);
   } else {
       /* Ask connection parameters
        * Connection will be handled in answer callback
        */
       wThread("g");
       state = GET_HELLO;
   }
}

void MainWindow::getData()
{
   if (state != IDLE) {
       showError("NO IDLE", nullptr);
       return;
   }
   wThread("r");
   ui->connectionTextBrowser->append("START DOWNLOAD");
   state = GET_DATA;

}

void MainWindow::answerHandler(const QByteArray &s)
{
    QString ct = QTime::currentTime().toString();

    uint8_t *glissant = (uint8_t *)s.constData();

    unsigned char id;
    int16_t v1, v2, v3;
    uint8_t *v1p, *v2p, *v3p;

    /*
    QString filename = "/tmp/Data.txt";
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        return;
    QTextStream out(&file);
             out   << id << ": "
                      << v1 << ":"
                      << v2 << ": "
                      << v3 << ";\n";
                      */
    QString sString(s);
    QStringList strList;

    long i;

    switch(state) {
    case GET_HELLO:
        strList = sString.split(QRegularExpression("\r\n"),QString::SkipEmptyParts);
        configVariables.magicNumber = strList.at(0).toUInt(nullptr, 16);
        configVariables.samplingSpeed = strList.at(1).toUInt(nullptr, 10);
        configVariables.accelerometerRange = strList.at(2).toUInt(nullptr, 10);
        // 3 -> calibrationMPU
        // 4 -> calibrationADXL
        configVariables.calibrationDelay = strList.at(5).toUInt(nullptr, 10);
        configVariables.acquisitionDelay = strList.at(6).toUInt(nullptr, 10);

        if (configVariables.magicNumber != 0xADDA)
            return;

        /* Very dirty solution to avoid target locking */
        QThread::msleep(200);
        /* We have a valid device, proceed */
        isConnected = true;
        fillConfigurationUI(true);

        break;
    case GET_DATA:

        ui->connectionTextBrowser->append("DOWNLOAD FINISHED");

        MPUt.clear();
        MPUx.clear();
        MPUy.clear();
        MPUz.clear();

        ADXLt.clear();
        ADXLx.clear();
        ADXLy.clear();
        ADXLz.clear();

        for ( i = 0; i*7 < s.size(); ++i) {

            id = *((unsigned char *) glissant);
            v1p = glissant + 1;
            v2p = glissant + 3;
            v3p = glissant + 5;

            v1 = *((int16_t *)v1p);
            v2 = *((int16_t *)v2p);
            v3 = *((int16_t *)v3p);

             glissant += 7;

             if (id == 'M') {
                 MPUt.append(1/800. * i/2);
                 MPUx.append((double)v1/32768.*4);
                 MPUy.append((double)v2/32768.*4);
                 MPUz.append((double)v3/32768.*4);
             }
             if (id == 'A') {
                 ADXLt.append(1/800. * i/2);
                 ADXLx.append((double)v1);
                 ADXLy.append((double)v2);
                 ADXLz.append((double)v3);
             }

        }
        setupPlot(ui->s1QCustomPlot, MPUt, MPUx, MPUy, MPUz);
        setupPlot(ui->s2QCustomPlot, ADXLt, ADXLx, ADXLy, ADXLz);
        break;
    case GET_CONFIG:
    case IDLE:
    case SAVE_CONFIG:
        break;
    }
    state = IDLE;
}


void MainWindow::errorHandler(const QString &s)
{
    showError("ERROR", s);
    state = IDLE;
    return;
}

void MainWindow::timeoutHandler(const QString &s)
{
    showError("TIMEOUT", s);
    state = IDLE;
    return;
}

void MainWindow::wThread(QString command)
{
    QString ct = QTime::currentTime().toString();
    thread.transaction(ui->serialPortComboBox->currentText(),
                      30000, command);
    ui->connectionTextBrowser->append(tr("H: %1 - %2")
                                      .arg(ct)
                                      .arg(command));
}

void MainWindow::showError(QString error, QString s)
{
    QString ct = QTime::currentTime().toString();
    ui->connectionTextBrowser->append(tr("H: %1 - %2 %3")
                                      .arg(ct)
                                      .arg(error)
                                      .arg(s));

}

void MainWindow::fillConfigurationUI(bool enable)
{
    if (enable) {
        ui->calDelayValLabel->setText(QString::number(configVariables.calibrationDelay));
        ui->sampleDelayValLabel->setText(QString::number(configVariables.acquisitionDelay));
        ui->sampleTimeValLabel->setText(QString::number(totalTimeCalculate(configVariables.samplingSpeed)));
        ui->accelRangeValLabel->setText(QString::number(configVariables.accelerometerRange));
        ui->sampleSpeedValLabel->setText(QString::number(configVariables.samplingSpeed));
        ui->connectGetConfPushButton->setText("Desconectar");
    } else {
        ui->connectGetConfPushButton->setText("Conectar");
    }

    ui->accelRangeValLabel->setEnabled(enable);
    ui->calDelayValLabel->setEnabled(enable);
    ui->sampleDelayValLabel->setEnabled(enable);
    ui->sampleSpeedValLabel->setEnabled(enable);
    ui->sampleTimeValLabel->setEnabled(enable);

    ui->getDataPushButton->setEnabled(enable);
    /* No save config in current version (view TODO) */
    /* ui->saveConfigPushButton->setEnabled(enable); */
    ui->save1PushButton->setEnabled(enable);
    ui->save2PushButton->setEnabled(enable);

    ui->serialPortComboBox->setEnabled(!enable);
}

double MainWindow::totalTimeCalculate(unsigned int sampleSpeed)
{
    return totalSamples/((double)sampleSpeed);

}

void MainWindow::setupPlot(QCustomPlot *customPlot,
                           QVector<double> &t,
                           QVector<double> &x,
                           QVector<double> &y,
                           QVector<double> &z)
{
  // add two new graphs and set their look:
  customPlot->addGraph();
  customPlot->graph(0)->setPen(QPen(Qt::blue)); // line color blue for first graph
  //customPlot->graph(0)->setBrush(QBrush(QColor(0, 0, 255, 20))); // first graph will be filled with translucent blue
  customPlot->addGraph();
  customPlot->graph(1)->setPen(QPen(Qt::red)); // line color red for second graph
  customPlot->addGraph();
  customPlot->graph(2)->setPen(QPen(Qt::green)); // line color red for third graph
  // configure right and top axis to show ticks but no labels:
  // (see QCPAxisRect::setupFullAxesBox for a quicker method to do this)
  customPlot->xAxis2->setVisible(true);
  customPlot->xAxis2->setTickLabels(false);
  customPlot->yAxis2->setVisible(true);
  customPlot->yAxis2->setTickLabels(false);
  // make left and bottom axes always transfer their ranges to right and top axes:
  connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->xAxis2, SLOT(setRange(QCPRange)));
  connect(customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->yAxis2, SLOT(setRange(QCPRange)));
  // pass data points to graphs:
  customPlot->graph(0)->setData(t, x);
  customPlot->graph(1)->setData(t, y);
  customPlot->graph(2)->setData(t, z);
  // let the ranges scale themselves so graph 0 fits perfectly in the visible area:
  customPlot->graph(0)->rescaleAxes();
  // same thing for graph 1, but only enlarge ranges (in case graph 1 is smaller than graph 0):
  customPlot->graph(1)->rescaleAxes();
  customPlot->graph(2)->rescaleAxes(true);
  // Note: we could have also just called customPlot->rescaleAxes(); instead
  // Allow user to drag axis ranges with mouse, zoom with mouse wheel and select graphs by clicking:
  customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
}

MainWindow::~MainWindow()
{
    delete ui;
}
