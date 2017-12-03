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
            &MainWindow::connectGetConfigButtonCB);
    connect(ui->getDataPushButton,
            &QPushButton::clicked, this,
            &MainWindow::getDataButtonCB);
    connect(ui->saveAPushButton,
            &QPushButton::clicked, this,
            &MainWindow::saveDataButtonCB);
    connect(ui->saveBPushButton,
            &QPushButton::clicked, this,
            &MainWindow::saveDataButtonCB);

   ui->serialPortComboBox->setFocus();

   fillConfigurationUI(false);

   isConnected = false;
   state = IDLE;
}

void MainWindow::connectGetConfigButtonCB()
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

void MainWindow::getDataButtonCB()
{
   if (state != IDLE) {
       showError("NO IDLE", nullptr);
       return;
   }
   /* Disable all buttons, these will be configured when the answer comes */
   ui->getDataPushButton->setEnabled(false);
   ui->connectGetConfPushButton->setEnabled(false);
   ui->saveAPushButton->setEnabled(false);
   ui->saveBPushButton->setEnabled(false);
   wThread("r");
   ui->connectionTextBrowser->append("START DOWNLOAD");
   state = GET_DATA;
}

void MainWindow::saveDataButtonCB()
{
    char origin = '0';
    QObject* obj = sender();

    /* Get the button that called this function and check available data */
    if (obj == ui->saveAPushButton) {
        origin = 'A';
        if (MPUt.size() == 0) {
            QMessageBox::information(
                        this,
                        tr("Acceleration Data Logger"),
                        tr("No hay datos en el sensor A. Ha realizado la descarga?") );
            return;
        }
    }
    else if (obj == ui->saveBPushButton) {
        origin = 'B';
        if (ADXLt.size() == 0) {
            QMessageBox::information(
                        this,
                        tr("Acceleration Data Logger"),
                        tr("No hay datos en el sensor B. Ha realizado la descarga?") );
            return;
        }

    }

    /* Open new file to write */
    QString filename = QFileDialog::getSaveFileName(this, tr("Guardar archivo"),
                                                    tr("%1/data-%2%3")
                                                    .arg(QStandardPaths::standardLocations(QStandardPaths::DesktopLocation).last())
                                                    .arg(origin)
                                                    .arg(".csv"),
                                                    tr("Data (*.csv *.txt)"));
    if (filename == ""){
        qDebug()<<"Nombre de archivo vacio";
        return;
    }

    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::information(0, "error", file.errorString());
        return;
    }

    QTextStream out(&file);

    if (origin == 'A') {
        for (int i = 0; i < MPUt.size(); i++) {
             out << MPUt.at(i) << ","
                 << MPUx.at(i) << ","
                 << MPUy.at(i) << ","
                 << MPUz.at(i) << "\n";
        }
    }
    else if (origin == 'B') {
        for (int i = 0; i < ADXLt.size(); i++) {
             out << ADXLt.at(i) << ","
                 << ADXLx.at(i) << ","
                 << ADXLy.at(i) << ","
                 << ADXLz.at(i) << "\n";
        }
    }

    ui->connectionTextBrowser->append(tr("%1 %2").arg("Datos guardados en ").arg(filename));
}

void MainWindow::answerHandler(const QByteArray &s)
{
    uint8_t *glissant = (uint8_t *)s.constData();

    unsigned char id;
    int16_t v1, v2, v3;
    uint8_t *v1p, *v2p, *v3p;

    long i;

    switch(state) {
    case GET_HELLO:
        /* Copy device configuration to local structure and check magicNumber */
        memcpy(&configVariables, (struct configStructure *)glissant, sizeof(struct configStructure));
        if (configVariables.magicNumber != 0xADDA)
            return;
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
            /* The mapping cannot be done using a struct, because of alignment issues */
            v1p = glissant + 1;
            v2p = glissant + 3;
            v3p = glissant + 5;
            v1 = *((int16_t *)v1p);
            v2 = *((int16_t *)v2p);
            v3 = *((int16_t *)v3p);

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
             glissant += 7;
        }
        setupPlot(ui->s1QCustomPlot, MPUt, MPUx, MPUy, MPUz);
        setupPlot(ui->s2QCustomPlot, ADXLt, ADXLx, ADXLy, ADXLz);
        ui->getDataPushButton->setEnabled(true);
        ui->connectGetConfPushButton->setEnabled(true);
        ui->saveAPushButton->setEnabled(true);
        ui->saveBPushButton->setEnabled(true);
        break;

    case GET_CONFIG:
    case SAVE_CONFIG:
    case IDLE:
        break;
    }
    /* After handling the answer, we go idle */
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
    ui->saveAPushButton->setEnabled(enable);
    ui->saveBPushButton->setEnabled(enable);

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
