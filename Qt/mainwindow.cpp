/* TODO
 *  - Continuously check USB conection
 *  - Variable acceleration range (currently fixed)
 *  - Get system configuration before downloading data (currently on connection)
 */

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "aboutme.h"

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

    connect(&thread,
            &MasterThread::downloaded, this,
            &MainWindow::downloadHandler);

    /* UI actions */
    connect(ui->connectGetConfPushButton,
            &QPushButton::clicked, this,
            &MainWindow::connectGetConfigButtonCB);
    connect(ui->uploadConfigPushButton,
            &QPushButton::clicked, this,
            &MainWindow::uploadConfigButtonCB);
    connect(ui->getDataPushButton,
            &QPushButton::clicked, this,
            &MainWindow::getDataButtonCB);
    connect(ui->saveMPUPushButton,
            &QPushButton::clicked, this,
            &MainWindow::saveDataButtonCB);
    connect(ui->saveADXLPushButton,
            &QPushButton::clicked, this,
            &MainWindow::saveDataButtonCB);
    connect(ui->updatePushButton,
            &QPushButton::clicked, this,
            &MainWindow::updatePortCB);
    connect(ui->aboutAction,
            &QAction::triggered, this,
            &MainWindow::aboutActionCB);
    connect(ui->closeAction,
            &QAction::triggered, this,
            &MainWindow::close);
    connect(ui->sampleRateComboBox,
            &QComboBox::currentTextChanged, this,
            &MainWindow::modifySampleRateCB);

    goToDisconnectedState();

    ui->serialPortComboBox->setFocus();

}

void MainWindow::connectGetConfigButtonCB()
{
   if (state == CONNECTED_IDLE) {
       /* Disconnect*/
       goToDisconnectedState();
   } else if (state == DISCONNECTED) {
       /* Ask connection parameters
        * Connection will be handled in answer callback
        */
       wThreadStr("g");
       state = GET_HELLO_CONFIG;
   } else {
        writeToConsole("ERROR", "State is not IDLE");
   }
}

void MainWindow::uploadConfigButtonCB()
{
    if (state != CONNECTED_IDLE) {
        writeToConsole("ERROR", "State is not IDLE");
        return;
    }
    wThreadStr("c");
    state = SEND_UPLOAD_COMMAND;
}

void MainWindow::getDataButtonCB()
{
   if (state != CONNECTED_IDLE) {
       writeToConsole("ERROR", "State is not IDLE");
       return;
   }
   /* Disable all buttons, these will be configured when the answer comes */
   wThreadStr("r");
   writeToConsole("INFO", "Start download");
   state = GET_DATA;
   fillConfigurationUI();
}

void MainWindow::saveDataButtonCB()
{
    char origin = '0';
    QObject* obj = sender();

    /* Get the button that called this function and check available data */
    if (obj == ui->saveMPUPushButton) {
        origin = 'M';
        if (MPUt.size() == 0) {
            QMessageBox::information(
                        this,
                        tr("Acceleration Data Logger"),
                        tr("No hay datos en el sensor MPU. Ha realizado la descarga?") );
            return;
        }
    }
    else if (obj == ui->saveADXLPushButton) {
        origin = 'A';
        if (ADXLt.size() == 0) {
            QMessageBox::information(
                        this,
                        tr("Acceleration Data Logger"),
                        tr("No hay datos en el sensor ADXL. Ha realizado la descarga?") );
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

    if (origin == 'M') {
        for (int i = 0; i < MPUt.size(); i++) {
             out << MPUt.at(i) << ","
                 << MPUx.at(i) << ","
                 << MPUy.at(i) << ","
                 << MPUz.at(i) << "\n";
        }
    }
    else if (origin == 'A') {
        for (int i = 0; i < ADXLt.size(); i++) {
             out << ADXLt.at(i) << ","
                 << ADXLx.at(i) << ","
                 << ADXLy.at(i) << ","
                 << ADXLz.at(i) << "\n";
        }
    }

    writeToConsole("INFO", tr("%1 %2").arg(filename).arg(" saved"));
}

void MainWindow::updatePortCB()
{
    goToDisconnectedState();
}

void MainWindow::modifySampleRateCB()
{
    double totalTime = totalTimeCalculate(ui->sampleRateComboBox->currentText().toInt());
    ui->sampleTimeValLabel->setText(QString::number(totalTime));
}

void MainWindow::aboutActionCB()
{
   AboutMe *aboutMe = new AboutMe();
   aboutMe->show();
}

void MainWindow::answerHandler(const QByteArray &s)
{
    uint8_t *glissant = (uint8_t *)s.constData();
    configStructure newConfigVariables;
    QByteArray newConfigByteArray;

    unsigned char id;
    int16_t v1, v2, v3;
    uint8_t *v1p, *v2p, *v3p;

    long i;

    switch(state) {
    case GET_HELLO_CONFIG:
        /* Copy device configuration to local structure and check magicNumber */
        memcpy(&configVariables, (struct configStructure *)glissant, sizeof(struct configStructure));
        if (configVariables.magicNumber != 0xADDA) {
            goToDisconnectedState();
            return;
        }
        /* We have a valid device, proceed */
        writeToConsole("INFO", "Connected");
        state = CONNECTED_IDLE;
        fillConfigurationUI();
        break;

    case GET_DATA:
        /* There is no data to be downloaded */
        if (strcmp(s, "--NO DATA--") == 0) {
            QMessageBox::information(
                        this,
                        tr("Acceleration Data Logger"),
                        tr("There is no data to download. Get some data and try again!"));
            state = CONNECTED_IDLE;
            fillConfigurationUI();
            break;
        }

        ui->downloadProgressBar->setValue(100);
        writeToConsole("INFO", "Download finished");

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
                MPUt.append(1/(double)configVariables.samplingSpeed * i/2);
                MPUx.append((double)v1 * MPUscaleFactor);
                MPUy.append((double)v2 * MPUscaleFactor);
                MPUz.append((double)v3 * MPUscaleFactor);
             }
             if (id == 'A') {
                 ADXLt.append(1/(double)configVariables.samplingSpeed * i/2);
                 ADXLx.append((double)v1 * ADXLscaleFactor);
                 ADXLy.append((double)v2 * ADXLscaleFactor);
                 ADXLz.append((double)v3 * ADXLscaleFactor);
             }
             glissant += 7;
        }
        setupPlot(ui->s1QCustomPlot, MPUt, MPUx, MPUy, MPUz);
        setupPlot(ui->s2QCustomPlot, ADXLt, ADXLx, ADXLy, ADXLz);
        state = CONNECTED_IDLE;
        fillConfigurationUI();
        break;

    case SEND_UPLOAD_COMMAND:
        memcpy(&newConfigVariables, &configVariables, sizeof(struct configStructure));
        newConfigVariables.acquisitionDelay = (uint16_t)ui->sampleDelaySpinBox->value();
        newConfigVariables.calibrationDelay = (uint16_t)ui->calibrationDelaySpinBox->value();
        newConfigVariables.samplingSpeed = ui->sampleRateComboBox->currentText().toInt(nullptr);
        newConfigVariables.filterType = ui->filterTypeComboBox->currentIndex();
        newConfigByteArray.setRawData((const char *)&newConfigVariables, sizeof(struct configStructure));
        wThreadBin(newConfigByteArray);
        state = UPLOADING_CONFIG;
        break;

    case UPLOADING_CONFIG:
        if (strcmp(s, "--Config OK--") != 0) {
            writeToConsole("ERORR", "Failed to upload new configuration");
            goToDisconnectedState();
            break;
        }
        writeToConsole("INFO", "Configuration Uploaded. Disconnecting.");
        /* Dirty hack, but it does the trick */
        goToDisconnectedState();
        break;

    case CONNECTED_IDLE:
    case DISCONNECTED:
        break;
    }
    /* After handling the answer, we go idle */
}

void MainWindow::downloadHandler(int d)
{
    ui->downloadProgressBar->setValue((int)((100*d)/(double)totalSize));
}


void MainWindow::errorHandler(const QString &s)
{
    writeToConsole("ERROR", s);
    goToDisconnectedState();
    return;
}

void MainWindow::timeoutHandler(const QString &s)
{
    writeToConsole("TIMEOUT", s);
    goToDisconnectedState();
    return;
}

void MainWindow::wThreadStr(QString commandStr)
{
    QByteArray command = commandStr.toLocal8Bit();
    wThreadBin(command);
    writeToConsole("SENT", commandStr);
}

void MainWindow::wThreadBin(QByteArray commandBin)
{
    thread.transaction(ui->serialPortComboBox->currentText(),
                      1000, commandBin);
}

void MainWindow::writeToConsole(QString type, QString msg)
{
    QString ct = QTime::currentTime().toString();
    ui->connectionTextBrowser->append(tr("H: %1 - %2 - %3")
                                      .arg(ct)
                                      .arg(type)
                                      .arg(msg));

}

void MainWindow::goToDisconnectedState()
{
    /* Set available ports in list */
    ui->serialPortComboBox->clear();
    const auto infos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : infos)
        ui->serialPortComboBox->addItem(info.portName());

    state = DISCONNECTED;
    fillConfigurationUI();
}

void MainWindow::fillConfigurationUI()
{
    bool enableButtons;
    switch (state) {
    case DISCONNECTED:
        ui->connectGetConfPushButton->setText("Connect");
        ui->serialPortComboBox->setEnabled(true);
        ui->updatePushButton->setEnabled(true);
        ui->downloadProgressBar->setValue(0);
        ui->downloadProgressBar->setEnabled(false);
        ui->connectGetConfPushButton->setEnabled(true);
        enableButtons = false;
        break;
    case CONNECTED_IDLE:
        ui->calibrationDelaySpinBox->setValue(configVariables.calibrationDelay);
        ui->sampleDelaySpinBox->setValue(configVariables.acquisitionDelay);
        ui->sampleTimeValLabel->setText(QString::number(totalTimeCalculate(configVariables.samplingSpeed)));
        ui->accelRangeValLabel->setText(QString::number(configVariables.accelerometerRange));
        ui->sampleRateComboBox->setCurrentText(QString::number(configVariables.samplingSpeed));
        ui->filterTypeComboBox->setCurrentIndex(configVariables.filterType);
        ui->offsetMPUValLabel->setText(arrayPrint(configVariables.calibrationMPU, 'M'));
        ui->offsetADXLValLabel->setText(arrayPrint(configVariables.calibrationADXL, 'A'));
        ui->connectGetConfPushButton->setText("Disconnect");
        ui->serialPortComboBox->setEnabled(false);
        ui->updatePushButton->setEnabled(false);
        ui->downloadProgressBar->setEnabled(true);
        ui->connectGetConfPushButton->setEnabled(true);
        enableButtons = true;
        break;
    case  GET_DATA:
        ui->downloadProgressBar->setValue(0);
        ui->downloadProgressBar->setEnabled(true);
        ui->serialPortComboBox->setEnabled(false);
        ui->updatePushButton->setEnabled(false);
        ui->connectGetConfPushButton->setEnabled(false);
        enableButtons = false;
        break;
    default:
        break;
    }
    ui->accelRangeValLabel->setEnabled(enableButtons);
    ui->calibrationDelaySpinBox->setEnabled(enableButtons);
    ui->sampleDelaySpinBox->setEnabled(enableButtons);
    ui->sampleRateComboBox->setEnabled(enableButtons);
    ui->filterTypeComboBox->setEnabled(enableButtons);
    ui->sampleTimeValLabel->setEnabled(enableButtons);
    ui->offsetMPUValLabel->setEnabled(enableButtons);
    ui->offsetADXLValLabel->setEnabled(enableButtons);
    ui->getDataPushButton->setEnabled(enableButtons);
    ui->saveMPUPushButton->setEnabled(enableButtons);
    ui->saveADXLPushButton->setEnabled(enableButtons);
    ui->uploadConfigPushButton->setEnabled(enableButtons);

}

double MainWindow::totalTimeCalculate(unsigned int sampleSpeed)
{
    return totalSize/14/((double)sampleSpeed);
}

QString MainWindow::arrayPrint(int16_t *vector, char sensor) {
    QString tmp;
    if (sensor == 'M')
        tmp = tr("[%1 %2 %3]")
                .arg(QString::number(vector[0] * MPUscaleFactor, 'g', 3))
                .arg(QString::number(vector[1] * MPUscaleFactor, 'g', 3))
                .arg(QString::number(vector[2] * MPUscaleFactor, 'g', 3));
    else if (sensor == 'A')
        tmp = tr("[%1 %2 %3]")
                .arg(QString::number(vector[0] * ADXLscaleFactor, 'g', 3))
                .arg(QString::number(vector[1] * ADXLscaleFactor, 'g', 3))
                .arg(QString::number(vector[2] * ADXLscaleFactor, 'g', 3));
    return tmp;
}

void MainWindow::setupPlot(QCustomPlot *customPlot,
                           QVector<double> &t,
                           QVector<double> &x,
                           QVector<double> &y,
                           QVector<double> &z)
{
    // add two new graphs and set their look:
    customPlot->clearGraphs();
    customPlot->legend->setVisible(true);
    customPlot->addGraph();
    customPlot->graph(0)->setPen(QPen(Qt::blue)); // line color blue for first graph
    customPlot->graph(0)->setName("x");
    //customPlot->graph(0)->setBrush(QBrush(QColor(0, 0, 255, 20))); // first graph will be filled with translucent blue
    customPlot->addGraph();
    customPlot->graph(1)->setPen(QPen(Qt::red)); // line color red for second graph
    customPlot->graph(1)->setName("y");
    customPlot->addGraph();
    customPlot->graph(2)->setPen(QPen(Qt::green)); // line color red for third graph
    customPlot->graph(2)->setName("z");
    // add some legends
    // configure right and top axis to show ticks but no labels:
    // (see QCPAxisRect::setupFullAxesBox for a quicker method to do this)
    customPlot->xAxis2->setVisible(true);
    customPlot->xAxis2->setTickLabels(false);
    customPlot->yAxis2->setVisible(true);
    customPlot->yAxis2->setTickLabels(false);
    // make left and bottom axes always transfer their ranges to right and top axes:
    connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->xAxis2, SLOT(setRange(QCPRange)));
    connect(customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->yAxis2, SLOT(setRange(QCPRange)));
    // set some labels
    customPlot->xAxis->setLabel("Time (s)");
    customPlot->yAxis->setLabel("Acceleration (m/s²)");
    // pass data points to graphs:
    customPlot->graph(0)->setData(t, x);
    customPlot->graph(1)->setData(t, y);
    customPlot->graph(2)->setData(t, z);
    customPlot->yAxis->setRange(-3 * 9.81, 3 * 9.81);
    customPlot->xAxis->setRange(0, totalTimeCalculate(configVariables.samplingSpeed));
    // Note: we could have also just called customPlot->rescaleAxes(); instead
    // Allow user to drag axis ranges with mouse, zoom with mouse wheel and select graphs by clicking:
    customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    customPlot->replot();
}

void   MainWindow::closeEvent(QCloseEvent*)
{
    qApp->quit();
}

MainWindow::~MainWindow()
{
    delete ui;
}
