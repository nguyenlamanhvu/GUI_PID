#include "mainwindow.h"
#include "ui_mainwindow.h"

QSerialPort * serialPort;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , serialPort(nullptr)
    , length(0)
    , stopPlot(true)
{
    ui->setupUi(this);
    // Adding title for widget
    QWidget::setWindowTitle("Serial Port Example");

    loadPorts();
    addMotorGraph();
}

MainWindow::~MainWindow()
{
    delete ui;
    if(serialPort != nullptr){
        serialPort->close();
        delete serialPort;
    }
}

void MainWindow::loadPorts()
{
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()){
        ui->cmbPorts->addItem(info.portName());
        qDebug() << info.portName();
    }
    // Baud Rate Ratios
    QList<qint32> baudRates = serialInfo->standardBaudRates(); // What baudrates does my computer support ?
    QList<QString> stringBaudRates;
    for(int i = 0 ; i < baudRates.size() ; i++){
        stringBaudRates.append(QString::number(baudRates.at(i)));
    }
    ui->cmbBaud->addItems(stringBaudRates);
    ui->cmbBaud->setCurrentText("9600");

    // Data Bits
    ui->cmbDataBits->addItem("5");
    ui->cmbDataBits->addItem("6");
    ui->cmbDataBits->addItem("7");
    ui->cmbDataBits->addItem("8");
    ui->cmbDataBits->setCurrentText("8");

    // Stop Bits
    ui->cmbStopBits->addItem("1 Bit");
    ui->cmbStopBits->addItem("1,5 Bits");
    ui->cmbStopBits->addItem("2 Bits");
    ui->cmbStopBits->setCurrentText("1 Bit");

    // Parities
    ui->cmbParity->addItem("No Parity");
    ui->cmbParity->addItem("Even Parity");
    ui->cmbParity->addItem("Odd Parity");
    ui->cmbParity->addItem("Mark Parity");
    ui->cmbParity->addItem("Space Parity");
    ui->cmbParity->setCurrentText("No Parity");

    //Flow Controls
    ui->cmbFlowControl->addItem("No Flow Control");
    ui->cmbFlowControl->addItem("Hardware Flow Control");
    ui->cmbFlowControl->addItem("Software Flow Control");
    ui->cmbFlowControl->setCurrentText("No Flow Control");
}

void MainWindow::on_btnOpenPort_clicked()
{
    if(serialPort != nullptr){
        serialPort->close();
        delete serialPort;
    }
    serialPort = new QSerialPort(this);
    serialPort->setPortName(ui->cmbPorts->currentText());

    // Baud Rate Ratios
    QString stringbaudRate = ui->cmbBaud->currentText();
    int intbaudRate = stringbaudRate.toInt();
    serialPort->setBaudRate(intbaudRate);

    // Data Bits
    QString dataBits = ui->cmbDataBits->currentText();
    if(dataBits == "5 Bits") {
       serialPort->setDataBits(QSerialPort::Data5);
    }
    else if((dataBits == "6 Bits")) {
       serialPort->setDataBits(QSerialPort::Data6);
    }
    else if(dataBits == "7 Bits") {
       serialPort->setDataBits(QSerialPort::Data7);
    }
    else if(dataBits == "8 Bits"){
       serialPort->setDataBits(QSerialPort::Data8);
    }

    // Stop Bits
    QString stopBits = ui->cmbStopBits->currentText();
    if(stopBits == "1 Bit") {
     serialPort->setStopBits(QSerialPort::OneStop);
    }
    else if(stopBits == "1,5 Bits") {
     serialPort->setStopBits(QSerialPort::OneAndHalfStop);
    }
    else if(stopBits == "2 Bits") {
     serialPort->setStopBits(QSerialPort::TwoStop);
    }

    // Parities
    QString parity = ui->cmbParity->currentText();
    if(parity == "No Parity"){
      serialPort->setParity(QSerialPort::NoParity);
    }
    else if(parity == "Even Parity"){
      serialPort->setParity(QSerialPort::EvenParity);
    }
    else if(parity == "Odd Parity"){
      serialPort->setParity(QSerialPort::OddParity);
    }
    else if(parity == "Mark Parity"){
      serialPort->setParity(QSerialPort::MarkParity);
    }
    else if(parity == "Space Parity") {
        serialPort->setParity(QSerialPort::SpaceParity);
    }

    //Flow Controls
    QString flowControl = ui->cmbFlowControl->currentText();
    if(flowControl == "No Flow Control") {
      serialPort->setFlowControl(QSerialPort::NoFlowControl);
    }
    else if(flowControl == "Hardware Flow Control") {
      serialPort->setFlowControl(QSerialPort::HardwareControl);
    }
    else if(flowControl == "Software Flow Control") {
      serialPort->setFlowControl(QSerialPort::SoftwareControl);
    }
    if(serialPort->open(QIODevice::ReadWrite)){
        QMessageBox::information(this,"Result","Port opened successfully.");
        QObject::connect(serialPort,&QSerialPort::readyRead,this,&MainWindow::readData);
    } else {
        QMessageBox::critical(this,"Port Error","Unable to open specified port...");
    }
}

void MainWindow::on_btnClosePort_clicked()
{
    if(!serialPort->isOpen()){
        QMessageBox::critical(this,"Port Error","Port is not opened.");
        return;
    }
    serialPort->close();
    QMessageBox::information(this,"Result","Port closed successfully.");
}

void MainWindow::on_btnRefresh_clicked()
{
    if(!serialPort->isOpen()){
        QMessageBox::critical(this,"Port Error","Port is not opened.");
        return;
    }
    ui->cmbPorts->clear();
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()){
        ui->cmbPorts->addItem(info.portName());
        qDebug() << info.portName();
    }
}

void MainWindow::on_btnSend_clicked()
{
    if(!serialPort->isOpen()){
        QMessageBox::critical(this,"Port Error","Port is not opened.");
        return;
    }
    serialPort->write(ui->lnMessage->text().toUtf8());
}

void MainWindow::readData()
{
    if(!serialPort->isOpen()){
        QMessageBox::critical(this,"Port Error","Port is not opened.");
        return;
    }
    auto data = serialPort->readAll();
    uint8_t lengthReceive = data[1] + 4;
    qDebug() << lengthReceive;
    //check footer
    if((int)data[lengthReceive-1] == 0x06){     //transfer successfully
        if((int)data[2] == 0x03){               //this is receive mode
            data.remove(lengthReceive-1,1);     //remove footer
            data.remove(0,3);                   //remove header, size and mode
            enc_val = QByteArrayToFloat(data);
            ui->lstMessages->addItem(QString::number(enc_val));
        }
        ui->lstMessages->addItem("Success");
    }
    else                                        //transfer unsuccessfully
        ui->lstMessages->addItem("Failure");
}

void MainWindow::on_btnClear_clicked()
{
    ui->lstMessages->clear();
}

void MainWindow::on_btnUpdateValue_clicked()
{
    if(!serialPort->isOpen()){
        QMessageBox::critical(this,"Port Error","Port is not opened.");
        return;
    }
//    serialPort->write(ui->lnSetPoint->text().toUtf8());
    setPoint = ui->lnSetPoint->text().toFloat();
    Kp = ui->lnKp->text().toFloat();
    Ki = ui->lnKi->text().toFloat();
    Kd = ui->lnKd->text().toFloat();
//    qDebug() << setPoint;
//    setPointArray = QByteArray::fromRawData(reinterpret_cast<char *>(&setPoint),sizeof(float));
//    qDebug() << setPointArray;
    //setting for mainArray
    this->mainArray.resize(20);
    this->mainArray.clear();
    if(ui->rdRun->isChecked()){
        this->mode = 0x01;          //SET RUN MODE
    }
    if(ui->rdStop->isChecked()){
        this->mode = 0x02;          //SET STOP MODE
        this->setPoint = 0;
    }
    //transfer setPoint
    floatToByteArray(setPoint);
    //transfer Kp
    floatToByteArray(Kp);
    //transfer Ki
    floatToByteArray(Ki);
    //transfer Kd
    floatToByteArray(Kd);
    addHeaderFooter();
    serialPort->write(this->mainArray);
}

void MainWindow::floatToByteArray(float floatValue)
{
    QByteArray arr = QByteArray::fromRawData(reinterpret_cast<char *>(&floatValue),sizeof(float));
    this->mainArray.append(arr);
    qDebug() << this->mainArray;
    this->length += 4;
    qDebug() << this->length;
}

float MainWindow::QByteArrayToFloat(QByteArray arr)
{
    QDataStream in (arr);
    float f;
    in.setByteOrder(QDataStream::LittleEndian);
    in.setFloatingPointPrecision(QDataStream::SinglePrecision);     //for float (4-bytes/32 bits)
    in >> f;
    return f;
}

void MainWindow::addHeaderFooter(void)
{
    this->mainArray.push_front(this->mode);
    this->mainArray.push_front(this->length);       //size of array without header, footer and length
    this->mainArray.push_front(0x0A);       //header: mainArray[0]=0x0A
    this->mainArray.push_back(0x05);        //footer: mainArray[size-1]=0x05
    this->length = 0;
}

void MainWindow::addMotorGraph(void)
{
    /* Add graph and set the curve line color to green */
    ui->motorPlot->addGraph();
    ui->motorPlot->graph(0)->setPen(QPen(Qt::red));
    ui->motorPlot->graph(0)->setAntialiasedFill(false);
    ui->motorPlot->graph(0)->setName("PID Output");
    ui->motorPlot->addGraph();
    ui->motorPlot->graph(1)->setPen(QPen(Qt::blue));
    ui->motorPlot->graph(1)->setAntialiasedFill(false);
    ui->motorPlot->graph(1)->setName("Ref. Input");

    /* Configure x-Axis as time in secs */
    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%s");
    ui->motorPlot->xAxis->setTicker(timeTicker);
    ui->motorPlot->axisRect()->setupFullAxesBox();

    /* Configure x and y-Axis to display Labels */
    ui->motorPlot->xAxis->setTickLabelFont(QFont(QFont().family(),8));
    ui->motorPlot->yAxis->setTickLabelFont(QFont(QFont().family(),8));
    ui->motorPlot->xAxis->setLabel("Time(s)");
    ui->motorPlot->yAxis->setLabel("Speed(RPM)");

    /* Make top and right axis visible, but without ticks and label */
    ui->motorPlot->xAxis2->setVisible(true);
    ui->motorPlot->yAxis->setVisible(true);
    ui->motorPlot->xAxis2->setTicks(false);
    ui->motorPlot->yAxis2->setTicks(false);
    ui->motorPlot->xAxis2->setTickLabels(false);
    ui->motorPlot->yAxis2->setTickLabels(false);

    ui->motorPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->motorPlot->legend->setVisible(true);
    /* Add title of the graph */

    ui->motorPlot->plotLayout()->insertRow(0);
    QCPTextElement *title = new QCPTextElement(ui->motorPlot, "PID Controller", QFont("sans", 17, QFont::Bold));
    ui->motorPlot->plotLayout()->addElement(0, 0, title);

    /* Set up and initialize the graph plotting timer */
    timerPlot = new QTimer(this);
    connect(timerPlot, SIGNAL(timeout()),this,SLOT(realtimePlot()));
    timerPlot->start(5);
}

void MainWindow::realtimePlot()
{    
    static QTime time(QTime::currentTime());
    static double storeKey = 0;
    double key = time.elapsed()/1000.0 + storeKey;
    static double lastPointKey = 0;
    if(this->stopPlot == false){
        if(key - lastPointKey > 0.002)
        {
            ui->motorPlot->graph(0)->addData(key, (double)enc_val);
            ui->motorPlot->graph(1)->addData(key, (double)setPoint);
            lastPointKey = key;
        }

        /* make key axis range scroll right with the data at a constant range of 8. */
        ui->motorPlot->graph(0)->rescaleValueAxis();
        ui->motorPlot->xAxis->setRange(key, 8, Qt::AlignRight);
        ui->motorPlot->replot();
    }
    else{
        time.restart();
        storeKey = lastPointKey;
    }
}

void MainWindow::on_btnStopPlot_clicked()
{
    this->stopPlot = true;
}

void MainWindow::on_btnClearPlot_clicked()
{
    ui->motorPlot->graph(0)->data()->clear();
    ui->motorPlot->graph(1)->data()->clear();
    ui->motorPlot->replot();
}

void MainWindow::on_btnRunPlot_clicked()
{
    this->stopPlot = false;
}
