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
        QMessageBox::critical(this,"Port Error","Unable to open specified port: "+ serialPort->errorString());
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
    ui->cmbPorts->clear();
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()){
        ui->cmbPorts->addItem(info.portName());
        qDebug() << info.portName();
    }
}

void MainWindow::on_btnSend_clicked()
{
    serialPort->write(ui->lnMessage->text().toUtf8());
}

void MainWindow::readData()
{
    if(!serialPort->isOpen()){
        QMessageBox::critical(this,"Port Error","Port is not opened.");
        return;
    }
    auto data = serialPort->readAll();
    qDebug() << data;
    this->gRxDataFrame = byteArrayToStruct(data);
    /*!< Check header and footer */
    if(this->gRxDataFrame.header == 0x0A && this->gRxDataFrame.footer == 0x06)
    {
        switch (this->gRxDataFrame.mode) {
        case GUI_RECEIVE_PARAMETER_LEFT:
            /*!< Check length of data buffer */
            if(this->gRxDataFrame.length == 16)
            {
                ui->lblRunningMotor->setText("Left Motor");
                /*!< Set point */
                memcpy(uartData.byteArray, this->gRxDataFrame.dataBuff, sizeof(uartData.byteArray));
                this->setPoint = uartData.floatValue;
                ui->lnSetPoint->setText(QString::number(this->setPoint));
                ui->lstMessages->addItem("Set Point: " + QString::number(this->setPoint));
                /*!< Kp */
                memcpy(uartData.byteArray, this->gRxDataFrame.dataBuff + 4, sizeof(uartData.byteArray));
                this->Kp = uartData.floatValue;
                ui->lnKp->setText(QString::number(this->Kp));
                ui->lstMessages->addItem("Kp: " + QString::number(this->Kp));
                /*!< Ki */
                memcpy(uartData.byteArray, this->gRxDataFrame.dataBuff + 8, sizeof(uartData.byteArray));
                this->Ki = uartData.floatValue;
                ui->lnKi->setText(QString::number(this->Ki));
                ui->lstMessages->addItem("Ki: " + QString::number(this->Ki));
                /*!< Kd */
                memcpy(uartData.byteArray, this->gRxDataFrame.dataBuff + 12, sizeof(uartData.byteArray));
                this->Kd = uartData.floatValue;
                ui->lnKd->setText(QString::number(this->Kd));
                ui->lstMessages->addItem("Kd: " + QString::number(this->Kd));
            }
            else                //transfer unsuccessfully
            {
                ui->lstMessages->addItem("Failure");
            }
            break;
        case GUI_RECEIVE_PARAMETER_RIGHT:
            if(this->gRxDataFrame.length == 16)
            {
                ui->lblRunningMotor->setText("Right Motor");
                /*!< Set point */
                memcpy(uartData.byteArray, this->gRxDataFrame.dataBuff, sizeof(uartData.byteArray));
                this->setPoint = uartData.floatValue;
                ui->lnSetPoint->setText(QString::number(this->setPoint));
                ui->lstMessages->addItem("Set Point: " + QString::number(this->setPoint));
                /*!< Kp */
                memcpy(uartData.byteArray, this->gRxDataFrame.dataBuff + 4, sizeof(uartData.byteArray));
                this->Kp = uartData.floatValue;
                ui->lnKp->setText(QString::number(this->Kp));
                ui->lstMessages->addItem("Kp: " + QString::number(this->Kp));
                /*!< Ki */
                memcpy(uartData.byteArray, this->gRxDataFrame.dataBuff + 8, sizeof(uartData.byteArray));
                this->Ki = uartData.floatValue;
                ui->lnKi->setText(QString::number(this->Ki));
                ui->lstMessages->addItem("Ki: " + QString::number(this->Ki));
                /*!< Kd */
                memcpy(uartData.byteArray, this->gRxDataFrame.dataBuff + 12, sizeof(uartData.byteArray));
                this->Kd = uartData.floatValue;
                ui->lnKd->setText(QString::number(this->Kd));
                ui->lstMessages->addItem("Kd: " + QString::number(this->Kd));
            }
            else                //transfer unsuccessfully
            {
                ui->lstMessages->addItem("Failure");
            }
            break;
        case GUI_RECEIVE_LEFT_SPEED_MODE:
            /*!< Check length of data buffer */
            if(this->gRxDataFrame.length == 4)
            {
                memcpy(uartData.byteArray, this->gRxDataFrame.dataBuff, sizeof(uartData.byteArray));
                this->enc_val = uartData.floatValue;
                ui->lstMessages->addItem("Left Motor: " + QString::number(enc_val));
                ui->lblRunningMotor->setText("Left Motor");
            }
            else                //transfer unsuccessfully
            {
                ui->lstMessages->addItem("Failure");
            }
            break;
        case GUI_RECEIVE_RIGHT_SPEED_MODE:
            /*!< Check length of data buffer */
            if(this->gRxDataFrame.length == 4)
            {
                memcpy(uartData.byteArray, this->gRxDataFrame.dataBuff, sizeof(uartData.byteArray));
                this->enc_val = uartData.floatValue;
                ui->lstMessages->addItem("Right Motor: " + QString::number(enc_val));
                ui->lblRunningMotor->setText("Right Motor");
            }
            else                //transfer unsuccessfully
            {
                ui->lstMessages->addItem("Failure");
            }
            break;
        default:
            break;
        }
//        if(this->gRxDataFrame.mode == GUI_RECEIVE_PARAMETER_MODE)
//        {
//            //Will finish when I have time.

//        }
//        else if (this->gRxDataFrame.mode == GUI_RECEIVE_LEFT_SPEED_MODE)
//        {
//            /*!< Check length of data buffer */
//            if(this->gRxDataFrame.length == 4)
//            {
//                memcpy(uartData.byteArray, this->gRxDataFrame.dataBuff, sizeof(uartData.byteArray));
//                this->enc_val = uartData.floatValue;
//                ui->lstMessages->addItem("Left Motor: " + QString::number(enc_val));
//                ui->lblRunningMotor->setText("Left Motor");
//            }
//            else                //transfer unsuccessfully
//            {
//                ui->lstMessages->addItem("Failure");
//            }
//        }
//        else if (this->gRxDataFrame.mode == GUI_RECEIVE_RIGHT_SPEED_MODE)
//        {
//            /*!< Check length of data buffer */
//            if(this->gRxDataFrame.length == 4)
//            {
//                memcpy(uartData.byteArray, this->gRxDataFrame.dataBuff, sizeof(uartData.byteArray));
//                this->enc_val = uartData.floatValue;
//                ui->lstMessages->addItem("Right Motor: " + QString::number(enc_val));
//                ui->lblRunningMotor->setText("Right Motor");
//            }
//            else                //transfer unsuccessfully
//            {
//                ui->lstMessages->addItem("Failure");
//            }
//        }
    }
    else                    //transfer unsuccessfully
    {
        ui->lstMessages->addItem("Failure");
    }
}

void MainWindow::on_btnClear_clicked()
{
    ui->lstMessages->clear();
}

void MainWindow::on_btnUpdateValue_clicked()
{
    /*!< Add length */
    this->gTxDataFrame.length = 0;
    /*!< Add header */
    this->gTxDataFrame.header = 0x0A;
    /*!< Add footer */
    this->gTxDataFrame.footer = 0x05;
    /*!< Add mode */
    if(ui->cbLeftMotor->isChecked() && ui->cbRightMotor->isChecked())
    {
        QMessageBox::critical(this,"Choose Motor Error","Unable to choose motor...");
        return;
    }
    else if(ui->cbLeftMotor->isChecked() && ui->rdRun->isChecked())
    {
        this->gTxDataFrame.mode = GUI_SET_LEFT_RUN_MODE;
    }
    else if(ui->cbLeftMotor->isChecked() && ui->rdStop->isChecked())
    {
        this->gTxDataFrame.mode = GUI_SET_LEFT_STOP_MODE;
    }
    else if(ui->cbRightMotor->isChecked() && ui->rdRun->isChecked())
    {
        this->gTxDataFrame.mode = GUI_SET_RIGHT_RUN_MODE;
    }
    else if(ui->cbRightMotor->isChecked() && ui->rdStop->isChecked())
    {
        this->gTxDataFrame.mode = GUI_SET_RIGHT_STOP_MODE;
    }
    else
    {
        QMessageBox::critical(this,"Choose Motor Error","Unable to choose motor...");
        return;
    }

    /*!< Add Set point value to gRxDataFrame */
    this->uartData.floatValue = ui->lnSetPoint->text().toFloat();
    memcpy(this->gTxDataFrame.dataBuff, this->uartData.byteArray, sizeof(uartData.byteArray));
    this->gTxDataFrame.length += sizeof(uartData.byteArray);
    /*!< Add Kp value to gRxDataFrame */
    this->uartData.floatValue = ui->lnKp->text().toFloat();
    memcpy(this->gTxDataFrame.dataBuff + 4, this->uartData.byteArray, sizeof(uartData.byteArray));
    this->gTxDataFrame.length += sizeof(uartData.byteArray);
    /*!< Add Ki value to gRxDataFrame */
    this->uartData.floatValue = ui->lnKi->text().toFloat();
    memcpy(this->gTxDataFrame.dataBuff + 8, this->uartData.byteArray, sizeof(uartData.byteArray));
    this->gTxDataFrame.length += sizeof(uartData.byteArray);
    /*!< Add Kd value to gRxDataFrame */
    this->uartData.floatValue = ui->lnKd->text().toFloat();
    memcpy(this->gTxDataFrame.dataBuff + 12, this->uartData.byteArray, sizeof(uartData.byteArray));
    this->gTxDataFrame.length += sizeof(uartData.byteArray);

    //setting for mainArray
    this->mainArray.resize(BUFF_SIZE);
    this->mainArray.clear();
    this->mainArray = structToByteArray(gTxDataFrame);

    qDebug() << this->mainArray;

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

QByteArray MainWindow::structToByteArray(const dataFrame_t &myDataFrame)
{
    QByteArray byteArray;
    QDataStream out(&byteArray, QIODevice::WriteOnly);

    // Write the struct members into the byte array
    out << myDataFrame.header;
    out << myDataFrame.length;
    out << static_cast<uint8_t>(myDataFrame.mode);
    out.writeRawData(reinterpret_cast<const char*>(myDataFrame.dataBuff), sizeof(myDataFrame.dataBuff));  // Write raw buffer
    out << myDataFrame.footer;

    return byteArray;
}

dataFrame_t MainWindow::byteArrayToStruct(const QByteArray &byteArray)
{
    dataFrame_t myDataFrame;
    QDataStream in(byteArray);

    uint8_t modeValue;  // Temp variable for enum

    // Read the data from the byte array into the struct members
    in >> myDataFrame.header;
    in >> myDataFrame.length;
    in >> modeValue;
    myDataFrame.mode = static_cast<motorMode_t>(modeValue);  // Cast back to enum
    in.readRawData(reinterpret_cast<char*>(myDataFrame.dataBuff), sizeof(myDataFrame.dataBuff));  // Read raw buffer
    in >> myDataFrame.footer;

    return myDataFrame;
}
/* Dump data */
uint8_t dumpData[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
void MainWindow::on_btnGetParameter_clicked()
{
    /*!< Clear data frame */
    memset(&gTxDataFrame, 0, sizeof(dataFrame_t));
    /*!< Add length */
    this->gTxDataFrame.length = 16;
    /*!< Add header */
    this->gTxDataFrame.header = 0x0A;
    /*!< Add footer */
    this->gTxDataFrame.footer = 0x05;
    /*!< Add mode */
    if(ui->cbLeftMotor->isChecked() && ui->cbRightMotor->isChecked())
    {
        QMessageBox::critical(this,"Choose Motor Error","Unable to choose motor...");
        return;
    }
    else if(ui->cbLeftMotor->isChecked())
    {
        this->gTxDataFrame.mode = GUI_GET_PARAMETER_LEFT;
    }
    else if(ui->cbRightMotor->isChecked())
    {
        this->gTxDataFrame.mode = GUI_GET_PARAMETER_RIGHT;
    }
    else
    {
        QMessageBox::critical(this,"Choose Motor Error","Unable to choose motor...");
        return;
    }

    memcpy(gTxDataFrame.dataBuff, dumpData, 16);

    //setting for mainArray
    this->mainArray.resize(BUFF_SIZE);
    this->mainArray.clear();
    this->mainArray = structToByteArray(gTxDataFrame);

    qDebug() << this->mainArray;

    serialPort->write(this->mainArray);
}
