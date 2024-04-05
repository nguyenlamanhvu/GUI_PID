#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QDebug>
#include <QMessageBox>
#include <QDataStream>
#include <QTimer>
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

#define BUFF_SIZE 20
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_btnOpenPort_clicked();

    void on_btnSend_clicked();

    void readData();

    void on_btnClosePort_clicked();

    void on_btnRefresh_clicked();

    void on_btnClear_clicked();

    void on_btnUpdateValue_clicked();
    void realtimePlot();
    void on_btnStopPlot_clicked();

    void on_btnClearPlot_clicked();

    void on_btnRunPlot_clicked();

private:
    Ui::MainWindow *ui;
    QSerialPort *serialPort;
    QSerialPortInfo *serialInfo;
    QTimer *timerPlot;
    //Main array
    QByteArray mainArray;
    //Length of main array
    uint8_t length;
    //Mode of motor
    uint8_t mode;               //mode: 0x01(SET RUN MOTOR)
                                //mode: 0x02(SET STOP MOTOR)
                                //mode: 0x03(RECEIVE MODE)
    //Stop plotting
    bool stopPlot;
    // Variables for setpoint value
    float setPoint = 0.0; // unit deg

    // Variables for Kp
    float Kp = 0.0;

    // Variables for Ki
    float Ki = 0.0;

    // Variables for Kd
    float Kd = 0.0;

    //Variales for encoder
    float enc_val = 0.0;
    void loadPorts();
    void floatToByteArray(float floatValue);
    float QByteArrayToFloat(QByteArray arr);
    void addHeaderFooter(void);
    void addMotorGraph(void);
};
#endif // MAINWINDOW_H
