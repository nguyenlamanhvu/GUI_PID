#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QDebug>
#include <QMessageBox>
#include <QDataStream>

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
private:
    Ui::MainWindow *ui;
    QSerialPort *serialPort;
    QSerialPortInfo *serialInfo;

    // Variables for setpoint value
    float setPoint = 0.0; // unit deg
    QByteArray setPointArray;

    // Variables for Kp
    float Kp = 0.0;
    QByteArray KpArray;

    // Variables for Ki
    float Ki = 0.0;
    QByteArray KiArray;

    // Variables for Kd
    float Kd = 0.0;
    QByteArray KdArray;

    void loadPorts();
    void floatToByteArray(float floatValue, QByteArray byteArray);
    float QByteArrayToFloat(QByteArray arr);
};
#endif // MAINWINDOW_H
