#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QDebug>
#include <QMessageBox>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

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

private:
    Ui::MainWindow *ui;
    QSerialPort *serialPort;
    QSerialPortInfo *serialInfo;
    void loadPorts();
};
#endif // MAINWINDOW_H
