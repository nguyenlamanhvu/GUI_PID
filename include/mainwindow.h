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

enum motorMode_t:uint8_t
{
    GUI_SET_LEFT_RUN_MODE = 		0x01,
    GUI_SET_LEFT_STOP_MODE = 		0x02,
    GUI_SET_RIGHT_RUN_MODE = 		0x03,
    GUI_SET_RIGHT_STOP_MODE = 		0x04,
    GUI_GET_PARAMETER_LEFT =        0x05,
    GUI_GET_PARAMETER_RIGHT =       0x06,
    GUI_RECEIVE_LEFT_SPEED_MODE = 	0x07,
    GUI_RECEIVE_RIGHT_SPEED_MODE =  0x08,
    GUI_RECEIVE_PARAMETER_LEFT = 	0x09,
    GUI_RECEIVE_PARAMETER_RIGHT =   0x0A,
};

#pragma pack(1) // 1 byte alignment
struct dataFrame_t
{
    uint8_t 	header;							/*!< Header of data frame */
    uint8_t 	length;							/*!< Length of data (exclude header, length, mode, footer) */
    motorMode_t	mode;							/*!< Mode */
    uint8_t		dataBuff[BUFF_SIZE - 4];        /*!< Data buffer */
    uint8_t		footer;							/*!< Footer of data frame */
};
#pragma pack()   // End of pragma pack scope

union FloatByteArray_t
{
    float floatValue;
    uint8_t byteArray[4];
};

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

    void on_btnGetParameter_clicked();

private:
    Ui::MainWindow *ui;
    QSerialPort *serialPort;
    QSerialPortInfo *serialInfo;
    QTimer *timerPlot;

    dataFrame_t gTxDataFrame;
    dataFrame_t gRxDataFrame;
    FloatByteArray_t uartData;

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
    QByteArray structToByteArray(const dataFrame_t &myDataFrame);
    dataFrame_t byteArrayToStruct(const QByteArray &byteArray);
};
#endif // MAINWINDOW_H
