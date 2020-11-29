#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "common.h"
#include "myrealsense.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    QSerialPort *serial;

private slots:
    void on_checkBox_stateChanged(int arg1);

    void on_comBoxMode_currentIndexChanged(int index);

    void show_img();

    void generate_extrin(bool isOK);

    void on_btnCaliStart_clicked();

    void on_btnCaliContinue_clicked();

    void on_btnCaliGen_clicked();

    void on_pushButton_clicked();

    void on_comboBoxFun_currentIndexChanged(int index);

    void on_serial_refresh_clicked();

    void on_serial_open_stateChanged(int arg1);

    void send_arrow_coords();

private:
    Ui::MainWindow *ui;
    rs2::MyRealsense *realsense;

    eCaliPos getCaliPos();
protected:
    bool eventFilter(QObject *obj, QEvent *event);
};
#endif // MAINWINDOW_H
