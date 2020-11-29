#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    realsense = new rs2::MyRealsense(this);

    ui->cover->installEventFilter(this);

    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
        QSerialPort serial;
        serial.setPort(info);
        ui->serial_list->addItem(serial.portName());
    }
    connect(realsense, &rs2::MyRealsense::frameOK, this, &MainWindow::show_img);
    connect(realsense, &rs2::MyRealsense::caliOK, this, &MainWindow::generate_extrin);
    connect(realsense, &rs2::MyRealsense::arrow_detected, this, &MainWindow::send_arrow_coords);
}

MainWindow::~MainWindow()
{
    realsense->close(ui->label);
    delete ui;
}

void MainWindow::on_checkBox_stateChanged(int arg1)
{
    if (arg1 == Qt::Checked) {
        DBG(GREEN "<camera> : " NONE "opening camera");
        RSMode mode = (RSMode)ui->comBoxMode->currentIndex();
        realsense->setMode(mode);
        DBG(GREEN "<camera> : " NONE "set mode to %d", mode);
        if (realsense->open(ui->cover, ui->labelFps) == false) {
            DBG(RED "realsense.open()" NONE);
        }
    } else if (arg1 == Qt::Unchecked) {
        DBG(GREEN "<camera> : " NONE "closing camera");
        realsense->close(ui->cover);
    }
}

void MainWindow::on_comBoxMode_currentIndexChanged(int index)
{
    RSMode mode = (RSMode)index;
    realsense->setMode(mode);
    DBG(GREEN "<camera> : " NONE "set mode to %d", mode);
}

void MainWindow::show_img()
{
    if (realsense->getState() != WORK) return;
    realsense->updateImg();
}

void MainWindow::generate_extrin(bool isOK)
{
    if (isOK)
        ui->btnCaliGen->setDisabled(false);
    else
        ui->btnCaliGen->setDisabled(true);
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    if (obj == ui->cover) {
        if (event->type() == QEvent::MouseButtonPress) {
            QMouseEvent *e = static_cast<QMouseEvent*>(event);
            if (e->button() == Qt::LeftButton) {
                if (realsense->getState() == STOP) {DBG("device not opened");return true;}
                int x = e->x(), y = e->y();
                int r = realsense->color_img.at<cv::Vec3b>(y, x)[2];
                int g = realsense->color_img.at<cv::Vec3b>(y, x)[1];
                int b = realsense->color_img.at<cv::Vec3b>(y, x)[0];
                float dist = realsense->getDist(x, y);
                DBG(BLUE "<mouse> : " NONE "(%d, %d) : (%d, %d, %d, %f)", x, y, r, g, b, dist);
                QString str_rgb = QString("R : %1 G : %2 B : %3").arg(r).arg(g).arg(b);
                cv::Point3f p_w = realsense->getWorldCoord(x, y);
                QString str_coor = QString("x : %1 y : %2 z : %3").arg(p_w.x).arg(p_w.y).arg(p_w.z);
                ui->labelRGB->setText(str_rgb);
                ui->labelCoor->setText(str_coor);
                return true;
            }
        }
    }
    return false;
}

void MainWindow::on_btnCaliStart_clicked()
{
    int n_row = ui->editNumX->text().toUInt();
    int n_col = ui->editNumY->text().toUInt();
    realsense->calibrate(n_row, n_col);
}

void MainWindow::on_btnCaliContinue_clicked()
{
    ui->btnCaliGen->setDisabled(true);
    realsense->reopen();
    ui->info->clear();
}

void MainWindow::on_btnCaliGen_clicked()
{
    realsense->setCaliPos(getCaliPos());
    realsense->is_x_inv = (bool)ui->chkInvX->checkState();
    realsense->is_y_inv = (bool)ui->chkInvY->checkState();
    realsense->is_z_inv = (bool)ui->chkInvZ->checkState();
    realsense->cali_x = ui->editWorldX->text().toFloat();
    realsense->cali_y = ui->editWorldY->text().toFloat();
    realsense->cali_z = ui->editWorldZ->text().toFloat();
    realsense->cali_step = ui->editBoxLen->text().toFloat();
    DBG(YELLOW "<calibrate> : " NONE "x : %f y : %f z : %f len : %f", realsense->cali_x, realsense->cali_y, realsense->cali_z, realsense->cali_step);
    realsense->genExtrin();
    ui->info->setStyleSheet("color:green");
    ui->info->setText(tr("GEN SUCCESS"));
}

eCaliPos MainWindow::getCaliPos()
{
    eCaliPos pos;
    if (ui->rbtnUL->isChecked()) pos = UL;
    else if (ui->rbtnUR->isChecked()) pos = UR;
    else if (ui->rbtnDL->isChecked()) pos = DL;
    else if (ui->rbtnDR->isChecked()) pos = DR;
    else pos = DL;
    return pos;
}

void MainWindow::on_pushButton_clicked()
{

}

void MainWindow::on_comboBoxFun_currentIndexChanged(int index)
{
    FUN fun = (FUN)index;
    if (fun == DETECT_ARROW)
        realsense->setMode(COLOR_MODE);
    realsense->setFUN(fun);
}

void MainWindow::on_serial_refresh_clicked()
{
    ui->serial_list->clear();
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
        QSerialPort serial;
        serial.setPort(info);
        ui->serial_list->addItem(serial.portName());
    }
}

void MainWindow::on_serial_open_stateChanged(int arg1)
{
    ui->serial_output->append(QChar(arg1 + '0'));
    if (ui->serial_open->checkState() == Qt::Checked) {
        qint32 baudrate = 115200;
        QString portname = ui->serial_list->currentText();

        ui->serial_output->append("device name : " + portname);
        serial = new QSerialPort();
        serial->setPortName(portname);
        if (!serial->open(QIODevice::ReadWrite)) {
            ui->serial_output->append("open failed");
            return;
        };
        serial->setBaudRate(baudrate);
        serial->setDataBits(QSerialPort::Data8);
        serial->setParity(QSerialPort::NoParity);
        serial->setStopBits(QSerialPort::OneStop);
        serial->setFlowControl(QSerialPort::NoFlowControl);
        // QObject::connect(serial, &QSerialPort::readyRead, this, &Widget::read_data);
        ui->serial_output->append("open device");
    } else {
        if (!serial->isOpen()) {
            ui->serial_output->append("device not open");
            return;
        }
        serial->clear();
        serial->close();
        serial->deleteLater();
        ui->serial_output->append("close device");
    }
}

void MainWindow::send_arrow_coords()
{
    if (serial->isOpen()) {
        std::vector<cv::Point> points = realsense->arrow_mid_points;
        std::vector<cv::Point3f> world_coords;
        for (size_t i = 0; i < points.size(); i++) {
            world_coords.push_back(realsense->getWorldCoord(points[i].x, points[i].y));
            serial->write("hello world\n");
        }
    }
}
