/********************************************************************************
** Form generated from reading UI file 'robotican_gui.ui'
**
** Created by: Qt User Interface Compiler version 5.2.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef ROBOTICAN_GUI_H
#define ROBOTICAN_GUI_H

#include <QtCore/QVariant>
#include <QAction>
#include <QApplication>
#include <QButtonGroup>
#include <QFrame>
#include <QHeaderView>
#include <QLabel>
#include <QMainWindow>
#include <QMenuBar>
#include <QProgressBar>
#include <QPushButton>
#include <QStatusBar>
#include <QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QLabel *gripper_lbl;
    QLabel *gps_lbl;
    QLabel *left_urf_lbl;
    QLabel *right_urf_lbl;
    QLabel *rear_urf_lbl;
    QLabel *arm_lbl;
    QLabel *label_7;
    QLabel *gripper_led;
    QLabel *battery_lbl;
    QLabel *label_12;
    QLabel *label_14;
    QLabel *arm_led;
    QFrame *line;
    QLabel *gps_led;
    QLabel *urf_right_led;
    QLabel *urf_left_led;
    QLabel *urf_rear_led;
    QFrame *line_2;
    QFrame *line_3;
    QPushButton *launch_btn;
    QPushButton *exit_btn;
    QLabel *batteryLbl;
    QProgressBar *battery_pbar;
    QFrame *line_5;
    QFrame *line_6;
    QLabel *pan_tilt_led;
    QLabel *label_15;
    QLabel *lidar_lbl;
    QLabel *odom_led;
    QLabel *lidar_led;
    QLabel *label_16;
    QLabel *label_17;
    QLabel *rear_cam_led;
    QLabel *front_cam_led;
    QLabel *rear_cam_lbl;
    QLabel *front_cam_lbl;
    QLabel *odom_lbl;
    QLabel *imu_led;
    QLabel *imu_lbl;
    QLabel *label_22;
    QFrame *line_13;
    QLabel *label_11;
    QLabel *label_13;
    QLabel *label_19;
    QFrame *line_14;
    QFrame *line_7;
    QFrame *line_8;
    QFrame *line_9;
    QFrame *line_15;
    QLabel *label_20;
    QFrame *line_4;
    QLabel *label_25;
    QLabel *pan_tilt_lbl;
    QLabel *kinnect2_lbl;
    QLabel *kinect2_led;
    QLabel *f200_lbl;
    QLabel *f200_led;
    QFrame *line_10;
    QLabel *battery_led;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString("MainWindow"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        MainWindow->setMinimumSize(QSize(333, 507));
        MainWindow->setMaximumSize(QSize(333, 507));
        QIcon icon;
        icon.addFile(QString(":/images/robotican_gui_logo.ico"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindow->setWindowIcon(icon);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString("centralwidget"));
        gripper_lbl = new QLabel(centralwidget);
        gripper_lbl->setObjectName(QString("gripper_lbl"));
        gripper_lbl->setGeometry(QRect(10, 320, 61, 21));
        QFont font;
        font.setPointSize(9);
        font.setBold(true);
        font.setWeight(75);
        gripper_lbl->setFont(font);
        gps_lbl = new QLabel(centralwidget);
        gps_lbl->setObjectName(QString("gps_lbl"));
        gps_lbl->setGeometry(QRect(130, 100, 31, 21));
        gps_lbl->setFont(font);
        left_urf_lbl = new QLabel(centralwidget);
        left_urf_lbl->setObjectName(QString("left_urf_lbl"));
        left_urf_lbl->setGeometry(QRect(150, 320, 31, 21));
        left_urf_lbl->setFont(font);
        right_urf_lbl = new QLabel(centralwidget);
        right_urf_lbl->setObjectName(QString("right_urf_lbl"));
        right_urf_lbl->setGeometry(QRect(210, 320, 41, 21));
        right_urf_lbl->setFont(font);
        rear_urf_lbl = new QLabel(centralwidget);
        rear_urf_lbl->setObjectName(QString("rear_urf_lbl"));
        rear_urf_lbl->setGeometry(QRect(270, 320, 41, 21));
        rear_urf_lbl->setFont(font);
        arm_lbl = new QLabel(centralwidget);
        arm_lbl->setObjectName(QString("arm_lbl"));
        arm_lbl->setGeometry(QRect(80, 320, 41, 21));
        arm_lbl->setFont(font);
        label_7 = new QLabel(centralwidget);
        label_7->setObjectName(QString("label_7"));
        label_7->setGeometry(QRect(40, 240, 41, 31));
        label_7->setPixmap(QPixmap(QString::fromUtf8(":/images/eng.png")));
        label_7->setScaledContents(true);
        gripper_led = new QLabel(centralwidget);
        gripper_led->setObjectName(QString("gripper_led"));
        gripper_led->setGeometry(QRect(20, 280, 31, 31));
        gripper_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        gripper_led->setScaledContents(true);
        battery_lbl = new QLabel(centralwidget);
        battery_lbl->setObjectName(QString("battery_lbl"));
        battery_lbl->setGeometry(QRect(20, 100, 71, 21));
        QFont font1;
        font1.setBold(true);
        font1.setWeight(75);
        battery_lbl->setFont(font1);
        battery_lbl->setScaledContents(true);
        label_12 = new QLabel(centralwidget);
        label_12->setObjectName(QString("label_12"));
        label_12->setGeometry(QRect(130, 120, 31, 31));
        label_12->setPixmap(QPixmap(QString::fromUtf8(":/images/gps.png")));
        label_12->setScaledContents(true);
        label_14 = new QLabel(centralwidget);
        label_14->setObjectName(QString("label_14"));
        label_14->setGeometry(QRect(230, 120, 31, 31));
        label_14->setPixmap(QPixmap(QString::fromUtf8(":/images/urf.png")));
        label_14->setScaledContents(true);
        arm_led = new QLabel(centralwidget);
        arm_led->setObjectName(QString("arm_led"));
        arm_led->setGeometry(QRect(80, 280, 31, 31));
        arm_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        arm_led->setScaledContents(true);
        line = new QFrame(centralwidget);
        line->setObjectName(QString("line"));
        line->setGeometry(QRect(0, 80, 331, 20));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        gps_led = new QLabel(centralwidget);
        gps_led->setObjectName(QString("gps_led"));
        gps_led->setGeometry(QRect(130, 160, 31, 31));
        gps_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        gps_led->setScaledContents(true);
        urf_right_led = new QLabel(centralwidget);
        urf_right_led->setObjectName(QString("urf_right_led"));
        urf_right_led->setGeometry(QRect(210, 280, 31, 31));
        urf_right_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        urf_right_led->setScaledContents(true);
        urf_left_led = new QLabel(centralwidget);
        urf_left_led->setObjectName(QString("urf_left_led"));
        urf_left_led->setGeometry(QRect(150, 280, 31, 31));
        urf_left_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        urf_left_led->setScaledContents(true);
        urf_rear_led = new QLabel(centralwidget);
        urf_rear_led->setObjectName(QString("urf_rear_led"));
        urf_rear_led->setGeometry(QRect(270, 280, 31, 31));
        urf_rear_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        urf_rear_led->setScaledContents(true);
        line_2 = new QFrame(centralwidget);
        line_2->setObjectName(QString("line_2"));
        line_2->setGeometry(QRect(320, 90, 20, 131));
        line_2->setFrameShape(QFrame::VLine);
        line_2->setFrameShadow(QFrame::Sunken);
        line_3 = new QFrame(centralwidget);
        line_3->setObjectName(QString("line_3"));
        line_3->setGeometry(QRect(160, 90, 20, 131));
        line_3->setFrameShape(QFrame::VLine);
        line_3->setFrameShadow(QFrame::Sunken);
        launch_btn = new QPushButton(centralwidget);
        launch_btn->setObjectName(QString("launch_btn"));
        launch_btn->setGeometry(QRect(10, 10, 71, 71));
        QFont font2;
        font2.setPointSize(12);
        font2.setBold(true);
        font2.setWeight(75);
        launch_btn->setFont(font2);
        QIcon icon1;
        icon1.addFile(QString(":/images/Shutdown.png"), QSize(), QIcon::Normal, QIcon::Off);
        launch_btn->setIcon(icon1);
        launch_btn->setIconSize(QSize(65, 65));
        launch_btn->setFlat(false);
        exit_btn = new QPushButton(centralwidget);
        exit_btn->setObjectName(QString("exit_btn"));
        exit_btn->setGeometry(QRect(250, 10, 71, 71));
        QFont font3;
        font3.setPointSize(16);
        font3.setBold(true);
        font3.setWeight(75);
        exit_btn->setFont(font3);
        QIcon icon2;
        icon2.addFile(QString(":/images/exit.png"), QSize(), QIcon::Normal, QIcon::Off);
        exit_btn->setIcon(icon2);
        exit_btn->setIconSize(QSize(70, 70));
        exit_btn->setFlat(false);
        batteryLbl = new QLabel(centralwidget);
        batteryLbl->setObjectName(QString("batteryLbl"));
        batteryLbl->setGeometry(QRect(10, 110, 91, 61));
        batteryLbl->setMaximumSize(QSize(100, 70));
        batteryLbl->setPixmap(QPixmap(QString::fromUtf8(":/images/battery.png")));
        batteryLbl->setScaledContents(true);
        battery_pbar = new QProgressBar(centralwidget);
        battery_pbar->setObjectName(QString("battery_pbar"));
        battery_pbar->setGeometry(QRect(30, 130, 61, 21));
        battery_pbar->setMaximumSize(QSize(100, 30));
        battery_pbar->setValue(0);
        line_5 = new QFrame(centralwidget);
        line_5->setObjectName(QString("line_5"));
        line_5->setGeometry(QRect(110, 90, 20, 131));
        line_5->setFrameShape(QFrame::VLine);
        line_5->setFrameShadow(QFrame::Sunken);
        line_6 = new QFrame(centralwidget);
        line_6->setObjectName(QString("line_6"));
        line_6->setGeometry(QRect(0, 210, 331, 20));
        line_6->setFrameShape(QFrame::HLine);
        line_6->setFrameShadow(QFrame::Sunken);
        pan_tilt_led = new QLabel(centralwidget);
        pan_tilt_led->setObjectName(QString("pan_tilt_led"));
        pan_tilt_led->setGeometry(QRect(270, 410, 31, 31));
        pan_tilt_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        pan_tilt_led->setScaledContents(true);
        label_15 = new QLabel(centralwidget);
        label_15->setObjectName(QString("label_15"));
        label_15->setGeometry(QRect(280, 120, 31, 31));
        label_15->setPixmap(QPixmap(QString::fromUtf8(":/images/odo.png")));
        label_15->setScaledContents(true);
        lidar_lbl = new QLabel(centralwidget);
        lidar_lbl->setObjectName(QString("lidar_lbl"));
        lidar_lbl->setGeometry(QRect(230, 100, 41, 21));
        lidar_lbl->setFont(font);
        odom_led = new QLabel(centralwidget);
        odom_led->setObjectName(QString("odom_led"));
        odom_led->setGeometry(QRect(280, 160, 31, 31));
        odom_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        odom_led->setScaledContents(true);
        lidar_led = new QLabel(centralwidget);
        lidar_led->setObjectName(QString("lidar_led"));
        lidar_led->setGeometry(QRect(230, 160, 31, 31));
        lidar_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        lidar_led->setScaledContents(true);
        label_16 = new QLabel(centralwidget);
        label_16->setObjectName(QString("label_16"));
        label_16->setGeometry(QRect(100, 370, 31, 31));
        label_16->setPixmap(QPixmap(QString::fromUtf8(":/images/camera.svg")));
        label_16->setScaledContents(true);
        label_17 = new QLabel(centralwidget);
        label_17->setObjectName(QString("label_17"));
        label_17->setGeometry(QRect(210, 240, 31, 31));
        label_17->setPixmap(QPixmap(QString::fromUtf8(":/images/laser.png")));
        label_17->setScaledContents(true);
        rear_cam_led = new QLabel(centralwidget);
        rear_cam_led->setObjectName(QString("rear_cam_led"));
        rear_cam_led->setGeometry(QRect(190, 410, 31, 31));
        rear_cam_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        rear_cam_led->setScaledContents(true);
        front_cam_led = new QLabel(centralwidget);
        front_cam_led->setObjectName(QString("front_cam_led"));
        front_cam_led->setGeometry(QRect(140, 410, 31, 31));
        front_cam_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        front_cam_led->setScaledContents(true);
        rear_cam_lbl = new QLabel(centralwidget);
        rear_cam_lbl->setObjectName(QString("rear_cam_lbl"));
        rear_cam_lbl->setGeometry(QRect(190, 450, 41, 21));
        rear_cam_lbl->setFont(font);
        front_cam_lbl = new QLabel(centralwidget);
        front_cam_lbl->setObjectName(QString("front_cam_lbl"));
        front_cam_lbl->setGeometry(QRect(130, 450, 41, 21));
        front_cam_lbl->setFont(font);
        odom_lbl = new QLabel(centralwidget);
        odom_lbl->setObjectName(QString("odom_lbl"));
        odom_lbl->setGeometry(QRect(280, 100, 41, 21));
        odom_lbl->setFont(font);
        imu_led = new QLabel(centralwidget);
        imu_led->setObjectName(QString("imu_led"));
        imu_led->setGeometry(QRect(180, 160, 31, 31));
        imu_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        imu_led->setScaledContents(true);
        imu_lbl = new QLabel(centralwidget);
        imu_lbl->setObjectName(QString("imu_lbl"));
        imu_lbl->setGeometry(QRect(180, 100, 31, 21));
        imu_lbl->setFont(font);
        label_22 = new QLabel(centralwidget);
        label_22->setObjectName(QString("label_22"));
        label_22->setGeometry(QRect(180, 120, 31, 31));
        label_22->setPixmap(QPixmap(QString::fromUtf8(":/images/imu.png")));
        label_22->setScaledContents(true);
        line_13 = new QFrame(centralwidget);
        line_13->setObjectName(QString("line_13"));
        line_13->setGeometry(QRect(260, 90, 20, 131));
        line_13->setFrameShape(QFrame::VLine);
        line_13->setFrameShadow(QFrame::Sunken);
        label_11 = new QLabel(centralwidget);
        label_11->setObjectName(QString("label_11"));
        label_11->setGeometry(QRect(210, 220, 31, 21));
        label_11->setFont(font);
        label_13 = new QLabel(centralwidget);
        label_13->setObjectName(QString("label_13"));
        label_13->setGeometry(QRect(10, 220, 111, 21));
        label_13->setFont(font);
        label_19 = new QLabel(centralwidget);
        label_19->setObjectName(QString("label_19"));
        label_19->setGeometry(QRect(90, 350, 61, 21));
        label_19->setFont(font);
        line_14 = new QFrame(centralwidget);
        line_14->setObjectName(QString("line_14"));
        line_14->setGeometry(QRect(110, 220, 20, 131));
        line_14->setFrameShape(QFrame::VLine);
        line_14->setFrameShadow(QFrame::Sunken);
        line_7 = new QFrame(centralwidget);
        line_7->setObjectName(QString("line_7"));
        line_7->setGeometry(QRect(0, 340, 331, 20));
        line_7->setFrameShape(QFrame::HLine);
        line_7->setFrameShadow(QFrame::Sunken);
        line_8 = new QFrame(centralwidget);
        line_8->setObjectName(QString("line_8"));
        line_8->setGeometry(QRect(320, 220, 20, 131));
        line_8->setFrameShape(QFrame::VLine);
        line_8->setFrameShadow(QFrame::Sunken);
        line_9 = new QFrame(centralwidget);
        line_9->setObjectName(QString("line_9"));
        line_9->setGeometry(QRect(230, 350, 20, 131));
        line_9->setFrameShape(QFrame::VLine);
        line_9->setFrameShadow(QFrame::Sunken);
        line_15 = new QFrame(centralwidget);
        line_15->setObjectName(QString("line_15"));
        line_15->setGeometry(QRect(320, 350, 20, 131));
        line_15->setFrameShape(QFrame::VLine);
        line_15->setFrameShadow(QFrame::Sunken);
        label_20 = new QLabel(centralwidget);
        label_20->setObjectName(QString("label_20"));
        label_20->setGeometry(QRect(90, 30, 151, 21));
        label_20->setPixmap(QPixmap(QString::fromUtf8(":/images/robotican.png")));
        label_20->setScaledContents(true);
        line_4 = new QFrame(centralwidget);
        line_4->setObjectName(QString("line_4"));
        line_4->setGeometry(QRect(210, 90, 20, 131));
        line_4->setFrameShape(QFrame::VLine);
        line_4->setFrameShadow(QFrame::Sunken);
        label_25 = new QLabel(centralwidget);
        label_25->setObjectName(QString("label_25"));
        label_25->setGeometry(QRect(260, 370, 51, 31));
        label_25->setPixmap(QPixmap(QString::fromUtf8(":/images/panTilt.png")));
        label_25->setScaledContents(true);
        pan_tilt_lbl = new QLabel(centralwidget);
        pan_tilt_lbl->setObjectName(QString("pan_tilt_lbl"));
        pan_tilt_lbl->setGeometry(QRect(260, 350, 51, 21));
        pan_tilt_lbl->setFont(font);
        kinnect2_lbl = new QLabel(centralwidget);
        kinnect2_lbl->setObjectName(QString("kinnect2_lbl"));
        kinnect2_lbl->setGeometry(QRect(10, 450, 61, 21));
        kinnect2_lbl->setFont(font);
        kinect2_led = new QLabel(centralwidget);
        kinect2_led->setObjectName(QString("kinect2_led"));
        kinect2_led->setGeometry(QRect(20, 410, 31, 31));
        kinect2_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        kinect2_led->setScaledContents(true);
        f200_lbl = new QLabel(centralwidget);
        f200_lbl->setObjectName(QString("f200_lbl"));
        f200_lbl->setGeometry(QRect(80, 450, 31, 21));
        f200_lbl->setFont(font);
        f200_led = new QLabel(centralwidget);
        f200_led->setObjectName(QString("f200_led"));
        f200_led->setGeometry(QRect(80, 410, 31, 31));
        f200_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        f200_led->setScaledContents(true);
        line_10 = new QFrame(centralwidget);
        line_10->setObjectName(QString("line_10"));
        line_10->setGeometry(QRect(0, 470, 331, 20));
        line_10->setFrameShape(QFrame::HLine);
        line_10->setFrameShadow(QFrame::Sunken);
        battery_led = new QLabel(centralwidget);
        battery_led->setObjectName(QString("battery_led"));
        battery_led->setGeometry(QRect(40, 160, 31, 31));
        battery_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        battery_led->setScaledContents(true);
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString("menubar"));
        menubar->setGeometry(QRect(0, 0, 333, 25));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Robotican GUI", 0));
        gripper_lbl->setText(QApplication::translate("MainWindow", "GRIPPER", 0));
        gps_lbl->setText(QApplication::translate("MainWindow", "GPS", 0));
        left_urf_lbl->setText(QApplication::translate("MainWindow", "LEFT", 0));
        right_urf_lbl->setText(QApplication::translate("MainWindow", "RIGHT", 0));
        rear_urf_lbl->setText(QApplication::translate("MainWindow", "REAR", 0));
        arm_lbl->setText(QApplication::translate("MainWindow", "ARM", 0));
        label_7->setText(QString());
        gripper_led->setText(QString());
        battery_lbl->setText(QApplication::translate("MainWindow", "BATTERY", 0));
        label_12->setText(QString());
        label_14->setText(QString());
        arm_led->setText(QString());
        gps_led->setText(QString());
        urf_right_led->setText(QString());
        urf_left_led->setText(QString());
        urf_rear_led->setText(QString());
#ifndef QT_NO_TOOLTIP
        launch_btn->setToolTip(QApplication::translate("MainWindow", "Open/close launch file", 0));
#endif // QT_NO_TOOLTIP
        launch_btn->setText(QString());
#ifndef QT_NO_TOOLTIP
        exit_btn->setToolTip(QApplication::translate("MainWindow", "Exit application", 0));
#endif // QT_NO_TOOLTIP
        exit_btn->setText(QString());
        batteryLbl->setText(QString());
        pan_tilt_led->setText(QString());
        label_15->setText(QString());
        lidar_lbl->setText(QApplication::translate("MainWindow", "LIDAR", 0));
        odom_led->setText(QString());
        lidar_led->setText(QString());
        label_16->setText(QString());
        label_17->setText(QString());
        rear_cam_led->setText(QString());
        front_cam_led->setText(QString());
        rear_cam_lbl->setText(QApplication::translate("MainWindow", "REAR", 0));
        front_cam_lbl->setText(QApplication::translate("MainWindow", "FRONT", 0));
        odom_lbl->setText(QApplication::translate("MainWindow", "ODOM", 0));
        imu_led->setText(QString());
        imu_lbl->setText(QApplication::translate("MainWindow", "IMU", 0));
        label_22->setText(QString());
        label_11->setText(QApplication::translate("MainWindow", "URF", 0));
        label_13->setText(QApplication::translate("MainWindow", "ARM MOVEMENT", 0));
        label_19->setText(QApplication::translate("MainWindow", "CAMMERA", 0));
        label_20->setText(QString());
        label_25->setText(QString());
        pan_tilt_lbl->setText(QApplication::translate("MainWindow", "PAN-TILT", 0));
        kinnect2_lbl->setText(QApplication::translate("MainWindow", "KINECT2", 0));
        kinect2_led->setText(QString());
        f200_lbl->setText(QApplication::translate("MainWindow", "F200", 0));
        f200_led->setText(QString());
        battery_led->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // ROBOTICAN_GUI_H
