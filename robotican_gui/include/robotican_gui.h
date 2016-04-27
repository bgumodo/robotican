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
    QLabel *label;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *label_2;
    QLabel *label_7;
    QLabel *gripper_led;
    QLabel *label_9;
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
    QLabel *label_8;
    QLabel *pan_tilt_led;
    QLabel *label_15;
    QLabel *label_10;
    QLabel *odom_led;
    QLabel *lidar_led;
    QLabel *label_16;
    QLabel *label_17;
    QLabel *rear_cam_led;
    QLabel *front_cam_led;
    QLabel *label_18;
    QLabel *label_23;
    QLabel *label_24;
    QLabel *imu_led;
    QLabel *label_21;
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
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString("MainWindow"));
        MainWindow->resize(437, 414);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        MainWindow->setMinimumSize(QSize(437, 414));
        MainWindow->setMaximumSize(QSize(437, 414));
        QIcon icon;
        icon.addFile(QString(":/images/robotican_gui_logo.ico"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindow->setWindowIcon(icon);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString("centralwidget"));
        label = new QLabel(centralwidget);
        label->setObjectName(QString("label"));
        label->setGeometry(QRect(130, 200, 61, 21));
        QFont font;
        font.setPointSize(9);
        font.setBold(true);
        font.setWeight(75);
        label->setFont(font);
        label_3 = new QLabel(centralwidget);
        label_3->setObjectName(QString("label_3"));
        label_3->setGeometry(QRect(330, 100, 31, 21));
        label_3->setFont(font);
        label_4 = new QLabel(centralwidget);
        label_4->setObjectName(QString("label_4"));
        label_4->setGeometry(QRect(140, 320, 31, 21));
        label_4->setFont(font);
        label_5 = new QLabel(centralwidget);
        label_5->setObjectName(QString("label_5"));
        label_5->setGeometry(QRect(200, 320, 41, 21));
        label_5->setFont(font);
        label_6 = new QLabel(centralwidget);
        label_6->setObjectName(QString("label_6"));
        label_6->setGeometry(QRect(260, 320, 41, 21));
        label_6->setFont(font);
        label_2 = new QLabel(centralwidget);
        label_2->setObjectName(QString("label_2"));
        label_2->setGeometry(QRect(200, 200, 41, 21));
        label_2->setFont(font);
        label_7 = new QLabel(centralwidget);
        label_7->setObjectName(QString("label_7"));
        label_7->setGeometry(QRect(200, 120, 41, 31));
        label_7->setPixmap(QPixmap(QString::fromUtf8(":/images/eng.png")));
        label_7->setScaledContents(true);
        gripper_led = new QLabel(centralwidget);
        gripper_led->setObjectName(QString("gripper_led"));
        gripper_led->setGeometry(QRect(140, 160, 31, 31));
        gripper_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        gripper_led->setScaledContents(true);
        label_9 = new QLabel(centralwidget);
        label_9->setObjectName(QString("label_9"));
        label_9->setGeometry(QRect(20, 140, 71, 21));
        label_9->setScaledContents(true);
        label_12 = new QLabel(centralwidget);
        label_12->setObjectName(QString("label_12"));
        label_12->setGeometry(QRect(330, 120, 31, 31));
        label_12->setPixmap(QPixmap(QString::fromUtf8(":/images/gps.png")));
        label_12->setScaledContents(true);
        label_14 = new QLabel(centralwidget);
        label_14->setObjectName(QString("label_14"));
        label_14->setGeometry(QRect(330, 240, 31, 31));
        label_14->setPixmap(QPixmap(QString::fromUtf8(":/images/urf.png")));
        label_14->setScaledContents(true);
        arm_led = new QLabel(centralwidget);
        arm_led->setObjectName(QString("arm_led"));
        arm_led->setGeometry(QRect(200, 160, 31, 31));
        arm_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        arm_led->setScaledContents(true);
        line = new QFrame(centralwidget);
        line->setObjectName(QString("line"));
        line->setGeometry(QRect(10, 80, 421, 20));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        gps_led = new QLabel(centralwidget);
        gps_led->setObjectName(QString("gps_led"));
        gps_led->setGeometry(QRect(330, 160, 31, 31));
        gps_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        gps_led->setScaledContents(true);
        urf_right_led = new QLabel(centralwidget);
        urf_right_led->setObjectName(QString("urf_right_led"));
        urf_right_led->setGeometry(QRect(200, 280, 31, 31));
        urf_right_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        urf_right_led->setScaledContents(true);
        urf_left_led = new QLabel(centralwidget);
        urf_left_led->setObjectName(QString("urf_left_led"));
        urf_left_led->setGeometry(QRect(140, 280, 31, 31));
        urf_left_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        urf_left_led->setScaledContents(true);
        urf_rear_led = new QLabel(centralwidget);
        urf_rear_led->setObjectName(QString("urf_rear_led"));
        urf_rear_led->setGeometry(QRect(260, 280, 31, 31));
        urf_rear_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        urf_rear_led->setScaledContents(true);
        line_2 = new QFrame(centralwidget);
        line_2->setObjectName(QString("line_2"));
        line_2->setGeometry(QRect(310, 90, 20, 131));
        line_2->setFrameShape(QFrame::VLine);
        line_2->setFrameShadow(QFrame::Sunken);
        line_3 = new QFrame(centralwidget);
        line_3->setObjectName(QString("line_3"));
        line_3->setGeometry(QRect(360, 90, 20, 131));
        line_3->setFrameShape(QFrame::VLine);
        line_3->setFrameShadow(QFrame::Sunken);
        launch_btn = new QPushButton(centralwidget);
        launch_btn->setObjectName(QString("launch_btn"));
        launch_btn->setGeometry(QRect(10, 10, 71, 71));
        QFont font1;
        font1.setPointSize(12);
        font1.setBold(true);
        font1.setWeight(75);
        launch_btn->setFont(font1);
        QIcon icon1;
        icon1.addFile(QString(":/images/turnOn.png"), QSize(), QIcon::Normal, QIcon::Off);
        launch_btn->setIcon(icon1);
        launch_btn->setIconSize(QSize(65, 65));
        launch_btn->setFlat(false);
        exit_btn = new QPushButton(centralwidget);
        exit_btn->setObjectName(QString("exit_btn"));
        exit_btn->setGeometry(QRect(360, 10, 71, 71));
        QFont font2;
        font2.setPointSize(16);
        font2.setBold(true);
        font2.setWeight(75);
        exit_btn->setFont(font2);
        QIcon icon2;
        icon2.addFile(QString(":/images/exit.png"), QSize(), QIcon::Normal, QIcon::Off);
        exit_btn->setIcon(icon2);
        exit_btn->setIconSize(QSize(70, 70));
        exit_btn->setFlat(false);
        batteryLbl = new QLabel(centralwidget);
        batteryLbl->setObjectName(QString("batteryLbl"));
        batteryLbl->setGeometry(QRect(30, 100, 51, 41));
        batteryLbl->setMaximumSize(QSize(100, 70));
        batteryLbl->setPixmap(QPixmap(QString::fromUtf8(":/images/battery.png")));
        batteryLbl->setScaledContents(true);
        battery_pbar = new QProgressBar(centralwidget);
        battery_pbar->setObjectName(QString("battery_pbar"));
        battery_pbar->setGeometry(QRect(10, 170, 91, 30));
        battery_pbar->setMaximumSize(QSize(100, 30));
        battery_pbar->setValue(0);
        line_5 = new QFrame(centralwidget);
        line_5->setObjectName(QString("line_5"));
        line_5->setGeometry(QRect(110, 90, 20, 131));
        line_5->setFrameShape(QFrame::VLine);
        line_5->setFrameShadow(QFrame::Sunken);
        line_6 = new QFrame(centralwidget);
        line_6->setObjectName(QString("line_6"));
        line_6->setGeometry(QRect(10, 210, 421, 20));
        line_6->setFrameShape(QFrame::HLine);
        line_6->setFrameShadow(QFrame::Sunken);
        label_8 = new QLabel(centralwidget);
        label_8->setObjectName(QString("label_8"));
        label_8->setGeometry(QRect(250, 200, 61, 21));
        label_8->setFont(font);
        pan_tilt_led = new QLabel(centralwidget);
        pan_tilt_led->setObjectName(QString("pan_tilt_led"));
        pan_tilt_led->setGeometry(QRect(260, 160, 31, 31));
        pan_tilt_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        pan_tilt_led->setScaledContents(true);
        label_15 = new QLabel(centralwidget);
        label_15->setObjectName(QString("label_15"));
        label_15->setGeometry(QRect(390, 240, 31, 31));
        label_15->setPixmap(QPixmap(QString::fromUtf8(":/images/odo.png")));
        label_15->setScaledContents(true);
        label_10 = new QLabel(centralwidget);
        label_10->setObjectName(QString("label_10"));
        label_10->setGeometry(QRect(330, 220, 41, 21));
        label_10->setFont(font);
        odom_led = new QLabel(centralwidget);
        odom_led->setObjectName(QString("odom_led"));
        odom_led->setGeometry(QRect(390, 280, 31, 31));
        odom_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        odom_led->setScaledContents(true);
        lidar_led = new QLabel(centralwidget);
        lidar_led->setObjectName(QString("lidar_led"));
        lidar_led->setGeometry(QRect(330, 280, 31, 31));
        lidar_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        lidar_led->setScaledContents(true);
        label_16 = new QLabel(centralwidget);
        label_16->setObjectName(QString("label_16"));
        label_16->setGeometry(QRect(40, 240, 31, 31));
        label_16->setPixmap(QPixmap(QString::fromUtf8(":/images/camera.svg")));
        label_16->setScaledContents(true);
        label_17 = new QLabel(centralwidget);
        label_17->setObjectName(QString("label_17"));
        label_17->setGeometry(QRect(200, 240, 31, 31));
        label_17->setPixmap(QPixmap(QString::fromUtf8(":/images/laser.png")));
        label_17->setScaledContents(true);
        rear_cam_led = new QLabel(centralwidget);
        rear_cam_led->setObjectName(QString("rear_cam_led"));
        rear_cam_led->setGeometry(QRect(70, 280, 31, 31));
        rear_cam_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        rear_cam_led->setScaledContents(true);
        front_cam_led = new QLabel(centralwidget);
        front_cam_led->setObjectName(QString("front_cam_led"));
        front_cam_led->setGeometry(QRect(20, 280, 31, 31));
        front_cam_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        front_cam_led->setScaledContents(true);
        label_18 = new QLabel(centralwidget);
        label_18->setObjectName(QString("label_18"));
        label_18->setGeometry(QRect(70, 320, 41, 21));
        label_18->setFont(font);
        label_23 = new QLabel(centralwidget);
        label_23->setObjectName(QString("label_23"));
        label_23->setGeometry(QRect(10, 320, 41, 21));
        label_23->setFont(font);
        label_24 = new QLabel(centralwidget);
        label_24->setObjectName(QString("label_24"));
        label_24->setGeometry(QRect(380, 220, 41, 21));
        label_24->setFont(font);
        imu_led = new QLabel(centralwidget);
        imu_led->setObjectName(QString("imu_led"));
        imu_led->setGeometry(QRect(380, 160, 31, 31));
        imu_led->setPixmap(QPixmap(QString::fromUtf8(":/images/ledOff.png")));
        imu_led->setScaledContents(true);
        label_21 = new QLabel(centralwidget);
        label_21->setObjectName(QString("label_21"));
        label_21->setGeometry(QRect(380, 100, 31, 21));
        label_21->setFont(font);
        label_22 = new QLabel(centralwidget);
        label_22->setObjectName(QString("label_22"));
        label_22->setGeometry(QRect(380, 120, 31, 31));
        label_22->setPixmap(QPixmap(QString::fromUtf8(":/images/imu.png")));
        label_22->setScaledContents(true);
        line_13 = new QFrame(centralwidget);
        line_13->setObjectName(QString("line_13"));
        line_13->setGeometry(QRect(420, 90, 20, 131));
        line_13->setFrameShape(QFrame::VLine);
        line_13->setFrameShadow(QFrame::Sunken);
        label_11 = new QLabel(centralwidget);
        label_11->setObjectName(QString("label_11"));
        label_11->setGeometry(QRect(200, 220, 31, 21));
        label_11->setFont(font);
        label_13 = new QLabel(centralwidget);
        label_13->setObjectName(QString("label_13"));
        label_13->setGeometry(QRect(170, 100, 111, 21));
        label_13->setFont(font);
        label_19 = new QLabel(centralwidget);
        label_19->setObjectName(QString("label_19"));
        label_19->setGeometry(QRect(30, 220, 61, 21));
        label_19->setFont(font);
        line_14 = new QFrame(centralwidget);
        line_14->setObjectName(QString("line_14"));
        line_14->setGeometry(QRect(110, 220, 20, 131));
        line_14->setFrameShape(QFrame::VLine);
        line_14->setFrameShadow(QFrame::Sunken);
        line_7 = new QFrame(centralwidget);
        line_7->setObjectName(QString("line_7"));
        line_7->setGeometry(QRect(0, 340, 431, 20));
        line_7->setFrameShape(QFrame::HLine);
        line_7->setFrameShadow(QFrame::Sunken);
        line_8 = new QFrame(centralwidget);
        line_8->setObjectName(QString("line_8"));
        line_8->setGeometry(QRect(310, 220, 20, 131));
        line_8->setFrameShape(QFrame::VLine);
        line_8->setFrameShadow(QFrame::Sunken);
        line_9 = new QFrame(centralwidget);
        line_9->setObjectName(QString("line_9"));
        line_9->setGeometry(QRect(360, 220, 20, 131));
        line_9->setFrameShape(QFrame::VLine);
        line_9->setFrameShadow(QFrame::Sunken);
        line_15 = new QFrame(centralwidget);
        line_15->setObjectName(QString("line_15"));
        line_15->setGeometry(QRect(420, 220, 20, 131));
        line_15->setFrameShape(QFrame::VLine);
        line_15->setFrameShadow(QFrame::Sunken);
        label_20 = new QLabel(centralwidget);
        label_20->setObjectName(QString("label_20"));
        label_20->setGeometry(QRect(90, 10, 261, 41));
        label_20->setPixmap(QPixmap(QString::fromUtf8(":/images/robotican.png")));
        label_20->setScaledContents(true);
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString("menubar"));
        menubar->setGeometry(QRect(0, 0, 437, 25));
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
        label->setText(QApplication::translate("MainWindow", "GRIPPER", 0));
        label_3->setText(QApplication::translate("MainWindow", "GPS", 0));
        label_4->setText(QApplication::translate("MainWindow", "LEFT", 0));
        label_5->setText(QApplication::translate("MainWindow", "RIGHT", 0));
        label_6->setText(QApplication::translate("MainWindow", "REAR", 0));
        label_2->setText(QApplication::translate("MainWindow", "ARM", 0));
        label_7->setText(QString());
        gripper_led->setText(QString());
        label_9->setText(QApplication::translate("MainWindow", "BATTERY", 0));
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
        label_8->setText(QApplication::translate("MainWindow", "PAN TILT", 0));
        pan_tilt_led->setText(QString());
        label_15->setText(QString());
        label_10->setText(QApplication::translate("MainWindow", "LIDAR", 0));
        odom_led->setText(QString());
        lidar_led->setText(QString());
        label_16->setText(QString());
        label_17->setText(QString());
        rear_cam_led->setText(QString());
        front_cam_led->setText(QString());
        label_18->setText(QApplication::translate("MainWindow", "REAR", 0));
        label_23->setText(QApplication::translate("MainWindow", "FRONT", 0));
        label_24->setText(QApplication::translate("MainWindow", "ODOM", 0));
        imu_led->setText(QString());
        label_21->setText(QApplication::translate("MainWindow", "IMU", 0));
        label_22->setText(QString());
        label_11->setText(QApplication::translate("MainWindow", "URF", 0));
        label_13->setText(QApplication::translate("MainWindow", "ARM MOVEMENT", 0));
        label_19->setText(QApplication::translate("MainWindow", "CAMMERA", 0));
        label_20->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // ROBOTICAN_GUI_H
