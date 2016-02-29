import serial
import threading
from BAL.protocol.packages.motor_to_takeover import REVERSE, NORMAL
from GUI.Schemes.takeoverConfig import Ui_Dialog
from PyQt4.QtGui import *
from BAL.protocol.protocol_handler import ProtocolHandler
from BAL.protocol.packages.clib_status import CANCEL, SAVE, RESET
from GUI.calib_help_win import CalibHelp

__author__ = 'caja'

FORWARD_TAB = 1
TURN_TAB = 2


class MainWindow(QDialog, Ui_Dialog):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        try:
            QMessageBox.warning(self, 'Warning',
                                'This Gui is for advance user only, If not use correctly it can damage the robot.\n When this Gui thurn on the RiCBoad will stop its functionality until you exit the gui.')
            self._protocol_handler = ProtocolHandler('RiCBoard')
            self._protocol_handler.in_config_mode()
            self._forwardClib = True
            self._turnClib = True

            self._old_forward_slide_max = 2000
            self._old_forward_slide_mid = 1500
            self._old_forward_slide_min = 1000
            self._old_turn_slide_max = 2000
            self._old_turn_slide_mid = 1500
            self._old_turn_slide_min = 1000

            self.finished.connect(self.on_close)
            self.saveBT.clicked.connect(self.on_save)
            self.loadBT.clicked.connect(self.on_load)
            self.leftClib.clicked.connect(self.on_forward_clib)
            self.leftSet.clicked.connect(self.on_forward_set)
            self.rightClib.clicked.connect(self.on_turn_clib)
            self.rightSet.clicked.connect(self.on_turn_set)
            self.leftClibSp.rangeChanged.connect(self.on_forward_range_change)
            self.leftClibSp.valueChanged.connect(self.on_forward_slide_value_change)
            self.righClibSp.rangeChanged.connect(self.on_turn_range_change)
            self.righClibSp.valueChanged.connect(self.on_turn_slide_value_change)

            self.forwardHelpB.clicked.connect(self.on_help_calib)
            self.turnHelpB.clicked.connect(self.on_help_calib)

        except serial.SerialException:
            QMessageBox.critical(self, "Error ",
                                 "can not connect to the device, please check if its connected to the computer.")

    def on_help_calib(self):
        help_calib = CalibHelp()
        help_calib.show()
        help_calib.exec_()

    def on_close(self, code):
        if self._protocol_handler.is_forward_clib():
            self._protocol_handler.send_clib_status(CANCEL)
            self._protocol_handler.set_forward_clib(False)

        if self._protocol_handler.is_turn_clib():
            self._protocol_handler.send_clib_status(CANCEL)
            self._protocol_handler.set_turn_clib(False)

        self._protocol_handler.in_config_mode()  # Exit the config mode

    def on_save(self):
        try:
            ch = self.takeoverCH.value()
            listen_mode = (self.takeoverCb.currentIndex() + 1)
            value = int(self.takeoverLe.text())
            status = self.takeoverEnable.isChecked()
            self._protocol_handler.send_channel_to_takeover_pkg(ch, listen_mode, value, status)
            forward_rev = NORMAL
            turn_rev = NORMAL
            if self.reverseCheckBox.isChecked():
                forward_rev = REVERSE
            if self.reverseCheckBox_2.isChecked():
                turn_rev = REVERSE
            self._protocol_handler.send_motor_to_takeover(self.rCChannelSpinBox.value(), self.maxCommandSpinBox.value(),
                                                          int(self.pinLineEdit.text()), FORWARD_TAB,
                                                          self.leftClibSp.minimum(), self.leftClibSp.value(),
                                                          self.leftClibSp.maximum(), False,
                                                          self.deadbandDoubleSpinBox.value(), forward_rev)
            self._protocol_handler.send_motor_to_takeover(self.rCChannelSpinBox_2.value(),
                                                          self.maxCommandSpinBox_2.value(),
                                                          int(self.pinLineEdit_2.text()), TURN_TAB,
                                                          self.righClibSp.minimum(), self.righClibSp.value(),
                                                          self.righClibSp.maximum(), False,
                                                          self.deadbandDoubleSpinBox_2.value(), turn_rev)
            QMessageBox.information(self, 'Info', 'Write is done.')

        except ValueError:
            QMessageBox.critical(self, "Error ", "Invalid value.")

    def on_load(self):
        self._protocol_handler.send_read_param()
        if self._protocol_handler.got_done():
            res_takeover = self._protocol_handler.load_channel_to_takeover()
            self.takeoverCH.setValue(res_takeover.get_channel())
            self.takeoverCb.setCurrentIndex(res_takeover.get_listen_mode() - 1)
            self.takeoverLe.setText(str(res_takeover.get_value()))
            self.takeoverEnable.setChecked(res_takeover.get_status())

            res_forward = self._protocol_handler.load_motor_to_takeover()
            self.rCChannelSpinBox.setValue(res_forward.get_channel())
            self.maxCommandSpinBox.setValue(res_forward.get_max_command())
            self.pinLineEdit.setText(str(res_forward.get_pin()))
            self.deadbandDoubleSpinBox.setValue(res_forward.get_deadband())
            self.leftClibSp.setMaximum(res_forward.get_max_joystick())
            self.leftClibSp.setMinimum(res_forward.get_min_joystick())
            self.leftClibSp.setValue(res_forward.get_mid_joystick())
            self.reverseCheckBox.setChecked(res_forward.is_reverse())
            self._old_forward_slide_max = res_forward.get_max_joystick()
            self._old_forward_slide_mid = res_forward.get_mid_joystick()
            self._old_forward_slide_min = res_forward.get_min_joystick()
            res_turn = self._protocol_handler.load_motor_to_takeover()
            self.rCChannelSpinBox_2.setValue(res_turn.get_channel())
            self.maxCommandSpinBox_2.setValue(res_turn.get_max_command())
            self.pinLineEdit_2.setText(str(res_turn.get_pin()))
            self.deadbandDoubleSpinBox_2.setValue(res_turn.get_deadband())
            self.righClibSp.setMaximum(res_turn.get_max_joystick())
            self.righClibSp.setMinimum(res_turn.get_min_joystick())
            self.righClibSp.setValue(res_turn.get_mid_joystick())
            self.reverseCheckBox_2.setChecked(res_turn.is_reverse())
            self._old_turn_slide_max = res_turn.get_max_joystick()
            self._old_turn_slide_mid = res_turn.get_mid_joystick()
            self._old_turn_slide_min = res_turn.get_min_joystick()
            QMessageBox.information(self, 'Done', 'Done reading the configuration')
        else:
            QMessageBox.critical(self, 'Error', 'RiCBoard now responding.'
                                                '\nPlease power off the robot and turn it back on.'
                                                '\nFor support contact this email: tom@robotican.net')

    def on_forward_clib(self):
        if self._forwardClib:
            rev_forward = NORMAL
            if self.reverseCheckBox.isChecked():
                rev_forward = REVERSE
            try:
                threading.Thread(target=self._protocol_handler.send_motor_to_takeover_and_clib, args=(
                self.rCChannelSpinBox.value(), self.maxCommandSpinBox.value(), int(self.pinLineEdit.text()),
                FORWARD_TAB, 2000, 1000, True, self.deadbandDoubleSpinBox.value(), rev_forward,
                self.leftClibSp)).start()
            except ValueError:
                QMessageBox.critical(self, "Error ", "Invalid value.")
            self.leftClib.setText("Cancel")
        else:
            self.leftClibSp.setMaximum(self._old_forward_slide_max)
            self.leftClibSp.setMinimum(self._old_forward_slide_min)
            self.leftClibSp.setValue(self._old_forward_slide_mid)
            self.leftClib.setText("Calibration")
            self._protocol_handler.send_clib_status(CANCEL)
        self._forwardClib = not self._forwardClib

    def on_forward_set(self):
        if not self._forwardClib:
            self._old_forward_slide_max = self.leftClibSp.maximum()
            self._old_forward_slide_mid = self.leftClibSp.value()
            self._old_forward_slide_min = self.leftClibSp.minimum()
            self._protocol_handler.send_clib_status(SAVE)
            self.leftClib.setText("Calibration")
            QMessageBox.information(self, 'Info', 'Calibration is done')
            self._forwardClib = True

    def on_turn_clib(self):
        if self._turnClib:
            rev_turn = NORMAL
            if self.reverseCheckBox_2.isChecked():
                rev_turn = REVERSE
            try:
                threading.Thread(target=self._protocol_handler.send_motor_to_takeover_and_clib, args=(
                self.rCChannelSpinBox_2.value(), self.maxCommandSpinBox_2.value(), int(self.pinLineEdit_2.text()),
                TURN_TAB, 2000, 1000, True, self.deadbandDoubleSpinBox_2.value(), rev_turn, self.righClibSp)).start()
            except ValueError:
                QMessageBox.critical(self, "Error ", "Invalid value.")
            self.rightClib.setText("Cancel")
        else:
            self.righClibSp.setMaximum(self._old_turn_slide_max)
            self.righClibSp.setMinimum(self._old_turn_slide_min)
            self.righClibSp.setValue(self._old_turn_slide_mid)
            self.rightClib.setText("Calibration")
            self._protocol_handler.send_clib_status(CANCEL)
        self._turnClib = not self._turnClib

    def on_turn_set(self):
        if not self._turnClib:
            self._old_turn_slide_max = self.righClibSp.maximum()
            self._old_turn_slide_mid = self.righClibSp.value()
            self._old_turn_slide_min = self.righClibSp.minimum()
            self._protocol_handler.send_clib_status(SAVE)
            self.rightClib.setText("Calibration")
            QMessageBox.information(self, 'Info', 'Calibration is done')
            self._turnClib = True

    def on_forward_range_change(self, min_slide, max_slide):
        self.leftMax.setText("Max: {0}".format(str(max_slide)))
        self.leftMin.setText("Min: {0}".format(str(min_slide)))

    def on_forward_slide_value_change(self, value):
        self.leftMid.setText("Mid: {0}".format(str(value)))

    def on_turn_range_change(self, min_slide, max_slide):
        self.rightMax.setText("Max: {0}".format(str(max_slide)))
        self.rightMin.setText("Min: {0}".format(str(min_slide)))

    def on_turn_slide_value_change(self, value):
        self.rightMid.setText("Mid: {0}".format(str(value)))
