#!/usr/bin/env python
import sys, os
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import roslib.packages
import roslib; roslib.load_manifest('sandia_hand_driver')
import rospy, math, signal
from sandia_hand_msgs.msg import RawFingerState

class MaintenanceWindow(QWidget):
  def __init__(self):
    super(MaintenanceWindow, self).__init__()
    self.f3_bootloader_btn = QPushButton("Install Bootloader via JTAG")
    self.f3_bootloader_btn.clicked.connect(self.f3BootloaderClicked)
    self.f3_application_btn = QPushButton("Install Application over USB/serial")
    self.f3_application_btn.clicked.connect(self.f3ApplicationClicked)
    self.f3_go_btn = QPushButton("Awesome")
    self.f3_go_btn.clicked.connect(self.f3GoClicked)
    self.f3_terminal = QTextEdit()
    self.f3_terminal.setStyleSheet("QTextEdit { background-color: black; color: white;}")
    self.driver_process = None
    self.tab_widget = QTabWidget()
    f3_firmware_tab = QWidget()
    f3_test_tab = QWidget()
    self.tab_widget.addTab(f3_firmware_tab, "F3 Firmware")
    self.tab_widget.addTab(f3_test_tab, "F3 Test")
    f3_firmware_vbox = QVBoxLayout(f3_firmware_tab)
    f3_firmware_vbox.addWidget(self.f3_bootloader_btn)
    f3_firmware_vbox.addWidget(self.f3_application_btn)
    f3_firmware_vbox.addWidget(self.f3_go_btn)
    f3_firmware_vbox.addWidget(self.f3_terminal)
    f3_firmware_vbox.addStretch(1)
    f3_test_grid = QGridLayout(f3_test_tab)
    f3_test_grid.setSpacing(5)
    f3_test_grid.setColumnMinimumWidth(0, 15)
    f3_test_grid.setColumnStretch(2, 1)
    f3_test_grid.addWidget(QLabel("Accelerometer magnitude"), 0, 2)
    self.f3_accel_mag_light = QLabel() #"FAIL")
    self.f3_accel_mag_light.setStyleSheet("QWidget {background-color:red}")
    self.f3_accel_mag_label = QLabel("0")
    self.f3_tactile_min = [65535] * 12
    self.f3_tactile_max = [0] * 12
    self.f3_tactile_labels = []
    self.f3_tactile_lights = []
    self.f3_tactile_desc = []
    f3_test_grid.addWidget(self.f3_accel_mag_light, 0, 0)
    f3_test_grid.addWidget(self.f3_accel_mag_label, 0, 1)
    for i in xrange(0,12):
      self.f3_tactile_lights.append(QLabel())
      self.f3_tactile_lights[i].setStyleSheet("QWidget {background-color:red}")
      f3_test_grid.addWidget(self.f3_tactile_lights[i], i+1, 0)
      self.f3_tactile_labels.append(QLabel("0"))
      f3_test_grid.addWidget(self.f3_tactile_labels[i], i+1, 1)
      self.f3_tactile_desc.append(QLabel("Tactile %d" % i))
      test_grid.addWidget(self.tactile_desc[i])

    vbox = QVBoxLayout()
    vbox.addWidget(self.tab_widget)
    self.setGeometry(850, 100, 600, 300)
    self.setLayout(vbox)
    self.setWindowTitle('Sandia Hand Maintenance')
    self.shd = roslib.packages.get_pkg_dir('sandia_hand_driver')
    #cp = QDesktopWidget().availableGeometry().center()
    #qr = self.frameGeometry()
    #qr.moveCenter(cp)
    #self.move(qr.topLeft())
    self.show()
    self.finger_sub = rospy.Subscriber('raw_state', RawFingerState, 
                                       self.finger_status_cb)
    self.connect(self, SIGNAL('updateUI'), self.onUpdateUI)
  def finger_status_cb(self, msg):
    #print "finger status"
    f3_accel_mag_squared = 0
    for i in xrange(0,3):
      f3_accel_mag_squared += msg.dp_accel[i]**2
    f3_accel_mag = math.sqrt(f3_accel_mag_squared)
    f3_tactile_raw = [0]*12
    f3_tactile_range = [0]*12
    for i in xrange(0,12):
      t = msg.dp_tactile[i]
      f3_tactile_raw[i] = t
      if t > self.f3_tactile_max[i]:
        self.f3_tactile_max[i] = t
      if t < self.f3_tactile_min[i]:
        self.f3_tactile_min[i] = t
      f3_tactile_range[i] = self.f3_tactile_max[i] - self.f3_tactile_min[i]
    self.emit(SIGNAL('updateUI'), f3_accel_mag, f3_tactile_raw, f3_tactile_range)
  def onUpdateUI(self, f3_accel_mag, f3_tactile_raw, f3_tactile_range):
    #print "onUpdateUI %d" % f3_accel_mag
    self.f3_accel_mag_label.setText("%d" % f3_accel_mag)
    #all_ok = True
    all_ok = False
    if (f3_accel_mag > 800 and f3_accel_mag < 1200):
      self.f3_accel_mag_light.setStyleSheet("QWidget {background-color:green}")
      #self.accel_mag_light.setText("OK")
    else:
      all_ok = False
      self.f3_accel_mag_light.setStyleSheet("QWidget {background-color:red}")
      #self.accel_mag_light.setText("FAIL")
    for i in xrange(0,12):
      self.f3_tactile_desc[i].setText("Tactile %02d: %d" % (i, f3_tactile_raw[i]))
      self.f3_tactile_labels[i].setText("%d" % f3_tactile_range[i])
      if f3_tactile_range[i] > 3000:
        self.f3_tactile_lights[i].setStyleSheet("QWidget {background-color:green}")
        #self.tactile_lights[i].setText("OK")
      else:
        all_ok = False
        self.f3_tactile_lights[i].setStyleSheet("QWidget {background-color:red}")
        #self.tactile_lights[i].setText("FAIL")
    if all_ok:
      if self.driver_process:
        print "stopping driver process"
        #self.driver_process.terminate() 
        #print "sleeping..."
        #cmd = "cd %s && bin/loose_finger_cli /dev/ttyUSB0 pb on && sleep 1 && cd %s/../../firmware/build && make f3-bl-gpnvm && make f3-bl-program && echo \"\ntasks complete\"" % (self.shd, self.shd)
        #self.process = QProcess(self)
        #self.process.finished.connect(self.bootloaderOnFinished)
        #self.process.setProcessChannelMode(QProcess.MergedChannels)
        #self.process.readyReadStandardOutput.connect(self.terminalReadOutput)
        #self.process.start("/bin/bash", ["-c", cmd])
        os.system("rosnode kill sandia_hand_loose_finger_node")
        print "sleeping..."
        os.system("sleep 0.25")
        print "quitting."
        self.driver_process = None
        QApplication.quit()
  def f3BootloaderClicked(self):
    cmd = "cd %s && bin/loose_finger_cli /dev/ttyUSB0 pb on && sleep 1 && cd %s/../../firmware/build && make f3-bl-gpnvm && make f3-bl-program && echo \"\ntasks complete\"" % (self.shd, self.shd)
    self.process = QProcess(self)
    self.process.finished.connect(self.f3BootloaderOnFinished)
    self.process.setProcessChannelMode(QProcess.MergedChannels)
    self.process.readyReadStandardOutput.connect(self.f3TerminalReadOutput)
    self.process.start("/bin/bash", ["-c", cmd])
  def f3BootloaderOnFinished(self, exitCode, exitStatus):
    #print "load bootloader finished, exitCode %d" % exitCode
    if exitCode != 0:
      QMessageBox.about(self, "error!", "error detected. see terminal output.")
  def f3GoOnFinished(self, exitCode, exitStatus):
    if exitCode != 0:
      QMessageBox.about(self, "error!", "error detected. see terminal output.")
    else:
      self.tab_widget.setCurrentIndex(1)
    self.driver_process = QProcess(self)
    cmd = "cd %s && bin/sandia_hand_loose_finger_node _use_proximal_phalange:=false" % self.shd
    self.driver_process.setProcessChannelMode(QProcess.MergedChannels)
    self.driver_process.start("/bin/bash", ["-c", cmd])
  def f3GoClicked(self):
    cmd = "cd %s && bin/loose_finger_cli /dev/ttyUSB0 pb on && sleep 1 && cd %s/../../firmware/build && make f3-bl-gpnvm && make f3-bl-program && cd %s && bin/loose_finger_cli /dev/ttyUSB0 dburn ../../firmware/build/f3/std/f3-std.bin && echo \"\ntasks complete\"" % (self.shd, self.shd, self.shd)
    self.process = QProcess(self)
    self.process.finished.connect(self.f3GoOnFinished)
    self.process.setProcessChannelMode(QProcess.MergedChannels)
    self.process.readyReadStandardOutput.connect(self.f3TerminalReadOutput)
    self.process.start("/bin/bash", ["-c", cmd])
  def f3ApplicationClicked(self):
    cmd = "cd %s && bin/loose_finger_cli /dev/ttyUSB0 dburn ../../firmware/build/f3/std/f3-std.bin && echo \"\ntasks complete\"" % (self.shd)
    self.process = QProcess(self)
    self.process.finished.connect(self.f3BootloaderOnFinished)
    self.process.setProcessChannelMode(QProcess.MergedChannels)
    self.process.readyReadStandardOutput.connect(self.f3TerminalReadOutput)
    self.process.start("/bin/bash", ["-c", cmd])
  def f3TerminalReadOutput(self):
    #sys.stdout.write(self.process.readAllStandardOutput())
    self.f3_terminal.append(QString(self.process.readAllStandardOutput()))
  def bootloaderOnReadErrors(self):
    print "readerrors"

if __name__ == '__main__':
  rospy.init_node('sandia_hand_maintenance')
  signal.signal(signal.SIGINT, signal.SIG_DFL) 
  app = QApplication(sys.argv)
  mw = MaintenanceWindow()
  sys.exit(app.exec_())
  if mw.driver_process:
    print "stopping driver process on exit"
    mw.driver_process.terminate() 
