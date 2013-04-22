#!/usr/bin/env python
import sys
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import roslib.packages
import roslib; roslib.load_manifest('sandia_hand_driver')
import rospy, math, signal
from sandia_hand_msgs.msg import RawFingerStatus

class MaintenanceWindow(QWidget):
  def __init__(self):
    super(MaintenanceWindow, self).__init__()
    self.bootloader_btn = QPushButton("Install Bootloader via JTAG")
    self.bootloader_btn.clicked.connect(self.bootloaderClicked)
    self.application_btn = QPushButton("Install Application over USB/serial")
    self.application_btn.clicked.connect(self.applicationClicked)
    self.terminal = QTextEdit()
    self.terminal.setStyleSheet("QTextEdit { background-color: black; color: white;}")
    tab_widget = QTabWidget()
    firmware_tab = QWidget()
    test_tab = QWidget()
    tab_widget.addTab(firmware_tab, "F3 Firmware")
    tab_widget.addTab(test_tab, "F3 Test")
    vbox = QVBoxLayout()
    vbox.addWidget(tab_widget)
    firmware_vbox = QVBoxLayout(firmware_tab)
    firmware_vbox.addWidget(self.bootloader_btn)
    firmware_vbox.addWidget(self.application_btn)
    firmware_vbox.addWidget(self.terminal)
    firmware_vbox.addStretch(1)

    test_grid = QGridLayout(test_tab)
    test_grid.setSpacing(5)
    test_grid.setColumnMinimumWidth(0, 15)
    test_grid.setColumnStretch(2, 1)
    test_grid.addWidget(QLabel("Accelerometer magnitude"), 0, 2)
    self.accel_mag_light = QLabel() #"FAIL")
    self.accel_mag_light.setStyleSheet("QWidget {background-color:red}")
    self.accel_mag_label = QLabel("0")
    self.tactile_min = [65535] * 12
    self.tactile_max = [0] * 12
    self.tactile_labels = []
    self.tactile_lights = []
    self.tactile_desc = []
    test_grid.addWidget(self.accel_mag_light, 0, 0)
    test_grid.addWidget(self.accel_mag_label, 0, 1)
    for i in xrange(0,12):
      self.tactile_lights.append(QLabel())
      self.tactile_lights[i].setStyleSheet("QWidget {background-color:red}")
      test_grid.addWidget(self.tactile_lights[i], i+1, 0)
      self.tactile_labels.append(QLabel("0"))
      test_grid.addWidget(self.tactile_labels[i], i+1, 1)
      self.tactile_desc.append(QLabel("Tactile %d" % i))
      test_grid.addWidget(self.tactile_desc[i])
    #test_vbox.addStretch(1)
    #hbox.addStretch(1)
    #vbox = QtGui.QVBoxLayout()
    #vbox.addStretch(1)
    #btn.resize(btn.sizeHint())
    #btn.move(50, 50)
    self.setGeometry(300, 300, 700, 300)
    self.setLayout(vbox)
    self.setWindowTitle('Sandia Hand Maintenance')
    self.shd = roslib.packages.get_pkg_dir('sandia_hand_driver')
    cp = QDesktopWidget().availableGeometry().center()
    qr = self.frameGeometry()
    qr.moveCenter(cp)
    self.move(qr.topLeft())
    self.show()
    self.finger_sub = rospy.Subscriber('raw_finger_status', RawFingerStatus, 
                                       self.finger_status_cb)
    self.connect(self, SIGNAL('updateUI'), self.onUpdateUI)
  def finger_status_cb(self, msg):
    #print "finger status"
    f3_accel_mag_squared = 0
    for i in xrange(0,3):
      f3_accel_mag_squared += msg.dp_accel[i]**2
    f3_accel_mag = math.sqrt(f3_accel_mag_squared)
    tactile_raw = [0]*12
    tactile_range = [0]*12
    for i in xrange(0,12):
      t = msg.dp_tactile[i]
      tactile_raw[i] = t
      if t > self.tactile_max[i]:
        self.tactile_max[i] = t
      if t < self.tactile_min[i]:
        self.tactile_min[i] = t
      tactile_range[i] = self.tactile_max[i] - self.tactile_min[i]
    self.emit(SIGNAL('updateUI'), f3_accel_mag, tactile_raw, tactile_range)
  def onUpdateUI(self, f3_accel_mag, tactile_raw, tactile_range):
    #print "onUpdateUI %d" % f3_accel_mag
    self.accel_mag_label.setText("%d" % f3_accel_mag)
    if (f3_accel_mag > 800 and f3_accel_mag < 1200):
      self.accel_mag_light.setStyleSheet("QWidget {background-color:green}")
      #self.accel_mag_light.setText("OK")
    else:
      self.accel_mag_light.setStyleSheet("QWidget {background-color:red}")
      #self.accel_mag_light.setText("FAIL")
    for i in xrange(0,12):
      self.tactile_desc[i].setText("Tactile %02d: %d" % (i, tactile_raw[i]))
      self.tactile_labels[i].setText("%d" % tactile_range[i])
      if tactile_range[i] > 3000:
        self.tactile_lights[i].setStyleSheet("QWidget {background-color:green}")
        #self.tactile_lights[i].setText("OK")
      else:
        self.tactile_lights[i].setStyleSheet("QWidget {background-color:red}")
        #self.tactile_lights[i].setText("FAIL")
  def bootloaderClicked(self):
    cmd = "cd %s && bin/loose_finger_cli /dev/ttyUSB0 pb on && sleep 1 && cd %s/../../firmware/build && make f3-bl-gpnvm && make f3-bl-program && echo \"\ntasks complete\"" % (self.shd, self.shd)
    self.process = QProcess(self)
    self.process.finished.connect(self.bootloaderOnFinished)
    self.process.setProcessChannelMode(QProcess.MergedChannels)
    self.process.readyReadStandardOutput.connect(self.terminalReadOutput)
    self.process.start("/bin/bash", ["-c", cmd])
  def bootloaderOnFinished(self, exitCode, exitStatus):
    #print "load bootloader finished, exitCode %d" % exitCode
    if exitCode != 0:
      QMessageBox.about(self, "error!", "error detected. see terminal output.")
  def applicationClicked(self):
    cmd = "cd %s && bin/loose_finger_cli /dev/ttyUSB0 dburn ../../firmware/build/f3/std/f3-std.bin && echo \"\ntasks complete\"" % (self.shd)
    self.process = QProcess(self)
    self.process.finished.connect(self.bootloaderOnFinished)
    self.process.setProcessChannelMode(QProcess.MergedChannels)
    self.process.readyReadStandardOutput.connect(self.terminalReadOutput)
    self.process.start("/bin/bash", ["-c", cmd])
  def terminalReadOutput(self):
    #sys.stdout.write(self.process.readAllStandardOutput())
    self.terminal.append(QString(self.process.readAllStandardOutput()))
  def bootloaderOnReadErrors(self):
    print "readerrors"

if __name__ == '__main__':
  rospy.init_node('sandia_hand_maintenance')
  signal.signal(signal.SIGINT, signal.SIG_DFL) 
  app = QApplication(sys.argv)
  mw = MaintenanceWindow()
  sys.exit(app.exec_())
