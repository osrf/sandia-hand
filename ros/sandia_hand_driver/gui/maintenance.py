#!/usr/bin/env python
import sys, os
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import roslib.packages
import roslib; roslib.load_manifest('sandia_hand_driver')
import rospy, math, signal
from sandia_hand_msgs.msg import RawFingerState

class BoardTab(QWidget):
  def __init__(self, board_name):
    super(BoardTab, self).__init__()
    self.board_name = board_name
    self.shd = roslib.packages.get_pkg_dir('sandia_hand_driver')
    self.driver_process = None
    self.terminal = QTextEdit()
    self.terminal.setStyleSheet("QTextEdit { background-color: black; color: white;}")
    self.test_grid = QGridLayout()
    self.test_grid.setSpacing(5)
    self.test_grid.setColumnMinimumWidth(0, 15)
    self.test_grid.setColumnStretch(2, 1)
  def processOnFinished(self, exitCode, exitStatus):
    if exitCode != 0:
      QMessageBox.about(self, "bad", "error detected. see terminal output.")
  def terminalReadOutput(self):
    #sys.stdout.write(self.process.readAllStandardOutput())
    self.terminal.append(QString(self.process.readAllStandardOutput()))
  def spawnProcess(self, cmd, onFinished):
    self.terminal.append(cmd + "\n")
    self.process = QProcess(self)
    self.process.finished.connect(onFinished)
    self.process.setProcessChannelMode(QProcess.MergedChannels)
    self.process.readyReadStandardOutput.connect(self.terminalReadOutput)
    self.process.start("/bin/bash", ["-c", cmd])
 
class MotorBoardTab(BoardTab):
  def __init__(self, board_name, exit_on_success):
    super(MotorBoardTab, self).__init__(board_name)
    self.bootloader_btn = QPushButton("Install Bootloader")
    self.bootloader_btn.clicked.connect(self.bootloaderClicked)
    self.application_btn = QPushButton("Install Application")
    self.application_btn.clicked.connect(self.applicationClicked)
    button_hbox = QHBoxLayout()
    button_hbox.addWidget(self.bootloader_btn)
    button_hbox.addWidget(self.application_btn)
    button_hbox.addStretch(1)
    vbox = QVBoxLayout()
    vbox.addLayout(button_hbox)
    #vbox.addLayout(self.test_grid)
    vbox.addStretch(1)
    vbox.addWidget(self.terminal)
    self.setLayout(vbox)
  def onUpdateUI(self, accel_raw):
    accel_mag = 0
    for i in xrange(0,3):
      accel_mag += accel_raw[i]**2
    accel_mag = math.sqrt(accel_mag)
  def bootloaderClicked(self):
    self.spawnProcess("cd %s/../../firmware/build && make fmcb-bl-gpnvm && make fmcb-bl-program && echo \"\ntasks complete\"" % (self.shd), self.processOnFinished)
  def applicationClicked(self):
    self.spawnProcess("cd %s && bin/loose_finger_cli /dev/ttyUSB0 burn ../../firmware/build/fmcb/std/fmcb-std.bin && echo \"\ntasks complete\"" % (self.shd), self.processOnFinished)

class TactileBoardTab(BoardTab):
  def __init__(self, board_name, num_taxels, exit_on_success):
    super(TactileBoardTab, self).__init__(board_name)
    self.num_taxels = num_taxels
    self.exit_on_success = exit_on_success
    self.bootloader_btn = QPushButton("Install Bootloader")
    self.bootloader_btn.clicked.connect(self.bootloaderClicked)
    self.application_btn = QPushButton("Install Application")
    self.application_btn.clicked.connect(self.applicationClicked)
    self.auto_btn = QPushButton("Awesome")
    self.auto_btn.clicked.connect(self.autoClicked)
    self.test_grid.addWidget(QLabel("Accelerometer magnitude"), 0, 2)
    self.accel_mag_light = QLabel() #"FAIL")
    self.accel_mag_light.setStyleSheet("QWidget {background-color:red}")
    self.accel_mag_label = QLabel("0")
    self.tactile_min = [65535] * self.num_taxels
    self.tactile_max = [0] * self.num_taxels
    self.tactile_labels = []
    self.tactile_lights = []
    self.tactile_desc = []
    self.test_grid.addWidget(self.accel_mag_light, 0, 0)
    self.test_grid.addWidget(self.accel_mag_label, 0, 1)
    for i in xrange(0, self.num_taxels):
      self.tactile_lights.append(QLabel())
      self.tactile_lights[i].setStyleSheet("QWidget {background-color:red}")
      self.test_grid.addWidget(self.tactile_lights[i], i+1, 0)
      self.tactile_labels.append(QLabel("0"))
      self.test_grid.addWidget(self.tactile_labels[i], i+1, 1)
      self.tactile_desc.append(QLabel("Tactile %d" % i))
      self.test_grid.addWidget(self.tactile_desc[i])
    button_hbox = QHBoxLayout()
    button_hbox.addWidget(self.bootloader_btn)
    button_hbox.addWidget(self.application_btn)
    button_hbox.addWidget(self.auto_btn)
    button_hbox.addStretch(1)
    self.vbox = QVBoxLayout()
    self.vbox.addLayout(button_hbox)
    self.vbox.addLayout(self.test_grid)
    self.vbox.addStretch(1)
    self.vbox.addWidget(self.terminal)
    self.setLayout(self.vbox)
  def onUpdateUI(self, accel_raw, tactile_raw):
    accel_mag = 0
    for i in xrange(0,3):
      accel_mag += accel_raw[i]**2
    accel_mag = math.sqrt(accel_mag)
    tactile_range = [0] * len(tactile_raw)
    for i in xrange(0, len(tactile_raw)):
      t = tactile_raw[i]
      if t > self.tactile_max[i]:
        self.tactile_max[i] = t
      if t < self.tactile_min[i]:
        self.tactile_min[i] = t
      tactile_range[i] = self.tactile_max[i] - self.tactile_min[i]
    self.accel_mag_label.setText("%d" % accel_mag)
    ready_to_exit = self.exit_on_success
    if (accel_mag > 800 and accel_mag < 1200):
      self.accel_mag_light.setStyleSheet("QWidget {background-color:green}")
    else:
      ready_to_exit = False
      self.accel_mag_light.setStyleSheet("QWidget {background-color:red}")
    for i in xrange(0, len(tactile_raw)):
      self.tactile_desc[i].setText("Tactile %02d: %d" % (i, tactile_raw[i]))
      self.tactile_labels[i].setText("%d" % tactile_range[i])
      if tactile_range[i] > 3000:
        self.tactile_lights[i].setStyleSheet("QWidget {background-color:green}")
      else:
        ready_to_exit = False
        self.tactile_lights[i].setStyleSheet("QWidget {background-color:red}")
    if ready_to_exit:
      if self.process:
        print "stopping child process"
        os.system("rosnode kill sandia_hand_loose_finger_node")
        print "sleeping..."
        os.system("sleep 0.25")
        print "quitting."
        self.driver_process = None
        QApplication.quit()
  def bootloaderClicked(self):
    self.spawnProcess("cd %s && bin/loose_finger_cli /dev/ttyUSB0 pb on && sleep 1 && cd %s/../../firmware/build && make %s-bl-gpnvm && make %s-bl-program && echo \"\ntasks complete\"" % (self.shd, self.shd, self.board_name, self.board_name), self.processOnFinished)
  def applicationClicked(self):
    self.spawnProcess("cd %s && bin/loose_finger_cli /dev/ttyUSB0 %s ../../firmware/build/%s/std/%s-std.bin && echo \"\ntasks complete\"" % (self.shd, self.applicationCmd(), bn, bn), self.processOnFinished)
  def autoOnFinished(self, exitCode, exitStatus):
    if exitCode != 0:
      QMessageBox.about(self, "bad!", "error detected. see terminal output.")
    if self.board_name == "f3":
      cmd = "cd %s && bin/sandia_hand_loose_finger_node _use_proximal_phalange:=false" % self.shd
    elif self.board_name == "f2":
      cmd = "cd %s && bin/sandia_hand_loose_finger_node _use_distal_phalange:=false" % self.shd
    else:
      QMessageBox.about(self, "bad!", "auto button not ready for this board")
      return
    self.spawnProcess(cmd, self.processOnFinished)
  def autoClicked(self):
    bn = self.board_name
    shd = self.shd
    self.spawnProcess("cd %s && bin/loose_finger_cli /dev/ttyUSB0 pb on && sleep 1 && cd %s/../../firmware/build && make %s-bl-gpnvm && make %s-bl-program && cd %s && bin/loose_finger_cli /dev/ttyUSB0 %s ../../firmware/build/%s/std/%s-std.bin && echo \"\ntasks complete\"" % (shd, shd, bn, bn, shd, self.applicationCmd(), bn, bn), self.autoOnFinished)
  def applicationCmd(self):
    bn = self.board_name
    if bn == "f3":
      return "dburn"
    elif bn == "f2":
      return "pburn"
    else:
      QMessageBox.about(self, "bad!", "application not defined for %s" % bn)
      return 

class MaintenanceWindow(QWidget):
  def __init__(self):
    super(MaintenanceWindow, self).__init__()
    self.tab_widget = QTabWidget()
    # todo: parse which tab is auto exit worthy
    auto_exit_board_name = rospy.get_param("~auto_exit_board_name", "")
    auto_awesome = rospy.get_param("~auto_awesome", False)
    self.f3_tab = TactileBoardTab("f3", 12, auto_exit_board_name == "f3") 
    self.f2_tab = TactileBoardTab("f2",  6, auto_exit_board_name == "f2")
    self.fmcb_tab = MotorBoardTab("fmcb", auto_exit_board_name == "fmcb")
    self.tab_widget.addTab(self.f3_tab, "F3")
    self.tab_widget.addTab(self.f2_tab, "F2")
    self.tab_widget.addTab(self.fmcb_tab, "FMCB")

    # todo: find a cleaner way to do this
    if auto_exit_board_name == "f2":
      self.tab_widget.setCurrentIndex(1) 
    elif auto_exit_board_name == "fmcb":
      self.tab_widget.setCurrentIndex(2)

    vbox = QVBoxLayout()
    vbox.addWidget(self.tab_widget)
    self.setGeometry(850, 100, 600, 600)
    self.setLayout(vbox)
    self.setWindowTitle('Sandia Hand Maintenance')
    #cp = QDesktopWidget().availableGeometry().center()
    #qr = self.frameGeometry()
    #qr.moveCenter(cp)
    #self.move(qr.topLeft())
    self.show()
    self.finger_sub = rospy.Subscriber('raw_state', RawFingerState, 
                                       self.finger_status_cb)
    self.connect(self, SIGNAL('updateF3'), self.f3_tab.onUpdateUI)
    self.connect(self, SIGNAL('updateF2'), self.f2_tab.onUpdateUI)
    self.connect(self, SIGNAL('updateFMCB'), self.fmcb_tab.onUpdateUI)
    if auto_awesome:
      if auto_exit_board_name == "f2":
        self.f2_tab.autoClicked()

  def finger_status_cb(self, msg):
    # copy everything out of the ROS thread and into UI threads
    f3_accel_raw = [0] * 3
    f2_accel_raw = [0] * 3
    fmcb_accel_raw = [0] * 3
    f3_tactile_raw = [0] * 12
    f2_tactile_raw = [0] *  6
    for i in xrange(0, 3):
      f3_accel_raw[i] = msg.dp_accel[i]
      f2_accel_raw[i] = msg.pp_accel[i]
      fmcb_accel_raw[i] = msg.fmcb_accel[i]
    for i in xrange(0, 12):
      f3_tactile_raw[i] = msg.dp_tactile[i]
    for i in xrange(0,  6):
      f2_tactile_raw[i] = msg.pp_tactile[i]
    self.emit(SIGNAL('updateF3'), f3_accel_raw, f3_tactile_raw)
    self.emit(SIGNAL('updateF2'), f2_accel_raw, f2_tactile_raw)
    self.emit(SIGNAL('updateFMCB'), fmcb_accel_raw)

if __name__ == '__main__':
  rospy.init_node('sandia_hand_maintenance')
  signal.signal(signal.SIGINT, signal.SIG_DFL) 
  app = QApplication(sys.argv)
  mw = MaintenanceWindow()
  sys.exit(app.exec_())

