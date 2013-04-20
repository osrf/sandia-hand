#!/usr/bin/env python
import sys
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import roslib.packages

class MaintenanceWindow(QWidget):
  def __init__(self):
    super(MaintenanceWindow, self).__init__()
    self.bootloader_btn = QPushButton("Install Bootloader via JTAG")
    self.bootloader_btn.clicked.connect(self.bootloaderClicked)
    self.application_btn = QPushButton("Install Application over USB/serial")
    self.application_btn.clicked.connect(self.applicationClicked)
    self.terminal = QTextEdit()
    self.terminal.setStyleSheet("QTextEdit { background-color: black; color: white;}")
    vbox = QVBoxLayout()
    vbox.addWidget(self.bootloader_btn)
    vbox.addWidget(self.application_btn)
    vbox.addWidget(self.terminal)
    vbox.addStretch(1)
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
  app = QApplication(sys.argv)
  mw = MaintenanceWindow()
  sys.exit(app.exec_())
