#include <QtGui>
#include <QMessageBox>
#include "dialog.h"
#include <string>
#include <vector>
#include <utility>
using std::string;
using std::vector;
using std::pair;
using std::make_pair;

Dialog::Dialog(QWidget *parent)
: QDialog(parent)
{
  tabWidget = new QTabWidget;
  tabWidget->addTab(new LoadFirmwareTab(this), tr("Load Firmware"));
  tabWidget->addTab(new FunctionalTestTab(this), tr("Functional Test"));
  buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok);
  connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
  //connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->setSizeConstraint(QLayout::SetNoConstraint);
  mainLayout->addWidget(tabWidget);
  mainLayout->addWidget(buttonBox);
  setLayout(mainLayout);
  setWindowTitle(tr("Sandia Hand"));
}

void Dialog::onFirmwareLoad(const QString &board_name_qstr)
{
  string board = board_name_qstr.toStdString();
  // first, program the bootloader
  string cmd = string("cd `rospack find sandia_hand_driver`/../../firmware/build && make ") + board + string("-bl-gpnvm && make ") + board + string("-bl-program");
  //if (board == string("f2") || board == string("f3"))
  //  cmd = "cd `rospack find sandia_hand_driver`/cli/loose_finger_cli 
  printf("%s\n", cmd.c_str());
  int rv = system(cmd.c_str());
  if (rv)
  {
    QMessageBox::critical(this, tr("Firmware Load Error"),
                          tr("Unable to program bootloader.\n"
                             "See terminal for details."));
    return;
  }
  if (board == string("fmcb"))
  {
    // install application image now
    cmd = string("cd `rospack find sandia_hand_driver` && bin/loose_finger_cli /dev/ttyUSB0 burn `rospack find sandia_hand_driver`/../../firmware/build/fmcb/std/fmcb-std.bin");
    rv = system(cmd.c_str());
    if (rv)
    {
      QMessageBox::critical(this, tr("Firmware Load Error"),
                            tr("Unable to load application image.\n"
                               "See terminal for details."));
      return;
    }
  }
  QMessageBox::information(this, tr("Firmware Load Complete"),
                           tr("Successfully loaded firmware."));
}

LoadFirmwareTab::LoadFirmwareTab(QWidget *parent)
: QWidget(parent)
{
  QVBoxLayout *mainLayout   = new QVBoxLayout;
  vector< pair<QPushButton *,QString> > buttons;
  buttons.push_back(make_pair(new QPushButton(tr("&Distal Phalange")),"f3"));
  buttons.push_back(make_pair(new QPushButton(tr("&Proximal Phalange")),"f2"));
  buttons.push_back(make_pair(new QPushButton(tr("&Motor Board")),"fmcb"));
  buttons.push_back(make_pair(new QPushButton(tr("&Right Palm")),"rpalm"));
  buttons.push_back(make_pair(new QPushButton(tr("&Left Palm")),"lpalm"));
  buttons.push_back(make_pair(new QPushButton(tr("Mother&board")),"mobo"));
  button_mapper = new QSignalMapper(this);
  for (size_t i = 0; i < buttons.size(); i++)
  {
    mainLayout->addWidget(buttons[i].first);
    button_mapper->setMapping(buttons[i].first, buttons[i].second);
    connect(buttons[i].first, SIGNAL(clicked()), button_mapper, SLOT(map()));
  }
  mainLayout->addStretch(1);
  setLayout(mainLayout);

  connect(button_mapper, SIGNAL(mapped(const QString &)),
          parent, SLOT(onFirmwareLoad(const QString &)));
}

FunctionalTestTab::FunctionalTestTab(QWidget *parent)
: QWidget(parent)
{
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addStretch(1);
  setLayout(mainLayout);
}
