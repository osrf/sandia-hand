#include <QtGui>
#include <QMessageBox>
#include <QComboBox>
#include <QGroupBox>
#include "homing_dialog.h"
#include <string>
#include <vector>
#include <utility>
#include <osrf_msgs/JointCommands.h>
using std::string;
using std::vector;
using std::pair;
using std::make_pair;
double ManualFingerSubtab::SLIDER_TICKS_PER_DEGREE = 2;

HomingDialog::HomingDialog(QWidget *parent)
: QDialog(parent)
{
  tabs_ = new QTabWidget;
  tabs_->addTab(new ManualTab(this, nh_), tr("Manual"));
  tabs_->addTab(new AutoTab(this), tr("Auto"));
  //button_box_ = new QDialogButtonBox(QDialogButtonBox::Ok);
  //connect(button_box_, SIGNAL(accepted()), this, SLOT(accept()));
  //connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
  QVBoxLayout *main_layout = new QVBoxLayout;
  main_layout->setSizeConstraint(QLayout::SetNoConstraint);
  main_layout->addWidget(tabs_);
  //main_layout->addWidget(button_box_);
  main_layout->addStretch(1);
  setLayout(main_layout);
  setWindowTitle(tr("Homing"));
}

/*
void Dialog::onFirmwareOperationChanged(const QString &operation_qstr)
{
  string op = operation_qstr.toStdString();
  printf("onFirmwareOperationChanged(%s)\n", op.c_str());
  firmware_operation_ = op;
}

void Dialog::onFirmwareButton(const QString &board_name_qstr)
{
  string board = board_name_qstr.toStdString();
  printf("onFirmwareButton(%s)\n", board.c_str());
  string cmd;
  if (firmware_operation_.find("ootloader") != std::string::npos) // fix this
  {
    cmd = string("cd `rospack find sandia_hand_driver`/../../firmware/build && make ") + board + string("-bl-gpnvm && make ") + board + string("-bl-program");
    if (board == string("mobo"))
      cmd = string("cd `rospack find sandia_hand_driver`/../../firmware/build && make mobo-mcu-set_boot_vector && make mobo-mcu-program");
  }
  else
  {
    if (board == string("mobo"))
      cmd = string("cd `rospack find sandia_hand_driver` && bin/sandia_hand_cli mmcu_burn ../../firmware/build/mobo/mcu/mobo-mcu.bin");
    else if (board == string("fmcb"))
      cmd = string("cd `rospack find sandia_hand_driver` && bin/sandia_hand_cli mmburn 0 ../../firmware/build/fmcb/std/fmcb-std.bin"); // todo: finger_idx
    //cmd = string("cd `rospack find sandia_hand_driver` && bin/loose_finger_cli /dev/ttyUSB0 burn `rospack find sandia_hand_driver`/../../firmware/build/fmcb/std/fmcb-std.bin");
  }
  if (cmd.length() == 0)
  {
    QMessageBox::critical(this, tr("Operation currently not implemented"),
                          tr("Operation currently not implemented."));
    return;
  }

  //if (board == string("f2") || board == string("f3"))
  //  cmd = "cd `rospack find sandia_hand_driver`/cli/loose_finger_cli 
  printf("%s\n", cmd.c_str());
  int rv = system(cmd.c_str());
  if (rv)
  {
    QMessageBox::critical(this, tr("Firmware Load Error"),
                          tr("See terminal for details."));
    return;
  }
  QMessageBox::information(this, tr("Firmware Load Complete"),
                           tr("Successfully loaded firmware."));
}
*/

ManualFingerSubtab::ManualFingerSubtab(QWidget *parent,
                                    ros::Publisher *finger_joint_commands_pub)
: QWidget(parent),
  finger_joint_commands_pub_(finger_joint_commands_pub)
{
  QGridLayout *grid = new QGridLayout(this);  
  const char *row_labels[3] = { "Abduction/Adduction:", 
                                "Proximal Flexion:", 
                                "Distal Flexion:" };
  for (int i = 0; i < 3; i++)
  {
    grid->addWidget(new QLabel(row_labels[i]), i, 0, Qt::AlignRight);
    sb_[i] = new QScrollBar(Qt::Horizontal);
    sb_[i]->setFocusPolicy(Qt::StrongFocus);
    sb_[i]->setMinimum(-120 * SLIDER_TICKS_PER_DEGREE);
    sb_[i]->setMaximum( 120 * SLIDER_TICKS_PER_DEGREE);
    sb_[i]->setValue(0);
    grid->addWidget(sb_[i], i, 1);
    connect(sb_[i], SIGNAL(valueChanged(int)), this, SLOT(sendFingerPose())); 
  }
  grid->addWidget(home_button_ = new QPushButton(tr("Set Finger Home")), 
                  3, 1, Qt::AlignCenter);
  connect(home_button_, SIGNAL(clicked()), this, SLOT(setFingerHome()));
  grid->setColumnMinimumWidth(1, 200);
  setLayout(grid);
}

void ManualFingerSubtab::sendFingerPose()
{
  osrf_msgs::JointCommands jc;
  jc.position.resize(3);
  for (int i = 0; i < 3; i++)
    jc.position[i] = (double)sb_[i]->value() / SLIDER_TICKS_PER_DEGREE * 
                     3.14 / 180.0;
  printf("finger pose: %.3f %.3f %.3f\n", 
         jc.position[0], jc.position[1], jc.position[2]);
  finger_joint_commands_pub_->publish(jc);
}

void ManualFingerSubtab::setFingerHome()
{
  printf("ManualFingerSubtab::setFingerHome\n");
}

ManualTab::ManualTab(QWidget *parent, ros::NodeHandle &nh)
: QWidget(parent),
  nh_(nh)
{
  for (int i = 0; i < 4; i++)
  {
    char topic[100];
    snprintf(topic, sizeof(topic), "finger_%d/joint_commands", i);
    finger_pubs_[i] = nh_.advertise<osrf_msgs::JointCommands>(topic, 1);
  }
  QVBoxLayout *main_layout = new QVBoxLayout;
  tabs_ = new QTabWidget;
  const char *tab_names[4] = { "Index", "Middle", "Pinkie", "Thumb" };
  for (int i = 0; i < 4; i++)
    tabs_->addTab(finger_tabs_[i] = 
                          new ManualFingerSubtab(this, &finger_pubs_[i]), 
                  tr(tab_names[i]));
  main_layout->addWidget(tabs_);
  main_layout->addStretch(1);
  setLayout(main_layout);
  //buttons.push_back(make_pair(new QPushButton(tr("&Distal Phalange")),"f3"));

/*
  vector< pair<QPushButton *,QString> > buttons;
  QComboBox *cb = new QComboBox; //(this);
  cb->addItem(tr("Update Firmware"));
  cb->addItem(tr("Install Bootloader"));
  QHBoxLayout *op_layout = new QHBoxLayout;
  op_layout->addWidget(new QLabel("Operation:"));
  op_layout->addWidget(cb);
  mainLayout->addLayout(op_layout);
  buttons.push_back(make_pair(new QPushButton(tr("&Distal Phalange")),"f3"));
  buttons.push_back(make_pair(new QPushButton(tr("&Proximal Phalange")),"f2"));
  buttons.push_back(make_pair(new QPushButton(tr("&Motor Board")),"fmcb"));
  buttons.push_back(make_pair(new QPushButton(tr("&Right Palm")),"rpalm"));
  buttons.push_back(make_pair(new QPushButton(tr("&Left Palm")),"lpalm"));
  buttons.push_back(make_pair(new QPushButton(tr("Mother&board")),"mobo"));
  button_mapper = new QSignalMapper(this);
  QGroupBox *gb = new QGroupBox(tr("Click Board Name to Perform Operation"));
  QVBoxLayout *gb_layout = new QVBoxLayout();
  for (size_t i = 0; i < buttons.size(); i++)
  {
    gb_layout->addWidget(buttons[i].first);
    button_mapper->setMapping(buttons[i].first, buttons[i].second);
    connect(buttons[i].first, SIGNAL(clicked()), button_mapper, SLOT(map()));
  }
  gb->setLayout(gb_layout);
  mainLayout->addWidget(gb);
  mainLayout->addStretch(1);
  setLayout(mainLayout);

  connect(button_mapper, SIGNAL(mapped(const QString &)),
          parent, SLOT(onFirmwareButton(const QString &)));
  connect(cb, SIGNAL(currentIndexChanged(const QString &)), 
          parent, SLOT(onFirmwareOperationChanged(const QString &)));
  */
}

AutoTab::AutoTab(QWidget *parent)
: QWidget(parent)
{
  QVBoxLayout *main_layout = new QVBoxLayout;
  main_layout->addStretch(1);
  setLayout(main_layout);
}

