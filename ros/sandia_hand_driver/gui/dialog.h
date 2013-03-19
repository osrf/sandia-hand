#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include <QSignalMapper>

class QDialogButtonBox;
class QFileInfo;
class QTabWidget;

class LoadFirmwareTab : public QWidget
{
  Q_OBJECT
public:
  LoadFirmwareTab(QWidget *parent = 0);
  QSignalMapper *button_mapper;
};

class FunctionalTestTab : public QWidget
{
  Q_OBJECT
public:
  FunctionalTestTab(QWidget *parent = 0);
};

class Dialog : public QDialog
{
  Q_OBJECT
public:
  Dialog(QWidget *parent = 0);
public slots:
  void onFirmwareLoad(const QString &board_name);
private:
  QTabWidget       *tabWidget;
  QDialogButtonBox *buttonBox;
};

#endif

