#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "pipe.h"
#include <QHash>
#include <QLocalSocket>
#include <QLocalServer>
#include <QProcess>
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
protected slots:
    void loadConfig(void);
    void newSocket(void);
    void send(void);
    void recieve(void);
    void respond(QLocalSocket*,QString&,QByteArray&);

    void triggerOverSeg(void);
    void finishOverSeg(int,QProcess::ExitStatus);
    void printProcessStdCout(void);
    void printProcessStdCerr(void);
    void printProcessError(QProcess::ProcessError);
private:
    Ui::MainWindow *ui;
    QLocalServer* _Server;
    Config* _Config;
    QHash<QString,QLocalSocket*> _Sockets;
};

#endif // MAINWINDOW_H
