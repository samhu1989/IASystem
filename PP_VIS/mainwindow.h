#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "datalist.h"
#include "segview.h"
#include "featureview.h"
#include <QKeyEvent>
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
public slots:
    void keyPressEvent(QKeyEvent*);
    void loadDump(void);
    void saveDump(void);
    void startSeg(void);
    void startFeature(void);
    void doicp(void);
    void domap(void);
    void doformat(void);
    void reset(QObject*);
    void showInMdi(QWidget* w);
private:
    Ui::MainWindow *ui;
    DataList* datalist;
    SegView* segview;
    FeatureView* fview;
};

#endif // MAINWINDOW_H
