#ifndef DATALIST_H
#define DATALIST_H

#include <QWidget>
#include <QAbstractItemModel>
#include <QCloseEvent>
#include "datalistmodel.h"
namespace Ui {
class DataList;
}

class DataList : public QWidget
{
    Q_OBJECT

public:
    explicit DataList(QWidget *parent = 0);
    ~DataList();
signals:
    void passToMdi(QWidget*);
public slots:
    void getSelected(QList<quint32>&);
    void doubleClicked(QModelIndex);
protected:
    void closeEvent(QCloseEvent*);
private:
    Ui::DataList *ui;
    DataListModel model;

};

#endif // DATALIST_H
