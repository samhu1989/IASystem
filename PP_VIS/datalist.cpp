#include "datalist.h"
#include "ui_datalist.h"
#include "pipe.h"
#include <QMessageBox>
#include "pcdviewer.h"
DataList::DataList(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DataList)
{
    ui->setupUi(this);
    ui->tableView->setModel(&model);

    connect(
            ui->tableView->horizontalHeader(),
            SIGNAL(sortIndicatorChanged(int,Qt::SortOrder)),
            ui->tableView,
            SLOT(sortByColumn(int))
            );

    connect(
            ui->tableView->horizontalHeader(),
            SIGNAL(sortIndicatorChanged(int,Qt::SortOrder)),
            ui->tableView,
            SLOT(doItemsLayout())
            );

    connect(
            ui->tableView,
            SIGNAL(doubleClicked(QModelIndex)),
            this,
            SLOT(doubleClicked(QModelIndex))
            );
}

void DataList::closeEvent(QCloseEvent* e)
{
    QMessageBox msgbox;
    if(QMessageBox::Ok==msgbox.information(this,QString("Releasing Dump Data"),QString("Are you sure ?"),QMessageBox::Ok,QMessageBox::Cancel))
    {
        Pipe::_PipeData.clear();
    }else{
        e->ignore();
    }
}

void DataList::getSelected(QList<quint32>& keys)
{
    QModelIndexList list = ui->tableView->selectionModel()->selectedRows();
    QList<int> rowList;
    foreach(QModelIndex idx,list)
    {
        rowList.push_back( idx.row() );
    }
    keys = model.getHashKeyForRows(rowList);
}

void DataList::doubleClicked(QModelIndex index)
{
    quint32 key;
    QString info;
    model.getKey(index,info,key);
    if(info=="PCD")
    {
        PCDViewer *viewer;
        viewer = new PCDViewer();
        QList<quint32> keylist;
        keylist.push_back(key);
        viewer->addClouds(keylist,true);
        passToMdi((QWidget*)viewer);
    }
}

DataList::~DataList()
{
    delete ui;
}
