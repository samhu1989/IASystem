#include "featuredialog.h"
#include "ui_featuredialog.h"
#include "histogram.h"
#include <QTableWidgetItem>
FeatureDialog::FeatureDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::FeatureDialog)
{
    ui->setupUi(this);
}

void FeatureDialog::addFeature(cv::Mat&left,cv::Mat&right,std::string&name)
{
    _FeatureL.push_back(cv::Mat());
    left.copyTo(_FeatureL.back());
    _FeatureR.push_back(cv::Mat());
    right.copyTo(_FeatureR.back());
    _FeatureName.push_back(name);
}

void FeatureDialog::calcDists(void)
{
    unsigned int idx;
    _FeatureDists.push_back(std::vector<double>());
    for(idx=0;idx<_FeatureL.size();++idx)
    {
        _FeatureDists[0].push_back(Histogram::getEuclidean(_FeatureL[idx],_FeatureR[idx]));
    }
    _DistName.push_back(std::string("Euclidean"));
    _FeatureDists.push_back(std::vector<double>());
    for(idx=0;idx<_FeatureL.size();++idx)
    {
        _FeatureDists[1].push_back(Histogram::getCosine(_FeatureL[idx],_FeatureR[idx]));
    }
    _DistName.push_back(std::string("Cosine"));
}

void FeatureDialog::showDists(void)
{
    ui->tableWidget->setColumnCount(_FeatureL.size());
    ui->tableWidget->setRowCount(_DistName.size());
    QStringList hHead;
    foreach(std::string name,_FeatureName)
    {
        hHead<<QString::fromStdString(name);
    }
    ui->tableWidget->setHorizontalHeaderLabels(hHead);
    QStringList vHead;
    foreach(std::string name,_DistName)
    {
        vHead<<QString::fromStdString(name);
    }
    ui->tableWidget->setVerticalHeaderLabels(vHead);
    unsigned int r,c;
    for(r=0;r<_FeatureDists.size();++r)
    {
        for(c=0;c<_FeatureDists[r].size();++c)
        {
            QString value;
            value = value.sprintf("%lf",_FeatureDists[r][c]);
            QTableWidgetItem* item = new QTableWidgetItem(value);
            ui->tableWidget->setItem( r,c,item );
        }
    }
}

FeatureDialog::~FeatureDialog()
{
    delete ui;
}
