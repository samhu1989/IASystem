#ifndef FEATUREDIALOG_H
#define FEATUREDIALOG_H

#include <QDialog>
#include "common_global.h"
#include "pipe.h"

namespace Ui {
class FeatureDialog;
}

class COMMONSHARED_EXPORT FeatureDialog : public QDialog
{
    Q_OBJECT

public:
    explicit FeatureDialog(QWidget *parent = 0);
    ~FeatureDialog();
    void addFeature(cv::Mat&left,cv::Mat&right,std::string&name);
    void calcDists(void);
    void showDists(void);
protected:


private:
    Ui::FeatureDialog *ui;
    std::vector<cv::Mat> _FeatureL;
    std::vector<cv::Mat> _FeatureR;
    std::vector<std::string> _FeatureName;
    std::vector<std::vector<double>> _FeatureDists;
    std::vector<std::string> _DistName;
};

#endif // FEATUREDIALOG_H
