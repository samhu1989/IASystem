#include "oversegview.h"
#include "ui_oversegview.h"

OverSegView::OverSegView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::OverSegView)
{
    ui->setupUi(this);
}

OverSegView::~OverSegView()
{
    delete ui;
}
