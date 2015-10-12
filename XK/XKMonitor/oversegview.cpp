#include "oversegview.h"
#include "ui_oversegview.h"

OverSegView::OverSegView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::OverSegView)
{
    ui->setupUi(this);
    v.setBackgroundColor(0.7,0.7,0.7);
    ui->w->setMinimumSize(320,240);
    v.addCoordinateSystem(0.3);
    ui->viewGridLayout->addWidget(&widget);
    widget.SetRenderWindow(v.getRenderWindow());
}

void OverSegView::showFromFile(void)
{
    ;
}

void OverSegView::showFromProc(void)
{
    ;
}

OverSegView::~OverSegView()
{
    delete ui;
}
