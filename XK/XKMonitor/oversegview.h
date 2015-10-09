#ifndef OVERSEGVIEW_H
#define OVERSEGVIEW_H

#include <QWidget>

namespace Ui {
class OverSegView;
}

class OverSegView : public QWidget
{
    Q_OBJECT

public:
    explicit OverSegView(QWidget *parent = 0);
    ~OverSegView();

private:
    Ui::OverSegView *ui;
};

#endif // OVERSEGVIEW_H
