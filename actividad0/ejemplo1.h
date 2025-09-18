#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "ui_counterDlg.h"
#include "timer.h"

class ejemplo1 : public QWidget, public Ui_Counter
{
    Q_OBJECT
    public:
        ejemplo1();

    public slots:
        void doButton();
        void doStart();
        void doCount();
        void setInterval();

    private:
        Timer timer;


};

#endif // ejemplo1_H
