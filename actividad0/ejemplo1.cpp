#include "ejemplo1.h"

char running = false;
int valor = 0;

ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();
	connect(Start, SIGNAL(clicked()), this, SLOT(doStart()));
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );
	connect(&timer, SIGNAL(timeout()), this, SLOT(doCount()));
	timer.start(500);

}

void ejemplo1::doStart()
{
	if (running)
	{
		running = false;
		Start->setText("START");
	}
	else
	{
		running = true;
		Start->setText("STOP");
	}
}

void ejemplo1::doCount()
{
	if (running)
	{
		lcdNumber-> display(valor++);
	}
}

void ejemplo1::doButton()
{
	valor = 0;
	lcdNumber-> display(valor);
}




