#include "ejemplo1.h"
#include <thread>
#include <chrono>
#include <iostream>

char running = false;
int valor = 0;

ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();
	connect(start, SIGNAL(clicked()), this, SLOT(doStart()));
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );
	connect(horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(setInterval()) );
	timer.connect(std::bind(&ejemplo1::doCount,this));
}

void ejemplo1::doStart()
{
	if (running)
	{
		timer.stop();
		running = false;
		start->setText("START");
	}
	else
	{
		timer.start(5);
		running = true;
		start->setText("STOP");
	}
}

void ejemplo1::doCount()
{
	lcdNumber-> display(valor++);
}

void ejemplo1::doButton()
{
	valor = 0;
	lcdNumber-> display(valor);
}

void ejemplo1::setInterval()
{
	lcdNumber_2-> display(horizontalSlider->value());
	timer.setInterval(horizontalSlider->value());
}




