#include "display_actions.h"


void DisplayActions::redrawLitingTime(int timeSec){
    display->fillRect(5, 0, 90, 46, BLACK);
    display->setTextColor(WHITE);
    display->setTextSize(3);
    display->setCursor(5, 15);
    display->print(timeSec/60/10);
    display->print(timeSec/60%10);
    display->print(":");
    display->print(timeSec%60/10);
    display->print(timeSec%60%10);
    needRedraw = true;
}


void DisplayActions::redrawTemperature(float temp){
    display->fillRect(5, 47, 89, 32, BLACK);
    display->setTextColor(WHITE);
    display->setTextSize(2);
    display->setCursor(5, 48);
    display->print(int(temp));
    display->print(".");
    display->print(int(temp*10)%10);
    display->print("C");
    needRedraw = true;
}


void DisplayActions::nextProgress(){
    if (progressPos > 0){
        display->fillRect(progressPos*12, 5, 6, 6, BLACK);
    }
    progressPos += 1;
    if (progressPos > maxProgress){
        progressPos = 1;
    }
    display->fillRect(progressPos*12, 5, 6, 6, WHITE);
    needRedraw = true;
}


void DisplayActions::drawLightStatus(bool isOn){
    display->fillRect(100, 5, 32, 34, BLACK);
    display->drawRect(100, 5, 26, 34, WHITE);
    if (isOn){
        display->fillRect(106, 10, 14, 24, WHITE);
    }
    needRedraw = true;
}


void DisplayActions::disableProgress(){
    if (progressPos > 0){
        display->fillRect(progressPos*12, 5, 6, 6, BLACK);
        needRedraw = true;
    }
}


void DisplayActions::turnOff(){
    display->fillRect(0,0, 128, 64, BLACK);
    needRedraw = true;
}


void DisplayActions::redraw(){
    if (needRedraw){
        display->display();
        needRedraw = false;
    }
}


void DisplayActions::redraw(bool force){
    if (force){
        display->display();
        needRedraw = false;
    }
}


DisplayActions::DisplayActions(Adafruit_SSD1306 *dsp)
{
    display = dsp;
    progressPos = 1;
    maxProgress = 128/12;
    needRedraw = true;
};


DisplayActions::~DisplayActions()
{
    display = nullptr;
};