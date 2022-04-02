#ifndef DISPLAY_ACTIONS
#define DISPLAY_ACTIONS

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


class DisplayActions
{
private:
    Adafruit_SSD1306 *display;
    int progressPos;
    int maxProgress;
    bool needRedraw;
public:
    DisplayActions(Adafruit_SSD1306*);
    ~DisplayActions();
    void redrawLitingTime(int timeSec);
    void redrawTemperature(float temp);
    void nextProgress();
    void disableProgress();
    void turnOff();
    void redraw();
    void redraw(bool force);
    void drawLightStatus(bool);
};


#endif