#ifndef STATE_MACHINE
#define STATE_MACHINE

#include "display_actions.h"

const int MAX_SEC = 60 * 60; // 60 min to 60 sec
const int TIME_DIFF_SEC = 10;


class StateMachine
{
private:
    volatile int displayTimerSecLeftDefault, lightTimerSecLeftDefault;
    volatile int displayTimerSecLeft;
    volatile int lightTimerSecLeft;
    volatile bool lightTimeChanged;
    volatile bool needStratTimer, needStopTimer, needPauseTimer;
    volatile bool lightOn, timerOn;
    volatile bool needTurnOnDisplay, needTurnOffDisplay, displayOn;
    short const relayPin, piezoPin;
    float temperature, prevTemperature;
    volatile bool tempChanged;
    DisplayActions *dispActions;
    float convertToCelsium();
    void beep(int f, int d){tone(piezoPin, f, d);}
public:
    StateMachine(DisplayActions*, int, int, short, short);
    ~StateMachine();

    void startTimer();
    void stopTimer();
    void pauseTimer();
    void toggleTimer(bool);

    void turnOnDisplay();
    void turnOffDisplay();

    void tickLightTimer();
    void incLedTimerSec();
    void decLightTimerSec();
    void resetLightTimerSec();
    void setDefaultLightTimeout(int);
    int getLightTimerSecLeft(){return lightTimerSecLeft;};

    void tickDisplayTimerSec();
    void resetDisplayTimerSec();

    void refreshState();

    bool isDisplayOn(){return displayOn;};
    bool isLightOn(){return lightOn;};

    bool checkAction();

    void setTemperature(float);
};

#endif