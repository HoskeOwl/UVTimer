#include "state_machine.h"


StateMachine::StateMachine(DisplayActions *displayActions, int displayActiveSec, int lightActiveSec, short relPin, short piezopin) : relayPin(relPin), piezoPin(piezopin)
 {
    pinMode(relayPin, OUTPUT);
    PORTD |= 0b1<<relayPin; // relay data pin inverted. So we turn it off
    pinMode(piezoPin, OUTPUT);
    dispActions = displayActions;
    displayTimerSecLeftDefault = displayActiveSec;
    lightTimerSecLeftDefault = lightActiveSec;
    displayTimerSecLeft = displayActiveSec;
    lightTimerSecLeft = lightActiveSec;
    needStratTimer = needStopTimer = needPauseTimer = timerOn = lightOn = false;
    needTurnOnDisplay = needTurnOffDisplay = false;
    lightTimeChanged = true;
    displayOn = true;
    temperature = prevTemperature = -9999;
    tempChanged = false;
};


StateMachine::~StateMachine(){}


void StateMachine::setDefaultLightTimeout(int defaultValue){
    if (!timerOn && lightTimerSecLeft == lightTimerSecLeftDefault){
        lightTimerSecLeft = defaultValue;
        lightTimeChanged = true;
    }
    lightTimerSecLeftDefault = defaultValue;
}


bool StateMachine::checkAction(){
    if (!displayOn){
        turnOnDisplay();
        return false;
    }else
        resetDisplayTimerSec();
    return true;
}


void StateMachine::startTimer(){
    needStopTimer = needPauseTimer = false;
    needStratTimer = true;
}


void StateMachine::stopTimer(){
    needStopTimer = true;
    needPauseTimer = needStratTimer = false;
}


void StateMachine::toggleTimer(bool isStop){
    if (!isLightOn())
        startTimer();
    else if (isLightOn() && isStop){
        stopTimer();
    }else if (isLightOn()){
        pauseTimer();
    }
}

void StateMachine::pauseTimer(){
    needPauseTimer = true;
    needStopTimer = needStratTimer = false;
}


void StateMachine::turnOnDisplay(){
    needTurnOffDisplay = false;
    needTurnOnDisplay = true;
    resetDisplayTimerSec();
}


void StateMachine::turnOffDisplay(){
    needTurnOffDisplay = true;
    needTurnOnDisplay = false;
}


void StateMachine::tickLightTimer(){
    if (timerOn && lightTimerSecLeft > 0){
        lightTimerSecLeft -= 1;
        lightTimeChanged = true;
    }
};


void StateMachine::incLedTimerSec(){
    if (lightTimerSecLeft < MAX_SEC){
        lightTimerSecLeft += TIME_DIFF_SEC;
        if (lightTimerSecLeft > MAX_SEC) lightTimerSecLeft = MAX_SEC;
        lightTimeChanged = true;
    }else{
        lightTimerSecLeft = TIME_DIFF_SEC;
    }
};


void StateMachine::decLightTimerSec(){
    if (lightTimerSecLeft > 0){
        lightTimerSecLeft -= TIME_DIFF_SEC;
        lightTimeChanged = true;
    }else{
        lightTimerSecLeft = lightTimerSecLeftDefault;
    }
};


void StateMachine::resetLightTimerSec(){
    lightTimerSecLeft = lightTimerSecLeftDefault;
    lightTimeChanged = true;
};


void StateMachine::tickDisplayTimerSec(){
    if (displayOn && displayTimerSecLeft > 0)
        displayTimerSecLeft -= 1;
    else if (displayOn){
        turnOffDisplay();
    }
};


void StateMachine::resetDisplayTimerSec(){
    displayTimerSecLeft = displayTimerSecLeftDefault;
    if (!displayOn && !needTurnOnDisplay) turnOnDisplay();
};


void StateMachine::setTemperature(float temp){
    temp = round(temp * 10) / 10;
    if (temp != prevTemperature){
        tempChanged = true;
        prevTemperature = temperature;
        temperature = temp;
    }
}


void StateMachine::refreshState(){
    if (needTurnOffDisplay){
        #ifdef DEBUG_SERIAL
        Serial.println("needTurnOffDisplay");
        #endif
        needTurnOffDisplay = false;
        displayOn = false;
        dispActions->turnOff();
        dispActions->redraw(true);
    }else if (needTurnOnDisplay){
        #ifdef DEBUG_SERIAL
        Serial.println("needTurnOnDisplay");
        #endif
        needTurnOnDisplay = false;
        displayOn = true;
        dispActions->redrawLitingTime(lightTimerSecLeft);
        dispActions->redrawTemperature(temperature);
        dispActions->drawLightStatus(lightOn);
        if (lightOn) dispActions->nextProgress();
    }

    if (needStopTimer || needPauseTimer){
        #ifdef DEBUG_SERIAL
        Serial.println("need Stop/Pause timer");
        #endif
        timerOn = false;
        if (needStopTimer) resetLightTimerSec();
        if (lightOn){
            PORTD |= (0b1<<relayPin); // relay data pin inverted. so turn it on.
            lightOn = false;
            dispActions->drawLightStatus(lightOn);
        }
        needStopTimer = false;
        needPauseTimer = false;
        if (displayOn) dispActions->disableProgress();
        beep(300, 200);
    }else if (needStratTimer){
        #ifdef DEBUG_SERIAL
        Serial.println("needStartTimer");
        #endif
        timerOn = true;
        needStratTimer = false;
        if (!lightOn)
            PORTD &= ~(0b1<<relayPin); // relay data pin inverted. So we turn it off
        lightOn = true;
        dispActions->drawLightStatus(lightOn);
        beep(500, 200);
    }

    if (lightTimeChanged){
        #ifdef DEBUG_SERIAL
        Serial.println("lightTimeChanged");
        #endif
        if (lightTimerSecLeft <= 0){
            timerOn = false;
            needStopTimer = false;
            needPauseTimer = false;
            needStratTimer = false;
            if (lightOn){
                PORTD |= (0b1<<relayPin); // relay data pin inverted. so turn it on.
                lightOn = false;
                dispActions->drawLightStatus(lightOn);
            }
            lightTimerSecLeft = lightTimerSecLeftDefault;
            beep(300, 200);
        }
        dispActions->redrawLitingTime(lightTimerSecLeft);
        lightTimeChanged = false;
    }

    if (tempChanged){
        #ifdef DEBUG_SERIAL
        Serial.println("tempChanged");
        #endif
        tempChanged = false;
        #ifdef DEBUG_SERIAL
        Serial.println(temperature);
        #endif
        dispActions->redrawTemperature(temperature);
    }
    if (displayOn) dispActions->redraw();
};