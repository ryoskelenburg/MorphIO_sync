#include "ofApp.h"
void ofApp::setup(){
    ofBackground(20, 20, 20);
    ofSetVerticalSync(true);
    ofSetFrameRate(FRAMERATE_NUM);
    font.load("font/franklinGothic.otf", 10);
    smallFont.load("font/franklinGothic.otf", 10);
    initArduino();
    gui.setup();
    
    /*-----------input init-------*/
    //1
    for (int i = 0; i< ANALOG_NUM; i++){
        analogPinNum[i] = {i}; //A0~2
        supplyValve[i] = {2 * (i+7) + 1}; //15,17,19
        vacuumValve[i] = {2 * (i+7)}; //14,16,18
        
        gui.add(operateMax[i].setup("minValue: A" + ofToString(analogPinNum[i]),MIDDLE_VALUE - THRESHOLD_VALUE, 0, 1023));
        gui.add(operateMin[i].setup("MaxValue: A" +   ofToString(analogPinNum[i]),MIDDLE_VALUE + THRESHOLD_VALUE, 0, 1023));
        
        setupHistoryPlot(i);
    }
    
    //2~
    for (int i = analogNumStart[1]; i < TOTAL_ANALOG_NUM; i++) {
        analogPinNum[i] = {i};
        gui.add(operateMax[i].setup("minValue: A" + ofToString(analogPinNum[i]),MIDDLE_VALUE - THRESHOLD_VALUE, 0, 1023));
        gui.add(operateMin[i].setup("MaxValue: A" +   ofToString(analogPinNum[i]),MIDDLE_VALUE + THRESHOLD_VALUE, 0, 1023));
        
        setupHistoryPlot(i);
    }
    
    
    /*---------output init----*/
    for (int i = valveNumStart; i < outputGPIO + valveNumStart; i++) {
        ard.sendDigital(i, ARD_LOW);
    }
    
    ard.sendPwm(3, 0);
    ard.sendPwm(5, 0);
    ard.sendPwm(6, 0);
    ard.sendPwm(9, 0);
    ard.sendPwm(10, 0);
    ard.sendPwm(11, 0);
}

void ofApp::update(){
    currentFrameRate = ofGetFrameRate();
    updateArduino();
    
    for (int i = 0; i < TOTAL_ANALOG_NUM; i++) {
        adjustAnalog(analogPinNum[i], i);
        
        plot[i]->update(propVol[i]);
        recordPlot[i]->update(recordPropVol[i][count]);
    }
    
    milliSeconds = ofGetElapsedTimeMillis();
    
    //*-select playback or sync-*//
    /*---playback---*/
    //workRecord();
    //workPlay();
    
    /*---sync---*/
    workRealtime();
    
    /*---check:動作テスト---*/
    checkOutput(1);
    
    count++;
}

//------------------------------------------------------------

void ofApp::adjustAnalog(int _pin, int _order){
    filteredValue[_order][1] = a * filteredValue[_order][0] + (1-a) * ard.getAnalog(_order);
    
    propVol[_order] = ofMap(filteredValue[_order][1], minValue[_order], maxValue[_order], DEFORM_RESOLUSION, 0);
    
    if(filteredValue[_order][1] > maxValue[_order]){
        maxValue[_order] = filteredValue[_order][1];
    }
    if(filteredValue[_order][1] < minValue[_order]){
        minValue[_order] = filteredValue[_order][1];
    }
    
    return propVol[_order], maxValue[_order], minValue[_order], filteredValue[_order][1];
}

void ofApp::updateVal(int _order){
    filteredValue[_order][0] = filteredValue[_order][1];
}

void ofApp::countClear(){
    count = 0;
    playCount = 0;
    startTime = 0;
}

void ofApp::workRecord(){
    //command "s"
    if (bRecord == true) {
        if(count == RECORD_NUM){
            bRecord = false;
            countClear();
        } else {
            //countがリミットにいくまで配列に格納
            for (int i = 0; i < TOTAL_ANALOG_NUM; i++) {
                recordPropVol[i][count] = propVol[i];
            }
        }
    }
}

void ofApp::workPlay(){
    //command 'r'
    if (bPlay == true) {
        if (playCount == RECORD_NUM) {
            bPlay = false;
            countClear();
            for (int i = 0; i < ANALOG_NUM; i++) {
                sendDigitalArduinoExhaust(i);
            }
        } else {
            for (int i = 0; i < ANALOG_NUM; i++) {
                //activeFeedforward(i);
            }
        }
        playCount++;
    }
}

void ofApp::workRealtime(){
    //command 'p'
    if (bReal == true) {
        //input
        for (int i = 3; i < TOTAL_ANALOG_NUM; i++) {
            //ffJudge(i);
            fbJudge(i, i-3);
            fbOutput(i-3, i);
        }
        //std::cout << "PWM[0]: " + ofToString(PWM[0]) << endl;
        //only one unit
//        fbJudge(4, 1);
//        fbOutput(1, 4);
        
    } else {
        for (int i = 0; i < ANALOG_NUM; i++) {
            sendDigitalArduinoExhaust(i);
        }
    }
}

/*--------feedforward------------*/
void ofApp::ffJudge(int number){
    //    delta[number] = recordPropVol[number][count] - recordPropVol[number][count-1];
    delta[number][0] = propVol[number] - oldPropVol[number];
    absDelta[number] = abs(delta[number][0]);
    
    //変化の判定
    if(absDelta[number] >= 1) {
        bDeform[number] = true;
        //bActive[number] = true;
        
    } else {
        bDeform[number] = false;
        //bActive[number] = false;
    }
    
    //正負の判定
    if (delta[number][0] > 0) { bPolarity[number] = true;}
    else if (delta[number][0] < 0){ bPolarity[number] = false;}
    
}

void ofApp::ffOutput(int number, int input){
    //出力
    if (bDeform[input] == true) {
        if (bPolarity[input] == true) {
            sendDigitalArduinoSupply(number,PWM[input]);
            std::cout << ofToString(number) + ": supply" << endl;
        } else {
            sendDigitalArduinoVacuum(number,PWM[input]);
            std::cout << ofToString(number) + ": vacuum" << endl;
        }
    } else {
        sendDigitalArduinoClose(number);
        std::cout << ofToString(number) + ": close" << endl;
    }
    
}

/*--------feedback------------*/

void ofApp::fbJudge(int teach, int child){ //目標値，センサー値
    
    delta[teach][0] = delta[teach][1]; //0が過去
    delta[teach][1] = propVol[teach] - propVol[child]; //偏差の取得
    absDelta[teach] = abs(delta[teach][1]); //偏差の絶対値
    integral += (delta[teach][1] + delta[teach][0]) / 2.0 * dt;
    
    int deltadelta = delta[teach][1] - delta[teach][0];
    
    p = KP * delta[teach][1]; //定数*偏差(0~100) 255 =  gain * 100
    i = KI * integral;
    d = KD * ((delta[teach][1] - delta[teach][0]) / dt);
    
    setPWM_PID(p, 0, 0, child);
    std::cout << "PWM[" + ofToString(child) + "]: " + ofToString(PWM[child]) << endl;
//    std::cout << "D[" + ofToString(child) + "]: " + ofToString(deltadelta) << endl;
    
    if(absDelta[teach] >= 1) {
        bDeform[teach] = true;
    } else {
        bDeform[teach] = false;
    }
    
    if (delta[teach][1] > 0) { bPolarity[teach] = true;}
    else if (delta[teach][1] < 0){ bPolarity[teach] = false;}
}

float ofApp::setPWM_PID(int p, int i, int d, int number){
    PWM[number] = abs(p + i + d);
    return PWM[number];
}

void ofApp::fbOutput(int child, int teach){
    if (bDeform[teach] == true) {
        if (bPolarity[teach] == true) {
            sendDigitalArduinoSupply(child, PWM[child]);
            std::cout << ofToString(child) + ": supply" << endl;
        } else {
            sendDigitalArduinoVacuum(child, PWM[child]);
            std::cout << ofToString(child) + ": vacuum" << endl;
        }
    } else {
        sendDigitalArduinoClose(child);
        std::cout << ofToString(child) + ": close" << endl;
    }
    
}

void ofApp::stopActuate(int number){
    if(ofGetElapsedTimeMillis() - startTime < 10) {
        bDeform[number] = true;
    } else {
        bDeform[number] = false;
    }
}

/*----------------arduino control------------*/

void ofApp::checkOutput(int x) {
    if (bCheck == true) {
        if (x == 1) {
            sendDigitalArduinoVacuum(0,0);
        }
    } else {
        sendDigitalArduinoExhaust(0);
    }
}

void ofApp::sendDigitalArduinoSupply(int number, int PWM){
    ard.sendDigital(supplyValve[number], ARD_HIGH);
    ard.sendDigital(vacuumValve[number], ARD_LOW);
    ard.sendPwm(supplyPump[number], PWM);
    ard.sendPwm(vacuumPump[number], 0);
}

void ofApp::sendDigitalArduinoVacuum(int number, int PWM){
    ard.sendDigital(supplyValve[number], ARD_LOW);
    ard.sendDigital(vacuumValve[number], ARD_HIGH);
    ard.sendPwm(supplyPump[number], 0);
    ard.sendPwm(vacuumPump[number], PWM);
}

void ofApp::sendDigitalArduinoClose(int number){
    ard.sendDigital(supplyValve[number], ARD_HIGH);
    ard.sendDigital(vacuumValve[number], ARD_LOW);
    ard.sendPwm(supplyPump[number], 0);
    ard.sendPwm(vacuumPump[number], 0);
}

void ofApp::sendDigitalArduinoExhaust(int number){
    ard.sendDigital(supplyValve[number], ARD_LOW);
    ard.sendDigital(vacuumValve[number], ARD_LOW);
    ard.sendPwm(supplyPump[number], 0);
    ard.sendPwm(vacuumPump[number], 0);
}

void ofApp::checkDigital(int number){
    ard.sendDigital(number, ARD_HIGH);
}

void ofApp::checkPWM(int number, int PWM) {
    ard.sendPwm(number, PWM);
}

void ofApp::clearDigital(){
    for (int i = 0; i < 20; i++) {
        ard.sendDigital(i, ARD_LOW);
    }
}
//*****--------Draw--------------------****//

void ofApp::draw(){
    gui.draw();
    drawLog();
    
    /*------------plot-----------*/
    //draw
    for (int i = 0; i < ANALOG_NUM; i++) {
        plot[i]->draw(0, 100 * (i+1) + 150, ofGetWidth(), 100);
        updateVal(i);
    }
    for (int i = 3; i < TOTAL_ANALOG_NUM; i++) {
        plot[i]->draw(0, 100 * (i-2) + 150, ofGetWidth(), 100);
        updateVal(i);
    }
    
    //    recordMode
    if (bDrawPlot == true) {
        if (playCount >= RECORD_NUM) {
            bDrawPlot = false;
        } else {
            for (int i = 0; i < TOTAL_ANALOG_NUM; i++){
                recordPlot[i]->draw(0,150 * (i+1) + 100, ofGetWidth(), 150);
            }
        }
    }
    
    for (int i = 0; i < TOTAL_ANALOG_NUM; i++) {
        oldPropVol[i] = propVol[i];
    }
}

void ofApp::drawLog(){
    ofSetColor(255);
    
    if (!bSetupArduino){
        font.drawString("Connect ready...\n", valueRow[1], 30);
    } else {
        if (bRecord == true) {
            font.drawString("Recording...\n", valueRow[1], 30);
        } else if (bPlay == true) {
            font.drawString("Playing...\n", valueRow[1], 30);
        } else {
            font.drawString("Connect succeed!\n", valueRow[1], 30);
        }
    }
    
    //    smallFont.drawString("millis: " + ofToString(milliSeconds), valueRow[1], 50);
    //    smallFont.drawString("startTime: " + ofToString(startTime), valueRow[1], 70);
    smallFont.drawString("pwm: " + ofToString(pwm) + " bRealTime" + ofToString(bReal) , valueRow[1], 50);
    //smallFont.drawString("framerate: " + ofToString(FRAMERATE_NUM), valueRow[1], 70);
    smallFont.drawString("Unit:1 " + ofToString(bActive[0]), valueRow[1], 90);
    smallFont.drawString("Unit:2 " + ofToString(bActive[1]), valueRow[1], 110);
    smallFont.drawString("Unit:3 " + ofToString(bActive[2]), valueRow[1], 130);
    
    ofSetColor(255);
    for (int i = 0; i < ANALOG_NUM; i++) {
        drawLogContents(i, 0, i);
        //drawArrayData(i);
    }
    
    for (int i = 3; i < TOTAL_ANALOG_NUM; i++) {
        drawLogContents(i, 1, i-3);
    }
}

void ofApp::drawLogContents(int _number, int _row, int _order){
    
    font.drawString("Propotion : " + ofToString(propVol[_number]), row[_row][_order], 700);
    
    smallFont.drawString("value  :  " + ofToString(filteredValue[_number][1]), row[_row][_order], 715);
    
    smallFont.drawString("minValue  :  " + ofToString(minValue[_number]), row[_row][_order], 730);
    smallFont.drawString("maxValue     :  " + ofToString(maxValue[_number]), row[_row][_order], 745);
//    smallFont.drawString("delta : " + ofToString(recordPropVol[_number][count] - recordPropVol[_number][count-1]), row[_row][_order], 760);
    smallFont.drawString("delta : " + ofToString(propVol[_number] - oldPropVol[_number]), row[_row][_order], 760);
}

void ofApp::drawArrayData(int _number) {
    smallFont.drawString("memory[0] :  " + ofToString(recordPropVol[_number][0]), valueRow[1], valueCol[_number] + 30);
    smallFont.drawString("memory[20] :  " + ofToString(recordPropVol[_number][20]), valueRow[1], valueCol[_number] + 50);
    smallFont.drawString("memory[100] :  " + ofToString(recordPropVol[_number][99]), valueRow[1], valueCol[_number] + 70);
}

/*-----Default oF------*/

void ofApp::keyPressed(int key){
    switch (key) {
        case 'f':
            ofToggleFullscreen();
            break;
        case 'c':
            for (int i = 0; i < TOTAL_ANALOG_NUM; i++) {
                minValue[i] = operateMin[i];
                maxValue[i] = operateMax[i];
            }
            break;
        case 'v':
            milliSeconds = 0;
            break;
        case 's':
            bRecord = true;
            countClear();
            break;
        case 'r':
            bPlay = true;
            countClear();
            for (int i = 0; i < TOTAL_ANALOG_NUM; i++) {
                plot[i]->reset();
                recordPlot[i]->reset();
            }
            bDrawPlot = true;
            break;
        case 'p':
            bReal = true;
            break;
        case 'o':
            bReal = false;
            break;
        case 'w':
            //bRecordWrite = true;
            break;
        case 't':
            bCheck = true;
            break;
        case  'y':
            bCheck = false;
            break;
        case '1':
            checkPWM(11, 100);
            break;
        case '2':
            checkPWM(11, 0);
            break;
        case '3':
            checkDigital(4);
            break;
        case '4':
            checkDigital(5);
            break;
        case '5':
            checkDigital(6);
            break;
        case '6':
            checkDigital(7);
            break;
        case '0':
            clearDigital();
            break;
        default:
            break;
    }
}

void ofApp::keyReleased(int key){
    switch (key) {
        default:
            break;
    }
}
void ofApp::mouseMoved(int x, int y ){
}
void ofApp::mouseDragged(int x, int y, int button){
}
void ofApp::mousePressed(int x, int y, int button){
}
void ofApp::mouseReleased(int x, int y, int button){
}
void ofApp::mouseEntered(int x, int y){
}
void ofApp::mouseExited(int x, int y){
}
void ofApp::windowResized(int w, int h){
}
void ofApp::gotMessage(ofMessage msg){
}
void ofApp::dragEvent(ofDragInfo dragInfo){
}

void ofApp::initArduino(){
    ard.connect("/dev/cu.usbmodem1411", 57600);
    
    ofAddListener(ard.EInitialized, this, &ofApp::setupArduino);
    bSetupArduino    = false;
}

void ofApp::setupArduino(const int & version) {
    ofRemoveListener(ard.EInitialized, this, &ofApp::setupArduino);
    bSetupArduino = true;
    ofLogNotice() << ard.getFirmwareName();
    ofLogNotice() << "firmata v" << ard.getMajorFirmwareVersion() << "." << ard.getMinorFirmwareVersion();
    
    //A0~2
    for (int i = analogNumStart[0]; i < TOTAL_ANALOG_NUM; i++) {
        ard.sendAnalogPinReporting(i, ARD_ANALOG);
    }
    
    //14~19 for the valve
    for (int i = valveNumStart; i < valveNumStart + outputGPIO; i++) {
        ard.sendDigitalPinMode(i, ARD_OUTPUT);
    }
    
    //3,5,6,9,10,11 for the pump
    //    for (int i = pumpNumStart; i < pumpNumStart + outputGPIO; i++) {
    //        ard.sendDigitalPinMode(i, ARD_OUTPUT);
    //    }
    ard.sendDigitalPinMode(3, ARD_PWM);
    ard.sendDigitalPinMode(5, ARD_PWM);
    ard.sendDigitalPinMode(6, ARD_PWM);
    ard.sendDigitalPinMode(9, ARD_PWM);
    ard.sendDigitalPinMode(10, ARD_PWM);
    ard.sendDigitalPinMode(11, ARD_PWM);
    
    ofAddListener(ard.EDigitalPinChanged, this, &ofApp::digitalPinChanged);
    ofAddListener(ard.EAnalogPinChanged, this, &ofApp::analogPinChanged);
}

void ofApp::updateArduino(){
    ard.update();
    //    if (bSetupArduino) {
    //        ard.sendPwm(11, (int)(128 + 128 * sin(ofGetElapsedTimef())));   // pwm...
    //    }
}

void ofApp::digitalPinChanged(const int & pinNum) {
    // do something with the digital input. here we're simply going to print the pin number and
    // value to the screen each time it changes
    buttonState = "digital pin: " + ofToString(pinNum) + " = " + ofToString(ard.getDigital(pinNum));
}

void ofApp::analogPinChanged(const int & pinNum) {
    // do something with the analog input. here we're simply going to print the pin number and
    // value to the screen each time it changes
    potValue = "analog pin: " + ofToString(pinNum) + " = " + ofToString(ard.getAnalog(pinNum));
}

void ofApp::setupHistoryPlot(int number){
    plot[number] = new ofxHistoryPlot(&currentFrameRate, "number:00" + ofToString(number), ofGetWidth(), false);
    plot[number]->setBackgroundColor(ofColor(0,0,0,0));
    plot[number]->setColor(ofColor(ofRandom( 100, 255 ),ofRandom( 100, 255 ),ofRandom( 100, 255 )));
    plot[number]->setRange(-10, DEFORM_RESOLUSION + 10);
    plot[number]->setRespectBorders(false);
    plot[number]->setLineWidth(2);
    plot[number]->setCropToRect(false);
    plot[number]->setShowSmoothedCurve(false);
    plot[number]->setSmoothFilter(0.1);
    
    recordPlot[number] = new ofxHistoryPlot(&currentFrameRate, "number:00" + ofToString(number), ofGetWidth()/2, false);
    recordPlot[number]->setBackgroundColor(ofColor(0,0,0,0));
    recordPlot[number]->setColor(ofColor(ofRandom( 100, 255 ),ofRandom( 100, 255 ),ofRandom( 100, 255 )));
    recordPlot[number]->setRange(-DEFORM_RESOLUSION - 10, DEFORM_RESOLUSION + 10);
    recordPlot[number]->setRespectBorders(false);
    recordPlot[number]->setLineWidth(2);
    recordPlot[number]->setCropToRect(false);
    recordPlot[number]->setShowSmoothedCurve(false);
    recordPlot[number]->setSmoothFilter(0.1);
}

double ofApp::ceil2(double dIn, int nLen){
    double dOut;
    dOut = dIn * pow(10.0, nLen);
    dOut = (double)(int)(dOut + 0.9);
    return dOut * pow(10.0, -nLen);
}

void ofApp::useImportData(){
    /*------import file--------
     myReadFile.open("text.txt",ofFile::ReadOnly);
     cout << myReadFile.readToBuffer().getText();
     auto input = ofSplitString(myReadFile.readToBuffer().getText(), "\n");
     for(int i= 0; i < RECORD_NUM;i++)
     {
     recordPropVol[i] = stoi(input[i]);
     }
     */
    /*-------export file------------------
     recordFile.open(ofGetTimestampString() + "recordData.txt",ofFile::WriteOnly);
     feedbackFile.open(ofGetTimestampString() + "feedBackData.txt",ofFile::WriteOnly);
     */
}

void ofApp::captureScreen(){
    //feedbackFile << propotionVolume[0]  << endl;
    //screen.grabScreen(0, 0 , ofGetWidth(), ofGetHeight());
    //screen.save(ofGetTimestampString() + "screenshot.png");
}



void ofApp::checkWrite(){
    //
    //    if (bRecordWrite == true) {
    //
    //        myTextFile.open("text.txt",ofFile::WriteOnly);
    //        for(int i = 0;i < RECORD_NUM;i++)
    //        {
    //            myTextFile << recordPropVol[i] << endl;
    //        }
    //        bRecordWrite = false;
    //    }
}

/*
 RFP32B03B: 3.8L/min = 63ml/sec
 elastV1.0: 90ml, resolution:8, 11.25ml
 resolution : 0.179sec = 179milliseconds
 the elapsed time in milliseconds (1000 milliseconds = 1 second).
 */
