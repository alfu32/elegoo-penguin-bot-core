#include "IRremote.h"
#include "Oscillator.h"
#include <Servo.h>
#include "NeoSWSerial.h"

#define BTN_UP 16736925
#define BTN_DOWN 16754775
#define BTN_LEFT 16761405
#define BTN_RIGHT 16720605
#define BTN_DANCE 16718055
#define BTN_MUSIC 16724175
#define BTN_MODE 16734885
#define BTN_ADD 16716015
#define BTN_SUB 16726215
#define BTN_IDLE 16712445
#define BTN_KEEP 4294967295
#define BTN_VOL 16743045
/*
         ---------------
        |     O   O     |
        |---------------|
YR 3==> |               | <== YL 2
         ---------------
            ||     ||
            ||     ||
RR 5==>   -----   ------  <== RL 4
         |-----   ------|
*/

#if 1 // red
#define TRIM_YL 0
#define TRIM_YR 0
#define TRIM_RL 0
#define TRIM_RR 0
#elif 0
#define TRIM_YL 0
#define TRIM_YR 0
#define TRIM_RL 0
#define TRIM_RR 0
#endif

#define SOFTWARE_RXD A2
#define SOFTWARE_TXD A3
#define YL_PIN 10 // 3
#define YR_PIN 9  // 2
#define RL_PIN 12 // 1
#define RR_PIN 6  // 0
#define RECV_PIN 3
#define ECHO_PIN 4
#define TRIG_PIN 5
#define ST188_R_PIN A1
#define ST188_L_PIN A0
#define VOLTAGE_MEASURE_PIN A4
#define VREF 1.1
#define RES1 10000
#define RES2 2000
#define INDICATOR_LED_PIN A5
#define MY1690_PIN 8
#define HT6871_PIN 7
#define N_SERVOS 4
#define INTERVALTIME 10.0
#define CENTRE 90
#define AMPLITUDE 30
#define ULTRA_HIGH_RATE 0.3
#define HIGH_RATE 0.5
#define MID_RATE 0.7
#define LOW_RATE 1.0
#define ULTRA_LOW_RATE 1.5
NeoSWSerial mp3Serial(SOFTWARE_RXD, SOFTWARE_TXD);
unsigned long moveTime;
unsigned long ledBlinkTime;
unsigned long voltageMeasureTime;
unsigned long infraredMeasureTime;
unsigned long irValue;
boolean irValueFlag = false;
int LED_value = 255;
boolean LED_flag = true;
char danceNum = 0;
int distance;
int st188Val_L;
int st188Val_R;
long int ST188Threshold;
long int ST188RightDataMin;
long int ST188LeftDataMin;
int UltraThresholdMin = 7;
int UltraThresholdMax = 20;
Oscillator servo[N_SERVOS];
IRrecv irrecv(RECV_PIN);
decode_results results;
enum MODE {
    IDLE,
    IRREMOTE,
    OBSTACLE,
    FOLLOW,
    MUSIC,
    DANCE,
    VOLUME
} mode = IDLE;
enum IRMODE {
    FORWARD,
    BACKWAED,
    TURNRIGHT,
    TURNLIFT,
    STOP,
} IRmode = STOP;
int musicIndex = 2;
int danceIndex = 2;
bool danceFlag = false;
unsigned long preIrValue;
unsigned long preIrMillis;
unsigned long preMp3Millis;
int t = 495;
double pause = 0;
class MY1690_16S {
public:
    int volume;
    String playStatus[5] = {"0", "1", "2", "3", "4"}; // STOP PLAYING PAUSE FF FR
    void playSong(unsigned char num, unsigned char vol) {
        setVolume(vol);
        setPlayMode(4);
        CMD_SongSelet[4] = num;
        checkCode(CMD_SongSelet);
        mp3Serial.write(CMD_SongSelet, 7);
        delay(10);
    };
    String getPlayStatus() {
        mp3Serial.write(CMD_getPlayStatus, 5);
        delay(10);
        return getStatus();
    }
    String getStatus() {
        String statusMp3 = "";
        while (mp3Serial.available()) {
            statusMp3 += (char)mp3Serial.read();
        }
        return statusMp3;
    };
    void stopPlay() {
        setPlayMode(4);
        mp3Serial.write(CMD_MusicStop, 5);
        delay(10);
    };
    void setVolume(unsigned char vol) {
        CMD_VolumeSet[3] = vol;
        checkCode(CMD_VolumeSet);
        mp3Serial.write(CMD_VolumeSet, 6);
        delay(10);
    };
    void volumePlus() {
        mp3Serial.write(CMD_VolumePlus, 5);
        delay(10);
    };
    void volumeDown() {
        mp3Serial.write(CMD_VolumeDown, 5);
        delay(10);
    };
    void setPlayMode(unsigned char mode) {
        CMD_PlayMode[3] = mode;
        checkCode(CMD_PlayMode);
        mp3Serial.write(CMD_PlayMode, 6);
        delay(10);
    };
    void checkCode(unsigned char *vs) {
        int val = vs[1];
        int i;
        for (i = 2; i < vs[1]; i++) {
            val = val ^ vs[i];
        }
        vs[i] = val;
    };
    void ampMode(int p, bool m) {
        pinMode(p, OUTPUT);
        if (m) {
            digitalWrite(p, HIGH);
        } else {
            digitalWrite(p, LOW);
        }
    };
    void init() {
        ampMode(HT6871_PIN, HIGH);
        stopPlay();
        volume = 15;
    }
private:
    byte CMD_MusicPlay[5] = {0x7E, 0x03, 0x11, 0x12, 0xEF};
    byte CMD_MusicStop[5] = {0x7E, 0x03, 0x1E, 0x1D, 0xEF};
    byte CMD_MusicNext[5] = {0x7E, 0x03, 0x13, 0x10, 0xEF};
    byte CMD_MusicPrev[5] = {0x7E, 0x03, 0x14, 0x17, 0xEF};
    byte CMD_VolumePlus[5] = {0x7E, 0x03, 0x15, 0x16, 0xEF};
    byte CMD_VolumeDown[5] = {0x7E, 0x03, 0x16, 0x15, 0xEF};
    byte CMD_VolumeSet[6] = {0x7E, 0x04, 0x31, 0x00, 0x00, 0xEF};
    byte CMD_PlayMode[6] = {0x7E, 0x04, 0x33, 0x00, 0x00, 0xEF};
    byte CMD_SongSelet[7] = {0x7E, 0x05, 0x41, 0x00, 0x00, 0x00, 0xEF};
    byte CMD_getPlayStatus[5] = {0x7E, 0x03, 0x20, 0x23, 0xEF};
} MP3;
void oscillate(int A[N_SERVOS], int O[N_SERVOS], int T, double phase_diff[N_SERVOS]) {
    for (int i = 0; i < 4; i++) {
        servo[i].SetO(O[i]);
        servo[i].SetA(A[i]);
        servo[i].SetT(T);
        servo[i].SetPh(phase_diff[i]);
    }
    double ref = millis();
    for (double x = ref; x < T + ref; x = millis()) {
        for (int i = 0; i < 4; i++) {
            servo[i].refresh();
        }
    }
}
unsigned long final_time;
unsigned long interval_time;
int oneTime;
int iteration;
float increment[N_SERVOS];
int oldPosition[] = {CENTRE, CENTRE, CENTRE, CENTRE};
void home() {
    int move1[] = {90, 90, 90, 90};
    moveNServos(t, move1);
    delay(t);
}
void moveNServos(int time, int newPosition[]) {
    for (int i = 0; i < N_SERVOS; i++) {
        increment[i] = ((newPosition[i]) - oldPosition[i]) / (time / INTERVALTIME);
    }
    final_time = millis() + time;
    iteration = 1;
    while (millis() < final_time) {
        interval_time = millis() + INTERVALTIME;
        oneTime = 0;
        while (millis() < interval_time) {
            if (oneTime < 1) {
                for (int i = 0; i < N_SERVOS; i++) {
                    servo[i].SetPosition(oldPosition[i] + (iteration * increment[i]));
                }
                iteration++;
                oneTime++;
            }
        }
    }

    for (int i = 0; i < N_SERVOS; i++) {
        oldPosition[i] = newPosition[i];
    }
}
void walk(int steps, int T, int dir) {
    int A[4] = {30, 30, 30, 30};
    int O[4] = {0, 0, 0, 0};
    double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(dir * 90),
                            DEG2RAD(dir * 90)
                           };
    for (int i = 0; i < steps; i++)
        oscillate(A, O, T, phase_diff);
}
void turn(int steps, int T, int dir) {
    int A[4] = {30, 30, 0, 0};
    if (dir == 1) {
        A[2] = 30;
        A[3] = 10;
    } else {
        A[2] = 10;
        A[3] = 30;
    }
    int O[4] = {0, 0, 0, 0};
    double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(90), DEG2RAD(90)};
    for (int i = 0; i < steps; i++)
        oscillate(A, O, T, phase_diff);
}
void moonWalkRight(int steps, int T) {
    int A[4] = {25, 25, 0, 0};
    int O[4] = { -15, 15, 0, 0};
    double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180 + 120), DEG2RAD(90), DEG2RAD(90)};

    for (int i = 0; i < steps; i++)
        oscillate(A, O, T, phase_diff);
}
void moonWalkLeft(int steps, int T) {
    int A[4] = {25, 25, 0, 0};
    int O[4] = { -15, 15, 0, 0};
    double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180 - 120), DEG2RAD(90), DEG2RAD(90)};

    for (int i = 0; i < steps; i++)
        oscillate(A, O, T, phase_diff);
}
void crusaito(int steps, int T) {
    int A[4] = {25, 25, 30, 30};
    int O[4] = { -15, 15, 0, 0};
    double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180 + 120), DEG2RAD(90), DEG2RAD(90)};
    for (int i = 0; i < steps; i++)
        oscillate(A, O, T, phase_diff);
    home();
}
void swing(int steps, int T) {
    int A[4] = {25, 25, 0, 0};
    int O[4] = { -15, 15, 0, 0};
    double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(90), DEG2RAD(90)};

    for (int i = 0; i < steps; i++)
        oscillate(A, O, T, phase_diff);
}
void upDown(int steps, int T) {
    int A[4] = {25, 25, 0, 0};
    int O[4] = { -15, 15, 0, 0};
    double phase_diff[4] = {DEG2RAD(180), DEG2RAD(0), DEG2RAD(270), DEG2RAD(270)};
    for (int i = 0; i < steps; i++)
        oscillate(A, O, T, phase_diff);
    home();
}
void flapping(int steps, int T) {
    int A[4] = {15, 15, 8, 8};
    int O[4] = { -A[0], A[1], 0, 0};
    double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180), DEG2RAD(-90), DEG2RAD(90)};

    for (int i = 0; i < steps; i++)
        oscillate(A, O, T, phase_diff);
}
void run(int steps, int T) {
    int A[4] = {10, 10, 10, 10};
    int O[4] = {0, 0, 0, 0};
    double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(90), DEG2RAD(90)};

    for (int i = 0; i < steps; i++)
        oscillate(A, O, T, phase_diff);
}
void backyard(int steps, int T) {
    int A[4] = {15, 15, 30, 30};
    int O[4] = {0, 0, 0, 0};
    double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(-90), DEG2RAD(-90)};

    for (int i = 0; i < steps; i++)
        oscillate(A, O, T, phase_diff);
}
void backyardSlow(int steps, int T) {
    int A[4] = {15, 15, 30, 30};
    int O[4] = {0, 0, 0, 0};
    double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(-90), DEG2RAD(-90)};

    for (int i = 0; i < steps; i++)
        oscillate(A, O, T, phase_diff);
}
void goingUp(int tempo) {
    int move1[] = {50, 130, 90, 90};
    moveNServos(tempo * HIGH_RATE, move1);
    delay(tempo / 2);
    home();
}
void drunk(int tempo) {
    int move1[] = {70, 70, 90, 90};
    int move2[] = {110, 110, 90, 90};
    int move3[] = {70, 70, 90, 90};
    int move4[] = {110, 110, 90, 90};
    moveNServos(tempo * MID_RATE, move1);
    moveNServos(tempo * MID_RATE, move2);
    moveNServos(tempo * MID_RATE, move3);
    moveNServos(tempo * MID_RATE, move4);
    home();
}
void noGravity(int tempo) {
    int move1[] = {120, 140, 90, 90};
    int move2[] = {120, 30, 90, 90};
    int move3[] = {120, 120, 90, 90};
    int move4[] = {120, 30, 120, 120};
    int move5[] = {120, 30, 60, 60};
    moveNServos(tempo * MID_RATE, move1);
    delay(tempo);
    moveNServos(tempo * MID_RATE, move2);
    moveNServos(tempo * MID_RATE, move3);
    moveNServos(tempo * MID_RATE, move2);
    delay(tempo);
    moveNServos(tempo * LOW_RATE, move4);
    delay(tempo);
    moveNServos(tempo * LOW_RATE, move5);
    delay(tempo);
    moveNServos(tempo * LOW_RATE, move4);
    delay(tempo);
    home();
}
void kickLeft(int tempo) {
    int move1[] = {120, 140, 90, 90};
    int move2[] = {120, 90, 90, 90};
    int move3[] = {120, 120, 90, 90};
    int move4[] = {120, 90, 120, 120};
    int move5[] = {120, 120, 60, 60};
    moveNServos(tempo * MID_RATE, move1);
    delay(tempo);
    moveNServos(tempo * MID_RATE, move2);
    delay(tempo / 4);
    moveNServos(tempo * MID_RATE, move3);
    delay(tempo / 4);
    moveNServos(tempo * LOW_RATE, move4);
    delay(tempo / 4);
    moveNServos(tempo * LOW_RATE, move5);
    delay(tempo / 4);
    home();
}
void kickRight(int tempo) {
    int move1[] = {40, 60, 90, 90};
    int move2[] = {90, 60, 90, 90};
    int move3[] = {60, 60, 90, 90};
    int move4[] = {90, 60, 120, 120};
    int move5[] = {60, 60, 60, 60};
    moveNServos(tempo * MID_RATE, move1);
    delay(tempo);
    moveNServos(tempo * MID_RATE, move2);
    delay(tempo / 4);
    moveNServos(tempo * MID_RATE, move3);
    delay(tempo / 4);
    moveNServos(tempo * LOW_RATE, move4);
    delay(tempo / 4);
    moveNServos(tempo * LOW_RATE, move5);
    delay(tempo / 4);
    home();
}
void legRaise(int tempo, int dir) {
    if (dir) {
        int move1[] = {70, 70, 60, 60};
        moveNServos(tempo * MID_RATE, move1);
        delay(tempo);

    } else {
        int move1[] = {110, 110, 120, 120};
        moveNServos(tempo * MID_RATE, move1);
        delay(tempo);
    }
    home();
}
void legRaise1(int tempo, int dir) {
    if (dir) {
        int move1[] = {50, 60, 90, 90};
        int move2[] = {60, 60, 120, 90};
        int move3[] = {60, 60, 60, 90};
        moveNServos(tempo * MID_RATE, move1);
        delay(tempo);
        moveNServos(tempo * LOW_RATE, move2);
        delay(tempo / 4);
        moveNServos(tempo * LOW_RATE, move3);
        delay(tempo / 4);
        moveNServos(tempo * LOW_RATE, move2);
        delay(tempo / 4);
        moveNServos(tempo * LOW_RATE, move3);
        delay(tempo / 4);
    } else {
        int move1[] = {120, 130, 90, 90};
        int move2[] = {120, 120, 90, 60};
        int move3[] = {120, 120, 90, 120};
        moveNServos(tempo, move1);
        delay(tempo);
        moveNServos(tempo * MID_RATE, move2);
        delay(tempo / 4);
        moveNServos(tempo * MID_RATE, move3);
        delay(tempo / 4);
        moveNServos(tempo * MID_RATE, move2);
        delay(tempo / 4);
        moveNServos(tempo * MID_RATE, move3);
        delay(tempo / 4);
    }
    home();
}
void legRaise2(int steps, int tempo, int dir) {
    if (dir) {
        int move1[] = {20, 60, 90, 90};
        int move2[] = {20, 90, 120, 90};
        for (int i = 0; i < steps; i++) {
            moveNServos(tempo * 0.7, move1);
            delay(tempo / 4);
            moveNServos(tempo * 0.7, move2);
            delay(tempo / 4);
        }
    } else {
        int move1[] = {120, 160, 90, 90};
        int move2[] = {90, 160, 90, 60};
        for (int i = 0; i < steps; i++) {
            moveNServos(tempo * 0.7, move1);
            delay(tempo / 4);
            moveNServos(tempo * 0.7, move2);
            delay(tempo / 4);
        }
    }
    home();
}
void legRaise3(int steps, int tempo, int dir) {
    if (dir) {
        int move1[] = {20, 60, 90, 90};
        int move2[] = {20, 90, 90, 90};
        for (int i = 0; i < steps; i++) {
            moveNServos(tempo * 0.5, move1);
            delay(tempo / 4);
            moveNServos(tempo * 0.5, move2);
            delay(tempo / 4);
        }
    } else {
        int move1[] = {120, 160, 90, 90};
        int move2[] = {90, 160, 90, 90};
        for (int i = 0; i < steps; i++) {
            moveNServos(tempo * 0.5, move1);
            delay(tempo / 4);
            moveNServos(tempo * 0.5, move2);
            delay(tempo / 4);
        }
    }
    home();
}
void legRaise4(int tempo, int dir) {
    if (dir) {
        int move1[] = {20, 60, 90, 90};
        int move2[] = {20, 90, 90, 90};

        moveNServos(tempo * MID_RATE, move1);
        delay(tempo / 4);
        moveNServos(tempo * MID_RATE, move2);
        delay(tempo / 4);

    } else {
        int move1[] = {120, 160, 90, 90};
        int move2[] = {90, 160, 90, 90};
        moveNServos(tempo * MID_RATE, move1);
        delay(tempo / 4);
        moveNServos(tempo * MID_RATE, move2);
        delay(tempo / 4);
    }
    home();
}
void sitdown() {
    int move1[] = {150, 90, 90, 90};
    int move2[] = {150, 30, 90, 90};
    moveNServos(t * ULTRA_LOW_RATE, move1);
    delay(t / 2);
    moveNServos(t * ULTRA_LOW_RATE, move2);
    delay(t / 2);
    home();
}
void lateral_fuerte(boolean dir, int tempo) {
    if (dir) {
        int move1[] = {CENTRE - 2 * AMPLITUDE, CENTRE - AMPLITUDE, CENTRE, CENTRE};
        int move2[] = {CENTRE + AMPLITUDE, CENTRE - AMPLITUDE, CENTRE, CENTRE};
        int move3[] = {CENTRE - 2 * AMPLITUDE, CENTRE - AMPLITUDE, CENTRE, CENTRE};
        moveNServos(tempo * LOW_RATE, move1);
        delay(tempo * 2);
        moveNServos(tempo * ULTRA_HIGH_RATE, move2);
        delay(tempo / 2);
        moveNServos(tempo * ULTRA_HIGH_RATE, move3);
        delay(tempo);
    } else {
        int move1[] = {CENTRE + AMPLITUDE, CENTRE + 2 * AMPLITUDE, CENTRE, CENTRE};
        int move2[] = {CENTRE + AMPLITUDE, CENTRE - AMPLITUDE, CENTRE, CENTRE};
        int move3[] = {CENTRE + AMPLITUDE, CENTRE + 2 * AMPLITUDE, CENTRE, CENTRE};
        moveNServos(tempo * LOW_RATE, move1);
        delay(tempo * 2);
        moveNServos(tempo * ULTRA_HIGH_RATE, move2);
        delay(tempo / 2);
        moveNServos(tempo * ULTRA_HIGH_RATE, move3);
        delay(tempo);
    }
    home();
}
void primera_parte() {
    int move2[4] = {90, 90, 90, 90};
    lateral_fuerte(1, t);
    moveNServos(t * 0.5, move2);
    lateral_fuerte(0, t);
    moveNServos(t * 0.5, move2);
    lateral_fuerte(1, t);
    moveNServos(t * 0.5, move2);
    lateral_fuerte(0, t);
    home();
}
void segunda_parte() {
    int move1[4] = {90, 90, 80, 100};
    int move2[4] = {90, 90, 100, 80};
    for (int x = 0; x < 3; x++) {
        for (int i = 0; i < 3; i++) {
            pause = millis();
            moveNServos(t * 0.15, move1);
            moveNServos(t * 0.15, move2);
            while (millis() < (pause + t));
        }
    }
    home();
}
void dance() {
    primera_parte();
    segunda_parte();
    moonWalkLeft(4, t * 2);
    moonWalkRight(4, t * 2);
    moonWalkLeft(4, t * 2);
    moonWalkRight(4, t * 2);
    primera_parte();

    for (int i = 0; i < 16; i++) {
        flapping(1, t / 4);
        delay(3 * t / 4);
    }

    moonWalkRight(4, t * 2);
    moonWalkLeft(4, t * 2);
    moonWalkRight(4, t * 2);
    moonWalkLeft(4, t * 2);

    drunk(t * 4);
    drunk(t * 4);
    drunk(t * 4);
    drunk(t * 4);
    kickLeft(t);
    kickRight(t);
    drunk(t * 8);
    drunk(t * 4);
    drunk(t / 2);
    delay(t * 4);

    drunk(t / 2);

    delay(t * 4);
    walk(2, t * 4, 1);
    home();
    backyard(2, t * 2);
    home();
    goingUp(t * 2);
    goingUp(t * 1);
    noGravity(t);

    delay(t);
    primera_parte();
    for (int i = 0; i < 32; i++) {
        flapping(1, t / 2);
        delay(t / 2);
    }

    for (int i = 0; i < 4; i++)
        servo[i].SetPosition(90);
}
void dance2() {
    lateral_fuerte(1, t);
    lateral_fuerte(0, t);
    drunk(t / 2);
    drunk(t);
    kickLeft(t);
    kickRight(t);
    walk(2, t * 4, 1);
    home();
    backyard(2, t * 4);
    noGravity(t);
    lateral_fuerte(1, t);
    lateral_fuerte(0, t);
    segunda_parte();
    upDown(5, 500);
}
void dance3() {
    sitdown();
    legRaise(t, 1);
    swing(5, t);
    legRaise1(t, 1);
    walk(2, t * 4, 1);
    home();
    noGravity(t);
    kickRight(t);
    goingUp(t);
    kickLeft(t);
    legRaise4(t, 1);
    backyard(2, t * 4);
    drunk(t);
    lateral_fuerte(1, 500);
    lateral_fuerte(0, 500);
    sitdown();
}
void dance4() {
    flapping(1, t);
    drunk(t);
    kickLeft(t);
    walk(2, t * 4, 1);
    home();
    lateral_fuerte(0, t);
    sitdown();
    legRaise(t, 1);
    swing(5, t);
    backyard(2, t * 4);
    goingUp(t);
    noGravity(t);
    upDown(5, t);
    legRaise1(t, 1);
    legRaise2(4, t, 0);
    kickRight(t);
    goingUp(t);
    legRaise3(4, t, 1);
    kickLeft(t);
    legRaise4(t, 1);
    segunda_parte();
    sitdown();
}
void start() {
    MP3.stopPlay();
    MP3.playSong(1, MP3.volume);
    startDance();
    MP3.stopPlay();
    servoAttach();
}
void startDance() {
    servoAttach();
    lateral_fuerte(1, t);
    lateral_fuerte(0, t);
    goingUp(t);
    servoDetach();
}
int getDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    return (int)pulseIn(ECHO_PIN, HIGH) / 58;
}
void obstacleMode() {
    // int move[] = {90, 90, 90, 90};
    // int delaytime = 500;
    bool turnFlag = true;
    servoDetach();
    distance = getDistance();
    Serial.print("distance:");
    Serial.print(distance);
    if (distance >= 1 && distance <= 300) {
        st188Val_L = analogRead(ST188_L_PIN);
        st188Val_R = analogRead(ST188_R_PIN);
        Serial.print("\tst188Val_L:");
        Serial.print(st188Val_L);
        Serial.print("\tst188Val_R:");
        Serial.print(st188Val_R);
        if (st188Val_L >= 1000 && st188Val_R >= 1000) {
            Serial.println("\tGO BACKWAED 1");
            servoAttach();
            walk(3, t * 4, -1);
            if (turnFlag) {
                turn(3, t * 4, 1);
            } else {
                turn(3, t * 4, -1);
            }
            // moveNServos(delaytime, move);
            // delay(delaytime);
            servoDetach();
        } else if (st188Val_L >= 1000 && st188Val_R < 1000) {
            Serial.println("\tTURN RIGHT 2");
            turnFlag = true;
            servoAttach();
            turn(3, t * 4, 1);
            // moveNServos(delaytime, move);
            // delay(delaytime);
            servoDetach();
        } else if (st188Val_L < 1000 && st188Val_R >= 1000) {
            Serial.println("\tTURN LEFT 3");
            turnFlag = false;
            servoAttach();
            turn(3, t * 4, -1);
            // moveNServos(delaytime, move);
            // delay(delaytime);
            servoDetach();
        } else if (st188Val_L < 1000 && st188Val_R < 1000) {
            if (distance < 5) {
                Serial.println("\tGO BACKWAED 4");
                servoAttach();
                walk(3, t * 4, -1);
                if (turnFlag) {
                    turn(3, t * 4, 1);
                } else {
                    turn(3, t * 4, -1);
                }
                // moveNServos(delaytime, move);
                // delay(delaytime);
                servoDetach();
            } else if (distance >= 5 && distance <= 20) {
                Serial.println("\tTURN RIGHT 5");
                servoAttach();
                if (turnFlag) {
                    turn(1, t * 4, 1);
                } else {
                    turn(1, t * 4, -1);
                }
                // moveNServos(delaytime, move);
                // delay(delaytime);
                servoDetach();
            } else {
                Serial.println("\tGO FORWARD 6");
                servoAttach();
                walk(1, t * 4, 1);
                // moveNServos(delaytime, move);
                // delay(delaytime);
                servoDetach();
            }
        }
    } else {
        Serial.println("\tSTOP 7");
        servoAttach();
        home();
        servoDetach();
    }
}
void followMode() {
    int move[] = {90, 90, 90, 90};
    int delaytime = 500;
    servoDetach();
    distance = getDistance();
    Serial.print("distance:");
    Serial.print(distance);
    if (distance >= 1 && distance <= 300) {
        st188Val_L = analogRead(ST188_L_PIN);
        st188Val_R = analogRead(ST188_R_PIN);
        Serial.print("\tst188Val_L:");
        Serial.print(st188Val_L);
        Serial.print("\tst188Val_R:");
        Serial.print(st188Val_R);
        if (st188Val_L >= 1000 && st188Val_R >= 1000) {
            Serial.println("\tGO FORWARD 1");
            servoAttach();
            walk(1, t * 4, 1);
            // moveNServos(delaytime, move);
            // delay(delaytime);
            servoDetach();
        } else if (st188Val_L >= 1000 && st188Val_R < 1000) {
            Serial.println("\tTURN LEFT 2");
            servoAttach();
            turn(1, t * 4, -1);
            // moveNServos(delaytime, move);
            // delay(delaytime);
            servoDetach();
        } else if (st188Val_L < 1000 && st188Val_R >= 1000) {
            Serial.println("\tTURN RIGHT 3");
            servoAttach();
            turn(1, t * 4, 1);
            // moveNServos(delaytime, move);
            // delay(delaytime);
            servoDetach();
        } else if (st188Val_L < 1000 && st188Val_R < 1000) {
            if (distance > 20) {
                Serial.println("\tSTOP 4");
                servoAttach();
                home();
                servoDetach();
            } else {
                Serial.println("\tGO FORWARD 5");
                servoAttach();
                walk(1, t * 4, 1);
                // moveNServos(delaytime, move);
                // delay(delaytime);
                servoDetach();
            }
        }
    } else {
        Serial.println("\tSTOP 6");
        servoAttach();
        home();
        servoDetach();
    }
}
void st188Adjust(int dis) {
    if (millis() - infraredMeasureTime > 1000 && dis > 20 && dis < 200 && analogRead(ST188_L_PIN) < 300 && analogRead(ST188_R_PIN) < 300) {
        unsigned long st188RightData = 0;
        unsigned long st188LeftData = 0;
        for (int n = 0; n < 10; n++) {
            st188LeftData += analogRead(ST188_L_PIN);
            st188RightData += analogRead(ST188_R_PIN);
        }
        ST188LeftDataMin = st188LeftData / 10;
        ST188RightDataMin = st188RightData / 10;
        ST188Threshold = ST188LeftDataMin - ST188RightDataMin;
        infraredMeasureTime = millis();
    }
}
void voltageMeasure() {
    if (millis() - voltageMeasureTime > 10000) {
        int ADCValue = analogRead(VOLTAGE_MEASURE_PIN);
        double volMeasure = ADCValue * VREF / 1024;
        double VCC = volMeasure * (RES1 + RES2) / RES2;
        // Serial.print("ADCValue = ");
        // Serial.print(ADCValue);
        // Serial.print("\t");
        // Serial.print("volMeasure = ");
        // Serial.print(volMeasure);
        // Serial.print("\t");
        Serial.print("VCC = ");
        Serial.print(VCC);
        Serial.println(" V");
        //double VCC = analogRead(VOLTAGE_MEASURE_PIN) * 1.1 * 6 / 1024;
        if (VCC < 4.8) {
            LED_flag = false;
        } else {
            LED_flag = true;
        }
        voltageMeasureTime = millis();
    }
    if (LED_flag) {
        analogWrite(INDICATOR_LED_PIN, 255);
    } else {
        if (millis() - ledBlinkTime < 500) {
            analogWrite(INDICATOR_LED_PIN, 255);
        } else if (millis() - ledBlinkTime >= 500 && millis() - ledBlinkTime < 1000) {
            analogWrite(INDICATOR_LED_PIN, 0);
        } else {
            ledBlinkTime = millis();
        }
    }
}
boolean getIRValue() {
    if (irrecv.decode(&results)) {
        irValue = results.value;
        Serial.println(irValue);
        irrecv.resume();
        return true;
    }
    return false;
}
void servoAttach() {
    servo[0].attach(RR_PIN);
    servo[1].attach(RL_PIN);
    servo[2].attach(YR_PIN);
    servo[3].attach(YL_PIN);
}
void servoDetach() {
    servo[0].detach();
    servo[1].detach();
    servo[2].detach();
    servo[3].detach();
}
void setup() {
    Serial.begin(9600);
    mp3Serial.begin(9600);
    pinMode(ECHO_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(INDICATOR_LED_PIN, OUTPUT);
    pinMode(VOLTAGE_MEASURE_PIN, INPUT);
    analogWrite(INDICATOR_LED_PIN, LED_value);
    MP3.init();
    irrecv.enableIRIn();
    analogReference(INTERNAL);
    servoAttach();
    servo[0].SetTrim(TRIM_RR);
    servo[1].SetTrim(TRIM_RL);
    servo[2].SetTrim(TRIM_YR);
    servo[3].SetTrim(TRIM_YL);
    home();
    servoDetach();
    delay(2000);
    start();
}
void loop() {
    voltageMeasure();
    if (getIRValue()) {
        Serial.print("irValue = ");
        Serial.println(irValue);
        switch (irValue) {
        case BTN_UP:
            MP3.stopPlay();
            mode = IRREMOTE;
            IRmode = FORWARD;
            break;
        case BTN_DOWN:
            MP3.stopPlay();
            mode = IRREMOTE;
            IRmode = BACKWAED;
            break;
        case BTN_LEFT:
            MP3.stopPlay();
            mode = IRREMOTE;
            IRmode = TURNLIFT;
            break;
        case BTN_RIGHT:
            MP3.stopPlay();
            mode = IRREMOTE;
            IRmode = TURNRIGHT;
            break;
        case BTN_MODE:
            servoDetach();
            Serial.println("BTN_MODE");
            if (mode == FOLLOW) {
                delay(10);
                MP3.stopPlay();
                delay(10);
                MP3.playSong(7, MP3.volume);
                mode = OBSTACLE;
                Serial.println("OBSTACLE");
            } else {
                delay(10);
                MP3.stopPlay();
                delay(10);
                MP3.playSong(6, MP3.volume);
                mode = FOLLOW;
                Serial.println("FOLLOW");
            }
            break;
        case BTN_IDLE:
            MP3.stopPlay();
            mode = IDLE;
            servoAttach();
            home();
            servoDetach();
            break;
        case BTN_MUSIC:
            servoDetach();
            MP3.stopPlay();
            mode = MUSIC;
            MP3.playSong(musicIndex, MP3.volume);
            preMp3Millis = millis();
            break;
        case BTN_DANCE:
            servoDetach();
            MP3.stopPlay();
            mode = DANCE;
            if (danceNum == 0) {
                delay(10);
                MP3.playSong(danceIndex, MP3.volume);
            }
            danceNum++;
            if (danceFlag == true || danceNum >= 2) {
                delay(10);
                MP3.stopPlay();
                delay(10);
                MP3.playSong(danceIndex, MP3.volume);
                servoAttach();
                switch (danceIndex) {
                case 2:
                    dance2();
                    break;
                case 3:
                    dance3();
                    break;
                case 4:
                    dance4();
                    break;
                default:
                    break;
                }
                servoDetach();
                danceFlag = false;
                MP3.stopPlay();
                danceNum = 0;
            }
            break;
        case BTN_SUB:
            servoDetach();
            MP3.stopPlay();
            if (mode == MUSIC) {
                musicIndex++;
                if (musicIndex > 4) {
                    musicIndex = 2;
                }
                MP3.playSong(musicIndex, MP3.volume);
            }
            if (mode == DANCE) {
                danceFlag = true;
                danceIndex++;
                if (danceIndex > 4) {
                    danceIndex = 2;
                }
                MP3.playSong(danceIndex, MP3.volume);
            }
            if (mode == VOLUME) {
                MP3.volumeDown();
                MP3.volume -= 1;
                if (MP3.volume <= 0) {
                    MP3.volume = 0;
                }
                MP3.playSong(5, MP3.volume);
            }
            break;
        case BTN_ADD:
            servoDetach();
            MP3.stopPlay();
            if (mode == MUSIC) {
                musicIndex--;
                if (musicIndex < 2) {
                    musicIndex = 4;
                }
                MP3.playSong(musicIndex, MP3.volume);
            }
            if (mode == DANCE) {
                danceFlag = true;
                danceIndex--;
                if (danceIndex < 2) {
                    danceIndex = 4;
                }
                MP3.playSong(danceIndex, MP3.volume);
            }
            if (mode == VOLUME) {
                MP3.volumePlus();
                MP3.volume += 1;
                if (MP3.volume >= 30) {
                    MP3.volume = 30;
                }
                MP3.playSong(5, MP3.volume);
            }
            break;
        case BTN_VOL:
            servoDetach();
            mode = VOLUME;
            delay(10);
            MP3.stopPlay();
            delay(10);
            MP3.playSong(5, MP3.volume);
            break;
        default:
            break;
        }
    }
    switch (mode) {
    case IDLE:
        break;
    case IRREMOTE:
        switch (IRmode) {
        case FORWARD:
            servoAttach();
            walk(1, t * 4, 1);
            servoDetach();
            break;
        case BACKWAED:
            servoAttach();
            walk(1, t * 4, -1);
            servoDetach();
            break;
        case TURNRIGHT:
            servoAttach();
            turn(1, t * 4, 1);
            servoDetach();
            break;
        case TURNLIFT:
            servoAttach();
            turn(1, t * 4, -1);
            servoDetach();
            break;
        default:
            break;
        }
        break;
    case OBSTACLE:
        servoAttach();
        obstacleMode();
        servoDetach();
        break;
    case FOLLOW:
        servoAttach();
        followMode();
        servoDetach();
        break;
    case MUSIC:
        if (millis() - preMp3Millis > 1000) {
            preMp3Millis = millis();
            if (MP3.getPlayStatus() == MP3.playStatus[0]) {
                musicIndex++;
                if (musicIndex > 4) {
                    musicIndex = 2;
                }
                MP3.playSong(musicIndex, MP3.volume);
            }
        }
        break;
    case DANCE:
        break;
    default: break;
    }
}
