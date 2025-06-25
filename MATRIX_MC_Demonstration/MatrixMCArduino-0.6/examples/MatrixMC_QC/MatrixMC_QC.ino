#include <MatrixMC.h>

MatrixMC matrixMC(true, false, false, false);

void setup()
{
    Serial.begin(115200);
    matrixMC.begin();
}

void loop()
{
    static uint32_t timer1 = 0;
    static uint32_t timer2 = 0;
    static int8_t   speed = 0, dir = 1, angleDir = 1;
    static uint8_t  angle = 0;

    matrixMC.loop();

    if (millis() > timer1) {
        timer1 = millis() + 100;

        int32_t counter[4];
        counter[0] = matrixMC.MC1.M1.getCounter();
        counter[1] = matrixMC.MC1.M2.getCounter();
        counter[2] = matrixMC.MC1.M3.getCounter();
        counter[3] = matrixMC.MC1.M4.getCounter();

        char buff[32];
        for (int i = 0; i < 4; i++) {
            sprintf(buff, "Counter[%d]: %ld", i, counter[i]);
            Serial.println(buff);
        }
        Serial.println();

        speed += dir;
        if (speed > 100 || speed < -100) {
            dir *= -1;
        } else {
            matrixMC.MC1.M1.setSpeed(speed);
            matrixMC.MC1.M2.setSpeed(speed);
            matrixMC.MC1.M3.setSpeed(speed);
            matrixMC.MC1.M4.setSpeed(speed);
        }
    }

    if (millis() > timer2) {
        timer2 = millis() + 20;
        angle += angleDir;
        if (angle > 180 || angle <= 0) {
            angleDir *= -1;
        } else {
            matrixMC.MC1.RC1.setAngle(angle);
            matrixMC.MC1.RC2.setAngle(angle);
        }
    }
}
