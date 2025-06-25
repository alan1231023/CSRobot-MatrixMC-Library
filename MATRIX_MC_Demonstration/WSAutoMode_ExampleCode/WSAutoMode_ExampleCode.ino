#include <MatrixMiniR4.h>
#include <MatrixMC.h>
unsigned long timer_1 = 0;

MatrixMC matrixMC(true, false, false, false);

void setup() {
  Serial.begin(115200);
  MiniR4.begin();
  matrixMC.begin();
  MiniR4.PWR.setBattCell(3);

  matrixMC.MC1.M1.setReverse(true);
  matrixMC.MC1.M2.setReverse(false);
  matrixMC.MC1.M3.setReverse(true);
  matrixMC.MC1.M4.setReverse(false);
  matrixMC.MC1.M1.setBrake(true);
  matrixMC.MC1.M2.setBrake(true);
  matrixMC.MC1.M3.setBrake(true);
  matrixMC.MC1.M4.setBrake(true);

  matrixMC.MC1.RC1.setReverse(false);
  matrixMC.MC1.RC2.setReverse(true);
  matrixMC.MC1.RC1.setAngle(0);
  matrixMC.MC1.RC2.setAngle(0);

  while(!(MiniR4.BTN_UP.getState() || MiniR4.BTN_DOWN.getState())){
    MiniR4.OLED.clearDisplay();
    MiniR4.OLED.setTextColor(SSD1306_WHITE);
    MiniR4.OLED.setCursor(10, 10);
    MiniR4.OLED.print("READY");
    MiniR4.OLED.display();
    matrixMC.loop();
  }
  MiniR4.OLED.clearDisplay();
  MiniR4.OLED.setTextColor(SSD1306_WHITE);
  MiniR4.OLED.setCursor(10, 10);
  MiniR4.OLED.print("RUN");
  MiniR4.OLED.display();
  MiniR4.OLED.setTextSize(1);
  matrixMC.loop();
}

// ----------------------------MAIN-------------------------------------
void loop() {
  // Motor_ShowDegrees();                 // 顯示馬達控制器角度M1~M4
  // Move( 30, 30);
  // Move_Deg( 50, 50, 360, true);        // 機器移動至指定角度
  // Move_Time( 50, 50, 1000, true);      // 機器移動直到時間到
}
// ---------------------------------------------------------------------

void Move( int M1, int M2) {
  matrixMC.MC1.M1.setPower(M1);
  matrixMC.MC1.M2.setPower(M2);
  MiniR4.M1.setPower(M1);
  matrixMC.loop();
}

void Move_Deg( int M1, int M2, int Degrees, bool Brake) {
  matrixMC.MC1.M1.resetCounter();
  matrixMC.MC1.M2.resetCounter();
  while( (abs(matrixMC.MC1.M1.getDegrees()) + abs(matrixMC.MC1.M2.getDegrees())) / 2 < Degrees) {
    Move( M1, M2);
  }
  if(Brake) {
    Move( 0, 0);
  }
}

void Move_Time( int M1, int M2, int Time, bool Brake) {
  timer_1 = millis();
  while(!((millis() - timer_1) > Time))
  {
    Move( M1, M2);
  }
  if(Brake) {
    Move( 0, 0);
  }
}

void Motor_ShowDegrees() {
  matrixMC.loop();
  int M1_Deg = matrixMC.MC1.M1.getDegrees();
  int M2_Deg = matrixMC.MC1.M2.getDegrees();
  int M3_Deg = matrixMC.MC1.M3.getDegrees();
  int M4_Deg = matrixMC.MC1.M4.getDegrees();

  MiniR4.OLED.clearDisplay();
  MiniR4.OLED.setTextColor(SSD1306_WHITE);
  MiniR4.OLED.setCursor(0, 0);
  MiniR4.OLED.print("M1:");
  MiniR4.OLED.print(M1_Deg);
  MiniR4.OLED.setCursor(0, 16);
  MiniR4.OLED.print("M2:");
  MiniR4.OLED.print(M2_Deg);
  MiniR4.OLED.setCursor(64, 0);
  MiniR4.OLED.print("M3:");
  MiniR4.OLED.print(M3_Deg);
  MiniR4.OLED.setCursor(64, 16);
  MiniR4.OLED.print("M4:");
  MiniR4.OLED.print(M4_Deg);
  MiniR4.OLED.display();
  delay(1);
}
