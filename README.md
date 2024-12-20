# electronic-load

#include <Adafruit_ADS1X15.h>
#include <Adafruit_INA219.h>
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

// ADS1115 및 INA219 객체 생성
Adafruit_ADS1115 ads;
Adafruit_INA219 ina219;

// 핀 및 모드 변수
const int pwmOutputPin = 9; // PWM 출력 핀
const int encoderPinA = 2;  // 로터리 인코더 핀 A
const int encoderPinB = 3;  // 로터리 인코더 핀 B
const int buttonPin = 4;    // 버튼 핀
const int LM35_PIN = A0;            // LM35 센서를 연결한 아날로그 핀

volatile int encoderValue = 0; // 로터리 인코더 값
volatile int lastEncoded = 0;  // 이전 인코더 상태

bool settingMode = false;     // 설정 모드 플래그

int buttonState = HIGH;
int lastButtonState = HIGH;

volatile int mode = 0;      // 현재 모드 (0: CC, 1: CP, 2: CR)

const char *modes[] = {"CC", "CP", "CR"};

// 공통 변수
float sense_voltage = 0.0; // 배터리 전압
float sense_current = 0.0; // 드레인 전류
float sense_Power = 0.0;
float sense_Resistor = 0.0;
float temperature = 0.0;

float P_a = 0.0;           // 전력 지령값
float I_a = 0.0;           // 전류 지령값
float R_a = 100.0;

float P_err = 0.0;         // 오차값
float P_err_int = 0.0;     // I 제어 적분값
float P_ref_b = 0.0;       // PI 제어 결과값
float P_ref = 0.0;         // 제한 후 최종 전력
float P_anti = 0.0;        // Anti-windup 보상값

float I_err = 0.0;         // 오차값
float I_err_int = 0.0;     // I 제어 적분값
float I_ref_b = 0.0;       // PI 제어 결과값
float I_ref = 0.0;         // 제한 후 최종 전력
float I_anti = 0.0;        // Anti-windup 보상값

// 제어기 이득 및 샘플링 주기
const float Kic = 0.2;      // 적분 이득
const float Kpc = 100.0;    // 비례 이득
const float Tsamp = 0.0001; // 샘플링 시간 (스위칭 주파수 역수)

const float P_max = 60; // 최대 출력 전력
const float P_min = 0.0; // 최소 출력 전력
const float P_ref_limit = 0.35 * P_max; // I_ref의 상한값 제한

const float I_max = 2.5; // 최대 출력 전력
const float I_min = 0.0; // 최소 출력 전력
const float I_ref_limit = 0.35 * I_max; // I_ref의 상한값 제한

unsigned long lastPrintTime = 0;       // 마지막 출력 시간
const unsigned long printInterval = 1000; // 시리얼 출력 간격 (1초)


const float maxVoltage = 12.6; // 최고 전압 (100%)
const float minVoltage = 7.8;  // 최저 전압 (0%)

float batteryPercentage = 0.0;

const float voltagePoints[] = {12.14, 12.05, 11.96, 11.91, 11.79, 11.47, 11.06, 10.73, 10.44, 10.09, 9.71, 9.12, 8.5};
const float percentagePoints[] = {98, 95, 90, 85, 80, 70, 60, 50, 40, 30, 20, 10, 5}; // 각 전압에 해당하는 잔량(%) 값
const int numPoints = sizeof(voltagePoints) / sizeof(voltagePoints[0]);


void setup() {
  // 시리얼 및 센서 초기화
  Serial.begin(9600);

   lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("LCD Init Done");
  delay(500);
  lcd.clear();

  // I2C 버스 재초기화
  Wire.begin();
  
  // ADS1115 초기화
  if (!ads.begin()) {
    Serial.println("ADS1115 초기화 실패!");
    while (1);
  }

  // INA219 초기화
  if (!ina219.begin()) {
    Serial.println("INA219 초기화 실패!");
    while (1);
  }

  // PWM 핀 설정
  
  pinMode(pwmOutputPin, OUTPUT);

  pinMode(buttonPin, INPUT_PULLUP);


  // 로터리 인코더 핀 설정 및 인터럽트
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);



  Serial.println("Setup Complete!");

  displayMode();

  Serial.println("\nI2C Scanner");
  for (uint8_t i = 0; i < 128; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("I2C device found at address 0x");
      Serial.println(i, HEX);
    }
  }

 
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
  ICR1 = 1599; 

}

void loop() {
  // 측정값 업데이트
  updateMeasurements();
   buttonState = digitalRead(buttonPin);

  // 버튼 눌림 감지 (HIGH → LOW 변화)
  if (buttonState == LOW && lastButtonState == HIGH) {
     delay(50); // 디바운싱 처리

    // encoderValue 초기화
    encoderValue = 0;
    Serial.println("Encoder Value Reset to 0");

   if (settingMode) {
      mode = (mode + 1) % 3; // 모드 순환
      Serial.print("Mode Changed to: ");
      Serial.println(modes[mode]);

      // CR 모드일 때 encoderValue를 60으로 설정
      if (mode == 2) { // CR 모드
        encoderValue = 60;
        Serial.println("Encoder Value Initialized to 1000 for CR Mode");
      }

      displayMode();
    } else {
      settingMode = true;
      Serial.println("Switched to Mode Setting");
      displayMode();
    }
  }

  // 버튼 상태 업데이트
  lastButtonState = buttonState;

  // 현재 상태에 따라 동작
  if (settingMode) {
    runModeFunction(); // 현재 모드에 따른 함수 실행
  } else {

  }


  // 로터리 인코더 값에 따라 I_a 또는 P_a 조정
  if (mode == 0) {
    I_a = encoderValue * 0.01; // 로터리 인코더 값으로 목표 전류 설정
    I_a = constrain(I_a, 0.0, 2.5); // I_a 값 제한 (0 ~ 2.5 A)
  } else if (mode == 1) {
    P_a = encoderValue * 0.01;
    P_a = constrain(P_a, 0.0, 60);
  }
  else if (mode == 2) {
    R_a = encoderValue;
    R_a = constrain(R_a, 5.0, 1000);
  }


  // 모드별 동작
  if (mode == 0) {
    controlCurrent();
} else if (mode == 1) {
    controlPower();
} else if (mode == 2) {
    controlResistor();
}

  batteryPercentage = calculateBatteryPercentage(sense_voltage);

  // 데이터 출력
  if (millis() - lastPrintTime >= printInterval) {
    lastPrintTime = millis();
    printData();
  }
}

void runModeFunction() {
  switch (mode) {
    case 0:
      controlCurrent();
      break;
    case 1:
      controlPower();
      break;
    case 2:
      controlResistor();
      break;
    default:
      Serial.println("Unknown Mode!");
      break;
  }
}

// --- 로터리 인코더 업데이트 ---
void updateEncoder() {
  int MSB = digitalRead(encoderPinA); // 핀 A 상태
  int LSB = digitalRead(encoderPinB); // 핀 B 상태

  // 현재 상태 계산
  int encoded = (MSB << 1) | LSB; 
  int sum = (lastEncoded << 2) | encoded;

  // 방향 결정 (테이블 반전)
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderValue++; // 시계 방향 (CW)
  } else if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderValue--; // 반시계 방향 (CCW)
  }

  // 현재 상태 저장
  lastEncoded = encoded;
}



// --- 측정값 업데이트 ---
void updateMeasurements() {
  int16_t adc0 = ads.readADC_SingleEnded(2);
  float V_L_sense = (adc0 * 0.1875 * 4.8) / 1000.0; // 전압 변환
  sense_voltage = V_L_sense;
  float I_D_sense = ina219.getCurrent_mA() / 1000.0; // 전류 변환
  sense_current = I_D_sense;
  float currentPower = sense_voltage * sense_current;
  sense_Power = currentPower;
  float currentResistor = sense_voltage / sense_current;
  sense_Resistor = currentResistor;

  // LM35 센서를 사용한 온도 측정
  int tempRaw = analogRead(LM35_PIN);                // 센서 값 읽기
  temperature = tempRaw * (5.0 / 1024.0) * 100.0;    // 전압 변환 후 섭씨 온도로 계산
}

// --- CC 모드 제어 ---
void controlCurrent() {

  I_err = I_a - sense_current;
  I_err_int += (I_err - I_anti) * Tsamp; 
  I_ref_b = (Kpc * I_err) + (Kic * I_err_int);

  if (I_ref_b > I_max) {
    I_ref = I_max;
  } else if (I_ref_b < I_min) {
    I_ref = I_min;
  } else {
    I_ref = I_ref_b;
  }

  // Anti-windup 보상
  I_anti = (1.0 / Kpc) * (I_ref_b - I_ref);

  if (I_ref > I_ref_limit) {
    I_ref = I_ref_limit;
  }

  OCR1A = (I_ref / I_max) * ICR1;

}

// --- CP 모드 제어 ---
void controlPower() {
  P_err = P_a - sense_Power;           // 오차 계산
  P_err_int += (P_err - P_anti) * Tsamp; // 오차 적분
  P_ref_b = (Kpc * P_err) + (Kic * P_err_int);

  if (P_ref_b > P_max) {
    P_ref = P_max;
  } else if (P_ref_b < P_min) {
    P_ref = P_min;
  } else {
    P_ref = P_ref_b;
  }

  // Anti-windup 보상
  P_anti = (1.0 / Kpc) * (P_ref_b - P_ref);

  // P_ref 제한: (P_ref / P_max)가 0.3 이상이 되지 않도록 제한
  if (P_ref > P_ref_limit) {
    P_ref = P_ref_limit;
  }

  OCR1A = (P_ref / P_max) * ICR1;

}

void controlResistor() {
  I_a = sense_voltage / R_a;
  I_err = I_a - sense_current;           // 오차 계산
  I_err_int += (I_err - I_anti) * Tsamp; // 오차 적분
  I_ref_b = (Kpc * I_err) + (Kic * I_err_int);

  if (I_ref_b > I_max) {
    I_ref = I_max;
  } else if (I_ref_b < I_min) {
    I_ref = I_min;
  } else {
    I_ref = I_ref_b;
  }

  // Anti-windup 보상
  I_anti = (1.0 / Kpc) * (I_ref_b - I_ref);

  // P_ref 제한: (P_ref / P_max)가 0.3 이상이 되지 않도록 제한
  if (I_ref > I_ref_limit) {
    I_ref = I_ref_limit;
  }

  OCR1A = (I_ref / I_max) * ICR1;

}

float calculateBatteryPercentage(float voltage) {
  if (voltage >= maxVoltage) {
    return 100.0; // 최고 전압 이상은 100%
  }
  if (voltage <= minVoltage) {
    return 0.0; // 최저 전압 이하 0%
  }

  // 선형 보간법 사용
  for (int i = 0; i < numPoints - 1; i++) {
    if (voltage <= voltagePoints[i] && voltage > voltagePoints[i + 1]) {
      // 두 포인트 사이에서 보간
      float voltageRange = voltagePoints[i] - voltagePoints[i + 1];
      float percentageRange = percentagePoints[i] - percentagePoints[i + 1];
      float proportion = (voltage - voltagePoints[i + 1]) / voltageRange;
      return percentagePoints[i + 1] + proportion * percentageRange;
    }
  }

  return 0.0; // 비정상적인 경우 (이론적으로 여기 도달하지 않음)
}

// --- 데이터 출력 ---
void printData() {
  Serial.println("-----------------------");
  Serial.print("Mode: ");
  if (mode == 0) Serial.println("CC");
  else if (mode == 1) Serial.println("CP");
  else if (mode == 2) Serial.println("CR");

  Serial.println(encoderValue);

  Serial.print("Voltage: ");
  Serial.print(sense_voltage, 4);
  Serial.println(" V");

  Serial.print("Current: ");
  Serial.print(sense_current, 4);
  Serial.println(" A");

  Serial.print("Power: ");
  Serial.print(sense_Power, 4);
  Serial.println(" W");
  
  Serial.print("Resistor: ");
  Serial.print(sense_Resistor, 4);
  Serial.println(" ohm");

  Serial.print("Temp: ");
  Serial.print(temperature, 2);
  Serial.println("C");

    lcd.setCursor(0, 1);
    lcd.print("V:");
    lcd.print(sense_voltage, 3);
    lcd.print("V");
    lcd.setCursor(10, 1);
    lcd.print("I:");
    lcd.print(sense_current,3);
    lcd.print("A");
    lcd.setCursor(0, 2);
    lcd.print("P:");
    lcd.print(sense_Power,3);
    lcd.setCursor(7, 2);
    lcd.print("W");
    lcd.setCursor(9, 2);
    lcd.print("Tem: ");
    lcd.print(temperature, 1);
    lcd.setCursor(18, 2);
    lcd.print("C"); // 공백으로 이전 값 지우기
    lcd.setCursor(0, 3);
    lcd.print("R:");
    lcd.print(sense_Resistor, 1);
    lcd.setCursor(6, 3);
    lcd.print("ohm");
    lcd.setCursor(10, 3);
    lcd.print("soc:");
    lcd.setCursor(14, 3);
    lcd.print(batteryPercentage);
    lcd.setCursor(18, 3);
    lcd.print("%");

  if (mode == 0) {
    Serial.print("Target Current: ");
    Serial.print(I_a, 4);
    Serial.println(" A");
    lcd.setCursor(8, 0);
    lcd.print("I_a: ");
    lcd.setCursor(13, 0);
    lcd.print(I_a, 2);
    lcd.setCursor(18, 0);
    lcd.print("A");
    
  } else if (mode == 1) {
    Serial.print("Target Power: ");
    Serial.print(P_a, 3);
    Serial.println(" W");
    lcd.setCursor(8, 0);
    lcd.print("P_a: ");
    lcd.setCursor(13, 0);
    lcd.print(P_a, 2);
    lcd.setCursor(18, 0);
    lcd.print("W");
  }
  else if (mode == 2) {
    Serial.print("Target Resistor: ");
    Serial.print(R_a, 1);
    Serial.println(" ohm");
    lcd.setCursor(8, 0);
    lcd.print("R_a: ");
    lcd.setCursor(13, 0);
    lcd.print(R_a, 1);
    lcd.setCursor(17, 0);
    lcd.print("ohm");
  }
}

// --- LCD에 현재 모드 표시 ---
void displayMode() {
  lcd.setCursor(0, 0); // 2번째 줄 첫 번째 위치
  lcd.print("Mode:                "); // 기존 텍스트 지우기
  lcd.setCursor(5, 0); // "Mode:" 뒤에 위치
  lcd.print(modes[mode]);
}



