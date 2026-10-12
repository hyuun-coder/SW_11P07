#include <Servo.h>

// 핀 설정
#define PIN_LED   9       // active-low
#define PIN_TRIG  12
#define PIN_ECHO  13
#define PIN_SERVO 10

// 초음파 센서 설정
#define SND_VEL 346.0
#define INTERVAL 100      // 측정 간격: ms
#define PULSE_DURATION 10
#define TIMEOUT 25000UL
#define SCALE (0.001 * 0.5 * SND_VEL)

// 거리 범위: mm
#define _DIST_MIN 180.0
#define _DIST_MAX 360.0

// EMA: 작으면 더 부드럽게, 크면 더 빠르게 반응
#define _EMA_ALPHA 0.3

// 직접 측정한 서보 보정값
#define _DUTY_MIN 500
#define _DUTY_MAX 4000
#define _DUTY_NEU 1500

Servo myservo;

unsigned long last_sampling_time = 0;
float dist_ema = (_DIST_MIN + _DIST_MAX) / 2.0;
bool has_valid_sample = false;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  digitalWrite(PIN_LED, HIGH);
  digitalWrite(PIN_TRIG, LOW);

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);

  Serial.begin(57600);
}

void loop() {
  if (millis() - last_sampling_time < INTERVAL) {
    return;
  }
  last_sampling_time = millis();

  float dist_raw = USS_measure();

  if (dist_raw > 0.0) {
    // 실제 측정 거리가 범위 안일 때만 LED 켜기
    bool in_range =
      (dist_raw >= _DIST_MIN && dist_raw <= _DIST_MAX);

    digitalWrite(PIN_LED, in_range ? LOW : HIGH);

    // 범위 제한: 18cm 미만은 18cm, 36cm 초과는 36cm
    float dist_filtered =
      constrain(dist_raw, _DIST_MIN, _DIST_MAX);

    // 첫 정상 측정값으로 EMA 초기화
    if (!has_valid_sample) {
      dist_ema = dist_filtered;
      has_valid_sample = true;
    } else {
      // EMA 필터
      dist_ema = _EMA_ALPHA * dist_filtered
               + (1.0 - _EMA_ALPHA) * dist_ema;
    }
  } else {
    // 측정 실패: 이전 EMA 및 서보 위치 유지
    digitalWrite(PIN_LED, HIGH);
  }

  // 거리 180~360mm를 비율 0~1로 변환
  float ratio =
    (dist_ema - _DIST_MIN) / (_DIST_MAX - _DIST_MIN);

  ratio = constrain(ratio, 0.0, 1.0);

  // 보정된 펄스 범위로 변환
  int duty = (int)(
    _DUTY_MIN + ratio * (_DUTY_MAX - _DUTY_MIN) + 0.5
  );

  myservo.writeMicroseconds(duty);

  // 시리얼 플로터 출력
  Serial.print("Min:");
  Serial.print(_DIST_MIN);
  Serial.print(",dist:");
  Serial.print(dist_raw);
  Serial.print(",ema:");
  Serial.print(dist_ema);
  Serial.print(",Servo:");
  Serial.print(myservo.read());
  Serial.print(",Max:");
  Serial.println(_DIST_MAX);
}

// 초음파 거리 측정: mm
float USS_measure() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long duration =
    pulseIn(PIN_ECHO, HIGH, TIMEOUT);

  return duration * SCALE;
}
