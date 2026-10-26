/*
  ==========================================================
   ถุงมือกายภาพบำบัด v4 - Arduino UNO R4 WiFi + บอร์ด K-Matrix Mini R4
   ควบคุมผ่าน BLE (ArduinoBLE) + Servo DS3218MG 4 ตัว (ดึงแยกทีละนิ้วได้)
  ==========================================================

  สิ่งที่เปลี่ยนจาก v3
    1) เปลี่ยนมอเตอร์ DC ตัวเดียว เป็น Servo 4 ตัว ต่อที่พอร์ต SERVO 1-4 ของบอร์ด
         SERVO 1 = นิ้วโป้ง, 2 = นิ้วชี้, 3 = นิ้วกลาง, 4 = นิ้วนาง + นิ้วก้อย
       (ถ้าเสียบสลับช่อง แก้ที่ SERVO_PORT ด้านล่าง)
    2) โหมดฝึก: กำ-แบ (เลือกได้ว่านิ้วไหนร่วมฝึก), ไล่นิ้ว, แตะนิ้วโป้ง
    3) คาลิเบรตแยกทีละนิ้ว และจำค่าไว้ใน EEPROM (ปิดเครื่องแล้วไม่ต้องตั้งใหม่)
    4) ส่งตำแหน่ง / ระยะกำสูงสุด / แรงต้าน แยกรายนิ้ว กลับไปให้เว็บแอป
    5) คำสั่งของ v3 ใช้ได้เหมือนเดิมทุกคำสั่ง (ไม่เลือกนิ้ว = ทั้งมือ)

  ระบบตรวจจับนิ้วเกร็งต้าน (เลือกเปิดได้ทั้งสองแบบ ด้านล่าง)
    USE_POT_FEEDBACK  : บัดกรีสายจากขากลาง pot ใน Servo ออกมาเข้าพอร์ต ANALOG
                        พอร์ต A1 = ขา A0 (โป้ง), A1 (ชี้) / พอร์ต A2 = ขา A2 (กลาง), A3 (นาง+ก้อย)
                        ได้ตำแหน่งจริงของแกน Servo ใช้จับว่า Servo ถูกต้านจนหมุนไม่ถึง
    USE_CURRENT_SENSE : INA219 คั่นสายไฟ + ของ Servo แต่ละตัว เสียบพอร์ต I2C1-I2C4
                        กระแสสูง = Servo ออกแรงมาก = นิ้วต้าน
    เพราะสายดึงต่ออนุกรมกับสปริง ถ้านิ้วต้าน สปริงจะยืดและ Servo ยังหมุนถึงเป้าหมายได้
    การวัดกระแส (INA219) จึงจับแรงต้านได้ไวกว่า ส่วน pot ใช้จับกรณีติดขัดหนัก และบอกตำแหน่งจริง
    ถ้าปิดทั้งสองแบบ ความปลอดภัยจะเหลือแค่: ระยะกำที่คาลิเบรต + ความเร็วแบบ ramp + สปริง + ESTOP

  ไลบรารีที่ต้องมี: ArduinoBLE, MatrixMiniR4

  ⚠️ ตั้งค่าครั้งแรก ให้ทดสอบโดยไม่สวมมือก่อนเสมอ
    - เปิดเครื่องก่อนสวมถุงมือทุกครั้ง (ตอนเปิดเครื่อง Servo จะหมุนไปตำแหน่งแบมือทันที)
    - เลือกนิ้ว (SEL:n) -> กด JOG จนสายเริ่มตึงพอดี -> ZERO -> JOG:+ จนกำได้ระดับที่ผู้ป่วยรับได้ -> SETMAX
    - ถ้ากด JOG:+ แล้วนิ้วคลายแทนที่จะกำ ให้ส่ง FLIP (สลับทิศของนิ้วที่เลือก)
    - ควรตั้งตำแหน่งแบมือไว้ห่างจาก 0° หรือ 180° อย่างน้อย 10° เพื่อให้มีระยะคลายเอ็น

  โปรโตคอลคำสั่ง (ข้อความ ASCII ผ่าน BLE หรือ Serial 115200 ก็ได้)
    คำสั่งเดิม : START, PAUSE, RESUME, STOP, ESTOP, TGT:10, SPD:50, HOLD:2000,
                ROM:140 (ระยะกำเป็นองศา Servo ของนิ้วที่เลือก), JOG:+ / JOG:-, ZERO, SETMAX, SYNC
    คำสั่งใหม่ : MODE:GRIP|WAVE|OPP, FMASK:1111 (โป้ง,ชี้,กลาง,นาง+ก้อย),
                SEL:0-4 (0 = ทุกนิ้ว ใช้กับ JOG/ZERO/SETMAX/ROM/FLIP), FLIP, CUR:900 (เกณฑ์กระแส mA)

  ข้อความที่ส่งกลับ
    เดิม : STARTED, PHASE:GRIP|HOLD|RELEASE|TWIST, ROUND:n, PEAK:pct, POS:pct,
           STALL:pct, DONE, RETURNING, RETURNED, PAUSED, RESUMED, STOPPED, ESTOP,
           ERR:<เหตุผล>, BATT:7.4, CFG:rom,target,speed,holdMs,mode,mask, STATE:phase,round,paused
    ใหม่ : ACT:0100 (นิ้วที่กำลังฝึกในจังหวะนี้), FPOS:a,b,c,d, FPEAK:a,b,c,d ('-' = ไม่ได้ร่วมฝึก),
           FSTALL:finger,pct, FCFG:rom1,rom2,rom3,rom4,sel, FCUR:mA1,mA2,mA3,mA4,
           SENSE:potMask,curMask,stallMa

  UUID (Nordic UART Service) เหมือน v3 แอปเดิมจึงหาเจอเหมือนเดิม
    Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
    RX     : 6E400002-B5A3-F393-E0A9-E50E24DCCA9E  (เขียนคำสั่งเข้ามา)
    TX     : 6E400003-B5A3-F393-E0A9-E50E24DCCA9E  (notify สถานะ)
*/

#include <ArduinoBLE.h>
#include <MatrixMiniR4.h>
#include <EEPROM.h>
#include <Wire.h>

// ---------- เลือกเซนเซอร์ (1 = เปิด, 0 = ปิด) ----------
#define USE_POT_FEEDBACK   0
#define USE_CURRENT_SENSE  0

// ---------- แบตเตอรี่ ----------
const uint8_t BATT_CELLS = 2;        // จำนวนก้อนถ่าน Li-ion ที่ต่ออนุกรม (2 = 7.4V, 3 = 11.1V)

// ---------- การต่อสาย ----------
const int NF = 4;                                  // จำนวน Servo / นิ้ว
const uint8_t SERVO_PORT[NF] = {1, 2, 3, 4};       // พอร์ต SERVO ของ โป้ง, ชี้, กลาง, นาง+ก้อย
const uint8_t POT_PIN[NF]    = {A0, A1, A2, A3};   // ขา analog ที่ต่อสายจาก pot
const uint8_t INA_CH[NF]     = {0, 1, 2, 3};       // 0-3 = พอร์ต I2C1-I2C4
const uint8_t INA_ADDR       = 0x40;
const float   INA_SHUNT_OHM  = 0.1;                // ตัวต้านทาน shunt บนโมดูล INA219 (ส่วนใหญ่ R100 = 0.1 โอห์ม)
const uint8_t I2C_MUX_ADDR   = 0x70;               // ตัวแยกช่อง I2C บนบอร์ด

// ---------- ค่าที่ปรับได้ ----------
const int  ANGLE_MIN = 0, ANGLE_MAX = 180;
const int  ROM_MIN_DEG     = 20;     // ระยะกำน้อยสุดที่ยอมให้ตั้ง
const int  ROM_DEFAULT_DEG = 140;
const int  JOG_STEP_DEG    = 3;      // กด JOG หนึ่งครั้ง ขยับกี่องศา
const int  TWIST_DEG       = 10;     // ระยะหมุนเลยตำแหน่งแบมือออกไปเพื่อคลายเอ็น
const float GRIP_SEC_SLOW  = 4.0;    // เวลากำจนสุด ที่ความเร็ว 0%
const float GRIP_SEC_FAST  = 0.8;    // เวลากำจนสุด ที่ความเร็ว 100%
const int  SLOWDOWN_PCT    = 12;     // ช่วงท้ายของระยะ (%) ที่ค่อยๆ ลดความเร็ว
const unsigned long STAGGER_MS   = 80;     // ให้แต่ละ Servo ออกตัวห่างกัน ลดไฟกระชาก
const unsigned long TICK_MS      = 10;
const unsigned long MOVE_TIMEOUT = 10000;  // จังหวะใดเกินเวลานี้ = ผิดปกติ
// pot feedback
const int  POT_MIN_SPAN    = 40;     // ค่า ADC ต้องต่างกันอย่างน้อยเท่านี้ระหว่างแบมือ-กำสุด (ไม่งั้นถือว่าไม่ได้ต่อสาย)
const int  POT_LAG_PCT     = 15;     // ตำแหน่งจริงห่างจากที่สั่งเกินกี่ % = ถูกต้าน
const unsigned long POT_STALL_MS = 500;
const int  OVERTRAVEL_PCT  = 25;     // ตำแหน่งจริงเกินระยะกำสุดกี่ % แล้วต้องตัด
// current sense
const int  STALL_MA_DEFAULT = 900;   // กระแสเกินนี้ขณะกำ = นิ้วต้าน (ปรับจากเว็บแอปได้ด้วย CUR:)
const int  HARD_MA          = 2200;  // กระแสเกินนี้นานๆ ไม่ว่าจังหวะไหน = ผิดปกติ ตัดทันที
const unsigned long CUR_STALL_MS = 150;
const unsigned long HARD_MS      = 300;
const int  STALL_BACKOFF_PCT = 3;    // ตรวจพบแรงต้านแล้ว คลายสายกลับเล็กน้อย

// ---------- ค่าที่ปรับจากเว็บได้ ----------
int  targetRounds  = 10;
int  speedPercent  = 50;
unsigned long holdMs = 2000;
enum Mode { MODE_GRIP, MODE_WAVE, MODE_OPP };
Mode mode = MODE_GRIP;
uint8_t fingerMask = 0b1111;   // bit0 = โป้ง ... bit3 = นาง+ก้อย
int  selFinger = 0;            // 0 = ทุกนิ้ว, 1-4 = นิ้วที่เลือกสำหรับคาลิเบรต

// ---------- ค่าคาลิเบรต (เก็บใน EEPROM) ----------
const uint32_t CFG_MAGIC = 0x52474C34;   // "RGL4"
struct Cfg {
  uint32_t magic;
  int16_t  openDeg[NF];     // มุม Servo ตอนแบมือ
  int16_t  closeDeg[NF];    // มุม Servo ตอนกำสุด
  int8_t   dir[NF];         // ทิศกำ: +1 = มุมเพิ่ม, -1 = มุมลด
  int16_t  potDeg1[NF], potAdc1[NF];   // จุดเทียบ pot จุดที่ 1 (ตั้งตอน ZERO)
  int16_t  potDeg2[NF], potAdc2[NF];   // จุดเทียบ pot จุดที่ 2 (ตั้งตอน SETMAX)
  int16_t  stallMa;
};
Cfg cfg;

// ---------- BLE ----------
BLEService uartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic rxChar("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite | BLEWriteWithoutResponse, 40);
BLECharacteristic txChar("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, 40);

// ---------- สถานะ ----------
enum Phase { IDLE, GRIP, HOLD, RELEASE, TWIST_OUT, TWIST_IN, RETURNING };
Phase phase = IDLE;

bool isRunning = false;
bool isPaused  = false;
bool afterReturnPaused = false;
int  roundsCompleted = 0;
unsigned long phaseStart = 0;

// ลำดับจังหวะในหนึ่งรอบ (แต่ละจังหวะ = กลุ่มนิ้วที่กำพร้อมกัน)
uint8_t steps[NF];
int  stepCount = 0;
int  stepIdx = 0;
uint8_t roundMask = 0;     // นิ้วที่ร่วมฝึกในรอบนี้
uint8_t activeMask = 0;    // นิ้วที่กำลังขยับในจังหวะนี้ (ใช้คำนวณ POS)

struct Finger {
  float cmd;               // มุมที่สั่งอยู่ตอนนี้ (องศา)
  int   sent;              // มุมล่าสุดที่ส่งให้ Servo
  float target;
  float rate;              // องศา/วินาที
  unsigned long startAt;
  bool  moving;
  bool  stalled;           // เจอแรงต้านในจังหวะนี้แล้ว
  int   peakPct;           // ระยะกำสูงสุดในรอบนี้
  int   adc;
  float ma;
  bool  inaOk;
  unsigned long overSince, hardSince, lagSince;
};
Finger fg[NF];

unsigned long lastTick = 0;
unsigned long lastSense = 0;
unsigned long lastPosSent = 0, lastFposSent = 0, lastCurSent = 0, lastBattSent = 0;
unsigned long lastBtnRead = 0;
bool lastBtnUp = false;
int  lastSentPct = -999;
int  lastFpos[NF] = {-999, -999, -999, -999};
int  lastFcur[NF] = {-999, -999, -999, -999};

String serialLine;

// =========================================================
void setup() {
  Serial.begin(115200);

  MiniR4.begin();
  MiniR4.PWR.setBattCell(BATT_CELLS);

  loadConfig();
  initFingers();

  if (!BLE.begin()) {
    Serial.println("เริ่มต้น BLE ไม่สำเร็จ");
    while (1);
  }
  BLE.setLocalName("SmartGlove");
  BLE.setAdvertisedService(uartService);
  uartService.addCharacteristic(rxChar);
  uartService.addCharacteristic(txChar);
  BLE.addService(uartService);

  rxChar.setEventHandler(BLEWritten, onRxWritten);
  BLE.setEventHandler(BLEConnected, onConnected);
  BLE.setEventHandler(BLEDisconnected, onDisconnected);

  BLE.advertise();
  Serial.println("BLE พร้อม (ชื่ออุปกรณ์: SmartGlove) — พิมพ์คำสั่งผ่าน Serial ได้ด้วย");
}

void loop() {
  BLE.poll();
  readSerial();
  readButtons();
  readSensors();
  checkSafety();
  updateMotion();
  sendTelemetry();
}

// =========================================================
// ค่าคาลิเบรต
// =========================================================
void defaultConfig() {
  cfg.magic = CFG_MAGIC;
  for (int f = 0; f < NF; f++) {
    cfg.openDeg[f]  = 20;
    cfg.closeDeg[f] = 20 + ROM_DEFAULT_DEG;
    cfg.dir[f]      = 1;
    cfg.potDeg1[f] = cfg.potAdc1[f] = -1;
    cfg.potDeg2[f] = cfg.potAdc2[f] = -1;
  }
  cfg.stallMa = STALL_MA_DEFAULT;
}

void loadConfig() {
  EEPROM.get(0, cfg);
  bool ok = cfg.magic == CFG_MAGIC && cfg.stallMa >= 200 && cfg.stallMa <= 3000;
  for (int f = 0; ok && f < NF; f++) {
    ok = cfg.openDeg[f] >= ANGLE_MIN && cfg.openDeg[f] <= ANGLE_MAX &&
         cfg.closeDeg[f] >= ANGLE_MIN && cfg.closeDeg[f] <= ANGLE_MAX &&
         (cfg.dir[f] == 1 || cfg.dir[f] == -1);
  }
  if (!ok) {
    Serial.println("ยังไม่มีค่าคาลิเบรตใน EEPROM ใช้ค่าเริ่มต้น");
    defaultConfig();
  }
}

void saveConfig() {
  EEPROM.put(0, cfg);
}

int romOf(int f) { return abs(cfg.closeDeg[f] - cfg.openDeg[f]); }

int clampAngle(int a) { return constrain(a, ANGLE_MIN, ANGLE_MAX); }

float angleFromPct(int f, float pct) {
  return cfg.openDeg[f] + (cfg.closeDeg[f] - cfg.openDeg[f]) * pct / 100.0;
}

float pctFromAngle(int f, float a) {
  int span = cfg.closeDeg[f] - cfg.openDeg[f];
  if (span == 0) return 0;
  return (a - cfg.openDeg[f]) * 100.0 / span;
}

// ให้ระยะกำของนิ้ว f เป็น rom องศา ตามทิศกำ (ถ้าชนขอบ 0/180 จะสั้นลง)
void setRom(int f, int rom) {
  cfg.closeDeg[f] = clampAngle(cfg.openDeg[f] + cfg.dir[f] * rom);
}

// =========================================================
// Servo
// =========================================================
void servoWrite(int f, int deg) {
  deg = clampAngle(deg);
  if (deg == fg[f].sent) return;
  fg[f].sent = deg;
  switch (SERVO_PORT[f]) {
    case 1: MiniR4.RC1.setAngle(deg); break;
    case 2: MiniR4.RC2.setAngle(deg); break;
    case 3: MiniR4.RC3.setAngle(deg); break;
    case 4: MiniR4.RC4.setAngle(deg); break;
  }
}

void initFingers() {
  for (int f = 0; f < NF; f++) {
    fg[f].sent = -1;
    fg[f].moving = false;
    fg[f].stalled = false;
    fg[f].ma = 0;
    fg[f].inaOk = false;
  }
#if USE_CURRENT_SENSE
  for (int f = 0; f < NF; f++) fg[f].inaOk = readCurrent(f) >= 0;
#endif
  for (int f = 0; f < NF; f++) {
    fg[f].adc = readPot(f);
    if (potOk(f)) {
      // รู้ตำแหน่งจริงอยู่แล้ว: เริ่มจากตรงนั้น แล้วค่อยๆ เลื่อนไปแบมือ
      fg[f].cmd = constrain(potAngle(f), (float)ANGLE_MIN, (float)ANGLE_MAX);
      servoWrite(f, round(fg[f].cmd));
      setTarget(f, cfg.openDeg[f], 30, f * STAGGER_MS);
    } else {
      // ไม่รู้ตำแหน่งจริง: Servo จะหมุนไปแบมือทันที (จึงต้องเปิดเครื่องก่อนสวมถุงมือ)
      fg[f].cmd = cfg.openDeg[f];
      servoWrite(f, cfg.openDeg[f]);
      delay(150);
    }
  }
}

void setTarget(int f, float targetDeg, float rate, unsigned long delayMs) {
  fg[f].target = clampAngle(round(targetDeg));
  fg[f].rate = rate;
  fg[f].startAt = millis() + delayMs;
  fg[f].moving = true;
  fg[f].stalled = false;
  fg[f].overSince = fg[f].lagSince = 0;
}

void freezeAll() {
  for (int f = 0; f < NF; f++) fg[f].moving = false;
}

bool started(int f) { return (long)(millis() - fg[f].startAt) >= 0; }

// เลื่อนมุมที่สั่งเข้าหาเป้าหมายทีละนิด (Servo จริงวิ่งเร็วมาก จึงต้องทำ ramp เอง)
void tickFingers(float dt) {
  for (int f = 0; f < NF; f++) {
    Finger &F = fg[f];
    if (!F.moving || !started(f)) continue;
    float err = F.target - F.cmd;
    if (fabsf(err) < 0.05) {
      F.cmd = F.target;
      F.moving = false;
    } else {
      float step = F.rate * dt;
      int rom = max(romOf(f), 1);
      float remainPct = fabsf(err) * 100.0 / rom;
      if (remainPct < SLOWDOWN_PCT) step *= max(0.3f, remainPct / SLOWDOWN_PCT);
      F.cmd += (err > 0 ? 1 : -1) * min(step, fabsf(err));
    }
    servoWrite(f, round(F.cmd));
  }
}

bool maskDone(uint8_t mask) {
  for (int f = 0; f < NF; f++) if ((mask >> f & 1) && fg[f].moving) return false;
  return true;
}

// =========================================================
// เซนเซอร์
// =========================================================
int readPot(int f) {
#if USE_POT_FEEDBACK
  long sum = 0;
  for (int i = 0; i < 4; i++) sum += analogRead(POT_PIN[f]);
  return sum / 4;
#else
  return -1;
#endif
}

bool potOk(int f) {
#if USE_POT_FEEDBACK
  return cfg.potAdc1[f] >= 0 && cfg.potAdc2[f] >= 0 &&
         abs(cfg.potDeg2[f] - cfg.potDeg1[f]) >= ROM_MIN_DEG &&
         abs(cfg.potAdc2[f] - cfg.potAdc1[f]) >= POT_MIN_SPAN;
#else
  return false;
#endif
}

// มุมจริงของแกน Servo จากค่า pot (เทียบจากสองจุดที่บันทึกตอน ZERO และ SETMAX)
float potAngle(int f) {
  return cfg.potDeg1[f] + (float)(fg[f].adc - cfg.potAdc1[f]) *
         (cfg.potDeg2[f] - cfg.potDeg1[f]) / (cfg.potAdc2[f] - cfg.potAdc1[f]);
}

// อ่านกระแสจาก INA219 (mA) คืนค่า -1 ถ้าอ่านไม่ได้
float readCurrent(int f) {
  Wire1.beginTransmission(I2C_MUX_ADDR);
  Wire1.write(1 << INA_CH[f]);
  if (Wire1.endTransmission() != 0) return -1;
  Wire1.beginTransmission(INA_ADDR);
  Wire1.write(0x01);                         // shunt voltage register
  if (Wire1.endTransmission() != 0) return -1;
  if (Wire1.requestFrom(INA_ADDR, (uint8_t)2) != 2) return -1;
  int16_t raw = (Wire1.read() << 8) | Wire1.read();
  return fabsf(raw * 0.01f / INA_SHUNT_OHM); // 1 LSB = 10 µV -> mV / โอห์ม = mA
}

void readSensors() {
  if (millis() - lastSense < 20) return;
  lastSense = millis();
  for (int f = 0; f < NF; f++) {
#if USE_POT_FEEDBACK
    fg[f].adc = readPot(f);
#endif
#if USE_CURRENT_SENSE
    if (fg[f].inaOk) {
      float ma = readCurrent(f);
      if (ma >= 0) fg[f].ma = fg[f].ma * 0.6 + ma * 0.4;   // กรองสัญญาณรบกวน
    }
#endif
  }
}

// ตำแหน่งนิ้วเป็น % ของระยะกำ (ใช้ค่าจริงจาก pot ถ้ามี ไม่งั้นใช้มุมที่สั่ง)
int fingerPct(int f) {
  float a = potOk(f) ? potAngle(f) : fg[f].cmd;
  return (int)round(pctFromAngle(f, a));
}

uint8_t potMask() {
  uint8_t m = 0;
  for (int f = 0; f < NF; f++) if (potOk(f)) m |= 1 << f;
  return m;
}

uint8_t curMask() {
  uint8_t m = 0;
  for (int f = 0; f < NF; f++) if (fg[f].inaOk) m |= 1 << f;
  return m;
}

// =========================================================
// ความปลอดภัย
// =========================================================
bool inStepGrip(int f) {
  return (phase == GRIP || phase == HOLD) && (steps[stepIdx] >> f & 1);
}

void checkSafety() {
  if (phase == IDLE) return;
  unsigned long now = millis();
  for (int f = 0; f < NF; f++) {
    Finger &F = fg[f];

#if USE_CURRENT_SENSE
    if (F.inaOk) {
      // กระแสสูงผิดปกติ ไม่ว่าจังหวะไหน
      if (F.ma > HARD_MA) {
        if (!F.hardSince) F.hardSince = now;
        else if (now - F.hardSince > HARD_MS) { fault("OVERCURRENT"); return; }
      } else F.hardSince = 0;

      // นิ้วต้านขณะกำ
      if (inStepGrip(f) && !F.stalled && F.ma > cfg.stallMa) {
        if (!F.overSince) F.overSince = now;
        else if (now - F.overSince > CUR_STALL_MS) onFingerStall(f);
      } else F.overSince = 0;
    }
#endif

#if USE_POT_FEEDBACK
    if (potOk(f) && started(f)) {
      int actual = fingerPct(f);
      if (actual > 100 + OVERTRAVEL_PCT) { fault("OVERTRAVEL"); return; }

      int lag = abs(actual - (int)round(pctFromAngle(f, F.cmd)));
      if (lag > POT_LAG_PCT && !F.stalled) {
        if (!F.lagSince) F.lagSince = now;
        else if (now - F.lagSince > POT_STALL_MS) {
          if (inStepGrip(f)) onFingerStall(f);
          else { fault("STALL"); return; }   // ติดขัดขณะคืนสาย = กลไกมีปัญหา
        }
      } else F.lagSince = 0;
    }
#endif
  }

  if ((phase == GRIP || phase == RELEASE || phase == TWIST_OUT || phase == TWIST_IN || phase == RETURNING) &&
      now - phaseStart > MOVE_TIMEOUT) {
    fault("TIMEOUT");
  }
}

// นิ้วเกร็งต้าน: นิ้วนี้หยุดดึง คลายกลับเล็กน้อย แล้วค้างไว้ นิ้วอื่นทำงานต่อ
void onFingerStall(int f) {
  Finger &F = fg[f];
  F.stalled = true;
  F.moving = false;
  F.overSince = F.lagSince = 0;
  float back = romOf(f) * STALL_BACKOFF_PCT / 100.0;
  F.cmd -= cfg.dir[f] * back;
  servoWrite(f, round(F.cmd));
  int pct = constrain(fingerPct(f), 0, 100);
  sendStatus("STALL:" + String(pct));
  sendStatus("FSTALL:" + String(f + 1) + "," + String(pct));
}

// =========================================================
// BLE / Serial
// =========================================================
void onConnected(BLEDevice central) {
  Serial.print("เชื่อมต่อกับ: ");
  Serial.println(central.address());
  for (int f = 0; f < NF; f++) lastFpos[f] = lastFcur[f] = -999;   // ให้ส่งตำแหน่งนิ้วไปให้แอปใหม่ทันที
}

void onDisconnected(BLEDevice central) {
  Serial.println("อุปกรณ์ตัดการเชื่อมต่อ — คืนมือสู่ตำแหน่งแบมือ");
  if (isRunning) stopTraining();
  BLE.advertise();
}

void onRxWritten(BLEDevice central, BLECharacteristic characteristic) {
  int len = min(rxChar.valueLength(), 40);
  char buf[41];
  memcpy(buf, rxChar.value(), len);
  buf[len] = '\0';
  String cmd(buf);
  cmd.trim();
  handleCommand(cmd);
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      serialLine.trim();
      if (serialLine.length()) handleCommand(serialLine);
      serialLine = "";
    } else if (serialLine.length() < 40) {
      serialLine += c;
    }
  }
}

// ปุ่ม BTN_UP บนบอร์ด = หยุดฉุกเฉิน (กดได้แม้ไม่มีโทรศัพท์)
void readButtons() {
  if (millis() - lastBtnRead < 50) return;
  lastBtnRead = millis();
  bool up = MiniR4.BTN_UP.getState();
  if (up && !lastBtnUp) emergencyStop();
  lastBtnUp = up;
}

void sendStatus(const String &msg) {
  if (BLE.connected()) txChar.writeValue(msg.c_str());
  Serial.println(msg);
}

bool selected(int f) { return selFinger == 0 || selFinger == f + 1; }

void handleCommand(String cmd) {
  cmd.toUpperCase();
  Serial.print("คำสั่ง: ");
  Serial.println(cmd);

  if (cmd == "ESTOP") {
    emergencyStop();
  } else if (cmd == "START") {
    startTraining();
  } else if (cmd == "PAUSE") {
    if (isPaused) resumeTraining();   // คงพฤติกรรมเดิม: กด PAUSE ซ้ำ = เล่นต่อ
    else pauseTraining();
  } else if (cmd == "RESUME") {
    resumeTraining();
  } else if (cmd == "STOP") {
    stopTraining();
  } else if (cmd.startsWith("TGT:")) {
    targetRounds = max(1, (int)cmd.substring(4).toInt());
    sendConfig();
  } else if (cmd.startsWith("SPD:")) {
    speedPercent = constrain((int)cmd.substring(4).toInt(), 0, 100);
    sendConfig();
  } else if (cmd.startsWith("HOLD:")) {
    holdMs = constrain(cmd.substring(5).toInt(), 0L, 10000L);
    sendConfig();
  } else if (cmd.startsWith("MODE:")) {
    String m = cmd.substring(5);
    if (m == "GRIP") mode = MODE_GRIP;
    else if (m == "WAVE") mode = MODE_WAVE;
    else if (m == "OPP") mode = MODE_OPP;
    else { sendStatus("ERR:UNKNOWN_CMD"); return; }
    sendConfig();                     // มีผลตั้งแต่รอบถัดไป
  } else if (cmd.startsWith("FMASK:")) {
    String m = cmd.substring(6);
    if (m.length() != NF) { sendStatus("ERR:UNKNOWN_CMD"); return; }
    uint8_t mask = 0;
    for (int f = 0; f < NF; f++) if (m[f] == '1') mask |= 1 << f;
    fingerMask = mask;
    sendConfig();                     // มีผลตั้งแต่รอบถัดไป
  } else if (cmd.startsWith("SEL:")) {
    selFinger = constrain((int)cmd.substring(4).toInt(), 0, NF);
    sendFingerConfig();
  } else if (cmd.startsWith("CUR:")) {
    cfg.stallMa = constrain((int)cmd.substring(4).toInt(), 200, 3000);
    saveConfig();
    sendSense();
  } else if (cmd.startsWith("ROM:")) {
    if (isRunning) { sendStatus("ERR:BUSY"); return; }
    int rom = constrain((int)cmd.substring(4).toInt(), ROM_MIN_DEG, ANGLE_MAX);
    for (int f = 0; f < NF; f++) if (selected(f)) setRom(f, rom);
    saveConfig();
    sendConfig();
  } else if (cmd == "JOG:+" || cmd == "JOG:-") {
    jog(cmd.endsWith("+") ? 1 : -1);
  } else if (cmd == "ZERO") {
    if (isRunning) { sendStatus("ERR:BUSY"); return; }
    for (int f = 0; f < NF; f++) {
      if (!selected(f)) continue;
      int rom = max(romOf(f), ROM_MIN_DEG);
      cfg.openDeg[f] = clampAngle(round(fg[f].cmd));
      setRom(f, rom);
      cfg.potDeg1[f] = cfg.openDeg[f];
      cfg.potAdc1[f] = readPot(f);
    }
    saveConfig();
    sendStatus("POS:0");
    sendConfig();
  } else if (cmd == "SETMAX") {
    if (isRunning) { sendStatus("ERR:BUSY"); return; }
    for (int f = 0; f < NF; f++) {
      if (selected(f) && (round(fg[f].cmd) - cfg.openDeg[f]) * cfg.dir[f] < ROM_MIN_DEG) {
        sendStatus("ERR:ROM_TOO_SMALL");
        return;
      }
    }
    bool potBad = false;
    for (int f = 0; f < NF; f++) {
      if (!selected(f)) continue;
      cfg.closeDeg[f] = clampAngle(round(fg[f].cmd));
      cfg.potDeg2[f] = cfg.closeDeg[f];
      cfg.potAdc2[f] = readPot(f);
#if USE_POT_FEEDBACK
      if (!potOk(f)) potBad = true;
#endif
    }
    saveConfig();
    sendConfig();
    sendSense();
    if (potBad) sendStatus("ERR:POT");
  } else if (cmd == "FLIP") {
    if (isRunning) { sendStatus("ERR:BUSY"); return; }
    for (int f = 0; f < NF; f++) {
      if (!selected(f)) continue;
      int rom = max(romOf(f), ROM_MIN_DEG);
      cfg.dir[f] = -cfg.dir[f];
      setRom(f, rom);
    }
    saveConfig();
    sendConfig();
  } else if (cmd == "SYNC") {
    sendConfig();
    sendSense();
    sendState();
  } else {
    sendStatus("ERR:UNKNOWN_CMD");
  }
}

const char *modeName(Mode m) {
  switch (m) {
    case MODE_WAVE: return "WAVE";
    case MODE_OPP: return "OPP";
    default: return "GRIP";
  }
}

String maskStr(uint8_t m) {
  String s;
  for (int f = 0; f < NF; f++) s += (m >> f & 1) ? '1' : '0';
  return s;
}

void sendConfig() {
  int rom = 0;
  for (int f = 0; f < NF; f++) if (selected(f)) rom = max(rom, romOf(f));
  sendStatus("CFG:" + String(rom) + "," + String(targetRounds) + "," +
             String(speedPercent) + "," + String(holdMs) + "," +
             modeName(mode) + "," + maskStr(fingerMask));
  sendFingerConfig();
}

void sendFingerConfig() {
  String s = "FCFG:";
  for (int f = 0; f < NF; f++) s += String(romOf(f)) + ",";
  sendStatus(s + String(selFinger));
}

void sendSense() {
  sendStatus("SENSE:" + maskStr(potMask()) + "," + maskStr(curMask()) + "," + String(cfg.stallMa));
}

void sendState() {
  sendStatus("STATE:" + String(phaseName(phase)) + "," + String(roundsCompleted) +
             "," + String(isPaused ? 1 : 0));
}

const char *phaseName(Phase p) {
  switch (p) {
    case GRIP: return "GRIP";
    case HOLD: return "HOLD";
    case RELEASE: return "RELEASE";
    case TWIST_OUT:
    case TWIST_IN: return "TWIST";
    case RETURNING: return "RETURNING";
    default: return "IDLE";
  }
}

// =========================================================
// ควบคุมการฝึก
// =========================================================
// ความเร็วเป็นองศา/วินาที คิดจากระยะกำของแต่ละนิ้ว ทุกนิ้วจึงกำถึงสุดพร้อมกัน
float gripRate(int f) {
  float secs = GRIP_SEC_SLOW - (GRIP_SEC_SLOW - GRIP_SEC_FAST) * speedPercent / 100.0;
  return max(romOf(f), 1) / secs;
}
float releaseRate(int f) { return gripRate(f) * 0.7; }   // แบมือนุ่มกว่ากำ

// สร้างลำดับจังหวะของหนึ่งรอบ ตามโหมดและนิ้วที่เลือก
void buildSteps() {
  stepCount = 0;
  roundMask = fingerMask;
  if (mode == MODE_GRIP) {
    steps[stepCount++] = fingerMask;
  } else if (mode == MODE_WAVE) {
    for (int f = 0; f < NF; f++) if (fingerMask >> f & 1) steps[stepCount++] = 1 << f;
  } else {   // MODE_OPP: นิ้วโป้งกำคู่กับนิ้วอื่นทีละนิ้ว
    roundMask |= 1;
    for (int f = 1; f < NF; f++) if (fingerMask >> f & 1) steps[stepCount++] = 1 | (1 << f);
    if (stepCount == 0) steps[stepCount++] = 1;
  }
}

void startTraining() {
  if (fingerMask == 0) { sendStatus("ERR:NO_FINGER"); return; }
  isRunning = true;
  isPaused = false;
  roundsCompleted = 0;
  sendStatus("STARTED");
  beginRound();
}

void pauseTraining() {
  if (!isRunning || isPaused || phase == RETURNING) return;
  isPaused = true;
  sendStatus("PAUSED");
  beginReturn(true);
}

void resumeTraining() {
  if (!isRunning || !isPaused) return;
  if (phase == RETURNING) { sendStatus("ERR:BUSY"); return; }
  isPaused = false;
  sendStatus("RESUMED");
  beginRound();   // เริ่มรอบนี้ใหม่ เพราะตอนพักมือถูกคืนไปแบมือแล้ว
}

void stopTraining() {
  isPaused = false;
  isRunning = true;   // ให้ updateMotion ทำงานต่อจนกลับตำแหน่งเสร็จ
  beginReturn(false);
}

void emergencyStop() {
  freezeAll();        // Servo ค้างไว้ที่ตำแหน่งปัจจุบัน ไม่กระตุก
  isRunning = false;
  isPaused = false;
  phase = IDLE;
  MiniR4.Buzzer.Tone(1800, 400);
  sendStatus("ESTOP");
}

void fault(const char *reason) {
  freezeAll();
  isRunning = false;
  isPaused = false;
  phase = IDLE;
  MiniR4.Buzzer.Tone(600, 600);
  sendStatus(String("ERR:") + reason);
}

void jog(int sign) {
  if (isRunning) { sendStatus("ERR:BUSY"); return; }
  for (int f = 0; f < NF; f++) {
    if (!selected(f)) continue;
    fg[f].moving = false;
    fg[f].cmd = clampAngle(round(fg[f].cmd) + sign * cfg.dir[f] * JOG_STEP_DEG);
    servoWrite(f, round(fg[f].cmd));
  }
  activeMask = selFinger ? 1 << (selFinger - 1) : 0b1111;
  sendStatus("POS:" + String(posPercent()));
}

// ---------- เริ่มแต่ละจังหวะ ----------
void enterPhase(Phase p) {
  phase = p;
  phaseStart = millis();
}

void beginRound() {
  buildSteps();
  stepIdx = 0;
  for (int f = 0; f < NF; f++) fg[f].peakPct = 0;
  beginGrip();
}

void beginGrip() {
  uint8_t m = steps[stepIdx];
  activeMask = m;
  int n = 0;
  for (int f = 0; f < NF; f++) {
    if (m >> f & 1) setTarget(f, cfg.closeDeg[f], gripRate(f), STAGGER_MS * n++);
  }
  enterPhase(GRIP);
  sendStatus("PHASE:GRIP");
  sendStatus("ACT:" + maskStr(m));
}

void beginRelease() {
  for (int f = 0; f < NF; f++) {
    if (steps[stepIdx] >> f & 1) setTarget(f, cfg.openDeg[f], releaseRate(f), 0);
  }
  enterPhase(RELEASE);
  sendStatus("PHASE:RELEASE");
}

void beginTwist(bool out) {
  activeMask = roundMask;
  for (int f = 0; f < NF; f++) {
    if (!(roundMask >> f & 1)) continue;
    float a = out ? cfg.openDeg[f] - cfg.dir[f] * TWIST_DEG : cfg.openDeg[f];
    setTarget(f, a, releaseRate(f), 0);
  }
  enterPhase(out ? TWIST_OUT : TWIST_IN);
  if (out) sendStatus("PHASE:TWIST");
}

void beginReturn(bool thenPause) {
  afterReturnPaused = thenPause;
  sendStatus("RETURNING");
  activeMask = 0b1111;
  int n = 0;
  for (int f = 0; f < NF; f++) setTarget(f, cfg.openDeg[f], releaseRate(f), STAGGER_MS * n++);
  enterPhase(RETURNING);
}

// =========================================================
// ลูปควบคุม (ไม่บล็อก)
// =========================================================
void updateMotion() {
  unsigned long now = millis();
  if (now - lastTick < TICK_MS) return;
  float dt = (now - lastTick) / 1000.0;
  lastTick = now;
  if (dt > 0.1) dt = 0.1;
  tickFingers(dt);

  switch (phase) {
    case IDLE:
      break;

    case GRIP:
      trackPeak();
      if (maskDone(steps[stepIdx])) {
        enterPhase(HOLD);
        sendStatus("PHASE:HOLD");
      }
      break;

    case HOLD:
      trackPeak();
      if (millis() - phaseStart >= holdMs) beginRelease();
      break;

    case RELEASE:
      if (maskDone(steps[stepIdx])) {
        if (stepIdx + 1 < stepCount) { stepIdx++; beginGrip(); }
        else beginTwist(true);
      }
      break;

    case TWIST_OUT:
      if (maskDone(roundMask)) beginTwist(false);
      break;

    case TWIST_IN:
      if (maskDone(roundMask)) finishRound();
      break;

    case RETURNING:
      if (maskDone(0b1111)) {
        phase = IDLE;
        sendStatus("RETURNED");
        if (!afterReturnPaused) {
          isRunning = false;
          sendStatus("STOPPED");
        }
      }
      break;
  }
}

void trackPeak() {
  for (int f = 0; f < NF; f++) {
    if (steps[stepIdx] >> f & 1) fg[f].peakPct = max(fg[f].peakPct, constrain(fingerPct(f), 0, 100));
  }
}

void finishRound() {
  roundsCompleted++;
  sendStatus("ROUND:" + String(roundsCompleted));

  long sum = 0;
  int n = 0;
  String s = "FPEAK:";
  for (int f = 0; f < NF; f++) {
    if (f) s += ",";
    if (roundMask >> f & 1) {
      s += String(fg[f].peakPct);
      sum += fg[f].peakPct;
      n++;
    } else {
      s += "-";
    }
  }
  sendStatus(s);
  sendStatus("PEAK:" + String(n ? sum / n : 0));

  if (roundsCompleted >= targetRounds) {
    isRunning = false;
    phase = IDLE;
    MiniR4.Buzzer.Tone(1200, 300);
    sendStatus("DONE");
    return;
  }
  beginRound();
}

// =========================================================
// ส่งข้อมูลกลับ
// =========================================================
// ตำแหน่งเฉลี่ยของนิ้วที่กำลังขยับ (ให้วงแหวนในแอปเดิมแสดงผลได้)
int posPercent() {
  long sum = 0;
  int n = 0;
  for (int f = 0; f < NF; f++) {
    if (activeMask >> f & 1) { sum += fingerPct(f); n++; }
  }
  return n ? sum / n : 0;
}

void sendTelemetry() {
  unsigned long now = millis();
  if (BLE.connected()) {
    if (phase != IDLE && now - lastPosSent >= 120) {
      int pct = posPercent();
      if (abs(pct - lastSentPct) >= 2) {
        lastSentPct = pct;
        lastPosSent = now;
        txChar.writeValue(("POS:" + String(pct)).c_str());
      }
    }

    if (now - lastFposSent >= 200) {
      bool changed = false;
      int p[NF];
      for (int f = 0; f < NF; f++) {
        p[f] = fingerPct(f);
        if (abs(p[f] - lastFpos[f]) >= 2) changed = true;
      }
      if (changed) {
        lastFposSent = now;
        String s = "FPOS:";
        for (int f = 0; f < NF; f++) { lastFpos[f] = p[f]; s += (f ? "," : "") + String(p[f]); }
        txChar.writeValue(s.c_str());
      }
    }

#if USE_CURRENT_SENSE
    if (now - lastCurSent >= 500) {
      bool changed = false;
      for (int f = 0; f < NF; f++) if (abs((int)fg[f].ma - lastFcur[f]) >= 20) changed = true;
      if (changed) {
        lastCurSent = now;
        String s = "FCUR:";
        for (int f = 0; f < NF; f++) { lastFcur[f] = (int)fg[f].ma; s += (f ? "," : "") + String(lastFcur[f]); }
        txChar.writeValue(s.c_str());
      }
    }
#endif
  }

  if (now - lastBattSent >= 10000) {
    lastBattSent = now;
    float v = MiniR4.PWR.getBattVoltage();
    if (v > 0) sendStatus("BATT:" + String(v, 1));
  }
}
