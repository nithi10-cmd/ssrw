/*
  ==========================================================
   ถุงมือกายภาพบำบัด v5 - Arduino UNO R4 WiFi + บอร์ด K-Matrix Mini R4
   ควบคุมผ่าน BLE (ArduinoBLE) + มอเตอร์ DC มี encoder 4 ตัว (ดึงแยกทีละนิ้วได้)
  ==========================================================

  สิ่งที่เปลี่ยนจาก v4
    - เปลี่ยน Servo (หมุนได้แค่ 180° ดึงสายได้สั้น กำมือไม่สุด) เป็นมอเตอร์ล้อหุ่นยนต์มี encoder
      แบบเดียวกับ v3 แต่ใช้ 4 ตัว ต่อที่พอร์ต MOTOR + ENCODER ของแต่ละมุมบอร์ด
         M1 = นิ้วโป้ง, M2 = นิ้วชี้, M3 = นิ้วกลาง, M4 = นิ้วนาง + นิ้วก้อย
      (ถ้าเสียบสลับช่อง แก้ที่ MOTOR_PORT ด้านล่าง)
    - มอเตอร์พันสายได้หลายรอบ ระยะกำตั้งได้ถึง 2880° ของมอเตอร์ (8 รอบ) ตั้งจำนวนรอบจากแอปได้โดยตรง (ROM:องศา)
    - ตรวจจับนิ้วเกร็งด้วย encoder แบบ v3 (ไม่ต้องใช้ pot หรือ INA219) แยกทีละนิ้ว
      สายดึงต่ออนุกรมกับสปริง: เมื่อนิ้วต้าน สปริงจะตึงจนมอเตอร์หมุนต่อไม่ไหว ระบบจะหยุดดึงนิ้วนั้น
      แรงดึงสูงสุดจึงถูกจำกัดด้วยกำลังมอเตอร์ (ปรับด้วยความเร็วในแอป)
    - โปรโตคอลเหมือน v4 ทุกอย่าง เว็บแอปเดิมใช้ได้ทันที (และคำสั่งของ v3 ก็ยังใช้ได้)

  ไลบรารีที่ต้องมี: ArduinoBLE, MatrixMiniR4

  ⚠️ ใช้งานครั้งแรก ให้ทดสอบโดยไม่สวมมือก่อนเสมอ
    - เปิดเครื่องตอนแบมือ / สายหย่อน เพราะตอนเปิดเครื่อง ตำแหน่งปัจจุบันของทุกนิ้วจะถือเป็น "แบมือ"
    - ในแอป: เลือกนิ้ว -> แบมือสุดแล้วกด "ตั้งเป็นแบมือ" (ZERO)
      -> กด "กำเพิ่ม" ทีละครั้งจนกำได้ระดับที่ผู้ป่วยรับได้ -> "ตั้งเป็นกำสุด" (SETMAX)
      ระยะกำของแต่ละนิ้วถูกจำไว้ใน EEPROM
    - ถ้ากด "กำเพิ่ม" แล้วนิ้วคลาย (สายพันกลับทาง) ให้กด "สลับทิศของนิ้วที่เลือก" (FLIP) แล้ว ZERO ใหม่
    - ทิศการนับของ encoder ปรับให้เองอัตโนมัติ (ตอนกด "กำเพิ่ม"/"คลาย" หรือตอนฝึก) แล้วจำไว้ใน EEPROM
      ไม่ต้องแก้โค้ด — ถ้ายังขึ้น ERR:DIRECTION อีก แปลว่า encoder นับไม่สม่ำเสมอ ให้ตรวจสาย encoder

  โปรโตคอลคำสั่ง (ข้อความ ASCII ผ่าน BLE หรือ Serial 115200 ก็ได้)
    START, PAUSE, RESUME, STOP, ESTOP, TGT:10, SPD:50, HOLD:2000,
    ROM:360 (ระยะกำเป็นองศามอเตอร์ ของนิ้วที่เลือก), JOG:+ / JOG:-, ZERO, SETMAX, SYNC,
    MODE:GRIP|WAVE|OPP, FMASK:1111 (โป้ง,ชี้,กลาง,นาง+ก้อย), SEL:0-4 (0 = ทุกนิ้ว), FLIP

  ข้อความที่ส่งกลับ
    STARTED, PHASE:GRIP|HOLD|RELEASE|TWIST, ROUND:n, PEAK:pct, POS:pct, STALL:pct, DONE,
    RETURNING, RETURNED, PAUSED, RESUMED, STOPPED, ESTOP, BATT:7.4,
    ERR:<เหตุผล>[,นิ้ว,พอร์ต] เช่น ERR:NO_MOTION,1,1 = มอเตอร์นิ้วโป้ง (M1) ไม่หมุน หรือ encoder ไม่นับ,
    CFG:rom,target,speed,holdMs,mode,mask, STATE:phase,round,paused,
    ACT:0100, FPOS:a,b,c,d, FPEAK:a,b,c,d ('-' = ไม่ได้ร่วมฝึก), FSTALL:finger,pct,
    FCFG:rom1,rom2,rom3,rom4,sel

  UUID (Nordic UART Service) เหมือน v3/v4
    Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
    RX     : 6E400002-B5A3-F393-E0A9-E50E24DCCA9E  (เขียนคำสั่งเข้ามา)
    TX     : 6E400003-B5A3-F393-E0A9-E50E24DCCA9E  (notify สถานะ)
*/

#include <ArduinoBLE.h>
#include <MatrixMiniR4.h>
#include <EEPROM.h>

// ---------- แบตเตอรี่ ----------
const uint8_t BATT_CELLS = 2;        // จำนวนก้อนถ่าน Li-ion ที่ต่ออนุกรม (2 = 7.4V, 3 = 11.1V)

// ---------- มอเตอร์ ----------
const int NF = 4;                                  // จำนวนมอเตอร์ / นิ้ว
const uint8_t MOTOR_PORT[NF] = {1, 2, 3, 4};       // พอร์ต M ของ โป้ง, ชี้, กลาง, นาง+ก้อย
const int8_t  ENC_SIGN_DEFAULT = -1;               // ทิศ encoder เริ่มต้น (ระบบปรับให้เองอัตโนมัติถ้าผิด)
// ทิศหมุนตอนกำ ของ โป้ง, ชี้, กลาง, นาง+ก้อย (+1 / -1) ถ้านิ้วไหนหมุนคนละทางกับนิ้วอื่น ให้สลับเครื่องหมายนิ้วนั้น
// (หรือกด "สลับทิศของนิ้วที่เลือก" ในแอปก็ได้ ไม่ต้องอัปโหลดใหม่ ค่าจะถูกจำไว้ในบอร์ด)
const int8_t  CLOSE_SIGN_DEFAULT[NF] = {1, 1, 1, 1};
const uint16_t MOTOR_PPR     = 545;                // ค่าเดียวกับ v3
const uint16_t MOTOR_RPM     = 200;

// ---------- ค่าที่ปรับได้ (ค่าเริ่มต้นเหมือน v3) ----------
const int  POS_TOLERANCE   = 8;      // ถือว่าถึงเป้าหมายเมื่อห่างไม่เกินกี่องศา
const int  SLOWDOWN_DEG    = 90;     // เริ่มลดความเร็วเมื่อเข้าใกล้เป้าหมายกี่องศา (กว้างกว่า v3 เพื่อไม่ให้เลยเป้าเวลาบอร์ดตอบช้า)
const int  MIN_POWER       = 25;     // กำลังต่ำสุดที่มอเตอร์ยังหมุนได้
const int  TWIST_POWER     = 35;     // กำลังตอนคลายเอ็น
const int  JOG_POWER       = 30;     // กำลังตอนคาลิเบรต
const unsigned long JOG_MS        = 150;
const unsigned long STALL_MS      = 600;    // ไม่ขยับเกินเวลานี้ = มีแรงต้าน
const int  STALL_MIN_DEG          = 4;      // ต้องขยับอย่างน้อยกี่องศาจึงนับว่ายังเคลื่อนที่
const unsigned long MOVE_TIMEOUT  = 6000;   // เวลาพื้นฐานของแต่ละจังหวะ บวกเพิ่มตามระยะกำ (ดู moveTimeout()) เกินแล้ว = ผิดปกติ
const unsigned long TIMEOUT_MS_PER_DEG = 12; // เผื่อเวลาเพิ่มต่อองศาของระยะกำ (มอเตอร์หมุนช้าสุด ~180 องศา/วินาทีเมื่อมีภาระ)
const int  OVERTRAVEL_DEG         = 90;     // หมุนเกินระยะกำสูงสุดกี่องศาแล้วต้องตัด
const int  WRONG_DIR_DEG          = 45;     // หมุนสวนทางเป้าหมายกี่องศาแล้วต้องตัด
const int  TWIST_DEG              = 60;     // ระยะหมุนเลย 0 ออกไปเพื่อคลายเอ็น
const unsigned long STAGGER_MS    = 150;    // ให้แต่ละมอเตอร์ออกตัวห่างกัน ลดไฟกระชาก (บอร์ดรีเซ็ตตอนเริ่มกำ ให้เพิ่มค่านี้)
// กำลังมอเตอร์ตอนกำ ที่ความเร็ว 0% และ 100% (v3 ใช้ 35..85 เพราะมอเตอร์ตัวเดียวดึงทั้งมือ
// v5 แยกนิ้วละตัว ภาระน้อยกว่ามาก จึงลดกำลังสูงสุดลง = แรงดึงนิ้วสูงสุดน้อยลง ปลอดภัยกว่า)
const int  GRIP_POWER_MIN = 30;
const int  GRIP_POWER_MAX = 70;
const int  ROM_MIN = 30, ROM_MAX = 2880, ROM_DEFAULT = 360;
// ค่า encoder ที่เปลี่ยนเร็วเกินกว่ามอเตอร์หมุนได้จริง = อ่านผิดพลาด (คิดตามเวลาที่ผ่านไปจริง เพราะ
// ถ้าการสื่อสารในบอร์ดช้า ช่วงห่างระหว่างการอ่านอาจนานกว่า 20 ms มาก)
const long MOTOR_MAX_DEG_PER_S = MOTOR_RPM * 6L;   // 200 RPM = 1200 องศา/วินาที
const unsigned long CMD_REFRESH_MS = 250;           // ส่งคำสั่งมอเตอร์ซ้ำเป็นระยะ เผื่อคำสั่งหายระหว่างทาง
const int  POWER_UNKNOWN = -32768;                  // คำสั่งล่าสุดส่งไม่สำเร็จ ต้องส่งใหม่

// ---------- ค่าที่ปรับจากเว็บได้ ----------
int  targetRounds  = 10;
int  speedPercent  = 50;
unsigned long holdMs = 2000;
enum Mode { MODE_GRIP, MODE_WAVE, MODE_OPP };
Mode mode = MODE_GRIP;
uint8_t fingerMask = 0b1111;   // bit0 = โป้ง ... bit3 = นาง+ก้อย
int  selFinger = 0;            // 0 = ทุกนิ้ว, 1-4 = นิ้วที่เลือกสำหรับคาลิเบรต

// ---------- ค่าคาลิเบรต (เก็บใน EEPROM) ----------
const uint32_t CFG_MAGIC = 0x5247410A;   // เปลี่ยนแล้วค่าที่บันทึกไว้จะถูกล้าง
struct Cfg {
  uint32_t magic;
  int16_t  romDeg[NF];      // ระยะกำ (องศามอเตอร์ จาก 0 = แบมือ)
  int8_t   closeSign[NF];   // ทิศหมุนที่พันสาย (+1 / -1) เปลี่ยนด้วย FLIP
  int8_t   encSign[NF];     // ทิศการนับของ encoder เทียบกับมอเตอร์ (+1 / -1) ปรับเองอัตโนมัติ
};
Cfg cfg;

// ---------- BLE ----------
BLEService uartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic rxChar("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite | BLEWriteWithoutResponse, 40);
BLECharacteristic txChar("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, 40);

// ---------- สถานะ ----------
enum Phase { IDLE, GRIP, HOLD, RELEASE, TWIST_OUT, TWIST_IN, RETURNING, JOG };
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
  int  pos;                // ตำแหน่งจาก encoder (องศามอเตอร์, 0 = แบมือ, บวก = กำ)
  int  target;
  int  dir;                // ทิศของการเคลื่อนที่นี้ (+1 กำ, -1 แบ, 0 = อยู่ที่เป้าแล้ว)
  int  power;              // กำลังสูงสุดของการเคลื่อนที่นี้
  int  sentPower;          // กำลังล่าสุดที่สั่งมอเตอร์ (ส่งเฉพาะตอนเปลี่ยน)
  unsigned long startAt;
  bool moving;
  bool stalled;            // เจอแรงต้านในจังหวะนี้แล้ว
  bool encFixed;           // ปรับทิศ encoder ไปแล้วในการเคลื่อนที่นี้ (ปรับได้ครั้งเดียว)
  bool encVerified;        // ยืนยันแล้วว่าทิศ encoder ถูก (เคยหมุนไปตามทิศที่สั่งได้ไกลพอ) จะไม่สลับอีก
  int  goodTravel;         // ระยะที่หมุนไปตามทิศที่สั่งในการเคลื่อนที่นี้
  int  jogStartPos;
  uint8_t jumpCount;       // จำนวนครั้งติดกันที่ค่า encoder กระโดดผิดปกติ
  unsigned long readAt;    // เวลาที่อ่าน encoder สำเร็จล่าสุด
  unsigned long cmdAt;     // เวลาที่ส่งคำสั่งมอเตอร์ล่าสุด
  int  startPos;           // ตำแหน่งตอนเริ่มเคลื่อนที่ (ใช้ตรวจว่ามอเตอร์ไม่หมุนเลย)
  int  progressPos;
  unsigned long progressTime;
  int  peakDeg;            // ระยะกำสูงสุดในรอบนี้
};
Finger fg[NF];

unsigned long lastPosRead = 0;
unsigned long lastPosSent = 0, lastFposSent = 0, lastBattSent = 0;
unsigned long lastBtnRead = 0;
bool lastBtnUp = false;
int  lastSentPct = -999;
int  lastFpos[NF] = {-999, -999, -999, -999};

String serialLine;

// =========================================================
void setup() {
  Serial.begin(115200);

  MiniR4.begin();
  MiniR4.PWR.setBattCell(BATT_CELLS);

  loadConfig();
  for (int f = 0; f < NF; f++) {
    motorInit(f);
    fg[f].pos = 0;
    fg[f].sentPower = POWER_UNKNOWN;   // บังคับให้ส่งคำสั่งเบรกครั้งแรก
    fg[f].readAt = millis();
    fg[f].moving = false;
    motorBrake(f);
  }

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
  readPositions();
  updateMotion();
  refreshBrakes();
  sendTelemetry();
}

// =========================================================
// ค่าคาลิเบรต
// =========================================================
void loadConfig() {
  EEPROM.get(0, cfg);
  bool ok = cfg.magic == CFG_MAGIC;
  for (int f = 0; ok && f < NF; f++) {
    ok = cfg.romDeg[f] >= ROM_MIN && cfg.romDeg[f] <= ROM_MAX &&
         (cfg.closeSign[f] == 1 || cfg.closeSign[f] == -1) &&
         (cfg.encSign[f] == 1 || cfg.encSign[f] == -1);
  }
  if (!ok) {
    Serial.println("ยังไม่มีค่าคาลิเบรตใน EEPROM ใช้ค่าเริ่มต้น");
    cfg.magic = CFG_MAGIC;
    for (int f = 0; f < NF; f++) {
      cfg.romDeg[f] = ROM_DEFAULT;
      cfg.closeSign[f] = CLOSE_SIGN_DEFAULT[f];
      cfg.encSign[f] = ENC_SIGN_DEFAULT;
    }
  }
}

void saveConfig() {
  EEPROM.put(0, cfg);
}

// =========================================================
// มอเตอร์ (บอร์ดมี M1-M4 เป็นคนละ class จึงเลือกด้วย switch)
// =========================================================
void motorInit(int f) {
  switch (MOTOR_PORT[f]) {
    case 1: MiniR4.M1.setPPR_RPM(MOTOR_PPR, MOTOR_RPM); MiniR4.M1.setReverse(false); MiniR4.M1.resetCounter(); break;
    case 2: MiniR4.M2.setPPR_RPM(MOTOR_PPR, MOTOR_RPM); MiniR4.M2.setReverse(false); MiniR4.M2.resetCounter(); break;
    case 3: MiniR4.M3.setPPR_RPM(MOTOR_PPR, MOTOR_RPM); MiniR4.M3.setReverse(false); MiniR4.M3.resetCounter(); break;
    case 4: MiniR4.M4.setPPR_RPM(MOTOR_PPR, MOTOR_RPM); MiniR4.M4.setReverse(false); MiniR4.M4.resetCounter(); break;
  }
}

// power บวก = หมุนทิศกำ
// ส่งคำสั่งเฉพาะตอนค่าเปลี่ยน หรือครบรอบ CMD_REFRESH_MS ถ้าบอร์ดตอบว่าไม่สำเร็จ จะส่งใหม่รอบถัดไป
void motorPower(int f, int power) {
  if (power == fg[f].sentPower && millis() - fg[f].cmdAt < CMD_REFRESH_MS) return;
  int p = power * cfg.closeSign[f];
  bool ok = false;
  switch (MOTOR_PORT[f]) {
    case 1: ok = MiniR4.M1.setPower(p); break;
    case 2: ok = MiniR4.M2.setPower(p); break;
    case 3: ok = MiniR4.M3.setPower(p); break;
    case 4: ok = MiniR4.M4.setPower(p); break;
  }
  fg[f].sentPower = ok ? power : POWER_UNKNOWN;
  fg[f].cmdAt = millis();
}

bool sendBrake(int f) {
  switch (MOTOR_PORT[f]) {
    case 1: return MiniR4.M1.setPower(0) && MiniR4.M1.setBrake(true);
    case 2: return MiniR4.M2.setPower(0) && MiniR4.M2.setBrake(true);
    case 3: return MiniR4.M3.setPower(0) && MiniR4.M3.setBrake(true);
    case 4: return MiniR4.M4.setPower(0) && MiniR4.M4.setBrake(true);
  }
  return false;
}

// เบรก: สำคัญต่อความปลอดภัย จึงลองซ้ำได้สูงสุด 3 ครั้งถ้าบอร์ดตอบว่าไม่สำเร็จ
void motorBrake(int f) {
  if (fg[f].sentPower == 0 && millis() - fg[f].cmdAt < CMD_REFRESH_MS) return;
  bool ok = false;
  for (int i = 0; i < 3 && !ok; i++) ok = sendBrake(f);
  fg[f].sentPower = ok ? 0 : POWER_UNKNOWN;
  fg[f].cmdAt = millis();
}

// นิ้วที่ไม่ได้เคลื่อนที่ ให้ส่งคำสั่งเบรกซ้ำเป็นระยะ (กันกรณีคำสั่งเบรกหายแล้วมอเตอร์หมุนค้าง)
void refreshBrakes() {
  if (phase == JOG) return;
  for (int f = 0; f < NF; f++) if (!fg[f].moving) motorBrake(f);
}

// อ่าน encoder (องศา, บวก = ทิศกำ) คืน false ถ้าอ่านไม่สำเร็จ
// ไม่ใช้ MiniR4.Mx.getDegrees() เพราะถ้าการสื่อสารในบอร์ดพลาด ฟังก์ชันนั้นคืนค่า 0 แทนที่จะแจ้ง error
// ทำให้ตำแหน่งกระโดดกลับไป 0 ชั่วขณะ แล้วระบบเข้าใจผิดว่ามอเตอร์หมุนสวนทาง
bool motorDegrees(int f, int &out) {
  int32_t d = 0;
  if (mmL.GetEncoderDegrees(MOTOR_PORT[f], d) != MMLower::RESULT::OK) return false;
  out = cfg.encSign[f] * cfg.closeSign[f] * d;
  return true;
}

void motorResetCounter(int f) {
  switch (MOTOR_PORT[f]) {
    case 1: MiniR4.M1.resetCounter(); break;
    case 2: MiniR4.M2.resetCounter(); break;
    case 3: MiniR4.M3.resetCounter(); break;
    case 4: MiniR4.M4.resetCounter(); break;
  }
  fg[f].pos = 0;
}

void brakeAll() {
  for (int f = 0; f < NF; f++) {
    fg[f].moving = false;
    motorBrake(f);
  }
}

// =========================================================
// BLE / Serial
// =========================================================
void onConnected(BLEDevice central) {
  Serial.print("เชื่อมต่อกับ: ");
  Serial.println(central.address());
  for (int f = 0; f < NF; f++) lastFpos[f] = -999;   // ให้ส่งตำแหน่งนิ้วไปให้แอปใหม่ทันที
}

void onDisconnected(BLEDevice central) {
  Serial.println("อุปกรณ์ตัดการเชื่อมต่อ — คืนมือสู่ตำแหน่งแบมือ");
  if (isRunning || phase == JOG) stopTraining();
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
  } else if (cmd.startsWith("ROM:")) {
    if (isRunning) { sendStatus("ERR:BUSY"); return; }
    int rom = constrain((int)cmd.substring(4).toInt(), ROM_MIN, ROM_MAX);
    for (int f = 0; f < NF; f++) if (selected(f)) cfg.romDeg[f] = rom;
    saveConfig();
    sendConfig();
  } else if (cmd == "JOG:+" || cmd == "JOG:-") {
    jog(cmd.endsWith("+") ? 1 : -1);
  } else if (cmd == "ZERO") {
    if (isRunning) { sendStatus("ERR:BUSY"); return; }
    for (int f = 0; f < NF; f++) if (selected(f)) motorResetCounter(f);
    activeMask = selFinger ? 1 << (selFinger - 1) : 0b1111;
    sendStatus("POS:0");
  } else if (cmd == "SETMAX") {
    if (isRunning) { sendStatus("ERR:BUSY"); return; }
    for (int f = 0; f < NF; f++) {
      if (selected(f) && fg[f].pos < ROM_MIN) { sendStatus("ERR:ROM_TOO_SMALL"); return; }
    }
    for (int f = 0; f < NF; f++) if (selected(f)) cfg.romDeg[f] = constrain(fg[f].pos, ROM_MIN, ROM_MAX);
    saveConfig();
    sendConfig();
  } else if (cmd == "FLIP") {
    if (isRunning) { sendStatus("ERR:BUSY"); return; }
    for (int f = 0; f < NF; f++) {
      if (!selected(f)) continue;
      motorBrake(f);
      cfg.closeSign[f] = -cfg.closeSign[f];
      fg[f].pos = -fg[f].pos;
    }
    saveConfig();
    sendConfig();
  } else if (cmd == "SYNC") {
    sendConfig();
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
  for (int f = 0; f < NF; f++) if (selected(f)) rom = max(rom, (int)cfg.romDeg[f]);
  sendStatus("CFG:" + String(rom) + "," + String(targetRounds) + "," +
             String(speedPercent) + "," + String(holdMs) + "," +
             modeName(mode) + "," + maskStr(fingerMask));
  sendFingerConfig();
}

void sendFingerConfig() {
  String s = "FCFG:";
  for (int f = 0; f < NF; f++) s += String(cfg.romDeg[f]) + ",";
  sendStatus(s + String(selFinger));
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
    case JOG: return "JOG";
    default: return "IDLE";
  }
}

// =========================================================
// ควบคุมการฝึก
// =========================================================
// กำลังมอเตอร์ตามความเร็วที่ตั้ง (ระยะกำเท่าเดิม แค่ไปถึงเร็วขึ้นหรือช้าลง)
// ต่อสปริงอนุกรม: กำลังที่ตั้งไว้ = แรงดึงสูงสุดที่นิ้วจะได้รับ
int gripPower()    { return GRIP_POWER_MIN + (GRIP_POWER_MAX - GRIP_POWER_MIN) * speedPercent / 100; }
int releasePower() { return max(MIN_POWER, gripPower() * 7 / 10); } // แบมือนุ่มกว่ากำ

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
  if (phase == JOG) brakeAll();
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
  brakeAll();
  isRunning = false;
  isPaused = false;
  phase = IDLE;
  MiniR4.Buzzer.Tone(1800, 400);
  sendStatus("ESTOP");
}

void fault(const String &reason) {
  brakeAll();
  isRunning = false;
  isPaused = false;
  phase = IDLE;
  MiniR4.Buzzer.Tone(600, 600);
  sendStatus("ERR:" + reason);
}

int jogDir = 0;

// encoder นับสวนทางกับที่สั่งมอเตอร์: สลับทิศการนับของนิ้วนั้น จำไว้ แล้วแจ้งแอป
void fixEncoder(int f) {
  cfg.encSign[f] = -cfg.encSign[f];
  saveConfig();
  fg[f].pos = -fg[f].pos;
  sendStatus("ENCFIX:" + String(f + 1));
}

void jog(int dir) {
  if (isRunning) { sendStatus("ERR:BUSY"); return; }
  activeMask = selFinger ? 1 << (selFinger - 1) : 0b1111;
  for (int f = 0; f < NF; f++) {
    fg[f].moving = false;
    if (selected(f)) readFingerFresh(f);
    fg[f].jogStartPos = fg[f].pos;
    if (selected(f)) motorPower(f, dir * JOG_POWER);
  }
  jogDir = dir;
  phase = JOG;
  phaseStart = millis();
}

// ---------- เริ่มแต่ละจังหวะ ----------
void enterPhase(Phase p) {
  phase = p;
  phaseStart = millis();
}

void startMove(int f, int target, int power, unsigned long delayMs) {
  Finger &F = fg[f];
  readFingerFresh(f);
  F.target = target;
  F.dir = abs(target - F.pos) <= POS_TOLERANCE ? 0 : (target > F.pos ? 1 : -1);
  F.power = power;
  F.startAt = millis() + delayMs;
  F.moving = true;
  F.stalled = false;
  F.startPos = F.pos;
  F.encFixed = false;
  F.goodTravel = 0;
  F.progressPos = F.pos;
  F.progressTime = F.startAt;
}

void beginRound() {
  buildSteps();
  stepIdx = 0;
  for (int f = 0; f < NF; f++) fg[f].peakDeg = 0;
  beginGrip();
}

void beginGrip() {
  uint8_t m = steps[stepIdx];
  activeMask = m;
  int n = 0;
  for (int f = 0; f < NF; f++) {
    if (m >> f & 1) startMove(f, cfg.romDeg[f], gripPower(), STAGGER_MS * n++);
  }
  enterPhase(GRIP);
  sendStatus("PHASE:GRIP");
  sendStatus("ACT:" + maskStr(m));
}

void beginRelease() {
  for (int f = 0; f < NF; f++) {
    if (steps[stepIdx] >> f & 1) startMove(f, 0, releasePower(), 0);
  }
  enterPhase(RELEASE);
  sendStatus("PHASE:RELEASE");
}

void beginTwist(bool out) {
  activeMask = roundMask;
  for (int f = 0; f < NF; f++) {
    if (roundMask >> f & 1) startMove(f, out ? -TWIST_DEG : 0, TWIST_POWER, 0);
  }
  enterPhase(out ? TWIST_OUT : TWIST_IN);
  if (out) sendStatus("PHASE:TWIST");
}

void beginReturn(bool thenPause) {
  afterReturnPaused = thenPause;
  sendStatus("RETURNING");
  activeMask = 0b1111;
  int n = 0;
  for (int f = 0; f < NF; f++) {
    startMove(f, 0, fg[f].pos > 0 ? releasePower() : TWIST_POWER, STAGGER_MS * n++);
  }
  enterPhase(RETURNING);
}

// =========================================================
// ลูปควบคุมมอเตอร์ (ไม่บล็อก)
// =========================================================
void readPositions() {
  if (millis() - lastPosRead < 20) return;
  lastPosRead = millis();
  static uint8_t cycle = 0;
  cycle++;
  for (int f = 0; f < NF; f++) {
    // นิ้วที่อยู่นิ่ง อ่านห่างขึ้น (ทุก 100 ms) ลดภาระการสื่อสารในบอร์ด
    if (!fg[f].moving && phase != JOG && cycle % 5) continue;
    readFinger(f);
  }
}

// อ่านตำแหน่งนิ้ว f หนึ่งครั้ง (กรองค่าที่อ่านพลาด) คืน true ถ้าได้ค่าใหม่
bool readFinger(int f) {
  int p;
  if (!motorDegrees(f, p)) return false;           // อ่านพลาด: ใช้ค่าเดิมไปก่อน
  unsigned long now = millis();
  long dt = min(now - fg[f].readAt, 1000UL);
  long maxJump = 30 + MOTOR_MAX_DEG_PER_S * dt * 3 / 2000;   // เผื่อ 1.5 เท่าของความเร็วสูงสุด
  if (abs(p - fg[f].pos) > maxJump && ++fg[f].jumpCount < 3) return false;   // เร็วเกินจริง: รอยืนยัน 3 ครั้ง
  fg[f].jumpCount = 0;
  fg[f].pos = p;
  fg[f].readAt = now;
  return true;
}

// อ่านตำแหน่งล่าสุดจริงๆ ก่อนเริ่มเคลื่อนที่ (นิ้วที่อยู่นิ่งอาจอ่านไว้นานแล้ว)
void readFingerFresh(int f) {
  for (int i = 0; i < 3 && !readFinger(f); i++);
}

bool started(int f) { return (long)(millis() - fg[f].startAt) >= 0; }

bool inStep(int f) {
  return (phase == GRIP || phase == HOLD) && (steps[stepIdx] >> f & 1);
}

// ขยับนิ้ว f เข้าหาเป้าหมาย (ตรรกะเดียวกับ stepMove ของ v3)
// คืนค่า false ถ้าเกิด fault และหยุดทั้งระบบแล้ว
bool stepFinger(int f) {
  Finger &F = fg[f];
  if (!F.moving || !started(f)) return true;

  int err = F.target - F.pos;
  int dir = F.dir;
  // ถึงเป้าหมาย หรือหมุนเลยไปแล้ว (เช่นผู้ป่วยกำ/แบช่วย หรือมอเตอร์ไหลเลยตอนบอร์ดตอบช้า)
  // ใช้ทิศตอนเริ่มเคลื่อนที่ จึงไม่หมุนกลับไปกลับมารอบเป้าหมาย
  if (dir == 0 || abs(err) <= POS_TOLERANCE || (dir > 0 && F.pos >= F.target) || (dir < 0 && F.pos <= F.target)) {
    F.moving = false;
    motorBrake(f);
    return true;
  }

  // ลดความเร็วเมื่อใกล้ถึง เพื่อไม่ให้กระชากและไม่เลยเป้าหมาย
  int power = F.power;
  if (abs(err) < SLOWDOWN_DEG) power = max(MIN_POWER, F.power * abs(err) / SLOWDOWN_DEG);
  motorPower(f, dir * power);

  // ตรวจว่ามอเตอร์ยังขยับอยู่ไหม
  int moved = F.pos - F.progressPos;
  if (abs(moved) >= STALL_MIN_DEG) {
    // ขยับสวนทิศทางที่สั่งมาก = encoder กลับทิศ
    if (moved * dir < 0 && abs(moved) >= WRONG_DIR_DEG) {
      if (F.encVerified || millis() - F.startAt < 200) {
        // ทิศ encoder ยืนยันแล้ว หรือเพิ่งเริ่ม (มอเตอร์อาจยังไหลจากจังหวะก่อน): ไม่ใช่ encoder กลับทิศ
        F.progressPos = F.pos;
        F.progressTime = millis();
        return true;
      }
      if (F.encFixed) {           // สลับแล้วยังสวนทางอีก = encoder มีปัญหาจริง
        fault("DIRECTION" + fingerTag(f));
        return false;
      }
      F.encFixed = true;
      fixEncoder(f);
      F.startPos = -F.startPos;
      F.progressPos = F.pos;
      F.progressTime = millis();
      return true;
    }
    if (moved * dir > 0) {
      F.goodTravel += abs(moved);
      if (F.goodTravel >= 2 * WRONG_DIR_DEG) F.encVerified = true;
      F.progressPos = F.pos;
      F.progressTime = millis();
    }
  } else if ((long)(F.readAt - F.progressTime) > (long)STALL_MS) {
    // ใช้เวลาของค่าที่อ่านได้ล่าสุด ไม่ใช่เวลาปัจจุบัน: ถ้าบอร์ดอ่านตำแหน่งไม่ได้ช่วงหนึ่ง จะไม่ตัดสินว่ามอเตอร์ติด
    // ติดอยู่ใกล้เป้าหมายมากแล้ว (กำลังต่ำสุดอาจไม่พอดันต่อ) ถือว่าถึงแล้ว
    if (abs(err) <= 3 * POS_TOLERANCE) {
      F.moving = false;
      motorBrake(f);
      return true;
    }
    // encoder ไม่ขยับเลยตั้งแต่เริ่ม = มอเตอร์ไม่หมุน หรือสาย encoder ไม่ได้เสียบ (ไม่ใช่นิ้วเกร็ง
    // เพราะต่อสปริงอนุกรม ถ้ามอเตอร์หมุนจริง สปริงต้องยืดและ encoder ต้องนับบ้าง)
    if (abs(F.pos - F.startPos) < STALL_MIN_DEG) {
      fault("NO_MOTION" + fingerTag(f));
      return false;
    }
    if (inStep(f)) {
      // นิ้วเกร็งต้าน (สปริงตึงจนมอเตอร์หมุนไม่ไหว): ไม่ฝืนดึงต่อ ค้างไว้ที่ตำแหน่งเท่าที่ไปได้
      F.moving = false;
      F.stalled = true;
      motorBrake(f);
      int pct = constrain(fingerPct(f), 0, 100);
      sendStatus("STALL:" + String(pct));
      sendStatus("FSTALL:" + String(f + 1) + "," + String(pct));
    } else {
      // ขณะคืนสายแล้วติด แปลว่ากลไกติดขัด ต้องหยุดให้คนตรวจสอบ
      fault("STALL" + fingerTag(f));
      return false;
    }
  }
  return true;
}

// ",นิ้ว,พอร์ต" ต่อท้ายข้อความ ERR ให้แอปบอกได้ว่าเป็นนิ้วไหน
String fingerTag(int f) {
  return "," + String(f + 1) + "," + String(MOTOR_PORT[f]);
}

// เวลาสูงสุดของจังหวะ: ระยะกำยิ่งมาก (หลายรอบ) ยิ่งใช้เวลานาน
unsigned long moveTimeout() {
  int rom = 0;
  for (int f = 0; f < NF; f++) rom = max(rom, (int)cfg.romDeg[f]);
  return MOVE_TIMEOUT + (unsigned long)rom * TIMEOUT_MS_PER_DEG;
}

bool maskDone(uint8_t mask) {
  for (int f = 0; f < NF; f++) if ((mask >> f & 1) && fg[f].moving) return false;
  return true;
}

void updateMotion() {
  // กันหมุนเกินระยะไม่ว่าอยู่จังหวะไหน (ยกเว้นตอนคาลิเบรต)
  if (phase != IDLE && phase != JOG) {
    for (int f = 0; f < NF; f++) {
      if (fg[f].pos > cfg.romDeg[f] + OVERTRAVEL_DEG) { fault("OVERTRAVEL" + fingerTag(f)); return; }
    }
  }

  for (int f = 0; f < NF; f++) {
    if (!stepFinger(f)) return;
  }

  if ((phase == GRIP || phase == RELEASE || phase == TWIST_OUT || phase == TWIST_IN || phase == RETURNING) &&
      millis() - phaseStart > moveTimeout()) {
    fault("TIMEOUT");
    return;
  }

  switch (phase) {
    case IDLE:
      break;

    case JOG:
      if (millis() - phaseStart >= JOG_MS) {
        brakeAll();
        phase = IDLE;
        // ตอนคาลิเบรต: ถ้าตำแหน่งเปลี่ยนสวนทางกับที่สั่ง แปลว่า encoder นับกลับทิศ
        for (int f = 0; f < NF; f++) {
          if (!selected(f)) continue;
          readFingerFresh(f);
          int moved = (fg[f].pos - fg[f].jogStartPos) * jogDir;
          if (moved <= -STALL_MIN_DEG && !fg[f].encVerified) fixEncoder(f);
          else if (moved >= 2 * STALL_MIN_DEG) fg[f].encVerified = true;
        }
        sendStatus("POS:" + String(posPercent()));
      }
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
    if (steps[stepIdx] >> f & 1) fg[f].peakDeg = max(fg[f].peakDeg, fg[f].pos);
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
      int pct = constrain(fg[f].peakDeg * 100L / cfg.romDeg[f], 0L, 100L);
      s += String(pct);
      sum += pct;
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
int fingerPct(int f) {
  return (int)((long)fg[f].pos * 100L / cfg.romDeg[f]);
}

// ตำแหน่งเฉลี่ยของนิ้วที่กำลังขยับ (ให้วงแหวนในแอปแสดงผลได้)
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
  }

  if (now - lastBattSent >= 10000) {
    lastBattSent = now;
    float v = MiniR4.PWR.getBattVoltage();
    if (v > 0) sendStatus("BATT:" + String(v, 1));
  }
}
