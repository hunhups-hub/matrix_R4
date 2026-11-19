# รายงานการประเมินความพร้อมของเฟิร์มแวร์ `matrix_R4` สำหรับการแข่งขัน Matrix MARC

- **โปรเจกต์**: `matrix_R4` (`Arduino UNO R4 WiFi` + `MATRIX Mini R4 Carrier Shield`)
- **ฐานโค้ดที่ตรวจสอบ (Codebase State)**: Branch `main` พร้อมชุดปรับปรุงเสถียรภาพฮาร์ดแวร์และโหมดแข่งขันเต็มกำลัง 100% (`Post-Stability & Full-Power Hardening`) `[Verified: Build/Test Run]`
- **คะแนนความพร้อมรวม (Overall Readiness Score)**: **`85.13 / 100`** *(เพิ่มขึ้นจาก `78.50 / 100` ก่อนปรับปรุงเสถียรภาพ)*
- **ระดับความพร้อม (Readiness Tier)**: **`Firmware-Ready (Track Calibration / Field Validation Required)`** — *เฟิร์มแวร์มีความเสถียรสูงและคงกำลังขับเต็ม 100% พร้อมลงสนาม เหลือเฉพาะการปรับจูนระยะทาง/มุมบนสนามแข่งจริง (HIL)*

---

## 1. บทสรุปผู้บริหาร (Executive Summary)

### 1.1 ภาพรวมผลคะแนนและระดับความพร้อม
จากการตรวจสอบโค้ดรายบรรทัดทั้ง 8 โมดูลใน `src/`, 8 เฮดเดอร์ใน `include/`, ชุดทดสอบ `test/test_robot_logic/test_main.cpp`, ไฟล์ตั้งค่า `platformio.ini` และไลบรารี `MatrixMiniR4 @ 1.2.6` ร่วมกับการรันคอมไพล์จริงบนเป้าหมาย `uno_r4_wifi`, การรัน Unit Test บน `native` และการวิเคราะห์เชิงสถิต (`cppcheck`) หลังการปรับปรุงเสถียรภาพ พบว่าโปรเจกต์ `matrix_R4` ได้คะแนนความพร้อมรวม **`85.13 / 100`** อยู่ในระดับสูงสุดของเฟิร์มแวร์ก่อนทดสอบภาคสนาม **`Firmware-Ready (Track Calibration / Field Validation Required)`**

| องค์ประกอบหลัก | ผลลัพธ์ | ป้ายสถานะการตรวจสอบ |
|---|---|---|
| **คอมไพล์เฟิร์มแวร์จริง (`uno_r4_wifi`)** | ผ่าน (`SUCCESS`, 0 warnings/errors ภายใต้ `-Werror`), RAM `43.9%`, Flash `41.5%` | `[Verified: Build/Test Run]` |
| **ชุดทดสอบอัตโนมัติ (`native` Unity)** | ผ่าน **`38 / 38`** เคส (0 Failures, 0 Ignored) ครอบคลุมครบทุกสาขาของ `robot_logic` และ `serial_control` | `[Verified: Build/Test Run]` |
| **การวิเคราะห์โค้ดเชิงสถิต (`pio check`)** | ผ่านสะอาด (`src/`: **`0 HIGH, 0 MEDIUM`** ทั้งบน `uno_r4_wifi` และ `native`) | `[Verified: Build/Test Run]` |
| **สถาปัตยกรรม Teleop, Failsafe & Auto Supervisor** | แข็งแรงระดับแข่งขัน (40 Hz scheduler, 5-stage kinodynamics 100% power, 1.4 ms SPI, 3s WDT, Rising-Edge E-Stop, Bounded STM32 Init/Guard) | `[Verified: Static Code Inspection]` |
| **ตรรกะภารกิจ Autonomous ปัจจุบัน (`auto_block_code.cpp`)** | ใช้ `MoveDegs` แบบ Open-Loop + `delay()` รวม `15.5 s` ยังไม่ใช้ Gyro/Sensor ป้อนกลับ | `[Verified: Static Code Inspection]` |
| **พฤติกรรมบนฮาร์ดแวร์และสนามแข่งจริง (HIL)** | ยังไม่ได้เชื่อมต่อบอร์ด UNO R4 WiFi, STM32, มอเตอร์, เซอร์โว และจอย PS2/MJ2 จริง | `[Unverified: Requires Physical Hardware/Field Test]` |

### 1.2 จุดแข็งหลักและรายการเสถียรภาพที่ได้รับการยกระดับ (Key Strengths & Hardened Stability Features)
1. **ระบบควบคุมเวลาจริงและไดรเวอร์ PS2 ที่กำหนดเวลาแน่นอน (Deterministic Real-Time Control)**: วงรอบควบคุม `40 Hz` (`kControlPeriodMs = 25UL`) แบบ non-blocking พร้อม resync เมื่อเลยคาบเวลา (`src/main.cpp:1042-1051`), ไดรเวอร์บิตแบง SPI ระดับรีจิสเตอร์ RA4M1 ที่ใช้เวลา ~`1.4 ms`/เฟรม (`src/ps2_driver.cpp:10-258`), การซิงก์ 3 เฟรมตั้งต้นตอนบูตเพื่อให้เข้าสถานะ `PS2 OK` และ `STB:100% L+ P+ C+ T+` ทันที (`src/main.cpp:890-914`), การตรวจสอบแพ็กเก็ต 5 ขั้น (`isPs2FrameValid`, `src/robot_logic.cpp:683-717`) และตัวจับเวลา Watchdog 3 วินาที (`src/main.cpp:1023`) `[Verified: Build/Test Run]` & `[Verified: Static Code Inspection]`
2. **ระบบขับเคลื่อน Kinodynamic 5 ขั้นเต็มกำลัง 100%**: `computeKinodynamicDrive()` (`src/robot_logic.cpp:417-522`) รวม Cubic Expo Shaping (`kThrottleExpoPct = 35%`, `kTurnExpoPct = 45%`, `kTurnMaxRatioPct = 80%`, และโหมด Precision `55%/60%`), Centripetal Traction Circle (`|v * w| / 100 <= 52`), CoM Anti-Tip Scaling ตามมุมแขน `RC2`, Balanced Trim (`±15%`), Coulomb Stiction Feedforward (`12%`), Coupled 2-DOF Slew Limiter พร้อมตัวคุมเพดานกำลัง Envelope, โหมด Precision 55% (`Select`), Reverse-Front (`R3`), Micro-Turn (`D-Pad L/R`) `[Verified: Build/Test Run]` และ Smart H-Bridge Hold-Brake (`src/minir4_io.cpp:577-636`) `[Verified: Static Code Inspection]`
3. **เกราะป้องกันโหมด Autonomous สำหรับ MATRIXblock R4 (`AutoMode` + Proxy Layer)**:
   - ซิงก์ทิศทางมอเตอร์ `f1`/`f2` จาก EEPROM เข้าสู่ `DriveDC.begin()` อัตโนมัติ (`resolveAutoDrivePolarity`, `src/robot_logic.cpp:402-415`) `[Verified: Build/Test Run]`
   - ตรวจเช็กผลลัพธ์ `DriveDC.begin()` ทุกครั้ง หาก STM32 ไม่ตอบสนองภายใน ~3 วินาที จะสั่งล็อกเบรกและตัดกลับโหมด Manual อย่างปลอดภัย (`AutoExitReason::DriveInitFail`, `src/auto_block_code.cpp:169-197`) `[Verified: Static Code Inspection]`
   - ตรวจจับทั้งล้อติดกำแพงและสาย UART หลุดกลางคันระหว่างคำสั่งบล็อกกิ้ง `DriveDC` (`beginBlockingDriveTask` / `checkDriveStall`, `src/auto_mode.cpp:141-226`) พร้อมรักษาสัญญาณ `PSB_TRIANGLE` แม้โค้ดผู้ใช้เรียก `read_gamepad()` ซ้ำ (`src/auto_block_code.cpp:487-513`) `[Verified: Static Code Inspection]`
4. **ความทนทานต่อความผิดพลาดของฮาร์ดแวร์และปุ่มฉุกเฉินทันที (Hardware Fault Isolation & Instant E-Stop)**:
   - ปุ่ม `BTN_DOWN` ทำงานเป็น **Rising-Edge E-Stop ทันทีที่กด (`downPressEdge`)** หลังผ่าน Debounce 30 ms โดยไม่ต้องรอปล่อยนิ้ว ทั้งในโหมด Manual และ Auto (`src/minir4_io.cpp:426-429`, `src/main.cpp:593`, `src/auto_mode.cpp:257`) ขณะที่ยังรองรับการกดค้างคู่ `BTN_UP + BTN_DOWN` 2.5 วินาทีเพื่อเข้า Bootloader `[Verified: Static Code Inspection]`
   - บูตระบบแบบจำกัดเวลา (`MiniR4IO::begin()`, `src/minir4_io.cpp:41-149`) โดยไม่ใช้ลูป `while` ไม่รู้จบของ `MiniR4.begin()`, ส่งเซอร์โว `RC3+RC4` แบบ Atomic เฟรมเดียวด้วย `mmL.SetAllServoAngle` โดยไม่มี fallback รายช่องที่ทำให้แขนบิดเหลื่อมเวลา และมี `noteCommFailure()` ป้องกันการบล็อก UART ซ้อนกันหลายคำสั่งในรอบเดียวเมื่อสาย STM32 หลุด (`~100–110 ms` สูงสุดต่อรอบ แทน `~1.0 s` เดิม) `[Verified: Static Code Inspection]`
   - คงเพดานกำลังขับสูงสุด **100% เต็ม** (`kEnableBatterySagThrottle = false`) พร้อมชดเชยแรงบิดสูงสุด `115%` ในระดับ `Normal`/`Low`/`Critical` และตัดบูสต์ส่วนเกินเหลือ `100%` เฉพาะเมื่อแรงดันตกถึงระดับฉุกเฉิน `Emergency` (`<= 6.4V` หรือ `<= 69%` ของแรงดันฐาน) เพื่อลดความเสี่ยงไฟตกโดยหุ่นไม่หนืด (`src/robot_logic.cpp:1049-1058`) `[Verified: Build/Test Run]`

### 1.3 ความเสี่ยงและข้อจำกัดที่เหลืออยู่ก่อนลงแข่ง (Remaining Risks & Field Calibration Items)
1. **[High] โค้ดภารกิจ Autonomous ปัจจุบันเดินแบบ Open-Loop Dead-Reckoning และมี `delay()` รวม 15.5 วินาที (`src/auto_block_code.cpp:630-682`)**: ใช้คำสั่ง `DriveDC.MoveDegs(±50, ±50, ...)` ล้วนจำนวน 11 ครั้ง และ `delay(500/1000)` ถึง 20 ครั้ง (รวม `15.5 s` + เบรกดีเลย์ `1.21 s` = `16.71 s`) โดยยังไม่ใช้ `MoveSyncDegs`/`MoveGyroDegs`/`TurnGyro` หรืออ่านเซนเซอร์สนาม ทำให้ระยะทางและมุมเลี้ยวไวต่อระดับแรงดันแบตเตอรี่และความเสียดทานพื้นสนามจริง `[Verified: Static Code Inspection]`
2. **[Medium] การคงกำลังขับ 100% เมื่อแบตเตอรี่อ่อน (`kEnableBatterySagThrottle = false`, `include/config.h:144`)**: เพื่อให้หุ่นยนต์แรงและเร็วเต็ม 100% ตลอดแมตช์ตามความต้องการของผู้ขับ เฟิร์มแวร์จึงไม่ลดเพดานกำลังลงเหลือ 72%/50%/35% (แต่ได้ตัดบูสต์ 115% เหลือ 100% ที่ระดับ `Emergency` แล้ว) ดังนั้นทีมแข่งต้องเปลี่ยนแบตเตอรี่ก้อนใหม่ทุกครั้งเมื่อหน้าจอแสดงความจุต่ำกว่า `75%` หรือขึ้นแฟลก `L`/`C`/`E` `[Verified: Build/Test Run]`
3. **[Low] ข้อควรระวังในการวางโค้ด Auto (`longjmp`) และมุมปุ่ม `Square`**: การตัดจบ Auto ใช้ `longjmp` (`src/auto_mode.cpp:127`) จึงไม่ควรประกาศตัวแปร C++ ที่มี destructor (เช่น `String`) ในโซนวางโค้ด `auto_block_code.cpp` และค่า `kRc3/4SquareDeg` (`30°`) ตั้งไว้ตรงกับ `kRc3/4InitDeg` และ `kRc3/4L1Deg` (`30°`) จึงเกิดการขยับสลับเมื่อกดหลังจากยกแขนด้วย `R1` (`140°`) `[Verified: Static Code Inspection]`

---

## 2. บริบทการแข่งขัน Matrix MARC และเกณฑ์การให้คะแนน (Competition Context & Weighted Rubric)

### 2.1 สเกลการให้คะแนนระดับหมวด (0–10 Scoring Anchors)
| คะแนน (0–10) | ความหมายเชิงวิศวกรรม (Anchor Definition) |
| :---: | :--- |
| **`0`** | **Missing / Broken** — ไม่มีฟังก์ชันงาน หรือคอมไพล์ไม่ผ่าน/ผิดพลาดร้ายแรงจนใช้งานไม่ได้ |
| **`2`** | **Stub / Major Defects** — มีเพียงโครงร่าง หรือมีบั๊กร้ายแรงที่ทำให้หุ่นยนต์ควบคุมไม่ได้/เสี่ยงพังทันที |
| **`5`** | **Partial / Needs Work** — ทำงานได้บางส่วนในแล็บ แต่ยังขาดความเสถียร เซนเซอร์ หรือการป้องกันที่จำเป็นสำหรับสนามแข่ง |
| **`8`** | **Solid / Competition-Capable** — โครงสร้างและการทำงานแข็งแรง พร้อมลงแข่งได้จริง โดยมีข้อควรระวังหรือจุดปรับจูนเล็กน้อยที่ระบุชัดเจน |
| **`10`** | **Verified Match-Hardened** — สมบูรณ์ครบถ้วน ผ่านทั้ง Unit/Integration Test และผ่านการทดสอบบนสนามแข่งจริง (HIL) แล้ว |

### 2.2 ตารางคะแนนถ่วงน้ำหนัก 7 หมวดรวม 100% (Weighted Rubric)
$$\text{Weighted Points} = \left(\frac{\text{Category Score}}{10}\right) \times \text{Weight (\%)} $$

| หมวดที่ | ชื่อหมวดการประเมิน | น้ำหนัก (%) | คะแนนเดิม | คะแนนปัจจุบัน (0–10) | คะแนนถ่วงน้ำหนัก | การคำนวณ |
| :-: | :--- | :---: | :---: | :---: | :---: | :--- |
| **1** | **Build, Static Hygiene & Automated Tests** | `15%` | `8.50` | **`9.00`** | **`13.50`** | `(9.00 / 10) × 15 = 13.50` |
| **2** | **Teleop Drive & Kinodynamic Precision** | `15%` | `9.00` | **`9.00`** | **`13.50`** | `(9.00 / 10) × 15 = 13.50` |
| **3** | **Manipulator & Servo Actuation (`RC1..RC4`)** | `10%` | `8.00` | **`8.50`** | **`8.50`** | `(8.50 / 10) × 10 = 8.50` |
| **4** | **Autonomous Mode & MARC Match Readiness** | `20%` | `6.00` | **`6.75`** | **`13.50`** | `(6.75 / 10) × 20 = 13.50` |
| **5** | **Wireless Link, Coprocessor & Failsafe Reliability** | `15%` | `8.00` | **`9.25`** | **`13.88`** | `(9.25 / 10) × 15 = 13.875` |
| **6** | **Power Management, Stall Safety & Persistence** | `10%` | `7.50` | **`8.00`** | **`8.00`** | `(8.00 / 10) × 10 = 8.00` |
| **7** | **Competition Operations, Diagnostics & Maintainability** | `15%` | `8.50` | **`9.50`** | **`14.25`** | `(9.50 / 10) × 15 = 14.25` |
| | **รวมทั้งหมด (Total)** | **`100%`** | `78.50` | — | **`85.13 / 100`** | `13.50 + 13.50 + 8.50 + 13.50 + 13.875 + 8.00 + 14.25 = 85.125` |

### 2.3 ระดับความพร้อม (Readiness Rating Tiers)
- **`90 – 100`**: **Match-Ready (Field-Verified)** — พร้อมลงสนามแข่งเต็มรูปแบบ ผ่านการทดสอบทั้งโค้ดและสนามจริง (HIL)
- **`75 – 89`**: **Firmware-Ready (Track Calibration / Field Validation Required)** — ซอฟต์แวร์แข็งแรงพร้อมใช้ แต่ต้องปรับจูนและทดสอบ HIL บนสนามจริง **(ระดับปัจจุบันของ `matrix_R4`: `85.13 / 100`)**
- **`60 – 74`**: **Conditional Readiness (Notable Autonomous/Reliability Gaps)** — พอใช้แข่งแบบมีเงื่อนไข มีจุดอ่อนสำคัญใน Auto หรือความเสถียรที่ควรแก้ก่อน
- **`< 60`**: **Not Ready for Competition** — ยังไม่พร้อมลงแข่ง ต้องแก้ไขโครงสร้างหลักหรือบั๊กร้ายแรงก่อน

---

## 3. ผลการรันตรวจสอบจริง (Live Verification Results)

### 3.1 การคอมไพล์เฟิร์มแวร์เป้าหมายจริง (`pio run -e uno_r4_wifi`) `[Verified: Build/Test Run]`
- **คำสั่ง**: `& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e uno_r4_wifi`
- **สถานะ**: `SUCCESS` (Exit code `0`), **0 Warnings / 0 Errors** ภายใต้ `-Wall -Wextra -Werror` (`platformio.ini:22-25`)
- **การใช้หน่วยความจำ**:
  - **RAM**: **`43.9%`** (`used 14396 bytes from 32768 bytes`) — เหลือพื้นที่ Heap/Stack อีก `18,372 bytes`
  - **Flash**: **`41.5%`** (`used 108820 bytes from 262144 bytes`) — เหลือพื้นที่โปรแกรมอีก `153,324 bytes`

### 3.2 การรันชุดทดสอบอัตโนมัติบนโฮสต์ (`pio test -e native -v`) `[Verified: Build/Test Run]`
- **คำสั่ง**: `& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" test -e native -v`
- **สถานะ**: `PASSED` (Exit code `0`), ผ่านครบ **`38 Tests 0 Failures 0 Ignored`** ใน 10 หมวดหมู่ (`test/test_robot_logic/test_main.cpp:1887-1951`) รวมถึงการทดสอบ Glitch absorption โดยไม่หลุดจาก `Operational`, การคุมเพดานกำลัง 2-DOF Slew (`55%`/`35%`), ความปลอดภัยจาก `int16_t` overflow, การหยุด `!w` ในแบตช์เดียว, ข้อความ Help `o/p` + `v`, และฟิลด์ Telemetry `rf=`/`com=`

### 3.3 การวิเคราะห์โค้ดเชิงสถิตด้วย `cppcheck` (`pio check`) `[Verified: Build/Test Run]`
1. **`pio check -e uno_r4_wifi --skip-packages`**: Exit code `0` (`PASSED`)
   - โฟลเดอร์ `src/`: **`0 HIGH, 0 MEDIUM`** (ไม่มีคำเตือนระดับ HIGH หรือ MEDIUM ในโค้ดของโปรเจกต์)
2. **`pio check -e native --skip-packages`**: Exit code `0` (`PASSED`)
   - โฟลเดอร์ `src/`: **`0 HIGH, 0 MEDIUM`** (แก้ไข `compareBoolExpressionWithInt`, `constVariable`, `redundantInitialization` และ `variableScope` ครบทั้งหมดแล้ว)

---

## 4. การประเมินรายหมวดพร้อมหลักฐานอ้างอิง (Category-by-Category Detailed Assessment)

### หมวดที่ 1: Build, Static Hygiene & Automated Tests (`15%`) — คะแนน **`9.00 / 10`** (`13.50 / 15.00` คะแนน)
- **หลักฐานยืนยัน**:
  - คอมไพล์ผ่านภายใต้ `-Wall -Wextra -Werror` ทั้งบน `uno_r4_wifi` และ `native`; ขอบเขต `#pragma GCC diagnostic` ใน `src/auto_block_code.cpp:616-699` ถูกจำกัดเฉพาะโซนโค้ดที่ผู้ใช้คัดลอกมาจาก MATRIXblock เท่านั้น ทำให้ชั้น Proxy ความปลอดภัยทั้งหมดอยู่ภายใต้ `-Werror` เต็มรูปแบบ `[Verified: Build/Test Run]`
  - `pio check` รายงาน `0 HIGH, 0 MEDIUM` ใน `src/` ทั้งสองสภาพแวดล้อม `[Verified: Build/Test Run]`
  - `pio test -e native -v` ผ่าน `38 / 38` เคส พร้อมอัปเดตคอมเมนต์และเพิ่ม assertion ครอบคลุมครบทุกสาขาของ `robot_logic.cpp` และ `serial_control.cpp` `[Verified: Build/Test Run]`
- **เหตุผลการหักคะแนน (`-1.0`)**:
  - โมดูลฮาร์ดแวร์ (`main.cpp`, `minir4_io.cpp`, `ps2_driver.cpp`, `oled_ui.cpp`, `auto_mode.cpp`, `auto_block_code.cpp`) ยังไม่มีชุดทดสอบจำลองฮาร์ดแวร์หรือ HIL อัตโนมัติ (`-1.0`) `[Verified: Static Code Inspection]`

### หมวดที่ 2: Teleop Drive & Kinodynamic Precision (`15%`) — คะแนน **`9.00 / 10`** (`13.50 / 15.00` คะแนน)
- **หลักฐานยืนยัน**:
  - ไปป์ไลน์ 5 ขั้นใน `computeKinodynamicDrive` (`src/robot_logic.cpp:417-522`) ผ่านการทดสอบครบทุกสาขาใน `test_main.cpp` (รวมถึง `kTurnExpoPct = 45%`, `kTurnMaxRatioPct = 80%` เมื่อขับเดินหน้าพร้อมเลี้ยวโค้ง, และการคุมเพดานกำลัง Envelope กันโอเวอร์ชูตหลัง 2-DOF Slew) และรักษาความแรงเต็ม 100% `[Verified: Build/Test Run]`
- **เหตุผลการหักคะแนน (`-1.0`)**:
  - ค่าพารามิเตอร์ทางฟิสิกส์ (`kMaxCentripetalLatAccel = 52`, `kMotorStictionMinPower = 12`, `kCoMHighMinScalePct = 65`) ต้องผ่านการทดสอบความเสียดทานล้อและจุดศูนย์ถ่วงบนตัวหุ่นจริง (`-1.0`) `[Unverified: Requires Physical Hardware/Field Test]`

### หมวดที่ 3: Manipulator & Servo Actuation (`RC1..RC4`) (`10%`) — คะแนน **`8.50 / 10`** (`8.50 / 10.00` คะแนน)
- **หลักฐานยืนยัน**:
  - `updateServoTargetsFromPs2` (`src/robot_logic.cpp:554-636`) ทดสอบครบทั้ง `L1`, `R1`, `L2`, `R2`, `D-Pad U/D`, `Square` และ `Circle` `[Verified: Build/Test Run]`
  - `MiniR4IO::applyServos` (`src/minir4_io.cpp:722-795`) และ `staggerBootServos` (`lines 311-338`) ส่งคำสั่ง `mmL.SetAllServoAngle` แบบ Atomic เฟรมเดียวเมื่อ `RC3`/`RC4` ขยับหรือมีการเปลี่ยนหลายช่อง โดยไม่ตกลงไปสั่งแยกทีละช่องที่อาจทำให้แขนหน้าซ้าย/ขวาบิดเหลื่อมเวลา `[Verified: Static Code Inspection]`
- **เหตุผลการหักคะแนน (`-1.5`)**:
  - `kRc3InitDeg`, `kRc3L1Deg`, `kRc3SquareDeg` (และ `RC4`) ตั้งไว้ที่ `30°` เท่ากัน (`include/config.h:115-123`) จึงเกิดการขยับสลับเฉพาะเมื่อกด `Square` หลังจากยกแขนด้วย `R1` (`140°`) (`-0.5`) `[Verified: Static Code Inspection]`
  - ขอบเขตกลไกจริงและช่วงพัลส์ `500–2500 us` บนเซอร์โวตัวจริงต้องตรวจบนหุ่นยนต์ (`-1.0`) `[Unverified: Requires Physical Hardware/Field Test]`

### หมวดที่ 4: Autonomous Mode & MARC Match Readiness (`20%`) — คะแนน **`6.75 / 10`** (`13.50 / 20.00` คะแนน)
- **หลักฐานยืนยัน**:
  - `AutoDriveDCProxy::begin` (`src/auto_block_code.cpp:169-197`) ซิงก์ทิศทางมอเตอร์ `f1`/`f2` จาก EEPROM อัตโนมัติผ่าน `RobotLogic::resolveAutoDrivePolarity` (`src/robot_logic.cpp:402-415`) `[Verified: Build/Test Run]` และสั่งเบรกตัดจบเข้า `AutoExitReason::DriveInitFail` ทันทีหากตั้งค่า `DriveDC` ไม่สำเร็จภายใน ~3 วินาที `[Verified: Static Code Inspection]`
  - `checkDriveStall` (`src/auto_mode.cpp:141-226`) ทำงานร่วมกับ `beginBlockingDriveTask()`/`endBlockingDriveTask()` สามารถตัดจบ `StallTimeout` ได้ทั้งเมื่อล้อติดกำแพงและเมื่อสาย UART ของ STM32 หลุดระหว่างรอคำสั่ง `DriveDC` `[Verified: Static Code Inspection]`
  - `AutoPs2Proxy::pollInternal` (`src/auto_block_code.cpp:487-513`) รักษาสัญญาณ `PSB_TRIANGLE` แม้มีการเรียก `read_gamepad()` ซ้ำก่อนตรวจ `Button(PSB_TRIANGLE)` `[Verified: Static Code Inspection]`
- **เหตุผลการหักคะแนน (`-3.25`)**:
  - ภารกิจ Auto ใน `src/auto_block_code.cpp:630-682` ยังเป็นการเดินแบบ Open-Loop `MoveDegs(±50, ±50)` 11 ครั้ง และ `delay()` รวม `15.5 s` โดยไม่ใช้ Gyro/Sync PID หรือเซนเซอร์สนาม (`-3.0`) `[Verified: Static Code Inspection]`
  - การตัดจบด้วย `longjmp` มีข้อจำกัดห้ามประกาศตัวแปรที่มี C++ destructor ในโซนวางโค้ด Auto (`-0.25`) `[Verified: Static Code Inspection]`

### หมวดที่ 5: Wireless Link, Coprocessor & Failsafe Reliability (`15%`) — คะแนน **`9.25 / 10`** (`13.88 / 15.00` คะแนน)
- **หลักฐานยืนยัน**:
  - ปุ่ม `BTN_DOWN` ทริกเกอร์ E-Stop ทันทีที่กด (`downPressEdge`) หลังผ่าน Debounce 30 ms (`src/minir4_io.cpp:426-429`, `src/main.cpp:593`, `src/auto_mode.cpp:257`) ไม่ต้องรอปล่อยปุ่ม `[Verified: Static Code Inspection]`
  - `MiniR4IO::begin` (`src/minir4_io.cpp:41-149`) ใช้การเริ่มต้น STM32 และมอเตอร์แบบจำกัดรอบแทน `MiniR4.begin()` จึงไม่ค้างในลูป `while` ไม่รู้จบก่อนเริ่ม Watchdog `[Verified: Static Code Inspection]`
  - `MiniR4IO::noteCommFailure` (`src/minir4_io.cpp:184-190`) ป้องกันการยิงคำสั่ง UART ซ้ำในรอบเดียวกันเมื่อ STM32 ไม่ตอบสนอง ลดเวลาบล็อกสูงสุดต่อรอบจาก `~0.9–1.1 s` เหลือเพียง `~100–110 ms` (1 คำสั่ง) `[Verified: Static Code Inspection]`
  - ด่านตรวจแพ็กเก็ต PS2 ทั้ง 5 ด่านใน `isPs2FrameValid` (`src/robot_logic.cpp:683-717`) ทำงานและถูกทดสอบครบทุกสาขาใน `test_main.cpp` `[Verified: Build/Test Run]`
- **เหตุผลการหักคะแนน (`-0.75`)**:
  - ไลบรารี `MMLower::WaitData` ยังเป็นการรอแบบบล็อกกิ้งสูงสุด `100 ms` ในคำสั่งแรกที่ STM32 ขาดการติดต่อ (`-0.25`) `[Verified: Static Code Inspection]`
  - ความทนทานต่อคลื่นรบกวน 2.4 GHz ในสนามแข่งขันจริงต้องทดสอบภาคสนาม (`-0.50`) `[Unverified: Requires Physical Hardware/Field Test]`

### หมวดที่ 6: Power Management, Stall Safety & Persistence (`10%`) — คะแนน **`8.00 / 10`** (`8.00 / 10.00` คะแนน)
- **หลักฐานยืนยัน**:
  - `BatteryStabilizer::motorProfile()` (`src/robot_logic.cpp:1035-1085`) คงกำลังขับสูงสุด `100%` (`kEnableBatterySagThrottle = false`) พร้อมชดเชยแรงบิดสูงสุด `115%` ในระดับ `Normal`/`Low`/`Critical` เพื่อให้หุ่นยนต์แรงเต็มพิกัด และตัดการบูสต์เกิน `100%` (`compPercent = 100U`) เฉพาะในระดับ `Emergency` (`<= 6.4V` หรือ `<= 69%` ของแรงดันฐาน) เพื่อลดความเสี่ยงไฟตกที่พื้นแรงดันต่ำสุด `[Verified: Build/Test Run]`
  - `MiniR4IO::applyLowerMcuConfig` (`src/minir4_io.cpp:151-182`) บังคับตั้ง `SetDCMotorSpeedRange(1/2, 0, 100)` ทุกครั้งที่กู้คืนการเชื่อมต่อ STM32 `[Verified: Static Code Inspection]`
- **เหตุผลการหักคะแนน (`-2.0`)**:
  - การเปิดโหมดกำลังขับ 100% เต็ม (`kEnableBatterySagThrottle = false`) ต้องอาศัยวินัยในการเปลี่ยนแบตเตอรี่เมื่อความจุต่ำกว่า `75%` และโหมด Manual ยังไม่มีการตัดกำลังอัตโนมัติเมื่อล้อติดขัด (`-1.0`) `[Verified: Static Code Inspection]`
  - พฤติกรรมแรงดันตกภายใต้โหลดกระชากของมอเตอร์จริงต้องทดสอบกับแบตเตอรี่แข่งจริง (`-1.0`) `[Unverified: Requires Physical Hardware/Field Test]`

### หมวดที่ 7: Competition Operations, Diagnostics & Maintainability (`15%`) — คะแนน **`9.50 / 10`** (`14.25 / 15.00` คะแนน)
- **หลักฐานยืนยัน**:
  - `initPs2Link` (`src/main.cpp:890-914`) ซิงก์เฟรม PS2 จนครบ `kPs2StableFrames` (3 เฟรม) ตั้งแต่ตอนบูต ทำให้เมื่อเปิดจอยไว้ก่อนเปิดหุ่น หน้าจอ OLED และรายงาน `[STAT]` ตอนบูตจะแสดงสถานะ `PS2 OK`, `src=PS2`, `ps2=OPERATIONAL`, และ `STB:100% L+ P+ C+ T+` ทันที `[Verified: Static Code Inspection]`
  - `StabilityMonitor::update` (`src/robot_logic.cpp:1150-1170`) ไม่หักคะแนน Link Penalty (`0%`, แสดง `L+`) เมื่อรันโหมด Autonomous โดยไม่เปิดจอย PS2 (`ControlSource::Auto`) และอัปเดตค่าความเสถียรทันทีทั้งตอนเข้าและออกจากโหมด Auto (`src/main.cpp:486-491, 565-567`) `[Verified: Build/Test Run]` & `[Verified: Static Code Inspection]`
  - คอมเมนต์ในชุดทดสอบและเอกสาร `README.md` สอดคล้องกับโค้ดจริงทุกจุด `[Verified: Static Code Inspection]`
- **เหตุผลการหักคะแนน (`-0.5`)**:
  - ความชัดเจนของหน้าจอ OLED และเสียงเตือน Buzzer ท่ามกลางแสงและเสียงในสนามแข่งจริงต้องตรวจสอบหน้างาน (`-0.5`) `[Unverified: Requires Physical Hardware/Field Test]`

---

## 5. ตารางสรุปสถานะความเสี่ยงและรายการตรวจสอบก่อนลงแข่ง (Risk Status & HIL Checklist)

### 5.1 สถานะการจัดการความเสี่ยงทั้งหมด
| ระดับปัจจุบัน | ประเด็น (Issue / Risk) | สถานะ | รายละเอียดการแก้ไขและแนวทางปฏิบัติ |
| :---: | :--- | :---: | :--- |
| **High** | **โหมด Auto เดินแบบ Open-Loop `MoveDegs` + `delay()` นาน `15.5 s`** (`src/auto_block_code.cpp:630-682`) | **ต้องจูนบนสนามจริง** | โค้ดภารกิจของผู้ใช้ยังใช้ `MoveDegs(±50, ±50)` และ `delay()` คงที่ ควรทดสอบวัดระยะบนสนามจริง หรือเปลี่ยนเป็น `MoveSyncDegs`/`MoveGyroDegs`/`TurnGyro` ตามกติกาสนาม `[Unverified: Requires Physical Hardware/Field Test]` |
| **Medium** | **เปิดโหมดกำลังขับ 100% (`kEnableBatterySagThrottle = false`)** (`include/config.h:144`, `src/robot_logic.cpp:1049-1058`) | **ปรับปรุงแล้ว + วินัยแบต** | คงกำลัง 100% และบูสต์ 115% ในระดับ `Normal`/`Low`/`Critical` เพื่อให้หุ่นแรงเต็มพิกัด และตัดบูสต์เหลือ `100%` ที่ระดับ `Emergency` (`<= 6.4V`) ให้เปลี่ยนแบตใหม่เมื่อจอแสดงต่ำกว่า `75%` `[Verified: Build/Test Run]` |
| **Resolved** | **`BTN_DOWN` E-Stop ทำงานทันทีที่กด (`downPressEdge`)** (`src/minir4_io.cpp:426-429`, `src/main.cpp:593`, `src/auto_mode.cpp:257`) | **แก้ไขสมบูรณ์** | เปลี่ยนเป็น Rising-Edge E-Stop ทันทีที่กดปุ่มผ่าน Debounce 30 ms ทั้งในโหมด Manual และ Auto โดยยังกดค้างคู่ `BTN_UP + BTN_DOWN` 2.5s เข้า Bootloader ได้ตามปกติ `[Verified: Static Code Inspection]` |
| **Resolved** | **ซิงก์ทิศมอเตอร์ EEPROM และตรวจ `DriveDC.begin()` ใน Auto** (`src/auto_block_code.cpp:169-197`, `src/robot_logic.cpp:402-415`) | **แก้ไขสมบูรณ์** | `AutoDriveDCProxy::begin()` นำค่ากลับทิศ `f1`/`f2` จาก EEPROM มาปรับทิศให้อัตโนมัติ และหากตั้งค่า STM32 ไม่สำเร็จภายใน ~3s จะสั่งเบรกและออกด้วย `DriveInitFail` ทันที `[Verified: Build/Test Run]` |
| **Resolved** | **จำกัดเวลาเริ่มต้น `MiniR4IO::begin()` และป้องกัน UART บล็อกซ้อน** (`src/minir4_io.cpp:41-149, 184-190, 722-795`) | **แก้ไขสมบูรณ์** | แทนที่ `MiniR4.begin()` ด้วยการเริ่มต้นที่มีขอบเขตเวลา, ส่งเซอร์โวด้วย `SetAllServoAngle` แบบ Atomic และใช้ `noteCommFailure()` จำกัดการบล็อก UART สูงสุดไม่เกิน 1 คำสั่ง (`~100 ms`) ต่อรอบ `[Verified: Static Code Inspection]` |
| **Resolved** | **คำเตือน `cppcheck` (`0 HIGH, 0 MEDIUM`) และขอบเขต `#pragma GCC diagnostic`** (`src/auto_block_code.cpp`, `src/ps2_driver.cpp`, `src/oled_ui.cpp`, `src/robot_logic.cpp`) | **แก้ไขสมบูรณ์** | แก้ไขคำเตือน `cppcheck` ใน `src/` จนเหลือ `0 HIGH, 0 MEDIUM` และย้าย `#pragma GCC diagnostic` ลงไปครอบเฉพาะโซนวางโค้ดของผู้ใช้ `[Verified: Build/Test Run]` |
| **Resolved** | **แก้ไขบั๊กความปลอดภัยและความเสถียรระดับลึกครบทุกโมดูล (Full-Repo Audit)** (`src/minir4_io.cpp`, `src/robot_logic.cpp`, `src/auto_mode.cpp`, `src/oled_ui.cpp`, `src/main.cpp`, `src/serial_control.cpp`, `src/ps2_driver.cpp`) | **แก้ไขสมบูรณ์** | ปิดจุดเสี่ยงสำคัญ: (1) `forceMotorsStop` ไม่ถูกหน่วงเมื่อ `commHealthy_` ปกติหรือก่อน Bootloader (2) `noteCommFailure` ใช้เวลาหลัง timeout จริงพร้อม signed wrap-safe backoff (3) ล้างสถานะปุ่มเมื่อ STM32 UART ล้ม (4) `FailsafeStateMachine` ดูดซับ Glitch 1–3 เฟรมโดยไม่หลุดจาก `Operational` (5) `AutoMode::checkSerialAbort` ใช้ ring buffer 32 ไบต์กัน `\r`/`\n` บล็อก `!`/`x`/`u` (6) บล็อกคำสั่งเซอร์โว Serial ระหว่าง `ESTOP` (7) สำรอง Framebuffer OLED ตอนกู้ I2C กันจอดำและล้าง hash cache เมื่อส่งพลาด (8) ป้องกัน `int16_t` overflow และ 2-DOF slew overshoot เกินเพดาน `55%`/`35%` (9) ล้าง `WDTRF` และรีเซ็ตตัวจับเวลาลูปหลังบูต/หลัง Auto `[Verified: Build/Test Run]` |
| **Low** | **ข้อควรระวัง `longjmp` ใน `auto_block_code.cpp` และมุมปุ่ม `Square` (`30°`)** (`src/auto_mode.cpp:127`, `include/config.h:115-123`) | **รับทราบตามการออกแบบ** | หลีกเลี่ยงการใช้ `String` หรือตัวแปรที่มี destructor ในโซนวางโค้ด Auto และปุ่ม `Square` จะสลับระหว่างมุม `R1` (`140°`) กับมุมดันพื้น (`30°`) `[Verified: Static Code Inspection]` |

### 5.2 รายการตรวจสอบภาคสนามก่อนแข่ง (Pit & Field HIL Pre-Competition Checklist)
1. **บูตระบบ & ตรวจค่าสถานะเริ่มต้น (`STB:100% L+ P+ C+ T+`)**:
   - เปิดจอย PS2/MJ2 แล้วเปิดสวิตช์แบตเตอรี่แข่งจริง ตรวจหน้าจอ OLED ต้องขึ้น `PS2 OK`, `STB:100% L+ P+ C+ T+`, `M1:+000 M2:+000 H` และไม่มี `rngErr`
2. **การขับ Teleop เต็มกำลัง 100% & ระบบเบรก**:
   - ทดสอบเดินหน้า/ถอยหลัง/เลี้ยวโค้งความเร็วสูงด้วย `LY`/`RX`, ทดสอบ `Select` (Precision 55%), `R3` (Reverse-Front), `D-Pad L/R` (Micro-Turn), และ `Start + D-Pad L/R` (Motor Trim พร้อมรีบูตเช็ก EEPROM)
3. **แขนกล & เซอร์โว `RC1..RC4`**:
   - ทดสอบ `RC1` (`L2=180° / R2=0°`), `RC2` (`D-Pad U/D`), `RC3+RC4` (`L1=30° / R1=140° / Square`), และ `Circle` (Drop/Hold preset)
4. **Failsafe & ปุ่มหยุดฉุกเฉินทันที (`BTN_DOWN`)**:
   - ปิดจอยขณะล้อหมุน: หุ่นต้องหยุดและล็อกเบรกภายใน `~100 ms` (`FAILSAFE`) และกลับมาเป็น `PS2 OK` อัตโนมัติเมื่อเปิดจอยใหม่
   - ทดสอบกด **`BTN_DOWN` ค้างไว้**: หุ่นต้องเข้า `ESTOP!` ทันทีที่นิ้วกดลงโดยไม่ต้องรอปล่อยนิ้ว และกด `BTN_UP` เพื่อปลดล็อก
5. **โหมด Autonomous & ความปลอดภัย**:
   - ทดสอบเริ่ม Auto ทั้งจาก `Triangle` และกด `BTN_UP` ค้าง `1.2 s` (ตรวจว่าขณะรัน Auto ค่าความเสถียรแสดง `L+` ไม่หักคะแนนแม้ไม่เปิดจอย), ทดสอบกด `Cross` ยกเลิกกลางคัน และปรับจูนระยะ `MoveDegs` ใน `src/auto_block_code.cpp` ให้ตรงกับพื้นสนามแข่งจริง
