# รายงานการประเมินความพร้อมของเฟิร์มแวร์ `matrix_R4` สำหรับการแข่งขัน Matrix MARC

- **โปรเจกต์**: `matrix_R4` (`Arduino UNO R4 WiFi` + `MATRIX Mini R4 Carrier Shield`)
- **ฐานโค้ดที่ตรวจสอบ (Git Baseline)**: Branch `main` Commit `cd6bafd` (`Refactor main.cpp into modular helpers and control stages`) ตรงกับ `origin/main` (พร้อมสถานะ `M README.md` ที่มีอยู่ก่อนการประเมิน SHA256 `1311356C3CBD5A3B0275DC447898A4CD12BD131F88C07740A8DD84D779F4BC06`) `[Verified: Build/Test Run]`
- **ขอบเขตการแก้ไขโค้ด (R2 Compliance)**: **ไม่มีการแก้ไขซอร์สโค้ดเดิม ไฟล์ตั้งค่า ชุดทดสอบ หรือ `README.md` ใดๆ** จัดทำเฉพาะรายงานฉบับนี้เพียงไฟล์เดียว `[Verified: Build/Test Run]`
- **คะแนนความพร้อมรวม (Overall Readiness Score)**: **`78.50 / 100`**
- **ระดับความพร้อม (Readiness Tier)**: **`Firmware-Ready (Track Calibration / Field Validation Required)`** — *เฟิร์มแวร์พร้อมใช้งานในระดับวิศวกรรมซอฟต์แวร์ แต่ต้องปรับจูนและทดสอบบนหุ่นยนต์/สนามจริงก่อนลงแข่งขัน*

---

## 1. บทสรุปผู้บริหาร (Executive Summary)

### 1.1 ภาพรวมผลคะแนนและระดับความพร้อม
จากการตรวจสอบโค้ดรายบรรทัดทั้ง 8 โมดูลใน `src/`, 8 เฮดเดอร์ใน `include/`, ชุดทดสอบ `test/test_robot_logic/test_main.cpp`, ไฟล์ตั้งค่า `platformio.ini` และไลบรารี `MatrixMiniR4 @ 1.2.6` ร่วมกับการรันคอมไพล์จริงบนเป้าหมาย `uno_r4_wifi`, การรัน Unit Test บน `native` และการวิเคราะห์เชิงสถิต (`cppcheck`) พบว่าโปรเจกต์ `matrix_R4` ได้คะแนนความพร้อมรวม **`78.50 / 100`** อยู่ในระดับ **`Firmware-Ready (Track Calibration / Field Validation Required)`**

| องค์ประกอบหลัก | ผลลัพธ์ | ป้ายสถานะการตรวจสอบ |
|---|---|---|
| **คอมไพล์เฟิร์มแวร์จริง (`uno_r4_wifi`)** | ผ่าน (`SUCCESS`, 0 warnings/errors ภายใต้ `-Werror`), RAM `43.9%`, Flash `41.7%` | `[Verified: Build/Test Run]` |
| **ชุดทดสอบอัตโนมัติ (`native` Unity)** | ผ่าน **`36 / 36`** เคส (0 Failures, 0 Ignored) | `[Verified: Build/Test Run]` |
| **การวิเคราะห์โค้ดเชิงสถิต (`pio check`)** | ผ่าน (`0 HIGH`, `1 MEDIUM` false-positive จาก `if constexpr` ใน `src/`) | `[Verified: Build/Test Run]` |
| **สถาปัตยกรรม Teleop, Failsafe & Auto Supervisor** | แข็งแรงระดับแข่งขัน (40 Hz scheduler, 5-stage kinodynamics, 1.4 ms SPI, 3s WDT) | `[Verified: Static Code Inspection]` |
| **ตรรกะภารกิจ Autonomous ปัจจุบัน (`auto_block_code.cpp`)** | ใช้ `MoveDegs` แบบ Open-Loop + `delay()` รวม `15.5 s` ยังไม่ใช้ Gyro/Sensor ป้อนกลับ | `[Verified: Static Code Inspection]` |
| **พฤติกรรมบนฮาร์ดแวร์และสนามแข่งจริง (HIL)** | ยังไม่ได้เชื่อมต่อบอร์ด UNO R4 WiFi, STM32, มอเตอร์, เซอร์โว และจอย PS2/MJ2 จริง | `[Unverified: Requires Physical Hardware/Field Test]` |

### 1.2 จุดแข็งหลัก (Key Strengths)
1. **ระบบควบคุมเวลาจริงและไดรเวอร์ PS2 ที่กำหนดเวลาแน่นอน (Deterministic Real-Time Control)**: วงรอบควบคุม `40 Hz` (`kControlPeriodMs = 25UL`) แบบ non-blocking พร้อม resync เมื่อเลยคาบเวลา (`src/main.cpp:1008-1017`), ไดรเวอร์บิตแบง SPI ระดับรีจิสเตอร์ RA4M1 ที่ใช้เวลา ~`1.4 ms`/เฟรม (`src/ps2_driver.cpp:10-236`) แทนที่ `PS2X::read_gamepad()` แบบ blocking เดิม, การตรวจสอบแพ็กเก็ต 5 ขั้น (`isPs2FrameValid`, `src/robot_logic.cpp:637-671`) และตัวจับเวลา Watchdog 3 วินาที (`src/main.cpp:994`) `[Verified: Static Code Inspection]`
2. **ระบบขับเคลื่อน Kinodynamic 5 ขั้นระดับแข่งขัน**: `computeKinodynamicDrive()` (`src/robot_logic.cpp:387-480`) รวม Cubic Expo Shaping (`kThrottleExpoPct` 35% และโหมด Precision 55%/60%), Centripetal Traction Circle (`|v * w| / 100 <= 52`), CoM Anti-Tip Scaling ตามมุมแขน `RC2`, Balanced Trim (`±15%`), Coulomb Stiction Feedforward (`12%`), Coupled 2-DOF Slew Limiter, โหมด Precision 55% (`Select`), Reverse-Front (`R3`) และ Micro-Turn (`D-Pad L/R`) `[Verified: Build/Test Run]` ส่วน `kTurnExpoPct` (45%), `kTurnMaxRatioPct` (80%, `line 443` เมื่อขับพร้อมเลี้ยว) และ Smart H-Bridge Hold-Brake (`src/minir4_io.cpp:437-498`) `[Verified: Static Code Inspection]`
3. **เกราะป้องกันโหมด Autonomous สำหรับ MATRIXblock R4 (`AutoMode` + Proxy Layer)**: ครอบ `MiniR4` และ `Serial` ด้วย Proxy ที่ปลอดภัย (`src/auto_block_code.cpp:49-506`), สามารถยกเลิกกลางคันได้ทันทีผ่าน PS2 `Cross`, `BTN_UP`, `BTN_DOWN`, Serial `'x'`/`'!'`, จำกัดเวลาสูงสุด `60 s` (`kAutoMaxDurationMs`), ตรวจจับล้อติดขัดจาก Encoder (`checkDriveStall`, `src/auto_mode.cpp:105-157`) และคืนค่าพอร์ต/เบรก/โหมดแมนนวลอย่างปลอดภัย (`restoreAfterAuto`, `src/minir4_io.cpp:547-585`) `[Verified: Static Code Inspection]`
4. **ความทนทานของฮาร์ดแวร์และการวินิจฉัยหน้างาน (Pit & Field Diagnostics)**: การกู้คืนบัส `Wire1` + ส่งข้อมูล OLED SSD1306 แบบแบ่งก้อน 64 ไบต์และลดความเร็วอัตโนมัติ `400 kHz -> 100 kHz` (`src/oled_ui.cpp:80-221`), ระบบฟื้นฟู STM32 `MMLower` หลังไฟตก/UART หลุด (`src/minir4_io.cpp:372-420`), ดัชนีเสถียรภาพเรียลไทม์ `StabilityMonitor` (`0..100%`), EEPROM พร้อม CRC-16-CCITT ที่ออฟเซ็ตปลอดภัย `0x80` และทางลัดเข้า Bootloader (`BTN_UP + BTN_DOWN` 2.5s หรือ `'u'`) `[Verified: Static Code Inspection]`

### 1.3 ความเสี่ยงและตัวขวางหลักก่อนลงแข่ง (Key Blockers & Risks)
1. **[High] โค้ด Autonomous ปัจจุบันเดินแบบ Open-Loop Dead-Reckoning และมี `delay()` รวม 15.5 วินาที (`src/auto_block_code.cpp:525-567`)**: ใช้คำสั่ง `DriveDC.MoveDegs(±50, ±50, ...)` ล้วนจำนวน 11 ครั้ง และ `delay(500/1000)` ถึง 20 ครั้ง (รวม 15.5 วินาที ไม่นับดีเลย์เบรกของไลบรารีอีก `11 × 110 ms = 1.21 s`, รวมเป็น `16.71 s`) โดยไม่มีการปิดวงด้วย `MoveSyncDegs`/`MoveGyroDegs`/`TurnGyro` หรืออ่านเซนเซอร์สนามใดๆ (แม้บรรทัดที่ 521 จะเรียก `MiniR4.I2C1.MJ5.begin()` ไว้แต่ไม่ได้ใช้งานต่อ) ทำให้ความแม่นยำไวต่อแรงดันแบตเตอรี่และความลื่นของพื้นสนาม `[Verified: Static Code Inspection]`
2. **[Medium] ปิด `kEnableBatterySagThrottle = false` ขณะที่ยังเปิดบูสต์ชดเชยแรงดันสูงสุด `115%` (`include/config.h:144`, `src/robot_logic.cpp:997-1004`)**: เมื่อแบตเตอรี่ตกถึงระดับ `Critical`/`Emergency` (`<= 6.8V`/`<= 6.4V` หรือ `<= 76%`/`69%` **ของแรงดันฐาน (`baselineMv_`)** ตามค่า `kBattCritSagPct`/`kBattEmergSagPct` ใน `include/config.h:137-138` ซึ่งเป็นคนละค่ากับ % SoC `batteryPercent()` บนจอ OLED) เฟิร์มแวร์ยังอนุญาตให้มอเตอร์ใช้กำลัง `100%` และคูณชดเชยได้ถึง `115%` เพิ่มความเสี่ยงที่บอร์ด STM32 หรือ UNO R4 จะไฟตกจนรีเซ็ตกลางแมตช์ `[Verified: Static Code Inspection]`
3. **[Medium] ข้อจำกัดเชิงปฏิบัติการและความปลอดภัยอีก 4 จุด**: (ก) `BTN_DOWN` E-Stop ทำงานเมื่อ**ปล่อยปุ่ม** (`outShortRelease`) ไม่ใช่ทันทีที่กดค้าง (`src/minir4_io.cpp:247-255`), (ข) `MiniR4.DriveDC.begin(1, 2, false, true)` ใน `src/auto_block_code.cpp:522` ฮาร์ดโค้ดทิศทางมอเตอร์ทับค่า EEPROM `f1`/`f2` และไม่ตรวจค่า error return `0x09U`, (ค) การยกเลิก Auto ใช้ `longjmp` (`src/auto_mode.cpp:92, 256`) ซึ่งจะเกิด Undefined Behavior หากภายหลังใส่ตัวแปร RAII/`String` ใน `auto_block_code.cpp`, และ (ง) ไลบรารี `MMLower` บล็อกรอ UART สูงสุด `100 ms`/ครั้ง (`MMLower.cpp:2207-2278`) ซึ่งอาจทำให้ลูปหน่วง `~0.9–1.1 s` หากสาย STM32 หลุดกลางคัน หรือค้างใน `while` ของ `MatrixMiniR4::begin()` ก่อนเริ่ม WDT `[Verified: Static Code Inspection]`

### 1.4 ข้อจำกัดสำคัญของการประเมินแบบ Static + Native Host (Static vs. HIL Caveat)
การประเมินนี้ดำเนินการแบบ **Static Code Inspection + Cross-Compilation (`uno_r4_wifi`) + Host Unit Testing (`native`)** โดย **ไม่ได้ทดสอบบนฮาร์ดแวร์จริง (Hardware-in-the-Loop: HIL)** เนื่องจาก `[env:native]` ทดสอบเฉพาะ `robot_logic.cpp` และ `serial_control.cpp` (`platformio.ini:39-41`) และ `[env:uno_r4_wifi]` ตั้งค่า `test_ignore = *` (`platformio.ini:27`) ดังนั้นพฤติกรรมทางกายภาพของบอร์ด STM32 (`MMLower` v6.0+), ความแรงสัญญาณ 2.4 GHz ของจอย PS2/MJ2 ในสนามที่มีคลื่นรบกวน, แรงเสียดทานล้อ/แคมมุมเซอร์โว `RC1..RC4` และความแม่นยำทางเรขาคณิตของเส้นทาง Autonomous ยังต้องผ่าน **รายการตรวจสอบหน้างาน (Pit/Field HIL Checklist)** ในหัวข้อที่ 5 ก่อนการแข่งขันจริง `[Unverified: Requires Physical Hardware/Field Test]`

---

## 2. บริบทการแข่งขัน Matrix MARC และระเบียบวิธีประเมิน (Competition Context & Evaluation Methodology)

### 2.1 บริบทของเวทีแข่งขัน Matrix MARC และแพลตฟอร์ม MATRIX Mini R4
**Matrix MARC (Master AI Robot Cup / MATRIX Alliance Robot Competition)** เป็นการแข่งขันหุ่นยนต์ที่ใช้ระบบนิเวศฮาร์ดแวร์และซอฟต์แวร์ของ MATRIX Robotics (เช่น รุ่น Task Master, League of Mecha-Clash, Rising Pioneers) ซึ่งโดยทั่วไปกำหนดให้ใช้คอนโทรลเลอร์หลักตระกูล MATRIX เพียง 1 ชุดต่อหุ่นยนต์ (ในที่นี้คือ **MATRIX Mini R4** ที่ประกอบด้วย **Arduino UNO R4 WiFi (RA4M1 @ 48 MHz)** สื่อสารกับ **STM32 Co-processor (`MMLower`)** ผ่าน UART D8/D9 @ 57,600 baud) และแบ่งการแข่งขันออกเป็น **2 ช่วงหลัก (Dual-Phase Match)**:
1. **ช่วง Autonomous Phase**: หุ่นยนต์ทำงานอัตโนมัติตามคำสั่งที่ตั้งไว้ (มักเริ่มต้นจากการกดปุ่ม/สัญญาณเริ่มรอบโดยห้ามบังคับ) ต้องการการออกตัวที่เชื่อถือได้ ความแม่นยำในการเคลื่อนที่/หยิบวาง และระบบตัดการทำงานฉุกเฉิน/กันมอเตอร์ไหม้
2. **ช่วง Teleop / Remote Control Phase**: ผู้ขับควบคุมหุ่นยนต์ด้วยจอยไร้สาย 2.4 GHz (MATRIX MJ2 / PS2) ต้องการการตอบสนองที่แม่นยำ ความเสถียรของสัญญาณ RF ท่ามกลางสัญญาณรบกวนในฮอลล์แข่ง การควบคุมแขนกล/มือจับ `RC1..RC4` ที่รวดเร็วและแม่นยำ การจัดการพลังงานแบตเตอรี่ และระบบ Failsafe/Recovery ที่ไม่ทำให้หุ่นยนต์ค้างหรือเดินต่อเมื่อสัญญาณขาด

### 2.2 สเกลการให้คะแนนระดับหมวด (0–10 Scoring Anchors)
แต่ละหมวดจากทั้งหมด 7 หมวดถูกประเมินบนสเกล **`0.0 – 10.0`** โดยใช้เกณฑ์อ้างอิงมาตรฐานดังนี้:

| คะแนน (0–10) | ความหมายเชิงวิศวกรรม (Anchor Definition) |
| :---: | :--- |
| **`0`** | **Missing / Broken** — ไม่มีฟังก์ชันงาน หรือคอมไพล์ไม่ผ่าน/ผิดพลาดร้ายแรงจนใช้งานไม่ได้ |
| **`2`** | **Stub / Major Defects** — มีเพียงโครงร่าง หรือมีบั๊กร้ายแรงที่ทำให้หุ่นยนต์ควบคุมไม่ได้/เสี่ยงพังทันที |
| **`5`** | **Partial / Needs Work** — ทำงานได้บางส่วนในแล็บ แต่ยังขาดความเสถียร เซนเซอร์ หรือการป้องกันที่จำเป็นสำหรับสนามแข่ง |
| **`8`** | **Solid / Competition-Capable** — โครงสร้างและการทำงานแข็งแรง พร้อมลงแข่งได้จริง โดยมีข้อควรระวังหรือจุดปรับจูนเล็กน้อยที่ระบุชัดเจน |
| **`10`** | **Verified Match-Hardened** — สมบูรณ์ครบถ้วน ผ่านทั้ง Unit/Integration Test และผ่านการทดสอบบนสนามแข่งจริง (HIL) แล้ว |

### 2.3 เกณฑ์ถ่วงน้ำหนัก 7 หมวดรวม 100% และสูตรคำนวณ (Weighted Rubric)
คะแนนถ่วงน้ำหนักของแต่ละหมวดคำนวณจากสูตร:
$$\text{Weighted Points} = \left(\frac{\text{Category Score}}{10}\right) \times \text{Weight (\%)} $$
และคะแนนรวมคือผลบวกของทั้ง 7 หมวด (`Total = Sum of Weighted Points`, เต็ม `100.00` คะแนน):

| หมวดที่ | ชื่อหมวดการประเมิน | น้ำหนัก (%) | คะแนน (0–10) | คะแนนถ่วงน้ำหนัก | การคำนวณ |
| :-: | :--- | :---: | :---: | :---: | :--- |
| **1** | **Build, Static Hygiene & Automated Tests** | `15%` | `8.5` | **`12.75`** | `(8.5 / 10) × 15 = 12.75` |
| **2** | **Teleop Drive & Kinodynamic Precision** | `15%` | `9.0` | **`13.50`** | `(9.0 / 10) × 15 = 13.50` |
| **3** | **Manipulator & Servo Actuation (`RC1..RC4`)** | `10%` | `8.0` | **`8.00`** | `(8.0 / 10) × 10 = 8.00` |
| **4** | **Autonomous Mode & MARC Match Readiness** | `20%` | `6.0` | **`12.00`** | `(6.0 / 10) × 20 = 12.00` |
| **5** | **Wireless Link, Coprocessor & Failsafe Reliability** | `15%` | `8.0` | **`12.00`** | `(8.0 / 10) × 15 = 12.00` |
| **6** | **Power Management, Stall Safety & Persistence** | `10%` | `7.5` | **`7.50`** | `(7.5 / 10) × 10 = 7.50` |
| **7** | **Competition Operations, Diagnostics & Maintainability** | `15%` | `8.5` | **`12.75`** | `(8.5 / 10) × 15 = 12.75` |
| | **รวมทั้งหมด (Total)** | **`100%`** | — | **`78.50 / 100`** | `12.75 + 13.50 + 8.00 + 12.00 + 12.00 + 7.50 + 12.75 = 78.50` |

### 2.4 ระดับความพร้อม (Readiness Rating Tiers)
- **`90 – 100`**: **Match-Ready (Field-Verified)** — พร้อมลงสนามแข่งเต็มรูปแบบ ผ่านการทดสอบทั้งโค้ดและสนามจริง
- **`75 – 89`**: **Firmware-Ready (Track Calibration / Field Validation Required)** — ซอฟต์แวร์แข็งแรงพร้อมใช้ แต่ต้องปรับจูนและทดสอบ HIL บนสนามจริง **(ระดับปัจจุบันของ `matrix_R4`: `78.50 / 100`)**
- **`60 – 74`**: **Conditional Readiness (Notable Autonomous/Reliability Gaps)** — พอใช้แข่งแบบมีเงื่อนไข มีจุดอ่อนสำคัญใน Auto หรือความเสถียรที่ควรแก้ก่อน
- **`< 60`**: **Not Ready for Competition** — ยังไม่พร้อมลงแข่ง ต้องแก้ไขโครงสร้างหลักหรือบั๊กร้ายแรงก่อน

### 2.5 ป้ายกำกับหลักฐานการตรวจสอบ (Verification Tags)
ทุกข้อเท็จจริงและคะแนนในรายงานฉบับนี้กำกับด้วยป้ายสถานะมาตรฐาน 3 ประเภท เพื่อแยกความจริงที่ทดสอบรันแล้วออกจากการอ่านโค้ดและสิ่งที่ต้องทดสอบบนฮาร์ดแวร์จริง:
- `[Verified: Build/Test Run]` — ยืนยันจากการรันคำสั่ง `pio run`, `pio test`, `pio check` หรือ `git` จริงในสภาพแวดล้อมโปรเจกต์
- `[Verified: Static Code Inspection]` — ยืนยันจากการอ่านและตรวจสอบไฟล์ซอร์สโค้ด/ไลบรารีจริงตาม `file:line` ที่อ้างอิง
- `[Unverified: Requires Physical Hardware/Field Test]` — ยังไม่สามารถยืนยันด้วยซอฟต์แวร์บนโฮสต์ได้ ต้องทดสอบบนบอร์ด หุ่นยนต์ และสนามแข่งจริง

---

## 3. ผลการรันตรวจสอบจริง (Live Verification Results)

คำสั่งทั้งหมดรันบน `C:\Users\MB523W\OneDrive\Documents\PlatformIO\Projects\matrix_R4` โดยใช้ PlatformIO Core (`C:\Users\MB523W\.platformio\penv\Scripts\pio.exe`) โดยไม่แก้ไขไฟล์เดิมของโปรเจกต์

### 3.1 การคอมไพล์เฟิร์มแวร์เป้าหมายจริง (`pio run -e uno_r4_wifi`) `[Verified: Build/Test Run]`
- **คำสั่ง**: `& "C:\Users\MB523W\.platformio\penv\Scripts\pio.exe" run -e uno_r4_wifi`
- **สถานะ**: `SUCCESS` (Exit code `0`, ใช้เวลา `2.53s`), **0 Warnings / 0 Errors** ภายใต้ `-Wall -Wextra -Werror` (`platformio.ini:22-25`)
- **การใช้หน่วยความจำ**:
  - **RAM**: **`43.9%`** (`used 14380 bytes from 32768 bytes`) — เหลือพื้นที่ Heap/Stack อีก `18,388 bytes`
  - **Flash**: **`41.7%`** (`used 109232 bytes from 262144 bytes`) — เหลือพื้นที่โปรแกรมอีก `152,912 bytes`

```text
Processing uno_r4_wifi (platform: renesas-ra; board: uno_r4_wifi; framework: arduino)
--------------------------------------------------------------------------------
CONFIGURATION: https://docs.platformio.org/page/boards/renesas-ra/uno_r4_wifi.html
PLATFORM: Renesas RA (1.9.0) > Arduino Uno R4 WiFi
HARDWARE: RA4M1 48MHz, 32KB RAM, 256KB Flash
PACKAGES: 
 - framework-arduinorenesas-uno @ 1.6.0 
 - tool-bossac @ 1.10901.0 (1.9.1) 
 - toolchain-gccarmnoneeabi @ 1.70201.0 (7.2.1)
Dependency Graph
|-- MatrixMiniR4 @ 1.2.6
|-- EEPROM @ 1.0
|-- SPI
|-- Wire
|-- WDT @ 0.0.1
Building in release mode
Checking size .pio\build\uno_r4_wifi\firmware.elf
RAM:   [====      ]  43.9% (used 14380 bytes from 32768 bytes)
Flash: [====      ]  41.7% (used 109232 bytes from 262144 bytes)
========================= [SUCCESS] Took 2.53 seconds =========================
```

### 3.2 การรันชุดทดสอบอัตโนมัติบนโฮสต์ (`pio test -e native -v`) `[Verified: Build/Test Run]`
- **คำสั่ง**: `& "C:\Users\MB523W\.platformio\penv\Scripts\pio.exe" test -e native -v`
- **สถานะ**: `PASSED` (Exit code `0`, ใช้เวลา `1.271s`), ผ่านครบ **`36 Tests 0 Failures 0 Ignored`** ใน 10 หมวดหมู่ (`test/test_robot_logic/test_main.cpp:1755-1808`)

```text
Testing...
test\test_robot_logic\test_main.cpp:1755:test_axis_deadzone_and_extremes:PASS
test\test_robot_logic\test_main.cpp:1756:test_differential_drive_zero_speed_boundary:PASS
test\test_robot_logic\test_main.cpp:1757:test_differential_drive_saturation_and_ratio_preservation:PASS
test\test_robot_logic\test_main.cpp:1758:test_differential_drive_pivot_turn:PASS
test\test_robot_logic\test_main.cpp:1761:test_asymmetric_slew_limits_and_step_delta:PASS
test\test_robot_logic\test_main.cpp:1762:test_asymmetric_slew_zero_crossing_trajectory:PASS
test\test_robot_logic\test_main.cpp:1765:test_ps2_decode_and_5stage_validation:PASS
test\test_robot_logic\test_main.cpp:1766:test_failsafe_burst_glitch_absorption_and_trip:PASS
test\test_robot_logic\test_main.cpp:1767:test_failsafe_persistent_loss_disconnect_and_reinit_cooldown:PASS
test\test_robot_logic\test_main.cpp:1768:test_servo_bounds_and_ps2_presets:PASS
test\test_robot_logic\test_main.cpp:1769:test_square_push_ground_toggle_and_l1_r1_override:PASS
test\test_robot_logic\test_main.cpp:1772:test_battery_stabilizer_multiprofile_baselines:PASS
test\test_robot_logic\test_main.cpp:1773:test_battery_stabilizer_progressive_sag_tiers:PASS
test\test_robot_logic\test_main.cpp:1774:test_battery_stabilizer_hysteresis_and_timed_recovery:PASS
test\test_robot_logic\test_main.cpp:1775:test_battery_stabilizer_regen_spike_protection:PASS
test\test_robot_logic\test_main.cpp:1778:test_stability_monitor_individual_penalties:PASS
test\test_robot_logic\test_main.cpp:1779:test_stability_monitor_saturation_and_deterministic_recovery:PASS
test\test_robot_logic\test_main.cpp:1782:test_clamp_long_and_adversarial_boundaries:PASS
test\test_robot_logic\test_main.cpp:1785:test_serial_wasd_hold_expiry_and_stop:PASS
test\test_robot_logic\test_main.cpp:1786:test_serial_direct_motor_bounds_and_garbage_rejection:PASS
test\test_robot_logic\test_main.cpp:1787:test_serial_servo_commands_and_flip_exact_match:PASS
test\test_robot_logic\test_main.cpp:1788:test_serial_speed_batching_timeout_and_estop_interrupt:PASS
test\test_robot_logic\test_main.cpp:1791:test_auto_mode_triangle_start_and_cross_exit:PASS
test\test_robot_logic\test_main.cpp:1794:test_cubic_expo_shaping_symmetry_and_bounds:PASS
test\test_robot_logic\test_main.cpp:1795:test_centripetal_traction_limiter_and_pivot_authority:PASS
test\test_robot_logic\test_main.cpp:1796:test_com_elevation_dynamic_scaling:PASS
test\test_robot_logic\test_main.cpp:1797:test_2dof_kinodynamic_slew_and_coulomb_stiction:PASS
test\test_robot_logic\test_main.cpp:1798:test_kinodynamic_drive_reverse_front_and_dpad_micro_turn:PASS
test\test_robot_logic\test_main.cpp:1799:test_precision_mode_full_stick_scaling_and_micro_turn:PASS
test\test_robot_logic\test_main.cpp:1800:test_circle_pose_preset_and_serial_competition_shortcuts:PASS
test\test_robot_logic\test_main.cpp:1803:test_rc2_smooth_servo_slew:PASS
test\test_robot_logic\test_main.cpp:1804:test_motor_balance_trim_and_stepping:PASS
test\test_robot_logic\test_main.cpp:1805:test_persistent_config_crc16_pack_unpack:PASS
test\test_robot_logic\test_main.cpp:1806:test_auto_encoder_stall_progress_logic:PASS
test\test_robot_logic\test_main.cpp:1807:test_serial_phase2_trim_and_rc2_smooth_keys:PASS
test\test_robot_logic\test_main.cpp:1808:test_servo_direction_inversion:PASS

-----------------------
36 Tests 0 Failures 0 Ignored 
OK
================= 36 test cases: 36 succeeded in 00:00:01.271 =================
```

### 3.3 การวิเคราะห์โค้ดเชิงสถิตด้วย `cppcheck` (`pio check`) `[Verified: Build/Test Run]`
1. **`pio check -e uno_r4_wifi --skip-packages`**: Exit code `0` (`PASSED` in `4.224s`)
   - โฟลเดอร์ `src/`: **`0 HIGH, 1 MEDIUM, 78 LOW`**
   - ไลบรารี `MatrixMiniR4`: **`4 HIGH, 226 MEDIUM, 144 LOW`** (รวม `MiniR4SmartCamReader.h:88 [missingReturn]` ที่ไม่ได้ใช้งานในโปรเจกต์)
2. **`pio check -e native --skip-packages`**: Exit code `0` (`PASSED` in `1.504s`)
   - โฟลเดอร์ `src/`: **`0 HIGH, 1 MEDIUM, 74 LOW`**
3. **รายละเอียดข้อพบใน `src/`** (ไม่นับ `[unusedFunction]` ที่เกิดจาก `pio check` วิเคราะห์แยกจาก `test/`):
   - **`MEDIUM` (1 รายการ)**: `src\auto_block_code.cpp:133: [medium:warning] Comparison of a boolean expression with an integer other than 0 or 1. [compareBoolExpressionWithInt]` — เกิดจาก `cppcheck` แปลผล C++17 `if constexpr (ID == static_cast<uint8_t>(-1))` ผิดพลาด (False Positive)
   - **`LOW` (style)**:
     - `src\ps2_driver.cpp:19`: `constVariable` (`kPorts` ควรประกาศเป็น `const`)
     - `src\oled_ui.cpp:255`: `redundantInitialization` ของตัวแปร `c1`
     - `src\robot_logic.cpp:948-949`: `redundantInitialization` ของ `emptyMv` และ `fullMv`
     - `src\robot_logic.cpp:847, 858, 869, 1048`: `knownConditionTrueFalse` (ผลจากขอบเขตวิเคราะห์ที่ไม่ได้รวม `test_main.cpp`)

```text
Component                                                   HIGH    MEDIUM    LOW
---------------------------------------------------------  ------  --------  -----
.pio\libdeps\uno_r4_wifi\MatrixMiniR4\src\Modules            0        50      120
.pio\libdeps\uno_r4_wifi\MatrixMiniR4\src\Modules\Sensors    4       176      24
src                                                          0        1       78

Total                                                        4       227      222
```

### 3.4 หมายเหตุพฤติกรรม `pio run -e native` และสถานะไฟล์ก่อน/หลังตรวจสอบ `[Verified: Build/Test Run]`
- **หมายเหตุ `pio run -e native`**: เมื่อรัน `pio run -e native` จะคอมไพล์ `robot_logic.o` และ `serial_control.o` ผ่าน แต่ล้มเหลวในขั้นตอน Link (`Exit code 1`, `undefined reference to 'WinMain@16'`) **ซึ่งเป็นพฤติกรรมที่ออกแบบไว้ถูกต้อง** เนื่องจาก `[env:native]` มี `build_src_filter = +<robot_logic.cpp> +<serial_control.cpp>` และ `test_build_src = yes` (`platformio.ini:39-42`) เพื่อให้ `main()` มาจาก `test/test_robot_logic/test_main.cpp` เฉพาะตอนรัน `pio test -e native` เท่านั้น
- **การรักษาสภาพโค้ดเดิม (R2 Preservation)**:
  - `git status -sb` แสดงเฉพาะ `* main...origin/main` และ `M README.md` ที่มีอยู่เดิมก่อนการประเมิน (`+52/-43`, SHA256 `1311356C3CBD5A3B0275DC447898A4CD12BD131F88C07740A8DD84D779F4BC06`)
  - ไฟล์ใน `src/`, `include/`, `test/`, `platformio.ini`, `.gitignore` และ `.vscode/*` มีการแก้ไข **0 บรรทัด**

---

## 4. การประเมินรายหมวดพร้อมหลักฐานอ้างอิง (Category-by-Category Detailed Assessment & Evidence Matrix)

### หมวดที่ 1: Build, Static Hygiene & Automated Tests (`15%`) — คะแนน **`8.5 / 10`** (`12.75 / 15.00` คะแนน)
- **เกณฑ์ย่อย**: ความสะอาดของการคอมไพล์ (`-Werror`), พื้นที่หน่วยความจำ RAM/Flash, คุณภาพโค้ดเชิงสถิต (`cppcheck`/ไม่มี dynamic allocation) และความครอบคลุมของ Unit Test
- **หลักฐานยืนยัน (Evidence)**:
  - `platformio.ini:22-25` และ `33-37` บังคับ `-Wall -Wextra -Werror` ทั้งบน `uno_r4_wifi` (`src/`) และ `native`; การรันจริงได้ 0 warnings/0 errors `[Verified: Build/Test Run]`
  - พื้นที่ใช้งานบน RA4M1 เหลือเฟือ: RAM `43.9%` (`14,380 / 32,768 B`), Flash `41.7%` (`109,232 / 262,144 B`) `[Verified: Build/Test Run]`
  - `pio test -e native -v` ผ่าน `36 / 36` เคส ครอบคลุม 10 กลุ่ม (Kinematics, Asymmetric Slew, PS2 Validation/Failsafe, Battery Sag/Hysteresis, StabilityMonitor, Clamping, SerialControl, Auto Start/Abort, Kinodynamic 5-Stage Models, EEPROM CRC-16/Trim/Smooth Servo/Stall Helper) `[Verified: Build/Test Run]`
  - ไม่มีการใช้ `String`, `malloc`, หรือ `new` ในโค้ดโปรเจกต์ `src/`/`include/`; การคำนวณ `snprintf` มีขอบเขตบัฟเฟอร์ (`line1..4[24]` ใน `src/oled_ui.cpp:308-311`) และจัดการ `millis()` rollover ด้วย `uint32_t` ทุกจุด (`int32_t` diff ใน `src/serial_control.cpp:599`) `[Verified: Static Code Inspection]`
- **เหตุผลการหักคะแนน (`-1.5`)**:
  - `[env:native]` คอมไพล์เฉพาะ `robot_logic.cpp` และ `serial_control.cpp` (`platformio.ini:39-41`) ทำให้ `main.cpp`, `minir4_io.cpp`, `ps2_driver.cpp`, `oled_ui.cpp`, `auto_mode.cpp` และ `auto_block_code.cpp` ยังไม่มี Mock Unit Test หรือ HIL Test (`test_ignore = *` ที่ `platformio.ini:27`) `(-1.0)` `[Verified: Static Code Inspection]`
  - `src/auto_block_code.cpp:23-32` ปิดเตือนคอมไพเลอร์ 9 รายการ (รวม `-Wreturn-type`) ภายใต้ `#pragma GCC diagnostic push/pop` เพื่อรองรับโค้ดจาก MATRIXblock `(-0.25)` `[Verified: Static Code Inspection]`
  - พบคอมเมนต์ล้าสมัย 3 จุดใน `test/test_robot_logic/test_main.cpp:485` (ระบุ `(180, 180)`), `517` (`L1 (70, 70)`), และ `1266` (`15%` ขณะที่ `config.h:158` เป็น `20`) แม้ตัว test assertions จะอ้างค่า `Config::` ถูกต้องและผ่านก็ตาม `(-0.25)` `[Verified: Static Code Inspection]`

### หมวดที่ 2: Teleop Drive & Kinodynamic Precision (`15%`) — คะแนน **`9.0 / 10`** (`13.50 / 15.00` คะแนน)
- **เกณฑ์ย่อย**: ความแม่นยำในการขับเคลื่อน, การกันล้อฟรี/กันคว่ำ, ความลื่นไหลของแรงบิด, โหมดช้า (Precision), การปรับสมดุลล้อ และระบบเบรก
- **หลักฐานยืนยัน (Evidence)**:
  - ลูปควบคุม `40 Hz` (`kControlPeriodMs = 25UL`, `include/config.h:22`) ใน `src/main.cpp:1008-1017` ป้องกัน burst catch-up เมื่อเกิด stall (`dtMs >= 50 ms`) `[Verified: Static Code Inspection]`
  - `computeAxisWithDeadzone` (`src/robot_logic.cpp:58-82`) สร้าง deadband `[109..146]` (`kStickDeadzoneRaw = 18`, `include/config.h:79`) และ `desaturateAndClampDrive` (`src/robot_logic.cpp:11-27`) รักษาอัตราส่วนเลี้ยวเมื่อผลรวมเกิน `±100%` `[Verified: Build/Test Run]`
  - ไปป์ไลน์ 5 ขั้นใน `computeKinodynamicDrive` (`src/robot_logic.cpp:387-480`):
    1. `applyCubicExpo` (`35%` throttle ปกติ, `55%/60%` Precision) + `R3` Reverse-Front (`line 425`) + D-Pad L/R Micro-Turn (`±26%` / `±18%`, `lines 432-436`) `[Verified: Build/Test Run]`, ส่วน `45%` normal turn expo (`kTurnExpoPct`) และ `kTurnMaxRatioPct = 80%` (`line 443`, ยังไม่ถูกทดสอบเชิงพฤติกรรมเมื่อ `throttle != 0` และ `turn != 0`) `[Verified: Static Code Inspection]`
    2. `limitCentripetalTurn` (`lines 173-193`) จำกัด `|v * w| / 100 <= 52` โดยคงการหมุนอยู่กับที่ (pivot turn) ไว้ 100% `[Verified: Build/Test Run]`
    3. `computeCoMDynamicScale` (`lines 195-206`) ลดอัตราเร่ง/ค่าแรงเหวี่ยงจาก `100% -> 65%` เมื่อยกแขน `RC2` ขึ้น (`140° -> 0°`) `[Verified: Build/Test Run]`
    4. `applyMotorBalanceTrim` (`lines 267-287`, `±15%`) + `applyCoulombStiction` (`lines 241-257`, `kMotorStictionMinPower = 12%`) `[Verified: Build/Test Run]`
    5. `apply2DofKinodynamicSlew` (`lines 208-239`) ในพิกัด `(V2=L+R, W2=L-R)` เพื่อรักษาความโค้งของเส้นทางและบังคับผ่าน `0` ก่อนกลับทิศ (`lines 100-103`) `[Verified: Build/Test Run]`
  - `MiniR4IO::applyMotors` (`src/minir4_io.cpp:437-498`) สั่ง `MiniR4.M1/M2.setBrake(true)` อัตโนมัติเมื่อเป้าหมายเป็น `(0,0)` และปลดเบรกเมื่อออกตัว `[Verified: Static Code Inspection]`
- **เหตุผลการหักคะแนน (`-1.0`)**:
  - ค่าพารามิเตอร์ทางฟิสิกส์ (`kMaxCentripetalLatAccel = 52`, `kMotorStictionMinPower = 12`, `kCoMHighMinScalePct = 65`) และการตอบสนองการขับแบบ Open-Loop PWM บนหุ่นยนต์และพื้นสนามจริงยังต้องผ่านการจูนหน้างาน `(-1.0)` `[Unverified: Requires Physical Hardware/Field Test]`

### หมวดที่ 3: Manipulator & Servo Actuation (`RC1..RC4`) (`10%`) — คะแนน **`8.0 / 10`** (`8.00 / 10.00` คะแนน)
- **เกณฑ์ย่อย**: การควบคุมแขนกลและมือจับ 4 ช่อง, การซิงค์แขนคู่, ความปลอดภัยขอบเขตมุม, กระแสกระชากตอนเปิดเครื่อง และการส่งต่อมุมระหว่างโหมด Auto/Manual
- **หลักฐานยืนยัน (Evidence)**:
  - `updateServoTargetsFromPs2` (`src/robot_logic.cpp:512-594`) รองรับครบ: `RC1` (`L2=180°`), `RC2` (`D-Pad U/D ±10°`, invert dir `true`), Preset `Circle` (`Drop 145/140°` <-> `Hold 75/70°`), และแขนหน้า `RC3+RC4` (`L1=30°, R1=140°`, `Square` toggle) พร้อม `clampServoTargets` (`0..180°`) `[Verified: Build/Test Run]`, ส่วน `RC1` (`R2=0°`, `lines 572-576`, `0` executions ใน `test_main.cpp`) `[Verified: Static Code Inspection]`
  - `RC2` มี Smooth Slew `6°/25ms`: ตรรกะ `slewServoAngle` (`src/robot_logic.cpp:289-301`) `[Verified: Build/Test Run]` และการเรียกใช้งานจริงใน `applyServos` (`src/minir4_io.cpp:591-595`) พร้อมสวิตช์เปิด/ปิดจาก `L3` (`src/main.cpp:628-633`) `[Verified: Static Code Inspection]`
  - `RC3+RC4` กำหนดพัลส์กว้าง `500–2500 us` (`configureRc34PulseRange`, `src/minir4_io.cpp:132-163`), กลับทิศฮาร์ดแวร์ `RC4` (`kRc4InvertDir = true`, `include/config.h:125`) และสั่งเคลื่อนที่พร้อมกันในเฟรม UART เดียวด้วย `mmL.SetAllServoAngle` (`src/minir4_io.cpp:622-637`) รวมถึงตอนบูต (`staggerBootServos`, `src/minir4_io.cpp:199-226`) เพื่อลดกระแสกระชาก `[Verified: Static Code Inspection]`
  - `AutoMode::recordServoAngle` (`src/auto_mode.cpp:329-351`) ซิงค์มุม `RC1..RC4` ล่าสุดจาก Auto กลับเข้าโหมด Manual และ `main.cpp:529-534` รีเซ็ตสถานะ toggle เก่าไม่ให้ค้าง `[Verified: Static Code Inspection]`
- **เหตุผลการหักคะแนน (`-2.0`)**:
  - ค่า `kRc3InitDeg`, `kRc3L1Deg`, `kRc3SquareDeg` (และ `RC4`) ถูกตั้งเป็น `30°` เท่ากันทั้งหมด (`include/config.h:115-123`) ทำให้ถ้ากด `Square` จากท่าเริ่มต้นหรือท่า `L1` เซอร์โวจะสั่ง `30 -> 30` ไม่เกิดการเคลื่อนที่ (จะได้ผลเฉพาะหลังกด `R1` ไป `140°` แล้วเท่านั้น) `(-0.5)` `[Verified: Static Code Inspection]`
  - หาก `mmL.SetAllServoAngle` ล้มเหลว `applyServos()` จะตกลงไปสั่งทีละช่อง (`src/minir4_io.cpp:639-670`) ซึ่งอาจทำให้แขนซ้าย/ขวาขยับเหลื่อมเวลาและบล็อก UART ได้นานขึ้นเมื่อ STM32 ขัดข้อง `(-0.5)` `[Verified: Static Code Inspection]`
  - ขอบเขตกลไกจริงบนตัวหุ่นยนต์และการรับช่วงพัลส์ `500–2500 us` ของเซอร์โวที่ใช้แข่งยังไม่ได้ทดสอบบนฮาร์ดแวร์จริง `(-1.0)` `[Unverified: Requires Physical Hardware/Field Test]`

### หมวดที่ 4: Autonomous Mode & MARC Match Readiness (`20%`) — คะแนน **`6.0 / 10`** (`12.00 / 20.00` คะแนน)
- **เกณฑ์ย่อย**: การเข้า/ออกโหมด Auto ตามกติกาแข่ง, ระบบ Proxy ความปลอดภัย MATRIXblock R4, การป้องกันหุ่นยนต์ค้าง/ชนกำแพง และคุณภาพของตรรกะภารกิจ Autonomous จริง
- **หลักฐานยืนยัน (Evidence)**:
  - **Auto Supervisor แข็งแกร่ง (`~9/10`)**: เริ่มโหมดได้ทั้ง PS2 `Triangle`, กด `BTN_UP` ค้าง `1.2 s` (ไม่ต้องพึ่งจอยที่จุดสตาร์ท) หรือ Serial `'g'` (`src/main.cpp:820-839`); มี Safe Proxy ครบ (`AutoMiniR4Proxy`, `AutoDriveDCProxy` พร้อม timeout `3 s` ป้องกันบั๊ก `DriveDC.begin()`, `AutoMj5Proxy` ข้าม `Wire1.begin()` เพื่อไม่ให้จอ OLED ค้าง, `AutoPs2Proxy` ยิง `PSB_TRIANGLE` 1 ช็อตอัตโนมัติ, `src/auto_block_code.cpp:49-506`); มีระบบตรวจล้อติดขัด `checkDriveStall` (`< 3°` นาน `1.6 s` หลังช่วง grace `1.25 s`, `src/auto_mode.cpp:105-157`), จำกัดเวลาสูงสุด `60 s` (`kAutoMaxDurationMs`), และ `yield()` สำหรับยกเลิกกลางคันได้ทันที (`src/auto_mode.cpp:415-422`) `[Verified: Static Code Inspection]`
- **เหตุผลการหักคะแนน (`-4.0`)**:
  - **[High] ภารกิจ Auto ปัจจุบันเป็น Open-Loop Dead-Reckoning ล้วนและมี `delay()` รวม `15.5 s`**: `src/auto_block_code.cpp:525-567` สั่ง `DriveDC.MoveDegs(±50, ±50, ...)` จำนวน 11 คำสั่ง และ `delay(500)`/`delay(1000)` รวม 20 ครั้ง (= `15,500 ms` + `11 × 110 ms` brake delay = `16.71 s` ที่หุ่นหยุดนิ่ง) โดยไม่ใช้ `MoveSyncDegs`/`MoveGyroDegs`/`TurnGyro` และไม่อ่านเซนเซอร์สนามใดๆ (ทั้งที่บรรทัด 521 เรียก `MiniR4.I2C1.MJ5.begin()`) ทำให้มีความเสี่ยงสูงที่จะเคลื่อนที่เพี้ยนตามแรงดันแบตและพื้นสนาม MARC `(-3.0)` `[Verified: Static Code Inspection]` & `[Unverified: Requires Physical Hardware/Field Test]`
  - **[Medium] ฮาร์ดโค้ดทิศทางมอเตอร์และไม่ตรวจค่าคืนกลับ**: `MiniR4.DriveDC.begin(1, 2, false, true)` (`src/auto_block_code.cpp:522`) ละเลยทิศทางที่บันทึกไว้ใน EEPROM (`f1`/`f2`) และไม่เช็กค่า error `0x09U` `(-0.5)` `[Verified: Static Code Inspection]`
  - **[Medium] ความเสี่ยง `longjmp` ข้าม Destructor**: `abortAuto()` (`src/auto_mode.cpp:87-93, 256`) ใช้ `longjmp` ซึ่งปลอดภัยกับโค้ดปัจจุบันที่ไม่มี RAII local แต่จะเป็น Undefined Behavior หากผู้ใช้งานวางโค้ดที่มี `String` หรือ C++ object ใน `auto_block_code.cpp` รวมถึงขอบเล็กน้อยของ `AutoPs2Proxy` หากเรียก `read_gamepad()` สองครั้งก่อนเช็ก `Button(PSB_TRIANGLE)` (`src/auto_block_code.cpp:396-415`) `(-0.5)` `[Verified: Static Code Inspection]`

### หมวดที่ 5: Wireless Link, Coprocessor & Failsafe Reliability (`15%`) — คะแนน **`8.0 / 10`** (`12.00 / 15.00` คะแนน)
- **เกณฑ์ย่อย**: ความเสถียรของลิงก์ไร้สาย PS2/MJ2, การคัดกรองแพ็กเก็ตขยะ, การเข้าสู่ Failsafe และต่อกลับ, การสื่อสาร STM32/I2C, Watchdog และ Emergency Stop
- **หลักฐานยืนยัน (Evidence)**:
  - `Ps2Driver` (`src/ps2_driver.cpp:10-236`) อ่านจอย 9 ไบต์ใน ~`1.4 ms` (~62 kHz) โดยปิด interrupt เฉพาะภายใน 1 ไบต์ (~130 us) และดีดกลับภายใน ~`0.8 ms` เมื่อถอดตัวรับสัญญาณ ไม่บล็อกลูปหลัก `[Verified: Static Code Inspection]`
  - `isPs2FrameValid` + `FailsafeStateMachine` (`src/robot_logic.cpp:637-768`) ตรวจ 5 ด่าน (ด่าน 1–4: โหมด `0x73`, sync `0x5A`, ปุ่ม D-Pad ขัดแย้ง, ปุ่มรูปทรงกดพร้อมกัน 4 ปุ่ม, `0xFF` ทั้งเฟรม และคัดเฟรม `0x00` ทั้งเฟรม `stuckLow` ออกได้ที่ด่าน 2 ใน `test_main.cpp:298-303`) ซับ glitch ชั่วคราว 3 เฟรม (`< 100 ms`), ตัดเข้า `Failsafe` (หยุดมอเตอร์ทันที) ที่เฟรมเสียที่ 4, ตัดเป็น `Disconnected` ที่ 120 เฟรม (`3000 ms`) หรือโหมดรีเซ็ต `0x41` 3 เฟรม และลองเชื่อมต่อใหม่แบบ non-blocking ทุก `500 ms` `[Verified: Build/Test Run]`, ส่วนสาขาด่าน 5 (`0x00` ทั้งเฟรมที่ `lines 664-668` ซึ่งถูกด่าน 2 บังไว้ใน `test_main.cpp:298`) `[Verified: Static Code Inspection]`
  - ลำดับความสำคัญใน `arbitrateDrive` (`src/main.cpp:763-780`): `E-stop > PS2 Failsafe > Cross Brake > Serial Drive > PS2 Manual > Coast-to-Stop`; มี Watchdog 3s (`src/main.cpp:994`), `verifyAndHealLowerMcu` (`src/minir4_io.cpp:372-420`) กู้ค่าตั้งค่าหลัง STM32 ไฟตก/ขาดการติดต่อ และ `OledUI::recoverBus` กู้บัส I2C ค้าง (`src/oled_ui.cpp:80-108`) `[Verified: Static Code Inspection]`
- **เหตุผลการหักคะแนน (`-2.0`)**:
  - **[Medium] `BTN_DOWN` E-Stop ทำงานเมื่อปล่อยปุ่ม (`outShortRelease`)** (`src/minir4_io.cpp:247-255`, `src/main.cpp:578-580`, `src/auto_mode.cpp:188-190`) เพื่อเปิดทางให้กด `BTN_UP+BTN_DOWN` ค้าง 2.5s เข้า Bootloader ได้ แต่หากผู้ควบคุมกดปุ่ม `BTN_DOWN` ค้างไว้ตอนฉุกเฉิน หุ่นจะยังไม่หยุดจนกว่าจะปล่อยนิ้ว `(-0.75)` `[Verified: Static Code Inspection]`
  - **[Medium] การบล็อกรอ UART ของ `MMLower` และการค้างตอนบูตในไลบรารี**: `MMLower::WaitData`/`CommReadData` (`MMLower.cpp:2207-2278`) รอนานสุด `100+10 ms` ต่อคำสั่ง ทำให้เมื่อ STM32 หลุดกลางคัน 1 รอบ (`applyServos` fallback + `GetButtonState` + `GetPowerInfo`) อาจหน่วงได้ ~`0.9–1.1 s` (ยังต่ำกว่า 3s WDT) และ `MatrixMiniR4::begin()` (`MatrixMiniR4.cpp:41-44, 67-76`) มีลูป `while` ไม่จำกัดเวลาเมื่อ STM32 ไม่ตอบสนองตอนเปิดเครื่องก่อนเริ่ม WDT `(-0.75)` `[Verified: Static Code Inspection]`
  - ความทนทานต่อสัญญาณรบกวน 2.4 GHz จากหุ่นยนต์หลายสิบทีมในสถานที่แข่งจริงยังไม่ได้ทดสอบภาคสนาม `(-0.5)` `[Unverified: Requires Physical Hardware/Field Test]`

### หมวดที่ 6: Power Management, Stall Safety & Persistence (`10%`) — คะแนน **`7.5 / 10`** (`7.50 / 10.00` คะแนน)
- **เกณฑ์ย่อย**: การตรวจวัดและกรองแรงดันแบตเตอรี่, การป้องกันไฟตก (Brownout), การป้องกันมอเตอร์/เกียร์บ็อกซ์ติดขัด และความปลอดภัยของการบันทึกค่าลง EEPROM
- **หลักฐานยืนยัน (Evidence)**:
  - `BatteryStabilizer` (`src/robot_logic.cpp:770-1031`) รองรับทั้ง USB (`<5.5V`), LiPo 2S/3S/4S และอะแดปเตอร์ 9–18V โดยใช้ Asymmetric EMA (`35%` ลง, `8%` ขึ้น), ป้องกันแรงดันกระชากจาก regen (`+3%` สูงสุด `350 mV` นาน `1500 ms`), Hysteresis `350 mV` และหน่วงการฟื้นระดับ `1500 ms` ทีละขั้น `[Verified: Build/Test Run]`
  - ระบบป้องกันความเสียหายทางกล/ซอฟต์แวร์: `applyAsymmetricSlewStep` ที่บังคับผ่านจุด `0` เพื่อลดกระแสกระชากเวลาย้อนทิศมอเตอร์ (`src/robot_logic.cpp:100-103`) และตัวช่วย `hasEncoderProgressed` `[Verified: Build/Test Run]`, ส่วนตัวเรียก `checkDriveStall` (`src/auto_mode.cpp:105-157`) และ Hardware WDT 3 วินาที (`src/main.cpp:994`) `[Verified: Static Code Inspection]`
  - EEPROM Persistence (`src/minir4_io.cpp:113-130`, `src/robot_logic.cpp:303-378`) เก็บ 8 ไบต์พร้อม Magic `'M','R'`, Version `1`, และ CRC-16-CCITT ที่ออฟเซ็ต `128` (`0x80`) หลบพื้นที่คาลิเบรต IMU ของ `MiniR4Motion` ที่ `50..73` (`MiniR4Motion.h:198-237`) พร้อมตรวจ `EEPROM.read(addr) != raw[i]` เพื่อถนอม Flash `[Verified: Build/Test Run]` & `[Verified: Static Code Inspection]`
- **เหตุผลการหักคะแนน (`-2.5`)**:
  - **[Medium] ปิดระบบลดกำลังเมื่อแบตอ่อน (`kEnableBatterySagThrottle = false` ที่ `include/config.h:144`) ขณะที่ยังเปิดบูสต์ชดเชยสูงสุด `115%`**: ใน `BatteryStabilizer::motorProfile()` (`src/robot_logic.cpp:997-1004`) แม้แรงดันตกถึงระดับ `Critical` หรือ `Emergency` (`<= 6.8V` / `<= 6.4V` หรือ `<= 76%`/`69%` **ของแรงดันฐาน (`baselineMv_`)** ตาม `kBattCritSagPct`/`kBattEmergSagPct` ที่ `include/config.h:137-138` ต่างจาก % SoC `batteryPercent()` บนจอ OLED) เฟิร์มแวร์ยังอนุญาตกำลัง `100%` และคูณชดเชยได้ถึง `115%` (`kBattCompMaxPct = 115U`) ซึ่งเพิ่มความเสี่ยงต่อภาวะไฟตกจนบอร์ดรีเซ็ตกลางเกม `(-1.5)` `[Verified: Build/Test Run]` & `[Verified: Static Code Inspection]`
  - โหมด Manual Teleop ไม่มีการตัดกำลังอัตโนมัติเมื่อล้อติดขัด (มี `checkDriveStall` เฉพาะใน Auto Mode) และยังไม่ได้ทดสอบพฤติกรรมแรงดันตกขณะขับดันจริงบนแบตใกล้หมด `(-1.0)` `[Verified: Static Code Inspection]` & `[Unverified: Requires Physical Hardware/Field Test]`

### หมวดที่ 7: Competition Operations, Diagnostics & Maintainability (`15%`) — คะแนน **`8.5 / 10`** (`12.75 / 15.00` คะแนน)
- **เกณฑ์ย่อย**: ความสะดวกในการเช็กสภาพหน้าพิต/หน้าเส้นสตาร์ท (OLED, LED, Buzzer, Serial CLI), ทางลัดอัปโหลดโค้ด, การแยกโมดูล/ตั้งค่า และความสอดคล้องของเอกสาร
- **หลักฐานยืนยัน (Evidence)**:
  - หน้าจอ OLED `128x32` (`src/oled_ui.cpp:300-410`) อัปเดตแบบ hash-dedup ทุก `180 ms` แสดงสถานะครบ: Tag สถานะ, แรงดัน/`%` แบต, **Stability Index (`STB:100% L+ P+ C+ T+`)**, กำลังมอเตอร์ + แฟลก `H`/`S`/`R`/`B` + ระดับแบต `L`/`C`/`E`, แถบความคืบหน้ากดค้างเข้า Auto/Bootloader, นาฬิกาจับเวลา Auto (`AUTO xx.xs X:Exit`), มุม `RC1..RC4` เสริมด้วยไฟ WS2812B 2 ดวง (`updateRgbLeds`) และเสียง Buzzer แยกเหตุการณ์ `[Verified: Static Code Inspection]`
  - Serial CLI (`src/serial_control.cpp:211-635`) รองรับทั้งขับสำรองด้วยคีย์บอร์ด WASD (หมดอายุอัตโนมัติใน `650 ms`), สั่งมอเตอร์/เซอร์โวตรง (`m L R`, `c ch deg`), กลับทิศมอเตอร์/ปรับ Trim (`f1`/`f2`/`,`/`.`) และสตรีม Telemetry (`'t'`) `[Verified: Build/Test Run]`, ส่วนสแกน I2C (`'v'`), คำสั่ง `'u'`, การทำงานเข้า Bootloader จริง (`enterUploadBootloader` และคอมโบ `BTN_UP+BTN_DOWN` 2.5s, `src/main.cpp:209-230`) รวมถึงการเขียน EEPROM จริง `[Verified: Static Code Inspection]`
  - โครงสร้างแยกไฟล์ชัดเจน (`include/config.h` ศูนย์กลางค่าคงที่, `robot_logic` แยกจากฮาร์ดแวร์) และมีคู่มือภาษาไทยละเอียดใน `README.md` (1,197 บรรทัด) `[Verified: Static Code Inspection]`
- **เหตุผลการหักคะแนน (`-1.5`)**:
  - คอมเมนต์เก่า 3 จุดใน `test/test_robot_logic/test_main.cpp:485, 517, 1266` และข้อความ log `"BTN_DOWN press"` (`src/main.cpp:580`) ที่จริงทำงานตอนปล่อยปุ่ม อาจทำให้ทีมพิตเข้าใจคลาดเคลื่อนได้เล็กน้อย `(-0.75)` `[Verified: Static Code Inspection]`
  - มีการแก้ไข `README.md` ที่ยังไม่ถูก commit ใน working tree (`M README.md`, `+52/-43`) และความชัดเจนของจอ OLED/เสียงเตือนในสภาพแวดล้อมสนามแข่งจริงยังต้องตรวจหน้างาน `(-0.75)` `[Verified: Build/Test Run]` & `[Unverified: Requires Physical Hardware/Field Test]`

---

## 5. ความเสี่ยง บั๊ก และรายการตรวจสอบก่อนลงแข่ง (Prioritized Risks, Bugs & Pre-Competition Checklist)

### 5.1 ตารางสรุปความเสี่ยงและข้อสังเกตเรียงตามลำดับความสำคัญ

| ระดับ | ประเด็นที่พบ (Issue / Risk) | หลักฐานอ้างอิง (`file:line`) | ผลกระทบและแนวทางรับมือก่อนแข่ง |
| :---: | :--- | :--- | :--- |
| **High** | **โหมด Auto เดินแบบ Open-Loop `MoveDegs` + `delay()` นาน `15.5 s`** | `src/auto_block_code.cpp:525-567` | ใช้ `MoveDegs(±50, ±50)` 11 ครั้ง และ `delay(500/1000)` 20 ครั้ง (`15.5 s` + `1.21 s` เบรกดีเลย์ = `16.71 s`) โดยไม่มี Gyro/Sync PID หรืออ่านเซนเซอร์ (`MJ5.begin()` ที่ `line 521` ไม่ได้ใช้งาน) เสี่ยงเดินเบี้ยวเมื่อแรงดันแบตเปลี่ยนหรือล้อลื่น **แนวทาง**: เปลี่ยนเป็น `MoveSyncDegs`/`MoveGyroDegs`/`TurnGyro` หรือเพิ่มเซนเซอร์จัดแนว และลดดีเลย์ส่วนเกินให้เหมาะสมกับเวลาแข่งจริง `[Verified: Static Code Inspection]` |
| **Medium** | **ปิด `kEnableBatterySagThrottle = false` แต่คงบูสต์ `115%` เมื่อแบตอ่อน** | `include/config.h:144`, `src/robot_logic.cpp:997-1004` | ในระดับ `Critical`/`Emergency` ยังจ่ายกำลัง `100%` และชดเชยได้ถึง `115%` เสี่ยงบอร์ดรีเซ็ตจากไฟตก **แนวทาง**: เปลี่ยนเป็น `true` สำหรับแมตช์ต่อเนื่อง หรือบังคับเปลี่ยนแบตเตอรี่ก้อนใหม่ทุกครั้งที่หน้าจอแสดงต่ำกว่า `75%` `[Verified: Build/Test Run]` & `[Verified: Static Code Inspection]` |
| **Medium** | **`BTN_DOWN` E-Stop ทำงานเมื่อปล่อยปุ่ม (ไม่ใช่ทันทีที่กด)** | `src/minir4_io.cpp:247-255`, `src/main.cpp:578-580`, `src/auto_mode.cpp:188-190` | กด `BTN_DOWN` ค้างยังไม่หยุดหุ่นจนกว่าจะปล่อยนิ้ว (ออกแบบเพื่อรองรับคอมโบ `BTN_UP+BTN_DOWN` 2.5s เข้า Upload) **แนวทาง**: ซักซ้อมผู้ควบคุมให้ "กดแล้วปล่อยทันที" หรือย้าย E-Stop เป็นขอบขาขึ้นหากไม่ใช้คอมโบในสนาม `[Verified: Static Code Inspection]` |
| **Medium** | **ฮาร์ดโค้ดทิศมอเตอร์และละเลยค่าคืนกลับของ `DriveDC.begin()` ใน Auto** | `src/auto_block_code.cpp:180-205, 522` | `DriveDC.begin(1, 2, false, true)` ไม่ใช้ค่ากลับทิศ `f1`/`f2` จาก EEPROM และไม่เช็กผลลัพธ์ `0x09U` (3s timeout) **แนวทาง**: ตรวจให้ `kMotorLeft/RightReverseInit` ใน `config.h` ตรงกับสายมอเตอร์จริงเสมอ และทดสอบกดเข้า Auto ในพิตว่าเดินหน้าถูกทิศ `[Verified: Static Code Inspection]` |
| **Medium** | **`longjmp` Abort เสี่ยง UB หากใส่ตัวแปร RAII/`String` ใน `auto_block_code.cpp`** | `src/auto_mode.cpp:87-93, 256` | การตัดจบ Auto ด้วย `longjmp` จะข้าม destructor ของ C++ (ปัจจุบัน `auto_block_code.cpp` ปลอดภัยเพราะไม่มี RAII local) **แนวทาง**: ห้ามประกาศตัวแปร `String` หรือ object ที่มี destructor ในโซนวางโค้ด Auto `[Verified: Static Code Inspection]` |
| **Medium** | **UART STM32 บล็อกสูงสุด `100 ms`/คำสั่ง และ `MiniR4.begin()` วนรอไม่รู้จบ** | `MMLower.cpp:2207-2278`, `MiniR4DC.h:92-97`, `src/minir4_io.cpp:622-670`, `MatrixMiniR4.cpp:41-44, 67-76` | หาก STM32 หลุดกลางคัน รอบนั้นอาจหน่วงรวม `~0.9–1.1 s` (ยังรอด 3s WDT); หากสาย STM32 หลุดตอนเปิดเครื่อง `MiniR4.begin()` จะค้างใน `while` ก่อน WDT ทำงาน **แนวทาง**: ตรวจความแน่นของบอร์ด Carrier Shield และสายแบตก่อนเปิดสวิตช์ทุกครั้ง `[Verified: Static Code Inspection]` |
| **Low** | **ปุ่ม `Square` (`30°`) ซ้ำกับมุม `Init` และ `L1` (`30°`)** | `include/config.h:115-123` | กด `Square` จากท่าเริ่มต้น `(30, 30)` หรือ `L1` `(30, 30)` จะไม่ขยับ (ทำงานเฉพาะหลังกด `R1` `140°`) **แนวทาง**: ปรับ `kRc3/4SquareDeg` หากต้องการท่ากดพื้นคนละองศากับ `L1` `[Verified: Static Code Inspection]` |
| **Low** | **คอมเมนต์ล้าสมัยใน Unit Test 3 จุด** | `test/test_robot_logic/test_main.cpp:485, 517, 1266` | คอมเมนต์ระบุ `(180, 180)`, `L1 (70, 70)`, `15%` ไม่ตรงค่าปัจจุบันใน `config.h` (ตัวเทสต์อ้างสัญลักษณ์ `Config::` จึงยังผ่านปกติ) **แนวทาง**: อัปเดตคอมเมนต์เมื่อแก้ไขไฟล์ทดสอบครั้งถัดไป `[Verified: Static Code Inspection]` |
| **Low** | **กรณีเรียก `PS2.read_gamepad()` สองครั้งใน Auto** | `src/auto_block_code.cpp:396-415` | การเรียก `read_gamepad()` ซ้ำก่อนเช็ก `Button(PSB_TRIANGLE)` จะล้าง `entryTriPending_` (ไม่เกิดกับโค้ด Auto ปัจจุบัน) **แนวทาง**: คงรูปแบบ `read_gamepad()` เดียวใน `loop()` ตามเทมเพลต MATRIXblock `[Verified: Static Code Inspection]` |
| **Low** | **ขอบเขต `#pragma GCC diagnostic` และขาด Mock/HIL สำหรับโมดูลฮาร์ดแวร์** | `src/auto_block_code.cpp:23-32`, `platformio.ini:27, 39-41` | `auto_block_code.cpp` ปิด 9 warnings ชั่วคราวในไฟล์เดียว; 6 โมดูลฮาร์ดแวร์ยังไม่มี Unit Test/HIL **แนวทาง**: ใช้ HIL Checklist ด้านล่างตรวจรับรองบนหุ่นจริง `[Verified: Static Code Inspection]` |

### 5.2 รายการตรวจสอบภาคสนามก่อนแข่ง (Pit & Field HIL Pre-Competition Checklist)
รายการต่อไปนี้ครอบคลุมจุดที่ติดป้าย `[Unverified: Requires Physical Hardware/Field Test]` ทั้งหมด ต้องทดสอบจริงกับหุ่นยนต์ก่อนลงสนาม:

1. **บูตระบบ & เวอร์ชัน STM32 (`MMLower`)**:
   - เปิดสวิตช์ด้วยแบตเตอรี่แข่งจริง (ไม่ใช่ USB อย่างเดียว) ตรวจ OLED และ Serial (`115200 baud`) ต้องไม่มีเสียงเตือน `FW < 6.0` หรือ `MiniR4 init returned non-OK`, ค่า `oled=0x3D(ok=1)` และไม่มี `rngErr` (`servoRangeErr == 0`)
2. **การขับ Teleop & ลักษณะทางกายภาพ**:
   - ทดสอบเดินหน้า/ถอยหลัง/หมุนตัวด้วย `LY`/`RX` บนพื้นสนามจริง, ทดสอบ `Select` (Precision 55%), `R3` (Reverse-Front), `D-Pad L/R` (Micro-Turn), และ `Start + D-Pad L/R` (ปรับ Trim วิ่งตรงแล้วรีบูตเพื่อเช็กว่าค่า EEPROM CRC-16 ยังอยู่)
   - ทดสอบ Smart Hold-Brake บนทางลาด/เมื่อปล่อยสติ๊ก และทดสอบ CoM Anti-Tip ขณะยกแขน `RC2` สูงสุด (`0°`) แล้วหมุนตัวเร็ว
3. **แขนกล & เซอร์โว `RC1..RC4`**:
   - ตรวจจุดสุดกลไกของ `RC1` (`0° / 180°`), `RC2` (`0°..180°`), `RC3+RC4` (`30° / 140°` พร้อมพัลส์ `500–2500 us`), `Circle` Drop/Hold preset และตรวจว่าสายไม่ตึง/เกียร์ไม่ขบ
4. **ความเสถียรของสัญญาณไร้สาย 2.4 GHz & Failsafe**:
   - ปิดจอย PS2/MJ2 หรือเดินออกนอกระยะขณะล้อหมุน: หุ่นต้องหยุดและล็อกเบรกภายใน `~100 ms` (`FAILSAFE`), จากนั้นเปิดจอยใหม่และขยับปุ่ม ต้องกลับมาเป็น `PS2 OK` อัตโนมัติโดยไม่ต้องรีเซ็ตหุ่น
   - ทดสอบ **กดแล้วปล่อย `BTN_DOWN`** (E-stop) และ **กดแล้วปล่อย `BTN_UP`** (Resume), รวมถึงทดสอบ `Cross` เบรกฉุกเฉิน
5. **โหมด Autonomous & ความปลอดภัย**:
   - ทดสอบเข้า Auto ทั้งจาก `Triangle` และ `BTN_UP` ค้าง `1.2 s` บนจุดสตาร์ทสนามจริง วัดเวลาและพิกัดปลายทางเทียบกับแรงดันแบต `100%` และ `~80%`, ทดสอบกด `Cross` ตัดจบกลางคัน (ล้อต้องหยุดสนิทและส่งต่อมุม `RC1..RC4` กลับ Manual ถูกต้อง), และทดสอบจับล้อให้หยุดหมุนเพื่อยืนยัน `StallTimeout` (`~2.85 s`)
6. **วินัยพลังงานแบตเตอรี่ (Battery Discipline)**:
   - เนื่องจาก `kEnableBatterySagThrottle = false` (`include/config.h:144`) ให้ตรวจแรงดันบน OLED และ **เปลี่ยนแบตก้อนใหม่ทันทีเมื่อความจุต่ำกว่า `75%` หรือเริ่มขึ้นแฟลก `L`/`C`/`E`**

---

## 6. คู่มือการตรวจสอบซ้ำโดยอิสระ (Independent Reproducibility Guide)

ผู้ตรวจอิสระสามารถรันคำสั่ง PowerShell แบบ Read-Only ต่อไปนี้ใน `C:\Users\MB523W\OneDrive\Documents\PlatformIO\Projects\matrix_R4` เพื่อตรวจสอบผลลัพธ์และยืนยันว่าไม่มีการแก้ไขซอร์สโค้ดเดิม:

```powershell
# 1. ยืนยันการคอมไพล์เฟิร์มแวร์จริง uno_r4_wifi (0 warnings/errors, RAM 43.9%, Flash 41.7%)
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e uno_r4_wifi

# 2. ยืนยันการผ่าน Unit Test ทั้ง 36 เคสบน native
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" test -e native -v

# 3. ยืนยันผลการวิเคราะห์เชิงสถิต cppcheck (src: 0 HIGH, 1 MEDIUM false-positive)
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" check -e uno_r4_wifi --skip-packages
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" check -e native --skip-packages

# 4. ยืนยันการรักษาสภาพโค้ดเดิม (R2) และแฮชของ README.md
git status -sb
git diff --stat
Get-FileHash README.md -Algorithm SHA256
# ผลที่คาดหวัง (ก่อน commit/push ใน Milestone M2): แสดงเฉพาะ M README.md เดิม (+52/-43, SHA256 1311356C3CBD5A3B0275DC447898A4CD12BD131F88C07740A8DD84D779F4BC06) และ ?? MATRIX_MARC_READINESS.md (ไฟล์ src/*, include/*, test/*, platformio.ini ไม่มี diff)
# ผลที่คาดหวัง (หลัง commit/push ขึ้น origin/main เสร็จสิ้น): git status -sb แสดงเฉพาะ ## main...origin/main (working tree สะอาด) และ git diff --stat ว่างเปล่า
```
