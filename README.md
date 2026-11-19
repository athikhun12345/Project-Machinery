# 90-Degree Pick and Place Mechanism

**นายอธิคุณ เนตรสุวรรณ (Athikhun Netrsuwarn)**  
รหัสนิสิต 6730564421  
วิชา 2103322 Design of Mechanism (Mech Machinery)  
ภาควิชาวิศวกรรมเครื่องกล คณะวิศวกรรมศาสตร์ จุฬาลงกรณ์มหาวิทยาลัย (ภาคการศึกษาที่ 1/2569)  


---

## Project overview

- **Project name:** 90-Degree Pick and Place Mechanism (10-Bar Planar Linkage)
- **Problem / purpose:** ออกแบบและจำลองกลไกหยิบ-วางชิ้นงานทำมุม 90 องศา (90° Pick and Place Mechanism) โดยใช้ต้นกำลังขับเคลื่อนแบบหมุนต่อเนื่องเพียงตัวเดียว (1-DOF) เพื่อแปลงการหมุนของข้อเหวี่ยง (Crank) ให้ปลายแขนทำงานเคลื่อนที่ระหว่างตำแหน่งแนวตั้งและแนวนอนที่ทำมุมตั้งฉากกัน พร้อมวิเคราะห์ Position, Velocity, Acceleration, Workspace และ Actuator Torque ด้วย Simscape Multibody
- **Team:** นายอธิคุณ เนตรสุวรรณ (ทำเดี่ยว — รับผิดชอบ MATLAB / Simscape Multibody)
- **Scope:** การวิเคราะห์พื้นฐานอย่าง Kinematic Diagram, DoF, Grashof Condition และการจำลองด้วย Simscape Multibody Simulation *เนื่องทำคนเดียวจึงรวมส่วน Symbolic Math และการสร้าง Physical Prototype*
- **Status:** Completed (Simulation & Documentation)
- **Last updated:** 2026-11-19

---

## Repository structure

โครงสร้างไฟล์จัดเรียงตามแนวทางของ Mechanical Engineering Project Template ดังนี้:

| Path | Purpose | Files included |
| :--- | :--- | :--- |
| `hardware/mechanical/` | ไฟล์โมเดล CAD 3 มิติ สำหรับนำเข้าใน Simscape | `Ground.step`, `R2 Brick5.step`, `R2 Brick7.step`, `R2 Brick9.step`, `R2 Brick15.step` |
| `simulation/` | ไฟล์โมเดลจำลอง Simulink/Simscape Multibody, สคริปต์พล็อตกราฟ และรูปภาพประกอบ Mask | `PickandPlace.slx`, `Machinery_MATLAB_Class3_Plot.m`, `4_bars.png`, `6_bars.png`, `Pick_and_Place.png` |
| `media/` | ภาพไดอะแกรมการคำนวณ, ภาพบล็อกและกราฟผลลัพธ์ (4 Bars / 6 Bars) และวิดีโอจำลองการเคลื่อนที่ | `Calculation.png` (ไดอะแกรมกลไก), `Simulink.png`, `PickandPlace.avi` (วิดีโอการเคลื่อนที่), โฟลเดอร์ `4 Bars/` (`Blocks`, `Follower`, `Input`), โฟลเดอร์ `6 Bars/` (`6 Graph`, `90Deg`, `Blocks`, `Correct Data`) |
| `docs/` | เอกสารรายงานสรุปผลโครงงานและรายละเอียดประกอบ | `Report.docx`, `Report.pdf` |
| `README.md` | รายงานสรุปภาพรวมของโครงงาน วิธีรัน และผลการวิเคราะห์ทั้งหมด | `README.md` |
---

## Kinematic model & preliminary analysis

![Kinematic Diagram and CAD Model](media/S__11845641.jpg)

กลไก 90-Degree Pick and Place ที่พัฒนาขึ้นเป็นกลไกระนาบแบบ **10-Bar Linkage** ในระนาบ $X\text{-}Z$ โดยแบ่งโครงสร้างออกเป็น 2 ชุดย่อยที่ทำงานเชื่อมโยงกัน ได้แก่ **ชุดขับเคลื่อน 4-Bar Linkage** (Links 1–4) และ **ชุดส่งกำลังขยายการเคลื่อนที่ 6-Bar Linkage** (Links 5–10)

### 1. Links and joints definition

| Link | บทบาท (Role) | ไฟล์ CAD อ้างอิง | สีในโมเดล 3 มิติ | ข้อต่อที่เชื่อมต่อ (Joints) |
| :---: | :--- | :--- | :--- | :--- |
| **1** | Ground (ฐานยึด) | `Ground.step` | เทา | $O_2, O_4, O_5, O_8$ |
| **2** | Crank (ก้านขับต้นกำลัง) | `R2 Brick5.step` | แดง | $O_2, A$ |
| **3** | Coupler (ก้านเชื่อมส่งกำลัง) | `R2 Brick15.step` | เขียวอ่อน | $A, B$ (เชื่อมชุด 6-Bar ที่จุดร่วม $B, C, D$) |
| **4** | Rocker (ก้านโยกตาม) | `R2 Brick7.step` | น้ำเงิน | $B, O_4$ |
| **5** | Left Main Arm | `R2 Brick9.step` | ม่วง | $O_5, F, E$ |
| **6** | Left Connecting Link | `R2 Brick7.step` | ชมพูบานเย็น | $F, D, C$ |
| **7** | Left Output Arm (End-Effector) | `R2 Brick15.step` | น้ำเงินเข้ม | $E, I$ (จุดวัดวิถีปลายแขน) |
| **8** | Right Main Arm | `R2 Brick9.step` | เขียวเข้ม | $O_8, G, H$ |
| **9** | Right Connecting Link | `R2 Brick7.step` | ส้ม / เหลืองทอง | $G, C$ |
| **10** | Right Output Arm | `R2 Brick15.step` | ฟ้าอ่อน | $H, I$ |

### 2. Degrees of freedom (DoF) analysis

ใช้สมการของ Grübler–Kutzbach สำหรับกลไกระนาบ (Planar Mechanism):
- จำนวนชิ้นส่วนทั้งหมด: $n = 10\text{ links}$
- จำนวนข้อต่อแบบ Lower Pair (Revolute Joints): $J_1 = 13\text{ joints}$ ได้แก่ $O_2, O_4, O_5, O_8, A, B, C, D, E, F, G, H, I$
- จำนวนข้อต่อแบบ Higher Pair: $J_2 = 0$

$$\text{DoF} = 3(n - 1) - 2J_1 - J_2 = 3(10 - 1) - 2(13) - 0 = 27 - 26 = 1$$

กลไกมีองศาอิสระเท่ากับ **1 DoF** จึงสามารถควบคุมการเคลื่อนที่ของกลไกทั้งระบบได้ด้วยต้นกำลังหมุนเพียงตัวเดียวที่จุด $O_2$

### 3. Grashof condition (4-Bar driving stage)

พิจารณาชุดขับเคลื่อน 4-Bar Linkage ($O_2\text{--}A\text{--}B\text{--}O_4$):

| Link | ความยาว (mm) | การกำหนดตัวแปร |
| :--- | :---: | :--- |
| Link 1 ($O_2 O_4$) | $169.19$ | $l$ (Longest link — Ground) |
| Link 2 ($O_2 A$) | $50.00$ | $s$ (Shortest link — Adjacent to ground) |
| Link 3 ($AB$) | $150.00$ | $p$ |
| Link 4 ($O_4 B$) | $70.00$ | $q$ |

ตรวจสอบเงื่อนไข Grashof:

$$l + s = 169.19 + 50.00 = 219.19\text{ mm} < p + q = 150.00 + 70.00 = 220.00\text{ mm}$$

เนื่องจาก $l + s < p + q$ และลิงก์ที่สั้นที่สุด ($s$) ยึดติดกับฐาน (Ground) ชุดขับเคลื่อนจึงเป็น **Grashof Crank-Rocker Mechanism** โดย Link 2 สามารถหมุนได้ครบรอบ $360^\circ$ อย่างต่อเนื่อง เพื่อขับให้ Link 4 แกว่งไป-กลับและส่งกำลังต่อไปยังชุด 6-Bar Linkage

---

## Multibody simulation setup (Simscape Multibody)

แบบจำลองในไฟล์ `PickandPlace.slx` ถูกจัดโครงสร้างเป็น 2 Subsystem หลัก (`4 bars` และ `6 bars`) เชื่อมต่อกับ `World Frame`, `Solver Configuration` และ `Mechanism Configuration`

### Simulation parameters
- **Coordinate plane:** ระนาบการเคลื่อนที่ $X\text{-}Z$ (แกน $+X$ แนวนอน, แกน $+Z$ แนวตั้งชี้ขึ้น)
- **Gravity vector:** $[0,\; 0,\; -9.80665]\text{ m/s}^2$
- **Body inertia & density:** คำนวณอัตโนมัติจากเรขาคณิตไฟล์ `.step` โดยกำหนดความหนาแน่น $\rho = 1000\text{ kg/m}^3$
- **Input actuation (Joint $O_2$):** ขับเคลื่อนด้วยความเร็วเชิงมุมคงที่ $\omega_2 = 2\text{ rad/s}$ ($\approx 19.10\text{ rpm}$), ความเร่งเชิงมุม $\alpha_2 = 0\text{ rad/s}^2$, มุมตั้งต้น $\theta_2(t) = 2t\text{ rad}$ (คาบการหมุน 1 รอบ $T = \pi \approx 3.1416\text{ s}$)
- **Solver settings:** `VariableStepAuto` (`ode45`), `Stop Time = 2*pi` ($\approx 6.2832\text{ s}$ หรือครบ 2 รอบการหมุน), `Max Step Size = 0.01 s`, `AbsTol = 1e-3`
- **Sensing architecture:**
  - **Joint $O_2$ (`Deg , Torque`):** วัดมุมหมุน $\theta_2$ (แปลงผ่าน `mod(deg, 360)` ให้อยู่ในช่วง $[0^\circ, 360^\circ]$) และแรงบิดขับเคลื่อน $T_2\text{ (N}\cdot\text{m)}$
  - **Joint $O_4$ (`Deg, Omega, Alpha`):** วัดมุมโยก $\theta_4\text{ (deg)}$, ความเร็วเชิงมุม $\omega_4\text{ (rad/s)}$ และความเร่งเชิงมุม $\alpha_4\text{ (rad/s}^2)$
  - **End-Effector Point $I$ (`6 bars/Parameter Result`):** ใช้ `Transform Sensor` วัดเทียบกับ `World Frame` ได้ค่า $x, z\text{ (m)}$, $v_x, v_z\text{ (m/s)}$, $a_x, a_z\text{ (m/s}^2)$ พร้อมส่งออกไปยัง `XY Graph`, `Scope` และบันทึกตัวแปร `pickpath` ไปยัง MATLAB Workspace

---

## Simulation results & kinematic analysis

จากการจำลองการทำงานครบ 2 รอบการหมุน ($0 \le t \le 2\pi\text{ s}$) พบว่ากลไกสามารถเคลื่อนที่ได้อย่างราบรื่นโดยไม่เกิดการติดขัด (Singularity) และให้วิถีการทำงานแบบ 90° Pick and Place ตามเป้าหมาย

### 1. End-Effector (Point $I$) workspace, position, velocity, and acceleration

![End-Effector Kinematics](media/end_effector_scope.jpg)

| ตัวแปร (Parameter) | ค่าต่ำสุด (Min) | ค่าสูงสุด (Max) | ช่วงการเปลี่ยนแปลง / พฤติกรรมสำคัญ |
| :--- | :---: | :---: | :--- |
| **Position $x$** | $\approx 0.000\text{ m}$ | $\approx 0.156\text{ m}$ | ระยะเคลื่อนที่แนวนอน $\Delta x \approx 156\text{ mm}$ |
| **Position $z$** | $\approx 0.128\text{ m}$ | $\approx 0.248\text{ m}$ | ระยะยกในแนวดิ่ง $\Delta z \approx 120\text{ mm}$ |
| **Velocity $v_x$** | $\approx -0.14\text{ m/s}$ | $\approx +0.23\text{ m/s}$ | ลดลงเข้าใกล้ $0$ ที่ตำแหน่งบนสุดและขวาสุดเพื่อเปลี่ยนทิศทาง |
| **Velocity $v_z$** | $\approx -0.16\text{ m/s}$ | $\approx +0.20\text{ m/s}$ | ตัดผ่านศูนย์ที่จุดสูงสุด ($z \approx 0.248\text{ m}$) และช่วงค้างระดับล่าง |
| **Acceleration $a_x$** | $\approx -1.70\text{ m/s}^2$ | $\approx +0.33\text{ m/s}^2$ | เกิดความหน่วงสูงสุด (Peak deceleration) ขณะกลับทิศทางฝั่งขวา ($t \approx 1.74\text{ s}, 4.88\text{ s}$) |
| **Acceleration $a_z$** | $\approx -0.70\text{ m/s}^2$ | $\approx +0.36\text{ m/s}^2$ | เปลี่ยนแปลงสอดคล้องกับจังหวะยกขึ้นและกดลงในแนวดิ่ง |

**ข้อสังเกตเชิงวิศวกรรม:**
- เมื่อจุดปลาย $I$ เคลื่อนที่ขึ้นสู่ตำแหน่งสูงสุดด้านบน ($z \approx 0.248\text{ m}$ ที่ $t \approx 0.35\text{ s}$ และ $3.50\text{ s}$) ตำแหน่งในแกน $x$ จะนิ่งอยู่ใกล้ $0\text{ m}$ (ช่วง Dwell แนวนอน) ทำให้ปลายแขนเข้าประชิดจุดหยิบ/วางด้านบนในแนวดิ่งได้อย่างแม่นยำ
- เมื่อเคลื่อนที่กวาดทำมุม $90^\circ$ ไปยังตำแหน่งขวาสุด ($x \approx 0.156\text{ m}$ ที่ $t \approx 1.70\text{ s}$ และ $4.85\text{ s}$) ความสูงในแกน $z$ จะค้างอยู่ในระดับต่ำประมาณ $0.13\text{ m}$ เพื่อเข้าประชิดชิ้นงานฝั่งขวาในแนวระดับ

### 2. Follower / Rocker analysis (Joint $O_4$)

![Rocker O4 Scope](media/o4_scope.png)

- **Angular position ($\theta_4$):** ก้านโยก Link 4 แกว่งไป-กลับระหว่างประมาณ **$-103^\circ$ ถึง $-3^\circ$** (มุมกวาดรวม $\Delta\theta_4 \approx 100^\circ$)
- **Angular velocity ($\omega_4$):** แกว่งในช่วงประมาณ **$-1.6$ ถึง $+1.6\text{ rad/s}$**
- **Angular acceleration ($\alpha_4$):** แกว่งในช่วงประมาณ **$-4.5$ ถึง $+11.9\text{ rad/s}^2$** โดยเกิดจุดยอดความเร่งเชิงมุมที่ $t \approx 1.74\text{ s}$ และ $4.88\text{ s}$ ซึ่งตรงกับจังหวะดีดกลับ (Quick-return stroke) ของกลไก 4-Bar และส่งผลให้เกิด Peak Acceleration ที่ปลายแขน $a_x$ ในเวลาเดียวกัน

### 3. Input crank & driving torque analysis (Joint $O_2$)

- แม้ว่าแบบจำลองนี้จะไม่มีภาระแรงภายนอก (External payload) กระทำที่ปลายแขน แต่ภายใต้แรงโน้มถ่วงและมวลความเฉื่อยของชิ้นส่วนทั้งหมด ($\rho = 1000\text{ kg/m}^3$) มอเตอร์ที่จุด $O_2$ ต้องออกแรงบิดขับเคลื่อน ($T_2$) แกว่งเป็นคาบในช่วงประมาณ **$-0.05$ ถึง $+0.05\text{ N}\cdot\text{m}$**
- ค่าแรงบิดที่เรียบและมีขนาดต่ำตลอดทั้ง $360^\circ$ ยืนยันว่ามุมส่งถ่ายกำลัง (Transmission Angle) ของกลไกอยู่ในเกณฑ์ดี ไม่เกิดอาการขัดตัวของข้อต่อ

### 4. Animation preview

- วิดีโอสาธิตการทำงานตลอด 2 รอบการหมุน (ส่งออกจาก Simscape Multibody Explorer): ดูได้ที่ `media/Mechanism.avi` (หรือไฟล์วิดีโอสาธิตในโฟลเดอร์ `media/`)

---

## Getting started (How to run)

1. ดาวน์โหลดหรือ Clone Repository นี้ลงในเครื่อง
2. เปิดโปรแกรม **MATLAB** (แนะนำเวอร์ชัน R2024a ขึ้นไป พร้อมติดตั้ง **Simulink** และ **Simscape Multibody**)
3. ตั้งค่า **Current Folder** ใน MATLAB ให้ชี้ไปยังโฟลเดอร์ของโปรเจกต์ และคลิกขวาที่โฟลเดอร์หลักเลือก **Add to Path > Selected Folders and Subfolders** เพื่อให้ Simulink ค้นหาไฟล์ `.step` และไฟล์รูป Mask (`4_bars.png`, `6_bars.png`) ได้ครบถ้วน
4. เปิดไฟล์ `simulation/PickandPlace.slx` แล้วกดปุ่ม **Run**
5. ตรวจสอบแอนิเมชัน 3 มิติในหน้าต่าง **Mechanics Explorer** และดับเบิลคลิกดูกราฟที่บล็อก Scope ภายใน `4 bars` (`Deg , Torque`, `Deg, Omega, Alpha`) และ `6 bars/Parameter Result` (`XY Graph`, `Scope`)

---

## Assumptions, safety, and validation

- **Modeling assumptions:** ชิ้นส่วนทุกชิ้นเป็นวัตถุแข็งเกร็ง (Rigid bodies) ไม่คิดการโก่งตัวหรือระยะคลอน (Clearance) ที่หมุดข้อต่อ, ไม่คิดแรงเสียดทานที่ Revolute Joints, ความเร็วเชิงมุมขาเข้าคงที่ $\omega_2 = 2\text{ rad/s}$ และไม่มีภาระชิ้นงานภายนอก (No-load condition)
- **Kinematic validation:** ตรวจสอบความถูกต้องของแบบจำลองเบื้องต้นจากเงื่อนไข Grashof ($219.19 < 220.00\text{ mm}$), ค่า $\text{DoF} = 1$, ความสอดคล้องของคาบเวลา ($T = \pi \approx 3.1416\text{ s}$ ต่อรอบ) และความสัมพันธ์ระหว่างอนุพันธ์ของกราฟตำแหน่ง ความเร็ว และความเร่ง (ตำแหน่งสูงสุด/ต่ำสุดของ $x, z, \theta_4$ ตรงกับจุดตัดศูนย์ของ $v_x, v_z, \omega_4$ ทุกจุด)
- **Safety considerations (หากนำไปสร้างจริงในอนาคต):** ควรติดตั้งการ์ดป้องกันจุดหนีบ (Pinch points) บริเวณข้อต่อกรรไกรของชุด 6-Bar และติดตั้งสวิตช์จำกัดระยะ (Limit switches) หากปรับใช้กับโหลดจริงที่มีแรงเฉื่อยสูงบริเวณจุดกลับทิศทางฝั่งขวา ($a_{x,\text{peak}} \approx -1.70\text{ m/s}^2$)

---

## Collaboration and file practices

- จัดแยกไฟล์ต้นฉบับที่แก้ไขได้ (`.step`, `.slx`, `.m`, `.docx`) และไฟล์สื่อแสดงผล (`.png`, `.jpg`, `.avi`) ตามโครงสร้างโฟลเดอร์ของ Template
- ระบุหน่วยวัด (`m`, `mm`, `deg`, `rad/s`, `m/s`, `m/s^2`, `N*m`) กำกับบนสัญญาณและกราฟทุกชุดอย่างชัดเจน
- ไม่ commit ไฟล์ชั่วคราวของ MATLAB/Simulink (`slprj/`, `*.slxc`, `*.autosave`) โดยจัดการผ่านไฟล์ `.gitignore`
