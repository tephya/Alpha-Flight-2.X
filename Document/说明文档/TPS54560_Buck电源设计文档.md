# TPS54560DDAR Buck电源设计文档

## 1. 设计概述

**电源架构：** 4S/6S VBAT → TPS54560DDAR Buck（5V）→ 双TLV75733PDBVR LDO（3.3V_Clean + 3.3V_Dirty）

**Buck IC：** TPS54560DDAR（TI，60V输入，5A，非同步Buck，外部补偿，HSOP-8 PowerPAD封装）

**替换原因：** 原RT8289最高耐压34V，6S电池满电25.2V时安全余量不足。TPS54560耐压60V，6S完全无忧。代价是需要额外设计两样东西：频率设定电阻、外部补偿网络。EN引脚悬空，使用内部UVLO（4.3V）。

---

## 2. 设计参数

|参数|值|说明|
|---|---|---|
|VOUT|5V||
|VIN范围|14.8V ~ 25.2V|4S标称~6S满电|
|IOUT（设计值）|1A|实际峰值约500mA，留100%余量|
|IOUT实际峰值|~515mA|见下方负载统计|
|f_SW|400kHz|RT/CLK电阻设定|
|拓扑|非同步Buck|外挂续流二极管|

### 2.1 5V负载统计

|供电域|负载|电流|
|---|---|---|
|3.3V_Clean（TLV75733P #1）|STM32F405 MCU|150mA（max）|
|3.3V_Clean|ICM-42688P ×2|2mA|
|3.3V_Dirty（TLV75733P #2）|SD卡 burst写入|200mA（peak）|
|3.3V_Dirty|ELRS接收机|~50mA|
|子板RT9013|GPS ATGM336H|30mA|
|子板RT9013|BME280 + QMC5883L|~3mA|
|5V直供|蜂鸣器（间歇性）|80mA|
|**总计**||**~515mA peak**|

---

## 3. 开关频率选定

**f_SW = 400kHz**，通过RT/CLK引脚外接电阻到GND设定。

**RT电阻计算（Eq 7）：**

$$R_T = \frac{101756}{f_{SW}^{1.008}} = \frac{101756}{400^{1.008}} = 242,k\Omega$$

**选定值：R_T = 243kΩ**（标准值）

**选择依据：** 36×36mm小板，频率太低电感体积大；频率太高对0.5A轻负载效率损失不值得。400kHz是合理中间值。

### 3.1 最大开关频率验证（Eq 9/10）

**Eq 9 — pulse skip上限：**（R_dc=190mΩ, V_d=700mV, R_DS(on)=92mΩ）

$$f_{SW(max,skip)} = \frac{1}{t_{ON}} \times \frac{I_O \times R_{dc} + V_{OUT} + V_d}{V_{IN} - I_O \times R_{DS(on)} + V_d} = \frac{1}{135ns} \times \frac{1 \times 0.19 + 5 + 0.7}{25.2 - 1 \times 0.092 + 0.7} = 1.69,MHz$$

**Eq 10 — frequency foldback上限：**（I_CL=7.5A, V_OUT(SC)=0.1V, f_DIV=8）

$$f_{SW(shift)} = \frac{f_{DIV}}{t_{ON}} \times \frac{I_{CL} \times R_{dc} + V_{OUT(SC)} + V_d}{V_{IN} - I_{CL} \times R_{DS(on)} + V_d} = \frac{8}{135ns} \times \frac{7.5 \times 0.19 + 0.1 + 0.7}{25.2 - 7.5 \times 0.092 + 0.7} = 5.23,MHz$$

400kHz远低于两个上限。✓

---

## 4. 电感选型

### 4.1 最小电感值计算（Eq 28）

$$L_{min} = \frac{V_{IN(max)} - V_{OUT}}{I_{OUT} \times K_{IND}} \times \frac{V_{OUT}}{V_{IN(max)} \times f_{SW}}$$

参数：VIN(max)=25.2V, VOUT=5V, IOUT=1A, K_IND=0.3, f_SW=400kHz

$$L_{min} = \frac{25.2 - 5}{1 \times 0.3} \times \frac{5}{25.2 \times 400000} = 67.3 \times 0.496\mu = 33.4,\mu H$$

**选定值：L = 33µH**

### 4.2 纹波电流验证

$$\Delta I_L = K_{IND} \times I_{OUT} = 0.3 \times 1A = 0.3A = 300mA$$

满足TPS54560要求的 >150mA 下限。✓

### 4.3 峰值电感电流

$$I_{L(peak)} = I_{OUT} + \frac{\Delta I_L}{2} = 1 + 0.15 = 1.15A$$

### 4.4 选型要求

- 电感值：33µH
- I_sat ≥ 2A（留余量）
- DCR ≈ 190mΩ
- 对于1A负载，DCR损耗 = I²×DCR = 1²×0.19 = 0.19W，可接受

### 4.5 公式来源说明

电感公式本质是从 ΔI = (VIN - VOUT) × t_ON / L 推导：

- t_ON = VOUT / (VIN × f_SW)，来自Buck稳态条件（导通期电感电流上升量 = 关断期下降量）
- K_IND = ΔI / IOUT
- 代入解L即得Eq 28

VIN取VIN(max)是因为该工况下占空比最小、(VIN-VOUT)最大，纹波电流最大——最恶劣工况。

---

## 5. 输出电容选型

**选定：3 × 47µF MLCC（GRM31CR60J476ME19）+ 100nF高频去耦**

### 5.1 DC Bias降额

GRM31CR60J476ME19在5V DC bias下实际电容 = 18.5µF/颗（SimSurfing实测）。 3颗并联有效电容 = **55.5µF**。

### 5.2 ESR（SimSurfing实测）

单颗ESR @ 400kHz = 2mΩ，3颗并联 = **0.67mΩ**。

### 5.3 三个约束条件计算

**Eq 32 — 负载瞬态响应：**

$$C_{OUT} > \frac{2 \times \Delta I_{OUT}}{f_{SW} \times \Delta V_{OUT}} = \frac{2 \times 0.5}{400k \times 0.2} = 12.5,\mu F$$

（ΔI_OUT=0.5A为worst case负载跳变，ΔV_OUT=4%×5V=0.2V）

**Eq 33 — 能量吸收/过冲：**

$$C_{OUT} > L \times \frac{I_{OH}^2 - I_{OL}^2}{V_f^2 - V_i^2} = 33\mu \times \frac{1^2 - 0.25^2}{5.2^2 - 5^2} = 33\mu \times \frac{0.9375}{2.04} = 15.2,\mu F$$

（I_OH=1A, I_OL=0.25A, V_f=5.2V, V_i=5V）

**Eq 34 — 纹波电压（目标25mV）：**

$$C_{OUT} > \frac{1}{8 \times f_{SW}} \times \frac{I_{RIPPLE}}{V_{ORIPPLE}} = \frac{1}{8 \times 400k} \times \frac{0.3}{0.025} = 3.75,\mu F$$

**最严格约束：Eq 33 → 15.2µF**

55.5µF >> 15.2µF，满足。✓

---

## 6. 续流二极管

**选定：B560C（60V Schottky，SMA封装，Vf=700mV@5A）**

替换原SS54。SS54额定40V，在6S（VIN=25.2V）下余量 = 40/25.2 = 1.59倍，刚好卡在1.5倍工程余量线上。配合ESC端680µF电容和飞控输入电容的双重抑制，SS54也能工作，但B560C（60V）更安心。

实际负载<1A，二极管工作极轻松，不同厂家的B560C性能差异可忽略，选LCSC销量大、有货的即可。

---

## 7. 输入电容

**选定：3 × 4.7µF/50V MLCC（GRM31CR71H475KA12，1206 X7R）+ 100nF高频去耦**

### 7.1 最小电容要求

Datasheet Section 8.2.2.6要求输入有效电容 ≥ 3µF（X5R/X7R）。

3 × 4.7µF = 14.1µF标称。50V额定在25.2V（6S满电）下DC bias降额约20%，有效电容 ≈ 11.3µF >> 3µF。✓

耐压50V > VIN(max) 25.2V。✓

### 7.2 输入纹波电流（Eq 38）

$$I_{CI(rms)} = I_{OUT} \times \sqrt{\frac{V_{OUT}}{V_{IN(min)}} \times \frac{V_{IN(min)} - V_{OUT}}{V_{IN(min)}}} = 1 \times \sqrt{\frac{5}{14.8} \times \frac{9.8}{14.8}} = 0.47,A$$

所选电容的纹波电流额定值需覆盖此值。SimSurfing Temp.rise曲线显示该电容在0.47A下温升几乎为零。✓

### 7.3 输入纹波电压（Eq 39）

$$\Delta V_{IN} = \frac{I_{OUT} \times 0.25}{C_{IN} \times f_{SW}} = \frac{1 \times 0.25}{11.3\mu \times 400k} = 55,mV$$

输入纹波55mV < VIN的1%（148mV），满足。✓

0.25为worst case占空比因子：精确公式中ΔV_IN ∝ D×(1-D)，D×(1-D)在D=0.5时取最大值0.25。

### 7.4 Layout要求

输入电容紧贴TPS54560的VIN引脚和续流二极管阳极之间放置，最小化VIN→IC→二极管→电容GND的高频电流环路面积。

---

## 8. Bootstrap电容

**0.1µF，X5R/X7R，≥10V**，接在BOOT和SW引脚之间。TPS54560内部集成bootstrap充电二极管，不需要外挂。

BOOT电容为高侧MOSFET栅极提供驱动电荷。TPS54560内部MOSFET的Qg约3nC，0.1µF电容上的电压损失 = Q/C = 3nC/100nF = 30mV，可忽略。值过小会导致栅极驱动电压塌陷，值过大会影响高占空比下的BOOT刷新。0.1µF是datasheet推荐的平衡值。

---

## 9. 反馈分压电阻（Eq 3）

$$R_{HS} = R_{LS} \times \frac{V_{OUT} - 0.8V}{0.8V} = 10.2k \times \frac{5 - 0.8}{0.8} = 10.2k \times 5.25 = 53.55,k\Omega$$

**选定值：R_HS = 53.6kΩ, R_LS = 10.2kΩ**（均为1%精度）

原理：稳态时FB引脚电压被钳在内部基准0.8V → 0.8V = VOUT × R_LS / (R_HS + R_LS)

---

## 10. EN引脚配置

**EN悬空。** 内部1.2µA上拉电流源自动拉高EN，上电即启动。内部UVLO阈值4.3V生效（VIN低于4.3V时锁定，高于4.3V时释放）。无需外部分压电阻。

4S最低放电电压约12V，6S约18V，均远高于4.3V内部UVLO阈值，不存在误锁定风险。

---

## 11. 补偿网络设计（Type 2A）

### 11.1 IC内部参数

| 参数    | 值          | 来源                    |
| ----- | ---------- | --------------------- |
| gm_ea | 350 µA/V   | Datasheet Section 6.6 |
| Aol   | 10,000 V/V | Datasheet Section 6.5 |
| gm_ps | 17 A/V     | Datasheet Section 6.5 |
| V_REF | 0.8V       | Datasheet Section 6.5 |

### 11.2 Modulator pole（Eq 44）

$$f_{p(mod)} = \frac{I_{OUT}}{2\pi \times V_{OUT} \times C_{OUT}} = \frac{1}{2\pi \times 5 \times 55.5\mu} = 573,Hz$$

本质是功率级的主极点：f = 1/(2π × R_L × C_OUT)，其中R_L = VOUT/IOUT = 5Ω。 负载越轻（IOUT越小），R_L越大，极点越低。

### 11.3 ESR zero（Eq 45）

$$f_{z(mod)} = \frac{1}{2\pi \times R_{ESR} \times C_{OUT}} = \frac{1}{2\pi \times 0.67m \times 55.5\mu} = 4.3,MHz$$

MLCC的ESR极低，ESR零点远在MHz级别，对补偿设计无影响。

### 11.4 Crossover frequency目标

$$f_{co1} = \sqrt{f_{p(mod)} \times f_{z(mod)}} = \sqrt{573 \times 4300000} = 49.6,kHz$$

$$f_{co2} = \sqrt{f_{p(mod)} \times \frac{f_{SW}}{2}} = \sqrt{573 \times 200000} = 10.7,kHz$$

**选定 f_co = 20kHz**（两个估算值之间的合理目标）

### 11.5 补偿电阻 R4（Eq 48）

$$R4 = \frac{2\pi \times f_{co} \times C_{OUT}}{gm_{ps}} \times \frac{V_{OUT}}{V_{REF} \times gm_{ea}}$$

$$= \frac{2\pi \times 20000 \times 55.5\mu}{17} \times \frac{5}{0.8 \times 350\mu} = 0.41 \times 17857 = 7326,\Omega$$

**选定值：R4 = 7.5kΩ**

### 11.6 补偿电容 C5（Eq 49，零点对齐modulator pole）

$$C5 = \frac{1}{2\pi \times R4 \times f_{p(mod)}} = \frac{1}{2\pi \times 7500 \times 573} = 37,nF$$

**选定值：C5 = 39nF**

### 11.7 补偿电容 C8（Eq 50/51，高频极点）

$$Eq,50:; C8 = \frac{C_{OUT} \times R_{ESR}}{R4} = \frac{55.5\mu \times 0.67m}{7500} = 5,pF$$

$$Eq,51:; C8 = \frac{1}{R4 \times f_{SW} \times \pi} = \frac{1}{7500 \times 400000 \times \pi} = 106,pF$$

取较大值。**选定值：C8 = 100pF**

### 11.8 补偿网络汇总

|元件|值|作用|
|---|---|---|
|R4|7.5kΩ|控制中频增益，设定crossover frequency|
|C5|39nF|R4串C5形成零点（f_Z ≈ 573Hz），对齐modulator pole|
|C8|100pF|形成高频极点，滚降开关频率附近的噪声增益|

### 11.9 补偿原理说明

TPS54560使用外部Type 2A补偿网络（R串C，并联C到GND），连接在COMP引脚和GND之间。

信号链路：输出电压 → FB分压采样 → 误差放大器（gm_ea）比较FB和内部0.8V基准 → COMP引脚输出 → 控制PWM占空比 → 调节输出电压。

补偿网络塑造误差放大器的频率响应：

- **零点Z1**（R4串C5）放在modulator pole附近（573Hz），补偿LC功率级引入的相位滞后
- **极点P2**（C8）放在高频处，压死开关纹波噪声
- **R4**设定中频增益平台高度，使环路增益在目标带宽（20kHz）处穿越0dB

C5比典型应用（4700pF）大很多，原因是本设计IOUT=1A（轻负载），modulator pole低至573Hz（典型应用5A时为1821Hz），零点需要放在更低频率，要求更大的电容。

---

## 12. 双LDO架构

### 12.1 3.3V_Clean（TLV75733PDBVR #1）

**负载：** MCU（STM32F405）、双ICM-42688P、子板I2C传感器（通过连接器） **峰值电流：** ~152mA **输入端电容：** 1µF + 100nF **输出端电容：** 1µF + 100nF **VDDA磁珠供电：** BLM18KG601SN1 → 独立VDDA滤波

### 12.2 3.3V_Dirty（TLV75733PDBVR #2）

**负载：** SD卡（burst写入200mA peak）、ELRS接收机（~50mA） **峰值电流：** ~250mA **输入端电容：** 1µF + 100nF **输出端电容：** 1µF（LDO端） **SD卡端电容：** 10µF + 100nF（紧贴卡座VDD引脚）

### 12.3 隔离目的

SD卡burst写入时的百毫安级突发电流会在3.3V轨道上产生纹波。双LDO将"脏"负载与IMU/MCU物理隔离，防止SD卡写入噪声耦合到Gyro数据中。

### 12.4 PCB Layout要求

- SD卡及其LDO放在板子角落/边缘
- SD卡的GND独立打via回到主GND，回流路径不得穿过MCU或IMU下方的地平面

---

## 13. TVS保护

**SMAJ20A已删除。** 原设计中SMAJ20A用于保护RT8289（34V耐压）免受反电动势尖峰击穿。现TPS54560耐压60V，配合ESC端680µF电容吸收尖峰，无需额外TVS。

---

## 14. VBAT ADC分压

沿用原设计：**75kΩ / 10kΩ**，兼容4S/6S。

ADC滤波电容（100nF）的GND紧靠MCU VSSA，远离Buck区域。

---

## 15. 元件清单（Buck模块）

|元件|型号/值|封装|说明|
|---|---|---|---|
|U_Buck|TPS54560DDAR|HSOP-8 PowerPAD|Buck IC，60V/5A|
|D_catch|B560C|SMA|60V Schottky续流二极管|
|L|33µH|待选型|I_sat≥2A，DCR≈190mΩ，LCSC选购|
|C_OUT ×3|47µF/6.3V|1206 (GRM31)|GRM31CR60J476ME19，有效55.5µF|
|C_OUT_HF|100nF|0603|输出高频去耦|
|C_IN ×3|4.7µF/50V|1206 (GRM31)|GRM31CR71H475KA12，有效~11.3µF|
|C_IN_HF|100nF|0603|输入高频去耦|
|C_BOOT|0.1µF/10V|0603|X5R/X7R，BOOT-SW之间|
|R_HS (FB)|53.6kΩ|0603|反馈上分压，1%精度|
|R_LS (FB)|10.2kΩ|0603|反馈下分压，1%精度|
|R_T|243kΩ|0603|RT/CLK→GND，设定400kHz|
|R4 (COMP)|7.5kΩ|0603|补偿电阻|
|C5 (COMP)|39nF|0603|补偿零点电容|
|C8 (COMP)|100pF|0603|补偿极点电容|
|EN|悬空|—|内部上拉，UVLO 4.3V|