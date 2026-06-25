**ELECTRIC STEERING MOTOR** KY173DD01005-08 KY173DD01005-08-HC KY173DD01005-08-HC-ZSKG 

0 

## Revision history 

|Versions|Content|Revision Date|Revised by|
|---|---|---|---|
|V1.0|First edition|Oct. 12, 2023|CFL|
|V1.1|Supplement<br>position<br>mode description for RS232<br>signal|Apr. 17, 2024|CFL|
|V1.2|Update motor drawing|Aug. 8th, 2024|CFL|
|V1.3|Revise factory default parameters|Dec. 11, 2024|CFL|
|V1.4|Add motor drawingfor model KY173DD01005-08-HC|Apr. 1, 2025|CFL|
|V1.5|1. Add CAN configuration tool instructions<br>2. Add external start/stop status bit<br>3. Add electrical angle CAN query command<br>4. Add motor drawing for model<br>KY173DD01005-08-HC-ZSKG<br>(incl. status switch)|May 14, 2025|LZJ|
|V1.6|1. Change the instructions of CAN configuration tool<br>version C2<br>2. Make notes on connector models and definition of 3<br>motor models|May 22, 2025|CFL|
|V1.7|Add the descriptions and examples to CAN control<br>demands|Sept. 13,<br>2025|ZMY|
|V1.8|Upgrade the controller with reverse polarity protection,<br>configuration modification with CAN demands, remove<br>RS232 control signal, and motor wiring definition<br>revisions.|Sept. 26,<br>2025|CFL|



Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

1 

## **Table of Contents** 

**==> picture [462 x 541] intentionally omitted <==**

**----- Start of picture text -----**<br>
|||||
|---|---|---|---|
|1. Overview|.......................................................................................................|错误！未定义书签。|
|1.1 KY173DD01005-08 specifications..............................................................|错误！未定义书签。|
|1.2 KY173DD01005-08-HC specifications.......................................................................................3|
|1.3 KY173DD01005-08-HC-ZSKG specifications ........................................................................... 4|
|1.4 Scope of application...................................................................................................................4|
|1.5 Testing conditions...................................................................................................................... 4|
|2. Functions and Parameters|.................................................................................................... 6|
|2.1 Main functions............................................................................................................................ 6|
|2.2 Working modes...........................................................................................................................6|
|2.3 Technical parameters .................................................................................................................6|
|3. Interface Descriptions..................................................................|错误！未定义书签。|
|3.1 Port definitions of KY173DD01005-08........................................................|错误！未定义书签。|
|3.2 Port definitions of KY173DD01005-08-HC................................................................................ 8|
|3.3 Port defintions of KY173DD01005-08-HC-ZSKG|(incl. status switch)......................................9|
|4|．|Operation Instructions|........................................................................................................ 11|
|4.1 Configuration Instructions of CAN|configuration tool...............................|错误！未定义书签。|1|
|4.2 Use Instructions of CAN|configuration tool..............................................................................11|
|4.3 Functions of parameters .............................................................................|错误！未定义书签。|
|4.4 Indicator lights.............................................................................................|错误！未定义书签。|
|4.5 CAN commands..........................................................................................|错误！未定义书签。|
|5. Failures Protection and Reset|............................................................................................27|
|6. Zeroing Reset (For internal use only)...................................................................27|
|7|．|Firmware Upgrade|................................................................................................................ 28|

**----- End of picture text -----**<br>


Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

2 

## **I. Overview** 

## 1.1 KY173DD01005-08 specifications 

## 1.2 KY173DD01005-08-HC specifications 

Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

3 

1.3 KY173DD01005-08-HC-ZSKG specifications 

- 1.4 Scope of application 

   - Continuous current 10A, Max. Current 17A 5s 

   - DC power supply rated voltage 12/24V (power supply range 9~37VDC) 

   - Speed mode, position mode 

- 1.5 Testing Conditions 

## 1.5.1 Power supply 

- Rated working power: 12VDC/24VDC (power supply range 9 ~ 37VDC). It’s recommended to use a battery as the power supply. 

- The power source should provide instantaneous current overload capability of 2 times continuous current. 

## 1.5.2 Requirements for operation 

- Operators must wear electrostatic wristbands; 

- The workbench must have anti-static measures; 

- It is strictly forbidden to plug and unplug the aviation plug with power on. 

Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

4 

## 1.5.3 Working environment: 

- Operating temperature: -25~55°C (based on ambient temperature); 

   - Storage temperature: -35~65°C (based on ambient temperature); 

- Humidity: 5%--90%RH, condensation (25°C) 

- Protection level: IP55 (in case of not mentioned in motor drawing), 

- Insulation performance: input to the chassis DC600V, leakage current 0.07mA.The insulation resistance is 20 MΩ or more. 

- Three-proof requirements: Meet the requirements of three defenses (dust-proof, moisture proof, salt spray proof). 

- Vibration requirements: frequency 5Hz ~ 25Hz, amplitude 3mm, 0.09g. Frequency 25Hz~200Hz, amplitude 1.47mm, 116g. 

   - Horizontal, vertical, and longitudinal directions are 30mins in each direction. 

- Cooling method: natural cooling 

Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

5 

## **2. Functions and Parameters** 

2.1 Main Functions 

- Working modes: speed mode, position mode 

- Feedback part: Linear encoder. 

- Control port: CAN 

- Realize motor speed control and data reading via CANbus 

- Internal temperature monitoring and protection of built-in controller 

- Overcurrent and overload protection 

- Overvoltage and undervoltage protection 

- Stall and over speed protection 

- Motor short circuit protection 

## 2.2 Working Modes 

**==> picture [377 x 63] intentionally omitted <==**

**----- Start of picture text -----**<br>
||||
|---|---|---|
|Working mode|Control signal|Feedback part|
|Speed mode|CAN|Linear encoder|
|Position mode|CAN|Linear encoder|

**----- End of picture text -----**<br>


2.3 Technical Parameters 

**==> picture [421 x 272] intentionally omitted <==**

**----- Start of picture text -----**<br>
|||||
|---|---|---|---|
|Parameters|Label|Parameter value|Unit|
|Limit power voltage|U|7-37|VDC|
|Max continuous current|Ic|10|A|
|Max peak current|Imax|Default 17 (adjustable)|A|
|PWM switching|
|fpwm|8|kHz|
|frequency|
|Output encoder power|+5Vout|5|VDC|
|supply|Icc|100|mA|
|Under voltage|
|Vu|Default 7 (adjustable)|V|
|protection|
|Over voltage|
|Vo|Default 37(adjustable)|V|
|protection|

**----- End of picture text -----**<br>


Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

6 

|Operating<br>temperature|Industrial grade<br>(standard product)|-25 ~ +55|℃|
|---|---|---|---|
||Low temp. grade|-40 ~ +65||
|Storage temperature|Industrial grade<br>(standard product)|-35 ~ +65|℃|
||Low temperature<br>grade|-55 ~ +85||



Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

7 

## **3. Interface Descriptions** 

## 3.1PortdescriptionsofKY173DD01005-08 

Connector type: DEUTSCH DT04-8P（male） 

## Connector Model: DT04-08P (Male), DT06-8S (Female), Brand：DEUTSCH 

|Port No.|Definition|Description|Recommended Wires|
|---|---|---|---|
|1|IN+|Power input +|2.5mm²|
|2|IN-|Power input -|2.5mm²|
|3|NC|||
|4|NC|||
|5|NC|||
|6|CAN-H|CAN-H|0.5 mm²|
|7|CAN-L|CAN-L|0.5 mm²|
|8|NC|||



- 3.2 Port definitions of KY173DD01005-08-HC 

Connector model: M16-8ZK-1 (Male) M16-8TJ-1 (Female) Brand: Ningbo Jinghao 

Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

8 

|Port No.|Definition|Description|Recommended Wires|
|---|---|---|---|
|1|IN+|Power input +|2.5mm²|
|2|NC|||
|3|IN-|Power input -|2.5mm²|
|4|NC|||
|5|NC|||
|6|CAN-H|CAN-H|0.5 mm²|
|7|CAN-L|CAN-L|0.5 mm²|
|8|NC|||



- 3.3 Port definitions of KY173DD01005-08-HC-ZSKG 

Connector model: WF16K8Z (2+6) (Male) WF16J8TR (Female) Brand: Ningbo Jinghao 

|Port No.|Definition|Description|Recommended Wires|
|---|---|---|---|
|1|IN+|Power input +|2.5mm²|
|2|NC|||
|3|IN-|Power input -|2.5mm²|
|4|NC|||
|5|NC|||
|6|CAN-H|CAN-H|0.5 mm²|



Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

9 

|7|CAN-L|CAN-L|0.5 mm|0.5 mm²|
|---|---|---|---|---|
|8|NC||||



Notes: 

- (1) CAN-H, CAN-L: To realize command control via CANBUS, parameter settings and operating state 

- commissioning, etc. 

- (2) **IN+ IN-** : Because the power cable required by the vehicle is long, and the voltage drop will increase due to wire loss when the current is high, it is recommended to select the cable diameter according to below table. 

Cautions: When the motor “undervoltage alarm”, there may be the following reasons: 

|Cable length (m)|Cable diameter (mm²)|Allowable continuous current (A)|
|---|---|---|
|1-3|2.5|＜17A|
|3-4.5|4|＜25A|



battery is aging, and the internal resistance of the battery will increase after a long time of use, thereby reducing the battery's discharge capacity. 

- The steering hydraulic pump is aging, the flow valve is blocked, etc., which causes the steering resistance to increase and the motor current to increase as well. 

- The cable diameter is too small or the voltage decreases too much, and when the torque is high, the voltage is pulled down, causing the driver undervoltage while detecting. 

Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

10 

## **4. Operation Instructions** 

- 4.1 Configuration instructions of CAN configuration tool 

   - 4.1.1 The motor parameters can be set by the configuration tool (provided by Keya). 

   - 4.1.2 The tool communicates with the motor controller via Cando_pro module, and the baud rate supports 250K, 500K, 1M. 

   - 4.1.3 The tool is applicable to Windows 8 with 64bit operating system or higher. 

- 4.2 Use instructions of CAN configuration tool 

ServoCAN tool is a user-friendly, visual configuration software specifically developed for auto 

steering motor using CAN communication. Currently, it only supports connection to a computer via the Cando_pro module (contact sales personnel for purchase). Simply double-click V-C2.exe to run it. The Cando_pro module is shown below: 

## 4.2.1 Debug interface 

- 1 Set corresponding node ID (default 1), set Baud rate (default 250k), then click “Connect” 

Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

11 

button on the top right corner. When “Connected: fw” appears on the top left corner, connection is successful. 

- 2 Current control signal of motor controller 

- 3 Current working mode of motor controller 

- 4 Current position sensor type of motor controller 

- 5 Current failure status of motor controller 

- 6 Current running data of motor controller 

- 7 The factory setting of working mode is speed mode, unable to modify, but switch to position mode via CAN command. It’s necessary to click “Disable” button before switching the working mode. 

- 8 The controller encoder count value reset button, please note that the position count value will be reset after clicking "Disable". 

- 9 Speed control mode: First, click “Enable” to enable the motor. Enter the corresponding value (-1000~0~1000 corresponds max. Speed of reverse~0 ~ max. Speed of forward). Click “Disable” to stop the motor. 

- 10 Position control mode: First, click “Enable” to enable the motor. Enter a value in “Speed setting” which represents the running speed at position mode (0~100 corresponds 0~ max. Speed of forward). Enter corresponding position value in “Position given” (0~10000 corresponds 0~360º). 

## 4.2.2 Calibration interface 

- Power supply current 10A or above 

- Set control signal to CAN 

- The motor shaft has no external load and is in a free state 

- Click “Start calibration” button and motor begins to zeroing. Wait until calibration is finished 

(appeared on the bottom left corner), power off the motor and restart it. 

Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

12 

Note: The calibration interface is used during the motor manufacturing. The motor has been well 

calibrated before delivery and users are not allowed to calibrate the motor randomly. 

## 4.2.3 Configuration interface 

Click “Config” and then click “Read settings”. Wait until the settings are downloaded, select the 

parameter to be modified. Enter the desired value in “Value” column, click “Write” behind it. Wait until the 

Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

13 

value in “RAM” column is same as “Value” column, click “Program” at the bottom right corner. After programming is completed, power off the motor and restart it. 

## 4.2.4 Firmware upgrade 

- Select the firmware version to be upgraded and click “Firmware upgrade”. 

- Caution:Do not power off the motor during the upgrade 

- After upgrading, it’s necessary to power off the motor and restart it. 

- 4.3 Functions of Parameters 

- **0000** Identifier. when the system is connected, identify the software communication or serial port 

- control. (No need to modify) 

- **0001** The number of motor poles (this motor is 32 poles) 

- **0002** Rated speed of motor (set to 100) 

- **0003** Maximum current of motor 

- **0004** Pulses per revolution of encoder. It’s set according to the encoder, default value is 2500 (This 

- parameter is invalid and you do not need to set it). 

- **0005** Kp parameter of motor current loop PI control (typical value 200) 

   - Can be modified appropriately. 

- **0006** Ki parameter of motor current loop PI control (typical value 50) 

   - Can be modified appropriately. 

Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

14 

- **0007** Kp parameter of motor speed loop PI control (typical value 500) 

Can be modified appropriately. 

- **0008** Ki parameter of motor speed loop PI control (typical value 30) Can be modified appropriately. 

- **0009-0012** Position loop PID control parameters 

- **0013** Acceleration time while rotation speed control. "3" means the acceleration time from 0 rpm to rated speed is 0.3s. 

- **0015** Zero position compensation of magnetic encoder 

- **0016** Zero position compensation of rotary encoder 

- **0018** Motor system address, or node number of control. 

This parameter is used in the slave of CANBUS. 

For example: set the data to 1, then the ID in CANbus: 0x0600000 + controller set address, it will be (0x06000001) 

- **0019** Control signal selection 

   - 2-CAN control; 3-RS232; 11- CAN/232 

Note: With control signal 11, RS232 and CAN signals can be switched freely. (directly send control commands), but cannot be controlled at the same time; and in this mode, there is no offline detection function. 

- **0020** Working mode selection, including speed control and position control 

   - 1-Speed control, 

   - 3-Absolute position control 

   - 4-Relative position control 

- **0021** CAN bus baud rate selection (Factory setting is 250k) 

   - 1-125k 

   - 2-250k 

3-500k 

3-1000k 

- **0022** Position sensor selection 

   - 12- Linear encoder 

0028 Overload time, 1- 200 corresponds to 0.1s-20s 

Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

15 

- 0033 Zero point of motor, modification is not allowed. 

- 0034 CAN heartbeat upload time (default 20ms) 

   - 0-2000 corresponds to 0-2000ms 

Other parameters: Reserved 

- 4.4 Indicator Lights 

Note: The indicator light is inside the motor and can’t be observed from outside. It’s only used for 

repairing. 

- 4.4.1 Status indicator (RED light): Observe the status of the motor according to the blinking frequency of 

the indicator or according to the corresponding the communication analysis fault bit (On the far 

right is Data1). 

|right is Data1).|||
|---|---|---|
|Number of<br>flashes/DATA bit|Definition|Cause of issue|
|The light is always on|Disabled status|Motor disabled|
|1|Normal status (0=Enable,<br>1=Disable)|Enable status, work properly|
|2|Overvoltage|Supply voltage is higher than upper limit of voltage<br>preset in the software|
|3|Hardware overcurrent<br>protection|Overcurrent protection caused by motor short<br>circuit and field tube damage|
|4|EEPROM error|Data saving error|
|5|Undervoltage|Supply voltage is lower than the lower limit of<br>voltage preset in the software|
|6|None|None|
|7|Software overcurrent protection<br>(Set protection value through<br>software)|The phase current reaches the software setting<br>protection value for 5 seconds and stop output.|
|8|Control mode failure|Wrong control mode selection|
|9|Working mode failure|Speed, torque working mode not selected or<br>wrong selection|
|10|None|None|



Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

16 

|11|Temperature alarm|The temperature is above 85 °C|
|---|---|---|
|12|Hall error|Motor Hall drops off or failure|
|13|Current sensor failure|Internal testing circuit damaged|
|14|232 break|232 mode, no 232 signal input|
|15|CAN break|CAN mode, no CAN signal input|
|16|None|None|



## 4.4.2 Enable indicator (BLUE) 

In any control mode, the blue indicator light will be on when the motor is enabled. 

The indicator light is off when the motor is disabled. 

- 4.5 CAN commands 

## 4.5.1 General configuration 

Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

17 

- CAN bus protocol baud rate 250Kb, 500Kb, 1000Kb, default setting is 250Kb. 

- CAN bus ID with extended ID 

- Sending data format: low digit first, high digit last (hexadecimal) 

- According to CAN2.0B format, the data adopts the query mode. 

- According to the CAN2.0B format, there is a fixed heartbeat and send related data. 

- The watchdog detects the line-off period of 1000ms (speed command is sent continuously, the interval must not exceed 1000ms) 

- Query data returning are hexadecimal data, which needs to be converted into decimal data. 

## 4.5.2 CAN bus commands 

Note 1: The motor controller ID is a decimal number in the configuration software, and in CAN software it is hexadecimal. 

Example 1: The configuration software sets the motor controller ID to 1, and the CAN software ID is 06000001 (extended ID). 

Note 2: ID of sending data: 0x0600000 + motor controller ID (hexadecimal) 

ID of returned data: 0x0580000 + motor controller ID (hexadecimal) 

ID of heartbeat data: 0x0700000 + motor controller ID (hexadecimal) 

Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

18 

From the position where the steering wheel is installed, a forward speed is given to rotate 

counterclockwise, that is, to rotate left, and a reverse speed is given to rotate clockwise, that is, to rotate right. 

Speed: -1000~ +1000 corresponds to negative rated speed~ rated speed 

Position: - 2147483648 ~ +2147483647 (10000/circle) 

Enable = 0x23 0D 20 01 00 00 00 00 Disable = 0x23 0C 20 01 00 00 00 00 Speed = 0x23 00 20 01 00 00 00 00 Position = 0x23 02 20 01 00 00 00 00 

Example: Default sending ID: 0x0600000+ motor controller ID (hexadecimal) Enable: 23 0D 20 01 00 00 00 00 

Returned ID: 0x0580000+ motor controller ID (hexadecimal) 

Data: 60 0D 20 00 00 00 00 00 

Disable: 23 0C 20 01 00 00 00 00 

Returned ID: 0x0580000 + motor controller ID (hexadecimal) 

Data: 60 0C 20 00 00 00 00 00 

Speed control: 23 00 20 01 DATA_L(H) DATA_L(L) DATA_H(H) DATA_H(L) Speed control: 23 00 20 01 03 E8 00 00 Speed 100RPM (0x03E8=1000) Returned ID: 0x0580000 + motor controller ID (hexadecimal) 

Data: 60 00 20 00 00 00 00 00 

Position control: 23 02 20 01 DATA_L(H) DATA_L(L) DATA_H(H) DATA_H(L) Position control: 23 02 20 01 27 10 00 00 Rotate 360°counterclockwise Returned ID: 0x0580000+ motor controller ID (hexadecimal) Data: 60 02 20 00 00 00 00 00 

Control Status Query 

Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

19 

Default sending ID: 0x0600000+motor controller ID (hexadecimal) 

Sending: 40 01 21 01 00 00 00 00 

Returned ID：0x0580000+ motor controller ID (hexadecimal) 

Data：60 01 21 01+【control mode】4bit+【external on/off】4bit+【position sensor】4bit+【control mode】 4bit 

## Example： 

Send：40 01 21 01 00 00 00 00 

Feedback：60 01 21 01 10 C2 ff ff Speed mode+ external (off) +linear encoder + CAN Feedback：60 01 21 01 30 C2 ff ff Position mode + external (on)+ linear encoder + CAN 

Feedback data 0 0x1 0x2 0x3 0x4 0x8 0xb 0xC ~~eeee ee ee ee ee~~ Absolute Relative Working mode Speed position position ~~NRNR ea a~~ Linear encoder Position sensor ~~RR OE~~ Control signal CAN RS232 232/CAN ~~NIN NE m~”_~~ On/Off feedback OFF ON ~~NRKL NVw—~~ 

Remarks: The ON/OFF (start/stop) status signal is only for motor model KY173DD01005-08-HC-ZSKG 

with a status switch. This switch is a self-resetting switch. When pressed, the signal status is 8, and when 

disconnected, it is 0. Customers can define the function of this switch according to their application. 

Motor current query: 

## Send: 40 00 21 01 00 00 00 00 

Returned ID: 0x0580000 + motor controller ID (hexadecimal) 

Data: 60 00 21 01 DATA 00 00 00 

DATA = ((unsigned char*)(&send_float)) 

Fault query: 

Send: 40 12 21 01 00 00 00 00 

Returned ID: 0x0580000 + motor controller ID (hexadecimal) 

Data: 60 12 21 01 DAT1 DAT2 00 00 

DAT1 =((unsigned char*)(&TYPE_RunData.err)) [L] 

Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

20 

DAT2 =((unsigned char*)(&TYPE_RunData.err)) [H] 

TYPE_RunData.err is the fault code. (Failure analysis is same as heatbeat.) 

Encoder speed query: 

Send: 40 03 21 01 00 00 00 00 

Returned ID: 0x0580000 + motor controller ID (hexadecimal) 

Data: 60 03 21 01 DAT1 DAT2 00 00 

DAT1 =((unsigned char*)(&send_float))[L] 

DAT2 =((unsigned char*)(&send_float))[H] 

Power supply voltage query: 

Send: 40 0D 21 02 00 00 00 00 

Returned ID: 0x0580000 + motor controller ID (hexadecimal) 

Data 60 0D 21 02 DATA 00 00 00 

DATA =((unsigned char*)(&send_float)) 

Motor temperature query: 

Send: 40 0F 21 01 00 00 00 00 

Returned ID: 0x0580000 + motor controller ID (hexadecimal) 

Data: 60 0F 21 01 DATA 00 00 00 

DATA = ((unsigned char*)(&send_short)) 

Encoder count value query: (10000/circle) 

Send: 40 04 21 01 00 00 00 00 

Returned ID: 0x0580000 + motor controller ID (hexadecimal) 

Data: 60 04 21 01 DAT1 DAT2 DAT3 DAT4 DAT1 = ((unsigned char*)(&send_int))[4]; DAT2 = ((unsigned char*)(&send_int))[3]; DAT3 = ((unsigned char*)(&send_int))[2]; DAT4 = ((unsigned char*)(&send_int))[1]; 

Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

21 

External sensor AD query (this function is optional): 

Send: 40 05 21 01 00 00 00 00 

Returned ID: 0x0580000 + motor controller ID (hexadecimal) 

Data: 60 05 21 01 DAT1 DAT2 00 00 

DAT1 =((unsigned char*)(&send_float))[L] 

DAT2 =((unsigned char*)(&send_float))[H] 

Note: Input 0-5V, corresponding to 0~5000 

Program version query: 

Send: 40 01 11 12 00 00 00 00 

Returned ID: 0x05800000 + motor controller ID (hexadecimal) Data: 60 01 11 12 DAT1 DAT2 DAT3 DAT4 

DAT1 = ((unsigned char*)(&send_int))[1]; 

DAT2 = ((unsigned char*)(&send_int))[2]; DAT3 = ((unsigned char*)(&send_int))[3]; DAT4 = ((unsigned char*)(&send_int))[4]; 

Electrical angle query: 

Send: 40 06 21 01 00 00 00 00 

Returned ID: 0x05800000 + motor controller ID (hexadecimal) 

Data: 60 06 21 01 DATA_L DATA_H 00 00 

DATA = ((unsigned char*) (&send_short)) 

Heartbeat return command: 

Return ID: 0x07000000 + motor controller ID (hexadecimal) 

Return command: Data0 Data1 Data2 Data3 Data4 Data5 Data6 Data7 

Data0 Data1, Cumulative value of angle (360°/circle) 

Data2 Data3, motor speed: with symbol – speed — + speed 

Data4 Data5, motor current: with symbol 

Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

22 

Data6 Data7, Control_Close (error code) 

(It means the high digit goes first and low digit goes after for heatbeat data.) 

## Example: 

Upload ID: 0x07000001 

Returned command: 01 68 00 14 00 00 40 01 

Cumulative value of angle: 0x0168 360°/circle, when angle value reaches 0xFFFF=65535, automatically clear and recount. 

Error code: 0x4001 (Binary 100000000000001) 

Reporting faults: CAN disconnected or disabled 

Failure analysis is same as serial port error. 

Data6: 

|Data6:||||||||
|---|---|---|---|---|---|---|---|
|Bit7|Bit6|Bit5|Bit4|Bit3|Bit2|Bit1|Bit0|
|None|CAN break|232 break|Current sensing|Hall failure|Temp.<br>protection|None|Working mode|



Data7: 

|Data7:||||||||
|---|---|---|---|---|---|---|---|
|Bit7|Bit6|Bit5|Bit4|Bit3|Bit2|Bit1|Bit0|
|Control<br>signal|Overcurrent|None|Undervoltage|E2PROM|Hardware<br>protection|Overvoltage|Disable|



## 4.5.3 CANBUS control example 

## 4.5.3.1 Speed Control: 

(Speed command value ‰）* (The maximum speed set in the software) = actual speed. 

4.5.3.2 If rated speed is 100rpm, from the position where the steering wheel is installed, a forward speed is given to rotate counterclockwise, that is, to rotate left, and a reverse speed is given to rotate clockwise, that is, to rotate right. 

then the given value of speed command -1000 ~ +1000 represents -100rpm ~ +100rpm (0xFC18）（0x03E8） 

The software setting control signal is CAN control (0019 is set to 2) 

The software setting working mode is set to speed control (0020 is set to 1) 

The system address in the software is 1 (0018 is set to 1) 

Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

23 

- If the given speed +50 (rated speed 100rpm) 

Control command ID: 0x06000001 (extended ID) 

Enable: 23 0D 20 01 00 00 00 00 

Given speed: 23 00 20 01 01 F4 00 00 (0x01F4 = 500) 

- If the given speed -50 (rated speed 100rpm) 

Control command ID: 0x06000001 (extended ID) 

Enable: 23 0D 20 01 00 00 00 00 

Given speed: 23 00 20 01 FE 0C FF FF 

º 4.5.3.3 Position control: The command value 10000 corresponds to 360 .（In RS232 command mode, 

1000 corresponds to 360º） 

The given position value - 50000~+ 50000 represents 5 circles clockwise~5 circles counterclockwise 

(0x3CB0 FFFF) (0XC350 0000) 

The software setting control signal is CAN control (0019 is set to 2) 

The software setting working mode is absolute position control (0020 is set to 3) 

Or the software setting working mode is set to relative position control (0020 is set to 4) 

The system address set in the software is 1 (0018 is set to 1) 

Control command ID: 0x06000001 (extended ID) 

The sequence of sending data: 

- （a） Disable: 23 0C 20 01 00 00 00 00 

- （b） Enable: 23 0D 20 01 00 00 00 00 

- （c） Position control: 23 02 20 01 DATA_L(H) DATA_L(L) DATA_H(H) DATA_H(L) 

For example, make the motor rotate 1.8 circles clockwise 

- （a） Make sure the position control has been switched on. 

- （b） Enable 23 0D 20 01 00 00 00 00 

- （c） Position control command: 23 02 20 01 B9 B0 FF FF 

For example: Make the motor rotate 72º at mechanical angle counterclockwise (72 * (10000 / 360) = 

2000 = 0x07D0) 

- （a） Make sure the position control has been switched on. 

Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

24 

   - （b） Enable: 23 0D 20 01 00 00 00 00 

   - （c） Position control command: 23 02 20 01 07 D0 00 00 

- 4.5.3.4 CAN command switch control mode 

   - 0x230C2009 = Position zeroing； 

   - 0x030D2011 = Speed mode； 

   - 0x030D2031 = Absolute position mode； 

   - 0x030D2041 = Relative position mode. 

   - 1 Speed mode switches to absolute position mode, disable the motor before switching, then send the command to switch to absolute position mode. After switching, send enable & absolute position commands. 

   - 2 When the speed mode switches to the relative position mode, disable the motor before switching, then send the command to switch to relative position mode. After switching, send enable & relative position commands. 

   - ③ Switch from absolute position mode to relative position mode, disable the motor before switching, then send the command to switch to relative position mode. After switching, send enable & relative position commands. 

   - ④ Relative position mode switches to absolute position mode. Disable the motor before switching, then send the command to switch to absolute position mode. After switching, send enable & absolute position commands. 

Example：Default sending ID：0x06000000+ motor controller ID (hexadecimal) 

**Speed mode** : The default mode is speed mode upon power-up. Before switching from other modes to speed mode, disable the motor first and then send the speed switching command. After the switching is completed, send enable +speed commands, the motor starts to run. The examples are as follows: 

Send: 03 0D 20 11 00 00 00 00 

Feedback: 63 0D 20 11 00 00 00 00 

Send：23 0D 20 01 00 00 00 00 Enable 

Send：23 00 20 01 03 E8 00 00 Speed command：0x03E8 （Forward 100RPM） Send：23 00 20 01 FC 18 FF FF Speed command：0xFFFFFC18 （Reverse 100RPM） 

Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

25 

**Absolute position mode** ： Before switching to position mode, first disable the motor, then send position switching command. After switching is completed, send enable and position commands, the motor starts to run. The examples are as follows: 

Send：03 0D 20 31 00 00 00 00 

Feedback：63 0D 20 31 00 00 00 00 

Send：23 0D 20 01 00 00 00 00 Enable 

Send：23 02 20 01 86 A0 00 01 0x186A0 Forward 10 circles（low digit first, high digit last） Send：23 02 20 01 27 40 FF 58 0xFF582740 Reverse 1100 circles（low digit first, high digit last） 

**Relative position mode** ：Before switching to relative position mode, first disable the motor, then send relative position switching command. After switching is completed, send enable and relative position commands, the motor starts to run. The examples are as follows: 

Send：03 0D 20 41 00 00 00 00 

Feedback：63 0D 20 41 00 00 00 00 

Send：23 0D 20 01 00 00 00 00 Enable 

Send：23 02 20 01 86 A0 00 01 0x186A0 Forward 10 circles（low digit first, high digit last） Send：23 02 20 01 79 60 FF FE 0xFFFE7960 Reverse 10 circles（low digit first, high digit last） 

**Set the current position value to zero** : Before zeroing the test position, first disable, then send the test zeroing command. After sending, the rotor mechanical position data is 0. Below is the 

example. 

Send: 23 0C 20 09 00 00 00 00 

Feedback: 63 0C 20 09 00 00 00 00 

Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

26 

## **5. Failure Protection and Reset** 

|No. of flashes|Definition|Reasons|Reset Measures|
|---|---|---|---|
|The light is always<br>on|Disable status|Motor disabled|Send Enable command|
|1|Normal situation|Enabled status, work properly|None|
|2|Overvoltage|Supply voltage is higher than upper limit of<br>voltage preset in the software|Disable reset after supply voltage is<br>normal.|
|3|Hardware overcurrent<br>protection|Overcurrent protection caused by motor<br>short circuit and field tube damage|Electrify again. ( If it cannot be<br>restored, return it to the factory for<br>testing)|
|4|EEPROM error|Data saving error|Electrify again.|
|5|Undervoltage|Supply voltage is lower than the lower limit<br>of voltage preset in the software|It will recover when supply voltage is<br>normal.|
|6|None|None|None|
|7|Software overcurrent<br>protection (Set protection<br>value through software)|The phase current reaches the software<br>setting protection value for 1 second and<br>stop output.|Disable reset|
|8|Control mode failure|Wrong control mode selection|Re-select control signal|
|9|None|None|None|
|10|None|None|None|
|11|Temperature alarm|The temperature is above 85 °C|Reset automatically after the<br>temperature goes down to 70℃|
|12|Hall error|Motor hall failure or wrong selection of<br>position feedback|Check parameters/Power on again (If<br>it cannot be restored, return it to the<br>factory for testing)|
|13|Current sensor failure|Internal current sensor is damaged|Power on again (If it cannot be<br>restored, return it to the factory for<br>testing)|
|14|232 break|232 mode, no 232 signal input|Check RS232 cables or send 232<br>commands|
|15|CAN break|CAN mode, no CAN signal input|Check CAN cables or send CAN<br>commands|
|16|None|None|None|



## **6. Zeroing Reset (For internal use only)** 

Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

27 

- 6.1 With the configuration tool, set parameter No. 19 (control signal) to 2 or 11, and set parameter No. 

33 to zero. 

- 6.2 Power on again. (Make sure the current of power supply is not less than 10A) 

- 6.3 Use CAN device to send commands to the motor: (hexadecimal) 

Id: 0x06000000+system ID 

Data: 23 0C 2F FB 00 00 00 00 

- 6.4 Wait until the failure code 4009 in the heartbeat data is uploaded, then power off and restart. 

   - Id: 07000000+system ID 

Data 00 00 00 00 00 00 40 09 

The whole process lasts about 30s. 

## **7. Firmware update** 

These motor models support BootLoader upgrade and can be directly upgraded through 232 or CAN. 

For details, please ask Keya technical staff for related protocols and commands. 

Email:info@jnky.com 

Address:Building66,XinmaoTechnologyCity,Jinan,Shandong,China 

www.dcmotorkeya.com 

28 

