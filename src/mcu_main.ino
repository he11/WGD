#include <SoftwareSerial.h>

// Debugging
//#define __DEBUG__
//#define __TEST__
#ifdef __DEBUG__
#define debug Serial
#endif

// Device Identifier
#define GAS_DETECTOR_ID 0xFF

// State define
#define INVERT_ON_STATE(n) (!(n))
typedef enum {OFF, ON} sw_state_t;
static bool pre_state = OFF;

// SOS Switch
#define SOSSW D2 // PA12
static volatile bool sos_requested = OFF;

// LCD Menu
//#include "hangul.h"
#include "ssd1306.h"
#define MENUSW D11 // PB5
#define LCD_OLED_EN A2 // PA3
#define DISPLAY_ON_TIME(n) ((n)*(2))
static bool display_on = OFF;
static uint8_t display_time;
static volatile bool display_menu = OFF;
//#define LCD_I2C_SDA D4 // PB7
//#define LCD_I2C_SCL D5 // PB6

// J1755 Command
#define J1755_ACK_PACKET_SIZE 16
typedef enum {
	REQ_STEP      = 0x09, // Real-time step
	REQ_BATT      = 0x13, // Remaining device battery
	REQ_HRV       = 0x56, // HRV (HRV(D1), Vascular Occulusion(D2), Heart Rate(D3), Fatigue(D4))
	SET_HR_PERIOD = 0x2A // Set HR execution period
} j1755_cmd_t;

typedef enum {
	ACK_STEP = 0x09, // ACK real-time step
	ACK_BATT = 0x13, // ACK remaining device battery
	ACK_HRV  = 0x56 // ACK HRV (HRV(D1), Vascular Occulusion(D2), Heart Rate(D3), Fatigue(D4))
} j1755_ack_t;

// Serial
#define MCU_BLE_RX 10
#define MCU_BLE_TX 9
#define MCU_RF447_RX 0
#define MCU_RF447_TX 1
#define RF447_IS_BUSY A0 // PA0
#define toRF447 Serial
SoftwareSerial toBLE(MCU_BLE_RX, MCU_BLE_TX); // RX, TX
//#define toBLE Serial

typedef enum {
	STX = 0x02, // Start of text
	ETX = 0x03, // End of text
	LF  = 0x0A, // Line feed
	CR  = 0x0D // Carriage return
} uart_cntl_t;

// BLE to MCU
#define BLE2MCU_BUFF_MAX_SIZE 16
static uint8_t bleToMcuBuff[BLE2MCU_BUFF_MAX_SIZE];
static volatile bool received = OFF;

// MCU to RF447
#define MCU2RF447_DATA_SIZE 14
static uint8_t mcuToRf447Data[MCU2RF447_DATA_SIZE];
typedef enum {
	STEPS = 0,  // Current steps 4byte
	VO    = 4,  // Vascular Occulusion 1byte
	HR,         // Heart Rate 1byte
	HRV,        // HRV 1byte
	FATIGUE,    // Fatigue 1byte
	GAS,        // Gas 4byte
	BATT  = 12, // Remaining Battery 1byte
	SOS,        // SOS Signal 1byte
} rf447_pkt_data_t;

// GAS Sensor
#define GAS_PIN A5 // PA6
static volatile int gas;

// Timer
#define DELAY_PERIOD 1
HardwareTimer* periodTimer;
static volatile bool onTime = ON;
static volatile bool delay_start = OFF;

// Function pre-define
void sendToBLE(const uint8_t characteristic, const uint8_t* command, uint8_t cmd_length);
void sendToRF447(const uint8_t* send_data);
bool checkAbnormal();
void process_step(const uint8_t* data);
void process_hrv(const uint8_t* data);
void process_battery(const uint8_t* data);



// Period timer isr
void period_timer()
{
	static uint8_t secCnt, minCnt;

	if (delay_start) {
		secCnt = 0;
		minCnt = 0;
		delay_start = OFF;
	}

	if (++secCnt == 60) {
		if (++minCnt >= DELAY_PERIOD) {
			onTime = ON;
			minCnt = 0;
		}
		secCnt = 0;
	}
}

void setup_timer() {
	TIM_TypeDef* ins1 = TIM1;
	periodTimer = new HardwareTimer(ins1);
	periodTimer->setOverflow(1, HERTZ_FORMAT); // 1HZ
	periodTimer->attachInterrupt(period_timer);
}

void request_sos() {
	sos_requested = ON;
}

void request_menu() {
	display_menu = ON;
}

void setup_gpio() {
	// SOS Switch
	pinMode(SOSSW, INPUT);
	attachInterrupt(digitalPinToInterrupt(SOSSW), request_sos, FALLING);

	// MENU Switch
	pinMode(MENUSW, INPUT);
	attachInterrupt(digitalPinToInterrupt(MENUSW), request_menu, FALLING);
}

void setup_serial() {
#ifdef __DEBUG__
	debug.begin(115200);
#endif
	toBLE.begin(19200);
	while(!toBLE);
	toRF447.begin(115200);
	while(!toRF447);
}

void setup_lcd() {
	ssd1306_128x64_i2c_init();
	ssd1306_fillScreen(0x00);
	ssd1306_setFixedFont(ssd1306xled_font6x8);
	ssd1306_displayOff();
#ifdef __DEBUG__
	debug.println("ssd1306 init complete");
#endif
}

// Set HR checking period
void set_hr_check_period() { // testing...
	uint8_t cmd[14] = {0,};
	cmd[0] = 0x02; // 0: Turn off 1: Work mode in time period 2: Interval work mode within time
	cmd[1] = 0x09; // Start time hour -> 0x01 == 1 hour
	cmd[2] = 0x00; // Start time minute -> 0x30 == 30 minute
	cmd[3] = 0x18; // End time hour -> 0x03 == 3 hour
	cmd[4] = 0x00; // End time minute -> 0x40 == 40 minute
	cmd[5] = 0xFF; // 0: Disable 1: Enable, Bit 0-6: Sunday - Saturday
	cmd[6] = 0x01; // When work mode is 2, how long time to test, unit is minute
	sendToBLE(SET_HR_PERIOD, cmd, 14);
}

// Parse serial data
bool parseData() {
	const uint8_t* pkt_sp = bleToMcuBuff;
	const uint8_t cmd = *pkt_sp;

#ifdef __DEBUG__
	debug.print("get data: ");
	for (int i=0; i<BLE2MCU_BUFF_MAX_SIZE; i++) {
		debug.printf("%02X ", bleToMcuBuff[i]);
	}
	debug.println();
#endif

	if (cmd == ACK_BATT) {
		uint8_t cnt = 0;
		uint16_t calc_crc = 0;
		do { calc_crc += pkt_sp[cnt]; } while (++cnt < J1755_ACK_PACKET_SIZE-1);
#ifdef __DEBUG__
		debug.printf("cnt: %d / origin crc: %02X / calc crc: %02X\n", cnt, pkt_sp[cnt], calc_crc & (uint8_t) 0xFF);
#endif
		if (pkt_sp[cnt] != (uint8_t)(calc_crc & 0xFF)) { return false; }
	}

	const uint8_t* data_sp = ++pkt_sp;

	if (!(cmd & 0x80)) {
		if (cmd == REQ_STEP) { process_step(data_sp); }
		else if (cmd == REQ_BATT) { process_battery(data_sp); }
		else if (cmd == REQ_HRV) { process_hrv(data_sp + 9); }
	} else { // Error processing
#ifdef __DEBUG__
		debug.printf("Response error, code: %02X", cmd);
#endif
		return false;
	}

	return true;
}

bool serial_event(uint8_t cmd) {
	bool start_data = false;
	uint8_t cnt = 0;

	memset(bleToMcuBuff, 0, BLE2MCU_BUFF_MAX_SIZE);
	while (toBLE.available() > 0) {
		uint8_t get_data = toBLE.read();
		start_data |= (get_data == cmd)? true : false;
		if (!start_data) { continue; }
		
		bleToMcuBuff[cnt++] = get_data;
		if (cnt >= BLE2MCU_BUFF_MAX_SIZE) { break; }
	}

	return (*bleToMcuBuff != 0)? parseData() : false;
}

void setup()
{
	setup_serial();
	setup_timer();
	setup_gpio();
	setup_lcd();
	set_hr_check_period();
}

void loop()
{
	bool cur_state = checkAbnormal();
	if (sos_requested) {
		cur_state = true;
	}
	bool abnormal = (cur_state && !pre_state)? ON : OFF;
	pre_state = cur_state;

	if (onTime || abnormal) {
		periodTimer->pause();
		onTime = OFF;
		uint8_t cmd[14] = {0,};

		// Get Current Steps
		cmd[0] = 0x01;
		toBLE.flush();
		while (!serial_event(ACK_STEP)) {
			sendToBLE(REQ_STEP, cmd, 14);
			delay(100);
		}

		// Get Device Battery
		cmd[0] = 0x99;
		toBLE.flush();
		while (!serial_event(ACK_BATT)) {
			sendToBLE(REQ_BATT, cmd, 14);
			delay(100);
		}

		// Get HRV
		cmd[0] = 0x00;
		toBLE.flush();
		while (!serial_event(ACK_HRV)) {
			sendToBLE(REQ_HRV, cmd, 14);
			delay(100);
		}

		received = ON;
		delay_start = ON;
		periodTimer->resume();
	}

	mcuToRf447Data[SOS] = sos_requested;
	if (sos_requested) {
		uint8_t cmd[14] = {0,};

		// Get Current Steps
		cmd[0] = 0x01;
		toBLE.flush();
		while (!serial_event(ACK_STEP)) {
			sendToBLE(REQ_STEP, cmd, 14);
			delay(100);
		}

		// Get Device Battery
		cmd[0] = 0x99;
		toBLE.flush();
		while (!serial_event(ACK_BATT)) {
			sendToBLE(REQ_BATT, cmd, 14);
			delay(100);
		}

		// Get HRV
		cmd[0] = 0x00;
		toBLE.flush();
		while (!serial_event(ACK_HRV)) {
			sendToBLE(REQ_HRV, cmd, 14);
			delay(100);
		}

		received = ON;
		sos_requested = OFF;
	}

	bool is_busy = digitalRead(RF447_IS_BUSY);
#ifdef __DEBUG__
	//debug.printf("is busy: %d\n", is_busy);
#endif
	if (received && !(is_busy)) {
		sendToRF447(mcuToRf447Data);
		received = OFF;
#ifdef __DEBUG__
		debug.print("Total: ");
		for (int i=0; i<MCU2RF447_DATA_SIZE; i++) {
			debug.printf("%02X ", mcuToRf447Data[i]);
		}
		debug.println();
		debug.printf("steps: %d\n", *(int*)(mcuToRf447Data + STEPS));
		debug.printf("battery: %u\n", mcuToRf447Data[BATT]);
#endif
	}

	if (display_on && !(--display_time)) {
		ssd1306_displayOff();
		display_on = OFF;
	}

	if (display_menu) {
		display_on = ON;
		display_time = DISPLAY_ON_TIME(3);
		char lcd_strings[6][22] = {
			"* DEVICE ID   : ",
			"* STEP COUNTS : ",
			"* HEART RATE  : ",
			"* FATIGUE     : ",
			"* GAS DENSITY : ",
			"* BATTERY     : "
		};

		sprintf(*lcd_strings + 16, "%5d", GAS_DETECTOR_ID);
		sprintf(*(lcd_strings + 1) + 16, "%5d", *(int*)(mcuToRf447Data + STEPS));
		sprintf(*(lcd_strings + 2) + 16, "%5d", *(mcuToRf447Data + HR));
		sprintf(*(lcd_strings + 3) + 16, "%5d", *(mcuToRf447Data + FATIGUE));
		sprintf(*(lcd_strings + 4) + 16, "%5d", *(int*)(mcuToRf447Data + GAS));
		sprintf(*(lcd_strings + 5) + 16, "%5d", *(mcuToRf447Data + BATT));

		ssd1306_displayOn();
		ssd1306_clearScreen();
		ssd1306_printFixed (0,   0, "=====================", STYLE_NORMAL);
		ssd1306_printFixed (0,   8, *(lcd_strings + 0), STYLE_NORMAL);
		ssd1306_printFixed (0,  16, *(lcd_strings + 1), STYLE_NORMAL);
		ssd1306_printFixed (0,  24, *(lcd_strings + 2), STYLE_NORMAL);
		ssd1306_printFixed (0,  32, *(lcd_strings + 3), STYLE_NORMAL);
		ssd1306_printFixed (0,  40, *(lcd_strings + 4), STYLE_NORMAL);
		ssd1306_printFixed (0,  48, *(lcd_strings + 5), STYLE_NORMAL);
		ssd1306_printFixed (0,  56, "=====================", STYLE_NORMAL);

		display_menu = OFF;
	}

WAIT:
	delay(500);
}

void sendToBLE(const uint8_t characteristic, const uint8_t* command, uint8_t cmd_length) {
	uint16_t calc_crc = characteristic;

	toBLE.write(characteristic);
	for (int i=0; i<cmd_length; i++) {
		toBLE.write(command[i]);
		calc_crc += command[i];
	}
	toBLE.write((uint8_t)(calc_crc & 0xFF)); // CRC
	toBLE.write(CR);
	toBLE.write(LF);
}

void sendToRF447(const uint8_t* send_data) {
	toRF447.write(STX);
	toRF447.write(GAS_DETECTOR_ID); // Device ID
	toRF447.write(send_data, MCU2RF447_DATA_SIZE);
	toRF447.write(ETX);
}

void process_step(const uint8_t* data) {
	memcpy(mcuToRf447Data + STEPS, data, 4);
}

// Need to fix the ordering 
void process_hrv(const uint8_t* data) {
	mcuToRf447Data[VO] = data[1];
	mcuToRf447Data[HR] = data[2];
	mcuToRf447Data[HRV] = data[0];
	mcuToRf447Data[FATIGUE] = data[3];
}

void process_battery(const uint8_t* data) {
	mcuToRf447Data[BATT] = data[0];
}

bool checkAbnormal() {
	gas = checkGas();
	memcpy(mcuToRf447Data + GAS, (const void*)&gas, 4);

	return (gas >= 250);
}

int checkGas() {
	int gas_volume = analogRead(GAS_PIN);
#ifdef __DEBUG__
	debug.print("GAS[");
	debug.print(gas_volume);  
	debug.print("]");
	debug.println();
#endif
	return gas_volume;
}
