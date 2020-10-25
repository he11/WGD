#include <SoftwareSerial.h>

// Debugging
#define __DEBUG__
//#define __TEST__
#ifdef __DEBUG__
#define debug Serial
#endif

// Device Identifier
#define GAS_DETECTOR_ID 0xFF

// State define
#define INVERT_ON_STATE(n) (!(n))
typedef enum {OFF, ON} sw_state_t;

// SOS Switch
#define SOSSW D2 // PA12
static volatile bool sos_requested = OFF;

// LCD Menu
#include "ssd1306.h"
//#include "hangul.h"
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
	REQ_STEP = 0x09, // Real-time step
	REQ_BATT = 0x13, // Remaining device battery
	REQ_HRV  = 0x56 // HRV (HRV(D1), Vascular Occulusion(D2), Heart Rate(D3), Fatigue(D4))
} req_cmd_t;

typedef enum {
	ACK_STEP = 0x09, // ACK real-time step
	ACK_BATT = 0x13, // ACK remaining device battery
	ACK_HRV  = 0x56 // ACK HRV (HRV(D1), Vascular Occulusion(D2), Heart Rate(D3), Fatigue(D4))
} ack_cmd_t;

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
#define BLE2MCU_BUFF_MAX_SIZE 15
static uint8_t bleToMcuBuff[BLE2MCU_BUFF_MAX_SIZE];
static volatile bool received = OFF;

// MCU to RF447
#define MCU2RF447_DATA_SIZE 14
static uint8_t mcuToRf447Data[MCU2RF447_DATA_SIZE];
typedef enum {
	//DEV_ID = , // Device Id
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
void sendToBLE(const uint8_t characteristic, const uint8_t cmd);
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
	debug.println("ssd1306 init complete");
}

// Parse serial data
bool parseData() {
	const uint8_t* s_pkt = bleToMcuBuff;
	const uint8_t cmd = *(s_pkt++);
	const uint8_t* s_data = s_pkt;

	if (!(cmd & 0x80)) {
#ifdef __DEBUG__
		debug.println("in parse func");
		for (int i=0; i<BLE2MCU_BUFF_MAX_SIZE; i++) {
			debug.printf("%02X ", bleToMcuBuff[i]);
		}
		debug.println();
#endif
#if 0
		if (cmd == REQ_STEP) { process_step(s_data); }
		else if (cmd == REQ_BATT) { process_battery(s_data); }
		else if (cmd == REQ_HRV) { process_hrv(s_data + 9); }
#endif
	} else { return false; } // Error processing

	return true;
}

bool serial_event(uint8_t cmd) {
	bool data_start = false;
	uint16_t data_sum = 0;
	bool is_data = false;
	uint8_t cnt = 0;

	memset(bleToMcuBuff, 0, BLE2MCU_BUFF_MAX_SIZE);
	while (toBLE.available() > 0) {
		uint8_t get_data = toBLE.read();
		data_start |= (get_data == cmd)? true : false;
		if (!data_start) { continue; }
		if (cnt == BLE2MCU_BUFF_MAX_SIZE) {
			uint8_t calc_crc = data_sum & (uint8_t)0xFF;
#ifdef __DEBUG__
			debug.printf("get: %02X / crc: %02X\n", get_data, calc_crc);
#endif
			if ((calc_crc != cmd) && (calc_crc == get_data)) {
				is_data = true;
				break;
			} else { return false; }
		}

		data_sum += get_data;
		bleToMcuBuff[cnt++] = get_data;
	}
#ifdef __DEBUG__
	debug.print("get data: ");
	for (int i=0; i<BLE2MCU_BUFF_MAX_SIZE; i++) {
		debug.printf("%02X ", bleToMcuBuff[i]);
	}
	debug.println();
#endif

	return (is_data)? parseData() : false;
}

void setup()
{
	setup_serial();
	setup_timer();
	setup_gpio();
//	setup_lcd();
}

void loop()
{
	bool abnormal = checkAbnormal();

	if (onTime || abnormal) { //TODO: gas delay
		periodTimer->pause();
		onTime = OFF;

		debug.println("on period~");
#if 0 // protocol bingsin
		// Get Current Steps
		while (!serial_event(ACK_STEP)) {
			sendToBLE(REQ_BATT, 0x01);
			delay(100);
		}
#endif
		// Get Device Battery
		while (!serial_event(ACK_BATT)) {
			sendToBLE(REQ_BATT, 0x99);
			delay(100);
		}
#if 0 // Impossible
		//toBLE.flush();
		// Get HRV
		while (!serial_event(ACK_HRV)) {
			sendToBLE(REQ_HRV, 0x00);
			delay(100);
		}
#endif
		delay_start = ON;
		periodTimer->resume();
	}

	if(sos_requested) {
		debug.println("sos~");
#if 0 // protocol bingsin
		// Get Current Steps
		while (!serial_event(ACK_STEP)) {
			sendToBLE(REQ_BATT, 0x01);
			delay(100);
		}
#endif
		// Get Device Battery
		while (!serial_event(ACK_BATT)) {
			sendToBLE(REQ_BATT, 0x99);
			delay(100);
		}

		//toBLE.flush();
		sendToBLE(REQ_HRV, 0x00);
		delay(100);
		serial_event(ACK_HRV);
#if 0 // Impossible
		// Get HRV
		while (!serial_event(ACK_HRV)) {
			sendToBLE(REQ_HRV, 0x00);
			delay(100);
		}
#endif
		sos_requested = OFF;
	}

	bool is_busy = digitalRead(RF447_IS_BUSY);
#ifdef __DEBUG__
	//debug.printf("is busy: %d\n", is_busy);
#endif
	if (received && !(is_busy)) {
		mcuToRf447Data[0] = 0xAA;
		mcuToRf447Data[1] = 0xBB;
		mcuToRf447Data[2] = 0xCC;
		mcuToRf447Data[3] = received;
		sendToRF447(mcuToRf447Data);
		received = OFF;
	}

	if (display_on && !(--display_time)) {
		ssd1306_displayOff();
		display_on = OFF;
	}
#ifdef __DEBUG__
	//debug.printf("display time: %d\n", display_time);
#endif

	if (display_menu) {
		debug.println("menu~");
		display_on = ON;
		display_time = DISPLAY_ON_TIME(3);
		ssd1306_displayOn();
#if 0
		sendToBLE(REQ_HRV, 0);
		//while (!toBLE.available());
		bool m = serial_event();
		digitalWrite(LCD_OLED_EN, HIGH);
#endif
//		uint8_t
//		sprintf(, "%d", );
		ssd1306_clearScreen();
		char* test = "테스트 한글입력";
	//	matrixPrint(0, 8, test);
#if 0
		ssd1306_printFixed (0,  8, "장치ID : ", STYLE_NORMAL);
		ssd1306_write('A');
		ssd1306_printFixed (0,  16, "걸음수 : ", STYLE_NORMAL);
		ssd1306_printFixed (0,  24, "혈압장치 : ", STYLE_NORMAL);
		ssd1306_printFixed (0,  32, "가스농도 : ", STYLE_NORMAL);
		ssd1306_printFixed (0,  40, "배터리 잔량 : ", STYLE_NORMAL);
		//ssd1306_printFixed (0, 16, "Line 2. Bold text", STYLE_BOLD);
#endif
		display_menu = OFF;
	}

WAIT:
	delay(500);
}

void sendToBLE(const uint8_t characteristic, const uint8_t cmd) {
	uint8_t requestCmd[15] = {characteristic, cmd, 0,};
	if (characteristic == REQ_HRV) {
		requestCmd[2] = 0xBB;
		requestCmd[3] = 0xCC;
	}

	uint16_t calc_crc = 0;
	for (int i=0; i<15; i++) {
		toBLE.write(requestCmd[i]);
		calc_crc += requestCmd[i];
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
	int calc_steps = 0;
	for (int i=0; i<4; i++) { calc_steps |= (data[i]<<(i<<3)); }
#if 1
	mcuToRf447Data[STEPS] = calc_steps;
#else
	memcpy(mcuToRf447Data[STEPS], calc_steps, 4);
#endif
}

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
	return (gas >= 250);
}

int checkGas() {
	int gas_volume = analogRead(GAS_PIN);
#if 0 //def __DEBUG__
	debug.print("GAS[");
	debug.print(gas_volume);  
	debug.print("]");
	debug.println();
#endif
	return gas_volume;
}
