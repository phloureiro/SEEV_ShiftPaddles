/*
Nome ALUNO A- José Arthur Masiero Faria
Nome ALUNO B- Pedro Henrique Soares Mourão Loureiro
IPLEIRIA - Instituto Politécnico de Leiria
ESTG - Escola Superior de Tecnologia e Gestão
LEEC- Licenciatura em Engenharia Eletrotécnica e de Computadores
SEEV - Sistemas Elétricos e Eletrónicos de Veículos

TP1:Pretende-se  neste  trabalho  prático desenvolver um miniprojeto de forma a demonstrar o
funcionamento de uma caixa de velocidades sequencial com sistema de patilhas para troca de marcha,
mais conhecidos como shift paddles.

LINK: https://www.youtube.com/watch?v=2vnfWa2ZqgQ

*/
#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>

/* Used as a loop counter to create a very crude delay. */
#define mainDELAY_LOOP_COUNT  400000 // ( 0xffffff )

//PINOS do DISPLAY OLED
#define TFT_MOSI 23  	//Data out
#define TFT_SCLK 18  	//Clock out
#define TFT_CS   2		//Chip Select
#define TFT_RST  21 	//Reset
#define TFT_DC   19		//Register Select

//LEDs shift light
#define LED1 5
#define LED2 25
#define LED3 27
#define LED4 15
#define LED5 4
#define LED6 16
#define LED7 17
#define LED8 22
#define LED9 12

//POTENCIOMETRO
#define POTPIN 34 				//Pino de entrada do potenciometro
#define POT_RESOLUTION 12		//Resolucao da leitura

//MOTOR
#define FREQ 5000			//Frequencia do motor
#define RESOLUTION 8		//Resolucao do PWM
#define PWMPIN 14			//Pino PWM do motor
#define PWMCHANNEL 0		//Canal de controlo PWM

//SENSOR DE HALL
#define HALLIN 26				//Pino de entrada do sensor de hall
#define HALL_RESOLUTION 12		//Resolucao da leitura

//BOTOES
#define INT1 32				//botao1 downshif
#define INT2 33				//botao2 upshift
#define MAX_RPM_UP 1900

//ATRIBUICAO DOS PINOS DO DISPLAY
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK,
TFT_RST);

void vTask_shiftlight(void *pvParameters);
void vTask_pedal(void *pvParameters);
void vTask_motor(void *pvParameters);
void Handler_hall(void);
void vTask_hall(void *pvParameters);
void Handler_down_shift(void);
void Handler_up_shift(void);
void vTask_down_shift(void *pvParameters);
void vTask_up_shift(void *pvParameters);
void vTask_display(void *pvParameters);
void escreve_texto(char*, uint16_t, short, short, short);
void retangulos(short, short);

SemaphoreHandle_t xBinarySemaphore_Hall;
SemaphoreHandle_t xBinarySemaphore_down_shift;
SemaphoreHandle_t xBinarySemaphore_up_shift;

SemaphoreHandle_t xMutexSPI;

QueueHandle_t xQueuePedal;
QueueHandle_t xQueueTPS;
QueueHandle_t xQueueMudanca;
QueueHandle_t xQueueHall;
QueueHandle_t xQueueRPM;

void setup(void) {

	int inicial_state = 0;
	int inicial_hall = 0;

	// Define a funcao "setup" como maxima prioridade ate ser apagada
	vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

	//configuracao do potenciometro
	analogReadResolution(POT_RESOLUTION);

	Serial.begin(921600);

	//configuracao do motor
	pinMode(PWMPIN, OUTPUT);					//PWMPIN como saida
	ledcSetup(PWMCHANNEL, FREQ, RESOLUTION);//configuracao da frequancia e resolucao do PWM
	ledcAttachPin(PWMPIN, PWMCHANNEL);//Definir o pino de entrada PWM para o motor

	//definir leds como output
	pinMode(LED1, OUTPUT);
	pinMode(LED2, OUTPUT);
	pinMode(LED3, OUTPUT);
	pinMode(LED4, OUTPUT);
	pinMode(LED5, OUTPUT);
	pinMode(LED6, OUTPUT);
	pinMode(LED7, OUTPUT);
	pinMode(LED8, OUTPUT);
	pinMode(LED9, OUTPUT);

	//leds devem estar desligados
	digitalWrite(LED1, LOW);
	digitalWrite(LED2, LOW);
	digitalWrite(LED3, LOW);
	digitalWrite(LED4, LOW);
	digitalWrite(LED5, LOW);
	digitalWrite(LED6, LOW);
	digitalWrite(LED7, LOW);
	digitalWrite(LED8, LOW);
	digitalWrite(LED9, LOW);

	//configuracao dos botoes
	pinMode(INT1, INPUT_PULLUP); //botao downshift esta em constante estado ascendente
	pinMode(INT2, INPUT_PULLUP); //botao upshift esta em constante estado ascendente

	//Associacao do botao downshift à funcao  Handler_down_shift quando ocorre flanco descendente
	attachInterrupt(digitalPinToInterrupt(INT1), Handler_down_shift, FALLING);
	//Associacao do botao upshift à funcao  Handler_up_shift quando ocorre flanco descendente
	attachInterrupt(digitalPinToInterrupt(INT2), Handler_up_shift, FALLING);

	//configuracao do sensor de hall
	pinMode(HALLIN, INPUT_PULLUP); //sensor de hall esta em constante estado ascendente
	//Associacao do sensor de hall à funcao  Handler_hall quando ocorre flanco descendente
	attachInterrupt(digitalPinToInterrupt(HALLIN), Handler_hall, FALLING);

	xQueuePedal = xQueueCreate(1, sizeof(float)); //Queue para armazenar valores do potenciometro

	xQueueTPS = xQueueCreate(1, sizeof(float)); //Queue para armazenar a % do potenciometro

	xQueueMudanca = xQueueCreate(1, sizeof(int)); //Queue para armazenar o ultimo led ligado
	xQueueSendToFront(xQueueMudanca, &inicial_state, 0); //envia um valor inicial (0) para a queue

	xQueueHall = xQueueCreate(1, sizeof(int)); //Queue para armazenar um contador de interrupcoes
	xQueueSendToFront(xQueueHall, &inicial_hall, 0); //envia um valor inicial (0) para a queue

	xQueueRPM = xQueueCreate(1, sizeof(float));	//Queue para armazenar as RPM do motor

	vSemaphoreCreateBinary(xBinarySemaphore_down_shift);
	vSemaphoreCreateBinary(xBinarySemaphore_up_shift);
	vSemaphoreCreateBinary(xBinarySemaphore_Hall);

	//Mutex para proteger display
	xMutexSPI = xSemaphoreCreateMutex();

	xTaskCreatePinnedToCore(vTask_shiftlight, "SHIFTLIGHT", 1024, NULL, 1, NULL, 1);

	xTaskCreatePinnedToCore(vTask_pedal, "PEDAL", 1024, NULL, 3, NULL, 1);

	xTaskCreatePinnedToCore(vTask_motor, "MOTOR", 1024, NULL, 4, NULL, 1);

	xTaskCreatePinnedToCore(vTask_hall, "HALL", 1024, NULL, 2, NULL, 1);

	xTaskCreatePinnedToCore(vTask_up_shift, "UPSHIFT", 1024, NULL, 5, NULL, 1);

	xTaskCreatePinnedToCore(vTask_down_shift, "DOWNSHIFT", 1024, NULL, 5, NULL, 1);

	xTaskCreatePinnedToCore(vTask_display, "DISPLAY", 4096, NULL, 1, NULL, 1);

	vTaskDelete( NULL);

}

void vTask_shiftlight(void *pvParameters) {

	portBASE_TYPE xStatus;
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	float TPS = 0.0, RPM = 0.0, bin = 0.0;

	int Ledpins[] = { 5, 25, 27, 15, 4, 16, 17, 22, 12 };
	int rotacao[] = { 1400, 1450, 1500, 1550, 1600, 1650, 1700, 1750, 1800 }; //ACENDE O LED DE ACORDO COM O RPM
	int quantLeds = 9;
	int warning = 1900;

	for (;;) {
		//Atualiza as RPM
		xStatus = xQueuePeek(xQueueRPM, &RPM, 0);
		if (xStatus != pdTRUE)
			Serial.print("Nao recebeu valor das RPM no display\r\n");

		for (int i = 0; i < quantLeds; i++) {
			Ledpins[i] = rotacao[i];

			if (RPM > 1400 && Ledpins[1] == rotacao[1])
				digitalWrite(LED1, HIGH);
			if (RPM > 1450 && Ledpins[2] == rotacao[2])
				digitalWrite(LED2, HIGH);
			if (RPM > 1500 && Ledpins[3] == rotacao[3])
				digitalWrite(LED3, HIGH);
			if (RPM > 1550 && Ledpins[4] == rotacao[4])
				digitalWrite(LED4, HIGH);
			if (RPM > 1600 && Ledpins[5] == rotacao[5])
				digitalWrite(LED5, HIGH);
			if (RPM > 1650 && Ledpins[6] == rotacao[6])
				digitalWrite(LED6, HIGH);
			if (RPM > 1700 && Ledpins[7] == rotacao[7])
				digitalWrite(LED7, HIGH);
			if (RPM > 1750 && Ledpins[8] == rotacao[8])
				digitalWrite(LED8, HIGH);
			if (RPM > 1800 && Ledpins[9] == rotacao[9])
				digitalWrite(LED9, HIGH);

			if (RPM > 1900
					&& Ledpins[1, 2, 3, 4, 5, 6, 7, 8, 9]
							== rotacao[1, 2, 3, 4, 5, 6, 7, 8, 9]) {
				vTaskDelay(100);
				//leds devem estar desligados
				digitalWrite(LED1, LOW);
				digitalWrite(LED2, LOW);
				digitalWrite(LED3, LOW);
				digitalWrite(LED4, LOW);
				digitalWrite(LED5, LOW);
				digitalWrite(LED6, LOW);
				digitalWrite(LED7, LOW);
				digitalWrite(LED8, LOW);
				digitalWrite(LED9, LOW);
				vTaskDelay(100);
				//leds devem estar desligados
				digitalWrite(LED1, HIGH);
				digitalWrite(LED2, HIGH);
				digitalWrite(LED3, HIGH);
				digitalWrite(LED4, HIGH);
				digitalWrite(LED5, HIGH);
				digitalWrite(LED6, HIGH);
				digitalWrite(LED7, HIGH);
				digitalWrite(LED8, HIGH);
				digitalWrite(LED9, HIGH);
			}

			if (RPM < 1400 && Ledpins[1] == rotacao[1])
				digitalWrite(LED1, LOW);
			if (RPM < 1450 && Ledpins[2] == rotacao[2])
				digitalWrite(LED2, LOW);
			if (RPM < 1500 && Ledpins[3] == rotacao[3])
				digitalWrite(LED3, LOW);
			if (RPM < 1550 && Ledpins[4] == rotacao[4])
				digitalWrite(LED4, LOW);
			if (RPM < 1600 && Ledpins[5] == rotacao[5])
				digitalWrite(LED5, LOW);
			if (RPM < 1650 && Ledpins[6] == rotacao[6])
				digitalWrite(LED6, LOW);
			if (RPM < 1700 && Ledpins[7] == rotacao[7])
				digitalWrite(LED7, LOW);
			if (RPM < 1750 && Ledpins[8] == rotacao[8])
				digitalWrite(LED8, LOW);
			if (RPM < 1800 && Ledpins[9] == rotacao[9])
				digitalWrite(LED9, LOW);

		}
	}

}

void vTask_pedal(void *pvParameters) {
	short analog_value = 0;
	float conversao = 0.0, TPS = 0.0;
	portBASE_TYPE xStatus;
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	for (;;) {
		analog_value = analogRead(POTPIN);
		//converte o valor analogico em valor de tensao segundo uma equacao linear
		conversao = 0.00057387 * (float) analog_value + 0.95;

		xStatus = xQueueSendToFront(xQueuePedal, &conversao, 0);
		if (xStatus != pdTRUE)
			Serial.print("Nao enviou dados para queue do pedal\r\n");

		//converte em % a posicao do potenciometro
		TPS = 0.024429 * (float) analog_value;
		xStatus = xQueueOverwrite(xQueueTPS, &TPS);
		if (xStatus != pdTRUE)
			Serial.print("Nao enviou dados para queue do TPS\r\n");

		vTaskDelayUntil(&xLastWakeTime, (100 / portTICK_PERIOD_MS));
	}
}

void vTask_motor(void *pvParameters) {
	short state = 0;
	float pedal = 0.0, rotacao = 0.0;
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	portBASE_TYPE xStatus;

	for (;;) {
		//Recebe valor do potenciometro
		xStatus = xQueueReceive(xQueuePedal, &pedal, 0);
		if (xStatus != pdTRUE)
			Serial.print("Nao recebeu pedal no motor\r\n");

		xStatus = xQueuePeek(xQueueMudanca, &state, 0);
		if (xStatus != pdTRUE)
			Serial.print("Nao recebeu led no motor\r\n");

		//De acordo com a mudança, o motor escolhe uma relacao para executar
		switch (state) {
		case 1:
			rotacao = 41.62 * pedal + 75.67;					//mudanca 1
			break;
		case 2:
			rotacao = 31.62 * pedal + 117.67;					//mudanca 2
			break;
		case 3:
			rotacao = 27.62 * pedal + 127.67;					//mudanca 3
			break;
		case 4:
			rotacao = 25.62 * pedal + 137.67;					//mudanca 4
			break;
		case 5:
			rotacao = 21.62 * pedal + 147.67;					//mudanca 5
			break;
		default:
			rotacao = 103.41; //Rotacao de ralenti
		}

		ledcWrite(PWMCHANNEL, (int) rotacao); //Envia valor duty cycle para o motor
		vTaskDelayUntil(&xLastWakeTime, (100 / portTICK_PERIOD_MS));
	}
}

void Handler_hall(void) {
	portBASE_TYPE xStatus;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	short cont;

	//Este metodo permite evitar variaveis globais recebendo e enviando um contador atualizado para uma queue

	//Recebe valor do contador
	xStatus = xQueueReceiveFromISR(xQueueHall, &cont, 0);
	if (xStatus != pdTRUE)
		Serial.print("Nao recebeu valor contador no hall\r\n");

	cont++; // incrementa o contador

	//Reescreve o valor de contador de volta na queue
	xStatus = xQueueOverwriteFromISR(xQueueHall, &cont, 0);
	if (xStatus != pdTRUE)
		Serial.print("Nao enviou valor contador no hall\r\n");
	//Da um semaforo a vtask_hall
	xSemaphoreGiveFromISR(xBinarySemaphore_Hall, &xHigherPriorityTaskWoken);
}

void vTask_hall(void *pvParameters) {

	short pulses = 0;
	float RPM = 0.0, pulses_read = 0.0;
	portBASE_TYPE xStatus;
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	for (;;) {
		//tarefa fica em block state enquanto nao receber semaforo
		xSemaphoreTake(xBinarySemaphore_Hall, portMAX_DELAY);

		//recebe o contador do sensor de hall
		xStatus = xQueueReceive(xQueueHall, &pulses, 0);
		if (xStatus != pdTRUE)
			Serial.print("Nao recebeu valor contador no hall\r\n");

		//Para maior precisao sao adquiridos dois valores de contador
		pulses_read = (pulses_read + pulses) / 2;
		RPM = 122.64 * (float) pulses_read - 30.7;
		//Serial.println((int) RPM);

		//Da o valor da RPM a queue
		xStatus = xQueueOverwrite(xQueueRPM, &RPM);
		if (xStatus != pdTRUE)
			Serial.print("Nao deu valor das RPM ao display\r\n");

		Serial.println(RPM);

		//Coloca a variavel a 0 para contar de novo o numero de vezes que existe interrupcoes
		pulses = 0;
		//Da o valor a queue
		xStatus = xQueueSendToBack(xQueueHall, &pulses, 0);
		if (xStatus != pdTRUE)
			Serial.print("Nao deu valor contador no hall\r\n");
		vTaskDelayUntil(&xLastWakeTime, (500 / portTICK_PERIOD_MS));
	}
}

void Handler_down_shift(void) {
	//permite ativar um semaforo para ativar a vTask_down_shift
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xBinarySemaphore_down_shift,
			&xHigherPriorityTaskWoken);
}

void Handler_up_shift(void) {
	//permite ativar um semaforo para ativar a vTask_up_shift
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xBinarySemaphore_up_shift, &xHigherPriorityTaskWoken);
}

void vTask_down_shift(void *pvParameters) {
	short mudanca = 0;
	uint32_t lastInterrupt = 0;
	portBASE_TYPE xStatus;
	//pega o semaforo para nao haver execucao da tarefa na inicializacao
	xSemaphoreTake(xBinarySemaphore_down_shift, 0);

	for (;;) {
		//tarefa fica em block state enquanto nao receber semaforo
		xSemaphoreTake(xBinarySemaphore_down_shift, portMAX_DELAY);

		//Interrompe durante 300ms as interrupcoes para evitar floating do botao visto ser mecanico
		if (millis() - lastInterrupt > 300) {

			//recebe o valor da ultima mudanca
			xStatus = xQueueReceive(xQueueMudanca, &mudanca, 0);
			if (xStatus != pdTRUE)
				Serial.print("Nao recebeu valor para downshift\r\n");

			//Redução da mudança
			if (mudanca >= 1) {
				mudanca--;
			}

			//envia o estado atual do ultimo led ligado
			xStatus = xQueueSendToFront(xQueueMudanca, &mudanca, 0);
			if (xStatus != pdTRUE)
				Serial.print("Nao enviou valor para downshift\r\n");

			lastInterrupt = millis();
		}
		vTaskDelay(20);
	}
}

void vTask_up_shift(void *pvParameters) {
	short mudanca = 0;
	static uint32_t lastInterrupt = 0;
	portBASE_TYPE xStatus;
	//pega o semaforo para nao haver execucao da tarefa na inicializacao
	xSemaphoreTake(xBinarySemaphore_up_shift, 0);

	for (;;) {
		//tarefa fica em block state enquanto nao receber semaforo
		xSemaphoreTake(xBinarySemaphore_up_shift, portMAX_DELAY);

		//Interrompe durante 300ms as interrupcoes para evitar floating do botao visto ser mecanico
		if (millis() - lastInterrupt > 300) {

			//recebe o estado do led
			xStatus = xQueueReceive(xQueueMudanca, &mudanca, 0);
			if (xStatus != pdTRUE)
				Serial.print("Nao recebeu valor para up_shift\r\n");

			//de acordo com o estado recebido, escolhe o a mudança
			if (mudanca < 5) {
				mudanca++;
			}

		}
		//atualiza a mudanca na queue
		xStatus = xQueueSendToFront(xQueueMudanca, &mudanca, 0);
		if (xStatus != pdTRUE)
			Serial.print("Nao enviou valor para up_shift\r\n");

		lastInterrupt = millis();
	}
	vTaskDelay(20);
}

void vTask_display(void *pvParameters) {
	portBASE_TYPE xStatus;
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	float TPS = 0.0, RPM = 0.0, bin = 0.0;
	short mudanca = 0;
	char buffer[20];

	//Incicializacao do display OLED
	tft.initR(INITR_BLACKTAB);	//Init ST7735S chip, black tab, INICIA DISPLAY
	tft.fillScreen(ST77XX_BLACK); 	//fundo preto
	tft.setTextWrap(true); 			//ajusta ao ecra
	tft.setRotation(3);				//define orientacao
	escreve_texto("MUDAN A:", ST77XX_WHITE, 2, 13, 2);
	escreve_texto("TPS:   %", ST77XX_WHITE, 2, 41, 2);
	escreve_texto("RPM:", ST77XX_WHITE, 2, 69, 2);
	escreve_texto("", ST77XX_WHITE, 2, 97, 2);
	//escreve carater especial (Ç)
	tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);	//escolhe cor da letra
	tft.setTextSize(2);				//tamanho da letra
	tft.setCursor(62, 13);			//definir posicao de escrita
	tft.cp437(true);					//ativa carateres especiais
	tft.write(128);					//escreve a letra na posição 128

	for (;;) {
		//Pega no mutex
		xSemaphoreTake(xMutexSPI, portMAX_DELAY);

		//Atualiza a mudanca
		xStatus = xQueuePeek(xQueueMudanca, &mudanca, 0);
		if (xStatus != pdTRUE)
			Serial.print("Nao recebeu valor led no display\r\n");

		sprintf(buffer, "%d", mudanca);
		for (int v = 98; v < 122; v = v + 12)
			//retangulos(v, 13);
			escreve_texto(buffer, ST77XX_RED, 98, 13, 2);

		//Atualiza o TPS em %
		xStatus = xQueuePeek(xQueueTPS, &TPS, 0);
		if (xStatus != pdTRUE)
			Serial.print("Nao recebeu valor de TPS no display\r\n");

		sprintf(buffer, "%3d", (int) TPS);
		for (int v = 50; v < 86; v = v + 12)
			//retangulos(v, 41);
			escreve_texto(buffer, ST77XX_RED, 50, 41, 2);

		//Atualiza as RPM
		xStatus = xQueuePeek(xQueueRPM, &RPM, 0);
		if (xStatus != pdTRUE)
			Serial.print("Nao recebeu valor das RPM no display\r\n");

		sprintf(buffer, "%4d", (int) RPM);
		for (int v = 50; v < 98; v = v + 12)
			//retangulos(v, 69);
			escreve_texto(buffer, ST77XX_RED, 50, 69, 2);

		if (RPM > MAX_RPM_UP && mudanca < 5) {
			sprintf(buffer, "SHIFT UP");
			escreve_texto(buffer, ST77XX_RED, 2, 97, 2);
		} else
			tft.fillRect(0, 90, 120, 30, ST77XX_BLACK);

		//Da o mutex
		xSemaphoreGive(xMutexSPI);
		vTaskDelayUntil(&xLastWakeTime, (738 / portTICK_PERIOD_MS));
	}
}

//Funcao que permite colocar pixeis pretos no lugar dos carateres a mudar no display, de modo a apagar o valor anterior
void retangulos(short x, short y) {
	for (int i = x; i < x + 5 * 2; i++) {
		for (int j = y; j < y + 7 * 2; j++) {
			tft.drawPixel(i, j, ST77XX_BLACK);
		}
	}
}

//Escreve no display
void escreve_texto(char *text, uint16_t color, short x, short y,
		short font_size) {
	tft.setCursor(x, y);			//definir posicao de escrita
	tft.setTextColor(color, ST77XX_BLACK);			//escolhe cor da letra
	tft.setTextSize(font_size);			//tamanho da letra
	tft.print(text);
//imprime o texto
}

void loop() {
	vTaskDelete( NULL);
}
