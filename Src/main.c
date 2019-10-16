/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim16;
DMA_HandleTypeDef hdma_tim8_ch1;

/* USER CODE BEGIN PV */
uint16_t ADC1ConvertedValues[2];
//int bemf_a =0;

char bi_direction = 1;
char reversed_direction = 0;

int zero_input_count =0;

int ticks = 0;
int toggled = 0;
int running = 0;
int advance = 0;
int blanktime;
int START_ARR=800;
int rising = 1;
int count = 0;

int forward = 1;
int error = 0;
int zc_counts;
int threshold_up = 50;
int threshold_down = 0;

int ADCtimer= 30;
int demagtime = 50;

int whatstepisthis = 0 ;       // for debugging
int myneutral = 1300; // for debugging
int myneutralup = 1500;

int IC_buffer_size = 64;
int smallestnumber = 20000;
uint32_t dma_buffer[64];
int propulse[4] = {0,0,0,0};
int dpulse[16] =  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

int calcCRC;
int checkCRC;
int dshotcommand;
int tocheck = 0;
int commandcount = 0;

int max_servo_deviation = 100;
int servorawinput = 0;
int input = 0;
int newinput =0;
int total = 0;
char inputSet = 0;
char dshot = 0;
char proshot = 0;
char multishot = 0;
char oneshot42 = 0;
char oneshot125 = 0;
char servoPwm = 0;

char armed = 0;


int integral = 0;
int threshold = 850;
int zcfound = 0;
int difference;
int bemfcounter;
int min_zc_counts_up = 1;
int min_zc_counts_down = 1;

int adc_timer = 600;
int ROC = 1;

int lastzctime;
int thiszctime;
int upthiszctime;
int uplastzctime;
int wait_time;


int phase = 1;

int tim2_start_arr= 616;

uint32_t last_adc_channel;

int duty_cycle= 33;   // USE EXTREME CAUTION WHEN TESTING

int bemf_rising = 1;

int step = 1;
int pot = 1000;

int commutation_interval = 6000;
int pwm = 1;
int floating =2;
int lowside = 3;
int zero_cross_offset_up= 1;  //  up and down offset adc read
int zero_cross_offset_down= 30;
int sensorless = 1;
int waitTime = 0;
int threshhold = 5;

int signaltimeout = 0;

uint8_t ubAnalogWatchdogStatus = RESET;

int bemf = 0;
int bemfb = 0;
int bemfc = 0;
int neutral = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//////////////////////////////////PHASE 1//////////////////////
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	if (x < in_min){
		x = in_min;
	}
	if (x > in_max){
		x = in_max;
	}
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

}
void phaseBPWM() {




		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_12, LL_GPIO_MODE_ALTERNATE); // low

		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);  // high

	}


void phaseBFLOAT() {




		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_12, LL_GPIO_MODE_OUTPUT);
		GPIOA->BRR = GPIO_PIN_12;
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
		GPIOA->BRR = GPIO_PIN_9;
	}


void phaseBLOW() {


	        // low mosfet on
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_12, LL_GPIO_MODE_OUTPUT);
		GPIOA->BSRR = GPIO_PIN_12;
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
		GPIOA->BRR = GPIO_PIN_9;
	}



//////////////////////////////PHASE 2//////////////////////////////////////////////////


void phaseAPWM() {



		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE);

		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);           //HIGH

	}


void phaseAFLOAT() {


	         // floating
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_11, LL_GPIO_MODE_OUTPUT);           // LOW
		GPIOA->BRR = GPIO_PIN_11;
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);            //HIGH
		GPIOA->BRR = GPIO_PIN_8;
	}



void phaseALOW() {


	            // lowside
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_11, LL_GPIO_MODE_OUTPUT);
		GPIOA->BSRR = GPIO_PIN_11;
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
		GPIOA->BRR = GPIO_PIN_8;
	}



///////////////////////////////////////////////PHASE 3 /////////////////////////////////////////////////



void phaseCPWM() {



		LL_GPIO_SetPinMode(GPIOF, GPIO_PIN_0, LL_GPIO_MODE_ALTERNATE); //low

		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);//high

	}



void phaseCFLOAT() {


		LL_GPIO_SetPinMode(GPIOF, GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
		GPIOF->BRR = GPIO_PIN_0;
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
		GPIOA->BRR = GPIO_PIN_10;
	}



void phaseCLOW() {



		LL_GPIO_SetPinMode(GPIOF, GPIO_PIN_0, LL_GPIO_MODE_OUTPUT); // low
		GPIOF->BSRR = GPIO_PIN_0;
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_10, LL_GPIO_MODE_OUTPUT); //high
		GPIOA->BRR = GPIO_PIN_10;
	}



void  comStep (int newStep){
//TIM14->CNT = 0;
switch(newStep)
{

        case 1:			//A-B
        	phaseAPWM();
        	phaseBLOW();
        	phaseCFLOAT();
        	break;


        case 2:		// C-B
        	phaseAFLOAT();
        	phaseBLOW();
        	phaseCPWM();
        	break;



        case 3:	// C-A
        	phaseALOW();
        	phaseBFLOAT();
        	phaseCPWM();
        	break;


        case 4:// B-A
        	phaseALOW();
        	phaseBPWM();
        	phaseCFLOAT();
        	break;


        case 5:    // B-C
        	phaseAFLOAT();
        	phaseBPWM();
        	phaseCLOW();
        	break;


        case 6:      // A-C
        	phaseAPWM();
        	phaseBFLOAT();
        	phaseCLOW();
        	break;
	}

//stop_time = TIM14->CNT;

}


void allOff() {                   // coast
	phaseAFLOAT();
	phaseBFLOAT();
	phaseCFLOAT();
}

void fullBrake(){                     // full braking shorting all low sides
	phaseALOW();
	phaseBLOW();
	phaseCLOW();
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

			if (htim->Instance==TIM16)
			{

			if(inputSet == 1){
				 if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)){

		     HAL_TIM_IC_Stop_DMA(&htim8,TIM_CHANNEL_1);
			 TIM15->CNT = 0;
//			 for (int i = 0; i < 8; i++){
//			 							dma_buffer[i]=0;
//			 						}


				 HAL_TIM_IC_Start_DMA(&htim8, TIM_CHANNEL_1, dma_buffer , IC_buffer_size);
			 }


			 }

		}

}

void detectInput(){
	smallestnumber = 20000;
	dshot = 0;
	proshot = 0;
	multishot = 0;
	oneshot42 = 0;
	oneshot125 = 0;
	servoPwm = 0;
//	int lastnumber = dma_buffer[0];
	for ( int j = 1 ; j < IC_buffer_size; j++){

		if(dma_buffer[j]  < smallestnumber){ // blank space
			smallestnumber = dma_buffer[j];
		}

	}

	if ((smallestnumber > 3)&&(smallestnumber < 20)){
		dshot = 1;
		TIM8->PSC = 69;
		IC_buffer_size = 32;
		TIM16->ARR = 8000;
	}
	if ((smallestnumber > 20)&&(smallestnumber < 30)){
			dshot = 1;
			TIM8->PSC = 113;
			TIM16->PSC = 4;
			TIM16->ARR = 8000;
			IC_buffer_size = 32;
		}

	if ((smallestnumber > 40 )&&(smallestnumber < 80)){
		proshot = 1;
		TIM8->PSC = 4;
		IC_buffer_size = 8;
		TIM16->ARR = 8000;
	}

	if (smallestnumber > 100){
		servoPwm = 1;
		TIM15->PSC = 169;
//		TIM16->PSC = 47;
//		TIM16->ARR = 10000;
		HAL_TIM_Base_Stop(&htim16);
		IC_buffer_size = 6;

	}

	if (smallestnumber == 0){
		inputSet = 0;
	}else{

		inputSet = 1;

		HAL_Delay(50);
		//	playInputTune();
	}
	HAL_TIM_IC_Start_DMA(&htim8, TIM_CHANNEL_1, dma_buffer , IC_buffer_size);
}

void computeProshotDMA(){

total = dma_buffer[1]+ dma_buffer[2] + dma_buffer[3] + dma_buffer[4]+ dma_buffer[5] + dma_buffer[6] + dma_buffer[7];

   if (( total < 118 && total > 98)&& (dma_buffer[0]> 100)){
   for (int i = 1; i < 8; i +=2){
    propulse[(i-1) / 2] = (dma_buffer[i] - 7);
					}
   }else{

	   return;
   }
	calcCRC = ((propulse[0]^propulse[1]^propulse[2])<<3
							|(propulse[0]^propulse[1]^propulse[2])<<2
							|(propulse[0]^propulse[1]^propulse[2])<<1
							|(propulse[0]^propulse[1]^propulse[2]));

	checkCRC = (propulse[3]<<3 | propulse[3]<<2 | propulse[3]<<1 | propulse[3]);
    if ( checkCRC == calcCRC){
    	tocheck = ((propulse[0]<<7 | propulse[1]<<3 | propulse[2]>>1));
    }else{
 //   	error++;
    }
    if (tocheck > 47 && tocheck < 2048){
    	newinput = tocheck ;
    				commandcount = 0;



    			}else if (tocheck > 1 && tocheck < 48 && input == 0){

    					dshotcommand = tocheck ;
    			}else{
    				commandcount++;
    				if (commandcount > 1){
    				newinput = tocheck ;
    			    commandcount = 0;
    			}
    			}

}

void computeDshotDMA(){

	if (dma_buffer[0] < 20){
		return;
	}

//	for (int i = 1; i < 32; i+=2){
//		dpulse[(i-1)>>1] = dma_buffer[i] ;
//	}
	 calcCRC = ((dma_buffer[1]^dma_buffer[9]^dma_buffer[17])<<3
			          |(dma_buffer[3]^dma_buffer[11]^dma_buffer[19])<<2
					|(dma_buffer[5]^dma_buffer[13]^dma_buffer[21])<<1
					|(dma_buffer[7]^dma_buffer[15]^dma_buffer[23])
				);
//	 calcCRC = ((dpulse[0]^dpulse[4]^dpulse[8])<<3
//			          |(dpulse[1]^dpulse[5]^dpulse[9])<<2
//					|(dpulse[2]^dpulse[6]^dpulse[10])<<1
//					|(dpulse[3]^dpulse[7]^dpulse[11])
//				);
//	 checkCRC = (dpulse[12]<<3 | dpulse[13]<<2 | dpulse[14]<<1 | dpulse[15]);
	 checkCRC = (dma_buffer[25]<<3 | dma_buffer[27]<<2 | dma_buffer[29]<<1 | dma_buffer[31]);
//
			if(calcCRC == checkCRC){
				tocheck = (
						dma_buffer[1]<<10 | dma_buffer[3]<<9 | dma_buffer[5]<<8 | dma_buffer[7]<<7
					| dma_buffer[9]<<6 | dma_buffer[11]<<5 | dma_buffer[13]<<4 | dma_buffer[15]<<3
					| dma_buffer[17]<<2 | dma_buffer[19]<<1 | dma_buffer[21]);
//				success++;
				}else{
					error++;
				}

			if (tocheck > 47 && tocheck < 2048){
				newinput = tocheck;
				commandcount = 0;
			}else if (tocheck > 1 && tocheck < 48 && input == 0){

					dshotcommand = tocheck ;


			}else{
				commandcount++;
				if (commandcount > 1){
				newinput = tocheck ;
			    commandcount = 0;
			}
			}

}


void computeServoInput(){

	if ( dma_buffer[1] < 2000 && dma_buffer[1] > 1000){
		if(dma_buffer[2]< 1000 || dma_buffer[2] > 2500){

		servorawinput = map(dma_buffer[1], 1100,2000,0,2000);
		}
	}else if( dma_buffer[2] < 2000 && dma_buffer[2] > 1000) {
		if(dma_buffer[1]< 1000 || dma_buffer[1] > 2500){
		servorawinput = map(dma_buffer[2], 1100,2000,0,2000);
		}
	}


	if (servorawinput - newinput > max_servo_deviation){
		newinput += max_servo_deviation;
	}else if(newinput - servorawinput > max_servo_deviation){
		newinput -= max_servo_deviation;
	}else{
		newinput = servorawinput;
	}


}


void transferComplete(){

//	compit = 0;
	signaltimeout = 0;

	if (inputSet == 1){
		if (dshot == 1){
			computeDshotDMA();

			return;
		}
		if (proshot == 1){
			computeProshotDMA();

			return;
		}

		if  (servoPwm == 1){
			computeServoInput();
				HAL_TIM_IC_Start_DMA(&htim8, TIM_CHANNEL_1, dma_buffer , 3);
				}
}

}
void switchADCPhase(){
	ADC_ChannelConfTypeDef sConfig;
//	HAL_ADC_Stop_IT(&hadc1);
	//clear old channel
	    /**Configure for the selected ADC regular channel to be converted.
	     *
	     *
	    */

	 if (step == 1 || step == 4){            //        in phase 1 or 4 read from phase c Pf1 adc2
		 HAL_ADC_Stop_IT(&hadc2);
		 HAL_ADC_Stop_IT(&hadc1);
		 //		  sConfig.Rank = ADC_RANK_NONE;
//	//	  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
//		  sConfig.Channel = last_adc_channel;
//		   	   		  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//		   	   		  {
//		   	   		    _Error_Handler(__FILE__, __LINE__);
//		   	   		  }
		   sConfig.Channel = ADC_CHANNEL_10;
		   sConfig.Rank = ADC_REGULAR_RANK_1;

	      if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	      {
	    	  Error_Handler();
	      }
	      HAL_ADC_Start_IT(&hadc2);
		}
   if (step == 2 || step == 5){            //        in phase two or 5 read from phase A PA4 adc2
	   HAL_ADC_Stop_IT(&hadc2);
	   HAL_ADC_Stop_IT(&hadc1);
	   //	  sConfig.Rank = ADC_RANK_NONE;
////	  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
//	  sConfig.Channel = last_adc_channel;
//	   	   		  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//	   	   		  {
//	   	   		    _Error_Handler(__FILE__, __LINE__);
//	   	   		  }

	   	   	sConfig.Channel = ADC_CHANNEL_17;
	   	   	  sConfig.Rank = ADC_REGULAR_RANK_1;

      if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
      {
    	  Error_Handler();
      }
      HAL_ADC_Start_IT(&hadc2);
	}


   if (step == 3 || step == 6){                         // phase B pb0 --- on adc 1 does not change
	   HAL_ADC_Stop_IT(&hadc2);
	   //	     sConfig.Rank = ADC_RANK_NONE;
//	     sConfig.Channel = last_adc_channel;
//	//     sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
//	     if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//				  {
//				    _Error_Handler(__FILE__, __LINE__);
//				  }
//	    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
//		sConfig.Channel = ADC_CHANNEL_1;
//
//	    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//				  {
//				    _Error_Handler(__FILE__, __LINE__);
//				  }
//	    last_adc_channel=ADC_CHANNEL_1;
	   HAL_ADC_Start_IT(&hadc1);      // zc found is not set to 0 until end of zcfound routine
			}
}
void commutate(){
	if (forward == 1){
		step++;
		if (step > 6) {
			step = 1;
		}
		if (step == 1 || step == 3 || step == 5) {
			rising = 1;                                // is back emf rising or falling
		}
		if (step == 2 || step == 4 || step == 6) {
			rising = 0;
		}
	}
	if (forward == 0){
		step--;
		if (step < 1) {
			step = 6;
		}
		if (step == 1 || step == 3 || step == 5) {
			rising = 0;
		}
		if (step == 2 || step == 4 || step == 6) {
			rising = 1;
		}
	}
	comStep(step);
	switchADCPhase();
	bemfcounter = 0;
//	zcfound = 0;
}

void playStartupTune(){
	TIM1->PSC = 75;
	TIM1->CCR1 = 5;
	TIM1->CCR2 = 5;
	TIM1->CCR3 = 5;
	comStep(2);
	HAL_Delay(100);
	TIM1->PSC = 50;
	HAL_Delay(100);
	TIM1->PSC = 25;
	HAL_Delay(100);
	allOff();
	TIM1->PSC = 0;
}

void playInputTune(){
	TIM1->PSC = 100;
	TIM1->CCR1 = 5;
	TIM1->CCR2 = 5;
	TIM1->CCR3 = 5;
	comStep(2);
	HAL_Delay(100);
	TIM1->PSC = 50;
	HAL_Delay(100);
	allOff();
	TIM1->PSC = 0;
}





void zcfoundroutine(){
	thiszctime = TIM2->CNT;
	TIM2->CNT = 0;
	commutation_interval = thiszctime + commutation_interval / 2;
	waitTime = commutation_interval / 2;
	blanktime = commutation_interval / 4;
	while (TIM2->CNT - thiszctime < waitTime - advance){

	}
	commutate();
	zc_counts++;
	while (TIM2->CNT - thiszctime < waitTime + blanktime){

	}
//	HAL_ADC_Start_IT(&hadc);
    zcfound = 0;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//TIM1->CCR4 = adc_timer;


if(!zcfound){
	bemf = HAL_ADC_GetValue(hadc);
if (rising){
	if (bemf > threshold_up){

     bemfcounter++;


	}
    }  else{
	if (bemf <= threshold_down){
		bemfcounter++;

	}
	}
}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_IWDG_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);             // uncomment for comp_pwm
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_Delay(10);


  if (HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_4) != HAL_OK)
  		{
	  Error_Handler();
    }

  if (HAL_ADC_Start_IT(&hadc1) != HAL_OK)
	  return 0;
  if (HAL_ADC_Start_IT(&hadc2) != HAL_OK)
 	  return 0;

  TIM4->CCR4 = 800;                // adc read timer.

  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim8);
  HAL_TIM_Base_Start_IT(&htim16);

  HAL_TIM_IC_Start_DMA(&htim8, TIM_CHANNEL_1, dma_buffer , 64);

  GPIOB->BSRR = GPIO_PIN_7;            // to put bemf into on or off time sense mode, high output for off time ( default)

  playStartupTune();

  //GPIOA->BSRR = GPIO_PIN_11;          // maybe LED?
  running = 0;
  duty_cycle = 1;
	if(HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  count++;
	  if (count  > 20000){
	 	 count = 0;
	 	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
	  }

	  if (inputSet == 0){
	 	 detectInput();
	  }
	 	  if (zero_input_count > 200 && !armed){
	 		  armed = 1;
	 		  playInputTune();
	 	  }

	 	  if (bi_direction == 1 && proshot == 0){

	 		if ( newinput > 1100 ){
	 			if(reversed_direction){
	 			if (forward == 1){
	 				forward = 0 ;
	 			}
	 			}else{
	 			if (forward == 0){
	 				forward = 1 ;
	 			}
	 			}
	 				input = (newinput - 1050)*3;
	 		}

	 		if (newinput < 800) {
	 			if(reversed_direction){
	 				if (forward == 0){
	 				forward = 1;
	 			}
	 			}else{


	 			if (forward == 1){
	 				forward = 0;
	 			}
	 			}
	 				input = (800 - newinput) * 3;
	 		}

	 		if (newinput > 800 && newinput < 1100){
	 			input = 0;

	 		}
	 	}else if((proshot || dshot)&& bi_direction){
	  					if ( newinput > 1100 ){

	  					  if (!forward){
	  						forward = 1 ;
	  					  }
	  						input = (newinput - 1100) * 2 ;


	  					}if ( newinput <= 1047 ){
	  						if(forward){
	  					forward = 0;
	  						}
	  						input = (newinput - 90) * 2 ;
	  					}
	  					if ((newinput > 1047 && newinput < 1100) || newinput < 100){
	  						input = 0;
	  					}


	  				}else{
	 	 input = newinput;
	 }




	 	if ((input > 47) && (armed == 1)) {
	 		  			running = 1;

	 		  			duty_cycle = (input  - 40) * 4;

	 		  			if (zc_counts < 20) {
	 		  				if (duty_cycle < 600) {
	 		  					duty_cycle = 600;
	 		  				}
	 		  				if (duty_cycle > 1800) {
	 		  					duty_cycle = 1800;
	 		  				}
	 		  			}

	 		  			if (zc_counts < 50 ){
	 		  				if (duty_cycle > 2000){
	 		  					duty_cycle = 2000;
	 		  				}
	 		  			}


	 		  				if (duty_cycle > 3000) {                             // safety!!!  set to only 50 percent power
	 		  					duty_cycle = 3000;
	 		  				}
	 		  				if (duty_cycle < 180) {
	 		  					duty_cycle = 180;
	 		  				}

	 		  				if (duty_cycle < 2000){
	 		  		 		 TIM4->CCR4 = 3000;
	 		  		 		 }


	 		  		 	 if (duty_cycle > 2000){
	 		  		 		 TIM4->CCR4 = 6000;
	 		  		 	 }

	 		  				TIM1->CCR1 = duty_cycle;// set duty cycle to 50 out of 768 to start.
	 		  				TIM1->CCR2 = duty_cycle;
	 		  				TIM1->CCR3 = duty_cycle;
	 		  				//	TIM1->CCR4 = duty_cycle;

	  }

	 if ( duty_cycle < 1000){
	 	min_zc_counts_up = 10;
	 	min_zc_counts_down = 10;
	 }else{
	 	min_zc_counts_up = commutation_interval / 5000;
	 	min_zc_counts_down = commutation_interval / 5000;
	 }


	 if (input < 140){
	 	 running = 0;
	 	 duty_cycle = 0;
	 	zc_counts = 0;






	 	 TIM1->CCR1 = duty_cycle;												// set duty cycle to 50 out of 768 to start.
	 	 TIM1->CCR2 = duty_cycle;
	 	 TIM1->CCR3 = duty_cycle;
	 	 TIM4->CNT = TIM1->CNT;                 // lazy synchronization
	  }


	// 	  GPIOF->BRR = GPIO_PIN_0;

	 	 // test speed of adc change

	 	//  GPIOF->BRR = GPIO_PIN_0;

	 	  if(HAL_IWDG_Refresh(&hiwdg) != HAL_OK)                   // watchdog refresh
	 	  			{
	 	  				/* Refresh Error */
	 	  				Error_Handler();
	 	  			}




	  if (!zcfound){
	 	  		  if (rising){
	 	  		 if (bemfcounter > min_zc_counts_up){
	 	  			 GPIOF->BSRR = GPIO_PIN_0;
	 	  			 zcfound = 1;
	 	  			 bemfcounter = 0;
	 	  			 zcfoundroutine();
	 	  		//	 break;

	 	  		}
	 	  		  }else{
	 	  			  if (bemfcounter > min_zc_counts_down){
	 	  			  			 GPIOF->BSRR = GPIO_PIN_0;
	 	  			  			 zcfound = 1;
	 	  			  			 bemfcounter = 0;
	 	  			  			 zcfoundroutine();
	 	  			  //			 break;

	 	  			  		}
	 	  		  }
	 	  	  }
  	  if (TIM2->CNT > 60000 && running == 1){           // this starts the motor
  		//  TIM3->CNT = commutation_interval / 2;
  		  zcfoundroutine();
  		zc_counts = 0;
  	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T4_CC4;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode 
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T4_CC4;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 80;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 35;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 125000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 7000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 3;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 3;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 8000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim16, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_7);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
