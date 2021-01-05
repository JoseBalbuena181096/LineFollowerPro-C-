/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "gpio.h"

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
#define white 1
#define black 0
#define OUT_LINE  100.0
//-----------------------
#define ON 33456254
#define OFF 33441974
#define ONE 33444014
#define TWO 33478694
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef enum {START,DATA,DATA_FULL}State_ir;
typedef enum {CENTER,RIGHT,LEFT} Out_state;
typedef enum {HOME,STARTING,RUN} State;

class IR_NEC
{
	private:
		uint32_t input_capture;
		uint32_t input_diference;
		uint32_t input_last;
		uint32_t data;
		uint8_t sample;
		State_ir  state;
		TIM_HandleTypeDef *htim;
		uint32_t TIM_CHANNEL;
	public:
		IR_NEC(TIM_HandleTypeDef *htim,uint32_t TIM_CHANNEL);
		void init(void);
		void decode(void);
		State_ir get_state(){return this->state;};
		uint32_t get_data(){return this->data;};
		void restart(void);
};

class Control_PID
{
	private:
		int u;
		float kp;
		float ki;
		float kd;
		float error_now;
		float error_last[6];
		float sum_errors;
	public:
		Control_PID(float kp,float ki,float kd);
		void set_tunings(float kp,float ki,float Kd);
		void compute(float error_now);
		int correction(void){return this->u;};
};

class ESC_Turbine
{
	private:
		int speed;
		TIM_HandleTypeDef *htim;
		uint32_t TIM_CHANNEL;
	public:
		ESC_Turbine(TIM_HandleTypeDef *htim,uint32_t TIM_CHANNEL);
		void init(void);
		void update_speed(int speed);
		void calibrate(void);
};


class Motor_Driver
{
	private:
		int speed;
		int MAX_SPEED;
		GPIO_TypeDef *GPIO;
		uint16_t DIS;
		uint16_t DIR;
		TIM_HandleTypeDef *htim;
		uint32_t TIM_CHANNEL;
	public:
		Motor_Driver(int MAX_SPEED,GPIO_TypeDef *GPIO,uint16_t DIS,uint16_t DIR,TIM_HandleTypeDef *htim,uint32_t TIM_CHANNEL);
		void init(void);
		void set_max_speed(int MAX_SPEED){this->MAX_SPEED = MAX_SPEED;};
		void update_speed(int speed,bool clockwise);
};

class Sensors_qtr
{
	private:
		bool background_color;
		int sensor_qtr[16];
		GPIO_TypeDef *GPIO;
		uint16_t S[4];
		ADC_HandleTypeDef *hadc;
 		Out_state out_state;
	public:
 		Sensors_qtr(ADC_HandleTypeDef *hadc,GPIO_TypeDef *GPIO,uint16_t S0, uint16_t S1, uint16_t S2, uint16_t S3);
 		int read_adc(void);
 		void read_sensors_qtr(void);
 		float update_error();
 		Out_state get_out_state(void){return this->out_state;};
 		int* get_sensors_qtr(void){return this->sensor_qtr;};
};


class Line_Follower
{
	private:
		int speed_left;
		int speed_right;
		int MAX_SPEED;
		int MED_SPEED;
		float error_now;
		State state;
		Control_PID *PID;
		ESC_Turbine *Turbine;
		Sensors_qtr *Sensors;
		Motor_Driver *Motor_left;
		Motor_Driver *Motor_right;
	public:
		Line_Follower();
		void init(void);
		void turn_on_off(IR_NEC *IR);
		void state_machine(void);
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
Line_Follower Line_follower;
IR_NEC Ir_Nec(&htim2,TIM_CHANNEL_1);
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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  Line_follower.init();
  Ir_Nec.init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
	  Line_follower.turn_on_off(&Ir_Nec);
	  Line_follower.state_machine();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
Line_Follower::Line_Follower()
{
	this->MAX_SPEED = 499;
	this->MED_SPEED = 345;
	this->speed_right = 0;
	this->speed_left = 0;
	this->error_now = 0.0;
	this->state = HOME;
	this->Sensors = new Sensors_qtr(&hadc1,GPIOE,S0_Pin,S1_Pin,S2_Pin,S3_Pin);
	this->Turbine = new ESC_Turbine(&htim4,TIM_CHANNEL_1);
	this->Motor_left = new Motor_Driver(this->MAX_SPEED,GPIOC,DIS_A_Pin,DIR_A_Pin,&htim3,TIM_CHANNEL_1);
	this->Motor_right = new Motor_Driver(this->MAX_SPEED,GPIOB,DIS_B_Pin,DIR_B_Pin,&htim3,TIM_CHANNEL_2);
	this->PID = new Control_PID(61.2444,1.0,168.4224);
}

void Line_Follower::init(void)
{
	this->Motor_left->init();
	this->Motor_right->init();
	this->Turbine->init();
	this->Turbine->calibrate();
}
void Line_Follower::turn_on_off(IR_NEC *IR)
{
	if(IR->get_state() == DATA_FULL)
	{
		switch (IR->get_data())
		{
			case ON:
				this->state = STARTING;
		      	break;
		    case OFF:
				this->state = HOME;
		        break;
		    case ONE:
		    	this->PID->set_tunings(51.4257, 1.0, 141.4208);
		        this->MAX_SPEED = 399;
		        this->MED_SPEED = 300;
		        this->Motor_left->set_max_speed(this->MAX_SPEED);
		        this->Motor_right->set_max_speed(this->MAX_SPEED);
		        this->state = STARTING;
		        break;
		    case TWO:
		    	this->PID->set_tunings(61.2444, 1.0, 168.4224);
		    	this->MAX_SPEED = 499;
		    	this->MED_SPEED = 345;
		        this->Motor_left->set_max_speed(this->MAX_SPEED);
		        this->Motor_right->set_max_speed(this->MAX_SPEED);
		    	this->state = STARTING;
		    	break;
		}
		IR->restart();
	}
}

void Line_Follower::state_machine(void)
{
	switch (this->state)
	{
		case HOME:
			this->Turbine->update_speed(12);
			this->speed_left = 0;
			this->speed_right = 0;
			break;
		case STARTING:
			this->Turbine->update_speed(34);
			this->state = RUN;
			HAL_Delay(1000);
			break;
		case RUN:
			//Read real error -8 to 8
			this->error_now = this->Sensors->update_error();
			//Out line
			if(this->error_now == OUT_LINE)
			{
				//Outline
				switch (this->Sensors->get_out_state())
				{
					case CENTER:
						this->speed_left = this->MED_SPEED;
						this->speed_right = this->MED_SPEED;
						break;
					case RIGHT:
					   	this->speed_left = this->MAX_SPEED;
						this->speed_right = (0 - this->MAX_SPEED);
						break;
					case LEFT:
					   	this->speed_left = (0 - this->MAX_SPEED);
					   	this->speed_right = this->MAX_SPEED;
					   	break;
				}
			}
			else
			{
				//Online state machine//
				this->PID->compute(this->error_now);
				this->speed_left = this->MED_SPEED + this->PID->correction();
				this->speed_right = this->MED_SPEED - this->PID->correction();
			}
			break;
		}
		this->Motor_left->update_speed(this->speed_left, true);
		this->Motor_right->update_speed(this->speed_right, false);
}

//-------------------------------------------------------------------------------------
Sensors_qtr::Sensors_qtr(ADC_HandleTypeDef *hadc, GPIO_TypeDef *GPIO, uint16_t S0, uint16_t S1, uint16_t S2, uint16_t S3)
{
	this->background_color = white;
	this->out_state = CENTER;
	for(int i=0;i<16;i++)
		this->sensor_qtr[i] = 0;
	this->hadc = hadc;
	this->GPIO = GPIO;
	this->S[0] = S0;
	this->S[1] = S1;
	this->S[2] = S2;
	this->S[3] = S3;
	return;
}
int Sensors_qtr::read_adc(void)
{
	int sensor_value = 4095;
	HAL_ADC_Start(this->hadc);
	if(HAL_ADC_PollForConversion(this->hadc,5) == HAL_OK)
		sensor_value  = (int)(HAL_ADC_GetValue(this->hadc));
	HAL_ADC_Stop(this->hadc);
	return sensor_value;
}

void Sensors_qtr::read_sensors_qtr()
{
	for(uint8_t i = 0;i<16;i++)
	{
		HAL_GPIO_WritePin(this->GPIO, S[0], (GPIO_PinState)(i & 0x01));
		HAL_GPIO_WritePin(this->GPIO, S[1], (GPIO_PinState)(i & 0x02));
		HAL_GPIO_WritePin(this->GPIO, S[2], (GPIO_PinState)(i & 0x04));
		HAL_GPIO_WritePin(this->GPIO, S[3], (GPIO_PinState)(i & 0x08));
		this->sensor_qtr[i] = this->read_adc();
	}
	return;
}
float Sensors_qtr::update_error()
{
	int max;
	int min;
	int threshold;
	int range;
	int bit_sensor[16];
	int sum = 0;
	int weigth[8] = {8,7,6,5,4,3,2,1};
	int errorLeft = 0;
	int errorRight = 0;
	float error_now = 0.0;
	//Read samples from each sensor
	this->read_sensors_qtr();
	max = min = this->sensor_qtr[0];
	for(int i=1;i<16;i++)
	{
		if(this->sensor_qtr[i]> max)
			max = this->sensor_qtr[i];
		if(this->sensor_qtr[i] < min)
			min = this->sensor_qtr[i];
	}
	range = max-min;
	if(range > 400)
	{
		threshold = (range/2)+min;
		for(int i=0;i<16;i++)
		{
			if(this->background_color)
				bit_sensor[i] = (this->sensor_qtr[i] < threshold) ? 1 : 0;
			else
				bit_sensor[i] = (this->sensor_qtr[i] > threshold) ? 1 : 0;
		}
		for(int i=0;i<8;i++)
		{
			errorLeft += bit_sensor[i]*weigth[i];
			errorRight += bit_sensor[15-i]*weigth[i];
		}
		for(int i=0;i<16;i++)
			sum += bit_sensor[i];
		error_now = (float)(errorRight-errorLeft)/(float)(sum);
		this->out_state = ((error_now <= 2.5)&&(error_now >=(0-2.5)))  ? CENTER : this->out_state;
		this->out_state = ((error_now > 2.5)&&(error_now <=8.0))        ?  RIGHT: this->out_state;
        this->out_state = ((error_now <(0-2.5))&&(error_now >=(0-8.0))) ? LEFT: this->out_state;
        return error_now;
	}
	else
		return  OUT_LINE;
	return error_now;
}
//--------------------------------------------------------------------------------------------------------------------------------------
Motor_Driver::Motor_Driver(int MAX_SPEED,GPIO_TypeDef *GPIO,uint16_t DIS,uint16_t DIR,TIM_HandleTypeDef *htim,uint32_t TIM_CHANNEL)
{
	this->MAX_SPEED = MAX_SPEED;
	this->GPIO = GPIO;
	this->DIS = DIS;
	this->DIR = DIR;
	this->htim = htim;
	this->TIM_CHANNEL = TIM_CHANNEL;
}
void Motor_Driver::init(void)
{
	HAL_TIM_PWM_Start(this->htim, this->TIM_CHANNEL);
	HAL_GPIO_WritePin(this->GPIO, this->DIS, GPIO_PIN_RESET);
}

void Motor_Driver::update_speed(int speed,bool clockwise)
{
	this->speed = speed;
	if(!this->speed)
	{
		HAL_GPIO_WritePin(this->GPIO,this->DIR,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(this->htim,this->TIM_CHANNEL,0);
	}
	else
	{
		if (this->speed >=1)
		{
			this->speed = (this->speed >= this->MAX_SPEED) ? this->MAX_SPEED : this->speed;
			if(clockwise)
				HAL_GPIO_WritePin(this->GPIO,this->DIR,GPIO_PIN_RESET);
			else
				HAL_GPIO_WritePin(this->GPIO,this->DIR,GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(this->htim,this->TIM_CHANNEL,this->speed);
		}
		else
		{
			this->speed *= (0-1);
			this->speed = (this->speed >= this->MAX_SPEED) ? this->MAX_SPEED : this->speed;
			if(clockwise)
				HAL_GPIO_WritePin(this->GPIO,this->DIR,GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(this->GPIO,this->DIR,GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(this->htim,this->TIM_CHANNEL,this->speed);
		}
	}
	return;
}
//--------------------------------------------------------------------------------------------------------------------------------------
Control_PID::Control_PID(float kp,float ki,float kd)
{
	this->kp = kp;
	this->ki = ki;
	this->kd = kp;
	this->error_now = 0.0;
	for(int i = 0; i<6;i++)
		this->error_last[i] = 0.0;
	this->sum_errors = 0.0;
	this->u = 0;
}
void Control_PID::set_tunings(float kp,float ki,float Kd)
{
	this->kp = kp;
	this->ki = ki;
	this->kd = kp;
}

void Control_PID::compute(float error_now)
{
	this->error_now = error_now;
	this->sum_errors = 0.0;
	for(int i =0;i<6;i++)
		this->sum_errors += this->error_last[i];
	this->u = (int)(this->kp * this->error_now + this->kd * (this->error_now - this->error_last[0])+ this->ki*this->sum_errors);
	this->error_last[5] = this->error_last[4];
	this->error_last[4] = this->error_last[3];
	this->error_last[3] = this->error_last[2];
	this->error_last[2] = this->error_last[1];
	this->error_last[1] = this->error_last[0];
	this->error_last[0] = this->error_now;
}
//--------------------------------------------------------------------------------------------------------------------------------------
ESC_Turbine::ESC_Turbine(TIM_HandleTypeDef *htim,uint32_t TIM_CHANNEL)
{
	this->htim = htim;
	this->TIM_CHANNEL = TIM_CHANNEL;
	this->speed = 0;
}
void ESC_Turbine::init(void)
{
	this->speed = 12;
	HAL_TIM_PWM_Start(this->htim,this->TIM_CHANNEL);
}
void ESC_Turbine::update_speed(int speed)
{
	this->speed = speed;
	if(this->speed < 0)
		this->speed = 0;
	else if(this->speed > 100)
		this->speed = 100;
	this->speed += 100;
	__HAL_TIM_SET_COMPARE(this->htim,this->TIM_CHANNEL,this->speed);
	return;
}
void ESC_Turbine::calibrate(void)
{
	  this->update_speed(12);
	  HAL_Delay(8000);
}

//-----------------------------------------------------------------------------------
IR_NEC::IR_NEC(TIM_HandleTypeDef *htim,uint32_t TIM_CHANNEL)
{
	this->input_capture = 0;
	this->input_diference = 0;
	this->input_last = 0;
	this->sample = 0;
	this->htim = htim;
	this->TIM_CHANNEL = TIM_CHANNEL;
}
void IR_NEC::init(void)
{
	HAL_TIM_IC_Start_IT(this->htim,this->TIM_CHANNEL);
	this->state = START;
}
void IR_NEC::decode(void)
{
	if(this->state == START)
	{
		//Restart counter
		__HAL_TIM_SetCounter(this->htim,0);
		//Clear variables
		this->input_capture = 0;
		this->input_last = 0;
		this->input_diference = 0;
		this->sample = 0;
		//start protocol
		this->state = DATA;
  }
  else if(this->state == DATA)
  {
	  //Read Timer
	  this->input_capture = __HAL_TIM_GetCompare(this->htim,this->TIM_CHANNEL);    //read TIM2 channel 1 capture value
	  //get difference between first pulse to second pulse
	  this->input_diference =  this->input_capture - this->input_last;
	  //if the logic bit 1 occurs
	  if(this->input_diference > 215 && this->input_diference < 235)//225
	  {
		 this->data |= (1UL << (31 - this->sample));   // write 1
		 this->sample++;//increase sample
	  }
	  //if the logic bit 0 occurs
	  else if(this->input_diference > 102 && this->input_diference < 122)//112
	  {
		  this->data &= ~(1UL << (31 - this->sample));//write 0
		  this->sample++;//increase sample
	  }
	  if(this->sample==31)//if sample is 31 data is full
		  this->state = DATA_FULL;
	  this->input_last = this->input_capture;
	  }
	return;
}

void IR_NEC::restart(void)
{
	HAL_Delay(185);
	this->data = 0;
	this->state = START;
	return;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
		Ir_Nec.decode();
   return;
}
//-------------------------------------------------------------------
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
