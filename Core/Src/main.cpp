
/** Main loop of the quadcopter, using RTOS*******************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu9250.hpp"
//#include "sbus.h"
#include "MadgwickAHRS.h"
#include "print_func.hpp"
#include "IMU_EKF.hpp"
#include "mpu_data_type.hpp"
#include "bmp180.h"
#include "flash.h"
#include "PID.h"
#include "MotorController.hpp"


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


osThreadId read_MPUHandle;
osThreadId read_BMPHandle;
osThreadId read_MagnetHandle;
osThreadId receiveTaskHandle;
osThreadId filterTaskHandle;
osMessageQId QueueHandle;
/* USER CODE BEGIN PV */
uint8_t Rx_data[2];
uint8_t Rx_buffer[24];
uint8_t sbus_buffer[7];

bool reading = false;

int start, end, t_sample, i;
int j;

double alt;

IMU_data *ptr;
osEvent retvalue;

IMU_calib_data imu_bias;

IMU_data data_imu_temp;
IMU_data data_imu_raw;
IMU_data data_imu_buffer;
IMU_data data_imu_com;
MAG_data mag_data;

EULER_angle drone_state;
EULER_angle magd_state;
EULER_angle com_state;

ESC_value esc_value;
ESC_value esc_stop;
ESC_value esc;

LPF gyro_lpf;
LPF gyro_lpf1;
LPF gyro_lpf2;

LPF acc_lpf;
LPF acc_lpf1;
LPF acc_lpf2;

PID_raw pid_temp;
PID_value pid;

PID pid_roll;
PID pid_pitch;
PID pid_yaw;

MotorController controller;

uint16_t thrust_control;

uint8_t QueueBuffer[ 32 * sizeof( IMU_data * ) ];
osStaticMessageQDef_t QueueControlBlock;

float pid_r_value,pid_p_value,pid_y_value;
//IMU_EKF ekf;

double x[7] = {1,0,0,0,0,0,0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void readMagnet(void const * argument);
void filterFunction(void const * argument);

/* USER CODE BEGIN PFP */
void init_nRF24(){
	 uint8_t ADDR[] = { 'n', 'R', 'F', '2', '4' }; // the address for RX pipe
	  nRF24_SetRFChannel(90); // set RF channel to 2490MHz
	  nRF24_SetDataRate(nRF24_DR_2Mbps); // 2Mbit/s data rate
	  nRF24_SetCRCScheme(nRF24_CRC_1byte); // 1-byte CRC scheme
	  nRF24_SetAddrWidth(5); // address width is 5 bytes
	  nRF24_SetAddr(nRF24_PIPE1, ADDR); // program pipe address
	  nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_ON, 10); // enable RX pipe#1 with Auto-ACK: enabled, payload length: 10 bytes
	  nRF24_SetTXPower(nRF24_TXPWR_0dBm); // configure TX power for Auto-ACK, good choice - same power level as on transmitter
	  nRF24_SetOperationalMode(nRF24_MODE_RX); // switch transceiver to the RX mode
	  nRF24_SetPowerMode(nRF24_PWR_UP); // wake-up transceiver (in case if it sleeping)
	  // then pull CE pin to HIGH, and the nRF24 will start a receive...
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
osMailQId(myMailQID); //Define mail queue

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
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart3, (uint8_t *) Rx_data, 1);
  HAL_UART_Receive_IT(&huart1, (uint8_t *) Rx_data, 1);

  check_AHRS(true);

  esc_stop.esc_value1 = 1000;
  esc_stop.esc_value2 = 1000;
  esc_stop.esc_value3 = 1000;
  esc_stop.esc_value4 = 1000;

  thrust_control = 1000;

  init_MPU();
  init_ESC();
  init_nRF24();
//  initBMP();

  pid = readFlash();

  print_pid_value(pid);

  controller.loadPID(pid);


//  pid_roll.load(pid.Kp1/100,pid.Ki1/100,pid.Kd1/100,0.01);
//  pid_pitch.load(pid.Kp1/100,pid.Ki1/100,pid.Kd1/100,0.01);
//  pid_yaw.load(pid.Kp1/100,pid.Ki1/100,pid.Kd1/100,0.01);
//  MX_TIM7_Init();
  BOARD_MODE = FLY_MODE;
  ARMED = UNARMED;
//  data_lpf.load(LPF_10HZ);

  gyro_lpf.load(LPF_50HZ);
  gyro_lpf1.load(LPF_50HZ);
  gyro_lpf2.load(LPF_50HZ);

  acc_lpf.load(LPF_10HZ);
  acc_lpf1.load(LPF_10HZ);
  acc_lpf2.load(LPF_10HZ);

  imu_bias = readIMUcalib();
//  writeCalibration(imu_bias);

  x[4] = bGx*DEC2RAD;
  x[5] = bGy*DEC2RAD;
  x[6] = bGz*DEC2RAD;

//  ekf.loadEKF(x,P,Q,R_full);

//  initBMP();

//  calibration_IMU();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of Queue */
  osMessageQStaticDef(Queue, 32, IMU_data *, QueueBuffer, &QueueControlBlock);
  QueueHandle = osMessageCreate(osMessageQ(Queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */


  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of read_MPU */
  osThreadDef(read_MPU, StartDefaultTask, osPriorityNormal, 0, 128);
  read_MPUHandle = osThreadCreate(osThread(read_MPU), NULL);

  /* definition and creation of read_BMP */
  osThreadDef(read_BMP, StartTask02, osPriorityNormal, 0, 128);
  read_BMPHandle = osThreadCreate(osThread(read_BMP), NULL);

  /* definition and creation of read_magnet */
//  osThreadDef(read_Magnet, readMagnet, osPriorityNormal, 0, 128);
//  read_MagnetHandle = osThreadCreate(osThread(read_Magnet), NULL);

  /* definition and creation of receiveTask */
  osThreadDef(receiveTask, StartTask03, osPriorityHigh, 0, 512);
  receiveTaskHandle = osThreadCreate(osThread(receiveTask), NULL);

  /* definition and creation of filterTask */
  osThreadDef(filterTask, filterFunction, osPriorityAboveNormal , 0, 10000);
  filterTaskHandle = osThreadCreate(osThread(filterTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {

  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {

  }
}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

//    data = osMailAlloc(QueueHandle, osWaitForever); /* Allocate memory */
	  data_imu_raw = process_MPU(true,false);
	  osMessagePut(QueueHandle , (uint32_t) (&data_imu_raw), 200);
	  adding_raw();
	  i++;
	//	     osDelay(2);
	  osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Task02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
//	  advance_print(data_imu_buffer);

//	  print_euler_compare(com_state,magd_state,drone_state);
//	  print_magnet(mag_data);
//	  print_raw(data_imu_buffer);
//	  print_raw_mag(data_imu_buffer, mag_data);
//	  print_euler(drone_state);
	  print_euler_compare(com_state,magd_state,drone_state);

	  osDelay(45);

  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the receiveTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
	  nRF24_CE_H; // start receiving
	  if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
	 //	    	  IRQ = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4);
	 //	          // the RX FIFO have some data, take a note what nRF24 can hold up to three payloads of 32 bytes...
	 	          pipe = nRF24_ReadPayload(nRF24_payload, &payload_length); // read a payload to buffer
	 	          nRF24_ClearIRQFlags(); // clear any pending IRQ bits

	 	      }
	  osDelay(100);
  }
  /* USER CODE END StartTask03 */
}

void readMagnet(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
		mag_data = process_magnet();
	//	alt =  get_alt_bmp();

	    osDelay(100);
  }
  /* USER CODE END StartTask03 */
}
/* USER CODE BEGIN Header_filterFunction */
/**
* @brief Function implementing the filterTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_filterFunction */
void filterFunction(void const * argument)
{
  /* USER CODE BEGIN filterFunction */
  /* Infinite loop */
  for(;;)
  {

		data_imu_temp.Gyro_x  = 0;
		data_imu_temp.Gyro_y  = 0;
		data_imu_temp.Gyro_z  = 0;

		data_imu_temp.Acc_x  = 0;
		data_imu_temp.Acc_y  = 0;
		data_imu_temp.Acc_z  = 0;
	  if(i!=0){
		  for(int count = 0; count < 10; count++){
		  	retvalue = osMessageGet(QueueHandle, 10);
		  	ptr = (IMU_data*) retvalue.value.p;
		  	data_imu_temp.Acc_x += ptr->Acc_x;
		  	data_imu_temp.Acc_y += ptr->Acc_y;
		  	data_imu_temp.Acc_z += ptr->Acc_z;

		  	data_imu_temp.Gyro_x += ptr->Gyro_x;
		  	data_imu_temp.Gyro_y += ptr->Gyro_y;
		  	data_imu_temp.Gyro_z += ptr->Gyro_z;
		  }
		  gyro_lpf.update(data_imu_temp.Gyro_x /i);
		  gyro_lpf1.update(data_imu_temp.Gyro_y /i);
		  gyro_lpf2.update(data_imu_temp.Gyro_z /i);

		  acc_lpf.update(data_imu_temp.Acc_x/i);
		  acc_lpf1.update(data_imu_temp.Acc_y/i);

		  acc_lpf2.update(data_imu_temp.Acc_z/i);

	  data_imu_buffer.Acc_x = acc_lpf.get();
	  data_imu_buffer.Acc_y = acc_lpf1.get();
	  data_imu_buffer.Acc_z = acc_lpf2.get();

	  data_imu_buffer.Gyro_x = gyro_lpf.get();
	  data_imu_buffer.Gyro_y = gyro_lpf1.get();
	  data_imu_buffer.Gyro_z = gyro_lpf2.get();

	  data_imu_com.Gyro_x = (gyro_lpf.get() - bGx);
	  data_imu_com.Gyro_y = (gyro_lpf1.get() - bGy);
	  data_imu_com.Gyro_z = (gyro_lpf2.get()  - bGz);


//	  data_imu_buffer.Acc_x =  acc_lpf.update(Acc_x_/i);
//	  data_imu_buffer.Acc_y =  acc_lpf1.update(Acc_y_/i);
//	  data_imu_buffer.Acc_z =  acc_lpf2.update(Acc_z_/i);


	  data_imu_com.Acc_x =  acc_lpf.get() - bAx;
	  data_imu_com.Acc_y =  acc_lpf1.get()- bAy;
	  data_imu_com.Acc_z =  acc_lpf2.get()- bAz;

////
//	  ekf.updateEKF(data_imu_buffer, 0.01);
//	  ekf.updateEKF(data_imu_buffer, mag_data, 0.01);

//	  data_imu_buffer.Acc_x =  gyro_angle_r;
//	  data_imu_buffer.Acc_y =  gyro_angle_p;
//	  data_imu_buffer.Acc_z =  gyro_angle_y;
//	  drone_state = ekf.getAngle();
//	  print_raw(data_imu_buffer);
	  }
	  magd_state = MadgwickAHRSupdateIMU(data_imu_com,0.01);
	  com_state = complementary_filter(data_imu_com, 0.01, 0.98);

//	  float mag_x = mag_data.Mag_x*cos(com_state.pitch) + mag_data.Mag_y*sin(com_state.roll)*sin(com_state.pitch) + mag_data.Mag_z*cos(com_state.roll)*sin(com_state.pitch);
//	  float mag_y = mag_data.Mag_y*cos(com_state.roll) - mag_data.Mag_z* sin(com_state.roll);
	  com_state.yaw = 180 * atan2(-mag_data.Mag_x,mag_data.Mag_y)/M_PI;

	  esc = controller.update(com_state,thrust_control);
      if((ARMED == ARMED_)  && (BOARD_MODE == FLY_MODE)){
	      set_ESC(esc);
      }
	  magd_state.yaw = esc.esc_value1;
      drone_state.roll = esc.esc_value2;
      drone_state.pitch = esc.esc_value3;
      drone_state.yaw = esc.esc_value4;
//	  pid_p_value = pid_pitch.round(pid_pitch.Out);
//	  pid_r_value = pid_pitch.round(pid_roll.Out);
////
//	  magd_state.pitch = pid_p_value;
//	  magd_state.roll = pid_r_value;
//	  com_state = cross2plus(com_state);
	  i = 0;

	  delete_raw();
      osDelay(10);
  }
  /* USER CODE END filterFunction */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM7) {
	  ulHighFrequencyTimerTicks ++;
  }
  /* USER CODE END Callback 1 */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);

	if (huart->Instance == USART1)  //current UART
	{

//		char trans_b[4];
		if(Rx_data[0] == 's'){
			j = 0;
//			BOARD_MODE = FLY_MODE;

             for(int count = 0; count< 24; count++)
            	 Rx_buffer[count] = 0x00;
		}
		else{
			if((Rx_data[0] != 'e')){
			 Rx_buffer[j] = Rx_data[0];
			 j++;
//			 HAL_UART_Receive_IT(&huart3, (uint8_t *) Rx_data, 1);

			}
			else{

				if(Rx_buffer[21] == 0x00){
					for(uint8_t k = 0; k<8; k++){
                        sbus_buffer[k] = Rx_buffer[k];
					}
				esc_value = sbus_decode(sbus_buffer);
				if(check_CRC(esc_value)){
					if(CRC_thurst(esc_value)){
						thrust_control = esc_value.esc_value1;
					}
					else{
						if(ARMED && (BOARD_MODE == TEST_MODE))
				        set_ESC(esc_value);
					}
				    j = 0;
//				   	HAL_UART_Transmit(&huart1,(uint8_t*) ack, strlen(ack),100);
				}
				else{
					 j = 0;
				}
				}
				else{
					 pid_temp = pid_decode(Rx_buffer);
					 if(check_CRC_pid(pid_temp)){
						 writeFlash(pid_temp.data);
						 pid = readFlash();
						 controller.loadPID(pid);
//						 pid_roll.load(pid.Kp1/100,pid.Ki1/100,pid.Kd1/100,0.01);
//						 pid_pitch.load(pid.Kp1/100,pid.Ki1/100,pid.Kd1/100,0.01);
						 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
					 }
				}
                Rx_data[0] = 0;

//				HAL_UART_Receive_IT(&huart3, (uint8_t *) Rx_data, 1);
			}
		}

		if((Rx_data[0] == 'd')){
			set_ESC(esc_stop);
			ARMED = UNARMED;
			BOARD_MODE = FLY_MODE;
		}
		if(Rx_data[0] == 'a'){
			ARMED = ARMED_;
			BOARD_MODE = FLY_MODE;
		}
//
		if(Rx_data[0] == 't'){
			BOARD_MODE = TEST_MODE;
		}
	}


//
	if (huart->Instance == USART3)  //current UART
	{

//		char trans_b[4];
		if(Rx_data[0] == 's'){
			j = 0;
//			BOARD_MODE = FLY_MODE;
//			reading = true;
             for(int count = 0; count< 24; count++)
            	 Rx_buffer[count] = 0x00;
		}
		else{
			if(Rx_data[0] != 'e'){
			 Rx_buffer[j] = Rx_data[0];
			 j++;
//			 HAL_UART_Receive_IT(&huart3, (uint8_t *) Rx_data, 1);

			}
			else{
				if(Rx_buffer[8] == 0x00){
					for(uint8_t k = 0; k<8; k++){
                        sbus_buffer[k] = Rx_buffer[k];
					}
				esc_value = sbus_decode(sbus_buffer);
				if(check_CRC(esc_value)){
				    set_ESC(esc_value);
				    j = 0;
//				   	HAL_UART_Transmit(&huart1,(uint8_t*) ack, strlen(ack),100);
				}
				else{
					 j = 0;
				}
				}
				else{
					 pid_temp = pid_decode(Rx_buffer);
					 if(check_CRC_pid(pid_temp)){
						 writeFlash(pid_temp.data);
						 pid = readFlash();
					 }
				}
                Rx_data[0] = 0;

//				HAL_UART_Receive_IT(&huart3, (uint8_t *) Rx_data, 1);
			}
		}

		if(Rx_data[0] == 'd'){
			set_ESC(esc_stop);
		}
	}
	 HAL_UART_Receive_IT(&huart1, (uint8_t *) Rx_data, 1);
	 HAL_UART_Receive_IT(&huart3, (uint8_t *) Rx_data, 1);
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */


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
