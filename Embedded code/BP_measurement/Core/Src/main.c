/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define _PUTCHAR_PROTOTYPE_ int __io_putchar(int ch)

//Measuremenet
#define NUM_SECTIONS 2
#define NUM_COEFFS 5
#define SHIFT 60
#define INDEX_3MM (155)
#define INDEX_7MM (363)
#define INDEX_3MM_original 7
#define INDEX_7MM_original 18
#define DIST_2MM 102

#define LENGTH 50
#define UPSAMPLE_RATE 20
#define NUM_REC 4
#define NEW_SIZE (UPSAMPLE_RATE*LENGTH)
#define INTEREST_SIZE (INDEX_7MM-INDEX_3MM)
#define WINDOW_SIZE 25
#define MAX_PEAK_NUM 8
#define SAVED_WALLS 4
#define SAVED_FOR_EVAL 40
#define INFINITY 32767

#define RX_BUFFER_SIZE_signal 256
#define RX_BUFFER_SIZE_calibration 10

#define RECORDING_LENGTH 1500

#define NUM_STATES (2 * NUM_SECTIONS) // Size of the state array

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

_PUTCHAR_PROTOTYPE_ {
	while (HAL_OK != HAL_UART_Transmit(&hlpuart1, (uint8_t *) &ch, 1, 0xFFFF)){}
	return ch;
}

// Variables for the logic and the communication
volatile uint8_t USART_RxBuffer_signal[256];
volatile uint8_t USART_RxBuffer_calibration[10];
//volatile uint8_t data = 0; // used for testing
volatile uint8_t started = 0;
volatile int16_t input[LENGTH] = {0};
volatile uint8_t start_algo = 0;
volatile float32_t SBP = 0;
volatile float32_t DBP = 0;
volatile uint16_t rec_index = 0;
volatile uint8_t walls_found = 0;
volatile uint8_t calibrated = 0;
float32_t k = 0;
volatile uint8_t results_idx = 0;
volatile uint8_t eval_idx = 0;
volatile uint8_t valid = 1;
volatile float32_t res = 0;

//Butterworth filter coefficients
const float32_t coefficients[NUM_SECTIONS * NUM_COEFFS] = {
    // Section 1 coefficients
    3.12389769e-05, 6.24779538e-05, 3.12389769e-05, +1.72593340e+00, -7.47447372e-01,
    // Section 2 coefficients
    1.00000000e+00, 2.00000000e+00, 1.00000000e+00, +1.86380049e+00, -8.87033000e-01
};

q15_t US_interpolated[NUM_REC][INTEREST_SIZE];
q15_t US_filtered[NUM_REC][INTEREST_SIZE];
q15_t interest[NUM_REC][INTEREST_SIZE];

q15_t covariance[NUM_REC-1][INTEREST_SIZE] = {0};

q15_t peaks[NUM_REC-1][MAX_PEAK_NUM] = {{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1}};
uint16_t max_arguments[NUM_REC-1][MAX_PEAK_NUM] = {{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1}};
uint16_t max_arguments_for_0[NUM_REC-1][MAX_PEAK_NUM] = {{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1}};
q15_t shifts[NUM_REC-1][MAX_PEAK_NUM] = {{INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY},{INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY},{INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY}};
q15_t correct_pair_corr[NUM_REC-1][MAX_PEAK_NUM] = {{INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY},{INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY},{INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY}};
uint16_t W_p_selector[NUM_REC-1] = {-1,-1,-1};
uint16_t W_d_selector[NUM_REC-1] = {-1,-1,-1};
uint16_t W1 = 0;
uint16_t W2 = 0;
uint16_t Proximal_Wall[SAVED_FOR_EVAL];
uint16_t Distal_Wall[SAVED_FOR_EVAL];
uint16_t results[SAVED_FOR_EVAL] = {0};
uint16_t results_buffer[SAVED_FOR_EVAL] = {0};
float32_t min_dia = 0xFFF;
float32_t max_dia = 0;
volatile uint8_t buff = 0;

q15_t cal_min = INFINITY;
q15_t cal_max = 0;
float32_t log_temp, log_value;
float32_t res_temp1, res_temp2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM16_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &hlpuart1)
    {
        // Start command has arrived
        if (USART_RxBuffer_calibration[0] == 's')
        {
            start_algo = 0;
            rec_index = 0;
            walls_found = 0;
            calibrated = 0;
            valid = 1;
            buff = 0;
            eval_idx = 0;

            // Tokenize string using strtok
            uint8_t *token;
            token = strtok((char *)USART_RxBuffer_calibration, ",");
            // Read calibration values of SBP and DBP
            token = strtok(NULL, ",");
            SBP = (float32_t)atoi((char *)token);
            token = strtok(NULL, ",");
            DBP = (float32_t)atoi((char *)token);

            // Clear the buffer and prepare to receive the next message
            memset(USART_RxBuffer_calibration, 0, sizeof(USART_RxBuffer_calibration));
            HAL_UART_Receive_IT(&hlpuart1, (uint8_t *)USART_RxBuffer_signal, RX_BUFFER_SIZE_signal);

            __HAL_TIM_SET_COUNTER(&htim16, 0);
            started = 1;
        }
        else if (started)
        {
            // Receiving and handling end message
            if (USART_RxBuffer_signal[0] == 'e') // end
            {
                started = 0;
                HAL_UART_Receive_IT(&hlpuart1, (uint8_t *)USART_RxBuffer_calibration, RX_BUFFER_SIZE_calibration);
                return;
            }

            // Receiving and handling data input
            uint8_t *token;
            uint16_t index = 0;
            token = strtok((char *)USART_RxBuffer_signal, ",");
            while (index < LENGTH)
            {
                // Convert token to integer
                input[index] = atoi((char *)token);
                index++;
                token = strtok(NULL, ",");
            }
            start_algo = 1;

            rec_index++;

            // Handling the end of the recording
            if (rec_index >= RECORDING_LENGTH)
            {
                started = 0;
                HAL_UART_Receive_IT(&hlpuart1, (uint8_t *)USART_RxBuffer_calibration, RX_BUFFER_SIZE_calibration);
            }
            else
                HAL_UART_Receive_IT(&hlpuart1, (uint8_t *)USART_RxBuffer_signal, RX_BUFFER_SIZE_signal);
        }
    }
}

// Interpolating the recorded signal and keeping the relevant parts
void zero_stuffing(const q15_t* inputArray, q15_t* outputArray, size_t upsamplingRate)
{
    // Initialize with CMSIS-DSP to zero
    arm_fill_q15(0, outputArray, INTEREST_SIZE);

    // Fill in the input samples
    for (int i = INDEX_3MM_original; i < INDEX_7MM_original; i++)
    {
        outputArray[(i - INDEX_3MM_original) * upsamplingRate] = inputArray[i];
    }
}

// Zero-phase backward-forward filtering
void sosfiltfilt(q15_t* input, q15_t* output, int x_len)
{
    float32_t input_f32[INTEREST_SIZE] = {0};
    float32_t output_1_f32[INTEREST_SIZE] = {0};
    float32_t output_2_f32[INTEREST_SIZE] = {0};
    float32_t state_f32[5 * NUM_SECTIONS] = {0};

    for (int i = 0; i < x_len; i++)
        input_f32[i] = (float32_t)input[i];

    // Initialize filter instance
    arm_biquad_casd_df1_inst_f32 S;
    arm_biquad_cascade_df1_init_f32(&S, NUM_SECTIONS, coefficients, state_f32);

    // Process the input signal
    arm_biquad_cascade_df1_f32(&S, input_f32, output_1_f32, INTEREST_SIZE);

    // Reverse the filtered data (filtfilt operation)
    for (int i = 0; i < x_len / 2; i++)
    {
        float32_t temp = output_1_f32[i];
        output_1_f32[i] = output_1_f32[x_len - i - 1];
        output_1_f32[x_len - i - 1] = temp;
    }

    // Process the input signal
    arm_biquad_cascade_df1_init_f32(&S, NUM_SECTIONS, coefficients, state_f32);
    arm_biquad_cascade_df1_f32(&S, output_1_f32, output_2_f32, INTEREST_SIZE);

    // Upscale for better granularity after converting to integer format
    arm_scale_f32(output_2_f32, (float32_t)256, output_2_f32, INTEREST_SIZE);

    // Reverse the filtered data again to get the final output
    for (int i = 0; i < x_len; i++)
    {
        output[i] = (q15_t)(output_2_f32[x_len - i - 1]);
    }
}

// Performing sliding window covariance and turning small values to 0
void sliding_window_covariance(q15_t* x, q15_t* y, uint32_t N, const uint32_t window_size, q15_t* cov_result)
{
    uint16_t i;
    q15_t x_window[window_size];
    q15_t y_window[window_size];
    q63_t temp_result;
    q15_t max_val;
    uint32_t max_index;

    // Iterate over each index in the range [window_size/2, N-window_size/2]
    for (i = window_size / 2; i < N - window_size / 2; i++)
    {
        // Get the current window of x and y centered at index i
        arm_copy_q15(&x[i - window_size / 2], x_window, window_size);
        arm_copy_q15(&y[i - window_size / 2], y_window, window_size);

        temp_result = 0;
        arm_dot_prod_q15(x_window, y_window, window_size, &temp_result);
        temp_result = temp_result >> 16;
        cov_result[i] = ((q15_t)__SSAT(temp_result, 16));
    }

    arm_max_q15(cov_result, N, &max_val, &max_index);

    // Apply thresholding
    for (i = 0; i < N; i++)
    {
        if (cov_result[i] < max_val / 20) // 5%
        {
            cov_result[i] = 0;
        }
    }
}

// Peak finding function
uint8_t find_peaks(q15_t arr[], uint16_t n, q15_t* peaks)
{
    // Handle edge cases for the first and last elements
    int num = 0;
    if (n > 1 && arr[0] > arr[1])
    {
        peaks[num] = 0;
        num++;
    }
    for (int i = 1; i < n - 1; i++)
    {
        if (arr[i] > arr[i - 1] && arr[i] >= arr[i + 1] && num < MAX_PEAK_NUM)
        {
            peaks[num] = i;
            num++;
        }
    }
    if (n > 1 && arr[n - 1] > arr[n - 2] && num < MAX_PEAK_NUM)
    {
        peaks[num] = n - 1;
        num++;
    }
    return num;
}

// Finding the index with the maximum value within a predefined window
void find_argmax_in_window(q15_t* arr, int n, int peak, int window, uint16_t* argmax)
{
    int start = peak - window;
    int end = peak + window;
    if (start < 0)
        start = 0;
    if (end >= n)
        end = n - 1;

    uint32_t arg;

    // Find the index of the maximum value within the window
    arm_max_q15(&arr[start], end - start + 1, NULL, &arg);

    // Adjust the index to account for the window offset
    *argmax = start + arg;
}

// Compare function for qsort
int compare(const void* a, const void* b)
{
    uint16_t x = *(uint16_t*)a;
    uint16_t y = *(uint16_t*)b;
    if (x < y)
        return -1;
    if (x > y)
        return 1;
    return 0;
}

// Median calculating function
uint16_t findMedian(uint16_t arr[], int n)
{
    // Sort the array using qsort
    qsort(arr, n, sizeof(uint16_t), compare);

    // Find the median
    if (n % 2 == 0)
    {
        return (arr[n / 2 - 1] + arr[n / 2]) / 2.0;
    }
    else
    {
        return arr[n / 2];
    }
}

// Compute correlation (returns 1 if negative, 0 if non-negative)
uint8_t compute_correlation(const q15_t* arr1, const q15_t* arr2, uint32_t n)
{
    q63_t dot_product = 0;
    q15_t mean1, mean2;
    q15_t centered_signal1[n];
    q15_t centered_signal2[n];

    // Compute means
    arm_mean_q15(arr1, n, &mean1);
    arm_mean_q15(arr2, n, &mean2);

    // Center the signals by subtracting the means
    arm_offset_q15(arr1, -mean1, centered_signal1, n);
    arm_offset_q15(arr2, -mean2, centered_signal2, n);

    // Compute dot product of the centered signals
    arm_dot_prod_q15(centered_signal1, centered_signal2, n, &dot_product);

    // Check if dot product is negative
    return (uint8_t)(dot_product < 0);
}

// Preprocessing of the recordings
void raw2proper(const q15_t* input, q15_t* output)
{
    q15_t tmp_output[INTEREST_SIZE] = {0};

    // Interpolating
    zero_stuffing(input, tmp_output, UPSAMPLE_RATE);

    // Zero-phase filtering
    sosfiltfilt(tmp_output, output, INTEREST_SIZE);
}

// Finding the walls, which are two peaks moving in opposite directions
uint8_t wall_finding()
{
    // Initialize variables for independent wall calculation
    for (int i = 0; i < NUM_REC - 1; i++)
    {
        for (int j = 0; j < MAX_PEAK_NUM; j++)
        {
            peaks[i][j] = -1;
            max_arguments[i][j] = -1;
            max_arguments_for_0[i][j] = -1;
            shifts[i][j] = INFINITY;
            correct_pair_corr[i][j] = INFINITY;
        }
    }

    uint8_t num[NUM_REC - 1] = {0};

    // Calculate covariances and find peaks in them
    for (int j = 0; j < NUM_REC - 1; j++)
    {
        sliding_window_covariance(interest[j], interest[j + 1], INTEREST_SIZE, WINDOW_SIZE, covariance[j]);
        num[j] = find_peaks(covariance[j], INTEREST_SIZE, peaks[j]);
        if (num[j] < 2 || num[j] > 8)
        {
            return 0;
        }
    }

    // Search for peaks moving in opposite directions with negative correlation
    uint8_t correct = 0;
    for (int j = 0; j < NUM_REC - 1; j++)
    {
        for (int i = 0; i < MAX_PEAK_NUM; i++)
        {
            if (peaks[j][i] == -1)
                break;
            find_argmax_in_window(interest[0], INTEREST_SIZE, peaks[j][i], WINDOW_SIZE, &max_arguments_for_0[j][i]);
            find_argmax_in_window(interest[j + 1], INTEREST_SIZE, peaks[j][i], WINDOW_SIZE, &max_arguments[j][i]);
            shifts[j][i] = max_arguments[j][i] - max_arguments_for_0[j][i];
        }
        for (int i = 0; i < MAX_PEAK_NUM - 1; i++)
        {
            if (shifts[j][i] == INFINITY || shifts[j][i + 1] == INFINITY)
                break;
            if (shifts[j][i] == 0 && shifts[j][i + 1] == 0)
                correct_pair_corr[j][i] = INFINITY;
            else
                correct_pair_corr[j][i] = (q15_t)__SSAT(((q31_t)shifts[j][i]) * ((q31_t)shifts[j][i + 1]), 16);
        }
        uint32_t min_index;
        arm_min_q15(correct_pair_corr[j], MAX_PEAK_NUM, NULL, &min_index);
        if (correct_pair_corr[j][min_index] < 0)
            correct++;
        W_p_selector[j] = peaks[j][min_index];
        W_d_selector[j] = peaks[j][min_index + 1];
    }

    if (correct < (NUM_REC - 1) / 2)
        return 0;

    // Determine the walls
    W1 = findMedian(W_p_selector, NUM_REC - 1);
    W2 = findMedian(W_d_selector, NUM_REC - 1);

    return 1;
}

// Track peaks in subsequent recordings based on known walls
void wall_tracking(q15_t* input, uint16_t* prox, uint16_t* dist)
{
    find_argmax_in_window(input, INTEREST_SIZE, W1, WINDOW_SIZE, prox);
    find_argmax_in_window(input, INTEREST_SIZE, W2, WINDOW_SIZE, dist);
}

// Check if the recording is still valid (two walls moving in opposite directions)
uint8_t eval()
{
    q15_t corr;
    corr = compute_correlation(Proximal_Wall, Distal_Wall, SAVED_FOR_EVAL);
    return corr;
}

// Final calculation of blood pressure
void calculate_results()
{
    // Calculate the temporary result based on the current buffer
    if (buff)
    {
        res_temp1 = k * ((float32_t)results[(eval_idx) % SAVED_FOR_EVAL] - min_dia) / min_dia;
    }
    else
    {
        res_temp1 = k * ((float32_t)results_buffer[(eval_idx) % SAVED_FOR_EVAL] - min_dia) / min_dia;
    }

    // Compute the exponential part of the formula
    arm_vexp_f32(&res_temp1, &res_temp2, 1);  // exp(beta * ((diameter - DD) / DD))

    // Calculate the final result
    res = DBP * res_temp2;

    // Print the result
    printf("%f\r\n", res);
}

// Calculate coefficient for calibration
void calculate_k()
{
    // Find the minimum value in the results array
    arm_min_q15(results, SAVED_FOR_EVAL, &cal_min, NULL);

    // Find the maximum value in the results array
    arm_max_q15(results, SAVED_FOR_EVAL, &cal_max, NULL);

    // Convert the minimum and maximum values to float
    max_dia = (float32_t)cal_max;
    min_dia = (float32_t)cal_min;

    // Calculate the natural logarithm of SBP
    arm_vlog_f32(&SBP, &log_value, 1);  // log(SBP)

    // Calculate the natural logarithm of DBP
    arm_vlog_f32(&DBP, &log_temp, 1);   // log(DBP)

    // Calculate the coefficient k(beta) using the formula
    k = (log_value - log_temp) / ((max_dia - min_dia) / min_dia);
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
  MX_LPUART1_UART_Init();
  MX_USB_HOST_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  __HAL_TIM_ENABLE_IT(&htim16, TIM_IT_UPDATE);
  //printf("Works!\r\n");

  // Reset the timer counter to zero
  //__HAL_TIM_SET_COUNTER(&htim16, 0);

  HAL_TIM_Base_Start_IT(&htim16);

  //HAL_Delay(1000);

  //HAL_TIM_Base_Stop_IT(&htim16);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    //MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

	if (start_algo)
	{
		// Convert raw input to proper format
		raw2proper(input, interest[(rec_index - 1) % NUM_REC]); // This is the preprocessing part


		// If enough records have been received, start finding walls
		if (rec_index >= NUM_REC)
		{
					// Wall finding process
					if (!walls_found)
					{
						uint8_t correct = wall_finding();
						if (correct)
						{
							walls_found = 1;
						}
					}

					// Wall tracking process
					if (walls_found)
					{
						wall_tracking(interest[(rec_index - 1) % NUM_REC], &Proximal_Wall[eval_idx], &Distal_Wall[eval_idx]);

						// Update results based on buffer flag
						if (!buff)
						{
							results[eval_idx] = Distal_Wall[eval_idx] - Proximal_Wall[eval_idx];
						}
						else
						{
							results_buffer[eval_idx] = Distal_Wall[eval_idx] - Proximal_Wall[eval_idx];
						}
						eval_idx++;
					}

					// Calibration process
					if (walls_found && !calibrated && eval_idx == SAVED_FOR_EVAL)
					{
						calculate_k(); // Calculate calibration coefficient
						calibrated = 1;
					}

					// Evaluation process
					if (walls_found && calibrated && eval_idx == SAVED_FOR_EVAL)
					{
						eval_idx = 0;
						buff = !buff; // Toggle buffer

						if (eval() == 0)
						{
							// Reset states if evaluation fails
							walls_found = 0;
							calibrated = 0;
							valid = 0;
						}
						else
						{
							valid = 1;
						}
					}
		}

		// Final results calculation
		if (walls_found && valid && calibrated)
		{
			calculate_results(); // Calculate and print results
		}
		else
		{
			printf("%d\r\n", 0); // Print zero if conditions are not met
		}


		// Reset start_algo flag
		start_algo = 0;
	}
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */
  HAL_UART_Receive_IT(&hlpuart1, (uint8_t *)USART_RxBuffer_calibration, RX_BUFFER_SIZE_calibration);
  /* USER CODE END LPUART1_Init 2 */

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

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 7999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 399;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, USB_PowerSwitchOn_Pin|SMPS_V1_Pin|SMPS_EN_Pin|SMPS_SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OverCurrent_Pin SMPS_PG_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin|SMPS_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_PowerSwitchOn_Pin SMPS_V1_Pin SMPS_EN_Pin SMPS_SW_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin|SMPS_V1_Pin|SMPS_EN_Pin|SMPS_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim16 && started)
  {
	printf("m\r\n");
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
