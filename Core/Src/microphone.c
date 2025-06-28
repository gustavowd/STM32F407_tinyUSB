/*
 * microphone.c
 *
 *  Created on: Jun 21, 2025
 *      Author: gustavo
 */
#include "microphone.h"
#include "main.h"
#include "fatfs.h"

extern I2S_HandleTypeDef hi2s2;

int32_t sampleBuffer[MIC_SAMPLES_PER_PACKET * 2];      // 7680 bytes (*2 because samples are 64 bit)
int16_t processBuffer[MIC_SAMPLES_PER_PACKET];         // 1920 bytes
QueueHandle_t	  mic_read_queue;

uint32_t mic_init(void){
	mic_read_queue = xQueueCreate(1, sizeof(uint32_t));
	if (mic_read_queue != NULL){
		return HAL_OK;
	}else{
		return HAL_ERROR;
	}
}

uint32_t mic_start(void){
	HAL_StatusTypeDef status = HAL_I2S_Receive_DMA(&hi2s2, (uint16_t*) sampleBuffer, MIC_SAMPLES_PER_PACKET * 2);
	return (uint32_t)status;
}

uint32_t mic_stop(void){
	HAL_StatusTypeDef status = HAL_I2S_DMAStop(&hi2s2);
	return (uint32_t)status;
}

uint32_t mic_pause(void){
	HAL_StatusTypeDef status = HAL_I2S_DMAPause(&hi2s2);
	return (uint32_t)status;
}

uint32_t mic_resume(void){
	HAL_StatusTypeDef status = HAL_I2S_DMAResume(&hi2s2);
	return (uint32_t)status;
}

uint32_t mic_read(void){
	uint32_t buffer_start;
	xQueueReceive(mic_read_queue, &buffer_start, portMAX_DELAY);
	int16_t *dest = processBuffer;
	int32_t *data_in = &sampleBuffer[buffer_start];
	uint32_t bytes_read = 0;

	for (uint16_t i = 0; i < (MIC_SAMPLES_PER_PACKET / 2); i++) {
		// dither the LSB with a random bit
		//int16_t sample = (data_in[0] & 0xfffffffe) | (rand() & 1);
		int16_t sample = (int16_t)*data_in;

		*dest++ = sample;     // left channel has data
		data_in += 2;
		bytes_read += 2;
	}
	return bytes_read;
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	portBASE_TYPE high_priority_task_woken = pdFALSE;
	uint32_t buffer_Start = 0;
	xQueueSendToBackFromISR(mic_read_queue, &buffer_Start, &high_priority_task_woken);
	portYIELD_FROM_ISR(high_priority_task_woken);
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
	portBASE_TYPE high_priority_task_woken = pdFALSE;
	uint32_t buffer_Start = MIC_SAMPLES_PER_PACKET;
	xQueueSendToBackFromISR(mic_read_queue, &buffer_Start, &high_priority_task_woken);
	portYIELD_FROM_ISR(high_priority_task_woken);
}

void record_wav(char *filename, uint32_t rec_time)
{
    // Use POSIX and C standard library functions to work with files.
    int32_t flash_wr_size = 0;
    uint32_t bytes_read = 0;

    uint32_t flash_rec_time = MIC_BYTE_RATE * rec_time;
    const wav_header_t wav_header = WAV_HEADER_PCM_DEFAULT(flash_rec_time, 16, MIC_SAMPLE_FREQUENCY, 1);

    // First check if file exists before creating a new file.
    FILINFO finfo;
    FIL		file;
    if (f_stat(filename, &finfo) == FR_OK) {
        // Delete it if it exists
        f_unlink(filename);
    }

    // Create new WAV file
    if (f_open(&file, filename, (FA_OPEN_APPEND | FA_WRITE)) != FR_OK){
        return;
    }

    // Write the header to the WAV file
    uint32_t bytes_written = 0;
    f_write(&file, &wav_header, sizeof(wav_header), (UINT *)&bytes_written);

    // Start recording
    mic_start();
    while (flash_wr_size < flash_rec_time) {
        // Read the RAW samples from the microphone
        //TickType_t start = xTaskGetTickCount();
        bytes_read = mic_read();
		//TickType_t stop = xTaskGetTickCount();
		//TickType_t start = xTaskGetTickCount();
		f_write(&file, processBuffer, bytes_read, (UINT *)&bytes_written);
		//TickType_t stop = xTaskGetTickCount();
		flash_wr_size += bytes_read;
    }

    //ESP_LOGI(SDCARD_TAG, "Recording done!");
    mic_stop();
    f_close(&file);
}
