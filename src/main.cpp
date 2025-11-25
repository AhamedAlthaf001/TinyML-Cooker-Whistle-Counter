#define EIDSP_QUANTIZE_FILTERBANK 0

#define EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW 3

// Change this to match the Edge Impulse library that you downloaded
#include <Speech_recognition_inferencing.h>
#include <driver/i2s.h>
#include <LiquidCrystal.h>
// Settings
#define DEBUG 1                 // Enable pin pulse during ISR
static const int debug_pin = 2; // Toggles each DAC ISR (if DEBUG is set to 1)
static const float threshold = 0.80;
#define I2S_SAMPLE_RATE 16000
#define ADC_INPUT ADC1_CHANNEL_7
const int rs = 16, en = 23, d4 = 17, d5 = 18, d6 = 19, d7 = 21;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


// Audio buffers, pointers and selectors
typedef struct
{
  signed short *buffers[2];
  unsigned char buf_select;
  unsigned char buf_ready;
  unsigned int buf_count;
  unsigned int n_samples;
} inference_t;

// Globals - DMA and ADC
volatile uint8_t recording = 0;
volatile boolean results0Ready = false;
volatile boolean results1Ready = false;
uint16_t adc_buf_0[16000]; // ADC results array 0
uint16_t adc_buf_1[16000]; // ADC results array 1

// Globals - Edge Impulse
static inference_t inference;
static bool record_ready = false;
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static int print_results = -(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW);

/**
 * @brief      Copy sample data in selected buf and signal ready when buffer is full
 *
 * @param[in]  *buf  Pointer to source buffer
 * @param[in]  buf_len  Number of samples to copy from buffer
 */
static void audio_rec_callback(uint16_t *buf, uint32_t buf_len)
{

  // Copy samples from DMA buffer to inference buffer
  if (recording)
  {
    for (uint32_t i = 0; i < buf_len; i++)
    {

      // Convert 12-bit unsigned ADC value to 16-bit PCM (signed) audio value
      inference.buffers[inference.buf_select][inference.buf_count++] =
          ((int16_t)buf[i] - 2048) * 16;

      // Swap double buffer if necessary
      if (inference.buf_count >= inference.n_samples)
      {
        inference.buf_select ^= 1;
        inference.buf_count = 0;
        inference.buf_ready = 1;
      }
    }
  }
}

void i2sInit()
{
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
      .sample_rate = I2S_SAMPLE_RATE,               // The format of the signal using ADC_BUILT_IN
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = I2S_COMM_FORMAT_I2S_MSB,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 4,
      .dma_buf_len = 1024,
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0};
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_adc_mode(ADC_UNIT_1, ADC_INPUT);
  i2s_adc_enable(I2S_NUM_0);
}

void reader(void *pvParameters)
{
  // The 4 high bits are the channel, and the data is inverted
  uint16_t offset = (int)ADC_INPUT * 0x1000 + 0xFFF;
  size_t bytes_read;
   uint32_t count = 0;
  while (1)
  {
    uint16_t buffer[2] = {0};
    i2s_read(I2S_NUM_0, &buffer, sizeof(buffer), &bytes_read, 15);
    //Serial.printf("%d  %d\n", offset - buffer[0], offset - buffer[1])
#if DEBUG
    digitalWrite(debug_pin, HIGH);
#endif
    // See which buffer has filled up, and dump results into large buffer
     if (bytes_read == sizeof(buffer)) {
       adc_buf_0[count] = offset - buffer[0];
       //adc_buf_1[count] = offset - buffer[0];
       count++;
     }
     else
     {
     //Serial.println("buffer empty");
     }
     if (count == I2S_SAMPLE_RATE) {
       audio_rec_callback(adc_buf_0, count);
       count=0;
      //i2s_adc_disable(I2S_NUM_0);
      //delay(2);
      //i2s_adc_enable(I2S_NUM_0);
     }
      // Debug: make pin low after copying buffer
#if DEBUG
      digitalWrite(debug_pin, LOW);
#endif
    }
  }


/**
 * @brief      Printf function uses vsnprintf and output using Arduino Serial
 *
 * @param[in]  format     Variable argument list
 */
void ei_printf(const char *format, ...)
{
  static char print_buf[1024] = {0};

  va_list args;
  va_start(args, format);
  int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
  va_end(args);

  if (r > 0)
  {
    Serial.write(print_buf);
  }
}

/**
 * @brief      Wait for full buffer
 *
 * @return     False if buffer overrun
 */
static bool microphone_inference_record(void)
{
  bool ret = true;

  if (inference.buf_ready == 1)
  {
    ei_printf(
        "Error sample buffer overrun. Decrease the number of slices per model window "
        "(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)\n");
    ret = false;
  }

  // TODO: Make this non-blocking (use RTOS?)
  while (inference.buf_ready == 0)
  {
    delay(1);
  }

  inference.buf_ready = 0;

  return ret;
}

/**
 * Get raw audio signal data
 */
static int microphone_audio_signal_get_data(size_t offset,
                                            size_t length,
                                            float *out_ptr)
{
  numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);

  return 0;
}

/**
 * @brief      Stop PDM and release buffers
 */
static void microphone_inference_end(void)
{
  // TODO: Stop DMA and ADC

  // Free up double buffer
  free(inference.buffers[0]);
  free(inference.buffers[1]);
}

/**
 * @brief     Print string to LCD
 * 
 * @param[in] String as char array
 */
void lcd_print_string(char str[])
{

  // Disable recording for 1-second hold-off
  recording = 0;

  Serial.println(str);

  // Re-enable recording
  recording = 1;
}

/*******************************************************************************
 * Main
 */

void setup()
{

  // Configure pin to toggle on DMA interrupt
#if DEBUG
  pinMode(debug_pin, OUTPUT);
#endif

  // Configure serial port for debugging
  Serial.begin(115200);

//lcd begin
  lcd.begin(16, 2);

   // Print a Welcome message to the LCD.
  lcd.print("TINYML SPEECH");

  // Print summary of inferencing settings (from model_metadata.h)
  ei_printf("Inferencing settings:\n");
  ei_printf("\tInterval: %.2f ms.\n", (float)EI_CLASSIFIER_INTERVAL_MS);
  ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
  ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) /
                                          sizeof(ei_classifier_inferencing_categories[0]));

  // Initialize classifier
  run_classifier_init();

  // Create double buffer for inference
  inference.buffers[0] = (int16_t *)malloc(EI_CLASSIFIER_SLICE_SIZE *
                                           sizeof(int16_t));
  if (inference.buffers[0] == NULL)
  {
    ei_printf("ERROR: Failed to create inference buffer 0");
    return;
  }
  inference.buffers[1] = (int16_t *)malloc(EI_CLASSIFIER_SLICE_SIZE *
                                           sizeof(int16_t));
  if (inference.buffers[1] == NULL)
  {
    ei_printf("ERROR: Failed to create inference buffer 1");
    free(inference.buffers[0]);
    return;
  }

  // Set inference parameters
  inference.buf_select = 0;
  inference.buf_count = 0;
  inference.n_samples = EI_CLASSIFIER_SLICE_SIZE;
  inference.buf_ready = 0;

  // Configure DMA to sample from ADC at 16kHz (start sampling immediately)
  i2sInit();
  // Create a task that will read the data
  xTaskCreatePinnedToCore(reader, "ADC_reader", 4096, NULL, 1, NULL, 1);

  // Start recording to inference buffers
  recording = 1;
}

void loop()
{

  // Wait until buffer is full
  bool m = microphone_inference_record();
  if (!m)
  {
    ei_printf("ERROR: Audio buffer overrun\r\n");
    return;
  }

  // Do classification (i.e. the inference part)
  signal_t signal;
  signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
  signal.get_data = &microphone_audio_signal_get_data;
  ei_impulse_result_t result = {0};
  EI_IMPULSE_ERROR r = run_classifier_continuous(&signal, &result, debug_nn);
  if (r != EI_IMPULSE_OK)
  {
    ei_printf("ERROR: Failed to run classifier (%d)\r\n", r);
    return;
  }

  // YOUR CODE GOES HERE! (if you want to do something with inference results)

  // ***Example below: print keyword to LCD***

  //

  // ***End example***

  // Print output predictions (once every 3 predictions)
  if (++print_results >= (EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW >> 1))
  {
    // Comment this section out if you don't want to see the raw scores
    ei_printf("Predictions (DSP: %d ms, NN: %d ms)\r\n", result.timing.dsp, result.timing.classification);
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
    {
      ei_printf("    %s: %.5f\r\n", result.classification[ix].label, result.classification[ix].value);
      if(result.classification[ix].value > threshold)
      {
        lcd.setCursor(4,1);
        lcd.print(result.classification[ix].label);
        lcd.print("           ");
      }
    }
    print_results = 0;
  }
}