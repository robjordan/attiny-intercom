#include <stdint.h>
#include <avr/interrupt.h>

#define NUM_BUFS 8
#define LEN_BUFS 32
/* ADC input PA4 (channel 4) - input from microphone */
#define MICPORT PORTA
#define MICPIN (1 << 4)
#define ADC_INPUT ADC_MUXPOS_AIN4_gc
/* DAC output pin PA6 - output to speaker */
#define SPEAKPORT PORTA
#define SPEAKPIN (1 << 6)
/* Sampling timer */
#define SAMPLE_TIMER TCB0
#define SAMPLE_FREQ 8000 /* Hz */
#define SAMPLE_TIMER_START() SAMPLE_TIMER.CTRLA |= TCB_ENABLE_bm
#define SAMPLE_TIMER_STOP() SAMPLE_TIMER.CTRLA &= ~(TCB_ENABLE_bm)
#define F_CPU 10000000  // 20Mhz prescaled by 2


volatile uint32_t systick_ms;

typedef enum {empty = 0, filling = 1, filled = 2} buffer_status;
typedef struct {
    buffer_status status;
    uint16_t samples[LEN_BUFS];
} buffer_t;

volatile buffer_t buffer[NUM_BUFS];
volatile uint8_t write_buf, write_pos, read_buf, read_pos = 0;

/* timer tick which is set to go off at 8000Hz */
/* needs to deal with output of samples; ADC is handled in a ResultReady IRQ */
ISR(TCB0_INT_vect){
    VPORTA.IN = PIN7_bm; // toggle the output  
    systick_ms++;
    dac_write_sample();
    TCB0.INTFLAGS = TCB_CAPT_bm;
    VPORTA.IN = PIN7_bm; // toggle the output
}

/* Part of RECORD routine - Grab result from ADC recording */
ISR(ADC0_RESRDY_vect) {
  VPORTA.IN = PIN7_bm; // toggle the output
  
  /* Interrupt flag cleared on reading of ADC result register */
  buffer[write_buf].samples[write_pos++] = ADC0.RES;
  if (write_pos == LEN_BUFS) {
    /* this one's full */
    buffer[write_buf].status = filled;
    write_buf = (write_buf + 1) % NUM_BUFS;
    write_pos = 0;

    if (buffer[write_buf].status != empty) {
      signal_error();
    }
    buffer[write_buf].status = filling;
  }
  VPORTA.IN = PIN7_bm; // toggle the output
}

void dac_write_sample(void) {

  static volatile uint8_t counter = 0;

  if (buffer[read_buf].status == filled) {
    /* Enable DAC */
	  DAC0.CTRLA |= DAC_ENABLE_bm;
    uint16_t sample = buffer[read_buf].samples[read_pos++];
    DAC0.DATA = (sample >> 5) & 0xFF;
    // DAC0.DATA = counter++;

    if (read_pos == LEN_BUFS) {
      // we've emptied this one
      buffer[read_buf].status = empty;
      read_buf = (read_buf+1) % NUM_BUFS;
      read_pos = 0;
    }
  }
}

void adc_init(void) {

  // Clear the buffers and buffer status
  (void)memset((void *)buffer, 0, sizeof buffer);
  /* ADC input */
	MICPORT.DIRCLR = MICPIN;
	/* 8 bit resolution */
	ADC0.CTRLA = ADC_RESSEL_10BIT_gc;
	/* Reference 2.5V */
	VREF.CTRLA |= VREF_ADC0REFSEL_2V5_gc;
  // How many samples to accumulate
  ADC0.CTRLB = ADC_SAMPNUM_ACC8_gc;
	/* Prescaler DIV2 */
	ADC0.CTRLC = ADC_PRESC_DIV2_gc;
	/* ADC input port (#defined) */
	ADC0.MUXPOS = ADC_INPUT;
	/* Enable */
	ADC0.CTRLA |= ADC_ENABLE_bm;
	/* Start Event Input enable (start conversion on event input) */
	ADC0.EVCTRL = ADC_STARTEI_bm;
	/* Clear interrupt flags */
	ADC0.INTFLAGS = 0xFF;
	/* Enable ADC result ready interrupt */
	ADC0.INTCTRL = ADC_RESRDY_bm;

}

void dac_init(void)
{
	/* DAC output pin */
	SPEAKPORT.DIRSET = SPEAKPIN;
	/* DAC reference 2.5V */
	VREF.CTRLA |= VREF_DAC0REFSEL_2V5_gc;
	/* Enable DAC output to pin */
	DAC0.CTRLA = DAC_OUTEN_bm;

	/* Ready to load first data and enable */
}

void timer_init(void) {
	/* Event input to SYNCCH0 from TCB0 */
	EVSYS.SYNCCH0 = EVSYS_SYNCCH0_TCB0_gc;

  //tcb periodic, div1, irq on ccmp, on
  uint16_t period = (F_CPU/SAMPLE_FREQ) - 1;
  SAMPLE_TIMER.CCMP = period;        // Aiming for 8kHz / 125us
  SAMPLE_TIMER.INTCTRL = TCB_CAPT_bm; //irq enable
  TCB0.CTRLA = TCB_ENABLE_bm; //enable, div1
}

void signal_error(void) {
	// Turn the LED on
    VPORTA.OUT |= PIN7_bm; 
    exit(-1);
}

void setup(){

  VPORTA.DIR |= PIN7_bm;  	//led on PA7 is an output
  VPORTA.OUT &= ~PIN7_bm;  	// pin low = LED off

  /* set the prescaler to 2 get 10MHz */
  ccp_write_io(
      (void *)&(CLKCTRL.MCLKCTRLB),
      CLKCTRL_PDIV_2X_gc | 1 << CLKCTRL_PEN_bp
  );

  timer_init();
  adc_init();
  dac_init();
  sei();

  /* Enable TCB0 event forwarding to ADC */
	EVSYS.ASYNCUSER1 = EVSYS_ASYNCUSER1_SYNCCH0_gc;

}

void loop() {

}