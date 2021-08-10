#include <stdint.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <Arduino.h>
#include <SPI.h>
#include <NRFLite.h>
#include <EEPROM.h>

#define NUM_BUFS 8
#define LEN_BUFS 32
/* ADC input PA4 (channel 4) - input from microphone */
#define MICPORT PORTA
const static uint8_t  MICPIN = (1 << 4);
const static uint8_t ADC_INPUT = ADC_MUXPOS_AIN4_gc;
/* DAC output pin PA6 - output to speaker */
#define SPEAKPORT PORTA
const static uint8_t  SPEAKPIN = (1 << 6);
/* the push-to-talk button */
#define PTTPORT PORTA
const static uint8_t  PTTPIN = (1 << 5);
// The NRF module interrupt
const static uint8_t IRQPIN = (1 << 3);

// Wireless module constants
const static uint8_t PIN_RADIO_CE = PIN_PB0;
const static uint8_t PIN_RADIO_CSN = PIN_PB1;
const static uint8_t PIN_RADIO_IRQ = PIN_PB3;
const static uint8_t PIN_LED = PIN_PA7;
const static uint8_t DEFAULT_CHANNEL = 111;

/* Sampling timer */
#define SAMPLE_TIMER TCB0
#define SAMPLE_FREQ 8000 /* Hz */
#define SAMPLE_TIMER_START() SAMPLE_TIMER.CTRLA |= TCB_ENABLE_bm
#define SAMPLE_TIMER_STOP() SAMPLE_TIMER.CTRLA &= ~(TCB_ENABLE_bm)
#define F_CPU 10000000  // 20Mhz prescaled by 2
#define LED_ON() VPORTA.OUT |= PIN7_bm
#define LED_OFF() VPORTA.OUT &= ~(PIN7_bm)
#define LED_TOGGLE() VPORTA.IN = PIN7_bm

void dac_write_sample(void);
void adc_init(void);
void dac_init(void);
void timer_init(void);
void signal_error(void);
// extern "C" {void ccp_write_io	(void *, unsigned char);}
void loop(void);

typedef enum {empty = 0, in_use = 1, filled = 2} buffer_status;
typedef struct {
    buffer_status status;
    uint8_t samples[LEN_BUFS];
} buffer_t;

// Buffer and pointers for ADC to Wireless
buffer_t adc_buffer[NUM_BUFS];
uint8_t adc_write_buf, adc_write_pos, adc_read_buf;

// Buffer and pointers for Wireless to DAC
buffer_t dac_buffer[NUM_BUFS];
uint8_t dac_write_buf, dac_read_buf, dac_read_pos = 0;

volatile uint8_t timer_tick = false;
volatile uint8_t adc_data_ready = false;
volatile uint16_t adc_sample = 0;
volatile enum ptt_mode {listening = 0, speaking = 1} ptt_mode;
volatile uint8_t radio_interrupted = false;

// Wireless
NRFLite _radio;
uint8_t _data;
uint32_t _lastSendTime;
volatile uint8_t _sendSucceeded, _sendFailed, _dataWasReceived; 
uint8_t radio_id, destination_radio_id, channel;



/* timer tick which is set to go off at 8000Hz */
/* needs to deal with output of samples; ADC is handled in a ResultReady IRQ */
ISR(TCB0_INT_vect){
//    timer_tick = true;
    TCB0.INTFLAGS = TCB_CAPT_bm;
    dac_write_sample();
}



/* Grab sample from ADC */
ISR(ADC0_RESRDY_vect) {

  /* Interrupt flag cleared on reading of ADC result register */
  adc_sample = ADC0.RES;
  adc_data_ready = true;

}

ISR(PORTA_PORT_vect) {
  uint8_t flags = PORTA.INTFLAGS;
  PORTA.INTFLAGS = flags; //clear flags
  if (flags & PTTPIN) {
  // PTT button flipped
    ptt_mode = ((PTTPORT.IN & PTTPIN) ? listening : speaking);

  }    
}

ISR(PORTB_PORT_vect) {
  uint8_t flags = PORTB.INTFLAGS;
  PORTB.INTFLAGS = flags; //clear flags
  if (flags & IRQPIN) {
    radio_interrupted = true;
  }
}

void dac_write_sample(void) {

  if (dac_buffer[dac_read_buf].status == filled) {
    /* Enable DAC */
	  DAC0.CTRLA |= DAC_ENABLE_bm;
    DAC0.DATA = dac_buffer[dac_read_buf].samples[dac_read_pos++];

    if (dac_read_pos == LEN_BUFS) {
      // we've emptied this one
      dac_buffer[dac_read_buf].status = empty;
      dac_read_buf = (dac_read_buf+1) % NUM_BUFS;
      dac_read_pos = 0;
    }
  }
}

// Write the most recent ADC sample to the buffer
// Return true if a buffer has been filled.
// Return false if a buffer still has space to write.
bool adc_write_sample(uint16_t sample) {

  bool rc = false;

  // Samples are inherently 10-bit, and 8 samples are aggregated, so we need to divide by 32
  // (Better to Compand, e.g. A-law).
  adc_buffer[adc_write_buf].samples[adc_write_pos++] = (uint8_t)((sample >> 5) & 0xFF);
  if (adc_write_pos == LEN_BUFS) {
    /* this one's full */
    rc = true;
    adc_buffer[adc_write_buf].status = filled;
    adc_write_buf = (adc_write_buf + 1) % NUM_BUFS;
    adc_write_pos = 0;
    if (adc_buffer[adc_write_buf].status != empty) {
//      Serial.println("error: ADC buffer overrun");
    }
    adc_buffer[adc_write_buf].status = in_use;
  }

  return rc; 
}

void radio_read_packet() {

  uint8_t len;

  while (len = _radio.hasData()) {
    // data packets are <= 32 bytes; in our case always 32
    if (len != LEN_BUFS) {
//      Serial.println("error: buffer length != 32");
    } else {
      if (dac_buffer[dac_write_buf].status != empty) {
//        Serial.println("error: DAC buffer overrun");
      }
      _radio.readData(&dac_buffer[dac_write_buf].samples[0]);
      dac_buffer[dac_write_buf].status = filled;
      dac_write_buf = (dac_write_buf + 1) % NUM_BUFS;
    }
  }

}

void adc_init(void) {

  // Clear the buffers and buffer status
  (void)memset((void *)adc_buffer, 0, sizeof adc_buffer);
  adc_write_buf = 0;
  adc_write_pos = 0;
  adc_read_buf = 0;
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
  /* Enable TCB0 event forwarding to ADC */
  EVSYS.ASYNCUSER1 = EVSYS_ASYNCUSER1_SYNCCH0_gc;
}

void dac_init(void)
{
  // Clear the buffers and buffer status
  (void)memset((void *)dac_buffer, 0, sizeof dac_buffer);
  dac_write_buf = 0;
  dac_read_buf = 0;
  dac_read_pos = 0;
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

void radio_init()
{
  radio_id = EEPROM.read(0);
  destination_radio_id = EEPROM.read(1);
  channel = DEFAULT_CHANNEL;
  
  Serial.println("About to init radio:");
  Serial.print("radio_id: ");
  Serial.println(radio_id);
  Serial.print("destination_radio_id: ");
  Serial.println(destination_radio_id);
  Serial.print("channel: ");
  Serial.println(channel);
  
  if (!_radio.init(radio_id, PIN_RADIO_CE, PIN_RADIO_CSN, NRFLite::BITRATE250KBPS, channel)) {

    Serial.println("Cannot communicate with radio.");
      while (1); // Wait here forever.
  } else {
    Serial.println("Radio initialised.");
  }

  // attachInterrupt(digitalPinToInterrupt(PIN_RADIO_IRQ), radioInterrupt, FALLING);
    

}

void setup() {

  VPORTA.DIR |= PIN7_bm;  	//led on PA7 is an output
  LED_OFF();

  // The push-to-talk button is an input with interrupt on both edges / change 
  PTTPORT.PIN5CTRL = PORT_PULLUPEN_bm | 0x1;

  // The NRF module interrupts on PIN_RADIO_IRQ (PB3) on a falling edge
  PORTB.PIN3CTRL = PORT_PULLUPEN_bm | 0x3;

  Serial.begin(115200);

  timer_init();
  adc_init();
  dac_init();
  radio_init();
  sei();


}

void loop() {

  static enum ptt_mode prev_ptt_mode = listening;

  if (adc_data_ready) {
    adc_data_ready = false;
    
    if (ptt_mode == speaking) {
      // Save an audio sample to buffer
      if (adc_write_sample(adc_sample)) {
        // There's a full buffer that needs to be sent to radio
        _radio.startSend(
          destination_radio_id, 
          &adc_buffer[adc_read_buf].samples[0], 
          LEN_BUFS,
          NRFLite::NO_ACK
        );
        adc_buffer[adc_read_buf].status = empty;
        adc_read_buf = (adc_read_buf + 1) % NUM_BUFS;
      }
    } else {
      
      // ptt_mode is listening
      if (prev_ptt_mode == speaking) {
        // Switch to listening mode
        prev_ptt_mode = listening;
        if (!_radio.startRx()) {
           Serial.println("Cannot switch back to listen mode.");
           while (true)
            ;
        }
      }
    }
  }


  if (radio_interrupted) {
    radio_interrupted = false;

    // Ask the radio what caused the interrupt.  This also resets the IRQ pin 
    // on the radio so a new interrupt can be triggered.
    // This is copied from the example interrupt handler, but in this program
    // it's not in interrupt context.
    uint8_t txOk, txFail, rxReady;
    _radio.whatHappened(txOk, txFail, rxReady);

    // txOk = the radio successfully transmitted data.
    // txFail = the radio failed to transmit data.
    // rxReady = the radio received data.

    if (txOk) {
//        Serial.println("Send data succeeded.");
//        adc_buffer[adc_read_buf].status = empty;
//        adc_read_buf = (adc_read_buf + 1) % NUM_BUFS;
    }
    
    if (txFail) {
//        Serial.println("Send data failed.");
        adc_buffer[adc_read_buf].status = empty;
    }

    if (rxReady) {
//      Serial.println("Packet received.");
      radio_read_packet();
    }

  }
}
