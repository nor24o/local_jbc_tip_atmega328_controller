

#include <EEPROM.h>  // Include the EEPROM library
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PID_v1.h>
#include <ezButton.h>

#define EEPROM_SIZE 512      // Size of EEPROM in bytes
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 32     // OLED display height, in pixels
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define _BV(bit) (1 << (bit))

// Define variables
volatile int encoder_count = 0;      // Encoder count variable
volatile byte enc_a_last_state = 0;  // Encoder pin A last state

// Define encoder pins
#define ENC_A_PIN 3  // PD3
#define ENC_B_PIN 2  // PD2
#define ENC_BTN 4
#define SLEEP_BTN 10


ezButton SLEEP_BTN_B(SLEEP_BTN);  // create ezButton object that attach to pin 7;
ezButton ENC_BTN_B(ENC_BTN);      // create ezButton object that attach to pin 7;
#define SO 12                     // MISO
#define TC_0 9                    // CS Pin of MAX6607
#define SCK 13                    // Serial Clock

#define MOSFET 11  // Serial Clock
uint8_t target_lo_nibble, target_hi_nibble;
volatile unsigned long long lastTime;

int position = 0;
double target = 250.0;
int default_target = 100.0;  // Default target temperature value
bool reinit_pwm=false;
//Define Variables we'll be connecting to
double Input, Output;

//Specify the links and initial tuning parameters
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &target, Kp, Ki, Kd, DIRECT);

bool fistBoot = true;
int run_state=1;
uint8_t get_low_byte(uint16_t num) {
  return (uint8_t)(num & 0xFF);
}

uint8_t get_high_byte(uint16_t num) {
  return (uint8_t)((num >> 8) & 0xFF);
}

uint16_t combine_bytes(uint8_t low_byte, uint8_t high_byte) {
  return (uint16_t)((high_byte << 8) | low_byte);
}

float read_temp(int pin) {
  unsigned int value = 0;
  int error_tc;
  float temp;

  digitalWrite(pin, LOW);  // Enable device

  /* Cycle the clock for dummy bit 15 */
  digitalWrite(SCK, HIGH);
  digitalWrite(SCK, LOW);
  for (int i = 11; i >= 0; i--) {
    digitalWrite(SCK, HIGH);        // Set Clock to HIGH
    value += digitalRead(SO) << i;  // Read data and add it to our variable
    digitalWrite(SCK, LOW);         // Set Clock to LOW
  }

  /* Read the TC Input inp to check for TC Errors */
  digitalWrite(SCK, HIGH);     // Set Clock to HIGH
  error_tc = digitalRead(SO);  // Read data
  digitalWrite(SCK, LOW);      // Set Clock to LOW

  digitalWrite(pin, HIGH);  //Disable Device

  temp = (value * 0.25);  // Multiply the value by 25 to get temp in [ch730]C

  if (error_tc != 0) {
    return 9999;
  } else {
    return temp;
  }
}

void encoder_ISR() {
  // Read encoder pins and update encoder count
  byte enc_a_state = digitalRead(ENC_A_PIN);
  byte enc_b_state = digitalRead(ENC_B_PIN);
  if (enc_a_state != enc_a_last_state) {
    if (enc_b_state != enc_a_state) {
      target += 5.0;
    } else {
      target -= 5.0;
    }
  }
  // Constrain target temperature to be between 250 and 450 Celsius
  target = constrain(target, 150.0, 450.0);
  enc_a_last_state = enc_a_state;

  // Check if the target temperature has changed and is not NaN
  if (!isnan(target) && (combine_bytes(target_lo_nibble, target_hi_nibble) != target)) {
    // Update the target temperature in EEPROM
    target_hi_nibble = get_high_byte(target);
    target_lo_nibble = get_low_byte(target);
    Serial.println(target_hi_nibble, HEX);
    Serial.println(target_lo_nibble, HEX);
    EEPROM.write(0, target_hi_nibble);
    EEPROM.write(1, target_lo_nibble);
    delay(10);  // Wait for write operation to complete
  }
}
int i = 0;
void setup() {
  Serial.begin(9600);








  pinMode(MOSFET, OUTPUT);
  //SPCR |= _BV(SPE); // Enable SPI in slave mode


  pinMode(SO, INPUT);
  pinMode(SCK, OUTPUT);
  pinMode(TC_0, OUTPUT);
  digitalWrite(TC_0, HIGH);

  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(ENC_BTN, INPUT_PULLUP);
  pinMode(SLEEP_BTN, INPUT_PULLUP);

  ENC_BTN_B.setDebounceTime(50);    // set debounce time to 50 milliseconds
  SLEEP_BTN_B.setDebounceTime(100);  // set debounce time to 50 milliseconds
  // Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoder_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encoder_ISR, CHANGE);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
  }

  // Read the target temperature from EEPROM
  target_hi_nibble = EEPROM.read(0);
  target_lo_nibble = EEPROM.read(1);
  // Check if the stored target temperature is within the valid range
  target = combine_bytes(target_lo_nibble, target_hi_nibble);
  Serial.println(target);
  /*if (isnan(target)) {
    // If the stored target temperature is outside the valid range or NaN, initialize it to the default value
    target = default_target;
    target_hi_nibble=get_high_byte(target);
    target_lo_nibble= get_low_byte(target);
    EEPROM.write(0, target_hi_nibble);
    EEPROM.write(1, target_lo_nibble);  // Save the default value to EEPROM
  }*/

  // Clear the buffer
  display.clearDisplay();

  display.setTextSize(2);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text



  TCCR2A = (1 << COM2A1) | (1 << WGM20) | (1 << WGM21);  // Set non-inverting mode for OC2B (Pin 3) and Fast PWM mode (WGM20, WGM21)
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);      // Set prescaler to 64 (CS22)
  OCR2A = 200;                                           // Set the TOP value for 50Hz PWM frequency (16MHz / (64 * 50Hz * (1 + 124)) = 50Hz)

  int temp = read_temp(TC_0);
  if (fistBoot && (temp < 100)) {
    display.setCursor(0, 0);
    display.print("Booting");
    display.display();
    fistBoot = false;
    do {
      i++;
      delay(1000);
      OCR2A = 20;
    } while (i < 4);
  }
  myPID.SetOutputLimits(0, 180);  // Set the output range to 0-200
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}



int buttonLastState = 1;
unsigned long long buttonTime = 0;

void loop() {
  ENC_BTN_B.loop();
  SLEEP_BTN_B.loop();

  if(ENC_BTN_B.isPressed())
    run_state=!run_state;

  if(ENC_BTN_B.isReleased())
    Serial.println("The button is released");

  if(SLEEP_BTN_B.isPressed()){
    run_state=0;
    delayMicroseconds(20);
  }


  if(SLEEP_BTN_B.isReleased()){
    run_state=1;
    delayMicroseconds(20);
  }



if(run_state==1){
  if(reinit_pwm)
  init_PWM();
  // OCR2B = 200;  // Set the duty cycle by updating the OCR2B register
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Set: ");
  display.setCursor(0, 16);
  display.print("Act: ");
  OCR2A = 0;
  delay(100);
  double temp = read_temp(TC_0);

  Input = temp;
  myPID.Compute();

  display.setCursor(55, 16);
  if (temp < 100) display.print(' ');
  if (temp < 10) display.print(' ');
  display.print((int)temp);

  display.setCursor(55, 0);
  if (target < 100) display.print(' ');
  if (target < 10) display.print(' ');
  display.print((int)target);
  display.print("   ");

  //int progressWidth = map(constrain(10 * (target - temp), 0, 255), 0, 100, 0, SCREEN_WIDTH / 2);
  int progressHeight = map(Output, 0, 100, 0, SCREEN_HEIGHT);
  // Calculate the y-coordinate of the top of the progress bar
  int progressTop = SCREEN_HEIGHT - progressHeight;

  // Draw the progress bar
  display.fillRect(SCREEN_WIDTH - 4, progressTop, SCREEN_WIDTH / 2, progressHeight, WHITE);

  display.display();
  if (Output < 70) {
    OCR2A = 0;
  } else
    OCR2A = Output;
  //Serial.println(Output);
  delay(70);
}
if(run_state==0){
  OCR2A =0;
  digitalWrite(MOSFET,LOW);
  reinit_pwm=true;
  display.clearDisplay();
  display.setTextSize(3);
  display.setCursor(0, 0);
  display.print("Sleep");
  display.display();
}
}

void init_PWM(){
    TCCR2A = (1 << COM2A1) | (1 << WGM20) | (1 << WGM21);  // Set non-inverting mode for OC2B (Pin 3) and Fast PWM mode (WGM20, WGM21)
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);      // Set prescaler to 64 (CS22)
    reinit_pwm=false;
    display.setTextSize(2);
}