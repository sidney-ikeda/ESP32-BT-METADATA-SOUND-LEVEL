/*
 * v1.0 - Included metadata for BluetoothA2DPSink and sound level for I2S
 * This sketch is a extension/expansion/rework of the follow library. Please refer the author for lasted versions and copyright.
*/
#include <Arduino.h>
#include <BluetoothA2DPSink.h>
#include <esp_log.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

//////
////// LCD config
#define MIN_LCD_LEVEL     3
#define MAX_LCD_LEVEL     160 //128
#define MAX_LCD_RESOL     128
#define MIN_SAMPLE_LEVEL  0
#define MAX_SAMPLE_LEVEL  7168 //65535 //32768
#define MAX_CHAR_PER_ID   70
#define MAX_CHAR_PER_TITLE 50
#define MAX_CHAR_PER_ARTIST 20
#define MAX_CHAR_PER_ALBUM 20
#define MAX_CHAR_PER_GENRE 10
#define MAX_CHAR_DISPLAY  (4 * MAX_CHAR_PER_ID) 
//#define SCROLL_SPEED      3
uint8_t scroll_speed = 3;
byte Lpeak = 0;
byte Rpeak = 0;

////// LED config
#define MIN_LED_LEVEL     0
#define MAX_LED_LEVEL     520 //1024
#define MAX_LED_RESOL     1024
#define LED_PEAK          352
int TOP;                // Don't allow the bars to go offscreen

/////////////////////////////////////////////////////////////////////////
////// 
////// Variables of bluetooth
char BtName[] = "ESP32-BT";
char title[MAX_CHAR_PER_TITLE];
char artist[MAX_CHAR_PER_ARTIST];
char album[MAX_CHAR_PER_ALBUM];
char genre[MAX_CHAR_PER_GENRE];
char music[MAX_CHAR_DISPLAY];

////// Time constants for call functions
unsigned long previousMillis = 0; // will store last time.
unsigned long interval = 30;      // interval in ms for call leds function.
unsigned long previousMillis2 = 0; // will store last time.
unsigned long interval2 = 50;   // interval in ms to call lcd function. Response also for lcd scroll speed.
unsigned long previousMillis3 = 0; // will store last time.
unsigned long interval3 = 20;   // interval in ms for call sounds level function.

/////// Variables of lcd
u8g2_uint_t x;           
u8g2_uint_t offset = 0;  // current offset for the scrolling text
u8g2_uint_t width;   // pixel total of width of the scrolling text (must be lesser than 128 unless U8G2_16BIT is defined
//u8g2_uint_t widthM;
//u8g2_uint_t widthT;
//u8g2_uint_t widthAl;    
//u8g2_uint_t widthAr;
int level;
int level_lcd_l;
int level_lcd_r;

//////// Variables of led
const int ledPin = 12;  // 12 corresponds to GPIO12
const int ledPin2 = 13;  // 13 corresponds to GPIO13
const int ledPin3 = 14;  // 14 corresponds to GPIO14
// setting PWM properties
const int freq = 5000;
const int ledChannel = 0; //led for Left channel
const int ledChannel2 = 1; //led for Right channel
//const int ledChannel3 = 2;
const int resolution = 10; //2^10 = 1024 steps
int level_led_l;
int level_led_r;

//////// Variables read data stream
uint32_t channel_len;
//uint16_t channel_l;
//uint16_t channel_r;
//int16_t sample_lr;
//uint16_t p_sample_lr;
//uint16_t temp_sample_lr;
int16_t sample_l;
uint16_t p_sample_l;
uint16_t temp_sample_l;
int16_t sample_r;
uint16_t p_sample_r;
uint16_t temp_sample_r;
bool data_received = false;
bool metadata_received = false;

//////// Sound level variables
const int sampleWindow = 25; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;
//unsigned int peakToPeak = 0;
int16_t peakToPeak_l = 0;   // peak-to-peak level
int16_t peakToPeak_r = 0;   // peak-to-peak level

BluetoothA2DPSink a2dp_sink;

U8G2_SSD1306_128X32_UNIVISION_2_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
//U8G2_SSD1306_128X32_UNIVISION_2_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);  // Adafruit ESP8266/32u4/ARM Boards + FeatherWing OLED
//U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);

int higher (int x, int y, int z) {
  int v = 0;
  if (x > y) {
    v = x;
  } else if (y > x) {
    v = y;
  }
  if (v > z) {
    v = v;
  } else if (z > v) {
    v = z;
  }
  return v;
}

void metadata_read(uint8_t id, const uint8_t *text){ // read metadata of music from BluetoothA2DPSink callback
    //ESP_LOGI(TAG, "Teste de callback do metadados atributo id: 0x%x e text: %s", id, text);
    if (id == 0x1) { //ESP_AVRC_MD_ATTR_TITLE
      //sprintf(genre, "%s", text);
      for (int i=0; i<MAX_CHAR_PER_TITLE - 1; i++) { // resizing the char array of id to fit in to lcd and avoid freezing
        title[i] = text[i];
      }
      //printf("Título: %s \n", title);
      //widthT = u8g2.getUTF8Width(title);    // calculate the pixel width of the text
    }
    if (id == 0x2) { //ESP_AVRC_MD_ATTR_ARTIST
      //sprintf(artist, "%s", text);
      for (int i=0; i<MAX_CHAR_PER_ARTIST - 1; i++) {
        artist[i] = text[i];
      }      
      //printf("Artista: %s \n", artist);
      //widthAr = u8g2.getUTF8Width(artist);
    }
    if (id == 0x4) { //ESP_AVRC_MD_ATTR_ALBUM
      //sprintf(album, "%s", text);
      for (int i=0; i<MAX_CHAR_PER_ALBUM - 1; i++) {
        album[i] = text[i];
      }
      //printf("Álbum: %s \n", album);
      //widthAl = u8g2.getUTF8Width(album);  
    }
    //if (id == 0x20) { //ESP_AVRC_MD_ATTR_GENRE
      //sprintf(genre, "%s", text);
      //for (int i=0; i<MAX_CHAR_PER_GENRE - 1; i++) {
      //  genre[i] = text[i];
      //}
      //printf("Genro: %s \n", text);
    //}
  //sprintf(music, "Title:%s Artist:%s Album:%s ", title, artist, album); 
  //width = u8g2.getUTF8Width(music);
  //width = higher(widthT, widthAr, widthAl);
  metadata_received = true;
} 

void data_received_callback() { // flag if sound data is incoming
  //ESP_LOGI(TAG, "Data packet received");
  data_received = true;
}

void example_disp_buf(uint8_t* data_buf, uint32_t len){ // only for view the data sound
    printf("======\n");
    for (int i = 0; i < len; i++) {
        printf("%02x ", data_buf[i]);
        if ((i + 1) % 8 == 0) {
            Serial.printf("\n");
        }
    }
    printf("======\n");
}

void read_data_stream(const uint8_t *data_stream, uint32_t len){ // sample channel L and R data from BluetoothA2DPSink callback
    
    //example_disp_buf((uint8_t*) data_stream, (uint32_t) len);
    for (int i=0; i<len/4; i++) {//use i<len/2 for sample both channel
      //temp_sample_lr = data_stream[i*2] | data_stream[i*2+1]<<8; //merging the 8bits data stream to 16bits to obtain each original channels
      ////http://lab2104.tu-plovdiv.bg/index.php/2019/03/11/binary-encoding-for-digital-audio/
      ////https://stackoverflow.com/questions/43120199/converting-2s-complement-of-values-back-and-forth-in-c-by-calculation-or-cast
      //sample_lr = static_cast<int16_t>(temp_sample_lr); //call i2s 2's complement converter function to reconstruct the actual amplitude of signal. 
      //p_sample_lr = abs(sample_lr); //converting negative part of sample to positive, it will reduce the peak a peak value but avoid work with dc value.
      //p_sample_lr = sample_lr + 0x8000; //add dc value: 2^15bits = 32768 = 0x8000 for get only positive values.
      //corr_data[i]= sample + 0x8000;
      
      temp_sample_l = data_stream[i*4] | data_stream[i*4+1]<<8; //sample only L channel
      sample_l = static_cast<int16_t>(temp_sample_l);
      p_sample_l = abs(sample_l);
      
      temp_sample_r = data_stream[i*4+2] | data_stream[i*4+3]<<8; //sample only R channel
      sample_r = static_cast<int16_t>(temp_sample_r);
      p_sample_r = abs(sample_r);
    }
    //channel_l = corr_data_l;
    //channel_r = corr_data_r;
    channel_len = len; //4096bytes
}

void sounds_level(){ // measure the sound level

   //https://learn.adafruit.com/adafruit-microphone-amplifier-breakout/measuring-sound-levels
   unsigned long startMillis = millis();  // Start of sample window
  
   //unsigned int signalMax = 0;
   //unsigned int signalMin = 4095;

   uint16_t signalMax_l = MIN_SAMPLE_LEVEL;
   uint16_t signalMin_l = MAX_SAMPLE_LEVEL;
   
   uint16_t signalMax_r = MIN_SAMPLE_LEVEL;
   uint16_t signalMin_r = MAX_SAMPLE_LEVEL;
    
   while (millis() - startMillis < sampleWindow){
      
      if (p_sample_l < MAX_SAMPLE_LEVEL || p_sample_r < MAX_SAMPLE_LEVEL)  // toss out spurious readings
      {
         if (p_sample_l > signalMax_l)
         {
            signalMax_l = p_sample_l;  // save just the max levels
         }
         else if (p_sample_l < signalMin_l)
         {
            signalMin_l = p_sample_l;  // save just the min levels
         }
         
         if (p_sample_r > signalMax_r)
         {
            signalMax_r = p_sample_r;  // save just the max levels
         }
         else if (p_sample_r < signalMin_r)
         {
            signalMin_r = p_sample_r;  // save just the min levels
         }
         
      }
   }
   //printf("pS_L:%d, pS_R:%d \n", p_sample_l, p_sample_r);
   //printf("s_MAX_L:%d, s_MIN_L:%d \n", signalMax_l, signalMin_l);
   peakToPeak_l = signalMax_l - signalMin_l;  // max - min = peak-peak amplitude
   peakToPeak_r = signalMax_r - signalMin_r;
   //printf("pToP_L:%d, pToP_R:%d \n", peakToPeak_l, peakToPeak_r);
 
}

void find_level(){ // resample the sound level of data to min-max width of lcd and leds

  level_lcd_l = map(peakToPeak_l, MIN_SAMPLE_LEVEL, MAX_SAMPLE_LEVEL, MIN_LCD_LEVEL, MAX_LCD_LEVEL);
  level_lcd_r = map(peakToPeak_r, MIN_SAMPLE_LEVEL, MAX_SAMPLE_LEVEL, MIN_LCD_LEVEL, MAX_LCD_LEVEL);
  if(level_lcd_l > MAX_LCD_RESOL) {level_lcd_l = MAX_LCD_RESOL;}
  if(level_lcd_r > MAX_LCD_RESOL) {level_lcd_r = MAX_LCD_RESOL;}

  level_led_l = map(peakToPeak_l, MIN_SAMPLE_LEVEL, MAX_SAMPLE_LEVEL, MIN_LED_LEVEL, MAX_LED_LEVEL);
  level_led_r = map(peakToPeak_r, MIN_SAMPLE_LEVEL, MAX_SAMPLE_LEVEL, MIN_LED_LEVEL, MAX_LED_LEVEL);
  if(level_led_l > MAX_LED_RESOL) {level_led_l = MAX_LED_RESOL;}
  if(level_led_r > MAX_LED_RESOL) {level_led_r = MAX_LED_RESOL;}
 
  TOP = (u8g2.getDisplayWidth() - 0);
  // Move peak up
  if (level_lcd_l > Lpeak) {
    Lpeak = min(TOP, level_lcd_l);
  }
  if (level_lcd_r > Rpeak) {
    Rpeak = min(TOP, level_lcd_r);
  }

  // Decay peak
  //EVERY_N_MILLISECONDS(60) { 
  if (millis()%4 == 0) {
      if (Lpeak > 0 ||  Rpeak > 0) {
        Lpeak -= 1; // Decay the peaks
        Rpeak -= 1; // Decay the peaks
      }
  }
  if (Lpeak <= 6) {Lpeak = 6;} 
  if (Rpeak <= 6) {Rpeak = 6;}
}

void lcd() { // show the scrolling metada and sound bar level

  //cases if some metadata is missing
  if (data_received == true) {
      if (u8g2.getUTF8Width(title)==0){
        sprintf(music, " Title: < unknow > ");
        scroll_speed = 2;
      } else if (u8g2.getUTF8Width(artist)==0){
        sprintf(music, " Title: %s  ", title);
        scroll_speed = 2;
      } else if (u8g2.getUTF8Width(album)==0) { 
        sprintf(music, " Title: %s  Artist: %s  ", title, artist);
        scroll_speed = 4;
      } else {
        sprintf(music, " Title: %s  Artist: %s  Album: %s  ", title, artist, album);
        scroll_speed = 5; 
      }     
  } else {
    sprintf(music, "%s   ", "Welcome to Smart Sound Box!");
    //width = u8g2.getUTF8Width(music);
    scroll_speed = 2;
  }
  width = u8g2.getUTF8Width(music);

  u8g2.firstPage();
   do { 
      // draw the scrolling text at current offset
      x = offset;
      do {              // repeated drawing of the scrolling text...                          
        //u8g2.clearBuffer();
        //u8g2.setCursor(x, 15); 
        //u8g2.printf("Title:%s Artist:%s Album:%s", title, artist, album);    // draw the scolling text
        u8g2.drawUTF8(x, 15, music);
        //u8g2.sendBuffer();
        x += width; // add the pixel width of the scrolling
      } while( x < u8g2.getDisplayWidth() ); // draw again until the complete display is filled
      
      u8g2.drawUTF8(0, 25, "L");
      u8g2.drawUTF8(0, 33, "R");  
      u8g2.drawBox(7,20,level_lcd_l,3); //draw the left bar
      //u8g2.drawVLine(Lpeak, 20, 3);
      u8g2.drawBox(Lpeak,20,3,3); // draw the square left peak 
      u8g2.drawBox(7,29,level_lcd_r,3); //draw the right bar
      u8g2.drawBox(Rpeak,29,3,3); //draw the square right peak
      //u8g2.print(width);         // this value must be lesser than 128 unless U8G2_16BIT is set
    } while ( u8g2.nextPage() );
    //delayMicroseconds(100);
    
    offset-=scroll_speed;              // scroll by one pixel
    if ( (u8g2_uint_t)offset < (u8g2_uint_t)-(width) ) {
    offset = 0;             // start over again
    data_received = false;
    }

    delayMicroseconds(10);
}

void leds() { // control the brightness of leds according to the sound level

  int LIn = level_led_l;
  int RIn = level_led_r;

  ledcWrite(ledChannel, LIn); //control the brightness of left led
  ledcWrite(ledChannel2, RIn); //control the brightness of right led
  //digitalWrite(ledPin3, HIGH);
  digitalWrite(ledPin3, LOW);

  if (LIn >= LED_PEAK || RIn >= LED_PEAK) { //turn on the peak led
    digitalWrite(ledPin3, HIGH);
  }
}

void setup() {
  
  Serial.begin(115200);
  
  //pinMode(12, OUTPUT);
  //pinMode(13, OUTPUT);
  pinMode(ledPin3, OUTPUT);

  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);
  ledcSetup(ledChannel2, freq, resolution);
  //ledcSetup(ledChannel3, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin, ledChannel);
  ledcAttachPin(ledPin2, ledChannel2);
  //ledcAttachPin(ledPin3, ledChannel3);
 
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_t0_15_tf);  //https://github.com/olikraus/u8g2/wiki/fntlistall //u8g2_font_t0_15_tf //u8g2_font_BBSesque_tf //u8g2_font_unifont_t_latin //u8g2_font_unifont_t_japanese3
  //u8g2.setFontRefHeightExtendedText();
  //u8g2.enableUTF8Print();   // enable UTF8 support for the Arduino print() function 
  u8g2.setFontMode(0);    // enable transparent mode, which is faster
  u8g2.setFontDirection(0);
  
  // Buildin led
  //pinMode(LED_BUILTIN, OUTPUT);
  //digitalWrite(LED_BUILTIN, LOW);

  // setup default i2s pins
  static const i2s_pin_config_t pin_config = {
      .bck_io_num = 26, //BCK
      .ws_io_num = 27, //LRCK
      .data_out_num = 25, //DIN
      .data_in_num = I2S_PIN_NO_CHANGE 
  }; 

  a2dp_sink.set_pin_config(pin_config);
  a2dp_sink.start(BtName);
  a2dp_sink.set_metadata_reader(metadata_read);
  //notified when a packet is received:
  a2dp_sink.set_on_data_received(data_received_callback);
  //access the packet:
  a2dp_sink.set_stream_reader(read_data_stream);
}

void loop() {

  unsigned long currentMillis = millis();
  //unsigned long currentMillis2 = micros();
  
  //printf("L:%d, R:%d \n", temp_sample_l, temp_sample_r);
  //printf("L:%d, R:%d \n", sample_l, sample_r);
  
  if (currentMillis - previousMillis >= interval) { // interval for call leds function.
    // save the last time you call
    previousMillis = currentMillis;
    leds();
  } 

  //unsigned long currentMillis2 = millis(); 
  if (currentMillis - previousMillis2 >= interval2) { // interval for call lcd function.
    // save the last time you call
    previousMillis2 = currentMillis;
    lcd();
  }

  //unsigned long currentMillis3 = millis(); 
  if (currentMillis - previousMillis3 >= interval3) { // interval for call level function.
    // save the last time you call
    previousMillis3 = currentMillis;
    sounds_level();
    find_level();
  }

  //delay(1);
  delayMicroseconds(10);
}
