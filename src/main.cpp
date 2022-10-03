#include <Arduino.h>
#include <FastLED.h>

#define NUM_LEDS 144
#define DATA_PIN 7

CRGB leds[NUM_LEDS];

//Functions
void setPixel(int Pixel, int red, int green, int blue);
void clearLedPanel();
void initVars();

double h0;    // m
double v;     // m/s, current velocity
double g;     // m/s^2, gravitiy
double t;     // starting time
double dt;    // time step between frames, default: dt = 0.001
double rho;   // coefficient of restitution
double tau;   // contact time for bounce, default: tau = 0.10
double hmax;  // maximum height
double h;
double hstop;   // stop when bounce is less than x m
bool freefall;  // state: freefall or in contact
double t_last;  // time we would have launched to get to h0 at t=0
double vmax;
int steps;

int cycle_delay = 0; // msec

int rgbcolor[3] = {255,0,0};


void setup() {
  // init serial interface
	Serial.begin(115200);

  // init FastLED
	pinMode(DATA_PIN, OUTPUT);
	FastLED.addLeds<WS2812,DATA_PIN, RGB>(leds, NUM_LEDS);

}

void loop() {
  // light 1st LED & wait
  clearLedPanel();
  setPixel(NUM_LEDS-1,rgbcolor[0],rgbcolor[1],rgbcolor[2]);
  setPixel(NUM_LEDS-2,rgbcolor[0],rgbcolor[1],rgbcolor[2]);
	FastLED.show();
  // init vars
  initVars();

  delay(2000);

  double hnew = 0;
  // calc ball position
  while(hmax > hstop){
    if(freefall){

      hnew = h + v*dt - 0.5*g*dt*dt;

      if(hnew<0.0){
        t = t_last + 2.0 * sqrt(2.0 * hmax / g);
        freefall = false;
        t_last = t + tau;
        h = 0.0;
      }else{
        t = t + dt;
        v = v - g*dt;
        h = hnew;
      }
        
    }else{
      t = t + tau;
      vmax = vmax * rho;
      v = vmax;
      freefall = true;
      h = 0.0;
    }
    
  hmax = 0.5*vmax*vmax/g;

  // y axis (height) -> h
  // x axis (time) -> t
  // map y axis to num of leds
  int led_num = map(h*100, 0.0, h0*100, 0, NUM_LEDS-1);

  // clear strip
  clearLedPanel();

  // set actual ball pos to strip
  setPixel(led_num, rgbcolor[0], rgbcolor[1], rgbcolor[2]);
  setPixel(led_num+1, rgbcolor[0], rgbcolor[1], rgbcolor[2]);
  FastLED.show();

  // wait for next cycle
  if(cycle_delay>0) delay(cycle_delay);

  steps++;
  }
  
  Serial.println("Stopped bouncing after " + String(steps) + " steps.");
  Serial.println("At time t = " + String(t) + " sec.");
  Serial.println("");

}

void setPixel(int Pixel, int red, int green, int blue) {
	// FastLED
	leds[Pixel].g = red;
	leds[Pixel].r = green;
	leds[Pixel].b = blue;
}

void clearLedPanel(){
	fill_solid(leds, NUM_LEDS, CRGB::Black);
	FastLED.show();
}

void initVars(){
  // initialize all variables
  h0 = 5.0;
  v = 0.0;
  g = 9.81;
  t = 0.0;
  dt = 0.005;
  rho = 0.75;
  tau = 0.05;
  hmax = h0;
  h = h0;
  hstop = 0.01;
  freefall = true;
  t_last = -sqrt(2.0 * h0 / g);
  vmax = sqrt(2.0 * hmax * g);
  steps = 0;
}