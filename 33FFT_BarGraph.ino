#include <PDM.h> //to get microphone input
#include <arduinoFFT.h> //for the Fourier transform

#define SAMPLES 1024 //Must be a power of 2
#define SAMPLING_FREQUENCY 16000
#define BAR_LENGTH 80
#define DELAY 250

arduinoFFT FFT = arduinoFFT(); // make FFT object

const double deltaF = 1.0 * (SAMPLING_FREQUENCY / SAMPLES);

short sampleBuffer[SAMPLES];
volatile int samplesRead;
double vReal[SAMPLES];
double vImag[SAMPLES];
void onPDMdata(void);

void txtBarGraph(double data[SAMPLES], float min_f, float max_f, int numGrouped);

void setup() {
  Serial.begin(20000000);
  while (!Serial) {
    ; // wait for serial port to connect. 
  }
  //set up PDM
  PDM.onReceive(onPDMdata);
  PDM.setBufferSize(SAMPLES);
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }
}

void loop() {
  if (samplesRead) {
    for (int i = 0; i < SAMPLES; i++) {
      // store mic values to vReal
      vReal[i] = sampleBuffer[i];
      vImag[i] = 0;
    }
    // compute the FFT and map it to vReal
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    
    samplesRead = 0;
  }
  // output the FFT as a bar graph in Serial monitor
  txtBarGraph(vReal, 100, 800, 4);
}


// HELPER FUNCTIONS
/*
 * This Function takes in the FFT data and outputs a text bar graph in the Serial Monitor
 * The frequencies range from min_f to max_f
 * The Frequnecies associated with one bar can be grouped into varying ranges with numGrouped 
 * numGrouped = 4 will group 4 index from the FFT to one bar
 * The max amplitude for one bar is hard coded at 80000
 */
void txtBarGraph(double data[SAMPLES], float min_f, float max_f, int numGrouped) {
  delay(DELAY);                   // wait some number of milliseconds. This defines the frame rate. Default of DELAY = 250 gives 4 frames per second
  int n = max_f / deltaF;         // indices associated with max_f and min_f
  int start = min_f / deltaF;     
  Serial.print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
  
  // Loop over the desired frequencies
  for(int i = start; i < n; i+=numGrouped) {
    Serial.print(((i) * deltaF)/1);
    Serial.print(" Hz to ");
    Serial.print(((i + numGrouped) * deltaF)/1);
    Serial.print(" Hz \t");
    Serial.print("[");
    
    float value = 0;
    char output[BAR_LENGTH];

    // grouping amplitudes
    for(int j = i; j <= i + numGrouped; j++) {
      value = value + data[j];
    }
    if(value > 80000.0) {value = 80000.0;}

    // map grouped amplitude to some int from 0 to BAR_LENGTH
    int val = (value / 80000.0) * BAR_LENGTH;

    // create output string that represents the bar
    for (int j = 0; j < BAR_LENGTH; j++) {
      if(j < val){output[j] = '#';}
      else{output[j] = '.';}
    }
    
    Serial.print(output);
    Serial.println("]");
  }
}

void onPDMdata()
{
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
}
