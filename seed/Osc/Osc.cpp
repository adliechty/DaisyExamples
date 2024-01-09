//to program, type ctrl p, then enter "task build_and_program_dfu"
#include <string.h>
#include "daisy_seed.h"
#include "daisysp.h"


#include <cmath>
#include <complex>
#include "shy_fft.h"

#include "fourier.h"
#include "wave.h"

// Use the daisy namespace to prevent having to type
// daisy:: before all libdaisy functions
using namespace daisy;
using namespace daisysp;

//////////////////////////////////
// FFT Items
//////////////////////////////////
#define PI 3.1415926535897932384626433832795
#define SR 48000
typedef float S; // sample type

// convenient lookup tables
Wave<S> hann([] (S phase) -> S { return 0.5 * (1 - cos(2 * PI * phase)); });
Wave<S> halfhann([] (S phase) -> S { return sin(PI * phase); });

const size_t bsize = 256;

bool controls_processed = false;

// 4 overlapping windows of size 2^12 = 4096
const size_t order = 12;
const size_t N = (1 << order);
const S sqrtN = sqrt(N);
const size_t laps = 4;
const size_t buffsize = 2 * laps * N;

// buffers for STFT processing
// audio --> in --(fft)--> middle --(process)--> out --(ifft)--> in -->
// each of these is a few circular buffers stacked end-to-end.
S in_pre_fft[buffsize]; // buffers for input and output (from / to user audio callback)
S middle[buffsize]; // buffers for unprocessed frequency domain data
S out_post_fft[buffsize]; // buffers for processed frequency domain data

ShyFFT<S, N, RotationPhasor>* fft; // fft object
Fourier<S, N>* stft; // stft object

// initial parameters for denoise process
// if testing without hardware control, try changing these values
// beta = mix between high- and low-energy frequency bands
// thresh = cutoff for designation of a bin as high- or low-energy
S beta = 1, thresh = 15;

// deal with analog & digital controls -- maybe update beta, thresh, ...
static void ProcessControls()
{

}

// shy_fft packs arrays as [real, real, real, ..., imag, imag, imag, ...]
inline void denoise(const S* in, S* out)
{
	// convenient constant for grabbing imaginary parts
	static const size_t offset = N / 2;

	S average = 0;
	for (size_t i = 0; i < N; i++)
	{
		out[i] = 0; 
		average += in[i] * in[i];
	}

	average /= N;

	for (size_t i = 0; i < N / 2; i++)
	{
		if ((in[i] * in[i] + in[i + offset] * in[i + offset]) < thresh * thresh * average)
		{
			// rescale the low-amplitude frequency bins by (1 - beta) ...
			out[i] = (1 - beta) * in[i];
			out[i + offset] = (1 - beta) * in[i + offset];
		}
		else
		{
			// ... and the high-amplitude ones by beta
			out[i] = beta * in[i];
			out[i + offset] = beta * in[i + offset];
		}
	}
}

/////////////////////////////////////////


// Declare a DaisySeed object called hardware
DaisySeed  hardware;
Oscillator osc[4];
AdEnv      env[4];
I2CHandle i2c;

// 5 second Audio Buffer
const int audio_buffer_length = 48000 * 2;
static float DSY_SDRAM_BSS audio_buffers[4][audio_buffer_length];
int playback_index[4] = {audio_buffer_length, audio_buffer_length, audio_buffer_length, audio_buffer_length};
int record_index[4] = {audio_buffer_length, audio_buffer_length, audio_buffer_length, audio_buffer_length};

Switch buttons[4];

const int DIO0 = 7;
const int DIO1 = 8;
const int DIO2 = 9;
const int DIO3 = 10;
const int DIO4 = 11;
const int DIO5 = 12;
const int DIO6 = 26;
const int DIO7 = 27;

const int AI0 = 15;
const int AI1 = 16;
const int AI2 = 17;
const int AI3 = 18;
const int AI4 = 19;
const int AI5 = 20;
const int AI6 = 24;
const int AI7 = 25;

const int AO0 = 22;
const int AO1 = 23;

const int DECAY_KNOB = 0;
const int VOLUME_KNOB = 1;
const int FREQ_KNOB = 3;
const int MODE_KNOB = 2;

float freq;
float frequencies[4];
int iteration = -1;
int address;

/*
void MicrophoneCallback(AudioHandle::InterleavingInputBuffer  in,
                        AudioHandle::InterleavingOutputBuffer out,
                     size_t                                size)
{
    for(size_t i = 0; i < size; i += 2)
    {
        out[i]     = out[i]   + in[i];// * 100.0;
        out[i + 1] = out[i+1] + in[i];// * 100.0;
    }
}

void Get9DofValues(int dx_accel[], int acceleration[], int rotation[], int magnetics[]){
    hardware.PrintLine("Test");
    I2CHandle::Result i2cResult = i2c.ReadDataAtAddress(0x30, 0xF, 1, data, 1, 1000);
    
 //   data[0] = 0xf;
 //   
 //   I2CHandle::Result i2cResult = i2c.TransmitBlocking(address, data, 1, 100);
 //   if (i2cResult == I2CHandle::Result::OK) {
 //       hardware.PrintLine("Result: %x", address);
 //   }
 //   address = address + 1;
 //   if (address > 0x7F) {address = 0;}
    
   dx_accel[0] = 1;
   dx_accel[1] = 0;
   dx_accel[2] = 0;

   acceleration[0] = 1;
   acceleration[1] = 0;
   acceleration[2] = 0;

   rotation[0] = 1;
   rotation[1] = 0;
   rotation[2] = 0;

   magnetics[0] = 1;
   magnetics[1] = 0;
   magnetics[2] = 0;

        
}

void 9DofSynth(AudioHandle::InterleavingInputBuffer  in,
               AudioHandle::InterleavingOutputBuffer out,
               size_t                                size)
{
    int dx_accel[3], acceleration[3], rotation[3], magnetics[3];
    button1.Debounce();
    if(button1.Pressed() {
        Get9DofValues(dx_accell, acceleration, rotation, magnetics);
        amp = (dx_accel[0] ^ 2 + dx_accel[1] ^ 2 + dx_accel[2] ^ 2) ^ 0.5;
        freq = 50 + (rotation[0] ^ 2 + rotation[1] ^ 2 + rotation[2] ^ 2) ^ 0.5;
        osc.SetFreq(mtof(freq * 127));
        osc.SetAmp(amp * 0.003);
    }

    for(size_t i = 0; i < size; i += 2)
    {
        osc_out = osc.Process();
        out[i]     = osc_out + in[i];// * 100.0;
        out[i + 1] = osc_out + in[i];// * 100.0;
    }
}
*/
void NotesMode(AudioHandle::InterleavingOutputBuffer out,
               size_t                                size,
               float knobs[4])
{
    float osc_out, env_out;
    for (int k = 0; k < 4; k += 1){
        
        if(buttons[k].RisingEdge()){ 
            env[k].Trigger();
            hardware.PrintLine("%d:  %f", k, knobs[k]);
            env[k].SetTime(ADENV_SEG_DECAY, 0.05 + 4.0 * knobs[DECAY_KNOB]);}
            if (knobs[FREQ_KNOB] < 0.25) {
                osc[k].SetFreq(frequencies[k] * 0.25);}
            else if (knobs[FREQ_KNOB] < 0.65) {
                osc[k].SetFreq(frequencies[k] * 0.5);}
            else if (knobs[FREQ_KNOB] < 0.85) {
                osc[k].SetFreq(frequencies[k] * 1.0);}
            else {
                osc[k].SetFreq(frequencies[k] * 2.0 );}
    }

    //Fill the block with samples
    for(size_t i = 0; i < size; i += 2)
    {
        out[i] = 0;
        out[i + 1] = 0;
        for (int j = 0; j < 4; j+= 1) {
            env_out = env[j].Process();
            osc[j].SetAmp(env_out * knobs[VOLUME_KNOB]);
            //osc[j].SetAmp(env_out * 0.5);
            osc_out = osc[j].Process();
            out[i]     = osc_out + out[i];// * 100.0;
            out[i + 1] = osc_out + out[i+1];// * 100.0;
        }
    }
}
void PlaybackMode(AudioHandle::InterleavingOutputBuffer out,
                  size_t                                size,
                  float knobs[4])
{
    for (int k = 0; k < 4; k += 1){
        if(buttons[k].RisingEdge()){ 
            playback_index[k] = 0;
        }
    }

    //Fill the block with samples
    for(size_t i = 0; i < size; i += 2)
    {
        out[i] = 0;
        out[i + 1] = 0;
        for (int j = 0; j < 4; j+= 1) {
            if (playback_index[j] < audio_buffer_length) {
                out[i]     = audio_buffers[j][playback_index[j]] + out[i];// * 100.0;
                out[i + 1] = audio_buffers[j][playback_index[j]] + out[i+1];// * 100.0;
                playback_index[j] = playback_index[j] + 1;
            }
        }
        out[i] = out[i] * knobs[VOLUME_KNOB] * 10.0f;
        out[i + 1] = out[i + 1] * knobs[VOLUME_KNOB] * 10.0f;
    }
}
void RecordMode(AudioHandle::InterleavingInputBuffer in,
                  size_t                              size,
                  float knobs[4])
{
    for (int k = 0; k < 4; k += 1){
        if(buttons[k].RisingEdge()){ 
            record_index[k] = 0;
        }
    }

    //Fill the block with samples
    for(size_t i = 0; i < size; i += 2)
    {
        for (int j = 0; j < 4; j+= 1) {
            if (record_index[j] < audio_buffer_length) {
                // sum left and right channel
                //if (buttons[j].Pressed()) {
                    audio_buffers[j][record_index[j]] = in[i];// + in[i + 1];// * 100.0;
                    record_index[j] = record_index[j] + 1;
                /*} else {
                    // If no longer pressed zero out the rest of the buffer
                    while (record_index[j] < audio_buffer_length){
                        audio_buffers[j][record_index[j]] = 0;
                        record_index[j] += 1;
                    }
                }*/
                
            }
        }
    }
}

void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                   AudioHandle::InterleavingOutputBuffer out,
                   size_t                                size)
{
    for(size_t i = 0; i < size; i += 1) {
        out[i] = 0;
    }

    
    float knobs[4];
    

    if (iteration != -1) {iteration += 1;}
    for (int k = 0; k < 4; k += 1){
        buttons[k].Debounce();
        knobs[k] = hardware.adc.GetFloat(k);
    }
    if (knobs[MODE_KNOB] < 0.25) {
        NotesMode(out, size, knobs);
    }
    else if (knobs[MODE_KNOB] < 0.75) {
        RecordMode(in, size, knobs);
    }
    else {
        PlaybackMode(out, size, knobs);
    }
    
    
}

void config_adcs() {
    //Create an ADC configuration
    AdcChannelConfig adcConfig[4];
    //Add pin 15 as an analog input in this config. We'll use this to read the knob
    adcConfig[0].InitSingle(hardware.GetPin(AI0));
    adcConfig[1].InitSingle(hardware.GetPin(AI1));
    adcConfig[2].InitSingle(hardware.GetPin(AI2));
    adcConfig[3].InitSingle(hardware.GetPin(AI3));
    hardware.adc.Init(adcConfig, 4);
}
void config_buttons() {
    // Set 1 kHz update rate for buttons, should be sufficient.
    
    buttons[0].Init(hardware.GetPin(DIO4), 1000);
    buttons[1].Init(hardware.GetPin(DIO5), 1000);
    buttons[2].Init(hardware.GetPin(DIO6), 1000);
    buttons[3].Init(hardware.GetPin(DIO7), 1000);
}
void zero_buffers() {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < audio_buffer_length; j ++) {
            audio_buffers[i][j] = 0;
        }
    }
}
void init_fft() {
    // initialize FFT and STFT objects
	fft = new ShyFFT<S, N, RotationPhasor>();
	fft->Init();
    // first parameter to Forier specifies the processor to manipulate the frequency domain data
	stft = new Fourier<S, N>(denoise, fft, &hann, laps, in_pre_fft, middle, out_post_fft);
}
int main(void)
{
    // Configure and Initialize the Daisy Seed
    // These are separate to allow reconfiguration of any of the internal
    // components before initialization.
    hardware.Configure();
    hardware.Init();
    hardware.SetAudioBlockSize(4);
    freq = 0;
    iteration = -1;
    address = 0;
    //How many samples we'll output per second
    float samplerate = hardware.AudioSampleRate();
    config_adcs();
    config_buttons();
    zero_buffers();
    
    
    frequencies[0] = 1046.50;  //C
    frequencies[1] = 1174.66;  //D
    frequencies[2] = 1318.51;  //E
    frequencies[3] = 1567.98;  //G

    //Set up oscillator
    for (int i = 0; i < 4; i += 1){
        osc[i].Init(samplerate);
        osc[i].SetWaveform(osc[i].WAVE_SIN);
        osc[i].SetAmp(1.f);
        osc[i].SetFreq(frequencies[i]);

        //Set up volume envelope
        env[i].Init(samplerate);
        //Envelope attack and decay times
        env[i].SetTime(ADENV_SEG_ATTACK, .01);
        env[i].SetTime(ADENV_SEG_DECAY, .4);
        //minimum and maximum envelope values
        env[i].SetMin(0.0);
        env[i].SetMax(1.f);
        env[i].SetCurve(0); // linear
    }
    // Debugging over USB UART
    hardware.StartLog(false);
    /*
    // I2C
    I2CHandle::Config i2c_conf;    
    i2c_conf.periph = I2CHandle::Config::Peripheral::I2C_1;
    i2c_conf.speed  = I2CHandle::Config::Speed::I2C_100KHZ;
    i2c_conf.mode   = I2CHandle::Config::Mode::I2C_MASTER;
    i2c_conf.pin_config.scl  = {DSY_GPIOB, 8};  // Daisy Pin 12
    i2c_conf.pin_config.sda  = {DSY_GPIOB, 9};  // Daisy Pin 13
    // initialise the peripheral
    i2c.Init(i2c_conf);
    */
    //Start the adc
    hardware.adc.Start();

    //Start calling the audio callback
    hardware.StartAudio(AudioCallback);

    // Loop forever
    for(;;) {}
}
