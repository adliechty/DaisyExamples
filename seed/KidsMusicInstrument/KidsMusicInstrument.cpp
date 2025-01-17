/*
This code is a modification of OscPocketD/VASynth, created by Staffan Melin, staffan.melin@oscillator.se.
It was later modified by (Steven @moonfriendsynth) to work with the Daisy Pod.
I (Christopher @Nettech15) have modified it further to run on the Daisy Seed without the use of the Daisy Pod board.
I (Tony Liechty) have modified it further to be a stand alone instrument using GPIOs and analog inputs to sensors.
Feel free to copy, modify, and improve this code to match your equipment and sound requirements.
*/

#include "daisy_seed.h"
#include "daisysp.h"

#include "main.h"
#include "vasynth.h"

using namespace daisy;
using namespace daisysp;

// globals
DaisySeed hardware;
MidiUsbHandler midi;
float sysSampleRate;
float sysCallbackRate;
extern uint8_t preset_max;
extern VASynthSetting preset_setting[PRESET_MAX];

uint8_t param_;
float pitch_bend = 1.0f;
float master_tune = 0.0f;

// sound
VASynth vasynth;
uint8_t gPlay = PLAY_ON;

void SequencerPlay(uint16_t);
void SequencerRecord(uint8_t, uint8_t);
void writeSram(uint32_t, uint8_t);
uint8_t readSram(uint32_t);
void writeSramWord(uint32_t, uint16_t);
uint16_t readSramWord(uint32_t);

uint16_t seqclock = 0, seqtime = 0, seqmode = 0, seqmsg;
uint16_t seqnote, seqvelocity;
uint32_t seqmem = 0x00010000;

int NUM_KEYS = 16;
int GPIO_KEY_BASE = 8;

// audio callback

void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                   AudioHandle::InterleavingOutputBuffer out,
                   size_t                                size)
{	
	float voice_left, voice_right;
	
	for (size_t n = 0; n < size; n += 2)
	{	
		if (gPlay == PLAY_ON)
		{			
			vasynth.Process(&voice_left, &voice_right);
			
			out[n] = voice_left + in[n];
			out[n + 1] = voice_right + in[n + 1];	
		} 
		else 
		{
			out[n] = 0;
			out[n + 1] = 0;		
		}		
	}
}

// midi handler
void HandleKeyPresses(int key_index, bool new_state)
{
	// Key assumed to be off unless found to be on
	MidiMessageType message_type = NoteOff;
		// Just supporting NOteOn and NoteOff for now.
	if (key_index >= GPIO_KEY_BASE && key_index < GPIO_KEY_BASE + NUM_KEYS && new_state) message_type = NoteOn;
	uint8_t note = key_index - GPIO_KEY_BASE + 128;
	uint8_t velocity = 255;
		
    switch(message_type)
    {
        case NoteOn:
        {
			if(seqmode == 1) SequencerRecord(note, velocity);
			vasynth.NoteOn(note, velocity);
			hardware.SetLed(true);
	        break;
        }
        case NoteOff:
        {
			if(seqmode == 1) SequencerRecord(note, 0);
			vasynth.NoteOff(note);
			hardware.SetLed(false);
	        break;
        } 
		/*
		case PitchBend:
        {
            PitchBendEvent p = m.AsPitchBend();
            if ((vasynth.midi_channel_ == MIDI_CHANNEL_ALL) || (p.channel == vasynth.midi_channel_))
			{
				vasynth.PitchBend(p.value);
			}			
	        break;
        } 
        case ControlChange:
        {
            ControlChangeEvent p = m.AsControlChange();
            switch(p.control_number)
            {
				case 1: // Modulation Wheel
					vasynth.lfo_amp_ = ((float)p.value / 127.0f);
                    vasynth.SetLFO();
                    break;
				case 7: // Data Slider Default (Volume)
					switch(param_)
					{
						case 0: // This is set as the default parameter
						{
							master_tune = 1.0f - ((float)p.value / 64.0f);
							break;
						}
						case 1:
						{
							vasynth.waveform_ = p.value >> 4;
							vasynth.SetWaveform();
							break;
						}
						case 2:
						{
							vasynth.osc2_waveform_ = p.value >> 4;
							vasynth.SetWaveform();
							break;
						}
						case 3:
						{
							vasynth.osc_mix_ = ((float)p.value / 127.0f);
							break;
						}
						case 4:
						{
							vasynth.osc2_detune_ = ((float)p.value / 255.0f);
							break;
						}
						case 5:
						{
							vasynth.osc2_transpose_ = (1.0f + ((float)p.value / 127.0f));
							break;
						}
						case 6:
						{
							vasynth.filter_res_ = ((float)p.value / 127.0f);
                    		vasynth.SetFilter();
							break;
						}
						case 7:
						{
							vasynth.osc_pw_ = ((float)p.value / 255.0f);
							break;
						}
						case 8:
						{
							vasynth.osc2_pw_ = ((float)p.value / 255.0f);
							break;
						}
						case 9:
						{
							vasynth.eg_f_attack_ = ((float)p.value / 127.0f);
							vasynth.SetEG();
							break;
						}
						case 10:
						{
							vasynth.eg_f_decay_ = ((float)p.value / 127.0f);
							vasynth.SetEG();
							break;
						}
						case 11:
						{
							vasynth.eg_f_sustain_ = ((float)p.value / 127.0f);
							vasynth.SetEG();
							break;
						}
						case 12:
						{
							vasynth.eg_f_release_ = ((float)p.value / 127.0f);
							vasynth.SetEG();
							break;
						}
						case 13:
						{
							vasynth.eg_a_attack_ = ((float)p.value / 127.0f);
							vasynth.SetEG();
							break;
						}
						case 14:
						{
							vasynth.eg_a_decay_ = ((float)p.value / 127.0f);
							vasynth.SetEG();
							break;
						}
						case 15:
						{
							vasynth.eg_a_sustain_ = ((float)p.value / 127.0f);
							vasynth.SetEG();
							break;
						}
						case 16:
						{
							vasynth.eg_a_release_ = ((float)p.value / 127.0f);
							vasynth.SetEG();
							break;
						}
						case 17:
						{
							vasynth.vcf_kbd_follow_= ((float)p.value / 127.0f);
							break;
						}
						case 18:
						{
							vasynth.env_kbd_follow_ = ((float)p.value / 127.0f);
							break;
						}
						case 19:
						{
							vasynth.vel_select_ = p.value >> 5;
							break;
						}
						case 20:
						{
							vasynth.eg_f_amount_ = ((float)p.value / 127.0f);
							break;
						}
						case 21:
						{
							vasynth.lfo_freq_ = ((float)p.value / 127.0f);
							vasynth.SetLFO();
							break;
						}
						case 22:
						{
							vasynth.pwmlfo_freq_ = ((float)p.value / 127.0f);
							vasynth.SetPWMLFO();
							break;
						}
						case 23:
						{
							vasynth.pwmlfo_amp_ = ((float)p.value / 511.0f);
							vasynth.SetPWMLFO();
							break;
						}
						case 24:
						{
							vasynth.pwm2lfo_freq_ = ((float)p.value / 127.0f);
							vasynth.SetPWM2LFO();
							break;
						}
						case 25:
						{
							vasynth.pwm2lfo_amp_ = ((float)p.value / 511.0f);
							vasynth.SetPWM2LFO();
							break;
						}
						case 26:
						{
							vasynth.vcavcflfo_freq_ = ((float)p.value / 127.0f);
							vasynth.SetVCAVCFLFO();
							break;
						}
						case 27:
						{
							vasynth.vcavcflfo_amp_ = ((float)p.value / 127.0f);
							vasynth.SetVCAVCFLFO();
							break;
						}
						case 28:
						{
							vasynth.vcavcflfo_waveform_ = p.value >> 4;
							vasynth.SetVCAVCFLFO();
							break;
						}
					}
					break;
                default: break;
            }
            break;
        }
		case ProgramChange:
        {
            ProgramChangeEvent p = m.AsProgramChange();
            if ((vasynth.midi_channel_ == MIDI_CHANNEL_ALL) || (p.channel == vasynth.midi_channel_))
			{
				if(p.program >= 29)
				{
					switch (p.program)
					{
						case 35:
						{
							// Sequencer Mode Record
							seqmode = 1;
							seqclock = 0;
							break;
						}
						case 36:
						{
							// Sequencer Mode Record End
							seqmode = 2;
							// Place a time-stamp and stop marker at the current position and stop the sequencer
							writeSramWord(seqmem, seqclock);
							seqmem++;
							seqmem++;
							writeSramWord(seqmem, 0xFFFF);
							seqmode = 0;
							seqclock = 0;
							seqmem = 0x00010000;
							break;
						}
						case 37:
						{
							// Sequencer Mode Stop
							seqmode = 0;
							seqclock = 0;
							seqmem = 0x00010000;
							break;
						}
						case 38:
						{
							// Sequencer Mode Play
							seqmode = 3;
							seqclock = 0;
							seqmem = 0x00010000;
							break;
						}
						case 39:
						{
							vasynth.FlashLoad(0);
							break;
						}
						case 40:
						{
							vasynth.FlashLoad(1);
							break;
						}
						case 41:
						{
							vasynth.FlashLoad(2);
							break;
						}
						case 42:
						{
							vasynth.FlashLoad(3);
							break;
						}
						case 43:
						{
							vasynth.FlashLoad(4);
							break;
						}
						case 44:
						{
							vasynth.FlashLoad(5);
							break;
						}
						case 45:
						{
							vasynth.FlashLoad(6);
							break;
						}
						case 46:
						{
							vasynth.FlashLoad(7);
							break;
						}
						case 47:
						{
							vasynth.FlashLoad(8);
							break;
						}
						case 48:
						{
							vasynth.FlashLoad(9);
							break;
						}
						case 49:
						{
							vasynth.FlashSave(0);
							break;
						}
						case 50:
						{
							vasynth.FlashSave(1);
							break;
						}
						case 51:
						{
							vasynth.FlashSave(2);
							break;
						}
						case 52:
						{
							vasynth.FlashSave(3);
							break;
						}
						case 53:
						{
							vasynth.FlashSave(4);
							break;
						}
						case 54:
						{
							vasynth.FlashSave(5);
							break;
						}
						case 55:
						{
							vasynth.FlashSave(6);
							break;
						}
						case 56:
						{
							vasynth.FlashSave(7);
							break;
						}
						case 57:
						{
							vasynth.FlashSave(8);
							break;
						}
						case 58:
						{
							vasynth.FlashSave(9);
							break;
						}
					}
				}	
				else
				{
					vasynth.ProgramChange(p.program);
					break;				
				}
			}    
        }
		*/
        default: break;
    }
}

void SequencerPlay(uint16_t modenum)
{
	if(modenum == 0)
	{
		seqclock = 0;
		seqmem = 0x00010000;
		return;
	}
	
	if(modenum == 3)
	{
		/* Read the time-stamp from SRAM and compare to seqclock*/
		seqtime = readSramWord(seqmem);
		
		while(seqtime == seqclock)
		{
			/* Move pointer to note data */
			seqmem++;
			seqmem++;
			
			/* Read the note from SRAM */
			seqnote = readSram(seqmem);
			seqmsg = seqnote & 0x80;
			seqnote = seqnote & 0x7F;
			seqmem++;
		
			/* Read the velocity from SRAM */
			seqvelocity = readSram(seqmem);
			seqmem++;
			
			if(seqvelocity == 0xFF)
			{
				/* End of sequence reached, reset the position to the beginning and restart */
				seqclock = 0;
				seqmem = 0x00010000;
				return;
			}
			
			if(seqmsg)
			{
				/* Play the note */
				vasynth.NoteOn(seqnote, seqvelocity);
			}
			else
			{
				/* Stop the note */
				vasynth.NoteOff(seqnote);
			}
			
			/* Check to see if there are other events at the current sequencer clock */
			seqtime = readSramWord(seqmem);
		}
	}
	
	/* All events processed so increment the sequencer clock */
	seqclock++;
}

void SequencerRecord(uint8_t recnote, uint8_t recvelocity)
{
	/* Write the timestamp to SRAM */
	writeSramWord(seqmem, seqclock);
	seqmem++;
	seqmem++;
		
	/* Write the note to SRAM */
	writeSram(seqmem, recnote);
	seqmem++;
		
	/* Write the velocity to SRAM */
	writeSram(seqmem, recvelocity);
	seqmem++;
	
	if(seqmem > 0x7FFF8)
	{
		/* There is no more memory left for the sequencer to use */
		/* Place a timestamp and stop marker at the current position and select stop mode */
		writeSramWord(seqmem, seqclock);
		seqmem++;
		seqmem++;
		writeSramWord(seqmem, 0xFFFF);
		seqmode = 0;
		seqclock = 0;
		seqmem = 0x00010000;
	}
}

/* SRAM memory handling routines */
void writeSram(uint32_t l_addr, uint8_t l_data)
{
  /* Pointer write on specific location of backup SRAM */
  *(volatile uint8_t *) (WRITE_READ_SRAM_ADDR + l_addr) = l_data;
}

uint8_t readSram(uint32_t l_addr)
{
  uint8_t i_retval;
	
  /* Pointer write from specific location of backup SRAM */
  i_retval =  *(volatile uint8_t *) (WRITE_READ_SRAM_ADDR + l_addr);
	
  return i_retval;
}

void writeSramWord(uint32_t l_addr_w, uint16_t l_data_w)
{
  /* Pointer write on specific location of backup SRAM */
  *(volatile uint16_t *) (WRITE_READ_SRAM_ADDR + l_addr_w) = l_data_w;
}

uint16_t readSramWord(uint32_t l_addr_w)
{
  uint16_t i_retval_w;
	
  /* Pointer write from specific location of backup SRAM */
  i_retval_w =  *(volatile uint16_t *) (WRITE_READ_SRAM_ADDR + l_addr_w);
	
  return i_retval_w;
}

/* Process sequencer requests */
void Callback(void* data)
{
	SequencerPlay(seqmode);
}

int main(void)
{
	// init hardware
	hardware.Init(true); // true = boost to 480MHz

	sysSampleRate = hardware.AudioSampleRate();
	sysCallbackRate = hardware.AudioCallbackRate();

	// init qspi flash for saving and loading patches
	QSPIHandle::Config qspi_config;
	qspi_config.device = QSPIHandle::Config::Device::IS25LP064A;
	qspi_config.mode   = QSPIHandle::Config::Mode::MEMORY_MAPPED;
	qspi_config.pin_config.io0 = {DSY_GPIOF, 8};
	qspi_config.pin_config.io1 = {DSY_GPIOF, 9};
	qspi_config.pin_config.io2 = {DSY_GPIOF, 7};
	qspi_config.pin_config.io3 = {DSY_GPIOF, 6};
	qspi_config.pin_config.clk = {DSY_GPIOF, 10};
	qspi_config.pin_config.ncs = {DSY_GPIOG, 6};
	hardware.qspi.Init(qspi_config);

	// setup vasynth initial values
	vasynth.Init();

	// load the default patch
	vasynth.First();

	// let everything settle
	System::Delay(100);
	
	// Start calling the audio callback
	hardware.StartAudio(AudioCallback);

	/* Create Timer Handle and Config for Playback of audio in QSPI*/
    TimerHandle         tim5;
    TimerHandle::Config tim_cfg;

    /** TIM5 with IRQ enabled */
    tim_cfg.periph     = TimerHandle::Config::Peripheral::TIM_5;
    tim_cfg.enable_irq = true;

    /** Configure frequency (240Hz) */
    auto tim_target_freq = 240;
    auto tim_base_freq   = System::GetPClk2Freq();
    tim_cfg.period       = tim_base_freq / tim_target_freq;

    /** Initialize timer */
    tim5.Init(tim_cfg);
    tim5.SetCallback(Callback);

    /** Start the timer, and generate callbacks at the end of each period */
    tim5.Start();

	Switch keys[NUM_KEYS - 1];
	
	for (int i = 0; i < NUM_KEYS; i++) {
		//PULL UP enabled by default
		keys[i].Init(hardware.GetPin(i + GPIO_KEY_BASE));
	}

	// Loop forever
	for(;;)
	{
		for (int i = 0; i < NUM_KEYS; i++) {
			keys[i].Debounce();
			if (keys[i].FallingEdge()) HandleKeyPresses(i, false);
			if (keys[i].RisingEdge())  HandleKeyPresses(i, true);
			
		}
	}
}
