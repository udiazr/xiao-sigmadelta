# xiao-sigmadelta
Class-D sigma-delta Arduino IDE Seeduino XIAO amplifier. 

This program reads PCM_S16_LE from USBSerial communication port
and modulates PWM using 2nd order sigma-delta 8bit modulator with OSR of 8 (356khz PWM)
The complementary PWM output is on pins A2 and A3. You will need a half-bridge driver to 
drive a pair of speakers. I use IXYS 2A Dual Low Side Gate Driver (IXDN602).

It also includes a digital BASS-BOOSTER modulated with a potentiometer that is need to be attached to pin A0.
The BASS-BOOSTER is a lowpass 2order IIR digital filter that is added to the main signal.

check this video to listen to it:
https://www.youtube.com/watch?v=L-1Woupe-yI

I use the alsa-arduino14.py python script to get the data from
GNU/Linux ALSA loopback driver. 
If you use Pulseaudio,  you need to load it via terminal:
pactl load-module module-loopback
or add it at the end of file as default in your /etc/pulse/default.pa file:
load-module module-loopback


It is mono channel only. Soon I will upload the stereo version but I need to do some ASM trickery.




