  #include <Adafruit_ZeroDMA.h>
#define SAMPLE_BLOCK_LENGTH 128

#define OVER 8


/*
 array([[1., 0., 0.]])
 1.0, 
 array([[ 2.2734375 ,  0.078125  ,  0.34765625], 
 [-0.42578125,  0.30859375, -0.8515625 ], 
 [-3.67578125,  0.07421875,  0.328125  ]]), 
 array([[ 2.90625   , -2.90625   ],
 [-0.        ,  0.        ],
 [-3.07421875,  3.07421875]])
*/
int16_t adc_sample_block[SAMPLE_BLOCK_LENGTH];
volatile uint8_t *active_adc_buffer,*inactive_adc_buffer,*inactive_sub_buffer,*active_sub_buffer;
uint8_t adc_buffer[SAMPLE_BLOCK_LENGTH * 4* OVER];
uint8_t sub_buffer[SAMPLE_BLOCK_LENGTH * 4];

volatile bool adc_buffer_filled = false,filling_first_half = true;

extern "C" void do_sampleado(int32_t *data);

Adafruit_ZeroDMA ADC_DMA;
Adafruit_ZeroDMA ADC_DMA1;
Adafruit_ZeroDMA ADC_DMA2;
Adafruit_ZeroDMA ADC_DMA3;

DmacDescriptor *dmac_per;
DmacDescriptor *dmac_descriptors1[2];
DmacDescriptor *dmac_descriptors[2];
DmacDescriptor *dmac_descriptors2[2];
DmacDescriptor *dmac_descriptors3[2];

//uint32_t period=1089>>BOVER;
//uint32_t period=1089;
//uint32_t period=2048;
//uint32_t period=1024;
//uint32_t period=512;
uint32_t period=2047/OVER;
//uint32_t period=136;
//uint32_t period=273;

void dma_callback(Adafruit_ZeroDMA *dma) {
  (void) dma;
  if (filling_first_half) {
    // DMA is filling the first half of the buffer, use data from the second half
    active_adc_buffer = &adc_buffer[SAMPLE_BLOCK_LENGTH*2*OVER];
    inactive_adc_buffer=&adc_buffer[0];
    inactive_sub_buffer=&sub_buffer[0];
    active_sub_buffer=&sub_buffer[SAMPLE_BLOCK_LENGTH*2];

    filling_first_half = false;
  } else {
    // DMA is filling the second half of the buffer, use data from the first half
    active_adc_buffer = &adc_buffer[0];
    inactive_adc_buffer=&adc_buffer[SAMPLE_BLOCK_LENGTH*2*OVER];
    inactive_sub_buffer=&sub_buffer[SAMPLE_BLOCK_LENGTH*2];
    active_sub_buffer=&sub_buffer[0];

    filling_first_half = true;
  }
  adc_buffer_filled = true;
}


void dma_callback1(Adafruit_ZeroDMA *dma) {
  ;
}


void dma_init(Adafruit_ZeroDMA *ADC_DMA,uint8_t * adc_buffer,DmacDescriptor **dmac_descriptors,void * peripheral,void (*dma_callback) (Adafruit_ZeroDMA *dma),int trigger, uint8_t salto,uint32_t block_salto,dma_beat_size beat_size,uint32_t samples){
  ADC_DMA->allocate();
  ADC_DMA->setTrigger(trigger);
  ADC_DMA->setAction(DMA_TRIGGER_ACTON_BEAT);

  dmac_descriptors[0] = ADC_DMA->addDescriptor(
           adc_buffer,
           peripheral,
           samples,
          // (SAMPLE_BLOCK_LENGTH*OVER)>>1,
           beat_size,
           true,
           false,salto,1);
  dmac_descriptors[0]->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_INT;

  dmac_descriptors[1] = ADC_DMA->addDescriptor(
           adc_buffer + block_salto ,peripheral,
           samples,
          // (SAMPLE_BLOCK_LENGTH*OVER)>>1,
           beat_size,
           true,
           false,salto,1);
  dmac_descriptors[1]->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_INT;
  
  ADC_DMA->loop(true);
  ADC_DMA->setCallback(dma_callback);
}

int32_t data[32];
int32_t A[32]={582,20,89,744,-744,   -109,79,-218 ,0,0, -941,19,84,-787,787, 0,0,0,0, 0,0,0,0,0,0};

void setup() {
 Serial.begin(0); 
 init_tcc0();
 memset(data,0,28*4);
 memset(adc_buffer,0x00,SAMPLE_BLOCK_LENGTH*OVER*2);
 memset(sub_buffer,0x00,SAMPLE_BLOCK_LENGTH*4);
 for(int i=0;i<SAMPLE_BLOCK_LENGTH;i++)
 adc_sample_block[i]=0x0000;
 dma_init(&ADC_DMA1,adc_buffer+2,dmac_descriptors1,(void *)(&TCC0->CCB[0].reg),dma_callback1,TCC0_DMAC_ID_OVF,1,2*SAMPLE_BLOCK_LENGTH*OVER,DMA_BEAT_SIZE_HWORD,(SAMPLE_BLOCK_LENGTH*OVER)>>1);
 dma_init(&ADC_DMA, adc_buffer,dmac_descriptors, (void *)(&TCC0->CCB[1].reg),dma_callback,TCC0_DMAC_ID_OVF,1,2*SAMPLE_BLOCK_LENGTH*OVER,DMA_BEAT_SIZE_HWORD,(SAMPLE_BLOCK_LENGTH*OVER)>>1);

 
 dma_init(&ADC_DMA2,sub_buffer+2,dmac_descriptors2,(void *)(&TCC1->CCB[1].reg),dma_callback1,TCC1_DMAC_ID_OVF,1,SAMPLE_BLOCK_LENGTH*2,DMA_BEAT_SIZE_HWORD,SAMPLE_BLOCK_LENGTH>>1);
 dma_init(&ADC_DMA3,sub_buffer,dmac_descriptors3, (void *)(&TCC1->CCB[0].reg),dma_callback1,TCC1_DMAC_ID_OVF,1,SAMPLE_BLOCK_LENGTH*2,DMA_BEAT_SIZE_HWORD,SAMPLE_BLOCK_LENGTH>>1);


 ADC_DMA2.startJob();  
 ADC_DMA3.startJob();
 ADC_DMA1.startJob();  
 ADC_DMA.startJob();
 
 //dmac_descriptors[0]->BTCNT.reg=1;
 //dmac_descriptors[1]->BTCNT.reg=1;
 
// dmac_descriptors1[0]->BTCNT.reg=1;
 //dmac_descriptors1[1]->BTCNT.reg=1;
 
 //dmac_descriptors1[0]->BTCNT.reg=dmac_descriptors[0]->BTCNT.reg;
 //dmac_descriptors1[1]->BTCNT.reg=dmac_descriptors[1]->BTCNT.reg;
 
 dmac_descriptors2[0]->BTCNT.reg=dmac_descriptors3[0]->BTCNT.reg;
 dmac_descriptors2[1]->BTCNT.reg=dmac_descriptors3[1]->BTCNT.reg;
}


volatile int32_t res0=0,res1=0,res2=0,alpha,alphac,beta=7,betac=249,y00=0,y11=0;
volatile bool positivo=0;


void loop() {
  if(adc_buffer_filled){
    adc_buffer_filled=false;
    uint16_t *buffer=(uint16_t *)inactive_adc_buffer;
    int32_t *sub_buf=(int32_t *)inactive_sub_buffer;
    /*A[23]=(int32_t)adc_sample_block;
    A[24]=(int32_t)buffer;
    A[25]=SAMPLE_BLOCK_LENGTH;
    A[26]=0xffffff00;
    A[27]=0x7f00;*/
    
    //do_sampleado(A);
    int32_t x0=0,x1=0,x2=0,v0,u0,out0,i=0,x3=0,x4=0,x5=0;
    ///{582,20,89,744,-744,   -109,79,-218 ,0,0, -941,19,84,-787,787,
    
    for(int k=0;k<SAMPLE_BLOCK_LENGTH;k+=2){
      u0=((adc_sample_block[k]+adc_sample_block[k+1])*243)>>9;
      y00=(betac*y00+beta*u0)>>8;
      y11=(betac*y11+beta*y00)>>8;
      u0=(alphac*u0+alpha*y11)>>8;
    for(int j=0;j<OVER;j++){
    x0=x2;
    v0=u0+x0;
    v0=(v0+0x80)&0xffffff00;
    out0=(v0>>8)+0x7f;
    buffer[i]=out0,buffer[i+1]=out0,i+=2;
    x1=x3;
    x2= (395*x0+ 53*x1+499*u0-499*v0)>>8;
    x3= (-459*x0+104*x1-256*u0+256*v0)>>8;
    /*
    A [[ 395.   53.]
      [-459.  104.]]
B    [[ 499. -499.]
      [-256.  256.]]*/
    /*x0=x3;
    v0=u0+x0;
    v0=(v0+0x80)&0xffffff00;
    out0=(v0>>8)+0x7f;
    buffer[i]=out0,buffer[i+1]=out0,i+=2;
    x1=x4,x2=x5;
    x3=( 582*x0+20*x1+89* x2+744*u0-744*v0)>>8;
    x4=(-109*x0+79*x1-218*x2)>>8;
    x5=(-941*x0+19*x1+84* x2-787*u0+787*v0)>>8;*/
    }}
    /*A[18]=(sa0+A[15]+0x80)&0xffffff00;
    out0=(A[18]+0x7f00)>>8;
    buffer[i]=out0,buffer[i+1]=out0,i+=2;
    A[15]=(A[0]* A[19]+A[1]* A[20]+A[2]* A[21]+A[3]* sa0+A[4]* A[22])>>16;
    A[16]=(A[5]* A[19]+A[6]* A[20]+A[7]* A[21])>>16;
    A[17]=(A[10]*A[19]+A[11]*A[20]+A[12]*A[21]+A[13]*sa0+A[14]*A[22])>>16;*/
   /*for(int k=0;k<SAMPLE_BLOCK_LENGTH;k++){
      sa0=adc_sample_block[k];
      k++;
      sa1=adc_sample_block[k];
      bass0=(2*betac*bass0+beta*(sa0+sa1))>>9;
      bass1=(betac*bass1+beta*bass0)>>8;
      bass2=(betac*bass2+beta*bass1)>>8;
      sa0=(alphac*bass2+alpha*sa0)>>8;
      sa0+=0x8000;
      for(int j=0;j<OVER;j++){
        dif0+=sa0-out0,dif00+=dif0-out0,out0=dif00&0xff00;
        val=out0>>8;
        buffer[i0]=val,buffer[i0+1]=val,i0+=2;
      }
      //val=(sa0+sa1)>>6;
      //sub_buf[ii]=val,sub_buf[ii+1],ii+=2;
    /*  
     *   
      base0=sa0>>8;
      res0=sa0&0xff;
      base1=sa1>>8;
      res1=sa1&0xff;


      
      
      bass=(betac*2*bass+beta*(sa1+sa0))>>9;
      bass1=(betac*bass1+beta*bass)>>8;
      bass2=(betac*bass2+beta*bass1)>>8;
      
      if(bass2>=0x8000)
        base2=(bass2-0x8000)>>4;
      else
        base2=((0x8000-bass2)<<12)&0xffff0000;
       sub_buf[ii]=base2,ii++;
       
       
      for(int j=0;j<OVER;j++){
        //sub_buf[ii]=base2,ii++;
           if(dif0>0x0)  
          buffer[i0]=base0+1,dif0-=(0x100-res0),i0+=2;
        else  
          buffer[i0]=base0,dif0+=res0,dif00+=dif0,i0+=2;
      }
      for(int j=0;j<OVER;j++){
        //sub_buf[ii]=base2,ii++;
        if(dif1>0x0)
          buffer[i1]=base1+1,dif1-=(0x100-res1),i1+=2;
        else  
          buffer[i1]=base1,dif1+=res1,i1+=2;
      }
/*
      for(int j=0;j<OVER;j++){
        //sub_buf[ii]=base2,ii++;
           if(dif0>0)  
          buffer[i]=base0+1,dif0-=(0x100-res0),i++;
        else  
          buffer[i]=base0,dif0+=res0,i++;
        if(dif1>0)
          buffer[i]=base1+1,dif1-=(0x100-res1),i++;
        else  
          buffer[i]=base1,dif1+=res1,i++;
      }*/

        /*if(((dif0*2)+dif00)>0)  
          buffer[i]=base0+1,dif0-=(0x100-res0),dif00+=dif0,i++;
        else  
          buffer[i]=base0,dif0+=res0,dif00+=dif0,i++;
        if(((dif1*2)+dif11)>0)
          buffer[i]=base1+1,dif1-=(0x100-res1),dif11+=dif1,i++;
        else  
          buffer[i]=base1,dif1+=res1,dif11+=dif1,i++;*/
      
      
 //   }
    Serial.readBytes((char*)adc_sample_block,SAMPLE_BLOCK_LENGTH*2); 
    alpha=(4*alpha+analogRead(7))>>3;
    alphac=256-alpha;
  }

}



void init_tcc0(){
  // Because we are using TCC0, limit period to 24 bits
  period = ( period < 0x00ffffff ) ? period : 0x00ffffff;
  SYSCTRL->DPLLCTRLB.reg |= SYSCTRL_DPLLCTRLB_REFCLK_REF0;    // Select the external 32.768kHz crystal as the clock source
  //SYSCTRL->DPLLCTRLB.reg |= SYSCTRL_DPLLCTRLB_REFCLK_REF1;    // Select the internal 32.768kHz oscillator as the clock source
  
  SYSCTRL->DPLLRATIO.reg = SYSCTRL_DPLLRATIO_LDRFRAC(4) |    // Generate a 32MHz DPLL clock source from the external 32.768kHz crystal
  //                         SYSCTRL_DPLLRATIO_LDR(2756);       // Frequency = 32.768kHz * (975 + 1 + 9/16) = 32MHz
  SYSCTRL_DPLLRATIO_LDR(2755);       // Frequency = 32.768kHz * (975 + 1 + 9/16) = 32MHz
 //                          SYSCTRL_DPLLRATIO_LDR(2929);       // Frequency = 32.768kHz * (975 + 1 + 9/16) = 32MHz
  
  SYSCTRL->DPLLCTRLA.reg = SYSCTRL_DPLLCTRLA_ENABLE;          // Enable the Digital Phase Locked Loop (DPLL)
  while (!SYSCTRL->DPLLSTATUS.bit.LOCK);                      // Wait for the DPLL to achieve lock

  GCLK->GENCTRL.reg = GCLK_GENCTRL_OE |           // Enable GCLK output 
                      GCLK_GENCTRL_IDC |          // Enable 50% duty cycle
                      GCLK_GENCTRL_GENEN |        // Enable GCLK
                      GCLK_GENCTRL_SRC_FDPLL |    // Set the clock source to FDPLL96M
                      GCLK_GENCTRL_ID(4); 
  while (GCLK->STATUS.bit.SYNCBUSY);   // Wait for synchronization  

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |        // Enable generic clock
                      GCLK_CLKCTRL_GEN_GCLK4 |    // Select GCLK4
                      GCLK_CLKCTRL_ID_TCC0_TCC1;                   
  // Divide counter by 1 giving 48 MHz (20.83 ns) on each TCC0 tick
  TCC0->CTRLA.reg |= TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV1_Val);
  TCC1->CTRLA.reg |= TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV1_Val);

  //TCC1->CTRLA.reg |= TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV1_Val);

  // Use "Normal PWM" (single-slope PWM): count up to PER, match on CC[n]
  //TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
  //TCC0->WAVE.reg = 
  uint32_t wave_form=TCC_WAVE_WAVEGEN_NPWM;
  //uint32_t wave_form=TCC_WAVE_WAVEGEN_DSBOTH;
  TCC0->WAVEB.reg = wave_form|TCC_WAVE_POL1 ; //| TCC_WAVE_POL2 ;// Select NPWM as waveform
  TCC1->WAVEB.reg = wave_form|TCC_WAVE_POL1; //| TCC_WAVE_POL2 ;// Select NPWM as waveform
  //uint32_t wave_form=TCC_WAVE_WAVEGEN_DSBOTH;
  //TCC0->WAVEB.reg = wave_form |TCC_WAVE_POL3|TCC_WAVE_POL2|TCC_WAVE_POL1|TCC_WAVE_POL0;// Select NPWM as waveform
  
  while (TCC0->SYNCBUSY.bit.WAVE);                // Wait for synchronization
  while (TCC1->SYNCBUSY.bit.WAVE);                // Wait for synchronization

  // Set the period (the number to count to (TOP) before resetting timer)
  //TCC1->PER.reg = period;
  TCC0->PER.reg = period;
  TCC1->PER.reg = period*OVER+OVER-1;
  while (TCC0->SYNCBUSY.bit.PER);
  while (TCC1->SYNCBUSY.bit.PER);

  // Set PWM signal to output 50% duty cycle
  // n for CC[n] is determined by n = x % 4 where x is from WO[x]
  //TCC1->CC[2].reg=period>>1;
  
  TCC0->CC[2].reg=period>>2;
  TCC0->CC[3].reg=period>>2;
  //TCC0->CC[1].reg=period>>2;
  //TCC0->CC[0].reg=period>>2;
  TCC1->CC[1].reg=period;
  TCC1->CC[0].reg=period;

  while (TCC0->SYNCBUSY.bit.CC2);

  // Configure PA18 (D10 on Arduino Zero) to be output
  PORT->Group[PORTA].DIRSET.reg = PORT_PA10;      // Set pin as output
  PORT->Group[PORTA].OUTCLR.reg = PORT_PA10;      // Set pin to low
  
  PORT->Group[PORTA].DIRSET.reg = PORT_PA11;      // Set pin as output
  PORT->Group[PORTA].OUTCLR.reg = PORT_PA11;      // Set pin to low

  PORT->Group[PORTA].DIRSET.reg = PORT_PA08;      // Set pin as output
  PORT->Group[PORTA].OUTCLR.reg = PORT_PA08;      // Set pin to low
  PORT->Group[PORTA].DIRSET.reg = PORT_PA09;      // Set pin as output
  PORT->Group[PORTA].OUTCLR.reg = PORT_PA09;      // Set pin to low
 
 
  // Enable the port multiplexer for PA18
  PORT->Group[PORTA].PINCFG[10].reg |= PORT_PINCFG_PMUXEN;
  PORT->Group[PORTA].PINCFG[11].reg |= PORT_PINCFG_PMUXEN;
  PORT->Group[PORTA].PINCFG[8].reg |= PORT_PINCFG_PMUXEN;
  PORT->Group[PORTA].PINCFG[9].reg |= PORT_PINCFG_PMUXEN;
  
  // Connect TCC0 timer to PA18. Function F is TCC0/WO[2] for PA18.
  // Odd pin num (2*n + 1): use PMUXO
  // Even pin num (2*n): use PMUXE
  
  PORT->Group[PORTA].PMUX[5].reg  = PORT_PMUX_PMUXE_E;
  PORT->Group[PORTA].PMUX[5].reg |= PORT_PMUX_PMUXO_E;
  //PORT->Group[PORTA].PMUX[5].reg  = PORT_PMUX_PMUXE_F;
  //PORT->Group[PORTA].PMUX[5].reg |= PORT_PMUX_PMUXO_F;
  //PORT->Group[PORTA].PMUX[4].reg  = PORT_PMUX_PMUXE_E;
  //PORT->Group[PORTA].PMUX[4].reg |= PORT_PMUX_PMUXO_E;
  PORT->Group[PORTA].PMUX[4].reg  = PORT_PMUX_PMUXE_E;
  PORT->Group[PORTA].PMUX[4].reg |= PORT_PMUX_PMUXO_E;

 // Enable output (start PWM)
  //TCC1->CTRLA.reg |= (TCC_CTRLA_ENABLE);
  TCC0->CTRLA.reg |= (TCC_CTRLA_ENABLE);
  TCC1->CTRLA.reg |= (TCC_CTRLA_ENABLE);
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
  TCC1->COUNT.reg=0;
  TCC0->COUNT.reg=0;
  
}
