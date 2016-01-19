

#include "compiler.h"
#include "intc.h"
#include "gpio.h"
#include "pwm.h"
#include "adc.h"
#include "board.h"

#define PWM_PER 128
#define PWM_DUR 127
unsigned int channel_id;
int counter=0;

#if __GNUC__
	__attribute__((__interrupt__))
#elif __ICCAVR32__
__interrupt
#endif

static void ADC_int_handler(void)
{		
		int LED1int;
		int light,pot;

		
//CONFIRM PB0 PUSH WITH LED5 FLASH
if (gpio_get_pin_value(GPIO_PUSH_BUTTON_0) == 1){
	gpio_set_gpio_pin(LED5_GPIO);}
else
	{
	gpio_clr_gpio_pin(LED5_GPIO);	
	counter++;

	
	}
//PB0 PUSH->COUNTER=COUNTER+1
//IF COUNTER==2 THEN LED0=TEMP*0.2POT
if (counter==2){ 

volatile avr32_pwm_t *pwm = &AVR32_PWM;
volatile avr32_adc_t *adc = &AVR32_ADC;

pot=adc_get_value(adc,1);
light=adc_get_value(adc,2);

LED1int=PWM_DUR*(0.2*pot)*(500/light)/1024;
pwm->channel[4].cupd=LED1int;}
//IF COUNTER IS 0 THEN LED0 OPEN
else if (counter==0){
	volatile avr32_pwm_t *pwm = &AVR32_PWM;
	volatile avr32_adc_t *adc = &AVR32_ADC;

	pot=adc_get_value(adc,1);
	light=adc_get_value(adc,2);

	LED1int=PWM_DUR*1024/1024;
	pwm->channel[4].cupd=LED1int;
}
//ELSE LED0 OFF
else{

	volatile avr32_pwm_t *pwm = &AVR32_PWM;
	volatile avr32_adc_t *adc = &AVR32_ADC;

	pot=adc_get_value(adc,1);
	light=adc_get_value(adc,2);

	LED1int=PWM_DUR*(0*pot)*light/1024;
	pwm->channel[4].cupd=LED1int;

}
gpio_clear_pin_interrupt_flag(GPIO_PUSH_BUTTON_0);
}


int main(void)
{
pwm_opt_t pwm_opt;        
avr32_pwm_channel_t pwm_channel;		
extern unsigned int channel_id;
	 
Disable_global_interrupt();
INTC_init_interrupts();
		
gpio_enable_module_pin(AVR32_PWM_4_1_PIN, AVR32_PWM_4_1_FUNCTION);

gpio_enable_module_pin(AVR32_ADC_AD_1_PIN, AVR32_ADC_AD_1_FUNCTION);
gpio_enable_module_pin(AVR32_ADC_AD_2_PIN, AVR32_ADC_AD_2_FUNCTION);

pwm_opt.diva = AVR32_PWM_DIVA_CLK_OFF;
pwm_opt.divb = AVR32_PWM_DIVB_CLK_OFF;
pwm_opt.prea = AVR32_PWM_PREA_MCK;
pwm_opt.preb = AVR32_PWM_PREB_MCK;
pwm_init(&pwm_opt);
	  

//CHANNEL SETTINGS
channel_id = 0;
pwm_channel.CMR.calg = PWM_MODE_LEFT_ALIGNED;       
pwm_channel.CMR.cpol = PWM_POLARITY_LOW;            
pwm_channel.CMR.cpd = PWM_UPDATE_DUTY;   
//SAMPLE RATE
pwm_channel.CMR.cpre = AVR32_PWM_CPRE_MCK_DIV_1024;    
pwm_channel.cdty = PWM_DUR;
pwm_channel.cprd = PWM_PER;  
pwm_channel.cupd = 0;   
pwm_channel_init(channel_id, &pwm_channel); 
pwm_start_channels(1 << channel_id); 

channel_id = 4;
pwm_channel.CMR.calg = PWM_MODE_LEFT_ALIGNED;       
pwm_channel.CMR.cpol = PWM_POLARITY_LOW;            
pwm_channel.CMR.cpd = PWM_UPDATE_DUTY;              
pwm_channel.CMR.cpre = AVR32_PWM_CPRE_MCK_DIV_1024;    
pwm_channel.cdty = PWM_DUR; 
pwm_channel.cprd = PWM_PER;
pwm_channel.cupd = 0;   
pwm_channel_init(channel_id, &pwm_channel); 
pwm_start_channels(1 << channel_id);  

volatile avr32_adc_t *adc = &AVR32_ADC;
adc->mr |= 0xF << AVR32_ADC_SHTIM_OFFSET;
adc->mr |= 0x1F << AVR32_ADC_STARTUP_OFFSET;
adc->mr |= 0x3F << AVR32_ADC_PRESCAL_OFFSET;
adc->ier |= 1 << AVR32_ADC_IER_EOC2_OFFSET;
adc->cher = 7; 
INTC_register_interrupt(&ADC_int_handler,AVR32_ADC_IRQ, AVR32_INTC_INT0);     
Enable_global_interrupt();
//INF LOOP FORCE COUNTER TO RESET WHEN COUNTER>2
int i;
while (1)			
	{
	for (i = 0; i < 500; i += 1)
	    	adc_start(adc);	
	if (counter==2)
			gpio_tgl_gpio_pin(LED1_GPIO);
	if (counter>2)
				counter=0;
			}
}
