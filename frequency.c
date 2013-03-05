#include "frequency.h"
#include "skc.h"
#include "tic.h"

#define FREQ_CHANNELS	2


#define FRAME_TIME_1	511  //границы кадров захвата
//#define FRAME_TIME_2	1023
//#define FRAME_TIME_3	1535
//#define FRAME_TIME_4	2047
#define FRAME_TIME_MAX	((FREQ_FRAME<<9)-1)//10239 //10 секунд

#define SHEAR			9	  //рассчитывается из максимума кадра захвата, как Nбит-2

# define CLI_EXT  EX1=0; EX0=0; ET0=0;
# define STI_EXT  EX1=1; EX0=1; ET0=1;


volatile unsigned char data freq_ready=0;//измерение готово

unsigned char volatile idata measure_state=0;//автомат состояний для оптимизации процесса вычисления(уменьшение нагрузки на процессор)
//------------------------------------------
volatile struct Frequency xdata frequency[FREQ_CHANNELS] ; //структура частотных каналов
//------------------------------------------
#define FRQ_CHNL_1 0
#define FRQ_CHNL_2 1
//#define FRQ_CHNL_3 2
//--------------------------------------------------------
void Frequency_Init(void) //инициализация частотных каналов
{
//	Timer0_Initialize();
	unsigned char i=0;

	Timer1_Initialize();
	EX1=1;//включаем внешние прерывания
	EX0=1;
//	IT0=1;
//	IT1=1;
	frequency[FRQ_CHNL_1].time_counter=0;

	for(i=0;i<FREQ_FRAME;i++)
	{
		frequency[0].frame[i].event_counter=0;	
		frequency[0].frame[i].timestamp=0;
		frequency[0].frame[i].time_copy=0;
		frequency[0].frame[i].event_copy=0;
		
		frequency[1].frame[i].event_counter=0;	
		frequency[1].frame[i].timestamp=0;
		frequency[1].frame[i].time_copy=0;
		frequency[1].frame[i].event_copy=0;
	}
	TIC_Init();
	TIC_Start();

	return;
}
//--------------------------------------------------------
void INT0_ISR(void) interrupt 0 //using 3//обработчик внешнего прерывания 0
{
	unsigned char cnt;
//	EA=0;
	cnt=(frequency[FRQ_CHNL_1].time_counter>>SHEAR);
	frequency[FRQ_CHNL_1].frame[cnt].event_counter++;
	frequency[FRQ_CHNL_1].frame[cnt].timestamp=frequency[FRQ_CHNL_1].time_counter;
//	kan[8].dannie++;
//	EA=1;
	//return;
}
//--------------------------------------------------------
void INT1_ISR(void) interrupt 2 //using 3//обработчик внешнего прерывания 1
{
 	unsigned char cnt;
//	EA=0;
	cnt=(frequency[FRQ_CHNL_1].time_counter>>SHEAR);
	frequency[FRQ_CHNL_2].frame[cnt].event_counter++;
	frequency[FRQ_CHNL_2].frame[cnt].timestamp=frequency[FRQ_CHNL_1].time_counter;
	//kan[10].dannie++;
//	return;
}
//--------------------------------------------------------
//void INT2_ISR(void) interrupt 1 //using 3//обработчик внешнего прерывания 2-использует внешний вход таймера T0
//{
//	unsigned char cnt;
//		
//	TH0=0xFF;
//	TL0=0xFF;
//
//	cnt=(frequency[FRQ_CHNL_1].time_counter>>SHEAR)&0x3;
//
//	frequency[FRQ_CHNL_3].frame[cnt].event_counter++;
//	frequency[FRQ_CHNL_3].frame[cnt].timestamp=frequency[FRQ_CHNL_1].time_counter;
//	return;
//}
//--------------------------------------------------------
void Timer1_ISR(void) interrupt 3 //using 3//обработчик прерывания счетного таймера частоты
{
	volatile unsigned char data temp_index;
	
	EA=0;
	TH1=0xE7;
	TL1=0xFF;
	TF1 = 0;
	EA=1;

	if(((frequency[FRQ_CHNL_1].time_counter)&FRAME_TIME_1)==FRAME_TIME_1)//через каждые полсекунды
	{	
		switch(frequency[FRQ_CHNL_1].time_counter)
		{
			case FRAME_TIME_1:
			{
				frequency[FRQ_CHNL_1].frame[0].time_copy=frequency[FRQ_CHNL_1].frame[0].timestamp+FRAME_TIME_MAX-frequency[FRQ_CHNL_1].frame[FREQ_FRAME-1].timestamp;
				frequency[FRQ_CHNL_1].frame[0].event_copy=frequency[FRQ_CHNL_1].frame[0].event_counter;	
	
				frequency[FRQ_CHNL_1].frame[FREQ_FRAME-1].event_counter=0;

				frequency[FRQ_CHNL_2].frame[0].time_copy=frequency[FRQ_CHNL_2].frame[0].timestamp+FRAME_TIME_MAX-frequency[FRQ_CHNL_2].frame[FREQ_FRAME-1].timestamp;
				frequency[FRQ_CHNL_2].frame[0].event_copy=frequency[FRQ_CHNL_2].frame[0].event_counter;	
	
				frequency[FRQ_CHNL_2].frame[FREQ_FRAME-1].event_counter=0;
			}
			break;
			
			
			default:
			{
				temp_index=(unsigned char)(frequency[FRQ_CHNL_1].time_counter>>SHEAR);
				frequency[FRQ_CHNL_1].frame[temp_index].time_copy=frequency[FRQ_CHNL_1].frame[temp_index].timestamp-frequency[FRQ_CHNL_1].frame[temp_index-1].timestamp;
				frequency[FRQ_CHNL_1].frame[temp_index].event_copy=frequency[FRQ_CHNL_1].frame[temp_index].event_counter;	
	
				frequency[FRQ_CHNL_1].frame[temp_index-1].event_counter=0;

				frequency[FRQ_CHNL_2].frame[temp_index].time_copy=frequency[FRQ_CHNL_2].frame[temp_index].timestamp-frequency[FRQ_CHNL_2].frame[temp_index-1].timestamp;
				frequency[FRQ_CHNL_2].frame[temp_index].event_copy=frequency[FRQ_CHNL_2].frame[temp_index].event_counter;	
	
				frequency[FRQ_CHNL_2].frame[temp_index-1].event_counter=0;
			}
			break;	
		}
		freq_ready=1;//готово
	}
		
	frequency[FRQ_CHNL_1].time_counter++;
	if(frequency[FRQ_CHNL_1].time_counter>=(FRAME_TIME_MAX+1))
	{
		frequency[FRQ_CHNL_1].time_counter=0;
	}
//EA=1;
	//return;
}
//------------------------------------------------------------
unsigned char Frequency_Measure_Process(void)//циклический процесс измерения частоты
{
    volatile unsigned long data  temp_time=0;
	volatile unsigned int data  temp_freq=0;
    volatile unsigned char data i=0;
	static volatile unsigned char  frq_chnl=0;

	union 
	{ 
		unsigned long long_num; 
		unsigned int  int_num[2];
	}  volatile data temp_event;

   if(freq_ready==0)
   {
   		return 0;
   }
   	
    temp_event.long_num=0;
	temp_time=0;

   	for(i=0;i<FREQ_FRAME;i++)	//суммируем значения всех кадров
	{
		temp_event.int_num[0]+= frequency[frq_chnl].frame[i].event_copy<<2;
		temp_time+=  frequency[frq_chnl].frame[i].time_copy;		
	}


	if((temp_time!=0)&&(temp_event.long_num!=(unsigned long)0))
	{
		temp_freq=temp_event.long_num/(unsigned int)temp_time;
	}
	else
	{
		  temp_freq=0;
	}


	if(temp_freq<=0xFFFF)
	{
//		channels[8+frq_chnl].channel_data=temp_freq;
		kan[9+2*frq_chnl].dannie=temp_freq;
	}
	else
	{
///		channels[8+frq_chnl].channel_data=0xFFFF;
		kan[9+2*frq_chnl].dannie=0xFFFF;
	}


	frq_chnl++;
	if(frq_chnl>=FREQ_CHANNELS)
	{
		frq_chnl=0;
	}  
	return 0;
}
//------------------------------------------------------------
 void TIC_ISR(void) interrupt 10 	//using 1   //прерывания таймера в 1 секунду
{
	kan[8].dannie+=50;
	kan[10].dannie+=50;
	TIMECON&=0xFB;
	
} 