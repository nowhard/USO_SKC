#include <ADuC847.h>
#include <stdio.H>
#include <stdlib.H>
#include <math.h>
#include "frequency.h"
#include "skc.h"
#define DEBUG  0
#define Hz  10

#define kol_izmer_data 3
#define kol_sec_sred 3
#define kol_sec_mgnov 1
#define Period_sred 200
#define Period_mgnov 100
//--------------------------------------------------------
  unsigned char code Crc8Table[256]={0x00, 0xBC, 0x01, 0xBD, 0x02, 0xBE, 0x03, 0xBF, 
 									 0x04, 0xB8, 0x05, 0xB9, 0x06, 0xBA, 0x07, 0xBB, 
									 0x08, 0xB4, 0x09, 0xB5, 0x0A, 0xB6, 0x0B, 0xB7, 
									 0x0C, 0xB0, 0x0D, 0xB1, 0x0E, 0xB2, 0x0F, 0xB3, 
									 0x10, 0xAC, 0x11, 0xAD, 0x12, 0xAE, 0x13, 0xAF, 
									 0x14, 0xA8, 0x15, 0xA9, 0x16, 0xAA, 0x17, 0xAB, 
									 0x18, 0xA4, 0x19, 0xA5, 0x1A, 0xA6, 0x1B, 0xA7, 
									 0x1C, 0xA0, 0x1D, 0xA1, 0x1E, 0xA2, 0x1F, 0xA3, 
									 0x20, 0x9C, 0x21, 0x9D, 0x22, 0x9E, 0x23, 0x9F, 
									 0x24, 0x98, 0x25, 0x99, 0x26, 0x9A, 0x27, 0x9B, 
									 0x28, 0x94, 0x29, 0x95, 0x2A, 0x96, 0x2B, 0x97, 
									 0x2C, 0x90, 0x2D, 0x91, 0x2E, 0x92, 0x2F, 0x93, 
									 0x30, 0x8C, 0x31, 0x8D, 0x32, 0x8E, 0x33, 0x8F, 
									 0x34, 0x88, 0x35, 0x89, 0x36, 0x8A, 0x37, 0x8B, 
									 0x38, 0x84, 0x39, 0x85, 0x3A, 0x86, 0x3B, 0x87, 
									 0x3C, 0x80, 0x3D, 0x81, 0x3E, 0x82, 0x3F, 0x83, 
									 0x40, 0xFC, 0x41, 0xFD, 0x42, 0xFE, 0x43, 0xFF, 
									 0x44, 0xF8, 0x45, 0xF9, 0x46, 0xFA, 0x47, 0xFB, 
									 0x48, 0xF4, 0x49, 0xF5, 0x4A, 0xF6, 0x4B, 0xF7, 
									 0x4C, 0xF0, 0x4D, 0xF1, 0x4E, 0xF2, 0x4F, 0xF3, 
									 0x50, 0xEC, 0x51, 0xED, 0x52, 0xEE, 0x53, 0xEF, 
									 0x54, 0xE8, 0x55, 0xE9, 0x56, 0xEA, 0x57, 0xEB, 
									 0x58, 0xE4, 0x59, 0xE5, 0x5A, 0xE6, 0x5B, 0xE7, 
									 0x5C, 0xE0, 0x5D, 0xE1, 0x5E, 0xE2, 0x5F, 0xE3, 
									 0x60, 0xDC, 0x61, 0xDD, 0x62, 0xDE, 0x63, 0xDF, 
									 0x64, 0xD8, 0x65, 0xD9, 0x66, 0xDA, 0x67, 0xDB, 
									 0x68, 0xD4, 0x69, 0xD5, 0x6A, 0xD6, 0x6B, 0xD7, 
									 0x6C, 0xD0, 0x6D, 0xD1, 0x6E, 0xD2, 0x6F, 0xD3, 
									 0x70, 0xCC, 0x71, 0xCD, 0x72, 0xCE, 0x73, 0xCF, 
									 0x74, 0xC8, 0x75, 0xC9, 0x76, 0xCA, 0x77, 0xCB, 
									 0x78, 0xC4, 0x79, 0xC5, 0x7A, 0xC6, 0x7B, 0xC7, 
									 0x7C, 0xC0, 0x7D, 0xC1, 0x7E, 0xC2, 0x7F, 0xC3};
//--------------------------------------------------------
sbit P3_5=P3^5;

unsigned long idata vremay_ch=0;

volatile unsigned long data temp_Hz_kanal_sred=0;
volatile unsigned long data temp_Hz_kanal_mgnov=0;

volatile unsigned int idata sec_kanal_sred=0;
volatile unsigned int idata sec_kanal_mgnov=0;
volatile unsigned int idata cycl_kanal=0;

volatile unsigned long idata Hz_data_sred=0;
volatile unsigned long idata Hz_data_mgnov=0;

volatile unsigned int xdata period_kanal_sred=0;
volatile unsigned int xdata period_kanal_mgnov=0;
volatile unsigned long xdata Hz_kanal_mgnov[kol_sec_mgnov][1000/Period_mgnov]={0};
volatile unsigned long xdata Hz_kanal_sred[kol_sec_sred][1000/Period_sred]={0};

volatile unsigned long data sym_kanal_sred=0;
volatile unsigned long data sym_kanal_mgnov=0;

unsigned char poschet_intervalov=0; 
 
volatile long data adc=0;   // временная переменная для снятия значения с регистров АЦП

#define ADC_CHN_NUM 6	 //количество каналов ацп (исправленная версия)

volatile long data adc_kanal[ADC_CHN_NUM]={0};	   // накопление значений АЦП

unsigned char data adc_kanal_gotov[ADC_CHN_NUM]={0};	 // готовность показания  АЦП канала 1

unsigned char data kol_vivod=0;

volatile unsigned char data symbol=0xFF;//принятый символ

// НЕОБХОДИМЫЕ ПЕРЕМЕННЫЕ ДЛЯ ОБМЕНА ПО ПРОТОКОЛУ +++++++++
											// количество выделяемых структур под каналы
unsigned char xdata adres_ust=0x01;							// адрес устройства
unsigned char xdata in_buffer[257]="\x00\xD7\x29\x01\x15\x10\x40\x00\x03\xD6\xCE\x48\x40\xA0\x01\x02\xD6\xCE\x48\x40\xB0\xAA";
					
unsigned char xdata bayt_statusa=0xC0;	                    // байт статуса узла
unsigned char xdata Name_ustr[]="<<__SKC__>>";			// наименование устройства не больше 20 байт
unsigned char xdata Versiya_prog[5]="\x30\x30\x30\x30\x31";					// версия программы ПЗУ	не больше 5 байт
unsigned char xdata Kol_kanalov=0;		                    // количество каналов
unsigned char xdata Primechanie[]="<-- GEOSPHERA_2008 -->";		                    // количество каналов не более 200 байт
unsigned char xdata Prinyat=0;										// количество принятых данных
unsigned char xdata Peredano=0;										// количество передаваемых данных
unsigned char xdata uart_gotov=0;								    // запрет приема во время передачи а так же сигнал о том что пришел кадр
unsigned char xdata kol_byte_prin=0;							    // сохраняет байт кадра - Длина данных
unsigned int  xdata vivedeno=0;	                                    // общая переменная, подсчитывает количество прининятых и переданных байт
unsigned char xdata Prinyato_vsego=0;
struct Kanal xdata kan[KANAL];


sbit LED2=P0^6;

//------------------------------
unsigned int Del_and_Paste_NULL(unsigned char *buffer,unsigned char kol,bit del_paste)    // kоl - это количество байт в буфере т.е. начиная с 1 а не индекс который начинается с 0
{
unsigned int xdata kol_nul=0;
unsigned char xdata i=0;
unsigned char xdata j=0;
if(del_paste==0)
 {
  for(i=0;i<kol;i++)
  {
   if(buffer[i]==0xD7)
     if(i<kol-1)
       if(buffer[i+1]!=0x29)
         {
         kol_nul++;
          for(j=kol;j>i;j--)
           {
           if(j==i+1)
            buffer[j]=0x00;
           else
            buffer[j]=buffer[j-1];
           }
          kol=kol+1;
         }

  }
 return kol_nul;
 }
 else
 {
   for(i=0;i<kol;i++)
    {
    if(buffer[i]==0xD7)
     if(i<kol-1)
       if(buffer[i+1]==0x00)
         {
         kol_nul++;
          for(j=i+1;j<kol-1;j++)
           {
           //if(j==i+1)
           // buffer[j]=0x00;
           //else
            buffer[j]=buffer[j+1];
           }
          kol=kol-1;
         }
     }
 return kol_nul;
  }
}
//-------------------------------
unsigned char CyclicControl(unsigned char *Spool,unsigned int Count)	 //  НЕОБХОДИМЫЙ КОД ДЛЯ ПРОТОКОЛА++++++++++++++++
{
     unsigned char crc = 0x0;

     while (Count--)
         crc = Crc8Table[crc ^ *Spool++];

     return crc;
}
//------------------------------ +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
unsigned char Protocol_24v(char *in_buffer,unsigned char Prinyato,unsigned char addr)// В переменную Prinyato нужно передавать количество принятых данных начиная с 1.
{
unsigned int xdata i=0;
unsigned char xdata j=0;
unsigned char xdata index=0;
unsigned char xdata max_nomer=0;
volatile float xdata tok1=0;
volatile float xdata napryajenie1=0;

if(in_buffer[0]==0x00&&in_buffer[1]==(char)(0xD7)&&in_buffer[2]==0x29) // проверка заголовка кадра
 {
 
  if(in_buffer[3]==adres_ust) // проверка адресса устройства
     {
	  
	 if(CyclicControl(in_buffer,Prinyato-1)==in_buffer[Prinyato-1])	// проверка контрольной суммы
	    {
		 switch(in_buffer[4])	// выполняем действия в зависимости от кода операции
		   {
		     case 0x01:	   // выдать полную информацию об устройстве 
			           in_buffer[3]=addr;  // адрес узла
			           in_buffer[4]=0x02;  // код операции
					   in_buffer[6]=bayt_statusa;
					   for(i=0;i<20;i++)				  // записываем наименование изделия
						   if(i<sizeof(Name_ustr)-1)
						     in_buffer[i+7]=Name_ustr[i];
						   else
						   in_buffer[i+7]=0x00;
                       for(i=0;i<5;i++)                   // записываем версию ПЗУ
					       if(i<sizeof(Versiya_prog))
						     in_buffer[i+27]=Versiya_prog[i];
					   in_buffer[32]=KANAL;		   // количество каналов
					   for(i=0;i<KANAL;i++)				   // данные по каналу
					      {
						  in_buffer[i*2+33]=((kan[i].tip)<<4)|kan[i].modifik; // байт данных
						  in_buffer[i*2+33+1]=0x00;
						  							// резерв байт
						  }
					   for(i=0;i<sizeof(Primechanie)-1;i++)					// записываем примечание
						 in_buffer[i+33+KANAL*2]=Primechanie[i];
				
					   in_buffer[5]=27+KANAL*2+sizeof(Primechanie);			// подсчет длины данных 
					   in_buffer[32+KANAL*2+sizeof(Primechanie)]=CyclicControl(in_buffer,32+KANAL*2+sizeof(Primechanie)); // подсчет контрольной суммы
					   return (32+KANAL*2+sizeof(Primechanie));	  // возвращаем длину пакета начиная с 0
					   break;
			 case 0x03:
			           // какие-либо дествия
			           in_buffer[0]=0x00;in_buffer[1]=0xD7;in_buffer[2]=0x29;
                       in_buffer[3]=addr;  // адрес узла
                       in_buffer[7]=in_buffer[4]; // код сбойной каманды
                       in_buffer[4]=0xFF;  // код операции
                       in_buffer[6]=bayt_statusa; // байт статуса узла
                       in_buffer[8]=0x00;	  // Нет ошибки (используется для подтверждения) 
                       in_buffer[5]=0x04;	  // длина данных
                       in_buffer[9]=CyclicControl(in_buffer,9);
			           return 9;
					   break;
			 case 0x05:
			            
					    break;
			 case 0x07:
			           
					   break;
			 case 0x09:
			            index=0;
					   while(index<in_buffer[5]-1)				   // данные по каналам
					      {
						  if(in_buffer[6+index]<KANAL)
						    {
							kan[in_buffer[6+index]].tip=(in_buffer[6+index+1]&0xF0)>>4;
							kan[in_buffer[6+index]].modifik=in_buffer[6+index+1]&0x0F;
							switch(in_buffer[6+index+1]&0xF0)
							{
							 case 0x10:	kan[in_buffer[6+index]].bayt_sostoyaniya1=in_buffer[6+index+2];
										index=index+1;
							}
							index=index+2;
							
							}
							else
							{
							in_buffer[0]=0x00;in_buffer[1]=0xD7;in_buffer[2]=0x29;
                            in_buffer[3]=addr;  // адрес узла
                            in_buffer[7]=in_buffer[4]; // код сбойной каманды
                            in_buffer[4]=0xFF;  // код операции
                            in_buffer[6]=bayt_statusa; // байт статуса узла
                            in_buffer[8]=0x01;	  // В запросе задан канал, не обслуживаемый измерительным модулем 
                            in_buffer[5]=0x04;	  // длина данных
                            in_buffer[9]=CyclicControl(in_buffer,9);
							return 9;
							}
						  }
					   in_buffer[0]=0x00;in_buffer[1]=0xD7;in_buffer[2]=0x29;
                       in_buffer[3]=addr;  // адрес узла
                       in_buffer[7]=in_buffer[4]; // код сбойной каманды
                       in_buffer[4]=0xFF;  // код операции
                       in_buffer[6]=bayt_statusa; // байт статуса узла
                       in_buffer[8]=0x00;	  // Нет ошибки (используется для подтверждения) 
                       in_buffer[5]=0x04;	  // длина данных
                       in_buffer[9]=CyclicControl(in_buffer,9);
					   return 9;
					   break;

					   break;
			 case 0x0A:
					   in_buffer[3]=addr;  // адрес узла

					   break;
			 case 0x0C:
					   in_buffer[3]=addr;  // адрес узла
					   break;
			 case 0x0E:
					   in_buffer[3]=addr;  // адрес узла
					   break;
			 case 0x10:
					   in_buffer[3]=addr;  // адрес узла
					   break;
			 case 0x12:
					      // сброс флага инициализации узла в байте статуса узла
					   bayt_statusa=0x40;
					   in_buffer[0]=0x00;in_buffer[1]=0xD7;in_buffer[2]=0x29;
                       in_buffer[3]=addr;  // адрес узла
                       in_buffer[7]=in_buffer[4]; // код сбойной каманды
                       in_buffer[4]=0xFF;  // код операции
                       in_buffer[6]=bayt_statusa; // байт статуса узла
                       in_buffer[8]=0x00;	  // Нет ошибки (используется для подтверждения) 
                       in_buffer[5]=0x04;	  // длина данных
                       in_buffer[9]=CyclicControl(in_buffer,9);
					   return 9;
					   break;
			 case 0x14:
			 		   index=0;
					   in_buffer[3]=addr;  // адрес узла
					   in_buffer[4]=0x15;  // код операции
					   in_buffer[6]=bayt_statusa;
					    for(i=0;i<KANAL;i++)				   // данные по каналам
					     {
						  in_buffer[index+7]=i;
						  index++;
						  in_buffer[index+7]=((kan[i].tip)<<4)|kan[i].modifik; // тип и модификация канала
						  index++;
						  switch(kan[i].tip)
						    {
							 case 0: switch(kan[i].modifik)
						                 {
										  case 0: 
										  case 1:
							        	  case 2: 
										  case 3:
										          //in_buffer[index+7]=((kan[i].dannie)&0x000000FF); // данные с АЦП
										          //index++;
												//-------------------
												  kan[i].dannie=(kan[i].dannie)+(((kan[i].dannie)&0x00000080)<<1);

												  if(kan[i].dannie>0x00FFFF00)	 //ограничение
												  	  kan[i].dannie=0x00FFFF00;
												//-------------------

												  in_buffer[index+7]=((kan[i].dannie)&0x0000FF00)>>8;
												  index++;
												 // if(kan[i].modifik==0x02|kan[i].modifik==0x03)		   // если модифи-я 2 и 3 т.е. 24-разрядные значения то 3 байта на значение 
												//  {
						    					  in_buffer[index+7]=((kan[i].dannie)&0x00FF0000)>>16;
												  index++;
						  						 // }
												  in_buffer[index+7]=kan[i].bayt_sostoyaniya1;	 // первый байт состояния канала
						                          index++;
						                          in_buffer[index+7]=kan[i].bayt_sostoyaniya2;	 // второй байт состояния канала
							                      index++;
												  break;

										  }
									  break;
							 
	
							 case 2: switch(kan[i].modifik)
						                 {	  
											  case 1:
											          in_buffer[index+7]=((kan[i].dannie)&0x000000FF); // данные с АЦП
											          index++;
													  in_buffer[index+7]=((kan[i].dannie)&0x0000FF00)>>8;
													  index++;
													  in_buffer[index+7]=kan[i].bayt_sostoyaniya1;	 // первый байт состояния канала
							                          index++;
												 break;

											 	case 0:
											          in_buffer[index+7]=((kan[i].dannie)&0x000000FF); // данные с АЦП
											          index++;
													  in_buffer[index+7]=((kan[i].dannie)&0x0000FF00)>>8;
													  index++;
													  in_buffer[index+7]=kan[i].bayt_sostoyaniya1;	 // первый байт состояния канала
							                          index++;
												break;

										  }
									  break;

							  case 1: switch(kan[i].modifik)
						                 {	  
										  case 0:
												 in_buffer[index+7]=((kan[i].dannie)&0x000000FF); // данные с АЦП
										         index++;
												 in_buffer[index+7]=((kan[i].dannie)&0x0000FF00)>>8;
												 index++;
												 in_buffer[index+7]=((kan[i].dannie)&0x00FF0000)>>16;
												 index++;
												 in_buffer[index+7]=((kan[i].dannie)&0xFF000000)>>24;
												 index++;
												 in_buffer[index+7]=kan[i].bayt_sostoyaniya1;	 // первый байт состояния канала
						                          index++;
												 break;
										  }
									   break;
							 
						     }

						   }


						  in_buffer[5]=index+2; 						 // подсчет длины данных двойка это 1(байт статуса)+1(контрольная сумма)
						  in_buffer[index+7]=CyclicControl(in_buffer,(unsigned int)(index+7)); // подсчет кс
						  return (unsigned char)(7+index);
						  break;

			 case 0x16:
					   in_buffer[3]=addr;  // адрес узла	   
					   break;
			 case 0x18:
					   in_buffer[3]=addr;  // адрес узла
					   break;
			 case 0x19:
					   in_buffer[3]=addr;  // адрес узла
					   break;
			 case 0x1A:
					   in_buffer[3]=addr;  // адрес узла
					   break;
			 case 0x1C:
					   in_buffer[3]=addr;  // адрес узла
					   break;
		   
		   	   default:in_buffer[0]=0x00;in_buffer[1]=0xD7;in_buffer[2]=0x29;
                       in_buffer[3]=addr;  // адрес узла
                       in_buffer[7]=in_buffer[4]; // код сбойной каманды
                       in_buffer[4]=0xFF;  // код операции
                       in_buffer[6]=bayt_statusa; // байт статуса узла
                       in_buffer[8]=0x06;          // Несуществующая команда
                       in_buffer[5]=0x04;	  // длина данных
                       in_buffer[9]=CyclicControl(in_buffer,9);
                       return 9;
		   			   break;
		   }

		}
		else
		{
		in_buffer[0]=0x00;in_buffer[1]=0xD7;in_buffer[2]=0x29;
        in_buffer[3]=addr;  // адрес узла
        in_buffer[7]=in_buffer[4]; // код сбойной каманды
        in_buffer[4]=0xFF;  // код операции
        in_buffer[6]=bayt_statusa; // байт статуса узла
        in_buffer[8]=0x11;	  // Неожиданный конец кадра
        in_buffer[5]=0x04;	  // длина данных
        in_buffer[9]=CyclicControl(in_buffer,9);
        return 9;
		}
	 }
	else
	return 0;
 }
 else
 {
 in_buffer[0]=0x00;in_buffer[1]=0xD7;in_buffer[2]=0x29;
 in_buffer[3]=addr;  // адрес узла
 in_buffer[7]=in_buffer[4]; // код сбойной каманды
 in_buffer[4]=0xFF;  // код операции
 in_buffer[6]=bayt_statusa; // байт статуса узла
 in_buffer[8]=0x09;	  // ошибка в структуре команды
 in_buffer[5]=0x04;	  // длина данных
 in_buffer[9]=CyclicControl(in_buffer,9);
 return 9;
 }
 return 0;											  
}
//--------------------------------------------------------- 
void _ADC_ (void) interrupt 6
{
RDY0=0;
switch(ADC0CON2&0x0F)
{
case 0x00:	// канал 1
           
           adc=ADC0H;
           adc=(((adc<<8)|ADC0M)<<8)|ADC0L;
  		   
           if(adc_kanal_gotov[0]<kol_izmer_data)
            {
	         adc_kanal[0]+=adc;
             adc_kanal_gotov[0]++;

			 if(adc_kanal_gotov[0]==kol_izmer_data)
			    ADC0CON2=((ADC0CON2&0xF0)|0x01);
			 

	        }
           break;
case 0x01:	// канал 2
           
           adc=ADC0H;
           adc=(((adc<<8)|ADC0M)<<8)|ADC0L;
  
           if(adc_kanal_gotov[1]<kol_izmer_data)
            {
	         adc_kanal[1]+=adc;
             adc_kanal_gotov[1]++;

			 if(adc_kanal_gotov[1]==kol_izmer_data)
			    ADC0CON2=((ADC0CON2&0xF0)|0x02);
			
	        }
           break;
case 0x02:	// канал 3
         
           adc=ADC0H;
           adc=(((adc<<8)|ADC0M)<<8)|ADC0L;
  
           if(adc_kanal_gotov[2]<kol_izmer_data)
            {
	         adc_kanal[2]+=adc;
             adc_kanal_gotov[2]++;

			 if(adc_kanal_gotov[2]==kol_izmer_data)
			   ADC0CON2=((ADC0CON2&0xF0)|0x03);
			
	        }
           break;
case 0x03:	// канал 4
           
           adc=ADC0H;
           adc=(((adc<<8)|ADC0M)<<8)|ADC0L;
  
           if(adc_kanal_gotov[3]<kol_izmer_data)
            {
	         adc_kanal[3]+=adc;
             adc_kanal_gotov[3]++;

			 if(adc_kanal_gotov[3]==kol_izmer_data)
			    ADC0CON2=((ADC0CON2&0xF0)|0x04);
			
	        }
           break;
case 0x04:	// канал 5
           
           adc=ADC0H;
           adc=(((adc<<8)|ADC0M)<<8)|ADC0L;
  
           if(adc_kanal_gotov[4]<kol_izmer_data)
            {
	         adc_kanal[4]+=adc;
             adc_kanal_gotov[4]++;

			 if(adc_kanal_gotov[4]==kol_izmer_data)
			    ADC0CON2=((ADC0CON2&0xF0)|0x05);
			
	        }
           break;

case 0x05:	// канал 6
           
           adc=ADC0H;
           adc=(((adc<<8)|ADC0M)<<8)|ADC0L;
  
           if(adc_kanal_gotov[5]<kol_izmer_data)
            {
	         adc_kanal[5]+=adc;
             adc_kanal_gotov[5]++;

			 if(adc_kanal_gotov[5]==kol_izmer_data)
			    ADC0CON2=((ADC0CON2&0xF0)|0x00);
			
	        }
           break;
}

}
//-----------------------------------------------------------------
void _TI_ (void) interrupt 4   // НЕОБХОДИМЫЙ КОД ДЛЯ ПРОТОКОЛА++++++++++++++++
{
static bit null=0;		 // переменная для сняти вставленного нуля 
static bit is_frame=0;//флаг-кадр а не мусор
if(TI==1)
{  
	 TI=0;
	  if(vivedeno<=Peredano)	 // передача данных
	  {
			SBUF=in_buffer[vivedeno];
			vivedeno++;    
	  }
	  else
	  {
		  P3_5=0;
		  vivedeno=0;
		  uart_gotov=0;
		  Prinyat=0;
		  RI=0;
	  }
}
else
{
 RI=0;
if(uart_gotov==0)	 // запрет приема вовремя передачи
{
//WR=0;
// if(Prinyat<6)		// общая переменная, подсчитывает количество прининятых и переданных байт
// {
//    switch(Prinyat)				 // проверка с начало ли кадр принимается
//	   {
//		case 0: if(SBUF!=(char)(0x00))
//		          {
//		          		if(SBUF==(char)(0xD7))
//						{
//							in_buffer[0]=0x0;//не зависим от нуля синхронизации
//							Prinyat++;
//							in_buffer[1]=0xD7;	
//						}
//				  		else
//						{
//							Prinyat=0;
//						}
//				  
//				  //WR=1;
//				  return;
//				  }
//				break;
//		case 1: if(SBUF!=(char)(0xD7))
//	     		  {
//		          Prinyat=0;
//				  //WR=1;
//				  return;
//				  }
//				break;
//		case 2: if(SBUF!=(char)(0x29))
//	     		  {
//		          Prinyat=0;
//				  //WR=1;
//				  return;
//				  }
//				break;
//
//	   default: break;
//	   } 
//   in_buffer[Prinyat]=SBUF;
//   Prinyat++;
//     if(Prinyat==6)
//	   {     
//        kol_byte_prin=in_buffer[Prinyat-1];  // получаем длинну данных после заголовка
//	  //if(kol_byte_prin!=(unsigned char)(0x01))
//	  // {
//	  // Prinyat=0;
//	  // WR=1;
//	  // return;
//	  // }
//	   }
// }
// else
// {
//  if(Prinyat<5+kol_byte_prin&&Prinyat<255)	  // принимаем указанное в kol_byte_prin число байт данные, 6 значит что обмен идет с компом, надо ставаить 5 чтобы обмениваться с устройствами
//   {
//	if(null==0)
//	 {
//     if(SBUF==(char)(0xD7))
//      null=1;
//     in_buffer[Prinyat]=SBUF;
//     Prinyat++;
//   	 }
//	 else				 // проверка идет ли после D7 вставленный нуль, если да то пропускаем его.
//	 {
//	  if(SBUF!=(char)(0x00))
//	   {
//	   		if(SBUF==(char)(0x29))
//			{
//				in_buffer[0]=0x0;//начало кадра-обнуляем
//				in_buffer[1]=0xD7;
//				in_buffer[2]=0x29;
//				Prinyat=3;					
//			}
//			else
//			{
//			   in_buffer[Prinyat]=SBUF;
//		       Prinyat++;
//			   null=0;				
//			}
//	   }
//	   else
//	   {
//	   null=0;
//	   }
//	 }
//   
//   }
//  else
//   {
//   //WR=1;
//   in_buffer[Prinyat]=SBUF;
//   Prinyato_vsego=Prinyat+1;
//   Prinyat=0;
//   null=0;
//   uart_gotov=1;
//   }
// }

	
//делаем анализ на начало кадра
		symbol=SBUF;
		switch(symbol)
		{
			case (char)(0xD7):
			{
				in_buffer[Prinyat]=symbol;
				Prinyat++;
				null=1;		 
			}
			break;

			case (char)(0x29):
			{
				if(null==1)
				{
					in_buffer[0]=0x0;
					in_buffer[1]=0xD7;
					in_buffer[2]=0x29;
					Prinyat=0x3;
					is_frame=1;//кадр начался
					null=0;		 	
				}
				else
				{
					in_buffer[Prinyat]=symbol;
					Prinyat++;	
				}
			}
			break;

			case (char)(0x0):
			{
 				if(null==1)	  //если после 0xD7-пропускаем
				{
					null=0;		
				}
				else
				{
					in_buffer[Prinyat]=symbol;
					Prinyat++;	
				}
			}
			break;

			default :
			{
				in_buffer[Prinyat]=symbol;
				Prinyat++;
				null=0;
				
			}
		}



	   if(Prinyat>6)
	   {
	   		  if(((Prinyat==6+kol_byte_prin) && (is_frame==1)) /*|| (Prinyat==255)*/)	  // принимаем указанное в kol_byte_prin число байт данные, 6 значит что обмен идет с компом, надо ставаить 5 чтобы обмениваться с устройствами
   			  {
				   Prinyato_vsego=Prinyat;
				   Prinyat=0;
				   null=0;
				   uart_gotov=1;
				   is_frame=0;	  			  			
			  }
			  
	   }
	   else
	   {
			   if(Prinyat==6)
			   {     
		        	kol_byte_prin=in_buffer[Prinyat-1];  // получаем длину данных после заголовка					 
			   }	   		
	   }


}
 }
}
//------------------------------
//unsigned long FloatToStrPC(void *cont)
//{
//unsigned long rezult=0;
//unsigned char *s=cont;
//rezult=(s[0]*0x1000000)|(s[1]*0x10000)|(s[2]*0x100)|s[3];
//
//return rezult;
//} 
//--------------------------------
void _TR2_ (void) interrupt 5
{
char i=0;
char j=0;
TF2=0;
 if(cycl_kanal<19)
  cycl_kanal++;
 else
  {
     poschet_intervalov++;

	 #if DEBUG==1
	    kol_vivod++;
     #endif

	 
	 temp_Hz_kanal_mgnov=((unsigned long)TH0*0x100)|TL0;
	 TH0=TL0=0;
	 temp_Hz_kanal_sred+=temp_Hz_kanal_mgnov;
	 Hz_data_mgnov+=temp_Hz_kanal_mgnov;//*=10;
	 
	 if(poschet_intervalov==2)
	 {
		 Hz_data_mgnov*=5;
		 kan[6].dannie=Hz_data_mgnov;//FloatToStrPC((void*)&Hz_data_mgnov);
		 Hz_data_mgnov=0;
	 }
	 /*if(sec_kanal_mgnov!=kol_sec_mgnov+1&&!(period_kanal_mgnov%(1000/Period_mgnov)))  
	   sec_kanal_mgnov++; 

	 if(sec_kanal_mgnov==kol_sec_mgnov+1)
	 {
	  
	  sym_kanal_mgnov-=Hz_kanal_mgnov[period_kanal_mgnov/(1000/Period_mgnov)][period_kanal_mgnov%(1000/Period_mgnov)];	
	  sym_kanal_mgnov+=temp_Hz_kanal_mgnov;
	  Hz_data_mgnov=(float)sym_kanal_mgnov/(float)kol_sec_mgnov;
	
	 }
	 else
	  sym_kanal_mgnov=sym_kanal_mgnov+temp_Hz_kanal_mgnov;

	 Hz_kanal_mgnov[period_kanal_mgnov/(1000/Period_mgnov)][period_kanal_mgnov%(1000/Period_mgnov)]=temp_Hz_kanal_mgnov;

	 if(period_kanal_mgnov<(kol_sec_mgnov*(1000/Period_mgnov))-1)	// подсчет периодов в 10 секундный интервал
	   	period_kanal_mgnov++;
	 else
	    period_kanal_mgnov=0;

	 temp_Hz_kanal_mgnov=0;*/



	if(poschet_intervalov==2) // вычисляется среднее значение частоты за 10 сек
	{
		 poschet_intervalov=0;
	
	     if(sec_kanal_sred!=kol_sec_sred+1&&!(period_kanal_sred%(1000/Period_sred)))  
		   sec_kanal_sred++;
	
		  if(sec_kanal_sred==kol_sec_sred+1)
		 {
		  sym_kanal_sred-=Hz_kanal_sred[period_kanal_sred/(1000/Period_sred)][period_kanal_sred%(1000/Period_sred)];
		  sym_kanal_sred+=temp_Hz_kanal_sred;
		  Hz_data_sred=(float)sym_kanal_sred/(float)kol_sec_sred;
		  kan[7].dannie=Hz_data_sred;//FloatToStrPC((void*)&Hz_data_sred);
		 }
		 else
		  sym_kanal_sred=sym_kanal_sred+temp_Hz_kanal_sred;
	
		 Hz_kanal_sred[period_kanal_sred/(1000/Period_sred)][period_kanal_sred%(1000/Period_sred)]=temp_Hz_kanal_sred;
	
		 if(period_kanal_sred<(kol_sec_sred*(1000/Period_sred))-1)	// подсчет периодов в 10 секундный интервал
		   	period_kanal_sred++;
		 else
		    period_kanal_sred=0;
	
		 temp_Hz_kanal_sred=0;

	}
     cycl_kanal=0;

  }
} 
//-----------------------------
/*void TIC_Crah(void) interrupt 10
{ 
flash_read(&vremay_ch,sizeof(vremay_ch),0x00000000);

 if(vremay_ch!=0)
   {
	vremay_ch-=1;
	flash_write(&vremay_ch,sizeof(vremay_ch),0x00000000);
	if(vremay_ch!=0)
     {
	  INTVAL=1;
	  TIMECON=0x2B;
	 }
   }

}
//---------------------------- */
void main (void)
{
unsigned char data i=0;
unsigned char data j=0;
//PLLCON=0x02;
PLLCON=0x01; //6MHz
// Каллибровка АЦП
ADC0CON1 = 0x27;      			// Full Buff, unipolar, 0->80mV
ADC0CON2 = 0x40;
SF       = 0x45;      			// ADC rate = 20Hz
ADCMODE  = 0x24;			// Ofs Cal
while(!CAL);
RDY0=0;
ADC0CON1 = 0x27;      			// Full Buff, unipolar, 0->80mV
ADC0CON2 = 0x40;
SF       = 0x45;      			// ADC rate = 20Hz
ADCMODE  = 0x25;
while(!CAL);
RDY0=0;	

P1=0xFF;
P3_5=0;

// КАНАЛ 1 - фиксированый АЦП
kan[0].nomer=0;		  // номера каналов должны строго идти последовательно и начинаться с нуля
kan[0].tip=0;
kan[0].modifik=0;
kan[0].bayt_sostoyaniya1=0x40;
kan[0].bayt_sostoyaniya2=0x06;
kan[0].dannie=0;
// КАНАЛ 2 - фиксированый АЦП
kan[1].nomer=1;
kan[1].tip=0;
kan[1].modifik=0;
kan[1].bayt_sostoyaniya1=0x40;
kan[1].bayt_sostoyaniya2=0x06;
kan[1].dannie=0;
// КАНАЛ 3 - фиксированый АЦП
kan[2].nomer=2;		 
kan[2].tip=0;
kan[2].modifik=0;
kan[2].bayt_sostoyaniya1=0x40;
kan[2].bayt_sostoyaniya2=0x06;
kan[2].dannie=0;
// КАНАЛ 4 - фиксированый АЦП
kan[3].nomer=3;		 
kan[3].tip=0;
kan[3].modifik=0;
kan[3].bayt_sostoyaniya1=0x40;
kan[3].bayt_sostoyaniya2=0x06;
kan[3].dannie=0;

// КАНАЛ 5 - фиксированый АЦП
kan[4].nomer=4;		 
kan[4].tip=0;
kan[4].modifik=0;
kan[4].bayt_sostoyaniya1=0x40;
kan[4].bayt_sostoyaniya2=0x06;
kan[4].dannie=0;

// КАНАЛ 6 - фиксированый АЦП
kan[5].nomer=5;		 
kan[5].tip=0;
kan[5].modifik=0;
kan[5].bayt_sostoyaniya1=0x40;
kan[5].bayt_sostoyaniya2=0x06;
kan[5].dannie=0;

// КАНАЛ 7 - частотный
kan[6].nomer=6;		 
kan[6].tip=2;
kan[6].modifik=0;
kan[6].bayt_sostoyaniya1=0x40;
kan[6].bayt_sostoyaniya2=0x06;
kan[6].dannie=0;
// КАНАЛ 8 - частотный усредненный
kan[7].nomer=7;		 
kan[7].tip=2;
kan[7].modifik=0;
kan[7].bayt_sostoyaniya1=0x40;
kan[7].bayt_sostoyaniya2=0x06;
kan[7].dannie=0;
// КАНАЛ 9 -dol
kan[8].nomer=8;		 
kan[8].tip=1;
kan[8].modifik=0;
kan[8].bayt_sostoyaniya1=0x40;
kan[8].bayt_sostoyaniya2=0x06;
kan[8].dannie=0x80000000;
// КАНАЛ 10 - частотный усредненный
kan[9].nomer=9;		 
kan[9].tip=2;
kan[9].modifik=1;
kan[9].bayt_sostoyaniya1=0x40;
kan[9].bayt_sostoyaniya2=0x06;
kan[9].dannie=0;
// КАНАЛ 11 - dol
kan[10].nomer=10;		 
kan[10].tip=1;
kan[10].modifik=0;
kan[10].bayt_sostoyaniya1=0x40;
kan[10].bayt_sostoyaniya2=0x06;
kan[10].dannie=0x80000000;
// КАНАЛ 12 - частотный усредненный
kan[11].nomer=11;		 
kan[11].tip=2;
kan[11].modifik=1;
kan[11].bayt_sostoyaniya1=0x40;
kan[11].bayt_sostoyaniya2=0x06;
kan[11].dannie=0;

// КОНФИГ АЦП
ADC0CON1=0xA7;	   //  2.5 обход буфера(0xA) mV
#if DEBUG==1
ADC0CON2=0x40;	  //  AIN1 -> AINCOM опора внешняя 2.5
#else
ADC0CON2=0x40; // 0x40
#endif
SF=32768/(Hz*8*4*kol_izmer_data);		  // 30 Гц на один канал
ADCMODE=0x2B; // 0x63 - REJ60 выключен и CHOP выключен АЦП



#if DEBUG==1
SCON=0x52;	  // Режим 1 - 8 бит, прием запрещен
T3CON=0x80;	//UART = 57600 Baud rate
T3FD=0x2D;
#else
SCON=0x50;	  // Режим 1 - 8 бит, прием запрещен
T3CON=0x82;	//UART = 57600 Baud rate
T3FD=0x2D;
ES=1; //разрешение прер-я от UART
#endif

T2CON=0x00;			  // таймер периода сбора частоты (0.01)
	RCAP2H=TH2=0x86;//0xC3;//0x86;//0x0B;	- для 6,29 МГц  //0x0A 
	RCAP2L=TL2=0x07;//0x03;//0x07;//0xF0; - для 6,29 МГц   //0x3C 

TMOD=0x05;		   // частотный канал
//TH0=TL0=0xFF;
TR0=1;
					//3,1789143880208333333333333333333e-7
EADC=1;				//0,0099307696024576822916666666666625
//EX1=1;
//EX0=1;
IT0=1;
IT1=1;
ET2=1;
ET0=1;
Frequency_Init();
//IEIP2=4;
//PADC=1;
//PX1=1;
//PX0=1;
//PT2=1;
EA=1; //разрешение всех прерываний
PS=1; //наивысший приоритет у UART
TR2=1;
 
 while(1)
   {
    //if(vremay_ch==0)
	//   while(1);

    if(adc_kanal_gotov[0]==kol_izmer_data)  // Канал 1
      {
	  kan[0].dannie=adc_kanal[0]/kol_izmer_data;
	  adc_kanal[0]=0;
	  adc_kanal_gotov[0]=0;
      }
	if(adc_kanal_gotov[1]==kol_izmer_data)  // Канал 2
      {
	  kan[1].dannie=adc_kanal[1]/kol_izmer_data;
	  adc_kanal[1]=0;
	  adc_kanal_gotov[1]=0;
      }
	if(adc_kanal_gotov[2]==kol_izmer_data)  // Канал 3
      {
	  kan[2].dannie=adc_kanal[2]/kol_izmer_data;
	  adc_kanal[2]=0;
	  adc_kanal_gotov[2]=0;
      }
	if(adc_kanal_gotov[3]==kol_izmer_data)  // Канал 4
      {
	  kan[3].dannie=adc_kanal[3]/kol_izmer_data;
	  adc_kanal[3]=0;
	  adc_kanal_gotov[3]=0;
      }
	if(adc_kanal_gotov[4]==kol_izmer_data)  // Канал 5
      {
	  kan[4].dannie=adc_kanal[4]/kol_izmer_data;
	  adc_kanal[4]=0;
	  adc_kanal_gotov[4]=0;
      }
	if(adc_kanal_gotov[5]==kol_izmer_data)  // Канал 6
      {
	  kan[5].dannie=adc_kanal[5]/kol_izmer_data;
	  adc_kanal[5]=0;
	  adc_kanal_gotov[5]=0;
      }

	 Frequency_Measure_Process();
	#if DEBUG==1
	if(kol_vivod>0)
	{ 
    printf("\rHz:%ld ",temp_Hz_kanal_mgnov);	
	printf("Hz_sr:%.1f ",Hz_data_sred);
	printf("K1:%ld ",kan[0].dannie);
	printf("K2:%ld ",kan[1].dannie);
	printf("K3:%ld ",kan[2].dannie);
	printf("K4:%ld ",kan[3].dannie);
   
	kol_vivod=0;
	}
	#else   
	 if(uart_gotov==1)		   //  НЕОБХОДИМЫЙ КОД ДЛЯ ПРОТОКОЛА++++++++++++++++
     {
	 uart_gotov=2;

     if(Peredano=Protocol_24v(in_buffer,Prinyato_vsego,0x01))	 
         {
	     Peredano=Peredano+Del_and_Paste_NULL(in_buffer,Peredano+1,0);
		 P3_5=1;
	     SBUF=in_buffer[vivedeno];
	     vivedeno++;
	     }
		 else
		 {
		 vivedeno=0;
		 uart_gotov=0;
		 }  
  
      }		
	#endif 
	  
	
	}
   

}