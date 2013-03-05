#ifndef SKC_H
#define SKC_H
struct Kanal 
{
unsigned char nomer;	  // номер канала 
unsigned char tip;		  // тип канала
unsigned char modifik;	  // модификатор канала
unsigned char bayt_sostoyaniya1;	// байт состояния канала
unsigned long dannie;		  // значение канала
unsigned char bayt_sostoyaniya2; 
};
#define KANAL 12



#endif