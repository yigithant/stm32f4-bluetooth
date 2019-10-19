#include "defines.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_hd44780.h"
#include "stm32f4xx.h" // Device header
#include "stm32f4xx_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f4xx_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f4xx_usart.h"            // Keil::Device:StdPeriph Drivers:USART

#include "stm32f4xx.h"

#define RCV_BUFFER_SIZE 40//RCV_BUFFER_SIZE  adinda bir degiskene 40 degerini atadik

void initBluetooth(void);//bluetooth un usart configlerini yaptik
void sendBluetoothChar(char item);//bilginin gönderilmesi için bir fonksiyon tanimladik


static volatile char rcvBuffer[RCV_BUFFER_SIZE];//rcvBuffer adinda bir degisken tanimladik. 
static char rcvMessage[RCV_BUFFER_SIZE];//rcvMessage adinda bir char degiskeni tanimladik. 
static volatile uint32_t rcvIndex = 0;//rcvIndex adinda bir degisken tanimladik buna 0 degerini atadik 
static volatile uint8_t rcvMessageFlag = 0;
static volatile uint8_t rcvMessageSize = 0;

 
int main(void) {
	//burda gpio config yapmamizin sebebi öbür mikroislemciden bilgi geldiginde kesme ugrayacagi için
	//kesmeye girdigini gözlemek için gpio configleri yapildi. 
	//usart kesmesi geldigi vakit ledler yaniyor.
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//ahb1 clock hattini ahb1periph_gpiod hattina enable edilmis oldu
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
 GPIO_InitTypeDef GPIO_InitStructure;
 //LED CONFIGURATION
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//sondaki pp push pull veya open drain olarak belirtilebilir.
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	initBluetooth();//bluetooth konfigürasyonlari için çagrilan fonksiyon
	
//lcd için atanan degerler
    uint8_t customChar[] = {
        0x1F,    // xxx 11111
        0x11,    // xxx 10001
        0x11,    // xxx 10001
        0x11,    // xxx 10001
        0x11,    // xxx 10001
        0x11,    // xxx 10001
        0x11,    // xxx 10001
        0x1F    // xxx 11111
    };
		
    //Initialize system
    SystemInit();
    
    //Initialize LCD 20 cols x 4 rows
    TM_HD44780_Init(20, 4);
    
    //Save custom character on location 0 in LCD
    TM_HD44780_CreateChar(0, &customChar[0]);
    
    
 
    while (1)
		{
    
    }

}



void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){return;}
uint16_t EVAL_AUDIO_GetSampleCallBack(void)
{
return -1;
}


void USART2_IRQHandler(void)//eger bluetooth un rx bacagindan kesme gelirse bu fonksiyona giriyor.
{
	
		GPIO_SetBits(GPIOD,GPIO_Pin_12);//kesme geldiginde buraya gelip gelmedigini kontrol ediyoruz.
	
	  char n;//n adinda bir tek string tanimladik
	
	  char x[13]="gonderildi\n  ";//usart üzerinden bilginin gelip gelmedigini kontrol etmek için tanimlanan char ifadesi
	
	  char y[15]="gonderiliyor\n  ";//usart üzerinden bilginin gelip gelmedigini kontrol etmek için tanimlanan char ifadesi
	
	  if(USART_GetITStatus(USART2,USART_IT_RXNE)==SET)
	  {
			
		if(USART_GetFlagStatus(USART2,USART_IT_RXNE)==SET)	
		{
		
		GPIO_SetBits(GPIOD,GPIO_Pin_13);//kesme geldiginde buraya gelip gelmedigini kontrol ediyoruz.
			
		rcvBuffer[rcvIndex]=USART_ReceiveData(USART2);//usart2 den gelen bir adet string 
		//ilk basta default olarak rcvIndex=0 olarak atadigimiz için rcvBuffer dizininde 0. diziye gelen string ifadesi buraya kaydedilir.

		if(rcvIndex+1>=RCV_BUFFER_SIZE || rcvBuffer[rcvIndex]=='\n')
		//eger rcvIndex degerinin 1 fazlasi RCV_BUFFER_SIZE degerine esit veya büyük olursa veya
		//rcvBuffer[rcvIndex] dizininde \n görürse bu bloga giriyor.
		//yani su sekilde gelen mesaj belirtilen miktardan fazla ise veya gelen mesaj son ise yani 
		//haberlesme sirasinda gönderilecek baska bir string ifade kalmadiginda en son olarak \n gönderiliyor eger bu gelmis ise bu bloga giriyor
		
		{
				
		rcvMessageFlag=1;//rcvMessageFlag degeri set ediliyor.
			
		uint32_t i;//i adinda bir degisken tanimladik
				
		uint16_t sutun;//sutun adinda bir degisken tanimladik
			
		uint16_t satir;//satir adinda bir degisken tanimladik
				
		for(i=0;i<RCV_BUFFER_SIZE;i++)//burdaki döngüde tüm rcvBuffer dizinine kaydedilen bilgiler sirayla 
		//rcvMessage dizinine kaydediliyor. 	
		{
		if(i<=rcvIndex)//i degeri rcvIndex e esit veya küçük ise bu bloga giriyor.
		{
			
		rcvMessage[i]=rcvBuffer[i];//atama burda yapiliyor
			
		}
		
		else	
		{
			
		rcvMessage[i]=0x00;//egergelen metin RCV_BUFFER_SIZE degerinden kucuk ise geriye kalan ifadelere bos degeri ataniyor
			
		}
		
		if(i==RCV_BUFFER_SIZE-1)//burda pc ye geri bilgi gönderiyoruz. 
		//gelen mesajin akibeti hakkinda bilgi gönderiliyor.
		{
		for(uint16_t i=0;i<17;i++)
		{
			
		sendBluetoothChar(y[i]);//y dizini içindeki gönderiliyor bilgisi kullaniciya geri döndürüldü
			
		}
		
		}
		
		}
		
		TM_HD44780_Clear();//daha önceden lcd ye yazdirdigimiz bilgiyi temizleyen fonksiyon
		
		for(uint16_t i=0;i<RCV_BUFFER_SIZE;i++)
		{
			
		sutun=i;//sutun degerine i degerini atadik
			
		satir=0;//satir degerine 0 degerini atadik
			
		n=rcvMessage[i];//rcvMessage dizinine kayit ettigimiz gelen bilgiyi tek tek n degerine atiyoruz
			
		if(i>15 && i<32)//eger i degeri 15den buyuk ve 32den kucuk ise bu bloga giriyor
	  {
			
		satir=1;//satir degerini 1 e esitliyoruz böylece lcd de bir alt sinira geçiyoruz
			
		sutun=i-16;//sutun degerini ise alt satira geçtigi için i-16 diyoruz böylece
		//alt satira lcdnin birinci karakterinden itibaren yazabiliyoruz
			
		}
		
		if(i==31)//i degeri 31 degerine yani lcd deki tüm degerleri doldurdugunda bu bloga giriyor
		{
			
		satir=0;//satir degerini 0 liyoruz
			
		}
		
		TM_HD44780_Puts(sutun,satir,&n);//lcd ye yazma islemi
		
		//gerekli satir ve sutuna n stringindeki tek bir ifade yaziliyor
		if(i==RCV_BUFFER_SIZE-1)//eger RCV_BUFFER_SIZE dan bir eksik degere ulasirsa bu bloga giriyor 
	  //çünkü eldeki tüm bilgi bu kadardi
		{
			
	  for(uint16_t i=0;i<15;i++)
		{
			
		sendBluetoothChar(x[i]);//ardindan kullaniciya bilginin geldigini bildiren mesaji gönderiyoruz usart2 üzerinden
							 
	  }
	  }
		//mesaja lcd ye yazilana kadar for döngüsünde döndürülüyor
	  }

		//buffer içindeki bilgi sifirlaniyor
		
		for(i=0;i<RCV_BUFFER_SIZE;i++)
		{
			
		rcvBuffer[i]=0x00;
			
		}
		
		rcvIndex=0;//rcvIndex degerimiz default olarak tekrardan 0 degerine ataniyor
		
		}
		
		else
		{
			
		rcvIndex++;//if bloguna girmez ise else bloguna giriyor ve rcvIndex degerimizi bir artiliyor
		//böylece her gelen kesmede yani bilgide rcvBuffer dizini bir artiriliyor.
		//böylece bilgi tek tek kaydedilmis oluyor
			
		}
		
		}
		
		GPIO_SetBits(GPIOD,GPIO_Pin_14);//kesme geldiginde buraya gelip gelmedigini kontrol ediyoruz.
    

    //Enable cursor blinking
    TM_HD44780_BlinkOn();
    
    //Show custom character at x = 1, y = 2 from RAM location 0
    TM_HD44780_PutCustom(1, 2, 0);
		
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
	}
}

void initBluetooth(void)
{

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);


	USART_InitTypeDef UsartStruct;
	UsartStruct.USART_BaudRate=9600;
	UsartStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	UsartStruct.USART_Mode=USART_Mode_Tx | USART_Mode_Rx;
	UsartStruct.USART_Parity=USART_Parity_No;
	UsartStruct.USART_StopBits=USART_StopBits_1;
	UsartStruct.USART_WordLength=USART_WordLength_8b;
	USART_Init(USART2,&UsartStruct);
	

	
	GPIO_InitTypeDef GPIOStruct;
	GPIOStruct.GPIO_Mode=GPIO_Mode_AF;
	GPIOStruct.GPIO_OType=GPIO_OType_PP;
	GPIOStruct.GPIO_Pin=GPIO_Pin_5 | GPIO_Pin_6;
	GPIOStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIOStruct);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
	

	NVIC_InitTypeDef NVICStruct;
	NVICStruct.NVIC_IRQChannel=USART2_IRQn;
	NVICStruct.NVIC_IRQChannelCmd=ENABLE;
	NVICStruct.NVIC_IRQChannelPreemptionPriority=1;
	NVICStruct.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVICStruct);
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
	USART_Cmd(USART2,ENABLE);
}

void sendBluetoothChar(char item)
{
	
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){}
	USART_SendData(USART2, (uint16_t)item);
}



