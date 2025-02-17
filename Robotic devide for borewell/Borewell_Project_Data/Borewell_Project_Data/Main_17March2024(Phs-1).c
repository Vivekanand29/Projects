/* 
 * File:   Main.c
 * Author: 
 *
 * Created on 10 March, 2024, 01:29 PM
 */

#include "Main.h"

extern volatile U8 receiveBufferUART[], receiveBufferUART2[];
extern volatile U16 receiveCounterUART, receiveCounterUART2;

U16 oxyConc = 0, oxyFlow = 0, oxyTmp = 0, press = 0;
U8 motn = 0;

int main(int argc, char** argv)
{
    MakePinOutput(BUZZER_DIR, BUZZER);
    SetBit(BUZZER_PORT, BUZZER);
    MakePinOutput(LED1_DIR, LED1);
    MakePinOutput(LED2_DIR, LED2);
    
    
    SetBit(LED1_PORT, LED1);
    SetBit(LED2_PORT, LED2);
    SetBit(BUZZER_PORT, BUZZER);      // Buzzer ON
    _delay_ms(100);            // Delay of 100 mSEC
    ClearBit(BUZZER_PORT, BUZZER);      // Buzzer OFF
    _delay_ms(100);            // Delay of 100 mSEC
    SetBit(BUZZER_PORT, BUZZER);      // Buzzer ON
    _delay_ms(100);            // Delay of 100 mSEC
    ClearBit(BUZZER_PORT, BUZZER);      // Buzzer OFF
    _delay_ms(100);            // Delay of 100 mSEC
    ClearBit(LED1_PORT, LED1);
    ClearBit(LED2_PORT, LED2);
    
    UART1Initialise(BAUDRATE_9600);     // for ESP32 CAM
    SetCurrentDevice1(WIFI_DEVICE);
    curSerialDevice1 = WIFI_DEVICE;

    
    EnableGlobalInterrupt();
    EnablePeripheralInterrupt();
    

    InitLCD();
//    ROBOTIC DEVICE FOR BOREWELL RESCUE OPERATION 
    PrintString(0x80, "Robotic Device  ");
    PrintString(0xC0, "Borewell Rescue ");
    
    _delay_ms(200);
    ClearLCD();
    
    while(1)
    {
        if(sensCnt > 500)
        {
            sensCnt = 0;
            toggleBit(LED1_PORT, LED1);
            U32 dhtData = ReadDHT11_1();
            U16 hum = dhtData >> 24;		// Get High Byte of Humidity
			U16 tmp = (dhtData & 0xFF00) >> 8;		// Get High Byte of Temp
            
            // Normal range of TVOC 0.3 to 0.5 mg/m3
            U32 tvGas = 300;
            
            press = 213;		//Read_Pressure();
            motn = 1;
            
            ClearLCD();
            
            Uart_Tx_Str((U8 *)"\r\nSens:");
            intToAscii((char*)dispBuff, tmp ,10);
			PrintString(0x80, (U8 *)"T:");
			PrintString(0x82, dispBuff);
            Uart_Tx_Str((U8 *)"\rTemp:");
            Uart_Tx_Str(dispBuff);

			intToAscii((char*)dispBuff, hum ,10);
			PrintString(0x85, (U8 *)"H:");
			PrintString(0x87, dispBuff);
            Uart_Tx_Str((U8 *)"\rHum:");
            Uart_Tx_Str(dispBuff);
            
            intToAscii((char*)dispBuff, tvGas ,10);
			PrintString(0x8A, (U8 *)"G:");
			PrintString(0x8C, dispBuff);
            Uart_Tx_Str((U8 *)"\rGas:");
            Uart_Tx_Str(dispBuff);
			
			intToAscii((char*)dispBuff, oxyConc ,10);
			PrintString(0xC0, (U8 *)"O:");
			PrintString(0xC2, dispBuff);
            Uart_Tx_Str((U8 *)"\rOxy:");
            Uart_Tx_Str(dispBuff);

			intToAscii((char*)dispBuff, press ,10);
			PrintString(0xC5, (U8 *)"P:");
			PrintString(0xC7, dispBuff);
            Uart_Tx_Str((U8 *)"\rPres:");
            Uart_Tx_Str(dispBuff);

			intToAscii((char*)dispBuff, motn ,10);
			PrintString(0xCB, (U8 *)"M:");
			PrintString(0xCD, dispBuff);
            Uart_Tx_Str((U8 *)"\rMotn:");
            Uart_Tx_Str(dispBuff);
            
			intToAscii((char*)dispBuff, oxyFlow ,10);
            Uart_Tx_Str((U8 *)"\rFlow:");
            Uart_Tx_Str(dispBuff);
            
			intToAscii((char*)dispBuff, oxyTmp ,10);
            Uart_Tx_Str((U8 *)"\rOxyTmp:");
            Uart_Tx_Str(dispBuff);
            
            Uart_Tx_Str((U8 *)"\r\n");
            
            if(tmp > 40 || tvGas > 500 || oxyConc < 20)
            {
                toggleBit(LED2_PORT, LED2);
                toggleBit(BUZZER_PORT, BUZZER);
            }
            else
            {
                ClearBit(LED2_PORT, LED2);
                ClearBit(BUZZER_PORT, BUZZER);
            }
        }
        
        UART_Rec_Msg_Handler();

        sensCnt++;
        __delay_ms(1);
    }
    return (0);
}

void _delay_ms(U16 del)
{
    for (; del > 0; del--)
    {
        __delay_ms(1);
    }
}


U8 UART_Rec_Msg_Handler(void)
{
	U8 *strAdrr = NULL;

	if(Rec_WiFi_Data == 1)
	{
        _delay_ms(3);
        
        // For oxygen sensor AOF1010:
        strAdrr = strchr(receiveBufferUART, 0x16);
        if(strAdrr[1] == 0x09 && strAdrr[2] == 0x01)
        {
            _delay_ms(7);
            //PrintString(0x80, "Oxy Received..  ");
            oxyConc = (((U16)strAdrr[3] << 8) + strAdrr[4]) / 10;
            oxyFlow = (((U16)strAdrr[5] << 8) + strAdrr[6]);
            oxyTmp = (((U16)strAdrr[7] << 8) + strAdrr[8]) / 10;
        }
        
        receiveCounterUART = 0;
        receiveBufferUART[0] = 0;       // Add null character
		Rec_WiFi_Data = 0;
        
		return 1;
    }
    return 0;
}