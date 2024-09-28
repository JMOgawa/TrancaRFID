extern sfr sbit MFRC522_CS;
extern sfr sbit MFRC522_Rst;
extern sfr sbit MFRC522_CS_Direction;
extern sfr sbit MFRC522_Rst_Direction;
//MF522 palavra de comando
#define PCD_IDLE              0x00               // sem ação; cancelar o comando atual
#define PCD_AUTHENT    0x0E             // Chave de Autenticação
#define PCD_RECEIVE     0x08           // receber dados
#define PCD_TRANSMIT    0x04          // transmitir dados
#define PCD_TRANSCEIVE    0x0C        // transmite e recebe dados,
#define PCD_RESETPHASE     0x0F        // Reset
#define PCD_CALCCRC 0x03                   // CRC Calcular
// Palavra de comando do cartão Mifare_One
#define PICC_REQIDL 0x26 // encontra a área da antena não entra em hibernação
#define PICC_REQALL 0x52 // encontra toda a área da antena de cartões
#define PICC_ANTICOLL 0x93 // anticolisão
#define PICC_SElECTTAG 0x93 // cartão de eleição
#define PICC_AUTHENT1A 0x60 // chave de autenticação A
#define PICC_AUTHENT1B 0x61 // chave de autenticação B
#define PICC_READ 0x30 // Bloco de Leitura
#define PICC_WRITE 0xA0 // bloco de escrita
#define PICC_DECREMENT 0xC0 // debit
#define PICC_INCREMENT 0xC1 // recarregar
#define PICC_RESTORE 0xC2 // transfere dados de bloco para o buffer
#define PICC_TRANSFER 0xB0 // salva os dados no buffer
#define PICC_HALT 0x50 // Sleep

// E MF522 o código de erro é retornado quando a comunicação
#define MI_OK                 0
#define MI_NOTAGERR           1
#define MI_ERR                2

//------------------MFRC522 Register---------------
//Page 0: Command and Status
#define     RESERVED00            0x00    
#define     COMMANDREG            0x01    
#define     COMMIENREG            0x02    
#define     DIVLENREG             0x03    
#define     COMMIRQREG            0x04    
#define     DIVIRQREG             0x05
#define     ERRORREG              0x06    
#define     STATUS1REG            0x07    
#define     STATUS2REG            0x08    
#define     FIFODATAREG           0x09
#define     FIFOLEVELREG          0x0A
#define     WATERLEVELREG         0x0B
#define     CONTROLREG            0x0C
#define     BITFRAMINGREG         0x0D
#define     COLLREG               0x0E
#define     RESERVED01            0x0F
//PAGE 1: Comando   
#define     RESERVED10            0x10
#define     MODEREG               0x11
#define     TXMODEREG             0x12
#define     RXMODEREG             0x13
#define     TXCONTROLREG          0x14
#define     TXAUTOREG             0x15
#define     TXSELREG              0x16
#define     RXSELREG              0x17
#define     RXTHRESHOLDREG        0x18
#define     DEMODREG              0x19
#define     RESERVED11            0x1A
#define     RESERVED12            0x1B
#define     MIFAREREG             0x1C
#define     RESERVED13            0x1D
#define     RESERVED14            0x1E
#define     SERIALSPEEDREG        0x1F
//PAGE 2: CFG    
#define     RESERVED20            0x20  
#define     CRCRESULTREGM         0x21
#define     CRCRESULTREGL         0x22
#define     RESERVED21            0x23
#define     MODWIDTHREG           0x24
#define     RESERVED22            0x25
#define     RFCFGREG              0x26
#define     GSNREG                0x27
#define     CWGSPREG              0x28
#define     MODGSPREG             0x29
#define     TMODEREG              0x2A
#define     TPRESCALERREG         0x2B
#define     TRELOADREGH           0x2C
#define     TRELOADREGL           0x2D
#define     TCOUNTERVALUEREGH     0x2E
#define     TCOUNTERVALUEREGL     0x2F
//PAGE 3: registro de teste     
#define     RESERVED30            0x30
#define     TESTSEL1REG           0x31
#define     TESTSEL2REG           0x32
#define     TESTPINENREG          0x33
#define     TESTPINVALUEREG       0x34
#define     TESTBUSREG            0x35
#define     AUTOTESTREG           0x36
#define     VERSIONREG            0x37
#define     ANALOGTESTREG         0x38
#define     TESTDAC1REG           0x39  
#define     TESTDAC2REG           0x3A   
#define     TESTADCREG            0x3B   
#define     RESERVED31            0x3C   
#define     RESERVED32            0x3D   
#define     RESERVED33            0x3E   
#define     RESERVED34            0x3F

static void MFRC522_Wr( char addr, char value )
{
        MFRC522_CS = 0;
        Soft_SPI_Write( ( addr << 1 ) & 0x7E );
        Soft_SPI_Write( value );
        MFRC522_CS = 1;
}
static char MFRC522_Rd( char addr )
{
char value;
        MFRC522_CS = 0;
        Soft_SPI_Write( (( addr << 1 ) & 0x7E) | 0x80 );
        value = Soft_SPI_Read( 0x00 );
        MFRC522_CS = 1;
        return value;
}
static void MFRC522_Clear_Bit( char addr, char mask )
{
     MFRC522_Wr( addr, MFRC522_Rd( addr ) & (~mask) );
}
static void MFRC522_Set_Bit( char addr, char mask )
{
     MFRC522_Wr( addr, MFRC522_Rd( addr ) | mask );
}
void MFRC522_Reset()
{
        MFRC522_Wr( COMMANDREG, PCD_RESETPHASE );
}
void MFRC522_AntennaOn()
{
 MFRC522_Set_Bit( TXCONTROLREG, 0x03 );
}
void MFRC522_AntennaOff()
{
 MFRC522_Clear_Bit( TXCONTROLREG, 0x03 );
}
void MFRC522_Init()
{
     MFRC522_CS_Direction = 0;
     MFRC522_Rst_Direction = 0;
     MFRC522_CS = 1;
     MFRC522_Rst = 1;        
     MFRC522_Reset();       
     MFRC522_Wr( TMODEREG, 0x8D );      //Tauto=1; f(Timer) = 6.78MHz/TPreScaler
     MFRC522_Wr( TPRESCALERREG, 0x3E ); //TModeReg[3..0] + TPrescalerReg
     MFRC522_Wr( TRELOADREGL, 30 );           
     MFRC522_Wr( TRELOADREGH, 0 );
        
     MFRC522_Wr( TXAUTOREG, 0x40 );    //100%ASK
     MFRC522_Wr( MODEREG, 0x3D );      // CRC valor inicial de 0x6363
        
     //MFRC522_Clear_Bit( STATUS2REG, 0x08 );//MFCrypto1On=0
     //MFRC522_Wr( RXSELREG, 0x86 );      //RxWait = RxSelReg[5..0]
     //MFRC522_Wr( RFCFGREG, 0x7F );     //RxGain = 48dB

     MFRC522_AntennaOn();
}

char MFRC522_ToCard( char command, char *sendData, char sendLen, char *backData, unsigned *backLen )
{
  char _status = MI_ERR;
  char irqEn = 0x00;
  char waitIRq = 0x00;
  char lastBits;
  char n;
  unsigned i;
  
  switch (command)
  {
    case PCD_AUTHENT:       // Cartões de certificação fechar
    {
      irqEn = 0x12;
      waitIRq = 0x10;
      break;
    }
    case PCD_TRANSCEIVE:    // Transmitir dados FIFO
    {
      irqEn = 0x77;
      waitIRq = 0x30;
      break;
    }
    default:
      break;
  }

  MFRC522_Wr( COMMIENREG, irqEn | 0x80 );  // solicitação de interrupção, 
  MFRC522_Clear_Bit( COMMIRQREG, 0x80 );   // Limpar todo o bit de solicitação de interrupção
  MFRC522_Set_Bit( FIFOLEVELREG, 0x80 );   //FlushBuffer=1, inicialização FIFO
  MFRC522_Wr( COMMANDREG, PCD_IDLE );      // Nenhuma ação; Cancelar o comando atual ???     
  // Escrevendo dados para o FIFO
  for ( i=0; i < sendLen; i++ )
  {
    MFRC522_Wr( FIFODATAREG, sendData[i] );
  }

  // Execute o comando
  MFRC522_Wr( COMMANDREG, command );
  if (command == PCD_TRANSCEIVE )
  {
    MFRC522_Set_Bit( BITFRAMINGREG, 0x80 ); //StartSend=1, transmissão de dados começa
 }
  // Esperando para receber dados para concluir
  // i de acordo com o ajuste de frequência do relógio, o cartão de operador M1 tempo máximo de espera 25ms ???
  i = 0xFFFF;   
  do
  {
    //CommIrqReg[7..0]
    //Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
    n = MFRC522_Rd( COMMIRQREG );
    i--;
  }
  while (i && !(n & 0x01) && !( n & waitIRq) );
  MFRC522_Clear_Bit( BITFRAMINGREG, 0x80 );    //StartSend=0
  if (i != 0)  {
    if( !( MFRC522_Rd( ERRORREG) & 0x1B ) ) //BufferOvfl Collerr CRCErr ProtecolErr
    {
      _status = MI_OK;
      if ( n & irqEn & 0x01 )      {
        _status = MI_NOTAGERR;       //??
      }
      if ( command == PCD_TRANSCEIVE )
      {
        n = MFRC522_Rd( FIFOLEVELREG );
        lastBits = MFRC522_Rd( CONTROLREG ) & 0x07;
        if (lastBits)
        {
          *backLen = (n-1) * 8 + lastBits;
        }
        else  {
          *backLen = n * 8;
        }

        if (n == 0) {
          n = 1;
        }
        if (n > 16)   {
          n = 16;
        }
        //Reading the received data in FIFO
        for (i=0; i < n; i++) {
          backData[i] = MFRC522_Rd( FIFODATAREG );
        }
  
  backData[i] = 0;
      }
    }
    else {
      _status = MI_ERR;
    }
  }
  //MFRC522_Set_Bit( CONTROLREG, 0x80 );
  //MFRC522_Wr( COMMANDREG, PCD_IDLE );
  return _status;
}
char MFRC522_Request( char reqMode, char *TagType )
{
  char _status;
  unsigned backBits;            // Os bits de dados recebidos

  MFRC522_Wr( BITFRAMINGREG, 0x07 ); //TxLastBists = BitFramingReg[2..0]   ???

  TagType[0] = reqMode;
  _status = MFRC522_ToCard( PCD_TRANSCEIVE, TagType, 1, TagType, &backBits );
  if ( (_status != MI_OK) || (backBits != 0x10) )
  {
    _status = MI_ERR;
  }
  return _status;
}
void MFRC522_CRC( char *dataIn, char length, char *dataOut )
{
char i, n;
    MFRC522_Clear_Bit( DIVIRQREG, 0x04 );
    MFRC522_Set_Bit( FIFOLEVELREG, 0x80 );    
    
 //Escreve dados no FIFO        
    for ( i = 0; i < length; i++ )
    {   
        MFRC522_Wr( FIFODATAREG, *dataIn++ );   
    }
    
    MFRC522_Wr( COMMANDREG, PCD_CALCCRC );
        
    i = 0xFF;
    //Espera a finalização do cálculo do CRC
    do 
    {
        n = MFRC522_Rd( DIVIRQREG );
        i--;
    }
    while( i && !(n & 0x04) );        //CRCIrq = 1
    dataOut[0] = MFRC522_Rd( CRCRESULTREGL );
    dataOut[1] = MFRC522_Rd( CRCRESULTREGM );        
}



char MFRC522_SelectTag( char *serNum )
{
  char i;
  char _status;
  char size;
  unsigned recvBits;
  char buffer[9];
  
  //MFRC522_Clear_Bit( STATUS2REG, 0x08 );   //MFCrypto1On=0
  
  buffer[0] = PICC_SElECTTAG;
  buffer[1] = 0x70;
  
  for ( i=2; i < 7; i++ )
  {
    buffer[i] = *serNum++;
  }
  
  MFRC522_CRC( buffer, 7, &buffer[7] );             
  
  _status = MFRC522_ToCard( PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits );

  if ( (_status == MI_OK) && (recvBits == 0x18) )
  {
    size = buffer[0];
  }
  else {
    size = 0;
  }
  return size;
}
//hibernation
void MFRC522_Halt()
{
  unsigned unLen;
  char buff[4];
  buff[0] = PICC_HALT;
  buff[1] = 0;
  MFRC522_CRC( buff, 2, &buff[2] );
  MFRC522_Clear_Bit( STATUS2REG, 0x80 );
  MFRC522_ToCard( PCD_TRANSCEIVE, buff, 4, buff, &unLen );
  MFRC522_Clear_Bit( STATUS2REG, 0x08 );
}
char MFRC522_Auth(char authMode, char BlockAddr, char *Sectorkey, char *serNum)
{
  char _status;
  unsigned recvBits;
  char i;
  char buff[12];
  
  // Verifique o endereço do bloco de comando + setor + senha + número de série do cartão
  buff[0] = authMode;
  buff[1] = BlockAddr;
  for ( i = 2; i < 8; i++ )  {
    buff[i] = Sectorkey[i-2];
  }
  for ( i = 8; i < 12; i++ )  {
    buff[i] = serNum[i-8];
  }
  _status = MFRC522_ToCard( PCD_AUTHENT, buff, 12, buff, &recvBits );
  
  if ( ( _status != MI_OK ) || !( MFRC522_Rd( STATUS2REG ) & 0x08 ) )
  {
    _status = MI_ERR;
  }
  return _status;
}
char MFRC522_Write( char blockAddr, char *writeData) {
  char _status;
  unsigned recvBits;
  char i;
  char buff[18];
  buff[0] = PICC_WRITE;
  buff[1] = blockAddr;
  
  MFRC522_CRC( buff, 2, &buff[2] );
  _status = MFRC522_ToCard( PCD_TRANSCEIVE, buff, 4, buff, &recvBits );

  if ( (_status != MI_OK) || (recvBits != 4) || ( (buff[0] & 0x0F) != 0x0A) )
  {
    _status = MI_ERR;
  }
  if (_status == MI_OK)
  {
    for ( i = 0; i < 16; i++ )                //Data to the FIFO write 16Byte
    {
      buff[i] = writeData[i];
    }    
    MFRC522_CRC( buff, 16, &buff[16] );
    _status = MFRC522_ToCard( PCD_TRANSCEIVE, buff, 18, buff, &recvBits );

    if ( (_status != MI_OK) || (recvBits != 4) || ( (buff[0] & 0x0F) != 0x0A ) )
    {
      _status = MI_ERR;
    }
  }
  return _status;
}
char MFRC522_Read( char blockAddr, char *recvData )
{
  char _status;
  unsigned unLen;

  recvData[0] = PICC_READ;
  recvData[1] = blockAddr;
  
  MFRC522_CRC( recvData, 2, &recvData[2] );
  
  _status = MFRC522_ToCard( PCD_TRANSCEIVE, recvData, 4, recvData, &unLen );
  if ( (_status != MI_OK) || (unLen != 0x90) )
  {
    _status = MI_ERR;
  }
  return _status;
}

char MFRC522_AntiColl( char *serNum )
{
  char _status;
  char i;
  char serNumCheck = 0;
  unsigned unLen;

  MFRC522_Wr( BITFRAMINGREG, 0x00 );     //TxLastBists = BitFramingReg[2..0]
  serNum[0] = PICC_ANTICOLL;
  serNum[1] = 0x20;
  MFRC522_Clear_Bit( STATUS2REG, 0x08 );
  _status = MFRC522_ToCard( PCD_TRANSCEIVE, serNum, 2, serNum, &unLen );

  if (_status == MI_OK)
  {
    for ( i=0; i < 4; i++ )
    {
      serNumCheck ^= serNum[i];
    }
    
    if ( serNumCheck != serNum[4] ) {
      _status = MI_ERR;
    }
  }
  return _status;
}
//0x0044 = Mifare_UltraLight
//0x0004 = Mifare_One (S50)
//0x0002 = Mifare_One (S70)
//0x0008 = Mifare_Pro (X)
//0x0344 = Mifare_DESFire
char MFRC522_isCard( char *TagType ) 
{
    if (MFRC522_Request( PICC_REQIDL, TagType ) == MI_OK)
        return 1;
    else
        return 0; 
}
char MFRC522_ReadCardSerial( char *str )
{
char _status; 
 _status = MFRC522_AntiColl( str );
 str[5] = 0;
 if (_status == MI_OK)
  return 1;
 else
  return 0;
}

Código:
//Habilitar as seguintes bibliotecas:
// - Software_SPI
// - Conversions e C_String
// - LCD
#include <Built_in.h>
//O Teste foi feito com cartões MIFARE 1K
sbit MFRC522_CS at RD4_Bit;
sbit MFRC522_Rst at RD3_Bit;
sbit SoftSPI_SDO at RD1_Bit;
sbit SoftSPI_CLK at RD0_Bit;
sbit SoftSPI_SDI at RD2_Bit;
sbit MFRC522_CS_Direction at TRISD4_Bit;
sbit MFRC522_Rst_Direction at TRISD3_Bit;
sbit SoftSPI_SDO_Direction at TRISD1_Bit;
sbit SoftSPI_CLK_Direction at TRISD0_Bit;
sbit SoftSPI_SDI_Direction at TRISD2_Bit;

// Conexões do módulo LCD
sbit LCD_RS at RB4_bit;
sbit LCD_EN at RB5_bit;
sbit LCD_D4 at RB0_bit;
sbit LCD_D5 at RB1_bit;
sbit LCD_D6 at RB2_bit;
sbit LCD_D7 at RB3_bit;
sbit LCD_RS_Direction at TRISB4_bit;
sbit LCD_EN_Direction at TRISB5_bit;
sbit LCD_D4_Direction at TRISB0_bit;
sbit LCD_D5_Direction at TRISB1_bit;
sbit LCD_D6_Direction at TRISB2_bit;
sbit LCD_D7_Direction at TRISB3_bit;
// Fim das conexões do módulo LCD
char key[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
char writeData[] = "Microcontrolandos";

void main() {
char msg[12];
char UID[6];
unsigned TagType;
char size;
char i;
  //Desativa as Portas Analogicas
     ADCON1 = 0x0F;
     //Inicializa Soft SPI
     Soft_SPI_Init();
     //Inicializa display
     Lcd_Init();
     Lcd_Cmd(_LCD_CLEAR);             
     Lcd_Cmd(_LCD_CURSOR_OFF);       
  
     //inicializa o modulo RFID
     MFRC522_Init();
     
     while(1)
     {
     //Verifica se há algum cartão
     if( MFRC522_isCard( &TagType ) )
     {
         Lcd_Cmd(_LCD_CLEAR);
         //Exibe o tipo do cartão no display
         ByteToHex( Lo(TagType), msg );
         ByteToHex( Hi(TagType), msg+2 );
         Lcd_Out( 1, 1, "TAG TYPE: " );
         Lcd_Out_CP( msg );

         //Faz a leitura do numero de serie
         if( MFRC522_ReadCardSerial( &UID ) )
         {
             //Posiciona o cursor 
             Lcd_Out( 1, 1, "" );
                 
             for( i=0; i < 5; i++)
             {
                 ByteToHex( UID[i], msg );
                 Lcd_Out_CP( msg );
             }
             size = MFRC522_SelectTag( &UID );
         }
               
         //Posiciona o cursor 
         Lcd_Out( 2, 1, "" );

         //Tenta realizar a autenticação A do setor 1( blocos: 4 - 7 )
         //bloco de autenticação é o 7
         if( MFRC522_Auth( PICC_AUTHENT1A, 7, &key, &UID ) == 0 )
         {
             //Escreve algo no bloco 4
             MFRC522_Write( 4, &writeData );
         }
         else if( MFRC522_Auth( PICC_AUTHENT1B, 7, &key, &UID ) == 0 )
         {
             //Escreve algo no bloco 4
             MFRC522_Write( 4, &writeData );
         }
         else
         {
            Lcd_Out_CP( "Erro" );
            continue;
         }           
         //Faz a leitura do bloco 4
         if( MFRC522_Read( 4, &writeData ) == 0 {
           Lcd_Out( 2, 1, &writeData );
         }
         //Estado de hibernação
         MFRC522_Halt();
     }
  }
}

extern sfr sbit MFRC522_CS;
extern sfr sbit MFRC522_Rst;
extern sfr sbit MFRC522_CS_Direction;
extern sfr sbit MFRC522_Rst_Direction;


//MF522 Command word
#define PCD_IDLE              0x00               //NO action; Cancel the current command
#define PCD_AUTHENT           0x0E               //Authentication Key
#define PCD_RECEIVE           0x08               //Receive Data
#define PCD_TRANSMIT          0x04               //Transmit data
#define PCD_TRANSCEIVE        0x0C               //Transmit and receive data,
#define PCD_RESETPHASE        0x0F               //Reset
#define PCD_CALCCRC           0x03               //CRC Calculate

// Mifare_One card command word
#define PICC_REQIDL          0x26               // find the antenna area does not enter hibernation
#define PICC_REQALL          0x52               // find all the cards antenna area
#define PICC_ANTICOLL        0x93               // anti-collision
#define PICC_SElECTTAG       0x93               // election card
#define PICC_AUTHENT1A       0x60               // authentication key A
#define PICC_AUTHENT1B       0x61               // authentication key B
#define PICC_READ            0x30               // Read Block
#define PICC_WRITE           0xA0               // write block
#define PICC_DECREMENT       0xC0               // debit
#define PICC_INCREMENT       0xC1               // recharge
#define PICC_RESTORE         0xC2               // transfer block data to the buffer
#define PICC_TRANSFER        0xB0               // save the data in the buffer
#define PICC_HALT            0x50               // Sleep

//And MF522 The error code is returned when communication
#define MI_OK                 0
#define MI_NOTAGERR           1
#define MI_ERR                2

//------------------MFRC522 Register---------------
//Page 0:Command and Status
#define     RESERVED00            0x00    
#define     COMMANDREG            0x01    
#define     COMMIENREG            0x02    
#define     DIVLENREG             0x03    
#define     COMMIRQREG            0x04    
#define     DIVIRQREG             0x05
#define     ERRORREG              0x06    
#define     STATUS1REG            0x07    
#define     STATUS2REG            0x08    
#define     FIFODATAREG           0x09
#define     FIFOLEVELREG          0x0A
#define     WATERLEVELREG         0x0B
#define     CONTROLREG            0x0C
#define     BITFRAMINGREG         0x0D
#define     COLLREG               0x0E
#define     RESERVED01            0x0F
//PAGE 1:Command     
#define     RESERVED10            0x10
#define     MODEREG               0x11
#define     TXMODEREG             0x12
#define     RXMODEREG             0x13
#define     TXCONTROLREG          0x14
#define     TXAUTOREG             0x15
#define     TXSELREG              0x16
#define     RXSELREG              0x17
#define     RXTHRESHOLDREG        0x18
#define     DEMODREG              0x19
#define     RESERVED11            0x1A
#define     RESERVED12            0x1B
#define     MIFAREREG             0x1C
#define     RESERVED13            0x1D
#define     RESERVED14            0x1E
#define     SERIALSPEEDREG        0x1F
//PAGE 2:CFG    
#define     RESERVED20            0x20  
#define     CRCRESULTREGM         0x21
#define     CRCRESULTREGL         0x22
#define     RESERVED21            0x23
#define     MODWIDTHREG           0x24
#define     RESERVED22            0x25
#define     RFCFGREG              0x26
#define     GSNREG                0x27
#define     CWGSPREG              0x28
#define     MODGSPREG             0x29
#define     TMODEREG              0x2A
#define     TPRESCALERREG         0x2B
#define     TRELOADREGH           0x2C
#define     TRELOADREGL           0x2D
#define     TCOUNTERVALUEREGH     0x2E
#define     TCOUNTERVALUEREGL     0x2F
//PAGE 3:TEST REGISTER     
#define     RESERVED30            0x30
#define     TESTSEL1REG           0x31
#define     TESTSEL2REG           0x32
#define     TESTPINENREG          0x33
#define     TESTPINVALUEREG       0x34
#define     TESTBUSREG            0x35
#define     AUTOTESTREG           0x36
#define     VERSIONREG            0x37
#define     ANALOGTESTREG         0x38
#define     TESTDAC1REG           0x39  
#define     TESTDAC2REG           0x3A   
#define     TESTADCREG            0x3B   
#define     RESERVED31            0x3C   
#define     RESERVED32            0x3D   
#define     RESERVED33            0x3E   
#define     RESERVED34            0x3F






static void MFRC522_Wr( char addr, char value )
{
        MFRC522_CS = 0;
        Soft_SPI_Write( ( addr << 1 ) & 0x7E );
        Soft_SPI_Write( value );
        MFRC522_CS = 1;
}







static char MFRC522_Rd( char addr )
{
char value;
        MFRC522_CS = 0;
        Soft_SPI_Write( (( addr << 1 ) & 0x7E) | 0x80 );
        value = Soft_SPI_Read( 0x00 );
        MFRC522_CS = 1;
        return value;
}



static void MFRC522_Clear_Bit( char addr, char mask )
{
     MFRC522_Wr( addr, MFRC522_Rd( addr ) & (~mask) );
}


static void MFRC522_Set_Bit( char addr, char mask )
{
     MFRC522_Wr( addr, MFRC522_Rd( addr ) | mask );
}

void MFRC522_Reset()
{
        MFRC522_Wr( COMMANDREG, PCD_RESETPHASE );
}



void MFRC522_AntennaOn()
{
 MFRC522_Set_Bit( TXCONTROLREG, 0x03 );
}



void MFRC522_AntennaOff()
{
 MFRC522_Clear_Bit( TXCONTROLREG, 0x03 );
}



void MFRC522_Init()
{
     MFRC522_CS_Direction = 0;
     MFRC522_Rst_Direction = 0;
     MFRC522_CS = 1;
     MFRC522_Rst = 1;
        
     MFRC522_Reset();
        
     MFRC522_Wr( TMODEREG, 0x8D );      //Tauto=1; f(Timer) = 6.78MHz/TPreScaler
     MFRC522_Wr( TPRESCALERREG, 0x3E ); //TModeReg[3..0] + TPrescalerReg
     MFRC522_Wr( TRELOADREGL, 30 );           
     MFRC522_Wr( TRELOADREGH, 0 );
        
     MFRC522_Wr( TXAUTOREG, 0x40 );    //100%ASK
     MFRC522_Wr( MODEREG, 0x3D );      // CRC valor inicial de 0x6363
        
     //MFRC522_Clear_Bit( STATUS2REG, 0x08 );//MFCrypto1On=0
     //MFRC522_Wr( RXSELREG, 0x86 );      //RxWait = RxSelReg[5..0]
     //MFRC522_Wr( RFCFGREG, 0x7F );     //RxGain = 48dB

     MFRC522_AntennaOn();
}



char MFRC522_ToCard( char command, char *sendData, char sendLen, char *backData, unsigned *backLen )
{
  char _status = MI_ERR;
  char irqEn = 0x00;
  char waitIRq = 0x00;
  char lastBits;
  char n;
  unsigned i;
  
  switch (command)
  {
    case PCD_AUTHENT:       //Certification cards close
    {
      irqEn = 0x12;
      waitIRq = 0x10;
      break;
    }
    case PCD_TRANSCEIVE:    //Transmit FIFO data
    {
      irqEn = 0x77;
      waitIRq = 0x30;
      break;
    }
    default:
      break;
  }

  MFRC522_Wr( COMMIENREG, irqEn | 0x80 );  //Interrupt request
  MFRC522_Clear_Bit( COMMIRQREG, 0x80 );   //Clear all interrupt request bit
  MFRC522_Set_Bit( FIFOLEVELREG, 0x80 );   //FlushBuffer=1, FIFO Initialization
  MFRC522_Wr( COMMANDREG, PCD_IDLE );      //NO action; Cancel the current command???
  
  
  
  //Writing data to the FIFO
  for ( i=0; i < sendLen; i++ )
  {
    MFRC522_Wr( FIFODATAREG, sendData[i] );
  }

  //Execute the command
  MFRC522_Wr( COMMANDREG, command );

  if (command == PCD_TRANSCEIVE )
  {
    MFRC522_Set_Bit( BITFRAMINGREG, 0x80 ); //StartSend=1,transmission of data starts  
  }

  //Waiting to receive data to complete
  //i according to the clock frequency adjustment, the operator M1 card maximum waiting time 25ms???
  i = 0xFFFF;   
  do
  {
    //CommIrqReg[7..0]
    //Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
    n = MFRC522_Rd( COMMIRQREG );
    i--;
  }
  while ( i && !(n & 0x01) && !( n & waitIRq ) );

  MFRC522_Clear_Bit( BITFRAMINGREG, 0x80 );    //StartSend=0

  if (i != 0)
  {
    if( !( MFRC522_Rd( ERRORREG ) & 0x1B ) ) //BufferOvfl Collerr CRCErr ProtecolErr
    {
      _status = MI_OK;
      if ( n & irqEn & 0x01 )
      {
        _status = MI_NOTAGERR;       //??
      }

      if ( command == PCD_TRANSCEIVE )
      {
        n = MFRC522_Rd( FIFOLEVELREG );
        lastBits = MFRC522_Rd( CONTROLREG ) & 0x07;
        if (lastBits)
        {
          *backLen = (n-1) * 8 + lastBits;
        }
        else
        {
          *backLen = n * 8;
        }

        if (n == 0)
        {
          n = 1;
        }

        if (n > 16)
        {
          n = 16;
        }

        //Reading the received data in FIFO
        for (i=0; i < n; i++)
        {
          backData[i] = MFRC522_Rd( FIFODATAREG );
        }
  
  backData[i] = 0;
      }
    }
    else
    {
      _status = MI_ERR;
    }

  }

  //MFRC522_Set_Bit( CONTROLREG, 0x80 );
  //MFRC522_Wr( COMMANDREG, PCD_IDLE );

  return _status;
}



char MFRC522_Request( char reqMode, char *TagType )
{
  char _status;
  unsigned backBits;            //The received data bits

  MFRC522_Wr( BITFRAMINGREG, 0x07 ); //TxLastBists = BitFramingReg[2..0]   ???

  TagType[0] = reqMode;
  _status = MFRC522_ToCard( PCD_TRANSCEIVE, TagType, 1, TagType, &backBits );

  if ( (_status != MI_OK) || (backBits != 0x10) )
  {
    _status = MI_ERR;
  }

  return _status;
}



void MFRC522_CRC( char *dataIn, char length, char *dataOut )
{
char i, n;
    MFRC522_Clear_Bit( DIVIRQREG, 0x04 );
    MFRC522_Set_Bit( FIFOLEVELREG, 0x80 );    
    
 //Escreve dados no FIFO        
    for ( i = 0; i < length; i++ )
    {   
        MFRC522_Wr( FIFODATAREG, *dataIn++ );   
    }
    
    MFRC522_Wr( COMMANDREG, PCD_CALCCRC );
        
    i = 0xFF;
    //Espera a finalização do Calculo do CRC
    do 
    {
        n = MFRC522_Rd( DIVIRQREG );
        i--;
    }
    while( i && !(n & 0x04) );        //CRCIrq = 1
        
    dataOut[0] = MFRC522_Rd( CRCRESULTREGL );
    dataOut[1] = MFRC522_Rd( CRCRESULTREGM );        
}



char MFRC522_SelectTag( char *serNum )
{
  char i;
  char _status;
  char size;
  unsigned recvBits;
  char buffer[9];
  
  //MFRC522_Clear_Bit( STATUS2REG, 0x08 );   //MFCrypto1On=0
  
  buffer[0] = PICC_SElECTTAG;
  buffer[1] = 0x70;
  
  for ( i=2; i < 7; i++ )
  {
    buffer[i] = *serNum++;
  }
  
  MFRC522_CRC( buffer, 7, &buffer[7] );             
  
  _status = MFRC522_ToCard( PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits );

  if ( (_status == MI_OK) && (recvBits == 0x18) )
  {
    size = buffer[0];
  }
  else
  {
    size = 0;
  }

  return size;
}



//hibernation
void MFRC522_Halt()
{
  unsigned unLen;
  char buff[4];
  
  buff[0] = PICC_HALT;
  buff[1] = 0;
  MFRC522_CRC( buff, 2, &buff[2] );

  MFRC522_Clear_Bit( STATUS2REG, 0x80 );
  MFRC522_ToCard( PCD_TRANSCEIVE, buff, 4, buff, &unLen );
  MFRC522_Clear_Bit( STATUS2REG, 0x08 );
}



char MFRC522_Auth( char authMode, char BlockAddr, char *Sectorkey, char *serNum )
{
  char _status;
  unsigned recvBits;
  char i;
  char buff[12];
  
  //Verify the command block address + sector + password + card serial number
  buff[0] = authMode;
  buff[1] = BlockAddr;
  
  for ( i = 2; i < 8; i++ )
  {
    buff[i] = Sectorkey[i-2];
  }
  
  for ( i = 8; i < 12; i++ )
  {
    buff[i] = serNum[i-8];
  }
  
  _status = MFRC522_ToCard( PCD_AUTHENT, buff, 12, buff, &recvBits );
  
  if ( ( _status != MI_OK ) || !( MFRC522_Rd( STATUS2REG ) & 0x08 ) )
  {
    _status = MI_ERR;
  }
  
  return _status;
}



char MFRC522_Write( char blockAddr, char *writeData )
{
  char _status;
  unsigned recvBits;
  char i;
  char buff[18];

  buff[0] = PICC_WRITE;
  buff[1] = blockAddr;
  
  MFRC522_CRC( buff, 2, &buff[2] );
  _status = MFRC522_ToCard( PCD_TRANSCEIVE, buff, 4, buff, &recvBits );

  if ( (_status != MI_OK) || (recvBits != 4) || ( (buff[0] & 0x0F) != 0x0A) )
  {
    _status = MI_ERR;
  }

  if (_status == MI_OK)
  {
    for ( i = 0; i < 16; i++ )                //Data to the FIFO write 16Byte
    {
      buff[i] = writeData[i];
    }
    
    MFRC522_CRC( buff, 16, &buff[16] );
    _status = MFRC522_ToCard( PCD_TRANSCEIVE, buff, 18, buff, &recvBits );

    if ( (_status != MI_OK) || (recvBits != 4) || ( (buff[0] & 0x0F) != 0x0A ) )
    {
      _status = MI_ERR;
    }
  }

  return _status;
}



char MFRC522_Read( char blockAddr, char *recvData )
{
  char _status;
  unsigned unLen;

  recvData[0] = PICC_READ;
  recvData[1] = blockAddr;
  
  MFRC522_CRC( recvData, 2, &recvData[2] );
  
  _status = MFRC522_ToCard( PCD_TRANSCEIVE, recvData, 4, recvData, &unLen );

  if ( (_status != MI_OK) || (unLen != 0x90) )
  {
    _status = MI_ERR;
  }

  return _status;
}



char MFRC522_AntiColl( char *serNum )
{
  char _status;
  char i;
  char serNumCheck = 0;
  unsigned unLen;

  MFRC522_Wr( BITFRAMINGREG, 0x00 );                //TxLastBists = BitFramingReg[2..0]

  serNum[0] = PICC_ANTICOLL;
  serNum[1] = 0x20;
  MFRC522_Clear_Bit( STATUS2REG, 0x08 );
  _status = MFRC522_ToCard( PCD_TRANSCEIVE, serNum, 2, serNum, &unLen );

  if (_status == MI_OK)
  {
    for ( i=0; i < 4; i++ )
    {
      serNumCheck ^= serNum[i];
    }
    
    if ( serNumCheck != serNum[4] )
    {
      _status = MI_ERR;
    }
  }

  return _status;
}

//0x0044 = Mifare_UltraLight
//0x0004 = Mifare_One (S50)
//0x0002 = Mifare_One (S70)
//0x0008 = Mifare_Pro (X)
//0x0344 = Mifare_DESFire
char MFRC522_isCard( char *TagType ) 
{
    if (MFRC522_Request( PICC_REQIDL, TagType ) == MI_OK)
        return 1;
    else
        return 0; 
}


char MFRC522_ReadCardSerial( char *str )
{
char _status; 
 _status = MFRC522_AntiColl( str );
 str[5] = 0;
 if (_status == MI_OK)
  return 1;
 else
  return 0;
}

//Habilitar as seguintes bibliotecas:
// - Software_SPI
// - Conversions e C_String
// - Lcd
#include <Built_in.h>
//cole aqui o código da biblioteca
//O Teste foi feito com cartões MIFARE 1K
sbit MFRC522_CS at RD4_Bit;
sbit MFRC522_Rst at RD3_Bit;
sbit SoftSPI_SDO at RD1_Bit;
sbit SoftSPI_CLK at RD0_Bit;
sbit SoftSPI_SDI at RD2_Bit;
sbit MFRC522_CS_Direction at TRISD4_Bit;
sbit MFRC522_Rst_Direction at TRISD3_Bit;
sbit SoftSPI_SDO_Direction at TRISD1_Bit;
sbit SoftSPI_CLK_Direction at TRISD0_Bit;
sbit SoftSPI_SDI_Direction at TRISD2_Bit;
// LCD module connections
sbit LCD_RS at RB4_bit;
sbit LCD_EN at RB5_bit;
sbit LCD_D4 at RB0_bit;
sbit LCD_D5 at RB1_bit;
sbit LCD_D6 at RB2_bit;
sbit LCD_D7 at RB3_bit;
sbit LCD_RS_Direction at TRISB4_bit;
sbit LCD_EN_Direction at TRISB5_bit;
sbit LCD_D4_Direction at TRISB0_bit;
sbit LCD_D5_Direction at TRISB1_bit;
sbit LCD_D6_Direction at TRISB2_bit;
sbit LCD_D7_Direction at TRISB3_bit;
// End LCD module connections
char key[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
char writeData[] = "Microcontrolandos";
void main()  {
char msg[12];
char UID[6];
unsigned TagType;
char size;
char i;
     //Desativa as Portas Analógicas
     ADCON1 = 0x0F;
     //Inicializa Soft SPI
     Soft_SPI_Init();
     //Inicializa display
     Lcd_Init();
     Lcd_Cmd(_LCD_CLEAR);             
     Lcd_Cmd(_LCD_CURSOR_OFF);            
//inicializa o modulo RFID
     MFRC522_Init ();
          while (1)     {
     //Verifica se há algum cartão
     if( MFRC522_isCard( &TagType ) )     {
         Lcd_Cmd(_LCD_CLEAR);
         //Exibe o tipo do cartão no display
         ByteToHex( Lo(TagType), msg );
         ByteToHex( Hi(TagType), msg+2 );
         Lcd_Out( 1, 1, "TAG TYPE: " );
         Lcd_Out_CP( msg );
         //Faz a leitura do número de serie
         if( MFRC522_ReadCardSerial( &UID ) )       {
             //Posiciona o cursor 
             Lcd_Out( 1, 1, "" );                
             for( i=0; i < 5; i++)      {
                 ByteToHex( UID[i], msg );
                 Lcd_Out_CP( msg );
             }
             size = MFRC522_SelectTag( &UID );
         }               
         //Posiciona o cursor 
         Lcd_Out( 2, 1, "" );
         //Tenta realizar a autenticação A do setor 1( blocos: 4 - 7 )
         //bloco de autenticação é o 7
         if( MFRC522_Auth( PICC_AUTHENT1A, 7, &key, &UID ) == 0 )         {
             //Escreve algo no bloco 4
             MFRC522_Write( 4, &writeData );
         }
         else if( MFRC522_Auth( PICC_AUTHENT1B, 7, &key, &UID ) == 0 )       {
             //Escreve algo no bloco 4
             MFRC522_Write( 4, &writeData );
         }
         else     {
            Lcd Out_CP( "Erro" );
            continue;
         }            
         //Faz a leitura do bloco 4
         if( MFRC522_Read( 4, &writeData ) == 0 )       {
           Lcd_Out( 2, 1, &writeData );
         }
         //Estado de hibernação
         MFRC522_Halt();
     }
  }
}
