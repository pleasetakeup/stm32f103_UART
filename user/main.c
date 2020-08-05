#include "stm8s.h"
#include "stm8s_clk.h"
#include "stm8s_i2c.h"
#include "stm8s_spi.h"
#include "stm8s_it.h"
#include "stm8s_gpio.h"
#include "stm8s_uart1.h"

#define SERIAL_TX_BUFFER_SIZE 128        //buffer大小
typedef uint16_t tx_buffer_index_t;

#define	I2C_SLAVE_ADDRES (0x10)

__IO uint16_t Event = 0x00;
volatile uint16_t temp1 = 0;
volatile uint16_t temp2 = 0;
volatile uint8_t  gotdata = 0;
uint8_t regaddr; //在i2c中获得regaddr的值
u8 uart_to_iic_buf[SERIAL_TX_BUFFER_SIZE];
u8 iic_to_uart_buf[SERIAL_TX_BUFFER_SIZE];
//unsigned char _rx_buffer[SERIAL_RX_BUFFER_SIZE];
//uint16_t _tx_buffer[SERIAL_TX_BUFFER_SIZE];  //设定存储数据的数组
//rx_buffer_index_t _rx_buffer_head;
//rx_buffer_index_t _rx_buffer_tail;
tx_buffer_index_t _tx_buffer_head = 0 ;  //不需要设定初始值吗？
tx_buffer_index_t _tx_buffer_tail = 0 ;
tx_buffer_index_t _iic_buffer_head = 0 ;  //IIC接收到的命令
tx_buffer_index_t _iic_buffer_tail = 0 ;


void Delay(u32 nCount)
{
  /* Decrement nCount value */
  while (nCount != 0) {
    nCount--;
  }
}

void uart_init( uint32_t baudrate)
{
  //Initalize the PD6, it's must to do this
  GPIO_Init(GPIOA, GPIO_PIN_6, GPIO_MODE_IN_FL_IT );
  GPIO_Init(GPIOA, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_FAST);
  //RX_EN();
  //Reset the UART1
  UART1_DeInit();

  //Initalize the UART1

  UART1_Init((u32)baudrate, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO, UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);


  //Open the IT.

  UART1_ClearITPendingBit(UART1_IT_RXNE);
  UART1_ITConfig( UART1_IT_RXNE_OR, ENABLE );

  //Open UART1
  UART1_Cmd( ENABLE );


  //Enable interrupt of all.
  //__enable_interrupt();
}


void uart1_txdata(u8 data)
{

  UART1_SendData8(data);
  while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
}

void uart1_txstr(u8 *p,u8 datalen)
{
  while(datalen) {
    uart1_txdata(*p);
    p++;
    datalen--;
  }
  while (UART1_GetFlagStatus( UART1_FLAG_TC) == RESET);
}



void i2c_init(void)
{
  //I2C_DeInit();

  GPIO_Init(GPIOB, GPIO_PIN_4, GPIO_MODE_OUT_OD_LOW_FAST); //IIC,OD outPut

GPIO_Init(GPIOB, GPIO_PIN_5, GPIO_MODE_OUT_OD_LOW_FAST); //IIC,OD outPut

 I2C_Init((uint32_t)(400000), (I2C_SLAVE_ADDRES << 1),
           I2C_DUTYCYCLE_2, I2C_ACK_CURR,
           I2C_ADDMODE_7BIT, 16 );

 I2C_ITConfig((I2C_IT_TypeDef)(I2C_IT_ERR | I2C_IT_EVT | I2C_IT_BUF), ENABLE); //I2C中断
}



////为了确定环形buffer空余的大小
//int availableForWrite(void)
//{
//  tx_buffer_index_t head = _tx_buffer_head;
//  tx_buffer_index_t tail = _tx_buffer_tail;
//  if (head >= tail) return SERIAL_TX_BUFFER_SIZE - 1 - head + tail;
//  return tail - head - 1;//头<尾
//}


//IIC接收数据和UART发送数据都在此函数实现。（中断）
void I2C_byte_received(u8 u8_RxData)
{

  tx_buffer_index_t i = (_iic_buffer_head + 1) % SERIAL_TX_BUFFER_SIZE;  
  while(i == _iic_buffer_tail)  //说明buffer满了     
  {
      uart1_txdata(iic_to_uart_buf[_iic_buffer_tail]);  
      _iic_buffer_tail = (_iic_buffer_tail + 1) % SERIAL_TX_BUFFER_SIZE;
      
  }
  iic_to_uart_buf[_iic_buffer_head] = u8_RxData; //将数据存入了_tx_buffer中，
  _iic_buffer_head = i;  //头指针增加
}

unsigned char read_buffer()
{


  if (_iic_buffer_head == _iic_buffer_tail)  //buffer为空，没有数据
  {
   return 0;
  } 
  else
  {
      uart1_txdata(iic_to_uart_buf[_iic_buffer_tail] );  
     _iic_buffer_tail = (_iic_buffer_tail + 1) % SERIAL_TX_BUFFER_SIZE;
 
  } 
  
}



 void uart_rxgetdata(void)//uart中断服务
{
   uint8_t temp;
    temp = (UART1_ReceiveData8());uart1_txdata(temp);
        uart_to_iic_buf[_tx_buffer_head] = temp;//放Buf里可以通过iic发回arduino
       _tx_buffer_head =(_tx_buffer_head +1) % SERIAL_TX_BUFFER_SIZE;
      UART1_ClearITPendingBit(UART1_IT_RXNE);            
}

void I2C_Slave_check_event_ISR(void)//IIC中断服务
{
  
//接收发送
    u8 temp = 0;
    uart1_txdata(temp);
    //错误处理
    if (I2C->SR2&0x01)//总线错误
    {
        I2C->SR2 &= ~0x01;//清零
    }
    else if(I2C->SR2&0x02)//仲裁失败
    {
        I2C->SR2 &= ~0x02;//清零
    }
    else if(I2C->SR2&0x04)//应答失败
    {
        I2C->SR2 &= ~0x04;//清零
    }
    else if(I2C->SR2&0x08)//上溢、下溢
    {
        I2C->SR2 &= ~0x08;//清零
    }
    if (I2C->SR1&0x02)//地址已经匹配(读SR1，SR3清除该位)
    {
        temp = I2C->SR3&0x07;
    }
    
    else if ((I2C->SR1&0x40)&&(0 == (I2C->SR3&0x04)))//接收时数据寄存器
    {
        temp = I2C->DR;    //读取数据寄存器
        I2C_byte_received(temp);
    }
    else if ((I2C->SR1&0x84)&&(I2C->SR3&0x04))//数据字节传送完
    {
      
     if(_tx_buffer_head != _tx_buffer_tail) {
      I2C->DR = uart_to_iic_buf[_tx_buffer_tail];
      _tx_buffer_tail =(_tx_buffer_tail + 1) % SERIAL_TX_BUFFER_SIZE;
     }else {
       I2C->DR = 0xee;//不发点东西的话被读时会阻塞在那，I2C接口将保持 为低电平，以等待DR寄存器写操作，所以主机请求多了就卡死了
     }
    }
    else if (I2C->SR1&0x10)//检测到停止位
    {
        I2C->CR2 = I2C->CR2;//清除停止位
    }
}

void checkConnect(){
u8 i;   
u8 asd[26]={0x55,0xAA,00,00,0x24,00,04,00,01,04,04,00,00,00,00,00,00,00,00,00,00,00,00,00,0x30,01};

for(i=0 ;i < 26;i++){
  uart1_txdata(asd[i]);
 //Delay(1000);
}
/*
for(i=0 ;i<26;i++){
  temp2 = UART1_ReceiveData8();
  
}
*/
//uart1_txdata(temp2);


}

void main(void)
{
  //u8 asd[]={0x55,0xAA,00,00,0x24,00,02,00,01,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,0x26,01};
 // CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV8);//一分频 不能开
  i2c_init();
  uart_init(115200);
//  uart1_txdata(0x56);
  //checkConnect();
  
  __enable_interrupt();//中断最后开 
  
  
  
  while(1) {
    
     read_buffer();

  }
}