# Esp32EbikeDisplay

内部FLASH SPI引脚不出   
GPIO29 SPICLK  CLK    
GPIO24 SPICS0  CS#    
GPIO30 SPID    MOSI  
GPIO25 SPIQ    MISO   
GPIO26 SPIWP  
GPIO28 SPIHD  

SPI2    
GPIO2  SPIQ     MISO    
GPIO4  FSPIHD    
GPIO5  FSPIWP    
GPIO6  FSPICLK  CLK    
GPIO7  FSPID    MOSI    
GPIO16(TX)  FSPICS0    CS0    
GPIO17(RX)  FSPICS1    CS1    
GPIO18      FSPICS2    CS2    
GPIO19      FSPICS3    CS3    
GPIO20      FSPICS4    CS4    
GPIO21      FSPICS5    CS5    

ADC    
GPIO 0-6（最好跳过4，5）   

LP_IIC  
IO6 SDA
IO7 SCL

LP UART  
LP_GPIO4  LP_UART_RX    
LP_GPIO5  LP_UART_TX    

STRAPPING  
IO8 上拉
IO9 上拉接开关切换下载模式