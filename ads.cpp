
#include "main.h"
#include "ads.h"

int samplingRate;
int windowSize;
int init_ADS () {
    printf("in ads_init\n");
    // Startup:
    // All Signals LOW

    //

    delay(500);   // make sure the ADS starts for sure


    bcm2835_gpio_write(ADS_START_PIN, LOW);
    bcm2835_gpio_write(ADS_CS_PIN, LOW);
    bcm2835_gpio_write(ADS_PWDN_PIN, LOW);
    bcm2835_gpio_write(ADS_RESET_PIN, LOW);
    delay(10);
    // !PWDN HIGH, !RESET = HIGH
    bcm2835_gpio_write(ADS_PWDN_PIN, HIGH);
    bcm2835_gpio_write(ADS_RESET_PIN, HIGH);


    delay(1000);   // make sure the ADS starts for sure
    // RESET PULSE
    bcm2835_gpio_write(ADS_RESET_PIN, LOW);
    delay(50);
    bcm2835_gpio_write(ADS_RESET_PIN, HIGH);
    printf("Power-Up Done \n");
    bcm2835_delayMicroseconds(18*TCLK);


    bcm2835_spi_transfer(ADS_SDATAC);

    printf("SDATAC sent \n");

    delay(10);

    // SEND WREG
    // 001 - 8k, 100 - 1k, 011 - 2k, 010 - 4k
    int ads_config1 = 0b10110100;
    writeREG(ADS_CONFIG1, ads_config1);
    samplingRate = ads_config1 & 0x07;
    switch (samplingRate) {
        case 1: samplingRate = 8000; break;
        case 2: samplingRate = 4000; break;
        case 3: samplingRate = 2000; break;
        case 4: samplingRate = 1000; break;
        default:  qDebug() << "ERROR: samplingRate wrong!!";
    }
    // Windowsize 50samples
    windowSize = (samplingRate/5);

    qDebug() << "samplingRate:" << samplingRate;

    delay(1);

    writeREG(ADS_CONFIG2,0b11000000);


    delay(1);

    writeREG(ADS_CONFIG3,0b11101100);


    delay(1);

    writeREG(ADS_LOFF, 0x00); // (standard 0x00) comperator threshold 95% and 5%, pullup or pulldown resistor dc lead-off
    //writeREG(ADS_LOFF, 0x40); // (standard 0x00) comperator threshold 90% and 10%, pullup or pulldown resistor dc lead-off
    //writeREG(ADS_LOFF, 0xA0); // (standard 0x00) comperator threshold 80% and 20%, pullup or pulldown resistor dc lead-off
    //writeREG(ADS_LOFF, 0xE0); // (standard 0x00) comperator threshold 80% and 20%, pullup or pulldown resistor dc lead-off


    //# gain 2 00010000 //gain 6 00110000 //gain12 01RESULTARRAY[counter%blockSize][2]01
    writeREG(ADS_CH1SET, ADS_gain);
    writeREG(ADS_CH2SET, ADS_gain);
    writeREG(ADS_CH3SET, ADS_gain);
    writeREG(ADS_CH4SET, ADS_gain);
    writeREG(ADS_CH5SET, ADS_gain);
    writeREG(ADS_CH6SET, ADS_gain);
    writeREG(ADS_CH7SET, ADS_gain);
    writeREG(ADS_CH8SET, ADS_gain);

    // disable all channels 1 = channel 1
    writeREG(ADS_RLD_SENSP, 0x00);
    //disable all channels
    writeREG(ADS_RLD_SENSN, 0x00);
    // ff enable 00 disable all channels
    writeREG(ADS_LOFF_SENSP, 0xff);
    // ff enable 00 disable all channels
    writeREG(ADS_LOFF_SENSN, 0xff);
    writeREG(ADS_LOFF_FLIP, 0x00);
    // 0F = default
    writeREG(ADS_GPIO, 0x0F);
    // Bit5: SRB1: 0 = open (default);
    writeREG(ADS_MISC1, 0x00);
    // always 0
    writeREG(ADS_MISC2, 0x00);

    delay(1);
    //writeREG(ADS_CONFIG4, 0x00); // edit
    writeREG(ADS_CONFIG4, 0x02); // (standard 0x00) turn on lead-off detection


    delay(100);

    readREG(ADS_ID, 0); //reg, number of registers -1
    readREG(ADS_CONFIG1, 0); //reg, number of registers -1
    printf("ADS_init_done\n" );
    return TRUE;
}

void start_ADS () {
    printf("start ADS\n");
    bcm2835_gpio_write(ADS_START_PIN, HIGH);

    bcm2835_spi_transfer(ADS_RDATAC);
    bcm2835_gpio_write(LED_POWER, LED_ON);

    //increase spi clk speed for RDATAC
    bcm2835_spi_setClockDivider(SPI_SPEED_FAST);

    // Enable Falling Edge Detect Enable for the specified pin
    bcm2835_gpio_afen(ADS_DRDY_PIN);

    ADS_STARTED = TRUE;
 }
void stop_ADS () {

    printf("stop ADS\n");
    bcm2835_gpio_write(ADS_START_PIN, LOW);
    bcm2835_spi_setClockDivider(SPI_SPEED_SLOW);
    bcm2835_spi_transfer(ADS_SDATAC);

    bcm2835_gpio_write(LED_POWER, LED_OFF);
    bcm2835_gpio_write(LED_STATUS, LED_OFF);
    ADS_STARTED = FALSE;


    printf("Closing window in 3 seconds!\n");
    delay(3000);
    exit(0);
}

void writeREG (char reg, char data) {
    //printf("write: %02X %02X \n",ADS_WREG | (reg & 0x1F),data );

    char buffer[] = {ADS_WREG | (reg & 0x1F), 0x00, data};

    bcm2835_spi_writenb (buffer, sizeof(buffer));
}
/*
    reg: register to read
    len: length of bytes to read -1
*/
void readREG (char reg, char len) {
    bcm2835_spi_transfer(ADS_SDATAC);

    delay(5);

    char buffer[] = {ADS_RREG | (reg & 0x1F), 0x00, 0x00};
    //char rbuffer[] = {0,0,0};
    bcm2835_spi_transfern(buffer, sizeof(buffer));

    for (int i=0;i<len+1;i++) {
        printf("REG:%02x: %02x\n",reg,buffer[i+2]);
    }
}
