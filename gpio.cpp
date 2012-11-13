#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <iostream>
#include "bcm2835-1.12/src/bcm2835.h"
#include "sched.h"

#define PIN_RESERVED 128

//this is based on TOP view of RPi board
// +---+---+
// | 1 | 2 |
// +---+---+
// | 3 | 4 |
// +---+---+
//    ...
enum boardPin
{
    pin1,
    pin2,
    pin3,
    pin4,
    pin5,
    pin6,
    pin7,
    pin8,
    pin9,
    pin10,
    pin11,
    pin12,
    pin13,
    pin14,
    pin15,
    pin16,
    pin17,
    pin18,
    pin19,
    pin20,
    pin21,
    pin22,
    pin23,
    pin24,
    pin25,
    pin26
};

enum eventType
{
    risingEdge,
    fallingEdge,
    highLevel,
    lowLevel
};

enum pinState
{
    pinStateUninit,
    pinStateInput,
    pinStateOutput
};


class Pin
{
  private:
    pinState state;
    bool output;
    unsigned int mapping;
  public:
    int number;
 
    Pin();
    //~Pin();
    void setState(pinState st) {state = st;};
    pinState getState(void) {return state;}
    void setOutput(bool val) {output = val;};
    bool getOutput(void) {return output;};
    void setMapping(unsigned int val) {mapping = val;};
    unsigned int getMapping(void) {return mapping;};

    void printState();
};

Pin::Pin()
{
    state = pinStateUninit;
    output = false;
};

void Pin::printState()
{
    using namespace std;
    cout << "| "<< 
      (this->getState() == pinStateUninit ? "-u-" : (this->getState() == pinStateInput ? "<--" : "-->")) << " | " << 
      (this->getOutput() == true ? '+' : '-') << " | " ;
    cout.width(3);
    cout << this->getMapping();
}

class piBase
{
  private:
    volatile unsigned *gpio;

    bool initialized;
    int version;
    Pin pins[26];
    
    void fillPins();
    
    bool setupGpios();
    
  
  public:
    static unsigned gpioController;
    static unsigned socBCM2708priferials;
    static unsigned blockSize;
    static unsigned pageSize;

    static unsigned boardV2pin[];
    static unsigned boardV1pin[];

    piBase(int ver);
    ~piBase();

    void setPinInput(boardPin pin);
    void setPinOutput(boardPin pin);
    void setPinInputPullUp(boardPin pin);

    void turnOn(boardPin pin);
    void turnOff(boardPin pin);
    void toggleOutput(boardPin pin);

    bool readInput(boardPin pin);
    int registerIrq(boardPin pin);

    bool clearEventDetection(boardPin pin, eventType type);
    bool setEventDetection(boardPin pin, eventType type);
    void waitForEvent(boardPin pin);

    void printPinsStatus();
    Pin &getPin(boardPin pin);
};

unsigned int piBase::boardV1pin[] = 
{
    PIN_RESERVED,	//3v3
    PIN_RESERVED,	//5v
    RPI_GPIO_P1_03,
    PIN_RESERVED,	//5v
    RPI_GPIO_P1_05,
    PIN_RESERVED,	//GND
    RPI_GPIO_P1_07,
    RPI_GPIO_P1_08,	//defaults to alt function 0 UART0_TXD
    PIN_RESERVED,	//GND
    RPI_GPIO_P1_10,	//defaults to alt function 0 UART0_RXD
    RPI_GPIO_P1_11,
    RPI_GPIO_P1_12,
    RPI_GPIO_P1_13,
    PIN_RESERVED,	//GND
    RPI_GPIO_P1_15,
    RPI_GPIO_P1_16,
    PIN_RESERVED,	//3v3
    RPI_GPIO_P1_18,
    RPI_GPIO_P1_19,	//MOSI when SPI0 in use
    PIN_RESERVED,	//GND
    RPI_GPIO_P1_21,	//MOS0 when SPI0 in use
    RPI_GPIO_P1_22,
    RPI_GPIO_P1_23,	//CLK when SPI0 in use
    RPI_GPIO_P1_24,	//CE0 when SPI0 in use
    PIN_RESERVED,	//GND
    RPI_GPIO_P1_26	//CE1 when SPI0 in use
};

unsigned int piBase::boardV2pin[] = 
{
    PIN_RESERVED,	 //3v3
    PIN_RESERVED,	 //5v
    RPI_V2_GPIO_P1_03,
    PIN_RESERVED,	 //5v
    RPI_V2_GPIO_P1_05,
    PIN_RESERVED,	 //GND
    RPI_V2_GPIO_P1_07,
    RPI_V2_GPIO_P1_08,	 //defaults to alt function 0 UART0_TXD
    PIN_RESERVED,	 //GND
    RPI_V2_GPIO_P1_10,   //defaults to alt function 0 UART0_RXD
    RPI_V2_GPIO_P1_11,
    RPI_V2_GPIO_P1_12,
    RPI_V2_GPIO_P1_13,
    PIN_RESERVED,	 //GND
    RPI_V2_GPIO_P1_15,
    RPI_V2_GPIO_P1_16,
    PIN_RESERVED,	 //3v3
    RPI_V2_GPIO_P1_18,
    RPI_V2_GPIO_P1_19,   //MOSI when SPI0 in use
    PIN_RESERVED,	 //GND
    RPI_V2_GPIO_P1_21,   //MOS0 when SPI0 in use
    RPI_V2_GPIO_P1_22,
    RPI_V2_GPIO_P1_23,   //CLK when SPI0 in use
    RPI_V2_GPIO_P1_24,   //CE0 when SPI0 in use
    PIN_RESERVED,	 //GND
    RPI_V2_GPIO_P1_26	 //CE1 when SPI0 in use
};



unsigned piBase::socBCM2708priferials = 0x20000000;
unsigned piBase::gpioController = piBase::socBCM2708priferials + 0x20000000;
unsigned piBase::blockSize = 4 * 1024;
unsigned piBase::pageSize = 4 * 1024;

Pin &piBase::getPin(boardPin pin)
{
    return pins[pin];
};

void piBase::printPinsStatus()
{
    using namespace std;

    for (int i = 0; i < 26; i++)
    {
        std::cout << "| ";
        cout.width(3);
        cout << i + 1;
        pins[i].printState();
        std::cout << std::endl;
    }
};

void piBase::fillPins()
{
    for (int i = 0; i < 26; i++)
    {
        if (version == 1)
        {
            pins[i].setMapping(boardV1pin[i]);
        }
        else
        {
            pins[i].setMapping(boardV2pin[i]);
        }
        pins[i].setState(pinStateUninit);
        pins[i].setOutput(false);
        pins[i].number = i;
    }
};

piBase::piBase(int ver)
{
    version = ver;        

//    if (bcm2835_init())
    {
        initialized = true;
        fillPins();
        return;
    }
    initialized = false;
};

piBase::~piBase()
{
//    bcm2835_close();
}

void piBase::setPinInput(boardPin pin)
{
    if (initialized && pins[pin].getMapping() != PIN_RESERVED)
    {
//        bcm2835_gpio_fsel(pins[pin].getMapping(), BCM2835_GPIO_FSEL_INPT);
        pins[pin].setState(pinStateInput);
    }
    else
        std::cout << "input error: " << pin + 1 << std::endl;
};

void piBase::setPinInputPullUp(boardPin pin)
{
    if (initialized && pins[pin].getMapping() != PIN_RESERVED 
		&& pins[pin].getState() == pinStateOutput)
    {
//        bcm2835_gpio_set_pud(pins[pin].getMapping(), BCM2835_GPIO_FSEL_INPT);
    }
    else
        std::cout << "input pull up error: " << pin + 1 << std::endl;
};

void piBase::setPinOutput(boardPin pin)
{
    if (initialized && pins[pin].getMapping() != PIN_RESERVED)
    {
//        bcm2835_gpio_fsel(pins[pin].getMapping(), BCM2835_GPIO_FSEL_OUTP);
        pins[pin].setState(pinStateOutput);
    }
};

void piBase::turnOff(boardPin pin)
{
    if (pins[pin].getState() == pinStateOutput)
    {
//        bcm2835_gpio_write(pins[pin].getMapping(), LOW);
        pins[pin].setOutput(false);
    }
    else
        std::cout << "turn off error: " << pin + 1 << std::endl;
};

void piBase::turnOn(boardPin pin)
{
    if (pins[pin].getState() == pinStateOutput)
    {
//        bcm2835_gpio_write(pins[pin].getMapping(), HIGH);
        pins[pin].setOutput(true);
    }
    else
        std::cout << "turn on error: " << pin + 1 << std::endl;
};

void piBase::toggleOutput(boardPin pin)
{
    if (pins[pin].getState() == pinStateOutput)
    {
        if (pins[pin].getOutput() == true)
            ;//bcm2835_gpio_write(pins[pin].getMapping(), LOW);
        else
            ;//bcm2835_gpio_write(pins[pin].getMapping(), HIGH);

        pins[pin].setOutput(!pins[pin].getOutput());
    }
};


//-------------------------------------------------
bool piBase::clearEventDetection(boardPin pin, eventType type)
{
    return false;
};

bool piBase::setEventDetection(boardPin pin, eventType type)
{
    switch (type)
    {
        case risingEdge:
            break;
        case lowLevel:
            //bcm2835_gpio_len(pins[pin].getMapping());
            break;
    }
    return true;
};

void piBase::waitForEvent(boardPin pin)
{
    if (bcm2835_gpio_eds(pins[pin].getMapping()))
    {
        std::cout << "have event for pin: " << pin + 1 << std::endl;
        bcm2835_gpio_set_eds(pins[pin].getMapping());
    }
};

bool piBase::setupGpios()
{
    int memory;
    if ((memory = open("/dev/mem", O_RDWR|O_SYNC)) < 0)
    {
        return false;
    }
    char *gpioMemory = new char[blockSize + pageSize - 1];

    //set pointer to 4K boundary
    if ((unsigned long)gpioMemory % pageSize)
    {
        gpioMemory += pageSize - ((unsigned long)gpioMemory % pageSize);
    }

    unsigned char *gpioMap = 
        (unsigned char*)mmap(gpioMemory, blockSize, PROT_READ|PROT_WRITE, 
                             MAP_SHARED|MAP_FIXED, memory, gpioController);

    if (gpioMap == MAP_FAILED)
    {
        close(memory);
        return false;
    }
    gpio = (volatile unsigned*)gpioMap;
    
    return true;
};
//-------------------------------------------------------

bool piBase::readInput(boardPin pin)
{
    uint8_t value = bcm2835_gpio_lev(pins[pin].getMapping());
    return value == 0 ? false : true;
};

#include <sstream>

//# Set up GPIO 4 and set to output
//echo "4" > /sys/class/gpio/export
//echo "out" > /sys/class/gpio/gpio4/direction

//# Clean up
//echo "4" > /sys/class/gpio/unexport
//echo "7" > /sys/class/gpio/unexport
int piBase::registerIrq(boardPin pin)
{
    using namespace std;
  
    stringstream out;
    out << pins[pin].getMapping();
    
    string gpioFileName = "/sys/class/gpio/gpio" + out.str() + "/value";

    cout << "file to open: " << gpioFileName << std::endl;
    
    int fd = open(gpioFileName.c_str(), O_RDONLY);
    
    if (fd < 0)
    {
        std::cout << "error opening gpio file for pin: " << pin + 1 << std::endl;
        return -1;
    }
    
    return fd;
}


void *print_input(int fd, epollEvents event, void* data)
{
    Pin *pin  = (Pin*)data;

    int value = 0;
    char rdbuf[5] = {'\0'};

    int ret = read(fd, rdbuf, sizeof rdbuf - 1);
    if (ret > 0)
    {
        std::cout << "have input value: " << rdbuf << " from pin " << pin->number << std::endl;
        lseek(fd, 0, SEEK_SET);
    }
};


int main(int argc, char *argv[])
{
    piBase raspberry(1);

    raspberry.printPinsStatus();

    raspberry.setPinInput(pin3);
    raspberry.setPinInputPullUp(pin3);
    raspberry.setPinInput(pin4);
    raspberry.setPinInput(pin5);
    raspberry.setPinInput(pin7);

    raspberry.setPinOutput(pin13);
    raspberry.setPinOutput(pin15);

    raspberry.turnOn(pin4);
    raspberry.turnOn(pin5);
    raspberry.turnOn(pin13);

    
    epollScheduler es(16);
    
    epollItem item;
    item.event = epollRead;
    if ((item.fd = raspberry.registerIrq(pin13)) > 0)
    {
        //TODO:
    }
    item.callback = print_input;
    item.data = (void*)&raspberry.getPin(pin13);
    
    es.addItem(item);
    es.addTimer(5000);

    es.run();

    /*while (1)
    {
        bool value = raspberry.readInput(pin3);
        std::cout << "read: " << value << std::endl;
        delay(500);
    }*/

    std::cout << std::endl << std::endl;

    raspberry.printPinsStatus();

    return 0;
}


