#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <iostream>
#include "bcm2835-1.12/src/bcm2835.h"
#include "sched.h"

#define PIN_RESERVED 128

#define high true
#define low false

//TODO: make safety check if event is set and direction is in before registering irc
//TODO: unexport file

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
    bothEdges,
    noneEdges
};

enum pinState
{
    pinUninit,
    pinInput,
    pinOutput,
    pinError,
    pinReserved
};

class Pin
{
  private:
    pinState state;
    bool outputValue;
    unsigned int realGpioNumber;
    unsigned int boardPinNumber;
    eventType eventDetection;
  public:
    Pin(unsigned int gpioNbr, unsigned int boardNbr);
    Pin(pinState st, bool val, unsigned int gpioNbr, unsigned int boardNbr);

    void setState(pinState st) {state = st;}
    pinState getState(void) {return state;}
    void setOutput(bool val) {outputValue = val;}
    bool getOutput(void) {return outputValue;}
    void setMapping(unsigned int val) {realGpioNumber = val;}
    unsigned int getMapping(void) {return realGpioNumber;}
    void setBoardAssoc(unsigned int assoc) {boardPinNumber = assoc;}
    unsigned int getBoardAssoc(void) {return boardPinNumber;}
    void setEvent(eventType event) {eventDetection = event;}
    eventType getEvent(void) {return eventDetection;}

    void printState();
};

Pin::Pin(unsigned int gpioNbr, unsigned int boardNbr)
{
    realGpioNumber = gpioNbr;
    boardPinNumber = boardNbr;
    state = pinUninit;
    outputValue = low;
    eventDetection = noneEdges;
}

Pin::Pin(pinState st, bool val, unsigned int gpioNbr, unsigned int boardNbr) : 
	state(st), outputValue(val), realGpioNumber(gpioNbr), boardPinNumber(boardNbr), eventDetection(noneEdges)
{
}

void Pin::printState()
{
    using namespace std;
    cout << "| "<< 
      (((getState() == pinUninit) || (getState() == pinReserved)) ? 
         "---" : ((getState() == pinInput) ? "<--" : "-->")) << " | " << 
         (getOutput() == high ? '+' : '-') << " | " ;
    cout.width(3);
    cout << getMapping();
}

//------------------  this is for IRQ derived pin operations  ---------------------------
class IrqPin : public Pin
{
  private:
    int fileDesc;

    bool eventDetectionModify(eventType type);

  public:
    IrqPin(unsigned int gpioNbr, unsigned int boardNbr);
    IrqPin(pinState st, bool val, unsigned int gpioNbr, unsigned int boardNbr);
    void registerIrq();
    bool clearEventDetection(void);
    bool setEventDetection(eventType type);
    pinState getPinRealStatus(void);
    bool setPinOperationMode(pinState state);
    bool getRealPinValue(void);
    int getIrqFileDesc(void) {return fileDesc;};
};

#include <sstream>
#include <fstream>

// --------------   methods to be used with IRQ based GPIO readings ------------------------------
IrqPin::IrqPin(unsigned int gpioNbr, unsigned int boardNbr) : Pin(gpioNbr, boardNbr)
{
    setMapping(gpioNbr);
    setBoardAssoc(boardNbr);
    setState(getPinRealStatus());
    if (getState() == pinInput || getState() == pinOutput)
    {
        setOutput(getRealPinValue());
    }
    else
        setOutput(low);
}

IrqPin::IrqPin(pinState st, bool val, unsigned int gpioNbr, unsigned int boardNbr) : 
        Pin(st, val, gpioNbr, boardNbr)
{
    
};

void IrqPin::registerIrq()
{
    using namespace std;
  
    stringstream out;
    out << getMapping();
    
    string gpioFileName = "/sys/class/gpio/gpio" + out.str() + "/value";
    
    fileDesc = open(gpioFileName.c_str(), O_RDONLY);
    
    if (fileDesc < 0)
    {
        std::cout << "error opening gpio file " << std::endl;
    }
}

bool IrqPin::clearEventDetection(void)
{ 
    return eventDetectionModify(noneEdges);
};

bool IrqPin::setEventDetection(eventType type)
{
    return eventDetectionModify(type);
}

bool IrqPin::eventDetectionModify(eventType type)
{
    using namespace std;

    //it makes sense only to set event detection for input pin
    if (getState() != pinInput)
    {
        cout << "unable to set event detection for NOT input pin" << endl;
        return false;
    }

    stringstream out;
    out << getMapping();

    string edgeFileName = "/sys/class/gpio/gpio" + out.str() + "/edge";

    ofstream edgeFile;
    edgeFile.open(edgeFileName.c_str(), ios::in);

    if (edgeFile.is_open() == false)
    {
        cout << "error opening file for setting gpio edge: " << edgeFileName << endl;
        return false;
    }
    
    switch (type)
    {
        case risingEdge:
            edgeFile << "rising";
            break;
        case fallingEdge:
            edgeFile << "falling";
            break;
        case bothEdges:
            edgeFile << "both";
            break;
        case noneEdges:
            edgeFile << "none";
            break;
    }
    setEvent(type);
    edgeFile.close();
    return true;
};

pinState IrqPin::getPinRealStatus(void)
{
    using namespace std;

    stringstream out;
    out << getMapping();

    string gpioName = "/sys/class/gpio/gpio" + out.str() + "/direction";
    ifstream gpioFile(gpioName.c_str());

    if (gpioFile.good())
    {
        if ((gpioFile.is_open() == false))
        {
            cout << "error occured while opening file: " +  gpioName << endl;
        }
        string pinStatus;
        gpioFile >> pinStatus;
        gpioFile.close();

        if (pinStatus.compare("in") == 0)
            return pinInput;
        else if (pinStatus.compare("out") == 0)
            return pinOutput;
        else
        {
            cout << "unknown pin " << getBoardAssoc() << " state: " << pinStatus << endl;
            return pinError;
        }
    }
    return pinUninit;
}

bool IrqPin::setPinOperationMode(pinState state)
{
    using namespace std;

    string fileName = "/sys/class/gpio/";

    //check if already exported
    pinState actualState = getPinRealStatus();

    //don't have to change state
    if (actualState == state)
        return true;

    //TODO: changing state from in to out 

    if ((state == pinInput || state == pinOutput) && actualState != pinUninit)
    {
        cout << "unable to change state of pin: " << getBoardAssoc() 
             << " from " << actualState << " to " << state << endl;
        return false;
    }

    if (state == pinUninit)
    {
        if (actualState != pinInput && actualState != pinOutput)
        {
            cout << "unable to uninitialize pin: " << getBoardAssoc() << " from " << actualState <<  endl;
            return false;
        }
        fileName += "unexport";
        ofstream unexportFile;
        unexportFile.open(fileName.c_str(), ios::in);
        if (unexportFile.is_open() == false)
        {
            cout << "error opening file for unexporting gpio" << endl;
            return false;
        }
        unexportFile << getMapping();
        unexportFile.close();
        return true;
    }
   
    fileName += "export";
    
    ofstream exportFile;
    exportFile.open(fileName.c_str()/*, ios::in*/);

    if (!exportFile.is_open())
    {
        cout << "error opening file for exporting gpio [" << fileName << "]" << endl;
        return false;
    }
    exportFile << getMapping();
    exportFile.close();

    stringstream out;
    out << getMapping();

    string directionFileName = "/sys/class/gpio/gpio" + out.str() + "/direction";

    ofstream directionFile;
    directionFile.open(directionFileName.c_str()/*, ios::in*/);

    if (directionFile.is_open() == false)
    {
        cout << "error opening file for setting gpio direction" << endl;
        return false;
    }
    directionFile << ((state == pinInput) ? "in" : "out");
    directionFile.close();
    setState(state);
    return true;
}

bool IrqPin::getRealPinValue(void)
{
    using namespace std;

    pinState state = getPinRealStatus();
    if (state == pinInput || state == pinOutput)
    {
        stringstream out;
        out << getMapping();

        string fileName = "/sys/class/gpio/gpio" + out.str() + "/value";
        ifstream valueFile;
        valueFile.open(fileName.c_str(), ios::out);
        if (valueFile.is_open() == false)
        {
            cout << "error reading value from file " + fileName << endl;
        }
        string value;
        valueFile >> value;

        valueFile.close();

        if (value.compare("1") == 0)
            return high;
        else if (value.compare("0") == 0)
            return low;

    }
    return false;
}

//----------------  board controll class  ----------------------------------
class piBase
{
  private:
    volatile unsigned *gpio;

    bool initialized;
    int version;
    Pin *pins[26];
    
    void fillPinsIrq();   
    bool setupGpios();
    
  
  public:
    static unsigned gpioController;
    static unsigned socBCM2708priferials;
    static unsigned blockSize;
    static unsigned pageSize;

    static unsigned boardV2pin[];
    static unsigned boardV1pin[];

    piBase(int, bool);
    ~piBase();

    void setPinInput(boardPin pin);
    void setPinOutput(boardPin pin);
    void setPinInputPullUp(boardPin pin);

    void turnOn(boardPin pin);
    void turnOff(boardPin pin);
    void toggleOutput(boardPin pin);

    bool readInput(boardPin pin);

    bool clearEventDetection(boardPin pin);
    bool setEventDetection(boardPin pin, eventType type);
    void waitForEvent(boardPin pin);

    void printPinsStatus();
    Pin *getPin(boardPin pin);
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

Pin *piBase::getPin(boardPin pin)
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
        pins[i]->printState();
        std::cout << std::endl;
    }
};

void piBase::fillPinsIrq()
{
    for (int i = 0; i < 26; i++)
    {
        if (version == 1)
        {
            if (boardV1pin[i] != PIN_RESERVED)
                pins[i] = new IrqPin(boardV1pin[i], i);
            else
                pins[i] = new IrqPin(pinReserved, low, boardV1pin[i], i);
        }
        else
        {
            if (boardV1pin[i] != PIN_RESERVED)
                pins[i] = new IrqPin(boardV2pin[i], i);
            else
                pins[i] = new IrqPin(pinReserved, low, boardV2pin[i], i);
        }
    }
};

piBase::piBase(int ver, bool useIrq)
{
    version = ver;

    if (useIrq)
    {
        fillPinsIrq();
        initialized = true;
        return;
    }

//    if (bcm2835_init())
    {
        initialized = true;
        //fillPins();
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
    if (initialized && pins[pin]->getMapping() != PIN_RESERVED)
    {
//        bcm2835_gpio_fsel(pins[pin].getMapping(), BCM2835_GPIO_FSEL_INPT);
        pins[pin]->setState(pinInput);
    }
    else
        std::cout << "input error: " << pin + 1 << std::endl;
};

void piBase::setPinInputPullUp(boardPin pin)
{
    if (initialized && pins[pin]->getMapping() != PIN_RESERVED 
		&& pins[pin]->getState() == pinOutput)
    {
//        bcm2835_gpio_set_pud(pins[pin].getMapping(), BCM2835_GPIO_FSEL_INPT);
    }
    else
        std::cout << "input pull up error: " << pin + 1 << std::endl;
};

void piBase::setPinOutput(boardPin pin)
{
    if (initialized && pins[pin]->getMapping() != PIN_RESERVED)
    {
//        bcm2835_gpio_fsel(pins[pin].getMapping(), BCM2835_GPIO_FSEL_OUTP);
        pins[pin]->setState(pinOutput);
    }
};

void piBase::turnOff(boardPin pin)
{
    if (pins[pin]->getState() == pinOutput)
    {
//        bcm2835_gpio_write(pins[pin].getMapping(), LOW);
        pins[pin]->setOutput(low);
    }
    else
        std::cout << "turn off error: " << pin + 1 << std::endl;
};

void piBase::turnOn(boardPin pin)
{
    if (pins[pin]->getState() == pinOutput)
    {
//        bcm2835_gpio_write(pins[pin].getMapping(), HIGH);
        pins[pin]->setOutput(high);
    }
    else
        std::cout << "turn on error: " << pin + 1 << std::endl;
};

void piBase::toggleOutput(boardPin pin)
{
    if (pins[pin]->getState() == pinOutput)
    {
        if (pins[pin]->getOutput() == high)
            ;//bcm2835_gpio_write(pins[pin].getMapping(), LOW);
        else
            ;//bcm2835_gpio_write(pins[pin].getMapping(), HIGH);

        pins[pin]->setOutput(!pins[pin]->getOutput());
    }
};


//-------------------------------------------------
void piBase::waitForEvent(boardPin pin)
{
    if (bcm2835_gpio_eds(pins[pin]->getMapping()))
    {
        std::cout << "have event for pin: " << pin + 1 << std::endl;
        bcm2835_gpio_set_eds(pins[pin]->getMapping());
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
    uint8_t value = bcm2835_gpio_lev(pins[pin]->getMapping());
    return value == 0 ? false : true;
};


void *print_input(int fd, epollEvents event, void* data)
{
    Pin *pin  = (Pin*)data;

    int value = 0;
    char rdbuf[5] = {'\0'};

    int ret = read(fd, rdbuf, sizeof rdbuf - 1);
    if (ret > 0)
    {
        std::cout << "have input value: " << rdbuf << " from pin " << pin->getBoardAssoc() << std::endl;
        lseek(fd, 0, SEEK_SET);
    }
};

#include <time.h>
#include <sys/time.h>

int try_recognize_button(int *button_recognition, int size)
{
    if (button_recognition[0] < 269 || button_recognition[0] > 282)
        return -1;

    if (button_recognition[0] == 275 || button_recognition[0] == 276)
    {
        if (button_recognition[1] == 11 || button_recognition[1] == 12)
            return 1;
        else if (button_recognition[1] == 17 || button_recognition[1] == 18 || button_recognition[1] == 19)
        {
             if (button_recognition[2] == 16 || button_recognition[2] == 17 || button_recognition[0] == 18)
                 return 2;
             else if (button_recognition[2] == 10 || button_recognition[2] == 11 || button_recognition[2] == 12)
                 return 5;
        }
    }
    else if (button_recognition[0] == 269 || button_recognition[0] == 270)
        return 3;
    else if (button_recognition[0] == 281 || button_recognition[0] == 282)
        return 4;
    else
        return -1;
}

int recognize_buton(int *button)
{
    if (button[0] == button[1] && button[1] == button[2])
        return button[0];
    if (button[1] == button[2] && button[2] == button[3])
        return button[1];
    if (button[0] == button[2] && button[2] == button[3])
        return button[0];
    if (button[0] == button[1] && button[1] == button[3])
        return button[0];
    return -1;
}

//CLOCKS_PER_SEC --> 1000000
void *print_input_irda(int fd, epollEvents event, void* data)
{
    //Pin *pin  = (Pin*)data;

    //int value = 0;
    //char rdbuf[5] = {'\0'};

    static int buttonRecognition[4] = {0};
    static int buttonKnown[4] = {0};
    static int buttonRecognitionCounter = 0;

    static int knownButtonCounter = 0;
   
    static timeval time = {0};
    timeval old_time; old_time.tv_sec = time.tv_sec; old_time.tv_usec = time.tv_usec;
    gettimeofday(&time, NULL);
    //int clk = clock();
    int carry = 0;
    int timeDiffUSec = time.tv_usec - old_time.tv_usec;
    if (timeDiffUSec < 0)
    {
        timeDiffUSec = 1000000 - old_time.tv_usec + time.tv_usec;
        carry = 1;
    }

    long time_diff = (time.tv_sec - old_time.tv_sec - carry) * 1000000L + (time.tv_usec - old_time.tv_usec);

    //if (time_diff % 250 > 210)
    //    time_diff += 50;

    if (time_diff > 80 * 250)
    {
//        std::cout << std::endl;
        buttonRecognitionCounter = 0;
    }
    if (buttonRecognitionCounter < 4)
    {
        buttonRecognition[buttonRecognitionCounter++] = (time_diff / 100);
    }
    else if (buttonRecognitionCounter == 4)
    {
//        std::cout << buttonRecognition[0] << " " << buttonRecognition[1] << " " << buttonRecognition[2] << " " << buttonRecognition[3] << " " << std::endl;
        buttonKnown[knownButtonCounter++] = try_recognize_button(buttonRecognition, 4);

        if (knownButtonCounter == 4)
        {
            int button = recognize_buton(buttonKnown);
            std::cout << "found button " << button << std::endl;
            knownButtonCounter = 0;
        }
        buttonRecognitionCounter++;
    }

    //std::cout << time_diff / 100 << " ";
    if (time_diff < 0)
{
    std::cout << time.tv_sec << " " << time.tv_usec << " " << std::endl;
    std::cout << old_time.tv_sec << " " << old_time.tv_usec << " " << std::endl;
}
    //std::cout << "---------------------------" << std::endl;
    //int ret = read(fd, rdbuf, sizeof rdbuf - 1);
    //if (ret > 0)
    //{
    //    std::cout << rdbuf[0] << " -> " << time_diff << " " << time_diff / 250 << std::endl;
    //    lseek(fd, 0, SEEK_SET);
    //}
    //else
    //    std::cout << "problem reading file" << std::endl;
};


int main(int argc, char *argv[])
{
    piBase raspberry(1, true);

    raspberry.printPinsStatus();

    IrqPin *ipin11 = static_cast<IrqPin*>(raspberry.getPin(pin11));
    ipin11->setPinOperationMode(pinInput);
    ipin11->registerIrq();

    //raspberry.setPinOperationMode(pin11, pinInput);

    IrqPin *ipin12 = static_cast<IrqPin*>(raspberry.getPin(pin12));
    ipin12->setPinOperationMode(pinInput);
    ipin12->setEventDetection(risingEdge);
    ipin12->registerIrq();

    //raspberry.setPinOperationMode(pin12, pinInput);
    //raspberry.setEventDetection(pin12, bothEdges);
    //raspberry.setPinInputPullUp(pin3);
//    raspberry.setPinOperationMode(pin13, pinInput);
    //raspberry.setPinInput(pin5);
    //raspberry.setPinInput(pin7);

    //raspberry.setPinOutput(pin15);
    //raspberry.setPinOutput(pin16);
    //raspberry.setPinOutput(pin18);

    //raspberry.turnOn(pin4);
    //raspberry.turnOn(pin5);
    //raspberry.turnOn(pin13);

    
    epollScheduler es(16);
    
    epollItem item;
    item.event = epollRead;
    if ((item.fd = ipin11->getIrqFileDesc()) > 0)
    {
        //TODO:
        std::cout << "registring pin11 with fd " <<  item.fd << " for IRQ success" << std::endl;
    }
    item.callback = print_input;
    item.data = (void*)ipin11;

    epollItem item2;
    item2.event = epollRead;
    if ((item2.fd = ipin12->getIrqFileDesc()) > 0)
    {
        //TODO:
        std::cout << "registring pin12 with fd " <<  item2.fd << " for IRQ success" << std::endl;
    }
    item2.callback = print_input_irda;
    item2.data = (void*)ipin12;
    
    es.addItem(item);
    es.addItem(item2);

    es.addTimer(10000);

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


