#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <string.h>

#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Watchdog.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/cpu.h)
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)
#include DeviceFamily_constructPath(driverlib/aon_batmon.h)

#include <ti/devices/cc13x0/driverlib/aux_adc.h>

/* Board Header files */
#include "Board.h"

/* Application Header files */
#include "SensorTask.h"
#include "Protocol.h"

#define SENSOR_TASK_STACK_SIZE 1024
#define SENSOR_TASK_TASK_PRIORITY   3

#define SENSOR_EVENT_ALL                         0xFFFFFFFF

#define BV(n)               (1 << (n))

#define TIMER_TIMEOUT 1000

#define KXTJ3_ADDRESS 0x0E

#define ON_OFF_WAIT_COUNT 3

#define SCALE_VALUE 15.987f

const PIN_Config sensorTaskPinTable[] = { LED_RED_GPIO | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
LED_BLUE_GPIO | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
HALL_SENSOR_GPIO | PIN_INPUT_EN | PIN_NOPULL,
PIN_TERMINATE
};

static PIN_Handle sensorTaskPinHandle;
static PIN_State sensorTaskPinState;

static I2C_Handle i2c;
static I2C_Params i2cParams;

static Task_Params sensorTaskParams;
Task_Struct sensorTask; /* not static so you can see in ROV */
static uint8_t sensorTaskStack[SENSOR_TASK_STACK_SIZE];

Event_Struct sensorTaskEvent; /* not static so you can see in ROV */
static Event_Handle sensorTaskEventHandle;

Clock_Struct sensorTimerClock;

static PacketSendRequestCallback packetSendRequestCallback;

static volatile uint8_t on_off_mode = 0x01;
static volatile uint8_t start_gps_state = 0x00;
static volatile uint8_t start_radio_tx_state = 0x00;

static uint8_t payload_buffer_size = 0;
static uint8_t payload_buffer[115] = { 0x00 };

static uint8_t battery_voltage = 0;

static uint8_t temperature = 0;

static int16_t max_ax = 0;
static int16_t max_ay = 0;
static int16_t max_az = 0;

extern uint8_t radio_init;
extern uint8_t mac_address[8];
extern uint32_t collection_cycle_timeout_count;
extern uint32_t collection_cycle_timer_count;

static void wait_ms(uint32_t wait)
{

    Task_sleep(wait * 1000 / Clock_tickPeriod);

}

void scCtrlReadyCallback(void)
{

}

void scTaskAlertCallback(void)
{

}

void watchdogCallback(uintptr_t watchdogHandle)
{

    while (1)
    {

    }

}

static void sensorTimerClockCallBack(UArg arg0)
{

    collection_cycle_timeout_count++;

    if (on_off_mode == 0x01)
    {

        if (collection_cycle_timeout_count >= collection_cycle_timer_count)
        {

            start_radio_tx_state = 0x01;

            collection_cycle_timeout_count = 0;

        }

    }
    else
    {

        collection_cycle_timeout_count = 0;

    }

}

static void initErrorUpdate(void)
{

    for (uint8_t i = 0; i < 4; i++)
    {

        PIN_setOutputValue(sensorTaskPinHandle, LED_RED_GPIO,
                           !PIN_getOutputValue(LED_RED_GPIO));
        PIN_setOutputValue(sensorTaskPinHandle, LED_BLUE_GPIO,
                           !PIN_getOutputValue(LED_BLUE_GPIO));

        wait_ms(500);

    }

    PIN_setOutputValue(sensorTaskPinHandle, LED_RED_GPIO, 0);
    PIN_setOutputValue(sensorTaskPinHandle, LED_BLUE_GPIO, 0);

}

static uint8_t init_kxtj3()
{

    uint8_t i2c_state = 0x01;

    uint8_t scan_count = 0;

    uint8_t txBuffer[2] = { 0x00 };
    uint8_t rxBuffer[6] = { 0x00 };

    I2C_Transaction i2cTransaction;

    I2C_init();
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(Board_I2C0, &i2cParams);

    if (i2c == NULL)
    {

        SysCtrlSystemReset();

    }

    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readCount = 0;

    for (uint8_t i = 0; i < 128; i++)
    {

        i2cTransaction.slaveAddress = i;
        txBuffer[0] = 0x00;

        if (I2C_transfer(i2c, &i2cTransaction))
        {

            scan_count++;

        }

    }

//KXTJ3 Setting
    i2cTransaction.slaveAddress = KXTJ3_ADDRESS;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.readBuf = rxBuffer;

//Reset
    txBuffer[0] = 0x1D;
    txBuffer[1] = (1 << 7);
    i2cTransaction.writeCount = 2;
    i2cTransaction.readCount = 0;
    i2c_state = i2c_state & I2C_transfer(i2c, &i2cTransaction);

    //Sleep 500ms
    wait_ms(500);

    //Standby Mode, 2G
    txBuffer[0] = 0x1B;
    txBuffer[1] = 0b00000000;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readCount = 0;
    i2c_state = i2c_state & I2C_transfer(i2c, &i2cTransaction);

    //Sleep 500ms
    wait_ms(500);

    //SampleRate 12.5Hz
    txBuffer[0] = 0x21;
    txBuffer[1] = 0x00;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readCount = 0;
    i2c_state = i2c_state & I2C_transfer(i2c, &i2cTransaction);

    //Motion detection disable
    txBuffer[0] = 0x1F;
    txBuffer[1] = 0x00;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readCount = 0;
    i2c_state = i2c_state & I2C_transfer(i2c, &i2cTransaction);

    //Operating Mode, 2G
    txBuffer[0] = 0x1B;
    txBuffer[1] = 0b10000000;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readCount = 0;
    i2c_state = i2c_state & I2C_transfer(i2c, &i2cTransaction);

    //Sleep 200ms
    wait_ms(200);

    return i2c_state;

}

static uint8_t update_kxtj3(int16_t *ax, int16_t *ay, int16_t *az)
{

    uint8_t i2c_state = 0x01;
    uint8_t txBuffer[2] = { 0x00 };
    uint8_t rxBuffer[6] = { 0x00 };

    I2C_Transaction i2cTransaction;

    txBuffer[0] = 0x06;

    i2cTransaction.slaveAddress = KXTJ3_ADDRESS;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.readBuf = rxBuffer;

    i2cTransaction.writeCount = 1;
    i2cTransaction.readCount = 6;

    i2c_state = i2c_state & I2C_transfer(i2c, &i2cTransaction);

    *ax = (int8_t) rxBuffer[1] * SCALE_VALUE;
    *ay = (int8_t) rxBuffer[3] * SCALE_VALUE;
    *az = (int8_t) rxBuffer[5] * SCALE_VALUE;

    return i2c_state;

}

static void sensorTaskFunction(UArg arg0, UArg arg1)
{

    uint8_t on_off_count = 0;

    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;

    I2C_init();

    sensorTaskPinHandle = PIN_open(&sensorTaskPinState, sensorTaskPinTable);
    PIN_setOutputValue(sensorTaskPinHandle, LED_RED_GPIO, 0);
    PIN_setOutputValue(sensorTaskPinHandle, LED_BLUE_GPIO, 0);

    if (!init_kxtj3())
    {

        initErrorUpdate();

    }

    PIN_setOutputValue(sensorTaskPinHandle, LED_RED_GPIO, 1);
    PIN_setOutputValue(sensorTaskPinHandle, LED_BLUE_GPIO, 1);

    AONBatMonEnable();

    while (1)
    {

        if (!PIN_getInputValue(HALL_SENSOR_GPIO))
        {

            if (on_off_mode == 0x01)
            {

                PIN_setOutputValue(sensorTaskPinHandle, LED_RED_GPIO, 0);
                PIN_setOutputValue(sensorTaskPinHandle, LED_BLUE_GPIO, 1);

            }
            else
            {

                PIN_setOutputValue(sensorTaskPinHandle, LED_RED_GPIO, 1);
                PIN_setOutputValue(sensorTaskPinHandle, LED_BLUE_GPIO, 0);

            }

            on_off_count += 1;

            if (on_off_count > ON_OFF_WAIT_COUNT)
            {

                on_off_mode = !on_off_mode;
                on_off_count = 0;

                PIN_setOutputValue(sensorTaskPinHandle, LED_RED_GPIO, 0);
                PIN_setOutputValue(sensorTaskPinHandle, LED_BLUE_GPIO, 0);

                wait_ms(500);

                PIN_setOutputValue(sensorTaskPinHandle, LED_RED_GPIO, 1);
                PIN_setOutputValue(sensorTaskPinHandle, LED_BLUE_GPIO, 1);

            }

        }
        else
        {

            PIN_setOutputValue(sensorTaskPinHandle, LED_RED_GPIO, 1);
            PIN_setOutputValue(sensorTaskPinHandle, LED_BLUE_GPIO, 1);

            on_off_count = 0;

        }

        if (on_off_mode == 0x01)
        {

            update_kxtj3(&ax, &ay, &az);

            uint32_t vector = powf(ax, 2);
            vector += powf(ay, 2);
            vector += powf(az, 2);
            vector = sqrtf(vector);

            uint32_t max_vector = powf(max_ax, 2);
            max_vector += powf(max_ay, 2);
            max_vector += powf(max_az, 2);
            max_vector = sqrtf(max_vector);

            if (vector > max_vector)
            {

                max_ax = ax;
                max_ay = ay;
                max_az = az;

            }

            if (start_radio_tx_state)
            {

                if (AONBatMonNewBatteryMeasureReady())
                {

                    uint32_t batteryAdcValue = AONBatMonBatteryVoltageGet();
                    batteryAdcValue = (batteryAdcValue * 125) >> 5;

                    float batteryConvert = (float) batteryAdcValue / 100.0f;
                    batteryConvert = roundf(batteryConvert);

                    if (batteryConvert > 0)
                    {

                        battery_voltage = (uint8_t) batteryConvert;

                    }

                }

                if (AONBatMonNewTempMeasureReady())
                {

                    temperature =
                    AONBatMonTemperatureGetDegC();

                    if (temperature > INT8_MAX)
                    {

                        temperature = INT8_MAX;

                    }
                    else if (temperature < INT8_MIN)
                    {

                        temperature = INT8_MIN;

                    }

                }

                payload_buffer_size = 0;
                memset(payload_buffer, 0x00, sizeof(payload_buffer));

                payload_buffer_size = sprintf((char*) payload_buffer, "%d.%d,",
                                              (battery_voltage / 10),
                                              (battery_voltage % 10));

                payload_buffer_size += sprintf(
                        (char*) payload_buffer + payload_buffer_size, "%d,",
                        max_ax);

                payload_buffer_size += sprintf(
                        (char*) payload_buffer + payload_buffer_size, "%d,",
                        max_ay);

                payload_buffer_size += sprintf(
                        (char*) payload_buffer + payload_buffer_size, "%d,",
                        max_az);

                payload_buffer_size += sprintf(
                        (char*) payload_buffer + payload_buffer_size, "%d",
                        temperature);

                if (packetSendRequestCallback)
                {

                    packetSendRequestCallback(payload_buffer,
                                              payload_buffer_size);

                }

                max_ax = 0;
                max_ay = 0;
                max_az = 0;

                start_radio_tx_state = 0x00;

            }

        }

        wait_ms(1000);

    }

}

void SensorTask_init(void)
{

    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&sensorTaskEvent, &eventParam);
    sensorTaskEventHandle = Event_handle(&sensorTaskEvent);

    Task_Params_init(&sensorTaskParams);
    sensorTaskParams.stackSize = SENSOR_TASK_STACK_SIZE;
    sensorTaskParams.priority = SENSOR_TASK_TASK_PRIORITY;
    sensorTaskParams.stack = &sensorTaskStack;
    Task_construct(&sensorTask, sensorTaskFunction, &sensorTaskParams, NULL);

    Clock_Params clockParams;
    Clock_Params_init(&clockParams);
    clockParams.period = TIMER_TIMEOUT * 1000 / Clock_tickPeriod;
    clockParams.startFlag = TRUE;
    Clock_construct(&sensorTimerClock, sensorTimerClockCallBack,
    TIMER_TIMEOUT * 1000 / Clock_tickPeriod,
                    &clockParams);

}

void SensorTask_registerPacketSendRequestCallback(
        PacketSendRequestCallback callback)
{

    packetSendRequestCallback = callback;

}

