#include <assert.h>
#include <string.h>
#include <os/mynewt.h>
#include <bsp/bsp.h>
#include <hal/hal_gpio.h>
#include "hal/hal_spi.h"
#include "hal/hal_bsp.h"
#include <console/console.h>
#include <os/os.h>
#include <sysinit/sysinit.h>

// This is a mashup of the Mynewt event queue example
//   https://mynewt.apache.org/latest/tutorials/os_fundamentals/event_queue.html
// and apache-mynewt-core/apps/spitest
// for DWM1001-DEV.
// Designed for two connected boards, one SPI master and one slave.
// Set SPI_xx_MASTER in target syscfg.yml to designate master,
// set SPI_xx_SLAVE in target syscfg.yml to designate slave.
// On button push, master will toggle LED and send a message via SPI.
// On SPI message receipt, slave will alter the received message to send out during next SPI transfer.
// BLE OTA (Over-the-Air) image update support is enabled.

#if MYNEWT_VAL(BLE_ENABLED)
#include "bleprph/bleprph.h"
#endif

#if MYNEWT_VAL(SPI_0_MASTER) || MYNEWT_VAL(SPI_1_MASTER) || MYNEWT_VAL(SPI_2_MASTER)
#define SPI_MASTER 1
#define SPI_SS_PIN  (MYNEWT_VAL(SPITEST_SS_PIN))
#if SPI_SS_PIN < 0
#error "SPITEST_SS_PIN must be set in the target config."
#endif
#define SPI_NUM  (MYNEWT_VAL(SPITEST_M_NUM))
#endif

#if MYNEWT_VAL(SPI_0_SLAVE) || MYNEWT_VAL(SPI_1_SLAVE) || MYNEWT_VAL(SPI_2_SLAVE)
#define SPI_SLAVE 1
#define SPI_NUM  (MYNEWT_VAL(SPITEST_S_NUM))
#endif

#if defined(SPI_MASTER) && defined(SPI_SLAVE)
#error "Cannot be SPI master and slave!"
#endif

#define TRIGGER_BUTTON BUTTON_2
#define BUTTON_LED LED_1
#define SLAVE_LED LED_2

#define SPI_BAUDRATE 500
#define SPI_TXRX_LEN 32

#define BUTTON_INTERRUPT_TASK_PRIO  5
#define BUTTON_INTERRUPT_TASK_STACK_SZ    512
static os_stack_t button_interrupt_task_stack[BUTTON_INTERRUPT_TASK_STACK_SZ];
static struct os_task button_interrupt_task_str;
uint8_t btn_cnt;

static void button_interrupt_ev_cb(struct os_event *);
static struct os_eventq button_interrupt_eventq;

#define SPI_CB_TASK_PRIO  4
#define SPI_CB_TASK_STACK_SZ    512
static os_stack_t spi_cb_task_stack[SPI_CB_TASK_STACK_SZ];
static struct os_task spi_cb_task_str;

#ifdef SPI_SLAVE
/* SPI Slave Task */
// This task priority should be higher (number lower) than SPI_CB_TASK
#define SPI_SLAVE_PRIO (3)
#define SPI_SLAVE_STACK_SIZE    OS_STACK_ALIGN(1024)
static os_stack_t spi_slave_task_stack[SPI_SLAVE_STACK_SIZE];
static struct os_task spi_slave_task;
#endif

static void spi_ev_cb(struct os_event *);
static struct os_eventq spi_cb_eventq;

struct os_sem g_spi_sem;

uint8_t g_spi_tx_buf[SPI_TXRX_LEN];
uint8_t g_spi_rx_buf[SPI_TXRX_LEN];
uint8_t g_spi_last_tx[SPI_TXRX_LEN];

static struct os_event gpio_ev = {
    .ev_cb = button_interrupt_ev_cb,
};

static void button_interrupt_ev_cb(struct os_event *ev)
{
    hal_gpio_toggle(BUTTON_LED);

#ifdef SPI_MASTER
    int i;
    btn_cnt++;
    memset(g_spi_tx_buf, btn_cnt, SPI_TXRX_LEN);
    hal_gpio_write(SPI_SS_PIN, 0);
    hal_spi_txrx(SPI_NUM, g_spi_tx_buf, g_spi_rx_buf, SPI_TXRX_LEN);
    hal_gpio_write(SPI_SS_PIN, 1);

    console_printf("Master transmitted: ");
    for (i = 0; i < SPI_TXRX_LEN; i++) {
        console_printf("%2x ", g_spi_tx_buf[i]);
    }
    console_printf("\n");
    console_printf("Master received: ");
    for (i = 0; i < SPI_TXRX_LEN; i++) {
        console_printf("%2x ", g_spi_rx_buf[i]);
    }
    console_printf("\n");
#endif
}

static void my_gpio_irq(void *arg)
{
    // This is the GPIO button interrupt.
    // For something like toggling an LED, could just stick that in here and be done.
    // But a solution good for both simple and complex tasks is to add an event to an event queue.
    os_eventq_put(&button_interrupt_eventq, &gpio_ev);
}

static void button_interrupt_task(void *arg)
{
    // All this task does is pull an item off the event queue and execute its callback.
    while (1) {
        os_eventq_run(&button_interrupt_eventq);
    }
}

void spi_cfg(int spi_num)
{
    struct hal_spi_settings my_spi;

    my_spi.data_order = HAL_SPI_MSB_FIRST;
    my_spi.data_mode = HAL_SPI_MODE0;
    my_spi.baudrate = SPI_BAUDRATE;
    my_spi.word_size = HAL_SPI_WORD_SIZE_8BIT;
    assert(hal_spi_config(spi_num, &my_spi) == 0);
}

static struct os_event spi_ev = {
    .ev_cb = spi_ev_cb,
};

static void spi_ev_cb(struct os_event *ev)
{
    hal_gpio_toggle(SLAVE_LED);

    int i;
    console_printf("Slave received: ");
    for (i = 0; i < SPI_TXRX_LEN; i++) {
        console_printf("%2x ", g_spi_rx_buf[i]);
    }
    console_printf("\n");
    console_printf("Slave transmitted: ");
    for (i = 0; i < SPI_TXRX_LEN; i++) {
        console_printf("%2x ", g_spi_last_tx[i]);
    }
    console_printf("\n");
}

void spi_slave_irqs_handler(void *arg, int len)
{
    memcpy(g_spi_last_tx, g_spi_tx_buf, SPI_TXRX_LEN); // g_spi_tx_buf will be overwritten by main SPI slave task before SPI_CB task has chance to print it out
    os_eventq_put(&spi_cb_eventq, &spi_ev);

    /* Post semaphore to task waiting for SPI slave */
    // Adds a token to semaphore, allowing tasks waiting on it to continue.
    os_sem_release(&g_spi_sem);
}

static void spi_cb_task(void *arg)
{
    // All this task does is pull an item off the event queue and execute its callback.
    while (1) {
        os_eventq_run(&spi_cb_eventq);
    }
}


void spis_task_handler(void *arg)
{
    spi_cfg(SPI_NUM);
    hal_spi_set_txrx_cb(SPI_NUM, spi_slave_irqs_handler, NULL);
    hal_spi_enable(SPI_NUM);

    /* Make the default character 0x77 */
    hal_spi_slave_set_def_tx_val(SPI_NUM, 0x77);

    /*
     * Fill buffer with 0x33 for first transfer.
     */
    memset(g_spi_tx_buf, 0x33, SPI_TXRX_LEN);
    hal_spi_txrx_noblock(SPI_NUM, g_spi_tx_buf, g_spi_rx_buf, SPI_TXRX_LEN);

    while (1) {
        /* Wait for semaphore from ISR */
        // "When a task desires exclusive access to the shared resource it requests the semaphore by calling os_sem_pend()"
        os_sem_pend(&g_spi_sem, OS_TIMEOUT_NEVER);

        memcpy(g_spi_tx_buf, g_spi_rx_buf, SPI_TXRX_LEN); // send back whatever we last received
        g_spi_tx_buf[0] = g_spi_tx_buf[0] + 7;//modify first entry
        hal_spi_txrx_noblock(SPI_NUM, g_spi_tx_buf, g_spi_rx_buf, SPI_TXRX_LEN);
    }
}

void init_tasks(void)
{
    /* Button interrupt event queue */
    os_eventq_init(&button_interrupt_eventq);

    /*
     * Create the task to process button interrupt events from the
     * button_interrupt_eventq event queue.
     */
    os_task_init(&button_interrupt_task_str, "button_interrupt_task",
                 button_interrupt_task, NULL,
                 BUTTON_INTERRUPT_TASK_PRIO, OS_WAIT_FOREVER,
                 button_interrupt_task_stack,
                 BUTTON_INTERRUPT_TASK_STACK_SZ);

    /* SPI interrupt event queue */
    os_eventq_init(&spi_cb_eventq);

    // Task to process events from SPI interrupt
    os_task_init(&spi_cb_task_str, "spi_cb_task",
                 spi_cb_task, NULL,
                 SPI_CB_TASK_PRIO, OS_WAIT_FOREVER,
                 spi_cb_task_stack,
                 SPI_CB_TASK_STACK_SZ);

    hal_gpio_irq_init(TRIGGER_BUTTON, my_gpio_irq, NULL, HAL_GPIO_TRIG_RISING, HAL_GPIO_PULL_UP);
    hal_gpio_irq_enable(TRIGGER_BUTTON);

    hal_gpio_init_out(BUTTON_LED, 1);

    /* Initialize global test semaphore */
    os_sem_init(&g_spi_sem, 0);

    #if defined(SPI_SLAVE)
        hal_gpio_init_out(SLAVE_LED, 1);
        os_task_init(&spi_slave_task, "spis", spis_task_handler, NULL,
                SPI_SLAVE_PRIO, OS_WAIT_FOREVER, spi_slave_task_stack, SPI_SLAVE_STACK_SIZE);
    #endif

    #if defined(SPI_MASTER)
        btn_cnt = 0;
        hal_gpio_init_out(SPI_SS_PIN, 1);
        spi_cfg(SPI_NUM);
        hal_spi_enable(SPI_NUM);
        hal_gpio_write(SPI_SS_PIN, 1);
    #endif
}

int main(int argc, char **argv)
{
    sysinit();

    #if MYNEWT_VAL(BLE_ENABLED)
    ble_init(0xABCD);
    #endif

    init_tasks();

    while (1) {
       os_eventq_run(os_eventq_dflt_get());
    }
    assert(0);//should never reach here
}