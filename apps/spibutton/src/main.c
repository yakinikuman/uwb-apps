
#include <assert.h>
#include <string.h>
#include <os/mynewt.h>
#include <bsp/bsp.h>
#include <hal/hal_gpio.h>
#include "hal/hal_spi.h"
#include <console/console.h>
#include <os/os.h>
#include <sysinit/sysinit.h>



#define BUTTON_INTERRUPT_TASK_PRIO  4
#define BUTTON_INTERRUPT_TASK_STACK_SZ    512

#define BLINK_LED LED_1

/* SPI Send Task */
#define SPI_SEND_PRIO (3)
#define SPI_SEND_STACK_SIZE    OS_STACK_ALIGN(1024)
struct os_task spi_send_task;

uint8_t btn_cnt;

static void button_interrupt_ev_cb(struct os_event *);

static struct os_eventq button_interrupt_eventq;

static os_stack_t button_interrupt_task_stack[BUTTON_INTERRUPT_TASK_STACK_SZ];
static struct os_task button_interrupt_task_str;

/* The spi txrx callback */
struct sblinky_spi_cb_arg
{
    int transfers;
    int txlen;
    uint32_t tx_rx_bytes;
};
struct sblinky_spi_cb_arg spi_cb_obj;
void *spi_cb_arg;

/* Global spi semaphore */
struct os_sem g_spi_sem;

#define SPI_BAUDRATE 500
#define SPI_TXRX_LEN 32

uint8_t g_spi_tx_buf[SPI_TXRX_LEN];
uint8_t g_spi_rx_buf[SPI_TXRX_LEN];

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

static struct os_event gpio_ev = {
    .ev_cb = button_interrupt_ev_cb,
};

static void button_interrupt_ev_cb(struct os_event *ev)
{
    // Event callback function.
    assert(ev != NULL);

    hal_gpio_toggle(BLINK_LED);

#ifdef SPI_MASTER
    int rc;
    int i;
    //g_spi_tx_buf[0] = 0x01;
    btn_cnt++;
    memset(g_spi_tx_buf, btn_cnt, SPI_TXRX_LEN);
    //assert(hal_gpio_read(SPI_SS_PIN) == 1);
    hal_gpio_write(SPI_SS_PIN, 0);
    rc = hal_spi_txrx(SPI_NUM, g_spi_tx_buf, g_spi_rx_buf, spi_cb_obj.txlen);
    assert(!rc);
    hal_gpio_write(SPI_SS_PIN, 1);

    console_printf("Master transmitted: ");
    for (i = 0; i < spi_cb_obj.txlen; i++) {
        console_printf("%2x ", g_spi_tx_buf[i]);
    }
    console_printf("\n");
    console_printf("Master received: ");
    for (i = 0; i < spi_cb_obj.txlen; i++) {
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


// Event queue example: https://mynewt.apache.org/latest/tutorials/os_fundamentals/event_queue.html
//   Blinks one LED via a task
//   Blinks another LED via a timed callout
//   Blinks third LED on button press --- kept this
// spitest - from apache-mynewt-core
//    Blocking API can only be used on master
//    Not quite sure whether blocking or non-blocking should be used...
// Master has to "know" how big slave's data is...
// Potentially Master will send some command data
// Slave side - send all the data RTDOA_tag project prints out.  Will prob have to pre-define a max # of anchors...
// How would a request/receive type SPI interface really work?
//   Master sends request (ignore received slave data)
//   Slave clears current SPI queue and sets ready bit to 0
//   Master waits a bit, then loops sending a "Ready?" and looking at received ready bit yes/no
//   When Slave is ready, set ready bit to 1 ... then immediately fill tx_buf with requested data ... ?
//   If master received ready = 1, then sends a "retrieve" message to get the data
//   The "ready" message is 



void spi_cfg(int spi_num)
{
    struct hal_spi_settings my_spi;

    my_spi.data_order = HAL_SPI_MSB_FIRST;
    my_spi.data_mode = HAL_SPI_MODE0;
    my_spi.baudrate = SPI_BAUDRATE;
    my_spi.word_size = HAL_SPI_WORD_SIZE_8BIT;
    assert(hal_spi_config(spi_num, &my_spi) == 0);
}


void spi_slave_irqs_handler(void *arg, int len)
{
    struct sblinky_spi_cb_arg *cb;

    hal_gpio_toggle(BLINK_LED);

    assert(arg == spi_cb_arg);
    if (spi_cb_arg) {
        cb = (struct sblinky_spi_cb_arg *)arg;
        ++cb->transfers;
        cb->tx_rx_bytes += len;
        cb->txlen = len;
    }

    int i;
    console_printf("Slave received: ");
    for (i = 0; i < spi_cb_obj.txlen; i++) {
        console_printf("%2x ", g_spi_rx_buf[i]);
    }
    console_printf("\n");
    console_printf("Slave transmitted: ");
    for (i = 0; i < spi_cb_obj.txlen; i++) {
        console_printf("%2x ", g_spi_tx_buf[i]);
    }
    console_printf("\n");

    /* Post semaphore to task waiting for SPI slave */
    os_sem_release(&g_spi_sem);
}



void spis_task_handler(void *arg)
{
    //int rc;

    spi_cb_arg = &spi_cb_obj;
    spi_cb_obj.txlen = SPI_TXRX_LEN;
    spi_cfg(SPI_NUM);
    hal_spi_set_txrx_cb(SPI_NUM, spi_slave_irqs_handler, spi_cb_arg);
    hal_spi_enable(SPI_NUM);

    /* Make the default character 0x77 */
    hal_spi_slave_set_def_tx_val(SPI_NUM, 0x77);

    /*
     * Fill buffer with 0x33 for first transfer.
     */
    memset(g_spi_tx_buf, 0x33, SPI_TXRX_LEN);
    //rc = hal_spi_txrx_noblock(SPI_NUM, g_spi_tx_buf, g_spi_rx_buf, SPI_TXRX_LEN);
    hal_spi_txrx_noblock(SPI_NUM, g_spi_tx_buf, g_spi_rx_buf, SPI_TXRX_LEN);

    while (1) {
        /* Wait for semaphore from ISR */
        os_sem_pend(&g_spi_sem, OS_TIMEOUT_NEVER);

        //memset(g_spi_tx_buf, 0x88, SPI_TXRX_LEN);
        memcpy(g_spi_tx_buf, g_spi_rx_buf, spi_cb_obj.txlen); // send back whatever we last received
        g_spi_tx_buf[0] = g_spi_tx_buf[0] + 7;//modify first entry
        //rc = hal_spi_txrx_noblock(SPI_NUM, g_spi_tx_buf, g_spi_rx_buf, SPI_TXRX_LEN);
        //assert(rc == 0);
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


    hal_gpio_irq_init(BUTTON_1, my_gpio_irq, NULL, HAL_GPIO_TRIG_RISING, HAL_GPIO_PULL_UP);
    hal_gpio_irq_enable(BUTTON_1);

    hal_gpio_init_out(BLINK_LED, 1);

    // Below - modified from spitest
    os_stack_t *pstack;
    (void)pstack;
    pstack = malloc(sizeof(os_stack_t)*SPI_SEND_STACK_SIZE);
    assert(pstack);

    /* Initialize global test semaphore */
    os_sem_init(&g_spi_sem, 0);//need?

    #if defined(SPI_SLAVE)
        os_task_init(&spi_send_task, "spis", spis_task_handler, NULL,
                SPI_SEND_PRIO, OS_WAIT_FOREVER, pstack, SPI_SEND_STACK_SIZE);
    #endif

    #if defined(SPI_MASTER)
        btn_cnt = 0;
        hal_gpio_init_out(SPI_SS_PIN, 1);
        spi_cfg(SPI_NUM);
        hal_spi_enable(SPI_NUM);
        hal_gpio_write(SPI_SS_PIN, 1);
        spi_cb_obj.txlen = SPI_TXRX_LEN;
    #endif
}

int main(int argc, char **argv)
{
    sysinit();

    init_tasks();

    while (1) {
       os_eventq_run(os_eventq_dflt_get());
    }
    assert(0);
}