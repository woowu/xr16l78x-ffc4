/*
 *  xr16l78x.c  -- EXAR multiport serial driver for XR16L78x family of UARTS.
 *
 */

#if 1       /* Enable dev_dbg */
#define DEBUG   1
#define DBG
#define DEBUG_MSG
#endif

#include <linux/module.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/serial_reg.h>
#include <linux/serial.h>
#include <linux/serial_core.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/irq.h>
#include <asm/bitops.h>
#include <asm/byteorder.h>
#include <asm/serial.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#include <linux/tty_flip.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/nmi.h>

/*------Define a slab cache structure-----------*/
#define BOTH_EMPTY  (UART_LSR_TEMT | UART_LSR_THRE)

#define SERIALEXAR_SHARE_IRQS 1 
unsigned int share_irqs = SERIALEXAR_SHARE_IRQS;

#define UART_XR788_NR   8 // MAX 8 ports per card for 788:

#define XR_788_MAJOR       40
#define XR_788_MINOR       0
#define XR78X_UART_FIFO_SIZE    64
#define PASS_LIMIT  256
#define XR788_COUNT 3

#define XR78X_UART_REG_SPACE    0x10

#if 1
#define XR_DEBUG_INTR(fmt...)   printk(fmt)
#else
#define XR_DEBUG_INTR(fmt...)   do { } while (0)
#endif

/*
 * We default to IRQ0 for the "no irq" hack.   Some
 * machine types want others as well - they're free
 * to redefine this in their header file.
 */
#define is_real_interrupt(irq)  ((irq) != 0)

struct uart_xr78x_port;
/**
 * xr78x core driver operations
 *
 * @setup_irq()     Setup irq handling. The xr78x driver links this
 *          port to the irq chain. Other drivers may @request_irq().
 * @release_irq()   Undo irq handling. The xr78x driver unlinks
 *          the port from the irq chain.
 */
struct uart_xr78x_ops {
    int     (*setup_irq)(struct uart_xr78x_port *);
    void    (*release_irq)(struct uart_xr78x_port *);
};

struct uart_xr78x_port {
    struct uart_port    port;
    struct timer_list   timer;      /* "no irq" timer */
    struct list_head    list;       /* ports on this IRQ */
    unsigned int        capabilities;   /* port capabilities */
    unsigned short      bugs;       /* port bugs */
    bool                fifo_bug;   /* min RX trigger if enabled */
    unsigned short      rev;
    unsigned int        tx_loadsz;
    unsigned char       acr;
    unsigned char       fcr;
    unsigned char       ier;
    unsigned char       lcr;
    unsigned char       mcr;
    unsigned char       mcr_mask;   /* mask of user bits */
    unsigned char       mcr_force;  /* mask of forced bits */

    unsigned char       cur_iotype; /* Running I/O type */

    /*
     * Some bits in registers are cleared on a read, so they must
     * be saved whenever the register is read but the bits will not
     * be immediately processed.
     */
#define LSR_SAVE_FLAGS UART_LSR_BRK_ERROR_BITS
    unsigned char       lsr_saved_flags;
#define MSR_SAVE_FLAGS UART_MSR_ANY_DELTA
    unsigned char       msr_saved_flags;
    /*
     * We provide a per-port pm hook.
     */
    void            (*pm)(struct uart_port *port,
                      unsigned int state, unsigned int old);
    const struct uart_xr78x_ops *ops;
    /* read and write the DLL/DLM register.*/
    int         (*dl_read)(struct uart_xr78x_port *);
    void        (*dl_write)(struct uart_xr78x_port *, int);
};

struct irq_info {
    struct      hlist_node node;
    int         irq;
    spinlock_t          lock;   /* Protects list not the hash */
    struct list_head    *head;
};

#define NR_IRQ_HASH     32  /* Can be adjusted later */
static struct hlist_head irq_lists[NR_IRQ_HASH];
static DEFINE_MUTEX(hash_mutex);    /* Used to walk the hash */

/*
 * Here we define the default xmit fifo size used for each type of UART.
 */
struct serial_uart_config {
    char    *name;
    int dfl_xmit_fifo_size;
    int flags;
};

#define PORT_MAX_XR 1 
#define XR788_TYPE 1 // the second entry that is [1] in the array
static const struct serial_uart_config uart_config[PORT_MAX_XR+1] = {
    { "Unknown",    1,  0 },
    { "XR78x",      64, 0 },
};

static inline struct uart_xr78x_port *up_to_xr78xp(struct uart_port *up)
{
    return container_of(up, struct uart_xr78x_port, port);
}

static unsigned int hub6_serial_in(struct uart_port *p, int offset)
{
    offset = offset << p->regshift;
    outb(p->hub6 - 1 + offset, p->iobase);
    return inb(p->iobase + 1);
}

static void hub6_serial_out(struct uart_port *p, int offset, int value)
{
    offset = offset << p->regshift;
    outb(p->hub6 - 1 + offset, p->iobase);
    outb(value, p->iobase + 1);
}

static unsigned int mem_serial_in(struct uart_port *p, int offset)
{
    offset = offset << p->regshift;
    return readb(p->membase + offset);
}

static void mem_serial_out(struct uart_port *p, int offset, int value)
{
    offset = offset << p->regshift;
    writeb(value, p->membase + offset);
}

static unsigned int io_serial_in(struct uart_port *p, int offset)
{
    offset = offset << p->regshift;
    return inb(p->iobase + offset);
}

static void io_serial_out(struct uart_port *p, int offset, int value)
{
    offset = offset << p->regshift;
    outb(value, p->iobase + offset);
}

static inline int serial_in(struct uart_xr78x_port *up, int offset)
{
    return up->port.serial_in(&up->port, offset);
}

static inline void serial_out(struct uart_xr78x_port *up, int offset, int value)
{
    up->port.serial_out(&up->port, offset, value);
}

/* Uart divisor latch read */
static int default_serial_dl_read(struct uart_xr78x_port *up)
{
    return serial_in(up, UART_DLL) | serial_in(up, UART_DLM) << 8;
}

/* Uart divisor latch write */
static void default_serial_dl_write(struct uart_xr78x_port *up, int value)
{
    serial_out(up, UART_DLL, value & 0xff);
    serial_out(up, UART_DLM, value >> 8 & 0xff);
}

static inline int serial_dl_read(struct uart_xr78x_port *up)
{
    return up->dl_read(up);
}

static inline void serial_dl_write(struct uart_xr78x_port *up, int value)
{
    up->dl_write(up, value);
}

static int serialxr78x_default_handle_irq(struct uart_port *port);

static void set_io_from_upio(struct uart_port *p)
{
    struct uart_xr78x_port *up = up_to_xr78xp(p);

    up->dl_read = default_serial_dl_read;
    up->dl_write = default_serial_dl_write;

    switch (p->iotype) {
    case UPIO_HUB6:
        p->serial_in = hub6_serial_in;
        p->serial_out = hub6_serial_out;
        break;

    case UPIO_MEM:
        p->serial_in = mem_serial_in;
        p->serial_out = mem_serial_out;
        break;

    default:
        p->serial_in = io_serial_in;
        p->serial_out = io_serial_out;
        break;
    }
    /* Remember loaded iotype */
    up->cur_iotype = p->iotype;
    p->handle_irq = serialxr78x_default_handle_irq;
}

/*
 * FIFO support.
 */
static void serialxr78x_clear_fifos(struct uart_xr78x_port *up)
{
//  if (up->capabilities & UART_CAP_FIFO) {
        serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO);
        serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO |
                   UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
        serial_out(up, UART_FCR, 0);
//  }
}

static void serialxr78x_stop_tx(struct uart_port *port)
{
    struct uart_xr78x_port *up = up_to_xr78xp(port);

    XR_DEBUG_INTR("****** %s", __func__);
    if (up->ier & UART_IER_THRI) {
        up->ier &= ~UART_IER_THRI;
        serial_out(up, UART_IER, up->ier);
    }
}

static void serialxr78x_start_tx(struct uart_port *port)
{
    struct uart_xr78x_port *up = up_to_xr78xp(port);

    XR_DEBUG_INTR("****** %s", __func__);
    if (!(up->ier & UART_IER_THRI)) {
        up->ier |= UART_IER_THRI;
        serial_out(up, UART_IER, up->ier);
    }
}

static void serialxr78x_stop_rx(struct uart_port *port)
{
    struct uart_xr78x_port *up = up_to_xr78xp(port);

    XR_DEBUG_INTR("****** %s", __func__);

    up->ier &= ~(UART_IER_RLSI | UART_IER_RDI); // old value is ~UART_IER_RLSI;
    up->port.read_status_mask &= ~UART_LSR_DR;
    serial_out(up, UART_IER, up->ier);
}

static void serialxr78x_disable_ms(struct uart_port *port)
{
    struct uart_xr78x_port *up = up_to_xr78xp(port);

    /* no MSR capabilities */
//  if (up->bugs & UART_BUG_NOMSR)
//      return;

    up->ier &= ~UART_IER_MSI;
    serial_port_out(port, UART_IER, up->ier);
}

static void serialxr78x_enable_ms(struct uart_port *port)
{
    struct uart_xr78x_port *up = up_to_xr78xp(port);

    XR_DEBUG_INTR("****** %s", __func__);

    up->ier |= UART_IER_MSI;
    serial_out(up, UART_IER, up->ier);
}

void serialxr78x_read_char(struct uart_xr78x_port *up, unsigned char lsr)
{
    struct uart_port *port = &up->port;
    unsigned char ch;
    char flag = TTY_NORMAL;

    if (likely(lsr & UART_LSR_DR))
        ch = serial_in(up, UART_RX);
    else
        /*
         * Intel 82571 has a Serial Over Lan device that will
         * set UART_LSR_BI without setting UART_LSR_DR when
         * it receives a break. To avoid reading from the
         * receive buffer without UART_LSR_DR bit set, we
         * just force the read character to be 0
         */
        ch = 0;

    port->icount.rx++;

    lsr |= up->lsr_saved_flags;
    up->lsr_saved_flags = 0;

    if (unlikely(lsr & UART_LSR_BRK_ERROR_BITS)) {
        if (lsr & UART_LSR_BI) {
            lsr &= ~(UART_LSR_FE | UART_LSR_PE);
            port->icount.brk++;
            /*
             * We do the SysRQ and SAK checking
             * here because otherwise the break
             * may get masked by ignore_status_mask
             * or read_status_mask.
             */
            if (uart_handle_break(port))
                return;
        } else if (lsr & UART_LSR_PE)
            port->icount.parity++;
        else if (lsr & UART_LSR_FE)
            port->icount.frame++;
        if (lsr & UART_LSR_OE)
            port->icount.overrun++;

        /*
         * Mask off conditions which should be ignored.
         */
        lsr &= port->read_status_mask;

        if (lsr & UART_LSR_BI) {
            pr_debug("%s: handling break\n", __func__);
            flag = TTY_BREAK;
        } else if (lsr & UART_LSR_PE)
            flag = TTY_PARITY;
        else if (lsr & UART_LSR_FE)
            flag = TTY_FRAME;
    }
    if (uart_handle_sysrq_char(port, ch))
        return;

    uart_insert_char(port, lsr, UART_LSR_OE, ch, flag);
}

/*
 * serialxr78x_rx_chars: processes according to the passed in LSR
 * value, and returns the remaining LSR bits not handled
 * by this Rx routine.
 */
static unsigned char
serialxr78x_rx_chars(struct uart_xr78x_port *up, unsigned char lsr)
{
    struct uart_port *port = &up->port;
    int max_count = 256;

    XR_DEBUG_INTR("****** %s", __func__);
    do {
        serialxr78x_read_char(up, lsr);
        if (--max_count == 0)
            break;
        lsr = serial_in(up, UART_LSR);
    } while (lsr & (UART_LSR_DR | UART_LSR_BI));

    tty_flip_buffer_push(&port->state->port);
    return lsr;
}


void serialxr78x_tx_chars(struct uart_xr78x_port *up)
{
    struct uart_port *port = &up->port;
    struct circ_buf *xmit = &port->state->xmit;
    int count;

    XR_DEBUG_INTR("****** %s", __func__);
    if (port->x_char) {
        serial_out(up, UART_TX, port->x_char);
        port->icount.tx++;
        port->x_char = 0;
        return;
    }

    if (uart_tx_stopped(port) || uart_circ_empty(xmit)) {
        serialxr78x_stop_tx(port);
        return;
    }

    count = up->tx_loadsz;
    do {
        serial_out(up, UART_TX, xmit->buf[xmit->tail]);
        xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
        port->icount.tx++;
        if (uart_circ_empty(xmit))
            break;
//      if ((up->capabilities & UART_CAP_HFIFO) &&
//          (serial_in(up, UART_LSR) & BOTH_EMPTY) != BOTH_EMPTY)
//          break;
    } while (--count > 0);

    if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
        uart_write_wakeup(port);

    /*
     * With RPM enabled, we have to wait until the FIFO is empty before the
     * HW can go idle. So we get here once again with empty FIFO and disable
     * the interrupt and RPM in __stop_tx()
     */
    if (uart_circ_empty(xmit) /* && !(up->capabilities & UART_CAP_RPM) */)
        serialxr78x_stop_tx(port);
}

/* Caller holds uart port lock */
//unsigned int serialxr78x_modem_status(struct uart_xr78x_port *up)
//{
//  int status;
//
//  status = serial_in(up, UART_MSR);
//
//  if ((status & UART_MSR_ANY_DELTA) == 0)
//      return status;
//
//  if (status & UART_MSR_TERI)
//      up->port.icount.rng++;
//  if (status & UART_MSR_DDSR)
//      up->port.icount.dsr++;
//  if (status & UART_MSR_DDCD)
//      uart_handle_dcd_change(&up->port, status & UART_MSR_DCD);
//  if (status & UART_MSR_DCTS)
//      uart_handle_cts_change(&up->port, status & UART_MSR_CTS);
//
//  wake_up_interruptible(&up->port.state->port.delta_msr_wait);
//
//  return status;
//}
unsigned int serialxr78x_modem_status(struct uart_xr78x_port *up)
{
    struct uart_port *port = &up->port;
    unsigned int status = serial_in(up, UART_MSR);

    status |= up->msr_saved_flags;
    up->msr_saved_flags = 0;
    if (status & UART_MSR_ANY_DELTA && up->ier & UART_IER_MSI &&
        port->state != NULL) {
        if (status & UART_MSR_TERI)
            port->icount.rng++;
        if (status & UART_MSR_DDSR)
            port->icount.dsr++;
        if (status & UART_MSR_DDCD)
            uart_handle_dcd_change(port, status & UART_MSR_DCD);
        if (status & UART_MSR_DCTS)
            uart_handle_cts_change(port, status & UART_MSR_CTS);

        wake_up_interruptible(&port->state->port.delta_msr_wait);
    }

    return status;
}

/*
 * This is the serial driver's interrupt routine.
 *
 * Arjan thinks the old way was overly complex, so it got simplified.
 * Alan disagrees, saying that need the complexity to handle the weird
 * nature of ISA shared interrupts.  (This is a special exception.)
 *
 * In order to handle ISA shared interrupts properly, we need to check
 * that all ports have been serviced, and therefore the ISA interrupt
 * line has been de-asserted.
 *
 * This means we need to loop through all ports. checking that they
 * don't have an interrupt pending.
 */
static irqreturn_t serialxr78x_interrupt(int irq, void *dev_id)
{
    struct irq_info *i = dev_id;
    struct list_head *l, *end = NULL;
    int pass_counter = 0, handled = 0;

    XR_DEBUG_INTR("xr16l78x: irq %d\n", irq);

    spin_lock(&i->lock);

    l = i->head;
    do {
        struct uart_xr78x_port *up;
        struct uart_port *port;

        up = list_entry(l, struct uart_xr78x_port, list);
        port = &up->port;

        if (port->handle_irq(port)) {
            handled = 1;
            end = NULL;
        } else if (end == NULL)
            end = l;

        l = l->next;

        if (l == i->head && pass_counter++ > PASS_LIMIT) {
            /* If we hit this, we're dead. */
            printk_ratelimited(KERN_ERR
                "serialxr78x: too much work for irq%d\n", irq);
            break;
        }
    } while (l != end);

    spin_unlock(&i->lock);

    pr_debug("%s(%d): end\n", __func__, irq);

    return IRQ_RETVAL(handled);
}

/*
 * To support ISA shared interrupts, we need to have one interrupt
 * handler that ensures that the IRQ line has been deasserted
 * before returning.  Failing to do this will result in the IRQ
 * line being stuck active, and, since ISA irqs are edge triggered,
 * no more IRQs will be seen.
 */
static void serial_do_unlink(struct irq_info *i, struct uart_xr78x_port *up)
{
    XR_DEBUG_INTR("****** %s", __func__);
    spin_lock_irq(&i->lock);

    if (!list_empty(i->head)) {
        if (i->head == &up->list)
            i->head = i->head->next;
        list_del(&up->list);
    } else {
        BUG_ON(i->head != &up->list);
        i->head = NULL;
    }
    spin_unlock_irq(&i->lock);
    /* List empty so throw away the hash node */
    if (i->head == NULL) {
        hlist_del(&i->node);
        kfree(i);
    }
}

static int serial_link_irq_chain(struct uart_xr78x_port *up)
{
    struct hlist_head *h;
    struct hlist_node *n;
    struct irq_info *i;
    int ret, irq_flags = up->port.flags & UPF_SHARE_IRQ ? IRQF_SHARED : 0;

    XR_DEBUG_INTR("****** %s", __func__);
    mutex_lock(&hash_mutex);

    h = &irq_lists[up->port.irq % NR_IRQ_HASH];

    hlist_for_each(n, h) {
        i = hlist_entry(n, struct irq_info, node);
        if (i->irq == up->port.irq)
            break;
    }

    if (n == NULL) {
        i = kzalloc(sizeof(struct irq_info), GFP_KERNEL);
        if (i == NULL) {
            mutex_unlock(&hash_mutex);
            printk("%s:: Not enough memory.", __func__);
            return -ENOMEM;
        }
        spin_lock_init(&i->lock);
        i->irq = up->port.irq;
        hlist_add_head(&i->node, h);
    }
    mutex_unlock(&hash_mutex);

    spin_lock_irq(&i->lock);

    if (i->head) {
        list_add(&up->list, i->head);
        spin_unlock_irq(&i->lock);
        printk("****%s:: add list");
        ret = 0;
    } else {
        INIT_LIST_HEAD(&up->list);
        i->head = &up->list;
        spin_unlock_irq(&i->lock);
        irq_flags |= up->port.irqflags;
        ret = request_irq(up->port.irq, serialxr78x_interrupt,
                  irq_flags, up->port.name, i);
        printk("****%s:: request irq=%d. ret=%d",
                __func__, up->port.irq, ret);
        if (ret < 0) {
            serial_do_unlink(i, up);
        }
    }

    return ret;
}

static void serial_unlink_irq_chain(struct uart_xr78x_port *up)
{
    /*
     * yes, some broken gcc emit "warning: 'i' may be used uninitialized"
     * but no, we are not going to take a patch that assigns NULL below.
     */
    struct irq_info *i;
    struct hlist_node *n;
    struct hlist_head *h;

    XR_DEBUG_INTR("****** %s", __func__);
    mutex_lock(&hash_mutex);

    h = &irq_lists[up->port.irq % NR_IRQ_HASH];

    hlist_for_each(n, h) {
        i = hlist_entry(n, struct irq_info, node);
        if (i->irq == up->port.irq)
            break;
    }

    BUG_ON(n == NULL);
    BUG_ON(i->head == NULL);

    if (list_empty(i->head))
        free_irq(up->port.irq, i);

    serial_do_unlink(i, up);
    mutex_unlock(&hash_mutex);
}

/*
 * This function is used to handle ports that do not have an
 * interrupt.  This doesn't work very well for 16450's, but gives
 * barely passable results for a 16550A.  (Although at the expense
 * of much CPU overhead).
 */
static void serialxr78x_timeout(struct timer_list *t)
{
    struct uart_xr78x_port *up = from_timer(up, t, timer);

    XR_DEBUG_INTR("****** %s", __func__);
    up->port.handle_irq(&up->port);
    mod_timer(&up->timer, jiffies + uart_poll_timeout(&up->port));
}

static void serialxr78x_backup_timeout(struct timer_list *t)
{
    struct uart_xr78x_port *up = from_timer(up, t, timer);
    unsigned int iir, ier = 0, lsr;
    unsigned long flags;

    XR_DEBUG_INTR("****** %s", __func__);
    spin_lock_irqsave(&up->port.lock, flags);

    /*
     * Must disable interrupts or else we risk racing with the interrupt
     * based handler.
     */
    if (up->port.irq) {
        ier = serial_in(up, UART_IER);
        serial_out(up, UART_IER, 0);
    }

    iir = serial_in(up, UART_IIR);

    /*
     * This should be a safe test for anyone who doesn't trust the
     * IIR bits on their UART, but it's specifically designed for
     * the "Diva" UART used on the management processor on many HP
     * ia64 and parisc boxes.
     */
    lsr = serial_in(up, UART_LSR);
    up->lsr_saved_flags |= lsr & LSR_SAVE_FLAGS;
    if ((iir & UART_IIR_NO_INT) && (up->ier & UART_IER_THRI) &&
        (!uart_circ_empty(&up->port.state->xmit) || up->port.x_char) &&
        (lsr & UART_LSR_THRE)) {
        iir &= ~(UART_IIR_ID | UART_IIR_NO_INT);
        iir |= UART_IIR_THRI;
    }

    if (!(iir & UART_IIR_NO_INT))
        serialxr78x_tx_chars(up);

    if (up->port.irq)
        serial_out(up, UART_IER, ier);

    spin_unlock_irqrestore(&up->port.lock, flags);

    /* Standard timer interval plus 0.2s to keep the port running */
    mod_timer(&up->timer,
        jiffies + uart_poll_timeout(&up->port) + HZ / 5);
}

static unsigned int serialxr78x_tx_empty(struct uart_port *port)
{
    struct uart_xr78x_port *up = up_to_xr78xp(port);
    unsigned long flags;
    unsigned int lsr;

    XR_DEBUG_INTR("****** %s", __func__);
    spin_lock_irqsave(&port->lock, flags);
    lsr = serial_port_in(port, UART_LSR);
    up->lsr_saved_flags |= lsr & LSR_SAVE_FLAGS;
    spin_unlock_irqrestore(&port->lock, flags);

    return (lsr & BOTH_EMPTY) == BOTH_EMPTY ? TIOCSER_TEMT : 0;
}

static unsigned int serialxr78x_get_mctrl(struct uart_port *port)
{
    struct uart_xr78x_port *up = up_to_xr78xp(port);
    unsigned long flags;
    unsigned char status;
    unsigned int ret;

    spin_lock_irqsave(&up->port.lock, flags);
    status = serial_in(up, UART_MSR);
    spin_unlock_irqrestore(&up->port.lock, flags);

//  status = serialxr78x_modem_status(up);

    ret = 0;
    if (status & UART_MSR_DCD)
        ret |= TIOCM_CAR;
    if (status & UART_MSR_RI)
        ret |= TIOCM_RNG;
    if (status & UART_MSR_DSR)
        ret |= TIOCM_DSR;
    if (status & UART_MSR_CTS)
        ret |= TIOCM_CTS;

    XR_DEBUG_INTR("*** 78x_get_mctrl, membase=0x%x, status=0x%08x\n",
            port->membase + UART_MSR, status);
    return ret;
}

static void serialxr78x_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
    struct uart_xr78x_port *up = up_to_xr78xp(port);
    unsigned char mcr = 0;

    if (mctrl & TIOCM_RTS)
        mcr |= UART_MCR_RTS;
    if (mctrl & TIOCM_DTR)
        mcr |= UART_MCR_DTR;
    if (mctrl & TIOCM_OUT1)
        mcr |= UART_MCR_OUT1;
    if (mctrl & TIOCM_OUT2)
        mcr |= UART_MCR_OUT2;
    if (mctrl & TIOCM_LOOP)
        mcr |= UART_MCR_LOOP;

    mcr = (mcr & up->mcr_mask) | up->mcr_force | up->mcr;
    serial_out(up, UART_MCR, mcr);

    XR_DEBUG_INTR("***78x_set_mctrl, membase=0x%x, mcr=0x%x, mctrl=0x%x\n",
            port->membase + UART_MCR, mcr, mctrl);
}

static void serialxr78x_break_ctl(struct uart_port *port, int break_state)
{
    struct uart_xr78x_port *up = up_to_xr78xp(port);
    unsigned long flags;

    spin_lock_irqsave(&up->port.lock, flags);
    if (break_state == -1)
        up->lcr |= UART_LCR_SBC;
    else
        up->lcr &= ~UART_LCR_SBC;
    serial_out(up, UART_LCR, up->lcr);
    spin_unlock_irqrestore(&up->port.lock, flags);
}

static int print_reg_info(struct uart_xr78x_port *up, int num)
{
    int i;
    unsigned char reg;
    int val;

    reg = 1;
    for(i = 0; i < 0x10; i++) {
        reg = 1 << i;
        if (!(reg & num)) continue;

        val = serial_in(up, i);
        printk("*** reg= 0x%x, membase=0x%x, val=%x\n",
            i + up->port.mapbase, i + up->port.membase, val);
    }
}

static int serialxr78x_startup(struct uart_port *port)
{
    struct uart_xr78x_port *up = up_to_xr78xp(port);
    unsigned long flags;
    unsigned char lsr, iir;
    int retval;

    XR_DEBUG_INTR("****** %s", __func__);
//  if (!port->fifosize)
//      port->fifosize = uart_config[port->type].fifo_size;
//  if (!up->tx_loadsz)
//      up->tx_loadsz = uart_config[port->type].tx_loadsz;
//  if (!up->capabilities)
//      up->capabilities = ?uart_config[port->type].flags;
    up->mcr = 0;

    if (port->iotype != up->cur_iotype)
        set_io_from_upio(port);

    /*
     * Clear the FIFO buffers and disable them.
     * (they will be reenabled in set_termios())
     */
    serialxr78x_clear_fifos(up);

    /*
     * Clear the interrupt registers.
     */
    serial_port_in(port, UART_LSR);
    serial_port_in(port, UART_RX);
    serial_port_in(port, UART_IIR);
    serial_port_in(port, UART_MSR);

    retval = up->ops->setup_irq(up);
    if (retval)
        goto out;

    /*
     * Now, initialize the UART
     */
    serial_port_out(port, UART_LCR, UART_LCR_WLEN8);

    spin_lock_irqsave(&port->lock, flags);

    /*
     * Most PC uarts need OUT2 raised to enable interrupts.
     */
    if (port->irq)
        port->mctrl |= TIOCM_OUT2;
    serialxr78x_set_mctrl(port, port->mctrl);

    /*
     * Do a quick test to see if we receive an interrupt when we enable
     * the TX irq.
     */
//  serial_port_out(port, UART_IER, UART_IER_THRI);
//  lsr = serial_port_in(port, UART_LSR);
//  iir = serial_port_in(port, UART_IIR);
//  serial_port_out(port, UART_IER, 0);
//  if (lsr & UART_LSR_TEMT && iir & UART_IIR_NO_INT) {
//      if (!(up->bugs & UART_BUG_TXEN)) {
//          up->bugs |= UART_BUG_TXEN;
//          pr_debug("%s - enabling bad tx status workarounds\n",
//               port->name);
//      }
//  } else {
//      up->bugs &= ~UART_BUG_TXEN;
//  }

    spin_unlock_irqrestore(&port->lock, flags);

    /*
     * Finally, enable interrupts.  Note: Modem status interrupts
     * are set via set_termios(), which will be occurring imminently
     * anyway, so we don't enable them here. ????
     */
    up->ier = UART_IER_MSI | UART_IER_RLSI | UART_IER_RDI;
    serial_out(up, UART_IER, up->ier);

    /*
     * Clear the interrupt registers again for luck, and clear the
     * saved flags to avoid getting false values from polling
     * routines or the previous session.
     */
    serial_port_in(port, UART_LSR);
    serial_port_in(port, UART_RX);
    serial_port_in(port, UART_IIR);
    serial_port_in(port, UART_MSR);

    up->lsr_saved_flags = 0;
    up->msr_saved_flags = 0;

    /*
     * Set the IER shadow for rx interrupts but defer actual interrupt
     * enable until after the FIFOs are enabled; otherwise, an already-
     * active sender can swamp the interrupt handler with "too much work".
     */
//  up->ier = UART_IER_RLSI | UART_IER_RDI;

    retval = 0;
out:
    return retval;
}

static void serialxr78x_shutdown(struct uart_port *port)
{
    struct uart_xr78x_port *up = (struct uart_xr78x_port *)port;
    unsigned long flags;

    XR_DEBUG_INTR("****** %s", __func__);
    /*
     * Disable interrupts from this port
     */
    spin_lock_irqsave(&port->lock, flags);
    up->ier = 0;
    serial_port_out(port, UART_IER, 0);
    spin_unlock_irqrestore(&port->lock, flags);

    synchronize_irq(port->irq);

    spin_lock_irqsave(&port->lock, flags);
    port->mctrl &= ~TIOCM_OUT2;

    serialxr78x_set_mctrl(port, port->mctrl);
    spin_unlock_irqrestore(&port->lock, flags);

    /*
     * Disable break condition and FIFOs
     */
    serial_port_out(port, UART_LCR,
            serial_port_in(port, UART_LCR) & ~UART_LCR_SBC);
    serialxr78x_clear_fifos(up);

    /*
     * Read data port to reset things, and then unlink from
     * the IRQ chain.
     */
    serial_port_in(port, UART_RX);

    up->ops->release_irq(up);
}

static unsigned int serialxr78x_get_divisor(struct uart_port *port, unsigned int baud)
{
    unsigned int quot;

    quot = uart_get_divisor(port, baud);

    return quot;
}

static unsigned char serialxr78x_compute_lcr(struct uart_xr78x_port *up,
                        tcflag_t c_cflag)
{
    unsigned char cval;

    switch (c_cflag & CSIZE) {
    case CS5:
        cval = UART_LCR_WLEN5;
        break;
    case CS6:
        cval = UART_LCR_WLEN6;
        break;
    case CS7:
        cval = UART_LCR_WLEN7;
        break;
    default:
    case CS8:
        cval = UART_LCR_WLEN8;
        break;
    }

    if (c_cflag & CSTOPB)
        cval |= UART_LCR_STOP;
    if (c_cflag & PARENB) {
        cval |= UART_LCR_PARITY;
//      if (up->bugs & UART_BUG_PARITY)
//          up->fifo_bug = true;
    }
    if (!(c_cflag & PARODD))
        cval |= UART_LCR_EPAR;

    return cval;
}

static void serialxr78x_set_divisor(struct uart_port *port,
        unsigned int baud, unsigned int quot)
{
    struct uart_xr78x_port *up = up_to_xr78xp(port);

    serial_port_out(port, UART_LCR, up->lcr | UART_LCR_DLAB);
    serial_dl_write(up, quot);
    serial_port_out(port, UART_LCR, up->lcr);

}

static unsigned int serialxr78x_get_baud_rate(struct uart_port *port,
                         struct ktermios *termios,
                         struct ktermios *old)
{
    /*
     * Ask the core to calculate the divisor for us.
     * Allow 1% tolerance at the upper limit so uart clks marginally
     * slower than nominal still match standard baud rates without
     * causing transmission errors.
     */
    return uart_get_baud_rate(port, termios, old,
                  port->uartclk / 16 / UART_DIV_MAX,
                  port->uartclk);
}

static void
serialxr78x_set_termios(struct uart_port *port, struct ktermios *termios,
               struct ktermios *old)
{
    struct uart_xr78x_port *up = up_to_xr78xp(port);
    unsigned char cval;
    unsigned long flags;
    unsigned int baud, quot;

    cval = serialxr78x_compute_lcr(up, termios->c_cflag);
    /*
     * Ask the core to calculate the divisor for us.
     */
    baud = serialxr78x_get_baud_rate(port, termios, old);
    quot = serialxr78x_get_divisor(port, baud);

    /*
     * Ok, we're now changing the port state.  Do it with
     * interrupts disabled.
     */
    spin_lock_irqsave(&up->port.lock, flags);

    up->lcr = cval;                 /* Save computed LCR */
    /*
     * Update the per-port timeout.
     */
    uart_update_timeout(port, termios->c_cflag, baud);

    up->port.read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
    if (termios->c_iflag & INPCK)
        up->port.read_status_mask |= UART_LSR_FE | UART_LSR_PE;
    if (termios->c_iflag & (BRKINT | PARMRK))
        up->port.read_status_mask |= UART_LSR_BI;

    /*
     * Characteres to ignore
     */
    up->port.ignore_status_mask = 0;
    if (termios->c_iflag & IGNPAR)
        up->port.ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
    if (termios->c_iflag & IGNBRK) {
        up->port.ignore_status_mask |= UART_LSR_BI;
        /*
         * If we're ignoring parity and break indicators,
         * ignore overruns too (for real raw support).
         */
        if (termios->c_iflag & IGNPAR)
            up->port.ignore_status_mask |= UART_LSR_OE;
    }

    /*
     * ignore all characters if CREAD is not set
     */
    if ((termios->c_cflag & CREAD) == 0)
        up->port.ignore_status_mask |= UART_LSR_DR;

    /*
     * CTS flow control flag and modem status interrupts
     */
    up->ier &= ~UART_IER_MSI;
    if (UART_ENABLE_MS(&up->port, termios->c_cflag))
        up->ier |= UART_IER_MSI;

    serial_out(up, UART_IER, up->ier);

    serialxr78x_set_divisor(port, baud, quot);
//  serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO);/* set fcr */
    serialxr78x_set_mctrl(port, port->mctrl);

    XR_DEBUG_INTR("****** %s:: ier=0x%x, lcr=0x%x, quot=0x%x, baud=%d",
            __func__, up->ier, cval, quot, baud);

    print_reg_info(up, 0x1FF);

    spin_unlock_irqrestore(&up->port.lock, flags);

    /* Don't rewrite B0 */
    if (tty_termios_baud_rate(termios))
        tty_termios_encode_baud_rate(termios, baud, baud);
}

static void
serialxr78x_set_ldisc(struct uart_port *port, struct ktermios *termios)
{
    if (termios->c_line == N_PPS) {
        port->flags |= UPF_HARDPPS_CD;
        spin_lock_irq(&port->lock);
        serialxr78x_enable_ms(port);
        spin_unlock_irq(&port->lock);
    } else {
        port->flags &= ~UPF_HARDPPS_CD;
        if (!UART_ENABLE_MS(port, termios->c_cflag)) {
            spin_lock_irq(&port->lock);
            serialxr78x_disable_ms(port);
            spin_unlock_irq(&port->lock);
        }
    }
}

/*
 *      EXAR ioctls
 */
#define     EXAR_READ_REG       (FIOQSIZE + 1)
#define     EXAR_WRITE_REG      (FIOQSIZE + 2)

struct xrioctl_rw_reg {
    unsigned char reg;
    unsigned char regvalue;
};

/*
 * This function is used to handle Exar Device specific ioctl calls
 * The user level application should have defined the above ioctl
 * commands with the above values to access these ioctls and the 
 * input parameters for these ioctls should be struct xrioctl_rw_reg
 * The Ioctl functioning is pretty much self explanatory here in the code,
 * and the register values should be between 0 to XR_17X15Y_EXTENDED_RXTRG
 */
static int
serialxr78x_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg)
{
    struct uart_xr78x_port *up = up_to_xr78xp(port);
    int ret = -ENOIOCTLCMD;
    struct xrioctl_rw_reg ioctlrwarg;

    switch (cmd)
    {
    case EXAR_READ_REG:
        if (copy_from_user(&ioctlrwarg, (void *)arg, sizeof(ioctlrwarg)))
            return -EFAULT;
        ioctlrwarg.regvalue = serial_in(up, ioctlrwarg.reg);
        if (copy_to_user((void *)arg, &ioctlrwarg, sizeof(ioctlrwarg)))
            return -EFAULT;
        ret = 0;
        XR_DEBUG_INTR("****** %s:: read, reg=%d, val=%d",
                __func__, ioctlrwarg.reg, ioctlrwarg.regvalue);

        break;
        
    case EXAR_WRITE_REG:

        if (copy_from_user(&ioctlrwarg, (void *)arg, sizeof(ioctlrwarg)))
            return -EFAULT;
        XR_DEBUG_INTR("****** %s:: write, reg=%d, val=%d",
                __func__, ioctlrwarg.reg, ioctlrwarg.regvalue);
        serial_out(up, ioctlrwarg.reg, ioctlrwarg.regvalue);
        ret = 0;
        break;
    }
    
    return ret;
}
          
static void
serialxr78x_pm(struct uart_port *port, unsigned int state,
          unsigned int oldstate)
{
    struct uart_xr78x_port *up = up_to_xr78xp(port);
    if (state) {
        /* sleep */
        serial_out(up, UART_IER, UART_IERX_SLEEP);
                        
        if (up->pm)
            up->pm(port, state, oldstate);
    } else {
        /* Wake up UART */
        serial_out(up, UART_IER, 0);
        
        if (up->pm)
            up->pm(port, state, oldstate);
    }
}

static unsigned int serialxr78x_port_size(struct uart_xr78x_port *pt)
{
    if (pt->port.mapsize)
        return pt->port.mapsize;

    return 8 << pt->port.regshift;
}

/*
 * Resource handling.
 */
static int serialxr78x_request_std_resource(struct uart_xr78x_port *up)
{
    unsigned int size = serialxr78x_port_size(up);
    struct uart_port *port = &up->port;
    int ret = 0;

    switch (port->iotype) {
    case UPIO_MEM:
        if (!port->mapbase)
            break;

        if (!request_mem_region(port->mapbase, size, "xrserial")) {
            XR_DEBUG_INTR("****%s:: request mem region failed.", __func__);
            ret = -EBUSY;
            break;
        }

        port->membase = ioremap_nocache(port->mapbase, size);
        if (!port->membase) {
            release_mem_region(port->mapbase, size);
            ret = -ENOMEM;
        }
        break;

    case UPIO_HUB6:
    case UPIO_PORT:
        if (!request_region(port->iobase, size, "xrserial"))
            ret = -EBUSY;
        break;
    }
    return ret;
}

static void serialxr78x_release_std_resource(struct uart_xr78x_port *up)
{
    unsigned int size = serialxr78x_port_size(up);
    struct uart_port *port = &up->port;

    switch (port->iotype) {
    case UPIO_MEM:
        if (!port->mapbase)
            break;

        iounmap(port->membase);
        port->membase = NULL;

        release_mem_region(port->mapbase, size);
        break;

    case UPIO_HUB6:
    case UPIO_PORT:
        release_region(port->iobase, size);
        break;
    }
}


static void serialxr78x_release_port(struct uart_port *port)
{   
    serialxr78x_release_std_resource(up_to_xr78xp(port));
}

static int serialxr78x_request_port(struct uart_port *port)
{
    return serialxr78x_request_std_resource(up_to_xr78xp(port));
}

static void serialxr78x_config_port(struct uart_port *port, int flags)
{
    struct uart_xr78x_port *up = up_to_xr78xp(port);
    
    XR_DEBUG_INTR("****** %s", __func__);
    if (flags & UART_CONFIG_TYPE) {
        up->port.type = XR788_TYPE;
        up->port.fifosize = uart_config[up->port.type].dfl_xmit_fifo_size;
        up->capabilities = uart_config[up->port.type].flags;    
    }
}

static const char *
serialxr78x_type(struct uart_port *port)
{
    int type = port->type;
    
    if (type >= ARRAY_SIZE(uart_config))
        type = 0;
    return uart_config[type].name;
}

/*
 * This handles the interrupt from one port.
 */
static int serialxr78x_handle_irq(struct uart_port *port, unsigned int iir)
{
    unsigned char status;
    unsigned long flags;
    struct uart_xr78x_port *up = up_to_xr78xp(port);

    XR_DEBUG_INTR("****** %s, iir=%d", __func__, iir);
    if (iir & UART_IIR_NO_INT)
        return 0;

    spin_lock_irqsave(&port->lock, flags);

    status = serial_port_in(port, UART_LSR);

    if (status & (UART_LSR_DR | UART_LSR_BI) &&
        iir & UART_IIR_RDI) {
        status = serialxr78x_rx_chars(up, status);
    }
    serialxr78x_modem_status(up);
    if (status & UART_LSR_THRE)
        serialxr78x_tx_chars(up);

    spin_unlock_irqrestore(&port->lock, flags);
    return 1;
}

static int serialxr78x_default_handle_irq(struct uart_port *port)
{
    unsigned int iir;
    int ret;

    iir = serial_port_in(port, UART_IIR);
    ret = serialxr78x_handle_irq(port, iir);

    return ret;
}

static int serialx78x_setup_irq(struct uart_xr78x_port *up)
{
    struct uart_port *port = &up->port;
    int retval = 0;

    XR_DEBUG_INTR("****** %s", __func__);
    /*
     * The above check will only give an accurate result the first time
     * the port is opened so this value needs to be preserved.
     */
//  if (up->bugs & UART_BUG_THRE) {
//      pr_debug("%s - using backup timer\n", port->name);
//
//      up->timer.function = serialxr78x_backup_timeout;
//      mod_timer(&up->timer, jiffies +
//            uart_poll_timeout(port) + HZ / 5);
//  }

    /*
     * If the "interrupt" for this port doesn't correspond with any
     * hardware interrupt, we use a timer-based system.  The original
     * driver used to do this with IRQ0.
     */
    if (!port->irq) {
        mod_timer(&up->timer, jiffies + uart_poll_timeout(port));
    } else
        retval = serial_link_irq_chain(up);

    return retval;
}

static void serialxr78x_release_irq(struct uart_xr78x_port *up)
{
    struct uart_port *port = &up->port;

    XR_DEBUG_INTR("****** %s", __func__);
    del_timer_sync(&up->timer);
    up->timer.function = serialxr78x_timeout;
    if (port->irq)
        serial_unlink_irq_chain(up);
}

static const struct uart_xr78x_ops ct_xr78x_port_ops = {
    .setup_irq      = serialx78x_setup_irq,
    .release_irq    = serialxr78x_release_irq,
};

static struct uart_ops serialxr78x_pops = {
    .tx_empty   = serialxr78x_tx_empty,
    .set_mctrl  = serialxr78x_set_mctrl,
    .get_mctrl  = serialxr78x_get_mctrl,
    .stop_tx    = serialxr78x_stop_tx,
    .start_tx   = serialxr78x_start_tx,
    .stop_rx    = serialxr78x_stop_rx,
    .enable_ms  = serialxr78x_enable_ms,
    .break_ctl  = serialxr78x_break_ctl,
    .startup    = serialxr78x_startup,
    .shutdown   = serialxr78x_shutdown,
    .set_termios    = serialxr78x_set_termios,
    .pm             = serialxr78x_pm,
    .type           = serialxr78x_type,
    .release_port   = serialxr78x_release_port,
    .request_port   = serialxr78x_request_port,
    .config_port    = serialxr78x_config_port,
    .ioctl          = serialxr78x_ioctl,
};

static struct uart_xr78x_port serialxr78x_ports[XR788_COUNT][UART_XR788_NR];

static unsigned int ports_inited[XR788_COUNT] = {0, 0, 0};

static inline void serialxr78x_init_port(struct uart_xr78x_port *up)
{
    struct uart_port *port = &up->port;

    memset(up, 0, sizeof(struct uart_xr78x_port));

    spin_lock_init(&up->port.lock);
    port->membase   = NULL;
    port->irq   = 0;
    port->type  = XR788_TYPE;
    port->flags = UPF_BOOT_AUTOCONF | ASYNC_SKIP_TEST;
    port->fifosize  = XR78X_UART_FIFO_SIZE;
    port->uartclk  = 921600 * 16;
    if (share_irqs)
        up->port.flags |= UPF_SHARE_IRQ;

    port->ops   = &serialxr78x_pops;

    up->mcr_mask = 0x13;    // (UART_MCR_LOOP | UART_MCR_RTS | UART_MCR_DTR)
    up->ops = &ct_xr78x_port_ops;
    up->tx_loadsz = XR78X_UART_FIFO_SIZE;
}

static void __init
serial78x_setup_ports(struct platform_device *dev, unsigned int dev_num)
{
    struct uart_xr78x_port *up;
    struct resource *res;
    unsigned int irq;
    int i;
    int rc;

    XR_DEBUG_INTR("****** %s", __func__);
    if (ports_inited[dev_num] == 1) {
        printk("Chip %d inited!\n", dev_num);
        return;
    }

    res = platform_get_resource(dev, IORESOURCE_MEM, 0);
    if (!res) {
        rc = -ENODEV;
        dev_info(&dev->dev, "%s:: No memory resource\n", __func__);
        return ;
    }
    XR_DEBUG_INTR("***** %s:: resource: start=0x%x", __func__, res->start);
    irq = platform_get_irq(dev, 0);
    XR_DEBUG_INTR("***** irq=0x%x", irq);

    for (i = 0, up = serialxr78x_ports[dev_num]; i < UART_XR788_NR;
         i++, up++) {
        serialxr78x_init_port(up);

        up->port.mapbase   = res->start+ (i * XR78X_UART_REG_SPACE);
        up->port.mapsize = XR78X_UART_REG_SPACE;
        up->port.irq      = irq;
        up->port.iotype   = UPIO_MEM;
        up->port.regshift = 0;
        up->port.line = i;

        set_io_from_upio(& up->port);
    }

    // set to the port initialized.
    ports_inited[dev_num] = 1;
}

static void __init
serialxr78x_register_ports(struct platform_device *dev,
        struct uart_driver *drv, unsigned int dev_num)
{
    int i;
    
    dev_info(& dev->dev, "***xr16l788:: register ports. dev_num=%d", dev_num);

    serial78x_setup_ports(dev, dev_num);
    
    for (i = 0; i < UART_XR788_NR; i++) {
        struct uart_xr78x_port *up = &serialxr78x_ports[dev_num][i];

        timer_setup(&up->timer, serialxr78x_timeout, 0);
        up->port.dev = &(dev->dev);

        uart_add_one_port(drv, &up->port);
        XR_DEBUG_INTR("***** mapbase=0x%x, membase=0x%x",
            up->port.mapbase, (unsigned int) up->port.membase);
    }
}

#define SERIALXR_CONSOLE    NULL
static struct uart_driver xr16l78x_uart_driver[] = {
    [0] = {
        .owner          = THIS_MODULE,
        .driver_name        = "xrserialA",
        .dev_name       = "ttyExA",
//      .dev_name       = "ttyS",
        .major          = XR_788_MAJOR,
        .minor          = XR_788_MINOR,
        .nr         	= UART_XR788_NR,
        .cons           = SERIALXR_CONSOLE,
    },
    [1] = {
        .owner          = THIS_MODULE,
        .driver_name        = "xrserialB",
        .dev_name       = "ttyExB",
//      .dev_name       = "ttyS",
        .major          = XR_788_MAJOR,
        .minor          = XR_788_MINOR +  UART_XR788_NR,
        .nr         	= UART_XR788_NR,
        .cons           = SERIALXR_CONSOLE,
    },
    [2] = {
        .owner          = THIS_MODULE,
        .driver_name        = "xrserialC",
        .dev_name       = "ttyExC",
//      .dev_name       = "ttyS",
        .major          = XR_788_MAJOR,
        .minor          = XR_788_MINOR + 2 * UART_XR788_NR,
        .nr         	= UART_XR788_NR,
        .cons           = SERIALXR_CONSOLE,
    },
};

static void serialxr78x_test_chip(struct resource *res)
{
    void* mapped_addr;
    unsigned int addr;
    unsigned char drev, dvid;
    int i;

    if (!res) return;

    addr = res->start;
    mapped_addr = ioremap(addr, resource_size(res));
    if (!mapped_addr) {
        XR_DEBUG_INTR("Failed to remap the memory. addr=0x%x", addr);
        return;
    }

    for (i = 0; i < 16; i++) {
        drev = readb(mapped_addr + i);
        XR_DEBUG_INTR("***** %s:: addr=0x%x, mapped=0x%x, val=0x%x",
            __func__, addr, mapped_addr, drev);
    }

    drev = readb(mapped_addr + 0x8C);
    dvid = readb(mapped_addr + 0x8D);
    XR_DEBUG_INTR("***** %s:: drev=0x%x, dvid=0x%x",
            __func__,  drev, dvid);

    iounmap(mapped_addr);
}

static int serialxr78x_probe(struct platform_device *dev)
{
    int ret;
    unsigned int devid;
    struct resource *res;
#if 0
    unsigned int irq;
#endif

    dev_info(& dev->dev, "***xr16l788 chip:: start probe.");
    res = platform_get_resource(dev, IORESOURCE_MEM, 0);
    if(res == NULL) {
        dev_err(& dev->dev, "Can't get resource!\n");
        return -1;
    }
    XR_DEBUG_INTR("xr16l78x: res start 0x%08x", res->start);

    devid = 0;

#if 0
    serialxr78x_register_ports(dev, &xr16l78x_uart_driver[devid], devid);
#else
    serialxr78x_test_chip(res);
#endif

    return 0;
}

static void
serialxr78x_unregister_ports(struct uart_driver *drv, unsigned int dev_num)
{
	int i;

	for (i = 0; i < UART_XR788_NR; i++)
		uart_remove_one_port(drv, &serialxr78x_ports[dev_num][i].port);
}

static int serialxr78x_remove(struct platform_device *dev)
{
#if 0
	serialxr78x_unregister_ports(&xr16l78x_uart_driver[0], 0);
#endif
    return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id xr16l78x_uart_of_match[] = {
    { .compatible = "exar,xr16l788", },
    {}
};
MODULE_DEVICE_TABLE(of, xr16l78x_uart_of_match);

static struct platform_driver xr16l78x_uart_of_driver = {
    .probe = serialxr78x_probe,
    .remove = serialxr78x_remove,
    .driver = {
        .name = "xr16l788",
        .of_match_table = xr16l78x_uart_of_match,
    },
};

static int __init serialxr78x_init(void)
{
    int ret;

    printk(KERN_INFO "Exar XR16L78x specific serial driver $Revision: 1.0 $ "
        "%d ports, IRQ sharing %sabled\n", (int) UART_XR788_NR,
        share_irqs ? "en" : "dis");

    ret = uart_register_driver(&xr16l78x_uart_driver[0]);
    if(ret < 0) {
        printk("Fail to Register driver!\n");
        return ret;
    }
    ret = platform_driver_register(&xr16l78x_uart_of_driver);
    if(ret < 0) {
        printk("xa16l78x: Fail to Register platform driver!\n");
        return ret;
    }
    
    if (ret >= 0) printk("xa16l78x: Driver Register Successfully!\n");
        
    return 0;
}

static void __exit serialxr78x_exit(void)
{
    uart_unregister_driver(&xr16l78x_uart_driver[0]);
    platform_driver_unregister(&xr16l78x_uart_of_driver);
}

module_init(serialxr78x_init);
module_exit(serialxr78x_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Exar XR16L78x serial driver: for 3 chips on FFC4  $");
