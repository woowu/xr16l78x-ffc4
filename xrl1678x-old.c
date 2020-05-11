/*
*      	xr1678x.c  -- EXAR multiport serial driver for XR16L78x family of UARTS.
*	Base on EXAR's XR16L78X driver V1.0
*
*/

//#include <linux/config.h>
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
//#include <linux/slab.h>
//#include <linux/smp_lock.h>
#include <linux/platform_device.h>

#include <linux/serial_reg.h>
#include <linux/serial.h>
#include <linux/serialP.h>
#include <linux/serial_core.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/irq.h>
#include <asm/bitops.h>
#include <asm/byteorder.h>
#include <asm/serial.h>
#include <asm/io.h>
#include <asm/uaccess.h>

//#include "xr1678x.h"

/*------Define a slab cache structure-----------*/
//static struct kmem_cache *xr788_drv_cachep;
//static struct kmem_cache *xr788_tty_cachep;


#define _INLINE_ inline

#define SERIALEXAR_SHARE_IRQS 1 
unsigned int share_irqs = SERIALEXAR_SHARE_IRQS;

#define UART_XR788_NR	8 // MAX 8 ports per card for 788:

#define XR_788_MAJOR       40
#define XR_788_MINOR       0
//#define XR_788_MAJOR       4
//#define XR_788_MINOR       64

#define PASS_LIMIT	256
#define XR788_COUNT 3

#if 0
#define DEBUG_INTR(fmt...)	printk(fmt)
#else
#define DEBUG_INTR(fmt...)	do { } while (0)
#endif

/*
 * We default to IRQ0 for the "no irq" hack.   Some
 * machine types want others as well - they're free
 * to redefine this in their header file.
 */
#define is_real_interrupt(irq)	((irq) != 0)


struct uart_xr_port {
	struct uart_port	port;
	struct timer_list	timer;		/* "no irq" timer */
	struct list_head	list;		/* ports on this IRQ */
	unsigned int		capabilities;	/* port capabilities */
	unsigned short		rev;
	unsigned int 		tx_loadsz;
	unsigned char		acr;
	unsigned char		ier;
	unsigned char		lcr;
	unsigned char		mcr_mask;	/* mask of user bits */
	unsigned char		mcr_force;	/* mask of forced bits */
	unsigned char		lsr_break_flag;

	/*
	 * We provide a per-port pm hook.
	 */
	void			(*pm)(struct uart_port *port,
				      unsigned int state, unsigned int old);
};

struct irq_info {
	spinlock_t		lock;
	struct list_head	*head;
};

static struct irq_info irq_lists[NR_IRQS];

/*
 * Here we define the default xmit fifo size used for each type of UART.
 */
#define PORT_MAX_XR 1 
#define XR788_TYPE 1 // the second entry that is [1] in the array
static const struct serial_uart_config uart_config[PORT_MAX_XR+1] = {
	{ "Unknown",	1,	0 },
	{ "XR78x",		64,	0 },
};

static _INLINE_ unsigned int serial_in(struct uart_xr_port *up, int offset)
{
    int ret;
	offset <<= up->port.regshift;

	switch (up->port.iotype) {
	case SERIAL_IO_HUB6:
		outb(up->port.hub6 - 1 + offset, up->port.iobase);
		return inb(up->port.iobase + 1);

	case SERIAL_IO_MEM:
        ret = readb(up->port.membase + offset);
#if 0
        printk("*** readb, addr=0x%08x, return value=0x%08x\n",
                (unsigned int)(up->port.membase + offset), ret);
#endif
		return ret;

	default:
		return inb(up->port.iobase + offset);
	}
}

static _INLINE_ void
serial_out(struct uart_xr_port *up, int offset, int value)
{
	offset <<= up->port.regshift;

	switch (up->port.iotype) {
	case SERIAL_IO_HUB6:
		outb(up->port.hub6 - 1 + offset, up->port.iobase);
		outb(value, up->port.iobase + 1);
		break;

	case SERIAL_IO_MEM:
#if 0
        printk("*** writeb, addr=0x%08x, value=0x%08x\n",
                (unsigned int)(up->port.membase + offset), value);
#endif
		writeb(value, up->port.membase + offset);
		break;

	default:
		outb(value, up->port.iobase + offset);
	}
}

/*
 * We used to support using pause I/O for certain machines.  We
 * haven't supported this for a while, but just in case it's badly
 * needed for certain old 386 machines, I've left these #define's
 * in....
 */
#define serial_inp(up, offset)		serial_in(up, offset)
#define serial_outp(up, offset, value)	serial_out(up, offset, value)

//static void serialxr78x_stop_tx(struct uart_port *port, unsigned int tty_stop)
static void serialxr78x_stop_tx(struct uart_port *port)
{
	struct uart_xr_port *up = (struct uart_xr_port *)port;

	if (up->ier & UART_IER_THRI) {
		up->ier &= ~UART_IER_THRI;
		serial_out(up, UART_IER, up->ier);
	}
}

static void serialxr78x_start_tx(struct uart_port *port)
{
	struct uart_xr_port *up = (struct uart_xr_port *)port;

	if (!(up->ier & UART_IER_THRI)) {
		up->ier |= UART_IER_THRI;
		serial_out(up, UART_IER, up->ier);
	}
}

static void serialxr78x_stop_rx(struct uart_port *port)
{
	struct uart_xr_port *up = (struct uart_xr_port *)port;

	up->ier &= ~UART_IER_RLSI;
	up->port.read_status_mask &= ~UART_LSR_DR;
	serial_out(up, UART_IER, up->ier);
}

static void serialxr78x_enable_ms(struct uart_port *port)
{
	struct uart_xr_port *up = (struct uart_xr_port *)port;

	up->ier |= UART_IER_MSI;
	serial_out(up, UART_IER, up->ier);
}
#if 0
static _INLINE_ void
receive_chars(struct uart_xr_port *up, int *status, struct pt_regs *regs)
{
	struct tty_struct *tty = up->port.state->port.tty;
	unsigned char ch;
	int max_count = 256;
	char flag;

	do {
//		if (unlikely(tty->flip.count >= TTY_FLIPBUF_SIZE)) {
//			tty->flip.work.func((void *)tty);
//			if (tty->flip.count >= TTY_FLIPBUF_SIZE)
//				return; // if TTY_DONT_FLIP is set
//		}
		ch = serial_inp(up, UART_RX);
//		*tty->flip.char_buf_ptr = ch;
//		*tty->flip.flag_buf_ptr = TTY_NORMAL;
//		up->port.icount.rx++;

		if (unlikely(*status & (UART_LSR_BI | UART_LSR_PE |
				       UART_LSR_FE | UART_LSR_OE))) {
			/*
			 * For statistics only
			 */
			if (*status & UART_LSR_BI) {
				*status &= ~(UART_LSR_FE | UART_LSR_PE);
				up->port.icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
				if (uart_handle_break(&up->port))
					goto ignore_char;
			} else if (*status & UART_LSR_PE)
				up->port.icount.parity++;
			else if (*status & UART_LSR_FE)
				up->port.icount.frame++;
			if (*status & UART_LSR_OE)
				up->port.icount.overrun++;

			/*
			 * Mask off conditions which should be ingored.
			 */
			*status &= up->port.read_status_mask;

			if (*status & UART_LSR_BI) {
//				*tty->flip.flag_buf_ptr = TTY_BREAK;
				 flag = TTY_BREAK;
			} else if (*status & UART_LSR_PE)
//				*tty->flip.flag_buf_ptr = TTY_PARITY;
				flag = TTY_PARITY;
			else if (*status & UART_LSR_FE)
			//	*tty->flip.flag_buf_ptr = TTY_FRAME;
				flag = TTY_FRAME;
		}
		if (uart_handle_sysrq_char(&up->port, ch))
			goto ignore_char;
#if 0	
		if ((*status & up->port.ignore_status_mask) == 0) {
			tty->flip.flag_buf_ptr++;
			tty->flip.char_buf_ptr++;
			tty->flip.count++;
		}
		if ((*status & UART_LSR_OE) &&
		    tty->flip.count < TTY_FLIPBUF_SIZE) {
			/*
			 * Overrun is special, since it's reported
			 * immediately, and doesn't affect the current
			 * character.
			 */
			*tty->flip.flag_buf_ptr = TTY_OVERRUN;
			tty->flip.flag_buf_ptr++;
			tty->flip.char_buf_ptr++;
			tty->flip.count++;
		}
#endif
		uart_insert_char(&up->port, status, UART_LSR_OE, ch, flag);
	ignore_char:
		*status = serial_inp(up, UART_LSR);
	} while ((*status & UART_LSR_DR) && (max_count-- > 0));
	tty_flip_buffer_push(tty);
}
#endif
static _INLINE_ void
receive_chars(struct uart_xr_port *up, int *status)
{
//	struct tty_struct *tty = up->port.info->tty;
	struct tty_struct *tty = up->port.state->port.tty;
	unsigned char ch, lsr = *status;
	int max_count = 256;
	char flag;

	do {
		ch = serial_inp(up, UART_RX);
		flag = TTY_NORMAL;
		up->port.icount.rx++;

		if (unlikely(lsr & (UART_LSR_BI | UART_LSR_PE |
				    UART_LSR_FE | UART_LSR_OE))) {
			/*
			 * For statistics only
			 */
			if (lsr & UART_LSR_BI) {
				lsr &= ~(UART_LSR_FE | UART_LSR_PE);
				up->port.icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
				if (uart_handle_break(&up->port))
					goto ignore_char;
			} else if (lsr & UART_LSR_PE)
				up->port.icount.parity++;
			else if (lsr & UART_LSR_FE)
				up->port.icount.frame++;
			if (lsr & UART_LSR_OE)
				up->port.icount.overrun++;

			/*
			 * Mask off conditions which should be ignored.
			 */
			lsr &= up->port.read_status_mask;

			if (lsr & UART_LSR_BI) {
				DEBUG_INTR("handling break....");
				flag = TTY_BREAK;
			} else if (lsr & UART_LSR_PE)
				flag = TTY_PARITY;
			else if (lsr & UART_LSR_FE)
				flag = TTY_FRAME;
		}
		if (uart_handle_sysrq_char(&up->port, ch))
			goto ignore_char;

		uart_insert_char(&up->port, lsr, UART_LSR_OE, ch, flag);

	ignore_char:
		lsr = serial_inp(up, UART_LSR);
	} while ((lsr & UART_LSR_DR) && (max_count-- > 0));
	spin_unlock(&up->port.lock);
	tty_flip_buffer_push(tty);
	spin_lock(&up->port.lock);
	*status = lsr;
}
#if 0
static _INLINE_ void transmit_chars(struct uart_xr_port *up)
{
	struct circ_buf *xmit = &up->port.info->xmit;
	int count;

	if (up->port.x_char) {
		serial_outp(up, UART_TX, up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(&up->port)) {
		serialxr78x_stop_tx(&up->port, 0);
		return;
	}

	count = up->port.fifosize;
	do {
		serial_out(up, UART_TX, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		up->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);
	
	if (uart_circ_empty(xmit))
		serialxr78x_stop_tx(&up->port, 0);
}
#endif
static _INLINE_ void transmit_chars(struct uart_xr_port *up)
{
//	struct circ_buf *xmit = &up->port.info->xmit;
	struct circ_buf *xmit = &up->port.state->xmit;
	int count;	

	if (uart_circ_empty(xmit) || uart_tx_stopped(&up->port)) {
		serialxr78x_stop_tx(&up->port);
		return;
	}
		
	count = up->tx_loadsz;
	do {
		serial_out(up, UART_TX, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		up->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);


	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	DEBUG_INTR("THRE...");

	if (uart_circ_empty(xmit))
		serialxr78x_stop_tx(&up->port);
}
static _INLINE_ void check_modem_status(struct uart_xr_port *up)
{
	int status;

	status = serial_in(up, UART_MSR);

	if ((status & UART_MSR_ANY_DELTA) == 0)
		return;

	if (status & UART_MSR_TERI)
		up->port.icount.rng++;
	if (status & UART_MSR_DDSR)
		up->port.icount.dsr++;
	if (status & UART_MSR_DDCD)
		uart_handle_dcd_change(&up->port, status & UART_MSR_DCD);
	if (status & UART_MSR_DCTS)
		uart_handle_cts_change(&up->port, status & UART_MSR_CTS);

	wake_up_interruptible(&up->port.state->port.delta_msr_wait);
}

/*
 * This handles the interrupt from one port.
 */
static inline void
//serialxr78x_handle_port(struct uart_xr_port *up, struct pt_regs *regs)
serialxr78x_handle_port(struct uart_xr_port *up)
{
	unsigned int status = serial_inp(up, UART_LSR);

	if (status & UART_LSR_DR)
//		receive_chars(up, &status, regs);
		receive_chars(up, &status);
	check_modem_status(up);
	if (status & UART_LSR_THRE)
		transmit_chars(up);
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
//static irqreturn_t serialxr78x_interrupt(int irq, void *dev_id, struct pt_regs *regs)
static irqreturn_t serialxr78x_interrupt(int irq, void *dev_id)
{
	struct irq_info *i = dev_id;
	struct list_head *l, *end = NULL;
	int pass_counter = 0;

	//DEBUG_INTR("serialxr78x_interrupt(%d)...", irq);

	spin_lock(&i->lock);

	l = i->head;
	do {
		struct uart_xr_port *up;
		unsigned int iir;

		up = list_entry(l, struct uart_xr_port, list);

		iir = serial_in(up, UART_IIR);
		if (!(iir & UART_IIR_NO_INT)) {
			spin_lock(&up->port.lock);
//			serialxr78x_handle_port(up, regs);
			serialxr78x_handle_port(up);
			spin_unlock(&up->port.lock);

			end = NULL;
		} else if (end == NULL)
			end = l;

		l = l->next;

		if (l == i->head && pass_counter++ > PASS_LIMIT) {
			/* If we hit this, we're dead. */
			printk(KERN_ERR "serialxr78x: too much work for "
				"irq%d\n", irq);
			break;
		}
	} while (l != end);

	spin_unlock(&i->lock);

	//DEBUG_INTR("end.\n");
	/* FIXME! Was it really ours? */
	return IRQ_HANDLED;
}

/*
 * To support ISA shared interrupts, we need to have one interrupt
 * handler that ensures that the IRQ line has been deasserted
 * before returning.  Failing to do this will result in the IRQ
 * line being stuck active, and, since ISA irqs are edge triggered,
 * no more IRQs will be seen.
 */
static void serial_do_unlink(struct irq_info *i, struct uart_xr_port *up)
{
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
}

static int serial_link_irq_chain(struct uart_xr_port *up)
{
	struct irq_info *i = irq_lists + up->port.irq;
	int ret, irq_flags = up->port.flags & UPF_SHARE_IRQ ? IRQF_SHARED : 0;

	spin_lock_irq(&i->lock);

	if (i->head) {
		list_add(&up->list, i->head);
		spin_unlock_irq(&i->lock);

		ret = 0;
	} else {
		INIT_LIST_HEAD(&up->list);
		i->head = &up->list;
		spin_unlock_irq(&i->lock);

		ret = request_irq(up->port.irq, serialxr78x_interrupt,
				  irq_flags, "xrserial", i);
		if (ret < 0)
			serial_do_unlink(i, up);
	}

	return ret;
}

static void serial_unlink_irq_chain(struct uart_xr_port *up)
{
	struct irq_info *i = irq_lists + up->port.irq;

	BUG_ON(i->head == NULL);

	if (list_empty(i->head))
		free_irq(up->port.irq, i);

	serial_do_unlink(i, up);
}

/*
 * This function is used to handle ports that do not have an
 * interrupt.  This doesn't work very well for 16450's, but gives
 * barely passable results for a 16550A.  (Although at the expense
 * of much CPU overhead).
 */
static void serialxr78x_timeout(unsigned long data)
{
	struct uart_xr_port *up = (struct uart_xr_port *)data;
	unsigned int timeout;
	unsigned int iir;
	
	iir = serial_in(up, UART_IIR);
	if (!(iir & UART_IIR_NO_INT)) {
		spin_lock(&up->port.lock);
//		serialxr78x_handle_port(up, NULL);
		serialxr78x_handle_port(up);
		spin_unlock(&up->port.lock);
	}

	timeout = up->port.timeout;
	timeout = timeout > 6 ? (timeout / 2 - 2) : 1;
	mod_timer(&up->timer, jiffies + timeout);
}

static unsigned int serialxr78x_tx_empty(struct uart_port *port)
{
	struct uart_xr_port *up = (struct uart_xr_port *)port;
	unsigned long flags;
	unsigned int ret;

	spin_lock_irqsave(&up->port.lock, flags);
	ret = serial_in(up, UART_LSR) & UART_LSR_TEMT ? TIOCSER_TEMT : 0;
	spin_unlock_irqrestore(&up->port.lock, flags);

	return ret;
}

static unsigned int serialxr78x_get_mctrl(struct uart_port *port)
{
	struct uart_xr_port *up = (struct uart_xr_port *)port;
	unsigned long flags;
	unsigned char status;
	unsigned int ret;

	spin_lock_irqsave(&up->port.lock, flags);
	status = serial_in(up, UART_MSR);
	spin_unlock_irqrestore(&up->port.lock, flags);

	ret = 0;
	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;
#if 0
    printk("*** 78x_get_mctrl, port=%p, return value=0x%08x\n", port, ret);
#endif
	return ret;
}

static void serialxr78x_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_xr_port *up = (struct uart_xr_port *)port;
	unsigned char mcr = 0;

#if 0
    printk("*** 78x_set_mctrl, port=%p, vaule=0x%08x\n",
            port, mctrl);
#endif
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

	mcr = (mcr & up->mcr_mask) | up->mcr_force;
	serial_out(up, UART_MCR, mcr);
}

static void serialxr78x_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_xr_port *up = (struct uart_xr_port *)port;
	unsigned long flags;

	spin_lock_irqsave(&up->port.lock, flags);
	if (break_state == -1)
		up->lcr |= UART_LCR_SBC;
	else
		up->lcr &= ~UART_LCR_SBC;
	serial_out(up, UART_LCR, up->lcr);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

static int serialxr78x_startup(struct uart_port *port)
{
	struct uart_xr_port *up = (struct uart_xr_port *)port;
	unsigned long flags;
	int retval;

	up->capabilities = uart_config[up->port.type].flags;

	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reeanbled in set_termios())
	 */
	serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO |
			UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	serial_outp(up, UART_FCR, 0);
	
	/*
	 * Clear the interrupt registers.
	 */
	(void) serial_inp(up, UART_LSR);
	(void) serial_inp(up, UART_RX);
	(void) serial_inp(up, UART_IIR);
	(void) serial_inp(up, UART_MSR);

	/*
	 * If the "interrupt" for this port doesn't correspond with any
	 * hardware interrupt, we use a timer-based system.  The original
	 * driver used to do this with IRQ0.
	 */
	if (!is_real_interrupt(up->port.irq)) {
		unsigned int timeout = up->port.timeout;

		timeout = timeout > 6 ? (timeout / 2 - 2) : 1;

		up->timer.data = (unsigned long)up;
		mod_timer(&up->timer, jiffies + timeout);
	} else {
		retval = serial_link_irq_chain(up);
		if (retval)
			return retval;
	}

	/*
	 * Now, initialize the UART
	 */
	serial_outp(up, UART_LCR, UART_LCR_WLEN8);

	spin_lock_irqsave(&up->port.lock, flags);
		
	/*
	* Most PC uarts need OUT2 raised to enable interrupts.
	*/
	if (is_real_interrupt(up->port.irq))
		up->port.mctrl |= TIOCM_OUT2;

	serialxr78x_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	/*
	 * Finally, enable interrupts.  Note: Modem status interrupts
	 * are set via set_termios(), which will be occurring imminently
	 * anyway, so we don't enable them here.
	 */
	up->ier = UART_IER_MSI | UART_IER_RLSI | UART_IER_RDI;	
	serial_outp(up, UART_IER, up->ier);

	/*
	 * And clear the interrupt registers again for luck.
	 */
	(void) serial_inp(up, UART_LSR);
	(void) serial_inp(up, UART_RX);
	(void) serial_inp(up, UART_IIR);
	(void) serial_inp(up, UART_MSR);

	return 0;
}

static void serialxr78x_shutdown(struct uart_port *port)
{
	struct uart_xr_port *up = (struct uart_xr_port *)port;
	unsigned long flags;
	
	/*
	 * Disable interrupts from this port
	 */
	up->ier = 0;
	serial_outp(up, UART_IER, 0);

	spin_lock_irqsave(&up->port.lock, flags);
	
	up->port.mctrl &= ~TIOCM_OUT2;

	serialxr78x_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	/*
	 * Disable break condition and FIFOs
	 */
	serial_out(up, UART_LCR, serial_inp(up, UART_LCR) & ~UART_LCR_SBC);
	serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO |
				  UART_FCR_CLEAR_RCVR |
				  UART_FCR_CLEAR_XMIT);
	serial_outp(up, UART_FCR, 0);

	/*
	 * Read data port to reset things, and then unlink from
	 * the IRQ chain.
	 */
	(void) serial_in(up, UART_RX);

	if (!is_real_interrupt(up->port.irq))
		del_timer_sync(&up->timer);
	else
		serial_unlink_irq_chain(up);
}

static unsigned int serialxr78x_get_divisor(struct uart_port *port, unsigned int baud)
{
	unsigned int quot;

	quot = uart_get_divisor(port, baud);

	return quot;
}

static void
serialxr78x_set_termios(struct uart_port *port, struct ktermios *termios,
		       struct ktermios *old)
{
	struct uart_xr_port *up = (struct uart_xr_port *)port;
	unsigned char cval;
	unsigned long flags;
	unsigned int baud, quot;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		cval = 0x00;
		break;
	case CS6:
		cval = 0x01;
		break;
	case CS7:
		cval = 0x02;
		break;
	default:
	case CS8:
		cval = 0x03;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		cval |= 0x04;
	if (termios->c_cflag & PARENB)
		cval |= UART_LCR_PARITY;
	if (!(termios->c_cflag & PARODD))
		cval |= UART_LCR_EPAR;
#ifdef CMSPAR
	if (termios->c_cflag & CMSPAR)
		cval |= UART_LCR_SPAR;
#endif

	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16); 
	quot = serialxr78x_get_divisor(port, baud);
	
	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&up->port.lock, flags);

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

	serial_outp(up, UART_LCR, cval | UART_LCR_DLAB);/* set DLAB */
	
	serial_outp(up, UART_DLL, quot & 0xff);		/* LS of divisor */
	serial_outp(up, UART_DLM, quot >> 8);		/* MS of divisor */
	serial_outp(up, UART_LCR, cval);		/* reset DLAB */
	up->lcr = cval;					/* Save LCR */
	
	serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO);/* set fcr */
	
	spin_unlock_irqrestore(&up->port.lock, flags);
}

/*
 *      EXAR ioctls
 */
//#define 	FIOQSIZE		0x5460 
#define		EXAR_READ_REG      	(FIOQSIZE + 1)
#define 	EXAR_WRITE_REG     	(FIOQSIZE + 2)

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
	struct uart_xr_port *up = (struct uart_xr_port *)port;
	int ret = -ENOIOCTLCMD;
	struct xrioctl_rw_reg ioctlrwarg;

	switch (cmd)
	{
		case EXAR_READ_REG:
		if (copy_from_user(&ioctlrwarg, (void *)arg, sizeof(ioctlrwarg)))
			return -EFAULT;
		ioctlrwarg.regvalue = serial_inp(up, ioctlrwarg.reg);
		if (copy_to_user((void *)arg, &ioctlrwarg, sizeof(ioctlrwarg)))
			return -EFAULT;
		ret = 0;
		break;
		
		case EXAR_WRITE_REG:
		if (copy_from_user(&ioctlrwarg, (void *)arg, sizeof(ioctlrwarg)))
			return -EFAULT;
		serial_outp(up, ioctlrwarg.reg, ioctlrwarg.regvalue);
		ret = 0;
		break;
	}
	
	return ret;
}
	      
static void
serialxr78x_pm(struct uart_port *port, unsigned int state,
	      unsigned int oldstate)
{
	struct uart_xr_port *up = (struct uart_xr_port *)port;
	if (state) {
		/* sleep */
		serial_outp(up, UART_IER, UART_IERX_SLEEP);
						
		if (up->pm)
			up->pm(port, state, oldstate);
	} else {
		/* wake */
		
		/* Wake up UART */
		serial_outp(up, UART_IER, 0);
		
		if (up->pm)
			up->pm(port, state, oldstate);
	}
}

static void serialxr78x_release_port(struct uart_port *port)
{	
}

static int serialxr78x_request_port(struct uart_port *port)
{
	return 0;
}

static void serialxr78x_config_port(struct uart_port *port, int flags)
{
	struct uart_xr_port *up = (struct uart_xr_port *)port;	
	
	if (flags & UART_CONFIG_TYPE)
	{	
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

static struct uart_ops serialxr78x_pops = {
	.tx_empty	= serialxr78x_tx_empty,
	.set_mctrl	= serialxr78x_set_mctrl,
	.get_mctrl	= serialxr78x_get_mctrl,
	.stop_tx	= serialxr78x_stop_tx,
	.start_tx	= serialxr78x_start_tx,
	.stop_rx	= serialxr78x_stop_rx,
	.enable_ms	= serialxr78x_enable_ms,
	.break_ctl	= serialxr78x_break_ctl,
	.startup	= serialxr78x_startup,
	.shutdown	= serialxr78x_shutdown,
	.set_termios	= serialxr78x_set_termios,
	.pm		= serialxr78x_pm,
	.type		= serialxr78x_type,
	.release_port	= serialxr78x_release_port,
	.request_port	= serialxr78x_request_port,
	.config_port	= serialxr78x_config_port,
	.ioctl		= serialxr78x_ioctl,
};

static struct uart_xr_port serialxr78x_ports[XR788_COUNT][UART_XR788_NR];

#define XR78x_BASE_IO 		0x500
#define XR78x_IRQ 		0x5
#define XR78x_UART_OFFSET 	0x10

static unsigned int ports_inited[XR788_COUNT] = {0, 0, 0};

static void __init serial78x_init_ports(struct platform_device *dev, unsigned int dev_num)
{
	struct uart_xr_port *up;
	unsigned int device_base;
	unsigned int irq;
	int i;

	if (ports_inited[dev_num] == 1)
	{
		printk("Chip %d inited!\n", dev_num);
		return;
	}
	if(dev_num == 0)
		device_base = S3C2410_CS3;
	else if(dev_num == 1)
		device_base =S3C2410_CS4;
	else
		device_base = S3C2410_CS5;
	
	irq = platform_get_irq(dev, 0);

	for (i = 0, up = serialxr78x_ports[dev_num]; i < UART_XR788_NR;
	     i++, up++) {
//		up->port.iobase   = device_base + (i*XR78x_UART_OFFSET);
		up->port.mapbase   = device_base + (i*XR78x_UART_OFFSET);
		up->port.irq      = irq;
		up->port.uartclk  = 921600 * 16;
//		up->port.flags    = ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST | UPF_RESOURCES; // STD_COM_FLAGS=ASYNC_BOOT_AUTOCONF|ASYNC_SKIP_TEST
		up->port.flags    = ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST; 
		up->port.hub6     = 0;
//		up->port.membase  = 0;
		up->port.membase  = ioremap(up->port.mapbase, 0x10);
	//	up->port.iotype   = SERIAL_IO_PORT;
		up->port.iotype   = SERIAL_IO_MEM;
		up->port.regshift = 0;
        up->mcr_mask      = 0x13;
		up->port.ops      = &serialxr78x_pops;
		if (share_irqs)
			up->port.flags |= UPF_SHARE_IRQ;
	}
	ports_inited[dev_num] = 1;
}

static void __init serialxr78x_register_ports(struct platform_device *dev, struct uart_driver *drv, unsigned int dev_num)
{
	int i;
	
	serial78x_init_ports(dev, dev_num);
	
	for (i = 0; i < UART_XR788_NR; i++) {
		struct uart_xr_port *up = &serialxr78x_ports[dev_num][i];

		up->port.line = i;
		up->port.ops = &serialxr78x_pops;
		init_timer(&up->timer);
		up->timer.function = serialxr78x_timeout;
		up->port.dev = &(dev->dev);

		uart_add_one_port(drv, &up->port);
	}
}

#define SERIALXR_CONSOLE	NULL
#if 1
static struct uart_driver serialxr78x_reg[] = {
	[0] = {
		.owner			= THIS_MODULE,
		.driver_name		= "xrserialA",
		.dev_name		= "ttyExA",
//		.dev_name		= "ttyS",
		.major			= XR_788_MAJOR,
		.minor			= XR_788_MINOR,
		.nr			= UART_XR788_NR,
		.cons			= SERIALXR_CONSOLE,
	},
	[1] = {
		.owner			= THIS_MODULE,
		.driver_name		= "xrserialB",
		.dev_name		= "ttyExB",
//		.dev_name		= "ttyS",
		.major			= XR_788_MAJOR,
		.minor			= XR_788_MINOR +  UART_XR788_NR,
		.nr			= UART_XR788_NR,
		.cons			= SERIALXR_CONSOLE,
	},
	[2] = {
		.owner			= THIS_MODULE,
		.driver_name		= "xrserialC",
		.dev_name		= "ttyExC",
//		.dev_name		= "ttyS",
		.major			= XR_788_MAJOR,
		.minor			= XR_788_MINOR + 2 * UART_XR788_NR,
		.nr			= UART_XR788_NR,
		.cons			= SERIALXR_CONSOLE,
	},
};
#endif
#if 0
static struct uart_driver serialxr78x_reg = {
		.owner			= THIS_MODULE,
		.driver_name		= "xrserial",
		.dev_name		= "ttyEx",
		.major			= XR_788_MAJOR,
		.minor			= XR_788_MINOR,
		.nr			= UART_XR788_NR * XR788_COUNT,
		.cons			= SERIALXR_CONSOLE,
};
#endif
	

//static unsigned int drv_reged = 0;

static int __devinit serialxr78x_probe(struct platform_device *dev)
{
	int ret;
	unsigned int devid;
	struct resource *res;
#if 0
	unsigned int irq;
#endif

	res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if(res == NULL)
	{
		printk("Can't get resource!\n");	
		return -1;
	}
	switch(res->start)
	{
		case S3C2410_CS3:
			devid = 0;
			break;
		case S3C2410_CS4:
			devid = 1;
			break;
		case S3C2410_CS5:
			devid = 2;
			break;
        default:
            printk("incorrect res->start!\n");
            return -1;
	}

	ret = uart_register_driver(&serialxr78x_reg[devid]);
#if 0
	serialxr78x_reg[devid].state = kmem_cache_zalloc(xr788_drv_cachep, GFP_KERNEL);
	if(NULL == serialxr78x_reg[devid].state)
	{
		printk("Can't alloc cache for driver of chip %d\n", devid);
		return -ENOMEM;
	}
	serialxr78x_reg[devid].tty_driver = kmem_cache_zalloc(xr788_tty_cachep, GFP_KERNEL);
	if(NULL == serialxr78x_reg[devid].tty_driver)
	{
		printk("Can't alloc cache for tty driver of chip %d\n", devid);
		return -ENOMEM;
	}
	ret = serialxr78x_register_driver(&serialxr78x_reg[devid]);
#endif
	if(ret < 0)
	{
		printk("Fail to Register driver!\n");
		return ret;
	}

	serialxr78x_register_ports(dev, &serialxr78x_reg[devid], devid);
	return 0;
}
static int __devinit serialxr78x_remove(struct platform_device *dev)
{
	return 0;
}

static struct platform_driver serialxr78x_driver = {
	.probe = serialxr78x_probe,
	.remove = serialxr78x_remove,
	.driver = {
		.name = "xr16l788",
		.owner = THIS_MODULE,
	},
};


/*
 * register_serial and unregister_serial allows for 16x50 serial ports to be
 * configured at run-time, to support PCMCIA modems.
 */
#if 0
static int __register_serial(struct serial_struct *req, int line)
{
	struct uart_port port;

	port.iobase   = req->port;
	port.membase  = req->iomem_base;
	port.irq      = req->irq;
	port.uartclk  = req->baud_base * 16;
	port.fifosize = req->xmit_fifo_size;
	port.regshift = req->iomem_reg_shift;
	port.iotype   = req->io_type;
	port.flags    = req->flags | UPF_BOOT_AUTOCONF;
	port.mapbase  = req->iomap_base;
	port.line     = line;
	
	if (share_irqs)
		port.flags |= UPF_SHARE_IRQ;

	/*
	 * to be safer, check and default to the standard clock rate.
	 */
	if (port.uartclk == 0)
		port.uartclk = 921600 * 16; // XR17x15y clock rate

	return uart_register_port(&serialxr78x_reg, &port);
}

/**
 *	register_serial - configure a xr17x15y serial port at runtime
 *	@req: request structure
 *
 *	Configure the serial port specified by the request. If the
 *	port exists and is in use an error is returned. If the port
 *	is not currently in the table it is added.
 *
 *	The port is then probed and if necessary the IRQ is autodetected
 *	If this fails an error is returned.
 *
 *	On success the port is ready to use and the line number is returned.
 */
int register_serial(struct serial_struct *req)
{
	return __register_serial(req, -1);
}

/**
 *	unregister_serial - remove a xr17x15y serial port at runtime
 *	@line: serial line number
 *
 *	Remove one serial port.  This may be called from interrupt
 *	context.
 */
void unregister_serial(int line)
{
	uart_unregister_port(&serialxr78x_reg, line);
}
#endif

static int __init serialxr78x_init(void)
{
	int ret, i;

	printk(KERN_INFO "Exar XR16L78x specific serial driver $Revision: 1.0 $ "
		"%d ports, IRQ sharing %sabled\n", (int) UART_XR788_NR,
		share_irqs ? "en" : "dis");

	for (i = 0; i < NR_IRQS; i++)
		spin_lock_init(&irq_lists[i].lock);
#if 0
	xr788_drv_cachep = kmem_cache_create("16L788", sizeof(struct uart_state) * UART_XR788_NR, 0, SLAB_HWCACHE_ALIGN, NULL);
	if(xr788_drv_cachep == NULL)
	{
		printk("Can't alloc slab cache!\n");
		return -ENOMEM;
	}	
	xr788_tty_cachep = kmem_cache_create("16L788TTY", sizeof(struct tty_driver), 0, SLAB_HWCACHE_ALIGN, NULL);
	if(xr788_tty_cachep == NULL)
	{
		printk("Can't alloc slab cache!\n");
		return -ENOMEM;
	}	
#endif

//	ret = uart_register_driver(&serialxr78x_reg);
	ret = platform_driver_register(&serialxr78x_driver);
	
	if (ret >= 0)
	//	serialxr78x_register_ports(&serialxr78x_reg);
		printk("Driver Register Successfully!\n");
		
	return ret;
}

static void __exit serialxr78x_exit(void)
{
	int i;
	int j;
			
//	for (i = 0; i < UART_XR788_NR; i++)
	for (i = 0; i < XR788_COUNT; i++)
	{
		for (j = 0; j < UART_XR788_NR; j++)
			uart_remove_one_port(&serialxr78x_reg[i], &serialxr78x_ports[i][j].port);
//		kmem_cache_free(xr788_drv_cachep, serialxr78x_reg[i].state);
//		kmem_cache_free(xr788_tty_cachep, serialxr78x_reg[i].tty_driver);
//		serialxr78x_unregister_driver(&serialxr78x_reg[i]);
		uart_unregister_driver(&serialxr78x_reg[i]);
	}	
//	uart_unregister_driver(&serialxr78x_reg);
//	kmem_cache_destroy(xr788_drv_cachep);
//	kmem_cache_destroy(xr788_tty_cachep);
}

module_init(serialxr78x_init);
module_exit(serialxr78x_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Exar XR16L78x serial driver: for 3 chips on FFC3  $");
