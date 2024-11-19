#include "ftdi.h"

static const char *ftdi_chip_name[] = {
	[SIO]		= "SIO",	/* the serial part of FT8U100AX */
	[FT232A]	= "FT232A",
	[FT232B]	= "FT232B",
	[FT2232C]	= "FT2232C/D",
	[FT232R]	= "FT232R",
	[FT232H]	= "FT232H",
	[FT2232H]	= "FT2232H",
	[FT4232H]	= "FT4232H",
	[FT4232HA]	= "FT4232HA",
	[FT232HP]	= "FT232HP",
	[FT233HP]	= "FT233HP",
	[FT2232HP]	= "FT2232HP",
	[FT2233HP]	= "FT2233HP",
	[FT4232HP]	= "FT4232HP",
	[FT4233HP]	= "FT4233HP",
	[FTX]		= "FT-X",
};


/* Used for TIOCMIWAIT */
#define FTDI_STATUS_B0_MASK	(FTDI_RS0_CTS | FTDI_RS0_DSR | FTDI_RS0_RI | FTDI_RS0_RLSD)
#define FTDI_STATUS_B1_MASK	(FTDI_RS_BI)
/* End TIOCMIWAIT */

#define WDR_TIMEOUT 5000 /* default urb timeout */
#define WDR_SHORT_TIMEOUT 1000	/* shorter urb timeout */

#define set_mctrl(port, set)		update_mctrl((port), (set), 0)
#define clear_mctrl(port, clear)	update_mctrl((port), 0, (clear))

static int update_mctrl(struct usb_serial_port *port, unsigned int set,
							unsigned int clear)
{
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	struct device *dev = &port->dev;
	unsigned value;
	int rv;

	if (((set | clear) & (TIOCM_DTR | TIOCM_RTS)) == 0) {
		dev_dbg(dev, "%s - DTR|RTS not being set|cleared\n", __func__);
		return 0;	/* no change */
	}

	clear &= ~set;	/* 'set' takes precedence over 'clear' */
	value = 0;
	if (clear & TIOCM_DTR)
		value |= FTDI_SIO_SET_DTR_LOW;
	if (clear & TIOCM_RTS)
		value |= FTDI_SIO_SET_RTS_LOW;
	if (set & TIOCM_DTR)
		value |= FTDI_SIO_SET_DTR_HIGH;
	if (set & TIOCM_RTS)
		value |= FTDI_SIO_SET_RTS_HIGH;
	rv = usb_control_msg(port->serial->dev,
			       usb_sndctrlpipe(port->serial->dev, 0),
			       FTDI_SIO_SET_MODEM_CTRL_REQUEST,
			       FTDI_SIO_SET_MODEM_CTRL_REQUEST_TYPE,
			       value, priv->channel,
			       NULL, 0, WDR_TIMEOUT);
	if (rv < 0) {
		dev_dbg(dev, "%s Error from MODEM_CTRL urb: DTR %s, RTS %s\n",
			__func__,
			(set & TIOCM_DTR) ? "HIGH" : (clear & TIOCM_DTR) ? "LOW" : "unchanged",
			(set & TIOCM_RTS) ? "HIGH" : (clear & TIOCM_RTS) ? "LOW" : "unchanged");
		rv = usb_translate_errors(rv);
	} else {
		dev_dbg(dev, "%s - DTR %s, RTS %s\n", __func__,
			(set & TIOCM_DTR) ? "HIGH" : (clear & TIOCM_DTR) ? "LOW" : "unchanged",
			(set & TIOCM_RTS) ? "HIGH" : (clear & TIOCM_RTS) ? "LOW" : "unchanged");
		/* FIXME: locking on last_dtr_rts */
		priv->last_dtr_rts = (priv->last_dtr_rts & ~clear) | set;
	}
	return rv;
}

static int change_speed(struct tty_struct *tty, struct usb_serial *serial)
{
	struct ftdi_private *priv = usb_get_serial_port_data(serial);
	uint16_t value;
	uint16_t index;
	uint32_t index_value;

	value = (uint16_t)index_value;
	index = (uint16_t)(index_value >> 16);
	if (priv->channel)
		index = (uint16_t)((index << 8) | priv->channel);

	int rv = usb_control_msg(serial->dev,
			    usb_sndctrlpipe(serial->dev, 0),
			    FTDI_SIO_SET_BAUDRATE_REQUEST,
			    FTDI_SIO_SET_BAUDRATE_REQUEST_TYPE,
			    value, index,
			    NULL, 0, WDR_SHORT_TIMEOUT);
	return rv;
}

static int write_latency_timer(struct usb_serial_port *port)
{
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	struct usb_device *udev = port->serial->dev;
	int rv;
	int l = priv->latency;

	if (priv->chip_type == SIO || priv->chip_type == FT232A)
		return -EINVAL;

	if (priv->flags & ASYNC_LOW_LATENCY)
		l = 1;

	dev_dbg(&port->dev, "%s: setting latency timer = %i\n", __func__, l);

	rv = usb_control_msg(udev,
			     usb_sndctrlpipe(udev, 0),
			     FTDI_SIO_SET_LATENCY_TIMER_REQUEST,
			     FTDI_SIO_SET_LATENCY_TIMER_REQUEST_TYPE,
			     l, priv->channel,
			     NULL, 0, WDR_TIMEOUT);
	if (rv < 0)
		dev_err(&port->dev, "Unable to write latency timer: %i\n", rv);
	return rv;
}

static int _read_latency_timer(struct usb_serial_port *port)
{
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	struct usb_device *udev = port->serial->dev;
	u8 buf;
	int rv;

	rv = usb_control_msg_recv(udev, 0, FTDI_SIO_GET_LATENCY_TIMER_REQUEST,
				  FTDI_SIO_GET_LATENCY_TIMER_REQUEST_TYPE, 0,
				  priv->channel, &buf, 1, WDR_TIMEOUT,
				  GFP_KERNEL);
	if (rv == 0)
		rv = buf;

	return rv;
}

static int read_latency_timer(struct usb_serial_port *port)
{
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	int rv;

	if (priv->chip_type == SIO || priv->chip_type == FT232A)
		return -EINVAL;

	rv = _read_latency_timer(port);
	if (rv < 0) {
		dev_err(&port->dev, "Unable to read latency timer: %i\n", rv);
		return rv;
	}

	priv->latency = rv;

	return 0;
}

static void get_serial_info(struct tty_struct *tty, struct serial_struct *ss)
{
	struct usb_serial_port *port = tty->driver_data;
	struct ftdi_private *priv = usb_get_serial_port_data(port);

	ss->flags = priv->flags;
	ss->baud_base = priv->baud_base;                                                                                                                                     
	ss->custom_divisor = priv->custom_divisor;
}

static int set_serial_info(struct tty_struct *tty, struct serial_struct *ss)
{
	struct usb_serial_port *port = tty->driver_data;
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	int old_flags, old_divisor;

	if (!capable(CAP_SYS_ADMIN)) {
		if ((ss->flags ^ priv->flags) & ~ASYNC_USR_MASK) {
			return -EPERM;
		}
	}

	old_flags = priv->flags;
	old_divisor = priv->custom_divisor;

	priv->flags = ss->flags & ASYNC_FLAGS;
	priv->custom_divisor = ss->custom_divisor;

	write_latency_timer(port);

	if ((priv->flags ^ old_flags) & ASYNC_SPD_MASK ||
			((priv->flags & ASYNC_SPD_MASK) == ASYNC_SPD_CUST &&
			 priv->custom_divisor != old_divisor)) {

		/* warn about deprecation unless clearing */
		if (priv->flags & ASYNC_SPD_MASK)
			dev_warn_ratelimited(&port->dev, "use of SPD flags is deprecated\n");

		change_speed(tty, port);
	}
	return 0;
}

static int get_lsr_info(struct usb_serial_port *port,
			unsigned int __user *retinfo)
{
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	unsigned int result = 0;

	if (priv->transmit_empty)
		result = TIOCSER_TEMT;

	if (copy_to_user(retinfo, &result, sizeof(unsigned int)))
		return -EFAULT;
	return 0;
}

static int ftdi_determine_type(struct usb_serial *serial)
{
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	struct usb_device *udev = serial->dev;
	unsigned int version, ifnum;

	ifnum = serial->interfa                                                                                     
	priv->channel = CHANNEL_A + ifnum;

	switch (version) {
	case 0x200:
		priv->chip_type = FT232A;
		priv->baud_base = 48000000 / 2;
		priv->channel = 0;
		/*
		 * FT232B devices have a bug where bcdDevice gets set to 0x200
		 * when iSerialNumber is 0. Assume it is an FT232B in case the
		 * latency timer is readable.
		 */
		if (udev->descriptor.iSerialNumber == 0 &&
				_read_latency_timer(port) >= 0) {
			priv->chip_type = FT232B;
		}
		break;
	case 0x400:
		priv->chip_type = FT232B;
		priv->baud_base = 48000000 / 2;
		priv->channel = 0;
		break;
	case 0x500:
		priv->chip_type = FT2232C;
		priv->baud_base = 48000000 / 2;
		break;
	case 0x600:
		priv->chip_type = FT232R;
		priv->baud_base = 48000000 / 2;
		priv->channel = 0;
		break;
	case 0x700:
		priv->chip_type = FT2232H;
		break;
	case 0x800:
		priv->chip_type = FT4232H;
		break;
	case 0x900:
		priv->chip_type = FT232H;
		break;
	case 0x1000:
		priv->chip_type = FTX;
		priv->baud_base = 48000000 / 2;
		break;
	case 0x2800:
		priv->chip_type = FT2233HP;
		break;
	case 0x2900:
		priv->chip_type = FT4233HP;
		break;
	case 0x3000:
		priv->chip_type = FT2232HP;
		break;
	case 0x3100:
		priv->chip_type = FT4232HP;
		break;
	case 0x3200:
		priv->chip_type = FT233HP;
		break;
	case 0x3300:
		priv->chip_type = FT232HP;
		break;
	case 0x3600:
		priv->chip_type = FT4232HA;
		break;
	default:
		if (version < 0x200) {
			priv->chip_type = SIO;
			priv->baud_base = 12000000 / 16;
			priv->channel = 0;
		} else {
			dev_err(&port->dev, "unknown device type: 0x%02x\n", version);
			return -ENODEV;
		}
	}

	dev_info(&udev->dev, "Detected %s\n", ftdi_chip_name[priv->chip_type]);

	return 0;
}


/*
 * Determine the maximum packet size for the device. This depends on the chip
 * type and the USB host capabilities. The value should be obtained from the
 * device descriptor as the chip will use the appropriate values for the host.
 */
static void ftdi_set_max_packet_size(struct usb_serial *serial)
{
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	struct usb_interface *interface = port->serial->interface;
	struct usb_endpoint_descriptor *ep_desc;
	unsigned num_endpoints;
	unsigned i;

	num_endpoints = interface->cur_altsetting->desc.bNumEndpoints;
	if (!num_endpoints)
		return;

	/*
	 * NOTE: Some customers have programmed FT232R/FT245R devices
	 * with an endpoint size of 0 - not good. In this case, we
	 * want to override the endpoint descriptor setting and use a
	 * value of 64 for wMaxPacketSize.
	 */
	for (i = 0; i < num_endpoints; i++) {
		ep_desc = &interface->cur_altsetting->endpoint[i].desc;
		if (!ep_desc->wMaxPacketSize) {
			ep_desc->wMaxPacketSize = cpu_to_le16(0x40);
			dev_warn(&port->dev, "Overriding wMaxPacketSize on endpoint %d\n",
					usb_endpoint_num(ep_desc));
		}
	}

	/* Set max packet size based on last descriptor. */
	priv->max_packet_size = usb_endpoint_maxp(ep_desc);
}


/*
 * ***************************************************************************
 * Sysfs Attribute
 * ***************************************************************************
 */

static ssize_t latency_timer_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct usb_serial_port *port = to_usb_serial_port(dev);
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	if (priv->flags & ASYNC_LOW_LATENCY)
		return sprintf(buf, "1\n");
	else
		return sprintf(buf, "%u\n", priv->latency);
}

/* Write a new value of the latency timer, in units of milliseconds. */
static ssize_t latency_timer_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *valbuf, size_t count)
{
	struct usb_serial_port *port = to_usb_serial_port(dev);
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	u8 v;
	int rv;

	if (kstrtou8(valbuf, 10, &v))
		return -EINVAL;

	priv->latency = v;
	rv = write_latency_timer(port);
	if (rv < 0)
		return -EIO;
	return count;
}
static DEVICE_ATTR_RW(latency_timer);

/* Write an event character directly to the FTDI register.  The ASCII
   value is in the low 8 bits, with the enable bit in the 9th bit. */
static ssize_t event_char_store(struct device *dev,
	struct device_attribute *attr, const char *valbuf, size_t count)
{
	struct usb_serial_port *port = to_usb_serial_port(dev);
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	struct usb_device *udev = port->serial->dev;
	unsigned int v;
	int rv;

	if (kstrtouint(valbuf, 0, &v) || v >= 0x200)
		return -EINVAL;

	dev_dbg(&port->dev, "%s: setting event char = 0x%03x\n", __func__, v);

	rv = usb_control_msg(udev,
			     usb_sndctrlpipe(udev, 0),
			     FTDI_SIO_SET_EVENT_CHAR_REQUEST,
			     FTDI_SIO_SET_EVENT_CHAR_REQUEST_TYPE,
			     v, priv->channel,
			     NULL, 0, WDR_TIMEOUT);
	if (rv < 0) {
		dev_dbg(&port->dev, "Unable to write event character: %i\n", rv);
		return -EIO;
	}

	return count;
}
static DEVICE_ATTR_WO(event_char);

/*
 * ***************************************************************************
 * FTDI driver specific functions
 * ***************************************************************************
 */

static int ftdi_probe(struct usb_serial *serial, const struct usb_device_id *id)
{
	const struct ftdi_quirk *quirk = (struct ftdi_quirk *)id->driver_info;

	if (quirk && quirk->probe) {
		int ret = quirk->probe(serial);
		if (ret != 0)
			return ret;
	}

	usb_set_serial_data(serial, (void *)id->driver_info);

	return 0;
}

static int ftdi_port_probe(struct usb_serial_port *port)
{
	const struct ftdi_quirk *quirk = usb_get_serial_data(port->serial);
	struct ftdi_private *priv;
	int result;

	priv = kzalloc(sizeof(struct ftdi_private), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mutex_init(&priv->cfg_lock);

	if (quirk && quirk->port_probe)
		quirk->port_probe(priv);

	usb_set_serial_port_data(port, priv);

	result = ftdi_determine_type(port);
	if (result)
		goto err_free;

	ftdi_set_max_packet_size(port);
	if (read_latency_timer(port) < 0)
		priv->latency = 16;
	write_latency_timer(port);

	result = ftdi_gpio_init(port);
	if (result < 0) {
		dev_err(&port->serial->interface->dev,
			"GPIO initialisation failed: %d\n",
			result);
	}

	return 0;

err_free:
	kfree(priv);

	return result;
}






static void ftdi_port_remove(struct usb_serial_port *port)
{
	struct ftdi_private *priv = usb_get_serial_port_data(port);

	ftdi_gpio_remove(port);

	kfree(priv);
}

static int ftdi_open(struct tty_struct *tty, struct usb_serial_port *port)
{
	struct usb_device *dev = port->serial->dev;
	struct ftdi_private *priv = usb_get_serial_port_data(port);

	/* No error checking for this (will get errors later anyway) */
	/* See ftdi_sio.h for description of what is reset */
	usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
			FTDI_SIO_RESET_REQUEST, FTDI_SIO_RESET_REQUEST_TYPE,
			FTDI_SIO_RESET_SIO,
			priv->channel, NULL, 0, WDR_TIMEOUT);

	/* Termios defaults are set by usb_serial_init. We don't change
	   port->tty->termios - this would lose speed settings, etc.
	   This is same behaviour as serial.c/rs_open() - Kuba */

	/* ftdi_set_termios  will send usb control messages */
	if (tty)
		ftdi_set_termios(tty, port, NULL);

	return usb_serial_generic_open(tty, port);
}

static void ftdi_dtr_rts(struct usb_serial_port *port, int on)
{
	struct ftdi_private *priv = usb_get_serial_port_data(port);

	/* Disable flow control */
	if (!on) {
		if (usb_control_msg(port->serial->dev,
			    usb_sndctrlpipe(port->serial->dev, 0),
			    FTDI_SIO_SET_FLOW_CTRL_REQUEST,
			    FTDI_SIO_SET_FLOW_CTRL_REQUEST_TYPE,
			    0, priv->channel, NULL, 0,
			    WDR_TIMEOUT) < 0) {
			dev_err(&port->dev, "error from flowcontrol urb\n");
		}
	}
	/* drop RTS and DTR */
	if (on)
		set_mctrl(port, TIOCM_DTR | TIOCM_RTS);
	else
		clear_mctrl(port, TIOCM_DTR | TIOCM_RTS);
}

/* The SIO requires the first byte to have:
 *  B0 1
 *  B1 0
 *  B2..7 length of message excluding byte 0
 *
 * The new devices do not require this byte
 */
static int ftdi_prepare_write_buffer(struct usb_serial_port *port,
						void *dest, size_t size)
{
	struct ftdi_private *priv;
	int count;
	unsigned long flags;

	priv = usb_get_serial_port_data(port);

	if (priv->chip_type == SIO) {
		unsigned char *buffer = dest;
		int i, len, c;

		count = 0;
		spin_lock_irqsave(&port->lock, flags);
		for (i = 0; i < size - 1; i += priv->max_packet_size) {
			len = min_t(int, size - i, priv->max_packet_size) - 1;
			c = kfifo_out(&port->write_fifo, &buffer[i + 1], len);
			if (!c)
				break;
			port->icount.tx += c;
			buffer[i] = (c << 2) + 1;
			count += c + 1;
		}
		spin_unlock_irqrestore(&port->lock, flags);
	} else {
		count = kfifo_out_locked(&port->write_fifo, dest, size,
								&port->lock);
		port->icount.tx += count;
	}

	return count;
}

#define FTDI_RS_ERR_MASK (FTDI_RS_BI | FTDI_RS_PE | FTDI_RS_FE | FTDI_RS_OE)

static int ftdi_process_packet(struct usb_serial_port *port,
		struct ftdi_private *priv, unsigned char *buf, int len)
{
	unsigned char status;
	bool brkint = false;
	int i;
	char flag;

	if (len < 2) {
		dev_dbg(&port->dev, "malformed packet\n");
		return 0;
	}

	/* Compare new line status to the old one, signal if different/
	   N.B. packet may be processed more than once, but differences
	   are only processed once.  */
	status = buf[0] & FTDI_STATUS_B0_MASK;
	if (status != priv->prev_status) {
		char diff_status = status ^ priv->prev_status;

		if (diff_status & FTDI_RS0_CTS)
			port->icount.cts++;
		if (diff_status & FTDI_RS0_DSR)
			port->icount.dsr++;
		if (diff_status & FTDI_RS0_RI)
			port->icount.rng++;
		if (diff_status & FTDI_RS0_RLSD) {
			struct tty_struct *tty;

			port->icount.dcd++;
			tty = tty_port_tty_get(&port->port);
			if (tty)
				usb_serial_handle_dcd_change(port, tty,
						status & FTDI_RS0_RLSD);
			tty_kref_put(tty);
		}

		wake_up_interruptible(&port->port.delta_msr_wait);
		priv->prev_status = status;
	}

	/* save if the transmitter is empty or not */
	if (buf[1] & FTDI_RS_TEMT)
		priv->transmit_empty = 1;
	else
		priv->transmit_empty = 0;

	if (len == 2)
		return 0;	/* status only */

	/*
	 * Break and error status must only be processed for packets with
	 * data payload to avoid over-reporting.
	 */
	flag = TTY_NORMAL;
	if (buf[1] & FTDI_RS_ERR_MASK) {
		/*
		 * Break takes precedence over parity, which takes precedence
		 * over framing errors. Note that break is only associated
		 * with the last character in the buffer and only when it's a
		 * NUL.
		 */
		if (buf[1] & FTDI_RS_BI && buf[len - 1] == '\0') {
			port->icount.brk++;
			brkint = true;
		}
		if (buf[1] & FTDI_RS_PE) {
			flag = TTY_PARITY;
			port->icount.parity++;
		} else if (buf[1] & FTDI_RS_FE) {
			flag = TTY_FRAME;
			port->icount.frame++;
		}
		/* Overrun is special, not associated with a char */
		if (buf[1] & FTDI_RS_OE) {
			port->icount.overrun++;
			tty_insert_flip_char(&port->port, 0, TTY_OVERRUN);
		}
	}

	port->icount.rx += len - 2;

	if (brkint || port->sysrq) {
		for (i = 2; i < len; i++) {
			if (brkint && i == len - 1) {
				if (usb_serial_handle_break(port))
					return len - 3;
				flag = TTY_BREAK;
			}
			if (usb_serial_handle_sysrq_char(port, buf[i]))
				continue;
			tty_insert_flip_char(&port->port, buf[i], flag);
		}
	} else {
		tty_insert_flip_string_fixed_flag(&port->port, buf + 2, flag,
				len - 2);
	}

	return len - 2;
}

static void ftdi_process_read_urb(struct urb *urb)
{
	struct usb_serial_port *port = urb->context;
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	char *data = urb->transfer_buffer;
	int i;
	int len;
	int count = 0;

	for (i = 0; i < urb->actual_length; i += priv->max_packet_size) {
		len = min_t(int, urb->actual_length - i, priv->max_packet_size);
		count += ftdi_process_packet(port, priv, &data[i], len);
	}

	if (count)
		tty_flip_buffer_push(&port->port);
}

static int ftdi_break_ctl(struct tty_struct *tty, int break_state)
{
	struct usb_serial_port *port = tty->driver_data;
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	uint16_t value;
	int ret;

	/* break_state = -1 to turn on break, and 0 to turn off break */
	/* see drivers/char/tty_io.c to see it used */
	/* last_set_data_value NEVER has the break bit set in it */

	if (break_state)
		value = priv->last_set_data_value | FTDI_SIO_SET_BREAK;
	else
		value = priv->last_set_data_value;

	ret = usb_control_msg(port->serial->dev,
			usb_sndctrlpipe(port->serial->dev, 0),
			FTDI_SIO_SET_DATA_REQUEST,
			FTDI_SIO_SET_DATA_REQUEST_TYPE,
			value, priv->channel,
			NULL, 0, WDR_TIMEOUT);
	if (ret < 0) {
		dev_err(&port->dev, "%s FAILED to enable/disable break state (state was %d)\n",
			__func__, break_state);
		return ret;
	}

	dev_dbg(&port->dev, "%s break state is %d - urb is %d\n", __func__,
		break_state, value);

	return 0;
}

static bool ftdi_tx_empty(struct usb_serial_port *port)
{
	unsigned char buf[2];
	int ret;

	ret = ftdi_get_modem_status(port, buf);
	if (ret == 2) {
		if (!(buf[1] & FTDI_RS_TEMT))
			return false;
	}

	return true;
}

/* old_termios contains the original termios settings and tty->termios contains
 * the new setting to be used
 * WARNING: set_termios calls this with old_termios in kernel space
 */
static void ftdi_set_termios(struct tty_struct *tty,
		             struct usb_serial_port *port,
		             const struct ktermios *old_termios)
{
	struct usb_device *dev = port->serial->dev;
	struct device *ddev = &port->dev;
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	struct ktermios *termios = &tty->termios;
	unsigned int cflag;
	uint16_t value, index;
	int ret;

	/* Force baud rate if this device requires it, unless it is set to
	   B0. */
	if (priv->force_baud && ((termios->c_cflag & CBAUD) != B0)) {
		dev_dbg(ddev, "%s: forcing baud rate for this device\n", __func__);
		tty_encode_baud_rate(tty, priv->force_baud,
					priv->force_baud);
	}

	/* Force RTS-CTS if this device requires it. */
	if (priv->force_rtscts) {
		dev_dbg(ddev, "%s: forcing rtscts for this device\n", __func__);
		termios->c_cflag |= CRTSCTS;
	}

	/*
	 * All FTDI UART chips are limited to CS7/8. We shouldn't pretend to
	 * support CS5/6 and revert the CSIZE setting instead.
	 *
	 * CS5 however is used to control some smartcard readers which abuse
	 * this limitation to switch modes. Original FTDI chips fall back to
	 * eight data bits.
	 *
	 * TODO: Implement a quirk to only allow this with mentioned
	 *       readers. One I know of (Argolis Smartreader V1)
	 *       returns "USB smartcard server" as iInterface string.
	 *       The vendor didn't bother with a custom VID/PID of
	 *       course.
	 */
	if (C_CSIZE(tty) == CS6) {
		dev_warn(ddev, "requested CSIZE setting not supported\n");

		termios->c_cflag &= ~CSIZE;
		if (old_termios)
			termios->c_cflag |= old_termios->c_cflag & CSIZE;
		else
			termios->c_cflag |= CS8;
	}

	cflag = termios->c_cflag;

	if (!old_termios)
		goto no_skip;

	if (old_termios->c_cflag == termios->c_cflag
	    && old_termios->c_ispeed == termios->c_ispeed
	    && old_termios->c_ospeed == termios->c_ospeed)
		goto no_c_cflag_changes;

	/* NOTE These routines can get interrupted by
	   ftdi_sio_read_bulk_callback  - need to examine what this means -
	   don't see any problems yet */

	if ((old_termios->c_cflag & (CSIZE|PARODD|PARENB|CMSPAR|CSTOPB)) ==
	    (termios->c_cflag & (CSIZE|PARODD|PARENB|CMSPAR|CSTOPB)))
		goto no_data_parity_stop_changes;

no_skip:
	/* Set number of data bits, parity, stop bits */

	value = 0;
	value |= (cflag & CSTOPB ? FTDI_SIO_SET_DATA_STOP_BITS_2 :
			FTDI_SIO_SET_DATA_STOP_BITS_1);
	if (cflag & PARENB) {
		if (cflag & CMSPAR)
			value |= cflag & PARODD ?
					FTDI_SIO_SET_DATA_PARITY_MARK :
					FTDI_SIO_SET_DATA_PARITY_SPACE;
		else
			value |= cflag & PARODD ?
					FTDI_SIO_SET_DATA_PARITY_ODD :
					FTDI_SIO_SET_DATA_PARITY_EVEN;
	} else {
		value |= FTDI_SIO_SET_DATA_PARITY_NONE;
	}
	switch (cflag & CSIZE) {
	case CS5:
		dev_dbg(ddev, "Setting CS5 quirk\n");
		break;
	case CS7:
		value |= 7;
		dev_dbg(ddev, "Setting CS7\n");
		break;
	default:
	case CS8:
		value |= 8;
		dev_dbg(ddev, "Setting CS8\n");
		break;
	}

	/* This is needed by the break command since it uses the same command
	   - but is or'ed with this value  */
	priv->last_set_data_value = value;

	if (usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
			    FTDI_SIO_SET_DATA_REQUEST,
			    FTDI_SIO_SET_DATA_REQUEST_TYPE,
			    value, priv->channel,
			    NULL, 0, WDR_SHORT_TIMEOUT) < 0) {
		dev_err(ddev, "%s FAILED to set databits/stopbits/parity\n",
			__func__);
	}

	/* Now do the baudrate */
no_data_parity_stop_changes:
	if ((cflag & CBAUD) == B0) {
		/* Disable flow control */
		if (usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
				    FTDI_SIO_SET_FLOW_CTRL_REQUEST,
				    FTDI_SIO_SET_FLOW_CTRL_REQUEST_TYPE,
				    0, priv->channel,
				    NULL, 0, WDR_TIMEOUT) < 0) {
			dev_err(ddev, "%s error from disable flowcontrol urb\n",
				__func__);
		}
		/* Drop RTS and DTR */
		clear_mctrl(port, TIOCM_DTR | TIOCM_RTS);
	} else {
		/* set the baudrate determined before */
		if (change_speed(tty, port))
			dev_err(ddev, "%s urb failed to set baudrate\n", __func__);
		/* Ensure RTS and DTR are raised when baudrate changed from 0 */
		if (old_termios && (old_termios->c_cflag & CBAUD) == B0)
			set_mctrl(port, TIOCM_DTR | TIOCM_RTS);
	}

no_c_cflag_changes:
	/* Set hardware-assisted flow control */
	value = 0;

	if (C_CRTSCTS(tty)) {
		dev_dbg(&port->dev, "enabling rts/cts flow control\n");
		index = FTDI_SIO_RTS_CTS_HS;
	} else if (I_IXON(tty)) {
		dev_dbg(&port->dev, "enabling xon/xoff flow control\n");
		index = FTDI_SIO_XON_XOFF_HS;
		value = STOP_CHAR(tty) << 8 | START_CHAR(tty);
	} else {
		dev_dbg(&port->dev, "disabling flow control\n");
		index = FTDI_SIO_DISABLE_FLOW_CTRL;
	}

	index |= priv->channel;

	ret = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
			FTDI_SIO_SET_FLOW_CTRL_REQUEST,
			FTDI_SIO_SET_FLOW_CTRL_REQUEST_TYPE,
			value, index, NULL, 0, WDR_TIMEOUT);
	if (ret < 0)
		dev_err(&port->dev, "failed to set flow control: %d\n", ret);
}

/*
 * Get modem-control status.
 *
 * Returns the number of status bytes retrieved (device dependant), or
 * negative error code.
 */
static int ftdi_get_modem_status(struct usb_serial_port *port,
						unsigned char status[2])
{
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	unsigned char *buf;
	int len;
	int ret;

	buf = kmalloc(2, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	/*
	 * The device returns a two byte value (the SIO a 1 byte value) in the
	 * same format as the data returned from the IN endpoint.
	 */
	if (priv->chip_type == SIO)
		len = 1;
	else
		len = 2;

	ret = usb_control_msg(port->serial->dev,
			usb_rcvctrlpipe(port->serial->dev, 0),
			FTDI_SIO_GET_MODEM_STATUS_REQUEST,
			FTDI_SIO_GET_MODEM_STATUS_REQUEST_TYPE,
			0, priv->channel,
			buf, len, WDR_TIMEOUT);

	/* NOTE: We allow short responses and handle that below. */
	if (ret < 1) {
		dev_err(&port->dev, "failed to get modem status: %d\n", ret);
		if (ret >= 0)
			ret = -EIO;
		ret = usb_translate_errors(ret);
		goto out;
	}

	status[0] = buf[0];
	if (ret > 1)
		status[1] = buf[1];
	else
		status[1] = 0;

	dev_dbg(&port->dev, "%s - 0x%02x%02x\n", __func__, status[0],
								status[1]);
out:
	kfree(buf);

	return ret;
}

static int ftdi_tiocmget(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	unsigned char buf[2];
	int ret;

	ret = ftdi_get_modem_status(port, buf);
	if (ret < 0)
		return ret;

	ret =	(buf[0] & FTDI_SIO_DSR_MASK  ? TIOCM_DSR : 0) |
		(buf[0] & FTDI_SIO_CTS_MASK  ? TIOCM_CTS : 0) |
		(buf[0] & FTDI_SIO_RI_MASK   ? TIOCM_RI  : 0) |
		(buf[0] & FTDI_SIO_RLSD_MASK ? TIOCM_CD  : 0) |
		priv->last_dtr_rts;

	return ret;
}

static int ftdi_tiocmset(struct tty_struct *tty,
			unsigned int set, unsigned int clear)
{
	struct usb_serial_port *port = tty->driver_data;

	return update_mctrl(port, set, clear);
}

static struct usb_serial_driver ftdi_device = {
	.driver = {
		.name =		"ftdi_sio",
		.dev_groups =	ftdi_groups,
	},
	.description =		"FTDI USB Serial Device",
	.id_table =		id_table_combined,
	.num_ports =		1,
	.bulk_in_size =		512,
	.bulk_out_size =	256,
	.probe =		ftdi_probe,
	.port_probe =		ftdi_port_probe,
	.port_remove =		ftdi_port_remove,
	.open =			ftdi_open,
	.dtr_rts =		ftdi_dtr_rts,
	.process_read_urb =	ftdi_process_read_urb,
	.prepare_write_buffer =	ftdi_prepare_write_buffer,
	.tiocmget =		ftdi_tiocmget,
	.tiocmset =		ftdi_tiocmset,
	.get_serial =		get_serial_info,
	.set_serial =		set_serial_info,
	.set_termios =		ftdi_set_termios,
	.break_ctl =		ftdi_break_ctl,
	.tx_empty =		ftdi_tx_empty,
};