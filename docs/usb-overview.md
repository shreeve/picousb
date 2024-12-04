## USB Overview

USB is ubiquitous. It is one of the most successful computing protocols ever
developed, but its details are surprisingly complicated. The [USB 2.0
Specification](https://www.usb.org/document-library/usb-20-specification)
defines two speeds: LS (Low Speed) which operates at 1.5 Mbps (megabits per
second), and FS (Full Speed) which operates at 12 Mbps. Low Speed USB is
typically used for devices that require lower bandwidth, such as keyboards and
mice, while Full Speed USB is used for a wider range of devices, including audio
devices, and storage devices which offer higher data transfer rates.

### Bus Topology

USB operates by connecting one or more USB devices on a common bus, which is
connected to a USB host. A device called a USB hub can connect other USB devices
upstream. A maximum of 127 USB devices can be connected, all controlled by a
single USB host, and each device assigned a unique address between 1 and 127.
Sometimes a USB device is referred to as a "function", but we will only use the
term "device".

<div align="center"><img width="300" src="https://github.com/user-attachments/assets/7445813e-bd64-46cf-912d-6d86c0976f0d"></div>

### Main Components

In order to understand USB, we begin at the most basic level and work up from
there:

1) [Connectors](#connectors)
2) [Wires](#wires)
3) [Voltages](#voltages)
4) [Line states](#line-states)
5) [Bits and bytes](#bits-and-bytes)
6) [Packets](#packets)
7) [Packet Types](#packet-types)
8) [Transactions](#transactions)
9) [Transfers](#transfers)
10) [Endpoints](#endpoints)
11) [Devices](#devices)
12) [Hubs](#hubs)
13) [Hosts](#hosts)
14) [Enumeration](#enumeration)

### Connectors

The main USB connectors encountered are USB A, USB B, Micro USB, and USB C.

<div align="center"><img width="480" src="https://github.com/user-attachments/assets/b1b18fef-ecb2-4345-81de-aca54810f419"></div>

### Wires

There are four main wires used for USB. These include VCC (which is a red wire
at +5V), Ground (which is a black wire at +0V), Data Positive (called DP or D+
and is a green wire), and Data Minus (called DM or D- and is a white wire). The
following diagram is for a USB Type A connector, but each of the other connector
types have similar wiring, but just use different connector pins.

<div align="center"><img width="300" src="https://github.com/user-attachments/assets/1217edcb-30db-4df2-a77d-33a3ee414424"></div>

### Voltages

The USB host and USB devices connected to a USB bus communicate with each other
by altering the voltage of the DP and DM wires. USB uses a method called
differential signaling, which involves transmitting information as the
difference in voltages between the differential pair of wires (DP and DM).

To create a differential '1' signal, the DP line goes high while the DM line is
low. To create a differential '0' signal, the DP line is low while the DM line
goes high. If you think of the DP line on top and the DM line on bottom, a
differential '1' is the greatest difference in voltages and a differential '0'
is the smallest difference in voltages.

Differential signaling enhances noise immunity, as any interference picked up
along the cable is likely to affect both lines equally and can be effectively
canceled out at the receiver. Differential signaling allows for more reliable
data transmission over longer distances and at higher speeds compared to
single-ended signaling methods.

<div align="center"><img width="480" src="https://github.com/user-attachments/assets/a4e2b224-447a-4c5e-9239-948937707275"></div>

### Line States

USB uses various line states to communicate efficiently, manage power states,
and synchronize data transfers across the USB bus. These states include:

### USB Line States and Their Meanings

| Line State                 | Meaning                                                                                              |
|:---------------------------|:-----------------------------------------------------------------------------------------------------|
| **Differential '1'**       | D+ high, D- low.                                                                                     |
| **Differential '0'**       | D+ low, D- high.                                                                                     |
| **Single Ended Zero** (SE0)| D+ and D- low. Used to signal a reset condition or the End of Packet (EOP).                          |
| **Single Ended One** (SE1) | D+ and D- high. This state is invalid in standard USB communication.                                 |

---

### USB States by Speed

| USB Speed      | Idle State                         | Data J State                        | Data K State                           |
|:---------------|:-----------------------------------|:------------------------------------|:---------------------------------------|
| **Low Speed**  | Differential '0' (D+ low, D- high) | Same as idle (Differential '0').    | Differential '1' (opposite of J).      |
| **Full Speed** | Differential '1' (D+ high, D- low) | Same as idle (Differential '1').    | Differential '0' (opposite of J).      |

---

### USB Protocol States

| State                     | Description                                                                                                                   |
|:--------------------------|:------------------------------------------------------------------------------------------------------------------------------|
| **Connect**               | When a device connects and drives the line idle for at least 2.5 μs.                                                          |
| **Reset**                 | SE0 state held for at least 10 ms, signaling a reset condition.                                                               |
| **Start of Packet (SOP)** | Transition from idle to SE0 for two bit times, then idle (J state) for 1 bit time, followed by the 8-bit sequence "KJKJKJKK". |
| **End of Packet (EOP)**   | SE0 state for 2 bit times, followed by a J state for 1 bit time.                                                              |
| **Suspend State**         | Idle state entered after 3 ms or more of inactivity, designed to save power.                                                  |
| **Resume State**          | Wakes a device from suspend. Begins with a K state for at least 20 ms, followed by SE0 and then J state.                      |
| **Disconnect**            | SE0 state held for at least 2 μs.                                                                                             |

### Bits and Bytes

The line states are represented by a series of zero and one bits. However,
because there is no central clock in USB from which to synchronize all events,
the actual bit transitions themselves are used for synchronization. If a
particular sequence of bits were to be represented by a long series of either
zeroes or ones, it could be impossible to maintain accurate timing solely by
observing bit changes, given that small variations in the timing intervals would
soon yield timing and bit framing errors.

To resolve this, a special encoding called Non-Return-to-Zero Inverted (NRZI) is
used. NRZI works by inverting the signal level if a '1' bit is encountered and
by passing the signal as-is if a '0' bit is encountered. This means that
consecutive '1's will result in the signal level changing state at each bit,
while consecutive '0's will result in no change in the level. This form of
encoding is used because it can make it easier to synchronize the transmitter
and receiver without requiring an additional clock signal.

Long sequences of zeros, however, still poses a problem for clock recovery. To
overcome this, USB 2.0 combines NRZI with a technique called bit stuffing. Bit
stuffing works by inserting a '0' in the data stream after every six consecutive
'1's in the data stream. This ensures that there will not be more than six
consecutive '1's, meaning there will not be a gap of more than six bit times
without a transition. The receiver, knowing this rule, removes these stuffed '0'
bits to recover the original data stream. By combining NRZI with bit stuffing,
USB ensures that there are enough transitions to keep the transmitter and
receiver in sync without the need for a separate clock signal. This is a form of
self-clocking signal, which allows USB to be a relatively simple and
cost-effective interface.

### Packets

The smallest useful unit of data in USB is known as a packet, which has three
components:

<div align="center">
  <table border="1">
  <tr style="background:#5383EC; color:white">
    <th style="text-align:center">1. SOP</th>
    <th style="text-align:center">2. Body</th>
    <th style="text-align:center">3. EOP</th>
  </tr>
  </table>
</div>

1) Start of Packet (SOP) sequence
2) Body, which depends on the type of packet
3) End of Packet (EOP) sequence

Each packet begins with a Start of Packet (SOP) sequence. This means the line
state goes from idle to SE0 for two bit times, then to a J state (idle) for 1
bit time, and is then followed by the 8 bit sequence "KJKJKJKK" (known as the
SYNC). The bits comprising the body of a packet depend on the type of packet
being sent. The End of Packet (EOP) sequence indicates the packet has ended and
is now complete. The SOP and EOP components are always the same. Sandwiched
between them is the actual data that defines the packet type and its contents.

<div align="center"><img width="500" src="https://github.com/user-attachments/assets/08861c65-92ff-4e3d-9893-70a5b550f325"></div>

### Packet Types

There are four main packet types, with 10 specific packets defined. These are
identified by a 4 bit Packet ID (PID).

| PID Type | PID Name | PID Bits |
| --- | --- | --- |
| Token     | SETUP    | 1101 |
| Token     | IN       | 1001 |
| Token     | OUT      | 0001 |
| Token     | SOF      | 0101 |
| Data      | DATA0    | 0011 |
| Data      | DATA1    | 1011 |
| Handshake | ACK      | 0010 |
| Handshake | NAK      | 1010 |
| Handshake | STALL    | 1110 |
| Special   | PRE      | 1100 |

PID Type bits are sent least significant bit (LSB) first. For example, a SETUP
packet would be *sent* as `1011` even though it is *defined* as `1101`. As a
check when sending these four bits, the inverse of these four bits is
immediately sent after, which helps to confirm that the correct PID Type has
been received. To continue with the SETUP packet example, the complete 8 bit
sequence sent is `10110100` (notice the second four bits are the opposite of the
first four, thus confirming proper transmission). Review the image of the ACK
packet to ensure that you understand each bit in the entire packet.

1. **Token Packets** are used for SETUP, IN, and OUT packets. They are always
the first packet in a transaction, identifying the targeted endpoint, and the
purpose of the transaction. The SOF packet is also defined as a Token packet,
but has a slightly different format and purpose, which is described below.

    <table border="1">
      <tr style="background:#5383EC; color:white">
        <th colspan="4" style="text-align:center">Token Packet (3 bytes)</th>
      </tr>
      <tr>
        <td>PID<br>8 bits</td>
        <td>ADDR<br>7 bits</td>
        <td>ENDP<br>4 bits</td>
        <td>CRC5<br>5 bits</td>
      </tr>
    </table>

1. **SOF Packets** are used to send a Start of Frame (SOF) packet every 1 ms on
full speed links. The frame is used as a time frame in which to schedule the
data transfers which are required. For example, an isochronous endpoint will be
assigned one transfer per frame.

    <table border="1">
      <tr style="background:#5383EC; color:white">
        <th colspan="3" style="text-align:center">SOF Packet (3 bytes)</th>
      </tr>
      <tr>
        <td>PID<br>8 bits</td>
        <td>Frame Number<br>11 bits</td>
        <td>CRC5<br>5 bits</td>
      </tr>
    </table>

1. **Data Packets** are used for DATA0 and DATA1 packets. If a transaction has a
data stage this is the packet format used.

    <table border="1">
      <tr style="background:#5383EC; color:white">
        <th colspan="3" style="text-align:center">Data Packet (3 to 1,026 bytes)</th>
      </tr>
      <tr>
        <td>PID<br>8 bits</td>
        <td>DATA<br>0 to 1023 bytes</td>
        <td>CRC16<br>16 bits</td>
      </tr>
    </table>

1. **Handshake Packets** are used for ACK and NAK packets. This is the packet format
used in the status stage of a transaction, when required.

    <table border="1">
      <tr style="background:#5383EC; color:white">
        <th style="text-align:center">Handshake Packet (1 byte)</th>
      </tr>
      <tr>
        <td>PID<br>8 bits</td>
      </tr>
    </table>

### Transactions

A transaction represents one complete operation and consists of one token packet
(SETUP, IN, or OUT), one optional data packet (DATA0 or DATA1), and one
handshake packet (ACK, NAK, or STALL), in that order.

Each token packet contains the device address and endpoint address. An IN token
means the transaction flows IN to the host from the device. An OUT token means
the transaction flows OUT from the host to the device. A SETUP token is like an
OUT token, but is always followed by a specially formatted 8 byte DATA0 packet
used during a process known as enumeration (defined below). After the token
packet, the optional DATA0 or DATA1 packet may contain the data payload for the
transaction. The handshake packet will be an ACK to indicate successful receipt
of the transaction, a NAK to indicate that the receiver is currently unable to
handle this transaction but the sender should try again, or a STALL to indicate
that an error has occured and the sender should not retry the transaction.

### Transfers

A transfer refers to a higher-level operation that consists of one or more
transactions. A transfer is a complete sequence of actions that achieves a
particular communication goal between the USB host and the USB device. Different
types of USB transfers cater to different communication needs, such as control
transfers, bulk transfers, interrupt transfers, and isochronous transfers.

Here's a summary of what each type of transfer entails:

1. **Control Transfers:** These are used to configure or send commands to a USB
   device. A control transfer starts with a setup transaction (which has a
   specific request or command for the device), followed by zero or more data
   transactions (to transfer requisite data), and concludes with a status
   transaction (to acknowledge completion). This type of transfer is essential
   for initializing and configuring devices.

1. **Bulk Transfers:** These are used for large, non-time-critical data
   transfers, such as file transfers. Bulk transfers can ensure reliable data
   delivery but do not guarantee timing; they use available bandwidth after
   other higher-priority transfers (like interrupt or isochronous) are
   completed. They are typically used for devices like printers or external hard
   drives.

1. **Interrupt Transfers:** These are used for devices that need immediate
   attention, like a mouse or keyboard. Interrupt transfers allow a device to
   signal the host that it needs to send or receive a small amount of data
   immediately. These transfers are periodic and reserved, ensuring timely
   delivery but typically involve small amounts of data.

1. **Isochronous Transfers:** These are used for time-sensitive data, such as
   audio or video streams, where a constant data rate and timely delivery are
   more critical than perfect accuracy. Isochronous transfers can send or
   receive data in real-time but do not have error correction; lost data is not
   retransmitted.

A transfer, therefore, represents a complete communication process in USB terms,
consisting of multiple transactions to perform a significant function or data
exchange between the host and the device, according to the specific requirements
of the communication type (control, bulk, interrupt, or isochronous). Each
transfer type is optimized for different kinds of data and service requirements,
ensuring the versatile applicability of USB for various devices and
applications.

### Endpoints

### Devices

### Hubs

### Hosts

### Enumeration

When a USB device is attached to a USB bus, the host should perform a reset and
then begin a very specific process known as enumeration. This process uses only
SETUP packets, CONTROL transfers, endpoint zero (EP0), and device zero (dev0)
until a new device address is set. The purpose of enumeration is to assign the
device a unique address and determine its attributes so that it can operate
properly.

A simple enumeration process looks like:

* **Connect** - The device connects to the USB bus.
* **Reset** - The host resets the USB line state.
* **GDD/8/IN** - The host sends a `Get Device Descriptor` request and asks the
  device to send 8 bytes IN (from the device to the host) from the first 8 bytes
  of its device descriptor, using `80 06 00 01 00 00 08 00`. The device responds
  with the first 8 bytes of its device descriptor, in this example `12 01 10 02
  00 00 00 40` is sent. Once this is received, the host sends back a zero length
  status packet (called a "ZLP") to indicate that it has received the
  information from the device and is done with this request. The last byte of
  the packet sent by the device (`0x40` in this example), indicates that the
  maximum packet size of the device's endpoint zero (EP0) is 64 bytes. Using
  this information, the host then assigns the device a unique address and then
  requests the full device descriptor.
* **SDA/0/OUT** - The host sends a `Set Device Address` request, which includes
  the newly assigned device address and asks for 0 bytes of data back from the
  device. An example packet is `00 05 01 00 00 00 00 00`, with the `0x01`
  meaning that the device is being assigned device address 1. This request is an
  OUT request and requests 0 bytes of data in return. Thus, the device only
  responds with a ZLP to indicate that the request has been received and
  acknowledged.
* **GDD/18/IN** - Now that the device has been assigned address 1, the host will
  request the full device descriptor from device 1, on endpoint 0, using a
  maximum packet size of 64 bytes, and it asks for 18 bytes of data. The reason
  it asks for 18 bytes is because the first byte of the device's initial
  response (the `0x12`) indicated that its full device descriptor is 18 bytes.
  Thus, this request would look like: `80 06 00 01 00 00 12 00`. The device will
  respond with all 18 bytes and the host will send back a ZLP to indicate that
  it has received the device's response, which may look something like `12 01 10
  02 00 00 00 40 8a 2e 0c 00 03 01 01 02 03 01`.
* **GCD/9/IN** - The host now issues the `Get Configuration Descriptor` to
  request the device send the first 9 bytes of its configuration data. The
  process is similar to the requests above. The request looks like `80 06 00 02
  00 00 09 00` and the response looks like `09 02 62 00 03 01 00 80 32`.
* **SDC/0/OUT** - The host now issues a `Set Device Configuration` request to
  tell the device to use one of its configurations. In practice, this always
  seems to be configuration 1. The request looks like `00 09 01 00 00 00 00 00`
  and there is no response, except for the ZLP packet indicating the device is
  now configured.

At this point, the device has been assigned a unique address and is now
considered to be enumerated and configured.

## `RP2040:` USB Controller

### Memory Layout

The USB controller uses two blocks of memory.

The first block of memory contains endpoint data buffers and consists of 4,096
bytes of DPSRAM (dual-port static random access memory), beginning at address
0x5010000. The second block of memory contains additional registers and consists
of 156 bytes of standard (non-DPSRAM) memory, beginning at address 0x50110000.

DPSRAM memory is different from most memory on the RP2040 because it:

* Can be accessed by the processor and USB controller at the same time
* Supports 8, 16, and 32 bit access (most registers only support 32-bit access)
* Does not support the usb_hw_set and usb_hw_clear aliases

In the two memory block tables below, **bold** indicates the minimal set of the
registers and buffers to be able to perform *USB device enumeration* in either
host or device mode.

### `HOST:` Memory Block 1: 0x50100000 - 0x50100FFF

| Offset | Bytes | Host             | Info  |
| ---    | ---   | ---              | ---   |
| 0x00   | 8     | **SETUP packet** | 8 bytes for setup packet     |
| 0x08   | 4     | ECR1             | ECR for 1st polled endpoint  |
| 0x10   | 4     | ECR2             | ECR for 2nd polled endpoint  |
| 0x18   | 4     | ECR3             | ECR for 3rd polled endpoint  |
| ...    | ...   | ...              | ... |
| 0x78   | 4     | ECR15            | ECR for 15th polled endpoint |
| 0x80   | 4     | **BCR**          | Buffer control register (BCR) for software endpoint |
| 0x88   | 4     | BCR1             | BCR for 1st polled endpoint  |
| 0x90   | 4     | BCR2             | BCR for 2nd polled endpoint  |
| 0x98   | 4     | BCR3             | BCR for 3rd polled endpoint  |
| ...    | ...   | ...              | ... |
| 0x100  | 4     | **ECR**          | Endpoint control register (ECR) for software endpoint |
| ...    | ...   | ...              | ... |
| 0x180  | 64    | **Buffer 1**     | Data for 1st data buffer     |
| 0x1c0  | 64    | Buffer 2         | Data for 2nd data buffer     |
| 0x200  | 64    | Buffer 3         | Data for 3rd data buffer     |
| ...    | ...   | ...              | ... |
| 0xFC0  | 64    | Buffer 58        | Data for 58th data buffer    |

### `DEVICE:` Memory Block 1: 0x50100000 - 0x50100FFF

| Offset | Bytes | Device           | Info  |
| ---    | ---   | ---              | ---   |
| 0x00   | 8     | **SETUP packet** | 8 bytes for setup packet |
| 0x08   | 4     | ECR1/IN          | ECR for EP1/IN   |
| 0x10   | 4     | ECR1/OUT         | ECR for EP1/OUT  |
| 0x18   | 4     | ECR2/IN          | ECR for EP2/IN   |
| 0x20   | 4     | ECR2/OUT         | ECR for EP2/OUT  |
| ...    | ...   | ...              | ... |
| 0x78   | 4     | ECR15/IN         | ECR for EP15/IN  |
| 0x7C   | 4     | ECR15/OUT        | ECR for EP15/OUT |
| 0x80   | 4     | **BCR0/IN**      | BCR for EP0/IN   |
| 0x84   | 4     | **BCR0/OUT**     | BCR for EP0/OUT  |
| 0x88   | 4     | BCR1/IN          | BCR for EP1/IN   |
| 0x8C   | 4     | BCR1/OUT         | BCR for EP1/OUT  |
| ...    | ...   | ...              | ... |
| 0x100  | 64    | **EP0B**         | EP0 buffer 0 (shared between IN and OUT) |
| 0x140  | 64    | EP0B1            | EP0 buffer 1 (if double buffering on EP0) |
| 0x180  | 64    | Buffer 3         |     |
| ...    | ...   | ...              | ... |
| 0xFC0  | 64    | Buffer 60        | Data for 60th data buffer |

### Memory Block 2: 0x50110000 - 0x5011009B

| Offset | Host    | Info  |
| ---    | ---     | ---   |
| 0x00 | ADDR_ENDP (**DAR**) | Device and endpoint address register (DAR) for software endpoint |
| 0x04 | ADDR_ENDP1 | DAR for 1st polled endpoint |
| 0x08 | ADDR_ENDP2 | DAR for 2nd polled endpoint |
| 0x0c | ADDR_ENDP3 | DAR for 3rd polled endpoint |
| ...  | ...  | ... |
| 0x3C | ADDR_ENDP15 | DAR for 15th polled endpoint |
| 0x40 | **MAIN_CTRL** | Main control register |
| 0x44 | SOF_WR | `HOST:` Set the frame number for the next SOF (Start of Frame) sent by the host controller. SOF packets are sent every 1 ms and the host will increment the frame number by 1 each time. |
| 0x48 | SOF_RD | `DEVICE:` Read the frame number from the last SOF (Start of Frame) received by the device controller. |
| 0x4C | SIE_CTRL (**SCR**) | SIE control register |
| 0x50 | SIE_STATUS (**SSR**) | SIE status register |
| 0x54 | INT_EP_CTRL | Polled endpoint control register |
| 0x58 | **BUFF_STATUS** | Buffer status register. When an endpoint enables an interrupt for its buffer status, this register will be set each time that buffer completes. |
| 0x5C | BUFF_CPU_SHOULD_HANDLE (**BCH**) | When an endpoint is set for double buffering and sends an interrupt per buffer completion, this register toggles between 0 for buf_0 and 1 for buf_1 to indicate the buffer to handle. |
| 0x60 | EP_ABORT | `DEVICE:` For each bit in this register, a NAK will be sent for every access to the corresponding endpoint, overriding the BCR for that endpoint. |
| 0x64 | EP_ABORT_DONE | `DEVICE:` Used in conjunction with EP_ABORT. Set once an endpoint is idle, so the programmer knows it is safe to modify the BCR. |
| 0x68 | EP_STALL_ARM | `DEVICE:` This bit must be set in conjunction with the STALL bit in the BCR to send a STALL on EP0. The controller clears these bits when a SETUP packet is received. |
| 0x6C | NAK_POLL | `HOST:` If a device replies with a NAK, this value indicates the number of microseconds (μs) before trying again. |
| 0x70 | EP_STATUS_STALL_NAK | `DEVICE:` Each bit indicates that the corresponding endpoint has a NAK or STALL condition. EP0 is enabled via SIE_CTRL and all other endpoints are enabled in their ECR. |
| 0x74 | **USB_MUXING** | Controls where the USB controller is connected (usually `to_phy`). |
| 0x78 | **USB_PWR** | Controls VBUS and overcurrent configuration. |
| 0x7C | USBPHY_DIRECT | In conjunction with USBPHY_DIRECT_OVERRIDE, this register allows direct control of the USB Phy. |
| 0x80 | USBPHY_DIRECT_OVERRIDE | Enables overrides for each control in USBPHY_DIRECT. |
| 0x84 | USBPHY_TRIM | Used to adjust trim values for USB Phy pull down resistors. |
| 0x88 | Unknown | Unknown |
| 0x8C | INTR | Interrupt status (show all) |
| 0x90 | **INTE** | Enable interrupts |
| 0x94 | INTF | Forced interrupts |
| 0x98 | **INTS** | Interrupt status (masked or forced) |

### Endpoint control register (ECR)

The endpoint control register (ECR) controls how the endpoint behaves.

| Bits  | Info  |
| ---   | ---   |
| 31    | Enable this endpoint |
| 30    | 0: single buffer (64 bytes), 1: double buffering (64 bytes x 2)|
| 29    | Raise an interrupt for every buffer transferred |
| 28    | Raise an interrupt for every 2 buffers transferred (only for double buffering) |
| 27:26 | Type of endpoint, 0: control, 1: isochronous, 2: bulk, 3: interrupt |
| 25:18 | Polling interval minus 1 ms (14 means poll every 15 ms) |
| 15:0  | Byte offset for DPSRAM data buffer (lowest 6 bits must be zero) |

### Buffer control register (BCR)

The buffer control register (BCR) controls how to deal with single or double
buffering for each endpoint. If the endpoint is configured for single buffering,
then only the low bits (15:0) are used and they refer to the first buffer
(buffer 0). If the endpoint is configured for double buffering, then the high
bits (31:16) refer to the second/double buffer (buffer 1). Notice that each 16
bit half of this register is not exactly the same.

This register is also special because the upper 16 bits and lower 16 bits can be
read or written to independently. If this endpoint is single buffered, this has
no impact. But, if the endpoint is double buffered, then care should be taken to
write each 16 bit half of the register independently. In practice, this means it
is better to consider this 32 bit register as two separate 16 bit registers when
writing to it.

| Bits   | Info  |
| ---    | ---   |
| *High* | *Only used if double buffering and refers only to the second/double buffer (buffer 1)* |
| 31     | FULL buffer. `HOST:` 0: buffer is empty, ready to receive an IN packet from the device; 1: buffer is full, ready to send an OUT packet to the device. `DEVICE:` 0: buffer is empty, ready to receive an OUT packet from the host; 1: buffer is full, ready to send an IN packet to the host.
| 30     | LAST buffer in the transfer. If INTE has TRANS_COMPLETE set (bit 3), then the TRANS_COMPLETE interrupt will be raised when this buffer completes. |
| 29     | DATA PID for this packet. 0: DATA0, 1: DATA1. |
| 28:27  | `Isochronous endpoints:` Byte offset of this second buffer relative to the first buffer, 0: 128, 1: 256, 3: 1024. |
| 26     | Buffer is AVAILABLE for, and now owned by, the controller. 0: controller clears this bit to indicate that the processer now owns this buffer, 1: processor sets this bit to indicate that the controller now owns this buffer. |
| 25:16  | Buffer length for this buffer. |

| Bits   | Info  |
| ---    | ---   |
| *Low*  | *Used for the first buffer (buffer 0) of this endpoint* |
| 15     | FULL buffer. `HOST:` 0: buffer is empty, ready to receive an IN packet from the device; 1: buffer is full, ready to send an OUT packet to the device. `DEVICE:` 0: buffer is empty, ready to receive an OUT packet from the host; 1: buffer is full, ready to send an IN packet to the host.
| 14     | LAST buffer in the transfer. If INTE has TRANS_COMPLETE set (bit 3), then the TRANS_COMPLETE interrupt will be raised when this buffer completes. |
| 13     | DATA PID for this packet. 0: DATA0, 1: DATA1. |
| 12     | `DEVICE:` Reset buffer select to buffer 0. |
| 11     | `HOST:` Received a STALL. `DEVICE:` Send a STALL. |
| 10     | Buffer is AVAILABLE for, and now owned by, the controller. 0: controller clears this bit to indicate that the processer now owns this buffer, 1: processor sets this bit to indicate that the controller now owns this buffer. |
| 9:0    | Buffer length for this buffer. |

### SETUP Packets

The following is the SETUP packet structure:

```c
struct usb_setup_packet {
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
};
```

### SETUP requests

The first 8 bytes of DPSRAM (for both `host` mode and `device` mode) are
reserved for SETUP packets, which are exactly 8 bytes in length. In order for
the host controller to send a SETUP request, the following is needed:

1) The USB controller must have been reset and configured
2) The MAIN_CTRL must have a value such as 0x3
3) The DAR must have a value of 0x0
4) The software ECR must have a value such as 0xA0000180
5) The 8 byte SETUP packet must be written to address 0x50100000
6) The software BCR must have a value such as 0x0000047B
7) The SCR (SIE_CTRL) must have a value such as 0X20008E0B

Once conditions such as this are established, the host controller will send a
packet to the device to begin the enumeration process.

#### References

* https://www.usbmadesimple.co.uk/
* https://www.beyondlogic.org/usbnutshell/
* https://codelv.com/usb/setup-packets/
* https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf
* https://www.youtube.com/watch?v=wdgULBpRoXk
* https://www.youtube.com/watch?v=N0O5Uwc3C0o
* https://www.youtube.com/watch?v=2lPzTU-3ONI
* https://chat.openai.com/
