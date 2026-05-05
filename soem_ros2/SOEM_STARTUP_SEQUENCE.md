# SOEM — EtherCAT Master Startup Sequence

This document explains, step by step, how SOEM opens the bus, discovers slaves, and brings them to the **Operational** state. All references point to source files in this repository.

---

## Overview

```
ec_init(ifname)
    └── ecx_setupnic()          → open RAW socket on NIC

ec_config_init(FALSE)
    ├── ecx_detect_slaves()     → count slaves via BRD
    ├── ecx_set_slaves_to_default() → reset registers via BWR
    └── per-slave loop
            ├── APWR → assign fixed node address
            ├── FPRD/FPWR → read topology, EEPROM (SII), mailbox config
            └── state: INIT → PRE-OP

ec_config_map(&IOmap)
    └── per-slave: configure FMMU + SyncManagers → state: PRE-OP → SAFE-OP

ec_configdc()
    └── synchronise Distributed Clocks (optional)

application loop
    ├── ec_writestate(0)        → request OPERATIONAL for all slaves
    ├── ec_send_processdata()   → LRW cyclic frame out
    └── ec_receive_processdata()→ LRW frame back, check WKC
```

---

## Step 1 — `ec_init(ifname)`: Open the RAW socket

**Source:** [`oshw/linux/nicdrv.c`](oshw/linux/nicdrv.c) → `ecx_setupnic()`

The very first call opens a **raw Linux packet socket** bound directly to the NIC — no IP stack involved:

```c
*psock = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_ECAT));   // EtherType 0x88A4
```

Key operations inside `ecx_setupnic()`:

| Operation | Purpose |
|-----------|---------|
| `socket(PF_PACKET, SOCK_RAW, ETH_P_ECAT)` | Capture/send only EtherCAT frames |
| `SIOCSIFFLAGS` with `IFF_PROMISC \| IFF_BROADCAST` | Receive all frames, including broadcast |
| `bind()` with `AF_PACKET` + `sll_ifindex` | Tie the socket to the specific NIC (e.g. `eth0`) |
| `ec_setupheader()` for each TX buffer | Pre-fill Ethernet header: dst=`FF:FF:FF:FF:FF:FF`, src=`priMAC`, EtherType=`0x88A4` |
| `pthread_mutex_init()` × 3 | Thread-safe index, TX and RX access |

> After this step the master can send and receive raw EtherCAT frames.

---

## Step 2 — `ec_config_init(FALSE)`: Slave discovery

**Source:** [`soem/ethercatconfig.c`](soem/ethercatconfig.c) → `ecx_detect_slaves()` + `ecx_config_init()`

### 2a — Count slaves (`ecx_detect_slaves`)

```c
// Reset all slaves to INIT and acknowledge errors
ecx_BWR(port, 0x0000, ECT_REG_ALCTL, sizeof(b), &b, EC_TIMEOUTRET3);

// Every slave on the bus increments wkc → wkc == number of slaves
wkc = ecx_BRD(port, 0x0000, ECT_REG_TYPE, sizeof(w), &w, EC_TIMEOUTSAFE);
*(context->slavecount) = wkc;
```

- **BWR** (Broadcast Write) — frame is written to every slave's register, then forwarded.
- **BRD** (Broadcast Read) — every slave increments the **Working Counter** by 1 before forwarding the frame. The returned `wkc` is exactly the slave count.

### 2b — Reset slave registers (`ecx_set_slaves_to_default`)

Another batch of BWR datagrams resets FMMUs, SyncManagers, DC registers, CRC error counters, etc., on all slaves simultaneously.

### 2c — Address assignment (per slave)

For each slave `1…N`, using **APWR** (Auto-increment Physical Write) commands:

```c
// Assign fixed node address: slave + EC_NODEOFFSET (0x3E9 by default)
ecx_APWRw(port, ADPh, ECT_REG_STADR, htoes(slave + EC_NODEOFFSET), EC_TIMEOUTRET3);
```

After this, every slave has a unique **fixed address** and can be addressed with FPRD/FPWR commands.

### 2d — EEPROM reading (SII — Slave Information Interface)

The master reads the SII EEPROM from each slave in pipelined batches:

| SII field | Content |
|-----------|---------|
| `ECT_SII_MANUF` | Manufacturer ID |
| `ECT_SII_ID` | Product code |
| `ECT_SII_REV` | Revision number |
| `ECT_SII_RXMBXADR` | Write mailbox address + size |
| `ECT_SII_TXMBXADR` | Read mailbox address + size |
| `ECT_SII_GENERAL` | Supported protocols (CoE, FoE, EoE, SoE), DC support, bus current |
| `ECT_SII_STRING` | Slave name string |
| `ECT_SII_SM` | SyncManager descriptors |
| `ECT_SII_FMMU` | FMMU function assignments |

### 2e — Topology detection

Each slave's `ECT_REG_DLSTAT` register is read to find active ports. This reconstructs the physical daisy-chain / tree topology and sets the `parent` field in `ec_slave[]`.

### 2f — INIT → PRE-OP transition

- Mailbox SyncManagers (SM0 write, SM1 read) are configured via FPWR.
- The state `EC_STATE_PRE_OP` is written to each slave's `ECT_REG_ALCTL`.
- `ecx_statecheck()` polls `ECT_REG_ALSTAT` until the slave confirms PRE-OP (or times out).

> In PRE-OP the **mailbox channel** (CoE/SDO) is available for configuration.

---

## Step 3 — `ec_config_map(&IOmap)`: PDO mapping and SAFE-OP

**Source:** [`soem/ethercatconfig.c`](soem/ethercatconfig.c) → `ecx_config_map_group()`

For each slave:

1. PDO assignment is read via CoE SDO (or SII if CoE is not available).
2. **FMMU** (Fieldbus Memory Management Unit) entries are configured: they map each slave's local process image into the master's flat `IOmap` byte array.
3. Process-data SyncManagers are configured:
   - **SM2** — outputs (master → slave)
   - **SM3** — inputs (slave → master)
4. The slave transitions to `EC_STATE_SAFE_OP`:
   - Process data exchange begins.
   - Slave inputs are valid, but **outputs are not yet applied** (safe-guard).

---

## Step 4 — `ec_configdc()`: Distributed Clocks (optional)

- Measures propagation delays between slaves by sending timestamped frames.
- Elects the first DC-capable slave as the **reference clock**.
- Synchronises all other slaves' DC clocks to it.

---

## Step 5 — SAFE-OP → OPERATIONAL

Back in the application (see [`test/linux/simple_test/simple_test.c`](test/linux/simple_test/simple_test.c)):

```c
// Wait for all slaves to confirm SAFE-OP
ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

// Request OPERATIONAL state
ec_slave[0].state = EC_STATE_OPERATIONAL;

// Send one valid process data frame to "warm up" slave outputs
ec_send_processdata();
ec_receive_processdata(EC_TIMEOUTRET);

// Broadcast OP request to all slaves
ec_writestate(0);

// Poll until all slaves confirm OP (max 200 retries)
do {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
} while (chk-- && ec_slave[0].state != EC_STATE_OPERATIONAL);
```

- `ec_send_processdata()` builds one **LRW** (Logical Read/Write) EtherCAT frame containing all slaves' output data and transmits it via the raw socket.
- `ec_receive_processdata()` receives the frame returned by the last slave (with all input data filled in) and verifies the **Working Counter**:
  - `expectedWKC = (outputsWKC × 2) + inputsWKC`
  - If `wkc >= expectedWKC` → all slaves responded correctly.

---

## EtherCAT State Machine

```
  ┌──────────────────────────────────────────────────────────────┐
  │  NIC socket open  (PF_PACKET / ETH_P_ECAT)                   │
  │      ec_init("eth0")  →  ecx_setupnic()                      │
  └──────────────────────────────┬───────────────────────────────┘
                                 │
                                 ▼
  ┌──────────────────────────────────────────────────────────────┐
  │  INIT                                                        │
  │  • BRD counts slaves                                         │
  │  • APWR assigns fixed node addresses                         │
  │  • EEPROM (SII) read: IDs, mailbox config, SM, FMMU          │
  │  • Topology (parent chain) reconstructed                     │
  └──────────────────────────────┬───────────────────────────────┘
                                 │  ec_config_init()
                                 ▼
  ┌──────────────────────────────────────────────────────────────┐
  │  PRE-OP                                                      │
  │  • Mailbox (CoE/SDO) available                               │
  │  • PDO assignment read/configured                            │
  └──────────────────────────────┬───────────────────────────────┘
                                 │  ec_config_map()
                                 ▼
  ┌──────────────────────────────────────────────────────────────┐
  │  SAFE-OP                                                     │
  │  • FMMU + SyncManagers configured                            │
  │  • Inputs valid; outputs frozen                              │
  │  • DC clocks synchronised  (ec_configdc)                     │
  └──────────────────────────────┬───────────────────────────────┘
                                 │  ec_writestate(OP)
                                 ▼
  ┌──────────────────────────────────────────────────────────────┐
  │  OPERATIONAL                                                 │
  │  • Outputs applied                                           │
  │  • Cyclic LRW loop running (ec_send/receive_processdata)     │
  │  • WKC checked every cycle                                   │
  └──────────────────────────────────────────────────────────────┘
```

---

## Key EtherCAT Command Types

| Command | Full name | Used for |
|---------|-----------|---------|
| `BWR` | Broadcast Write | Reset all slaves simultaneously |
| `BRD` | Broadcast Read | Count slaves (WKC increment trick) |
| `APWR` | Auto-increment Physical Write | Assign fixed addresses during init |
| `FPRD` | Fixed address Physical Read | Per-slave register/EEPROM reads |
| `FPWR` | Fixed address Physical Write | Per-slave SM/FMMU configuration |
| `LRW` | Logical Read/Write | Cyclic process data exchange |

---

## Buffer and Frame Management (`nicdrv.c`)

The driver maintains a pool of `EC_MAXBUF` (16) indexed TX/RX buffer slots:

```
ecx_getindex()      → allocate a free slot, set status = ALLOC
ecx_outframe_red()  → send TX buffer via primary socket (+ secondary if redundant)
ecx_inframe()       → non-blocking receive; match by index; store out-of-order frames
ecx_waitinframe()   → blocking receive with timeout
ecx_srconfirm()     → send + blocking receive, retry until WKC≥0 or timeout
```

In **redundant mode** (two NICs, two ring ports on each slave), both sockets send the same frame. Source MAC word 1 (`priMAC[1]` vs `secMAC[1]`) is used to determine which ring the returned frame traversed, allowing the driver to detect and compensate for partial cable failures transparently.
