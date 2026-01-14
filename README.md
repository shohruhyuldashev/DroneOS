
```
# 🚁 DroneOS — Bare-Metal Microkernel for ARM Cortex-A53

**DroneOS** — bu Raspberry Pi Zero 2W (ARM Cortex-A53) uchun noldan yozilgan **bare-metal microkernel** asosidagi eksperimental drone operating system.  
Loyiha maqsadi — **bootloader, kernel, scheduler va hardware driverlarni 0 dan yozib**, real embedded OS qanday ishlashini amaliy o‘rganish.



## ✨ Features

- Custom ARM startup code (`startup.s`)
- Custom linker script (`linker.ld`)
- Bare-metal C microkernel
- Priority-based task scheduler
- Simple heap allocator (`kmalloc`)
- UART debug driver
- Inter-Process Communication (IPC)
- Drone-specific tasks:
  - Flight Controller Task
  - Sensor Task
  - RC Receiver Task
  - Telemetry Task
- QEMU test build profile
- Real Raspberry Pi Zero 2W build profile


## 🧠 Architecture Overview

```

Boot ROM
↓
startup.s  → stack setup + BSS clear + kernel entry
↓
kernel_main()
↓
Scheduler + Tasks + Drivers

```


## 🛠 Build Requirements

Toolchain:

```

arm-none-eabi-gcc
arm-none-eabi-binutils
make

```

QEMU (for testing):

```

qemu-system-arm

````


## ⚙️ Build Instructions

### 🔹 Real Raspberry Pi Zero 2W Build

```bash
make rpi
````

Output:

```
kernel7.img
```

Copy to SD card boot partition:

```
kernel=kernel7.img
arm_64bit=0
enable_uart=1
```

Insert SD card into Raspberry Pi Zero 2W and connect UART (115200 baud).



### 🔹 QEMU Test Build

```bash
make qemu
```

Run in QEMU:

```
bash
qemu-system-arm \
  -M raspi2b \
  -kernel kernel7.img \
  -nographic
```

(Used for CPU + kernel logic testing)


## 🧩 File Structure

```
DroneOS/
│
├── startup.s        → ARM boot code
├── linker.ld        → Memory layout
├── kernel.c         → Microkernel core
├── drivers.c        → Hardware abstraction
├── flight_control.c
├── rc_receiver.c
├── Makefile
├── config.txt       → RPi boot config
└── README.md
```



## 🛰 Current Status

* ✅ ARM startup & linker working
* ✅ Kernel entry confirmed
* ✅ Scheduler implemented
* ✅ UART debug driver
* 🔧 Timer interrupts (in progress)
* 🔧 Context switching (planned)
* 🔧 MMU & user-space (future)


## 🎯 Project Goal

To build a **fully custom microkernel-based embedded OS** for drone flight control systems, focusing on:

* Low-level ARM architecture
* Real-time task scheduling
* Hardware driver development
* Secure and minimal OS design



## 👨‍💻 Author

**CyberBro (Shohruh)**
Embedded OS & Cybersecurity Enthusiast
Developing DroneOS from scratch 🚀



## 📜 License

This project is for educational and research purposes.

````



## ✅ Endi GitHub’ga qo‘shish

```bash
nano README.md     # yuqoridagi matnni joylash
git add README.md
git commit -m "Add project README"
git push
````



