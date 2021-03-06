# smfd

### Supermicro fan daemon

A Linux daemon to manage fan speeds on Supermicro X10 (and possibly other) systems.

Copyright 2021 Ian Pilcher <<arequipeno@gmail.com>>

## Why?

In theory, this program should not be needed.  The BMC should manage the fan speeds to
balance noise and temperatures.  In reality, I have found that the BMC in my
[SYS-5028D-TN4T](https://www.supermicro.com/en/products/system/midtower/5028/SYS-5028D-TN4T.cfm)
does not adequately cool the hard drives in the system.

The BMC provides 4 fan management modes &mdash; standard, full, optimal, and heavy I/O.  Standard
and optimal modes adjust the CPU fan speed in respnse to changes in the CPU (and possibly PCH)
temperature, but they do not appear to monitor the hard disk temperatures, and the "system" fan
runs at a very low speed.  Full speed runs both the CPU and system fans at 100%; this does keep
the disk drives cool, but it runs the CPU fan at more than 6,000 RPM, which creates an annoying
whine.  Heavy I/O mode sounds promising, but it also runs the CPU fan at an unnecessarily high
speed, even when not required.

## How?

Fortunately, it is possible to exercise a degree of programatic control of the fan speeds (see
[here](https://forums.servethehome.com/index.php?resources/supermicro-x9-x10-x11-fan-speed-control.20/)
and [here](https://www.supermicro.com/support/faqs/faq.cfm?faq=31537)).  When the BMC fan mode is
set to full, fan zone duty cycles can be set via IPMI.

### Fan zones

The [X10SDV-TLN4F](https://www.supermicro.com/products/motherboard/Xeon/D/X10SDV-TLN4F.cfm) system
board in my server has 2 fan management zones.  Headers `FAN1`, `FAN2`, and `FAN3` appear to be
controlled by zone 0, and `FAN4` (which is inconveniently located under my GPU card) is controlled
by zone 1.  As shipped, the CPU fan is connected to `FAN2`.  My server's
[721TQ-250B ](https://www.supermicro.com/en/products/chassis/tower/721/SC721TQ-250B) chassis only
accomodates a single fan (in addition to the CPU fan), so I have connected it to `FAN4`.  This
configuration allows the two fans to be controlled independently; zone 0 controls the CPU fan, and
zone 1 controls the "system" fan.

## Installation

The installation steps below are written for Fedora 33, but they should work on (or be easily
adapted to) any modern Linux distribution.

#### 1. Install required libraries and header files

* [FreeIPMI](https://www.gnu.org/software/freeipmi/) for communication with the (local) BMC
* [libatasmart](http://0pointer.de/blog/projects/being-smart.html) for reading disk drive
  temperatures
* [LibYAML](https://github.com/yaml/libyaml) for parsing the configuration file

```
$ sudo dnf install freeipmi-devel libatasmart-devel libyaml-devel
```

#### 2. Clone this repository

```
$ git clone https://github.com/ipilcher/smfd.git
???

$ cd smfd
```

#### 3. Build and load the SELinux policy module

This requires that the `selinux-policy-devel` package be installed.

```
$ make -f /usr/share/selinux/devel/Makefile
???

$ sudo semodule -i smfd.pp
```

#### 4. Build the daemon

```
$ gcc -O3 -Wall -Wextra -o smfd smfd.c -lfreeipmi -latasmart -lyaml
```

#### 5. Install the daemon

```
$ sudo cp smfd /usr/local/bin/
$ sudo chcon -t smfd_exec_t /usr/local/bin/smfd
```

#### 6. Create the SDR cache

```
$ sudo ipmi-sensors
Caching SDR repository information: /root/.freeipmi/sdr-cache/sdr-cache-???.localhost
???

$ sudo mkdir /var/lib/smfd
$ sudo cp /root/.freeipmi/sdr-cache/sdr-cache-???.localhost /var/lib/smfd/sdr-cache
$ sudo chcon -R -t smfd_var_lib_t /var/lib/smfd
```

> **NOTE:** The name of the SDR cache file created by `ipmi-sensors` varies, based on the system
> hostname.

The output of `ipmi-sensors` includes the IPMI SDR record IDs of the fan sensors, which are needed
in the next step.  For example:

```
540  | FAN1            | Fan               | N/A        | RPM   | N/A
607  | FAN2            | Fan               | 4600.00    | RPM   | 'OK'
674  | FAN3            | Fan               | N/A        | RPM   | N/A
741  | FAN4            | Fan               | 2000.00    | RPM   | 'OK'
```

#### 7. Customize the configuration file

```
$ sudo mkdir /etc/smfd
$ sudo cp config.yaml /etc/smfd/
$ sudo chcon -R -t smfd_etc_t /etc/smfd
$ sudo vi /etc/smfd/config.yaml
???
```

#### 8. Create a user

```
$ sudo useradd -c 'Supermicro fan daemon' -d /var/lib/smfd -r -s /usr/sbin/nologin -G disk smfd
```

#### 9. Create a `udev` rule for the in-band IPMI device

```
$ echo 'SUBSYSTEM=="ipmi", KERNEL=="ipmi0", GROUP="smfd"' | sudo tee /etc/udev/rules.d/99-ipmi-smfd.rules
```

#### 10. Reboot the system and check the permissions of `/dev/ipmi0`

```
$ ls -l /dev/ipmi0
crw-rw----. 1 root smfd 239, 0 Mar 30 13:13 /dev/ipmi0
```

#### 11. Create a unit file

```
$ cat << EOF | sudo tee /etc/systemd/system/smfd.service
[Unit]
Description=Supermicro X10 fan daemon

[Service]
Type=simple
User=smfd
AmbientCapabilities=CAP_SYS_RAWIO
ExecStart=/usr/local/bin/smfd

[Install]
WantedBy=multi-user.target
EOF
???

$ sudo systemctl daemon-reload
```

#### 12. Enable and start the service

```
$ sudo systemctl enable smfd.service --now
```

#### 13. Check the log

Double check that the daemon is managing your fan speeds appropriately by checking its logs.

```
$ sudo journalctl -f -u smfd.service
```

## Signals

`smfd` reacts to two signals while it is running.

* `SIGUSR1` will toggle debugging messages on and off.

* `SIGUSR2` will cause the daemon to immediately log information about the system state and the
  temperatures that it monitors.  This is the same information is normally logged periodically.
  (See `log_interval` in `config.yaml`.)  Sending this signal resets the logging data and interval
  start.
