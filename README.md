PDB Linux Kernel
====================

The board is a Variscite [VAR-SOM-AM35](http://www.variscite.com/products/system-on-module-som/cortex-a8/var-som-am35-cpu-ti-am3517-am3505) without graphic support.

Board support
-------------

  * use NAND ECC 'BCH8' to be able to modify U-boot bch8 options
  * Add a kernel config to toggle NAND read-only/read-write mode


> Network card cannot read ethernet address from eeprom. Instead we read it from kernel boot parameter (eg. eth=ETHADDR)

System requirements
-------------------

For Ubuntu and Debian Linux distributions

    $ sudo apt-get install libncurses5-dev

Modify Kernel config
--------------------

Copy defconfig file and edit it with menuconfig 

    $ cp arch/arm/configs/var-som-am35_defconfig .config
    $ make ARCH=arm menuconfig

Create a new defconfig file and update the old one

    $ make ARCH=arm savedefconfig
    $ cp defconfig arch/arm/configs/var-som-am35_defconfig

