ccflags-y	:= -D_CONN_IO373B_SMBD -D_EC_FLASH_MASK
ccflags-$(CONFIG_IO373B_DEBUG)	+= -DDEBUG

io373b-objs := io373b_smbd.o Ec.o EcFlash.o EcFlashMask.o MaskRom.o Conn.o
obj-$(CONFIG_IO373B_SMBD)	+= io373b.o
obj-$(CONFIG_IO373B_ACPI)	+= io373b_acpi.o
obj-$(CONFIG_IO373B_PS2)	+= io373b_ps2.o
