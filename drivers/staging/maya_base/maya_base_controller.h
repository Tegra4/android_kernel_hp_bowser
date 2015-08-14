/*
 * maya_base_controller.h
 */

#ifndef _MAYA_BASE_CONTROLLER_H_
#define _MAYA_BASE_CONTROLLER_H_

//					cmd		1.Name			2.Mode	3.Format	4.Size in Bytes
#define EC_REG_BAT_CELLVOLTAGE4		0x00	//	BAT_CellVoltage4	R	unsigned int	2
#define EC_REG_BAT_CELLVOLTAGE3		0x01	//	BAT_CellVoltage3	R	unsigned int	2
#define EC_REG_BAT_CELLVOLTAGE2		0x02	//	BAT_CellVoltage2	R	unsigned int	2
#define EC_REG_BAT_CELLVOLTAGE1		0x03	//	BAT_CellVoltage1	R	unsigned int	2

#define EC_REG_SYSTEM_STATE		0x27	//	SYSTEM_STATE		W			1
#define EC_REG_ECRAM_EVENT1		0x28	//	ECRAM_EVENT1		W			1
#define EC_REG_WRITE_PROTECT		0x29	//	Write protection	WR			1

#define EC_REG_MUTE_LED_BLINKING	0x40	//	Mute LED blinking	W			1
#define EC_REG_CAPSLOCK_LED_BLINKING	0x41	//	CapsLock LED blinking	W			1


//					offset
#define EC_REG_KBC_VERSION		0x500	//	KBC version		R	String		6
#define	EC_REG_MASKROM			0xF011	//


/* System State */
#define MAYA_BASE_S0			0x00
#define MAYA_BASE_S3			0x03
#define MAYA_BASE_S4			0x04
#define MAYA_BASE_S5			0x05

/* ECRAM Event1 */
#define MAYA_BASE_LID_CLOSE		0x01
#define MAYA_BASE_MUTE_LED_ON		0x02

// MaskRom
#define MAYA_BASE_MASK_ROM_BIT		0x01

// Write Protection
#define MAYA_BASE_WRITE_LOCK		0x01

// #define EC_REG
#endif
