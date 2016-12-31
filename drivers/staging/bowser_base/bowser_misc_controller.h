/*
 * bowser_misc_controller.h
 */

#ifndef _BOWSER_BASE_CONTROLLER_H_
#define _BOWSER_BASE_CONTROLLER_H_

//					cmd		1.Name			2.Mode	3.Format	4.Size in Bytes
#define EC_REG_BAT_CELLVOLTAGE4		0x00	//	BAT_CellVoltage4	R	unsigned int	2
#define EC_REG_BAT_CELLVOLTAGE3		0x01	//	BAT_CellVoltage3	R	unsigned int	2
#define EC_REG_BAT_CELLVOLTAGE2		0x02	//	BAT_CellVoltage2	R	unsigned int	2
#define EC_REG_BAT_CELLVOLTAGE1		0x03	//	BAT_CellVoltage1	R	unsigned int	2

#define EC_REG_SYSTEM_STATE		0x27	//	SYSTEM_STATE		W			1
#define EC_REG_ECRAM_EVENT1		0x28	//	ECRAM_EVENT1		W			1
#define EC_REG_WRITE_PROTECT		0x29	//	Write protection	WR			1

#define EC_REG_KB_MATRIX_TYPE		0x38	//	Keyboard Matrix type	WR	unsigned char	1

#define EC_REG_MESSAGE_LED			0x39	//	Message Led		WR	unsigned char 1

#define EC_REG_MUTE_LED_BLINKING	0x40	//	Mute LED blinking	W			1
#define EC_REG_CAPSLOCK_LED_BLINKING	0x41	//	CapsLock LED blinking	W			1
#define EC_REG_SCREEN_STATE		0x42	//	Screen state		W			1
#define EC_REG_FORCE_DISCHARGE		0x45	//	battery force discharge	W			1


//					offset
#define EC_REG_KBC_VERSION		0x500	//	KBC version		R	String		6
#define	EC_REG_MASKROM			0xF011	//
#define EC_REG_GET_AC_STATUS		0xFC31  //	AC status		R


/* System State */
#define BOWSER_EC_S0			0x00
#define BOWSER_EC_S3			0x03
#define BOWSER_EC_S4			0x04
#define BOWSER_EC_S5			0x05

/* ECRAM Event1 */
#define BOWSER_MISC_LID_CLOSE		0x01
#define BOWSER_MISC_MUTE_LED_ON		0x02
#define BOWSER_MISC_S3_USB_VBUS		0x08

// MaskRom
#define BOWSER_EC_MASK_ROM_BIT		0x01

// Write Protection
#define BOWSER_BASE_WRITE_LOCK		0x01

// #define EC_REG
#endif
