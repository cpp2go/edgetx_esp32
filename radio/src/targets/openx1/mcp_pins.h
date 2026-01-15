#ifndef _MCP_PINS_FOR_BOARD_H_
#define _MCP_PINS_FOR_BOARD_H_

#define MCP23017_DIR_REG 0xA0FFFFFF // G0A G0B G1A输入 G1B7输入 G1B(0-4,6)输出 （输入为1,输出为0）
#define MCP23017_PULLUP  0xA0FFFFFF

#define MCP_ROTARY_A_DET (1 << ((ROTARY_ENCODER_GPIO - 1) * 8 + ROTARY_ENCODER_GPIO_PIN_A)) // G0A6
#define MCP_ROTARY_B_DET (1 << ((ROTARY_ENCODER_GPIO - 1) * 8 + ROTARY_ENCODER_GPIO_PIN_B)) // G0A7

// input pins
#define USB_GPIO_PIN_VBUS  (8 * 3 + 5)  // G1B5
#define MCP_PWR_SW_DET (1 << (8 * 3 + 7))  // G1B7

// output pins
#define MCP_INTERNAL_PROTO_LED   (8 * 3 + 0)  // G1B0
//#define MCP_INTMOD_BOOT   (8 * 3 + 1)  // G1B1
#define MCP_INTMOD_5V_EN  (8 * 3 + 2)  // G1B2
//#define MCP_EXTMOD_BOOT   (8 * 3 + 3)  // G1B3
#define MCP_EXTMOD_5V_EN  (8 * 3 + 4)  // G1B4
#define MCP_PWR_EN (8 * 3 + 6)  // G1B6

#endif // _MCP_PINS_FOR_BOARD_H_