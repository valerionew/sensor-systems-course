#ifndef LIS2DE
#define LIS2DE

#define LIS2DE_ADDR_READ    (0b0101000<<1)
#define LIS2DE_ADDR_WRITE   (0b0101000<<1) + 1
#define LIS2DE12_ADDR_READ  (0b0011000<<1)
#define LIS2DE12_ADDR_WRITE (0b0011000<<1) + 1


#define LIS2DE_STATUS_REG_AUX       0x07U
#define LIS2DE_OUT_TEMP_L           0x0CU
#define LIS2DE_OUT_TEMP_H           0x0DU
#define LIS2DE_WHO_AM_I             0x0FU
#define LIS2DE_CTRL_REG0            0x1EU
#define LIS2DE_TEMP_CFG_REG         0x1FU
#define LIS2DE_CTRL_REG1            0x20U
#define LIS2DE_CTRL_REG2            0x21U
#define LIS2DE_CTRL_REG3            0x22U
#define LIS2DE_CTRL_REG4            0x23U
#define LIS2DE_CTRL_REG5            0x24U
#define LIS2DE_CTRL_REG6            0x25U
#define LIS2DE_REFERENCE            0x26U
#define LIS2DE_STATUS_REG           0x27U
#define LIS2DE_FIFO_READ_START      0x28U
#define LIS2DE_OUT_X_H              0x29U
#define LIS2DE_OUT_Y_H              0x2BU
#define LIS2DE_OUT_Z_H              0x2DU
#define LIS2DE_FIFO_CTRL_REG        0x2EU
#define LIS2DE_FIFO_SRC_REG         0x2FU
#define LIS2DE_INT1_CFG             0x30U
#define LIS2DE_INT1_SRC             0x31U
#define LIS2DE_INT1_THS             0x32U
#define LIS2DE_INT1_DURATION        0x33U
#define LIS2DE_INT2_CFG             0x34U
#define LIS2DE_INT2_SRC             0x35U
#define LIS2DE_INT2_THS             0x36U
#define LIS2DE_INT2_DURATION        0x37U
#define LIS2DE_CLICK_CFG            0x38U
#define LIS2DE_CLICK_SRC            0x39U
#define LIS2DE_CLICK_THS            0x3AU
#define LIS2DE_TIME_LIMIT           0x3BU
#define LIS2DE_TIME_LATENCY         0x3CU
#define LIS2DE_TIME_WINDOW          0x3DU
#define LIS2DE_ACT_THS              0x3EU
#define LIS2DE_ACT_DUR              0x3FU

#endif
