#ifndef __CMD_H
#define __CMD_H

#include <cstdint>

enum class CmdType : uint16_t
{
    READ_VER_CMD                = 0x0000,
    WRITE_REG_CMD               = 0x0001,
    READ_REG_CMD                = 0x0002,
    WRITE_MTT_CMD               = 0x0003,
    READ_MTT_CMD                = 0x0004,
    WRITE_SN_CMD                = 0x0010,
    READ_SN_CMD                 = 0x0011,
    WRITE_SYS_CMD               = 0x0012,
    READ_SYS_CMD                = 0x0013,
    CASCADING_MODE_CMD          = 0x0014,
    FFT_ZEROFILL_CMD            = 0x0015,
    MCU_SPECIAL_FUNC_CMD        = 0x0017,
    START_I2C_TEST_CMD          = 0x0020,
    STOP_I2C_TEST_CMD           = 0x0021,
    GET_I2C_TEST_RESULT_CMD     = 0x0022,
    WRITE_PD_CALIBRATION_CMD    = 0x0026,
    READ_PD_CALIBRATION_CMD     = 0x0027,

    GET_MCU_ID_CMD              = 0x0065,
    SOFT_GET_APP_USE_CMD        = 0x0061,
    GET_APP_RUN_CMD             = 0x0060,
    SOFT_RESET_CMD              = 0x0064,
    GET_WHICH_DEMO              = 0X0066,

    START_CFG_CMD               = 0x00FF,
    FINISH_CFG_CMD              = 0x00FE,
};

#endif //__CMD_H