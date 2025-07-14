# Protocol version
PROTOCOL_VERSION = 2.0

# Control table address
# config EEPROM
ADDR_OPERATING_MODE = 11
ADDR_CURRENT_LIMIT = 38
ADDR_MAX_POSITION_LIMIT = 48
ADDR_MIN_POSITION_LIMIT = 52

# Config RAM
ADDR_TORQUE_ENABLE = 64

# Error
ADDR_ERROR = 70

# Velocity PID Gains
ADDR_VELOCITY_P_GAIN = 76
ADDR_VELOCITY_I_GAIN = 78
ADDR_VELOCITY_D_GAIN = 80

# Reading
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_PWM = 124

# Writing
ADDR_GOAL_POSITION = 116
ADDR_GOAL_VELOCITY = 104
ADDR_GOAL_CURRENT = 102
ADDR_GOAL_PWM = 100
ADDR_PROFILE_VELOCITY = 112
ADDR_PROFILE_ACCELERATION = 108

# Status
ADDR_MOVING = 122
ADDR_MOVING_STATUS = 123

# Byte lens
LEN_VELOCITY = 4  # uint32_t (0.229 rpm/unit)
LEN_POSITION = 4  # uint32_t (0-4095 for 0-360°)
LEN_CURRENT = 2  # uint16_t (1 mA/unit)


operating_modes_xm = {"current": 0, "velocity": 1, "position": 3, "extended position": 4, "current-based position": 5, "pwm": 16}

operating_modes_xl = {"velocity": 1, "position": 3, "extended position": 4, "pwm": 16}


# 2s complement conversion
max_register_value = {
    "current": 65536,
    "pwm": 65536,
    "position": 4294967296,
    "velocity": 4294967296,
    "1 byte": 256,
    "2 bytes": 65536,
    "4 bytes": 4294967296,
}
