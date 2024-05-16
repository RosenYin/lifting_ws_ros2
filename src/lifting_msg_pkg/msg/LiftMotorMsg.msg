uint8 motor_id
bool init_state

uint8 ctrl_mode
uint8 back_vol
float64 back_current

uint8 state_bit
bool run_state
bool overflow_i
bool overflow_v
bool encode_err
bool pos_bias_over
bool under_vol
bool over_load
bool external_ctrl_mode

int16 motor_speed
int16 back_speed
uint8 location_complete
int32 back_pos
int32 back_height
int64 target_height
int64 target_pos
bool up_limit
bool down_limit

bool reach_target_pos
int64 fps_error