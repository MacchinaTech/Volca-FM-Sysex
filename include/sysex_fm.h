#include "sysex_fm.c"

struct SysexHeader
{
    uint8_t status; 
    uint8_t ID; 
    uint8_t globalChannel;  
    uint8_t formatNumber;
    uint8_t byteCountMSB;
    uint8_t byteCountLSB;
    uint8_t end;
};

struct Operator
{
    uint8_t EG_rate_1;              //0-99
    uint8_t EG_rate_2;              //0-99
    uint8_t EG_rate_3;              //0-99
    uint8_t EG_rate_4;              //0-99
    uint8_t EG_level_1;             //0-99
    uint8_t EG_level_2;             //0-99
    uint8_t EG_level_3;             //0-99
    uint8_t EG_level_4;             //0-99
    uint8_t scale_breakPoint;       //0-99
    uint8_t scale_leftDepth;        //0-99
    uint8_t scale_rightDepth;       //0-99
    uint8_t scale_leftCurve;        //0-3
    uint8_t scale_rightCurve;       //0-3
    uint8_t scale_rate;             //0-7
    uint8_t amp_mod;                //0-3
    uint8_t key_velocity;           //0-7
    uint8_t output_level;           //0-99
    uint8_t osc_mode;               //0-1
    uint8_t freq_coarse;            //0-31
    uint8_t freq_fine;              //0-99
    uint8_t osc_detune;             //0-14
    uint8_t active;                 //0-1
};

struct Global
{
    uint8_t pitch_EG_rate_1;        //0-99
    uint8_t pitch_EG_rate_2;        //0-99
    uint8_t pitch_EG_rate_3;        //0-99
    uint8_t pitch_EG_rate_4;        //0-99
    uint8_t pitch_EG_level_1;       //0-99
    uint8_t pitch_EG_level_2;       //0-99
    uint8_t pitch_EG_level_3;       //0-99
    uint8_t pitch_EG_level_4;       //0-99
    uint8_t algorithm;              //0-31
    uint8_t feedback;               //0-7
    uint8_t osc_sync;               //0-1
    uint8_t LFO_speed;              //0-99
    uint8_t LFO_delay;              //0-99
    uint8_t LFO_pitchMod;           //0-99
    uint8_t LFO_ampMod;             //0-99
    uint8_t LFO_Sync;               //0-1
    uint8_t LFO_Wave;
    uint8_t pitch_mod;
    uint8_t transpose;
    uint8_t voice_name_1;
    uint8_t voice_name_2;
    uint8_t voice_name_3;
    uint8_t voice_name_4;
    uint8_t voice_name_5;
    uint8_t voice_name_6;
    uint8_t voice_name_7;
    uint8_t voice_name_8;
    uint8_t voice_name_9;
    uint8_t voice_name_10;  
};

struct SysexHeader device =
{
    .status = 0xF0,
    .ID = 0x43,
    .globalChannel = 0x00,
    .formatNumber = 0x00,
    .byteCountMSB = 0x01,
    .byteCountLSB = 0x1B,
    .end = 0xF7
};

struct Operator OP6 = 
{
    .EG_rate_1 = 0,
    .EG_rate_2 = 1,
    .EG_rate_3 = 2,
    .EG_rate_4 = 3,
    .EG_level_1 = 4,
    .EG_level_2 = 5,
    .EG_level_3 = 6,
    .EG_level_4 = 7,
    .scale_breakPoint = 8,
    .scale_leftDepth = 9,
    .scale_rightDepth = 10,
    .scale_leftCurve = 0,
    .scale_rightCurve = 0,
    .scale_rate = 0,
    .amp_mod = 0,
    .key_velocity = 5,
    .output_level = 16,
    .osc_mode = 0,
    .freq_coarse = 18,
    .freq_fine = 19,
    .osc_detune = 0,  
    .active = 1  
};

struct Operator OP5 = 
{
    .EG_rate_1 = 0,
    .EG_rate_2 = 1,
    .EG_rate_3 = 2,
    .EG_rate_4 = 3,
    .EG_level_1 = 4,
    .EG_level_2 = 5,
    .EG_level_3 = 6,
    .EG_level_4 = 7,
    .scale_breakPoint = 8,
    .scale_leftDepth = 9,
    .scale_rightDepth = 10,
    .scale_leftCurve = 0,
    .scale_rightCurve = 0,
    .scale_rate = 0,
    .amp_mod = 0,
    .key_velocity = 5,
    .output_level = 16,
    .osc_mode = 0,
    .freq_coarse = 18,
    .freq_fine = 19,
    .osc_detune = 0,  
    .active = 1  
};

struct Operator OP4 = 
{
    .EG_rate_1 = 0,
    .EG_rate_2 = 1,
    .EG_rate_3 = 2,
    .EG_rate_4 = 3,
    .EG_level_1 = 4,
    .EG_level_2 = 5,
    .EG_level_3 = 6,
    .EG_level_4 = 7,
    .scale_breakPoint = 8,
    .scale_leftDepth = 9,
    .scale_rightDepth = 10,
    .scale_leftCurve = 0,
    .scale_rightCurve = 0,
    .scale_rate = 0,
    .amp_mod = 0,
    .key_velocity = 5,
    .output_level = 16,
    .osc_mode = 0,
    .freq_coarse = 18,
    .freq_fine = 19,
    .osc_detune = 0,  
    .active = 1  
};

struct Operator OP3 = 
{
    .EG_rate_1 = 0,
    .EG_rate_2 = 1,
    .EG_rate_3 = 2,
    .EG_rate_4 = 3,
    .EG_level_1 = 4,
    .EG_level_2 = 5,
    .EG_level_3 = 6,
    .EG_level_4 = 7,
    .scale_breakPoint = 8,
    .scale_leftDepth = 9,
    .scale_rightDepth = 10,
    .scale_leftCurve = 0,
    .scale_rightCurve = 0,
    .scale_rate = 0,
    .amp_mod = 0,
    .key_velocity = 5,
    .output_level = 16,
    .osc_mode = 0,
    .freq_coarse = 18,
    .freq_fine = 19,
    .osc_detune = 0,  
    .active = 1  
};

struct Operator OP2 = 
{
    .EG_rate_1 = 0,
    .EG_rate_2 = 1,
    .EG_rate_3 = 2,
    .EG_rate_4 = 3,
    .EG_level_1 = 4,
    .EG_level_2 = 5,
    .EG_level_3 = 6,
    .EG_level_4 = 7,
    .scale_breakPoint = 8,
    .scale_leftDepth = 9,
    .scale_rightDepth = 10,
    .scale_leftCurve = 0,
    .scale_rightCurve = 0,
    .scale_rate = 0,
    .amp_mod = 0,
    .key_velocity = 5,
    .output_level = 16,
    .osc_mode = 0,
    .freq_coarse = 18,
    .freq_fine = 19,
    .osc_detune = 0,  
    .active = 1  
};

struct Operator OP1 = 
{
    .EG_rate_1 = 0,
    .EG_rate_2 = 1,
    .EG_rate_3 = 2,
    .EG_rate_4 = 3,
    .EG_level_1 = 4,
    .EG_level_2 = 5,
    .EG_level_3 = 6,
    .EG_level_4 = 7,
    .scale_breakPoint = 8,
    .scale_leftDepth = 9,
    .scale_rightDepth = 10,
    .scale_leftCurve = 0,
    .scale_rightCurve = 0,
    .scale_rate = 0,
    .amp_mod = 0,
    .key_velocity = 5,
    .output_level = 16,
    .osc_mode = 0,
    .freq_coarse = 18,
    .freq_fine = 19,
    .osc_detune = 0,  
    .active = 1  
};

struct Global global = 
{
.pitch_EG_rate_1 = 1,
.pitch_EG_rate_2 = 2,
.pitch_EG_rate_3 = 3,
.pitch_EG_rate_4 = 4,
.pitch_EG_level_1 = 5,
.pitch_EG_level_2 = 6,
.pitch_EG_level_3 = 7,
.pitch_EG_level_4 = 8,
.algorithm = 9,
.feedback = 0,
.osc_sync = 1,
.LFO_speed = 40,
.LFO_delay = 40,
.LFO_pitchMod = 40,
.LFO_ampMod = 40,
.LFO_Sync = 0,
.LFO_Wave = 2,
.pitch_mod = 0,
.transpose = 0,
.voice_name_1 = 0,
.voice_name_2 = 0,
.voice_name_3 = 0x52,
.voice_name_4 = 0x4F,
.voice_name_5 = 0x53,
.voice_name_6 = 0x53,
.voice_name_7 = 0x43,
.voice_name_8 = 0x4F,
.voice_name_9 = 0,
.voice_name_1 = 0
};
