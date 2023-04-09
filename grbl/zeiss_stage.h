/* 
  zeiss_stage.h - processes coordinates and speed, then passes it off to Zeiss Stage over CAN
  drop in for GRBL stepper control logic. 
*/

#ifndef zeiss_h
#define zeiss_h

#define CMD_BUFFER_SIZE 8

#define XYID 0x72
#define ZID  0xC

// initialize CAN bus and microscope stage settings
void mscope_init();

/*
  planner.c replacements
*/

/*
  commands will need to be queued as otherwise they will be lost 
  current movement status can be queried, so a system will need to be in place
  that constantly queries for movement status and sends commands on availability
*/
// core call to process and send coordinate data to microscope
#ifdef USE_LINE_NUMBERS
  void queue_stage_cmd(float *target, float speed, uint8_t invert_feed_rate, int32_t line_number);
#else
  void queue_stage_cmd(float *target, float speed, uint8_t invert_feed_rate);
#endif

/* 
   protocol.c uses to set system ACTIVE state 
   report.c uses to get current line number

   if using line numbers, returns that
   else, returns current number of commands since last reset

   replaces plan_get_current_block()
*/      
int32_t st_get_linenumber();

/*
  stepper.c replacements
*/

// start sending commands
void st_wake_up();

// halt sending commands
void st_go_idle();

// clear the command buffer
void st_reset();

#endif
