/* 
  zeiss_stage.h - processes coordinates and speed, then passes it off to Zeiss Stage over CAN
  drop in for GRBL stepper control logic. 
*/

#include "grbl.h"
#include <mcp_can.h>
#include <SPI.h>

MCP_CAN CAN(9); // Set CS to pin 9

typedef struct {
    int cmd_no;
    long xyz[3];
    long speed;
} cmd_struct;

cmd_struct* cmd_buffer;
volatile int cmd_q_front; // will always point to next available command. if F = B, no command avail
volatile int cmd_q_back; // will always point to available free space. If B+1 = F, queue is full

#ifndef USE_LINE_NUMBERS
int cmds_since_reset;
#endif

volatile int last_run_cmd;

// initialize CAN bus and microscope stage settings
void mscope_init() {
  
   char can_data[8];

   while(CAN.begin(MCP_ANY, CAN_100KBPS, MCP_16MHZ) != CAN_OK) {
       delay(100);
   }
   CAN.setMode(MCP_NORMAL);

   cmds_since_reset = 0;
   last_run_cmd = 0;

   cmd_buffer = calloc(CMD_BUFFER_SIZE, sizeof(cmd_struct));
   cmd_q_front = cmd_q_back = 0;
   
   // Zeiss initialization commands
   sprintf(can_data, "xm1");
   CAN.sendMsgBuf(114, 0, 3, (unsigned char *)can_data);
   sprintf(can_data, "ym1");
   CAN.sendMsgBuf(114, 0, 3, (unsigned char *)can_data);

   // if required, set speed and accel commands here
   /*
   uint16_t accel = 2000;  // sets accel to "2.00 units" 
   sprintf(can_data, "<axis>A%ld", accel); // <axis> being X, Y, or Z
   uint8_t velocity = 25;  // actually limited to 0-127
   sprintf(can_data, "<axis>V%ld", accel); // <axis> being X, Y, or Z
   CAN.sendMsgBuf(114, 0, <data len>, (unsigned char *)can_data);
   */


   // we don't really care when the interrupt is called, 
   // by default it runs roughly once every millisecond 
   OCR0A = 0xA5;
}

// check if the axis is currently busy
uint8_t move_available(char axis_idx) {
  unsigned char len;
  unsigned char can_data[8];
  long unsigned int rxId;
  char res;
  char id;

  // send is available command on can
  sprintf(can_data, "%ct", ('X' + axis_idx));
  if (axis_idx < 2) id = XYID;
  else id = ZID;
  CAN.sendMsgBuf(id, 0, 2, (unsigned char *)can_data);

  // wait for response
  // note: this is not normally how CAN buses work, but in this case it behaves more like a master/slave serial line
  // a single CAN command will have an immediate CAN response. 
  // the bus is not populated, the client (us) always initiates. 
  while(CAN_MSGAVAIL != CAN.checkReceive()) {} 
  while(CAN_MSGAVAIL == CAN.checkReceive()) {
    
    delay(5); // need to let the message settle into the buffer
    CAN.readMsgBuf(&rxId, &len, can_data);

    // data returned is either 0 or 255 in hex-ascii 
    res = strtol((char *)can_data, NULL, 16);
  }

  return !res;
}

// main dequeue thread
ISR(TIMER0_COMPA_vect) {

    static int fired[3] = {0, 0, 0};

    // check if there is a command to send in the buffer
    if (cmd_q_front != cmd_q_back) {

      unsigned char len = 0;
      unsigned char can_data[8];
      char id;
    
      // foreach axis
      for(char i = 0; i < 3; i++) {
        if(move_available(i) && !fired[i]) {

          // this check is probably not needed in the grbl code
          long cur_move = cmd_buffer[cmd_q_front].xyz[i];
          if(cur_move != -1) {

            if(i < 2) id = XYID;
            else id = ZID;
          
            // send the command
            sprintf(can_data, "%cJ%06lx", ('X' + i), cur_move&0xFFFFFF);
            CAN.sendMsgBuf(id, 0, 8, (unsigned char *)can_data);
          }

          fired[i] = 1;
        }
      }

      if(fired[0] && fired[1] && fired[2]) {
        last_run_cmd = cmd_buffer[cmd_q_front].cmd_no;
    
        // clear the command from the buffer
        memset(&cmd_buffer[cmd_q_front], 0, sizeof(cmd_struct));
        cmd_q_front++;
        if(cmd_q_front >= CMD_BUFFER_SIZE) {
          cmd_q_front = 0;
        }

        fired[0] = fired[1] = fired[2] = 0;
      }
    }
    // else do nothing
}

// core call to process and send coordinate data to microscope
// target is float in grbl but changed to long for ease of testing
#ifdef USE_LINE_NUMBERS
  void queue_stage_cmd(long *target, float feed_rate, uint8_t invert_feed_rate, int32_t line_number) {
#else
  void queue_stage_cmd(long *target, float feed_rate, uint8_t invert_feed_rate) {
#endif
      // generate new cmd struct object
      cmd_struct cmd;

      // assumes x, y, z in order
      cmd.xyz[0] = target[0];
      cmd.xyz[1] = target[1];
      cmd.xyz[2] = target[2];

      /*
         invert_feed_rate is a boolean, if it's true the feed rate was desired as 1/feed_rate 
         both invert_feed_rate and feed_rate are currently unused, as Zeiss stage movement defualts
         are adequate for our needs. See mscope_init() above for setting global speed and accel rates
      */
      cmd.speed = feed_rate;
      
#ifdef USE_LINE_NUMBERS
      cmd.cmd_no = line_number;
#else
      cmd.cmd_no = cmds_since_reset;
      cmds_since_reset++;
#endif

      // check if there's room in the queue for another command
      // FIXME: this will hardlock the arduino if the buffer is filled before the ISR is started
      while(cmd_q_back + 1 == cmd_q_front || ((cmd_q_back + 1 == CMD_BUFFER_SIZE) && cmd_q_front == 0)) {
          // dwell here until there is
          // each delay statement costs 1ms, which aligns with a single ISR call
          // the original GRBL also dwelled when the queue was full, so original functionality isn't really changed
          delay(1);
      }
      memcpy(&cmd_buffer[cmd_q_back], &cmd, sizeof(cmd_struct));
      cmd_q_back++;
      if(cmd_q_back >= CMD_BUFFER_SIZE) cmd_q_back = 0; // wrap around case
}

/* 
   protocol.c uses to set system ACTIVE state 
   report.c uses to get current line number

   if using line numbers, returns that
   else, returns current number of commands since last reset

   replaces plan_get_current_block()
*/      
int32_t st_get_linenumber() {

    return last_run_cmd;

}

// start sending commands
void st_wake_up() {
    // enable dequeue ISR 
    TIMSK0 |= (1<<OCIE0A);
}

// halt sending commands
void st_go_idle() {
    // halt dequeue ISR
    TIMSK0 &= ~(1<<OCIE0A);
}

// reinitialize the important bits
void st_reset() {

    // stop sending messages
    st_go_idle();

    // clear command queue and reset pointers
    memset(cmd_buffer, 0, (sizeof(cmd_struct) * CMD_BUFFER_SIZE));
    cmd_q_front = cmd_q_back = 0;

    #ifndef USE_LINE_NUMBERS
    cmds_since_reset = 0;
    #endif
}
