#include "control.h"
#include "robot_config.h"
#include <math.h>
#include <string.h>
static float vl, vr, il, ir;
static uint32_t last_command, last_step;
static bool armed, faulted;
static float clamp(float v,float lo,float hi){return fminf(hi,fmaxf(lo,v));}
static void stop(void){vl=vr=il=ir=0;board_safe_stop();}
void control_init(uint32_t now){
  armed=false;faulted=false;last_command=last_step=now;stop();
}
void control_command(uint8_t type,float a,float b,uint32_t now){
  if(!isfinite(a)||!isfinite(b))return;
  Sensors s=board_sensors();
  if(type==0){armed=false;stop();return;}
  if(faulted||!s.valid||s.estop||s.driver_fault)return;
  if(type==1){ /* 显式使能，禁止自动恢复 */
    if(s.arm_stowed&&s.battery_soc>ROBOT_LOW_SOC){
      armed=true;last_command=now;stop();
    }
    return;
  }
  if(!armed)return;
  if(type==2){ /* v m/s, omega rad/s；示例中心距0.50m */
    if(!s.arm_stowed)return;
    float l=a-b*ROBOT_TRACK_SPACING_M/2.0f,r=a+b*ROBOT_TRACK_SPACING_M/2.0f;
    float scale=fmaxf(1.0f,fmaxf(fabsf(l),fabsf(r))/ROBOT_MAX_TRACK_MPS);
    vl=l/scale;vr=r/scale;last_command=now;
  }else if(type==3){ /* 0收拢、1观察；不包含自动按钮按压 */
    if(a!=0.0f&&a!=1.0f)return;
    stop();
    if(fabsf(s.left_mps)<0.01f&&fabsf(s.right_mps)<0.01f){
      if(!board_arm_start((unsigned)a)){faulted=true;armed=false;}
      last_command=now;
    }
  }else if(type==4){last_command=now;} /* 独立心跳，不恢复使能 */
}
static float pi(float target,float actual,float *integral,float dt){
  if(fabsf(target)<0.00001f){*integral=0;return 0;}
  float e=target-actual,candidate=clamp(*integral+e*dt,-1,1);
  float u=ROBOT_SPEED_KP*e+ROBOT_SPEED_KI*candidate; /* 示例增益，必须实机整定 */
  if(fabsf(u)<=1||u*e<0)*integral=candidate;
  return clamp(u,-1,1);
}
void control_step(uint32_t now){
  uint32_t elapsed=now-last_step;
  if(elapsed<ROBOT_CONTROL_MS)return;
  last_step=now;
  Sensors s=board_sensors();
  bool invalid=!s.valid||!isfinite(s.left_mps)||!isfinite(s.right_mps)
    ||!isfinite(s.clearance_m)||s.clearance_m<0
    ||!isfinite(s.battery_soc)||s.battery_soc<0||s.battery_soc>1;
  if(elapsed>ROBOT_MAX_CYCLE_MS||invalid||s.estop||s.driver_fault||s.battery_soc<=ROBOT_CRITICAL_SOC){
    faulted=true;armed=false;stop();return;
  }
  if(!armed){stop();return;}
  if(now-last_command>ROBOT_COMMAND_TIMEOUT_MS){armed=false;stop();return;}
  /* 障碍恢复后仍等待新的运动指令，旧目标已清除。 */
  if(!s.arm_stowed||s.clearance_m<=ROBOT_STOP_CLEARANCE_M){stop();return;}
  float dt=elapsed*0.001f;
  board_motor(pi(vl,s.left_mps,&il,dt),pi(vr,s.right_mps,&ir,dt));
}
static uint16_t crc16(const uint8_t *p,unsigned n){
  uint16_t c=0xffff;
  while(n--){c^=*p++;for(unsigned i=0;i<8;i++)c=(c&1)?(c>>1)^0xa001:c>>1;}
  return c;
}
bool control_frame(const uint8_t f[16],uint32_t now){
  if(f[0]!=0xa5||f[1]!=0x5a||f[2]!=1)return false;
  uint16_t c=crc16(f,14);
  if(f[14]!=(uint8_t)c||f[15]!=(uint8_t)(c>>8))return false;
  float a,b;memcpy(&a,f+6,4);memcpy(&b,f+10,4);
  control_command(f[3],a,b,now);return true;
}
