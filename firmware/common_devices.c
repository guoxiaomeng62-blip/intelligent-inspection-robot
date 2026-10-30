#include "common_devices.h"
#include "robot_config.h"
#include <math.h>
void motor_write(Motor *m,float pwm,uint32_t now){
 if(!isfinite(pwm))pwm=0;
 pwm=fminf(1,fmaxf(-1,pwm));
 int dir=pwm<0?-1:1;
 if(pwm==0){__HAL_TIM_SET_COMPARE(m->timer,m->channel,0);return;}
 if(dir!=m->direction){
   __HAL_TIM_SET_COMPARE(m->timer,m->channel,0);
   if(m->pending!=dir){m->pending=dir;m->changed_at=now;return;}
   if(now-m->changed_at<MOTOR_REVERSAL_DELAY_MS)return;
   HAL_GPIO_WritePin(m->dir_port,m->dir_pin,dir>0?GPIO_PIN_SET:GPIO_PIN_RESET);
   m->direction=dir;m->pending=0;
   /* DIR改变后下一循环才输出PWM。 */
   return;
 }
 m->pending=0;
 uint32_t arr=__HAL_TIM_GET_AUTORELOAD(m->timer);
 __HAL_TIM_SET_COMPARE(m->timer,m->channel,(uint32_t)(fabsf(pwm)*arr));
}
static bool wr(I2C_HandleTypeDef *b,uint8_t reg,uint8_t value){
 return HAL_I2C_Mem_Write(b,PCA_ADDRESS_7BIT<<1,reg,I2C_MEMADD_SIZE_8BIT,&value,1,2)==HAL_OK;
}
bool pca_init(I2C_HandleTypeDef *b){
 /* PCA9685内部25MHz，50Hz舵机输出；只允许初始化时短暂等待。 */
 if(!wr(b,0x00,0x10)||!wr(b,0xfe,PCA_PRESCALE)||!wr(b,0x01,0x04)||!wr(b,0x00,0x20))return false;
 HAL_Delay(1);
 return wr(b,0x00,0xa0);
}
bool pca_servo(I2C_HandleTypeDef *b,unsigned ch,unsigned us){
 /* 默认限制为1000~2000us；按具体伺服手册设置，不能当作角度值。 */
 if(ch>=6||us<SERVO_MIN_US||us>SERVO_MAX_US)return false;
 unsigned off=(unsigned)(us*(PCA_OSCILLATOR_HZ/1000000.0f)/(PCA_PRESCALE+1u)+0.5f);
 uint8_t bytes[4]={0,0,(uint8_t)off,(uint8_t)(off>>8)};
 return HAL_I2C_Mem_Write(b,PCA_ADDRESS_7BIT<<1,0x06+4*ch,I2C_MEMADD_SIZE_8BIT,bytes,4,2)==HAL_OK;
}
bool sonar_trigger(Sonar *s,GPIO_TypeDef *p,uint16_t pin,TIM_HandleTypeDef *timer,uint32_t now){
 if(s->phase&&now-s->started<40)return false;
 s->phase=1;s->started=now;
 HAL_GPIO_WritePin(p,pin,GPIO_PIN_SET);
 uint32_t start=__HAL_TIM_GET_COUNTER(timer);
 /* TIM5必须32位自由运行1MHz，等待仅12us；中断保持开启。 */
 while((uint32_t)(__HAL_TIM_GET_COUNTER(timer)-start)<12){}
 HAL_GPIO_WritePin(p,pin,GPIO_PIN_RESET);
 return true;
}
void sonar_edge(Sonar *s,bool high,uint32_t us,uint32_t now){
 if(s->phase==1&&high){s->rise=us;s->phase=2;}
 else if(s->phase==2&&!high){s->width=us-s->rise;s->stamp=now;s->phase=0;}
}
bool sonar_read(Sonar *s,uint32_t now,float *metres){
 /* EXTI可能更新结构，用极短临界区取得一致快照。 */
 uint32_t mask=__get_PRIMASK();__disable_irq();
 uint32_t width=s->width,stamp=s->stamp;
 if(!mask)__enable_irq();
 if(!width||width<116||width>23200||now-stamp>SONAR_STALE_MS)return false;
 *metres=width*0.0001715f;return true;
}
float encoder_mps(uint16_t count,uint16_t *prev,float mpc,float dt){
 int32_t diff=(int32_t)count-*prev;*prev=count;
 if(diff>32767)diff-=65536;
 if(diff<-32768)diff+=65536;
 return dt>0?diff*mpc/dt:0;
}
