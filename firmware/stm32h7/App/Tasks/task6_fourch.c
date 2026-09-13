#include "task6.h"
#include "lcd.h"
#include "songjia_motor.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define N 4U
#define SONG_QUERY_MS 50U
#define DM_QUERY_MS 20U
#define CMS 20U
#define DMS 250U
#define FBT 150U
#define SINE_PERIOD_MS 20000U
#define SINE_COMMAND_MS 20U
#define SINE_PWM_PERMILLE 700
#define SINE_TWO_PI 6.28318530718f
#define STEP 10
#define LIM 120
#define CPR 1560L
#define SIGN_A (-1)
#define SIGN_B (-1)
#define SIGN_C (-1)
#define SIGN_D (-1)
typedef enum { SONG_PID, DM_PID } Mode;
typedef enum { CFG_TYPE, CFG_POL, CFG_PARAM, CFG_OK } Cfg;
typedef struct { float i, e; } Pid;
static const int8_t sign[N]={SIGN_A,SIGN_B,SIGN_C,SIGN_D};
static Mode mode; static Cfg cfg; static Pid pid[N];
static int16_t target, cps[N], rpm[N], pwm[N];
static uint8_t run, force, valid, song_command_pending, sine_run;
static uint32_t setup, query, control, display, tx, seq, sine_start, sine_command;
static int16_t sine_pwm;
static char cache[5][32]; static uint16_t ccol[5];
static int16_t clamp(int16_t x,int16_t lo,int16_t hi){return x<lo?lo:(x>hi?hi:x);}
static int16_t rpm_cps(int16_t x){return (int16_t)((int32_t)x*CPR/60L);}
static int16_t cps_rpm(int16_t x){int32_t a=x<0?-(int32_t)x:x; a=(a*60L+CPR/2L)/CPR;return x<0?(int16_t)-a:(int16_t)a;}
static int16_t rpm_cmps(int16_t x){int32_t a=x<0?-(int32_t)x:x; a=(a*419L+500L)/1000L;return x<0?(int16_t)-a:(int16_t)a;}
static void reset(void){uint8_t i;for(i=0;i<N;i++){pid[i].i=pid[i].e=0;pwm[i]=0;}}
static int16_t update(uint8_t k,int16_t t,int16_t a){float e=(float)t-a,d,o,ff=.40f*t;pid[k].i+=e*.02f;if(pid[k].i>1000)pid[k].i=1000;if(pid[k].i<-1000)pid[k].i=-1000;d=(e-pid[k].e)/.02f;pid[k].e=e;if(t>0)ff+=20;if(t<0)ff-=20;o=ff+.08f*e+.10f*pid[k].i+.001f*d;return clamp((int16_t)o,-300,300);}
static void line(uint16_t y,const char*s,uint16_t color,uint8_t n){if(!force&&valid&&ccol[n]==color&&!strcmp(cache[n],s))return;LCD_Fill(16,y,270,y+19,BLACK);LCD_ShowString(18,y,(const uint8_t*)s,color,BLACK,16,0);snprintf(cache[n],sizeof(cache[n]),"%s",s);ccol[n]=color;}
static void frame(void){LCD_WR_REG(0x28);LCD_Fill(0,0,LCD_W,LCD_H,BLACK);LCD_Fill(0,0,7,LCD_H,BRRED);LCD_ShowString(18,7,(const uint8_t*)"MG513P30 4CH TEST",WHITE,BLACK,24,0);LCD_DrawLine(18,35,268,35,BRRED);LCD_ShowString(18,196,(const uint8_t*)"UP/DN:SPEED MID:SINE",GRAY,BLACK,16,0);LCD_ShowString(18,216,(const uint8_t*)"RIGHT:MODE LEFT:STOP",YELLOW,BLACK,16,0);LCD_WR_REG(0x29);}
static void ui(uint32_t now){char s[32];uint8_t rx=SongjiaMotor_IsFeedbackFresh(now,FBT),ok=(uint32_t)(now-tx)<=FBT;line(44,mode==SONG_PID?"MODE: SONG PID":"MODE: DM PID",CYAN,0);if(sine_run)snprintf(s,sizeof(s),"SINE PWM: %+d%%",sine_pwm/10);else snprintf(s,sizeof(s),"TARGET: %+d RPM",target);line(68,s,WHITE,1);snprintf(s,sizeof(s),"A:%+d RPM B:%+d RPM",rpm[0],rpm[1]);line(96,s,WHITE,2);snprintf(s,sizeof(s),"C:%+d RPM D:%+d RPM",rpm[2],rpm[3]);line(120,s,WHITE,3);snprintf(s,sizeof(s),"UART TX:%s RX:%s",ok?"OK":"WAIT",rx?"OK":"LOST");line(148,s,ok&&rx?GREEN:YELLOW,4);LCD_Fill(16,172,270,191,BLACK);LCD_ShowString(18,172,(const uint8_t*)(sine_run?"STATE: SINE 20s":(run?"STATE: RUN":"STATE: STOP")),rx?GREEN:YELLOW,BLACK,16,0);valid=1;force=0;}
static void sent(uint8_t ok,uint32_t now){if(ok)tx=now;}
static void stop(uint32_t now){sent(SongjiaMotor_Stop(),now);run=0;sine_run=0;song_command_pending=0;sine_pwm=0;reset();}
static void input(InputEvent_t e,uint32_t now){if(e==INPUT_EVENT_CENTER){if(sine_run)stop(now);else if(cfg==CFG_OK&&SongjiaMotor_IsFeedbackFresh(now,FBT)){reset();run=1;sine_run=1;sine_start=now;sine_command=now-SINE_COMMAND_MS;}}else if(e==INPUT_EVENT_RIGHT&&!run){mode=mode==SONG_PID?DM_PID:SONG_PID;reset();}else if(!sine_run&&(e==INPUT_EVENT_UP||e==INPUT_EVENT_DOWN)){target=clamp((int16_t)(target+(e==INPUT_EVENT_UP?STEP:-STEP)),-LIM,LIM);if(mode==SONG_PID){run=target?1U:0U;song_command_pending=1U;}else if(!target)stop(now);else{run=1;control=now-CMS;}}if(e!=INPUT_EVENT_NONE)force=1;}
static void enter(uint32_t now){mode=SONG_PID;cfg=CFG_TYPE;target=sine_pwm=0;run=force=valid=song_command_pending=sine_run=0;memset(cps,0,sizeof(cps));memset(rpm,0,sizeof(rpm));reset();setup=query=control=display=sine_start=sine_command=now;tx=seq=0;frame();SongjiaMotor_Init();sent(SongjiaMotor_Stop(),now);sent(SongjiaMotor_SetMotorType(0),now);force=1;ui(now);}
static void tick(uint32_t now,InputEvent_t e){const SongjiaMotorFeedback_t*f;uint8_t i,fresh=0;uint32_t query_period,elapsed;int16_t t;SongjiaMotor_Process(now);if(cfg==CFG_TYPE&&now-setup>=50){sent(SongjiaMotor_SetEncoderPolarity(0),now);cfg=CFG_POL;setup=now;}else if(cfg==CFG_POL&&now-setup>=50){cfg=CFG_OK;query=now-SONG_QUERY_MS;}query_period=sine_run?DM_QUERY_MS:((mode==SONG_PID)?SONG_QUERY_MS:DM_QUERY_MS);if(cfg==CFG_OK&&now-query>=query_period){query=now;sent(SongjiaMotor_RequestEncoder20ms(),now);}input(e,now);f=SongjiaMotor_GetFeedback();if(f->sequence!=seq){seq=f->sequence;fresh=1;for(i=0;i<N;i++){int32_t v=(int32_t)f->encoder_20ms[i]*50L*sign[i];cps[i]=(int16_t)clamp((int16_t)v,-32768,32767);rpm[i]=cps_rpm(cps[i]);}}if(run&&!SongjiaMotor_IsFeedbackFresh(now,FBT))stop(now);if(sine_run&&now-sine_command>=SINE_COMMAND_MS){sine_command=now;elapsed=(uint32_t)(now-sine_start)%SINE_PERIOD_MS;sine_pwm=(int16_t)(sinf(SINE_TWO_PI*(float)elapsed/(float)SINE_PERIOD_MS)*(float)SINE_PWM_PERMILLE);sent(SongjiaMotor_SetPwmPermille(sine_pwm,sine_pwm,sine_pwm,sine_pwm),now);}else if(run&&mode==SONG_PID&&song_command_pending&&cfg==CFG_OK){t=rpm_cmps(target);sent(SongjiaMotor_SetSpeedCentiMps(t,t,t,t),now);song_command_pending=0U;}else if(run&&mode==DM_PID&&fresh){t=rpm_cps(target);for(i=0;i<N;i++)pwm[i]=update(i,t,cps[i]);sent(SongjiaMotor_SetPwmPermille(pwm[0],pwm[1],pwm[2],pwm[3]),now);}if(force||now-display>=DMS){display=now;ui(now);}}
static void exit(void){stop(HAL_GetTick());}
const AppTask_t Task6_Definition={enter,tick,exit};
