#include "scheduler.h"

#include "Arduino.h"

extern bool is_tick_elapsed();

CScheduler& CScheduler::getInstance() {
    static CScheduler    instance;
    return instance;
}



CScheduler::CScheduler() {
   unsigned char i = 0;
   for(i = 0; i < TASK_DEFINED_TASK; i++ ){
      TaskList[i].task = nullptr;
   }
}


CScheduler::~CScheduler() {


}


void CScheduler::add(Task_f func, TaskNum_t tsk) {
   switch(tsk) {

#if(USE_TASK_2ms == 1)
   case TASK_2ms:
      TaskList[TASK_2ms].task = func;
      TaskList[TASK_2ms].time = 2;
      TaskList[TASK_2ms].restart_time = 2;
   break;
#endif

#if(USE_TASK_5ms == 1)
   case TASK_5ms:
      TaskList[TASK_5ms].task = func;
      TaskList[TASK_5ms].time = 5;
      TaskList[TASK_5ms].restart_time = 5;
   break;
#endif

#if(USE_TASK_10ms == 1)
   case TASK_10ms:
      TaskList[TASK_10ms].task = func;
      TaskList[TASK_10ms].time = 9;
      TaskList[TASK_10ms].restart_time = 10;
   break;
#endif

#if(USE_TASK_15ms == 1)
   case TASK_15ms:
      TaskList[TASK_15ms].task = func;
      TaskList[TASK_15ms].time = 13;
      TaskList[TASK_15ms].restart_time = 15;
   break;
#endif

#if(USE_TASK_20ms == 1)
   case TASK_20ms:
      TaskList[TASK_20ms].task = func;
      TaskList[TASK_20ms].time = 21;
      TaskList[TASK_20ms].restart_time = 20;
   break;
#endif

#if(USE_TASK_50ms == 1)
   case TASK_50ms:
      TaskList[TASK_50ms].task = func;
      TaskList[TASK_50ms].time = 47;
      TaskList[TASK_50ms].restart_time = 50;
   break;
#endif

#if(USE_TASK_100ms == 1)
 case TASK_100ms:
    TaskList[TASK_100ms].task = func;
    TaskList[TASK_100ms].time = 93;
    TaskList[TASK_100ms].restart_time = 100;
 break;
#endif

#if(USE_TASK_200ms == 1)
   case TASK_200ms:
      TaskList[TASK_200ms].task = func;
      TaskList[TASK_200ms].time = 199;
      TaskList[TASK_200ms].restart_time = 200;
   break;
#endif

#if(USE_TASK_500ms == 1)
   case TASK_500ms:
      TaskList[TASK_500ms].task = func;
      TaskList[TASK_500ms].time = 505;
      TaskList[TASK_500ms].restart_time = 500;
   break;
#endif

#if(USE_TASK_1s == 1)
   case TASK_1sec:
      TaskList[TASK_1sec].task = func;
      TaskList[TASK_1sec].time = 997;
      TaskList[TASK_1sec].restart_time = 1000;
   break;
#endif
   default:
   break;
   }   
}

void CScheduler::run() {
   unsigned char i = 0;
   unsigned char oneTaskExecuted = 0;

   if(is_tick_elapsed()) {
      for(i = 0; i < TASK_DEFINED_TASK; i++) {
         if(TaskList[i].task != nullptr) {
            
            if(TaskList[i].time > 0) {
               TaskList[i].time--;
            }
            
            if(oneTaskExecuted == 0 && TaskList[i].time == 0) {
               TaskList[i].task();
               TaskList[i].time = TaskList[i].restart_time;
               oneTaskExecuted = 1;
            }
         }
      }
   }
}