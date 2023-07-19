#ifndef SCIENCE_KIT_SCHEDULER
#define SCIENCE_KIT_SCHEDULER

extern bool is_tick_elapsed();


typedef enum {
#if(USE_TASK_2ms == 1)
 TASK_2ms,
#endif
#if(USE_TASK_5ms == 1)
 TASK_5ms,
#endif
#if(USE_TASK_10ms == 1)
 TASK_10ms,
#endif
#if(USE_TASK_15ms == 1)
 TASK_15ms,
#endif
#if(USE_TASK_20ms == 1)
 TASK_20ms,
#endif
#if(USE_TASK_50ms == 1)
 TASK_50ms,
#endif
#if(USE_TASK_100ms == 1)
 TASK_100ms,
#endif
#if(USE_TASK_200ms == 1)
 TASK_200ms,
#endif
#if(USE_TASK_500ms == 1)
 TASK_500ms,
#endif
#if(USE_TASK_1s == 1)
 TASK_1sec,
#endif
 TASK_DEFINED_TASK
} TaskNum_t;


using Task_f = void (*)();

class CTask {
public:
   Task_f task;
   unsigned short time;
   unsigned short restart_time;
};


class CScheduler {
public:
   static CScheduler& getInstance();
   CScheduler(CScheduler const&)               = delete;
   void operator=(CScheduler const&)           = delete;
   ~CScheduler();
   
   void add(Task_f func, TaskNum_t tsk);
   void shedule();
   

private:
   CScheduler();

   CTask TaskList[TASK_DEFINED_TASK];




};







#endif