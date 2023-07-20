#ifndef CH_AT_WRAPPER_H
#define CH_AT_WRAPPER_H

#include "Arduino.h"
#include "chAT.hpp"

using namespace SudoMaker;

typedef enum {
  APP_NORMAL,
  APP_DIAGNOSTIC,
  APP_DIAGNOSTIC_LED_ON,
  APP_DIAGNOSTIC_LED_ON_F1,
  APP_DIAGNOSTIC_LED_ON_F2,
  APP_DIAGNOSTIC_LED_ON_R1,
  APP_DIAGNOSTIC_LED_ON_R
} AppSt_t;



class CAtWrapper {
private:
   void add_cmds();
   std::unordered_map<std::string, std::function<chAT::CommandStatus(chAT::Server&, chAT::ATParser&)>> command_table;
   chAT::Server at_srv;
   arduino::HardwareSerial *serial;

   AppSt_t app_status = APP_NORMAL;
   bool valid_command = false;

   float f1 = 0.0;
   float f2 = 0.0;
   float a1 = -1.0;
   float a2 = -1.0;

public:
   CAtWrapper(arduino::HardwareSerial *s);
   CAtWrapper() = delete ;
   int run();

   float getF1() { return (float)f1; }
   float getF2() { return (float)f2; }
   void resetF1() { f1 = 0.0; }
   void resetF2() { f2 = 0.0; }

   float getA1() { return (float)a1; }
   float getA2() { return (float)a2; }
   void resetA1() { a1 = -1.0; }
   void resetA2() { a2 = -1.0; }

   AppSt_t getStatus() {
      return app_status;
   }
   void resetStatus() {
      app_status = APP_NORMAL;
   }
};





#endif
