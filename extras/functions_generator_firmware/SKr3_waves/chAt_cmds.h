#ifndef CHAT_COMMANDS_H
#define CHAT_COMMANDS_H

#include "chATWrapper.h"
#include "commands.h"
#include "pin_def.h"
#include "led_gauge.h"
#include "led_array.h"
#include <string>


using namespace std;

void CAtWrapper::add_cmds() {
   
   /* ....................................................................... */
   command_table[_ENTER_PRODUCTION_MODE] = [this](auto & srv, auto & parser) {
   /* ....................................................................... */
      switch (parser.cmd_mode) {
         case chAT::CommandMode::Run: {
           app_status = APP_NORMAL;
           return chAT::CommandStatus::OK;
         }
         case chAT::CommandMode::Write: {
            if (parser.args.size() != 1) {
               return chAT::CommandStatus::ERROR;
            }
            auto &production_code = parser.args[0];
            if(production_code == _PRODUCTION_CODE_MODE) {
               app_status = APP_DIAGNOSTIC;
               srv.write_response_prompt();
               return chAT::CommandStatus::OK;
            }
            return chAT::CommandStatus::ERROR;
         }
         default:
             return chAT::CommandStatus::ERROR;
      }
      return chAT::CommandStatus::OK;
   };

   /* ....................................................................... */
   command_table[_READ_POTENTIOMETER] = [this](auto & srv, auto & parser) {
   /* ....................................................................... */
      if(app_status == APP_NORMAL) {
        return chAT::CommandStatus::ERROR;
      }
      
      switch (parser.cmd_mode) {
         case chAT::CommandMode::Read: {
           if (parser.args.size() != 1) {
               return chAT::CommandStatus::ERROR;
           }

           auto &pot = parser.args[0];
           int v = 0;
           if(pot == "P1") {
              v = analogRead(PH_1);
           }
           else if(pot == "P2") {
              v = analogRead(PH_2);
           }
           else if(pot == "F1") {
              v = analogRead(FQ_1);
           }
           else if(pot == "F2") {
              v = analogRead(FQ_2);
           }
           else {
              return chAT::CommandStatus::ERROR;
           }
           
           if(v > ADC_RES) {
              v = 0;
           }
           v = map(v, 0,ADC_RES, 0, 10);
           string val = to_string(v);
           srv.write_response_prompt();
           srv.write_str(val);
           //Serial.print(v);
           srv.write_line_end(); 
           srv.dontSendOk();  
           return chAT::CommandStatus::OK;
         }
      
         default:
             return chAT::CommandStatus::ERROR;
      }
      return chAT::CommandStatus::OK;
   };

   /* ....................................................................... */
   command_table[_LED_CMD] = [this](auto & srv, auto & parser) {
   /* ....................................................................... */
      if(app_status == APP_NORMAL) {
        return chAT::CommandStatus::ERROR;
      }
  
      switch (parser.cmd_mode) {
         case chAT::CommandMode::Write: {
            if (parser.args.size() != 1) {
               return chAT::CommandStatus::ERROR;
            }
            auto &st = parser.args[0];
            if(st == "ON") {
               app_status = APP_DIAGNOSTIC_LED_ON;
               srv.write_response_prompt();
               return chAT::CommandStatus::OK;
            }
            else if(st == "ON_F1") {
              app_status = APP_DIAGNOSTIC_LED_ON_F1;
              srv.write_response_prompt();
               return chAT::CommandStatus::OK;
            }
            else if(st == "ON_F2") {
              app_status = APP_DIAGNOSTIC_LED_ON_F2;
              srv.write_response_prompt();
              return chAT::CommandStatus::OK;
            }
            else if(st == "ON_R") {
              app_status = APP_DIAGNOSTIC_LED_ON_R;
              srv.write_response_prompt();
              return chAT::CommandStatus::OK;
            }
            else if(st == "OFF") {
               app_status = APP_DIAGNOSTIC;
               srv.write_response_prompt();
               return chAT::CommandStatus::OK;
            }
            return chAT::CommandStatus::ERROR;
         }
      
         default:
             return chAT::CommandStatus::ERROR;
      }
      return chAT::CommandStatus::OK;
   };

   /* ....................................................................... */
   command_table[_SET_FREQ_1] = [this](auto & srv, auto & parser) {
   /* ....................................................................... */
      if(app_status == APP_NORMAL) {
        return chAT::CommandStatus::ERROR;
      }
  
      switch (parser.cmd_mode) {
         case chAT::CommandMode::Write: {
            if (parser.args.size() != 1) {
               return chAT::CommandStatus::ERROR;
            }
            auto &f = parser.args[0];
            
            for(int i = 0; i < f.size(); i++) {
              if((f[i] >= 0x30 && f[i] <= 0x39) || f[i] == 0x2E ) {
                
              }
              else {
                return chAT::CommandStatus::ERROR;
              }
            }
            f1 = (float)atof(f.c_str());
            srv.write_response_prompt();
            return chAT::CommandStatus::OK;
         }
      
         default:
             return chAT::CommandStatus::ERROR;
      }
      return chAT::CommandStatus::OK;
   };

   /* ....................................................................... */
   command_table[_SET_FREQ_2] = [this](auto & srv, auto & parser) {
   /* ....................................................................... */
      if(app_status == APP_NORMAL) {
        return chAT::CommandStatus::ERROR;
      }
  
      switch (parser.cmd_mode) {
         case chAT::CommandMode::Write: {
            if (parser.args.size() != 1) {
               return chAT::CommandStatus::ERROR;
            }
            auto &f = parser.args[0];
            for(int i = 0; i < f.size(); i++) {
              if((f[i] >= 0x30 && f[i] <= 0x39) || f[i] == 0x2E ) {
                
              }
              else {
                return chAT::CommandStatus::ERROR;
              }
            }
            f2 = (float)atof(f.c_str());
            srv.write_response_prompt();
            return chAT::CommandStatus::OK;
         }
      
         default:
             return chAT::CommandStatus::ERROR;
      }
      return chAT::CommandStatus::OK;
   };

   
   /* ....................................................................... */
   command_table[_SET_AMPLITUDE_1] = [this](auto & srv, auto & parser) {
   /* ....................................................................... */
      if(app_status == APP_NORMAL) {
        return chAT::CommandStatus::ERROR;
      }
  
      switch (parser.cmd_mode) {
         case chAT::CommandMode::Write: {
            if (parser.args.size() != 1) {
               return chAT::CommandStatus::ERROR;
            }
            auto &f = parser.args[0];
            for(int i = 0; i < f.size(); i++) {
              if((f[i] >= 0x30 && f[i] <= 0x39) || f[i] == 0x2E ) {
                
              }
              else {
                return chAT::CommandStatus::ERROR;
              }
            }
            
            
            a1 = (float)atof(f.c_str());
            if(a1 > 1.0 || a1 < 0.0) {
              a1 = 0.0;
            }
            
            srv.write_response_prompt();
            return chAT::CommandStatus::OK;
         }
      
         default:
             return chAT::CommandStatus::ERROR;
      }
      return chAT::CommandStatus::OK;
   };
   /* ....................................................................... */
   command_table[_SET_AMPLITUDE_2] = [this](auto & srv, auto & parser) {
   /* ....................................................................... */
      if(app_status == APP_NORMAL) {
        return chAT::CommandStatus::ERROR;
      }
  
      switch (parser.cmd_mode) {
         case chAT::CommandMode::Write: {
            if (parser.args.size() != 1) {
               return chAT::CommandStatus::ERROR;
            }
            auto &f = parser.args[0];
            for(int i = 0; i < f.size(); i++) {
              if((f[i] >= 0x30 && f[i] <= 0x39) || f[i] == 0x2E ) {
                
              }
              else {
                return chAT::CommandStatus::ERROR;
              }
            }
            
            
            a2 = (float)atof(f.c_str());
            if(a2 > 1.0 || a2 < 0.0) {
              a2 = 0.0;
            }
            
            srv.write_response_prompt();
            return chAT::CommandStatus::OK;
         }
      
         default:
             return chAT::CommandStatus::ERROR;
      }
      return chAT::CommandStatus::OK;
   };
}

#endif
