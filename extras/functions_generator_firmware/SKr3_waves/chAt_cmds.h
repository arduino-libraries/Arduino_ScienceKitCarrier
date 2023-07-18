#ifndef CHAT_COMMANDS_H
#define CHAT_COMMANDS_H

#include "chATWrapper.h"
#include "commands.h"

void CAtWrapper::add_cmds() {
   
   /* ....................................................................... */
   command_table[_ENTER_PRODUCTION_MODE] = [this](auto & srv, auto & parser) {
   /* ....................................................................... */
      switch (parser.cmd_mode) {
         case chAT::CommandMode::Write: {
            Serial.println("A");

            if (parser.args.size() != 1) {
               return chAT::CommandStatus::ERROR;
            }

            auto &production_code = parser.args[0];
            Serial.println(production_code.c_str());
            if(production_code == _PRODUCTION_CODE_MODE) {
               return chAT::CommandStatus::OK;
            }

            
            return chAT::CommandStatus::ERROR;
         }
         default:
             return chAT::CommandStatus::ERROR;
      }
      return chAT::CommandStatus::OK;
   };

}

#endif