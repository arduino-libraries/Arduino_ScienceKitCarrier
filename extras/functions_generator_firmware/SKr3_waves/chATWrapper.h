#ifndef CH_AT_WRAPPER_H
#define CH_AT_WRAPPER_H

#include "Arduino.h"
#include "chAT.hpp"

using namespace SudoMaker;

class CAtWrapper {
private:
   void add_cmds();
   std::unordered_map<std::string, std::function<chAT::CommandStatus(chAT::Server&, chAT::ATParser&)>> command_table;
   chAT::Server at_srv;
   arduino::HardwareSerial *serial;

   int mode;

public:
   CAtWrapper(arduino::HardwareSerial *s);
   CAtWrapper() = delete ;
   int run();
};





#endif
