#include "chATWrapper.h"
#include "chAt_cmds.h"



/* -------------------------------------------------------------------------- */
int CAtWrapper::run() {
/* -------------------------------------------------------------------------- */   
   at_srv.run();
}



/* -------------------------------------------------------------------------- */
CAtWrapper::CAtWrapper(arduino::HardwareSerial *s) : mode(0) {
/* -------------------------------------------------------------------------- */   
   /* set up serial */
   serial = s;

   /* set up chatAt server callbacks */
   at_srv.set_io_callback({
      .callback_io_read = [this](auto buf, auto len) {
         if (!serial->available()) {
            yield();
            return (unsigned int)0;
         }
         return serial->readBytes(buf, min((unsigned int)serial->available(), len));
      },
      .callback_io_write = [this](auto buf, auto len) {
         return serial->write(buf, len);
      },
   });

   at_srv.set_command_callback([this](chAT::Server & srv, const std::string & command) {
      auto it = command_table.find(command);

      if (it == command_table.end()) {
         return chAT::CommandStatus::ERROR;
      } 
      else {
         return it->second(srv, srv.parser());
      }
   });

   add_cmds();
  
}