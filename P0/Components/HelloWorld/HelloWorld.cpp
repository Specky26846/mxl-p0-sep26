// ======================================================================
// \title  HelloWorld.cpp
// \author lauraf26846
// \brief  cpp file for HelloWorld component implementation class
// ======================================================================

#include "P0/Components/HelloWorld/HelloWorld.hpp"

namespace P0 {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

HelloWorld ::HelloWorld(const char* const compName) : HelloWorldComponentBase(compName) {}

HelloWorld ::~HelloWorld() {}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

void HelloWorld ::SAY_HI_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, const Fw::CmdStringArg& greeting) {
    Fw::LogStringArg eventGreeting(greeting.toChar());
    this->log_ACTIVITY_HI_Hello(eventGreeting);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

}  // namespace P0
