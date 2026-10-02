// ======================================================================
// \title  HelloWorld.hpp
// \author lauraf26846
// \brief  hpp file for HelloWorld component implementation class
// ======================================================================

#ifndef P0_HelloWorld_HPP
#define P0_HelloWorld_HPP

#include "P0/Components/HelloWorld/HelloWorldComponentAc.hpp"

namespace P0 {

class HelloWorld final : public HelloWorldComponentBase {
  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct HelloWorld object
    HelloWorld(const char* const compName  //!< The component name
    );

    //! Destroy HelloWorld object
    ~HelloWorld();

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------

    //! Handler implementation for command SAY_HI
    //!
    //! Command to issue greeting with maximum length of 20 characters
    void SAY_HI_cmdHandler(FwOpcodeType opCode,              //!< The opcode
                           U32 cmdSeq,                       //!< The command sequence number
                           const Fw::CmdStringArg& greeting  //!< Greeting to repeat in the SayHiEvent event
                           ) override;
};

}  // namespace P0

#endif
