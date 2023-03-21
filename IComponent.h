#pragma once

#include <inttypes.h>
#include <memory>
#include <queue>

namespace UART
{
    /** Definition of control lines within control word. */
    enum CtrlFlags : uint16_t
    {
        Null = 0,
        All = 0xffff,
        BO = 0b0000000000000001,
        BI = 0b0000000000000010,
        AO = 0b0000000000000100,
        AI = 0b0000000000001000,
        RO = 0b0000000000010000,
        RI = 0b0000000000100000,
        MI = 0b0000000001000000,
        HI = 0b0000000010000000,
        EOFI = 0b0000000100000000,
        ES = 0b0000001000000000,
        EC = 0b0000010000000000,
        IC = 0b0000100000000000,
        II = 0b0001000000000000,
        CEME = 0b0010000000000000,
        CO = 0b0100000000000000,
        CI = 0b1000000000000000,
        ACTLOW = 0b1101100101111111
    };

    /** Component interface class of the CPU. */
    class IComponent
    {
    public:
        /** Virtual interface destructor. */
        virtual ~IComponent () = default;

        virtual void reset () = 0;
        virtual void fallingEdge () = 0;
        virtual void beingLow () = 0;
        virtual void risingEdge () = 0;
        virtual void gettingHigh () = 0;
        virtual void beingHigh () = 0;
    };

    class IRegister : public IComponent
    {
    public:
        /** Virtual interface destructor. */
        virtual ~IRegister () = default;

        virtual uint16_t get () const = 0;
    };

    using PortLine = uint8_t;
    using CtrlLine = uint16_t;
    using FlagLine = uint8_t;

    /** Create register A component. */
    IRegister* createRegA (PortLine* port, const CtrlLine* ctrl);
    IRegister* createRegB (PortLine* port, const CtrlLine* ctrl);
    IRegister* createRegInstr (PortLine* port, const CtrlLine* ctrl);
    IRegister* createRegFlags (FlagLine* flags, const CtrlLine* ctrl);
    IRegister* createRegSteps (PortLine* port, const CtrlLine* ctrl);
    IRegister* createRegPC (PortLine* port, const CtrlLine* ctrl);
    IRegister* createRegMar (PortLine* port, const CtrlLine* ctrl);

    IComponent* createAlu (PortLine* port,
                           const CtrlLine* ctrl,
                           const IRegister* regA,
                           const IRegister* regB,
                           FlagLine* flags);

    using InputBuffer = std::queue<uint8_t>;
    IComponent* createMemory (PortLine* port, const CtrlLine* ctrl, const IRegister* mar, InputBuffer* buffer);

    IComponent* createControl (PortLine* port,
                               CtrlLine* ctrl,
                               const IRegister* regInstr,
                               const IRegister* regFlags,
                               IRegister* regSteps);
}
