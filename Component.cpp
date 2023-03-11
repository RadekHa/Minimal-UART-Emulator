#include "Component.h"


namespace fs = std::filesystem;
using namespace std;
using namespace UART;


///////////////////////////////////////////////////////////////////////////////
// Factory functions.

IRegister* UART::createRegA (PortLine* port, const CtrlLine* ctrl)
{
    auto instance = make_unique<Register<CtrlFlags::AI, CtrlFlags::AO, CtrlFlags::Null, CtrlFlags::Null> > (port, ctrl);
    return instance.release ();
}

IRegister* UART::createRegB (PortLine* port, const CtrlLine* ctrl)
{
    auto instance = make_unique<Register<CtrlFlags::BI, CtrlFlags::BO, CtrlFlags::Null, CtrlFlags::Null> > (port, ctrl);
    return instance.release ();
}

IRegister* UART::createRegInstr (PortLine* port, const CtrlLine* ctrl)
{
    auto instance =
        make_unique<Register<CtrlFlags::II, CtrlFlags::Null, CtrlFlags::Null, CtrlFlags::Null> > (port, ctrl);
    return instance.release ();
}

IRegister* UART::createRegFlags (PortLine* port, const CtrlLine* ctrl)
{
    auto instance = make_unique<Register<CtrlFlags::EOFI, CtrlFlags::Null, CtrlFlags::Null, CtrlFlags::Null> > (port,
                                                                                                                ctrl);
    return instance.release ();
}

IRegister* UART::createRegSteps (PortLine* port, const CtrlLine* ctrl)
{
    auto instance =
        make_unique<Register<CtrlFlags::EOFI, CtrlFlags::AO, CtrlFlags::All, CtrlFlags::Null> > (port, ctrl);
    return instance.release ();
}

IRegister* UART::createRegPC (PortLine* port, const CtrlLine* ctrl)
{
    auto instance = make_unique<Register<CtrlFlags::CI, CtrlFlags::CO, CtrlFlags::CEME, CtrlFlags::HI> > (port, ctrl);
    return instance.release ();
}

IRegister* UART::createRegMar (PortLine* port, const CtrlLine* ctrl)
{
    auto instance = make_unique<Register<CtrlFlags::MI, CtrlFlags::Null, CtrlFlags::CEME, CtrlFlags::HI> > (port, ctrl);
    return instance.release ();
}

IComponent* UART::createAlu (PortLine* port,
                             const CtrlLine* ctrl,
                             const IRegister* regA,
                             const IRegister* regB,
                             FlagLine* flags)
{
    auto instance = make_unique<Alu<CtrlFlags::EOFI, CtrlFlags::ES, CtrlFlags::EC> > (port, ctrl, regA, regB, flags);
    return instance.release ();
}

IComponent* UART::createMemory (PortLine* port, const CtrlLine* ctrl, const IRegister* mar, InputBuffer* buffer)
{
    auto instance = make_unique<Memory<CtrlFlags::RI, CtrlFlags::RO> > (port, ctrl, mar, buffer);
    instance->init ("ROM.bin");
    return instance.release ();
}

IComponent* UART::createControl (PortLine* port,
                                 CtrlLine* ctrl,
                                 const IRegister* regInstr,
                                 const IRegister* regFlags,
                                 IRegister* regSteps)
{
    auto instance = make_unique<Control> (port, ctrl, regInstr, regFlags, regSteps);
    instance->init ("CTRL_LSB.bin", "CTRL_MSB.bin");
    return instance.release ();
}

///////////////////////////////////////////////////////////////////////////////
// Control

Control::Control (PortLine* port,
                  CtrlLine* ctrl,
                  const IRegister* regInstr,
                  const IRegister* regFlags,
                  IRegister* regSteps)
    : m_portLine {port}
    , m_ctrlLine {ctrl}
    , m_regInstr {regInstr}
    , m_regFlags {regFlags}
    , m_regSteps {regSteps}
    , m_microcode {}
{
}

void Control::init (const fs::path& lsbName, const fs::path& msbName)
{
    ifstream lsb (lsbName, std::ios::binary | std::ios::in);
    ifstream msb (msbName, std::ios::binary | std::ios::in);

    if (lsb && msb)
    {
        for (int i = 0; i < 0x2000; i++)
        {
            lsb.read ((char*) &m_microcode [i] + 0, 1);
            msb.read ((char*) &m_microcode [i] + 1, 1);
            m_microcode [i] ^= ACTLOW;
        }
    }
    else
    {
        throw invalid_argument{"Control files not found!"};
    }
}

void Control::reset ()
{
}

void Control::fallingEdge ()
{
    *m_ctrlLine =
        m_microcode [((m_regFlags->get () & 0b111) << 10) | ((m_regInstr->get () & 0b111111) << 4) |
                     (m_regSteps->get () & 0b1111)];

    // immediate asnychroneous reset
    if (*m_ctrlLine & CtrlFlags::IC)
    {
        m_regSteps->reset ();

        *m_ctrlLine =
            m_microcode [((m_regFlags->get () & 0b111) << 10) | ((m_regInstr->get () & 0b111111) << 4) |
                         (m_regSteps->get () & 0b1111)];
    }

    if (*m_ctrlLine & CtrlFlags::HI)
    {
        *m_portLine = 0x7f;
    }
    else
    {
        // also set the state of the port (pull-up to +5V)
        *m_portLine = 0xff;
    }
}

void Control::beingLow ()
{
}

void Control::risingEdge ()
{
}

void Control::gettingHigh ()
{
}

void Control::beingHigh ()
{
}
