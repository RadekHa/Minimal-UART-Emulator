#pragma once

#include "IComponent.h"

#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace UART
{
    /** Modeled after 74LS161 / 74HC161(4 - Bit Counter) bzw. 74HC173(4 - Bit Register Latch). */
    template<CtrlFlags InMask, CtrlFlags OutMask, CtrlFlags CountMask, CtrlFlags HiMask>
    class Register : public IRegister
    {
    public:
        Register (PortLine* port, const CtrlLine* ctrl)
            : m_data {}
            , m_portLine {port}
            , m_ctrlLine {ctrl}
        {
        }

        virtual ~Register () = default;
        /** The copy constructor is deleted. */
        Register (const Register&) = delete;
        /** The copy assignment operator is deleted. */
        Register& operator= (const Register&) = delete;

        virtual uint16_t get () const override
        {
            return m_data;
        }

        virtual void reset () override
        {
            m_data = 0;
        }

        virtual void fallingEdge () override
        {
        }

        virtual void beingLow () override
        {
            if (*m_ctrlLine & OutMask)
            {
                if (*m_ctrlLine & HiMask)
                {
                    *m_portLine = m_data >> 8;
                }
                else
                {
                    *m_portLine = m_data & 0x00ff;
                }
            }
        }

        virtual void risingEdge () override
        {
            if (*m_ctrlLine & InMask)
            {
                if (*m_ctrlLine & HiMask)
                {
                    m_data = (m_data & 0x00ff) | *m_portLine << 8;
                }
                else
                {
                    m_data = (m_data & 0xff00) | *m_portLine;
                }
            }
            else if ((*m_ctrlLine & CountMask) || (CountMask == CtrlFlags::AI))
            {
                m_data++;
            }
        }

        virtual void gettingHigh () override
        {
        }

        virtual void beingHigh () override
        {
        }

    private:
        /** Internal storage. */
        uint16_t m_data;

        PortLine* m_portLine;
        const CtrlLine* m_ctrlLine;
    };


    template<CtrlFlags OutMask, CtrlFlags InvMask, CtrlFlags CinMask>
    class Alu : public IComponent
    {
    public:
        Alu (PortLine* port,
             const CtrlLine* ctrl,
             const IRegister* registerA,
             const IRegister* registerB,
             FlagLine* flags)
            : m_portLine {port}
            , m_ctrlLine {ctrl}
            , m_registerA {registerA}
            , m_b {registerB}
            , m_flagLine {flags}
        {
        }

        virtual void fallingEdge ()  override
        {
        }

        virtual void beingLow ()  override
        {
            int a, b;                                                                                                               // A and B registers
            a = static_cast<int> (m_registerA->get ()) & 0xff;

            if (*m_ctrlLine & InvMask)
            {
                b = int (~(m_b->get ()) & 0xff);
            }
            else
            {
                b = int (m_b->get ()) & 0xff;
            }
            // add operation with carry in
            int result = a + b + bool (*m_ctrlLine & CinMask);

            // zero flag
            if ((result & 0xff) == 0)
            {
                *m_flagLine |= 1;
            }
            else
            {
                *m_flagLine &= ~1;
            }

            // carry flag
            if (result > 0xff)
            {
                *m_flagLine |= 2;
            }
            else
            {
                *m_flagLine &= ~2;
            }

            // negative flag
            if (result & 0x80)
            {
                *m_flagLine |= 4;
            }
            else
            {
                *m_flagLine &= ~4;
            }

            // output result if EO is active
            if (*m_ctrlLine & OutMask)
            {
                *m_portLine = static_cast<uint8_t> (result);
            }
        }

        virtual void risingEdge ()  override
        {
        }

        virtual void gettingHigh ()  override
        {
            // 74HC283 works asynchronous, use "EO|AI, EO|RI" instead of "EO|AI|RI" (same for BI)
            beingLow ();
        }

        virtual void beingHigh () override
        {
        }

    private:
        PortLine* m_portLine;
        const CtrlLine* m_ctrlLine;
        const IRegister* m_registerA;
        const IRegister* m_b;
        FlagLine* m_flagLine;
    };


    template<CtrlFlags InMask, CtrlFlags OutMask, size_t size = 0x8000>
    class Memory : public IComponent
    {
    public:
        Memory (PortLine* port, const CtrlLine* ctrl, const IRegister* mar, InputBuffer* buffer)
            : m_inBuffer {buffer}
            , m_portLine {port}
            , m_ctrlLine {ctrl}
            , m_mar {mar}
            , m_data {}
        {
        }

        void init (const std::filesystem::path& romName)
        {
            std::ifstream rom (romName, std::ios::binary | std::ios::in);

            if (rom)
            {
                rom.read (m_data.data (), 0x2000);
            }
            else
            {
                throw std::invalid_argument{"Memory file not found!"};
            }
        }

        virtual void fallingEdge ()  override
        {
        }

        virtual void beingLow ()  override
        {
            if (*m_ctrlLine & OutMask)
            {
                // Terminal -> Port
                if (m_mar->get () & 0x8000)
                {
                    if (!m_inBuffer->empty ())
                    {
                        *m_portLine = m_inBuffer->front ();
                        m_inBuffer->pop ();
                    }
                    else
                    {
                        *m_portLine = 0;
                    }
                }
                else
                {
                    // RAM -> Port
                    *m_portLine = m_data [m_mar->get ()];
                }
            }
        }

        virtual void risingEdge ()  override
        {
        }

        virtual void gettingHigh ()  override
        {
        }

        virtual void beingHigh () override
        {
            if (*m_ctrlLine & InMask)
            {
                if (((m_mar->get () & 0x8000) == 0x8000) && (*m_portLine != 0))
                {
                    // 0x8000-0xffff: schreiben in UART
                    std::cout << *m_portLine;
                }
                else if (m_mar->get () >= 0x2000)
                {
                    // 0x2000-0x7fff: RAM, do not overwrite ROM 0x0000-0x1fff																																			// 0x0000-0x7fff: schreiben in RAM
                    m_data [m_mar->get ()] = *m_portLine;
                }
            }
        }

    private:
        InputBuffer* m_inBuffer;
        std::array<char, size> m_data;

        PortLine* m_portLine;
        const CtrlLine* m_ctrlLine;
        const IRegister* m_mar;
    };


    class Control : public IComponent
    {
    public:
        Control (PortLine* port,
                 CtrlLine* ctrl,
                 const IRegister* regInstr,
                 const IRegister* regFlags,
                 IRegister* regSteps);

        void init (const std::filesystem::path& lsbName, const std::filesystem::path& msbName);

        virtual void fallingEdge ()  override;
        virtual void beingLow ()  override;
        virtual void risingEdge ()  override;
        virtual void gettingHigh ()  override;
        virtual void beingHigh () override;

    private:
        PortLine* m_portLine;
        CtrlLine* m_ctrlLine;
        const IRegister* m_regInstr;
        const IRegister* m_regFlags;
        IRegister* m_regSteps;
        uint16_t m_microcode [0x2000];
    };
}
