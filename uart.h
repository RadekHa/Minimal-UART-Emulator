#pragma once

#include <array>
#include <iostream>
#include <filesystem>

namespace UART
{
    // Definition of control lines within control word.
    enum Flags : uint16_t
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

    template<typename T>
    struct BaseType
    {
        using DataType = uint16_t;
    };

    template<>
    struct BaseType<class BusLine>
    {
        using DataType = uint8_t;
    };
    template<>
    struct BaseType<class FlagLine>
    {
        using DataType = uint8_t;
    };

    template<class T>
    class Line
    {
    public:
        using DataType = typename BaseType<T>::DataType;

        static DataType get ()
        {
            return m_data;
        }

        static void set (DataType data)
        {
            m_data = data;
        }

        static void reset ()
        {
            m_data = 0;
        }

    protected:
        static DataType m_data;
    };

    template<typename T>
    typename Line<T>::DataType Line<T>::m_data = 0;


    template<class T, class Port, class Ctrl, Flags InMask, Flags OutMask, Flags CountMask, Flags HiMask>
    class Register : public Line<T>
    {
    public:

        static void beingLow ()
        {
            if (Ctrl::get () & OutMask)
            {
                if (Ctrl::get () & HiMask)
                {
                    Port::set (Line<T>::get () >> 8);
                }
                else
                {
                    Port::set (Line<T>::get () & 0x00ff);
                }
            }
        }

        static void risingEdge ()
        {
            if (Ctrl::get () & InMask)
            {
                if (Ctrl::get () & HiMask)
                {
                    Line<T>::m_data = (Line<T>::get () & 0x00ff) | Port::get () << 8;
                }
                else
                {
                    Line<T>::m_data = (Line<T>::get () & 0xff00) | Port::get ();
                }
            }
            else if ((Ctrl::get () & CountMask) || (CountMask == Flags::All))
            {
                Line<T>::m_data++;
            }
        }
    };


    class BusLine : public Line<BusLine>
    {
    };

    class FlagLine : public Line<FlagLine>
    {
    };

    class CtrlLine : public Line<CtrlLine>
    {
    };

    class AReg : public Register<AReg, BusLine, CtrlLine, Flags::AI, Flags::AO, Flags::Null, Flags::Null>
    {
    };

    class BReg : public Register<BReg, BusLine, CtrlLine, Flags::BI, Flags::BO, Flags::Null, Flags::Null>
    {
    };

    class IReg : public Register<IReg, BusLine, CtrlLine, Flags::II, Flags::Null, Flags::Null, Flags::Null>
    {
    };

    class FReg : public Register<FReg, BusLine, CtrlLine, Flags::EOFI, Flags::Null, Flags::Null, Flags::Null>
    {
    };

    class SReg : public Register<SReg, BusLine, CtrlLine, Flags::EOFI, Flags::Null, Flags::All, Flags::Null>
    {
    };

    class PC : public Register<PC, BusLine, CtrlLine, Flags::CI, Flags::CO, Flags::CEME, Flags::HI>
    {
    };

    class Mar : public Register<PC, BusLine, CtrlLine, Flags::MI, Flags::Null, Flags::CEME, Flags::HI>
    {
    };


    template<class Port, class Ctrl, Flags OutMask, Flags InvMask, Flags CinMask, class AReg, class BReg, class Flag>
    class Adder
    {
    public:
        static void beingLow ()
        {
            // A and B registers
            int a, b;
            a = int (AReg::get ()) & 0xff;

            if (Ctrl::get () & InvMask)
            {
                b = int (~(BReg::get ()) & 0xff);
            }
            else
            {
                b = int (BReg::get ()) & 0xff;
            }
            // Add operation with carry in.
            int result = a + b + bool (Ctrl::get () & CinMask);

            // zero flag
            if ((result & 0xff) == 0)
            {
                Flag::Set (Flag::get () | 1);
            }
            else
            {
                Flag::Set (Flag::get () & ~1);
            }

            // carry flag
            if (result > 0xff)
            {
                Flag::Set (Flag::get () | 2);
            }
            else
            {
                Flag::Set (Flag::get () & ~2);
            }

            // Negative flag.
            if (result & 0x80)
            {
                Flag::Set (Flag::get () | 4);
            }
            else
            {
                Flag::Set (Flag::get () & ~4);
            }

            if (Ctrl::get () & OutMask)
            {
                // Output result if EO is active.
                Port::set (uint8_t (result));
            }
        }

        // 74HC283 works asynchronous, use "EO|AI, EO|RI" instead of "EO|AI|RI" (same for BI)
        static void gettingHigh ()
        {
            beingLow ();
        }
    };

    using Alu = Adder<BusLine, CtrlLine, Flags::EOFI, Flags::ES, Flags::EC, AReg, BReg, FlagLine>;

    class InputBuffer
    {
    public:
        static bool isValue ()
        {
            return m_dataPresent;
        }

        static char pop ()
        {
            m_dataPresent = false;
            return m_data;
        }

        static char m_data;
        static bool m_dataPresent;
    };

    char InputBuffer::m_data = 0;
    bool InputBuffer::m_dataPresent = false;


    template<class Port, class Ctrl, Flags InMask, Flags OutMask, class Mar, class InBuf, unsigned int size = 32768>
    class Memory
    {
    public:

        static void init (const std::filesystem::path& romName)
        {
            std::ifstream rom (romName, std::ios::binary | std::ios::in);

            if (rom)
            {
                rom.read (m_data.data (), 0x2000);
            }
        }

        static void beingLow ()
        {
            if ((Ctrl::get () & OutMask))
            {
                // Terminal -> Port
                if (Mar::get () & 0x8000)
                {
                    if (InBuf::isValue ())
                    {
                        Port::set (InBuf::pop ());
                    }
                    else
                    {
                        Port::reset ();
                    }
                }
                else
                {
                    // RAM -> Port
                    Port::set (m_data [Mar::get ()]);
                }
            }
        }

        static void beingHigh ()
        {
            if ((Ctrl::get () & InMask))
            {
                if (((Mar::get () & 0x8000) == 0x8000) && (Port::get () != 0))
                {
                    // 0x8000-0xffff: schreiben in UART
                    std::cout << Port::get ();
                }
                else if (Mar::get () >= 0x2000)
                {
                    // 0x2000-0x7fff: RAM, do not overwrite ROM 0x0000-0x1fff																																			// 0x0000-0x7fff: schreiben in RAM
                    m_data [Mar::get ()] = Port::get ();
                }
            }
        }

        static std::array<char, size> m_data;
    };

    template<class Port, class Ctrl, Flags InMask, Flags OutMask, class Mar, class InBuf, unsigned int size>
    std::array<char, size> Memory<Port, Ctrl, InMask, OutMask, Mar, InBuf, size>::m_data;

    using Mem = Memory<BusLine, CtrlLine, Flags::RI, Flags::RO, Mar, InputBuffer>;
}
