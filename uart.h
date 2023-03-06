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

            if ((result & 0xff) == 0)
            {
                mFlagLines |= 1;
            }
            else
            {
                // zero flag
                mFlagLines &= ~1;
            }

            if (result > 0xff)
            {
                mFlagLines |= 2;
            }
            else
            {
                // carry flag
                mFlagLines &= ~2;
            }

            if (result & 0x80)
            {
                mFlagLines |= 4;
            }
            else
            {
                // Negative flag.
                mFlagLines &= ~4;
            }

            if (mCtrlLines & mOutMask)
            {
                // Output result if EO is active.
                Port::set (uint8_t (result));
            }
        }

        // 74HC283 works asynchroneous, use "EO|AI, EO|RI" instead of "EO|AI|RI" (same for BI)
        static void gettingHigh ()
        {
            beingLow ();
        }
    };

    using Alu = Adder<BusLine, CtrlLine, Flags::EOFI, Flags::ES, Flags::EC, AReg, BReg, FlagLine>;
}
