// ======================================================================
// \title  GPSComponent.hpp
// \author lauraf26846
// \brief  hpp file for GPSComponent component implementation class
// ======================================================================

#ifndef P0_GPSComponent_HPP
#define P0_GPSComponent_HPP

#include "P0/Components/GPSComponent/GPSComponentComponentAc.hpp"

namespace P0 {

class GPSComponent final : public GPSComponentComponentBase {
  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct GPSComponent object
    GPSComponent(const char* const compName  //!< The component name
    );

    //! Destroy GPSComponent object
    ~GPSComponent();

    // configure the neo-m9n gps
    void configure();

  private:
    bool m_hasFix = false;
    bool m_reportedDataUnavailable = false;
    U32 m_readCount = 0;
    U32 m_missedPackets = 0;  // consecutive ticks with no nav-pvt returned
    static constexpr U8  MIN_SATELLITES_FOR_FIX = 4;
    static constexpr U32 READ_LOG_INTERVAL = 10;
    // 1 miss is normal timing jitter - fault after 5 consecutive misses at 1Hz (~5s with no data)
    static constexpr U32 MAX_MISSED_PACKETS = 5;
    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------

    //! Handler implementation for command GPS_RESET
    void GPS_RESET_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                              U32 cmdSeq            //!< The command sequence number
                              ) override;

    //! Handler implementation for the scheduled GPS poll
    void run_handler(FwIndexType portNum, U32 context) override;
                                
    // polls neo-m9n for ubx-nav-pvt and parses position/velocity
    Drv::I2cStatus readGpsData(F64& lat, F64& lon, F32& alt, F32& speed, U8& numSats, bool& packetFound);

    // sends a ubx config message over i2c
    void sendUbxCfg(const U8* payload, U8 payloadLen);
    
    // writes all gps telemetry channels and tracks fix state
    void reportGpsTelemetry(F64 lat, F64 lon, F32 alt, F32 speed, U8 numSats);
};

}  // namespace P0

#endif
