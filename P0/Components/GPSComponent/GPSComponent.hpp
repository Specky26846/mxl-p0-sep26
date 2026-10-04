// ======================================================================
// \title  GPSComponent.hpp
// \author lauraf26846
// \brief  hpp file for GPSComponent component implementation class
// ======================================================================

#ifndef P0_GPSComponent_HPP
#define P0_GPSComponent_HPP

#include "P0/Components/GPSComponent/GPSComponentComponentAc.hpp"

namespace P0 {

// latest position solution decoded from nmea
struct GpsSolution {
    F64 lat = 0.0;
    F64 lon = 0.0;
    F32 alt = 0.0f;       // metres above msl
    F32 speed = 0.0f;     // m/s
    U8 numSats = 0;
    bool fixValid = false;
    U32 utcTime = 0;      // hhmmss
};

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
    U32 m_missedPackets = 0;  // consecutive ticks with no gga returned
    static constexpr U32 NMEA_LINE_MAX = 128;  // nmea spec caps sentences at 82 chars
    char m_nmeaLine[NMEA_LINE_MAX] = {};
    U32 m_nmeaLen = 0;
    F32 m_lastSpeed = 0.0f;   // from the most recent rmc
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
                                
    // drains the neo-m9n i2c stream and decodes nmea gga/rmc into a solution
    Drv::I2cStatus readGpsData(GpsSolution& sol, bool& packetFound);

    // accumulates one byte into the nmea line buffer, parsing on end of line
    void processNmeaByte(char c, GpsSolution& sol, bool& packetFound);

    // checksums and decodes one nmea sentence - true if a gga was decoded
    bool parseNmeaSentence(char* line, GpsSolution& sol);

    // sends a ubx config message over i2c
    void sendUbxCfg(const U8* payload, U8 payloadLen);
    
    // writes all gps telemetry channels and tracks fix state
    void reportGpsTelemetry(const GpsSolution& sol);
};

}  // namespace P0

#endif
