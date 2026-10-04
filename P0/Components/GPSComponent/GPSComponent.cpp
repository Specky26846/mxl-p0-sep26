// ======================================================================
// \title  GPSComponent.cpp
// \author lauraf26846
// \brief  cpp file for GPSComponent component implementation class
// ======================================================================

#include "P0/Components/GPSComponent/GPSComponent.hpp"
#include <cstdlib>
#include <cstring>
#include <Drv/Ports/I2cStatusEnumAc.hpp>

namespace P0 {

static constexpr U8  GPS_I2C_ADDRESS = 0x42;
static constexpr U8  GPS_DATA_REG    = 0xFF;
static constexpr U32 GPS_CHUNK_SIZE  = 32;

// upper bound on chunks read per tick - nmea output is ~1KB/s at default rates, so leave headroom to drain the buffer
static constexpr U32 GPS_MAX_CHUNKS  = 64;
static constexpr U32 NMEA_MAX_FIELDS = 20;
static constexpr F32 KNOTS_TO_MPS    = 0.514444f;

static U8 hexNibble(char c) {
    if (c >= '0' && c <= '9') return static_cast<U8>(c - '0');
    if (c >= 'A' && c <= 'F') return static_cast<U8>(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return static_cast<U8>(c - 'a' + 10);
    return 0xFF;
}

// nmea lat/lon is ddmm.mmmm (or dddmm.mmmm) Degrees and Decimal Minutes (geographic coord format)
static F64 nmeaToDegrees(const char* value, const char* hemisphere) {
    if (value[0] == '\0') return 0.0;
    F64 raw = strtod(value, nullptr);
    F64 deg = static_cast<F64>(static_cast<I32>(raw / 100.0));
    F64 result = deg + (raw - deg * 100.0) / 60.0;
    if (hemisphere[0] == 'S' || hemisphere[0] == 'W') result = -result;
    return result;
}

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

GPSComponent ::GPSComponent(const char* const compName) : GPSComponentComponentBase(compName) {}

GPSComponent ::~GPSComponent() {}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

void GPSComponent ::GPS_RESET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    this->m_hasFix = false;
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void GPSComponent::run_handler(FwIndexType portNum, U32 context) {
    GpsSolution sol;
    bool packetFound = false;

    Drv::I2cStatus status = this->readGpsData(sol, packetFound);
    if (status != Drv::I2cStatus::I2C_OK || !packetFound) {
        this->tlmWrite_Latitude(0.0);
        this->tlmWrite_Longitude(0.0);
        this->tlmWrite_Altitude(0.0f);
        this->tlmWrite_GroundSpeed(0.0f);
        this->tlmWrite_NumSatellites(0);
        this->tlmWrite_UTCTime(0);

        if (++this->m_missedPackets >= MAX_MISSED_PACKETS) {
            if (this->m_hasFix) {
                this->m_hasFix = false;
                this->log_WARNING_HI_GpsFixLost();
            }
            if (!this->m_reportedDataUnavailable) {
                this->log_WARNING_HI_GpsDataUnavailable();
                this->m_reportedDataUnavailable = true;
            }
        }
        return;
    }

    this->m_missedPackets = 0;
    this->m_reportedDataUnavailable = false;
    this->reportGpsTelemetry(sol);
}

// HELPER FUNCTIONS

// push all the gps telemetry channels and log when we acquire or lose a fix
void GPSComponent::reportGpsTelemetry(const GpsSolution& sol) {
    bool currentFix = sol.fixValid && (sol.numSats >= MIN_SATELLITES_FOR_FIX);

    this->tlmWrite_Latitude(sol.lat);
    this->tlmWrite_Longitude(sol.lon);
    this->tlmWrite_Altitude(sol.alt);
    this->tlmWrite_GroundSpeed(sol.speed);
    this->tlmWrite_NumSatellites(sol.numSats);
    this->tlmWrite_UTCTime(sol.utcTime);

    // throttle the event log - gga fires every second and event bandwidth is limited
    if (++m_readCount % READ_LOG_INTERVAL == 0) {
        this->log_ACTIVITY_LO_GpsReading(sol.lat, sol.lon, sol.alt, sol.numSats);
    }

    if (currentFix && !this->m_hasFix) {
        this->m_hasFix = true;
        this->log_ACTIVITY_HI_GpsFixAcquired(sol.numSats);
    } else if (!currentFix && this->m_hasFix) {
        this->m_hasFix = false;
        this->log_WARNING_HI_GpsFixLost();
    }
}

// NEO-M9N only gives data through reg 0xff in 32-byte chunks padded with 0xff fill - strip those and feed the nmea line parser
Drv::I2cStatus GPSComponent::readGpsData(GpsSolution& sol, bool& packetFound) {
    packetFound = false;
    U32 emptyChunks = 0;

    for (U32 attempts = 0; attempts < GPS_MAX_CHUNKS; attempts++) {
        U8 regAddr = GPS_DATA_REG;
        Fw::Buffer writeBuffer(&regAddr, sizeof(regAddr));
        U8 chunk[GPS_CHUNK_SIZE] = {};
        Fw::Buffer readBuffer(chunk, GPS_CHUNK_SIZE);

        Drv::I2cStatus status = this->busWriteRead_out(0, GPS_I2C_ADDRESS, writeBuffer, readBuffer);
        if (status != Drv::I2cStatus::I2C_OK) {
            return status;
        }

        // 0xff is fill, not real data - everything else is nmea ascii
        bool hasData = false;
        for (U32 i = 0; i < GPS_CHUNK_SIZE; i++) {
            if (chunk[i] != 0xFF) {
                this->processNmeaByte(static_cast<char>(chunk[i]), sol, packetFound);
                hasData = true;
            }
        }

        // 3 empty chunks in a row means the gps has nothing more to send
        if (!hasData) {
            emptyChunks++;
            if (emptyChunks >= 3) break;
        } else {
            emptyChunks = 0;
        }
    }

    // speed comes from rmc, which may have arrived on an earlier tick than the gga
    sol.speed = this->m_lastSpeed;
    return Drv::I2cStatus::I2C_OK;
}

// sentences can straddle chunk/tick boundaries, so accumulate into a persistent line buffer
void GPSComponent::processNmeaByte(char c, GpsSolution& sol, bool& packetFound) {
    if (c == '$') {
        this->m_nmeaLen = 0;
        this->m_nmeaLine[this->m_nmeaLen++] = c;
        return;
    }
    if (this->m_nmeaLen == 0) {
        return;  // waiting for start of a sentence
    }
    if (c == '\r' || c == '\n') {
        this->m_nmeaLine[this->m_nmeaLen] = '\0';
        if (this->parseNmeaSentence(this->m_nmeaLine, sol)) {
            packetFound = true;
        }
        this->m_nmeaLen = 0;
        return;
    }
    if (this->m_nmeaLen < NMEA_LINE_MAX - 1) {
        this->m_nmeaLine[this->m_nmeaLen++] = c;
    } else {
        this->m_nmeaLen = 0;  // overlong line, drop it
    }
}

// validates checksum and decodes gga/rmc - returns true only when a gga updated the solution
bool GPSComponent::parseNmeaSentence(char* line, GpsSolution& sol) {
    // checksum is xor of everything between '$' and '*', sent as two hex digits
    char* star = strchr(line, '*');
    if (star == nullptr || star[1] == '\0' || star[2] == '\0') return false;
    U8 hi = hexNibble(star[1]);
    U8 lo = hexNibble(star[2]);
    if (hi == 0xFF || lo == 0xFF) return false;
    U8 sum = 0;
    for (const char* p = line + 1; p < star; p++) {
        sum ^= static_cast<U8>(*p);
    }
    if (sum != static_cast<U8>((hi << 4) | lo)) return false;
    *star = '\0';

    // split on commas in place - strtok would collapse the empty fields nmea uses for "no data"
    char* fields[NMEA_MAX_FIELDS] = {};
    U32 numFields = 0;
    fields[numFields++] = line + 1;
    for (char* p = line + 1; *p != '\0' && numFields < NMEA_MAX_FIELDS; p++) {
        if (*p == ',') {
            *p = '\0';
            fields[numFields++] = p + 1;
        }
    }

    // match on sentence type only so any talker id works (GP, GN, GL...)
    size_t idLen = strlen(fields[0]);
    if (idLen < 5) return false;
    const char* type = fields[0] + idLen - 3;

    // $xxGGA,time,lat,N,lon,E,quality,numSV,hdop,alt,M,...
    if (strcmp(type, "GGA") == 0 && numFields >= 10) {
        sol.utcTime = static_cast<U32>(strtoul(fields[1], nullptr, 10));  // hhmmss
        sol.lat = nmeaToDegrees(fields[2], fields[3]);
        sol.lon = nmeaToDegrees(fields[4], fields[5]);
        sol.fixValid = strtol(fields[6], nullptr, 10) > 0;
        sol.numSats = static_cast<U8>(strtoul(fields[7], nullptr, 10));
        sol.alt = static_cast<F32>(strtod(fields[9], nullptr));  // metres above msl
        return true;
    }

    // $xxRMC,time,status,lat,N,lon,E,speed(knots),...
    if (strcmp(type, "RMC") == 0 && numFields >= 8) {
        this->m_lastSpeed = (fields[2][0] == 'A')
                                ? static_cast<F32>(strtod(fields[7], nullptr)) * KNOTS_TO_MPS
                                : 0.0f;
    }
    return false;
}

// ubx frame: 0xb5 0x62 (sync), class 0x06, id 0x8a (cfg-valset), len, payload, then fletcher checksum
void GPSComponent::sendUbxCfg(const U8* payload, U8 payloadLen) {
    U8 msg[64] = {};
    msg[0] = 0xB5;   // ubx sync char 1
    msg[1] = 0x62;   // ubx sync char 2
    msg[2] = 0x06;   // class: cfg
    msg[3] = 0x8A;   // id: valset
    msg[4] = payloadLen;
    msg[5] = 0x00;   // length high byte (always 0 for short configs)
    memcpy(&msg[6], payload, payloadLen);

    // fletcher checksum covers everything from the class byte onward (not the two sync bytes)
    U8 ck_a = 0, ck_b = 0;
    for (U8 i = 2; i < 6 + payloadLen; i++) {
        ck_a += msg[i];
        ck_b += ck_a;
    }
    msg[6 + payloadLen]     = ck_a;
    msg[6 + payloadLen + 1] = ck_b;

    Fw::Buffer writeBuffer(msg, 6 + payloadLen + 2);
    this->busWrite_out(0, GPS_I2C_ADDRESS, writeBuffer);
}

// tell the gps we're a cubesat, airborne dynamic model under 4g
void GPSComponent::configure() {
    U8 payload[] = {
        0x00, 0x03, 0x00, 0x00,
        0x21, 0x00, 0x11, 0x20,
        0x08
    };
    this->sendUbxCfg(payload, sizeof(payload));
}

}  // namespace P0
