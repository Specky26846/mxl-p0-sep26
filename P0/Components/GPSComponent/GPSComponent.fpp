module P0 {
    @ NEO M9N component
    active component GPSComponent {

        # One async command/port is required for active components
        # This should be overridden by the developers with a useful command/port
        
        # resets gps fix state
        async command GPS_RESET opcode 0

        event GpsFixAcquired(
            numSats: U8
        ) severity activity high \
          format "GPS fix acquired with {} satellites"

        event GpsFixLost \
            severity warning high \
            format "GPS fix lost"

        event GpsDataUnavailable \
            severity warning high \
            format "GPS data unavailable; telemetry set to zero"

        event GpsReading(lat: F64, lon: F64, alt: F32, sats: U8) \
            severity activity low \
            format "GPS reading: lat={f}, lon={f}, alt={f} m, sats={}"

        telemetry Latitude: F64
        telemetry Longitude: F64
        telemetry Altitude: F32
        telemetry GroundSpeed: F32
        telemetry NumSatellites: U8
        telemetry UTCTime: U32
        
        # rate group input
        async input port run: Svc.Sched

        # i2c ports for neo-m9n
        output port busWriteRead: Drv.I2cWriteRead
        output port busWrite: Drv.I2c

        ###############################################################################
        # Standard AC Ports: Required for Channels, Events, Commands, and Parameters  #
        ###############################################################################
        @ Port for requesting the current time
        time get port timeCaller

        @ Enables command handling
        import Fw.Command

        @ Enables event handling
        import Fw.Event

        @ Enables telemetry channels handling
        import Fw.Channel

        @ Port to return the value of a parameter
        param get port prmGetOut

        @Port to set the value of a parameter
        param set port prmSetOut

    }
}