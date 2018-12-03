// Temp/Humidity Driver
#require "HTS221.device.lib.nut:2.0.1"
// LED Drive
#require "APA102.device.lib.nut:2.0.0"
// GPS Location Libraries
#require "GPSParser.device.lib.nut:1.0.0"
#require "GPSUARTDriver.device.lib.nut:1.1.0"
// Accelerometer driver
#require "LIS3DH.device.lib.nut:2.0.2"

// LocationMonitor makes interfacing with the Pix Hawk Module easy
class LocationMonitor {

    _gps          = null;

    _lastLat      = null;
    _lastLng      = null;
    _locCheckedAt = null;
    _isMoving     = null;

    _geofenceCB   = null;
    _gfCtr        = null;
    _distFromCtr  = null;
    _inBounds     = null;

    constructor(configurePixHawk) {
        // Configure class constants
        const GPS_BAUD_RATE    = 9600; // This is the default for ublox, but if it doesn't work try 38400
        const GPS_RX_FIFO_SIZE = 4096;
        // Use to reduce niose, so gps isn't jumping around when asset is not moving
        const LOC_THRESHOLD    = 0.00030;

        // delay a bit to ensure any previous I2C transactions have completed
        // as Pixhawk may corrupt I2C transactions during power-on
        imp.sleep(0.5);
        hardware.pinYG.configure(DIGITAL_OUT, 1);
        hardware.uartNU.setrxfifosize(GPS_RX_FIFO_SIZE);
        // Configure UART
        hardware.uartNU.configure(GPS_BAUD_RATE, 8, PARITY_NONE, 1, NO_CTSRTS);
        // Ensure Pixhawk tx line is high and stable
        imp.sleep(0.5);

        // Pixhawk may not be in the correct mode when booted, send command
        // to configure GPS to send NMEA sentences
        // Note this doesn't change the boot state of the pixhawk, so will need
        // to be called on every boot if needed.
        if (configurePixHawk) {
            _sendPixhawkConfigCommand(hardware.uartNU, GPS_BAUD_RATE);
        }

        // Initialize GPS UART Driver
        local gpsOpts = { "gspDataReady" : gpsHandler.bindenv(this),
                          "parseData"    : true,
                          "baudRate"     : GPS_BAUD_RATE };
        _gps = GPSUARTDriver(hardware.uartNU, gpsOpts);
    }

    function getLocation() {
        return {"lat" : _lastLat, "lng" : _lastLng, "ts" : _locCheckedAt, "isMoving": _isMoving};
    }

    function gpsHandler(hasLoc, data) {
        if (hasLoc) {
            // print(data);
            local lat = _gps.getLatitude();
            local lng = _gps.getLongitude();

            // Updated location if it has changed
            if (locChanged(lat, lng) ) {
                _lastLat = lat;
                _lastLng = lng;
            }
            // Update location received timestamp
            _locCheckedAt = time();

            // XX Bug: Do not calculate distence when no geofence
            // if ("sentenceId" in data && data.sentenceId == GPS_PARSER_GGA) {
            //     calculateDistance(data);
            // } 

        } else if (!_gps.hasFix() && "numSatellites" in data) {
            // This will log a ton - use to debug only, not in application
            server.log("GSV data received. Satellites: " + data.numSatellites);
        }
    }

    function inBounds() {
        return _inBounds;
    }

    function enableGeofence(distance, ctrLat, ctrLng, cb) {
        _distFromCtr = distance;
        _geofenceCB = cb;

        // use a hardcoded altitude, 30 meters
        local alt = 30.00;
        try {
            local lat = ctrLat.tofloat();
            local lng = ctrLng.tofloat();
            _gfCtr = _getCartesianCoods(lat, lng, alt);
        } catch(e) {
            server.error("Error configuring geofence coordinates: " + e);
        }

    }

    function disableGeofence() {
        _geofenceCB = null;
        _gfCtr = null;
        _distFromCtr = null;
        _inBounds = null;
    }

    // Use location threshold to filter out noise when not moving
    function locChanged(lat, lng) {
        local changed = false;

        if (_lastLat == null || _lastLng == null) {
            changed = true;
        } else {
            local latDiff = math.fabs(lat.tofloat() - _lastLat.tofloat());
            local lngDiff = math.fabs(lng.tofloat() - _lastLng.tofloat());
            if (latDiff > LOC_THRESHOLD) changed = true;
            if (lngDiff > LOC_THRESHOLD) changed = true;
        }
        _isMoving = changed;
        return changed;
    }

    function calculateDistance(data) {
        // Only calculate if we have altitude, latitude and longitude
        if (!("altitude" in data) || !("latitude" in data) || !("longitude" in data)) return;

        try {
            local lat = data.latitude.tofloat();
            local lng = data.longitude.tofloat();
            local alt = data.altitude.tofloat();

            local new  = _getCartesianCoods(lat, lng, alt);
            local dist = math.sqrt((new.x - _gfCtr.x)*(new.x - _gfCtr.x) + (new.y - _gfCtr.y)*(new.y - _gfCtr.y) + (new.z - _gfCtr.z)*(new.z - _gfCtr.z));

            // server.log("New distance: " + dist + " M");
            local inBounds = (dist <= _distFromCtr);
            if (_geofenceCB != null && inBounds != _inBounds) {
                _geofenceCB(inBounds);
            }
            // Track previous state, so we only trigger callback on a change
            _inBounds = inBounds;
        } catch (e) {
            // Couldn't calculate
            server.error("Error calculating distance: " + e);
        }
    }

    function _getCartesianCoods(lat, lng, alt) {
        local latRad = lat * PI / 180;
        local lngRad = lng * PI / 180;
        local cosLat = math.cos(latRad);
        local result = {};

        result.x <- alt * cosLat * math.sin(lngRad);
        result.y <- alt * math.sin(latRad);
        result.z <- alt * cosLat * math.cos(lngRad);

        return result;
    }

    function _sendPixhawkConfigCommand(uart, baudrate) {
        server.log("Configuring pixhawk...");

        // UBX CFG-PRT command values
        local header          = 0xb562;     // Not included in checksum
        local portConfigClass = 0x06;
        local portConfigId    = 0x00;
        local length          = 0x0014;
        local port            = 0x01;       // uart port
        local reserved1       = 0x00;
        local txReady         = 0x0000;     // txready not enabled
        local uartMode        = 0x000008c0; // mode 8 bit, no parity, 1 stop
        local brChars         = (baudrate > 57600) ? format("%c%c%c%c", baudrate, baudrate >> 8, baudrate >> 16, 0) : format("%c%c%c%c", baudrate, baudrate >> 8, 0, 0);
        local inproto         = 0x0003;     // inproto NMEA and UBX
        local outproto        = 0x0002;     // outproto NMEA
        local flags           = 0x0000;     // default timeout
        local reserved2       = 0x0000;

        // Assemble UBX payload
        local payload = format("%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",
            portConfigClass,
            portConfigId,
            length,
            length >> 8,
            port,
            reserved1,
            txReady,
            txReady >> 8,
            uartMode,
            uartMode >> 8,
            uartMode >> 16,
            uartMode >> 24,
            brChars[0],
            brChars[1],
            brChars[2],
            brChars[3],
            inproto,
            inproto >> 8,
            outproto,
            outproto >> 8,
            flags,
            flags >> 8,
            reserved2,
            reserved2 >> 8);

        // Send UBX CFG-PRT (UBX formatted) to configure input NMEA mode
        uart.write(format("%c%c", header >> 8, header));
        uart.write(payload);
        uart.write(_calcUbxChecksum(payload));
        uart.flush();
        imp.sleep(1);

        // Assemble NMEA payload
        local nmeaCmd = format("$PUBX,41,%d,%04d,%04d,%04d,0*", port, inproto, outproto, baudrate);
        // Send UBX CFG-PRT (NMEA formatted) to configure input NMEA mode
        uart.write(nmeaCmd);
        uart.write(format("%02x", GPSParser._calcCheckSum(nmeaCmd)));
        uart.write("\r\n");
        uart.flush();
        imp.sleep(1);
    }

    function _calcUbxChecksum(pkt) {
        local cka=0, ckb=0;
        foreach(a in pkt) {
            cka += a;
            ckb += cka;
        }
        cka = cka&0xff;
        ckb = ckb&0xff;

        return format("%c%c", cka, ckb);
    }

}

// Define constants
const SLEEP_TIME = 15;

// Declare Global Variables
tempSensor <- null;
led <- null;
gps <- LocationMonitor(true);
accel <- null;

// Define functions
function takeReading(){
    
  local conditions = {};
  local reading = tempSensor.read();
  conditions.temp <- reading.temperature;
  conditions.humid <- reading.humidity;
  
  local gpsLoc = gps.getLocation();

  conditions.lat <- gpsLoc.lat;
  conditions.lng <- gpsLoc.lng;
  conditions.isMoving <- gpsLoc.isMoving;
 
  local val = accel.getAccel();
  server.log(format("Acceleration (G): (%0.2f, %0.2f, %0.2f)", val.x, val.y, val.z));
  local abs = math.sqrt((val.x * val.x * 1.0) + 
                (val.y * val.y * 1.0) + 
                (val.z * val.z * 1.0));
  
  conditions.mag <-abs;
 
  // Send 'conditions' to the agent
  agent.send("reading.sent", conditions);

  // Set the imp to sleep when idle, ie. program complete
  imp.wakeup(5.0, takeReading);
}

// Start of program

// Configure I2C bus for sensors
local i2c = hardware.i2cKL;
i2c.configure(CLOCK_SPEED_400_KHZ);

tempSensor = HTS221(i2c);
tempSensor.setMode(HTS221_MODE.ONE_SHOT);

// Configure Accel
accel = LIS3DH(hardware.i2cKL, 0x32);
accel.setDataRate(100);

// Configure LED
hardware.spiYJTHU.configure(SIMPLEX_TX, 7500);
local pixel = APA102(hardware.spiYJTHU, 1);

// Nice start message :) 
server.log("*** Device starting (impC001) ...");
server.log(imp.getsoftwareversion());

// Take a reading
takeReading();
