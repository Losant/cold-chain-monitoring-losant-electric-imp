// Import libraries
#require "JSONEncoder.class.nut:2.0.0"
#require "Losant.agent.lib.nut:1.0.0"

// ID and API Token for My Losant Application
const LOSANT_APPLICATION_ID   = "APPLICATION_ID";
const LOSANT_DEVICE_API_TOKEN = "DEVICE_API_TOKEN";

// Device ID Global
lsntDeviceId <- "DEVICE ID";

// Init Losant
losant <- Losant(LOSANT_APPLICATION_ID, LOSANT_DEVICE_API_TOKEN);

device.on("reading.sent", function(data) {
  server.log(http.jsonencode(data));

  // Generate Payload
  local payload = {
    "time" : losant.createIsoTimeStamp(),
    "data" : {}
  };

  // Make sure all the data is there. 
  if ("lat" in data && "lng" in data) payload.data.location <- format("%s,%s", data.lat, data.lng);
  if ("temp" in data) payload.data.temperature <- data.temp;
  if ("humid" in data) payload.data.humidity <- data.humid;
  if ("mag" in data) payload.data.magnitude <- data.mag;
  if ("isMoving" in data) payload.data.isMoving <- data.isMoving;

  // Send Device State to Losant
  losant.sendDeviceState(lsntDeviceId, payload, function(res) {
    server.log(res.statuscode);
    server.log(res.body);
  });
});