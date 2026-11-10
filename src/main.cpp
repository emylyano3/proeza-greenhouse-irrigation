#include <ESP8266WiFi.h>
#include <ESPDomotic.h>
#include <time.h>
#include <string>
#include <sstream>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

/*
HEAP Improvement https://learn.adafruit.com/memories-of-an-arduino/optimizing-sram
Avoid String https://hackingmajenkoblog.wordpress.com/2016/02/04/the-evils-of-arduino-strings/
*/

void setup();
void loop();
void initializeScheduling();
bool saveCronConf(uint8_t cronNo);
void loadCronConf(uint8_t cronNo);
void updateCronField(uint8_t cronNo, uint8_t index, const char* value);
void checkIrrigation();
bool isTimeToCheckSchedule ();
bool isTimeToIrrigate ();
void receiveMqttMessage(char* topic, uint8_t* payload, unsigned int length);
void updateCronCommand(char* topic, unsigned char* payload, unsigned int length);
bool changeStateCommand(char* topic, unsigned char* payload, unsigned int length);
void publishCronField(uint8_t cronNo, uint8_t fieldNo);
void publishCronState();
bool isValidCronValue(uint8_t fieldNo, const char* value);

void initSensor();
void publishSensorData();
bool isTimeToReadSensor();
void publishValue(const char* name, float value, Channel *channel);

/* Constants */
const char* DAYS_OF_WEEK[]    = {"SUN","MON","TUE","WED","THU","FRI","SAT"};
const char* MONTHS_OF_YEAR[]  = {"JAN","FEB","MAR","APR","MAY","JUN","JUL","AUG","SEP","OCT","NOV","DIC"};

typedef char* tCron[6];

/* Crons management */
const char*   CRONS_CONF_FILES[]      = {"CRON_0.conf", "CRON_1.conf", "CRON_2.conf"};
const size_t  CRON_FIELD_STRING_SIZE  = CRON_FIELD_SIZE + 1;
const size_t  CRON_FIELDS             = 6;
const char*   CRON_FIELDS_NAMES[]     = {"SEC", "MIN", "HOU", "DOM", "MON", "DOW"};

const char* MODULE_TYPE               = "irrigation";

/* Irrigation control */
tCron           _irrCronExpressions[MAX_CRONS] = {
  {new char[CRON_FIELD_STRING_SIZE], new char[CRON_FIELD_STRING_SIZE], new char[CRON_FIELD_STRING_SIZE], new char[CRON_FIELD_STRING_SIZE], new char[CRON_FIELD_STRING_SIZE], new char[CRON_FIELD_STRING_SIZE]}, // 3 + 1 (max field length + '\0')
  {new char[CRON_FIELD_STRING_SIZE], new char[CRON_FIELD_STRING_SIZE], new char[CRON_FIELD_STRING_SIZE], new char[CRON_FIELD_STRING_SIZE], new char[CRON_FIELD_STRING_SIZE], new char[CRON_FIELD_STRING_SIZE]},
  {new char[CRON_FIELD_STRING_SIZE], new char[CRON_FIELD_STRING_SIZE], new char[CRON_FIELD_STRING_SIZE], new char[CRON_FIELD_STRING_SIZE], new char[CRON_FIELD_STRING_SIZE], new char[CRON_FIELD_STRING_SIZE]}
};
bool            _cronsLoaded          = false; // true once crons were read from flash (before that, expressions hold garbage)
bool            _irrigating           = false;
uint8_t         _currChannel          = 0;
unsigned long   _irrLastScheduleCheck = -TIMER_CHECK_THRESHOLD_SECONDS * 1000;

/* Channels control */
#ifdef NODEMCUV2
const uint8_t CHANNEL_A_PIN = D1;
const uint8_t CHANNEL_B_PIN = D2;
const uint8_t CHANNEL_C_PIN = D4;
const uint8_t CHANNEL_D_PIN = D5;
const uint8_t SENSOR_PIN    = D6;
const uint8_t LED_PIN       = D7;
// TODO Define pin consts in configuration file (ini file)
#elif ESP12
const uint8_t CHANNEL_A_PIN = 2;
const uint8_t CHANNEL_B_PIN = 4;
const uint8_t CHANNEL_C_PIN = 5;
const uint8_t CHANNEL_D_PIN = 16;
const uint8_t SENSOR_PIN    = 14;
const uint8_t LED_PIN       = 13;
#endif

  // Id, Name, pin, state, timer 
Channel       _channelA ("A", "channel_A", CHANNEL_A_PIN, OUTPUT, HIGH, CHANNEL_DEFAULT_TIMER_SECONDS * 1000);
Channel       _channelB ("B", "channel_B", CHANNEL_B_PIN, OUTPUT, HIGH, CHANNEL_DEFAULT_TIMER_SECONDS * 1000);
Channel       _channelC ("C", "channel_C", CHANNEL_C_PIN, OUTPUT, HIGH, CHANNEL_DEFAULT_TIMER_SECONDS * 1000);
Channel       _channelD ("D", "channel_D", CHANNEL_D_PIN, OUTPUT, HIGH, CHANNEL_DEFAULT_TIMER_SECONDS * 1000);
Channel       _sensorChannel ("dht22", "sensor", SENSOR_PIN, INPUT, LOW); 
DHT_Unified   _temperatureSensor(SENSOR_PIN, DHT22);

ESPDomotic _domoticModule;
uint32_t   _sensorReadThreshold;
uint32_t   _lastSensorRead;

template <typename T> void debug(T t) {
  Serial.println(t);
}

 /* recursive variadic function */
template<typename T, typename... Args> void debug(T t, Args... args) {
  Serial.print(t);
  Serial.print(" ");
  debug(args...) ;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  debug(F("Starting module"));
  std::ostringstream s;
  s << "Proeza irrigation " << ESP.getChipId();
  std::string chipname(s.str());
  _domoticModule.setPortalSSID(chipname.c_str());
  _domoticModule.setFeedbackPin(LED_PIN);
  _domoticModule.setMqttMessageCallback(receiveMqttMessage);
  // Every (re)connection to the broker republishes the cron state, so the home automation always knows the real config
  _domoticModule.setMqttConnectionCallback(publishCronState);
  _domoticModule.setConfigPortalTimeout(CONFIG_PORTAL_TIMEOUT);
  _domoticModule.setWifiConnectTimeout(WIFI_CONNECT_TIMEOUT);
  _domoticModule.setConfigFileSize(CONFIG_FILE_SIZE);
  _domoticModule.setModuleType(MODULE_TYPE);
  _domoticModule.addChannel(&_channelA);
  _domoticModule.addChannel(&_channelB);
  _domoticModule.addChannel(&_channelC);
  _domoticModule.addChannel(&_channelD);
  _domoticModule.init();
  debug(F("Connected to wifi network. Local IP"), WiFi.localIP());
  configTime(TIMEZONE * 3600, 0, "ar.pool.ntp.org", "br.pool.ntp.org", "cl.pool.ntp.org"); // argentina, brazil, chile
  debug(F("Waiting for time..."));
  time_t now;
  while (!(now = time(nullptr))) {
    delay(500);
  }
  Serial.printf("Current time: %s", ctime(&now));
  initializeScheduling();
  initSensor();
}

void initializeScheduling () {
  debug(F("Initializing scheduling"));
  for (uint8_t cronNo = 0; cronNo < MAX_CRONS; ++cronNo) {
    loadCronConf(cronNo);
  }
  _cronsLoaded = true;
  // The broker connection may have been established before the crons were loaded (callback skipped it), so cover that case
  if (_domoticModule.getMqttClient()->connected()) {
    publishCronState();
  }
}

void initSensor() {
  _temperatureSensor.begin();
  sensor_t sensor;
  _temperatureSensor.temperature().getSensor(&sensor);
  _sensorReadThreshold = max(sensor.min_delay / 1000, SENSE_THRESHOLD_SECONDS * 1000);
  debug("Sensor read delay:", _sensorReadThreshold);
}

void loop() {
  _domoticModule.loop();
  checkIrrigation();
  publishSensorData();
  delay(LOOP_DELAY);
}

void publishSensorData() {
  if (isTimeToReadSensor()) {
    _lastSensorRead = millis();
    sensors_event_t event;
    _temperatureSensor.temperature().getEvent(&event);
    publishValue("temperature", event.temperature, &_sensorChannel);
    debug("Temperature: ", event.temperature);
    _temperatureSensor.humidity().getEvent(&event);
    publishValue("humidity", event.relative_humidity, &_sensorChannel);
    debug("Humidity: ", event.relative_humidity);
  }
}

bool isTimeToReadSensor() {
  return _lastSensorRead + _sensorReadThreshold < millis();
}

void publishValue(const char* name, float value, Channel *channel) {
    if (!isnan(value)) {
        _domoticModule.getMqttClient()->publish(
            _domoticModule.getChannelTopic(channel, name).c_str(), 
            String(value).c_str());
    }
}

void checkIrrigation() {
  if (!_irrigating) {
    if (isTimeToCheckSchedule()) {
      if (isTimeToIrrigate()) {
        debug(F("Irrigation sequence started"));
        _irrigating = true;
        _currChannel = 0;
        _domoticModule.getMqttClient()->publish(_domoticModule.getStationTopic("feedback/state").c_str(), "1");
      }
    }
  } else {
    if (_currChannel >= _domoticModule.getChannelsCount()) {
      debug(F("No more channels to process. Stoping irrigation sequence."));
      _irrigating = false;
      _domoticModule.getMqttClient()->publish(_domoticModule.getStationTopic("feedback/state").c_str(), "0");
    } else {
      Channel *channel = _domoticModule.getChannel(_currChannel);
      if (!channel->isEnabled()) {
        debug(F("Channel is disabled, going to next one."), channel->name);
        ++_currChannel;
      } else {
        if (channel->state == HIGH) {
          debug(F("Openning channel"), channel->name);
          _domoticModule.updateChannelState(channel, LOW);
        } else {
           if (channel->timeIsUp()) {
            debug(F("Closing channel"), channel->name);
            _domoticModule.updateChannelState(channel, HIGH);
            _currChannel++;
            delay(CLOSE_VALVE_DELAY);
          }
        }
      }
    }
  }
}

bool isTimeToCheckSchedule () {
  bool isTime = _irrLastScheduleCheck + TIMER_CHECK_THRESHOLD_SECONDS * 1000 < millis();
  _irrLastScheduleCheck = isTime ? millis() : _irrLastScheduleCheck;
  return isTime;
}

bool isTimeToIrrigate () {
  debug(F("Checking if it is time to start irrigation"));
  time_t now = time(nullptr);
  Serial.printf("Current time: %s", ctime(&now));
  struct tm * ptm = localtime(&now);
  const char* mon = ptm->tm_mon < 12 ? MONTHS_OF_YEAR[ptm->tm_mon] : "";
  const char* dow = ptm->tm_wday < 7 ? DAYS_OF_WEEK[ptm->tm_wday] : "";
  // Evaluates cron at minute level. Seconds granularity is not needed for irrigarion scheduling.
  boolean tti;
  uint8_t cronNo = 0;
  do {
    tti = true;
    tti &= _irrCronExpressions[cronNo][1][0] == '*' 
                      || atoi(_irrCronExpressions[cronNo][1]) == ptm->tm_min 
                      || (  
                          ptm->tm_min > atoi(_irrCronExpressions[cronNo][1]) 
                          && 
                          ptm->tm_min - (TIMER_CHECK_THRESHOLD_SECONDS / 60) <= atoi(_irrCronExpressions[cronNo][1])
                        ); // Minute
    tti &= _irrCronExpressions[cronNo][2][0] == '*' 
                      || atoi(_irrCronExpressions[cronNo][2]) == ptm->tm_hour; // Hour
    tti &= _irrCronExpressions[cronNo][3][0] == '*' 
                      || atoi(_irrCronExpressions[cronNo][3]) == ptm->tm_mday; // Day of month
    tti &= _irrCronExpressions[cronNo][4][0] == '*' 
                      || strncasecmp(mon, _irrCronExpressions[cronNo][4], CRON_FIELD_SIZE) == 0 
                      || atoi(_irrCronExpressions[cronNo][4]) == ptm->tm_mon + 1; // Month
    tti &= _irrCronExpressions[cronNo][5][0] == '?'
                      || strncasecmp(dow, _irrCronExpressions[cronNo][5], CRON_FIELD_SIZE) == 0
                      || atoi(_irrCronExpressions[cronNo][5]) == ptm->tm_wday + 1; // Day of week
    debug(F("Checking cron"), cronNo);
    ++cronNo;
  } while (!tti && cronNo < MAX_CRONS);
  debug(F("IS TTI?"), tti ? "YES" : "NO");
  return tti;
}

void receiveMqttMessage(char* topic, unsigned char* payload, unsigned int length) {
  debug(F("MQTT message on topic"), topic);
  // Station topic
  std::string receivedTopic = std::string(topic);
  if (receivedTopic.find(_domoticModule.getStationTopic("command/cron/").c_str()) != std::string::npos) {
    updateCronCommand(topic, payload, length);
  } else if (receivedTopic.find(_domoticModule.getStationTopic("command/state").c_str()) != std::string::npos) {
    changeStateCommand(topic, payload, length);
    _domoticModule.getMqttClient()->publish(_domoticModule.getStationTopic("feedback/state").c_str(), _irrigating ? "1" : "0");
  } else {
    debug(F("Not a station topic"));
  }
}

// Publishes (retained) the current value of one cron field. Retained so any subscriber, even one that connects later or
// restarts, gets the real config immediately instead of waiting for someone to send a command.
void publishCronField(uint8_t cronNo, uint8_t fieldNo) {
  String topic = _domoticModule.getStationTopic("feedback/cron/") + String(cronNo) + "/" + String(fieldNo);
  _domoticModule.getMqttClient()->publish(topic.c_str(), _irrCronExpressions[cronNo][fieldNo], true);
}

void publishCronState() {
  if (!_cronsLoaded) {
    return;
  }
  for (uint8_t cronNo = 0; cronNo < MAX_CRONS; ++cronNo) {
    for (uint8_t fieldNo = 0; fieldNo < CRON_FIELDS; ++fieldNo) {
      publishCronField(cronNo, fieldNo);
    }
  }
}

// Minutes and hours must be '*' or a number in range. Other fields accept names (JAN, MON...) or '?', so only length is checked.
bool isValidCronValue(uint8_t fieldNo, const char* value) {
  size_t len = strlen(value);
  if (len == 0 || len > CRON_FIELD_SIZE) {
    return false;
  }
  if (fieldNo != 1 && fieldNo != 2) {
    return true;
  }
  if (len == 1 && value[0] == '*') {
    return true;
  }
  for (size_t i = 0; i < len; ++i) {
    if (value[i] < '0' || value[i] > '9') {
      return false;
    }
  }
  int number = atoi(value);
  return number <= (fieldNo == 1 ? 59 : 23);
}

void updateCronCommand (char* topic, unsigned char* payload, unsigned int length) {
  // Expected topic tail: <cronNo>/<fieldNo>
  String prefix = _domoticModule.getStationTopic("command/cron/");
  const char* tail = topic + prefix.length();
  char* end;
  long cronID = strtol(tail, &end, 10);
  if (end == tail || *end != '/') {
    debug(F("Invalid cron topic"), topic);
    return;
  }
  const char* fieldStart = end + 1;
  long fieldID = strtol(fieldStart, &end, 10);
  if (end == fieldStart || *end != '\0' || cronID < 0 || cronID >= MAX_CRONS || fieldID < 0 || fieldID >= (long) CRON_FIELDS) {
    debug(F("Cron or field out of range"), topic);
    return;
  }
  char value[CRON_FIELD_STRING_SIZE] = {'\0'};
  if (length > 0 && length <= CRON_FIELD_SIZE) {
    memcpy(value, payload, length); // payload is not guaranteed to be null terminated
  }
  if (isValidCronValue(fieldID, value)) {
    updateCronField(cronID, fieldID, value);
    saveCronConf(cronID);
  } else {
    debug(F("Invalid cron value rejected for"), topic);
  }
  // Always answer with the real value: confirms an accepted change and makes subscribers revert a rejected one
  publishCronField(cronID, fieldID);
}

void loadCronConf(uint8_t cronNo) {
  debug(F("Getting configuration for cron number"), cronNo);
  char* conf = _domoticModule.getConf(CRONS_CONF_FILES[cronNo]);
  if (conf) {
    debug(F("Cron conf loaded"), conf);
    size_t i = 0, chunkNo = 0, length = strlen(conf);
    while (i < length && chunkNo < CRON_FIELDS) {
      if (conf[i] == ' ') {
        ++i;
      } else {
        char value[CRON_FIELD_STRING_SIZE] = {'\0'}; 
        int j = 0;
        while (i < length && conf[i] != ' ' && j < CRON_FIELD_SIZE) 
          value[j++] = conf[i++];
        updateCronField(cronNo, chunkNo++, value);
      }
    }
    delete[] conf;
  } else { 
    debug(F("Conf not found for cron number"), cronNo, F(". Setting defaults"));
    updateCronField(cronNo, 0, "0");
    updateCronField(cronNo, 1, "0");
    updateCronField(cronNo, 2, "6");
    updateCronField(cronNo, 3, "*");
    updateCronField(cronNo, 4, "*");
    updateCronField(cronNo, 5, "?");
  }
}

void updateCronField(uint8_t cronNo, uint8_t fieldNo, const char* value) {
  debug(F("Updating cron ["), cronNo, F("] field ["), CRON_FIELDS_NAMES[fieldNo], F("] with value:"), value);
  strncpy(_irrCronExpressions[cronNo][fieldNo], value, CRON_FIELD_SIZE);
  _irrCronExpressions[cronNo][fieldNo][CRON_FIELD_SIZE] = '\0';
}

bool saveCronConf (uint8_t cronNo) {
  char toSave[CRON_FIELDS * CRON_FIELD_SIZE + CRON_FIELDS] = {'\0'}; //fields size + spaces + end char
  size_t cursor = 0;
  for (size_t fieldNo = 0; fieldNo < CRON_FIELDS; ++fieldNo) {
    size_t i = 0;
    while (i < CRON_FIELD_SIZE && _irrCronExpressions[cronNo][fieldNo][i] != '\0') {
      toSave[cursor++] = _irrCronExpressions[cronNo][fieldNo][i++];
    }
    if (fieldNo != CRON_FIELDS - 1) 
      toSave[cursor++]= ' '; //no space after last field
  }
  debug(F("Saving cron"), cronNo, F(" conf: "), toSave);
  return _domoticModule.updateConf(CRONS_CONF_FILES[cronNo], toSave);
}

bool changeStateCommand(char* topic, unsigned char* payload, unsigned int length) {
  if (length != 1) {
    debug(F("Invalid payload"));
  } else {
    switch (payload[0]) {
      case '0':
        if (_irrigating) {
          // Set irrigation end to 0 to simulate it should have ended 
          Channel *channel = _domoticModule.getChannel(_currChannel);
          _domoticModule.updateChannelState(channel, HIGH);
          _irrigating = false;
          debug(F("Irrigation stopped"));
        }
        return true;
      case '1':
        if (!_irrigating) {
          _irrigating = true;
          _currChannel = 0;
          debug(F("Irrigation started"));
        }
        return true;
      default:
        debug(F("Invalid state"), payload[0]);
    }
  }
  return false;
}
