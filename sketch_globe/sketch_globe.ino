#include <Adafruit_DotStar.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// constants
#define RPS 24 // rotations per second
#define RPM (RPS * 60) // 1440 rotations per minute
#define LEDS_PER_ARM 43
#define NUMBER_OF_ARMS 1
#define Y_RES (LEDS_PER_ARM * NUMBER_OF_ARMS) // 43 vertical resolution
#define X_RES (Y_RES * 2) // 86 horizontal resolution
#define RES (X_RES * Y_RES)
#define ARM_AREA_SIZE (X_RES / NUMBER_OF_ARMS) // 86
#define REFRESHES_PER_SECOND (RPS * X_RES) // 2064
#define TIMER_INTERVAL (10000000 / REFRESHES_PER_SECOND) // 4844 * 1/10 microseconds
#define MAX_PACKET_SIZE 512
#define MESSAGE_BUFFER_SIZE (RES * 4)

// macros
#define INIT_GLOBE_ARM(dataPin, clockPin) { dataPin, clockPin, Adafruit_DotStar(Y_RES, dataPin, clockPin, DOTSTAR_BGR) }

typedef struct {
  int dataPin;
  int clockPin;
  Adafruit_DotStar strip;
} GlobeArm_t;

static GlobeArm_t arms[NUMBER_OF_ARMS] = {
  INIT_GLOBE_ARM(23, 18)
};

// blue (0-7), green (8-15), red (16-23)
static uint32_t ledBuffer[X_RES][Y_RES];
static int colCursor = 0;

static SemaphoreHandle_t timerSemaphore;

// wifi network name and password
static const char * networkName = "DLink AP";
static const char * networkPwd = "Something";

// ip address to send UDP data to
static const char * udpAddress = "192.168.0.103";
static const int udpPort = 1234;
static boolean connected = false;

static WiFiUDP udp;

static char messageBuffer[MESSAGE_BUFFER_SIZE]; // buffer to hold incoming message
static int messageBufferCursor = 0;

static char packetBuffer[MAX_PACKET_SIZE]; //buffer to hold incoming packet

inline int min(int a, int b) {
  return ((a) < (b) ? (a) : (b));
}

void initTestBuffer() {
  int area = X_RES / 3;
  for (int rowCursor = 0; rowCursor < Y_RES; ++rowCursor) {
    for (int colCursor = 0; colCursor < X_RES; ++colCursor) {
      if (colCursor < area) {
        ledBuffer[colCursor][rowCursor] = 0xFF0000;
      } else if (colCursor < area * 2) {
        ledBuffer[colCursor][rowCursor] = 0x00FF00;
      } else {
        ledBuffer[colCursor][rowCursor] = 0x0000FF;
      }
    }
  }
}

void refreshLeds() {
  for (int armCursor = 0; armCursor < NUMBER_OF_ARMS; ++armCursor) {
    for (int rowCursor = 0; rowCursor < LEDS_PER_ARM; ++rowCursor) {
      arms[armCursor].strip.setPixelColor(rowCursor, ledBuffer[colCursor + (armCursor * ARM_AREA_SIZE)][(rowCursor + armCursor) * NUMBER_OF_ARMS]);
    }
  }

  for (int armCursor = 0; armCursor < NUMBER_OF_ARMS; ++armCursor) {
    arms[armCursor].strip.show();
  }

  colCursor = ++colCursor % ARM_AREA_SIZE;
}

void IRAM_ATTR onTimer() {
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

void initGlobeArms() {
  for (int armCursor = 0; armCursor < NUMBER_OF_ARMS; ++armCursor) {
    arms[armCursor].strip.begin(); // initialize pins for output
    arms[armCursor].strip.show(); // turn all LEDs off
  }
}

void initTimer() {
  // create semaphore to inform us when the timer has been triggered
  timerSemaphore = xSemaphoreCreateBinary();

  // use 1st timer of 4 (counted from zero)
  // set 80 divider for prescaler (see ESP32 Technical Reference Manual for more info)
  // 80 MHz basic clock divided by 8 -> 10 MHz timer clock (100 ns per tick)
  hw_timer_t* timer = timerBegin(0, 8, true);

  // attach onTimer function to timer
  timerAttachInterrupt(timer, &onTimer, true);

  // set alarm to call onTimer function after TIMER_INTERVAL elapsed
  timerAlarmWrite(timer, TIMER_INTERVAL, true);

  // start alarm
  timerAlarmEnable(timer);
}

void initWifi(const char * ssid, const char * pwd) {
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);

  //register event handler
  WiFi.onEvent(wifiEventHandler);

  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

void wifiEventHandler(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      // when connected set
      Serial.print("WiFi connected! IP address: ");
      Serial.println(WiFi.localIP());
      // initializes the UDP state
      // this initializes the transfer buffer
      udp.begin(WiFi.localIP(), udpPort);
      connected = true;
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      connected = false;
      break;
  }
}

void setup() {
  Serial.begin(115200);

  // initialize buffer with test image
  initTestBuffer();

  initGlobeArms();
  initTimer();
  initWifi(networkName, networkPwd);
}

void loop() {
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {
    refreshLeds();
  }

  if (connected) {
    int packetSize = 0;
    if (packetSize = udp.parsePacket()) {
      int len = udp.read(packetBuffer, min(MAX_PACKET_SIZE, packetSize));
      if (len > 0) {
        //        Serial.println("Receiving Packet...");
        //        Serial.println(len);
        char* packetBufferStart = packetBuffer;
        if ((len > 4) && packetBuffer[4] == '[') {
          // new message
          int messageSize = *((int*) packetBuffer);
          packetBufferStart += 5;
          len -= 5;
          //          Serial.println("New Message:");
          //          Serial.println(messageSize);
          memcpy(messageBuffer, packetBufferStart, len);
          messageBufferCursor = len;
        } else if (messageBufferCursor > 0) {
          len = min(len, MESSAGE_BUFFER_SIZE - messageBufferCursor);
          memcpy(messageBuffer + messageBufferCursor, packetBufferStart, len);
          messageBufferCursor += len;
          //        Serial.println("Message Cursor:");
          Serial.println(messageBufferCursor);
          Serial.println("Length:");
          Serial.println(len);
          Serial.println(*(packetBufferStart + len) );
          if (*(packetBufferStart + len) == ']') {
            // found message end mark
            //memcpy(ledBuffer, messageBuffer, messageBufferCursor);
            Serial.println("End of Message:");
            Serial.println(messageBufferCursor);
            messageBufferCursor = 0;
          }
        }
      }
    }
  }
}
