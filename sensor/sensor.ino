/* Algoritmo que del sensor, esta es el sensor de vibracion*/

/****** -- USO DE LOS PROCESADORES ******/
/****** -- LIBRERIAS -- ******/
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <ADXL345_WE.h>
/*********************** -- DEFINICIONES -- ************************/
// Direccion del accelerometroADXL345
#define ADXL345_I2CADDR 0x53  
/************************* -- SETTINGS -- **************************/
// Creacion del objeto myAcc: "My accelerometro"
ADXL345_WE myAcc = ADXL345_WE(ADXL345_I2CADDR);
/******* -- STRUCTURAS DE DATOS EN COMUNICACION ESP NOW -- *********/
// Estructuras del mensage a enviar
typedef struct struct_message {
  byte mac[6];            // MAC (Own) del emisor
  byte s2write;           // bandera de estados
  int variable;           // que variables vamos a afectar
  int tsample, nsamples;  // tapertuta y numero de muestras
  int pos[2];             // en que posiciones¿afectar?
  float datos[50];        // en si los datos
} struct_message;
// Estructura del mensage a recibir
typedef struct struct_message2 {
  byte request;           // ¿Que necesita apa?
  }struct_message2;

/********************* -- VARIABLES GLOBALES -- ********************/
// Creemos las variables de envio y recepcion de ESP_NOW, 
struct_message data2send;
struct_message2 data2rec;

// MAC Address del Gateway - edit as required
uint8_t broadcastAddress[] = {0x98, 0xCD, 0xAC, 0x49, 0xC4, 0x10};

// Variables de la medidas del sensor
float accx[1750], accy[1750], accz[1750];

// Variables de tiempo
unsigned long timer, start, enlapsed;

// variables para extraer en pruebas
uint8_t mys2write;
byte mac[6];
int meas; // lmeas = 1722,
int count;
int duration=500;
/************ --REVEICED AND SENT CALLBACKS -- ************/

// Peer info
esp_now_peer_info_t peerInfo;

// Callback function called para enviar datos
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback funtion called when data is Received
void OnDataRecv(const uint8_t * mac,const uint8_t *incomingData, int len) {
  memcpy(&data2rec, incomingData, sizeof(data2rec));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("el request que se recive es : ");
  Serial.println(data2rec.request);
}


/************* -- FUNCIONES PARA EL ENVIO DE PAQUETES -- *************/
// Funcion que envia una matriz entera. 
void enviopack(float medida[1750], int lon){
  // Preparemos las variables a enviar con el seccionaminto de los paquetes
  int ini;
  while( ini <= lon ){
    data2send.pos[0] = ini;
    data2send.pos[1] = ini + 50;
    for(int y = data2send.pos[0]; y < data2send.pos[1]; y++){
      data2send.datos[y-ini] = medida[y];
    //  Serial.println(y-ini);
    }
    // Send message via ESP-NOW  -- ahora incluyendo el paquete
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &data2send, sizeof(data2send)); 
    if (result == ESP_OK) {Serial.println("Sending confirmed");
    }else {Serial.println("Sending error");
    }
    delay(10);
    ini=ini+50;
    // Serial.print("el inicializador es ");Serial.println(ini);
  }
}

void setup() {
 
  // Set up Serial Monitor
  Serial.begin(115200);

 // Inicio del bus I2C 
  Wire.begin();
  Wire.setClock(400000);

  // Iniciemos el accelerador
  if(!myAcc.init()){
    Serial.println("ADXL345 not connected!");
  }
  /* Choose the data rate         Hz
    ADXL345_DATA_RATE_3200    3200
    ADXL345_DATA_RATE_1600    1600
    ADXL345_DATA_RATE_800      800
    ADXL345_DATA_RATE_400      400
    ADXL345_DATA_RATE_200      200
    ADXL345_DATA_RATE_100      100
    ADXL345_DATA_RATE_50        50
    ADXL345_DATA_RATE_25        25
    ADXL345_DATA_RATE_12_5      12.5  
    ADXL345_DATA_RATE_6_25       6.25
    ADXL345_DATA_RATE_3_13       3.13
    ADXL345_DATA_RATE_1_56       1.56
    ADXL345_DATA_RATE_0_78       0.78
    ADXL345_DATA_RATE_0_39       0.39
    ADXL345_DATA_RATE_0_20       0.20
    ADXL345_DATA_RATE_0_10       0.10
  */
  myAcc.setDataRate(ADXL345_DATA_RATE_3200);
  // Configuremoslo en Resolución full pa todos los tiros
  myAcc.setFullRes(true);
  /* Choose the measurement range
    ADXL345_RANGE_16G    16g     
    ADXL345_RANGE_8G      8g     
    ADXL345_RANGE_4G      4g   
    ADXL345_RANGE_2G      2g
  */ 
  myAcc.setRange(ADXL345_RANGE_4G);
/* Uncomment to enable Low Power Mode. It saves power but slightly
    increases noise. LowPower only affetcs Data Rates 12.5 Hz to 400 Hz.
*/
  // myAcc.setLowPower(true);

  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Guardemos la mac del sensor
  WiFi.macAddress(data2send.mac);

  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the send callback
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);
  
  //meas = 0x0007;
  
  Serial.println("Aqui estamos desde el sensor ");
}
void loop() {

    if ((data2rec.request & 0x01) == 0x01){
      count = 0;
      Serial.println("iniciando la toma...");
      start=millis();
      data2send.tsample = duration;
      enlapsed= start + duration;
      while ( millis() < enlapsed){
        xyzFloat g = myAcc.getGValues(); 
        accx[count] = g.x;
        accy[count] = g.y;
        accz[count] = g.z;
        count++;
      }
    data2send.nsamples=count;
    meas = 0x0007;
    Serial.println("lista la toma");
    data2rec.request = data2rec.request & 0xFE;
    }
    if ((data2rec.request & 0x02) == 0x02){
      if ((meas & 0x0001) == 0x0001){
        data2send.variable = 0x0001;
        enviopack(accx, count);
        meas = meas & 0xFFFE;
      }
      if ((meas & 0x0002) == 0x0002){
        data2send.variable = 0x0002;
        enviopack(accy, count);
        meas = meas & 0xFFFD;
      }
      if ((meas & 0x0004) == 0x0004){
        data2send.variable = 0x0004;
        enviopack(accz, count);
        meas = meas & 0xFFFB;
      }
    data2rec.request = data2rec.request & 0xFD;
    Serial.print("La variable meas quedo en : "); Serial.println(meas);
    Serial.print("La variable request queda en "); Serial.println(data2rec.request);
    }  
  
  }
/*
void loop() {

  String msg;
  int duration=500;
   if (Serial.available() > 0){
    msg = Serial.readString();
    if (msg == "meas1"){
      count = 0;
      Serial.println("iniciando la toma...");
      start=millis();
      data2send.tsample = duration;
      enlapsed= start + duration;
      while ( millis() < enlapsed){
        xyzFloat g = myAcc.getGValues(); 
        accx[count] = g.x;
        accy[count] = g.y;
        accz[count] = g.z;
        count++;
      }
    data2send.nsamples=count;
    Serial.println("lista la toma");
    }    
    else if (msg == "meas2"){
      if ((meas & 0x0001) == 0x0001){
        data2send.variable = 0x0001;
        enviopack(accx, lmeas);
        meas = meas & 0xFFFE;
      }
      if ((meas & 0x0002) == 0x0002){
        data2send.variable = 0x0002;
        enviopack(accy, lmeas);
        meas = meas & 0xFFFD;
      }
      if ((meas & 0x0004) == 0x0004){
        data2send.variable = 0x0004;
        enviopack(accz, lmeas);
        meas = meas & 0xFFFB;
      }
    Serial.print("La variable meas quedo en : "); Serial.println(meas);
    }
  }
}
*/
