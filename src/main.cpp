#include <Arduino.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoHttpClient.h>
#include <Arduino_JSON.h>

const char *ssid = "KARAN";
const char *password = "12345679";

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
char daysOfTheWeek[7][12] = {"Ahad", "Senin", "Selasa", "Rabu", "Kamis", "Jumat", "Sabtu"};

String jam, menit, detik, hari;

// ini script Karan
char serverAddress[] = "20.20.20.25";
int serverPort = 3001;

WiFiClient wifi;
// HttpClient client = HttpClient(wifi, serverAddress, serverPort);
HttpClient client = HttpClient(wifi, serverAddress, serverPort);

// Untuk Angin Gunakan pin D6 pada Arduino
//================================================
volatile byte rpmcount; // count signals
volatile unsigned long last_micros;
unsigned long timeold;
unsigned long timemeasure = 25.00; // seconds
int timetoSleep = 1;               // minutes
unsigned long sleepTime = 15;      // minutes
unsigned long timeNow;
int GPIO_pulse = D6; //
float rpm, rps;      // frequencies
float radius = 0.1;  // meters - measure of the lenght of each the anemometer wing
float velocity_kmh;  // km/h
float velocity_ms;   // m/s
float omega = 0;     // rad/s
float calibration_value = 2.0;
//================================================

// Untuk Curah Hujan Gunakan pin D5 pada Arduino
//================================================
const int pin_interrupt = D5; // Arduino = D3
long int jumlah_tip = 0;
long int temp_jumlah_tip = 0;
float curah_hujan = 0.00;
float curah_hujan_per_menit = 0.00;
float curah_hujan_per_jam = 0.00;
float curah_hujan_per_hari = 0.00;
float curah_hujan_hari_ini = 0.00;
float temp_curah_hujan_per_menit = 0.00;
float temp_curah_hujan_per_jam = 0.00;
float temp_curah_hujan_per_hari = 0.00;
float milimeter_per_tip = 0.70;
String cuaca = "Berawan";
volatile boolean flag = false;
//================================================

void IRAM_ATTR hitung_curah_hujan()
{
  flag = true;
}

void IRAM_ATTR rpm_anemometer()
{
  if (long(micros() - last_micros) >= 5000)
  { // time to debounce measures
    rpmcount++;
    last_micros = micros();
  }
}

void setup()
{
  Serial.begin(9600);
  //====================== SETUP Curah Hujan ================================================
  pinMode(pin_interrupt, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_interrupt), hitung_curah_hujan, FALLING); // Akan menghitung tip jika pin berlogika dari HIGH ke LOW
  //====================== END SETUP Curah Hujan ================================================

  //====================== SETUP Angin ================================================
  pinMode(GPIO_pulse, INPUT_PULLUP);
  digitalWrite(GPIO_pulse, LOW);

  detachInterrupt(digitalPinToInterrupt(GPIO_pulse));                         // force to initiate Interrupt on zero
  attachInterrupt(digitalPinToInterrupt(GPIO_pulse), rpm_anemometer, RISING); // Initialize the intterrupt pin
  rpmcount = 0;
  rpm = 0;
  timeold = 0;
  timeNow = 0;
  //====================== SETUP Angin ================================================

  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Jaringan Terhubung");

  // Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  // bacaWaktu();
  // printSerial_angin();
  // printSerial_curah_hujan();
}

String konversi_jam(String angka) // Fungsi untuk supaya jika angka satuan ditambah 0 di depannya, Misalkan jam 1 maka jadi 01 pada LCD
{
  if (angka.length() == 1)
  {
    angka = "0" + angka;
  }
  else
  {
    angka = angka;
  }
  return angka;
}

void bacaWaktu()
{
  timeClient.begin();
  timeClient.setTimeOffset(28800);
  timeClient.update();
  hari = String(daysOfTheWeek[timeClient.getDay()]);
  jam = String(timeClient.getHours(), DEC);
  menit = String(timeClient.getMinutes(), DEC);
  detik = String(timeClient.getSeconds(), DEC);
}

void printSerial_angin()
{
  Serial.print("RPS=");
  Serial.println(rps);
  Serial.print("RPM=");
  Serial.println(rpm);
  Serial.print("Kecepatan=");
  Serial.print(velocity_ms);
  Serial.print(" m/s, ");
  Serial.print(velocity_kmh);
  Serial.println(" Km/jam");
  String angin_kmh = String(velocity_kmh, 2);
  // Firebase.setString(fbdo, "/angin_kmh",angin_kmh);
  client.post("/api/from_esp32", "application/json", "{\"kecepatan_per_detik\":\"" + String(velocity_ms) + "\",\"kecepatan_per_jam\":\"" + String(velocity_kmh) + "\",\"rps\":\"" + String(rps) + "\",\"rpm\":\"" + String(rpm) + "\"}");
  String response = client.responseBody();
  JSONVar myObject = JSON.parse(response);
  if (JSON.typeof(myObject) == "undefined")
  {
    Serial.println("Parsing input failed!");
    return;
  }
  else
  {
    Serial.println("Parsing input success!");
    Serial.println(myObject);
  }
}

void printSerial_curah_hujan()
{
  Serial.println("Waktu=" + konversi_jam(jam) + ":" + konversi_jam(menit));
  Serial.print("Cuaca=");
  Serial.println(cuaca); // Print cuaca hari ini (Ini bukan ramalan cuaca tapi membaca cuaca yang sudah terjadi/ sedang terjadi hari ini)
  Serial.print("Jumlah tip=");
  Serial.print(jumlah_tip);
  Serial.println(" kali ");
  Serial.print("Curah hujan hari ini=");
  Serial.print(curah_hujan_hari_ini, 1);
  Serial.println(" mm ");
  Serial.print("Curah hujan per menit=");
  Serial.print(curah_hujan_per_menit, 1);
  Serial.println(" mm ");
  Serial.print("Curah hujan per jam=");
  Serial.print(curah_hujan_per_jam, 1);
  Serial.println(" mm ");
  Serial.print("Curah hujan per hari=");
  Serial.print(curah_hujan_per_hari, 1);
  Serial.println(" mm ");

  String waktu = hari + ", " + konversi_jam(jam) + ":" + konversi_jam(menit);
  // Firebase.setString(fbdo, "/waktu",waktu);
  // Firebase.setFloat(fbdo, "/curah_hujan",curah_hujan_hari_ini);
  client.post("/api/from_esp32/curah_hujan", "application/json", "{\"jumlah_tip\":\"" + String(jumlah_tip) + "\",\"curah_hujan_per_menit\":\"" + String(curah_hujan_per_menit) + "\",\"curah_hujan_per_jam\":\"" + String(curah_hujan_per_jam) + "\",\"curah_hujan_per_hari\":\"" + String(curah_hujan_per_hari) + "\"}");
  String response = client.responseBody();
  JSONVar myObject = JSON.parse(response);
  if (JSON.typeof(myObject) == "undefined")
  {
    Serial.println("Parsing input failed!");
    return;
  }
  else
  {
    Serial.println("Parsing input success!");
    Serial.println(myObject);
  }
}

void checkWifi()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi Disconnected");
    delay(1000);
    ESP.restart();
  }
}

void loop()
{
  // checkwifi
  checkWifi();


  // Measure RPM
  if ((millis() - timeold) >= timemeasure * 1000)
  {
    Serial.println(rpmcount);
    detachInterrupt(digitalPinToInterrupt(GPIO_pulse)); // Disable interrupt when calculating
    rps = float(rpmcount) / float(timemeasure);         // rotations per second
    rpm = 60 * rps;                                     // rotations per minute
    omega = 2 * PI * rps;                               // rad/s
    velocity_ms = omega * radius * calibration_value;   // m/s
    velocity_kmh = velocity_ms * 3.6;                   // km/h
    printSerial_angin();                                // print serial 25 detik sekali
    timeold = millis();
    rpmcount = 0;
    attachInterrupt(digitalPinToInterrupt(GPIO_pulse), rpm_anemometer, RISING); // enable interrupt
  }

  if (flag == true)
  {
    curah_hujan += milimeter_per_tip; // Akan bertambah nilainya saat tip penuh
    jumlah_tip++;
    delay(500);
    flag = false; // reset flag
  }

  bacaWaktu();
  curah_hujan_hari_ini = jumlah_tip * milimeter_per_tip;
  temp_curah_hujan_per_menit = curah_hujan;

  // Probabilistik Curah Hujan https://www.bmkg.go.id/cuaca/probabilistik-curah-hujan.bmkg
  if (curah_hujan_hari_ini <= 0.00 && curah_hujan_hari_ini <= 0.50)
  {
    cuaca = "Berawan";
  }
  if (curah_hujan_hari_ini > 0.50 && curah_hujan_hari_ini <= 20.00)
  {
    cuaca = "Hujan Ringan";
  }
  if (curah_hujan_hari_ini > 20.00 && curah_hujan_hari_ini <= 50.00)
  {
    cuaca = "Hujan Sedang";
  }
  if (curah_hujan_hari_ini > 50.00 && curah_hujan_hari_ini <= 100.00)
  {
    cuaca = "Hujan Lebat";
  }
  if (curah_hujan_hari_ini > 100.00 && curah_hujan_hari_ini <= 150.00)
  {
    cuaca = "Hujan Sangat Lebat";
  }
  if (curah_hujan_hari_ini > 150.00)
  {
    cuaca = "Hujan ekstrem";
  }
  if (detik.equals("0")) // Hanya print pada detik 0
  {
    curah_hujan_per_menit = temp_curah_hujan_per_menit; // Curah hujan per menit dihitung ketika detik 0
    temp_curah_hujan_per_jam += curah_hujan_per_menit;  // Curah hujan per jam dihitung dari penjumlahan curah hujan per menit namun disimpan dulu dalam variabel temp
    if (menit.equals("0"))
    {
      curah_hujan_per_jam = temp_curah_hujan_per_jam;   // Curah hujan per jam baru dihitung ketika menit 0
      temp_curah_hujan_per_hari += curah_hujan_per_jam; //// Curah hujan per hari dihitung dari penjumlahan curah hujan per jam namun disimpan dulu dalam variabel temp
      temp_curah_hujan_per_jam = 0.00;                  // Reset temp curah hujan per jam
    }
    // if (menit.equals("0") && jam.equals("0"))
    if (menit.equals("0"))
    {
      curah_hujan_per_hari = temp_curah_hujan_per_hari; // Curah hujan per jam baru dihitung ketika menit 0 dan jam 0 (Tengah malam)
      temp_curah_hujan_per_hari = 0.00;                 // Reset temp curah hujan per hari
      curah_hujan_hari_ini = 0.00;                      // Reset curah hujan hari ini
      jumlah_tip = 0;                                   // Jumlah tip di reset setiap 24 jam sekali (Tengah malam)
    }
    temp_curah_hujan_per_menit = 0.00;
    curah_hujan = 0.00;
    delay(1000);
  }
  if ((jumlah_tip != temp_jumlah_tip) || (detik.equals("0"))) // Print serial setiap 1 menit atau ketika jumlah_tip berubah
  {
    printSerial_curah_hujan();
    Serial.println(" ");
  }
  temp_jumlah_tip = jumlah_tip;
}
