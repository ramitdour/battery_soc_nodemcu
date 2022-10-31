#include <BatterySoc.h>

#ifdef INA219_SENSOR
Adafruit_INA219 ina219;
#endif

#ifdef ADS1115_SENSOR
Adafruit_ADS1115 ads1115;
#endif

// Ticker::Ticker(fptr callback, uint32_t timer, uint16_t repeats, interval_t mode)
Ticker ticker_update_lcd(update_lcd_display, 850, 0, MILLIS); // the interval time is 100us and the internal resolution is micro seconds

void writeStringToEEPROM(int addrOffset,  String strToWrite)
{
  byte len = strToWrite.length();

  Serial.print(strToWrite);
  Serial.print("\n");
  
  EEPROM.write(addrOffset, len);

  for (int i = 0; i < len; i++)
  {
    EEPROM.write(addrOffset + 1 + i, strToWrite[i]);
    Serial.print(strToWrite[i]);
  }
  Serial.print("\n");
  Serial.print("<done write\n");
  EEPROM.commit() ? Serial.println("ok") :Serial.println("not ok");
}

String readStringFromEEPROM(int addrOffset)
{
  Serial.print("Stratred reading\n");

  int newStrLen = EEPROM.read(addrOffset);
  Serial.println(newStrLen);

  char data[newStrLen + 1];

  for (int i = 0; i < newStrLen; i++)
  {
    data[i] = EEPROM.read(addrOffset + 1 + i);
    Serial.print(data[i]);
  }
  data[newStrLen] = '\0'; // !!! NOTE !!! Remove the space between the slash "/" and "0" (I've added a space because otherwise there is a display bug)

  Serial.print("\n");
  Serial.print(" <done read\n");
  return String(data);
}

void tick_update_EEPROM_data()
{
  writeStringToEEPROM(0, String(battery_capacity_coulomb, 2));
  Serial.println("tick EEPROM");
}

Ticker ticker_update_EEPROM_data(tick_update_EEPROM_data, 10000, 0, MILLIS); // the interval time is 100us and the internal resolution is micro seconds

#ifdef LCD_16X2_I2C

LiquidCrystal_I2C lcd(I2C_ADD_LCD_16X2, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

// void lcd_set_msg(String message, uint8_t lcd_row = 0, uint8_t lcd_col = 0, bool clear_screen = true, bool align_centre = true)
void lcd_set_msg(String message, uint8_t lcd_row, uint8_t lcd_col, bool clear_screen, bool align_centre)
{
  if (message.length() > 16)
  {
    message = message.substring(0, 15);
  }

  if (clear_screen)
  {
    // clear the screen
    lcd.clear();
  }

  if (align_centre)
  {
    lcd_col = 8 - uint8_t((message.length() + 1) / 2);
  }

  lcd.setCursor(lcd_col, lcd_row);
  lcd.print(message.c_str());

#ifdef DEBUG_CODE

  // Serial.print(lcd_col);
  // Serial.println(lcd_row);
  Serial.println(message);

#endif
}

void setup_lcd_i2c()
{
  lcd.init(); // initialize the lcd
  lcd.backlight();

  // clear the screen
  lcd.clear();

#ifdef DEBUG_CODE

  // Below code can be commented ; only for testing purpose
  lcd.setCursor(0, 0);
  lcd.print("0,0");

  lcd.setCursor(0, 1);
  lcd.print("1,0");

  delay(500);

  // clear the screen
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("IoT Bakers");

  delay(500);

  lcd.setCursor(0, 1);
  lcd.print("Suntronics");

  delay(500);

  // Some test cases
  // lcd_set_msg("0123456789ABCDEF");
  // delay(1000);
  // lcd_set_msg("0123456789ABCDE", 1);
  // delay(1000);
  // lcd_set_msg("123456789ABCDE", 0);
  // lcd_set_msg("123456789ABCDE", row, col ,clr,centre);
  // delay(2000);

  // clear the screen
  lcd.clear();

#else

  lcd.setCursor(0, 0);
  lcd.print("Company Name");

  lcd.setCursor(1, 0);
  lcd.print("IoTBakers");

  delay(500);

  // clear the screen
  lcd.clear();

#endif
}

#endif

#ifdef LED_7SEG_TM1637_I2C

void setup_7seg_i2c()
{
  ;
}

#endif

void setup_push_buttons()
{
  ;
}

#ifdef ANALOG_INPUT_POT

void setup_analog_read_pot()
{
  ;
}

#endif

// Does nothing as of now
void setup_analog_read_vbat()
{
  ;
}

#ifdef ADS1115_SENSOR

void setup_ads1115()
{
  // Serial.println("Getting differential reading from AIN0 (P) and AIN1 (N)");
  // Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  ads1115.setGain(GAIN_SIXTEEN); // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  if (!ads1115.begin())
  {
    lcd_set_msg("...");
    delay(5000);

#ifdef DEBUG_CODE
    lcd_set_msg("Failed ADS");
    Serial.println("Failed to initialize ADS.");
    delay(2000);
#endif
  }
  else
  {
    lcd_set_msg("...");
    delay(700);

#ifdef DEBUG_CODE
    lcd_set_msg("Success ADS");
    Serial.println("Success to initialize ADS.");
    delay(700);
#endif
  }
}

#endif

#ifdef INA219_SENSOR

void setup_ina219()
{
  ;
}

#endif

#ifdef DEBUG_CODE

void print_BatterySocData_h()
{
  delay(1000);
  Serial.print("battery_voltage");
  Serial.print("\t");
  Serial.println(battery_voltage);

  Serial.print("battery_capacity_ah");
  Serial.print("\t");
  Serial.println(battery_capacity_ah);

  Serial.print("battery_capacity_coulomb");
  Serial.print("\t");
  Serial.println(battery_capacity_coulomb);

  Serial.print("shunt_voltage");
  Serial.print("\t");
  Serial.println(shunt_voltage);

  Serial.println("\n\n===================");

  Serial.print("SHUNT_RESISTANCE");
  Serial.print("\t");
  Serial.println(SHUNT_RESISTANCE);

  Serial.print("SHUNT_MULTIPLIER");
  Serial.print("\t");
  Serial.println(SHUNT_MULTIPLIER);

  Serial.print("BATTERY_CAPACITY_AH");
  Serial.print("\t");
  Serial.println(BATTERY_CAPACITY_AH);

  Serial.print("BATTERY_CAPACITY_COULOMB");
  Serial.print("\t");
  Serial.println(BATTERY_CAPACITY_COULOMB);

  Serial.print("battery_capacity_milli_coulomb");
  Serial.print("\t");
  Serial.println(battery_capacity_milli_coulomb);

  // Serial.print("shunt_voltage");
  // Serial.print("\t");
  // Serial.println(shunt_voltage);
  // Serial.print("BATTERY_CAPACITY_AH");
  // Serial.print("\t");
  // Serial.println(BATTERY_CAPACITY_AH);

  // Serial.print("shunt_voltage");
  // Serial.print("\t");
  // Serial.println(shunt_voltage);
  // Serial.print("BATTERY_CAPACITY_AH");
  // Serial.print("\t");
  // Serial.println(BATTERY_CAPACITY_AH);

  // Serial.print("shunt_voltage");
  // Serial.print("\t");
  // Serial.println(shunt_voltage);

  Serial.println("\n\n===================");
}

#endif

// TODO : domething
ESP8266WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

uint8_t current_ws_channel = 0;

void send_data_back_to_websocket()
{
  String ws_data =
      String(raw_a0_read) + ";" +
      String(raw_ads_read) + ";" +
      String(current_read_micros) + ";" +
      String(last_read_micros) + ";;";

  webSocket.sendTXT(current_ws_channel, ws_data);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  current_ws_channel = num;
  switch (type)
  {
  case WStype_DISCONNECTED:
    Serial.printf("[%u] Disconnected!\n", num);
    break;
  case WStype_CONNECTED:
  {
    IPAddress ip = webSocket.remoteIP(num);
    Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);

    // send message to client
    webSocket.sendTXT(num, "Connected");
  }
  break;
  case WStype_TEXT:
    Serial.printf("[%u] get Text: %s\n", num, payload);

    if (payload[0] == '#')
    {
      // we get RGB data

      // decode rgb data
      uint32_t rgb = (uint32_t)strtol((const char *)&payload[1], NULL, 16);

      // analogWrite(LED_RED,    ((rgb >> 16) & 0xFF));
      // analogWrite(LED_GREEN,  ((rgb >> 8) & 0xFF));
      // analogWrite(LED_BLUE,   ((rgb >> 0) & 0xFF));
    }

    break;
  }
}

void setup()
{

#ifdef DEBUG_CODE

  //  Setup serial
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);

  print_BatterySocData_h();

#endif
  
  EEPROM.begin(512);  //Initialize EEPROM

  delay(2000);

  // TODO: re read

    Serial.println("bat_colmbs");

  // writeStringToEEPROM(0, "108000.00");
  delay(100);
  String bat_colmbs = readStringFromEEPROM(0);
  Serial.println(bat_colmbs);
  battery_capacity_coulomb = bat_colmbs.toDouble();
  battery_capacity_milli_coulomb = battery_capacity_coulomb * 1000;

  Serial.println(battery_capacity_coulomb);
  Serial.println(battery_capacity_milli_coulomb);

  // Connect to Wi-Fi
  WiFi.begin("Ayush", "Homewifi@123");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println(WiFi.localIP());
  delay(5000);
  // start webSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

#ifdef LCD_16X2_I2C

  setup_lcd_i2c();

#endif

#ifdef LED_7SEG_TM1637_I2C

  setup_7seg_i2c();

#endif

  setup_push_buttons();

#ifdef ANALOG_INPUT_POT

  setup_analog_read_pot();

#endif

#ifdef ADS1115_SENSOR

  setup_ads1115();

#endif

#ifdef INA219_SENSOR

  setup_ina219();

#endif

  ticker_update_lcd.start();   // TODO : Find right place to call it
  ticker_update_EEPROM_data.start();
  last_read_micros = micros(); // TODO : Find right place to call it
}

// int16_t raw_a0_read;
void analog_read_vbat()
{
  // battery_voltage

  // *(R2_VOLTAGE_DIVIDER_BATTERY_VOLTAGE/(R2_VOLTAGE_DIVIDER_BATTERY_VOLTAGE + R1_VOLTAGE_DIVIDER_BATTERY_VOLTAGE))
  // v across R2 =  x = (5.0*(analogRead(PIN_ANALOG_BATTERY_VOLTAGE)))/(1023)

  // battery_voltage = ((5.0 * (analogRead(PIN_ANALOG_BATTERY_VOLTAGE))) / (1023)) * ((R2_VOLTAGE_DIVIDER_BATTERY_VOLTAGE + R1_VOLTAGE_DIVIDER_BATTERY_VOLTAGE) / R2_VOLTAGE_DIVIDER_BATTERY_VOLTAGE);
  raw_a0_read = analogRead(PIN_ANALOG_BATTERY_VOLTAGE);
  battery_voltage = (raw_a0_read * BAT_VOLTAGE_DIVIDER_CONST_VAL) / 1000.0;

  // TODO : reduce the nuber of constant in the equation
  // battery_voltage =
}

void show_vbat_lcd()
{
  lcd_set_msg("V bat");
  lcd_set_msg(String(battery_voltage, 2) + " V", 1, 0, false, true);
}

#ifdef DEBUG_CODE
unsigned long temp_counter = 0; // TODO: delete , and related code also
#endif

double calculte_milli_coulombs_transffred()
{
  // dt is in micro seconds not ms .
  double dt_ms = current_read_micros - last_read_micros;

  // shunt_voltage is in mV
  // float calculte_coulombs_transffred =       ((shunt_voltage / (1000.0)) / (SHUNT_RESISTANCE)) * (current_read_micros - last_read_micros)* (0.000001);
  // float calculte_milli_coulombs_transffred = ((shunt_voltage / (1000.0)) / (SHUNT_RESISTANCE)) * (current_read_micros - last_read_micros)* (0.001);

  //      (shunt_voltage mV) * (dt in uS) * (10 ^ -6)
  //  =  --------------------------------------------
  //      (1000.0) * (SHUNT_RESISTANCE Ohm)

  //                                      (shunt_voltage mV) * 1000.0             *(dt in uS) * (10 ^ -6)
  //  calculte_coulombs_transffred  =  --------------------------------------------------------------------
  //                                          (1000.0) * (SHUNT_RESISTANCE mOhm)

  //                                          (shunt_voltage mV) * 1000.0             *(dt in uS) * (10 ^ -6) * (10 ^ 3)
  //  calculte_milli_coulombs_transffred  =  ---------------------------------------------------------------------------
  //                                              (1000.0) * (SHUNT_RESISTANCE mOhm)

  //                                           shunt_voltage mV    * (dt in uS) * (10 ^ -3)
  // calculte_milli_coulombs_transffred  =    ---------------------
  //                                           SHUNT_RESISTANCE mOhm

  // for testing

#ifdef DEBUG_CODE

  // if hard coded / Pot  used for testing
  // double calculte_coulombs_transffred = (-75.0 / (1000.0 * SHUNT_RESISTANCE)) * (dt_ms) * (0.000001);
  // double calculte_milli_coulombs_transffred = (map(analogRead(A0), 0, 1023, -8000, 8000) * (dt_ms)) * (0.001);

  // if shunt_voltage
  double calculte_milli_coulombs_transffred = ((shunt_voltage * dt_ms) / SHUNT_RESISTANCE_MILLI_OHM) * (0.001);
  double calculte_coulombs_transffred = calculte_milli_coulombs_transffred / 1000.0;

  temp_counter++;
  if (temp_counter < 200)
  {
    Serial.print("dt_ms\t");
    Serial.println(dt_ms);

    Serial.print("last_read_micros\t");
    Serial.println(last_read_micros);

    Serial.print("current_read_micros\t");
    Serial.println(current_read_micros);

    Serial.print("shunt_voltage\t");
    Serial.println(shunt_voltage);

    Serial.print("calculte_coulombs_transffred\t");
    Serial.println(calculte_coulombs_transffred, 10);

    Serial.print("calculte_milli_coulombs_transffred\t");
    Serial.println(calculte_milli_coulombs_transffred, 10);

    Serial.print("battery_capacity_coulomb\t");
    Serial.println(battery_capacity_coulomb, 10);

    Serial.print("battery_capacity_milli_coulomb\t");
    Serial.println(battery_capacity_milli_coulomb, 10);

    Serial.print("BATTERY_CAPACITY_COULOMB\t");
    Serial.println(BATTERY_CAPACITY_COULOMB * 1000, 10);

    Serial.print("BATTERY_CAPACITY_MILLI_COULOMB\t");
    Serial.println(BATTERY_CAPACITY_MILLI_COULOMB * 1000, 10);

    Serial.print("% bat\t");
    Serial.println(((battery_capacity_coulomb * 100.0) / BATTERY_CAPACITY_COULOMB * 1000), 5);

    Serial.print("% batA\t");
    Serial.println((battery_capacity_coulomb / BATTERY_CAPACITY_COULOMB * 1000) * 100.0, 5);

    Serial.print("% batB\t");
    Serial.println((int(battery_capacity_coulomb) / int(BATTERY_CAPACITY_COULOMB * 1000)) * 100.0, 5);

    Serial.print("% batC\t");
    Serial.println((int(battery_capacity_coulomb) * 100.0 / int(BATTERY_CAPACITY_COULOMB * 1000)), 5);

    Serial.print("% bat\t");
    Serial.println(((battery_capacity_milli_coulomb * 100.0) / BATTERY_CAPACITY_MILLI_COULOMB * 1000), 5);

    Serial.print("% batA\t");
    Serial.println((battery_capacity_milli_coulomb / BATTERY_CAPACITY_MILLI_COULOMB * 1000) * 100.0, 5);

    Serial.print("% batB\t");
    Serial.println((int(battery_capacity_milli_coulomb) / int(BATTERY_CAPACITY_MILLI_COULOMB * 1000)) * 100.0, 5);

    Serial.print("% batC\t");
    Serial.println((int(battery_capacity_milli_coulomb) * 100.0 / int(BATTERY_CAPACITY_MILLI_COULOMB * 1000)), 5);

    Serial.println("");
  }

#else

  // double calculte_milli_coulombs_transffred = ((shunt_voltage / SHUNT_RESISTANCE_MILLI_OHM) * (dt_ms)) * (0.001);
  double calculte_milli_coulombs_transffred = ((shunt_voltage * dt_ms) / SHUNT_RESISTANCE_MILLI_OHM) * (0.001);

#endif

  last_read_micros = current_read_micros;
  return calculte_milli_coulombs_transffred;
}

void show_coulomb_lcd()
{
  lcd_set_msg("BAT COULOMB");
  lcd_set_msg(String(battery_capacity_coulomb, 2) + " C", 1, 0, false, true);
}

void show_battery_percent_lcd()
{
  lcd_set_msg("BAT PERCENT");
  // lcd_set_msg(String(((battery_capacity_coulomb * 100.0) / BATTERY_CAPACITY_COULOMB), 2) + " %", 1, 0, false, true);
  // lcd_set_msg(String((0.003), 2) + " %", 1, 0, false, true);

  // float percent_val = int(battery_capacity_coulomb) * 100.0 / int(BATTERY_CAPACITY_COULOMB);

  // milli coulomb
  float percent_val = uint32_t(battery_capacity_coulomb) * 100.0 / uint32_t(BATTERY_CAPACITY_COULOMB);

  // float percent_val = double(battery_capacity_milli_coulomb) * 100.0 / double(BATTERY_CAPACITY_MILLI_COULOMB * 1000);

  lcd_set_msg(String(percent_val, 2) + " %", 1, 0, false, true);
}

// int16_t raw_ads_read;
void differential_analog_read_ads_vshunt()
{
  raw_ads_read = ads1115.readADC_Differential_0_1();
  // shunt_voltage = ads1115.readADC_Differential_0_1() * SHUNT_MULTIPLIER;
  shunt_voltage = raw_ads_read * SHUNT_MULTIPLIER;
  current_read_micros = micros();
}

void show_vshunt_lcd()
{
  lcd_set_msg("V Shunt");
  lcd_set_msg(String(shunt_voltage, 2) + " mV", 1, 0, false, true);
}

void update_lcd_display()
{

  switch (lcd_screen_no)
  {
  case 0:
    show_vbat_lcd();
    break;

  case 1:
    show_vshunt_lcd();
    break;

  case 2:
    show_coulomb_lcd();
    break;

  case 3:
    show_battery_percent_lcd();
    break;

  default:
    lcd_screen_no = 0;
    break;
  }

  lcd_screen_no++;
  // lcd_screen_no = 3;
  if (lcd_screen_no == 4)
  {
    lcd_screen_no = 0;
  }
}

void update_charge_transfer_data()
{

  // battery_capacity_coulomb += double(calculte_coulombs_transffred());
  // battery_capacity_coulomb = long(battery_capacity_coulomb) + long(calculte_coulombs_transffred());

  int calculte_milli_coulombs_transffred_temp = int(calculte_milli_coulombs_transffred());

  // if battery_capacity_milli_coulomb goes into negative
  // when battart is fully drained then stop  at zero

  if (flag_battery_fully_drained == false && calculte_milli_coulombs_transffred_temp < 0 && battery_capacity_milli_coulomb < uint32_t(-1 * calculte_milli_coulombs_transffred_temp))
  {
    // calculte_milli_coulombs_transffred_temp = battery_capacity_milli_coulomb;

    battery_capacity_milli_coulomb = 0;
    battery_capacity_coulomb = 0;
    flag_battery_fully_drained = true;

#ifdef DEBUG_CODE
    Serial.println("bat fully drained 0% !!!");
    Serial.println(battery_capacity_milli_coulomb);
    Serial.println(battery_capacity_coulomb);
#endif
  }
  else
  {
    // When battary is fully charged , Stop calculating the coulomb
    if (flag_battery_fully_charged == false && battery_capacity_milli_coulomb >= battery_capacity_milli_coulomb_const)
    {
      battery_capacity_milli_coulomb = battery_capacity_milli_coulomb_const;
      battery_capacity_coulomb = uint32_t(BATTERY_CAPACITY_COULOMB);
      flag_battery_fully_charged = true;

#ifdef DEBUG_CODE
      Serial.println("bat fully charged 100% !!!");
      Serial.println(battery_capacity_milli_coulomb);
      Serial.println(battery_capacity_coulomb);
#endif
    }
    else
    {
      if ((flag_battery_fully_charged == true && calculte_milli_coulombs_transffred_temp > 0) || (flag_battery_fully_drained == true && calculte_milli_coulombs_transffred_temp < 0))
      {
        // do nothing
        ;
        // if battery is fully charged and it is trying to over charge
        // or
        // if battery is fully drained and it is trying to over drained
      }
      else
      {
        battery_capacity_milli_coulomb = battery_capacity_milli_coulomb + calculte_milli_coulombs_transffred_temp;
        battery_capacity_coulomb = double(battery_capacity_milli_coulomb / 1000.0);
      }

      if (battery_capacity_coulomb > 0 && battery_capacity_coulomb < BATTERY_CAPACITY_COULOMB)
      {
        flag_battery_fully_charged = false;
        flag_battery_fully_drained = false;
      }
    }
  }
}

void process_and_update_data()
{
  update_charge_transfer_data();
}

void read_sensor_data()
{
  analog_read_vbat();
  differential_analog_read_ads_vshunt();

  send_data_back_to_websocket(); // TODO : websocket
}

void ticker_loop()
{
  ticker_update_lcd.update();
  ticker_update_EEPROM_data.update();
}

void loop()
{

  read_sensor_data();

  process_and_update_data();

  ticker_loop();

  // send data to computer
  webSocket.loop();
}