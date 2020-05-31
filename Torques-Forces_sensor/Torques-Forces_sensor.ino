#include <Servo.h>
#include <LiquidCrystal.h>
//#include <avr/pgmspace.h>

// инициализируем объект-экран, передаём использованные
// для подключения контакты на Arduino в порядке:
// RS, E, DB4, DB5, DB6, DB7
LiquidCrystal lcd(11, 12, 5, 4, 3, 2);

char desplayLine0[17];//16 симвлов в строке на экране + 1 (нужно)
char desplayLine1[17];
char float_str1[8];
char float_str2[8];

int adc_value = 0;
double voltage_average_value = 0;
double dc_voltage_V0 = 4.85;//
double dc_voltage_V1 = 0;
const float resistance = 0.2;//
unsigned long sample_count = 0;
const unsigned long n = 100;//5000//
long delta_voltage_sum = 0;
long voltage_sum = 0;

double dc_A1, dc_A2, dc_A3;

const double dc_wall_A1 = 80;//mA
const double dc_wall_A2 = 70;
const double dc_wall_A3 = 300;

const bool Adaptive_Enable = false;//

Servo servoA1;  // create servo object to control a servo
Servo servoA2;  // create servo object to control a servo
Servo servoA3;  // create servo object to control a servo
// twelve servo objects can be created on most boards
String myString;
float a1, a2, a3;
const float deltaA = 2;//DEG
float a1d = 29.1, a2d = 26, a3d = - 5; //значения для выставления нулевых значений



float dc_A1_filtered = 0;
float dc_A2_filtered = 0;

double M1, M2, F1, F2;

double k1 = 0.0116;//
double k2 = 0.0272;//0.0374//0.0306
double M10 = 0;//
double M20 = 2.4;//0.2

double l1 = 15;
double l2 = 16;

typedef struct
{
  double X;
  double Y;
  double length() {
    return sqrt(X * X + Y * Y);
  };
  void norm() {
    X = X / length(); Y = Y / length();
  };
} Point2D;

Point2D V_l1, V_l2, V_F1, V_F2, V_F, V_F_norm;




//================ Класс фильтр калмана =====================

class CalmanFilter
{
  public:
    float filter(float val) {  //функция фильтрации
      Pc = P + varProcess;
      G = Pc / (Pc + varVolt);
      P = (1 - G) * Pc;
      Xp = Xe;
      Zp = Xp;
      Xe = G * (val - Zp) + Xp; // "фильтрованное" значение
      return (Xe);
    };
    void SetVarVolt(float value) {
      varVolt = value;
    };// среднее отклонение (ищем в excel)
    void SetVarProcess(float value) {
      varProcess = value;
    };// скорость реакции на изменение (подбирается вручную)
  private:
    float varVolt = 68.5;  // среднее отклонение (ищем в excel)
    float varProcess = 0.5; // скорость реакции на изменение (подбирается вручную)
    float Pc = 0.0;
    float G = 0.0;
    float P = 1.0;
    float Xp = 0.0;
    float Zp = 0.0;
    float Xe = 0.0;

};

//================ Класс фильтр калмана =====================

// переменные для калмана
CalmanFilter calmanFilter_dc_A1;
CalmanFilter calmanFilter_dc_A2;
// переменные для калмана


void setup()
{
  Serial.begin(9600);
  // pinMode(13, OUTPUT);

  servoA1.attach(9);             // a1
  servoA2.attach(10, 544, 2480); // a2
  servoA3.attach(6, 544, 2500);  // a3
  Serial.begin(9600);
  a1 = 57.63;                    //
  a2 = 66.29;                    //
  a3 = 0.56;                     //начальные значения углов
  SendTaskToServos(a1, a2, a3);

  // устанавливаем размер (количество столбцов и строк) экрана
  lcd.begin(16, 2);

  //фильтры калмана для сенсоров тока
  calmanFilter_dc_A1 = *(new CalmanFilter);
  calmanFilter_dc_A1.SetVarVolt(68.5);//!!
  calmanFilter_dc_A1.SetVarProcess(0.5);//!!

  calmanFilter_dc_A2 = *(new CalmanFilter);
  calmanFilter_dc_A2.SetVarVolt(39.5);//!!
  calmanFilter_dc_A2.SetVarProcess(0.05);//!!//0.2
}

void loop()

{
  //================================ ТОК ========================================

  dc_A1 = GetCurrent_mA(A0, A1);
  dc_A2 = GetCurrent_mA(A2, A3);
  dc_A1_filtered = calmanFilter_dc_A1.filter(dc_A1);
  dc_A2_filtered = calmanFilter_dc_A2.filter(dc_A2);

  PrintCurrent();

  //================================ МОМЕНТЫ И СИЛЫ ========================================

  M1 = k1 * dc_A1_filtered + M10;
  M2 = k2 * dc_A2_filtered + M20;

  //В кг
  F1 = M1 / l1;
  F2 = M2 / l2;

  //test
  //сила F1
  if (dc_A2_filtered < 10)//отсекаем
    F1 = 0;
  else
    F1 = 2.5755 * dc_A2_filtered + 52.217;
  F1 /= 1000;// в кг
  //сила F2
  if (dc_A2_filtered < 15)//отсекаем
    F2 = 0;
  else
    F2 = 149.86 * log(dc_A2_filtered) - 349.09; //log = ln
  F2 /= 1000;// в кг
  //test

  V_l1.X = cos(PI - a1 * DEG_TO_RAD); V_l1.Y = sin(PI - a1 * DEG_TO_RAD);//направляющий вектор звена l1
  V_l2.X = cos(PI + a2 * DEG_TO_RAD); V_l2.Y = sin(PI + a2 * DEG_TO_RAD);//направляющий вектор звена l2

  V_F1.X = V_l1.Y; V_F1.Y = -V_l1.X; //направляющий нормированный вектор силы F1//!!temp!!{повернут направо от звена l1 под прямым углом}:{нужно учесть направление момента M1}
  V_F2.X = V_l2.Y; V_F2.Y = -V_l2.X; //направляющий нормированный вектор силы F1//!!temp!!{повернут направо от звена l1 под прямым углом}:{нужно учесть направление момента M1}

  //??
  //длины векторов сил в кг
  V_F1.X *= F1; V_F1.Y *= F1;
  V_F2.X *= F2; V_F2.Y *= F2;

  V_F.X = V_F1.X + V_F2.X; V_F.Y = V_F1.Y + V_F2.Y;

  V_F_norm.X = V_F.X / V_F.length(); V_F_norm.Y = V_F.Y / V_F.length(); //нормированный вектор силы F
  //??


  PrintForces();

  //======================= Коллаборативный режим ============================

  if (Adaptive_Enable) {
    if (dc_A1 > dc_wall_A1)
    {
      a1 = a1 + deltaA;
      servoA1.write(a1 + a1d);
    }
    if (dc_A2 > dc_wall_A2)
    {
      a2 = a2 - deltaA;
      servoA2.write(a2 + a1d);
    }
    if (dc_A3 > dc_wall_A3)
    {
      a3 = a3 + deltaA;
      servoA3.write(a3 + a1d);
    }
  }
  //================================ СЕРВО ============================================

  if (Serial.available()) {
    myString = Serial.readString();
    int commaIndex = myString.indexOf(',');
    int secondCommaIndex = myString.indexOf(',', commaIndex + 1);

    a1 = myString.substring(0, commaIndex).toFloat();
    a2 = myString.substring(commaIndex + 1, secondCommaIndex).toFloat();
    a3 = myString.substring(secondCommaIndex + 1).toFloat(); // To the end of the string
    SendTaskToServos(a1, a2, a3);

    delay(20);//чтение раз в 20 млс
    delay(300);//200млс повернуть серво, запас 100 млс
  }

  //=================================================================================

  //PrintAngles();

  //=================================================================================

  updateDisplay();
}



void updateDisplay() {

  lcd.setCursor(0, 0);
  lcd.print(desplayLine0);
  lcd.setCursor(0, 1);
  lcd.print(desplayLine1);

}

void PrintCurrent()
{
  //  Serial.print(dc_A1);
  //  Serial.print(" ");
  //  Serial.println(dc_A2);
  ////  Serial.print(dc_A3);
  ////  Serial.print(" mA");

  //=========== Serial Plotter =======================

  //      Serial.print(dc_A2);
  //      Serial.print(" ");
  //      Serial.println(dc_A2_filtered);

  //Serial.println(dc_A2);//---



  //==================================================

  int a1 = round(dc_A1_filtered);
  int a2 = round(dc_A2_filtered);

  sprintf(desplayLine0, "A1,A2:%-3d,%-3d mA", a1, a2);

}

void PrintForces()
{
  //в г
  int f1 = round(F1 * 1000);
  int f2 = round(F2 * 1000);
  int f = round(V_F.length() * 1000);

  dtostrf(V_F_norm.X, 4, 2, float_str1);
  dtostrf(V_F_norm.Y, 4, 2, float_str2);
  //  sprintf(desplayLine1, "V_F=(%-3s;%-3s)", float_str1, float_str2);

  sprintf(desplayLine1, "F=%-3d,%-3d,%-3d   ", f, f1, f2); //все 16 символов обновляем

}

void PrintAngles()
{

  Serial.print("                        ");
  Serial.print(a1);
  Serial.print("__");
  Serial.print(a2);
  Serial.print("__");
  Serial.print(a3);
  Serial.println(" DEG  a1__a2__a3");
}

void SendTaskToServos(float a1, float a2, float a3) {
  servoA1.write(a1 + a1d);
  servoA2.write(a2 + a2d);
  servoA3.write(90 - a3 + a3d);
}

double GetCurrent_mA(int analogPin0, int analogPin1)// берем значения по модулю
{
  delta_voltage_sum = 0;
  for (sample_count = 0; sample_count < n; sample_count ++)
  {
    adc_value = analogRead(analogPin0) - analogRead(analogPin1);
    delta_voltage_sum += adc_value;
    delayMicroseconds(10);
  }
  if (delta_voltage_sum < 0) delta_voltage_sum = 0; //значения около нуля обнулим для устранения ошибки  // берем значения по модулю
  voltage_average_value = (float)delta_voltage_sum / n;
  return (voltage_average_value * 0.00488) * 1000 / resistance;

}

double GetVoltage(int analogPin)
{
  voltage_sum = 0;
  for (sample_count = 0; sample_count < n; sample_count ++)
  {
    adc_value = analogRead(analogPin);
    voltage_sum += adc_value;
    delayMicroseconds(10);
  }
  voltage_average_value = (float)voltage_sum / n;
  return voltage_average_value * 0.00488;
}
