#include <Servo.h>
#include <LiquidCrystal.h>
//#include <avr/pgmspace.h>

// инициализируем объект-экран, передаём использованные
// для подключения контакты на Arduino в порядке:
// RS, E, DB4, DB5, DB6, DB7
LiquidCrystal lcd(11, 12, 5, 4, 3, 2);

char desplayLine0[20];//16 симвлов в строке на экране + 1 (нужно)
char desplayLine1[20];
char float_str1[8];
char float_str2[8];

int adc_value = 0;
double voltage_average_value = 0;
double dc_voltage_V0 = 4.85;//
double dc_voltage_V1 = 0;
const float resistance = 2.5;//
unsigned long sample_count = 0;
const unsigned long n = 15;//5000//100
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
double M20 = 0;//0.2

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
    };// среднее отклонение (ищем в excel) (фильтрация второй раз)
    void SetVarProcess(float value) {
      varProcess = value;
    };// скорость реакции на изменение (подбирается вручную) (фильтрация второй раз)
  private:

    float varVolt1 = 68.5;  // среднее отклонение (ищем в excel)
    float varProcess1 = 0.5; // скорость реакции на изменение (подбирается вручную)
    float varVolt2 = 68.5;  // среднее отклонение (ищем в excel)
    float varProcess2 = 0.5; // скорость реакции на изменение (подбирается вручную)

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

class TwoLevelFilter//класс двухуревневой фильтрации
{
  public:

    TwoLevelFilter(float varVolt1, float varProcess1, float varVolt2, float varProcess2) {
      _f1 = *(new CalmanFilter());
      _f2 = *(new CalmanFilter());

      _f1.SetVarVolt(varVolt1);
      _f1.SetVarProcess(varProcess1);

      _f2.SetVarVolt(varVolt2);
      _f2.SetVarProcess(varProcess2);
    };
  public:

    float filter(float val) {
      _val_filtered1 = _f1.filter(val);
      _val_filtered2 = _f2.filter(_val_filtered1);
      return _val_filtered2;
    };

    float GetVal_filtered1() {
      return _val_filtered1;
    };
    float GetVal_filtered2() {
      return _val_filtered2;
    };

  private:

    CalmanFilter _f1, _f2;
    float _val_filtered1, _val_filtered2;

};

//фильтры калмана для сенсоров тока
TwoLevelFilter calmanFilter_dc_A1 = *(new TwoLevelFilter(68.5, 0.5, 0.5, 0.5));
TwoLevelFilter calmanFilter_dc_A2 = *(new TwoLevelFilter(218.8, 10, 218.8, 1));
//фильтры калмана для сенсоров тока


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

}


int ArrayCompare(const int *AFirst, const int *ASecond)
{
  if (*AFirst < *ASecond) return -1;
  return (*AFirst == *ASecond) ? 0 : 1;
}

// функция считывает аналоговый вход заданное количество раз
// и возвращает медианное отфильтрованное значение
float readMedian (int pin0, int pin1, int samples) {
  // массив для хранения данных
  long raw[samples];
  long tmp = 0;
  // считываем вход и помещаем величину в ячейки массива
  for (int i = 0; i < samples; i++) {
    for (int j = 0; j < samples; j++) {
      tmp += analogRead(pin0) - analogRead(pin1);
    }
    raw[i] = tmp / samples;
  }
  // сортируем массив по возрастанию значений в ячейках
  size_t M_Size = sizeof(raw) / sizeof(raw[0]);

  qsort(raw, M_Size, sizeof(raw[0]), ArrayCompare);


  // возвращаем значение средней ячейки массива
  return (float)raw[samples / 2];
}

void loop()

{
  //================================ ТОК ========================================

  //  dc_A1 = GetCurrent_mA(A0, A1);
  //  dc_A2 = GetCurrent_mA(A2, A3);

  dc_A1 = readMedian(A0, A1, n) * 0.00488 * 1000 / resistance; //---
  dc_A2 = readMedian(A2, A3, n) * 0.00488 * 1000 / resistance; //---

  dc_A1_filtered = calmanFilter_dc_A1.filter(dc_A1);
  dc_A2_filtered = calmanFilter_dc_A2.filter(dc_A2);

  PrintCurrent();

  //================================ МОМЕНТЫ И СИЛЫ ========================================

  M1 = k1 * dc_A1_filtered + M10;
  M2 = k2 * dc_A2_filtered + M20;

  //В кг
  F1 = abs(M1 / l1);
  F2 = abs(M2 / l2);



  V_l1.X = cos(PI - a1 * DEG_TO_RAD); V_l1.Y = sin(PI - a1 * DEG_TO_RAD);//направляющий вектор звена l1
  V_l2.X = cos(PI + a2 * DEG_TO_RAD); V_l2.Y = sin(PI + a2 * DEG_TO_RAD);//направляющий вектор звена l2

  //положительное направление куртящего момента Mi - против часовой стрелки
  if (M1 <= 0) {
    V_F1.X = V_l1.Y; V_F1.Y = -V_l1.X; //направляющий нормированный вектор силы F1//!!temp!!{повернут направо от звена l1 под прямым углом}
  }
  else {
    V_F1.X = -V_l1.Y; V_F1.Y = V_l1.X; //налево
  }

  if (M2 <= 0) {
    V_F2.X = V_l2.Y; V_F2.Y = -V_l2.X; //направляющий нормированный вектор силы F2//!!temp!!{повернут направо от звена l1 под прямым углом}
  }
  else {
    V_F2.X = -V_l2.Y; V_F2.Y = V_l2.X; //налево
  }
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
  //=========== Serial Plotter =======================

  Serial.print(0);
  Serial.print(" ");
  Serial.print(dc_A1);
  Serial.print(" ");
  Serial.println(dc_A1_filtered);
  //  Serial.print(" ");
  //  Serial.println(calmanFilter_dc_A2.GetVal_filtered2());

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
  sprintf(desplayLine1, "V_F=(%-3s;%-3s)", float_str1, float_str2);

  //  sprintf(desplayLine1, "F=%-3d,%-3d,%-3d   ", f, f1, f2); //все 16 символов обновляем

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

double GetCurrent_mA(int analogPin0, int analogPin1)
{
  delta_voltage_sum = 0;
  for (sample_count = 0; sample_count < n; sample_count ++)
  {
    adc_value = analogRead(analogPin0) - analogRead(analogPin1);
    delta_voltage_sum += adc_value;
    delayMicroseconds(10);
  }

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
