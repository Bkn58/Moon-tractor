/*
   MoonTractor 1.0 Bkn58
 Программа управления 3-х колесным устройством.
  - 2 шаговых двигателя с редуктором на каждом из 2-задних колес
  - сервопривод, на котором закреплен ультразвуковой измеритель расстояний
  - ультразвуковой измеритель расстояний
  - сенсорная кнопка старт/стоп
*/

#include <Stepper.h>     // стандартная библиотека управления шаговым двигателем
#include <Servo.h>       // стандартная библиотека управления сервоприводом
#include "Ultrasonic.h"  // локальная библиотека ультразвукового измерителя расстояния

/*
* типы базовых маневров
*/
const int  FORVARD_TYPE    =1; // движение вперед
const int  RIGHT_TYPE      =2; // движение вправо
const int  LEFT_TYPE       =3; // движение влево
const int  BACK_TYPE       =4; // движение назад
const int  RIGHT_TINY_TYPE =5; // выравнивание вправо
const int  LEFT_TINY_TYPE  =6; // выравнивание влево

Servo servo1;
const int pinServo=10;          // пин для подключения сервопривода
Ultrasonic ultrasonic(11,12);   // Выводы для подключения HC-SR04 Trig - 12, Echo - 13
const int START_STOP_PIN = 13;  // порт сенсорной кнопки

// initialize the stepper library on pins 2 through 9:
const int stepsPerRevolution = 2;   // количество шагов ШД, производимое за один loop
Stepper myStepperL(stepsPerRevolution*100, 2, 4, 3, 5);
Stepper myStepperR(stepsPerRevolution*100, 6, 8, 7, 9);
int pos = 0; // переменная для хранения позиции сервопривода
int dir =1;  // направление вращения сервопривода

int MAXSTEPS = 100;     // максимальное общее кол-во шагов при выполнени маневра 
int stepCount = 0;      // текущее количество шагов
int reversL = 1;        // флаг направления вращения левого ШД
int reversR = 1;        // флаг направления правого правого ШД
int motorSpeed = 150;   // скорость ШД (1...200)
int evolutionStart = 0; // начало выполнения какого- то маневра (вперед, назад, влево, вправо)
int evolutionType = 0;  // тип маневра 1-вперед, 2-назад, 3-влево, 4-вправо, 5-корр. вправо, 6-корр.влево.
float dist_cm90;        // расстояние до препятствия прямо по курсу
float dist_cmR;         // расстояние до препятствия справа
float dist_cmL;         // расстояние до препятствия слева
int angle = 0;          // текущий угол сонара
int startStop=0;        // 1-старт, 0-стоп

/*
* Отслеживание состояния сенсорной кнопки.
*/
void checkStartStop () {
   // чтение сенсорной кнопки
  if ((startStop==1) && (digitalRead(START_STOP_PIN))) {
     while (digitalRead(START_STOP_PIN)) {
	   delay (10); // ждем отпускания кнопки для стоп
     }
     startStop=0; // стоп
  }
  if (startStop==0) {
     do {      		// ждем нажатия кнопки для пуска
       delay (10);
     } while (!digitalRead(START_STOP_PIN));
     while (digitalRead(START_STOP_PIN)) {
	   delay (10); // ждем отпускания кнопки для пуска
     }
     startStop=1;
  }
}
/*
* Измерение дистанции и сохранение минимального мгновенного значения
* в одной из переменных dist_cm90, dist_cmR, dist_cmL в зависимости от текущего угла (+-5 гр.)
*/
void distanceMeasure () {
	int curDist;
	if ((angle >= 0) && (angle <=3)) {
	  curDist = ultrasonic.read(CM);
	  if (dist_cmR > curDist) dist_cmR = curDist;
	}
	if ((angle >= 88) && (angle <=92)) {
	  curDist = ultrasonic.read(CM);
	  if (dist_cm90 > curDist) dist_cm90 = curDist;
	}
	if ((angle >= 177) && (angle <=180)) {
	  curDist = ultrasonic.read(CM);
	  if (dist_cmL > curDist) dist_cmL = curDist;
	}
}
/*
* Поворот сервопривода на одно деление.
*/
const int loopPerGrad = 3; // количество основных циклов на один градус
int loopCnt = 0;           // счетчик циклов
int directionRotate = 1;   // направление вращения сонара 1-справа на лево, -1 - слева на  право
void servRotate () {
	if (loopCnt == 0) {
	  servo1.write(angle); // поворот сервопривода на угол 1 гр.
	  if (angle == 180) {
		  directionRotate = -1;
	  }
	  if (angle == 0) {
		  directionRotate = 1;
	  }
	  angle = angle + directionRotate;
	}
	loopCnt++;
	loopCnt = loopCnt % loopPerGrad;
}
/*
* Быстрый замер трех расстояний (слева, прямо, справа).
*/
void measureRL () {
	servo1.write(180); // поворот сервоприводов на угол 180 гр.
    delay(1000);     // пауза для ожидания поворота сервоприводов
	dist_cmL = ultrasonic.read(CM); // расстояние до препятствия
	
	servo1.write(90); // поворот сервоприводов на угол 90 гр.
    delay(300);     // пауза для ожидания поворота сервоприводов
	dist_cm90 = ultrasonic.read(CM); // расстояние до препятствия

	servo1.write(0); // поворот сервоприводов на угол 0 гр.
    delay(300);     // пауза для ожидания поворота сервоприводов
	dist_cmR = ultrasonic.read(CM); // расстояние до препятствия

	angle = 0;
	directionRotate = 1;
}
/*
* Задаем движение вперед.
* int step - количество шагов, неоходимое для выполнения маневра
*/
void forward (int step) {
		reversL = 1;        // флаг направления левого
		reversR = 1;        // флаг направления правого
	    evolutionType = FORVARD_TYPE;      // тип маневра - движение вперед
	    evolutionStart = 1;
		MAXSTEPS = step;
}
/*
* Задаем движение назад.
* int step - количество шагов, неоходимое для выполнения маневра
*/
void back (int step) {
	// задаем движение назад
		reversL = -1;        // флаг направления левого
		reversR = -1;        // флаг направления правого
	    evolutionType = BACK_TYPE;      // тип маневра - движение назад
	    evolutionStart = 1;
		MAXSTEPS = step;
}
/*
* Задаем движение вправо.
* int step - количество шагов, неоходимое для выполнения маневра
*/
void right (int step) {
	// задаем движение вправо
		reversL = 1;        // флаг направления левого
		reversR = -1;        // флаг направления правого
	    evolutionType = RIGHT_TYPE;      // тип маневра - движение вправо
	    evolutionStart = 1;
		MAXSTEPS = step;
}
/*
* Задаем движение влево.
* int step - количество шагов, неоходимое для выполнения маневра
*/
void left (int step) {
	// задаем движение влево
		reversL = -1;        // флаг направления левого
		reversR = 1;        // флаг направления правого
	    evolutionType = LEFT_TYPE;      // тип маневра - движение влево
	    evolutionStart = 1;
		MAXSTEPS = step;
}
/*
* Задаем выравнивание вправо.
* int step - количество шагов, неоходимое для выполнения маневра
*/
void rightTiny (int step) {
	// задаем движение вправо
	    right (step);
	    evolutionType = RIGHT_TINY_TYPE;      // тип маневра - выравнивание вправо
}
/*
* Задаем выравнивание влево.
* int step - количество шагов, неоходимое для выполнения маневра
*/
void leftTiny (int step) {
	// задаем движение влево
		left (step);
	    evolutionType = LEFT_TINY_TYPE;      // тип маневра - движение влево
}

void setup() {

 pinMode (START_STOP_PIN,INPUT);  // инициализация сенсорной кнопки стоп-пуск
 servo1.attach(pinServo);         // инициализация сервопривода с установкой в 90 град.
 
 startStop=0;                     // первоначальное состояние "стоп"
   
 checkStartStop ();               // чтение сенсорной кнопки
 measureRL ();                    // первоначальный быстрый замер дистанциий для 3-х точек (180-90-0 град)
 
 myStepperL.setSpeed(motorSpeed); // задание скорости левого ШД
 myStepperR.setSpeed(motorSpeed); // задание скорости правого ШД

 // СТАРТ движения вперед
 forward (100);

}

void loop() {

  // чтение сенсорной кнопки
  checkStartStop ();
  // вращение сонара на 1 град. 
  servRotate ();
  // измерение дистанции для данного угла поворота
  distanceMeasure ();  

  // маневр выполняется ?
  if (evolutionStart > 0) { 
    myStepperR.step(stepsPerRevolution* reversL);
    myStepperL.step(stepsPerRevolution* reversR);
    stepCount += stepsPerRevolution;
    if (stepCount > MAXSTEPS){
      // маневр закончен
      stepCount = 0;
      evolutionStart = 0;
	  
	  // анализ расстояний и принятие решения о следующем маневре
	  if ((evolutionType!= FORVARD_TYPE)&&(evolutionType!= RIGHT_TINY_TYPE)&&(evolutionType!= LEFT_TINY_TYPE)) 
		  measureRL ();  // быстрый замер расстояний с поворотом сонара в 3-х точках (слева, прямо, справа)
	  if ((dist_cm90 <= 7.0)||(evolutionType == BACK_TYPE)) { 
	    // препятствие прямо по курсу - оцениваем расстояния справа и слева
		if ((dist_cmR <=5.0) && (dist_cmL <= 5.0)) {
			back (3000); // узкий коридор, задний ход
		}
		else if (dist_cmR > dist_cmL) {
			left (1150);
		}
		else {
			right (1150);
		}
	  }
	  else if ((evolutionType == FORVARD_TYPE) && ((dist_cmR <=5.0))) {
		  rightTiny (300); // выравнивание вправо
		  dist_cmR = 6.0;
	  }
	  else if ((evolutionType == FORVARD_TYPE) && ((dist_cmL <=5.0))) {
		  leftTiny (300);  // выравниваем вправо
		  dist_cmL = 6.0;
	  }
	  else {
		forward (100);  // продолжаем движение вперед
	  }
    }
  }
}
