#include <Wire.h>

#define RECON 1
#define ALL_HIGH 2

typedef struct
{
  /* function pointer */
  void (*func)(void);
  /* activation counter */
  volatile int exec;
} TaskAper_t;

TaskAper_t TasksAper[1]; //mudar o 1 se quiser mais tasks aperiodicas

int first_plane = 1;
int plane_counter = 1;

int mode = 0;
int mode_read = 0;

//var para botao
int switch_old = 255;

// Slave addr -> 010.0A2A1A0
int PortExpLEFT = 0x24;   //100
int PortExpCENTER = 0x26; //110
int PortExpRIGHT = 0x27;  //111
int IOCON = 0x0A;
int IODIR = 0x00;
int IPOL = 0x01;
int GPINTEN = 0x02;
int GPPU = 0x06;
int GPIO = 0x09;
int OLAT = 0x0A;

int SR_DIN = 2;  //cabo verde
int SR_CLK = 3;  //cabo azul
int intPin = 18; //botao exterior

int PortExpLEFT_A = 1;
int PortExpLEFT_B = 1;
int PortExpLEFT_B_old = 1;
int PortExpCENTER_A = 1;
int PortExpCENTER_B = 1;
int PortExpRIGHT_A = 1;
int PortExpRIGHT_B = 1;

int LED1 = 1;
int LED2 = 1;

int begin_animation = 0;
int state = 1;

int state_counter = 0;
int help_counter = 0;
int state_animation[] = {2, 4, 2, 3, 5, 3, 1, 1, 1};

/*  Image bank
    0 -> full cube
    1 -> small cube
    2 -> expanded cube horizontal
    3 -> expanded cube vertical
    example: for plane 1 with full cube -> image[0][0][0-5]
*/
int8_t image[][6][6] = {
  { //Full cube
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000}
  },
  { //small cube
    {0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111},
    {0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111},
    {0b11111111, 0b11111111, 0b11110011, 0b11001111, 0b11111111, 0b11111111},
    {0b11111111, 0b11111111, 0b11110011, 0b11001111, 0b11111111, 0b11111111},
    {0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111},
    {0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111}
  },
  { //expanded horizontal 1 led
    {0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111},
    {0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111},
    {0b11111111, 0b11001111, 0b11100001, 0b10000111, 0b11110011, 0b11111111},
    {0b11111111, 0b11001111, 0b11100001, 0b10000111, 0b11110011, 0b11111111},
    {0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111},
    {0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111}
  },
  { //expanded vertical 1 led
    {0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111},
    {0b11111111, 0b11111111, 0b11110011, 0b11001111, 0b11111111, 0b11111111},
    {0b11111111, 0b11111111, 0b11110011, 0b11001111, 0b11111111, 0b11111111},
    {0b11111111, 0b11111111, 0b11110011, 0b11001111, 0b11111111, 0b11111111},
    {0b11111111, 0b11111111, 0b11110011, 0b11001111, 0b11111111, 0b11111111},
    {0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111}
  },
  { //expanded horizontal 2 led
    {0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111},
    {0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111},
    {0b11110011, 0b11001111, 0b11000000, 0b00000011, 0b11110011, 0b11001111},
    {0b11110011, 0b11001111, 0b11000000, 0b00000011, 0b11110011, 0b11001111},
    {0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111},
    {0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111}
  },
  { //expanded vertical 2 led
    {0b11111111, 0b11111111, 0b11110011, 0b11001111, 0b11111111, 0b11111111},
    {0b11111111, 0b11111111, 0b11110011, 0b11001111, 0b11111111, 0b11111111},
    {0b11111111, 0b11111111, 0b11110011, 0b11001111, 0b11111111, 0b11111111},
    {0b11111111, 0b11111111, 0b11110011, 0b11001111, 0b11111111, 0b11111111},
    {0b11111111, 0b11111111, 0b11110011, 0b11001111, 0b11111111, 0b11111111},
    {0b11111111, 0b11111111, 0b11110011, 0b11001111, 0b11111111, 0b11111111}
  }
};

// these are HW definitions
// LEDs ports
#define d1 13 // built-in LED - positive logic

#define ON LOW // built-in LED has positive logic
#define OFF HIGH

//*************** Multi-tasking kernel ******************

//*************** kernel variables ******************
// defines max number of tasks supported
#define MAXT 10

// kernel structure that defines the properties of each task
// --> the Task Control Block (TCB)
typedef struct
{
  /* period in ticks */
  int period;
  /* ticks until next activation */
  int offset;
  /* function pointer */
  void (*func)(void);
  /* activation counter */
  int exec;
} Sched_Task_t;

// array of Task Control Blocks - TCBs
Sched_Task_t Tasks[MAXT];

// index of currently running task (MAXT to force searching all TCBs initially)
byte cur_task = MAXT;

// kernel initialization routine

int Sched_Init(void)
{
  /* - Initialise data
    structures.
  */
  byte x;
  for (x = 0; x < MAXT; x++)
    Tasks[x].func = 0;
  // note that "func" will be used to see if a TCB
  // is free (func=0) or used (func=pointer to task code)
  /* - Configure interrupt
    that periodically
    calls
    Sched_Schedule().
  */
}

// adding a task to the kernel

int Sched_AddT(void (*f)(void), int d, int p)
{
  byte x;
  for (x = 0; x < MAXT; x++)
    if (!Tasks[x].func)
    {
      // finds the first free TCB
      Tasks[x].period = p;
      Tasks[x].offset = d; //first activation is "d" after kernel start
      Tasks[x].exec = 0;
      Tasks[x].func = f;
      return x;
    }
  return -1; // if no free TCB --> return error
}

// Kernel scheduler, just activates periodic tasks
// "offset" is always counting down, activate task when 0
// then reset to "period"
// --> 1st activation at "offset" and then on every "period"

void Sched_Schedule(void)
{
  byte x;
  for (x = 0; x < MAXT; x++)
  {
    if ((Tasks[x].func) && (Tasks[x].offset))
    {
      // for all existing tasks (func!=0) and not at 0, yet
      Tasks[x].offset--; //decrement counter
      if (!Tasks[x].offset)
      {
        /* offset = 0 --> Schedule Task --> set the "exec" flag/counter */
        // Tasks[x].exec++;  // accummulates activations if overrun UNSUITED TO REAL_TIME
        Tasks[x].exec = 1;                 // if overrun, following activation is lost
        Tasks[x].offset = Tasks[x].period; // reset counter
      }
    }
  }
}

// Kernel dispatcher, takes highest priority ready task and runs it
// calls task directly within the stack scope of the the interrupted (preempted task)
//  --> preemptive -> one-shot model

void Sched_Dispatch(void)
{
  // save current task to resume it after preemption
  byte prev_task;
  byte x;
  prev_task = cur_task; // save currently running task, for the case it is preempted
  for (x = 0; x < cur_task; x++)
  {
    // x searches from 0 (highest priority) up to x (current task)
    if ((Tasks[x].func) && (Tasks[x].exec))
    {
      // if a TCB has a task (func!=0) and there is a pending activation
      Tasks[x].exec--; // decrement (reset) "exec" flag/counter

      cur_task = x; // preempt current task with x (new high priority one)
      interrupts(); // enable interrupts so that this task can be preempted

      Tasks[x].func(); // Execute the task

      noInterrupts();       // disable interrupts again to continue the dispatcher cycle
      cur_task = prev_task; // resume the task that was preempted (if any)

      // Delete task if one-shot, i.e., only runs once (period=0 && offset!0)
      if (!Tasks[x].period)
        Tasks[x].func = 0;
    }
  }
  if (TasksAper[0].exec) {
    TasksAper[0].exec = 0;

    interrupts();

    TasksAper[0].func();

    noInterrupts();
  }
}

//*************** tasks code ******************
// This is the code of the tasks, normally they would something more useful !

void T1()
{
  I2C_write(PortExpLEFT, OLAT + 0x10, 0x00000000);
  I2C_write(PortExpLEFT, OLAT, 0x00000000);
  I2C_write(PortExpCENTER, OLAT + 0x10, 0x00000000);
  I2C_write(PortExpCENTER, OLAT, 0x00000000);
  I2C_write(PortExpRIGHT, OLAT + 0x10, 0x00000000);
  I2C_write(PortExpRIGHT, OLAT, 0x00000000);
}

void T2()
{
  if (first_plane)
  {
    CLK_1cycle();
    digitalWrite(SR_DIN, HIGH);
    first_plane = 0;
    plane_counter = 1;
    loadNextImage(plane_counter);
    CLK_1cycle();
    return;
  }
  else
    digitalWrite(SR_DIN, LOW);

  plane_counter++;

  CLK_1cycle();
  loadNextImage(plane_counter);
  CLK_1cycle();

  if (plane_counter == 6)
  {
    first_plane = 1;
  }
  //Serial.print("clock-> ");
  //Serial.println(plane_counter);
}

void T3()
{
  //PortExpLEFT
  //port B
  I2C_read(PortExpLEFT, GPIO + 0x10, 1);

  if (Wire.available() <= 1) {
    PortExpLEFT_B = Wire.read();
    //Serial.println(PortExpLEFT_B, BIN);
    if (PortExpLEFT_B == 0b10111111 && PortExpLEFT_B_old == 0b11111111) {
      Serial.println("ON!");
      begin_animation = 1;    //transition for expanded cube horizontal
    } else {}

    PortExpLEFT_B_old = PortExpLEFT_B;
  } else {}
}

void T5()
{
  begin_animation = 1;    //transition for expanded cube horizontal
}

void T4()
{
  Serial.write('4');
  delay(80);
}

/*****************  Arduino framework  ********************/

// the setup function runs once when you press reset or power the board
// used to configure hardware resources and software structures

void setup()
{
  Serial.begin(115200);
  Wire.begin(); // Initiate the Wire library
  I2C_write(PortExpLEFT, IOCON, 0b10100000);
  //BANK   = 1 : sequential register addresses
  //MIRROR = 0 : use configureInterrupt
  //SEQOP  = 1 : sequential operation disabled, address pointer does not increment
  //DISSLW = 0 : slew rate enabled
  //HAEN   = 0 : hardware address pin is always enabled on 23017
  //ODR    = 0 : open drain output
  //INTPOL = 0 : interrupt active low

  IOCON = 0x05; //update IOCON addr because of BANK update

  //Initialization of LEDs
  I2C_write(PortExpLEFT, OLAT, 0b11111100);          //Port A
  I2C_write(PortExpLEFT, OLAT + 0x10, 0b00111111);   //Port B
  I2C_write(PortExpCENTER, OLAT, 0b11111100);        //Port A
  I2C_write(PortExpCENTER, OLAT + 0x10, 0b00111111); //Port B
  I2C_write(PortExpRIGHT, OLAT, 0b11111100);         //Port A
  I2C_write(PortExpRIGHT, OLAT + 0x10, 0b00111111);  //Port B

  //I/O configuration -> device 0x24 (LEFT)
  I2C_write(PortExpLEFT, IODIR, 0b00000011);          //Port A
  I2C_write(PortExpLEFT, IODIR + 0x10, 0b11000000);   //Port B
  I2C_write(PortExpCENTER, IODIR, 0b00000011);        //Port A
  I2C_write(PortExpCENTER, IODIR + 0x10, 0b11000000); //Port B
  I2C_write(PortExpRIGHT, IODIR, 0b00000011);         //Port A
  I2C_write(PortExpRIGHT, IODIR + 0x10, 0b11000000);  //Port B

  //Pullup configuration -> device 0x24 (LEFT)
  I2C_write(PortExpLEFT, GPPU, 0b00000011);          //Port A
  I2C_write(PortExpLEFT, GPPU + 0x10, 0b11000000);   //Port B
  I2C_write(PortExpCENTER, GPPU, 0b00000011);        //Port A
  I2C_write(PortExpCENTER, GPPU + 0x10, 0b11000000); //Port B
  I2C_write(PortExpRIGHT, GPPU, 0b00000011);         //Port A
  I2C_write(PortExpRIGHT, GPPU + 0x10, 0b11000000);  //Port B

  pinMode(SR_DIN, OUTPUT);
  pinMode(SR_CLK, OUTPUT);

  pinMode(intPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(intPin), ACTIVATE_APER, FALLING);
  AddTaskAper(T5);

  digitalWrite(SR_DIN, LOW);
  digitalWrite(SR_CLK, LOW);

  for (int i = 0; i < 16; i++)
  {
    CLK_cycle(SR_CLK, 1);
  }

  // serial comms

  // run the kernel initialization routine
  Sched_Init();

  // add all periodic tasks  (code, offset, period) in ticks
  // for the moment, ticks in 2ms -- see below timer frequency
  Sched_AddT(T3, 1, 2);
  Sched_AddT(T1, 1, 0); // highest priority
  Sched_AddT(T2, 1, 2);
  // Sched_AddT(T4, 1, 10);

  noInterrupts(); // disable all interrupts

  // timer 1 control registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // register for the frequency of timer 1
  OCR1A = 125; // compare match register 16MHz/256/500Hz -- tick = 2ms
  //OCR1A = 625; // compare match register 16MHz/256/100Hz -- tick = 10ms
  //OCR1A = 6250; // compare match register 16MHz/256/10Hz
  //OCR1A = 31250; // compare match register 16MHz/256/2Hz
  //OCR1A = 31;    // compare match register 16MHz/256/2kHz
  TCCR1B |= (1 << WGM12);  // CTC mode
  TCCR1B |= (1 << CS12);   // 256 prescaler
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt

  interrupts(); // enable all interrupts
}

//timer1 interrupt service routine (ISR)
ISR(TIMER1_COMPA_vect)
{
  Sched_Schedule(); // invokes the scheduler to update tasks activations
  // invokes the dispatcher to execute the highest priority ready task
  Sched_Dispatch();
}

// the loop function runs over and over again forever
// in this case, it does nothing, all tasks are executed by the kernel
void loop()
{
}

void loadNextImage(int iPlan)
{
  if (begin_animation == 1) {
    I2C_write(PortExpLEFT,  OLAT + 0x10, image[state_animation[state_counter]][iPlan - 1][0]);
    I2C_write(PortExpLEFT,  OLAT     , image[state_animation[state_counter]][iPlan - 1][1]);
    I2C_write(PortExpCENTER, OLAT + 0x10, image[state_animation[state_counter]][iPlan - 1][2]);
    I2C_write(PortExpCENTER, OLAT     , image[state_animation[state_counter]][iPlan - 1][3]);
    I2C_write(PortExpRIGHT, OLAT + 0x10, image[state_animation[state_counter]][iPlan - 1][4]);
    I2C_write(PortExpRIGHT, OLAT     , image[state_animation[state_counter]][iPlan - 1][5]);

    help_counter++;

    if (help_counter == 150) {
      state_counter++;
      help_counter = 0;
    }

    if (state_counter == 8) {
      state_counter = 0;
      begin_animation = 0;
      state = 1;
    }
  }
  else {
    I2C_write(PortExpLEFT,  OLAT + 0x10, image[1][iPlan - 1][0]);
    I2C_write(PortExpLEFT,  OLAT     , image[1][iPlan - 1][1]);
    I2C_write(PortExpCENTER, OLAT + 0x10, image[1][iPlan - 1][2]);
    I2C_write(PortExpCENTER, OLAT     , image[1][iPlan - 1][3]);
    I2C_write(PortExpRIGHT, OLAT + 0x10, image[1][iPlan - 1][4]);
    I2C_write(PortExpRIGHT, OLAT     , image[1][iPlan - 1][5]);
  }
}

void I2C_write(int device, int reg, int val)
{
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void I2C_read(int device, int reg, int n)
{
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(device, n);
}

void CLK_cycle(int clk, int num)
{
  for (int i = 0; i < num; i++)
  {
    digitalWrite(clk, HIGH);
    digitalWrite(clk, LOW);
  }
}

void CLK_1cycle(void)
{
  digitalWrite(SR_CLK, HIGH);
  digitalWrite(SR_CLK, LOW);
}

void ACTIVATE_APER() {
  TasksAper[0].exec = 1;
}

void AddTaskAper(void (*f)(void))
{
  TasksAper[0].exec = 0;
  TasksAper[0].func = f;
}
