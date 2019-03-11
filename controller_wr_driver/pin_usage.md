# Использование модулей периферии контроллера

## Tickless mode => GPT5 

## Драйвер управления приводом руля 
Driver | Pins | Input / Output
-------|------|-------
  PWM1 | PE_11 |output (channel 1)

## Драйвер управления приводом движения 
Driver | Pins | Input / Output
-------|------|-------
  PWM1 | PE_9 | output (channel 1)
  PWM1 | PE11 | output (channel 2)
  GPT1 |      |

## Драйвер работы с энкодером
Driver | Pins | Input / Output
-------|------|-------
   EXT |  PD3 | Input (channel 3)
   EXT |  PD4 | Input (channel 4)
   EXT |  PD5 | Input (channel 5)

## Драйвер одометрии
Driver | Pins | Input / Output
-------|------|-------
  GPT2 |	  | 

## Драйвер OC от рулевой сервы
Driver | Pins | Input / Output
-------|------|-------
  ADC1 | PC0  | Input (Channel 10)
  GPT4 |      | 

## Драйвер ручного управления
Driver | Pins | Input / Output
-------|------|-------
ICU9 | E5 | input (Timer 9)
ICU8 | C6 | input (Timer 8)

## Драйвер для СУ приводами
Driver | Pins | Input / Output
-------|------|-------
   VT  |      | steering/speed control

## Драйвер отслеживания заряда батарей
Driver | Pins | Input / Output
-------|------|-------
  ADC3 | 