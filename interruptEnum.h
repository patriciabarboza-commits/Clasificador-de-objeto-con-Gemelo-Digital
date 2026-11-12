#pragma once

// Interrupciones externas del Uno/Nano. El valor es el
// número que recibe attachInterrupt:
//   INT_0 -> pin 2
//   INT_1 -> pin 3
// No pueden llamarse INT0/INT1: avr/io.h ya las define
// como macros y el enum no compilaría
enum interrupt{
    INT_0,
    INT_1
};
