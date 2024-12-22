#pragma once
#include "vex.h"
#include <functional>


// usage: Function $return_type { //function body }
#define Function [&]() -> 

/* 
 * usages: 
 * run_after(Function bool {return [condition];}, Function void {[insert function]});
 * run_after(condition, function); // Note that these are functions that were declared earlier in the code. They should not have any arguments.
 */
#define run_after(condition, function) \
  task([]() -> int {                   \
    while(!condition())                \
      wait(10, msec);                  \
    function();                        \
    return 0;                          \
  });

/* DECLARE NON-DRIVE RELATED FUNCTIONS HERE
 */



