/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//

#pragma once

#define USE_PP_FOLLOW true

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <limits>

#include "v5.h"
#include "v5_vcs.h"

#include "robot-config.h"
#include "Nerd-Template/vector2.h"
#include "Nerd-Template/pose.h"
#include "Nerd-Template/steer-command.h"
#include "Nerd-Template/waypoint.h"
#include "Nerd-Template/state.h"
#include "Nerd-Template/cubic-bezier.h"

#include "util.h"
#include "async.h"

#include "Nerd-Template/config-structs.h"
#include "Nerd-Template/settle.h"
#include "Nerd-Template/PID.h"
#include "Nerd-Template/velocity-controller.h"

#include "Nerd-Template/smooth-path.h"
#include "Nerd-Template/trajectory.h"
#include "Nerd-Template/odom.h"
#include "Nerd-Template/pure-pursuit.h"
#include "Nerd-Template/ramsete.h"
#include "Nerd-Template/drive.h"
#include "Nerd-Template/path-point.h"

#include "lady-brown.h"
#include "intake.h"
#include "mogo-mech.h"

#include "autons.h"

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)