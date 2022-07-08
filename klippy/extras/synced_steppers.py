# Start and move multiple [manual] steppers at the same time
# Note, using this feature may place the printer in an invalid state
#
# Copyright (C) 2022 Alexander Nagelberg <alex@alexnagelberg.com>
#
# This file may be distributed under the terms of the GNU GPLv2 license.

import chelper
from . import force_move

class _Mover:
  def __init__(self, stepper):
    ffi_main, ffi_lib = chelper.get_ffi()
    self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
    self.trapq_append = ffi_lib.trapq_append
    self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
    self.stepper_kinematics = ffi_main.gc(ffi_lib.cartesian_stepper_alloc(b'x'), ffi_lib.free)
    self.stepper = stepper["stepper"]
    self.dist = stepper["dist"]
    self.speed = stepper["speed"]
    self.accel = stepper["accel"]
  def set_stepper_kinematics(self, sk = None):
    return self.stepper.set_stepper_kinematics(sk or self.stepper_kinematics)
  def set_trapq(self, tq = None):
    return self.stepper.set_trapq(tq or self.trapq)
  def calc_move_time(self):
    return force_move.calc_move_time(self.dist, self.speed, self.accel)
  def do_trapq_append(self, move_times, print_time):
    self.trapq_append(self.trapq, print_time, move_times[1], move_times[2], move_times[1], 0., 0., 0.,
      move_times[0], 0., 0., 0., move_times[3], self.accel)
  def generate_steps(self, move_times, print_time):
    _print_time = print_time + move_times[1] + move_times[1] + move_times[2]
    self.stepper.generate_steps(_print_time)
    return _print_time
  def finalize_moves(self, print_time):
    self.trapq_finalize_moves(self.trapq, print_time + 99999.9)

class SyncedSteppers:
  def __init__(self, config):
    self.printer = config.get_printer()
    gcode = self.printer.lookup_object('gcode')
    gcode.register_command('MOVE_SYNC', self.cmd_MOVE_SYNC, desc="Moves STEPPER[n] at the same time")
    self.printer.register_event_handler("klippy:connect", self.handle_connect)

  def move_steppers(self, steppers, wait=True):
    movers = list(map(lambda stepper : _Mover(stepper), steppers))
    self._toolhead.flush_step_generation()

    prev_sk = list(map(lambda x : x.set_stepper_kinematics(), movers))
    prev_trapq = list(map(lambda x : x.set_trapq(), movers))
    [stepper["stepper"].set_position((0., 0., 0.)) for stepper in steppers]
    move_times = list(map(lambda x : x.calc_move_time(), movers))
    print_time = self._toolhead.get_last_move_time()
    [movers[i].do_trapq_append(move_times[i], print_time) for i in range(len(movers))]
    print_times = [movers[i].generate_steps(move_times[i], print_time) for i in range(len(movers))]
    [movers[i].finalize_moves(print_times[i]) for i in range(len(movers))]
    [movers[i].set_trapq(prev_trapq[i]) for i in range(len(prev_trapq))]
    [movers[i].set_stepper_kinematics(prev_sk[i]) for i in range(len(prev_sk))]

    for i in range(len(print_times)):
      self._toolhead.note_kinematic_activity(print_times[i])
    self._toolhead.dwell(0)
    if wait: self._toolhead.wait_moves()

  def cmd_MOVE_SYNC(self, gcmd):
    steppers = []
    for parameter in gcmd.get_command_parameters():
      if parameter.startswith('STEPPER') and parameter[7:].isnumeric():
        steppers.append(gcmd.get(parameter))

    steppers = [{
      'stepper': self.printer.lookup_object(stepper).get_steppers()[0],
      'dist': gcmd.get_int('DIST'),
      'speed': gcmd.get_int('SPEED', default=0),
      'accel': gcmd.get_int('ACCEL', default=0)
    } for stepper in steppers]

    self.move_steppers(steppers)

def load_config(config):
  return SyncedSteppers(config)
