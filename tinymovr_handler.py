import time
import math
import can
import pint
from tinymovr.tee import init_tee
from tinymovr.config import get_bus_config, create_device

ureg = pint.UnitRegistry()
params = get_bus_config(["canine", "slcan_disco"])
print(params)
time.sleep(3)
params["bitrate"] = 1_000_000
init_tee(can.Bus(**params))
class Actor:
    def __init__(self, node_id = 13, gear_adjustment = 1, direction_adjustment = 1, p_gain = 20, offset = 0):
        time.sleep(0.75)
        self.node_id = node_id
        self.gear_adjustment = gear_adjustment
        self.direction_adjustment = direction_adjustment
        self.offset = offset
        self.tm = create_device( node_id=node_id)
        time.sleep(0.5)
        self.initial_position = self.tm.encoder.position_estimate
        self.zero_reference = self.initial_position.to(ureg.rad)
        print(f"Initial position of Actor {node_id}: {self.initial_position} / {self.zero_reference}")
        self.tm.controller.velocity.limit = 250_000
        self.tm.controller.position.p_gain = p_gain
        self.tm.controller.position.setpoint = self.initial_position.magnitude

    def set_mode(self, mode):
        if mode == "idle":
            self.tm.controller.idle()
        elif mode == "position":
            self.tm.controller.position_mode()

    def set_position(self, position):
        position = position + self.offset
        position2tick = ((self.direction_adjustment * position) / (2 * math.pi)) * 83_886 * self.gear_adjustment
        target_position = (position2tick + self.initial_position.magnitude)
        #print(f'Initialposition {self.initial_position} / position2tick: {position2tick} / target: {target_position}')
        self.tm.controller.position.setpoint = target_position

    def set_position_ticks(self, position):
        target_position = (position + self.initial_position).magnitude
        print(target_position)
        self.tm.controller.position.setpoint = target_position

    def get_position(self):
        current_position = self.tm.encoder.position_estimate.to(ureg.rad) - (self.zero_reference/(83_886 * self.gear_adjustment))
        return current_position

    def get_temperature(self):
        return self.tm.temp

    def get_current(self):
        return self.tm.Ibus

    def get_voltage(self):
        return self.tm.Vbus

    def get_power(self):
        return self.tm.power

    def get_id(self):
        return self.node_id


class Actors:
    def __init__(self):
        self.actors = [None]*12
        self.actors[0] = Actor(node_id=1, gear_adjustment=4, direction_adjustment=1, p_gain=50, offset=0)
        self.actors[1] = Actor(node_id=2, gear_adjustment=1, direction_adjustment=-1, p_gain=50, offset=-1.5707963267949)
        self.actors[2] = Actor(node_id=3, gear_adjustment=1, direction_adjustment=-1, p_gain=50, offset=3.14159265359)
        self.actors[6] = Actor(node_id=4, gear_adjustment=4, direction_adjustment=-1,p_gain=50, offset=0)
        self.actors[7] = Actor(node_id=5, gear_adjustment=1, direction_adjustment=-1,p_gain=50, offset=-1.5707963267949)
        self.actors[8] = Actor(node_id=6, gear_adjustment=1, direction_adjustment=-1,p_gain=50, offset=3.14159265359)
        self.actors[3] = Actor(node_id=7, gear_adjustment=4, direction_adjustment=1, p_gain=50, offset=0)
        self.actors[4] = Actor(node_id=8, gear_adjustment=1, direction_adjustment=1, p_gain=50, offset=-1.5707963267949)
        self.actors[5] = Actor(node_id=9, gear_adjustment=1, direction_adjustment=1, p_gain=50, offset=3.14159265359)
        self.actors[9] = Actor(node_id=10, gear_adjustment=4, direction_adjustment=-1, p_gain=50, offset=0)
        self.actors[10] = Actor(node_id=11, gear_adjustment=1, direction_adjustment=1, p_gain=50, offset=-1.5707963267949)
        self.actors[11] = Actor(node_id=12, gear_adjustment=1, direction_adjustment=1, p_gain=50, offset=3.14159265359)

    def add_actor(self, actor):
        self.actors.append(actor)

    def set_position_mode(self):
        for x in self.actors:
            x.set_mode('position')
            print(f"Actor {x.node_id} active")
            time.sleep(0.25)

    def get_positions(self):
        return [actor.get_position() for actor in self.actors]

    def get_temperatures(self):
        return [actor.get_temperature() for actor in self.actors]

    def get_currents(self):
        return [actor.get_current() for actor in self.actors]

    def get_voltages(self):
        return [actor.get_voltage() for actor in self.actors]

    def set_positions(self, positions):
        """
        Set positions for all actors based on a list of 12 values.
        positions: List of 12 float values representing target positions for each actor.
        """
        if len(positions) != 12:
            raise ValueError("Exactly 12 position values are required.")
        for actor, position in zip(self.actors, positions):
            actor.set_position(position)
            print(f"Actor {actor.node_id}, {position}")

    def stand_up(self):
        self.set_position_mode()
        l2 = [1.57079633, 1.55026663, 1.52973693, 1.50920723, 1.48867754,
              1.46814784, 1.44761814, 1.42708844, 1.40655875, 1.38602905,
              1.36549935, 1.34496965, 1.32443996, 1.30391026, 1.28338056,
              1.26285086, 1.24232117, 1.22179147, 1.20126177, 1.18073207,
              1.16020238, 1.13967268, 1.11914298, 1.09861328, 1.07808359,
              1.05755389, 1.03702419, 1.01649449, 0.9959648, 0.9754351,
              0.9549054, 0.9343757, 0.91384601, 0.89331631, 0.87278661,
              0.85225691, 0.83172722, 0.81119752, 0.79066782, 0.77013812,
              0.74960843, 0.72907873, 0.70854903, 0.68801933, 0.66748964,
              0.64695994, 0.62643024, 0.60590054, 0.58537085, 0.56484115]

        l3 = [-3.14159265, -3.10030538, -3.05901811, -3.01773084, -2.97644357,
              -2.9351563, -2.89386903, -2.85258176, -2.81129449, -2.77000722,
              -2.72871995, -2.68743268, -2.64614541, -2.60485814, -2.56357087,
              -2.5222836, -2.48099633, -2.43970906, -2.39842179, -2.35713452,
              -2.31584725, -2.27455998, -2.23327271, -2.19198544, -2.15069817,
              -2.1094109, -2.06812363, -2.02683636, -1.98554909, -1.94426182,
              -1.90297455, -1.86168728, -1.82040001, -1.77911274, -1.73782547,
              -1.6965382, -1.65525093, -1.61396366, -1.57267639, -1.53138912,
              -1.49010185, -1.44881458, -1.40752731, -1.36624004, -1.32495277,
              -1.2836655, -1.24237823, -1.20109096, -1.15980369, -1.11851642]

        for x, y in zip(l2, l3):
            self.actors[0].set_position(0)
            self.actors[1].set_position(x)
            self.actors[2].set_position(y)

            self.actors[3].set_position(0)
            self.actors[4].set_position(x)
            self.actors[5].set_position(y)

            self.actors[6].set_position(0)
            self.actors[7].set_position(x)
            self.actors[8].set_position(y)

            self.actors[9].set_position(0)
            self.actors[10].set_position(x)
            self.actors[11].set_position(y)

            time.sleep(0.001)  # Delays for 50 milliseconds

"""
actors = Actors()
print(actors.get_voltages())
#actors.stand_up()
actors.set_position_mode()
actors.actors[0].set_position(-0.25488387) #1
actors.actors[3].set_position(0.25488387) #7
actors.actors[6].set_position(-0.25488387) #4
actors.actors[9].set_position(0.25488387) #10


"""



