#!/usr/bin/env python
# license removed for brevity
import numpy as np
import math
import skfuzzy.control as ctrl

class Fuzzy():
  def __init__(self, orbit_dis=50, orbit_ang=0):
    self.orbit_dis = orbit_dis
    self.orbit_ang = orbit_ang
    self.setting()
    self.system()

  def setting(self):
    universe0 = np.linspace(-150, 150, 5)      # Delta Destination Distance
    universe1 = np.linspace(-180, 180, 5)      # Delta Destination Angular
    universe2 = np.linspace(-20, 20, 5)        # Velocity
    universe3 = np.array([-50, -20, 0, 20, 50]) # Angular Velocity

    dD = ctrl.Antecedent(universe0, 'dD')
    dT = ctrl.Antecedent(universe1, 'dT')
    oV = ctrl.Consequent(universe2, 'oV')
    oW = ctrl.Consequent(universe3, 'oW')

    names0 = ['nb', 'ns',  'ze', 'ps', 'pb']
    dD.automf(names=names0)
    dT.automf(names=names0)
    oV.automf(names=names0)
    oW.automf(names=names0)

    rule0 = ctrl.Rule(antecedent=(dD['nb'] & dT['nb']), consequent=(oV['nb'], oW['nb']))
    rule1 = ctrl.Rule(antecedent=(dD['nb'] & dT['ns']), consequent=(oV['nb'], oW['ns']))
    rule2 = ctrl.Rule(antecedent=(dD['nb'] & dT['ze']), consequent=(oV['nb'], oW['ze']))
    rule3 = ctrl.Rule(antecedent=(dD['nb'] & dT['ps']), consequent=(oV['nb'], oW['ps']))
    rule4 = ctrl.Rule(antecedent=(dD['nb'] & dT['pb']), consequent=(oV['nb'], oW['pb']))

    rule5 = ctrl.Rule(antecedent=(dD['ns'] & dT['nb']), consequent=(oV['ns'], oW['nb']))
    rule6 = ctrl.Rule(antecedent=(dD['ns'] & dT['ns']), consequent=(oV['ns'], oW['ns']))
    rule7 = ctrl.Rule(antecedent=(dD['ns'] & dT['ze']), consequent=(oV['ns'], oW['ze']))
    rule8 = ctrl.Rule(antecedent=(dD['ns'] & dT['ps']), consequent=(oV['ns'], oW['ps']))
    rule9 = ctrl.Rule(antecedent=(dD['ns'] & dT['pb']), consequent=(oV['ns'], oW['pb']))

    rule10 = ctrl.Rule(antecedent=(dD['ze'] & dT['nb']), consequent=(oV['ze'], oW['nb']))
    rule11 = ctrl.Rule(antecedent=(dD['ze'] & dT['ns']), consequent=(oV['ze'], oW['ns']))
    rule12 = ctrl.Rule(antecedent=(dD['ze'] & dT['ze']), consequent=(oV['ze'], oW['ze']))
    rule13 = ctrl.Rule(antecedent=(dD['ze'] & dT['ps']), consequent=(oV['ze'], oW['ps']))
    rule14 = ctrl.Rule(antecedent=(dD['ze'] & dT['pb']), consequent=(oV['ze'], oW['pb']))

    rule15 = ctrl.Rule(antecedent=(dD['ps'] & dT['nb']), consequent=(oV['ps'], oW['nb']))
    rule16 = ctrl.Rule(antecedent=(dD['ps'] & dT['ns']), consequent=(oV['ps'], oW['ns']))
    rule17 = ctrl.Rule(antecedent=(dD['ps'] & dT['ze']), consequent=(oV['ps'], oW['ze']))
    rule18 = ctrl.Rule(antecedent=(dD['ps'] & dT['ps']), consequent=(oV['ps'], oW['ps']))
    rule19 = ctrl.Rule(antecedent=(dD['ps'] & dT['pb']), consequent=(oV['ps'], oW['pb']))

    rule20 = ctrl.Rule(antecedent=(dD['pb'] & dT['nb']), consequent=(oV['pb'], oW['nb']))
    rule21 = ctrl.Rule(antecedent=(dD['pb'] & dT['ns']), consequent=(oV['pb'], oW['ns']))
    rule22 = ctrl.Rule(antecedent=(dD['pb'] & dT['ze']), consequent=(oV['pb'], oW['ze']))
    rule23 = ctrl.Rule(antecedent=(dD['pb'] & dT['ps']), consequent=(oV['pb'], oW['ps']))
    rule24 = ctrl.Rule(antecedent=(dD['pb'] & dT['pb']), consequent=(oV['pb'], oW['pb']))

    self.my_rules = [rule0,  rule1,  rule2,  rule3,  rule4,
                rule5,  rule6,  rule7,  rule8,  rule9,
                rule10, rule11, rule12, rule13, rule14,
                rule15, rule16, rule17, rule18, rule19,
                rule20,  rule21,  rule22,  rule23,  rule24]

  def system(self):
    self.system = ctrl.ControlSystem(rules=self.my_rules)
    self.sim = ctrl.ControlSystemSimulation(self.system, flush_after_run=21 * 21 + 1)

  def fuzzy(self, D = 0, T = 0):
    dD = D - self.orbit_dis
    dT = T - self.orbit_ang
    if dD < -200:
      print("catch")
      dD = -200
    elif dD > 200:
      print("catch")
      dD = 200
    elif dT < -180:
      print("catch")
      dT = -180
    elif dT > 180:
      print("catch")
      dT = 180
    self.sim.input['dD'] = dD
    self.sim.input['dT'] = dT
    self.sim.compute()

    return(self.sim.output['oV'], self.sim.output['oW'])
