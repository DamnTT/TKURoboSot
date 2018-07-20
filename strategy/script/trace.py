#!/usr/bin/env python
# license removed for brevity
import rospy
from vision.msg import Object
from geometry_msgs.msg import Twist
import numpy as np
import math
import skfuzzy.control as ctrl

# Sparse universe makes calculations faster, without sacrifice accuracy.
# Only the critical points are included here; making it higher resolution is
# unnecessary.
universe0 = np.linspace(0, 200, 5) # Object Distance: 0~200 cm
universe1 = np.linspace(-180, 180, 5) # Object Angular: -180~180 degs
universe2 = np.linspace(0, 10, 5) # Velocity: 0~100%
#universe3 = np.linspace(-25, 25, 5) # Angular Velocity: -100~100%
universe3 = np.array([-70, -30, 0, 30, 70]) # Angular Velocity: -100~100%

# Create the three fuzzy variables
dD = ctrl.Antecedent(universe0, 'dD')
dT = ctrl.Antecedent(universe1, 'dT')
oV = ctrl.Consequent(universe2, 'oV')
oW = ctrl.Consequent(universe3, 'oW')

# Here we use the convenience `automf` to populate the fuzzy variables with
# terms. The optional kwarg `names=` lets us specify the names of our Terms.
names0 = ['nb', 'ns',  'ze', 'ps', 'pb']
names1 = ['ze', 'pss', 'ps', 'pb', 'pbb']
dD.automf(names=names1)
dT.automf(names=names0)
oV.automf(names=names1)
oW.automf(names=names0)

"""
Define complex rules
--------------------

This system has a complicated, fully connected set of rules defined below.
"""
rule0 = ctrl.Rule(antecedent=(dD['ze'] & dT['nb']), consequent=(oV['ze'], oW['nb']))
rule1 = ctrl.Rule(antecedent=(dD['ze'] & dT['ns']), consequent=(oV['ps'], oW['ns']))
rule2 = ctrl.Rule(antecedent=(dD['ze'] & dT['ze']), consequent=(oV['pbb'], oW['ze']))
rule3 = ctrl.Rule(antecedent=(dD['ze'] & dT['ps']), consequent=(oV['ps'], oW['ps']))
rule4 = ctrl.Rule(antecedent=(dD['ze'] & dT['pb']), consequent=(oV['ze'], oW['pb']))

rule5 = ctrl.Rule(antecedent=(dD['pss'] & dT['nb']), consequent=(oV['pss'], oW['nb']))
rule6 = ctrl.Rule(antecedent=(dD['pss'] & dT['ns']), consequent=(oV['ps'], oW['ns']))
rule7 = ctrl.Rule(antecedent=(dD['pss'] & dT['ze']), consequent=(oV['pbb'], oW['ze']))
rule8 = ctrl.Rule(antecedent=(dD['pss'] & dT['ps']), consequent=(oV['ps'], oW['ps']))
rule9 = ctrl.Rule(antecedent=(dD['pss'] & dT['pb']), consequent=(oV['pss'], oW['pb']))

rule10 = ctrl.Rule(antecedent=(dD['ps'] & dT['nb']), consequent=(oV['ps'], oW['nb']))
rule11 = ctrl.Rule(antecedent=(dD['ps'] & dT['ns']), consequent=(oV['ps'], oW['ns']))
rule12 = ctrl.Rule(antecedent=(dD['ps'] & dT['ze']), consequent=(oV['pbb'], oW['ze']))
rule13 = ctrl.Rule(antecedent=(dD['ps'] & dT['ps']), consequent=(oV['ps'], oW['ps']))
rule14 = ctrl.Rule(antecedent=(dD['ps'] & dT['pb']), consequent=(oV['ps'], oW['pb']))

rule15 = ctrl.Rule(antecedent=(dD['pb'] & dT['nb']), consequent=(oV['pb'], oW['nb']))
rule16 = ctrl.Rule(antecedent=(dD['pb'] & dT['ns']), consequent=(oV['pb'], oW['ns']))
rule17 = ctrl.Rule(antecedent=(dD['pb'] & dT['ze']), consequent=(oV['pbb'], oW['ze']))
rule18 = ctrl.Rule(antecedent=(dD['pb'] & dT['ps']), consequent=(oV['pb'], oW['ps']))
rule19 = ctrl.Rule(antecedent=(dD['pb'] & dT['pb']), consequent=(oV['pb'], oW['pb']))

rule20 = ctrl.Rule(antecedent=(dD['pbb'] & dT['nb']), consequent=(oV['pbb'], oW['nb']))
rule21 = ctrl.Rule(antecedent=(dD['pbb'] & dT['ns']), consequent=(oV['pbb'], oW['ns']))
rule22 = ctrl.Rule(antecedent=(dD['pbb'] & dT['ze']), consequent=(oV['pbb'], oW['ze']))
rule23 = ctrl.Rule(antecedent=(dD['pbb'] & dT['ps']), consequent=(oV['pbb'], oW['ps']))
rule24 = ctrl.Rule(antecedent=(dD['pbb'] & dT['pb']), consequent=(oV['pbb'], oW['pb']))

my_rules = [rule0,  rule1,  rule2,  rule3,  rule4,
            rule5,  rule6,  rule7,  rule8,  rule9,
            rule10, rule11, rule12, rule13, rule14,
            rule15, rule16, rule17, rule18, rule19,
            rule20,  rule21,  rule22,  rule23,  rule24]
"""
Despite the lengthy ruleset, the new fuzzy control system framework will
execute in milliseconds. Next we add these rules to a new ``ControlSystem``
and define a ``ControlSystemSimulation`` to run it.
"""
system = ctrl.ControlSystem(rules=my_rules)

# Later we intend to run this system with a 21*21 set of inputs, so we allow
# that many plus one unique runs before results are flushed.
# Subsequent runs would return in 1/8 the time!
sim = ctrl.ControlSystemSimulation(system, flush_after_run=21 * 21 + 1)

def fuzzy(dD, dT):
  sim.input['dD'] = dD
  sim.input['dT'] = dT
  sim.compute()

  return(sim.output['oV'], sim.output['oW'])

def fuzzy(dT):
  sim.input['dD'] = dD
  sim.input['dT'] = dT
  sim.compute()

  return(sim.output['oW'])