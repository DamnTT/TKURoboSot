r""" Environment definition enhance the readability of the code 

"""

import math

# math

RAD2DEG = 180/math.pi
DEG2RAD = math.pi/180

# game state

## basic
GAMESTATE_HALT          =   0
GAMESTATE_KICK_OFF      =   1
GAMESTATE_FREE_KICK     =   2
GAMESTATE_PENALTY_KICK  =   3
GAMESTATE_FREE_BALL     =   4
GAMESTATE_THROW_IN      =   5
GAMESTATE_CORNER_KICK   =   6
GAMESTATE_GOAL_KICK     =   7

## special purpose
GAMESTATE_RUN_LOCATION  =   501


# role

## basic
NO_EXISTS               =  -1
ROLE_HALT               =   0
ROLE_GOALKEEPER         =   1  
ROLE_ATTACK             =   2
ROLE_SUPPORT            =   3
ROLE_FREE_KICK          =   4
ROLE_PENALTY_KICK       =   5
ROLE_FREE_BALL          =   6
ROLE_THROW_IN           =   7
ROLE_CORNER_KICK        =   8
ROLE_GOAL_KICK          =   9

## special purpose

ROLE_RUN_LOCATION       =   501


# finite state machine 

FSM_HALT = 1
FSM_BRAKE = 2