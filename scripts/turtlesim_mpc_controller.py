from casadi import *

# Declare variables
x = MX.sym("x",2)

# Form the NLP
f = x**2 - x*4 + 4 # objective
f = sqrt(f)

print(f)

MySolver = "ipopt"


