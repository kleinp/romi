m = 0.05    # kg
r = 0.082   # meters
h = 0.040   # meters

Ix = m/12*(3*pow(r, 2) + pow(h, 2))
Iy = Ix
Iz = m*pow(r, 2)

print("Ix = %0.6f" % Ix)
print("Iy = %0.6f" % Iy)
print("Iz = %0.6f" % Iz)
