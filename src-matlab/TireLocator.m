function [FLWheelT, FLWheelR, FRWheelT, FRWheelR, BLWheelT, BLWheelR, BRWheelT, BRWheelR] = TireLocator(CGx, CGy, CGz, CarA, CarB, CarDf, CarDr)
CG: x y yaw
ALL IN THE BODY FRAME
Return: Z, Roll, Pitch, Zdot, Rolldot, Pitchdot


Global: x y z yaw Yawdot
Body: Xdot Ydot Zdot roll pitch Delta rolldot pitchdot

Yawdot ~ Delta
end