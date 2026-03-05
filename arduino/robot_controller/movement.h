#ifndef MOVEMENT_H
#define MOVEMENT_H

void initMovement();
void move_motors(int leftPWM, int rightPWM);
void moveForward(int speed);
void stopMovement();
void turnLeft(int speed);
void turnRight(int speed);

#endif
