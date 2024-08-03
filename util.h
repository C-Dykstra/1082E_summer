#pragma once

class Chassis;

struct Point{
    Point(){x=0;y=0;}
    Point(float pX,float pY):x(pX),y(pY){}
    Point(float pX,float pY,float pTheta):x(pX),y(pY),theta(pTheta){}
    Point(Point basis, float pTheta):x(basis.x),y(basis.y),theta(pTheta),leftVelo(basis.leftVelo),
        rightVelo(basis.rightVelo),instruction(basis.instruction){}


    float x,y;
    float theta=0;
    float leftVelo=0,rightVelo=0;
    int instruction=0;
    double timeToPoint=0; //When used, summative
};

float calc_dist(Chassis& chassis, Point point);

float calc_dist(Point pointOne, Point pointTwo);

Point calcCarrot(Chassis& chassis, Point point, float dLead, float gLead=1);