#include<Arduino.h>
#include<string.h>
#include<GcodeReading.h>
#include<iostream>
#include<iterator>
/*
该项目的指令接收频率不高，只需简单的读取缓冲区内存
*版本:1-0-0
*修改日志
*/
int GcodeGoalCoordinate[3]; //Gcode X Y Z
int GGState; //Gcode G00 G28 G04 nocommad
                    //       1   2   3    0
int SReadN;
int SReadSums = SRead.sive();
int i = 0;
int sumsget(int somethings[])
{
    int *e = end(SRead);
    for(int b = 0; b <= SReadSums; b++)
    {

    }
}


void GcodeDiscern()
{
    for(int a = 0; a < SRead.size(); a ++)
    {
    }
}
void GcodeGet()
{
    if(Serial.available > 0 && GGState == 0)
    {
        SReadN = Serial.read();
        if(SReadN == 'G')
        {
            SRead[i] = SReadN;
            i ++;
        }
        if(SReadN == '\n')
        {
            i == 0;
            GGState == 1;
        }
    }
}

void GcodeCut()
{
    if(GGState == 1)
}