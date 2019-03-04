int SRead[];

typedef struct 
{
    //the Gcode Standard form : G X Y and Z
    int G[];
    int X[];
    int Y[];
    int z[]; 
    //the Compatible Gcode User-defined code RS
    int RS[];
}GcodeRead_t;

GcodeRead_t GcodeRead;
