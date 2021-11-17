#include <murAPI.hpp>
#include <thread>
#include <PID.h>
#include <functions.h>

int mode = 0;

//green - 36, 77, 130, 255, 0, 255
//yellow - 27, 41, 149, 255, 0, 255
//triangle - 41, 72, 50, 255, 161, 255
Object obj;

int main() 
{
    pid_dir.SetOutputLimits(-180, 180);
    pid_dir.SetMode(AUTOMATIC);
    pid_depth.SetOutputLimits(-100, 100);
    pid_depth.SetMode(AUTOMATIC);
    mur.addDetectorToList(Object::RECTANGLE, 0);
    mur.addDetectorToList(Object::CIRCLE, 1);
    if(mode == 0)
        set(0, 40, 10);
    bool saw = 0;
    while(true)
    {
        switch(mode)
        {
            case 0:
                keepAngle();
                keepDepth();
                for (const auto &obj : mur.getDetectedObjectsList(0)) 
                {
                    if (obj.type == Object::RECTANGLE) 
                    {
                        set(330, 100, 0);
                        mur.setPortA(-5);
                        mur.setPortB(5);
                        mode = 1;
                    }
                }
            break;
            case 1:
                obj = detectColor(mur.getCameraTwoFrame(), 36, 77, 130, 255, 0, 255);
                if (obj.x != -1) 
                {
                    set_to_obj(obj, 5, 100);
                    saw = 1;
                }
                if(saw && obj.x == -1)
                {
                    set(0, 100, -10);
                    mode = 2;
                    saw = 0;
                    t.stop();
                    t.start();
                    while(t.elapsed() <= 2000)
                    {
                        keepDepth();
                        keepAngle();
                    }
                    set(330, 100, 0);
                }
            break;
            case 2:
                obj = detectColor(mur.getCameraTwoFrame(), 0, 0, 153, 255, 116, 255);
                set_to_obj(obj, 5, 100);
                if (obj.x != -1 && !saw) 
                {
                    saw = 1;
                }
                if(saw && obj.x == -1)
                {
                    mode = 3;
                    saw = 0;
                    set(0, 100, -10);
                    t.stop();
                    t.start();
                    while(t.elapsed() <= 3000)
                    {
                        keepDepth();
                        keepAngle();
                    }
                    set(330, 100, 0);
                }
            break;
            case 3:
                obj = detectColor(mur.getCameraTwoFrame(), 27, 41, 149, 255, 0, 255);
                set_to_obj(obj, 5, 100);
                if(obj.size >= 150)
                {
                    mur.shoot();
                    mode = 4;
                    set(0, 130, 20);
                    t.stop();
                    t.start();
                    while(t.elapsed() <= 2000)
                    {
                        keepDepth();
                        keepAngle();
                    }
                    set(0, 70, 10);
                }
            break;
            case 4:
                keepAngle();
                keepDepth();
                if (set_to_line() != -1) 
                {
                    needSpeed = 10;
                    mode = 5;
                }
            break;
            case 5:
                obj = detectColor(mur.getCameraOneFrame(), 41, 72, 50, 255, 161, 255);
                if(obj.x != -1)
                    mode = 6;
                keepDepth();
                keepAngle();
            break;
            case 6:
                obj = detectColor(mur.getCameraOneFrame(), 41, 72, 50, 255, 161, 255);
                if(abs(obj.x - 160) < 5 && abs(obj.y - 120) < 5)
                {
                    mur.setPortA(0);
                    mur.setPortB(0);
                    mur.setPortD(0);
                    mur.drop();
                    set(0, 60, 10);
                    mode = 7;
                }
                else
                    set_to_obj_f(obj, 70);
            break;
            case 7:
                if (set_to_line() != -1) 
                {
                    set(mur.getYaw(), 50, 10);
                    mode = 8;
                }
                else
                {
                    keepDepth();
                    keepAngle();
                }
            break;
            case 8:
                obj = detectColor(mur.getCameraOneFrame(), 22, 31, 163, 255, 88, 117);
                if(obj.x != -1)
                {
                    set_to_obj_f(obj, 40);
                }
                else
                {
                    keepDepth();
                    keepAngle();
                }
                if(abs(obj.x - 160) < 5 && abs(obj.y - 120) < 5)
                {
                    mur.setPortA(0);
                    mur.setPortB(0);
                    mur.setPortD(0);
                    std::cout << "End" << std::endl;
                    setDepth(180);
                }
            break;
        }
        
    }
    return 0;
}