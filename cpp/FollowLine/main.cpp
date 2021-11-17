#include <murAPI.hpp>
#include <thread>
#include <PID.h>
#include <functions.h>

using namespace cv;

//green - 36, 77, 130, 255, 0, 255
//yellow - 27, 41, 149, 255, 0, 255
//triangle - 41, 72, 50, 255, 161, 255
Object obj;
int mode = 0;

int main() 
{
    pid_dir.SetOutputLimits(-180, 180);
    pid_dir.SetMode(AUTOMATIC);
    pid_depth.SetOutputLimits(-100, 100);
    pid_depth.SetMode(AUTOMATIC);
    mur.addDetectorToList(Object::RECTANGLE, 0);
    mur.addDetectorToList(Object::CIRCLE, 1);
    //setDepth(100);
    
    while(true)
    {
        /*
        if(mode == 0)
        {
            keepDepth();
            obj = detectColor(mur.getCameraOneFrame(), 19, 44, 219, 255, 89, 138);
            if(obj.angle != -1)
                set_to_obj_f(obj);
            if(abs(obj.x - 160) <= 5 && abs(obj.y - 120) <= 5)
                std::cout << "Succeed" << std::endl;
        }*/
        if(mode == 0)
        {
            set(0, 100, 40);
            mode = 1;
        }
        if(mode == 1)
        {
            
            if(followLine() == -1)
            {
                keepDepth();
                keepAngle();
            }
            obj = detectColor(mur.getCameraOneFrame(), 19, 44, 219, 255, 89, 138);
            if(obj.angle != -1)
            {
                std::cout << vertex() << std::endl;
                mur.setPortA(0);
                mur.setPortB(0);
                mode = 2;
            }
        }
        if(mode == 2)
        {
            obj = detectColor(mur.getCameraOneFrame(), 19, 44, 219, 255, 89, 138);
            std::cout << vertex() << std::endl;
            set_to_obj_f(obj);
            keepDepth();
            if(abs(obj.x - 160) <= 10 && abs(obj.y - 120) <= 10)
            {

                if(vertex() == 5)
                {
                    mur.setPortD(0);
                    mur.drop();
                    
                    mode = 21;
                }
                else if(vertex() == 4)
                {
                    mur.setPortD(0);
                    mode = 21;
                }
            }
            
        }
        if(mode == 21)
        {
            mur.setPortD(0);
            set(mur.getYaw(), 100, 0);
            t.stop();
            t.start();
            while(t.elapsed() <= 5000)
            {
                followLine();
                sleepFor(50);
            }
            mode = 3;
        }
        if(mode == 3)
        {
            
            followLine();
            obj = detectColor(mur.getCameraOneFrame(), 19, 44, 219, 255, 89, 138);
            if(obj.angle != -1)
            {
                mur.setPortA(0);
                mur.setPortB(0);
                mode = 4;
            }
        }
        if(mode == 4)
        {
            obj = detectColor(mur.getCameraOneFrame(), 19, 44, 219, 255, 89, 138);
            //std::cout << obj.x << std::endl;
            set_to_obj_f(obj);
            keepDepth();
            std::cout << vertex() << std::endl;
            if(abs(obj.x - 160) <= 2 && abs(obj.y - 120) <= 20)
            {
                if(vertex() == 5)
                {
                    mur.drop();
                    mur.setPortD(0);
                    set(320, 100, 0);
                    t.stop();
                    t.start();
                    while(t.elapsed() <= 5000)
                    {
                        keepDepth();
                        obj = detectColor(mur.getCameraTwoFrame(), 58, 74, 202, 255, 150, 192);
                        set_to_obj(obj, 0, 100, 0);
                        mur.setPortD(-15);
                    }
                    mode = 5;
                }
                else if(vertex() == 4)
                {
                    mur.setPortD(0);
                    set(320, 100, 0);
                    t.stop();
                    t.start();
                    while(t.elapsed() <= 5000)
                    {
                        keepDepth();
                        obj = detectColor(mur.getCameraTwoFrame(), 58, 74, 202, 255, 150, 192);
                        set_to_obj(obj, 0, 100, 0);
                        mur.setPortD(-15);
                    }
                    mode = 5;
                }
            }
        }
        if(mode == 5)
        {
            
            std::cout << "mode 5" << std::endl;
            obj = detectColor(mur.getCameraTwoFrame(), 58, 74, 202, 255, 150, 192);
            if(obj.x != -1)
            {
                mur.setPortD(0);
                set_to_obj(obj, -5, 100);
            }
            else
            {
                keepDepth();
                mur.setPortD(-15);
            }
            if(abs(obj.x - 160) < 5 && abs(obj.y - 120) < 5)
            {
                Object obj1 = target(0);
                if(obj1.x != -1)
                {
                    Setpoint_depth = 168;
                    mode = 6;
                }

            }
        }
        if(mode == 6)
        {
            //std::cout << "mode 6" << std::endl;
            obj = target(0);
            set_to_obj(obj, 5, 100, 0);
            keepDepth();
            if(obj.x == -1)
            {
                mur.shoot();
                mur.setPortA(0);
                mur.setPortB(0);
                mode = 99;
                set((int)mur.getYaw(), 60, 30);
                t.stop();
                t.start();
                while(t.elapsed() <= 5000)
                {
                    keepDepth();
                    keepAngle();
                }
                set(145, 140, -5);
                mur.setPortA(0);
                mur.setPortB(0);
                mur.setPortD(0);
                mode = 13;
            }
        }
        if(mode == 13)
        {
            std::cout << "mode 13" << std::endl;
            obj = detectColor(mur.getCameraTwoFrame(), 28, 34, 232, 255, 83, 105);
            if(obj.x != -1)
            {
                set_to_obj(obj, -10, 100);
            }
            if(abs(obj.x - 160) < 5 && abs(obj.y - 120) < 5)
            {
                Object obj1 = target(1);
                if(obj1.x != -1)
                    mode = 14;

            }
        }
        if(mode == 14)
        {
            t.stop();
            t.start();
            while(t.elapsed() < 4000)
            {
                obj = detectColor(mur.getCameraTwoFrame(), 28, 34, 232, 255, 83, 105);
                if(obj.x != -1)
                {
                    set_to_obj(obj, -5, 100);
                }
            }
            mode = 141;
        }
        if(mode == 141)
        {
            
            std::cout << "mode 141" << std::endl;
            obj = target(1);
            set_to_obj(obj, 5, 100);
            if(obj.x == -1)
            {
                mur.shoot();
                mur.setPortA(0);
                mur.setPortB(0);
                mode = 15;
                set(0, 90, 0);
            }
        }
        if(mode == 15)
        {
            std::cout << "mode 15" << std::endl;
            followLine();
            obj = detectColor(mur.getCameraOneFrame(), 19, 44, 219, 255, 89, 138);
            if(obj.angle != -1)
            {
                std::cout << vertex() << std::endl;
                mur.setPortA(0);
                mur.setPortB(0);
                mur.setPortD(0);
                setDepth(100);
                mode = 16;
            }
        }
        if(mode == 16)
        {
            std::cout << "mode 16" << std::endl;
            obj = detectColor(mur.getCameraOneFrame(), 19, 44, 219, 255, 89, 138);
            set_to_obj_f(obj);
            keepDepth();
            std::cout << vertex() << std::endl;
            if(abs(obj.x - 160) <= 20 && abs(obj.y - 120) <= 20)
            {
                if(vertex() == 5)
                {
                    mur.setPortD(0);
                    mur.drop();
                    set(0, 90, 0);
                    mode = 18;
                }
                else if(vertex() == 4)
                {
                    mur.setPortD(0);
                    set(0, 90, 0);
                    mode = 18;
                }
            }
        }
        if(mode == 18)
        {
            t.stop();
            t.start();
            while(t.elapsed() < 8000)
            {
                followLine();
            }
            mode = 19;
        }
        if(mode == 19)
        {
            followLine();
            obj = detectColor(mur.getCameraOneFrame(), 27, 31, 239, 255, 91, 106);
            if(obj.x != -1)
                mode = 20;
        }
        if(mode == 20)
        {
            obj = detectColor(mur.getCameraOneFrame(), 27, 31, 239, 255, 91, 106);
            set_to_obj_f(obj);
            if(abs(obj.x - 160) < 20 && abs(obj.y - 120) < 20)
            {
                set(0, 200, 0);
                mode = 9999;
            }
        }
        /*
        imshow("", img);
        waitKey(1);
        */
        
    }
    return 0;
}