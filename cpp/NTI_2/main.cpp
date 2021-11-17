#include <murAPI.hpp>
#include <thread>
#define WHITE 0
#define GREEN 1
#define RED 2
#define ORANGE 3
#define KOFF 0.2
#define KP 7
using namespace cv;
Timer timer;
Timer t;
Timer timer_rot;
bool findBox = 0, dir = 0, gate = 0, right_gate, g_pos;
int err, angle_local, angle = 0, color = -1, mode = 0, pos = 0, doit = 0;
int map(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
int rotate(int val)
{
  if(val > 359)return (val -= 359);
  if(val < 0) return (val += 359);
  else return(val);
}
int rotate_line(float obj_ang, int r_ang)
{
    if(r_ang >= 180)
        return (obj_ang - map(r_ang, 180, 359, 180, 0));
    else 
        return (obj_ang - map(r_ang, 0, 180, 180, 0));
}
Object detectGate (cv:: Mat img) {
    cv::Scalar lower(39, 80, 190);
    cv::Scalar upper(68, 225, 255);
    cv::cvtColor(img, img, CV_BGR2HSV);
    cv::inRange(img, lower, upper, img);
    std::vector<std::vector<cv::Point>>contours;
    cv::findContours(img, contours, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    Object objectToRet;
    
    for (std::size_t i = 0; i < contours.size(); i++) {
        if (contours.at(i).size() < 100) {
            continue;
        }
        if (std::fabs(cv::contourArea(contours.at(i))) < 100.0) {
            continue;
        }
    
        cv::RotatedRect bEllipse = cv::fitEllipse(contours.at(i));
        objectToRet.x = (int) bEllipse.center.x;
        objectToRet.y = (int) bEllipse.center.y;
        objectToRet.angle = bEllipse.angle;
        objectToRet.type = Object::RECTANGLE;
        return objectToRet;
    }
    return objectToRet;
}
int yaw2line(int val)
{
  if(val > 179)return (val -= 179);
  if(val < 0) return (val += 179);
  else return(val);
}

int constrain(int x, int a, int b) {
    if(x < a) {
        return a;
    }
    else if(b < x) {
        return b;
    }
    else
        return x;
}
void keepDepth(int hold)
{
    mur.setPortC(constrain(abs(constrain(hold - mur.getInputAOne(), 0, 100) * 10), 0, 100));
}
void setup(int hold = 0, int depth = -1);
void setup(int hold, int depth)
{
  while(true)
  {
    int angle_local = mur.getYaw();
    angle_local = rotate(angle_local - hold);
    if (depth != -1)
        keepDepth(depth);
    if(angle_local > 0 && angle_local <= 359)
    {
      timer.stop();
      timer.start();
      if(angle_local < 180)
      {
        err = map(constrain(angle_local, 0, 80), 0, 80, 1, 10);
        //std :: cerr << err << " " << angle_local << std :: endl;
        mur.setPortA(constrain(err, -100, 100));
        mur.setPortB(constrain(-err, -100, 100));
      }
      else if(angle_local >= 180)
      {
        err = map(constrain(angle_local, 280, 359), 359, 280, 1, 10);
        mur.setPortA(constrain(-err, -100, 100));
        mur.setPortB(constrain(err, -100, 100));
      }
    }
    else 
    {
      //timer.start();
      mur.setPortA(0);
      mur.setPortB(0);
    }
    if(timer.elapsed() >= 2000)
    {
        //timer.stop();
        return;
    }
    //std :: cerr << timer.elapsed() << std :: endl;
    //std :: cerr << angle_local << std :: endl;
  }
}

void setDepth(int depth)
{
    while(true)
    {
        mur.setPortC(constrain(depth - mur.getInputAOne() + 5, -100, 100) * 3);
        if(abs(depth - mur.getInputAOne()) >= 5)
        {
            timer.stop();
            timer.start();
        }
        if(timer.elapsed() >= 2000)
        {
            mur.setPortC(0);
            break;
        }
    }
}

void forward(int hold, int sp)
{
  int angle_local = mur.getYaw();
        angle_local = rotate(angle_local - hold);
          if(angle_local < 180)
          {
            err = map(constrain(angle_local, 0, 80), 0, 80, 1, 5);
            mur.setPortA(constrain(err - sp, -100, 100));
            mur.setPortB(constrain(- sp, -100, 100));
          }
          else if(angle_local >= 180)
          {
            err = map(constrain(angle_local, 280, 359), 359, 280, 1, 5);
            mur.setPortA(constrain(- sp, -100, 100));
            mur.setPortB(constrain(err - sp, -100, 100));
          }
          else
          {
              mur.setPortA(constrain(- sp, -100, 100));
              mur.setPortB(constrain(- sp, -100, 100));       
          }
}
VideoCapture m_capOne;
Mat img;
Mat img_2;

int getColor(int x, int y)
{
    unsigned char * p = img.ptr(y, x); // Y first, X after
    int r = p[2];
    int g = p[1];
    int b = p[0];
    if((r >= 110 && r <= 140) && g <= 10 && b <= 10)
        return(RED);
    else if(r <= 10 && (g >= 85 && g <= 115) && b <= 10)
        return(GREEN);
    else if((r >= 105 && r <= 135) && (g >= 15 && g <= 45) && b <= 10)
        return(ORANGE);
    else if((r >= 110 && r <= 140) && (g >= 110 && g <= 140) && (b >= 110 && b <= 140))
        return(WHITE);
    else
        return(-1);
}
int findBox_ = 0;
void m_3()
{
    for (const auto &obj : mur.getDetectedObjectsList(0))
            {
                if (obj.type == Object::RECTANGLE)
                {
                    if(mur.getYaw() <= 45 && mur.getYaw() >= 15 && (obj.angle <= 15 || obj.angle > 165))
                    {
                        pos = 0;
                        setup(30, 40);
                        angle = 30;
                        timer_rot.stop();
                        timer_rot.start();
                        while(timer_rot.elapsed() <= 4000)
                            forward(angle, 10);
                        timer_rot.stop();
                        mode = 4;
                    }
                    else if(mur.getYaw() >= 315 && mur.getYaw() <= 345 && (obj.angle <= 15 || obj.angle > 165))
                    {
                        pos = 1;
                        setup(315, 40);
                        angle = 315;
                        timer_rot.stop();
                        timer_rot.start();
                        while(timer_rot.elapsed() <= 4000)
                            forward(angle, 10);
                        timer_rot.stop();
                        mode = 4;
                    }
                    else
                        pos = 2;
                    //std::cerr << pos << "  "<< obj.x << "  "<< obj.y << "  " << int(obj.angle);
                    circle(img, Point(obj.x, obj.y), 5, Scalar(0, 255, 0), CV_FILLED, 8, 0);
                }
            }
            keepDepth(40);
}
void m_4()
{
    forward(angle, 5);
            for (const auto &obj : mur.getDetectedObjectsList(0))
            {
                if (obj.type == Object::RECTANGLE)
                {
                    std::cerr << obj.y;
                    if(obj.y >= 100)
                    {
                        setup(0, 40);
                        mode = 6;
                    }
                }
            }
            keepDepth(40);
}
void drop()
{
    if(findBox)
        {
            /*
            for (const auto &obj : mur.getDetectedObjectsList(0))
            {
                if (obj.type == Object::RECTANGLE)
                {
                    color = getColor(obj.x, obj.y);
                    if(color == 1)
                    {
                        forward(90, 0);//stop
                        //findBox = 0;    !!!!!!!!!!!!!!!!!!!!!!!!
                        setup(0, 20);
                        timer_rot.stop();
                    }
                    circle(img, Point(obj.x, obj.y), 5, Scalar(0, 255, 0), CV_FILLED, 8, 0);
                }
            }*/
            color = getColor(160, 120);
            if(color == 1)
            {
                forward(90, 0);
                //findBox = 0;
                setup(0, 125);
                std::cerr << "drop";
                mur.drop();
                timer_rot.stop();
                mode = 7;
                return;
            }
            if(timer_rot.elapsed() < 10000 && dir == 0)
            {
                forward(90, 10);
                keepDepth(100);
            }
            else if(timer_rot.elapsed() < 20000 && dir == 1)
            {
                forward(90, -10);
                keepDepth(100);
            }
            else
            {
                dir = !dir;
                timer_rot.stop();
                timer_rot.start();
            }
        }
        else
        {
            if(!findBox_)
            {
                forward(0, 20);
                keepDepth(20);
            }
            else
            {
                forward(0, 5);
                keepDepth(30);
            }
            
            for (const auto &obj : mur.getDetectedObjectsList(0))
            {
                if (obj.type == Object::RECTANGLE)
                {
                    color = getColor(obj.x, obj.y);
                    if(color == 0 && findBox_ && obj.y >= 100)
                    {
                        setup(90, 100);
                        findBox = 1;
                        img = mur.getCameraOneFrame();
                        color = getColor(160, 180);
                        if(color == 1)
                        {
                            timer_rot.stop();
                            timer_rot.start();
                            while(timer_rot.elapsed() <= 7000)
                            {
                                forward(90, -10);
                                keepDepth(100);
                            }
                            timer_rot.stop();
                        }
                        else
                        {
                            color = getColor(160, 40);
                            if(color == 1)
                            {
                                timer_rot.stop();
                                timer_rot.start();
                                while(timer_rot.elapsed() <= 4000)
                                {
                                    forward(90, -10);
                                    keepDepth(100);
                                }
                                timer_rot.stop();
                            }
                        }
                        
                        timer_rot.stop();
                        timer_rot.start();
                    }
                    else if(color == 0 && !findBox_)
                    {
                        findBox_ = 1;
                    }
                    circle(img, Point(obj.x, obj.y), 5, Scalar(0, 255, 0), CV_FILLED, 8, 0);
                }
            }
        }
}
void m_7()
{
    forward(0, 15);
    keepDepth(40);
    for (const auto &obj : mur.getDetectedObjectsList(0))
    {
        if (obj.type == Object::TRIANGLE)
        {
            mode = 8;
        }
    }
}
void m_8()
{
    forward(0, 0);
    keepDepth(20);
    for (const auto &obj : mur.getDetectedObjectsList(0))
    {
        if (obj.type == Object::TRIANGLE && obj.y <= 130)
        {
            setup(0, 0);
            mode = 8;
        }
    }
}
int main() 
{
    mur.initCamera(0);
    mur.initCamera(1);
    mur.addDetectorToList(Object::RECTANGLE, 1);
    mur.addDetectorToList(Object::RECTANGLE, 0);
    mur.addDetectorToList(Object::TRIANGLE, 0);
    
    sleepFor(1000);
    img_2 = mur.getCameraTwoFrame();
    //setup(0, 20);//angle(, depth)
    Object obj = detectGate(img_2);
    if (obj.type == Object::RECTANGLE) 
    {
        g_pos = obj.x >= 120;
        circle(img_2, Point(obj.x, obj.y), 10, Scalar(0, 255, 0), CV_FILLED, 8, 0);
    }
    if(!g_pos)
        setup(270, 80);
    else
        setup(90, 80);
    //imshow("img_2", img_2);
    t.start();
    std::cerr << g_pos;
    while(true)
    {
        img = mur.getCameraOneFrame();
        img_2 = mur.getCameraTwoFrame();
        if(mode == 0 && !g_pos)
        {
            keepDepth(80);
            if(doit == 0)
            {
                if(t.elapsed() <= 2000)
                    forward(270, 15);
                else
                {
                    doit = 1;
                    setup(0, 80);
                    
                }
            }
            if(doit == 1)
            {
                forward(0, 20);
                for (const auto &obj : mur.getDetectedObjectsList(0))
                {
                    if (obj.type == Object::RECTANGLE)
                    {
                        mode = 3;
                        setup(30, 40);
                        t.stop();
                        t.start();
                        while(t.elapsed() <= 1000)
                            forward(30, -20);
                    }
                }
            }
        }
        else if(mode == 0 && g_pos)
        {
            keepDepth(80);
            if(doit == 0)
            {
                if(t.elapsed() <= 3000)
                    forward(90, 15);
                else
                {
                    doit = 1;
                    setup(0, 80);
                    
                }
            }
            if(doit == 1)
            {
                forward(0, 20);
                for (const auto &obj : mur.getDetectedObjectsList(0))
                {
                    if (obj.type == Object::RECTANGLE)
                    {
                        mode = 3;
                        setup(330, 40);
                        t.stop();
                        t.start();/*
                        while(t.elapsed() <= 1000)
                        {
                            forward(350, -20);
                            for (const auto &obj : mur.getDetectedObjectsList(0))
                            {
                                if (obj.type == Object::RECTANGLE)
                                {
                                    break;
                                }
                                else
                                    forward(350, -20);
                            }
                        }*/
                    }
                }
            }
        }
        if(mode == 3)
        {
            m_3();
        }
        if(mode == 4)
        {
            m_4();
        }
        if(mode == 6)
        {
            drop();
        }
        else if(mode == 7)
        {
            m_7();
        }
        if(mode == 8)
        {
            m_8();
        }
        circle(img, Point(160, 180), 3, Scalar(0, 255, 0), CV_FILLED, 8, 0);
        circle(img, Point(160, 40), 3, Scalar(0, 255, 0), CV_FILLED, 8, 0);
        circle(img, Point(160, 120), 2, Scalar(0, 0, 255), CV_FILLED, 8, 0);
        //imshow("img_1", img);
        //imshow("img_2", img_2);
        if(waitKey(30) >= 0) break;
    }
}