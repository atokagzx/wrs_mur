#include <murAPI.hpp>
#include <thread>
#include <PID.h>

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


double Setpoint_dir = 180, Input_dir, Output_dir;
double Setpoint_depth = 180, Input_depth, Output_depth;
int angle_of_go = 0, speed_of_go = 0;
bool walking = 0;
PID pid_dir(&Input_dir, &Output_dir, &Setpoint_dir, 0.95, 0, 0.96, DIRECT);
PID pid_depth(&Input_depth, &Output_depth, &Setpoint_depth, 0.95, 0, 0.96, DIRECT);
Timer every_go_t;




bool findBox = 0, dir = 0, gate = 0, right_gate, g_pos, targ_pos = 1, out = 1, b_pos;
int err, angle_local, angle = 0, color = -1, mode = 0, pos = 0, doit = 0, t_pos = 0, t_ang, d_mode = 0, last_depth = 30, eps = 160;;




int map(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
double rotate(double val)
{
  if(val > 359.9999) return (val -= 359.9999);
  if(val < 0) return (val += 359.9999);
  else return(val);
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
Object detectColor(cv::Mat img, int hmin, int hmax, int smin, int smax, int vmin, int vmax)
{
    cv::Scalar lower(hmin,smin,vmin);
    cv::Scalar upper(hmax,smax,vmax);
    cv::cvtColor(img,img,CV_BGR2HSV);
    cv::inRange(img,lower,upper,img);
    std::vector<std::vector<cv::Point> > cntrs;
    cv::findContours(img,cntrs,CV_RETR_TREE,CV_CHAIN_APPROX_NONE);
    Object obj2t;
    for(std::size_t i = 0; i <cntrs.size();i++)
    {
        if(cntrs.at(i).size() < 5) continue;
        if(std::fabs(cv::contourArea(cntrs.at(i))) < 300.0) continue;      
        cv::RotatedRect bEllipse = cv::fitEllipse(cntrs.at(i));
        obj2t.x = (int)bEllipse.center.x;
        obj2t.y = (int)bEllipse.center.y;
        obj2t.angle = bEllipse.angle;
        obj2t.type = Object::RECTANGLE;
        obj2t.size = bEllipse.boundingRect().width;
        return obj2t;
    }
    return obj2t;
}
Object detectBasket (cv:: Mat img) {
    cv::Scalar lower(30, 221, 58);
    cv::Scalar upper(144, 255, 119);
    cv::cvtColor(img, img, CV_BGR2HSV);
    cv::inRange(img, lower, upper, img);
    std::vector<std::vector<cv::Point>>contours;
    cv::findContours(img, contours, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    Object objectToRet;
    
    for (std::size_t i = 0; i < contours.size(); i++) {
        if (contours.at(i).size() < 10) {
            continue;
        }
        if (std::fabs(cv::contourArea(contours.at(i))) < 100.0) {
            continue;
        }
    
        cv::RotatedRect bEllipse = cv::fitEllipse(contours.at(i));
        objectToRet.x = (int) bEllipse.center.x;
        objectToRet.y = (int) bEllipse.center.y;
        objectToRet.angle = bEllipse.angle;
        for (auto obj : mur.getDetectedObjectsList(0)) {
            if (obj.type == Object :: RECTANGLE) 
                objectToRet.type = Object :: RECTANGLE;
        }
        return objectToRet;
    }
    return objectToRet;
}
Object detectYellow (cv:: Mat img) {
cv::Scalar lower (20, 150 , 100);
cv::Scalar upper (40,255, 255);
 cv::cvtColor(img, img, CV_BGR2HSV);
 cv::inRange(img, lower, upper, img);
 std::vector<std::vector<cv::Point>>contours;
 cv::findContours(img, contours, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
 Object objectToRet;
 
 for (std::size_t i = 0; i < contours.size(); i++) {
 if (contours.at(i).size() < 100) {
 continue;
 }
 if (std::fabs(cv::contourArea(contours.at(i))) < 300.0) {
 continue;
 }
 
 cv::RotatedRect bEllipse = cv::fitEllipse(contours.at(i));
 objectToRet.x = (int) bEllipse.center.x;
 objectToRet.y = (int) bEllipse.center.y;
 objectToRet.angle = bEllipse.angle;
 
 objectToRet.size = bEllipse.boundingRect().width;
   objectToRet.type = Object::RECTANGLE;
 return objectToRet;
}
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
    mur.setPortC(constrain(abs(constrain(hold - mur.getInputAOne(), 0, 100) * 10), 0, 50));
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
    if(timer.elapsed() >= 1200)
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
        mur.setPortC(constrain(depth - mur.getInputAOne() + 5, -50, 50) * 3);
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

int getColor(int x, int y, int cam)
{
    Mat img_l;
    if(cam == 1)
    {
        img_l = mur.getCameraOneFrame();
    }
    else if(cam == 2)
    {
        img_l = mur.getCameraTwoFrame();
    }
    unsigned char * p = img_l.ptr(y, x); // Y first, X after
    int r = p[2];
    int g = p[1];
    int b = p[0];
    //std::cout << r << "   " << g << "   " << b << std::endl;
    if(((r >= 110 && r <= 140) && g <= 10 && b <= 10) || ((r >= 160 && r <= 220) && (g >= 40 && g <= 100) && (b >= 55 && b <= 115)))
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
void go_every()
{
    if(every_go_t.elapsed() >= 70)
    {
        Input_depth = mur.getInputAOne() - 11;
        pid_depth.Compute();
        mur.setPortC(Output_depth);
        double angle_l = rotate(mur.getYaw() - angle_of_go + 180);
        Input_dir = angle_l;
        pid_dir.Compute();
        int out_dir = int(Output_dir + 0.05);
        //std::cout << angle_l<< "  "<<-out_dir << std::endl;
        if(((abs(int(mur.getYaw()) - angle_of_go) >= 2 && abs(int(mur.getYaw()) - angle_of_go) < 358) || abs(int(mur.getInputAOne()) - Setpoint_depth) >= 2) && !walking)
        {
            mur.setPortA(constrain(-out_dir, -100, 100));
            mur.setPortB(constrain(out_dir, -100, 100));
        }
        else
        {
            walking = 1;
            mur.setPortA(constrain(-out_dir - speed_of_go, -100, 100));
            mur.setPortB(constrain(out_dir - speed_of_go, -100, 100));
        }
        every_go_t.stop();
        every_go_t.start();
    }
    imshow("1", img);
    imshow("2", img_2);
    waitKey(1);
}

void set_to_obj(int x, int y)
{
        Setpoint_depth = 0;
        Input_depth = map(y - 120, -120, 120, 120, -120);
        pid_depth.Compute();
        mur.setPortC(Output_depth + 10);
        
        Input_dir = x - 160 + 180;
        //std::cout << Input_dir << std::endl;
                    pid_dir.Compute();
                    int out_dir = int(Output_dir + 0.05) * 0.2;
                    mur.setPortA(constrain(out_dir - 15, -50, 50));
                    mur.setPortB(constrain(-out_dir - 15, -50, 50));
                    sleepFor(50);
}
int set_angle_bin()
{
    double angle_local = 999999;
    bool can_roll = 0;
    while(abs(int(mur.getYaw()) - int(angle_local)) > 2)
    {
        img = mur.getCameraOneFrame();
        for (const auto &obj : mur.getDetectedObjectsList(0))
        {
            if (obj.type == Object::RECTANGLE && getColor(obj.x, obj.y, 1) == 0)
                {
                    angle_local = obj.angle;
                    if(angle_local < 90)
                        angle_local = map(angle_local, 90, 0, -90, 0);
                    else
                        angle_local = map(angle_local, 180, 90, 0, 90);
                    angle_local = rotate(mur.getYaw() - angle_local - 90);
                    can_roll = 1;
                }
        }
        if(can_roll)
        {
            Input_depth = mur.getInputAOne() - 11;
            pid_depth.Compute();
            mur.setPortC(Output_depth);
            
            double angle_for_pid = rotate(mur.getYaw() - angle_local + 180);
            //std::cout << int(angle_local) << std::endl;
            Input_dir = angle_for_pid;
            pid_dir.Compute();
            int out_dir = int(Output_dir + 0.05);
            mur.setPortA(constrain(-out_dir, -100, 100));
            mur.setPortB(constrain(out_dir, -100, 100));
        }
        sleepFor(50);
    }
    return angle_local;
}
int set_angle_rect()
{
    double angle_local = 999999;
    bool can_roll = 0;
    while(abs(int(mur.getYaw()) - int(angle_local)) > 2)
    {
        for (const auto &obj : mur.getDetectedObjectsList(0))
        {
            if (obj.type == Object::RECTANGLE)
                {
                    angle_local = obj.angle;
                    if(angle_local < 90)
                        angle_local = map(angle_local, 90, 0, -90, 0);
                    else
                        angle_local = map(angle_local, 180, 90, 0, 90);
                    angle_local = rotate(mur.getYaw() - angle_local);
                    can_roll = 1;
                }
        }
        if(can_roll)
        {
            Input_depth = mur.getInputAOne() - 11;
            pid_depth.Compute();
            mur.setPortC(Output_depth);
            
            double angle_for_pid = rotate(mur.getYaw() - angle_local + 180);
            //std::cout << int(angle_local) << std::endl;
            Input_dir = angle_for_pid;
            pid_dir.Compute();
            int out_dir = int(Output_dir + 0.05);
            mur.setPortA(constrain(-out_dir, -100, 100));
            mur.setPortB(constrain(out_dir, -100, 100));
        }
        sleepFor(50);
    }
    return angle_local;
}
void set(int angle_local, int depth_local, int speed)
{
    Setpoint_depth = depth_local;
    angle_of_go = angle_local;
    speed_of_go = speed;
    every_go_t.stop();
    every_go_t.start();
    while((abs(int(mur.getYaw()) - angle_local) >= 2 && abs(int(mur.getYaw()) - angle_local) < 358) || abs(int(mur.getInputAOne()) - depth_local) >= 3)
    {
        go_every();
        walking = 0;
        //std::cout << abs(int(mur.getYaw()) - angle_local)<< "  "<<abs(int(mur.getInputAOne()) - depth_local) << std::endl;
        //sleepFor(50);
    }
}
int main() 
{
    pid_dir.SetOutputLimits(-180, 180);
    pid_dir.SetMode(AUTOMATIC);
    
    pid_depth.SetOutputLimits(-100, 100);
    pid_depth.SetMode(AUTOMATIC);
    mur.addDetectorToList(Object::RECTANGLE, 1);
    mur.addDetectorToList(Object::RECTANGLE, 0);
    mur.addDetectorToList(Object::TRIANGLE, 0);
    img = mur.getCameraOneFrame();
    img_2 = mur.getCameraTwoFrame();
    if(mode == 0)
    {
        sleepFor(1000);
        set(0, 50, 0);
            img_2 = mur.getCameraTwoFrame();
            Object obj = detectGate(img_2);
            circle(img_2, Point(obj.x, obj.y), 10, Scalar(0, 255, 0), CV_FILLED, 8, 0);
            if (obj.type == Object::RECTANGLE) 
            {
                if (obj.x < eps) 
                {
                    while(obj.x < 150)
                    {
                        img_2 = mur.getCameraTwoFrame();
                        obj = detectGate(img_2);
                        circle(img_2, Point(obj.x, obj.y), 10, Scalar(0, 255, 0), CV_FILLED, 8, 0);
                        mur.setPortD(-10);
                        go_every();
                        imshow("2", img_2);
                        waitKey(1);
                    }
                    mur.setPortD(0);
                }
                else
                {
                    while(obj.x >= 200)
                    {
                        img_2 = mur.getCameraTwoFrame();
                        obj = detectGate(img_2);
                        circle(img_2, Point(obj.x, obj.y), 10, Scalar(0, 255, 0), CV_FILLED, 8, 0);
                        mur.setPortD(10);
                        go_every();
                        imshow("2", img_2);
                        waitKey(1);
                    }
                    mur.setPortD(0);
                 }
            }
        set(0, 70, 10);
        t.stop();
        t.start();
        mode = 1;
    }
    while(true)
    {
        img = mur.getCameraOneFrame();
        img_2 = mur.getCameraTwoFrame();
        go_every();
        if(mode == 1)
        {
            for (const auto &obj : mur.getDetectedObjectsList(0))
            {
                if (obj.type == Object::RECTANGLE)
                {
                    set(0, 40, 0);
                    set(set_angle_rect(), 60, 8);
                    mode = 2;
                    std::cout << "ready_2" << std::endl;
                    t.stop();
                    t.start();
                    while(t.elapsed() <= 5000)
                    {
                        go_every();
                    }
                    std::cout << "ready_3" << std::endl;
                }
            }
            

        }
        else if(mode == 2)
        {
            for (const auto &obj : mur.getDetectedObjectsList(0))
            {
                if (obj.type == Object::RECTANGLE)
                {
                    set(set_angle_rect(), 15, 10);
                    mode = 3;
                }
            }
        }
        else if(mode == 3)
        {
            Object obj = detectBasket(mur.getCameraOneFrame());
            if(obj.type == Object::RECTANGLE && obj.y >= 85)
            {
                //set(set_angle_bin(), 15, 0);
                set(mur.getYaw(), 15, 0);
                mode = 4;
            }
        }
        else if(mode == 4)
        {
            while(true)
            {
                Object obj = detectColor(mur.getCameraOneFrame(), 42, 75, 113, 255, 75, 130);
                go_every();
                if(obj.x != -1 && obj.y != -1)
                {
                    while(obj.x < 170)
                    {
                        obj = detectColor(mur.getCameraOneFrame(), 42, 75, 113, 255, 75, 130);
                        img = mur.getCameraOneFrame();
                        circle(img, Point(obj.x, obj.y), 10, Scalar(0, 255, 0), CV_FILLED, 8, 0);
                        std::cout << obj.x << std::endl;
                        mur.setPortD(-10);
                        go_every();
                        imshow("1", img);
                        waitKey(1);
                    }
                    while(obj.x >= 150 || obj.x == -1)
                    {
                        obj = detectColor(mur.getCameraOneFrame(), 42, 75, 113, 255, 75, 130);
                        img = mur.getCameraOneFrame();
                        circle(img, Point(obj.x, obj.y), 10, Scalar(0, 255, 0), CV_FILLED, 8, 0);
                        std::cout << obj.x << std::endl;
                        mur.setPortD(10);
                        go_every();
                        imshow("1", img);
                        waitKey(1);
                    }
                    mur.setPortD(0);
                    mur.drop();
                    set(270, 60, 0);
                    mode = 6;
                    break;
                }
            }
        }
        else if(mode == 6)
        {
            while(mode == 6)
            {
                img_2 = mur.getCameraTwoFrame();
                Object obj = detectYellow(mur.getCameraTwoFrame());
                    if (obj.x != -1 && obj.y != -1) 
                    {
                        set_to_obj(obj.x, obj.y);
                        circle(img_2, Point(obj.x, obj.y), 5, Scalar(0, 255, 0), CV_FILLED, 8, 0);
                        std::cout << obj.size << std::endl;
                        if(obj.size >= 100)
                        {
                            mur.shoot();
                            std::cout << "ready_20" << std::endl;
                            mode = 7;
                        }
                    }
                    else
                    {
                        Object obj_red = detectColor(img_2, 169, 255 , 55, 255 , 34, 207);
                        if(obj_red.x != -1 && obj_red.y != -1)
                        {
                            set_to_obj(obj_red.x, obj_red.y);
                            circle(img_2, Point(obj_red.x, obj_red.y), 5, Scalar(0, 255, 0), CV_FILLED, 8, 0);
                        }
                        else
                        {
                            mur.setPortA(-5);
                            mur.setPortB(5);
                            keepDepth(60);
                        }
                    }
                
            }
        }
        else if(mode == 7)
        {
            std::cout << "ready_21" << std::endl;
            set(mur.getYaw(), last_depth, 0);
            while(mode == 7)
            {
                mur.setPortA(-2);
                mur.setPortB(2);
                Input_depth = mur.getInputAOne() - 11;
                pid_depth.Compute();
                mur.setPortC(Output_depth);
                
                img_2 = mur.getCameraTwoFrame();
                Object obj = detectColor(img_2, 111, 119, 164, 255, 63, 255);
                circle(img_2, Point(obj.x, obj.y), 10, Scalar(0, 255, 0), CV_FILLED, 8, 0);
                imshow("2", img_2);
                waitKey(1);
                if(abs(obj.x - 158) <= 2)
                {
                    mode = 8;
                    std::cout << "ready_22" << std::endl;
                    set(mur.getYaw(), 40, 10);
                    std::cout << "ready_23" << std::endl;
                }
            }
        }
        else if(mode == 8)
        {
            for (const auto &obj : mur.getDetectedObjectsList(0))
            {
                if (obj.type == Object::TRIANGLE && getColor(obj.x, obj.y, 1) != ORANGE)
                {
                    circle(img, Point(obj.x, obj.y), 10, Scalar(0, 255, 0), CV_FILLED, 8, 0);
                    //imshow("Triangle", img);
                    //waitKey(1);
                    std::cout << "ready_24" << std::endl;
                    set(mur.getYaw(), 0, 0);
                    std::cout << "ready_25" << std::endl;
                    mode = 2019;
                }
            }
        }
    }
}