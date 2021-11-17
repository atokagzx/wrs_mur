using namespace cv;

Timer timer;
Timer t;
Timer timer_rot;

double Setpoint_dir = 180, Input_dir, Output_dir;
double Setpoint_depth = 80, Input_depth, Output_depth;
int needAngle = 0, needSpeed = 0;
bool isReady = 0;
PID pid_dir(&Input_dir, &Output_dir, &Setpoint_dir, 0.95, 0, 0.96, DIRECT);
PID pid_depth(&Input_depth, &Output_depth, &Setpoint_depth, 0.95, 0, 0.96, DIRECT);
Timer keepAngle_t;
Timer keepDepth_t;

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

int constrain(int x, int a, int b)
{
    if(x < a)
        return a;
    else if(b < x)
        return b;
    else
        return x;
}
Object detectColor(cv::Mat _img, int hmin, int hmax, int smin, int smax, int vmin, int vmax)
{
    Mat img = _img.clone();
    Scalar lower(hmin,smin,vmin);
    Scalar upper(hmax,smax,vmax);
    cvtColor(img,img,CV_BGR2HSV);
    inRange(img,lower,upper,img);
    std::vector<std::vector<cv::Point> > cntrs;
    findContours(img,cntrs,CV_RETR_TREE,CV_CHAIN_APPROX_NONE);
    Object obj2t;
    
    for(std::size_t i = 0; i <cntrs.size();i++)
    {
        if(cntrs.at(i).size() < 5) continue;
        if(std::fabs(cv::contourArea(cntrs.at(i))) < 300.0) continue;      
        RotatedRect bEllipse = cv::fitEllipse(cntrs.at(i));
        obj2t.x = (int)bEllipse.center.x;
        obj2t.y = (int)bEllipse.center.y;
        obj2t.angle = bEllipse.angle;
        obj2t.type = -1;
        obj2t.size = bEllipse.boundingRect().width;
        return obj2t;
    }
    return obj2t;
}

void keepAngle()
{
    if(keepAngle_t.elapsed() >= 70)
    {
        double angle_l = rotate(mur.getYaw() - needAngle + 180);
        Input_dir = angle_l;
        pid_dir.Compute();
        int out_dir = int(Output_dir + 0.05);
        //std::cout << angle_l<< "  "<<-out_dir << std::endl;
        if(abs(int(mur.getYaw()) - needAngle) >= 2 && abs(int(mur.getYaw()) - needAngle) < 358 && !isReady)
        {
            mur.setPortA(constrain(-out_dir, -100, 100));
            mur.setPortB(constrain(out_dir, -100, 100));
        }
        else
        {
            isReady = 1;
            mur.setPortA(constrain(-out_dir - needSpeed, -100, 100));
            mur.setPortB(constrain(out_dir - needSpeed, -100, 100));
        }
        keepAngle_t.stop();
        keepAngle_t.start();
    }
}

void keepDepth()
{
    if(keepDepth_t.elapsed() >= 70)
    {
        Input_depth = mur.getInputAOne() - 11;
        pid_depth.Compute();
        mur.setPortC(Output_depth);
        keepDepth_t.stop();
        keepDepth_t.start();
    }
}

void set_to_obj(Object _obj, int _speed, int _depth)
{
    if(_obj.x != -1)
    {
        Setpoint_depth = 0;
        Input_depth = map(_obj.y - 120, -120, 120, 120, -120);
        pid_depth.Compute();
        mur.setPortC(Output_depth + 10);
                    
        Input_dir = _obj.x - 160 + 180;
        pid_dir.Compute();
        int out_dir = int(Output_dir + 0.05) * 0.2;
        mur.setPortA(constrain(out_dir - 5, -50, 50));
        mur.setPortB(constrain(-out_dir - 5, -50, 50));
        sleepFor(50);
    }
    else
    {
        mur.setPortA(-2);
        mur.setPortB(2);
        Setpoint_depth = _depth;
        keepDepth();
    }
}
void set_to_obj_f(Object _obj, int _depth)
{
    Setpoint_depth = _depth;
    keepDepth();
    if(_obj.x != -1)
    {
        if(abs(_obj.y - 120) > 5)
        {
            Setpoint_dir = 0;
            Input_dir = map(_obj.y - 120, -120, 120, 120, -120);
            pid_dir.Compute();
            mur.setPortA(Output_dir);
            mur.setPortB(Output_dir);
            sleepFor(50);
        }
        else if(abs(_obj.x - 160) > 5)
        {
            Setpoint_dir = 0;
            Input_dir = map(_obj.x - 160, -160, 160, 160, -160);
            pid_dir.Compute();
            mur.setPortD(Output_dir * 0.3);
            sleepFor(50);
        }
    }
    else
    {
        mur.setPortA(0);
        mur.setPortB(0);
        mur.setPortD(0);
        Setpoint_depth = _depth;
        keepDepth();
    }
}
int set_to_line()
{
    Object color = detectColor(mur.getCameraOneFrame(), 2, 9, 225, 255, 102, 135);
    mur.addDetectorToList(Object::RECTANGLE, 0);
    //std::cout << "f" << std:: endl;
    Object _obj;
    for (const auto &_obj : mur.getDetectedObjectsList(0))
    {
        if (_obj.type == Object::RECTANGLE && abs(color.x - _obj.x) <= 10 && abs(color.y - _obj.y) <= 10)
        {
            //std::cout << obj.x << " " << obj.y << std:: endl;
            //std::cout << color.x << " " << color.y << std:: endl;
            
            double angle_local = _obj.angle;
            if(angle_local < 90)
                angle_local = map(angle_local, 90, 0, -90, 0);
            else
                angle_local = map(angle_local, 180, 90, 0, 90);
            angle_local = rotate(mur.getYaw() - angle_local);
            
            while(abs(int(mur.getYaw()) - int(angle_local)) > 2)
            {
                keepDepth();
                color = detectColor(mur.getCameraOneFrame(), 2, 9, 225, 255, 102, 135);
                for (const auto &_obj : mur.getDetectedObjectsList(0))
                {
                    if (_obj.type == Object::RECTANGLE && abs(color.x - _obj.x) <= 10 && abs(color.y - _obj.y) <= 10)
                    {
                        angle_local = _obj.angle;
                        if(angle_local < 90)
                            angle_local = map(angle_local, 90, 0, -90, 0);
                        else
                            angle_local = map(angle_local, 180, 90, 0, 90);
                        angle_local = rotate(mur.getYaw() - angle_local);
                    }
                }
                double angle_for_pid = rotate(mur.getYaw() - angle_local + 180);
                Input_dir = angle_for_pid;
                pid_dir.Compute();
                int out_dir = int(Output_dir + 0.05);
                mur.setPortA(constrain(-out_dir, -100, 100));
                mur.setPortB(constrain(out_dir, -100, 100));
                sleepFor(50);
            }
            mur.setPortA(0);
            mur.setPortB(0);
            needAngle = mur.getYaw();
            needSpeed = 0;
            return angle_local;
        }
    }
    sleepFor(50);
    return -1;
}
void setAngle(int _needAngle, int _needSpeed)
{
    Setpoint_dir = 180;
    needAngle = _needAngle;
    needSpeed = _needSpeed;
    keepAngle_t.stop();
    keepAngle_t.start();
    while(abs(int(mur.getYaw()) - needAngle) >= 2 && abs(int(mur.getYaw()) - needAngle) < 358)
    {
        keepAngle();
        isReady = 0;
    }
}
void setDepth(int _depth)
{
    Setpoint_depth = _depth;
    keepDepth_t.stop();
    keepDepth_t.start();
    while(abs(int(mur.getInputAOne()) - Setpoint_depth) >= 2)
        keepDepth();
}
void set(int _needAngle, int _depth, int _needSpeed)
{
    Setpoint_dir = 180;
    needAngle = _needAngle;
    needSpeed = 0;
    Setpoint_depth = _depth;
    keepDepth_t.stop();
    keepAngle_t.stop();
    keepDepth_t.start();
    keepAngle_t.start();
    while((abs(int(mur.getYaw()) - needAngle) >= 2 && abs(int(mur.getYaw()) - needAngle) < 358) || (abs(int(mur.getInputAOne()) - Setpoint_depth) >= 2))
    {
        keepDepth();
        keepAngle();
        isReady = 0;
    }
    needSpeed = _needSpeed;
}