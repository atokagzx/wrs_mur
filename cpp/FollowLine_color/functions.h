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
Object target_first(bool h = 0)
{
    int hmin, hmax, smin, smax, vmin, vmax;
    if(!h)
    {//green
        hmin = 59; hmax = 63; smin =  232; smax = 255; vmin = 136; vmax = 185;
    }
    else
    {//yellow
        hmin = 28; hmax = 34; smin =  214; smax = 255; vmin = 136; vmax = 199;
    }
    Mat img = mur.getCameraTwoFrame();
    Scalar lower(hmin,smin,vmin);
    Scalar upper(hmax,smax,vmax);
    cvtColor(img,img,CV_BGR2HSV);
    inRange(img,lower,upper,img);
    std::vector<std::vector<cv::Point> > cntrs;
    findContours(img,cntrs,CV_RETR_TREE,CV_CHAIN_APPROX_NONE);
    Object obj2t;
    Mat frame = mur.getCameraTwoFrame();
    int size = 0;
    for(std::size_t i = 0; i < cntrs.size(); i++)
    {
        if(cntrs.at(i).size() > 50)
            size ++;
    }
    if(size != 3)
        return obj2t;
    int max_h, max_n = 0;
    if(h)
        max_h = 999;
    else
        max_h = 0;
    for(std::size_t i = 0; i < cntrs.size(); i++)
    {
        if(cntrs.at(i).size() <= 50) continue;
        if(std::fabs(cv::contourArea(cntrs.at(i))) < 50.0) continue;      
        RotatedRect bEllipse = cv::fitEllipse(cntrs.at(i));
        obj2t.x = (int)bEllipse.center.x;
        obj2t.y = (int)bEllipse.center.y;
        obj2t.type = -1;
        obj2t.angle = bEllipse.angle;
        obj2t.size = bEllipse.boundingRect().width;
        circle(frame, Point(obj2t.x + 2, obj2t.y + 2), 2, Scalar(255, 0, 0), 3);
        
        if(obj2t.y > max_h && !h)
        {
            max_h = obj2t.y;
            max_n = i;
        }
        else if(obj2t.y < max_h && h)
        {
            max_h = obj2t.y;
            max_n = i;
        }
    }
    RotatedRect bEllipse = cv::fitEllipse(cntrs.at(max_n));
    obj2t.x = (int)bEllipse.center.x;
    obj2t.y = (int)bEllipse.center.y - 10;
    obj2t.angle = bEllipse.angle;
    obj2t.size = bEllipse.boundingRect().width;
    circle(frame, Point(obj2t.x, obj2t.y), 2, Scalar(0, 255, 0), 3);
    std::cout << "Detected" << std::endl;
    imshow("", frame);
    waitKey(1);
    return obj2t;
}
int vertex() {
    
        
    cv::Mat image;
    Mat img = mur.getCameraOneFrame();
    int hmin = 28, hmax = 42, smin =  219, smax = 255, vmin = 91, vmax = 156;
    std::vector<std::vector<cv::Point> > contours;
    static std::vector<cv::Point> hull;
    image = img.clone();
    cv::Scalar lower(hmin, smin, vmin);
    cv::Scalar upper(hmax, smax, vmax);
    cv::cvtColor(image, image, CV_BGR2HSV);
    cv::inRange(image, lower,upper, image);
    double cannyParams = cv::threshold(image, image, 0, 255, CV_THRESH_BINARY_INV + CV_THRESH_OTSU);
    cv::Canny(image, image, cannyParams, cannyParams / 2.0F);
    cv::findContours(image, contours, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    Object objectToRet;
    for(std::size_t i = 0; i < contours.size(); ++i) 
    {
        if(contours.at(i).size() < 5) 
            continue;
        if(std::fabs(cv::contourArea(contours.at(i))) < 100.0)
            continue;
        cv::convexHull(contours.at(i), hull, true);
        cv::approxPolyDP(hull, hull, 15, true);
        if (!cv::isContourConvex(hull)) {
            continue;
        }
        
        //return hull.size();
    }
    return hull.size();
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

void set_to_obj(Object _obj, int _speed, int _depth, bool d = 1)
{
    if(_obj.x != -1)
    {
        if(d)
        {
            Setpoint_depth = 0;
            Input_depth = map(_obj.y - 120, -120, 120, 120, -120);
            pid_depth.Compute();
            mur.setPortC(Output_depth + 10);
        }     
        Input_dir = _obj.x - 160 + 180;
        pid_dir.Compute();
        int out_dir = int(Output_dir + 0.05) * 0.2;
        mur.setPortA(constrain(out_dir - _speed, -50, 50));
        mur.setPortB(constrain(-out_dir - _speed, -50, 50));
        sleepFor(50);
    }
    else
    {
        mur.setPortA(-2);
        mur.setPortB(2);
        if(d)
            Setpoint_depth = _depth;
        keepDepth();
    }
}
void set_to_obj_f(Object _obj)
{
    if(_obj.x != -1)
    {
        if(abs(_obj.y - 120) > 5)
        {
            Setpoint_dir = 0;
            Input_dir = map(_obj.y - 120, -120, 120, 40, -40);
            pid_dir.Compute();
            mur.setPortA(Output_dir);
            mur.setPortB(Output_dir);
            sleepFor(50);
        }
        if(abs(_obj.x - 160) > 5)
        {
            int x_speed = map(constrain((_obj.x - 160) * 1.5, - 100, 100), -100, 100, -20, 20);
            mur.setPortD(x_speed);
            sleepFor(50);
        }
    }
    else
    {
        mur.setPortA(0);
        mur.setPortB(0);
        mur.setPortD(0);
    }
}
int followLine()
{
        int hmin = 6, hmax = 9, smin =  235, smax = 255, vmin = 114, vmax = 128;
        keepDepth();
        //keepAngle();
        Mat img = mur.getCameraOneFrame();
        //img = img(cv::Rect(0, 0, 320, 120));
        Scalar lower(hmin,smin,vmin);
        Scalar upper(hmax,smax,vmax);
        cvtColor(img,img,CV_BGR2HSV);
        inRange(img,lower,upper,img);
        std::vector<std::vector<cv::Point> > cntrs;
        findContours(img,cntrs,CV_RETR_TREE,CV_CHAIN_APPROX_NONE);
        Object obj2t;
        
        for(std::size_t i = 0; i < cntrs.size();i++)
        {
            if(cntrs.at(i).size() < 5) continue;
            if(std::fabs(cv::contourArea(cntrs.at(i))) < 300.0) continue;      
            RotatedRect bEllipse = cv::fitEllipse(cntrs.at(i));
            obj2t.x = (int)bEllipse.center.x;
            obj2t.y = (int)bEllipse.center.y;
            obj2t.angle = bEllipse.angle;
            obj2t.type = -1;
            obj2t.size = bEllipse.boundingRect().width;
        }
        if(obj2t.angle != -1)
        {
            double angle_local = obj2t.angle;
            if(angle_local < 90)
                angle_local = map(angle_local, 90, 0, -90, 0);
            else
                angle_local = map(angle_local, 180, 90, 0, 90);
            angle_local = rotate(mur.getYaw() - angle_local);
            double angle_for_pid = rotate(mur.getYaw() - angle_local + 180);
            Input_dir = angle_for_pid;
            pid_dir.Compute();
            int out_dir = int(Output_dir + 0.05);
            if(abs(angle_local - mur.getYaw()) > 2)
            {
                mur.setPortA(constrain(-out_dir  - 10, -100, 100));
                mur.setPortB(constrain(out_dir - 10, -100, 100));
            }
            else
            {
                mur.setPortA(constrain(-out_dir - 40, -100, 100));
                mur.setPortB(constrain(out_dir - 40, -100, 100));
            }
            int x_speed = map(constrain((obj2t.x - 160) * 1.5, - 100, 100), -100, 100, -30, 30);
            if(abs(obj2t.x - 160) > 5)
            {
                mur.setPortD(x_speed);
            }
            return angle_local;
        }
        else
        {
            keepAngle();
            keepDepth();
            return -1;
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
    while(abs(int(mur.getYaw()) - needAngle) >= 5 && abs(int(mur.getYaw()) - needAngle) < 355)
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
    while(abs(int(mur.getInputAOne()) - Setpoint_depth) >= 5)
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
    while((abs(int(mur.getYaw()) - needAngle) >= 5 && abs(int(mur.getYaw()) - needAngle) < 355) || (abs(int(mur.getInputAOne()) - Setpoint_depth) >= 5))
    {
        keepDepth();
        keepAngle();
        isReady = 0;
    }
    needSpeed = _needSpeed;
}