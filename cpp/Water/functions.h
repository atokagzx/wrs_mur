using namespace cv;

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

void setupShoot()
{
    mur.setPortD(-100);
}

void shoot()
{
    mur.setPortD(100);
    sleepFor(3000);
    mur.setPortD(0);
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
        if(std::fabs(cv::contourArea(cntrs.at(i))) < 50.0) continue;      
        RotatedRect bEllipse = cv::fitEllipse(cntrs.at(i));
        obj2t.x = (int)bEllipse.center.x;
        obj2t.y = (int)bEllipse.center.y;
        obj2t.angle = bEllipse.angle;
        obj2t.size = bEllipse.boundingRect().width * bEllipse.boundingRect().height / 100;
        return obj2t;
    }
    return obj2t;
}
int setLine(int _angle)
{
    if(_angle != -1)
    {
        ////
        _angle = _angle + 90;
        if(_angle >= 180)
            _angle -= 180;
        else if(_angle < 0)
            _angle += 180;
        ////
        float kpA = 5, kpB = 0.9; 
        int err = 90 - _angle;
        mur.setPortA((err) * kpA);
        mur.setPortB((-err) * kpB);
        //std::cout << _angle << std::endl;
        if(abs(90 - _angle) <= 2)
            return mur.getYaw();
        else
            return -1;
    }
    else
    {
        mur.setPortA(0);
        mur.setPortB(0);
        mur.setPortC(-100);
        return -1;
    }
}
void setObj(Object obj, int _speed)
{
    if(obj.size != -1)
    {
        int x = map(160 - obj.x, -160, 160, -100, 100);
        float kpA = 5, kpB = 0.9;
        mur.setPortA((x - _speed) * kpA);
        mur.setPortB((-x - _speed) * kpB);
        
        if(obj.y <=  110)
        {
            mur.setPortC(-100);
        }
        else if(obj.y >= 130)   //!!!!!!!!!!!
        {
            mur.setPortC(0);
        }
        //std::cout << x<< std::endl;
    }
    else
    {
        mur.setPortA(0);
        mur.setPortB(0);
        mur.setPortC(0);
    }
}
void keepAngle(int _angle, int _speed) 
{
    int yaw = rotate(mur.getYaw() - _angle);
    yaw = 360 - yaw;     //!!!!!!!!!!
    float kpA = 5, kpB = 0.9; 
    int err = 180 - yaw;
    mur.setPortA((err - _speed) * kpA);
    mur.setPortB((-err - _speed) * kpB);
    
}
void keepDepth(int _need)
{
    if(mur.getInputBOne() >= _need)  //!!!!!!!!!!!
    {
        mur.setPortC(-100);
    }
    if(mur.getInputBOne() < _need)   //!!!!!!!!!!!
    {
        mur.setPortC(0);
    }
}