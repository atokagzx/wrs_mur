#include <murAPI.hpp>
/*
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
void forward()
{
    if(mur.getYaw() < 358 && mur.getYaw() >= 180)
    {
        int err = map(mur.getYaw(), 180, 360, 80, 5);
        mur.setPortA(err);
        mur.setPortB(-err * 0.8);
    }
    else if(mur.getYaw() >= 2 && mur.getYaw() < 180)
    {
        int err = map(mur.getYaw(), 180, 5, 80, 5);
        mur.setPortA(-err);
        mur.setPortB(err * 0.8);
    }
    else
    {
        mur.setPortA(0);
        mur.setPortB(0);
    }
}
 
int main()
{
    while(true)
    {
        forward();
    }
    return 0;
}*/