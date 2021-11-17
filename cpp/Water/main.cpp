#include <murAPI.hpp>
#include <thread>
#include <functions.h>

using namespace cv;

Object obj;
Timer t;

int mode = 1, angle, depth = 80;
void testShoot()
{
    setupShoot();
    sleepFor(5000);
    shoot();
}
int main() 
{
    testShoot();
    return 0;
}