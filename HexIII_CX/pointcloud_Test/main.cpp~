#include "kinect.h"
#include <iostream>
#include <time.h>
#include <sys/stat.h>

using namespace std;

int main()
{

    time_t t = time(NULL);
    tm* local = new tm;
    char buf[26] = {0};
    localtime_r(&t,local);
    strftime(buf, 64, "%Y-%m-%d %H-%M-%S", local);
    mkdir(buf,S_IRWXU | S_IRWXG);
    chdir(buf);

    Kinect visionsensor;
    visionsensor.viewcloud();
    visionsensor.start();

    while (1)
    {
      char c = getchar();
      if (c == 'c')
          visionsensor.capture();
    }
}
