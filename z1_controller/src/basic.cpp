#include "unitreeArm.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "z1_controller");
    ros::AsyncSpinner subSpinner(1); // one threads
    subSpinner.start();
    usleep(300000); // must wait 300ms, to get first state

    unitreeArm::IOROS ioInter;

    // move z1 to [forward]
    double forward[7] = {0, 1.5, -1, -0.54, 0, 0, -1};
    ioInter.sendCmd(forward, 1000);

    return 0;
}