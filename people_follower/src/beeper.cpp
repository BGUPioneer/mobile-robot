// testing sound
#include <sound_play/sound_play.h>
#include <unistd.h>
 
 
void sleepok(int t, ros::NodeHandle &nh)
{
   if (nh.ok())
                 sleep(t);
}

int main(int argc, char **argv){


    ros::init(argc, argv, "beeper");
    ros::NodeHandle nh;
    sound_play::SoundClient sc;

sleepok(1, nh);

//const char *str5 = "/home/robot/workspace/ros/catkin/sounds/following_started.wav";
    //sc.startWave(str5);
    //sleepok(8, nh);
    //sc.stopWave(str5);


    while(nh.ok())
  {
    
    const char *str1 = "/home/robot/workspace/ros/catkin/sounds/beep_beep2.wav";
    sc.startWave(str1);
    sleepok(4, nh);
    sc.stopWave(str1);

    sleepok(1, nh);
 
  }



      ros::spin();

}




