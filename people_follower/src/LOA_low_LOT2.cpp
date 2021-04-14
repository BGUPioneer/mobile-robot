// Intro
#include <sound_play/sound_play.h>
#include <unistd.h>
 
 
void sleepok(int t, ros::NodeHandle &nh)
{
   if (nh.ok())
                 sleep(t);
}

int main(int argc, char **argv){


    ros::init(argc, argv, "LOA_low_LOT2");
    ros::NodeHandle nh;
    sound_play::SoundClient sc;

    sleepok(1, nh);

    const char *str1 = "/home/robot/workspace/ros/catkin/sounds/following_consent.wav";
    //const char *str1 = "/home/robot/workspace/ros/catkin/sounds/following_consentEng.wav";
    sc.startWave(str1);
    sleepok(8, nh);
    sc.stopWave(str1);

    const char *str2 = "/home/robot/workspace/ros/catkin/sounds/following_started_why.wav";
    //const char *str2 = "/home/robot/workspace/ros/catkin/sounds/following_started_whyEng.wav";
    sc.startWave(str2);
    sleepok(8, nh);
    sc.stopWave(str2);

while(nh.ok())
  {
    
  const char *str3= "/home/robot/workspace/ros/catkin/sounds/following_following_why.wav";
    //const char *str3= "/home/robot/workspace/ros/catkin/sounds/following_following_whyEng.wav";
        sc.startWave(str3);
        sleepok(12, nh);
        sc.stopWave(str3);

        sleepok(1, nh);

  }


      ros::spin();

}
