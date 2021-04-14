// Intro
#include <sound_play/sound_play.h>
#include <unistd.h>
 
 
void sleepok(int t, ros::NodeHandle &nh)
{
   if (nh.ok())
                 sleep(t);
}

int main(int argc, char **argv){



    ros::init(argc, argv, "LOA_high_LOT1");
    ros::NodeHandle nh;
    sound_play::SoundClient sc;

    sleepok(1, nh);


    const char *str1 = "/home/robot/workspace/ros/catkin/sounds/following_started.wav";
    //const char *str1 = "/home/robot/workspace/ros/catkin/sounds/following_startedEng.wav";
    sc.startWave(str1);
    sleepok(8, nh);
    sc.stopWave(str1);


while(nh.ok())
  {
    
  const char *str2= "/home/robot/workspace/ros/catkin/sounds/following_following.wav";
    //const char *str2= "/home/robot/workspace/ros/catkin/sounds/following_followingEng.wav";
        sc.startWave(str2);
        sleepok(8, nh);
        sc.stopWave(str2);

        sleepok(1, nh);


  }


      ros::spin();

}
