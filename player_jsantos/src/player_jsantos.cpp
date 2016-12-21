/* _________________________________
   |                                 |
   |           INCLUDES              |
   |_________________________________| */
#include <ros/ros.h>
#include <rwsfi2016_libs/player.h>

/* _________________________________
   |                                 |
   |              CODE               |
   |_________________________________| */
using namespace std;
using namespace ros;


/**
 * @brief MyPlayer extends class Player, i.e., there are additional things I can do with MyPlayer and not with any Player, e.g., to order a movement.
 */
class MyPlayer: public rwsfi2016_libs::Player
{
  public:

    /**
     * @brief Constructor, nothing to be done here
     * @param name player name
     * @param pet_name pet name
     */
    MyPlayer(string player_name, string pet_name="/dog"): Player(player_name, pet_name){};

    void play(const rwsfi2016_msgs::MakeAPlay& msg)
    {
      //Custom play behaviour. Now I will win the game

        double distance_to_arena = getDistanceToArena();
        ROS_INFO("distance_to_arena = %f", distance_to_arena);
/*
        if (distance_to_arena > 6) //behaviour move to the center of arena
        {
            string arena = "/map";
            move(msg.max_displacement, getAngleToPLayer(arena));
        }
*/
        double near_player_distance = 1000.0;
        int index_near_player = 0;

        std::cout << "Number of Preys: "<< preys_team->players.size() << std::endl;

        for(int i=0; i < msg.blue_alive.size(); i++)
        {
           // std::cout << preys_team->players[i] << std::endl;
            float distance = getDistanceToPlayer(msg.blue_alive[i]);
            //std::cout << "Get Distance: "<< isnan(distance) << std::endl;

            if( (distance < near_player_distance) && (!isnan(distance)))
            {
                near_player_distance = distance;
                index_near_player    = i;
            }
        }

      //Behaviour follow the closest prey
      move(msg.max_displacement, getAngleToPLayer(msg.blue_alive[index_near_player]) );


      if (getDistanceToArena() < 5)
      {
          //Behaviour follow the closest prey
          move(msg.max_displacement, getAngleToPLayer(msg.blue_alive[index_near_player]) );
      }
      else
      {
          move(msg.max_displacement, getAngleToPLayer(msg.blue_alive[index_near_player]) + M_PI );
      }

    }
};


/**
 * @brief The main function. All you need to do here is enter your name and your pets name
 * @param argc number of command line arguments
 * @param argv values of command line arguments
 * @return result
 */
int main(int argc, char** argv)
{
  // ------------------------
  //Replace this with your name
  // ------------------------
  string my_name = "jsantos";
  string my_pet = "/turtle";

  //initialize ROS stuff
  ros::init(argc, argv, my_name);

  //Creating an instance of class MyPlayer
  MyPlayer my_player(my_name, my_pet);

  //Infinite spinning (until ctrl-c)
  ros::spin();
}
