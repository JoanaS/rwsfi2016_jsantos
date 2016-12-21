/* _________________________________
   |                                 |
   |           INCLUDES              |
   |_________________________________| */
#include <ros/ros.h>
#include <rwsfi2016_libs/player.h>
#include <visualization_msgs/Marker.h>

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

    ros::Publisher publisher;
    visualization_msgs::Marker bocas_msg;

    /**
     * @brief Constructor, nothing to be done here
     * @param name player name
     * @param pet_name pet name
     */
    MyPlayer(string player_name, string pet_name="/dog"): Player(player_name, pet_name)
    {
        publisher = node.advertise<visualization_msgs::Marker>("/bocas", 1);
                    bocas_msg.header.frame_id = name;
                    bocas_msg.ns = name;
                    bocas_msg.id = 0;
                    bocas_msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                    bocas_msg.action = visualization_msgs::Marker::ADD;
                    bocas_msg.scale.z = 0.4;
                    bocas_msg.pose.position.y = 0.3;
                    bocas_msg.color.a = 1.0; // Don't forget to set the alpha!
                    bocas_msg.color.r = 0.0;
                    bocas_msg.color.g = 0.0;
        bocas_msg.color.b = 0.0;
    };

    void play(const rwsfi2016_msgs::MakeAPlay& msg)
    {
      //Custom play behaviour. Now I will win the game
        double distance_to_arena = getDistanceToArena();
        ROS_INFO("distance_to_arena = %f", distance_to_arena);

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

        double dist_min_hunter = 1000.0;
        double dist_hunter = 0.0;
        int angleMinHunter = 0;

        for (int pl=0; pl < hunters_team->players.size(); pl++)
        {
            dist_hunter = getDistanceToPlayer(hunters_team->players[pl]);

            if ((dist_hunter < dist_min_hunter) && (!isnan(dist_hunter)))
            {
                angleMinHunter = pl;
                dist_min_hunter = dist_hunter;
            }
        }

        float finalAngle;

        if (distance_to_arena > 6) //behaviour move to the center of arena
        {
            string arena = "/map";
            move(msg.max_displacement, getAngleToPLayer(arena));
        }
        else if(dist_min_hunter < near_player_distance)
        {
            double angle_temp = getAngleToPLayer(hunters_team->players[angleMinHunter]);
            finalAngle = angle_temp+M_PI;
            if (angle_temp > 0)
                finalAngle = angle_temp-M_PI;

            move(msg.max_displacement,  finalAngle);
            bocas_msg.text = "Deixem-me jogar!";
        }
        else
            move(msg.max_displacement, getAngleToPLayer(msg.blue_alive[index_near_player]) );



        publisher.publish(bocas_msg);



      //Behaviour follow the closest prey
      //move(msg.max_displacement, getAngleToPLayer(msg.blue_alive[index_near_player]) );
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
  string my_pet = "/cat";

  //initialize ROS stuff
  ros::init(argc, argv, my_name);

  //Creating an instance of class MyPlayer
  MyPlayer my_player(my_name, my_pet);

  //Infinite spinning (until ctrl-c)
  ros::spin();
}
