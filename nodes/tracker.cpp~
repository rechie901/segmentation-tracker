#include "ros/ros.h"
//#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include "/home/sarthak/catkin_ws/src/lidar_tracker/include/centroids.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <algorithm>
using namespace std;
using namespace Eigen;

//lidar_tracker::centroids track;
lidar_tracker::centroids predicted;

            visualization_msgs::Marker marker;
            

struct track
{
    float x, y, mean_x, mean_y, vel_x, vel_y, var_x, var_y; // track at time t
    float x_pred, y_pred, vel_x_pred, vel_y_pred; // predicted states at time t+1
    float z_x, z_y; // measurements observed
    float z_x_pred, z_y_pred; // predicted measurements at time t+1
    float v_x, v_y; // measurements residual
};
vector < track > tracks;


// Constant Matrix


/*void update_mean_var(float x, float y, float mean_x, float mean_y, float var_x, float var_y)
{
    cout<<" lalalala"<< endl;
    x=100;
    mean_x=(x+mean_x)/2;
    mean_y=(y+mean_y)/2;
     var_x=(x-mean_x)*(x-mean_x);
    var_y=(y-mean_y)*(y-mean_y);
}
*/

void chatterCallback(const lidar_tracker::centroidsConstPtr& input)
{
  float T = 0.1; // 100 ms
  //lidar_tracker::centroids centroids;
  int num_input = input->points.size();
  int num_tracks = tracks.size();
  
    
  MatrixXf F(4,4);
  F(0,0),F(1,1),F(2,2),F(3,3) = 1.0;
  F(0,1),F(0,3),F(1,0),F(1,2),F(2,0),F(2,1),F(2,3),F(3,0),F(3,1),F(3,2) = 0.0;
  F(0,2),F(1,3) = T;
  //F()
  MatrixXf H(4,4);
  H(0,0),H(1,1),H(2,2),H(3,3)=1.0;
  H(0,1),H(0,2),H(0,3),H(1,0),H(1,2),H(1,3),H(2,0),H(2,1),H(2,3),H(3,0),H(3,1),H(3,2)=0.0;
  ////
  VectorXf X_pred(4);// state prediction vector at t+1
  VectorXf X(4); // state vector at t
  VectorXf Z_pred(4); // measurement prediction vector at t+1
  VectorXf Z(4); // measurement vector at t
  VectorXf r(4); // residual
  VectorXf u(4);// noise
  u(0),u(1) = 0.05;
  u(2),u(3) = 0;
  VectorXf v(4); // noise
  v(0),v(1) = 0.05;
  v(2),v(3) = 0;
 
  if(num_input > 0)
  {
         if(num_tracks == 0)  // Initializing the empty pool
         {
             tracks.resize(input->points.size());
           for(std::vector<int>::size_type i = 0; i != input->points.size(); i++)
            {
                   tracks[i].x=input->points[i].x;
                   tracks[i].y=input->points[i].y;
                   //tracks[i].mean_x=(tracks[i].x+tracks[i].mean_x)/2;
                   //tracks[i].z=0.0;
                //std::cout<<tracks[i].mean_x<<std::endl;
            }
         }
         else //(num_tracks < num_input)
         {
                 //state_prediction;
                 
            for(std::vector<int>::size_type i = 0; i != tracks.size(); i++)
            { 
                
                X_pred(0)=tracks[i].x_pred;
                X_pred(1)=tracks[i].y_pred;
                X_pred(2)=tracks[i].vel_x_pred;
                X_pred(3)=tracks[i].vel_y_pred;
              
              
                X(0)=tracks[i].x;
                X(1)=tracks[i].y;
                X(2)=tracks[i].vel_x;
                X(3)=tracks[i].vel_y;
              
              
                Z_pred(0)=tracks[i].z_x_pred;
                Z_pred(1)=tracks[i].z_y_pred;
                Z_pred(2)=0;
                Z_pred(3)=0;
              

                Z(0)=tracks[i].z_x;
                Z(1)=tracks[i].z_y;
                Z(2)=0;
                Z(3)=0;
              

                r(0) = tracks[i].v_x;
                r(1) = tracks[i].v_y;
                r(2),r(3) = 0;
                
              
               // cout<< tracks[i].mean_x << "  "<< tracks[i].mean_y<<"  "<<tracks[i].var_x<<"  "<<tracks[i].var_y << endl;
            }           
                X_pred = F*X + u;
                Z_pred = H*X_pred + v;
                vector<track> newtrack;
                // filtering of tracks
                cout<<" track size before erase : "<<tracks.size()<<endl;
                for(std::vector<int>::size_type j = 0; j != tracks.size();)
                       {
                           //int x= tracks[*j].x;
                           float beta_x = exp(-(pow((tracks[j].z_x - tracks[j].mean_x),2))/tracks[j].var_x);
                           float beta_y = exp(-(pow((tracks[j].z_y - tracks[j].mean_y),2))/tracks[j].var_y);
                           //cout<< pow((tracks[j].z_x - tracks[j].mean_x),2) << "  "<< tracks[j].mean_y<<"  "<<tracks[j].var_x<<"  "<<tracks[j].var_y << endl;
                            cout<< " beta "<<beta_x<< " "<< beta_y<< " "<< endl;
                              if(beta_x < 0.05 || beta_y < 0.05 || beta_x == 0.367879)
                              {
                                      tracks.erase(tracks.begin()+j);
                                      //cout<< "erase[j] "<<tracks.size()<< endl;
                                      //j++;
                                    //tracks[j]=0;
                                      
                                       
                              }
                              else
                                j++;
                             

                       }
                       //cout<<" track size before erase : "<<tracks.size()<<endl;
                //tracks.swap(newtrack);
                //if(erase.size()>0)
                  // tracks.erase(remove( tracks.begin(), tracks.end(), 0 ), tracks.end());
                    //cout<< "erase size : "<<erase.size()<< endl;
                    // for(std::vector<int>::size_type j = 0; j != erase.size(); j++)
                    // {
                    //   cout<< "erase[j] "<<erase[j]<< endl; 
                    //   tracks.erase(tracks.begin()+erase[j]);
                    // }
                  
                cout<<" track size after erase : "<<tracks.size()<<endl;
                
                if(tracks.size() > 0)
                {

                      // MAP
                     
                      float beta = 0.1; 
                      vector < float > map_x;
                      vector < float > map_y;
                      
                      for(std::vector<int>::size_type i = 0; i != input->points.size(); i++)
                      {
                        map_y.resize(tracks.size());
                        map_x.resize(tracks.size());
                             for(std::vector<int>::size_type j = 0; j != tracks.size(); j++)
                             {
                                 map_x[j] = exp(-(pow((input->points[i].x - tracks[j].mean_x),2))/tracks[j].var_x);
                                 map_y[j] = exp(-(pow((input->points[i].y - tracks[j].mean_y),2))/tracks[j].var_y);
                                 cout<< tracks[j].mean_x << "  "<< tracks[j].mean_y<<"  "<<tracks[j].var_x<<"  "<<tracks[j].var_y << endl;
                                 cout<< " map_x " << map_x[i] << " map_y " <<map_y[i]<< endl; 
                             }
                             auto index_x = max_element(map_x.begin(), map_x.end());
                             int k_x = distance(map_x.begin(), index_x);
                             auto index_y = max_element(map_y.begin(), map_y.end());
                             int k_y = distance(map_y.begin(), index_y);
                             if(k_x == k_y && map_y[k_x]>beta && map_x[k_y]>beta)
                             {
                                 tracks[k_x].z_x=input->points[i].x;
                                 tracks[k_y].z_y=input->points[i].y;
                                 cout<<"associated"<<endl;

                              }
                              else
                              {
                                  track points;
                                  points.x = input->points[i].x;
                                  points.y = input->points[i].y;
                                  cout<< " added" << " track size : "<<tracks.size()<< endl;
                                  tracks.push_back(points);

                              }              
                      }
                      r = Z - Z_pred;
                      X = X_pred + r;  
                                      
                       //filter_tracks;
                       //MAP;
                       //update;
                }

        std::cout<<"Observation Size: "<<num_input<<std::endl;
        std::cout<<"Track Size: "<<tracks.size() <<std::endl;
              for(std::vector<int>::size_type i = 0; i != tracks.size(); i++)
              { 
                  tracks[i].mean_x=(tracks[i].x+tracks[i].mean_x)/2;
                  tracks[i].mean_y=(tracks[i].y+tracks[i].mean_y)/2;
                   tracks[i].var_x=(tracks[i].x-tracks[i].mean_x)*(tracks[i].x-tracks[i].mean_x);
                  tracks[i].var_y=(tracks[i].y-tracks[i].mean_y)*(tracks[i].y-tracks[i].mean_y);
                  //cout<< tracks[i].mean_x << "  "<< tracks[i].mean_y<<"  "<<tracks[i].var_x<<"  "<<tracks[i].var_y << endl;
              }//update_mean&Variance;
        
        //cout<< " in else loop";
        int l=0;
        marker.header.frame_id = "velodyne";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.points.resize(tracks.size());
          //marker.colors.resize(tracks.size());
        marker.scale.x = 1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;       
        for(std::vector<int>::size_type i = 0; i != tracks.size(); i++)
              { 
                   
                  marker.points[i].x = tracks[i].x;
                  marker.points[i].y = tracks[i].y;
                  marker.points[i].z = 0.0;
                 
                 /* marker.colors[i].a = 1.0; // Don't forget to set the alpha!
                  marker.colors[i].r = l/(l+1);
                  marker.colors[i].g = 1.0/(l+1);
                  marker.colors[i].b = 0.0;
                  */
                  marker.color.a = 1.0; // Don't forget to set the alpha!
                  marker.color.r = 0.0;
                  marker.color.g = 1.0;
                  marker.color.b = 0.0;
                  l=l+1;
                   
              }  
          }     
  }
   
 
 
}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/cluster_centroids", 10, chatterCallback);
  ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker> ( "visualization_marker", 10 );
  
  while (n.ok())
  {
    //msg->header.stamp = ros::Time::now().toNSec();
   
    vis_pub.publish (marker);
    ros::spinOnce ();
  }
  //ros::spin();
  return 0;
}
