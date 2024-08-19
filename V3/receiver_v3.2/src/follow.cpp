#include "follow.hpp"
#include "pins.hpp"
#include <Arduino.h>
#include <Wire.h>

#define FRONT_ADDRESS 5
#define LEFT_ADDRESS 6
#define RIGHT_ADDRESS 7

//consider constant height of remote above ground
//TO DO: check if the anchor locations are correct
const double anchor_location[3][3] = {
  //x, y, z - z not used but for future
  //0, 0 is the center of the robot 
  {0, 0.4, 0},
  {-0.4, -0.4, 0},
  {0.4, -0.4, 0},
};

struct anchor_distance{
    double frontDistance;
    double leftDistance;
    double rightDistance;
};

struct anchor_packet{
    double distance;
    uint8_t status;
};

struct remote_location{
    double r;
    double theta;
};

anchor_distance anchorDistances = {  //in meters
    .frontDistance = 0,
    .leftDistance = 0,
    .rightDistance = 0
};

remote_location remoteLocation = { //polar coords of the remote (define the reference frame later)
    .r = 0,
    .theta = 0
};

anchor_packet anchorDataPacket;

bool pollAnchors();
void calculatePosition();

void followTask(void *){    
    delay(10);
    Wire1.begin(ESP_I2C_SDA, ESP_I2C_SCL, 100000);    
    delay(500);


    //https://arxiv.org/pdf/2103.01885
    while(true){
        //add follow code here
        if(!pollAnchors()) return; //TO DO: check if I can actually return from this task and if it's safe to do so

        calculatePosition();

        //write to shared state to command requestedmotion task

        vTaskDelay(pdMS_TO_TICKS(0.5));
    }
    vTaskDelete( NULL );
}

bool pollAnchors(){
    Wire1.requestFrom(FRONT_ADDRESS, sizeof(anchor_packet));
    Wire.readBytes((byte*)&anchorDataPacket, sizeof(anchorDataPacket)); //what if it doesn't return anything?? check this failure mode

    if(anchorDataPacket.status == 0) return false;
    anchorDistances.frontDistance = anchorDataPacket.distance;



    Wire1.requestFrom(LEFT_ADDRESS, sizeof(anchor_packet));
    Wire.readBytes((byte*)&anchorDataPacket, sizeof(anchorDataPacket));

    if(anchorDataPacket.status == 0) return false;
    anchorDistances.leftDistance = anchorDataPacket.distance;


    Wire1.requestFrom(RIGHT_ADDRESS, sizeof(anchor_packet));
    Wire.readBytes((byte*)&anchorDataPacket, sizeof(anchorDataPacket));

    if(anchorDataPacket.status == 0) return false;
    anchorDistances.rightDistance = anchorDataPacket.distance;

    return true;
}

void calculatePosition() {
  double anc1_x = anchor_location[0][0];
  double anc1_y = anchor_location[0][1];
  double anc2_x = anchor_location[1][0];
  double anc2_y = anchor_location[1][1];
  double anc3_x = anchor_location[2][0];
  double anc3_y = anchor_location[2][1];
  double anc1_dist = anchorDistances.frontDistance;
  double anc2_dist = anchorDistances.leftDistance;
  double anc3_dist = anchorDistances.rightDistance;
  
     /* This gives for granted that the z plane is the same for anchor and tags */
    double A = ( (-2*anc1_x) + (2*anc2_x) );
    double B = ( (-2*anc1_x) + (2*anc2_y) );
    double C = (anc1_dist*anc1_dist) - (anc2_dist*anc2_dist) - (anc1_x*anc1_x) + (anc2_x*anc2_x) - (anc1_y*anc1_y) + (anc2_y*anc2_y);
    double D = ( (-2*anc2_x) + (2*anc3_x) );
    double E = ( (-2*anc2_y) + (2*anc3_y) );
    double F = (anc2_dist*anc2_dist) - (anc3_dist*anc3_dist) - (anc2_x*anc2_x) + (anc3_x*anc3_x) - (anc2_y*anc2_y) + (anc3_y*anc3_y);

    double x = (C*E-F*B) / (E*A-B*D);
    double y = (C*D-A*F) / (B*D-A*E);

    Serial.printf("x: %.2f \t", x);
    Serial.printf("y: %.2f \n", y);

    double theta = atan2(x, y) * 360 / (3.1416*2.0);
    double r = sqrt(x*x + y*y);
    Serial.printf("r: %.2f \t theta: %.2f\n", r, theta);

    remoteLocation.r = r;
    remoteLocation.theta = theta;
}