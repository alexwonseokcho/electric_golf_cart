//consider constant height of remote above ground
const float anchor_location[3][3] = {
  //x, y, z - z not used but for future
  //0, 0 is the center of the robot 
  {0, 0.4, 0},
  {-0.4, -0.4, 0},
  {0.4, -0.4, 0},
};

// typedef struct anchorDistances {
//   float anc1;
//   float anc2;
//   float anc3;
// }anchorDistances;

typedef struct tagLocation{
  float direction;
  float magnitude;
}tagLocation;

// anchorDistances ancReading; declared in esp_now_functions

anchorDistances avgAnchorReading;

void setupAnchor(){
  ancReading.anc1 = 0;
  ancReading.anc2 = 0;
  ancReading.anc3 = 0;
  
  avgAnchorReading.anc1 = -1;
  avgAnchorReading.anc2 = -1;
  avgAnchorReading.anc3 = -1;
}

const float avgWeight = 0.5;

void processAnchorDistance(){
  if(avgAnchorReading.anc1 == -1){
    avgAnchorReading.anc1 = ancReading.anc1;
    avgAnchorReading.anc2 = ancReading.anc2;
    avgAnchorReading.anc3 = ancReading.anc3;
  }
  else{
    avgAnchorReading.anc1 = avgAnchorReading.anc1 * (1.0 - avgWeight) + ancReading.anc1 * avgWeight;
    avgAnchorReading.anc2 = avgAnchorReading.anc2 * (1.0 - avgWeight) + ancReading.anc2 * avgWeight;
    avgAnchorReading.anc3 = avgAnchorReading.anc3 * (1.0 - avgWeight) + ancReading.anc3 * avgWeight;
  }
  Serial.printf("front: %.2f \t", avgAnchorReading.anc1);
  Serial.printf("left: %.2f \t", avgAnchorReading.anc2);
  Serial.printf("right: %.2f \t", avgAnchorReading.anc3);



}

void calculatePosition() {
  float anc1_x = anchor_location[0][0];
  float anc1_y = anchor_location[0][1];
  float anc2_x = anchor_location[1][0];
  float anc2_y = anchor_location[1][1];
  float anc3_x = anchor_location[2][0];
  float anc3_y = anchor_location[2][1];
  float anc1_dist = avgAnchorReading.anc1;
  float anc2_dist = avgAnchorReading.anc2;
  float anc3_dist = avgAnchorReading.anc3;
  
     /* This gives for granted that the z plane is the same for anchor and tags */
    double A = ( (-2*anc1_x) + (2*anc2_x) );
    double B = ( (-2*anc1_x) + (2*anc2_y) );
    double C = (anc1_dist*anc1_dist) - (anc2_dist*anc2_dist) - (anc1_x*anc1_x) + (anc2_x*anc2_x) - (anc1_y*anc1_y) + (anc2_y*anc2_y);
    double D = ( (-2*anc2_x) + (2*anc3_x) );
    double E = ( (-2*anc2_y) + (2*anc3_y) );
    double F = (anc2_dist*anc2_dist) - (anc3_dist*anc3_dist) - (anc2_x*anc2_x) + (anc3_x*anc3_x) - (anc2_y*anc2_y) + (anc3_y*anc3_y);

    float x = (C*E-F*B) / (E*A-B*D);
    float y = (C*D-A*F) / (B*D-A*E);
  Serial.printf("x: %.2f \t", x);
  Serial.printf("y: %.2f \n", y);

  float theta = atan2(x, y) * 360 / (3.1416*2.0);
  float r = sqrt(x*x + y*y);
  Serial.printf("r: %.2f \t theta: %.2f\n", r, theta);

}


void printAnchorLocation(){
  
}

