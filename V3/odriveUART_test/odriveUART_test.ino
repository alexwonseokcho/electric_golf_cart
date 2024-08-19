 
#include <ODriveUART.h>
#include <HardwareSerial.h>

// pin 12: RX - connect to ODrive TX
// pin 11: TX - connect to ODrive RX
HardwareSerial odrive_serial(1);

ODriveUART odrive(odrive_serial);

void setup() {
  odrive_serial.begin(115200, SERIAL_8N1, 12, 11);

  Serial.begin(115200); // Serial to PC
  
  delay(10);

  Serial.println("Waiting for ODrive...");
  while (odrive.getState() == AXIS_STATE_UNDEFINED) {
    delay(100);
  }

  Serial.println("found ODrive");
  
  Serial.print("DC voltage: ");
  Serial.println(odrive.getParameterAsFloat("vbus_voltage"));
  
  Serial.println("Enabling closed loop control...");
  while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrive.clearErrors();
    odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    delay(10);
  }
  
  Serial.println("ODrive running!");
}

void loop() {

  //turns/2
  odrive_serial.write("w axis0.controller.config.vel_ramp_rate 0.5\n");
  // delay(10);
  // odrive_serial.write("w axis0.controller.input_vel 1.0");
  odrive.setVelocity(
    1
  );
  delay(5000);
  Serial.println(odrive.getParameterAsFloat("axis0.controller.config.vel_ramp_rate"));

  odrive_serial.write("w axis0.controller.config.vel_ramp_rate 10\n");
  // odrive.setParameter("axis0.controller.config.vel_ramp_rate", 10.0);
  odrive.setVelocity(
    0
  );
  delay(1000);
  // odrive_serial.write("w axis0.controller.input_vel 0.0");
  Serial.println(odrive.getParameterAsFloat("axis0.controller.config.vel_ramp_rate"));

  while(true){yield();}

  

  ODriveFeedback feedback = odrive.getFeedback();
  Serial.print("pos:");
  Serial.print(feedback.pos);
  Serial.print(", ");
  Serial.print("vel:");
  Serial.print(feedback.vel);
  Serial.println();
}
