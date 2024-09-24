#include <Arduboy2.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <avr/power.h>

Arduboy2 arduboy;

MPU6050 mpu;

Quaternion q;
VectorFloat gravity;
float ypr[3]; // yaw, pitch, roll

#define PITCH_OFFSET -3.07
#define ROLL_OFFSET -0.77
#define YAW_OFFSET -5.72

uint8_t fifoBuffer[64]; // FIFO storage buffer

float CubeM[8][3] = {
  {-40, -40, -40}, {40, -40, -40}, {40, 40, -40}, {-40, 40, -40},
  {-40, -40, 40}, {40, -40, 40}, {40, 40, 40}, {-40, 40, 40}
};

int CubeV[12][3] = {
  {5, 1, 8}, {1, 4, 8}, {6, 5, 7}, {5, 8, 7},
  {2, 6, 3}, {6, 7, 3}, {1, 2, 4}, {2, 3, 4},
  {4, 3, 8}, {3, 7, 8}, {5, 6, 1}, {6, 2, 1}
};

float ax = 0, ay = 0, az = 0;
float scale = 0.5; // Adjust the scale factor
float cameraDistance = 200; // Distance of the camera from the object

void setup() {
  arduboy.begin();

  power_twi_enable(); // Enable TWI (I2C) power

  Wire.begin();
  
  mpu.initialize();
  if (mpu.dmpInitialize() == 0) {
    mpu.setDMPEnabled(true);
  }
  arduboy.clear();
}

float degreesToRadians(float degrees) {
  return degrees * PI / 180.0;
}

void loop() {
  if (!arduboy.nextFrame()) return;
  arduboy.clear();

  arduboy.pollButtons();
  
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float pitch = ypr[1] * 180 / PI;
    float roll = ypr[2] * 180 / PI;
    float yaw = ypr[0] * 180 / PI;
    
    float z = yaw - YAW_OFFSET;
    float x = pitch - PITCH_OFFSET;
    float y = roll - ROLL_OFFSET;
    
    // Adjust x and y angles based on cube's rotation
    float temp_x = x;
    float temp_y = y;
    
    // Rotate the x and y angles based on the cube's current rotation angles
    x = temp_x * cos(az) - temp_y * sin(az);
    y = temp_x * sin(az) + temp_y * cos(az);
    
    ax = degreesToRadians(-y);
    ay = degreesToRadians(-x);
    az = degreesToRadians(-z);
  }

  float transformed[8][2];

  for (int i = 0; i < 8; i++) {
    float x = CubeM[i][0];
    float y = CubeM[i][1];
    float z = CubeM[i][2];

    float newX = x * cos(ay) * cos(az) + y * (cos(ax) * -sin(az) + sin(ax) * sin(ay) * cos(az)) + z * (-sin(ax) * -sin(az) + cos(ax) * sin(ay) * cos(az));
    float newY = x * cos(ay) * sin(az) + y * (cos(ax) * cos(az) + sin(ax) * sin(ay) * sin(az)) + z * (-sin(ax) * cos(az) + cos(ax) * sin(ay) * sin(az));
    float newZ = x * -sin(ay) + y * sin(ax) * cos(ay) + z * cos(ax) * cos(ay);

    // Apply perspective transformation
    float perspectiveScale = cameraDistance / (cameraDistance + newZ);
    transformed[i][0] = newX * perspectiveScale * scale + 64;
    transformed[i][1] = newY * perspectiveScale * scale + 32;
  }

  for (int i = 0; i < 12; i++) {
    int v0 = CubeV[i][0] - 1;
    int v1 = CubeV[i][1] - 1;
    int v2 = CubeV[i][2] - 1;

    // Calculate the normal vector of the face
    float ux = transformed[v1][0] - transformed[v0][0];
    float uy = transformed[v1][1] - transformed[v0][1];
    float vx = transformed[v2][0] - transformed[v0][0];
    float vy = transformed[v2][1] - transformed[v0][1];
    float normalZ = ux * vy - uy * vx;

    // If the normal vector is facing away from the camera, skip drawing the face
    if (normalZ > 0) {
      arduboy.drawLine(transformed[v0][0], transformed[v0][1], transformed[v1][0], transformed[v1][1], WHITE);
      arduboy.drawLine(transformed[v1][0], transformed[v1][1], transformed[v2][0], transformed[v2][1], WHITE);
      arduboy.drawLine(transformed[v2][0], transformed[v2][1], transformed[v0][0], transformed[v0][1], WHITE);
    }
  }

  arduboy.display();
}
