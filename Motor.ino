#include "Motor_config.h"

void setup() {
  Serial.begin(57600);
  Serial1.begin(115200);

  // ROS node initialization
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  
  // Subscriber
  nh.subscribe(cmd_vel_sub);

  // Publisher
  nh.advertise(odom_publisher);

  // Define the rotary encoder for left motor
  pinMode(ENCODER_A_LEFT, INPUT);
  pinMode(ENCODER_B_LEFT, INPUT);
  pinMode(ENCODER_A_RIGHT, INPUT);
  pinMode(ENCODER_B_RIGHT, INPUT);
  
  pinMode(Tx1Pin, INPUT);
  pinMode(Rx1Pin, OUTPUT);

  digitalWrite(ENCODER_A_LEFT, HIGH);
  digitalWrite(ENCODER_B_LEFT, HIGH);
  digitalWrite(ENCODER_A_RIGHT, HIGH);
  digitalWrite(ENCODER_B_RIGHT, HIGH);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_LEFT), encoderLeftMotor, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_RIGHT), encoderRightMotor, RISING);

  /* Velocity mode   */
  MB_WriteSingleRegFunCode_06H(L_Wheel, 0x2032, 0x0003);    // Left Set Profile Velocity Mode
  delay(10);
  MB_WriteSingleRegFunCode_06H(R_Wheel, 0x2032, 0x0003);    // Right Set Profile Velocity Mode
  delay(10);
  MB_WriteSingleRegFunCode_06H(L_Wheel, 0x2031, 0x0008);    // Left Motor enable
  delay(10);
  MB_WriteSingleRegFunCode_06H(R_Wheel, 0x2031, 0x0008);    // Right Motor enable
  delay(10);

  /* Absolute Position mode */
  /*
    MB_WriteSingleRegFunCode_06H(L_Wheel, 0x2032, 0x0002);
    delay(1000);
    MB_WriteSingleRegFunCode_06H(L_Wheel, 0x2036, 0x0064);
    delay(1000);
    MB_WriteSingleRegFunCode_06H(L_Wheel, 0x2031, 0x0008);
    delay(1000);
    MB_WriteMultipleRegFunCode_10H(L_Wheel, 0x2034, 0x0000, 0x5000);
    delay(1000);
    MB_WriteSingleRegFunCode_06H(L_Wheel, 0x2031, 0x0010);
    delay(1000);
    Serial1.end();
  */
}

void loop() {
  /* 10sec counter */
  /*
      now_time = millis();
      if (now_time - pre_time > 1000)
      {
          pre_time = now_time;
      }
  */

    //MB_WriteSingleRegFunCode_06H(L_Wheel, 0x203A, 0x000A);    // Left Set target speed 100RPM(0x0064 = 10)

    //MB_WriteSingleRegFunCode_06H(R_Wheel, 0x203A, 0xFFF6);    // Right Set target speed -100RPM(0xFF9C = -10)
  delay(4000);
  /*
  // update odometry
  float dt;
  float dx, dy, dz;
  float qx, qy, qz, qw;

  dt = (float)(millis() - odom_prev_time) * 0.001f;
  odom_prev_time = millis();

  motor_right_est = TICK2RAD * tick_left;
  motor_left_est = TICK2RAD * tick_right;

  // compute linear and angular estimated velocity
  linear_vel_est = WHEEL_RADIUS * (motor_right_est + motor_left_est) / 2.0f;
  angular_vel_est = (WHEEL_RADIUS / BASE_LENGTH) * (motor_right_est - motor_left_est);

  // compute translation and rotation
  yaw_est += angular_vel_est * dt;
  dx = cos(yaw_est) * linear_vel_est * dt;
  dy = sin(yaw_est) * linear_vel_est * dt;
  // dz = 0.0f;

  // compute quaternion
  // qx = 0.0f;
  // qy = 0.0f;
  qz = sin(abs(yaw_est) / 2.0f) * sign(yaw_est);
  qw = cos(abs(yaw_est) / 2.0f);

  // feed odom message
  odom.header.stamp = nh.now();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  odom.pose.pose.position.x += dx;
  odom.pose.pose.position.y += dy;
  odom.pose.pose.position.z = 0.0f;

  odom.pose.pose.orientation.x = 0.0f;
  odom.pose.pose.orientation.y = 0.0f;
  odom.pose.pose.orientation.z = qz;
  odom.pose.pose.orientation.w = qw;

  // Velocity expressed in base_link frame
  odom.twist.twist.linear.x = linear_vel_est;
  odom.twist.twist.linear.y = 0.0f;
  odom.twist.twist.angular.z = angular_vel_est;

  odom_publisher.publish(&odom);
  nh.spinOnce();*/
}

uint16_t MB_CRC16(uint8_t *pushMsg, uint8_t usDataLen)
{
  uint8_t uchCRCHi = 0xff;
  uint8_t uchCRCLo = 0xff;
  uint8_t uIndex;
  while (usDataLen--)
  {
    uIndex = uchCRCHi ^ *pushMsg++;
    uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
    uchCRCLo = auchCRCLo[uIndex];
  }
  return (uchCRCHi << 8 | uchCRCLo);
}

void MB_ReadRegFunCode_03H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
  uint16_t TxCount = 0;
  uint16_t crc = 0;

  Tx_Buf[TxCount++] = _addr;
  Tx_Buf[TxCount++] = 0x03;
  Tx_Buf[TxCount++] = _reg >> 8;
  Tx_Buf[TxCount++] = _reg;
  Tx_Buf[TxCount++] = _num >> 8;
  Tx_Buf[TxCount++] = _num;

  crc = MB_CRC16((uint8_t*)&Tx_Buf, TxCount);

  Tx_Buf[TxCount++] = crc >> 8;
  Tx_Buf[TxCount++] = crc;
  /* Transmit to ZLAC8015 */
  digitalWrite(Tx1Pin, HIGH);
  for (uint8_t i = 0 ; i < 8 ; i++)
  {
    Serial1.write(Tx_Buf[i]);
  }
  digitalWrite(Tx1Pin, LOW);
}

void MB_WriteSingleRegFunCode_06H(uint8_t _addr, uint16_t _reg, uint16_t _data)
{
  uint16_t TxCount = 0;
  uint16_t crc = 0;
  Tx_Buf[TxCount++] = _addr;            // 역주소에서
  Tx_Buf[TxCount++] = 0x06;             // 기능코드
  Tx_Buf[TxCount++] = _reg >> 8;        // 레지스터 주소 하이 바이트
  Tx_Buf[TxCount++] = _reg;             // 레지스터 주소 로우 바이트
  Tx_Buf[TxCount++] = _data >> 8;       // 레지스터(16bits) 개수가 높은 바이트
  Tx_Buf[TxCount++] = _data;            // 로우 바이트

  crc = MB_CRC16((uint8_t*)&Tx_Buf, TxCount);
  Tx_Buf[TxCount++] = crc >> 8;         // crc 하이 바이트
  Tx_Buf[TxCount++] = crc;              // crc 로우 바이트

  digitalWrite(Tx1Pin, HIGH);
  for (uint8_t i = 0 ; i < 8 ; i++)
  {
    Serial1.write(Tx_Buf[i]);
  }
  digitalWrite(Tx1Pin, LOW);
}

void MB_WriteMultipleRegFunCode_10H(uint8_t _addr, uint16_t _reg, uint16_t _num0, uint16_t _num1)
{
  uint16_t TxCount = 0;
  uint16_t crc = 0;

  Tx_Buf[TxCount++] = _addr;            // 역주소에서
  Tx_Buf[TxCount++] = 0x10;             // 기능코드
  Tx_Buf[TxCount++] = _reg >> 8;        // 레지스터 주소 하이 바이트
  Tx_Buf[TxCount++] = _reg;             // 레지스터 주소 로우 바이트
  Tx_Buf[TxCount++] = 0x00;
  Tx_Buf[TxCount++] = 0x02;             // 0x0002 = register address 2개
  Tx_Buf[TxCount++] = 0x04;             // Number of bytes = 4

  Tx_Buf[TxCount++] = _num0 >> 8;       // High 8 bits of data 0
  Tx_Buf[TxCount++] = _num0;
  Tx_Buf[TxCount++] = _num1 >> 8;       // High 8 bits of data 1
  Tx_Buf[TxCount++] = _num1;

  crc = MB_CRC16((uint8_t*)&Tx_Buf, TxCount);
  Tx_Buf[TxCount++] = crc;              // crc 로우 바이트
  Tx_Buf[TxCount++] = crc >> 8;         // crc 하이 바이트

  digitalWrite(Tx1Pin, HIGH);
  for (uint8_t i = 0 ; i < 13 ; i++)
  {
    Serial1.write(Tx_Buf[i]);
  }
  digitalWrite(Tx1Pin, LOW);
}

void vel_command()
{
  delay_cur_t = micros();  // 3.5 char byte : dealy 0.00030381sec (0.30381milli sec = 303.381micro sec)

  if (delay_cur_t - delay_pre1_t > 400)
  {
    MB_WriteSingleRegFunCode_06H(L_Wheel, 0x203A, 0x000A);    // Left Set target speed 100RPM(0x0064 = 10)
    delay_pre1_t = 800;
  }

  if (delay_cur_t - delay_pre2_t > 400)
  {
    MB_WriteSingleRegFunCode_06H(R_Wheel, 0x203A, 0xFFF6);    // Right Set target speed -100RPM(0xFF9C = -10)
  }
}

void CmdVel_Move()
{
  delay_cur_t = 0;
  delay_pre1_t = 0;
  delay_pre2_t = 0;

  xrpm_left_ref = 0x0000 + drpm_left_ref;
  xrpm_right_ref = 0x0000 + drpm_right_ref;

  delay_cur_t = micros();  // 3.5 char byte : dealy 0.00030381sec (0.30381milli sec = 303.381micro sec)

  if (delay_cur_t - delay_pre1_t > 400)
  {
    MB_WriteSingleRegFunCode_06H(L_Wheel, 0x203A, xrpm_left_ref);    // Left Set target speed 100RPM(0x0064 = 10)
    delay_pre1_t = 800;
  }

  if (delay_cur_t - delay_pre2_t > 400)
  {
    MB_WriteSingleRegFunCode_06H(R_Wheel, 0x203A, xrpm_right_ref);    // Right Set target speed -100RPM(0xFF9C = -10)
  }
}

void CmdVelCB(const geometry_msgs::Twist& cmd_vel)
{
  linear_vel_ref  = cmd_vel.linear.x;
  angular_vel_ref = cmd_vel.angular.z;

  drpm_left_ref  = linear_vel_ref - angular_vel_ref * (BASE_LENGTH / 2);
  drpm_right_ref = linear_vel_ref + angular_vel_ref * (BASE_LENGTH / 2);

  CmdVel_Move();
}

void encoderLeftMotor()
{
  if (digitalRead(ENCODER_A_LEFT) == digitalRead(ENCODER_B_LEFT)) tick_left++;
  else tick_left--;
}

void encoderRightMotor()
{
  if (digitalRead(ENCODER_A_RIGHT) == digitalRead(ENCODER_B_RIGHT)) tick_right--;
  else tick_right++;
}

/* EBIMU Example */
/*
    float euler[3];
    if (EBimuAsciiParser(euler, 3))
    {
    Serial.print("\n\r");
    Serial.print(euler[0]);   Serial.print(" ");
    Serial.print(euler[1]);   Serial.print(" ");
    Serial.print(euler[2]);   Serial.print(" ");
    }
    byte buffer [8] ;
    while (Serial2.available() > 0)
    {
    for (uint8_t i = 0 ; i < 8 ; i++)
    {
        buffer[i] = Serial2.read ();
        Serial.write(buffer[i]);
    }
    }
    Serial2.end();
*/

/*
  ISR(USART2_RX_vect) {
  char c = UDR0;
  Rx_Buf[rx_buf_head] = c;
  if (rx_buf_head == 64 - 1)
    rx_buf_head = 0;
  else
    rx_buf_head++;
  }
*/
