/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////Nuri Motor Protocol/////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
//속도: 1rpm 000A 5rpm 0032, 10rpm 0064, 15rpm 0096, 25rpm 00FA, 50rpm 01F4 75pm 2EE 100rpm 03E8//
//위치: 1도: 0064, 5도 01F4, 10 03E8, 15 05DC, 20 07D0, 45 1194, 90 2328
//시간: 0.1초: 0001, 0.5초:0005, 1초:000A, 1.5초:000F, 2초: 0014
//PID GAIN: 0x00~0xFE, (0~254)
//전류: 0x01 ~ 정격전류, 1 ~ 정격전류(x100 mA)
//피드백 요청: Mod = 0xA0(ping), 0xA1(위치) 0xA2(속도), 0xA3(위치제어기),0xA4(속도제어기),0xA5...
//Baudrate: 0x06 9600, 0x0D 115200

///////////////////////////////송신///////////////////////////////////////////////////////////////
//위치, 속도 제어: Pos_Vel(아이디, 방향, 위, 치, 속, 도 ) done
//가감속 위치 제어: Acc_Pos(아이디, 방향, 위, 치, 시간 ) done
//가감속 속도 제어: Acc_Vel(아이디, 방향, 속, 도, 시간 ) done
//위치 제어기 설정: Pos_Gain(아이디, P, I, D, 전류 ) done
//속도 제어기 설정: Vel_Gain(아이디, P, I, D, 전류 ) done
//아이디 설정: Set_ID(바꾸고 싶은 아이디, 바꿀 아이디) done
//통신 속도 설정: Baud_Set(아이디, 통신속도) done
//통신 응답 시간 설정: Baud_Time_Set(아이디, 응답시간) done
//제어 On/Off 설정: Con_ONOFF(아이디, ON/OFF) done
//위치 제어모드 설정: Pos_Con_Mode(아이디, 절대/상대 위치제어 모드) done
//위치 초기화: Pos_Init(아이디) done
//공장 초기화: Initiallizing(아이디) done
//피드백 요청: FB_Request(아이디, 모드)

///////////////////////////////수신////////////////////////////////////////////////////////////////
//Ping: Ping(아이디)
//위치 피드백: Pos_FB(아이디)
//속도 피드백: Vel_FB(아이디)
//위치 제어기 피드백: Pos_Con_FB(아이디)
//속도 제어기 피드백: Vel_Con_FB(아이디)
//통신 응답시간 피드백: Commu_Respond_FB(아이디)
//제어 On/Off 피드백: Con_ONOFF_FB(아이디)
//위치 제어 모드 피드백: Pos_Con_Mode_FB(아이디)

#define ID0 0x00
#define ID1 0x01
#define CCW 0x00
#define CW 0x01
#define ON 0x00
#define OFF 0x01
#define ABS 0x00 // 절대
#define RELA 0x01 // 상대

// 모터 돌림
// ㄹㄹ
// ㄹㅇㄹ
// ㄹㅇㄹ

void setup() {
      Serial.begin(9600);
      Acc_Pos(ID1, CCW, 0x11, 0x94, 0x32);
      Acc_Pos(ID0, CW, 0x11, 0x94, 0x32);

      delay(10000);

      Acc_Pos(ID1, CW, 0x00, 0x00, 0x32);
      Acc_Pos(ID0, CCW, 0x00, 0x00, 0x32);

      
      /*Acc_Pos(ID0, CCW, 0x23, 0x28, 0x32);
      Acc_Pos(ID1, CW, 0x23, 0x28, 0x32);  
      delay(10000);
      Acc_Pos(ID0, CW, 0x00, 0x00, 0x32);
      Acc_Pos(ID1, CCW, 0x00, 0x00, 0x32);*/       
}

void Pos_Vel(int ID, int Dir, int posi, int tion, int velo, int city){
    int Check_Sum = 0;
    int Data_Size = 0x07;
    int Mode = 0x01;
    Check_Sum = ~(ID + Data_Size + Mode + Dir + posi + tion + velo + city);  
    
    Serial.write(0xFF); //고정값
    Serial.write(0xFE);
    Serial.write(ID);  //ID 설정
    Serial.write(Data_Size); //Data 크기
    Serial.write(Check_Sum); //Check Sum
    Serial.write(Mode); //모드
    Serial.write(Dir); // 방향
    Serial.write(posi); //위
    Serial.write(tion); //치
    Serial.write(velo); //속
    Serial.write(city); //도
}

void Acc_Pos(int ID, int Dir, int posi, int tion, int sec){
    int Check_Sum = 0x00;
    int Data_Size = 0x06;
    int Mode = 0x02;
    Check_Sum = ~(ID + Data_Size + Mode + Dir + posi + tion + sec);
    
    Serial.write(0xFF); //고정값
    Serial.write(0xFE);
    Serial.write(ID);  //ID 설정
    Serial.write(Data_Size); //Data 크기
    Serial.write(Check_Sum); //Check Sum
    Serial.write(Mode); //모드
    Serial.write(Dir); // 방향
    Serial.write(posi); //위
    Serial.write(tion); //치
    Serial.write(sec); //위치 도달 시간 0x01 ~ 0xFF(1 ~ 255)[0.1s]    
}

void Acc_Vel(int ID, int Dir, int velo, int city, int sec){
    int Check_Sum = 0x00;
    int Data_Size = 0x06;
    int Mode = 0x03;
    Check_Sum = ~(ID + Data_Size + Mode + Dir + velo + city + sec);
    
    Serial.write(0xFF); //고정값
    Serial.write(0xFE);
    Serial.write(ID);  //ID 설정
    Serial.write(Data_Size); //Data 크기
    Serial.write(Check_Sum); //Check Sum
    Serial.write(Mode); //모드
    Serial.write(Dir); // 방향
    Serial.write(velo); //속
    Serial.write(city); //도
    Serial.write(sec); //속도 도달 시간 0x01 ~ 0xFF(1 ~ 255)[0.1s]    
}

void Pos_Gain(int ID, int Kp, int Ki, int Kd, int Current){
    int Check_Sum = 0x00;
    int Data_Size = 0x06;
    int Mode = 0x04;
    Check_Sum = ~(ID + Data_Size + Mode + Kp + Ki + Kd + Current);
    
    Serial.write(0xFF); //고정값
    Serial.write(0xFE);
    Serial.write(ID);  //ID 설정
    Serial.write(Data_Size); //Data 크기
    Serial.write(Check_Sum); //Check Sum
    Serial.write(Mode); //모드
    Serial.write(Kp); // P
    Serial.write(Ki); // I
    Serial.write(Kd); // D
    Serial.write(Current); //전류
}

void Vel_Gain(int ID, int Kp, int Ki, int Kd, int Current){
    int Check_Sum = 0x00;
    int Data_Size = 0x06;
    int Mode = 0x05;
    Check_Sum = ~(ID + Data_Size + Mode + Kp + Ki + Kd + Current);
    
    Serial.write(0xFF); //고정값
    Serial.write(0xFE);
    Serial.write(ID);  //ID 설정
    Serial.write(Data_Size); //Data 크기
    Serial.write(Check_Sum); //Check Sum
    Serial.write(Mode); //모드
    Serial.write(Kp); // P
    Serial.write(Ki); // I
    Serial.write(Kd); // D
    Serial.write(Current); //전류
}

void Set_ID(int Current_ID, int Next_ID){
    int Check_Sum = 0;
    int Data_Size = 0x03;
    int Mode = 0x06;
    Check_Sum = ~(Current_ID + Data_Size + Mode + Next_ID);  
    
    Serial.write(0xFF); //고정값
    Serial.write(0xFE);
    Serial.write(Current_ID);  //ID 설정
    Serial.write(Data_Size); //Data 크기
    Serial.write(Check_Sum); //Check Sum
    Serial.write(Mode); //모드
    Serial.write(Next_ID); // 방향
}

void Baud_Set(int ID, int Baud){
    int Check_Sum = 0x00;
    int Data_Size = 0x03;
    int Mode = 0x07;
    Check_Sum = ~(ID + Data_Size + Mode + Baud);
    
    Serial.write(0xFF); //고정값
    Serial.write(0xFE);
    Serial.write(ID);  //ID 설정
    Serial.write(Data_Size); //Data 크기
    Serial.write(Check_Sum); //Check Sum
    Serial.write(Mode); //모드
    Serial.write(Baud); // 절대 위치 제어

}

void Baud_Time_Set(int ID, int Time){
    int Check_Sum = 0x00;
    int Data_Size = 0x03;
    int Mode = 0x08;
    Check_Sum = ~(ID + Data_Size + Mode + Time );
    
    Serial.write(0xFF); //고정값
    Serial.write(0xFE);
    Serial.write(ID);  //ID 설정
    Serial.write(Data_Size); //Data 크기
    Serial.write(Check_Sum); //Check Sum
    Serial.write(Mode); //모드
    Serial.write(Time); //Mirco Sec
}

void Con_ONOFF(int ID, int ONOFF){
    int Check_Sum = 0x00;
    int Data_Size = 0x03;
    int Mode = 0x0A;
    Check_Sum = ~(ID + Data_Size + Mode + ONOFF );

    Serial.write(0xFF); //고정값
    Serial.write(0xFE);
    Serial.write(ID);  //ID 설정
    Serial.write(Data_Size); //Data 크기
    Serial.write(Check_Sum); //Check Sum
    Serial.write(Mode); //모드
    Serial.write(ONOFF); //Mirco Sec
}

void Pos_Con_Mode(int ID, int Mod){
    int Check_Sum = 0x00;
    int Data_Size = 0x03;
    int Mode = 0x0B;
    Check_Sum = ~(ID + Data_Size + Mode + Mod);
    
    Serial.write(0xFF); //고정값
    Serial.write(0xFE);
    Serial.write(ID);  //ID 설정
    Serial.write(Data_Size); //Data 크기
    Serial.write(Check_Sum); //Check Sum
    Serial.write(Mode); //모드
    Serial.write(Mod); // 절대 위치 제어

}

void Pos_Init(int ID){
    int Check_Sum = 0x00;
    int Data_Size = 0x02;
    int Mode = 0x0C; 
    Check_Sum = ~(ID + Data_Size + Mode);
  
    Serial.write(0xFF); //고정값
    Serial.write(0xFE);
    Serial.write(ID);  //ID 설정
    Serial.write(Data_Size); //Data 크기
    Serial.write(Check_Sum); //Check Sum
    Serial.write(Mode); //모드
}

void Initiallizing(int ID){
    int Check_Sum = 0x00;
    int Data_Size = 0x02;
    int Mode = 0x0D;    
    Check_Sum = ~(ID + Data_Size + Mode);    
    
    Serial.write(0xFF); //고정값
    Serial.write(0xFE);
    Serial.write(ID);  //ID 설정
    Serial.write(Data_Size); //Data 크기
    Serial.write(Check_Sum); //Check Sum
    Serial.write(Mode); //모드
}

void FB_Request(int ID, int Mode){
    int Check_Sum = 0x00;
    int Data_Size = 0x02;  
    Check_Sum = ~(ID + Data_Size + Mode);    
    
    Serial.write(0xFF); //고정값
    Serial.write(0xFE);
    Serial.write(ID);  //ID 설정
    Serial.write(Data_Size); //Data 크기
    Serial.write(Check_Sum); //Check Sum
    Serial.write(Mode); //모드
}

void loop() {
  
}
