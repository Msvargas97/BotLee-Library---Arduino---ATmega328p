void rot90(int8_t  dir){
int M1,M2;
getSpeedsFix(&M1,&M2,-360*dir,-360*dir); //Calcula según el voltaje de la bateria
motors.setSpeeds(M1,M2);
delay(40);
motors.stop();
}

void arco(int8_t dir){
int M1,M2;	
getSpeedsFix(&M1,&M2,
  ((dir*-1 > 0) ? 260 : 360),
  ((dir*-1 > 0) ? 360 : 260));
motors.setSpeeds(-M1,M2);
delay(400);
motors.stop();
}

void rot45(int8_t dir){
int M1,M2;
getSpeedsFix(&M1,&M2,
            260*dir,
            -260*dir); //Calcula según el voltaje de la bateria
motors.setSpeeds(M1,M2);
delay(18);
motors.stop();
}
