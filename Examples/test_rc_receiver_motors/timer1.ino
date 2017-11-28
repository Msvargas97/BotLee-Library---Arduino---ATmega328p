volatile unsigned long ovf_count_t1 = 0; //contador timer1 en el ISR

#ifdef __cplusplus
extern "C"{
#endif
ISR(TIMER1_OVF_vect) {
  static int i,cont;
  static bool flag;
  if(flagSelect){
    if(cont >= 800){
      cont = 0;
      flagSelect = false;
    }
  }
  if(++i >= 20){
  ovf_count_t1++;  
  i=0;
  cont += ( (flagSelect) ? 1 : 0);
  }
}
#ifdef __cplusplus
} // extern "C"
#endif

unsigned long getTimer() {
  unsigned long t;
  TIMER1_OFF();
  t = ovf_count_t1;
  TIMER1_ON();
  return t;
}
void resetTimer(){
  TIMER1_OFF();
  ovf_count_t1 = 0;
  TIMER1_ON();
}

