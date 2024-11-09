void matikanDriver() {
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);
  digitalWrite(R_EN1,LOW);
  digitalWrite(L_EN1,LOW);
  digitalWrite(R_EN2,LOW);
  digitalWrite(L_EN2,LOW);
  digitalWrite(R_EN3,LOW);
  digitalWrite(L_EN3,LOW);
}

void hidupkanDriver() {
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN1,HIGH);
  digitalWrite(L_EN1,HIGH);
  digitalWrite(R_EN2,HIGH);
  digitalWrite(L_EN2,HIGH);
  digitalWrite(R_EN3,HIGH);
  digitalWrite(L_EN3,HIGH);
}

void reset() {
  matikanDriver();
}

void maju() { 
  analogWrite(LPWM,(40+vSpeed));
  analogWrite(RPWM,0);
  analogWrite(LPWM1,0);
  analogWrite(RPWM1,(40+vSpeed));
  analogWrite(LPWM2,(25+vSpeed));
  analogWrite(RPWM2,0);
  analogWrite(LPWM3,0);
  analogWrite(RPWM3,(25+vSpeed));
}

void mundur() {
  analogWrite(LPWM,0);
  analogWrite(RPWM,(40+vSpeed));
  analogWrite(LPWM1,(40+vSpeed));
  analogWrite(RPWM1,0);
  analogWrite(LPWM2,0);
  analogWrite(RPWM2,(25+vSpeed));
  analogWrite(LPWM3,(25+vSpeed));
  analogWrite(RPWM3,0);
}

void kanan() {
  analogWrite(LPWM,(40+vSpeed));
  analogWrite(RPWM,0);
  analogWrite(LPWM1,(40+vSpeed));
  analogWrite(RPWM1,0);
  analogWrite(LPWM2,0);
  analogWrite(RPWM2,(40+vSpeed));
  analogWrite(LPWM3,0);
  analogWrite(RPWM3,(40+vSpeed));
}

void kiri() {
  analogWrite(LPWM,0);
  analogWrite(RPWM,(40+vSpeed));
  analogWrite(LPWM1,0);
  analogWrite(RPWM1,(40+vSpeed));
  analogWrite(LPWM2,(40+vSpeed));
  analogWrite(RPWM2,0);
  analogWrite(LPWM3,(40+vSpeed));
  analogWrite(RPWM3,0);
}

void putar_kanan() {
  analogWrite(LPWM,0);
  analogWrite(RPWM,(30+vSpeed));
  analogWrite(LPWM1,0);
  analogWrite(RPWM1,(30+vSpeed));
  analogWrite(LPWM2,0);
  analogWrite(RPWM2,(30+vSpeed));
  analogWrite(LPWM3,0);
  analogWrite(RPWM3,(30+vSpeed));
}

void putar_kiri() {
  analogWrite(LPWM,(30+vSpeed));
  analogWrite(RPWM,0);
  analogWrite(LPWM1,(30+vSpeed));
  analogWrite(RPWM1,0);
  analogWrite(LPWM2,(30+vSpeed));
  analogWrite(RPWM2,0);
  analogWrite(LPWM3,(30+vSpeed));
  analogWrite(RPWM3,0);
}

void kanan_bawah() {
  analogWrite(LPWM,0);
  analogWrite(RPWM,0);
  analogWrite(LPWM1,(30+vSpeed));
  analogWrite(RPWM1,0);
  analogWrite(LPWM2,0);
  analogWrite(RPWM2,(30+vSpeed));
  analogWrite(LPWM3,0);
  analogWrite(RPWM3,0);
}

void kiri_atas() {
  analogWrite(LPWM,0);
  analogWrite(RPWM,0);
  analogWrite(LPWM1,0);
  analogWrite(RPWM1,(30+vSpeed));
  analogWrite(LPWM2,(30+vSpeed));
  analogWrite(RPWM2,0);
  analogWrite(LPWM3,0);
  analogWrite(RPWM3,0);
}

void kiri_bawah() {
  analogWrite(LPWM,0);
  analogWrite(RPWM,(30+vSpeed));
  analogWrite(LPWM1,0);
  analogWrite(RPWM1,0);
  analogWrite(LPWM2,0);
  analogWrite(RPWM2,0);
  analogWrite(LPWM3,(30+vSpeed));
  analogWrite(RPWM3,0);
}

void kanan_atas() {
  analogWrite(LPWM,(30+vSpeed));
  analogWrite(RPWM,0);
  analogWrite(LPWM1,0);
  analogWrite(RPWM1,0);
  analogWrite(LPWM2,0);
  analogWrite(RPWM2,0);
  analogWrite(LPWM3,0);
  analogWrite(RPWM3,(30+vSpeed));
}

void berhenti() {   
  analogWrite(LPWM,0);
  analogWrite(RPWM,0);
  analogWrite(LPWM1,0);
  analogWrite(RPWM1,0);
  analogWrite(LPWM2,0);
  analogWrite(RPWM2,0);
  analogWrite(LPWM3,0);
  analogWrite(RPWM3,0);
}
