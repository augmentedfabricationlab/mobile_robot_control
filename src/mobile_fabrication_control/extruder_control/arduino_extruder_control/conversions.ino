union byte_float {
  byte b[4];
  float fval;
};

union byte_long {
  byte b[4];
  long lval;
};

float read_float(EthernetClient c) {
  byte_float bf;
  for (int i = 0; i < 4; i++) {
    bf.b[4-i-1] = c.read();
  }
  return bf.fval;
}

long read_long(EthernetClient c) {
  byte_long bl;
  for (int i = 0; i < 4; i++) {
    bl.b[4-i-1] = c.read();
  }
  return bl.lval;
}

void send_float(EthernetClient c, float float_value) {
  byte_float bf;
  bf.fval = float_value;
  for (int i = 0; i < 4; i++) {
    c.write(bf.b[4-i-1]);
  }
}

void send_long(EthernetClient c, long long_value) {
  // Serial.println(long_value);
  byte_long bl;
  bl.lval = long_value;
  for (int i = 0; i < 4; i++) {
    c.write(bl.b[4-i-1]);
  }
}

void send_string(EthernetClient c, String msg, int bytes) {
  byte msg_bytes[bytes];
  msg.getBytes(msg_bytes, bytes);
  for (int i = 0; i < bytes; i++) {
    c.write(msg_bytes[i]);
  }
}
