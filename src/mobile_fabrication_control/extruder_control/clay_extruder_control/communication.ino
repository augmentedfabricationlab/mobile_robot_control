// Reading data ====================================================================================
void read_header(EthernetClient c) {
  recv_header.msg_type = read_long(c);
  recv_header.msg_len = read_long(c);
  recv_header.msg_response = read_long(c);
}

void read_motor_data(EthernetClient c) {
  motordata.motor_state = read_long(c);
  motordata.motor_maxspeed = read_float(c);
  motordata.motor_speed = read_float(c);
}

void read_motor_state(EthernetClient c) {
  motordata.motor_state = read_long(c);
}

void read_digital_output_data(EthernetClient c) {
  dodata.digital_out_pin = read_long(c);
  dodata.do_pin_state = read_long(c);
}

// Sending data ====================================================================================

void send_header(EthernetClient c, HEADER hdr) {
  Serial.print("Sending: ");
  Serial.println(msg_names[hdr.msg_type]);
  send_long(c, hdr.msg_type);
  send_long(c, hdr.msg_len);
  send_long(c, hdr.msg_response);
}

void send_msg_received(EthernetClient c) {
  HEADER hdr = {MSG_RECEIVED, RETURN_MSG_LEN, RETURN_MSG_RSP};
  send_header(c, hdr);  
}

void send_msg_executed(EthernetClient c) {
  HEADER hdr = {MSG_EXECUTED, RETURN_MSG_LEN, RETURN_MSG_RSP};
  send_header(c, hdr);
}

void send_msg_info(EthernetClient c) {
  HEADER hdr = {MSG_INFO, 52, RETURN_MSG_RSP};
  send_header(c, hdr);
  send_string(c, info.arduino_ip, 16);
  send_string(c, info.arduino_gateway, 16);
  send_string(c, info.arduino_subnet, 16);
  send_long(c, info.arduino_port);
}

// Interface =======================================================================================

void execute_msg(EthernetClient c) {
  switch (recv_header.msg_type) {
          case MSG_STOP:
            motordata.motor_state = 0;
            motor_setup();
            break;
          
          case MSG_INFO:
            send_msg_info(c);
            break;
          
          case MSG_DODATA:
            read_digital_output_data(c);
            set_digital_out();
            break;
          
          case MSG_MOTORDATA:
            read_motor_data(c);
            motor_setup();
            break;
  
          case MSG_MOTORSTATE:
            read_motor_state(c);
            break;
     
          default:
            Serial.println("MSG TYPE UNKNOWN");
            break;
  }
}

void check_for_client(){
  EthernetClient client = server.available();
  if (client) {
    if (!CONNECTED) {
      Serial.println("New client connected");
      CONNECTED = true;
    }
    if (client.available() > 0) {
      // read the bytes incoming from the client:
      read_header(client);
      send_msg_received(client);
      Serial.println(recv_header.msg_type);
      execute_msg(client);
      send_msg_executed(client);
    }
    if (!client.connected()) {
      client.stop();
      CONNECTED == false;
      Serial.println("Client has been disconnected");
    }
  }
  check_client_interval.restart();
}
