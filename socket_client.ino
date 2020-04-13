/*
 * WebSocketClient.ino
 *
 *  Created on: 24.05.2015
 *
 */

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {

    // check connections
    case WStype_DISCONNECTED:
      USE_SERIAL.printf("[WSc] Disconnected!\n");
      break;
    case WStype_CONNECTED: {
      USE_SERIAL.printf("[WSc] Connected to url: %s\n", payload);

      // send message to server when Connected
      webSocket.sendTXT("hello from huzzah");
    }
      break;
    case WStype_TEXT:
      USE_SERIAL.printf("[WSc] get text: %s\n", payload);
      // send message to server
//      if ((char*)payload == "pls send midi from my plant called théodore") 
        webSocket.sendTXT(midiToSend);

      break;
    case WStype_BIN:
      USE_SERIAL.printf("[WSc] get binary length: %u\n", length);
      hexdump(payload, length);

      // send data to server
      // webSocket.sendBIN(payload, length);
      break;
        case WStype_PING:
            // pong will be send automatically
            USE_SERIAL.printf("[WSc] get ping\n");
            break;
        case WStype_PONG:
            // answer to a ping we send
            USE_SERIAL.printf("[WSc] get pong\n");
            break;
    }

}

//void loop() {
//  webSocket.loop();
//}
