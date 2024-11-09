void commandCallback(const std_msgs::String& msg) {
  String receivedString = msg.data;

  cmd_status.data = "received";
  cmd_status_pub.publish(&cmd_status);

  if (receivedString == "reset") {
    if (!isReset) {
      isReset = true;
      reset();
    } else {
      reset();
      isReset = false;

      hidupkanDriver();
    }
  }
  else if (!isCommandDone) {
    cmd_status.data = "[Arduino] Sedang menjalankan perintah|0";
    cmd_status_pub.publish(&cmd_status);
    return;
  }
  else if (isReset) {
    cmd_status.data = "[Arduino] RESETTING...|0";
    cmd_status_pub.publish(&cmd_status);
    return;
  }
  isCommandDone = false;

  // motor---
  if (receivedString == "maju") {
    maju();
  }
  else if (receivedString == "mundur") {
    mundur();
  }
  else if (receivedString == "kanan") {
    kanan();
  }
  else if (receivedString == "kiri") {
    kiri();
  }
  else if (receivedString == "putar_kanan") {
    putar_kanan();
  }
  else if (receivedString == "putar_kiri") {
    putar_kiri();
  }
  else if (receivedString == "kanan_bawah") {
    kanan_bawah();
  }
  else if (receivedString == "kiri_atas") {
    kiri_atas();
  }
  else if (receivedString == "kiri_bawah") {
    kiri_bawah();
  }
  else if (receivedString == "kanan_atas") {
    kanan_atas();
  }
  else if (receivedString == "berhenti") {
    berhenti();
  }

  isCommandDone = true;
  cmd_status.data = "[Arduino] Selesai menjalankan perintah|1";
  cmd_status_pub.publish(&cmd_status);
}
