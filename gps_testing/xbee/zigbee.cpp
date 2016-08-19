/****************************************************************************
*
*   Copyright (c) 2011 Carrick Detweiler
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation; either version 2 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program; if not, write to the Free Software
*   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*
* Long ago based on following the ROS tutorial.
*
******************************************************************************/

#include <ros/ros.h>
#include <zigbee/ATCmdResponse.h>
#include <zigbee/ATCmd.h>
#include <zigbee/ADC.h>
#include <cereal_port/CerealPort.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <string>
#include <vector>
using namespace std;


class Zigbee {
public:
  Zigbee();
  void parse();
  void sendATCommand(const char *s,int address);
  void sendATCommand(const char * s,int address, unsigned char frameID);
private:
  void sendPacket(unsigned char *data, int length);
  void parsePacketIO(unsigned char *data, int length);
  void parsePacket(unsigned char *data, int length);
  void parsePacketCmdResponse(unsigned char *data, int length);

  ///////Callbacks//////////
  void sendATCommandCallback(const zigbee::ATCmd::ConstPtr& cmd);

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle private_nh_;

  /**
   * Serial port we are using
   **/
  cereal::CerealPort * serialPort;

  /**
   * Number of ADCs the zigbee has.  Do not change this!
   **/
  static const int numADCs = 5;

  /**
   * Number of Digital IO pins the zigbee has.  Do not change this!
   **/
  static const int numDIOs = 9;

  /**
   * Portname to use
   **/
  string portname;
  int serial_rate_;

  /**
   * Our publishers for analog and digital I/O messages
   **/
  ros::Publisher adc_pub[numADCs];
  ros::Publisher dio_pub[numDIOs];

  /**
   * Publisher for responses to remote AT commands
   **/
  ros::Publisher remoteATResp_pub;

  /**
   * Subscriber to subscribe to remote AT commands (that we will send)
   **/
  ros::Subscriber remoteATCommand_sub;

};

/**
 * Constructor to advertise messages
 **/
Zigbee::Zigbee(void): private_nh_("~") {
  //Get/store the serial port
	portname = "/dev/ttyUSB0";
  private_nh_.param("serial_device",portname,portname);
  ros::NodeHandle nh;
  //Create and connect the serial port
  serialPort = new cereal::CerealPort();
  try{
	private_nh_.param("serial_rate",serial_rate_,19200);
    serialPort->open(portname.c_str(),serial_rate_);
    //serialPort->open(portname.c_str(),19200);
    ROS_INFO("Opened zigbee on serial port %s at rate %d",portname.c_str(), (int) serial_rate_);
    //serialPort->open(portname,57600);
    //serialPort->open(portname,115200);
    //serialPort->open(portname,9600);
  }catch(cereal::Exception& e){
    ROS_FATAL("Error opening serial port %s",portname.c_str());
  }

  char tmpstr[16];
  for(int i = 0; i < numADCs; i++){
    snprintf(tmpstr,16,"ADC%d",i);
    adc_pub[i] = nh.advertise<zigbee::ADC>(tmpstr,10);
  }
  for(int i = 0; i < numDIOs; i++){
    snprintf(tmpstr,16,"DIO%d",i);
    dio_pub[i] = nh.advertise<std_msgs::Bool>(tmpstr,10);
  }

  //Publisher for sending remote AT response messages
  remoteATResp_pub = nh.advertise<zigbee::ATCmdResponse>("RemoteATResponse",10);
  //Subscriber to remote AT messages we should send to remote radios
  remoteATCommand_sub = nh.subscribe("RemoteATCommand",10,&Zigbee::sendATCommandCallback,this);

}

/**
 * Packetize and send a packet with the specified data, which is the
 * API-specific structure.
 **/
void Zigbee::sendPacket(unsigned char *data, int length){
  char header[3];

  ROS_DEBUG_NAMED("RawTX","Zigbee sendPacket(): Start of sending packet");
  //Start byte
  header[0] = 0x7E;
  //Send the correct length
  header[1] = (length)>>8;
  header[2] = (length)&0xff;
  //Send
  serialPort->write(header,3);
  ROS_DEBUG_NAMED("RawTX","Zigbee sendPacket(): Sending (start)    %#x",header[0]);
  ROS_DEBUG_NAMED("RawTX","Zigbee sendPacket(): Sending (len MSB)  %#x",header[1]);
  ROS_DEBUG_NAMED("RawTX","Zigbee sendPacket(): Sending (lend LSB) %#x",header[2]);
  //Now send the data
  serialPort->write((char *)data,length);

  //Compute and send checksum
  unsigned char checksum = 0;
  for(int i = 0; i < length; i++){
    ROS_DEBUG_NAMED("RawTX","Zigbee sendPacket(): Sending %#x",data[i]);
    checksum += data[i];
    checksum &= 0xff;
  }
  checksum = 0xff - checksum;
  serialPort->write((char *)&checksum,1);
  ROS_DEBUG_NAMED("RawTX","Zigbee sendPacket(): Sending (chksum) %#x",checksum);
}

/**
 * Callback for getting AT sending messages
 **/
void Zigbee::sendATCommandCallback(const zigbee::ATCmd::ConstPtr& cmd){
  sendATCommand(cmd->at.c_str(),cmd->to16,cmd->frame);
}

/**
 * Send the AT command to the remote zigbee with the specified 16-bit address.
 **/
void Zigbee::sendATCommand(const char * s,int address){
  static unsigned char frameID = 1;
  if(frameID == 0) frameID++;
  sendATCommand(s,address,frameID++);
}

/**
 * Send the AT command to the remote zigbee with the specified 16-bit
 * address and a specific frameID.  If the frame ID is non-zero then
 * any response will contain this frameID as well.
 **/
void Zigbee::sendATCommand(const char * s,int address, unsigned char frameID){
  const unsigned int STR_LEN = 64;
  char buff[STR_LEN];
  ROS_DEBUG_NAMED("ATCmdsTX","Sending AT cmd to %#x (len %d): %s",address,strlen(s),s);

  int i = 0;
  //API identifier: Remote AT command
  buff[i++] = 0x17;
  //Frame ID (if non-zero ACKs will contain this number)
  buff[i++] = frameID;
  //64 bit address (ignored if 16 bit address is not 0xFFFE)
  buff[i++] = 0x00;buff[i++] = 0x00;buff[i++] = 0x00;buff[i++] = 0x00;
  buff[i++] = 0x00;buff[i++] = 0x00;buff[i++] = 0xFF;buff[i++] = 0xFF;
  //16 bit address (MSB first)
  buff[i++] = (address>>8)&0xff;buff[i++] = 0xff&address;
  //Indicate that changes should be applied immediately.
  buff[i++] = 0x02;

  if(strlen(s) > STR_LEN-i){
    ROS_ERROR("Zigbee sendATCommand(): Buffer size exceeded, skipping command: %s",s);
  }
  strncpy(buff+i,s,sizeof(s));
  sendPacket((unsigned char*)buff,strlen(s)+i);
}

/**
 * Parse a 16 bit address IO packet
 **/
void Zigbee::parsePacketIO(unsigned char *data, int length){
  int activeChannels = 0;
  int totalSamples;
  ros::Time startTime = ros::Time::now();

  //Need at least 4 bytes
  if(length < 4){
    ROS_ERROR("Zigbee parsePacketIO(): packet not long enough");
    return;
  }

  //Start at 1 as 0 is the packet ID byte
  int i = 1;

  //Not sure what the first 4 bytes do....
  i+=4;

  //Total samples is the number of samples sent based on the IT
  //register.  This isn't the number of channels read, rather it is
  //the number of samples per channel sent in this packet.
  totalSamples = data[i++];
  //Active channels are first two bytes. Format:
  // (bit 16) N/A A5 A4 A3 A2 A1 A0 D8   D7 D6 D5 D4 D3 D2 D1 D0  (bit 0)
  activeChannels = data[i++] << 8;
  activeChannels += data[i++];
  //For each sample...
  for(int sample = 0; sample < totalSamples; sample++){
    int digitalIO = 0;
    //0x1ff needs to be changed if numDIOs changes....
    if((activeChannels & 0x1ff) != 0){
      digitalIO = data[i++] << 8;
      digitalIO += data[i++];
    }
    for(int j=0;j<(numDIOs+numADCs);j++){
      if((activeChannels >> j)&0x01){
        if(j < numDIOs){
          ROS_DEBUG_NAMED("IOrx","Zigbee parsePacketIO(): Digital IO Channel %d: %#x",j,(digitalIO>>j)&0x01);
          std_msgs::Bool msg;
          msg.data = (digitalIO>>j)&0x01;
          dio_pub[j].publish(msg);
        }else{
          int sample = data[i++] << 8;
          sample += data[i++];
          ROS_DEBUG_NAMED("IOrx","Zigbee parsePacketIO(): ADC Channel %d Sample: %#x",j-numDIOs,sample);
          zigbee::ADC msg;
          msg.header.stamp = startTime;
          msg.data = sample;
          adc_pub[j-numDIOs].publish(msg);
        }
      }
    }
  }

  ROS_DEBUG_NAMED("IOrx","Zigbee parsePacketIO(): totalSamples: %d activeChannels: %#x",totalSamples,activeChannels);
  if(i > length){
    ROS_ERROR("Zigbee parsePacketIO(): ERROR exceeded data length!!! Packet len %d, used %d",length,i);
  }
}

/**
 * Parse API 0x97 which is a response from a remote AT command
 **/
void Zigbee::parsePacketCmdResponse(unsigned char *data, int length){
  int i = 0;
  zigbee::ATCmdResponse cmdResp;

  //API ID
  i++;
  //Frame ID
  cmdResp.frame = data[i++];
  //64 bit address
  cmdResp.from64 = data[i++];
  cmdResp.from64 = (cmdResp.from64<<8) + data[i++];
  cmdResp.from64 = (cmdResp.from64<<8) + data[i++];
  cmdResp.from64 = (cmdResp.from64<<8) + data[i++];
  cmdResp.from64 = (cmdResp.from64<<8) + data[i++];
  cmdResp.from64 = (cmdResp.from64<<8) + data[i++];
  cmdResp.from64 = (cmdResp.from64<<8) + data[i++];
  cmdResp.from64 = (cmdResp.from64<<8) + data[i++];
  //16 bit address
  cmdResp.from16 = data[i++] << 8;
  cmdResp.from16 |= data[i++];
  //Command name
  char cmd[3];
  cmd[0] = data[i++];
  cmd[1] = data[i++];
  cmd[2] = '\0';
  cmdResp.at = cmd;
  //Status
  cmdResp.status = data[i++];
  switch(cmdResp.status){
  case 0: cmdResp.status_str = "OK"; break;
  case 1: cmdResp.status_str = "Error"; break;
  case 2: cmdResp.status_str = "Invalid Cmd"; break;
  case 3: cmdResp.status_str = "Invalid Param"; break;
  case 4: cmdResp.status_str = "No Response"; break;
  default: cmdResp.status_str = "Unknown Response Code!";
  }
  //Copy in the data
  for(;i<length;i++){
    cmdResp.responseData.push_back(data[i]);
  }
  ROS_DEBUG_NAMED("RemoteATCmdRX","i is %d, length is %d",i,length);

  ROS_DEBUG_NAMED("RemoteATCmdRX","From (16-bit) %#x, frame:%#x, Cmd: %s, Response code: %s, Response data len: %d",
                  cmdResp.from16,cmdResp.frame,cmd,cmdResp.status_str.c_str(),cmdResp.responseData.size());

  if(i > length){
    ROS_ERROR("Zigbee parsePacketCmdResponse(): ERROR exceeded data length!!! Packet len %d, used %d",length,i);
  }

  remoteATResp_pub.publish(cmdResp);
}

/**
 * Parse a valid API packet received from the zigbee.  This is all of
 * the data that is received (without the API packet headers and
 * checksum, but with the packet headers itself).
 **/
void Zigbee::parsePacket(unsigned char *data, int length){
  ROS_DEBUG_NAMED("RawRX","Zigbee parsePacket(): Packet recieved with length %d and byte[0] %#x",
            length,data[0]);
  if(length < 1) return;
  
  //16-bit address packet
  if(data[0] == 0x83){
    parsePacketIO(data,length);
  }else if(data[0] == 0x97){
    //Remote Command Response
    parsePacketCmdResponse(data,length);
  }else{
    ROS_WARN("Zigbee parsePacket(): Unknown message type %#x",data[0]);
  }
}

#define STATE_START 0
#define STATE_LEN0 1
#define STATE_LEN1 2
#define STATE_DATA 3
#define STATE_CHECKSUM 4

/**
 * Parse any data that has come in on the serial line
 **/
void Zigbee::parse(void){
  static int state = STATE_START;
  static int length = 0;
  static int bytesRead = 0;
  static int checksum = 0;
  #define MAX_DATA_LEN 128
  static unsigned char data[MAX_DATA_LEN];

  unsigned char c;
  try{
    //Last argument is number of MS to wait for a byte
    while(serialPort->read((char *)&c,1,5)){
      ROS_DEBUG_NAMED("RawRX","Zigbee parse(): Got %c (%#x)",((c>=' ') && (c <= '~'))?c:'?',c);
      switch(state){
      case STATE_START:
        if(c == 0x7E){
          ROS_DEBUG_NAMED("RawRX","Zigbee parseByte(): Got start byte");
          state = STATE_LEN0;
        }
        break;
      case STATE_LEN0:
        length = c;
        state = STATE_LEN1;
        break;
      case STATE_LEN1:
        length = c + (length<<8);
        //Make sure we can fit this much data
        if(length > MAX_DATA_LEN){
          ROS_WARN("Zigbee parseByte(): Max data len exceeded, got %d, max %d",
                   length,MAX_DATA_LEN);
          break;
        }
        checksum = 0;
        bytesRead = 0;
        state = STATE_DATA;
        break;
      case STATE_DATA:
        checksum += (unsigned char) c;
        checksum &= 0xff;
        data[bytesRead++] = c;
        //See if we are done
        if(bytesRead >= length){
          state = STATE_CHECKSUM;
        }
        break;
      case STATE_CHECKSUM:
        //Final computation
        checksum = 0xff - checksum;
        if(((unsigned char)c) != checksum){
          ROS_WARN("Zigbee parseByte(): Invalid checksum");
          state = STATE_START;
          break;
        }
        parsePacket(data,length);
        state = STATE_START;
        break;
      default:
        state = STATE_START;
      }
    }
  }catch(cereal::TimeoutException e){
    //Don't do anything if there is a serial timeout
  }


}


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "zigbee");

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  //ros::spin();

  /**
   * Create our zigbee instance
   **/
  Zigbee zig;


  //Run loop at 50Hz
  ros::Rate loop_rate(50);

  //While ros still thinks things look good
  while (ros::ok()){
    //Parse any new data that has come in
    zig.parse();

    //zig.sendATCommand("CH",0x20);

    //Do any processing
    ros::spinOnce();

    //Sleep for the remaining time of this loop iteration
    loop_rate.sleep();
  }


  return 0;
}
