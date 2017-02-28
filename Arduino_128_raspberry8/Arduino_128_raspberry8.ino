// MediaFlow,
// TinyFarmer MasterNode Impletation

#include <ArduinoJson.h>

#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>

#define NETWORKID     50  //the same on all nodes that talk to each other
#define NODEID        1  

#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY      RF69_915MHZ
#define ENCRYPTKEY     "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HCW    true // set to 'true' if you are using an RFM69HCW module

// 기타 설정
#define SERIAL_SPEED                  115200 //115200 /* 115200 /* 시리얼 속도 */

#define RFM69_CS      8
#define RFM69_IRQ     7
#define RFM69_IRQN    7  // Pin 2 is IRQ 0!

#define JSON_BUFFER_SIZE              200 /* JSON 버퍼 사이즈 */

int16_t packetnum = 0;  // packet counter, we increment per xmission

RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);

bool promiscuousMode = false;

//로봇청소기 설정
int pins[] = {3, 4, 13, 14, 28, 29, 30, 31, 32 ,33 ,34, 35, 41, 42, 43, 44};
int adcpin[] = {A0, A1, A2, A3, A4, A5, A6, A7};
int adcdata[] = {0, 1, 2, 3, 4, 5, 6, 7}; 
int j = 0;
int y = 0;
char index;
char index1;
char Position;
int lootin;
int lootin10;
int num;

typedef struct 
{
  int           nodeId; //store this nodeId
  float         temp;   //temperature maybe?
  float         hum;   //temperature maybe?
  long          ill;
  int           gas;
  float         PM25;
  float         PM10;
} Payload;
Payload theData;

void setup()
{
  while (!Serial); 
  
  for(int i = 0 ; i < 16; i++)
  { 
    pinMode(pins[i], OUTPUT);
  }
  
  Serial.begin(SERIAL_SPEED);
  Serial.println("Feather RFM69HCW Receiver");
  
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  if (IS_RFM69HCW) 
  {
    radio.setHighPower();    // Only for RFM69HCW & HW!
  }
  radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)
  
  radio.encrypt(ENCRYPTKEY);
  radio.promiscuous(promiscuousMode);
  
  //pinMode(LED, OUTPUT);
  
  char buff[50];
  
  Serial.print("\nListening at ");
  Serial.print(FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(" MHz");
}

byte ackCount=0;

void loop()
{
  for(int a = 0; a < 8; a++)
  {
    adcdata[a] = analogRead(adcpin[a]);
  }

  promiscuousMode = !promiscuousMode;
  radio.promiscuous(promiscuousMode);
  
  if (radio.receiveDone())
  {
    Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
    Serial.print(" [RX_RSSI:");Serial.print(radio.readRSSI());Serial.println("]");
    Serial.println("");
    
    /*if (promiscuousMode)
    {
      Serial.print("to [");Serial.print(radio.TARGETID, DEC);Serial.print("] ");
    }*/

    if (radio.DATALEN != sizeof(Payload))
      Serial.print("Invalid payload received, not matching Payload struct!");
    else
    {
        theData = *(Payload*)radio.DATA; //assume radio.DATA actually contains our struct and not something else
      
        StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
        JsonObject& root = jsonBuffer.createObject();

        // 시리얼로 해당 내용 그대로 전달
        root["ptCd"] = "06";
        root["id"] = theData.nodeId;
        root["temp"] = theData.temp;
        root["hum"] = theData.hum;
        root["ill"] = theData.ill;
        root["gas"] = theData.gas;
        root["PM 2.5"] = theData.PM25;
        root["PM 10"] = theData.PM10;
        
        root.printTo(Serial);
        Serial.println("");
      
      /*Serial.print(" Id=");
      Serial.print(theData.nodeId);
      Serial.print(" temp=");
      Serial.print(theData.temp);
      Serial.print(" hum=");
      Serial.println(theData.hum);*/
    }

    Serial.println("");
    
    if (radio.ACKRequested())
    {
      byte theNodeID = radio.SENDERID;
      radio.sendACK();
      Serial.print(" - ACK sent.");

      // When a node requests an ACK, respond to the ACK
      // and also send a packet requesting an ACK (every 3rd one only)
      // This way both TX/RX NODE functions are tested on 1 end at the GATEWAY
      if (ackCount++%3==0)
      {
        Serial.print(" Pinging node ");
        Serial.print(theNodeID);
        Serial.print(" - ACK...");
        delay(3); //need this when sending right after reception .. ?
        if (radio.sendWithRetry(theNodeID, "ACK TEST", 8, 0))  // 0 = only 1 attempt, no retries
          Serial.print("ok!");
        else Serial.print("nothing");
      }
    }
    
    Serial.println("");
    Serial.println("");
    //Blink(LED,3);
  }

  // 서버로 부터의 센서 취득 주기 메시지 파싱 및 해당 내용 EEPROM 기록
  if (Serial.available() >= 1)
  {
    index = Serial.read();

    if(index == 'C')
    {
          //Serial.print("loop:");
          lootin10 = Serial.read();
          lootin = Serial.read();
          num = (lootin10-'0')*10 + (lootin-'0');
          Serial.println(num);
          //delay(100);
          
          //Serial.print("MODE:");
          index1 = Serial.read();
          Serial.println(index1);
          Serial.println("CRLEAN START");       
    }
  }

  if(index1 == 'G')
  {       
         if(y>0)
         {
              sr_mot(33 ,32, 70);
              sl_mot(35 ,34, 120);
              vm_mot(43);
            
            if(num < j)
            {
                 Random(); 
                 num = 0;
            }
          
            else
            {
                 byuk();
            }
          }
          
          else
          {
              turn();
          }
        }
          
          else if(index1 == 'K')
          {
             st_bk(); 
             digitalWrite(33, LOW);
             digitalWrite(32, LOW); 
             digitalWrite(35, LOW);
             digitalWrite(34, LOW); 
             
             digitalWrite(43, LOW);
             digitalWrite(12, LOW);
             digitalWrite(14, LOW);
          }

           else if(index1 == 'W')
             {
               sr_mot(33 ,32, 70);
                sl_mot(35 ,34, 120);
                vm_mot(43);  
                delay(num);
                go(); 
                index1 = 0;
             }
             
             else if(index1== 'S')
             {
               sr_mot(33 ,32, 70);
                sl_mot(35 ,34, 120);
                vm_mot(43);  
                back();
                delay(num);
                index1 = 0;
             }
             
             else if(index1 == 'D')
             {
                sr_mot(33 ,32, 70);
                sl_mot(35 ,34, 120);
                vm_mot(43);
                right();  
                delay(num);
                index1 = 0;
             }
             
             else if(index1 == 'A')
             {
                sr_mot(33 ,32, 70);
                sl_mot(35 ,34, 120);
                vm_mot(43);  
                left();  
                delay(num);
                index1 = 0;
             }

            else if(index1 == 0)
            {
                st_bk(); 
                 digitalWrite(33, LOW);
                 digitalWrite(32, LOW); 
                 digitalWrite(35, LOW);
                 digitalWrite(34, LOW); 
                 
                 digitalWrite(43, LOW);
                 digitalWrite(12, LOW);
                 digitalWrite(14, LOW);
            }

       delay(30);                  
}

void turn()
{
  if((adcdata[0] > 500) || (adcdata[1] > 500) || (adcdata[2] < 150) || (adcdata[3] < 150) || (adcdata[4] < 150))
  {
    y++;
  }
  
  else
  {  
    go();
    /*delay(1000);
    right();
    delay(1500);
    go();
    delay(1500);*/
  }
}

void byuk()
{
  if((adcdata[0] > 500) || (adcdata[1] > 500) || (adcdata[2] < 150) || (adcdata[3] < 150) || (adcdata[4] < 150))
  {
    back();
    delay(400);
    st_bk();
    delay(100);
    left();
    delay(300);
    j++;
  }
  
  else
  {
    mr_mot(29 ,28 ,120);
    ml_mot(30 ,31 ,50);
  }  
}

void Random()
{
  if((adcdata[0] > 500)||(adcdata[2]<150)&&(adcdata[3]>150))
  {
    back();
    delay(400);
    st_bk();
    delay(100);
    left();
    delay(300);
  }
  
  else if((adcdata[1] > 500)||(adcdata[3]<150)&&(adcdata[2]>150))
  {
    back();
    delay(400);
    st_bk();
    delay(100);
    right();
    delay(300);
  }
  
  else if((adcdata[0] > 500) && (adcdata[1] > 500)||(adcdata[3]<150)&&(adcdata[2]<150)||(adcdata[4] < 150))
  {
    back();
    delay(400);
    st_bk();
    delay(100);
    right();
    delay(300);
  }

  else
  {
    go();
  } 
}

void mr_mot(int mrl , int mrr ,int mpr)
{
  analogWrite(3, mpr);
  digitalWrite(mrl, LOW);
  digitalWrite(mrr, HIGH);
}

void ml_mot(int mll , int mlr ,int mpl)
{
  analogWrite(4, mpl);
  digitalWrite(mll, LOW);
  digitalWrite(mlr, HIGH);
}

void sr_mot(int srl , int srr, int spr)
{
  analogWrite(12, spr);
  digitalWrite(srl, LOW);
  digitalWrite(srr, HIGH);
}

void sl_mot(int sll , int slr, int spl)
{
  analogWrite(14, spl);
  digitalWrite(sll, LOW);
  digitalWrite(slr, HIGH);
}

void vm_mot(int vml)
{
  digitalWrite(vml, HIGH);
}

void go()
{
    mr_mot(29 ,28 ,120);
    ml_mot(30 ,31 ,120);
}

void back()
{
    mr_mot(28 ,29 ,140);
    ml_mot(31 ,30 ,140);
}

void left()
{
    mr_mot(28 ,29 ,140);
    ml_mot(30 ,31 ,140);
}

void right()
{
    mr_mot(29 ,28 ,140);
    ml_mot(31 ,30 ,140);
}

void st_bk()
{
    digitalWrite(28, LOW);
    digitalWrite(29, LOW);
    digitalWrite(30, LOW);
    digitalWrite(31, LOW);
    analogWrite(3, 0);
    analogWrite(4, 0);
}
